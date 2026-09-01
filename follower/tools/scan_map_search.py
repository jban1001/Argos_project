#!/usr/bin/env python3
"""맵 전체를 훑어 현재 스캔이 가장 잘 맞는 자세를 찾는다.

무엇을 가르는가
---------------
AMCL 이 수렴하지 않을 때 "자세를 못 찾은 것"인지 "맵이 낡은 것"인지 갈린다.

    잘 맞는 자세가 존재한다  -> 맵은 멀쩡하다.  AMCL 설정/초기값 문제다.
    어느 자세에서도 안 맞는다 -> 맵이 실제 방과 다르다.  다시 그려야 한다.

방법
----
맵의 점유 셀에 대한 거리 변환(distance transform)을 미리 구해 두고, 후보
자세마다 스캔 끝점을 옮겨 "가장 가까운 벽까지의 거리"의 중앙값을 점수로 쓴다.
낮을수록 잘 맞는다.  거친 격자로 훑은 뒤 최고점 주변을 다시 촘촘히 훑는다.

로봇을 움직이지 않는다.  계산만 한다.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def exact_distance_transform(occupied: np.ndarray, res: float) -> np.ndarray:
    """점유 셀까지의 정확한 거리(미터)를 셀마다 구한다.

    scipy 가 없어 직접 구현했다.  맵이 146 x 84 (12264 셀) 로 작고 점유 셀도
    수백 개라 전수 비교가 근사 알고리즘보다 간단하고 빠르다.
    """
    h, w = occupied.shape
    oy, ox = np.nonzero(occupied)
    if len(ox) == 0:
        return np.full((h, w), 1e9)
    gy, gx = np.mgrid[0:h, 0:w]
    flat_y = gy.ravel().astype(np.float32)
    flat_x = gx.ravel().astype(np.float32)
    best = np.full(flat_x.shape, np.inf, dtype=np.float32)
    # 점유 셀을 덩어리로 나눠 메모리를 아낀다
    chunk = 256
    for i in range(0, len(ox), chunk):
        sx = ox[i:i + chunk].astype(np.float32)
        sy = oy[i:i + chunk].astype(np.float32)
        d2 = ((flat_x[:, None] - sx[None, :]) ** 2 +
              (flat_y[:, None] - sy[None, :]) ** 2).min(axis=1)
        np.minimum(best, d2, out=best)
    return (np.sqrt(best) * res).reshape(h, w)


class Grab(Node):
    def __init__(self):
        super().__init__("scan_map_search")
        self.map = None
        self.scan = None
        q = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, "/map", self._m, q)
        self.create_subscription(LaserScan, "/follower/scan", self._s,
                                 qos_profile_sensor_data)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/follower/initialpose", 10)

    def _m(self, m): self.map = m
    def _s(self, m): self.scan = m


def search_occlusion(occ, dist, res, ox, oy, w, h, s0, pts, samples,
                     xs, ys, yaws, inlier_m, w_through):
    """가려짐을 허용하는 자세 탐색.

    왜 이 기준인가
    --------------
    방에 물건이 늘면 라이다는 맵의 벽 대신 그 물건을 본다.  끝점이 맵의 벽
    위에 있기를 요구하면 가려진 빔이 전부 불일치로 세어져, 자세가 맞아도
    점수가 나쁘게 나온다 (2026-08-31: 최선 자세에서도 적합률 39%).

    물리적으로 가려짐(측정 < 예상)은 정상이고, **맵의 벽을 뚫고 보는 것
    (측정 > 예상)만 불가능**하다.  그래서 빔이 지나온 경로에 맵의 점유 셀이
    있는지를 벌점으로 쓴다.  가려짐에는 벌점을 주지 않는다.

    점수 = w_through * (벽을 뚫은 빔 비율) + (1 - 끝점 적합률)
    앞항이 자세를 결정하고, 뒷항은 같은 값일 때 갈라준다.
    """
    best = (1e9, 0.0, 0.0, 0.0, 0.0, 0.0)
    ex, ey = pts[:, 0], pts[:, 1]
    sx, sy = samples[:, 0], samples[:, 1]
    n = len(ex)
    k = len(sx) // n
    for yaw in yaws:
        c, sn = math.cos(yaw), math.sin(yaw)
        rex, rey = c * ex - sn * ey, sn * ex + c * ey
        rsx, rsy = c * sx - sn * sy, sn * sx + c * sy
        for x in xs:
            iex = ((rex + x - ox) / res).astype(np.int32)
            isx = ((rsx + x - ox) / res).astype(np.int32)
            okex = (iex >= 0) & (iex < w)
            oksx = (isx >= 0) & (isx < w)
            iexc, isxc = np.clip(iex, 0, w - 1), np.clip(isx, 0, w - 1)
            for y in ys:
                iey = ((rey + y - oy) / res).astype(np.int32)
                isy = ((rsy + y - oy) / res).astype(np.int32)
                oke = okex & (iey >= 0) & (iey < h)
                oks = oksx & (isy >= 0) & (isy < h)
                iec, isc = np.clip(iey, 0, h - 1), np.clip(isy, 0, h - 1)

                hit = occ[isc, isxc] & oks              # 경로가 벽을 지났는가
                through = hit.reshape(n, k).any(axis=1)
                # 끝점이 그 벽 근처면 뚫은 게 아니라 그 벽을 맞힌 것이다
                d_end = np.where(oke, dist[iec, iexc], 1.0)
                through &= (d_end > inlier_m)
                through_frac = float(through.mean())
                inlier = float((d_end <= inlier_m).mean())
                score = w_through * through_frac + (1.0 - inlier)
                if score < best[0]:
                    best = (score, float(x), float(y), float(yaw),
                            inlier, through_frac)
    return best


def search(dist, res, ox, oy, w, h, pts, xs, ys, yaws, cap):
    """후보 자세 격자를 훑어 가장 점수가 낮은 (점수, x, y, yaw, 적합률) 을 준다.

    맵 밖으로 나간 끝점은 **잘라내지 않고 벌점(cap)** 을 준다.  잘라내면
    경계 밖으로 밀려난 점이 가장자리 셀로 옮겨져 오히려 좋은 점수를 받고,
    탐색이 맵 가장자리로 끌려간다 (2026-08-31 에 실제로 그렇게 틀렸다).

    점수는 중앙값이 아니라 평균이다.  맵의 9% 가 점유라 어느 자세든 절반은
    한 셀 안에 들어와 중앙값이 0.05 로 포화된다.
    """
    best = (1e9, 0.0, 0.0, 0.0, 0.0)
    px, py = pts[:, 0], pts[:, 1]
    n = len(px)
    for yaw in yaws:
        c, s = math.cos(yaw), math.sin(yaw)
        rx = c * px - s * py
        ry = s * px + c * py
        for x in xs:
            fx = (rx + x - ox) / res
            ix = fx.astype(np.int32)
            okx = (ix >= 0) & (ix < w)
            ixc = np.clip(ix, 0, w - 1)
            for y in ys:
                fy = (ry + y - oy) / res
                iy = fy.astype(np.int32)
                ok = okx & (iy >= 0) & (iy < h)
                iyc = np.clip(iy, 0, h - 1)
                d = np.where(ok, dist[iyc, ixc], cap)
                d = np.minimum(d, cap)
                score = float(d.mean())
                if score < best[0]:
                    inlier = float((d < 0.10).sum()) / n
                    best = (score, float(x), float(y), float(yaw), inlier)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--near-x", type=float, default=None,
                    help="이 좌표 주변만 탐색한다 (--near-y 와 같이 쓴다)")
    ap.add_argument("--near-y", type=float, default=None)
    ap.add_argument("--near-radius", type=float, default=0.6,
                    help="--near-x/y 주변 탐색 반경 (m)")
    ap.add_argument("--coarse-xy", type=float, default=0.20)
    ap.add_argument("--coarse-yaw", type=float, default=10.0)
    ap.add_argument("--fine-xy", type=float, default=0.04)
    ap.add_argument("--fine-yaw", type=float, default=1.5)
    ap.add_argument("--cap", type=float, default=1.0, help="거리 상한 (m)")
    ap.add_argument("--publish", action="store_true",
                    help="찾은 자세를 /follower/initialpose 로 발행한다")
    ap.add_argument("--inlier-m", type=float, default=0.10,
                    help="끝점이 이 거리 안이면 맵의 벽을 맞힌 것으로 본다")
    ap.add_argument("--w-through", type=float, default=3.0,
                    help="벽뚫음 벌점 가중치")
    ap.add_argument("--publish-max-through", type=float, default=0.10,
                    help="벽뚫음 비율이 이보다 크면 발행하지 않는다")
    ap.add_argument("--publish-sigma-xy", type=float, default=0.10)
    ap.add_argument("--publish-sigma-yaw", type=float, default=5.0)
    args = ap.parse_args()

    rclpy.init()
    node = Grab()
    try:
        # 실외 지도는 561x464 = 254 KiB 다.  Pi 의 UDP 수신 버퍼가
        # 208 KiB 뿐이라 조각이 유실되고, 재전송을 거쳐 도착하기까지
        # 15 s 를 넘긴다.  넉넉히 기다린다.
        end = time.time() + 90
        while time.time() < end and (node.map is None or node.scan is None):
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.map is None or node.scan is None:
            print("맵 또는 스캔을 못 받았다."); return 1

        g, scan = node.map, node.scan
        w, h, res = g.info.width, g.info.height, g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        data = np.array(g.data, dtype=np.int16).reshape(h, w)
        occupied = data >= 65
        if not occupied.any():
            print("맵에 점유 셀이 없다."); return 1
        dist = exact_distance_transform(occupied, res)

        pts = []
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r <= max(scan.range_min, 0.05) or r > 20.0:
                continue
            a = scan.angle_min + i * scan.angle_increment
            pts.append((r * math.cos(a), r * math.sin(a)))
        pts = np.array(pts)

        # 라이다 프레임 -> base_link.  라이다는 roll pi 로 뒤집혀 있어서
        # 2D yaw 만으로는 못 옮긴다.  TF 의 3D 회전을 그대로 쓴다.
        tf = None
        deadline = time.time() + 10.0      # static TF 가 버퍼에 들어올 시간을 준다
        last = ""
        while time.time() < deadline and tf is None:
            try:
                tf = node.buf.lookup_transform(
                    "follower_base_link", scan.header.frame_id, rclpy.time.Time())
            except Exception as exc:
                last = str(exc)
                rclpy.spin_once(node, timeout_sec=0.1)
        if tf is None:
            print(f"TF follower_base_link <- {scan.header.frame_id} 실패: {last}")
            return 1
        q = tf.transform.rotation
        R = np.array([
            [1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w)],
            [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z - q.x * q.w)],
            [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x * q.x + q.y * q.y)]])
        t3 = np.array([tf.transform.translation.x, tf.transform.translation.y,
                       tf.transform.translation.z])
        p3 = np.concatenate([pts, np.zeros((len(pts), 1))], axis=1)
        pts = (p3 @ R.T + t3)[:, :2]
        print(f"끝점을 follower_base_link 로 옮겼다 "
              f"(t=({t3[0]:+.3f}, {t3[1]:+.3f}), det(R)={np.linalg.det(R):+.3f})")
        print(f"맵 {w}x{h} @ {res} m, 점유 {int(occupied.sum())} 셀")
        print(f"스캔 유효 빔 {len(pts)} / {len(scan.ranges)}")
        print(f"거친 격자: xy {args.coarse_xy} m, yaw {args.coarse_yaw} deg ... ", end="", flush=True)

        x_lo, x_hi = ox, ox + w * res
        y_lo, y_hi = oy, oy + h * res
        if args.near_x is not None and args.near_y is not None:
            r = args.near_radius
            x_lo, x_hi = max(x_lo, args.near_x - r), min(x_hi, args.near_x + r)
            y_lo, y_hi = max(y_lo, args.near_y - r), min(y_hi, args.near_y + r)
            print(f"탐색 범위 제한: x {x_lo:+.2f}~{x_hi:+.2f} "
                  f"y {y_lo:+.2f}~{y_hi:+.2f} (반경 {r} m)")
        # 빔 경로 표본.  센서 원점에서 끝점까지 몇 지점을 찍어, 그 사이에
        # 맵의 벽이 있는지 본다.  0.95 까지만 보는 이유는 끝점 바로 앞은
        # 자기가 맞힌 벽이라 뚫은 것이 아니기 때문이다.
        fracs = np.array([0.25, 0.40, 0.55, 0.70, 0.82, 0.90])
        sens = np.array([t3[0], t3[1]])
        samples = np.concatenate(
            [sens + f * (pts - sens) for f in fracs], axis=0)
        print(f"빔 경로 표본 {len(fracs)} 지점 x {len(pts)} 빔")

        t0 = time.time()
        best = search_occlusion(occupied, dist, res, ox, oy, w, h, sens,
                                pts, samples,
                                np.arange(x_lo, x_hi, args.coarse_xy),
                                np.arange(y_lo, y_hi, args.coarse_xy),
                                np.radians(np.arange(0, 360, args.coarse_yaw)),
                                args.inlier_m, args.w_through)
        print(f"{time.time()-t0:.1f} s")
        print(f"  최적 {best[0]:.4f}  at ({best[1]:+.3f}, {best[2]:+.3f}) "
              f"yaw {math.degrees(best[3]):+.1f}  적합률 {best[4]*100:.1f}%  "
              f"벽뚫음 {best[5]*100:.1f}%")

        print("촘촘히 재탐색 ... ", end="", flush=True)
        t0 = time.time()
        best = search_occlusion(occupied, dist, res, ox, oy, w, h, sens,
                                pts, samples,
                                np.arange(best[1] - args.coarse_xy, best[1] + args.coarse_xy, args.fine_xy),
                                np.arange(best[2] - args.coarse_xy, best[2] + args.coarse_xy, args.fine_xy),
                                np.radians(np.arange(math.degrees(best[3]) - args.coarse_yaw,
                                                     math.degrees(best[3]) + args.coarse_yaw,
                                                     args.fine_yaw)),
                                args.inlier_m, args.w_through)
        print(f"{time.time()-t0:.1f} s")
        print(f"\n최적 자세  x={best[1]:+.3f}  y={best[2]:+.3f}  "
              f"yaw={math.degrees(best[3]):+.2f} deg")
        print(f"점수 = {best[0]:.4f}   (낮을수록 좋다)")
        print(f"  적합률   {best[4]*100:5.1f}%   끝점이 맵의 벽 {args.inlier_m} m 안")
        print(f"  벽뚫음   {best[5]*100:5.1f}%   맵의 벽을 통과해 본 빔 -- 이게 낮아야 자세가 맞다")
        print(f"  가려짐   {(1-best[4]-best[5])*100:5.1f}%   맵에 없는 물체를 본 빔 (정상)")
        print()
        if args.publish:
            if best[5] > args.publish_max_through:
                print(f"벽뚫음 {best[5]*100:.1f}% 가 "
                      f"{args.publish_max_through*100:.0f}% 보다 크다. "
                      f"발행하지 않는다 -- 틀린 자세를 넣으면 AMCL 이 더 나빠진다.")
            else:
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = "map"
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.pose.pose.position.x = best[1]
                msg.pose.pose.position.y = best[2]
                msg.pose.pose.orientation.z = math.sin(best[3] / 2)
                msg.pose.pose.orientation.w = math.cos(best[3] / 2)
                c = [0.0] * 36
                c[0] = c[7] = args.publish_sigma_xy ** 2
                c[35] = math.radians(args.publish_sigma_yaw) ** 2
                msg.pose.covariance = c
                for _ in range(3):
                    node.pose_pub.publish(msg)
                    rclpy.spin_once(node, timeout_sec=0.1)
                print(f"/follower/initialpose 로 발행했다 "
                      f"(sigma xy {args.publish_sigma_xy} m, yaw {args.publish_sigma_yaw} deg)")
        if best[5] < 0.05:
            print("판정: 맵의 벽을 뚫고 보는 빔이 거의 없다 -- 자세가 맞다.")
            print("      적합률이 낮아도 그것은 가려짐이지 불일치가 아니다.")
        elif best[5] < 0.15:
            print("판정: 대체로 맞는다. 뚫은 빔이 조금 있다.")
        else:
            print("판정: 벽을 뚫고 보는 빔이 많다. 자세가 틀렸거나 맵이 방과 다르다.")
        return 0
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
