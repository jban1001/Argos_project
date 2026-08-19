#!/usr/bin/env python3

"""
localization 품질 직접 측정

현재 TF (map -> laser_frame) 로 스캔을 map 좌표계에 투영한 뒤
각 점이 map 의 occupied 셀에서 얼마나 떨어져 있는지 잰다.

localization 이 맞으면 스캔 점 대부분이 벽 위에 얹힌다.
왕복 복귀 오차 같은 간접 지표와 달리
"지금 이 순간 자세가 맞는가"를 직접 답한다.

주의: 맵에 없는 물체(사람, 옮겨진 의자)는 당연히 오차로 잡히므로
      전체 분포를 보고 판단할 것.
"""

import math
import time

import numpy as np
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

from tf2_ros import Buffer, TransformListener


def yaw_from_quat(q):

    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class LocalizationCheck(Node):

    def __init__(self):

        super().__init__("check_localization")

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(
            OccupancyGrid, "/map", self.map_cb, map_qos
        )

        self.create_subscription(
            LaserScan, "/scan", self.scan_cb, qos_profile_sensor_data
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map = None
        self.scan = None
        self.dist_field = None

    def map_cb(self, msg):

        self.map = msg

        a = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width
        )

        occupied = a > 50

        if occupied.sum() == 0:
            return

        # occupied 셀까지의 거리장 [m]
        self.dist_field = (
            distance_transform_edt(~occupied) * msg.info.resolution
        )

    def scan_cb(self, msg):
        self.scan = msg

    def wait(self, timeout=15.0):

        end = time.monotonic() + timeout

        while time.monotonic() < end:

            rclpy.spin_once(self, timeout_sec=0.05)

            if (
                self.map is None
                or self.scan is None
                or self.dist_field is None
            ):
                continue

            # TF 버퍼가 채워질 때까지 기다린다.
            # 짧게 사는 노드라 이걸 안 하면 첫 조회가 실패한다.
            try:
                self.tf_buffer.lookup_transform(
                    "map",
                    self.scan.header.frame_id,
                    rclpy.time.Time(),
                )

            except Exception:
                continue

            return True

        return False

    def evaluate(self):

        s = self.scan
        m = self.map

        try:
            tf = self.tf_buffer.lookup_transform(
                "map", s.header.frame_id, rclpy.time.Time()
            )

        except Exception as e:
            print(f"[FAIL] TF map -> {s.header.frame_id}: {e}")
            return None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        th = yaw_from_quat(tf.transform.rotation)

        n = len(s.ranges)

        r = np.asarray(s.ranges, dtype=np.float64)

        ang = s.angle_min + np.arange(n) * s.angle_increment

        ok = (
            np.isfinite(r)
            & (r > s.range_min)
            & (r < min(s.range_max, 8.0))
        )

        if ok.sum() < 20:
            print("[FAIL] 유효 스캔 점 부족")
            return None

        lx = r[ok] * np.cos(ang[ok])
        ly = r[ok] * np.sin(ang[ok])

        c, sn = math.cos(th), math.sin(th)

        mx = tx + c * lx - sn * ly
        my = ty + sn * lx + c * ly

        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y

        col = np.floor((mx - ox) / res).astype(np.int64)
        row = np.floor((my - oy) / res).astype(np.int64)

        inside = (
            (col >= 0) & (col < m.info.width)
            & (row >= 0) & (row < m.info.height)
        )

        if inside.sum() < 20:
            print("[FAIL] 맵 범위 안 스캔 점 부족")
            return None

        d = self.dist_field[row[inside], col[inside]]

        return {
            "pose": (tx, ty, math.degrees(th)),
            "n": int(inside.sum()),
            "outside": int((~inside).sum()),
            "mean": float(d.mean()),
            "median": float(np.median(d)),
            "p90": float(np.percentile(d, 90)),
            "within_5cm": float((d <= 0.05).mean()),
            "within_10cm": float((d <= 0.10).mean()),
            "within_20cm": float((d <= 0.20).mean()),
        }


def main():

    rclpy.init()

    node = LocalizationCheck()

    try:

        if not node.wait():
            print("[FAIL] /map, /scan, TF 준비 안 됨")
            return

        print()
        print("=" * 58)
        print(" LOCALIZATION 품질 (스캔 점 -> 맵 벽 거리)")
        print("=" * 58)

        for i in range(5):

            rclpy.spin_once(node, timeout_sec=0.5)

            res = node.evaluate()

            if res is None:
                continue

            if i == 0:
                print(f" 자세 = ({res['pose'][0]:+.3f}, "
                      f"{res['pose'][1]:+.3f}, "
                      f"{res['pose'][2]:+.1f} deg)")
                print()
                print(f"{'점수':>6} {'평균':>8} {'중앙':>8} {'p90':>8} "
                      f"{'<5cm':>7} {'<10cm':>7} {'<20cm':>7}")

            print(
                f"{res['n']:>6} "
                f"{res['mean']:>8.3f} {res['median']:>8.3f} "
                f"{res['p90']:>8.3f} "
                f"{100*res['within_5cm']:>6.1f}% "
                f"{100*res['within_10cm']:>6.1f}% "
                f"{100*res['within_20cm']:>6.1f}%"
            )

            time.sleep(0.4)

        print()
        print(" 판정 기준 (맵 해상도 0.05 m)")
        print("   중앙값 < 0.05 m, <10cm 비율 > 80%  -> 잘 맞음")
        print("   중앙값 0.05~0.15 m                 -> 느슨함")
        print("   중앙값 > 0.15 m                    -> localization 어긋남")

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
