#!/usr/bin/env python3

"""
LiDAR 장착 위치(base_link -> laser_frame) 실측

추측하지 않고 스캔 정합(ICP)으로 x, y, yaw 를 구한다.

원리
----
laser 가 base_link 기준 l = (lx, ly), yaw offset psi 에 장착되어 있을 때

1) 제자리 회전 theta:
       p_B = R(-theta) p_A + t
       t   = (R(-theta) - I) l
   -> l = (R(-theta) - I)^-1 t

2) 직진 d (base_link x 방향):
       laser frame 에서의 이동은 R(-psi) * (d, 0)
   -> psi = -atan2(t_y, t_x)

두 동작 모두 정지 -> 이동 -> 정지 로 수행해서
저속 회전 LiDAR 의 스캔 왜곡을 피한다.
"""

import math
import time

import numpy as np
from scipy.spatial import cKDTree

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


import sys

ROTATE_W = 0.25
ROTATE_TIME = 6.0        # 약 86 deg : 회전 ICP 조건수 개선

FORWARD_V = 0.10
FORWARD_TIME = 3.0

# 인자로 reverse 를 주면 후진으로 psi 를 측정한다.
# 전진/후진 결과를 평균하면 직진 편향(오른쪽 모터 약함)이 상쇄된다.
if "reverse" in sys.argv[1:]:
    FORWARD_V = -FORWARD_V

SETTLE_TIME = 1.5

ICP_ITERS = 60
ICP_REJECT = 0.30       # 대응점 최대 거리 [m]


def yaw_from_odom(msg):

    q = msg.pose.pose.orientation

    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def scan_to_xy(scan):

    n = len(scan.ranges)

    ang = scan.angle_min + np.arange(n) * scan.angle_increment

    r = np.asarray(scan.ranges, dtype=float)

    ok = (
        np.isfinite(r)
        & (r > scan.range_min)
        & (r < scan.range_max)
    )

    return np.stack(
        [r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])],
        axis=1
    )


def rot(theta):

    c = math.cos(theta)
    s = math.sin(theta)

    return np.array([[c, -s], [s, c]])


def icp(src, dst, init_theta=0.0):
    """
    src 를 dst 에 맞추는 강체 변환 (R, t) 를 구한다.
        dst ~= R * src + t
    """

    tree = cKDTree(dst)

    R = rot(init_theta)
    t = np.zeros(2)

    cur = (R @ src.T).T + t

    for _ in range(ICP_ITERS):

        dist, idx = tree.query(cur, k=1)

        m = dist < ICP_REJECT

        if m.sum() < 30:
            break

        a = cur[m]
        b = dst[idx[m]]

        ca = a.mean(axis=0)
        cb = b.mean(axis=0)

        H = (a - ca).T @ (b - cb)

        U, _, Vt = np.linalg.svd(H)

        d = np.sign(np.linalg.det(Vt.T @ U.T))

        Rd = Vt.T @ np.diag([1.0, d]) @ U.T

        td = cb - Rd @ ca

        cur = (Rd @ cur.T).T + td

        R = Rd @ R
        t = Rd @ t + td

        if np.linalg.norm(td) < 1e-5 and abs(math.atan2(Rd[1, 0], Rd[0, 0])) < 1e-5:
            break

    dist, idx = tree.query(cur, k=1)
    m = dist < ICP_REJECT

    rmse = float(np.sqrt((dist[m] ** 2).mean())) if m.sum() else float("nan")

    theta = math.atan2(R[1, 0], R[0, 0])

    return theta, t, rmse, int(m.sum())


class LidarMountCalib(Node):

    def __init__(self):

        super().__init__("calibrate_lidar_mount")

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(
            LaserScan, "/scan", self.scan_cb, qos_profile_sensor_data
        )

        self.create_subscription(
            Odometry, "/odom", self.odom_cb, 10
        )

        self.scan = None
        self.odom = None

    def scan_cb(self, msg):
        self.scan = msg

    def odom_cb(self, msg):
        self.odom = msg

    def spin_for(self, d):

        end = time.monotonic() + d

        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_data(self, timeout=10.0):

        end = time.monotonic() + timeout

        while time.monotonic() < end:

            rclpy.spin_once(self, timeout_sec=0.05)

            if self.scan is not None and self.odom is not None:
                return True

        return False

    def publish(self, v, w):

        m = Twist()
        m.linear.x = float(v)
        m.angular.z = float(w)

        self.pub.publish(m)

    def stop(self):

        for _ in range(15):
            self.publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    def grab_scan(self):

        self.scan = None

        end = time.monotonic() + 5.0

        while self.scan is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

        return self.scan

    def move(self, v, w, duration):

        p0 = self.odom.pose.pose.position
        x0, y0 = p0.x, p0.y
        yaw0 = yaw_from_odom(self.odom)

        end = time.monotonic() + duration

        while time.monotonic() < end:

            self.publish(v, w)

            rclpy.spin_once(self, timeout_sec=0.01)

            time.sleep(0.02)

        self.stop()
        self.spin_for(SETTLE_TIME)

        p1 = self.odom.pose.pose.position
        yaw1 = yaw_from_odom(self.odom)

        dx = p1.x - x0
        dy = p1.y - y0

        dyaw = math.atan2(
            math.sin(yaw1 - yaw0),
            math.cos(yaw1 - yaw0)
        )

        # base_link 기준 이동량
        c = math.cos(yaw0)
        s = math.sin(yaw0)

        fwd = c * dx + s * dy
        lat = -s * dx + c * dy

        return fwd, lat, dyaw


def main():

    rclpy.init()

    node = LidarMountCalib()

    try:

        if not node.wait_data():
            print("[FAIL] /scan 또는 /odom 없음")
            return

        print()
        print("=" * 62)
        print(" ARGOS LiDAR MOUNT CALIBRATION (base_link -> laser_frame)")
        print("=" * 62)

        # ============ 1. 제자리 회전 -> x, y ============

        node.spin_for(1.0)

        a = node.grab_scan()
        pa = scan_to_xy(a)

        fwd, lat, dyaw = node.move(0.0, ROTATE_W, ROTATE_TIME)

        b = node.grab_scan()
        pb = scan_to_xy(b)

        print()
        print(f" [ROTATE] odom d_yaw = {math.degrees(dyaw):+.2f} deg, "
              f"이동 {math.hypot(fwd, lat)*1000:.0f} mm")

        # p_B = R(-theta) p_A + t   ->  src=pa, dst=pb
        th, t, rmse, n = icp(pa, pb, init_theta=-dyaw)

        print(f"          ICP: dtheta = {math.degrees(th):+.2f} deg, "
              f"t = ({t[0]:+.4f}, {t[1]:+.4f}) m, "
              f"rmse = {rmse:.4f}, n = {n}")

        M = rot(th) - np.eye(2)

        if abs(np.linalg.det(M)) < 1e-6:
            print("          [FAIL] 회전량 부족")
            l_rot = None
        else:
            # l_rot = R(-psi) * l   (psi 는 아래 직진 시험에서 구한다)
            l_rot = np.linalg.solve(M, t)

        # ============ 2. 직진 -> yaw offset ============

        node.spin_for(1.5)

        a2 = node.grab_scan()
        pa2 = scan_to_xy(a2)

        fwd2, lat2, dyaw2 = node.move(FORWARD_V, 0.0, FORWARD_TIME)

        b2 = node.grab_scan()
        pb2 = scan_to_xy(b2)

        print()
        print(f" [FORWARD] odom 이동 = {fwd2:+.4f} m "
              f"(lat {lat2:+.4f}), d_yaw = {math.degrees(dyaw2):+.2f} deg")

        th2, t2, rmse2, n2 = icp(pa2, pb2, init_theta=-dyaw2)

        print(f"          ICP: dtheta = {math.degrees(th2):+.2f} deg, "
              f"t = ({t2[0]:+.4f}, {t2[1]:+.4f}) m, "
              f"rmse = {rmse2:.4f}, n = {n2}")

        # laser frame 에서 본 이동 방향의 반대가 로봇 전진 방향
        # 로봇이 전진하면 스캔 점군은 laser frame 에서 반대로 이동한다.
        # 따라서 로봇 전진 방향의 laser frame 표현은 -t2 이고,
        # 그 방위각이 곧 -psi 가 아니라 psi 의 정의에 따라
        #   base 의 +x 를 laser frame 으로 본 방향 = R(-psi)*(1,0)
        # 이므로 psi = -atan2(-t2_y, -t2_x)
        if FORWARD_V > 0:
            move_dir = math.atan2(-t2[1], -t2[0])
        else:
            move_dir = math.atan2(t2[1], t2[0])

        psi = -move_dir

        if l_rot is None:
            lx = ly = float("nan")
        else:
            l = rot(psi) @ l_rot
            lx, ly = float(l[0]), float(l[1])

        print()
        print("=" * 62)
        print(" 결과 (base_link -> laser_frame)")
        print("=" * 62)
        print(f"   x   = {lx:+.4f} m")
        print(f"   y   = {ly:+.4f} m")
        print(f"   yaw = {math.degrees(psi):+.2f} deg")
        print()
        print(" static_transform_publisher 인자:")
        print(f"   --x {lx:.4f} --y {ly:.4f} --z 0.02 "
              f"--yaw {psi:.4f} --frame-id base_link "
              f"--child-frame-id laser_frame")
        print()
        print(" ※ 반드시 자로 실측한 값과 비교해서 확인할 것")

    except KeyboardInterrupt:
        print("\n[STOP] 사용자 중단")

    finally:

        try:
            node.stop()
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        print("\n[DONE] motor stop")


if __name__ == "__main__":
    main()
