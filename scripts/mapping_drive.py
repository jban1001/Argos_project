#!/usr/bin/env python3

"""
ARGOS SLAM 매핑 주행

안전 우선 설계:
  - 전진은 LiDAR 전방 여유거리를 확인한 뒤에만 한다.
  - 여유거리가 STOP_DIST 이하가 되면 즉시 정지.
  - 총 주행 거리 / 총 시간 상한.
  - Ctrl+C, 예외, 종료 시 항상 /cmd_vel 0.

LiDAR 가 base_link 기준 약 177도 돌아가 장착되어 있으므로
스캔 각도는 반드시 TF (base_link -> laser_frame) 로 변환해서 쓴다.
각도를 직접 가정하지 말 것.

사용:
    python3 mapping_drive.py spin          360도 제자리 회전만 (가장 안전)
    python3 mapping_drive.py spin forward  회전 + 전방 왕복
"""

import sys
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from tf2_ros import Buffer, TransformListener


SPIN_W = 0.25
SPIN_TURNS = 1.0

FORWARD_V = 0.08

STOP_DIST = 0.55          # 이 거리 이하로 접근하면 정지 [m]
FRONT_HALF_ANGLE = 0.35   # 전방 판정 반각 [rad] (약 20도)

MAX_FORWARD_DIST = 1.2    # 한 방향 최대 전진 거리 [m]
MAX_RUN_TIME = 180.0      # 전체 상한 [s]


def yaw_from_quat(q):

    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class MappingDrive(Node):

    def __init__(self):

        super().__init__("mapping_drive")

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(
            LaserScan, "/scan", self.scan_cb, qos_profile_sensor_data
        )

        self.create_subscription(
            Odometry, "/odom", self.odom_cb, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan = None
        self.odom = None

        self.laser_yaw = None
        self.laser_x = 0.0
        self.laser_y = 0.0

        self.t_start = time.monotonic()

    def scan_cb(self, msg):
        self.scan = msg

    def odom_cb(self, msg):
        self.odom = msg

    def spin_once(self, t=0.01):
        rclpy.spin_once(self, timeout_sec=t)

    def spin_for(self, d):

        end = time.monotonic() + d

        while time.monotonic() < end:
            self.spin_once()

    def wait_data(self, timeout=15.0):

        end = time.monotonic() + timeout

        while time.monotonic() < end:

            self.spin_once(0.05)

            if self.scan is None or self.odom is None:
                continue

            if self.laser_yaw is None:
                self.lookup_laser_tf()

            if self.laser_yaw is not None:
                return True

        return False

    def lookup_laser_tf(self):

        try:
            tf = self.tf_buffer.lookup_transform(
                "base_link", "laser_frame", rclpy.time.Time()
            )

        except Exception:
            return

        self.laser_x = tf.transform.translation.x
        self.laser_y = tf.transform.translation.y
        self.laser_yaw = yaw_from_quat(tf.transform.rotation)

        self.get_logger().info(
            f"base_link -> laser_frame : "
            f"x={self.laser_x:.3f} y={self.laser_y:.3f} "
            f"yaw={math.degrees(self.laser_yaw):.1f} deg"
        )

    def front_clearance(self):
        return self.clearance(0.0)

    def rear_clearance(self):
        return self.clearance(math.pi)

    def clearance(self, heading):
        """
        base_link 기준 heading 방향의 최소 여유거리.

        heading = 0    -> 전방
        heading = pi   -> 후방

        laser 각도 a 는 base_link 기준으로 (a + laser_yaw) 이므로
        그 값이 heading 근처인 점들만 본다.
        """

        s = self.scan

        if s is None or self.laser_yaw is None:
            return None

        best = float("inf")

        n = len(s.ranges)

        for i in range(n):

            r = s.ranges[i]

            if not math.isfinite(r):
                continue

            if not (s.range_min < r < s.range_max):
                continue

            a = s.angle_min + i * s.angle_increment

            a_rel = a + self.laser_yaw - heading

            a_rel = math.atan2(math.sin(a_rel), math.cos(a_rel))

            if abs(a_rel) > FRONT_HALF_ANGLE:
                continue

            a_base = math.atan2(
                math.sin(a + self.laser_yaw),
                math.cos(a + self.laser_yaw)
            )

            # laser 가 base_link 앞쪽 x 만큼 나가 있으므로 보정
            r_base = r - self.laser_x * math.cos(a_base)

            if r_base < best:
                best = r_base

        return None if math.isinf(best) else best

    def publish(self, v, w):

        m = Twist()
        m.linear.x = float(v)
        m.angular.z = float(w)

        self.pub.publish(m)

    def stop(self):

        for _ in range(15):
            self.publish(0.0, 0.0)
            self.spin_once()
            time.sleep(0.02)

    def timed_out(self):
        return time.monotonic() - self.t_start > MAX_RUN_TIME

    # -------------------------------------------------

    def do_spin(self, direction=1.0):

        target = 2.0 * math.pi * SPIN_TURNS

        print(f"\n[SPIN] 제자리 회전 {math.degrees(target):.0f} deg "
              f"@ {SPIN_W} rad/s")

        yaw_prev = yaw_from_quat(self.odom.pose.pose.orientation)

        turned = 0.0

        while turned < target and not self.timed_out():

            self.publish(0.0, direction * SPIN_W)

            self.spin_once()

            time.sleep(0.02)

            yaw_now = yaw_from_quat(self.odom.pose.pose.orientation)

            d = yaw_now - yaw_prev
            d = math.atan2(math.sin(d), math.cos(d))

            turned += abs(d)
            yaw_prev = yaw_now

        self.stop()

        print(f"[SPIN] 완료: {math.degrees(turned):.1f} deg")

    def do_forward(self, sign=1.0):

        label = "전진" if sign > 0 else "후진"

        print(f"\n[{label}] 최대 {MAX_FORWARD_DIST} m, "
              f"정지거리 {STOP_DIST} m")

        p0 = self.odom.pose.pose.position
        x0, y0 = p0.x, p0.y

        travelled = 0.0

        while travelled < MAX_FORWARD_DIST and not self.timed_out():

            clear = (
                self.front_clearance() if sign > 0
                else self.rear_clearance()
            )

            if clear is None:
                print(f"  [정지] {label} 스캔 없음")
                break

            if clear < STOP_DIST:
                print(f"  [정지] {label} 여유 {clear:.2f} m")
                break

            self.publish(sign * FORWARD_V, 0.0)

            self.spin_once()

            time.sleep(0.02)

            p = self.odom.pose.pose.position

            travelled = math.hypot(p.x - x0, p.y - y0)

        self.stop()

        print(f"[{label}] 완료: {travelled:.3f} m")

        return travelled


def main():

    modes = sys.argv[1:] or ["spin"]

    rclpy.init()

    node = MappingDrive()

    try:

        if not node.wait_data():
            print("[FAIL] /scan, /odom, TF 준비 안 됨")
            return

        clear = node.front_clearance()

        print(f"\n시작 전방 여유거리 = "
              f"{clear:.2f} m" if clear else "\n전방 여유거리 측정 불가")

        if "spin" in modes:
            node.do_spin(+1.0)
            node.spin_for(2.0)

        if "cw" in modes:
            node.do_spin(-1.0)
            node.spin_for(2.0)

        if "forward" in modes:

            d = node.do_forward(+1.0)

            node.spin_for(2.0)

            if d > 0.05:
                node.do_forward(-1.0)

        print("\n[DONE] 매핑 주행 종료")

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

        print("[DONE] motor stop")


if __name__ == "__main__":
    main()
