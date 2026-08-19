#!/usr/bin/env python3

"""
ARGOS /cmd_vel 방향 및 open-loop 속도 검증

argos_base_driver + wheel_odometry_node 가 떠 있는 상태에서 실행한다.

1) 전진 : linear.x > 0  -> /odom x 증가, yaw 변화 거의 없음
2) 회전 : angular.z > 0 -> /odom yaw 증가 (CCW)

각 동작 후 /cmd_vel 0 을 반복 발행하고 종료한다.
(base_driver watchdog 도 별도로 동작하므로 이중 안전)

사용:
    python3 check_cmd_vel_direction.py forward
    python3 check_cmd_vel_direction.py rotate
    python3 check_cmd_vel_direction.py both
"""

import sys
import math
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


FORWARD_V = 0.10
FORWARD_TIME = 2.0

ROTATE_W = 0.5
ROTATE_TIME = 2.0


def yaw_from_odom(msg):

    q = msg.pose.pose.orientation

    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class DirectionCheck(Node):

    def __init__(self):

        super().__init__("check_cmd_vel_direction")

        self.pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.odom = None

    def odom_callback(self, msg):
        self.odom = msg

    def spin_for(self, duration):

        end = time.monotonic() + duration

        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_odom(self, timeout=5.0):

        end = time.monotonic() + timeout

        while self.odom is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

        return self.odom is not None

    def snapshot(self):

        msg = self.odom

        return (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_odom(msg)
        )

    def publish(self, v, w):

        msg = Twist()

        msg.linear.x = float(v)
        msg.angular.z = float(w)

        self.pub.publish(msg)

    def stop(self):

        for _ in range(15):

            self.publish(0.0, 0.0)

            rclpy.spin_once(self, timeout_sec=0.01)

            time.sleep(0.02)

    def run_motion(self, name, v, w, duration):

        print()
        print("=" * 46)
        print(f" {name}")
        print(f" cmd_vel : linear.x={v}  angular.z={w}")
        print(f" time    : {duration}s")
        print("=" * 46)

        x0, y0, yaw0 = self.snapshot()

        t0 = time.monotonic()

        end = t0 + duration

        while time.monotonic() < end:

            self.publish(v, w)

            rclpy.spin_once(self, timeout_sec=0.01)

            time.sleep(0.04)

        elapsed = time.monotonic() - t0

        self.stop()

        # 정지 후 잔여 이동 반영
        self.spin_for(0.5)

        x1, y1, yaw1 = self.snapshot()

        dx = x1 - x0
        dy = y1 - y0

        dyaw = math.atan2(
            math.sin(yaw1 - yaw0),
            math.cos(yaw1 - yaw0)
        )

        dist = math.hypot(dx, dy)

        print(f"  d_x       = {dx:+.4f} m")
        print(f"  d_y       = {dy:+.4f} m")
        print(f"  distance  = {dist:.4f} m")
        print(f"  d_yaw     = {math.degrees(dyaw):+.2f} deg")
        print(f"  elapsed   = {elapsed:.2f} s")
        print(f"  mean v    = {dist / elapsed:+.4f} m/s")
        print(f"  mean w    = {dyaw / elapsed:+.4f} rad/s")

        return dx, dy, dyaw, elapsed


def main():

    mode = sys.argv[1] if len(sys.argv) > 1 else "both"

    rclpy.init()

    node = DirectionCheck()

    try:

        if not node.wait_odom():

            print("[FAIL] /odom 수신 없음. wheel_odometry_node 확인.")
            return

        print()
        print("로봇 주변 공간 확보했는지 확인하고 ENTER.")
        print("(Ctrl+C 시 즉시 정지)")

        if sys.stdin.isatty():
            input()
        else:
            print("(non-interactive: 3초 후 자동 시작)")
            node.spin_for(3.0)

        if mode in ("forward", "both"):

            dx, dy, dyaw, t = node.run_motion(
                "TEST 1 : FORWARD",
                FORWARD_V,
                0.0,
                FORWARD_TIME
            )

            if dx > 0.02:
                print("  [OK] 전진 방향 정상")
            else:
                print("  [FAIL] 전진이 안 되거나 뒤로 감 -> motor sign 확인")

        if mode in ("rotate", "both"):

            if mode == "both":
                print("\n3초 후 회전 테스트 시작...")
                node.spin_for(3.0)

            dx, dy, dyaw, t = node.run_motion(
                "TEST 2 : ROTATE CCW (angular.z > 0)",
                0.0,
                ROTATE_W,
                ROTATE_TIME
            )

            if dyaw > 0.05:
                print("  [OK] angular.z>0 -> CCW 정상")
            elif dyaw < -0.05:
                print("  [FAIL] 회전 방향 반대 -> motor_cmd_swap 파라미터 반전 필요")
            else:
                print("  [FAIL] 회전 거의 없음 -> PWM/track_width 확인")

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

        print("\n[DONE] cmd_vel 0 발행 완료")


if __name__ == "__main__":
    main()
