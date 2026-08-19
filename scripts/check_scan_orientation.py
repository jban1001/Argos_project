#!/usr/bin/env python3

"""
LiDAR 스캔 방향(좌우 반전) 검증 + odom yaw 스케일 교차 검증

원리
----
로봇이 정지 -> 스캔 A 취득 -> CCW 로 delta 만큼 회전 -> 정지 -> 스캔 B 취득.

로봇이 +delta (CCW) 회전하면 동일한 정지 물체의 로봇 기준 방위각은
delta 만큼 감소한다. 따라서

    f_B(theta) = f_A(theta + delta)

즉 index shift s = delta / angle_increment 가 양수로 나와야 한다.

s 의 부호가 반대로 나오면 스캔이 좌우 반전된 것이므로
ydlidar 설정의 inverted / reversion 을 뒤집어야 한다.

또한 |s * angle_increment| 와 odom 의 |delta| 를 비교하면
LiDAR 를 기준으로 한 track_width(회전 스케일) 검증도 된다.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


ROTATE_W = 0.25
ROTATE_TIME = 2.0

SETTLE_TIME = 1.5


def yaw_from_odom(msg):

    q = msg.pose.pose.orientation

    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class ScanOrientationCheck(Node):

    def __init__(self):

        super().__init__("check_scan_orientation")

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.scan = None
        self.odom = None

    def scan_callback(self, msg):
        self.scan = msg

    def odom_callback(self, msg):
        self.odom = msg

    def spin_for(self, duration):

        end = time.monotonic() + duration

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

        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)

        self.pub.publish(msg)

    def stop(self):

        for _ in range(15):
            self.publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    def grab_scan(self):
        """정지 상태에서 새 스캔 1장을 확실히 받는다."""

        self.scan = None

        end = time.monotonic() + 5.0

        while self.scan is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

        return self.scan

    def rotate(self, w, duration):

        yaw0 = yaw_from_odom(self.odom)

        end = time.monotonic() + duration

        while time.monotonic() < end:

            self.publish(0.0, w)

            rclpy.spin_once(self, timeout_sec=0.01)

            time.sleep(0.02)

        self.stop()

        self.spin_for(SETTLE_TIME)

        yaw1 = yaw_from_odom(self.odom)

        d = yaw1 - yaw0

        return math.atan2(math.sin(d), math.cos(d))


def valid_mask(scan):

    out = []

    for r in scan.ranges:

        ok = (
            not math.isinf(r)
            and not math.isnan(r)
            and scan.range_min < r < scan.range_max
        )

        out.append(ok)

    return out


def best_shift(a, b, max_shift):
    """
    f_B(i) ~= f_A(i + s) 가 되는 s 를 찾는다.
    유효 점만 사용한 평균 제곱 오차 최소화.
    """

    n = len(a.ranges)

    ma = valid_mask(a)
    mb = valid_mask(b)

    ra = a.ranges
    rb = b.ranges

    best = None

    scores = []

    for s in range(-max_shift, max_shift + 1):

        total = 0.0
        count = 0

        for i in range(0, n, 2):   # 2칸 간격 샘플링 (속도)

            j = (i + s) % n

            if not mb[i] or not ma[j]:
                continue

            d = rb[i] - ra[j]

            total += d * d
            count += 1

        if count < n // 8:
            continue

        mse = total / count

        scores.append((mse, s, count))

        if best is None or mse < best[0]:
            best = (mse, s, count)

    return best, scores


def main():

    rclpy.init()

    node = ScanOrientationCheck()

    try:

        if not node.wait_data():
            print("[FAIL] /scan 또는 /odom 수신 실패")
            return

        print()
        print("=" * 60)
        print(" LiDAR SCAN ORIENTATION CHECK")
        print("=" * 60)

        node.spin_for(1.0)

        scan_a = node.grab_scan()

        if scan_a is None:
            print("[FAIL] scan A 취득 실패")
            return

        inc = scan_a.angle_increment

        print(f" angle_increment = {inc:.6f} rad "
              f"({math.degrees(inc):.4f} deg)")
        print(f" points          = {len(scan_a.ranges)}")
        print()
        print(f" CCW 회전 : w={ROTATE_W}, {ROTATE_TIME}s")

        dyaw = node.rotate(ROTATE_W, ROTATE_TIME)

        scan_b = node.grab_scan()

        if scan_b is None:
            print("[FAIL] scan B 취득 실패")
            return

        if len(scan_b.ranges) != len(scan_a.ranges):
            print("[FAIL] scan 길이 불일치")
            return

        print(f" odom d_yaw      = {math.degrees(dyaw):+.2f} deg")

        max_shift = int(abs(dyaw) / inc * 2.5) + 10

        best, scores = best_shift(scan_a, scan_b, max_shift)

        if best is None:
            print("[FAIL] 유효 점 부족 - 상관 계산 불가")
            return

        mse, s, count = best

        d_scan = s * inc

        print(f" scan shift      = {s} index "
              f"= {math.degrees(d_scan):+.2f} deg")
        print(f" match mse       = {mse:.5f} (n={count})")
        print()

        if abs(dyaw) < math.radians(5):
            print(" [FAIL] 회전량이 너무 작아 판정 불가")
            return

        if d_scan * dyaw > 0:
            print(" [OK] 스캔 방향 정상 (반전 없음)")
            print("      -> ydlidar inverted/reversion 설정 유지")
        else:
            print(" [FAIL] 스캔이 좌우 반전되어 있음")
            print("      -> ydlidar 설정에서 inverted 를 뒤집어야 함")
            print("      -> 이 상태로 SLAM 하면 맵이 반드시 깨진다")

        ratio = abs(d_scan) / abs(dyaw)

        print()
        print(f" |scan| / |odom| = {ratio:.3f}")

        if 0.88 <= ratio <= 1.12:
            print(" [OK] odom yaw 스케일 정상 (track_width 신뢰 가능)")
        else:
            print(" [WARN] odom yaw 스케일 오차 큼")
            print(f"        보정 track_width ~ "
                  f"{0.496243 * ratio:.6f} m")

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
