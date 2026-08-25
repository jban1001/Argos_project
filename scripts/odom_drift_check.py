#!/usr/bin/env python3

"""
정지 상태 /odom drift 측정  (9단계 B)

    python3 odom_drift_check.py [--seconds 30] [--topic /odom]

정지한 로봇에서 /odom 의 yaw 와 위치가 얼마나 흘러가는지 잰다.
wheel-only 와 EKF 를 같은 방법으로 재서 비교하려고 만들었다.
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


RAD_TO_DEG = 180.0 / math.pi


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class DriftCheck(Node):

    def __init__(self, topic: str, seconds: float):
        super().__init__("odom_drift_check")

        self.seconds = seconds

        self.first = None
        self.last = None
        self.count = 0

        self.yaw_unwrapped = 0.0
        self.yaw_prev = None

        self.vx_max = 0.0
        self.wz_max = 0.0

        self.create_subscription(Odometry, topic, self.cb, 10)

        self.t0 = time.monotonic()

    def cb(self, msg: Odometry):

        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        p = msg.pose.pose.position

        if self.first is None:
            self.first = (p.x, p.y, yaw)
            self.yaw_prev = yaw

        else:
            d = yaw - self.yaw_prev
            d = math.atan2(math.sin(d), math.cos(d))
            self.yaw_unwrapped += d
            self.yaw_prev = yaw

        self.last = (p.x, p.y, yaw)
        self.count += 1

        self.vx_max = max(self.vx_max, abs(msg.twist.twist.linear.x))
        self.wz_max = max(self.wz_max, abs(msg.twist.twist.angular.z))

    def done(self) -> bool:
        return time.monotonic() - self.t0 >= self.seconds

    def report(self, topic: str):

        elapsed = time.monotonic() - self.t0

        print()
        print("=" * 62)
        print("정지 상태 drift : {}".format(topic))
        print("=" * 62)

        if self.first is None:
            print("메시지를 하나도 못 받았다.")
            return

        x0, y0, _ = self.first
        x1, y1, _ = self.last

        dx = x1 - x0
        dy = y1 - y0
        dist = math.hypot(dx, dy)

        yaw_deg = self.yaw_unwrapped * RAD_TO_DEG

        print("측정 시간          : {:.1f} s".format(elapsed))
        print("메시지 수          : {}".format(self.count))
        print("실제 주기          : {:.2f} Hz".format(self.count / elapsed))
        print()
        print("위치 drift         : {:.5f} m  (dx={:+.5f} dy={:+.5f})".format(
            dist, dx, dy))
        print("yaw drift          : {:+.4f} deg".format(yaw_deg))
        print("yaw drift 속도     : {:+.4f} deg/min".format(
            yaw_deg / elapsed * 60.0))
        print()
        print("정지 중 |vx| 최대  : {:.5f} m/s".format(self.vx_max))
        print("정지 중 |wz| 최대  : {:.5f} rad/s".format(self.wz_max))
        print("=" * 62)


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--topic", type=str, default="/odom")
    args = parser.parse_args()

    rclpy.init()
    node = DriftCheck(args.topic, args.seconds)

    print("{} 에서 {:.0f} 초간 정지 drift 측정...".format(
        args.topic, args.seconds))

    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)

        node.report(args.topic)

    except KeyboardInterrupt:
        node.report(args.topic)

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
