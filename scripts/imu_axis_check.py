#!/usr/bin/env python3

"""
MPU6050 축 / 부호 / 적분오차 검증 도구  (3단계)

사용법
------
    source ~/argos_project/scripts/argos_env.sh
    ros2 run argos_odometry mpu6050_node        # 다른 터미널
    python3 ~/argos_project/scripts/imu_axis_check.py

    # wheel odometry 와 함께 비교하려면
    python3 ~/argos_project/scripts/imu_axis_check.py --with-odom

무엇을 보는가
-------------
ROS 규약(REP-103)은 base_link 에서 x=전방, y=좌측, z=상방이고
yaw 는 z 축 기준 반시계(CCW) 가 양수다.

따라서 로봇을 왼쪽(CCW)으로 돌리면 angular_velocity.z > 0 이어야 한다.

부호가 반대라면 값에 -1 을 곱하지 말고
MPU6050 의 실제 장착 방향과 base_link -> imu_link 의 회전을 먼저 본다.
TF 회전으로 표현할 수 있는 것을 코드에서 부호로 때우면
나중에 accel 을 융합할 때 반드시 어긋난다.

적분 오차
---------
MPU6050 에는 magnetometer 가 없어 절대 yaw 기준이 없다.
90 도 / 360 도 수동 회전 후 적분값이 얼마나 어긋나는지 재서
장기 드리프트의 크기를 파악한다.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


RAD_TO_DEG = 180.0 / math.pi


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class AxisCheck(Node):

    def __init__(self, with_odom: bool, threshold_dps: float):
        super().__init__("imu_axis_check")

        self.threshold = threshold_dps / RAD_TO_DEG

        # --- IMU 적분 상태 ---
        self.imu_yaw = 0.0
        self.last_imu_ns = None
        self.imu_count = 0

        self.gz = 0.0
        self.gz_peak_pos = 0.0
        self.gz_peak_neg = 0.0

        self.accel = (0.0, 0.0, 0.0)

        # 회전 방향 판정용
        self.direction = "정지"

        # --- odometry 비교 상태 ---
        self.with_odom = with_odom
        self.odom_yaw = None
        self.odom_yaw_start = None
        self.odom_yaw_unwrapped = 0.0
        self.odom_yaw_prev = None

        self.create_subscription(
            Imu, "/imu/data_raw", self.imu_cb, qos_profile_sensor_data
        )

        if with_odom:
            self.create_subscription(Odometry, "/odom", self.odom_cb, 10)

        self.create_timer(0.1, self.render)

        self.start_time = time.monotonic()

    # ---------------------------------------------------------

    def imu_cb(self, msg: Imu):

        stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000
            + msg.header.stamp.nanosec
        )

        self.gz = msg.angular_velocity.z

        self.accel = (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )

        if self.last_imu_ns is not None:
            dt = (stamp_ns - self.last_imu_ns) / 1e9

            # 시각 역행이나 비정상 간격은 적분에서 제외한다.
            if 0.0 < dt < 0.5:
                self.imu_yaw += self.gz * dt

        self.last_imu_ns = stamp_ns
        self.imu_count += 1

        if self.gz > self.gz_peak_pos:
            self.gz_peak_pos = self.gz

        if self.gz < self.gz_peak_neg:
            self.gz_peak_neg = self.gz

        if self.gz > self.threshold:
            self.direction = "CCW (왼쪽)  gz > 0"

        elif self.gz < -self.threshold:
            self.direction = "CW  (오른쪽) gz < 0"

        else:
            self.direction = "정지"

    def odom_cb(self, msg: Odometry):

        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        self.odom_yaw = yaw

        if self.odom_yaw_prev is None:
            self.odom_yaw_start = yaw
            self.odom_yaw_unwrapped = 0.0

        else:
            d = yaw - self.odom_yaw_prev
            d = math.atan2(math.sin(d), math.cos(d))
            self.odom_yaw_unwrapped += d

        self.odom_yaw_prev = yaw

    # ---------------------------------------------------------

    def render(self):

        elapsed = time.monotonic() - self.start_time

        ax, ay, az = self.accel
        norm = math.sqrt(ax * ax + ay * ay + az * az)

        line = (
            "\r"
            "t={:6.1f}s  n={:6d}  "
            "gz={:+8.4f} rad/s ({:+7.2f} deg/s)  "
            "적분 yaw={:+8.2f} deg  "
            "{:<18s}"
        ).format(
            elapsed,
            self.imu_count,
            self.gz,
            self.gz * RAD_TO_DEG,
            self.imu_yaw * RAD_TO_DEG,
            self.direction,
        )

        if self.with_odom and self.odom_yaw is not None:
            line += "  odom yaw={:+8.2f} deg  차이={:+7.2f} deg".format(
                self.odom_yaw_unwrapped * RAD_TO_DEG,
                (self.imu_yaw - self.odom_yaw_unwrapped) * RAD_TO_DEG,
            )

        line += "  |a|={:5.2f}".format(norm)

        sys.stdout.write(line)
        sys.stdout.flush()

    def summary(self):

        print("\n")
        print("=" * 68)
        print("IMU 축/부호 검증 요약")
        print("=" * 68)
        print("표본 수                : {}".format(self.imu_count))
        print(
            "적분 yaw               : {:+.2f} deg".format(
                self.imu_yaw * RAD_TO_DEG
            )
        )
        print(
            "gz 최대 (CCW 방향)     : {:+.4f} rad/s ({:+.2f} deg/s)".format(
                self.gz_peak_pos, self.gz_peak_pos * RAD_TO_DEG
            )
        )
        print(
            "gz 최소 (CW 방향)      : {:+.4f} rad/s ({:+.2f} deg/s)".format(
                self.gz_peak_neg, self.gz_peak_neg * RAD_TO_DEG
            )
        )

        if self.with_odom and self.odom_yaw_prev is not None:
            print(
                "odom 적분 yaw          : {:+.2f} deg".format(
                    self.odom_yaw_unwrapped * RAD_TO_DEG
                )
            )
            print(
                "IMU - odom             : {:+.2f} deg".format(
                    (self.imu_yaw - self.odom_yaw_unwrapped) * RAD_TO_DEG
                )
            )

        print()
        print("판정 기준")
        print("  왼쪽(CCW) 회전 -> gz 최대가 양수여야 한다")
        print("  오른쪽(CW) 회전 -> gz 최소가 음수여야 한다")
        print()
        print("부호가 반대면 코드에 -1 을 곱하지 말고")
        print("base_link -> imu_link 의 yaw 회전을 180 도 돌릴 것.")
        print("=" * 68)


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--with-odom",
        action="store_true",
        help="/odom 의 yaw 와 함께 비교한다",
    )

    parser.add_argument(
        "--threshold",
        type=float,
        default=3.0,
        help="회전으로 볼 최소 각속도 [deg/s]",
    )

    args = parser.parse_args()

    rclpy.init()

    node = AxisCheck(args.with_odom, args.threshold)

    print("=" * 68)
    print("IMU 축/부호 검증  -  Ctrl+C 로 종료하면 요약이 나온다")
    print("=" * 68)
    print("1) 로봇을 왼쪽(CCW, 반시계)으로 천천히 90 도 돌린다")
    print("2) 오른쪽(CW, 시계)으로 천천히 90 도 돌려 제자리로")
    print("3) 한 바퀴(360 도) 돌린 뒤 적분 yaw 오차를 본다")
    print("=" * 68)
    print()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.summary()

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
