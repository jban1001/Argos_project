#!/usr/bin/env python3

"""
LaserScan 고정 각도 격자 재샘플링

문제
----
YDLIDAR X4 Pro 는 /dev/ttyTHS1 (GPIO UART) 연결이라 모터 속도를 제어할 수 없고,
기동 직후 약 11.6 Hz 로 돌다가 3.67 Hz 로 느려진다.
그 결과 회전당 점 개수가 430 ~ 1362 로 계속 변한다.

YDLidar SDK 의 fixed_resolution 은 single-channel 라이다에서
기동 직후 측정한 평균 점 개수로 크기를 고정해 버리므로
(CYdLidar.cpp: m_FixedSize = ((mean + 5) / 10) * 10)
나중에 실제 점 개수가 늘면 한 바퀴를 잘라내 버린다.
실제로 360 deg 라고 표시된 스캔에 113 deg 구간만 담기는 상황이 발생했다.

반대로 fixed_resolution 을 끄면 점 개수가 매 스캔 달라지는데,
slam_toolbox(Karto) 는 첫 스캔에서 점 개수를 고정하고
다른 개수의 스캔은 통째로 버린다.
    LaserRangeScan contains 1358 range readings, expected 1360

해결
----
드라이버 출력(/scan_raw)을 고정 크기 각도 격자로 재샘플링해서
항상 같은 점 개수의 /scan 을 발행한다.
라이다 회전 속도가 변해도 하위 노드는 영향을 받지 않는다.

한 bin 에 여러 점이 들어오면 최소 거리를 취한다 (장애물 기준 보수적).
비어 있는 bin 은 무효값으로 둔다.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


class ScanNormalizer(Node):

    def __init__(self):

        super().__init__("scan_normalizer")

        self.declare_parameter("input_topic", "/scan_raw")
        self.declare_parameter("output_topic", "/scan")

        # 출력 격자 크기. 1440 = 0.25 deg 로 센서 원래 분해능(0.265 deg)에 맞춘다.
        # 720(0.5 deg)으로 하면 1351 개 측정점이 658 개 bin 으로 뭉개져
        # 측정값의 절반을 버리게 되고 스캔 매칭 정밀도가 그만큼 떨어진다.
        self.declare_parameter("num_bins", 1440)

        # 빈 bin 에 넣을 값. inf 로 두면 costmap 이 그 방향을 비울 수 있다.
        self.declare_parameter("empty_is_inf", False)

        # LaserScan 규약상 header.stamp 는 첫 번째 광선의 시각이다.
        # (YDLidar SDK: outscan.stamp = global_nodes[0].stamp)
        # 그런데 이 라이다는 한 바퀴에 0.27 초가 걸리므로,
        # 그 시각의 자세로 스캔 전체를 정합하면 평균 scan_time/2 만큼
        # 뒤처진 자세를 쓰게 된다. 0.25 rad/s 회전 중이면 약 2 deg 계통 오차이고,
        # 회전 방향이 바뀔 때마다 반대로 틀어져 벽이 이중으로 찍힌다.
        # stamp 를 스캔 중앙 시각으로 옮겨 이 편향을 없앤다.
        self.declare_parameter("stamp_at_midpoint", True)

        self.num_bins = int(self.get_parameter("num_bins").value)
        self.empty_is_inf = bool(
            self.get_parameter("empty_is_inf").value
        )

        self.stamp_at_midpoint = bool(
            self.get_parameter("stamp_at_midpoint").value
        )

        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)

        self.pub = self.create_publisher(
            LaserScan, out_topic, qos_profile_sensor_data
        )

        self.sub = self.create_subscription(
            LaserScan, in_topic, self.scan_cb, qos_profile_sensor_data
        )

        self.angle_min = -np.pi
        self.angle_max = np.pi

        self.angle_increment = (
            (self.angle_max - self.angle_min) / self.num_bins
        )

        self.count = 0
        self.dropped = 0

        self.get_logger().info(
            f"{in_topic} -> {out_topic}, "
            f"{self.num_bins} bins "
            f"({np.degrees(self.angle_increment):.3f} deg/bin)"
        )

    def scan_cb(self, msg):

        n = len(msg.ranges)

        if n < 2:
            return

        r = np.asarray(msg.ranges, dtype=np.float64)

        ang = (
            msg.angle_min
            + np.arange(n) * msg.angle_increment
        )

        valid = (
            np.isfinite(r)
            & (r > msg.range_min)
            & (r < msg.range_max)
        )

        # 각도를 [-pi, pi) 로 정규화
        a = np.arctan2(np.sin(ang), np.cos(ang))

        idx = np.floor(
            (a - self.angle_min) / self.angle_increment
        ).astype(np.int64)

        np.clip(idx, 0, self.num_bins - 1, out=idx)

        out = np.full(self.num_bins, np.inf, dtype=np.float64)

        vi = idx[valid]
        vr = r[valid]

        if vi.size:
            # bin 당 최소 거리
            np.minimum.at(out, vi, vr)

        filled = int(np.isfinite(out).sum())

        if not self.empty_is_inf:
            out[~np.isfinite(out)] = 0.0

        new = LaserScan()

        new.header = msg.header

        if self.stamp_at_midpoint and msg.scan_time > 0.0:

            shift_ns = int(msg.scan_time * 0.5 * 1e9)

            total = (
                msg.header.stamp.sec * 1_000_000_000
                + msg.header.stamp.nanosec
                + shift_ns
            )

            new.header.stamp.sec = total // 1_000_000_000
            new.header.stamp.nanosec = total % 1_000_000_000

        new.angle_min = float(self.angle_min)
        new.angle_max = float(
            self.angle_min
            + (self.num_bins - 1) * self.angle_increment
        )
        new.angle_increment = float(self.angle_increment)

        new.time_increment = float(
            msg.scan_time / self.num_bins
            if msg.scan_time > 0.0 else 0.0
        )
        new.scan_time = float(msg.scan_time)

        new.range_min = float(msg.range_min)
        new.range_max = float(msg.range_max)

        new.ranges = out.astype(np.float32).tolist()
        new.intensities = []

        self.pub.publish(new)

        self.count += 1

        if self.count % 100 == 1:

            self.get_logger().info(
                f"in {n} pts -> out {self.num_bins} bins, "
                f"{filled} filled ({100.0*filled/self.num_bins:.0f}%)"
            )


def main(args=None):

    rclpy.init(args=args)

    node = ScanNormalizer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
