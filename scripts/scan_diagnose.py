#!/usr/bin/env python3

"""
LiDAR 주기 / timestamp 진단  (8단계)

    python3 scan_diagnose.py [--seconds 60]

IMU 를 넣어도 LiDAR 자체 문제는 그대로 남는다.
그래서 IMU 통합 성공 여부와 분리해서 따로 잰다.

무엇을 재는가
-------------
  /scan_raw 와 /scan 의
    - 평균 주기와 표준편차
    - 순간적인 끊김 (평균의 2 배를 넘는 간격)
    - header.stamp 역행 (뒤로 가는 timestamp)
    - stamp 와 실제 수신 시각의 차이
    - scan_time / time_increment 가 실제 주기와 맞는지
    - 유효 측정점 비율

판단 기준
---------
  정지 상태에서도 벽이 이중으로 보이면
      -> LiDAR 장착 진동, 전원, UART, 스캔 방향, 거리 품질을 먼저 본다
  회전 중에만 이중선 간격이 커지면
      -> odometry yaw, gyro bias, scan timestamp, motion distortion 을 본다
  주기가 크게 변하거나 timestamp 가 부정확하면
      -> IMU 통합과 별개의 문제로 보고한다
"""

from __future__ import annotations

import argparse
import math
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


class TopicStats:

    def __init__(self, name: str):
        self.name = name

        self.recv_times = []       # 수신 시각 (monotonic)
        self.stamps = []           # header.stamp [s]
        self.gaps = []             # 수신 간격
        self.stamp_gaps = []       # stamp 간격

        self.backwards = 0         # timestamp 역행 횟수
        self.count = 0

        self.scan_time = None
        self.time_increment = None
        self.angle_min = None
        self.angle_max = None
        self.n_points = None

        self.valid_ratios = []

        self._last_recv = None
        self._last_stamp = None

    def add(self, msg: LaserScan):

        now = time.monotonic()

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self._last_recv is not None:
            self.gaps.append(now - self._last_recv)

        if self._last_stamp is not None:
            d = stamp - self._last_stamp
            self.stamp_gaps.append(d)

            if d <= 0.0:
                self.backwards += 1

        self._last_recv = now
        self._last_stamp = stamp

        self.recv_times.append(now)
        self.stamps.append(stamp)

        self.count += 1

        self.scan_time = msg.scan_time
        self.time_increment = msg.time_increment
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.n_points = len(msg.ranges)

        valid = sum(
            1 for r in msg.ranges
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max
        )

        if self.n_points:
            self.valid_ratios.append(valid / self.n_points)

    def report(self):

        print()
        print("=" * 66)
        print("토픽 : {}".format(self.name))
        print("=" * 66)

        if self.count < 3:
            print("메시지가 {} 개뿐이라 통계를 낼 수 없다.".format(self.count))
            return

        mean_gap = statistics.fmean(self.gaps)
        std_gap = statistics.pstdev(self.gaps)

        print("메시지 수            : {}".format(self.count))
        print("평균 주기            : {:.4f} s  ({:.3f} Hz)".format(
            mean_gap, 1.0 / mean_gap))
        print("주기 표준편차        : {:.4f} s  ({:.1f} % of mean)".format(
            std_gap, std_gap / mean_gap * 100.0))
        print("주기 최소 / 최대     : {:.4f} / {:.4f} s".format(
            min(self.gaps), max(self.gaps)))

        # 평균의 2 배를 넘으면 한 바퀴를 통째로 놓친 것으로 본다.
        dropouts = [g for g in self.gaps if g > 2.0 * mean_gap]

        print("끊김 (>2x 평균)      : {} 회".format(len(dropouts)))

        if dropouts:
            print("  최대 끊김          : {:.4f} s".format(max(dropouts)))

        print()
        print("timestamp 역행       : {} 회".format(self.backwards))

        if self.stamp_gaps:
            mean_sgap = statistics.fmean(self.stamp_gaps)
            print("stamp 평균 간격      : {:.4f} s  ({:.3f} Hz)".format(
                mean_sgap, 1.0 / mean_sgap if mean_sgap else float("nan")))

        # stamp 와 수신 시각의 상대 흐름 비교.
        # 절대 offset 은 clock 기준이 달라 의미가 없고, 기울기가 중요하다.
        span_recv = self.recv_times[-1] - self.recv_times[0]
        span_stamp = self.stamps[-1] - self.stamps[0]

        print("수신 경과 / stamp 경과: {:.3f} s / {:.3f} s  (차이 {:+.3f} s)".format(
            span_recv, span_stamp, span_stamp - span_recv))

        print()
        print("scan_time            : {:.6f} s".format(self.scan_time))
        print("time_increment       : {:.8f} s".format(self.time_increment))
        print("점 개수              : {}".format(self.n_points))

        # time_increment * (점 개수) 가 실제 한 바퀴 시간과 맞아야 한다.
        implied = self.time_increment * self.n_points

        print("time_increment x 점수 : {:.4f} s".format(implied))
        print("  실제 평균 주기      : {:.4f} s".format(mean_gap))

        if implied > 0 and abs(implied - mean_gap) > 0.2 * mean_gap:
            print("  [주의] 20% 이상 어긋난다. motion distortion 보정에 영향.")

        if self.scan_time > 0 and abs(self.scan_time - mean_gap) > 0.2 * mean_gap:
            print("  [주의] scan_time 이 실제 주기와 20% 이상 다르다.")

        print()
        print("각도 범위            : {:.4f} ~ {:.4f} rad ({:.1f} ~ {:.1f} deg)".format(
            self.angle_min, self.angle_max,
            math.degrees(self.angle_min), math.degrees(self.angle_max)))

        if self.valid_ratios:
            print("유효 측정점 비율     : 평균 {:.1f} %  최소 {:.1f} %".format(
                statistics.fmean(self.valid_ratios) * 100.0,
                min(self.valid_ratios) * 100.0))

        print("=" * 66)


class ScanDiagnose(Node):

    def __init__(self, seconds: float):
        super().__init__("scan_diagnose")

        self.seconds = seconds

        self.raw = TopicStats("/scan_raw")
        self.norm = TopicStats("/scan")

        self.create_subscription(
            LaserScan, "/scan_raw", self.raw.add, qos_profile_sensor_data
        )

        self.create_subscription(
            LaserScan, "/scan", self.norm.add, qos_profile_sensor_data
        )

        self.t0 = time.monotonic()

    def done(self) -> bool:
        return time.monotonic() - self.t0 >= self.seconds


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=60.0)
    args = parser.parse_args()

    rclpy.init()
    node = ScanDiagnose(args.seconds)

    print("LiDAR 진단 {:.0f} 초...".format(args.seconds))

    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    node.raw.report()
    node.norm.report()

    print()
    print("해석 지침")
    print("  주기 표준편차가 평균의 10% 를 넘거나 끊김이 잦으면")
    print("  UART(/dev/ttyTHS1), 전원, 케이블을 먼저 본다.")
    print("  이건 IMU 를 넣어도 해결되지 않는다.")

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
