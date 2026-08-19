#!/usr/bin/env python3

"""
ARGOS PWM <-> wheel speed 실측

제자리 회전만 사용하므로 주행 공간이 필요 없다.

base_driver 의 현재 매핑:
    pwm = speed / max_wheel_speed * max_pwm

따라서 원하는 PWM 을 만들려면
    speed = pwm / max_pwm * max_wheel_speed
    w     = 2 * speed / track_width
로 /cmd_vel 을 준다.

측정은 /odom 의 yaw 변화로부터
    wheel_speed = |w| * track_width / 2
로 역산한다.
"""

import math
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


import yaml

CONFIG = "/home/odyssey/argos_project/config/base_driver.yaml"

with open(CONFIG) as f:
    _P = yaml.safe_load(f)["argos_base_driver"]["ros__parameters"]

TRACK_WIDTH = float(_P["track_width"])

MAX_WHEEL_SPEED = float(_P["max_wheel_speed"])
MAX_PWM = int(_P["max_pwm"])

PWM_DEADBAND = float(_P["pwm_deadband"])
PWM_GAIN = float(_P["pwm_gain"])


def speed_for_pwm(pwm):
    """
    base_driver 의 speed->pwm 매핑의 역함수.
    원하는 PWM 이 실제로 나가도록 cmd_vel 을 만든다.
    """

    return (pwm - PWM_DEADBAND) / PWM_GAIN

import sys

if len(sys.argv) > 1:
    PWM_LEVELS = [int(a) for a in sys.argv[1:]]
else:
    PWM_LEVELS = [55, 70, 85, 100, 120]

RAMP_TIME = 0.7
RUN_TIME = 1.3
PAUSE_TIME = 1.2


def yaw_from_odom(msg):

    q = msg.pose.pose.orientation

    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class PwmSpeedCalib(Node):

    def __init__(self):

        super().__init__("calibrate_pwm_speed")

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

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

    def measure(self, pwm, direction):
        """
        정상상태 각속도를 /odom twist 로 직접 평균낸다.

        변위/시간 방식은 가감속 구간과 관성 활강(coast) 때문에
        오차가 크므로 사용하지 않는다.
        """

        wheel_speed_cmd = speed_for_pwm(pwm)

        w_cmd = direction * 2.0 * wheel_speed_cmd / TRACK_WIDTH

        t0 = time.monotonic()

        # 1) 가속 구간은 버린다
        while time.monotonic() - t0 < RAMP_TIME:

            self.publish(0.0, w_cmd)

            rclpy.spin_once(self, timeout_sec=0.01)

            time.sleep(0.02)

        # 2) 정상상태 구간만 평균
        samples = []

        t1 = time.monotonic()

        while time.monotonic() - t1 < RUN_TIME:

            self.publish(0.0, w_cmd)

            rclpy.spin_once(self, timeout_sec=0.01)

            if self.odom is not None:
                samples.append(self.odom.twist.twist.angular.z)

            time.sleep(0.02)

        self.stop()

        if not samples:
            w_meas = 0.0
        else:
            w_meas = sum(samples) / len(samples)

        wheel_speed_meas = abs(w_meas) * TRACK_WIDTH / 2.0

        return {
            "pwm": pwm,
            "dir": direction,
            "w_cmd": w_cmd,
            "w_meas": w_meas,
            "speed_cmd": wheel_speed_cmd,
            "speed_meas": wheel_speed_meas,
            "n": len(samples),
        }


def main():

    rclpy.init()

    node = PwmSpeedCalib()

    results = []

    try:

        if not node.wait_odom():
            print("[FAIL] /odom 없음")
            return

        print()
        print("=" * 62)
        print(" ARGOS PWM <-> WHEEL SPEED CALIBRATION (in-place rotation)")
        print("=" * 62)
        print(f"{'PWM':>5} {'dir':>4} {'w_cmd':>9} {'w_meas':>9} "
              f"{'speed_meas':>11}")

        direction = 1.0

        for pwm in PWM_LEVELS:

            r = node.measure(pwm, direction)

            results.append(r)

            print(
                f"{r['pwm']:>5} {int(r['dir']):>4} "
                f"{r['w_cmd']:>9.3f} {r['w_meas']:>9.3f} "
                f"{r['speed_meas']:>11.4f}"
            )

            # 좌우 번갈아 회전해서 제자리 유지
            direction = -direction

            node.spin_for(PAUSE_TIME)

        # ---------------------------------------------
        # 선형 회귀 : speed = gain * (pwm - pwm_dead)
        # ---------------------------------------------

        xs = [r["pwm"] for r in results]
        ys = [r["speed_meas"] for r in results]

        n = len(xs)

        mx = sum(xs) / n
        my = sum(ys) / n

        num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
        den = sum((x - mx) ** 2 for x in xs)

        slope = num / den if den else 0.0
        intercept = my - slope * mx

        pwm_dead = -intercept / slope if slope else 0.0

        speed_at_max = slope * MAX_PWM + intercept

        print()
        print("-" * 62)
        print(f" speed = {slope:.6f} * pwm + ({intercept:.6f})")
        print(f" deadband pwm       = {pwm_dead:.1f}")
        print(f" speed @ pwm {MAX_PWM}   = {speed_at_max:.4f} m/s")
        print("-" * 62)
        print()
        print(" base_driver.yaml 권장값:")
        print(f"   max_wheel_speed: {speed_at_max:.4f}")
        print(f"   pwm_deadband:    {pwm_dead:.1f}")
        print(f"   pwm_gain:        {1.0/slope:.2f}   # pwm per (m/s)")
        print()

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
