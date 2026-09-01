#!/usr/bin/env python3
r"""Nav2 의 /cmd_vel 을 MCU 명령 문법으로 옮긴다.

    /follower/cmd_vel (Twist 또는 TwistStamped)
        -> /follower/motor_command (std_msgs/String)  "C,<throttle>,<yaw_rate>"

두 축의 성격이 다르다
---------------------
각속도는 정확하다.  펌웨어(followingbot_mega_fire.ino:576)가

    yawError = targetYawRate - filteredGz          # 자이로 Z 폐루프
    turnCorrection = YAW_KP * yawError

로 자이로에 대해 닫혀 있으므로 `yaw_rate` 는 물리 단위 deg/s 지령이다.
따라서 rad/s -> deg/s 는 단순 환산이고 보정이 필요 없다.  범위는 +-90.

직진은 그렇지 않다.  `throttle` 은 개루프 PWM(+-180)이라 m/s 와의 대응이
차체·배터리·바닥에 따라 달라진다.  **이 값은 반드시 실측해야 한다.**
tools/measure_drive_scale.py 가 /follower/odom_scan 으로 측정한다.

지어낸 기본값을 두지 않는다
---------------------------
`pwm_per_mps` 와 `min_moving_pwm` 은 기본값이 음수(미설정)이고, 그 상태에서는
이 노드가 **주행 명령을 내지 않고 거부한다**.  추정값을 넣어 두면 그것이
조용히 실측처럼 쓰이기 때문이다.  정지 명령은 미설정이어도 항상 나간다.

명령 문법은 follower_ws/src/follower_serial_bridge/commands.py 의
`_DRIVE = ^[Cc],(-?\d{1,3}),(-?\d{1,3}(?:\.\d{1,3})?)$` 를 따른다.
전후 반전(invert_drive)은 시리얼 브리지가 하므로 여기서 하지 않는다.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import String

# 펌웨어 한계.  followingbot_mega_fire.ino 의 주석/constrain 과 같은 값이다.
FIRMWARE_MAX_PWM = 180
FIRMWARE_MAX_YAW_DPS = 90.0


class CmdVelBridge(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_bridge")

        self.declare_parameter("cmd_vel_topic", "/follower/cmd_vel")
        self.declare_parameter("command_topic", "/follower/motor_command")
        # 타입은 지정하지 않고 둘 다 구독한다.  Nav2 의 enable_stamped_cmd_vel
        # 이 True 면 TwistStamped, False 면 Twist 로 나오는데, 한쪽만 구독하면
        # 조용히 아무것도 안 받는다 -- 2026-08-31 에 실제로 그렇게 막혔다.
        # 토픽에는 한 종류만 흐르므로 둘을 다 열어도 중복 수신은 없다.

        # --- 실측이 필요한 값.  음수 = 미설정 ---------------------------------
        # 대응은 비례가 아니라 1차식이다.  2026-08-31 실측:
        #     PWM   80   100   120   140   150   160
        #     m/s  .082 .107 .150 .164 .190 .182     (각 3 회 중앙값)
        #     pwm = 700.8 * v + 22.9,  min_moving_pwm 80,  최고 0.19 m/s
        # 절편을 빼면 v=0.19 에서 PWM 133 이 나가 실제 필요한 156 에 못 미친다.
        self.declare_parameter("pwm_per_mps", -1.0)
        self.declare_parameter("pwm_intercept", 0.0)
        self.declare_parameter("min_moving_pwm", -1)

        # --- 선택값 (안전 한계) ----------------------------------------------
        self.declare_parameter("max_pwm", 150)
        self.declare_parameter("max_yaw_rate_dps", 12.0)
        self.declare_parameter("cmd_timeout_s", 0.5)
        self.declare_parameter("zero_linear_deadband_mps", 0.01)

        self._pwm_per_mps = float(self.get_parameter("pwm_per_mps").value)
        self._pwm_intercept = float(self.get_parameter("pwm_intercept").value)
        self._min_pwm = int(self.get_parameter("min_moving_pwm").value)
        self._max_pwm = min(int(self.get_parameter("max_pwm").value), FIRMWARE_MAX_PWM)
        self._max_yaw = min(float(self.get_parameter("max_yaw_rate_dps").value),
                            FIRMWARE_MAX_YAW_DPS)
        self._timeout = float(self.get_parameter("cmd_timeout_s").value)
        self._deadband = float(self.get_parameter("zero_linear_deadband_mps").value)

        self._calibrated = self._pwm_per_mps > 0.0 and self._min_pwm > 0
        self._last_cmd_time = None
        self._stopped = True
        self._warned = False

        self._pub = self.create_publisher(
            String, str(self.get_parameter("command_topic").value), 10)

        topic = str(self.get_parameter("cmd_vel_topic").value)
        # 발행자의 타입을 물어보고 그쪽으로 구독한다.
        # 한 노드가 같은 토픽을 두 타입으로 구독하는 것은 ROS 2 가 막는다
        # ("incompatible type ... at subscription.c:112").  그래서 고르는 수밖에
        # 없고, 고정하면 Nav2 의 enable_stamped_cmd_vel 이 바뀔 때 조용히
        # 끊긴다 -- 2026-08-31 에 실제로 그렇게 막혔다.  그래서 물어본다.
        kind = self._detect_type(topic)
        if kind == "TwistStamped":
            self.create_subscription(TwistStamped, topic,
                                     lambda m: self._on_twist(m.twist), 10)
        else:
            self.create_subscription(Twist, topic, self._on_twist, 10)
        self.get_logger().info(f"{topic} 구독 타입: {kind}")
        self._seen_type = None

        self.create_timer(0.1, self._watchdog)

        if self._calibrated:
            self.get_logger().info(
                f"{topic} -> {self._pub.topic_name}  "
                f"pwm={self._pwm_per_mps:.1f}*v{self._pwm_intercept:+.1f} "
                f"min_pwm={self._min_pwm} "
                f"max_pwm={self._max_pwm} max_yaw={self._max_yaw:.1f} dps")
        else:
            self.get_logger().error(
                "pwm_per_mps / min_moving_pwm 가 설정되지 않았다. "
                "주행 명령을 내지 않는다. tools/measure_drive_scale.py 로 "
                "측정한 뒤 파라미터로 넘겨라.")

    def _detect_type(self, topic: str, timeout_s: float = 10.0) -> str:
        """토픽 발행자의 메시지 타입을 알아낸다. 못 찾으면 Twist 로 본다."""
        import time as _t
        end = _t.time() + timeout_s
        while _t.time() < end:
            for info in self.get_publishers_info_by_topic(topic):
                if info.topic_type.endswith("TwistStamped"):
                    return "TwistStamped"
                if info.topic_type.endswith("Twist"):
                    return "Twist"
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().warn(
            f"{topic} 에 발행자가 없다. Twist 로 가정한다.")
        return "Twist"

    # ------------------------------------------------------------------
    def _on_twist(self, twist: Twist) -> None:
        if self._seen_type is None:
            self._seen_type = True
            self.get_logger().info("첫 cmd_vel 수신 -- 배관 연결됨")
        self._last_cmd_time = self.get_clock().now()

        v = float(twist.linear.x)
        yaw_dps = math.degrees(float(twist.angular.z))
        yaw_dps = max(-self._max_yaw, min(self._max_yaw, yaw_dps))

        if not self._calibrated:
            if not self._warned:
                self.get_logger().error("미보정 상태 -- cmd_vel 을 버린다")
                self._warned = True
            self._send_stop()
            return

        if abs(v) < self._deadband:
            throttle = 0
        else:
            throttle = int(round(abs(v) * self._pwm_per_mps + self._pwm_intercept))
            # 데드밴드: 이보다 작으면 차체가 아예 안 움직인다.  0 을 보내면
            # Nav2 가 멈춘 채로 대기하므로, 최소 구동 PWM 까지 올린다.
            throttle = max(throttle, self._min_pwm)
            throttle = min(throttle, self._max_pwm)
            if v < 0:
                throttle = -throttle

        if throttle == 0 and abs(yaw_dps) < 0.05:
            self._send_stop()
            return

        # 문법상 소수점 3 자리까지만 허용된다.
        self._publish(f"C,{throttle},{yaw_dps:.3f}")
        self._stopped = False

    # ------------------------------------------------------------------
    def _watchdog(self) -> None:
        if self._last_cmd_time is None:
            return
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if age > self._timeout and not self._stopped:
            self.get_logger().warn(f"cmd_vel {age:.2f} s 끊김 -- 정지")
            self._send_stop()

    def _send_stop(self) -> None:
        if not self._stopped:
            self._publish("S")
            self._stopped = True

    def _publish(self, text: str) -> None:
        message = String()
        message.data = text
        self._pub.publish(message)


def main() -> None:
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._send_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
