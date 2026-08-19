#!/usr/bin/env python3

"""
ARGOS base driver

- /dev/ttyCH341USB0 : 오른쪽 UNO (motor command TX + right encoder RX)
- /dev/ttyCH341USB1 : 왼쪽 UNO   (left encoder RX)

subscribe : /cmd_vel
publish   : /wheel_ticks/left, /wheel_ticks/right

모터 명령 형식:  "M,<a>,<b>\\n"

중요 (하드웨어 실측 결과):
    calibrate_rotation_only.py 가 M,+55,-55 를 보냈을 때
    LEFT  distance = -1.556 m
    RIGHT distance = +1.561 m
    -> 왼쪽 뒤로 / 오른쪽 앞으로 = CCW (yaw +)

    따라서 M,a,b 의 a 는 RIGHT motor, b 는 LEFT motor 이다.
    이 순서는 motor_cmd_swap 파라미터로 제어한다.
"""

import re
import time
import atexit
import signal

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64
from geometry_msgs.msg import Twist, TwistStamped

import serial


class ArgosBaseDriver(Node):

    def __init__(self):

        super().__init__("argos_base_driver")

        # 생성자 중간에 실패해도 close() 가 안전하도록 먼저 선언
        self.right_ser = None
        self.left_ser = None

        self._right_buf = b""
        self._left_buf = b""

        self._stopped = False

        # -----------------------------
        # Parameters
        # -----------------------------

        self.declare_parameter("right_port", "/dev/ttyCH341USB0")
        self.declare_parameter("left_port", "/dev/ttyCH341USB1")
        self.declare_parameter("baudrate", 115200)

        # calibration 에서 얻은 effective track width
        self.declare_parameter("track_width", 0.496243)

        # 속도 <-> PWM 변환 (실측 선형 모델)
        #   speed = 0.00088689 * pwm - 0.00546725   (R^2 = 0.9994)
        #   -> pwm = pwm_deadband + speed * pwm_gain
        # calibrate_pwm_speed.py 로 재측정 가능
        self.declare_parameter("max_wheel_speed", 0.1010)
        self.declare_parameter("pwm_deadband", 6.16)
        self.declare_parameter("pwm_gain", 1127.54)
        self.declare_parameter("min_pwm", 10)
        self.declare_parameter("max_pwm", 120)

        # /cmd_vel 안전 제한
        self.declare_parameter("max_linear_speed", 0.30)
        self.declare_parameter("max_angular_speed", 1.20)

        # watchdog
        self.declare_parameter("cmd_timeout", 0.5)

        # M,a,b 에서 a=RIGHT, b=LEFT 이면 True
        self.declare_parameter("motor_cmd_swap", True)

        # 개별 모터 극성 보정
        self.declare_parameter("left_motor_sign", 1.0)
        self.declare_parameter("right_motor_sign", 1.0)

        # Arduino auto-reset 대기
        self.declare_parameter("startup_delay", 2.5)

        # Nav2 가 TwistStamped 를 쓸 때만 True
        self.declare_parameter("use_stamped_cmd_vel", False)

        g = self.get_parameter

        self.right_port = str(g("right_port").value)
        self.left_port = str(g("left_port").value)
        self.baudrate = int(g("baudrate").value)

        self.track_width = float(g("track_width").value)

        self.max_wheel_speed = float(g("max_wheel_speed").value)
        self.pwm_deadband = float(g("pwm_deadband").value)
        self.pwm_gain = float(g("pwm_gain").value)
        self.min_pwm = int(g("min_pwm").value)
        self.max_pwm = int(g("max_pwm").value)

        self.max_linear_speed = float(g("max_linear_speed").value)
        self.max_angular_speed = float(g("max_angular_speed").value)

        self.cmd_timeout = float(g("cmd_timeout").value)

        self.motor_cmd_swap = bool(g("motor_cmd_swap").value)

        self.left_motor_sign = float(g("left_motor_sign").value)
        self.right_motor_sign = float(g("right_motor_sign").value)

        self.startup_delay = float(g("startup_delay").value)

        self.use_stamped_cmd_vel = bool(g("use_stamped_cmd_vel").value)

        # -----------------------------
        # Serial
        # -----------------------------

        self.right_ser = serial.Serial(
            self.right_port,
            self.baudrate,
            timeout=0
        )

        self.left_ser = serial.Serial(
            self.left_port,
            self.baudrate,
            timeout=0
        )

        self.get_logger().info(
            f"RIGHT / MOTOR : {self.right_port}"
        )

        self.get_logger().info(
            f"LEFT          : {self.left_port}"
        )

        self.get_logger().info(
            "Waiting for Arduino startup..."
        )

        time.sleep(self.startup_delay)

        self.right_ser.reset_input_buffer()
        self.left_ser.reset_input_buffer()

        # 시작하자마자 정지 상태 보장
        self.stop_motor()

        # -----------------------------
        # ROS publishers
        # -----------------------------

        self.left_pub = self.create_publisher(
            Int64,
            "/wheel_ticks/left",
            10
        )

        self.right_pub = self.create_publisher(
            Int64,
            "/wheel_ticks/right",
            10
        )

        # -----------------------------
        # ROS subscriber
        # -----------------------------

        if self.use_stamped_cmd_vel:

            self.cmd_sub = self.create_subscription(
                TwistStamped,
                "/cmd_vel",
                self.cmd_stamped_callback,
                10
            )

        else:

            self.cmd_sub = self.create_subscription(
                Twist,
                "/cmd_vel",
                self.cmd_callback,
                10
            )

        # -----------------------------
        # Motor state
        # -----------------------------

        self.left_pwm = 0
        self.right_pwm = 0

        self.last_cmd_time = time.monotonic()

        self.watchdog_active = True

        # 100Hz serial read
        self.read_timer = self.create_timer(
            0.01,
            self.read_encoders
        )

        # 20Hz motor command
        self.motor_timer = self.create_timer(
            0.05,
            self.send_motor_command
        )

        self.get_logger().info(
            "motor_cmd_swap = "
            f"{self.motor_cmd_swap} "
            "(True -> 'M,right,left')"
        )

        self.get_logger().info(
            f"pwm = {self.pwm_deadband:.2f} + speed * "
            f"{self.pwm_gain:.2f}  "
            f"(max_wheel_speed = {self.max_wheel_speed:.4f} m/s)"
        )

        self.get_logger().info(
            "ARGOS BASE DRIVER READY"
        )

    # =====================================================
    # Encoder
    # =====================================================

    def parse_count(self, text):

        text = text.strip()

        if not text:
            return None

        # 모터 명령 echo 는 무시
        if text.startswith("M,"):
            return None

        if re.fullmatch(r"[+-]?\d+", text):
            return int(text)

        numbers = re.findall(r"-?\d+", text)

        if not numbers:
            return None

        return int(numbers[-1])

    def read_lines(self, ser, buf):
        """
        timeout=0 인 포트에서 완결된 line 만 추출한다.

        readline() 을 그대로 쓰면 '\\n' 이 아직 안 온 상태에서
        잘린 숫자가 반환되어 tick 이 튀는 문제가 있으므로
        반드시 buffer 를 누적한 뒤 '\\n' 기준으로 분리한다.
        """

        try:
            data = ser.read(4096)

        except Exception as e:

            self.get_logger().error(
                f"SERIAL READ: {e}"
            )

            return buf, []

        if not data:
            return buf, []

        buf = buf + data

        if b"\n" not in buf:

            # 비정상적으로 긴 rubbish 는 잘라낸다
            if len(buf) > 512:
                buf = buf[-256:]

            return buf, []

        parts = buf.split(b"\n")

        remainder = parts[-1]

        if len(remainder) > 256:
            remainder = remainder[-256:]

        lines = [
            p.decode("utf-8", errors="ignore").strip()
            for p in parts[:-1]
        ]

        return remainder, lines

    def read_encoders(self):

        # RIGHT
        self._right_buf, right_lines = self.read_lines(
            self.right_ser,
            self._right_buf
        )

        for text in right_lines:

            value = self.parse_count(text)

            if value is None:
                continue

            msg = Int64()
            msg.data = value

            self.right_pub.publish(msg)

        # LEFT
        self._left_buf, left_lines = self.read_lines(
            self.left_ser,
            self._left_buf
        )

        for text in left_lines:

            value = self.parse_count(text)

            if value is None:
                continue

            msg = Int64()
            msg.data = value

            self.left_pub.publish(msg)

    # =====================================================
    # cmd_vel
    # =====================================================

    def cmd_stamped_callback(self, msg):

        self.cmd_callback(msg.twist)

    def cmd_callback(self, msg):

        v = float(msg.linear.x)
        w = float(msg.angular.z)

        v = max(
            -self.max_linear_speed,
            min(self.max_linear_speed, v)
        )

        w = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, w)
        )

        # differential / skid-steer kinematics
        left_speed = v - w * self.track_width / 2.0
        right_speed = v + w * self.track_width / 2.0

        # 좌/우 wheel 중 하나라도 최대 속도를 넘으면
        # 둘 다 같은 비율로 줄여서 Nav2가 요구한 곡률을 유지한다.
        peak = max(abs(left_speed), abs(right_speed))

        if peak > self.max_wheel_speed:
            scale = self.max_wheel_speed / peak
            left_speed *= scale
            right_speed *= scale

        self.left_pwm = self.speed_to_pwm(
            left_speed * self.left_motor_sign
        )

        self.right_pwm = self.speed_to_pwm(
            right_speed * self.right_motor_sign
        )

        self.last_cmd_time = time.monotonic()

    def speed_to_pwm(self, speed):
        """
        실측 선형 모델의 역함수.

            speed = (pwm - pwm_deadband) / pwm_gain
            pwm   = pwm_deadband + speed * pwm_gain

        deadband 를 보상하므로 명령 속도와 실제 속도가 일치한다.
        """

        if abs(speed) < 0.002:
            return 0

        speed = max(
            -self.max_wheel_speed,
            min(self.max_wheel_speed, speed)
        )

        sign = 1.0 if speed > 0.0 else -1.0

        pwm = int(round(
            self.pwm_deadband +
            abs(speed) * self.pwm_gain
        ))

        pwm = max(
            self.min_pwm,
            min(self.max_pwm, pwm)
        )

        return int(sign * pwm)

    # =====================================================
    # Motor
    # =====================================================

    def format_command(self, left_pwm, right_pwm):

        if self.motor_cmd_swap:
            # M,a,b -> a = RIGHT, b = LEFT
            return f"M,{right_pwm},{left_pwm}\n"

        return f"M,{left_pwm},{right_pwm}\n"

    def send_motor_command(self):

        if self._stopped:
            return

        # cmd_vel watchdog
        if self.watchdog_active:

            if (
                time.monotonic() - self.last_cmd_time
                > self.cmd_timeout
            ):

                self.left_pwm = 0
                self.right_pwm = 0

        command = self.format_command(
            self.left_pwm,
            self.right_pwm
        )

        try:

            self.right_ser.write(
                command.encode()
            )

        except Exception as e:

            self.get_logger().error(
                f"MOTOR SERIAL: {e}"
            )

    def stop_motor(self):

        if self.right_ser is None:
            return

        for _ in range(10):

            try:

                self.right_ser.write(b"M,0,0\n")
                self.right_ser.flush()

            except Exception:
                pass

            time.sleep(0.02)

    def close(self):

        self._stopped = True

        self.left_pwm = 0
        self.right_pwm = 0

        self.stop_motor()

        for ser in (self.right_ser, self.left_ser):

            if ser is None:
                continue

            try:
                ser.close()

            except Exception:
                pass

        self.right_ser = None
        self.left_ser = None


def main(args=None):

    rclpy.init(args=args)

    node = ArgosBaseDriver()

    # SIGTERM / SIGINT 어느 쪽이든 모터 정지 보장
    def shutdown_handler(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, shutdown_handler)

    atexit.register(node.close)

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.close()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
