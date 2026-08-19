#!/usr/bin/env python3

import re
import sys
import time
import tty
import termios
import select
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64


# ==========================================
# ARGOS 설정
# ==========================================

RIGHT_PORT = "/dev/ttyCH341USB0"   # 오른쪽 UNO + 모터 명령
LEFT_PORT  = "/dev/ttyCH341USB1"   # 왼쪽 UNO

BAUD = 115200

# 너무 빠르면 낮추면 됨
PWM = 60

SEND_INTERVAL = 0.05

# 혹시 SSH 입력이 안 되는 상황 대비
MAX_RUN_TIME = 20.0


class ForwardOdomTest(Node):

    def __init__(self):
        super().__init__("forward_odom_test")

        self.right_ser = serial.Serial(
            RIGHT_PORT,
            BAUD,
            timeout=0
        )

        self.left_ser = serial.Serial(
            LEFT_PORT,
            BAUD,
            timeout=0
        )

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

        self.left_count = None
        self.right_count = None

        print("[WAIT] Arduino startup...")
        time.sleep(2.5)

        self.left_ser.reset_input_buffer()
        self.right_ser.reset_input_buffer()

        self.stop_motor()

    def parse_count(self, text):

        text = text.strip()

        if not text:
            return None

        # 혹시 모터 명령 echo가 있으면 무시
        if text.startswith("M,"):
            return None

        # 숫자만 오는 경우
        if re.fullmatch(r"[+-]?\d+", text):
            return int(text)

        # E,1234 / COUNT=1234 등
        numbers = re.findall(r"-?\d+", text)

        if not numbers:
            return None

        return int(numbers[-1])

    def read_encoders(self):

        # ==============================
        # RIGHT
        # ==============================

        while self.right_ser.in_waiting > 0:

            try:
                text = (
                    self.right_ser.readline()
                    .decode(errors="ignore")
                    .strip()
                )

                value = self.parse_count(text)

                if value is not None:
                    self.right_count = value

                    msg = Int64()
                    msg.data = value
                    self.right_pub.publish(msg)

            except Exception as e:
                print("RIGHT SERIAL ERROR:", e)

        # ==============================
        # LEFT
        # ==============================

        while self.left_ser.in_waiting > 0:

            try:
                text = (
                    self.left_ser.readline()
                    .decode(errors="ignore")
                    .strip()
                )

                value = self.parse_count(text)

                if value is not None:
                    self.left_count = value

                    msg = Int64()
                    msg.data = value
                    self.left_pub.publish(msg)

            except Exception as e:
                print("LEFT SERIAL ERROR:", e)

    def send_motor(self, left, right):

        command = f"M,{left},{right}\n"

        self.right_ser.write(
            command.encode()
        )

    def stop_motor(self):

        for _ in range(10):

            try:
                self.send_motor(0, 0)
            except Exception:
                pass

            time.sleep(0.02)

    def wait_for_encoders(self):

        print("[WAIT] Encoder data...")

        start = time.monotonic()

        while True:

            self.read_encoders()

            if (
                self.left_count is not None
                and self.right_count is not None
            ):
                print(
                    f"[OK] LEFT={self.left_count}, "
                    f"RIGHT={self.right_count}"
                )
                return

            if time.monotonic() - start > 10:
                raise RuntimeError(
                    "Encoder data timeout"
                )

            time.sleep(0.01)

    def run_forward(self):

        print()
        print("======================================")
        print("       ARGOS 1m ODOM TEST")
        print("======================================")
        print()
        print(f"PWM = {PWM}")
        print()
        print("ENTER : 전진 시작")
        print("SPACE : 즉시 정지 + 종료")
        print("X     : 즉시 정지 + 종료")
        print("Q     : 긴급 정지 + 종료")
        print()
        print("로봇 기준점에서 정확히 1m 위치를")
        print("테이프로 표시한 뒤 시작하세요.")
        print()

        input("준비 완료 -> ENTER ")

        old_settings = termios.tcgetattr(
            sys.stdin
        )

        last_send = 0.0
        last_print = 0.0
        start_time = time.monotonic()

        try:

            tty.setcbreak(
                sys.stdin.fileno()
            )

            print()
            print(">>> FORWARD START")
            print(">>> 1m 지점에서 SPACE!")
            print()

            while True:

                now = time.monotonic()

                # 모터 명령 지속 전송
                if now - last_send >= SEND_INTERVAL:

                    self.send_motor(
                        PWM,
                        PWM
                    )

                    last_send = now

                # encoder 읽고 ROS topic 발행
                self.read_encoders()

                # 상태 출력
                if now - last_print >= 0.25:

                    print(
                        f"\rTIME={now-start_time:5.2f}s  "
                        f"LEFT={self.left_count}  "
                        f"RIGHT={self.right_count}    ",
                        end="",
                        flush=True
                    )

                    last_print = now

                # 키 입력
                if select.select(
                    [sys.stdin],
                    [],
                    [],
                    0.01
                )[0]:

                    key = sys.stdin.read(1).lower()

                    if key in (" ", "x", "q"):

                        print()
                        print("[STOP]")
                        break

                # 안전 timeout
                if now - start_time >= MAX_RUN_TIME:

                    print()
                    print("[TIMEOUT] 자동 정지")
                    break

        finally:

            self.stop_motor()

            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                old_settings
            )

        print()
        print("MOTOR STOP / TEST COMPLETE")

    def close(self):

        self.stop_motor()

        self.left_ser.close()
        self.right_ser.close()


def main():

    rclpy.init()

    node = ForwardOdomTest()

    try:

        node.wait_for_encoders()
        node.run_forward()

    except KeyboardInterrupt:

        print()
        print("!!! EMERGENCY STOP !!!")

    except Exception as e:

        print()
        print("[ERROR]", e)

    finally:

        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
