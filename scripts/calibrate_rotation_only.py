#!/usr/bin/env python3

import os
import re
import math
import time
import serial
import sys
import termios
import tty
import select


RIGHT_PORT = "/dev/ttyCH341USB0"   # right encoder + motor command
LEFT_PORT  = "/dev/ttyCH341USB1"   # left encoder

BAUD = 115200

# 기존 직진 calibration 결과
LEFT_TICKS_PER_METER = 13640.0
RIGHT_TICKS_PER_METER = 15392.0

LEFT_SIGN = -1.0
RIGHT_SIGN = 1.0

# 회전은 조금 천천히
ROTATE_PWM = 55

# 기존 20초가 너무 짧았으므로 넉넉하게
ROTATE_TIMEOUT = 60.0

SEND_INTERVAL = 0.05

OUTPUT_FILE = os.path.expanduser(
    "~/argos_project/config/odometry_calibration.yaml"
)


def parse_count(line):

    line = line.strip()

    if not line:
        return None

    if line.startswith("M,"):
        return None

    if re.fullmatch(r"[+-]?\d+", line):
        return int(line)

    nums = re.findall(r"-?\d+", line)

    if not nums:
        return None

    return int(nums[-1])


class RotationCalibration:

    def __init__(self):

        self.right_ser = None
        self.left_ser = None

        self.right_count = None
        self.left_count = None

    def connect(self):

        print("[OPEN] RIGHT/MOTOR =", RIGHT_PORT)

        self.right_ser = serial.Serial(
            RIGHT_PORT,
            BAUD,
            timeout=0
        )

        print("[OPEN] LEFT        =", LEFT_PORT)

        self.left_ser = serial.Serial(
            LEFT_PORT,
            BAUD,
            timeout=0
        )

        time.sleep(2.5)

        self.right_ser.reset_input_buffer()
        self.left_ser.reset_input_buffer()

        self.stop()

    def send_motor(self, left, right):

        cmd = f"M,{left},{right}\n"

        self.right_ser.write(
            cmd.encode()
        )

    def stop(self):

        if self.right_ser is None:
            return

        for _ in range(10):

            try:
                self.send_motor(0, 0)
            except Exception:
                pass

            time.sleep(0.02)

    def read_encoders(self):

        while self.right_ser.in_waiting > 0:

            try:

                text = (
                    self.right_ser.readline()
                    .decode(errors="ignore")
                    .strip()
                )

                value = parse_count(text)

                if value is not None:
                    self.right_count = value

            except Exception:
                pass

        while self.left_ser.in_waiting > 0:

            try:

                text = (
                    self.left_ser.readline()
                    .decode(errors="ignore")
                    .strip()
                )

                value = parse_count(text)

                if value is not None:
                    self.left_count = value

            except Exception:
                pass

    def wait_encoder(self):

        print("[WAIT] encoder...")

        start = time.monotonic()

        while True:

            self.read_encoders()

            if (
                self.left_count is not None
                and self.right_count is not None
            ):

                print(
                    f"[OK] LEFT={self.left_count} "
                    f"RIGHT={self.right_count}"
                )

                return

            if time.monotonic() - start > 10:
                raise RuntimeError("Encoder timeout")

            time.sleep(0.01)

    def snapshot(self):

        end = time.monotonic() + 0.15

        while time.monotonic() < end:

            self.read_encoders()

            time.sleep(0.005)

        return (
            int(self.left_count),
            int(self.right_count)
        )

    def rotate(self):

        print()
        print("========================================")
        print(" ARGOS ROTATION CALIBRATION")
        print("========================================")
        print()
        print("현재 로봇이 바라보는 방향을")
        print("바닥에 테이프로 표시하세요.")
        print()
        print("ENTER를 누르면 CCW(반시계)로 회전합니다.")
        print()
        print("정확히 360도를 돌아")
        print("처음 방향과 일치하는 순간 SPACE를 누르세요.")
        print()
        print("SPACE = STOP")
        print("Q     = EMERGENCY STOP")
        print()
        print("최대 안전시간 = 60초")
        print()

        input("준비 완료 -> ENTER ")

        start_left, start_right = self.snapshot()

        print()
        print("START LEFT =", start_left)
        print("START RIGHT =", start_right)
        print()

        old_settings = termios.tcgetattr(
            sys.stdin
        )

        start_time = time.monotonic()
        last_send = 0.0
        last_print = 0.0

        try:

            tty.setcbreak(
                sys.stdin.fileno()
            )

            while True:

                now = time.monotonic()

                # 실제 하드웨어 확인 결과:
                # +LEFT, -RIGHT = CCW
                if now - last_send >= SEND_INTERVAL:

                    self.send_motor(
                        +ROTATE_PWM,
                        -ROTATE_PWM
                    )

                    last_send = now

                self.read_encoders()

                if now - last_print >= 0.25:

                    print(
                        f"\rTIME={now-start_time:5.1f}s  "
                        f"LEFT={self.left_count}  "
                        f"RIGHT={self.right_count}    ",
                        end="",
                        flush=True
                    )

                    last_print = now

                if select.select(
                    [sys.stdin],
                    [],
                    [],
                    0.01
                )[0]:

                    key = sys.stdin.read(1).lower()

                    if key == " ":

                        print("\n[SPACE] STOP")
                        break

                    if key == "q":

                        print("\n[Q] EMERGENCY STOP")
                        raise KeyboardInterrupt

                if now - start_time >= ROTATE_TIMEOUT:

                    print()
                    print("[TIMEOUT] MOTOR STOP")

                    raise RuntimeError(
                        "60초 안에 360도 calibration이 "
                        "완료되지 않았습니다."
                    )

        finally:

            self.stop()

            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                old_settings
            )

        time.sleep(0.5)

        end_left, end_right = self.snapshot()

        raw_left = end_left - start_left
        raw_right = end_right - start_right

        # 전진 기준 부호 적용
        left_distance = (
            raw_left
            * LEFT_SIGN
            / LEFT_TICKS_PER_METER
        )

        right_distance = (
            raw_right
            * RIGHT_SIGN
            / RIGHT_TICKS_PER_METER
        )

        track_width = abs(
            (
                right_distance
                - left_distance
            )
            /
            (2.0 * math.pi)
        )

        print()
        print("========================================")
        print(" RESULT")
        print("========================================")

        print("RAW LEFT  =", raw_left)
        print("RAW RIGHT =", raw_right)

        print(
            f"LEFT distance  = {left_distance:.6f} m"
        )

        print(
            f"RIGHT distance = {right_distance:.6f} m"
        )

        print()
        print(
            f"EFFECTIVE TRACK WIDTH = "
            f"{track_width:.6f} m"
        )

        return track_width

    def save(self, track_width):

        text = f"""wheel_odometry_node:
  ros__parameters:
    left_ticks_per_meter: {LEFT_TICKS_PER_METER:.6f}
    right_ticks_per_meter: {RIGHT_TICKS_PER_METER:.6f}
    left_sign: {LEFT_SIGN:.1f}
    right_sign: {RIGHT_SIGN:.1f}
    track_width: {track_width:.6f}
"""

        with open(
            OUTPUT_FILE,
            "w",
            encoding="utf-8"
        ) as f:

            f.write(text)

        print()
        print("[SAVE]")
        print(OUTPUT_FILE)

    def close(self):

        self.stop()

        if self.left_ser:
            self.left_ser.close()

        if self.right_ser:
            self.right_ser.close()


def main():

    cal = RotationCalibration()

    try:

        cal.connect()

        cal.wait_encoder()

        track_width = cal.rotate()

        cal.save(track_width)

        print()
        print("CALIBRATION COMPLETE")

    except KeyboardInterrupt:

        print()
        print("!!! EMERGENCY STOP !!!")

    except Exception as e:

        print()
        print("[ERROR]", e)

    finally:

        cal.close()


if __name__ == "__main__":
    main()

