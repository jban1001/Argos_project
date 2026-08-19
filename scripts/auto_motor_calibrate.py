#!/usr/bin/env python3

import os
import re
import math
import time
import shutil
import serial
import sys
import termios
import tty
import select
from datetime import datetime


# ============================================================
# ARGOS PORT CONFIG
# ============================================================

RIGHT_PORT = "/dev/ttyCH341USB0"   # motor command + right encoder
LEFT_PORT  = "/dev/ttyCH341USB1"   # left encoder

BAUD = 115200


# ============================================================
# CALIBRATION MOTOR POWER
# ============================================================

# 너무 빠르게 하지 않음
FORWARD_PWM = 65
ROTATE_PWM  = 60

SEND_INTERVAL = 0.05

# 혹시 키 입력을 못 하더라도 자동 정지하는 안전 제한
FORWARD_TIMEOUT = 15.0
ROTATE_TIMEOUT  = 20.0


# ============================================================
# OUTPUT
# ============================================================

OUTPUT_FILE = os.path.expanduser(
    "~/argos_project/config/odometry_calibration.yaml"
)


class ArgosCalibration:

    def __init__(self):

        self.right_ser = None
        self.left_ser = None

        self.right_count = None
        self.left_count = None

        self.running = False


    # ========================================================
    # SERIAL
    # ========================================================

    def connect(self):

        print()
        print("==========================================")
        print("       ARGOS MOTOR ODOM CALIBRATION")
        print("==========================================")
        print()

        print(f"[OPEN] RIGHT/MOTOR : {RIGHT_PORT}")
        self.right_ser = serial.Serial(
            RIGHT_PORT,
            BAUD,
            timeout=0
        )

        print(f"[OPEN] LEFT        : {LEFT_PORT}")
        self.left_ser = serial.Serial(
            LEFT_PORT,
            BAUD,
            timeout=0
        )

        # Arduino가 serial open으로 reset될 수 있으므로 대기
        print("[WAIT] Arduino startup...")
        time.sleep(2.5)

        self.right_ser.reset_input_buffer()
        self.left_ser.reset_input_buffer()

        self.stop()

        print("[OK] Serial connected")


    # ========================================================
    # ENCODER PARSER
    # ========================================================

    def parse_count(self, line):

        line = line.strip()

        if not line:
            return None

        # motor command echo가 있다면 무시
        if line.startswith("M,"):
            return None

        # 숫자 하나만 출력하는 경우
        if re.fullmatch(r"[+-]?\d+", line):
            return int(line)

        # E,1234 / COUNT=1234 같은 형식 지원
        numbers = re.findall(r"-?\d+", line)

        if not numbers:
            return None

        return int(numbers[-1])


    def read_serials(self):

        # RIGHT
        while self.right_ser.in_waiting > 0:

            try:
                line = (
                    self.right_ser
                    .readline()
                    .decode(errors="ignore")
                    .strip()
                )

                value = self.parse_count(line)

                if value is not None:
                    self.right_count = value

            except Exception:
                pass

        # LEFT
        while self.left_ser.in_waiting > 0:

            try:
                line = (
                    self.left_ser
                    .readline()
                    .decode(errors="ignore")
                    .strip()
                )

                value = self.parse_count(line)

                if value is not None:
                    self.left_count = value

            except Exception:
                pass


    def wait_encoder(self):

        print("[WAIT] Encoder data...")

        start = time.monotonic()

        while True:

            self.read_serials()

            if (
                self.left_count is not None
                and self.right_count is not None
            ):
                break

            if time.monotonic() - start > 10.0:
                raise RuntimeError(
                    "Encoder data timeout"
                )

            time.sleep(0.01)

        print(
            f"[OK] LEFT={self.left_count} "
            f"RIGHT={self.right_count}"
        )


    def snapshot(self):

        # 최신 serial data 취득
        end_time = time.monotonic() + 0.15

        while time.monotonic() < end_time:
            self.read_serials()
            time.sleep(0.005)

        return (
            int(self.left_count),
            int(self.right_count)
        )


    # ========================================================
    # MOTOR
    # ========================================================

    def send_motor(self, left, right):

        command = f"M,{left},{right}\n"

        self.right_ser.write(
            command.encode()
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


    # ========================================================
    # KEY CONTROL
    # ========================================================

    def run_until_space(
        self,
        left_pwm,
        right_pwm,
        timeout,
        title
    ):

        print()
        print("------------------------------------------")
        print(title)
        print("------------------------------------------")
        print()
        print(
            f"MOTOR COMMAND : L={left_pwm}, R={right_pwm}"
        )
        print()
        print("SPACE : 즉시 정지")
        print("Q     : 긴급 종료")
        print()
        print(">>> 움직이기 시작합니다.")
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

                # ------------------------------------------
                # motor watchdog refresh
                # ------------------------------------------

                if now - last_send >= SEND_INTERVAL:

                    self.send_motor(
                        left_pwm,
                        right_pwm
                    )

                    last_send = now

                # ------------------------------------------
                # encoder read
                # ------------------------------------------

                self.read_serials()

                # ------------------------------------------
                # display
                # ------------------------------------------

                if now - last_print >= 0.25:

                    elapsed = now - start_time

                    print(
                        f"\rTIME={elapsed:5.2f}s  "
                        f"LEFT={self.left_count}  "
                        f"RIGHT={self.right_count}   ",
                        end="",
                        flush=True
                    )

                    last_print = now

                # ------------------------------------------
                # keyboard
                # ------------------------------------------

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

                # ------------------------------------------
                # timeout
                # ------------------------------------------

                if now - start_time >= timeout:

                    print()
                    print(
                        "[TIMEOUT] 안전을 위해 자동 정지"
                    )

                    break

        finally:

            self.stop()

            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                old_settings
            )

        time.sleep(0.4)

        return self.snapshot()


    # ========================================================
    # SAVE
    # ========================================================

    def save_yaml(
        self,
        left_tpm,
        right_tpm,
        left_sign,
        right_sign,
        track_width
    ):

        os.makedirs(
            os.path.dirname(OUTPUT_FILE),
            exist_ok=True
        )

        # 기존 calibration 파일 백업
        if os.path.exists(OUTPUT_FILE):

            stamp = datetime.now().strftime(
                "%Y%m%d_%H%M%S"
            )

            backup = (
                OUTPUT_FILE
                + f".bak_{stamp}"
            )

            shutil.copy2(
                OUTPUT_FILE,
                backup
            )

            print()
            print(
                f"[BACKUP] {backup}"
            )

        text = f"""wheel_odometry_node:
  ros__parameters:
    left_ticks_per_meter: {left_tpm:.6f}
    right_ticks_per_meter: {right_tpm:.6f}
    left_sign: {left_sign:.1f}
    right_sign: {right_sign:.1f}
    track_width: {track_width:.6f}
"""

        with open(
            OUTPUT_FILE,
            "w",
            encoding="utf-8"
        ) as f:

            f.write(text)

        print()
        print(
            f"[SAVE] {OUTPUT_FILE}"
        )


    # ========================================================
    # CALIBRATION
    # ========================================================

    def calibrate(self):

        self.wait_encoder()

        # ====================================================
        # STEP 1 : STRAIGHT 1m
        # ====================================================

        print()
        print("==========================================")
        print(" STEP 1 : STRAIGHT DISTANCE CALIBRATION")
        print("==========================================")
        print()
        print("로봇 정면으로 정확히 1.000 m 지점을")
        print("테이프로 표시하세요.")
        print()
        print("ENTER를 누르면 로봇이 저속으로 전진합니다.")
        print()
        print("로봇 기준점이 1m 표시선에 도달하는 순간")
        print("SPACE를 누르세요.")
        print()

        input("준비 완료 -> ENTER ")

        left_start, right_start = self.snapshot()

        print()
        print(
            f"START LEFT  = {left_start}"
        )
        print(
            f"START RIGHT = {right_start}"
        )

        left_end, right_end = self.run_until_space(
            FORWARD_PWM,
            FORWARD_PWM,
            FORWARD_TIMEOUT,
            "[FORWARD 1m]"
        )

        left_delta_raw = (
            left_end - left_start
        )

        right_delta_raw = (
            right_end - right_start
        )

        if (
            left_delta_raw == 0
            or right_delta_raw == 0
        ):
            raise RuntimeError(
                "Forward encoder delta is zero"
            )

        left_sign = (
            1.0
            if left_delta_raw > 0
            else -1.0
        )

        right_sign = (
            1.0
            if right_delta_raw > 0
            else -1.0
        )

        # 정확히 1m가 기준이므로 그대로 ticks/m
        left_tpm = abs(
            float(left_delta_raw)
        )

        right_tpm = abs(
            float(right_delta_raw)
        )

        print()
        print("========== FORWARD RESULT ==========")
        print(
            f"LEFT raw delta  = {left_delta_raw}"
        )
        print(
            f"RIGHT raw delta = {right_delta_raw}"
        )
        print()
        print(
            f"LEFT sign       = {left_sign:+.0f}"
        )
        print(
            f"RIGHT sign      = {right_sign:+.0f}"
        )
        print()
        print(
            f"LEFT ticks/m    = {left_tpm:.3f}"
        )
        print(
            f"RIGHT ticks/m   = {right_tpm:.3f}"
        )

        # ====================================================
        # STEP 2 : CCW 360deg
        # ====================================================

        print()
        print("==========================================")
        print(" STEP 2 : ROTATION CALIBRATION")
        print("==========================================")
        print()
        print("현재 로봇의 정면 방향을")
        print("바닥에 테이프로 표시하세요.")
        print()
        print("ENTER를 누르면 저속으로")
        print("반시계(CCW) 제자리 회전합니다.")
        print()
        print("정확히 처음 방향으로 돌아오는 순간")
        print("SPACE를 누르세요.")
        print()

        input("준비 완료 -> ENTER ")

        rot_left_start, rot_right_start = (
            self.snapshot()
        )

        rot_left_end, rot_right_end = (
            self.run_until_space(
                -ROTATE_PWM,
                +ROTATE_PWM,
                ROTATE_TIMEOUT,
                "[CCW 360deg]"
            )
        )

        rot_left_raw = (
            rot_left_end
            - rot_left_start
        )

        rot_right_raw = (
            rot_right_end
            - rot_right_start
        )

        # Forward 기준 encoder sign 적용
        left_distance = (
            rot_left_raw
            * left_sign
            / left_tpm
        )

        right_distance = (
            rot_right_raw
            * right_sign
            / right_tpm
        )

        # differential / skid-steer
        #
        # dtheta = (dR-dL) / W
        #
        # 정확히 한 바퀴 = 2*pi

        track_width = abs(
            (
                right_distance
                - left_distance
            )
            /
            (2.0 * math.pi)
        )

        print()
        print("========== ROTATION RESULT ==========")

        print(
            f"LEFT rotation  = "
            f"{left_distance:.6f} m"
        )

        print(
            f"RIGHT rotation = "
            f"{right_distance:.6f} m"
        )

        print()

        print(
            f"EFFECTIVE TRACK WIDTH = "
            f"{track_width:.6f} m"
        )

        # sanity
        if not (
            0.10 <= track_width <= 1.50
        ):

            print()
            print(
                "[WARNING] track width 결과가 "
                "일반적인 범위를 벗어났습니다."
            )

        # ====================================================
        # SAVE
        # ====================================================

        self.save_yaml(
            left_tpm,
            right_tpm,
            left_sign,
            right_sign,
            track_width
        )

        print()
        print("==========================================")
        print("        CALIBRATION COMPLETE")
        print("==========================================")
        print()

        print(
            f"LEFT ticks/m   = {left_tpm:.3f}"
        )

        print(
            f"RIGHT ticks/m  = {right_tpm:.3f}"
        )

        print(
            f"LEFT sign      = {left_sign:+.0f}"
        )

        print(
            f"RIGHT sign     = {right_sign:+.0f}"
        )

        print(
            f"TRACK WIDTH    = {track_width:.6f} m"
        )

        print()
        print("저장:")
        print(OUTPUT_FILE)
        print()


    # ========================================================
    # CLOSE
    # ========================================================

    def close(self):

        print()
        print("[STOP] Sending motor stop...")

        self.stop()

        if self.left_ser is not None:
            self.left_ser.close()

        if self.right_ser is not None:
            self.right_ser.close()

        print("[EXIT]")


def main():

    robot = ArgosCalibration()

    try:

        robot.connect()

        robot.calibrate()

    except KeyboardInterrupt:

        print()
        print()
        print("!!! EMERGENCY STOP !!!")

    except Exception as e:

        print()
        print(
            f"[ERROR] {e}"
        )

    finally:

        robot.close()


if __name__ == "__main__":
    main()
