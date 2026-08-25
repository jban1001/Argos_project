#!/usr/bin/env python3

"""
ARGOS 화재 탐색 주행 (fire_seeker)

맵도 AMCL 도 Nav2 도 쓰지 않는 순수 반응형 동작이다.

  PATROL   계속 돌아다닌다. 방식이 두 가지다 (PATROL_MODE).

           "wall"   한쪽 벽과 일정 거리를 유지하며 따라 돈다.
                    방 둘레를 한 바퀴 도는 커버리지가 나온다.
                    빔 두 개(옆 90도, 대각 45도)로 벽까지 거리뿐 아니라
                    벽의 기울기까지 추정해서 조향한다.
                    거리 하나만 P 제어하면 좌우로 진동한다.

           "bounce" 앞이 트이면 직진, 막히면 TURN.
                    원래 동작이다. 단순하지만 방 가운데서
                    같은 곳만 왕복할 수 있다.

  TURN     제자리에서 돌아 앞을 확보한 뒤 다시 PATROL 로 돌아간다.
           wall 모드에서는 벽 반대쪽으로 돈다 (안쪽 코너 처리).
           bounce 모드에서는 좌우 중 더 트인 쪽으로 돈다.

  FIRE_STOP 불이 확정되면 일단 완전히 멈춘다.

  ALIGN    제자리에서 몸을 불 쪽으로 돌린다. 전진하지 않는다.

  APPROACH 정면이 맞으면 불 쪽으로 다가간다.
           가면서도 조금씩 방향을 보정한다.

  HOLD     FIRE_STOP_DIST 까지 접근했으면 멈춰서 유지한다.

  (불을 LOST_SECONDS 동안 못 보면 PATROL 로 되돌아간다)

화재 판단
---------
YOLO 로 bbox 를 얻어 "방향" 을 구하고,
아두이노 온도/가스 + MLP 로 "진짜 불인지" 를 판단한다.
MLP 는 방향을 주지 못하므로 둘 다 필요하다.

  방향  <- YOLO bbox 중심 x
  판단  <- MLP 확률 >= FIRE_PROB_THRESHOLD 가
           CONFIRM_SECONDS 동안 연속 유지

주의: 현재 학습된 MLP 는 threshold 0.70 에서 recall 이 약 0.25 다.
      (오경보는 거의 없지만 실제 불의 3/4 를 놓친다)
      로봇이 불을 잘 못 잡으면 FIRE_PROB_THRESHOLD 를 낮추거나,
      세션을 더 쌓아 재학습한 뒤 올리는 것이 맞다.
      급하면 REQUIRE_SENSOR_GATE = False 로 YOLO 단독 동작 가능.

LiDAR 는 base_link 기준 약 177도 돌아가 장착되어 있다.
스캔 각도는 반드시 TF (base_link -> laser_frame) 로 변환해서 쓴다.
각도를 직접 가정하지 말 것.  (mapping_drive.py 와 동일 규칙)

실행
----
    # 터미널 1 : 베이스 + 오도메트리 + LiDAR + TF 만 띄운다
    #
    #   ★ argos_navigation.launch.py (Nav2) 를 띄우면 안 된다.
    #     velocity_smoother 가 /cmd_vel 에 0 을 20 Hz 로 계속 쏘기 때문에
    #     이 노드의 명령과 상쇄되어 로봇이 움직이지 않는다.
    #
    source /opt/ros/jazzy/setup.bash
    source ~/argos_project/ros2_ws/install/setup.bash
    source ~/ydlidar_ws/install/setup.bash
    ros2 launch argos_bringup argos_bringup.launch.py

    # 터미널 2
    source /opt/ros/jazzy/setup.bash
    ~/.venv/bin/python ~/argos_project/scripts/fire_seeker.py

    # 순찰 방식 지정
    ~/.venv/bin/python ~/argos_project/scripts/fire_seeker.py --patrol wall --side right
    ~/.venv/bin/python ~/argos_project/scripts/fire_seeker.py --patrol bounce

    ~/.venv/bin/python 을 써야 ultralytics / sklearn 이 잡힌다.
    ROS 를 source 한 상태면 venv 에서도 rclpy 가 보인다.
    ROS_DOMAIN_ID 는 ~/.bashrc 의 42 를 그대로 쓴다.
"""

import os
import re
import sys
import math
import time
import atexit
import signal
import threading
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import joblib

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from tf2_ros import Buffer, TransformListener


# ==========================================================
# 설정
# ==========================================================

YOLO_DIR = Path.home() / "YOLO"

MODEL_PATH = str(YOLO_DIR / "best.engine")
MLP_MODEL_PATH = str(YOLO_DIR / "fire_mlp.pkl")

# --- 카메라 ---
WEBCAM_DEVICE = 0

# best.engine 의 내부 입력은 640x640 이다. 1280x720 으로 잡아도
# 어차피 640 으로 줄여서 넣으므로 탐지 성능 차이가 없다.
# 반면 1280x720 MJPG 는 USB 대역과 디코드 비용을 크게 먹고
# "Corrupt JPEG data" 가 쏟아진다. 그래서 640x480 으로 받는다.
#
# bearing 은 화면 폭 대비 비율로 계산하므로 해상도와 무관하다.
WEBCAM_WIDTH = 640
WEBCAM_HEIGHT = 480
WEBCAM_FPS = 30

# 카메라 수평 화각 [deg]. Logitech C270(046d:0825) 기준 약 60도.
# 정밀 캘리브레이션은 하지 않는다. 방향만 대충 맞으면 되고,
# 접근하면서 오차가 계속 줄어들기 때문이다.
CAMERA_HFOV_DEG = 60.0

# 카메라가 base_link 기준 어느 쪽을 보는지 [rad].
# 0.0 = 정면(+x). 뒤를 보게 달았으면 math.pi 로 바꾼다.
CAMERA_YAW_OFFSET = 0.0

# --- 아두이노 센서 ---
ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUDRATE = 115200

# --- YOLO ---
CONF_THRES = 0.01
TARGET_CLASSES = ["fire", "smoke", "cigarette_butt", "spark"]

# spark는 오탐 박스가 화면을 어지럽히지 않도록 이 confidence 이상일 때만
# HUD에 표시한다. 감지값과 MLP 입력에는 별도로 계속 반영된다.
SPARK_DISPLAY_CONF = 0.60

# 이 confidence 이상인 bbox 만 "방향" 근거로 쓴다.
BEARING_CONF = 0.10

# 방향을 뽑을 클래스 우선순위
BEARING_CLASSES = ["fire", "smoke", "spark"]

# --- MLP ---
# 학습 때와 순서가 반드시 같아야 한다 (train_MLP.py 와 동일)
FEATURE_COLUMNS = [
    "fire_conf",
    "smoke_conf",
    "cigarette_conf",
    "spark_conf",
    "temperature",
    "gas",
    "temp_change",
    "gas_change",
]

# spark 양성/음성 데이터가 충분하지 않은 기존 MLP에서는 작은 오탐도
# 위험확률을 크게 올릴 수 있다. ARGOS 순찰 실행기가 이 값을 낮추면 실제
# YOLO spark 표시는 유지하되 MLP 입력의 spark_conf만 축소한다.
MLP_SPARK_WEIGHT = 1.0

CHANGE_WINDOW_SECONDS = 2.0

FIRE_PROB_THRESHOLD = 0.70

# 이 시간만큼 연속으로 기준을 넘어야 "불" 로 확정한다
CONFIRM_SECONDS = 1.0

# False 로 두면 아두이노/MLP 없이 YOLO confidence 만으로 판단한다
REQUIRE_SENSOR_GATE = True


def mlp_spark_feature(confs):
    """가중치를 적용한 MLP용 spark confidence를 반환한다."""
    weight = max(0.0, float(MLP_SPARK_WEIGHT))
    return float(confs.get("spark", 0.0)) * weight


def should_display_detection(class_name, confidence):
    """클래스별 HUD 표시 조건을 반환한다."""
    if class_name == "spark":
        return float(confidence) >= float(SPARK_DISPLAY_CONF)
    return True

# REQUIRE_SENSOR_GATE = False 일 때 쓰는 기준
YOLO_ONLY_FIRE_CONF = 0.40

# --- 주행 ---
# 실측 상한: 바퀴 0.158 m/s, 제자리 회전 0.637 rad/s (docs/STATUS.md)
# 상한의 60~70% 정도만 쓴다.

# PATROL : 앞이 트여 있으면 계속 직진한다
PATROL_V = 0.10           # 순찰 전진 속도 [m/s]

# 전방 여유가 이보다 작으면 막힌 것으로 보고 TURN 으로 넘어간다 [m]
PATROL_STOP_DIST = 0.60

# TURN : 전방 여유가 이만큼 확보될 때까지 제자리에서 돈다 [m]
# PATROL_STOP_DIST 보다 넉넉해야 돌자마자 다시 막히지 않는다.
PATROL_RESUME_DIST = 0.90

PATROL_TURN_W = 0.40      # 순찰 회전 각속도 [rad/s]

# 한 번의 TURN 에서 이 각도를 넘게 돌았는데도 앞이 안 트이면
# 반대 방향으로 바꿔 본다 [deg]
PATROL_TURN_GIVEUP_DEG = 200.0

# --- 순찰 방식 ---
#
#   "wall"   한쪽 벽과 일정 거리를 유지하며 따라 돈다.
#            방을 한 바퀴 도는 커버리지가 나온다.
#
#   "bounce" 앞이 트이면 직진, 막히면 트인 쪽으로 회전.
#            원래 fire_seeker 의 동작이다. 단순하지만
#            방 가운데서 같은 곳만 왕복할 수 있다.
#
# 실행 시 --patrol wall / --patrol bounce 로 바꿀 수 있다.
PATROL_MODE = "wall"

# 어느 쪽 벽을 따라갈지. "right" 또는 "left".
# 실행 시 --side 로 바꿀 수 있다.
WALL_SIDE = "right"

# 벽과 유지할 거리 [m]
#
# 차체 폭이 0.53 m 라 반폭이 0.265 m 다.
# 여기에 여유를 더해 0.55 m 로 잡는다. 너무 붙이면
# 바깥 코너를 돌 때 벽에 스친다.
WALL_TARGET_DIST = 0.55

# 벽 기울기 추정에 쓰는 두 빔 사이 각 [rad]
WALL_BEAM_ANGLE = math.radians(45.0)

# 벽 거리 측정 시 사용할 반각 [rad]
# 전방 판정(25도)보다 좁아야 옆 벽만 깨끗하게 잡힌다.
WALL_HALF_ANGLE = math.radians(12.0)

# 이보다 멀면 따라갈 벽이 없다고 본다 [m]
WALL_LOST_DIST = 1.6

# 조향 시 몇 m 앞을 내다볼지 [m]
# 클수록 부드럽지만 코너 반응이 늦다.
WALL_LOOKAHEAD = 0.35

# 거리 오차 -> 각속도 비례 게인
WALL_KP = 1.6

# 벽 추종 중 각속도 상한 [rad/s]
WALL_MAX_W = 0.35

# 벽을 놓쳤을 때(바깥 코너) 벽 쪽으로 감아 도는 각속도 [rad/s]
WALL_SEARCH_W = 0.30

# 벽을 놓친 채 이 시간이 지나면 제자리 회전으로 다시 찾는다 [s]
WALL_SEARCH_TIMEOUT = 6.0

# FIRE_STOP : 불을 확정하면 이 시간만큼 완전히 멈춘다 [s]
# 달리던 관성을 죽이고 나서 몸을 돌려야 방위각이 안 흔들린다.
FIRE_STOP_PAUSE = 0.7

# ALIGN / APPROACH
ALIGN_W_MAX = 0.35        # 몸 돌릴 때 각속도 상한 [rad/s]
TURN_KP = 1.2             # 방위각 -> 각속도 비례 게인

# 이 각도 안에 들어오면 정면을 맞춘 것으로 본다 [deg]
ALIGN_TOL_DEG = 8.0

# 접근 중 이 각도를 벗어나면 다시 ALIGN 으로 돌아간다 [deg]
# ALIGN_TOL_DEG 보다 커야 두 상태를 왔다갔다 하지 않는다.
#
# 상한 주의: 방위각은 bbox 중심을 화면 폭으로 정규화해서 구하므로
#            최대치가 CAMERA_HFOV_DEG / 2 다. 지금은 60/2 = 30 도.
#            30 이상으로 잡으면 이 조건이 영영 성립하지 않아
#            재정렬이 아예 없어진다.
#
# 2026-08-25 실측: 20 도일 때 접근 도중 방위각이 +21, +23 도로 흔들려
#                  ALIGN <-> APPROACH 를 3회 오갔다.
#                  그 흔들림은 APPROACH 안의 조향으로 충분히 잡히므로
#                  26 도로 올려 흡수한다. 불이 화면 가장자리(>26도)까지
#                  밀려나면 그때는 멈춰서 다시 조준하는 것이 맞다.
REALIGN_DEG = 26.0

APPROACH_V = 0.08         # 접근 전진 속도 [m/s]
APPROACH_W_MAX = 0.25     # 접근 중 조향 각속도 상한 [rad/s]

# 불에 이만큼까지 다가가면 멈춘다 [m]
# 전방 LiDAR 여유 기준이다. 불 자체가 LiDAR 에 안 잡힐 수도 있으므로
# APPROACH_MAX_SECONDS 로도 같이 끊는다.
FIRE_STOP_DIST = 0.60

# 한 번의 접근이 이 시간을 넘으면 멈춘다 [s]
# 불이 LiDAR 에 안 잡히는 물체일 때 무한 전진하는 것을 막는다.
APPROACH_MAX_SECONDS = 25.0

# 전방 여유 판정 반각 [rad]
FRONT_HALF_ANGLE = math.radians(25.0)

# 좌우 어느 쪽이 트였는지 볼 때 쓰는 방향 [rad]
SIDE_HEADING = math.radians(60.0)

# 이 시간 동안 새 스캔이 없으면 눈이 먼 것으로 보고 정지 [s]
# LiDAR 가 3.66 Hz 까지 떨어지는 개체라 여유를 크게 잡는다.
SCAN_TIMEOUT = 1.5

# 불을 이 시간 동안 못 보면 PATROL 로 복귀 [s]
LOST_SECONDS = 2.0

# YOLO bbox 를 순간적으로 놓쳤을 때 마지막 방위각을 유지할 시간 [s]
#
# 2026-08-25 실측: MLP 는 98~99% 로 계속 "불" 이라고 하는데
# YOLO bbox 는 프레임의 1/3 에서만 잡혔다.
# 방위각이 없으면 ALIGN 이 그 자리에서 멈추고,
# LOST_SECONDS 뒤 PATROL 로 튕겨나갔다가 MLP 때문에 즉시
# FIRE_STOP 으로 되돌아오는 것을 반복해서 APPROACH 진입이 0회였다.
#
# 마지막 방위각을 잠깐 붙들어 두면 끊긴 프레임을 건너뛸 수 있다.
# 너무 길게 잡으면 불이 사라진 뒤에도 엉뚱한 쪽으로 붙으므로
# LOST_SECONDS 보다 짧게 유지한다.
BEARING_HOLD_SECONDS = 1.2

# 전체 동작 시간 상한 [s]
MAX_RUN_TIME = 600.0

# 화면 표시 (headless 면 자동으로 꺼진다)
SHOW_WINDOW = True


# ==========================================================
# 아두이노 센서 (new_main.py 의 파서를 그대로 사용)
# ==========================================================

NUMBER_PATTERN = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)"

CSV_PATTERN = re.compile(
    rf"^\s*({NUMBER_PATTERN})\s*,\s*({NUMBER_PATTERN})\s*$"
)

GAS_PATTERN = re.compile(
    rf"(?:GAS(?:_RAW)?|가스(?:값)?)"
    rf"\s*[:=]\s*({NUMBER_PATTERN})",
    re.IGNORECASE,
)

IR_PATTERN = re.compile(
    rf"(?:IR(?:_TEMP(?:ERATURE)?)?|"
    rf"TEMP(?:ERATURE)?|"
    rf"적외선(?:\s*온도)?|온도)"
    rf"\s*[:=]\s*({NUMBER_PATTERN})",
    re.IGNORECASE,
)

sensor_lock = threading.Lock()

sensor_data = {
    "gas_raw": 0,
    "ir_temperature": 0.0,
    "last_update": 0.0,
}


def parse_sensor_line(line):

    csv_match = CSV_PATTERN.fullmatch(line)

    if csv_match:
        gas_raw = int(float(csv_match.group(1)))
        ir_temperature = float(csv_match.group(2))
        return gas_raw, ir_temperature

    gas_raw = None
    ir_temperature = None

    gas_match = GAS_PATTERN.search(line)

    if gas_match:
        gas_raw = int(float(gas_match.group(1)))

    ir_match = IR_PATTERN.search(line)

    if ir_match:
        ir_temperature = float(ir_match.group(1))

    return gas_raw, ir_temperature


def read_arduino(stop_event):
    """아두이노 시리얼을 계속 읽어 sensor_data 를 갱신한다."""

    import serial

    pending_gas = None
    pending_temp = None

    while not stop_event.is_set():

        try:
            with serial.Serial(
                port=ARDUINO_PORT,
                baudrate=ARDUINO_BAUDRATE,
                timeout=1,
            ) as ser:

                print(f"[Arduino] {ARDUINO_PORT} 연결됨")

                while not stop_event.is_set():

                    raw = ser.readline()

                    if not raw:
                        continue

                    line = raw.decode("utf-8", errors="ignore").strip()

                    if not line:
                        continue

                    gas_raw, ir_temperature = parse_sensor_line(line)

                    if gas_raw is not None:
                        pending_gas = gas_raw

                    if ir_temperature is not None:
                        pending_temp = ir_temperature

                    if pending_gas is None or pending_temp is None:
                        continue

                    with sensor_lock:
                        sensor_data["gas_raw"] = pending_gas
                        sensor_data["ir_temperature"] = pending_temp
                        sensor_data["last_update"] = time.monotonic()

        except Exception as error:
            if not stop_event.is_set():
                print("[Arduino] 오류:", repr(error))
                time.sleep(1.0)


def get_sensor_data():
    """(gas, temp, connected). 3초 이상 갱신이 없으면 끊긴 것으로 본다."""

    with sensor_lock:
        gas_raw = sensor_data["gas_raw"]
        ir_temperature = sensor_data["ir_temperature"]
        last_update = sensor_data["last_update"]

    connected = (
        last_update > 0
        and time.monotonic() - last_update <= 3.0
    )

    if not connected:
        return 0, 0.0, False

    return gas_raw, ir_temperature, True


# ==========================================================
# ROS 노드
# ==========================================================

def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class FireSeeker(Node):

    def __init__(self):
        super().__init__("fire_seeker")

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(
            LaserScan, "/scan", self.scan_cb, qos_profile_sensor_data
        )

        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan = None
        self.last_scan_time = 0.0

        # /scan 이 끊길 때 원인을 가리기 위한 계측
        self.scan_count = 0
        self.scan_times = deque(maxlen=40)

        self.laser_x = None
        self.laser_yaw = None

        self.odom_yaw = None

        self.t_start = time.monotonic()

        # ROS 콜백 전용 스레드
        #
        # 주행 루프는 YOLO 추론(약 23 ms) 때문에 초당 40바퀴 정도밖에
        # 못 돈다. 반면 들어오는 콜백은 /tf + /odom + /scan 을 합쳐
        # 초당 100개가 넘는다.
        #
        # 루프 안에서 spin_once() 를 한 번씩만 부르면 한 바퀴에 콜백을
        # 하나밖에 못 처리해서 큐가 밀리고, /scan 이 굶어 죽는다.
        # (증상: /scan 은 11 Hz 로 멀쩡한데 "스캔이 끊겼다" 가 뜬다)
        #
        # 그래서 콜백은 전용 스레드에서 계속 돌린다.
        self._executor = None
        self._spin_thread = None

    def start_spin(self):
        """ROS 콜백을 전용 스레드에서 처리하기 시작한다."""

        from rclpy.executors import SingleThreadedExecutor

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)

        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True
        )
        self._spin_thread.start()

    def stop_spin(self):

        if self._executor is not None:
            try:
                self._executor.shutdown()
            except Exception:
                pass
            self._executor = None

    # ---------------- ROS 입력 ----------------

    def scan_cb(self, msg):
        self.scan = msg

        now = time.monotonic()
        self.last_scan_time = now

        self.scan_count += 1
        self.scan_times.append(now)

    def scan_rate(self):
        """최근 수신 이력으로 계산한 실제 /scan 주파수 [Hz]."""

        if len(self.scan_times) < 2:
            return 0.0

        span = self.scan_times[-1] - self.scan_times[0]

        if span <= 0.0:
            return 0.0

        return (len(self.scan_times) - 1) / span

    def scan_age(self):
        if self.last_scan_time <= 0.0:
            return float("inf")

        return time.monotonic() - self.last_scan_time

    def odom_cb(self, msg):
        self.odom_yaw = yaw_from_quat(msg.pose.pose.orientation)

    def scan_fresh(self):
        """
        스캔이 살아있는지.

        LiDAR 가 3.66 Hz 까지 느려지는 개체라 여유를 크게 잡는다.
        (docs/STATUS.md 의 모터 회전수 불안정 항목)
        """
        return (
            self.last_scan_time > 0.0
            and time.monotonic() - self.last_scan_time < SCAN_TIMEOUT
        )

    def spin_once(self, t=0.005):
        """
        콜백은 start_spin() 의 전용 스레드가 처리한다.
        여기서 rclpy.spin_once 를 또 부르면 executor 가 충돌하므로
        스레드가 살아있는 동안에는 잠깐 양보만 한다.
        """

        if self._executor is not None:
            time.sleep(t)
        else:
            rclpy.spin_once(self, timeout_sec=t)

    def lookup_laser_tf(self):

        try:
            tf = self.tf_buffer.lookup_transform(
                "base_link", "laser_frame", rclpy.time.Time()
            )
        except Exception:
            return

        self.laser_x = tf.transform.translation.x
        self.laser_yaw = yaw_from_quat(tf.transform.rotation)

        self.get_logger().info(
            f"base_link -> laser_frame : "
            f"x={self.laser_x:.3f} "
            f"yaw={math.degrees(self.laser_yaw):.1f} deg"
        )

    def wait_data(self, timeout=15.0):
        """스캔과 TF 가 다 들어올 때까지 기다린다. 없으면 움직이지 않는다."""

        t0 = time.monotonic()

        while time.monotonic() - t0 < timeout:

            self.spin_once(0.05)

            if self.scan is not None and self.laser_yaw is None:
                self.lookup_laser_tf()

            if self.scan is not None and self.laser_yaw is not None:
                return True

        return False

    # ---------------- 안전 ----------------

    def clearance(self, heading=0.0, half_angle=None, stat="min"):
        """
        base_link 기준 heading 방향의 여유거리 [m].

        laser 각도 a 는 base_link 기준으로 (a + laser_yaw) 이므로
        그 값이 heading 근처인 점들만 본다.
        (mapping_drive.py 와 동일)

        half_angle : 판정 반각 [rad]. 생략하면 FRONT_HALF_ANGLE.
                     벽 추종처럼 특정 방향만 좁게 보고 싶을 때 준다.

        stat : "min"    가장 가까운 점. 충돌 회피용.
               "median" 중앙값. 벽까지 거리처럼 값이 흔들리면 안 될 때.
                        먼지/반사 같은 단발 이상치에 안 흔들린다.

        반환값 None 은 "그 방향 range_max 안에 아무 반사점도 없다" 는 뜻이다.
        가까워서 None 이 아니라 뚫려 있어서 None 이므로 호출 쪽에서
        절대 0 처럼 다루면 안 된다.
        """

        s = self.scan

        if s is None or self.laser_yaw is None:
            return None

        if half_angle is None:
            half_angle = FRONT_HALF_ANGLE

        values = []
        best = float("inf")

        for i, r in enumerate(s.ranges):

            if not math.isfinite(r):
                continue

            if not (s.range_min < r < s.range_max):
                continue

            a = s.angle_min + i * s.angle_increment

            if abs(wrap(a + self.laser_yaw - heading)) > half_angle:
                continue

            a_base = wrap(a + self.laser_yaw)

            # laser 가 base_link 앞쪽 x 만큼 나가 있으므로 보정
            r_base = r - self.laser_x * math.cos(a_base)

            if stat == "median":
                values.append(r_base)
            elif r_base < best:
                best = r_base

        if stat == "median":

            if not values:
                return None

            values.sort()

            return values[len(values) // 2]

        return None if math.isinf(best) else best

    # ---------------- 벽 추종 ----------------

    def wall_geometry(self, side_sign):
        """
        벽까지의 수직거리와 벽의 기울기를 추정한다.

        빔 두 개를 쓴다.
            a : 옆쪽 (side_sign * 90 deg)
            b : 옆앞 대각 (side_sign * 45 deg)

        거리 하나만 P 제어하면 로봇이 벽에 대해 비스듬한지 아닌지를
        구분하지 못해서 좌우로 계속 진동한다.
        두 빔을 쓰면 벽의 기울기 alpha 까지 나오므로
        "지금 거리" 가 아니라 "조금 뒤의 거리" 를 보고 조향할 수 있다.

            alpha  : 로봇 진행방향과 벽이 이루는 각.
                     0 이면 벽과 나란히 가는 중.
            d_now  : 현재 벽까지 수직거리
            d_next : WALL_LOOKAHEAD 만큼 더 간 뒤의 예상 수직거리

        반환 : (d_now, d_next, alpha) 또는 벽을 못 찾으면 None
        """

        theta = WALL_BEAM_ANGLE

        # perp : 옆 90도. 벽에 수직으로 쏘는 빔.
        perp = self.clearance(
            side_sign * math.pi / 2.0,
            half_angle=WALL_HALF_ANGLE,
            stat="median",
        )

        # diag : 90 - theta. 앞쪽으로 theta 만큼 기울인 빔.
        diag = self.clearance(
            side_sign * (math.pi / 2.0 - theta),
            half_angle=WALL_HALF_ANGLE,
            stat="median",
        )

        # None = 그 방향이 뚫려 있다 = 따라갈 벽이 없다
        if perp is None or diag is None:
            return None

        if perp > WALL_LOST_DIST or diag > WALL_LOST_DIST:
            return None

        denom = diag * math.sin(theta)

        if abs(denom) < 1e-6:
            return None

        # 벽과 나란하면 alpha = 0.
        # alpha > 0 이면 벽에서 멀어지는 쪽으로 기울어 가는 중이다.
        alpha = math.atan2(diag * math.cos(theta) - perp, denom)

        d_now = perp * math.cos(alpha)
        d_next = d_now + WALL_LOOKAHEAD * math.sin(alpha)

        return d_now, d_next, alpha

    # ---------------- 출력 ----------------

    def publish(self, v, w):
        m = Twist()
        m.linear.x = float(v)
        m.angular.z = float(w)
        self.pub.publish(m)

    def stop(self):
        for _ in range(15):
            self.publish(0.0, 0.0)
            self.spin_once()
            time.sleep(0.02)

    def timed_out(self):
        return time.monotonic() - self.t_start > MAX_RUN_TIME


# ==========================================================
# 화재 인식
# ==========================================================

class FireDetector:

    def __init__(self):

        from ultralytics import YOLO

        print("[YOLO] 모델 로딩 중...")
        self.model = YOLO(MODEL_PATH, task="detect")
        print("[YOLO] classes:", self.model.names)

        self.mlp = None

        if REQUIRE_SENSOR_GATE:
            print("[MLP] 모델 로딩 중...")
            self.mlp = joblib.load(MLP_MODEL_PATH)
            print("[MLP] 로딩 완료")

        self.cap = cv2.VideoCapture(WEBCAM_DEVICE, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, WEBCAM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, WEBCAM_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, WEBCAM_FPS)

        if not self.cap.isOpened():
            raise RuntimeError("카메라를 열지 못했습니다.")

        self.history = deque()

        # TensorRT 첫 추론은 워밍업 때문에 6~7초 걸린다.
        # 주행 루프 안에서 터지면 그 한 틱만 비정상적으로 길어지므로
        # 미리 한 번 돌려 둔다.
        print("[YOLO] 워밍업 중...")

        ret, frame = self.cap.read()

        if ret:
            self.model(frame, conf=CONF_THRES, verbose=False)

        print("[YOLO] 준비 완료")

    def class_name(self, cls_id):
        names = self.model.names

        if isinstance(names, dict):
            return names.get(cls_id, str(cls_id))

        if 0 <= cls_id < len(names):
            return names[cls_id]

        return str(cls_id)

    def step(self):
        """
        한 프레임 처리.

        반환 dict:
            ok          프레임을 읽었는지
            frame       표시용 프레임
            confs       클래스별 최대 confidence
            bearing     불의 방위각 [rad], 없으면 None
                        (+ = 로봇 기준 왼쪽 / CCW)
            prob        MLP 화재 확률
            sensor_ok   센서 연결 여부
            is_fire     이번 프레임이 기준을 넘었는지
        """

        ret, frame = self.cap.read()

        if not ret:
            return {"ok": False}

        results = self.model(frame, conf=CONF_THRES, verbose=False)

        confs = {c: 0.0 for c in TARGET_CLASSES}
        boxes = {}

        for result in results:

            if result.boxes is None:
                continue

            for box in result.boxes:

                cls_id = int(box.cls[0].item())
                confidence = float(box.conf[0].item())
                name = self.class_name(cls_id)

                if name not in TARGET_CLASSES:
                    continue

                confs[name] = max(confs[name], confidence)

                if confidence < BEARING_CONF:
                    continue

                if name not in boxes or confidence > boxes[name]["conf"]:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    boxes[name] = {
                        "conf": confidence,
                        "box": (x1, y1, x2, y2),
                    }

        # ---- 방위각 ----
        bearing = None
        bearing_box = None
        bearing_class = None
        bearing_conf = 0.0

        for name in BEARING_CLASSES:

            if name not in boxes:
                continue

            x1, _, x2, _ = boxes[name]["box"]
            cx = 0.5 * (x1 + x2)

            width = frame.shape[1]

            # 화면 중앙 기준 -1 ~ +1
            norm_x = (cx - 0.5 * width) / (0.5 * width)

            # 화면 오른쪽(+) 은 로봇 기준 시계방향(-) 이다
            bearing = wrap(
                -norm_x * math.radians(CAMERA_HFOV_DEG) * 0.5
                + CAMERA_YAW_OFFSET
            )

            bearing_box = boxes[name]["box"]
            bearing_class = name
            bearing_conf = float(boxes[name]["conf"])
            break

        # ---- 센서 + MLP ----
        gas_raw, ir_temperature, sensor_ok = get_sensor_data()

        prob = 0.0
        temp_change = 0.0
        gas_change = 0.0

        if sensor_ok:

            now = time.monotonic()
            self.history.append((now, ir_temperature, gas_raw))

            while (
                self.history
                and now - self.history[0][0] > CHANGE_WINDOW_SECONDS
            ):
                self.history.popleft()

            if len(self.history) >= 2:
                _, old_temp, old_gas = self.history[0]
                temp_change = ir_temperature - old_temp
                gas_change = gas_raw - old_gas

        else:
            self.history.clear()

        if self.mlp is not None and sensor_ok:

            mlp_input = pd.DataFrame(
                [[
                    confs["fire"],
                    confs["smoke"],
                    confs["cigarette_butt"],
                    mlp_spark_feature(confs),
                    ir_temperature,
                    gas_raw,
                    temp_change,
                    gas_change,
                ]],
                columns=FEATURE_COLUMNS,
            )

            probabilities = self.mlp.predict_proba(mlp_input)[0]
            fire_index = list(self.mlp.classes_).index(1)
            prob = float(probabilities[fire_index])

        # ---- 판단 ----
        if REQUIRE_SENSOR_GATE:
            is_fire = sensor_ok and prob >= FIRE_PROB_THRESHOLD
        else:
            is_fire = confs["fire"] >= YOLO_ONLY_FIRE_CONF

        return {
            "ok": True,
            "frame": frame,
            "confs": confs,
            "bearing": bearing,
            "bearing_box": bearing_box,
            "bearing_class": bearing_class,
            "bearing_conf": bearing_conf,
            "prob": prob,
            "sensor_ok": sensor_ok,
            "gas": gas_raw,
            "temp": ir_temperature,
            "is_fire": is_fire,
        }

    def release(self):
        try:
            self.cap.release()
        except Exception:
            pass


# ==========================================================
# 메인
# ==========================================================

def draw_hud(det, state, clear_m, scan_ok=True):

    frame = det["frame"]

    display_bearing = should_display_detection(
        det.get("bearing_class"), det.get("bearing_conf", 0.0)
    )

    if display_bearing and det.get("bearing_box") is not None:
        x1, y1, x2, y2 = det["bearing_box"]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

    bearing_text = (
        "none" if det["bearing"] is None or not display_bearing
        else f"{math.degrees(det['bearing']):+.1f}deg"
    )

    if not scan_ok:
        clear_text = "NO SCAN"
    elif clear_m is None:
        clear_text = "open"
    else:
        clear_text = f"{clear_m:.2f}m"

    lines = [
        f"STATE: {state}",
        f"BEARING: {bearing_text}   FRONT: {clear_text}",
        f"MLP: {det['prob'] * 100:.1f}%  "
        f"SENSOR: {'OK' if det['sensor_ok'] else 'DISCONNECTED'}",
        f"fire={det['confs']['fire']:.2f} "
        f"smoke={det['confs']['smoke']:.2f}",
    ]

    for i, text in enumerate(lines):
        cv2.putText(
            frame, text, (20, 40 + i * 34),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2,
        )

    return frame


def parse_args(argv):
    """
    사용법
        fire_seeker.py [--patrol wall|bounce] [--side right|left]
                       [--gate mlp|yolo] [--fire-conf 0.40]

    --gate mlp   YOLO + 아두이노 센서 + MLP 로 판단 (기본)
    --gate yolo  MLP 를 빼고 YOLO confidence 만으로 판단.
                 MLP 가 아직 미완성이라 불이 없어도 0.96 을 내놓는
                 상태에서는 이쪽을 써야 순찰이 돈다.
                 (2026-08-25 실측: 불 없음 / fire conf 0.000 / MLP 0.958)

    --fire-conf  --gate yolo 일 때 불로 볼 fire confidence 기준.
                 불을 못 잡으면 낮추고, 헛detect 하면 올린다.

    아무것도 안 주면 파일 상단의 PATROL_MODE / WALL_SIDE /
    REQUIRE_SENSOR_GATE / YOLO_ONLY_FIRE_CONF 를 쓴다.
    """

    global PATROL_MODE, WALL_SIDE, REQUIRE_SENSOR_GATE
    global YOLO_ONLY_FIRE_CONF

    i = 0

    while i < len(argv):

        a = argv[i]

        if a == "--patrol" and i + 1 < len(argv):

            v = argv[i + 1]

            if v not in ("wall", "bounce"):
                print(f"[인자] --patrol 값이 이상하다: {v}")
                sys.exit(2)

            PATROL_MODE = v
            i += 2

        elif a == "--side" and i + 1 < len(argv):

            v = argv[i + 1]

            if v not in ("right", "left"):
                print(f"[인자] --side 값이 이상하다: {v}")
                sys.exit(2)

            WALL_SIDE = v
            i += 2

        elif a == "--gate" and i + 1 < len(argv):

            v = argv[i + 1]

            if v not in ("mlp", "yolo"):
                print(f"[인자] --gate 값이 이상하다: {v}")
                sys.exit(2)

            REQUIRE_SENSOR_GATE = (v == "mlp")
            i += 2

        elif a == "--fire-conf" and i + 1 < len(argv):

            try:
                YOLO_ONLY_FIRE_CONF = float(argv[i + 1])
            except ValueError:
                print(f"[인자] --fire-conf 숫자가 아니다: {argv[i + 1]}")
                sys.exit(2)

            i += 2

        elif a in ("-h", "--help"):
            print(parse_args.__doc__)
            sys.exit(0)

        else:
            print(f"[인자] 모르는 인자: {a}")
            sys.exit(2)


def main():

    parse_args(sys.argv[1:])

    show = SHOW_WINDOW and bool(os.environ.get("DISPLAY"))

    rclpy.init()
    node = FireSeeker()

    # 콜백 전용 스레드를 먼저 띄운다.
    # 이게 없으면 YOLO 추론에 밀려 /scan 콜백이 굶는다.
    node.start_spin()

    stop_event = threading.Event()

    arduino_thread = None

    if REQUIRE_SENSOR_GATE:
        arduino_thread = threading.Thread(
            target=read_arduino, args=(stop_event,), daemon=True
        )
        arduino_thread.start()

    detector = None

    def shutdown():
        stop_event.set()
        try:
            node.stop()
        except Exception:
            pass
        try:
            node.stop_spin()
        except Exception:
            pass
        if detector is not None:
            detector.release()
        if show:
            cv2.destroyAllWindows()

    atexit.register(shutdown)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

    try:
        detector = FireDetector()

        print("[TF] scan / laser_frame 대기 중...")

        if not node.wait_data():
            print("[중지] /scan 또는 base_link->laser_frame TF 가 없다. "
                  "argos_bringup.launch.py 가 떠 있는지 확인할 것.")
            return

        print(
            "[판정] "
            + ("YOLO + 센서 + MLP" if REQUIRE_SENSOR_GATE
               else f"YOLO 단독 (fire conf >= {YOLO_ONLY_FIRE_CONF})")
        )

        if PATROL_MODE == "wall":
            print(
                f"[시작] PATROL (벽 추종, {WALL_SIDE} 벽, "
                f"목표 거리 {WALL_TARGET_DIST:.2f} m)"
            )
        else:
            print("[시작] PATROL (직진/회전)")

        state = "PATROL"

        last_fire_seen = 0.0
        confirm_started = None
        last_scan_warn = 0.0

        # 주행 루프 자체가 느려지는지 보기 위한 계측
        loop_times = deque(maxlen=40)
        loop_hz = 0.0
        last_status = 0.0

        # TURN 상태에서 쓰는 값
        turn_dir = 1.0          # +1 = CCW(왼쪽), -1 = CW(오른쪽)
        turn_from_yaw = None    # 돌기 시작한 odom yaw
        turned = 0.0            # 지금까지 돈 각도 [rad]

        # 벽 추종에서 쓰는 값
        #   side_sign = +1 이면 왼쪽 벽(+90도), -1 이면 오른쪽 벽(-90도)
        side_sign = 1.0 if WALL_SIDE == "left" else -1.0

        wall_lost_since = None  # 벽을 놓친 시각. None 이면 잡고 있다.
        wall_info = "-"         # 상태 출력용

        # 방위각 유지 (BEARING_HOLD_SECONDS 참고)
        last_bearing = None
        last_bearing_time = 0.0

        # 상태 출력이 루프에서 방위각 계산보다 먼저 나오므로
        # 첫 바퀴에서 참조할 수 있도록 미리 정의해 둔다.
        bearing = None
        bearing_fresh = False

        # FIRE_STOP / APPROACH 에서 쓰는 값
        stop_started = 0.0
        approach_started = 0.0

        def enter(new_state):
            nonlocal state
            state = new_state

        while rclpy.ok() and not node.timed_out():

            node.spin_once()

            det = detector.step()

            if not det["ok"]:
                node.publish(0.0, 0.0)
                continue

            now = time.monotonic()
            prev_state = state

            loop_times.append(now)

            if len(loop_times) >= 2:
                span = loop_times[-1] - loop_times[0]
                if span > 0.0:
                    loop_hz = (len(loop_times) - 1) / span

            # 5초마다 상태 한 줄. 조용히 멈춰 있을 때 원인을 보기 위함이다.
            if now - last_status > 5.0:
                wall_text = (
                    f"| 벽 {wall_info} " if PATROL_MODE == "wall" else ""
                )

                print(
                    f"[상태] {state} "
                    f"| /scan {node.scan_rate():.1f} Hz "
                    f"(나이 {node.scan_age():.2f}s, 누적 {node.scan_count}) "
                    f"| 루프 {loop_hz:.0f} Hz "
                    f"{wall_text}"
                    f"| MLP {det['prob'] * 100:.0f}% "
                    f"| 방위 {'실시간' if bearing_fresh else ('유지' if bearing is not None else '없음')} "
                    f"| 센서 {'OK' if det['sensor_ok'] else 'X'}"
                )
                last_status = now

            bearing = det["bearing"]

            # ---- 방위각 끊김 보정 ----
            # YOLO 가 한두 프레임 놓쳐도 직전 방위각으로 버틴다.
            bearing_fresh = bearing is not None

            if bearing_fresh:
                last_bearing = bearing
                last_bearing_time = now

            elif (
                last_bearing is not None
                and now - last_bearing_time <= BEARING_HOLD_SECONDS
            ):
                bearing = last_bearing

            else:
                last_bearing = None

            # ---- 화재 확정 (연속 유지 시간) ----
            if det["is_fire"]:
                if confirm_started is None:
                    confirm_started = now
                confirmed = (now - confirm_started) >= CONFIRM_SECONDS
            else:
                confirm_started = None
                confirmed = False

            if confirmed and bearing is not None:
                last_fire_seen = now

            # ---- 전방 여유 ----
            clear_m = node.clearance(0.0)
            scan_ok = node.scan_fresh()

            # blocked 판정
            #
            #   스캔이 끊김        -> 눈이 멀었다. 무조건 정지.
            #   스캔 O, 반사점 X   -> 전방 range_max 안에 아무것도 없다.
            #                        넓은 공간이므로 "뚫림" 이다.
            #   스캔 O, 반사점 O   -> 거리로 판정.
            def too_close(limit):
                if not scan_ok:
                    return True
                if clear_m is None:
                    return False
                return clear_m < limit

            # ======================================================
            # 스캔이 끊기면 무엇을 하던 중이든 멈춘다
            # ======================================================
            if not scan_ok:

                node.publish(0.0, 0.0)

                if now - last_scan_warn > 2.0:

                    # 원인을 가릴 수 있게 숫자를 같이 찍는다.
                    #
                    #   받은 스캔 0장          -> LiDAR 가 아예 안 붙었다
                    #   직전 Hz 정상, 나이 큼  -> LiDAR 가 도중에 멈췄다
                    #   직전 Hz 도 낮음        -> LiDAR 회전수가 떨어졌다
                    #                            (docs/STATUS.md 모터 불안정)
                    print(
                        f"[경고] /scan 끊김 "
                        f"| 마지막 수신 {node.scan_age():.1f}s 전 "
                        f"| 누적 {node.scan_count}장 "
                        f"| 직전 실측 {node.scan_rate():.1f} Hz "
                        f"| 주행루프 {loop_hz:.0f} Hz"
                    )
                    last_scan_warn = now

                if state in ("PATROL", "TURN"):
                    turn_from_yaw = None

                if show:
                    cv2.imshow(
                        "fire_seeker",
                        draw_hud(det, state, clear_m, scan_ok),
                    )
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

                continue

            # ======================================================
            # 불이 확정되면 순찰을 중단하고 정지부터 한다
            # ======================================================
            if confirmed and bearing is not None:

                if state in ("PATROL", "TURN"):
                    enter("FIRE_STOP")
                    stop_started = now
                    turn_from_yaw = None

            # 불을 놓친 지 오래되면 순찰로 복귀
            elif state in ("FIRE_STOP", "ALIGN", "APPROACH", "HOLD"):

                if now - last_fire_seen > LOST_SECONDS:
                    enter("PATROL")

            # ======================================================
            # 상태별 동작
            # ======================================================

            if state == "PATROL" and PATROL_MODE == "wall":

                # ---------- 한쪽 벽을 따라 돈다 ----------

                if too_close(PATROL_STOP_DIST):

                    # 안쪽 코너다. 벽 반대쪽으로 제자리 회전한다.
                    # (오른쪽 벽을 따라가는 중이면 왼쪽으로 돈다)
                    turn_dir = -side_sign

                    turn_from_yaw = node.odom_yaw
                    turned = 0.0
                    wall_lost_since = None

                    enter("TURN")
                    node.publish(0.0, 0.0)

                else:

                    geom = node.wall_geometry(side_sign)

                    if geom is None:

                        # 따라가던 벽이 사라졌다.
                        # 대부분 바깥 코너를 돈 직후다.
                        # 벽 쪽으로 완만하게 감아 들어가며 다시 잡는다.

                        if wall_lost_since is None:
                            wall_lost_since = now

                        if now - wall_lost_since > WALL_SEARCH_TIMEOUT:

                            # 이만큼 감아 돌았는데도 못 찾으면
                            # 전진을 멈추고 제자리에서 찾는다.
                            turn_dir = side_sign
                            turn_from_yaw = node.odom_yaw
                            turned = 0.0
                            wall_lost_since = None

                            enter("TURN")
                            node.publish(0.0, 0.0)

                        else:
                            node.publish(
                                PATROL_V * 0.7,
                                side_sign * WALL_SEARCH_W,
                            )

                    else:

                        wall_lost_since = None

                        d_now, d_next, alpha = geom

                        # d_next 가 목표보다 작다 = 벽에 너무 붙었다
                        # -> 벽 반대쪽으로 조향
                        err = WALL_TARGET_DIST - d_next

                        w = -side_sign * WALL_KP * err
                        w = max(-WALL_MAX_W, min(WALL_MAX_W, w))

                        # 앞이 가까울수록, 많이 꺾을수록 감속한다
                        v = PATROL_V

                        if clear_m is not None:
                            margin = (clear_m - PATROL_STOP_DIST) / 0.4
                            v *= max(0.3, min(1.0, margin))

                        v *= max(0.5, 1.0 - 0.5 * abs(w) / WALL_MAX_W)

                        node.publish(v, w)

                        wall_info = (
                            f"{d_now:.2f}m "
                            f"alpha {math.degrees(alpha):+.0f}deg"
                        )

            elif state == "PATROL":

                # ---------- 원래 방식: 트이면 직진, 막히면 회전 ----------

                if too_close(PATROL_STOP_DIST):

                    # 좌우 중 더 트인 쪽으로 돈다
                    left = node.clearance(SIDE_HEADING)
                    right = node.clearance(-SIDE_HEADING)

                    # None = 그 방향은 range_max 까지 뚫림 = 가장 좋다
                    left_score = float("inf") if left is None else left
                    right_score = float("inf") if right is None else right

                    turn_dir = 1.0 if left_score >= right_score else -1.0

                    turn_from_yaw = node.odom_yaw
                    turned = 0.0

                    enter("TURN")
                    node.publish(0.0, 0.0)

                else:
                    node.publish(PATROL_V, 0.0)

            elif state == "TURN":

                # 돈 각도를 odom 으로 누적한다
                if node.odom_yaw is not None:

                    if turn_from_yaw is None:
                        turn_from_yaw = node.odom_yaw

                    turned += abs(wrap(node.odom_yaw - turn_from_yaw))
                    turn_from_yaw = node.odom_yaw

                if not too_close(PATROL_RESUME_DIST):
                    # 앞이 충분히 트였다
                    enter("PATROL")
                    turn_from_yaw = None
                    node.publish(0.0, 0.0)

                elif turned > math.radians(PATROL_TURN_GIVEUP_DEG):
                    # 이만큼 돌았는데도 계속 막힌다. 반대로 돌아본다.
                    turn_dir = -turn_dir
                    turned = 0.0
                    node.publish(0.0, turn_dir * PATROL_TURN_W)

                else:
                    node.publish(0.0, turn_dir * PATROL_TURN_W)

            elif state == "FIRE_STOP":

                # 관성을 죽인다. 이 동안에는 아무 명령도 주지 않는다.
                node.publish(0.0, 0.0)

                if now - stop_started >= FIRE_STOP_PAUSE:
                    enter("ALIGN")

            elif state == "ALIGN":

                if bearing is None:
                    # 방향을 모르면 움직이지 않는다
                    node.publish(0.0, 0.0)

                elif abs(bearing) <= math.radians(ALIGN_TOL_DEG):
                    # 몸을 다 돌렸다
                    enter("APPROACH")
                    approach_started = now
                    node.publish(0.0, 0.0)

                else:
                    # 제자리에서 몸만 돌린다
                    w = TURN_KP * bearing
                    w = max(-ALIGN_W_MAX, min(ALIGN_W_MAX, w))
                    node.publish(0.0, w)

            elif state == "APPROACH":

                if bearing is None:
                    # 순간적으로 놓쳤다. LOST_SECONDS 까지는 기다린다.
                    node.publish(0.0, 0.0)

                elif too_close(FIRE_STOP_DIST):
                    # 충분히 가까워졌거나 앞이 막혔다
                    enter("HOLD")
                    node.publish(0.0, 0.0)

                elif now - approach_started > APPROACH_MAX_SECONDS:
                    # 불이 LiDAR 에 안 잡히는 경우 무한 전진 방지
                    print(f"[제한] 접근 {APPROACH_MAX_SECONDS:.0f}s 초과 -> 정지")
                    enter("HOLD")
                    node.publish(0.0, 0.0)

                elif abs(bearing) > math.radians(REALIGN_DEG):
                    # 많이 틀어졌으면 다시 제자리에서 맞춘다
                    enter("ALIGN")
                    node.publish(0.0, 0.0)

                else:
                    # 가면서 조금씩 보정
                    w = TURN_KP * bearing
                    w = max(-APPROACH_W_MAX, min(APPROACH_W_MAX, w))
                    node.publish(APPROACH_V, w)

            else:  # HOLD
                node.publish(0.0, 0.0)

            # ---- 상태가 바뀔 때만 한 줄 ----
            if state != prev_state:

                if clear_m is None:
                    clear_text = "탁 트임"
                else:
                    clear_text = f"{clear_m:.2f} m"

                bearing_text = (
                    "없음" if bearing is None
                    else f"{math.degrees(bearing):+.0f}deg"
                )

                print(
                    f"[{state}] {prev_state} -> {state} | "
                    f"전방 {clear_text} | 불 {bearing_text} | "
                    f"MLP {det['prob'] * 100:.0f}%"
                )

            if show:
                cv2.imshow(
                    "fire_seeker",
                    draw_hud(det, state, clear_m, scan_ok),
                )

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        if node.timed_out():
            print(f"[종료] 시간 상한 {MAX_RUN_TIME:.0f}s 도달")

    except KeyboardInterrupt:
        print("\n[중단] Ctrl+C")

    finally:
        shutdown()

        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
