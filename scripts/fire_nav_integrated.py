#!/usr/bin/env python3

"""ARGOS Nav2 자동 순찰 + 화재 접근 (인지 결과는 토픽으로만 받는다)

fire_nav_patrol.py 에서 인지 부분을 전부 들어낸 것이다.

  이 노드가 하지 않는 것
      YOLO 를 만들지 않는다.        (인지 원본이 1개만 만든다)
      카메라를 열지 않는다.          (인지 원본이 1개만 연다)
      /dev/ttyACM0 를 열지 않는다.   (인지 원본이 1개만 연다)
      MLP 를 읽지 않는다.            (인지 원본이 텔레그램 판단에만 쓴다)
      텔레그램을 보내지 않는다.      (인지 원본이 보낸다)

  이 노드가 하는 것
      /argos/fire_detection 구독  ->  화재 방위각 계산  ->  접근 제어
      Nav2 자동 순찰 (목표 선정 / goal 전송 / 취소)
      /cmd_vel 중재와 LiDAR 안전거리

인지 결과는 fire_perception_main.py 가 발행한다. 그쪽이 인지팀 원본
new_main_robot_map.py 를 수정 없이 실행하는 메인 프로세스다.

방위각
------
토픽에는 norm_x (화면 중앙 기준 -1 ~ +1) 만 실려 온다. 카메라 화각을 모르는
순수 기하값이다. 여기서 카메라 파라미터를 적용해 방위각으로 바꾼다.

    bearing = -norm_x * HFOV/2 + yaw_offset

화면 오른쪽(+)은 로봇 기준 시계방향(-)이므로 부호가 뒤집힌다.
클래스별 confidence 기준도 여기서 정한다. 인지팀이 자기 화면 표시 기준
(DISPLAY_CONF)을 어떻게 바꾸든 주행에는 영향이 없다.

방위각 부호 — 실물로 확인된 상태다
----------------------------------
fire_seeker.py 현장 시험에서 "왼쪽에 있는 불을 보면 왼쪽으로 돈다" 가
확인되었다. 이 파일은 그때와 같은 계산을 쓰므로 그대로 유효하다.

    bearing 계산   -norm_x * HFOV/2 + offset      fire_seeker 와 동일
    ALIGN 제어     w = TURN_KP * bearing          fire_seeker 와 동일
    카메라 경로    cv2.VideoCapture(0, CAP_V4L2)  원본도 동일, 영상 반전 없음
                   (원본 gstreamer_pipeline 은 정의만 있고 호출되지 않는다)

    왼쪽 불 -> norm_x < 0 -> bearing > 0 -> angular.z > 0 -> CCW -> 좌회전

해상도가 640x480 에서 1280x720 으로 바뀌었지만 norm_x 가 정규화 값이라
영향이 없다.

카메라를 다시 장착했다면 30초만 재확인하면 된다. 불을 화면 오른쪽에 두고

    ros2 topic echo /argos/fire_detection --once

boxes.fire.norm_x 가 양수면 그대로 두고, 음수면 영상이 좌우 반전된
것이므로 bearing_sign 을 -1.0 으로 둔다. 코드는 고치지 않는다.

참고: angular.z > 0 이 실제로 CCW 인지는 별도 도구가 있다.

         python3 check_cmd_vel_direction.py rotate

텔레그램 전송 후 출발
--------------------
화재를 확정하면 일단 멈추고, 인지 원본이 텔레그램을 정상 전송할 때까지
기다린 뒤에 접근을 시작한다 (ALERT_WAIT).

원본의 발송 조건은 센서 연결 + MLP >= 0.70 + 쿨다운이다.
MLP 는 threshold 0.70 에서 precision 1.000 / recall 0.667 이다
(training_result.txt, 2026-08-25 재학습). 오경보는 없지만 위험 상황의
약 1/3 은 알림이 나가지 않고, 쿨다운 중에 다시 만나면 반드시 안 나간다.

그래서 두 가지 탈출구를 둔다.

  alert_expected=false   인지가 "이번엔 안 보낸다" 고 알려준 경우.
                         alert_skip_grace 만 지나면 바로 출발한다.
  alert_wait_timeout     그 외의 경우. 이 시간을 넘기면 알림이 나가지
                         않았어도 접근을 시작한다.

이게 없으면 로봇이 나오지 않을 알림을 기다리며 계속 서 있게 된다.

상태
----
  NAV_PATROL   Nav2 가 순찰한다. 이 노드는 /cmd_vel 을 통과만 시킨다.
  FIRE_STOP    위험 확정. Nav2 목표를 취소하고 완전히 멈춘다.
  ALERT_WAIT   멈춘 채로 텔레그램 전송 완료를 기다린다.
  ALIGN        제자리에서 몸을 불 쪽으로 돌린다.
  APPROACH     불 쪽으로 다가간다. 가면서 방향을 보정한다.
  HOLD         fire_stop_dist 까지 붙었으면 멈춰서 유지한다.

위험 종류에 따라 ALERT_WAIT 뒤가 갈린다.

  불 (YOLO fire >= fire_conf, 방향을 알 수 있음)
      FIRE_STOP -> ALERT_WAIT -> ALIGN -> APPROACH -> HOLD

  불이 아닌 위험 (담배꽁초 / 연기 등, 원본 MLP 위험 판정)
      FIRE_STOP -> ALERT_WAIT -> 순찰 재개
      접근하지 않는다. 알림을 보내는 것이 목적이다.
      재개 후 danger_stop_cooldown 동안은 같은 위험으로 다시 멈추지 않는다.

실행
----
    # 터미널 1
    ros2 launch argos_bringup argos_bringup.launch.py

    # 터미널 2
    ros2 launch argos_bringup argos_navigation.launch.py \
        nav_cmd_vel_topic:=/cmd_vel_nav_auto

    # 터미널 3  (먼저 떠 있어야 한다. TensorRT 워밍업 6~7초)
    ~/.venv/bin/python ~/argos_project/scripts/fire_perception_main.py

    # 터미널 4
    ~/.venv/bin/python ~/argos_project/scripts/fire_nav_integrated.py \
        --ros-args --params-file ~/argos_project/config/fire_nav_integrated.yaml
"""

from __future__ import annotations

import json
import math
import random
import signal
import sys
import threading
import time
from typing import Optional

import numpy as np

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def sector_clearance(
    ranges,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    laser_x: float,
    laser_yaw: float,
    heading: float,
    half_angle: float,
) -> Optional[float]:
    """base_link 기준 부채꼴 안의 최소 여유거리를 계산한다."""

    best = float("inf")

    for index, distance in enumerate(ranges):
        if not math.isfinite(distance):
            continue
        if not (range_min < distance < range_max):
            continue

        laser_angle = angle_min + index * angle_increment
        base_angle = wrap(laser_angle + laser_yaw)

        if abs(wrap(base_angle - heading)) > half_angle:
            continue

        corrected = distance - laser_x * math.cos(base_angle)
        best = min(best, corrected)

    return None if math.isinf(best) else best


def _disk_is_free(
    grid: np.ndarray,
    gx: int,
    gy: int,
    radius_cells: int,
    free_max: int,
) -> bool:
    height, width = grid.shape

    if (
        gx - radius_cells < 0
        or gy - radius_cells < 0
        or gx + radius_cells >= width
        or gy + radius_cells >= height
    ):
        return False

    window = grid[
        gy - radius_cells:gy + radius_cells + 1,
        gx - radius_cells:gx + radius_cells + 1,
    ]

    yy, xx = np.ogrid[-radius_cells:radius_cells + 1,
                      -radius_cells:radius_cells + 1]
    disk = xx * xx + yy * yy <= radius_cells * radius_cells
    values = window[disk]

    return bool(np.all((values >= 0) & (values <= free_max)))


def select_patrol_target(
    grid: np.ndarray,
    resolution: float,
    origin_x: float,
    origin_y: float,
    robot_x: float,
    robot_y: float,
    min_radius: float,
    max_radius: float,
    clearance: float,
    free_max: int,
    attempts: int,
    rng: random.Random,
) -> Optional[tuple[float, float, float]]:
    """현재 위치 주변의 안전한 자유 공간에서 순찰 목표를 고른다."""

    if grid.ndim != 2 or resolution <= 0.0:
        return None

    radius_cells = max(1, int(math.ceil(clearance / resolution)))

    start_gx = int((robot_x - origin_x) / resolution)
    start_gy = int((robot_y - origin_y) / resolution)

    height, width = grid.shape
    if not (0 <= start_gx < width and 0 <= start_gy < height):
        return None

    # 예전에는 로봇 자신의 원판이 깨끗하지 않으면 여기서 바로 포기했다.
    # 그건 빠져나갈 수 없는 상태를 만든다: 제 자리가 위험하다고 판단해서
    # 안 움직이는데, 안 움직이니 영원히 그 자리다.  2026-09-01 실외 시험에서
    # 순찰 목표 4개를 소화한 뒤 원판에 미지 7 · 점유 13 셀이 걸려 151 번
    # 연속으로 표본을 한 번도 못 뽑고 멈춰 섰다.
    #
    # 로봇은 이미 그 자리에 물리적으로 있다.  나가는 길을 찾는 것이 맞다.
    # 목표점 쪽 검사는 그대로 엄격하게 두므로 위험한 곳으로 보내지는 않는다.

    for _ in range(max(1, attempts)):
        angle = rng.uniform(-math.pi, math.pi)
        distance = rng.uniform(min_radius, max_radius)

        target_x = robot_x + distance * math.cos(angle)
        target_y = robot_y + distance * math.sin(angle)

        target_gx = int((target_x - origin_x) / resolution)
        target_gy = int((target_y - origin_y) / resolution)

        if not _disk_is_free(
            grid, target_gx, target_gy, radius_cells, free_max
        ):
            continue

        # 목표점 자체만 여기서 검사한다. 중간에 장애물이 있어도
        # SmacPlannerLattice 가 우회 경로를 만들 수 있으므로 직선 구간을
        # 강제하면 유효한 순찰 목표를 지나치게 많이 버리게 된다.
        return target_x, target_y, angle

    return None


def clone_twist(source: Twist) -> Twist:
    target = Twist()
    target.linear.x = source.linear.x
    target.linear.y = source.linear.y
    target.linear.z = source.linear.z
    target.angular.x = source.angular.x
    target.angular.y = source.angular.y
    target.angular.z = source.angular.z
    return target


class FireNavIntegrated(Node):

    def __init__(self) -> None:
        super().__init__("argos_fire_nav")

        self._declare_parameters()
        self._read_parameters()

        self._data_lock = threading.Lock()
        self._control_lock = threading.Lock()
        self._goal_lock = threading.Lock()
        self._detection_lock = threading.Lock()

        self._detection: Optional[dict] = None
        self._detection_at = 0.0
        self._detection_count = 0
        self._bad_detection_logged = 0.0

        self._map_grid: Optional[np.ndarray] = None
        self._map_resolution = 0.0
        self._map_origin_x = 0.0
        self._map_origin_y = 0.0
        self._map_origin_yaw = 0.0
        self._pose: Optional[PoseWithCovarianceStamped] = None
        self._initial_pose_received = not self.require_initial_pose

        self._scan: Optional[LaserScan] = None
        self._last_scan_at = 0.0
        self._laser_x: Optional[float] = None
        self._laser_yaw: Optional[float] = None

        self._control_mode = "stop"
        self._nav_cmd = Twist()
        self._nav_cmd_at = 0.0
        self._fire_cmd = Twist()
        self._fire_cmd_at = 0.0

        self._goal_token = 0
        self._goal_pending = False
        self._goal_handle = None
        self._goal_sent_at = 0.0
        self._next_goal_after = 0.0
        self._goal_count = 0

        self._rng = random.Random()
        self._last_wait_log = 0.0
        self._last_scan_log = 0.0

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        detection_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(
            String,
            self.detection_topic,
            self._detection_cb,
            detection_qos,
        )
        self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_cb, map_qos
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self._pose_cb,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            self._initial_pose_cb,
            10,
        )
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist,
            self.nav_cmd_topic,
            self._nav_cmd_cb,
            10,
        )

        self._fire_cmd_pub = self.create_publisher(
            Twist, self.fire_cmd_topic, 10
        )
        # HOLD 지점(불 0.6 m 앞)에서 확정한 화재 좌표.
        # 팔로워봇이 이 좌표로 와서 물을 뿌린다.
        #
        # 왜 여기서 잡는 좌표가 정확한가
        #   단안 카메라는 방위각만 주고 거리를 못 준다. 그런데 HOLD 는
        #   전방 여유가 fire_stop_dist(0.6 m) 이하가 됐을 때 들어오므로
        #   "불이 정면 0.6 m 앞" 이라는 것이 확정된다. 거리 추정이 사라진다.
        #
        # transient_local 로 둬서 팔로워봇이 늦게 접속해도 마지막 목표를 받는다.
        self._fire_target_pub = self.create_publisher(
            PoseStamped,
            "/main/fire_target",
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
            ),
        )

        # 화재 처리 구간 신호.
        #
        # true  = 지금 화재를 처리 중이다 (정지/정렬/접근/유지)
        # false = 순찰 중이다
        #
        # 인지 쪽이 이걸 보고 "접근하는 동안에는 원본 알림을 더 보내지
        # 않는다". 쿨다운 숫자로 맞추면 접근 시간이 바뀔 때마다 깨지는데,
        # 상태로 막으면 그럴 일이 없다.
        #
        # transient_local 이라 인지가 늦게 떠도 현재 상태를 바로 받는다.
        self._episode_pub = self.create_publisher(
            Bool,
            "/argos/fire_episode",
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
            ),
        )

        # 팔로워봇 진화 지령.
        #
        # 팔로워 쪽 fire_supervisor 가 /fire/dispatch 를 구독한다.
        # 프로토콜은 엄격하다 (follower_fire_control/protocol.py).
        #   필드 7개 정확히: schema, mission_id, frame_id, x, y, yaw,
        #                   main_cleared
        #   schema 는 1, frame_id 는 "map", main_cleared 는 JSON bool.
        #   추가 필드가 있거나 하나라도 빠지면 거부된다.
        #
        # 같은 mission_id 로 두 번 보낸다.
        #   HOLD 도달   main_cleared=false  -> 팔로워는 WAIT_CLEARANCE
        #   순찰 복귀   main_cleared=true   -> 팔로워가 출발
        #
        # 메인봇이 불 0.6 m 앞에 서 있는 동안 팔로워가 물을 뿌리면
        # 그대로 맞기 때문이다. 두 번째 지령이 "이제 비켰다" 는 신호다.
        #
        # 좌표는 두 번 다 완전히 같아야 한다. 팔로워는 1e-6 이내로
        # 비교해서 다르면 "same mission_id cannot change target" 으로
        # 거부한다. 그래서 첫 지령을 그대로 저장해 두었다가 재사용한다.
        self._dispatch_pub = self.create_publisher(
            String, "/fire/dispatch", 10
        )

        # 마지막으로 보낸 지령 (mission_id, x, y, yaw)
        self._dispatch: Optional[tuple] = None

        # 인지 프로세스에 "지금 이 자리에서 알림을 보내라" 고 요청한다.
        # 텔레그램 전송은 인지팀 원본이 소유하므로 우리가 직접 못 보낸다.
        self._alert_request_pub = self.create_publisher(
            String, "/argos/fire_alert_request", 10
        )

        self._output_cmd_pub = self.create_publisher(
            Twist, self.output_cmd_topic, 10
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._navigator = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )

        self.create_timer(1.0 / self.mux_rate, self._mux_tick)
        self.create_timer(0.5, self._patrol_tick)

        self.get_logger().info(
            "화재 순찰 제어기 시작: 초기에는 정지, "
            f"{self.detection_topic} 수신 후 Nav2 순찰 허용"
        )

    # ---------------- 파라미터 ----------------

    def _declare_parameters(self) -> None:
        defaults = {
            # 인지 토픽
            "detection_topic": "/argos/fire_detection",
            # 이 시간 동안 새 인지 결과가 없으면 인지가 끊긴 것으로 본다.
            # 프레임 단위 판단이 아니라 "인지 프로세스 생존 확인" 용도다.
            # 원본은 화재 확정 순간 텔레그램 사진/지도를 메인 루프 안에서
            # 동기로 만들기 때문에 수백 ms 발행이 끊긴다. 넉넉히 잡는다.
            "detection_timeout": 1.5,

            # 카메라 기하
            "camera_hfov_deg": 60.0,
            "camera_yaw_offset_deg": 0.0,
            # 카메라가 뒤집혀 달렸거나 영상이 좌우 반전되면 화면 오른쪽이
            # 로봇 기준 오른쪽이 아니게 된다. 그때 이 값을 -1.0 으로 두면
            # 코드를 고치지 않고 부호만 뒤집을 수 있다.
            "bearing_sign": 1.0,

            # 클래스별 방위각 근거 최소 confidence
            "bearing_conf_fire": 0.10,
            "bearing_conf_smoke": 0.25,
            "bearing_conf_spark": 0.50,
            # 담배꽁초도 방위각 후보에 넣는다. 0.99 는 사실상 "쓰지 않음"
            # 이었다. 접근 대상이 되었으므로 실제 임계값을 준다.
            "bearing_conf_cigarette_butt": 0.50,
            "bearing_priority": ["fire", "smoke", "spark", "cigarette_butt"],

            # 접근해서 좌표를 넘길 대상. 여기 있는 클래스만 ALIGN/APPROACH
            # 로 간다. 나머지는 MLP danger 경로로 "멈춰서 알리고 복귀" 만 한다.
            #
            # 담배꽁초를 넣은 이유
            #   꺼지지 않은 꽁초는 발화원이다. 팔로워봇이 물을 뿌릴 대상이
            #   되므로 불과 같은 처리를 한다.
            #   접근을 원치 않으면 이 목록에서 빼면 된다. 그래도 danger
            #   경로로 정지와 알림은 그대로 동작한다.
            "approach_classes": ["fire", "cigarette_butt"],

            # 접근을 시작할 클래스별 confidence. fire 는 기존 fire_conf 를 쓴다.
            # 꽁초는 작고 오탐이 잦아 불보다 높게 잡는다.
            "approach_conf_cigarette_butt": 0.50,
            "approach_conf_smoke": 0.60,
            "approach_conf_spark": 0.50,

            # 화재 확정
            "fire_conf": 0.35,
            "confirm_seconds": 1.0,
            "bearing_hold_seconds": 1.2,
            "lost_seconds": 2.0,
            "hold_release_seconds": 5.0,

            # 텔레그램 전송을 기다렸다가 출발한다
            #   sent      정상 전송된 뒤에 출발          (기본)
            #   attempted 전송을 시도한 뒤에 출발 (실패 포함)
            #   none      기다리지 않는다
            "alert_gate": "sent",
            "alert_wait_timeout": 10.0,
            # 인지가 "이번 화재는 알림 대상이 아니다"(alert_expected=false)
            # 라고 알려줘도 이 시간만큼은 기다려 본다. 원본의 연속 확인
            # 1초와 순간적인 확률 흔들림을 흡수하기 위한 여유다.
            "alert_skip_grace": 1.5,

            # 불을 확정한 뒤 접근할지 여부.
            #
            # true 인 이유
            #   메인봇이 불 근처까지 가면 "메인봇의 현재 위치" 자체가
            #   불 위치의 대용이 된다. 단안 카메라로 불까지의 거리를
            #   추정할 필요가 없어진다. 팔로워봇은 그 좌표로 오면 된다.
            #
            # 대신 반드시 자리를 떠야 한다. 팔로워봇이 그 지점에 물을
            # 뿌리기 때문이다. hold_max_seconds 를 참고할 것.
            "approach_on_fire": True,

            # HOLD 에서 최대 이 시간까지만 머문다.
            #
            # 원래는 불이 사라져야만 순찰로 복귀했다. 그러면 불이 계속
            # 타는 동안 메인봇이 0.6 m 앞에 계속 서 있게 되는데,
            # 그 자리가 팔로워봇의 물줄기가 향하는 곳이다.
            # 알릴 만큼 알렸으면 비켜야 한다.
            "hold_max_seconds": 15.0,

            # 불이 아닌 위험(담배꽁초 / 연기 등)에서도 멈춰서 알림을
            # 보내고, 전송이 끝나면 접근하지 않고 순찰로 돌아간다.
            # 스파크 / 담배꽁초는 오탐이 잦다.  이 값 미만이면 그 둘만으로는
            # 위험상황으로 확정하지 않는다.  불 / 연기 / 센서 단독은 그대로 둔다.
            "danger_conf_spark": 0.50,
            "danger_conf_cigarette_butt": 0.50,
            "danger_stop_enabled": True,
            # 같은 위험을 계속 보면서 매번 멈추지 않도록 하는 억제 시간.
            # 원본 텔레그램 쿨다운과 같게 두는 것이 자연스럽다.
            "danger_stop_cooldown": 30.0,

            "require_initial_pose": True,

            # 자동 순찰 목표 선택
            "patrol_min_radius": 0.8,
            "patrol_max_radius": 2.5,
            "patrol_clearance": 0.35,
            "patrol_goal_timeout": 75.0,
            "patrol_retry_delay": 1.5,
            "patrol_sample_attempts": 120,
            "patrol_free_max": 10,

            # 화재 접근
            "fire_stop_pause": 0.7,
            "align_tolerance_deg": 8.0,
            "realign_deg": 26.0,
            "turn_kp": 1.2,
            "align_w_max": 0.35,
            "approach_v": 0.08,
            "approach_w_max": 0.25,
            "fire_stop_dist": 0.60,
            "approach_max_seconds": 25.0,

            # LiDAR / 속도 중재
            "scan_timeout": 1.5,
            "front_half_angle_deg": 25.0,
            "nav_cmd_timeout": 0.50,
            "fire_cmd_timeout": 0.30,
            "mux_rate": 20.0,

            # 토픽
            "nav_cmd_topic": "/cmd_vel_nav_auto",
            "fire_cmd_topic": "/cmd_vel_fire",
            "output_cmd_topic": "/cmd_vel_nav",
            "map_topic": "/map",
            "pose_topic": "/amcl_pose",
            "initial_pose_topic": "/initialpose",
            "scan_topic": "/scan",
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self) -> None:
        value = lambda name: self.get_parameter(name).value

        self.detection_topic = str(value("detection_topic"))
        self.detection_timeout = float(value("detection_timeout"))

        self.camera_hfov = math.radians(float(value("camera_hfov_deg")))
        self.camera_yaw_offset = math.radians(
            float(value("camera_yaw_offset_deg"))
        )
        self.bearing_sign = 1.0 if float(value("bearing_sign")) >= 0.0 else -1.0

        self.bearing_priority = [
            str(name) for name in value("bearing_priority")
        ]
        self.approach_classes = [
            str(name) for name in value("approach_classes")
        ]
        self.approach_conf = {
            "fire": float(value("fire_conf")),
            "cigarette_butt": float(value("approach_conf_cigarette_butt")),
            "smoke": float(value("approach_conf_smoke")),
            "spark": float(value("approach_conf_spark")),
        }
        self.bearing_conf = {
            "fire": float(value("bearing_conf_fire")),
            "smoke": float(value("bearing_conf_smoke")),
            "spark": float(value("bearing_conf_spark")),
            "cigarette_butt": float(value("bearing_conf_cigarette_butt")),
        }

        self.fire_conf = float(value("fire_conf"))
        self.confirm_seconds = float(value("confirm_seconds"))
        self.bearing_hold_seconds = float(value("bearing_hold_seconds"))
        self.lost_seconds = float(value("lost_seconds"))
        self.hold_release_seconds = float(value("hold_release_seconds"))

        self.alert_gate = str(value("alert_gate")).strip().lower()

        if self.alert_gate not in {"sent", "attempted", "none"}:
            raise ValueError(
                "alert_gate 는 'sent', 'attempted', 'none' 중 하나여야 합니다"
            )

        self.alert_wait_timeout = float(value("alert_wait_timeout"))
        self.alert_skip_grace = float(value("alert_skip_grace"))
        self.approach_on_fire = bool(value("approach_on_fire"))
        self.hold_max_seconds = float(value("hold_max_seconds"))
        self.danger_class_floor = {
            "spark": float(value("danger_conf_spark")),
            "cigarette_butt": float(value("danger_conf_cigarette_butt")),
        }
        self.danger_stop_enabled = bool(value("danger_stop_enabled"))
        self.danger_stop_cooldown = float(value("danger_stop_cooldown"))

        self.require_initial_pose = bool(value("require_initial_pose"))

        self.patrol_min_radius = float(value("patrol_min_radius"))
        self.patrol_max_radius = float(value("patrol_max_radius"))
        self.patrol_clearance = float(value("patrol_clearance"))
        self.patrol_goal_timeout = float(value("patrol_goal_timeout"))
        self.patrol_retry_delay = float(value("patrol_retry_delay"))
        self.patrol_sample_attempts = int(value("patrol_sample_attempts"))
        self.patrol_free_max = int(value("patrol_free_max"))

        self.fire_stop_pause = float(value("fire_stop_pause"))
        self.align_tolerance = math.radians(
            float(value("align_tolerance_deg"))
        )
        self.realign_angle = math.radians(float(value("realign_deg")))
        self.turn_kp = float(value("turn_kp"))
        self.align_w_max = float(value("align_w_max"))
        self.approach_v = float(value("approach_v"))
        self.approach_w_max = float(value("approach_w_max"))
        self.fire_stop_dist = float(value("fire_stop_dist"))
        self.approach_max_seconds = float(value("approach_max_seconds"))

        self.scan_timeout = float(value("scan_timeout"))
        self.front_half_angle = math.radians(
            float(value("front_half_angle_deg"))
        )
        self.nav_cmd_timeout = float(value("nav_cmd_timeout"))
        self.fire_cmd_timeout = float(value("fire_cmd_timeout"))
        self.mux_rate = max(1.0, float(value("mux_rate")))

        self.nav_cmd_topic = str(value("nav_cmd_topic"))
        self.fire_cmd_topic = str(value("fire_cmd_topic"))
        self.output_cmd_topic = str(value("output_cmd_topic"))
        self.map_topic = str(value("map_topic"))
        self.pose_topic = str(value("pose_topic"))
        self.initial_pose_topic = str(value("initial_pose_topic"))
        self.scan_topic = str(value("scan_topic"))

    # ---------------- 인지 토픽 ----------------

    def _detection_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError) as error:
            now = time.monotonic()

            if now - self._bad_detection_logged > 5.0:
                self._bad_detection_logged = now
                self.get_logger().error(f"인지 메시지 해석 실패: {error}")
            return

        if not isinstance(payload, dict):
            return

        with self._detection_lock:
            self._detection = payload
            self._detection_at = time.monotonic()
            self._detection_count += 1
            first = self._detection_count == 1

        if first:
            self.get_logger().info(
                f"인지 토픽 수신 시작: {self.detection_topic}"
            )

    def latest_detection(self) -> tuple[Optional[dict], float]:
        """최신 인지 결과와 그 나이(초)를 돌려준다."""

        with self._detection_lock:
            payload = self._detection
            received_at = self._detection_at

        if payload is None:
            return None, float("inf")

        return payload, time.monotonic() - received_at

    def detection_fresh(self, age: float) -> bool:
        return age <= self.detection_timeout

    # ---------------- 방위각 ----------------

    def bearing_from(
        self, payload: dict
    ) -> tuple[Optional[float], Optional[str]]:
        """토픽의 norm_x 에 카메라 파라미터를 적용해 방위각을 만든다.

        클래스별 confidence 기준과 우선순위는 이 노드의 파라미터가 정한다.
        인지팀의 화면 표시 기준과는 무관하다.
        """

        boxes = payload.get("boxes") or {}

        for name in self.bearing_priority:
            box = boxes.get(name)

            if not isinstance(box, dict):
                continue

            try:
                confidence = float(box.get("conf", 0.0))
                norm_x = float(box["norm_x"])
            except (KeyError, TypeError, ValueError):
                continue

            if confidence < self.bearing_conf.get(name, 1.0):
                continue

            # 화면 오른쪽(+)은 로봇 기준 시계방향(-)이다.
            bearing = wrap(
                -self.bearing_sign * norm_x * self.camera_hfov * 0.5
                + self.camera_yaw_offset
            )
            return bearing, name

        return None, None

    @staticmethod
    def fire_confidence(payload: dict) -> float:
        try:
            return float((payload.get("confs") or {}).get("fire", 0.0))
        except (TypeError, ValueError):
            return 0.0

    @staticmethod
    def class_confidence(payload: dict, name: str) -> float:
        try:
            return float((payload.get("confs") or {}).get(name, 0.0))
        except (TypeError, ValueError):
            return 0.0

    def pose_ready(self) -> bool:
        """화재 좌표를 계산해도 되는 상태인가.

        화재 좌표는 로봇 자세에 전적으로 의존한다.  위치추정이 안 된
        상태에서 계산하면 그 오차가 그대로 팔로워에게 전달되어 엉뚱한
        곳에 물을 쏘게 된다.  2026-09-01 시험에서 실제로 그랬다:
        메인봇이 초기 자세를 받기 60 s 전에 지령 두 건을 보냈고,
        약 0.8 m 어긋난 좌표가 나갔다.

        순찰 쪽(_patrol_tick)에는 이 검사가 있었지만 화재 대응 쪽에는
        없었다.  알림은 인지 노드가 따로 내므로 여기서 막아도 통보가
        누락되지는 않는다.
        """
        return self._initial_pose_received or not self.require_initial_pose

    def weak_class_only(self, payload: dict) -> Optional[str]:
        """스파크/담배만 잡혔는데 둘 다 기준 미달이면 그 사유를 돌려준다.

        게이트 대상이 아닌 클래스가 하나라도 잡혔으면 판단하지 않는다.
        아무것도 안 잡혔으면(가스/온도 단독) 역시 막지 않는다.
        """
        confs = payload.get("confs")
        if not isinstance(confs, dict):
            return None

        for name in confs:
            if (name not in self.danger_class_floor
                    and self.class_confidence(payload, name) > 0.0):
                return None

        seen = []
        for name, floor in self.danger_class_floor.items():
            value = self.class_confidence(payload, name)
            if value <= 0.0:
                continue
            if value >= floor:
                return None
            seen.append(f"{name}={value:.3f}<{floor:.2f}")

        return ", ".join(seen) if seen else None

    def approach_target(self, payload: dict, bearing_class):
        """접근 대상인지 판단하고 (클래스, confidence) 를 돌려준다.

        방위각을 뽑아낸 클래스만 후보다. 조준할 수 없는 것에는
        다가갈 수 없기 때문이다.
        """

        if bearing_class is None:
            return None, 0.0

        if bearing_class not in self.approach_classes:
            return None, self.class_confidence(payload, bearing_class)

        conf = self.class_confidence(payload, bearing_class)
        threshold = self.approach_conf.get(bearing_class, 1.0)

        if conf < threshold:
            return None, conf

        return bearing_class, conf

    @staticmethod
    def telegram_state(payload: dict) -> str:
        telegram = payload.get("telegram")

        if not isinstance(telegram, dict):
            return "unknown"

        return str(telegram.get("state", "unknown"))

    @staticmethod
    def mlp_danger(payload: dict) -> Optional[bool]:
        """원본 MLP 의 위험 판정. 불 / 담배꽁초 / 연기가 모두 여기 반영된다."""

        danger = payload.get("mlp_danger")

        return danger if isinstance(danger, bool) else None

    @staticmethod
    def alert_expected(payload: dict) -> Optional[bool]:
        """인지가 알려준 "이번 화재가 알림 대상인지". 모르면 None."""

        expected = payload.get("alert_expected")

        return expected if isinstance(expected, bool) else None

    def alert_ready(self, payload: dict) -> bool:
        """텔레그램 게이트를 통과했는지 판단한다."""

        if self.alert_gate == "none":
            return True

        state = self.telegram_state(payload)

        if self.alert_gate == "sent":
            return state == "sent"

        # attempted: 성공이든 실패든 전송을 시도했으면 통과
        return state in ("sent", "failed")

    # ---------------- 콜백 ----------------

    def _map_cb(self, msg: OccupancyGrid) -> None:
        data = np.asarray(msg.data, dtype=np.int16)

        if data.size != msg.info.width * msg.info.height:
            self.get_logger().error("/map 데이터 크기가 metadata 와 다름")
            return

        with self._data_lock:
            self._map_grid = data.reshape(
                (msg.info.height, msg.info.width)
            ).copy()
            self._map_resolution = float(msg.info.resolution)
            self._map_origin_x = float(msg.info.origin.position.x)
            self._map_origin_y = float(msg.info.origin.position.y)
            self._map_origin_yaw = yaw_from_quat(msg.info.origin.orientation)

    def send_fire_dispatch(self, cleared: bool) -> None:
        """팔로워봇에 진화 지령을 보낸다.

        cleared=False 는 "불은 여기다, 다만 내가 아직 그 앞에 있다",
        cleared=True 는 "비켰다, 와도 된다" 는 뜻이다.
        """

        if self._dispatch is None:
            return

        mission_id, x, y, yaw = self._dispatch

        payload = {
            "schema": 1,
            "mission_id": mission_id,
            "frame_id": "map",
            "x": x,
            "y": y,
            "yaw": yaw,
            "main_cleared": bool(cleared),
        }

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._dispatch_pub.publish(msg)

        self.get_logger().warn(
            f"팔로워 지령 [{mission_id}] "
            f"({x:+.3f}, {y:+.3f}) main_cleared={cleared}"
        )

    def set_fire_episode(self, active: bool) -> None:
        """화재 처리 구간의 시작/끝을 인지 쪽에 알린다.

        구간이 끝난다는 것은 메인봇이 불 앞을 떠나 순찰로 돌아간다는
        뜻이다. 그 순간이 팔로워봇에게 "이제 와도 된다" 고 알릴 때다.
        """

        msg = Bool()
        msg.data = bool(active)
        self._episode_pub.publish(msg)

        if not active and self._dispatch is not None:
            self.send_fire_dispatch(cleared=True)
            self._dispatch = None

    def report_fire_location(self, bearing: float, distance: float) -> bool:
        """HOLD 지점에서 화재 좌표를 확정해 발행하고 알림을 요청한다.

        bearing 은 base_link 기준 화재 방위각[rad], distance 는 그 방향
        전방 여유거리[m]. HOLD 진입 조건이 "전방 여유 < fire_stop_dist"
        이므로 distance 는 실측값이며 추정이 아니다.
        """

        with self._data_lock:
            pose = self._pose

        if pose is None:
            self.get_logger().warn(
                "화재 좌표를 낼 수 없다: /amcl_pose 를 아직 못 받았다"
            )
            return False

        rx = float(pose.pose.pose.position.x)
        ry = float(pose.pose.pose.position.y)
        ryaw = yaw_from_quat(pose.pose.pose.orientation)

        # map 기준 화재 방향 = 로봇 heading + 카메라 방위각
        fire_yaw = wrap(ryaw + bearing)
        fx = rx + distance * math.cos(fire_yaw)
        fy = ry + distance * math.sin(fire_yaw)

        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "map"
        target.pose.position.x = fx
        target.pose.position.y = fy
        target.pose.orientation.z = math.sin(fire_yaw / 2.0)
        target.pose.orientation.w = math.cos(fire_yaw / 2.0)

        self._fire_target_pub.publish(target)

        request = {
            "fire": {"x": round(fx, 3), "y": round(fy, 3),
                     "yaw": round(fire_yaw, 4)},
            "robot": {"x": round(rx, 3), "y": round(ry, 3),
                      "yaw": round(ryaw, 4)},
            "bearing_deg": round(math.degrees(bearing), 1),
            "distance_m": round(distance, 3),
            "stamp": time.time(),
        }

        msg = String()
        msg.data = json.dumps(request, separators=(",", ":"))
        self._alert_request_pub.publish(msg)

        # 팔로워 지령용 mission_id. 프로토콜 제약은
        #   [A-Za-z0-9][A-Za-z0-9_.:-]{0,63}
        # 이라 하이픈과 숫자만 쓴다.
        mission_id = "argos-{}".format(int(time.time() * 1000))
        self._dispatch = (mission_id, round(fx, 6), round(fy, 6),
                          round(fire_yaw, 6))

        # 아직 내가 불 앞에 있다. 팔로워는 WAIT_CLEARANCE 로 대기한다.
        self.send_fire_dispatch(cleared=False)

        self.get_logger().warn(
            f"화재 좌표 확정: map ({fx:+.2f}, {fy:+.2f})  "
            f"로봇 ({rx:+.2f}, {ry:+.2f})  "
            f"전방 {distance:.2f} m  방위 {math.degrees(bearing):+.0f}deg "
            "-> /main/fire_target 발행, 알림 요청"
        )

        return True

    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        with self._data_lock:
            self._pose = msg

    def _initial_pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        del msg

        if not self._initial_pose_received:
            self.get_logger().info(
                "2D Pose Estimate 수신. 자동 순찰을 허용한다."
            )

        self._initial_pose_received = True
        self._next_goal_after = time.monotonic() + 1.0

    def _scan_cb(self, msg: LaserScan) -> None:
        with self._data_lock:
            self._scan = msg
            self._last_scan_at = time.monotonic()

    def _nav_cmd_cb(self, msg: Twist) -> None:
        with self._control_lock:
            self._nav_cmd = clone_twist(msg)
            self._nav_cmd_at = time.monotonic()

    # ---------------- 속도 중재 ----------------

    def set_control_mode(self, mode: str) -> None:
        if mode not in ("stop", "nav", "fire"):
            raise ValueError(f"unknown control mode: {mode}")

        with self._control_lock:
            self._control_mode = mode

            if mode != "fire":
                self._fire_cmd = Twist()
                self._fire_cmd_at = time.monotonic()

        self._output_cmd_pub.publish(Twist())

    def set_fire_twist(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        with self._control_lock:
            self._fire_cmd = msg
            self._fire_cmd_at = time.monotonic()

        self._fire_cmd_pub.publish(msg)

    def scan_fresh(self) -> bool:
        with self._data_lock:
            last_scan_at = self._last_scan_at

        return (
            last_scan_at > 0.0
            and time.monotonic() - last_scan_at < self.scan_timeout
        )

    def _mux_tick(self) -> None:
        now = time.monotonic()

        with self._control_lock:
            mode = self._control_mode
            nav_cmd = clone_twist(self._nav_cmd)
            nav_age = now - self._nav_cmd_at
            fire_cmd = clone_twist(self._fire_cmd)
            fire_age = now - self._fire_cmd_at

        output = Twist()

        if mode == "fire" and fire_age <= self.fire_cmd_timeout:
            output = fire_cmd
        elif (
            mode == "nav"
            and nav_age <= self.nav_cmd_timeout
            and self.scan_fresh()
        ):
            output = nav_cmd

        self._output_cmd_pub.publish(output)

    def stop_all(self) -> None:
        self.set_control_mode("stop")

        for _ in range(15):
            self._fire_cmd_pub.publish(Twist())
            self._output_cmd_pub.publish(Twist())
            time.sleep(0.02)

    # ---------------- LiDAR ----------------

    def lookup_laser_tf(self) -> bool:
        if self._laser_yaw is not None:
            return True

        try:
            transform = self._tf_buffer.lookup_transform(
                "base_link",
                "laser_frame",
                rclpy.time.Time(),
            )
        except Exception:
            return False

        self._laser_x = float(transform.transform.translation.x)
        self._laser_yaw = yaw_from_quat(transform.transform.rotation)

        self.get_logger().info(
            "LiDAR TF 확보: "
            f"x={self._laser_x:.3f}, "
            f"yaw={math.degrees(self._laser_yaw):.1f} deg"
        )
        return True

    def front_clearance(self) -> Optional[float]:
        with self._data_lock:
            scan = self._scan

        if scan is None or self._laser_x is None or self._laser_yaw is None:
            return None

        return sector_clearance(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            self._laser_x,
            self._laser_yaw,
            0.0,
            self.front_half_angle,
        )

    # ---------------- Nav2 순찰 ----------------

    def _patrol_tick(self) -> None:
        with self._control_lock:
            if self._control_mode != "nav":
                return

        now = time.monotonic()

        if not self.scan_fresh():
            if now - self._last_scan_log > 3.0:
                self.get_logger().warn("/scan 없음: 자동 순찰 정지 유지")
                self._last_scan_log = now
            return

        if self.require_initial_pose and not self._initial_pose_received:
            if now - self._last_wait_log > 5.0:
                self.get_logger().info(
                    "RViz에서 2D Pose Estimate 를 지정하면 순찰을 시작한다."
                )
                self._last_wait_log = now
            return

        with self._goal_lock:
            goal_active = self._goal_handle is not None
            goal_pending = self._goal_pending
            goal_age = now - self._goal_sent_at

        if goal_active and goal_age > self.patrol_goal_timeout:
            self.get_logger().warn("순찰 목표 시간 초과: 취소 후 재선정")
            self.cancel_nav_goal()
            self._next_goal_after = now + self.patrol_retry_delay
            return

        if goal_active or goal_pending or now < self._next_goal_after:
            return

        if not self._navigator.wait_for_server(timeout_sec=0.0):
            if now - self._last_wait_log > 5.0:
                self.get_logger().info("NavigateToPose 서버 대기 중")
                self._last_wait_log = now
            return

        target = self._choose_patrol_target()

        if target is None:
            if now - self._last_wait_log > 5.0:
                self.get_logger().warn(
                    "현재 위치 주변에서 안전한 순찰 목표를 찾지 못함"
                )
                self._last_wait_log = now
            self._next_goal_after = now + self.patrol_retry_delay
            return

        self._send_patrol_goal(*target)

    def _choose_patrol_target(self):
        with self._data_lock:
            if self._map_grid is None or self._pose is None:
                return None

            grid = self._map_grid.copy()
            resolution = self._map_resolution
            origin_x = self._map_origin_x
            origin_y = self._map_origin_y
            robot_x = float(self._pose.pose.pose.position.x)
            robot_y = float(self._pose.pose.pose.position.y)

        return select_patrol_target(
            grid,
            resolution,
            origin_x,
            origin_y,
            robot_x,
            robot_y,
            self.patrol_min_radius,
            self.patrol_max_radius,
            self.patrol_clearance,
            self.patrol_free_max,
            self.patrol_sample_attempts,
            self._rng,
        )

    def _send_patrol_goal(self, x: float, y: float, yaw: float) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        with self._goal_lock:
            self._goal_token += 1
            token = self._goal_token
            self._goal_pending = True
            self._goal_sent_at = time.monotonic()

        future = self._navigator.send_goal_async(goal)
        future.add_done_callback(
            lambda result, goal_token=token:
            self._goal_response(result, goal_token)
        )

        self.get_logger().info(
            f"순찰 목표 전송: x={x:.2f}, y={y:.2f}, "
            f"yaw={math.degrees(yaw):+.0f} deg"
        )

    def _goal_response(self, future, token: int) -> None:
        try:
            handle = future.result()
        except Exception as error:
            self.get_logger().error(f"순찰 목표 전송 오류: {error}")

            with self._goal_lock:
                if token == self._goal_token:
                    self._goal_pending = False
                    self._next_goal_after = (
                        time.monotonic() + self.patrol_retry_delay
                    )
            return

        with self._goal_lock:
            current = token == self._goal_token

            if current:
                self._goal_pending = False

        if not current:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return

        if handle is None or not handle.accepted:
            self.get_logger().warn("순찰 목표 거부됨")

            with self._goal_lock:
                self._next_goal_after = (
                    time.monotonic() + self.patrol_retry_delay
                )
            return

        with self._goal_lock:
            self._goal_handle = handle
            self._goal_count += 1

        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda result, goal_token=token:
            self._goal_result(result, goal_token)
        )

    def _goal_result(self, future, token: int) -> None:
        try:
            status = int(future.result().status)
        except Exception as error:
            self.get_logger().error(f"순찰 결과 오류: {error}")
            status = GoalStatus.STATUS_UNKNOWN

        with self._goal_lock:
            if token != self._goal_token:
                return

            self._goal_handle = None
            self._goal_pending = False
            self._next_goal_after = (
                time.monotonic() + self.patrol_retry_delay
            )

        labels = {
            GoalStatus.STATUS_SUCCEEDED: "도착",
            GoalStatus.STATUS_CANCELED: "취소",
            GoalStatus.STATUS_ABORTED: "실패",
        }
        self.get_logger().info(
            f"순찰 목표 종료: {labels.get(status, str(status))}"
        )

    def cancel_nav_goal(self) -> None:
        with self._goal_lock:
            self._goal_token += 1
            handle = self._goal_handle
            self._goal_handle = None
            self._goal_pending = False
            self._next_goal_after = (
                time.monotonic() + self.patrol_retry_delay
            )

        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as error:
                self.get_logger().warn(f"Nav2 목표 취소 오류: {error}")


def run() -> int:
    rclpy.init()
    node = FireNavIntegrated()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    shutting_down = False

    def request_shutdown(*_args):
        nonlocal shutting_down
        shutting_down = True

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    try:
        node.get_logger().info(
            f"인지 결과 대기 중: {node.detection_topic} "
            "(fire_perception_main.py 를 먼저 띄워야 한다)"
        )

        state = "NAV_PATROL"
        confirm_started = None
        danger_confirm_started = None
        # 약한 스파크/담배로 위험 판정을 보류했다는 로그의 도배를 막는다.
        weak_log_at = 0.0
        # 자세를 못 잡아 화재 대응을 미뤘다는 로그도 마찬가지다.
        pose_wait_log_at = 0.0
        # ALERT_WAIT 가 끝난 뒤 무엇을 할지. "approach" 또는 "patrol".
        wait_then = "approach"
        danger_suppressed_until = 0.0
        # 접근하지 않고 순찰로 복귀했을 때, 같은 불에 곧바로 다시
        # 걸려 멈추는 것을 막는다. danger 쪽과 달리 화재 분기는
        # 원래 아무 억제도 없었다.
        fire_suppressed_until = 0.0
        # 이번 화재 건에서 좌표를 이미 보고했는지. HOLD 에 머무는 동안
        # 매 틱 재발행하지 않기 위한 것이다.
        hold_reported = False
        last_fire_seen = 0.0
        last_bearing = None
        last_bearing_at = 0.0
        state_started = time.monotonic()
        approach_started = 0.0
        alert_wait_started = 0.0
        alert_logged = False
        last_diag = 0.0
        last_stale_log = 0.0
        perception_started = False

        while rclpy.ok() and not shutting_down:
            time.sleep(0.02)

            now = time.monotonic()
            detection, age = node.latest_detection()

            # ---- 인지 결과가 아직 한 번도 안 왔다 ----
            if detection is None:
                if now - last_stale_log > 5.0:
                    last_stale_log = now
                    node.get_logger().warn(
                        f"{node.detection_topic} 아직 수신 없음: 정지 유지"
                    )
                continue

            if not perception_started:
                perception_started = True
                node.set_control_mode("nav")
                # 기동 시 게이트를 확실히 열어 둔다. transient_local 이라
                # 인지가 나중에 떠도 이 값을 받는다.
                node.set_fire_episode(False)
                node.get_logger().info(
                    "인지 준비 완료: Nav2 자동 순찰을 허용한다 "
                    f"(접근 기준 fire >= {node.fire_conf:.2f})"
                )

            fresh = node.detection_fresh(age)

            # ---- 인지가 끊겼다 ----
            if not fresh:
                if now - last_stale_log > 2.0:
                    last_stale_log = now
                    node.get_logger().warn(
                        f"인지 결과가 {age:.1f}s 동안 갱신되지 않음"
                    )

                if state != "NAV_PATROL":
                    # 눈 없이 불 쪽으로 움직이지 않는다.
                    node.set_fire_twist(0.0, 0.0)
                # NAV_PATROL 중이면 Nav2 순찰은 계속한다.
                # 인지가 죽었다고 순찰까지 멈추면 로봇이 굳어버리고,
                # 장애물 회피는 Nav2 costmap 이 계속 담당한다.

            bearing, bearing_class = node.bearing_from(detection)

            if bearing is not None:
                last_bearing = bearing
                last_bearing_at = now
            elif (
                last_bearing is not None
                and now - last_bearing_at <= node.bearing_hold_seconds
            ):
                bearing = last_bearing
            else:
                last_bearing = None

            # 접근은 반드시 화면에 실제 bbox 가 있어야 한다.
            # MLP 확률만으로는 움직이지 않는다 (recall 이 낮다).
            #
            # 대상은 fire 하나가 아니라 approach_classes 에 있는 클래스다.
            # 방위각을 뽑아낸 그 클래스가 접근 대상인지 본다.
            hazard_class, fire_confidence = node.approach_target(
                detection, bearing_class
            )
            raw_fire = fresh and hazard_class is not None

            if raw_fire:
                if confirm_started is None:
                    confirm_started = now
                confirmed = now - confirm_started >= node.confirm_seconds
            else:
                confirm_started = None
                confirmed = False

            if raw_fire and bearing is not None:
                last_fire_seen = now

            # 위험상황 판단. 원본 MLP 는 불 / 연기 / 담배꽁초 / 스파크를
            # 하나의 확률로 합쳐서 내므로 그 판정을 그대로 쓴다.
            # 아두이노가 끊기면 MLP 가 판정을 못 하니 fire 조건을 OR 로 남긴다.
            danger_now = fresh and (
                node.mlp_danger(detection) is True or raw_fire
            )

            # 약한 스파크 / 담배꽁초만으로는 위험으로 확정하지 않는다.
            # MLP 는 네 클래스를 한 확률로 합치므로 여기서 따로 걸러야 한다.
            if danger_now:
                weak = node.weak_class_only(detection)
                if weak is not None:
                    danger_now = False
                    if now - weak_log_at > 5.0:
                        node.get_logger().info(
                            f"신뢰도 미달로 위험 판정 보류 ({weak})"
                        )
                        weak_log_at = now

            if danger_now:
                if danger_confirm_started is None:
                    danger_confirm_started = now
                danger_confirmed = (
                    now - danger_confirm_started >= node.confirm_seconds
                )
            else:
                danger_confirm_started = None
                danger_confirmed = False

            clear_m = node.front_clearance()
            scan_ok = node.scan_fresh() and node.lookup_laser_tf()

            # 자세가 없으면 화재 좌표를 만들 수 없다.  왜 가만히 있는지
            # 알 수 있게 남긴다.  알림 자체는 인지 노드가 따로 낸다.
            if (confirmed and bearing is not None
                    and not node.pose_ready()
                    and now - pose_wait_log_at > 5.0):
                node.get_logger().warn(
                    "불을 봤지만 위치추정 전이라 대응을 미룬다 "
                    "(2D Pose Estimate 를 먼저 받아야 좌표가 맞다)"
                )
                pose_wait_log_at = now

            # ---------------- 상태기계 ----------------

            if state == "NAV_PATROL":
                if (
                    confirmed
                    and bearing is not None
                    and now >= fire_suppressed_until
                    and node.pose_ready()
                ):
                    node.set_control_mode("fire")
                    node.set_fire_twist(0.0, 0.0)
                    node.cancel_nav_goal()

                    wait_then = (
                        "approach" if node.approach_on_fire else "patrol"
                    )

                    hold_reported = False
                    node.set_fire_episode(True)
                    state = "FIRE_STOP"
                    state_started = now

                    node.get_logger().warn(
                        f"위험 확정 [{hazard_class}]: Nav2 목표 취소, "
                        f"제어권 획득 (알림 후 {wait_then})"
                    )

                elif (
                    danger_confirmed
                    and node.danger_stop_enabled
                    and now >= danger_suppressed_until
                ):
                    # 불은 아니지만 위험이다 (담배꽁초 / 연기 등).
                    # 멈춰서 알림이 나가게 하고, 접근은 하지 않는다.
                    node.set_control_mode("fire")
                    node.set_fire_twist(0.0, 0.0)
                    node.cancel_nav_goal()
                    wait_then = "patrol"
                    node.set_fire_episode(True)
                    state = "FIRE_STOP"
                    state_started = now
                    node.get_logger().warn(
                        "위험상황 감지(불 아님): 정지 후 알림 전송 대기"
                    )

            elif not scan_ok:
                node.set_fire_twist(0.0, 0.0)

                if now - last_diag > 2.0:
                    node.get_logger().error(
                        "화재 접근 중 /scan 또는 LiDAR TF 없음: 정지"
                    )
                    last_diag = now

            elif (
                # 이 검사는 "불 쪽으로 가는 중" 일 때만 의미가 있다.
                # 불이 아닌 위험(wait_then == "patrol")은 애초에 방향을
                # 쫓지 않으므로 last_fire_seen 이 갱신되지 않는다.
                # 여기서 걸러내지 않으면 정지와 순찰 복귀를 무한 반복한다.
                wait_then == "approach"
                and state not in ("HOLD", "ALERT_WAIT")
                and now - last_fire_seen > node.lost_seconds
            ):
                node.set_fire_twist(0.0, 0.0)
                node.set_control_mode("nav")
                node.set_fire_episode(False)
                state = "NAV_PATROL"
                node.get_logger().warn(
                    "화재 방향을 잃음: 정지 후 Nav2 순찰 복귀"
                )

            elif state == "FIRE_STOP":
                node.set_fire_twist(0.0, 0.0)

                if now - state_started >= node.fire_stop_pause:
                    if node.alert_gate == "none":
                        if wait_then == "approach":
                            state = "ALIGN"
                            state_started = now
                            node.get_logger().info("화재 방향 정렬 시작")
                        else:
                            node.set_control_mode("nav")
                            node.set_fire_episode(False)
                            state = "NAV_PATROL"
                            danger_suppressed_until = (
                                now + node.danger_stop_cooldown
                            )
                            node.get_logger().info(
                                "위험 확인 완료: 접근 없이 순찰 재개"
                            )
                    else:
                        state = "ALERT_WAIT"
                        state_started = now
                        alert_wait_started = now
                        alert_logged = False
                        node.get_logger().info(
                            "텔레그램 전송 대기 "
                            f"(gate={node.alert_gate}, "
                            f"제한 {node.alert_wait_timeout:.0f}s)"
                        )

            elif state == "ALERT_WAIT":
                # 멈춰 있어야 사진이 흔들리지 않는다.
                node.set_fire_twist(0.0, 0.0)

                waited = now - alert_wait_started
                telegram_state = node.telegram_state(detection)
                release = None

                if node.alert_ready(detection):
                    release = f"텔레그램 {telegram_state} ({waited:.1f}s 대기)"

                elif waited >= node.alert_wait_timeout:
                    release = f"텔레그램 대기 시간 초과 (상태={telegram_state})"

                elif (
                    node.alert_expected(detection) is False
                    and telegram_state not in ("queued", "sending")
                    and waited >= node.alert_skip_grace
                ):
                    # 원본이 애초에 알림을 보내지 않을 조건이다.
                    # (MLP 미달 / 센서 끊김 / 쿨다운)
                    # 나오지 않을 알림을 기다리며 서 있을 이유가 없다.
                    release = "이번 위험은 텔레그램 대상이 아님"

                elif not alert_logged and waited > 2.0:
                    alert_logged = True
                    node.get_logger().info(
                        f"텔레그램 상태={telegram_state} — 계속 대기"
                    )

                if release is not None:
                    if wait_then == "approach":
                        state = "ALIGN"
                        state_started = now
                        node.get_logger().info(
                            f"{release}: 화재 방향 정렬 시작"
                        )
                    else:
                        # 불이 아닌 위험은 접근하지 않는다.
                        # 알림이 나갔으면 그걸로 할 일은 끝이다.
                        node.set_control_mode("nav")
                        node.set_fire_episode(False)
                        state = "NAV_PATROL"
                        danger_suppressed_until = (
                            now + node.danger_stop_cooldown
                        )
                        fire_suppressed_until = (
                            now + node.danger_stop_cooldown
                        )
                        node.get_logger().info(
                            f"{release}: 접근 없이 순찰 재개 "
                            f"({node.danger_stop_cooldown:.0f}s 동안 "
                            "같은 위험으로 다시 멈추지 않는다)"
                        )

            elif state == "ALIGN":
                if bearing is None:
                    node.set_fire_twist(0.0, 0.0)
                elif abs(bearing) <= node.align_tolerance:
                    node.set_fire_twist(0.0, 0.0)
                    state = "APPROACH"
                    approach_started = now
                    node.get_logger().info("화재 방향 정렬 완료, 접근 시작")
                else:
                    angular = max(
                        -node.align_w_max,
                        min(node.align_w_max, node.turn_kp * bearing),
                    )
                    node.set_fire_twist(0.0, angular)

            elif state == "APPROACH":
                if bearing is None:
                    node.set_fire_twist(0.0, 0.0)
                elif clear_m is not None and clear_m < node.fire_stop_dist:
                    node.set_fire_twist(0.0, 0.0)
                    state = "HOLD"
                    state_started = now
                    hold_reported = node.report_fire_location(
                        bearing if bearing is not None else 0.0, clear_m
                    )
                    node.get_logger().warn(
                        f"화재 접근 완료: 전방 {clear_m:.2f} m, 정지 유지"
                    )
                elif now - approach_started > node.approach_max_seconds:
                    node.set_fire_twist(0.0, 0.0)
                    state = "HOLD"
                    state_started = now
                    # 상한으로 멈춘 경우라 거리가 fire_stop_dist 라는 보장이
                    # 없다. 실측 전방거리를 쓰고, 그것도 없으면 보고하지 않는다.
                    hold_reported = (
                        node.report_fire_location(
                            bearing if bearing is not None else 0.0, clear_m
                        )
                        if clear_m is not None
                        else False
                    )
                    node.get_logger().warn(
                        "화재 접근 시간 상한 도달: 정지 유지"
                    )
                elif abs(bearing) > node.realign_angle:
                    node.set_fire_twist(0.0, 0.0)
                    state = "ALIGN"
                    state_started = now
                    node.get_logger().info("화재가 화면 가장자리: 재정렬")
                else:
                    angular = max(
                        -node.approach_w_max,
                        min(node.approach_w_max, node.turn_kp * bearing),
                    )
                    node.set_fire_twist(node.approach_v, angular)

            elif state == "HOLD":
                # 여기가 "메인봇이 불 근처에 도달한 지점"이다.
                # 팔로워봇은 이 좌표로 와서 물을 뿌린다.
                # 그래서 알릴 것을 알렸으면 반드시 비켜야 한다.
                node.set_fire_twist(0.0, 0.0)

                held = now - state_started
                tg_state = node.telegram_state(detection)

                # 진입 시 pose 가 없어 보고를 못 했으면 여기서 다시 시도한다.
                if not hold_reported and clear_m is not None:
                    hold_reported = node.report_fire_location(
                        bearing if bearing is not None else 0.0, clear_m
                    )

                if node.alert_ready(detection):
                    # 전송이 확인됐다. 기다릴 이유가 없다.
                    node.set_control_mode("nav")
                    node.set_fire_episode(False)
                    state = "NAV_PATROL"
                    fire_suppressed_until = now + node.danger_stop_cooldown
                    node.get_logger().info(
                        f"텔레그램 {tg_state} 확인 ({held:.1f}s): "
                        "즉시 순찰 복귀, 팔로워봇 진화 공간 확보"
                    )

                elif now - last_fire_seen > node.hold_release_seconds:
                    node.set_control_mode("nav")
                    node.set_fire_episode(False)
                    state = "NAV_PATROL"
                    node.get_logger().info(
                        "화재가 사라진 상태 유지: Nav2 순찰 복귀"
                    )

                elif held >= node.hold_max_seconds:
                    # 전송 확인을 못 받았어도 자리를 뜬다.
                    #
                    # 원본은 전송 실패 시 재시도하지 않는다.
                    # ALERT_COOLDOWN_SECONDS = 60 이고, 쿨다운이 "성공"이
                    # 아니라 "큐에 넣은 시점"부터 시작하기 때문이다.
                    # 그래서 여기서 더 기다려 봐야 60초 전에는 아무 일도
                    # 일어나지 않는다. 물줄기 앞을 비우는 쪽이 우선이다.
                    node.set_control_mode("nav")
                    node.set_fire_episode(False)
                    state = "NAV_PATROL"
                    fire_suppressed_until = now + node.danger_stop_cooldown
                    node.get_logger().warn(
                        f"HOLD {node.hold_max_seconds:.0f}s 경과 "
                        f"(텔레그램={tg_state}): 전송 확인 없이 순찰 복귀. "
                        "원본은 60s 쿨다운이라 즉시 재시도하지 않는다."
                    )

            # ---------------- 진단 ----------------

            if now - last_diag > 5.0:
                last_diag = now

                clear_text = "open" if clear_m is None else f"{clear_m:.2f}m"
                bearing_text = (
                    "none"
                    if bearing is None
                    else f"{math.degrees(bearing):+.0f}deg"
                )
                source = bearing_class or "-"

                node.get_logger().info(
                    f"state={state} scan={'OK' if scan_ok else 'X'} "
                    f"det={'OK' if fresh else f'{age:.1f}s'} "
                    f"front={clear_text} "
                    f"hazard={hazard_class or '-'}({fire_confidence:.2f}) "
                    f"bearing={bearing_text}({source}) "
                    f"danger={node.mlp_danger(detection)} "
                    f"tg={node.telegram_state(detection)}"
                    f"/{node.alert_expected(detection)} "
                    f"patrol_goals={node._goal_count}"
                )

        return 0

    except Exception as error:
        node.get_logger().error(f"화재 순찰 종료 오류: {error!r}")
        return 1

    finally:
        node.cancel_nav_goal()
        node.stop_all()

        try:
            executor.shutdown()
        except Exception:
            pass

        spin_thread.join(timeout=2.0)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(run())
