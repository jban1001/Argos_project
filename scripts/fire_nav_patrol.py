#!/usr/bin/env python3

"""ARGOS Nav2 자동 순찰과 화재 접근을 하나의 안전 제어기로 통합한다.

평상시에는 Nav2가 /cmd_vel_nav_auto 를 내고, 이 노드가 /cmd_vel_nav 으로
전달한다. 화재가 확정되면 즉시 Nav2 목표를 취소하고 화재 접근 명령만
/cmd_vel_nav 으로 전달한다. velocity_smoother 와 base driver watchdog 은
기존 경로 그대로 유지된다.

YOLO 원본은 수정하지 않는다. 모델과 텔레그램 모듈은
~/YOLO/YOLO_BACK 안의 복사본을 사용한다.
"""

from __future__ import annotations

import importlib.util
import math
import os
import random
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import cv2
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

    if not _disk_is_free(
        grid, start_gx, start_gy, radius_cells, free_max
    ):
        return None

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


def danger_active(
    detection,
    gate: str,
    mlp_threshold: float,
    yolo_threshold: float,
) -> bool:
    """new_main 기준 MLP gate 또는 시연용 YOLO gate를 판정한다."""
    if gate == "mlp":
        return (
            bool(detection.get("sensor_ok"))
            and float(detection.get("prob", 0.0)) >= mlp_threshold
        )
    if gate == "yolo":
        return (
            float(detection.get("confs", {}).get("fire", 0.0))
            >= yolo_threshold
        )
    raise ValueError("gate 는 'mlp' 또는 'yolo' 여야 합니다")


class FireNavPatrol(Node):

    def __init__(self) -> None:
        super().__init__("fire_nav_patrol")

        self._declare_parameters()
        self._read_parameters()

        self._data_lock = threading.Lock()
        self._control_lock = threading.Lock()
        self._goal_lock = threading.Lock()

        self._map_grid: Optional[np.ndarray] = None
        self._map_resolution = 0.0
        self._map_origin_x = 0.0
        self._map_origin_y = 0.0
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
            "YOLO 준비 후 Nav2 순찰 허용"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "detector_script": str(
                Path.home() / "argos_project" / "scripts" / "fire_seeker.py"
            ),
            "yolo_backup_dir": str(Path.home() / "YOLO" / "YOLO_BACK"),
            "fire_conf": 0.20,
            "confirm_seconds": 1.0,
            "bearing_hold_seconds": 1.2,
            "lost_seconds": 2.0,
            "hold_release_seconds": 5.0,
            "show_camera": True,
            "telegram_enabled": True,
            "telegram_alert_script": str(
                Path.home() / "YOLO" / "YOLO_BACK" / "telegram_alert.py"
            ),
            "telegram_gate": "mlp",
            "telegram_fire_probability": 0.70,
            "telegram_yolo_conf": 0.20,
            "telegram_confirm_seconds": 1.0,
            "telegram_cooldown_seconds": 60.0,
            "telegram_send_photo": True,
            "require_initial_pose": True,
            "patrol_min_radius": 0.8,
            "patrol_max_radius": 2.5,
            "patrol_clearance": 0.35,
            "patrol_goal_timeout": 75.0,
            "patrol_retry_delay": 1.5,
            "patrol_sample_attempts": 120,
            "patrol_free_max": 10,
            "fire_stop_pause": 0.7,
            "align_tolerance_deg": 8.0,
            "realign_deg": 26.0,
            "turn_kp": 1.2,
            "align_w_max": 0.35,
            "approach_v": 0.08,
            "approach_w_max": 0.25,
            "fire_stop_dist": 0.60,
            "approach_max_seconds": 25.0,
            "scan_timeout": 1.5,
            "front_half_angle_deg": 25.0,
            "nav_cmd_timeout": 0.50,
            "fire_cmd_timeout": 0.30,
            "mux_rate": 20.0,
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

        self.detector_script = str(value("detector_script"))
        self.yolo_backup_dir = str(value("yolo_backup_dir"))
        self.fire_conf = float(value("fire_conf"))
        self.confirm_seconds = float(value("confirm_seconds"))
        self.bearing_hold_seconds = float(value("bearing_hold_seconds"))
        self.lost_seconds = float(value("lost_seconds"))
        self.hold_release_seconds = float(value("hold_release_seconds"))
        self.show_camera = bool(value("show_camera"))
        self.telegram_enabled = bool(value("telegram_enabled"))
        self.telegram_alert_script = str(value("telegram_alert_script"))
        self.telegram_gate = str(value("telegram_gate")).strip().lower()
        if self.telegram_gate not in {"mlp", "yolo"}:
            raise ValueError("telegram_gate 는 'mlp' 또는 'yolo' 여야 합니다")
        self.telegram_fire_probability = float(
            value("telegram_fire_probability")
        )
        self.telegram_yolo_conf = float(value("telegram_yolo_conf"))
        self.telegram_confirm_seconds = float(
            value("telegram_confirm_seconds")
        )
        self.telegram_cooldown_seconds = float(
            value("telegram_cooldown_seconds")
        )
        self.telegram_send_photo = bool(value("telegram_send_photo"))
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

    def map_position(self) -> Optional[tuple[float, float]]:
        with self._data_lock:
            if self._pose is None:
                return None
            point = self._pose.pose.pose.position
            return float(point.x), float(point.y)

    # ---------------- 입력 ----------------

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

        if (
            scan is None
            or self._laser_x is None
            or self._laser_yaw is None
        ):
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
                self.get_logger().warn(
                    "/scan 없음: 자동 순찰 정지 유지"
                )
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


def load_detector(node: FireNavPatrol):
    script_path = Path(node.detector_script).expanduser()

    if not script_path.is_file():
        raise FileNotFoundError(f"detector script 없음: {script_path}")

    spec = importlib.util.spec_from_file_location(
        "argos_fire_seeker_detector",
        str(script_path),
    )

    if spec is None or spec.loader is None:
        raise RuntimeError(f"detector script 로드 실패: {script_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    # 원본 ~/YOLO 파일이 아니라 사용자가 지정한 YOLO_BACK 복사본만 읽는다.
    backup_dir = Path(node.yolo_backup_dir).expanduser()
    module.MODEL_PATH = str(backup_dir / "best.engine")
    module.MLP_MODEL_PATH = str(backup_dir / "fire_mlp.pkl")

    # 접근은 아래 주행 루프에서 YOLO confidence로 별도 판단한다.
    # 텔레그램이 MLP gate이면 센서값과 MLP 확률도 함께 계산한다.
    module.REQUIRE_SENSOR_GATE = (
        node.telegram_enabled and node.telegram_gate == "mlp"
    )
    module.YOLO_ONLY_FIRE_CONF = node.fire_conf
    module.SHOW_WINDOW = False

    detector = module.FireDetector()
    return module, detector


def load_telegram_alerter(node: FireNavPatrol):
    script_path = Path(node.telegram_alert_script).expanduser()
    if not script_path.is_file():
        raise FileNotFoundError(f"텔레그램 모듈 없음: {script_path}")

    spec = importlib.util.spec_from_file_location(
        "argos_telegram_alert",
        str(script_path),
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"텔레그램 모듈 로드 실패: {script_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    alerter = module.TelegramAlerter(
        enabled=node.telegram_enabled,
        cooldown_seconds=node.telegram_cooldown_seconds,
        send_photo=node.telegram_send_photo,
        save_dir=Path(node.yolo_backup_dir) / "fire_alerts",
        logger=lambda message: node.get_logger().info(message),
    )
    return module, alerter


def run() -> int:
    rclpy.init()
    node = FireNavPatrol()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    detector = None
    detector_module = None
    alerter = None
    alerter_module = None
    sensor_stop_event = None
    sensor_thread = None
    show = False
    shutting_down = False

    def request_shutdown(*_args):
        nonlocal shutting_down
        shutting_down = True

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    try:
        detector_module, detector = load_detector(node)

        if node.telegram_enabled and node.telegram_gate == "mlp":
            sensor_stop_event = threading.Event()
            sensor_thread = threading.Thread(
                target=detector_module.read_arduino,
                args=(sensor_stop_event,),
                daemon=True,
            )
            sensor_thread.start()

        if node.telegram_enabled:
            alerter_module, alerter = load_telegram_alerter(node)

        show = node.show_camera and bool(os.environ.get("DISPLAY"))

        node.get_logger().info(
            f"YOLO 준비 완료 (fire confidence >= {node.fire_conf:.2f})"
        )
        node.set_control_mode("nav")

        state = "NAV_PATROL"
        confirm_started = None
        alert_started = None
        last_fire_seen = 0.0
        last_bearing = None
        last_bearing_at = 0.0
        state_started = time.monotonic()
        approach_started = 0.0
        last_diag = 0.0

        while rclpy.ok() and not shutting_down:
            detection = detector.step()

            if not detection.get("ok"):
                node.set_fire_twist(0.0, 0.0)
                time.sleep(0.05)
                continue

            now = time.monotonic()
            bearing = detection.get("bearing")
            bearing_fresh = bearing is not None

            if bearing_fresh:
                last_bearing = bearing
                last_bearing_at = now
            elif (
                last_bearing is not None
                and now - last_bearing_at <= node.bearing_hold_seconds
            ):
                bearing = last_bearing
            else:
                last_bearing = None

            # 로봇 접근은 반드시 화면에 실제 fire bbox가 있어야 한다.
            # 미완성 MLP의 단독 오탐으로 로봇이 움직이지 않도록 분리한다.
            raw_fire = (
                float(detection.get("confs", {}).get("fire", 0.0))
                >= node.fire_conf
            )

            if raw_fire:
                if confirm_started is None:
                    confirm_started = now
                confirmed = now - confirm_started >= node.confirm_seconds
            else:
                confirm_started = None
                confirmed = False

            if raw_fire and bearing is not None:
                last_fire_seen = now

            clear_m = node.front_clearance()
            scan_ok = node.scan_fresh() and node.lookup_laser_tf()

            alert_raw = danger_active(
                detection,
                node.telegram_gate,
                node.telegram_fire_probability,
                node.telegram_yolo_conf,
            )

            if node.telegram_gate == "mlp":
                alert_threshold = node.telegram_fire_probability
                alert_probability = float(detection.get("prob", 0.0))
            else:
                alert_threshold = node.telegram_yolo_conf
                alert_probability = float(
                    detection.get("confs", {}).get("fire", 0.0)
                )

            if alert_raw:
                if alert_started is None:
                    alert_started = now
                alert_confirmed = (
                    now - alert_started >= node.telegram_confirm_seconds
                )
            else:
                alert_started = None
                alert_confirmed = False

            if (
                alert_confirmed
                and alerter is not None
                and alerter.ready(now)
            ):
                alert_frame = detector_module.draw_hud(
                    detection,
                    state,
                    clear_m,
                    scan_ok,
                )
                message = alerter_module.build_message(
                    detection,
                    gate=node.telegram_gate,
                    threshold=alert_threshold,
                    robot_state=state,
                    front_clearance=clear_m,
                    pose=node.map_position(),
                )
                if alerter.enqueue(
                    text=message,
                    frame=alert_frame,
                    probability=alert_probability,
                ):
                    # new_main.py와 같이 전송 후 연속 확인 시간을 다시 잰다.
                    alert_started = None

            if state == "NAV_PATROL":
                if confirmed and bearing is not None:
                    node.set_control_mode("fire")
                    node.set_fire_twist(0.0, 0.0)
                    node.cancel_nav_goal()
                    state = "FIRE_STOP"
                    state_started = now
                    node.get_logger().warn(
                        "화재 확정: Nav2 목표 취소, 화재 접근 제어권 획득"
                    )

            else:
                if not scan_ok:
                    node.set_fire_twist(0.0, 0.0)

                    if now - last_diag > 2.0:
                        node.get_logger().error(
                            "화재 접근 중 /scan 또는 LiDAR TF 없음: 정지"
                        )
                        last_diag = now

                elif state != "HOLD" and now - last_fire_seen > node.lost_seconds:
                    node.set_fire_twist(0.0, 0.0)
                    node.set_control_mode("nav")
                    state = "NAV_PATROL"
                    node.get_logger().warn(
                        "화재 방향을 잃음: 정지 후 Nav2 순찰 복귀"
                    )

                elif state == "FIRE_STOP":
                    node.set_fire_twist(0.0, 0.0)

                    if now - state_started >= node.fire_stop_pause:
                        state = "ALIGN"
                        state_started = now
                        node.get_logger().info("화재 방향 정렬 시작")

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
                        node.get_logger().warn(
                            f"화재 접근 완료: 전방 {clear_m:.2f} m, 정지 유지"
                        )
                    elif now - approach_started > node.approach_max_seconds:
                        node.set_fire_twist(0.0, 0.0)
                        state = "HOLD"
                        state_started = now
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
                    node.set_fire_twist(0.0, 0.0)

                    if now - last_fire_seen > node.hold_release_seconds:
                        node.set_control_mode("nav")
                        state = "NAV_PATROL"
                        node.get_logger().info(
                            "화재가 사라진 상태 유지: Nav2 순찰 복귀"
                        )

            if now - last_diag > 5.0:
                clear_text = "open" if clear_m is None else f"{clear_m:.2f}m"
                bearing_text = (
                    "none"
                    if bearing is None
                    else f"{math.degrees(bearing):+.0f}deg"
                )
                node.get_logger().info(
                    f"state={state} scan={'OK' if scan_ok else 'X'} "
                    f"front={clear_text} fire={detection['confs']['fire']:.2f} "
                    f"bearing={bearing_text} patrol_goals={node._goal_count}"
                )
                last_diag = now

            if show:
                frame = detector_module.draw_hud(
                    detection,
                    state,
                    clear_m,
                    scan_ok,
                )
                cv2.imshow("ARGOS fire patrol", frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            time.sleep(0.005)

        return 0

    except Exception as error:
        node.get_logger().error(f"화재 순찰 종료 오류: {error!r}")
        return 1

    finally:
        node.cancel_nav_goal()
        node.stop_all()

        if detector is not None:
            detector.release()

        if sensor_stop_event is not None:
            sensor_stop_event.set()
        if sensor_thread is not None:
            sensor_thread.join(timeout=2.0)

        if alerter is not None:
            alerter.close()

        if show:
            cv2.destroyAllWindows()

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
