#!/usr/bin/env python3
"""Arbitrate normal following and a fail-closed fire-response mission."""

from __future__ import annotations

import json
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener

from .mission import (MissionConfig, MissionController, MissionInputs,
                      MissionState)
from .nav2_driver import Nav2Driver
from .mode_protocol import (MODES, allows_dispatch, allows_follow,
                            parse_mode_request)
from .protocol import parse_cancel, parse_dispatch


class FireSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("fire_supervisor")
        self.declare_parameter("enable_motion", False)
        # "nav2"  주행을 Nav2 에 맡긴다.  장애물이 있는 실내에서는 이쪽이다.
        # "point" mission.py 의 내장 점 제어기(직선).  장애물이 없을 때만 쓴다.
        self.declare_parameter("navigator", "nav2")
        # 앞에 슬래시를 붙여 전역으로 둔다.  이 노드는 /follower 네임스페이스에
        # 있어서 상대 이름 "navigate_to_pose" 는 /follower/navigate_to_pose 로
        # 풀리는데, Nav2 의 bt_navigator 는 전역 /navigate_to_pose 로 낸다.
        self.declare_parameter("nav2_action", "/navigate_to_pose")
        self.declare_parameter("enable_pump", False)
        # auto preserves the field-tested behavior: follow while idle and accept
        # fire dispatches. Other modes deliberately remove one or both abilities.
        self.declare_parameter("initial_mode", "auto")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "follower_base_link")
        self.declare_parameter("scan_topic", "/follower/scan")
        self.declare_parameter("front_obstacle_distance_m", 0.45)
        self.declare_parameter("front_obstacle_half_angle_deg", 30.0)
        self.declare_parameter("follow_command_max_age_s", 0.2)
        self.declare_parameter("arrival_radius_m", 0.18)
        self.declare_parameter("yaw_tolerance_deg", 8.0)
        self.declare_parameter("pivot_above_deg", 20.0)
        self.declare_parameter("max_throttle", 150)
        self.declare_parameter("min_throttle", 140)
        self.declare_parameter("throttle_gain_per_m", 60.0)
        self.declare_parameter("max_yaw_rate_dps", 12.0)
        self.declare_parameter("steering_gain_dps_per_deg", 0.5)
        self.declare_parameter("pose_max_age_s", 0.5)
        self.declare_parameter("localization_grace_s", 1.0)
        self.declare_parameter("telemetry_max_age_s", 0.3)
        self.declare_parameter("settle_duration_s", 1.0)
        self.declare_parameter("obstacle_timeout_s", 10.0)
        self.declare_parameter("mission_timeout_s", 120.0)
        self.declare_parameter("spray_duration_s", 3.0)
        self.declare_parameter("max_spray_duration_s", 10.0)
        self.declare_parameter("pump_feedback_timeout_s", 0.6)
        # 임무를 마치면 지령받은 자리로 돌아간다.
        self.declare_parameter("return_home", True)
        self.declare_parameter("home_arrival_radius_m", 0.30)
        self.declare_parameter("return_timeout_s", 120.0)

        def value(name: str):
            return self.get_parameter(name).value

        self._enable_motion = bool(value("enable_motion"))
        self._enable_pump = bool(value("enable_pump"))
        self._mode = str(value("initial_mode")).lower()
        if self._mode not in MODES:
            raise ValueError(f"initial_mode must be one of {', '.join(MODES)}")
        self._mode_request_id: str | None = None
        spray_duration = float(value("spray_duration_s"))
        max_spray_duration = float(value("max_spray_duration_s"))
        if spray_duration <= 0.0 or spray_duration > max_spray_duration:
            raise ValueError(
                f"spray_duration_s must be in (0, {max_spray_duration:g}]")
        min_throttle, max_throttle = (
            int(value("min_throttle")), int(value("max_throttle")))
        if min_throttle < 0 or max_throttle < min_throttle or max_throttle > 180:
            raise ValueError("require 0 <= min_throttle <= max_throttle <= 180")

        self._controller = MissionController(MissionConfig(
            arrival_radius_m=float(value("arrival_radius_m")),
            yaw_tolerance_deg=float(value("yaw_tolerance_deg")),
            pivot_above_deg=float(value("pivot_above_deg")),
            max_throttle=max_throttle,
            min_throttle=min_throttle,
            throttle_gain_per_m=float(value("throttle_gain_per_m")),
            max_yaw_rate_dps=float(value("max_yaw_rate_dps")),
            steering_gain_dps_per_deg=float(value("steering_gain_dps_per_deg")),
            pose_max_age_s=float(value("pose_max_age_s")),
            localization_grace_s=float(value("localization_grace_s")),
            telemetry_max_age_s=float(value("telemetry_max_age_s")),
            settle_duration_s=float(value("settle_duration_s")),
            obstacle_timeout_s=float(value("obstacle_timeout_s")),
            mission_timeout_s=float(value("mission_timeout_s")),
            spray_duration_s=spray_duration,
            pump_feedback_timeout_s=float(value("pump_feedback_timeout_s")),
            pump_enabled=self._enable_pump,
            return_home=bool(value("return_home")),
            home_arrival_radius_m=float(value("home_arrival_radius_m")),
            return_timeout_s=float(value("return_timeout_s")),
        ))
        self._map_frame = str(value("map_frame"))
        self._base_frame = str(value("base_frame"))
        self._obstacle_distance = float(value("front_obstacle_distance_m"))
        self._obstacle_half_angle = math.radians(
            float(value("front_obstacle_half_angle_deg")))
        self._follow_max_age = float(value("follow_command_max_age_s"))

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._follow_command = "S"
        self._follow_command_stamp: float | None = None
        self._obstacle = False
        self._telemetry_stamp: float | None = None
        self._left_pwm = 0
        self._right_pwm = 0
        self._pump_feedback: bool | None = None

        self._navigator = str(self.get_parameter("navigator").value).lower()
        if self._navigator not in ("nav2", "point"):
            self.get_logger().warn(
                f"navigator '{self._navigator}' 를 모른다. point 로 둔다.")
            self._navigator = "point"
        self._nav2 = (Nav2Driver(self, str(self.get_parameter("nav2_action").value))
                      if self._navigator == "nav2" else None)
        if self._nav2 is not None and not self._nav2.available:
            self.get_logger().error(
                "navigator=nav2 인데 nav2_msgs 가 없다. 주행 명령을 내지 않는다.")

        self._motor_pub = self.create_publisher(String, "motor_command", 10)
        self._pump_pub = self.create_publisher(String, "pump_command", 10)
        self._status_pub = self.create_publisher(String, "fire/status", 10)
        mode_status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._mode_status_pub = self.create_publisher(
            String, "mode/status", mode_status_qos)
        self.create_subscription(String, "follow/state", self._on_follow_state, 10)
        self.create_subscription(String, "mode/set", self._on_mode_set, 10)
        self.create_subscription(String, "/fire/dispatch", self._on_dispatch, 10)
        self.create_subscription(String, "/fire/cancel", self._on_cancel, 10)
        self.create_subscription(Empty, "/fire/reset", self._on_reset, 10)
        self.create_subscription(String, "mcu/telemetry", self._on_telemetry, 10)
        self.create_subscription(
            LaserScan, str(value("scan_topic")), self._on_scan,
            qos_profile_sensor_data)
        self.create_timer(1.0 / float(value("control_rate_hz")), self._tick)
        self.create_timer(1.0, self._publish_mode_status)

        self.get_logger().info(
            "fire supervisor ready: mode=%s motion=%s pump=%s (volatile dispatch)" %
            (self._mode, "ENABLED" if self._enable_motion else "dry-run",
             "ENABLED" if self._enable_pump else "dry-run"))
        self._publish_mode_status()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_follow_state(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            command = payload["command"]
        except (json.JSONDecodeError, KeyError, TypeError):
            return
        if isinstance(command, str) and 0 < len(command) <= 32:
            self._follow_command = command
            self._follow_command_stamp = self._now()

    def _publish_mode_status(self) -> None:
        status = {
            "schema": 1,
            "request_id": self._mode_request_id,
            "mode": self._mode,
            "follow_allowed": allows_follow(self._mode),
            "dispatch_allowed": allows_dispatch(self._mode),
            "mission_id": (None if self._controller.dispatch is None
                           else self._controller.dispatch.mission_id),
            "mission_state": self._controller.state.value,
            "motion_enabled": self._enable_motion,
            "pump_enabled": self._enable_pump,
        }
        self._publish(
            self._mode_status_pub, json.dumps(status, separators=(",", ":")))

    def _stop_for_mode_change(self) -> None:
        if self._nav2 is not None and self._nav2.active:
            self._nav2.cancel("mode change")
        if self._controller.dispatch is not None:
            if self._controller.state not in (
                    MissionState.IDLE, MissionState.COMPLETE, MissionState.FAILED):
                self._controller.cancel(self._controller.dispatch.mission_id)
            if self._controller.state in (MissionState.COMPLETE, MissionState.FAILED):
                self._controller.reset()
        self._follow_command = "S"
        self._follow_command_stamp = None
        if self._enable_motion:
            self._publish(self._motor_pub, "S")
        if self._enable_pump:
            self._publish(self._pump_pub, "P,0")

    def _on_mode_set(self, message: String) -> None:
        try:
            request = parse_mode_request(message.data)
        except ValueError as exc:
            self.get_logger().error(f"rejected mode request: {exc}")
            return
        changed = request.mode != self._mode
        if changed:
            self._stop_for_mode_change()
            self._mode = request.mode
        self._mode_request_id = request.request_id
        self.get_logger().warn(
            f"mode {'changed' if changed else 'confirmed'}: {self._mode}")
        self._publish_mode_status()

    def _on_dispatch(self, message: String) -> None:
        try:
            dispatch = parse_dispatch(message.data)
        except ValueError as exc:
            self.get_logger().error(f"rejected fire dispatch: {exc}")
            return
        if not allows_dispatch(self._mode):
            self.get_logger().error(
                f"rejected fire dispatch {dispatch.mission_id}: "
                f"mode={self._mode} does not allow dispatch")
            return
        accepted, reason = self._controller.accept_dispatch(dispatch, self._now())
        text = f"fire dispatch {dispatch.mission_id}: {reason}"
        if accepted:
            self.get_logger().info(text)
        else:
            self.get_logger().error(text)

    def _on_cancel(self, message: String) -> None:
        try:
            mission_id = parse_cancel(message.data)
        except ValueError as exc:
            self.get_logger().error(f"rejected fire cancel: {exc}")
            return
        accepted, reason = self._controller.cancel(mission_id)
        text = f"fire cancel {mission_id}: {reason}"
        if accepted:
            self.get_logger().warn(text)
        else:
            self.get_logger().error(text)

    def _on_reset(self, _message: Empty) -> None:
        accepted, reason = self._controller.reset()
        if accepted:
            self.get_logger().info(reason)
        else:
            self.get_logger().error(reason)

    def _on_telemetry(self, message: String) -> None:
        fields: dict[str, str] = {}
        for token in message.data.split(",")[1:]:
            key, separator, value = token.partition(":")
            if separator:
                fields[key] = value
        try:
            self._left_pwm = int(fields["L"])
            self._right_pwm = int(fields["R"])
            self._pump_feedback = bool(int(fields["P"]))
            self._telemetry_stamp = self._now()
        except (KeyError, ValueError):
            self._pump_feedback = None

    def _on_scan(self, message: LaserScan) -> None:
        nearest = float("inf")
        for index, distance in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if abs(angle) > self._obstacle_half_angle:
                continue
            if math.isfinite(distance) and message.range_min <= distance <= message.range_max:
                nearest = min(nearest, float(distance))
        self._obstacle = nearest < self._obstacle_distance

    def _pose(self, now: float) -> tuple[tuple[float, float, float] | None, float]:
        try:
            found = self._buffer.lookup_transform(
                self._map_frame, self._base_frame, Time(),
                timeout=Duration(seconds=0.02))
        except Exception:
            return None, float("inf")
        q = found.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        stamp = Time.from_msg(found.header.stamp).nanoseconds * 1e-9
        pose = (float(found.transform.translation.x),
                float(found.transform.translation.y), yaw)
        return pose, max(0.0, now - stamp)

    def _publish(self, publisher, value: str) -> None:
        message = String()
        message.data = value
        publisher.publish(message)

    def _tick(self) -> None:
        now = self._now()
        pose, pose_age = self._pose(now)
        follow_age = (float("inf") if self._follow_command_stamp is None
                      else max(0.0, now - self._follow_command_stamp))
        follow_command = (
            self._follow_command
            if (allows_follow(self._mode) and follow_age <= self._follow_max_age)
            else "S")
        telemetry_age = (float("inf") if self._telemetry_stamp is None
                         else max(0.0, now - self._telemetry_stamp))
        output = self._controller.update(MissionInputs(
            now=now,
            pose=pose,
            pose_age_s=pose_age,
            # Nav2 주행 중에는 회피를 코스트맵에 맡긴다.  여기서 켜두면
            # 모터는 못 세우면서 미션만 취소하는 게이트가 된다.
            obstacle=(self._obstacle and self._nav2 is None),
            mcu_stopped=(self._left_pwm == 0 and self._right_pwm == 0),
            telemetry_age_s=telemetry_age,
            pump_feedback=self._pump_feedback,
            follow_command=follow_command,
        ))

        # --- 주행 경로 선택 ---------------------------------------------------
        # navigator=nav2 일 때 NAVIGATING 구간의 모터 명령은 Nav2 배관
        # (controller_server -> velocity_smoother -> cmd_vel_bridge) 이 낸다.
        # 여기서 같이 내면 두 곳이 같은 모터를 밀게 되므로 막는다.
        nav2_driving = False
        if self._nav2 is not None:
            if output.state is MissionState.NAVIGATING:
                target = self._controller.dispatch
                # 목표 전송도 enable_motion 으로 막는다.  Nav2 는 자기 배관으로
                # 모터를 움직이므로, 이 문을 안 걸면 dry-run 이 dry-run 이 아니다.
                if self._enable_motion and target is not None:
                    self._nav2.ensure_goal(target.mission_id, target.x,
                                           target.y, target.yaw, target.frame_id)
                # Nav2 가 실제로 목표를 물고 있는 동안만 조용히 있는다.
                # Nav2 는 자기 yaw 허용오차(11.5 deg)에서 손을 떼는데,
                # 감독기 도착 판정은 8 deg 라 그 사이가 남는다.  여기서
                # 계속 막으면 아무도 그 각도를 못 지운다.
                nav2_driving = self._nav2.active
            elif output.state is MissionState.RETURNING:
                # 목표 좌표만 출발 지점으로 바꾼다.  ensure_goal 의 키에
                # x, y, yaw 가 들어가므로 이것만으로 새 목표가 나간다.
                target = self._controller.dispatch
                home = self._controller.home
                if (self._enable_motion and target is not None
                        and home is not None):
                    self._nav2.ensure_goal(target.mission_id, home[0],
                                           home[1], home[2], target.frame_id)
                nav2_driving = self._nav2.active
            elif self._nav2.active:
                self._nav2.cancel(f"state={output.state.value}")

        if (self._enable_motion and output.motor_command is not None
                and not nav2_driving):
            self._publish(self._motor_pub, output.motor_command)
        if self._enable_pump:
            self._publish(self._pump_pub, output.pump_command)

        status = {
            "schema": 1,
            "mode": self._mode,
            "mission_id": output.mission_id,
            "target": (None if self._controller.dispatch is None else {
                "frame_id": self._controller.dispatch.frame_id,
                "x": self._controller.dispatch.x,
                "y": self._controller.dispatch.y,
                "yaw": self._controller.dispatch.yaw,
                "main_cleared": self._controller.dispatch.main_cleared,
            }),
            "state": output.state.value,
            "reason": output.reason,
            "motor_command": output.motor_command,
            "pump_command": output.pump_command,
            "motion_enabled": self._enable_motion,
            "pump_enabled": self._enable_pump,
            "pose_age_s": None if not math.isfinite(pose_age) else round(pose_age, 3),
            "telemetry_age_s": (None if not math.isfinite(telemetry_age)
                                else round(telemetry_age, 3)),
            "obstacle": self._obstacle,
            "distance_m": (None if output.distance_m is None
                           else round(output.distance_m, 3)),
            "yaw_error_deg": (None if output.yaw_error_deg is None
                              else round(output.yaw_error_deg, 2)),
            "navigator": self._navigator,
            "nav2": None if self._nav2 is None else self._nav2.status(),
        }
        self._publish(self._status_pub, json.dumps(status, separators=(",", ":")))

    def shutdown(self) -> None:
        if self._nav2 is not None and self._nav2.active:
            self._nav2.cancel("shutdown")
        if self._enable_motion:
            self._publish(self._motor_pub, "S")
        if self._enable_pump:
            self._publish(self._pump_pub, "P,0")


def main(args=None) -> int:
    rclpy.init(args=args)
    try:
        node = FireSupervisor()
    except ValueError as exc:
        print(f"fire_supervisor startup refused: {exc}")
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
