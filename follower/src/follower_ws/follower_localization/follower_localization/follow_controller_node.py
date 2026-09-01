"""메인 로봇의 궤적을 따라가는 명령을 만든다 (Phase 8~9).

Subscribes:
    <main>/amcl_pose          메인 로봇 자세 -> 궤적으로 쌓는다
    /follower/aruco/status    마커가 보이는가 (상태 판정용)
    TF  map -> follower_base_link   팔로워 현재 자세

Publishes:
    /follower/motor_command   std_msgs/String   (기본값: 발행하지 않음)

왜 궤적을 따라가는가
--------------------
메인 로봇을 향해 직진하면 코너에서 안쪽으로 잘라 들어가 벽이나 장애물을
만난다. 메인 로봇이 **지나간 경로**를 따라가면 메인이 통과한 곳만 통과한다.
그래서 현재 위치가 아니라 궤적을 쌓고, 끝에서 follow_distance 만큼 거슬러
올라간 점을 목표로 삼는다.

기본적으로 발행하지 않는다
--------------------------
publish_commands 가 false 면 명령 문자열을 만들어 로그로만 낸다. 제어
논리를 실제 바퀴 없이 확인하기 위한 것이고, 준비됐을 때 명시적으로 켠다.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from follower_localization import transforms as tf
from follower_localization.state_machine import (
    FollowerStateMachine,
    FollowState,
    Inputs,
    StateLimits,
)
from follower_localization.frames import base_to_camera, find_config_dir
from follower_localization.target_generator import (
    TargetLimits,
    TrajectoryFollower,
    direct_marker_command,
    steering_command,
)


class FollowController(Node):

    def __init__(self, **kwargs) -> None:
        super().__init__("follow_controller", **kwargs)

        self.declare_parameter("publish_commands", False)
        self.declare_parameter("max_throttle_limit", 180)
        self.declare_parameter("max_yaw_rate_limit_dps", 90.0)
        self.declare_parameter("max_throttle", 60)
        self.declare_parameter("min_throttle", 25)
        self.declare_parameter("max_yaw_rate_dps", 45.0)
        self.declare_parameter("follow_distance_m", 1.0)
        self.declare_parameter("history_length_m", 10.0)
        self.declare_parameter("min_spacing_m", 0.02)
        self.declare_parameter("min_history_m", 0.15)
        self.declare_parameter("distance_deadband_m", 0.15)
        self.declare_parameter("angle_deadband_deg", 5.0)
        self.declare_parameter("steering_gain_dps_per_deg", 1.5)
        self.declare_parameter("throttle_gain_per_m", 60.0)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "follower_base_link")
        _config = find_config_dir()
        self.declare_parameter("frames_config",
                               str(_config / "follower_frames.yaml"))
        self.declare_parameter("cam_imu_config",
                               str(_config / "cam_imu.yaml"))
        self.declare_parameter("main_pose_topic", "/amcl_pose")

        def value(name):
            return self.get_parameter(name).value

        # 실제 상한이 하드웨어 한계를 넘지 않는지. 설정 실수로 MCU 가 거부하는
        # 명령을 계속 보내면 로봇은 그냥 멈춰 있고 원인은 안 보인다.
        self._max_throttle = int(value("max_throttle"))
        self._max_yaw = float(value("max_yaw_rate_dps"))
        throttle_limit = int(value("max_throttle_limit"))
        yaw_limit = float(value("max_yaw_rate_limit_dps"))
        if self._max_throttle > throttle_limit:
            raise ValueError(
                f"max_throttle {self._max_throttle} 이 하드웨어 한계 "
                f"{throttle_limit} 를 넘는다")
        if self._max_yaw > yaw_limit:
            raise ValueError(
                f"max_yaw_rate_dps {self._max_yaw} 가 하드웨어 한계 "
                f"{yaw_limit} 를 넘는다")

        self._publish = bool(value("publish_commands"))
        self._min_throttle = int(value("min_throttle"))
        self._follow_distance = float(value("follow_distance_m"))
        self._distance_deadband = float(value("distance_deadband_m"))
        self._angle_deadband = float(value("angle_deadband_deg"))
        self._steering_gain = float(value("steering_gain_dps_per_deg"))
        self._throttle_gain = float(value("throttle_gain_per_m"))
        self._map_frame = str(value("map_frame"))
        self._base_frame = str(value("base_frame"))

        self._trajectory = TrajectoryFollower(TargetLimits(
            follow_distance_m=self._follow_distance,
            min_spacing_m=float(value("min_spacing_m")),
            history_length_m=float(value("history_length_m")),
            min_history_m=float(value("min_history_m"))))
        self._machine = FollowerStateMachine(StateLimits())

        # 마커를 로봇 기준으로 옮기는 정적 변환. 캘리브레이션 파일만 읽으므로
        # VIO 나 AMCL 이 죽어도 유효하다 -- 직접 추종이 그것들과 무관해야
        # 하는 이유가 여기 있다.
        self._t_base_cam = base_to_camera(
            Path(str(value("frames_config"))), Path(str(value("cam_imu_config"))))

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._command_pub = self.create_publisher(String, "motor_command", 10)
        self._state_pub = self.create_publisher(String, "follow/state", 10)

        self._aruco_last_seen: float | None = None
        self._main_stamp: float | None = None
        self._global_last_correction: float | None = None
        self._vio_last_stamp: float | None = None
        # 마커 관측: (거리 m, 방위 deg). 방위는 base_link 규약으로 좌가 양.
        self._marker: tuple[float, float] | None = None

        self.create_subscription(PoseWithCovarianceStamped,
                                 str(value("main_pose_topic")),
                                 self._on_main_pose, 10)
        self.create_subscription(String, "aruco/status", self._on_aruco_status, 10)
        self.create_subscription(PoseWithCovarianceStamped, "aruco/pose",
                                 self._on_aruco_pose, 10)
        self.create_timer(1.0 / float(value("control_rate_hz")), self._tick)

        self.get_logger().info(
            f"follow_controller: 추종거리 {self._follow_distance:.2f} m, "
            f"throttle <= {self._max_throttle}, yaw <= {self._max_yaw:.0f} deg/s, "
            f"명령 발행 {'켜짐' if self._publish else '꺼짐 (로그만)'}")

    # --- 입력 --------------------------------------------------------------

    def _on_main_pose(self, message: PoseWithCovarianceStamped) -> None:
        self._main_stamp = Time.from_msg(message.header.stamp).nanoseconds * 1e-9
        self._trajectory.add(float(message.pose.pose.position.x),
                             float(message.pose.pose.position.y))

    def _on_aruco_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except ValueError:
            return
        if payload.get("accepted"):
            self._aruco_last_seen = self._now()

    def _on_aruco_pose(self, message: PoseWithCovarianceStamped) -> None:
        """마커를 로봇 기준 (거리, 방위) 로 바꿔 둔다.

        카메라 기준 관측을 base_link 로 옮기는 데 쓰는 것은 정적 캘리브레이션
        뿐이다. TF 트리를 조회하지 않으므로 VIO 나 협조 보정이 죽어 있어도
        이 값은 나온다 -- 직접 추종이 그것들과 무관해야 하는 이유다.
        """
        p = message.pose.pose.position
        marker_in_base = self._t_base_cam @ np.array([p.x, p.y, p.z, 1.0])
        x, y = float(marker_in_base[0]), float(marker_in_base[1])
        range_m = float(math.hypot(x, y))
        if range_m < 1e-6:
            return
        # base_link 규약: x 앞, y 좌. 좌가 양의 방위이고 양의 yaw_rate 다.
        self._marker = (range_m, math.degrees(math.atan2(y, x)))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _follower_pose(self):
        """map -> follower_base_link. 없으면 None."""
        try:
            found = self._buffer.lookup_transform(
                self._map_frame, self._base_frame, Time(),
                timeout=Duration(seconds=0.02))
        except Exception:
            return None
        matrix = tf.make_transform(
            [found.transform.translation.x, found.transform.translation.y,
             found.transform.translation.z],
            [found.transform.rotation.x, found.transform.rotation.y,
             found.transform.rotation.z, found.transform.rotation.w])
        stamp = Time.from_msg(found.header.stamp).nanoseconds * 1e-9
        self._global_last_correction = stamp
        self._vio_last_stamp = stamp
        return matrix

    # --- 제어 --------------------------------------------------------------

    def _tick(self) -> None:
        now = self._now()
        pose = self._follower_pose()

        decision = self._machine.update(Inputs(
            now=now,
            aruco_last_seen=self._aruco_last_seen,
            global_last_correction=self._global_last_correction,
            main_pose_stamp=self._main_stamp,
            vio_last_stamp=self._vio_last_stamp,
            vio_position_sigma_m=0.0,
            have_trajectory=len(self._trajectory) > 0))

        command = decision.stop_command
        target = None

        # 주 경로: 마커가 보이면 마커만 보고 간다. 지도도 VIO 도 쓰지 않는다.
        if command is None and decision.state is FollowState.ARUCO_LOCAL_FOLLOW:
            if self._marker is None:
                command = "S"
            else:
                range_m, bearing_deg = self._marker
                command = direct_marker_command(
                    range_m=range_m,
                    bearing_deg=bearing_deg,
                    follow_distance_m=self._follow_distance,
                    max_throttle=self._max_throttle,
                    min_throttle=self._min_throttle,
                    distance_deadband_m=self._distance_deadband,
                    angle_deadband_deg=self._angle_deadband,
                    steering_gain_dps_per_deg=self._steering_gain,
                    max_yaw_rate_dps=self._max_yaw,
                    throttle_gain_per_m=self._throttle_gain,
                    scale=decision.throttle_scale)

        # 보조 경로: 마커를 놓쳤을 때만 지도 궤적을 따른다.
        if command is None and pose is not None:
            target = self._trajectory.target(self._follow_distance)
            if target is None:
                # 궤적이 아직 짧다. 움직이지 않는 편이 낫다.
                command = "S"
            else:
                command = steering_command(
                    follower_xy=pose[:2, 3],
                    follower_yaw=tf.yaw_of(pose),
                    target=target,
                    max_throttle=self._max_throttle,
                    min_throttle=self._min_throttle,
                    distance_deadband_m=self._distance_deadband,
                    angle_deadband_deg=self._angle_deadband,
                    steering_gain_dps_per_deg=self._steering_gain,
                    max_yaw_rate_dps=self._max_yaw,
                    throttle_gain_per_m=self._throttle_gain,
                    scale=decision.throttle_scale)
        elif command is None:
            # 자세를 모르면 조향할 수 없다. 상태 기계가 LOST 로 가기 전이라도
            # 여기서는 정지가 유일하게 안전한 선택이다.
            command = "S"

        self._publish_state(decision, target, command)
        if self._publish:
            message = String()
            message.data = command
            self._command_pub.publish(message)

        # 발행할 때도 로그를 낸다. 예전에는 발행 모드에서 아무 말이 없어서,
        # 화면에는 MCU: STOP 만 흐르고 "왜 정지인지" 가 어디에도 안 보였다.
        # 그 상태로는 마커를 못 본 것인지, 궤적이 없는 것인지, VIO 가 죽은
        # 것인지 구분할 수 없어 짐작으로 고치게 된다.
        prefix = "" if self._publish else "[발행안함] "
        detail = f" -- {decision.reason}" if decision.state is not FollowState.ARUCO_LOCAL_FOLLOW else ""
        self.get_logger().info(
            f"{prefix}{decision.state.value}: {command}{detail}",
            throttle_duration_sec=1.0)

    def _publish_state(self, decision, target, command: str) -> None:
        payload = {
            "state": decision.state.value,
            "reason": decision.reason,
            "throttle_scale": round(decision.throttle_scale, 3),
            "command": command,
            "published": self._publish,
            "trajectory_points": len(self._trajectory),
        }
        if target is not None:
            payload["target_xy"] = [round(float(v), 3) for v in target.position]
            payload["along_path_m"] = round(target.along_path_m, 3)
            payload["clamped"] = bool(target.clamped)
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self._state_pub.publish(message)


def main(argv=None) -> int:
    rclpy.init(args=argv)
    try:
        node = FollowController()
    except ValueError as exc:
        print(f"follow_controller 기동 거부: {exc}")
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
