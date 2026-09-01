"""ArUco 관측과 메인 로봇 AMCL 자세로 map -> follower_odom 을 보정한다.

Subscribes:
    /follower/aruco/pose      T_C_A, 가장 잘 맞는 해
    /follower/aruco/pose_alt  T_C_A, 두 번째 해
    <main>/amcl_pose          메인 로봇의 map 기준 자세 (읽기 전용)

Publishes:
    TF  map -> follower_odom

왜 map -> follower_odom 인가
---------------------------
VIO 는 follower_odom -> follower_base_link 를 연속적으로 갱신한다. 그 위에
map -> follower_odom 하나만 보정하면 전체 사슬이 일관되게 유지되고, ArUco 가
안 보이는 동안에도 VIO 가 계속 굴러간다. 절대 위치를 base_link 에 직접
쓰면 VIO 와 충돌해서 자세가 튄다.

두 해 중 고르기
---------------
정사각 평면 마커는 정면에 가까울수록 두 PnP 해의 재투영 오차가 비슷해져
프레임마다 뒤집힌다. 이 로봇은 마커 정면에서 따라가므로 늘 그 상태다.
메인 로봇 AMCL 자세와 팔로워의 직전 추정으로 "마커가 이렇게 보일 것" 을
만들고, 두 해 중 그것과 가까운 쪽을 고른다.

시각 정합
---------
TF 조회는 관측 시각으로 한다 ("최신" 이 아니라). 무선 구간의 TF 지연이
p95 539 ms 로 측정됐는데, 최신 값을 쓰면 그 지연만큼 어긋난 오도메트리와
현재 영상을 섞게 된다.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from follower_localization import transforms as tf
from follower_localization.config import UnmeasuredValue
from follower_localization.frames import base_to_camera
from follower_localization.frames import find_config_dir
from follower_localization.gating import ArucoGate, GateLimits, GroundCorrection
from follower_localization.marker_mount import (
    expected_marker_in_camera,
    main_base_to_marker,
)


def pose_to_matrix(pose) -> np.ndarray:
    position = [pose.position.x, pose.position.y, pose.position.z]
    quaternion = [pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w]
    return tf.make_transform(position, quaternion)


def matrix_to_transform(matrix: np.ndarray, stamp, parent: str,
                        child: str) -> TransformStamped:
    message = TransformStamped()
    message.header.stamp = stamp
    message.header.frame_id = parent
    message.child_frame_id = child
    translation, quaternion = tf.split_transform(matrix)
    message.transform.translation.x = float(translation[0])
    message.transform.translation.y = float(translation[1])
    message.transform.translation.z = float(translation[2])
    message.transform.rotation.x = float(quaternion[0])
    message.transform.rotation.y = float(quaternion[1])
    message.transform.rotation.z = float(quaternion[2])
    message.transform.rotation.w = float(quaternion[3])
    return message


class CooperativeLocalization(Node):

    def __init__(self, **kwargs) -> None:
        super().__init__("cooperative_localization", **kwargs)
        # 빌드 방식에 따라 경로를 세는 것이 어긋난다. 실제로 있는 곳을 찾는다.
        config_dir = find_config_dir()

        self.declare_parameter("main_robot_config", str(config_dir / "main_robot.yaml"))
        self.declare_parameter("frames_config", str(config_dir / "follower_frames.yaml"))
        self.declare_parameter("cam_imu_config", str(config_dir / "cam_imu.yaml"))
        self.declare_parameter("main_pose_topic", "/amcl_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "follower_odom")
        self.declare_parameter("base_frame", "follower_base_link")
        # 메인 자세의 나이 제한. 0 이하면 제한하지 않는다 (기본값).
        #
        # 처음에 1.0 초로 뒀는데 그러면 아예 출발을 못 한다 (2026-08-29 실측:
        # 나이가 1.02 -> 5.77 -> 10.80 초로 계속 늘며 모든 관측을 버렸다).
        # AMCL 은 메인 로봇이 움직일 때만 자세를 낸다. 정지 중에는 새 메시지가
        # 없고, 출발 직전은 언제나 정지 상태다. 어떤 유한한 값을 골라도
        # "그보다 오래 서 있었으면 출발 못 하는" 로봇이 된다.
        #
        # 묵은 자세가 틀리려면 그 사이 메인 로봇이 움직였어야 하는데,
        # 움직였다면 AMCL 이 발행했을 것이므로 그 경우는 애초에 묵지 않는다.
        # 남는 위험은 링크가 끊겨 움직임을 못 듣는 경우뿐이고, 그것은 나이
        # 제한으로 잡을 문제가 아니다 -- 링크 감시로 잡을 문제다.
        self.declare_parameter("max_main_pose_age_s", 0.0)
        self.declare_parameter("correction_rate_mps", 0.30)
        # 점프 게이트. 보정이 드물게 들어오면 그 사이 오차가 쌓여 한계를
        # 넘고, 넘으면 또 기각되어 스스로 잠긴다 (2026-08-29 실측:
        # position_jump / yaw_jump 로 연속 기각되며 추종이 끊겼다).
        # 재기동 없이 풀 수 있도록 설정으로 뺀다.
        self.declare_parameter("max_position_jump_m", 0.5)
        self.declare_parameter("max_yaw_jump_deg", 20.0)

        self._t_base_cam = base_to_camera(
            Path(self.get_parameter("frames_config").value),
            Path(self.get_parameter("cam_imu_config").value))
        config = Path(self.get_parameter("main_robot_config").value)
        self._t_main_marker = main_base_to_marker(config)

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._max_age = float(self.get_parameter("max_main_pose_age_s").value)

        # marker_id 는 -1 (아무거나) 로 둔다. 마커 ID 검사는 aruco_pose 가
        # 이미 했고, 여기서 다시 하려면 pose 메시지에 없는 ID 를 지어내야
        # 한다. 게이트에 자기 기대값을 넣어 통과시키면 검사한 척만 하는 것이라
        # 실제로 틀린 마커가 와도 못 잡는다.
        self._gate = ArucoGate(GateLimits(
            marker_id=-1,
            max_position_jump_m=float(
                self.get_parameter("max_position_jump_m").value),
            max_yaw_jump_deg=float(
                self.get_parameter("max_yaw_jump_deg").value)))
        self._correction = GroundCorrection(
            max_speed_mps=float(self.get_parameter("correction_rate_mps").value))

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._broadcaster = TransformBroadcaster(self)

        self._main_pose: tuple[Time, np.ndarray] | None = None
        self._prior: np.ndarray | None = None
        self._alt: tuple[Time, np.ndarray] | None = None
        self._last_update: Time | None = None

        self.create_subscription(PoseWithCovarianceStamped,
                                 str(self.get_parameter("main_pose_topic").value),
                                 self._on_main_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped, "aruco/pose_alt",
                                 self._on_alternative, 10)
        self.create_subscription(PoseWithCovarianceStamped, "aruco/pose",
                                 self._on_observation, 10)

        self.get_logger().info(
            "cooperative_localization: "
            f"{self._map_frame} -> {self._odom_frame} 보정 시작")

    def _on_main_pose(self, message: PoseWithCovarianceStamped) -> None:
        self._main_pose = (Time.from_msg(message.header.stamp),
                           pose_to_matrix(message.pose.pose))

    def _on_alternative(self, message: PoseWithCovarianceStamped) -> None:
        self._alt = (Time.from_msg(message.header.stamp),
                     pose_to_matrix(message.pose.pose))

    def _choose_solution(self, stamp: Time, observed: np.ndarray,
                         t_map_main: np.ndarray) -> np.ndarray:
        """두 해 중 사전값에 가까운 쪽. 사전값이 없으면 관측 그대로."""
        if self._alt is None or self._prior is None:
            return observed
        alt_stamp, alternative = self._alt
        if abs((alt_stamp - stamp).nanoseconds) > 5_000_000:      # 5 ms
            return observed              # 같은 프레임의 짝이 아니다

        expected = expected_marker_in_camera(
            t_map_main, self._t_main_marker, self._prior, self._t_base_cam)
        _, best_deg = tf.transform_distance(expected, observed)
        _, alt_deg = tf.transform_distance(expected, alternative)
        if alt_deg < best_deg:
            self.get_logger().debug(
                f"두 번째 해를 채택 ({alt_deg:.1f} 도 vs {best_deg:.1f} 도)")
            return alternative
        return observed

    def _on_observation(self, message: PoseWithCovarianceStamped) -> None:
        if self._main_pose is None:
            return
        stamp = Time.from_msg(message.header.stamp)
        main_stamp, t_map_main = self._main_pose
        age = abs((stamp - main_stamp).nanoseconds) * 1e-9
        if self._max_age > 0.0 and age > self._max_age:
            self.get_logger().warn(
                f"메인 자세가 {age:.2f} 초 묵었다 -- 건너뛴다", throttle_duration_sec=5.0)
            return

        # VIO: follower_odom -> follower_base_link, 관측 시각으로 조회한다.
        try:
            odom = self._buffer.lookup_transform(
                self._odom_frame, self._base_frame, stamp,
                timeout=Duration(seconds=0.05))
        except Exception as exc:                       # tf2 예외 종류가 많다
            # 관측 시각을 정확히 요구하면 VIO TF 가 몇 ms 뒤처졌을 때
            # "미래를 조회한다" 며 실패한다 (2026-08-29 실측: 3~90 ms 뒤짐).
            # 그 정도 지연은 이 속도에서 무의미한데, 버리면 보정이 거의
            # 들어가지 않아 팔로워의 지도상 위치가 계속 틀린다. 최신 TF 로
            # 물러난다 -- 조금 낡은 보정이 보정 없음보다 낫다.
            try:
                odom = self._buffer.lookup_transform(
                    self._odom_frame, self._base_frame, Time())
                self.get_logger().debug(
                    f"관측 시각 TF 없음, 최신으로 대체: {exc}")
            except Exception as exc2:
                self.get_logger().warn(f"VIO TF 조회 실패: {exc2}",
                                       throttle_duration_sec=5.0)
                return
        t_odom_base = tf.make_transform(
            [odom.transform.translation.x, odom.transform.translation.y,
             odom.transform.translation.z],
            [odom.transform.rotation.x, odom.transform.rotation.y,
             odom.transform.rotation.z, odom.transform.rotation.w])

        observed = self._choose_solution(
            stamp, pose_to_matrix(message.pose.pose), t_map_main)

        t_map_follower = tf.follower_pose_in_map(
            t_map_main, self._t_main_marker, observed, self._t_base_cam)

        distance = float(np.linalg.norm(observed[:3, 3]))
        result = self._gate.check(
            marker_id=-1,
            t_map_follower=t_map_follower,
            distance_m=distance,
            reprojection_px=0.0,        # aruco_pose 가 이미 걸렀다
            stamp=stamp.nanoseconds * 1e-9)
        if not result.accepted:
            self.get_logger().info(f"관측 기각: {result.reason}",
                                   throttle_duration_sec=5.0)
            return

        self._prior = t_map_follower
        target = tf.flatten_to_ground(
            tf.map_to_odom(t_map_follower, t_odom_base))

        now = stamp.nanoseconds * 1e-9
        dt = 0.0 if self._last_update is None else now - self._last_update
        self._last_update = now
        corrected = self._correction.update(target, max(dt, 0.0))

        self._broadcaster.sendTransform(matrix_to_transform(
            corrected, message.header.stamp, self._map_frame, self._odom_frame))


def main(argv=None) -> int:
    rclpy.init(args=argv)
    try:
        node = CooperativeLocalization()
    except UnmeasuredValue as exc:
        print(f"cooperative_localization 기동 거부: {exc}")
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
