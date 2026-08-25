#!/usr/bin/env python3

"""
Main Robot -> Follower Robot 공유 계층

입력
----
  TF  map -> base_link      (AMCL 또는 slam_toolbox + wheel_odometry_node)
  /map   nav_msgs/OccupancyGrid

출력
----
  /main/pose        geometry_msgs/PoseStamped   frame_id = map     (기본 20 Hz)
  /main/trajectory  nav_msgs/Path               frame_id = map     (기본 2 Hz)
  /main/map         nav_msgs/OccupancyGrid      frame_id = map     (전달, latched)

왜 TF 를 공유하지 않고 토픽으로 내보내는가
------------------------------------------
프롬프트 §14 는 main_odom / main_base_link 네임스페이스를 요구하지만,
메인봇 스택 전체(AMCL, Nav2, argos_base_driver, wheel_odometry_node,
slam_toolbox, 모든 costmap)가 odom / base_link 를 쓴다.
이걸 개명하면 지금 동작 중인 항법 스택을 전부 손봐야 한다.

대신 메인봇은 map 기준 pose 를 "토픽" 으로 내보내고,
Follower 쪽 노드가 그 토픽을 받아 자기 TF 트리에
map -> main_base_link 를 세운다.

이러면
  - 메인봇 프레임 이름을 하나도 안 바꾼다
  - 두 로봇이 같은 base_link / odom 을 쓰는 일이 없다        (§32-7)
  - 같은 TF 를 두 노드가 동시에 발행하는 일이 없다            (§32-6)
  - TF 트리에 루프가 안 생긴다                                (§14)
  - PoseStamped 에 측정 시각이 실려 있으므로 Follower 의
    tf2 buffer 가 카메라 시각으로 보간할 수 있다              (§21)

trajectory
----------
현재 위치를 그대로 쫓으면 코너를 대각선으로 자른다 (§22).
그래서 메인봇이 실제로 지나온 점을 일정 간격으로 남긴다.

간격/최대길이는 파라미터다. 무한정 쌓지 않는다.

QoS
---
  /main/map        transient_local + reliable
                   늦게 붙은 Follower 도 맵을 받아야 한다
  /main/pose       reliable, depth 1
                   최신값만 의미가 있다
  /main/trajectory reliable, transient_local, depth 1
                   재접속 시 궤적을 다시 받아야 한다

TF 가 없으면 아무것도 발행하지 않는다.
"마지막 값을 계속 재발행" 하면 Follower 가 정지한 메인봇으로 오해한다.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path

from tf2_ros import Buffer, TransformListener


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


class MainShareNode(Node):

    def __init__(self) -> None:

        super().__init__("main_share_node")

        # ---------------- parameters ----------------

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("pose_rate", 20.0)
        self.declare_parameter("trajectory_rate", 2.0)

        # 궤적 점 사이 최소 간격 [m]
        self.declare_parameter("trail_spacing", 0.08)

        # 궤적 최대 점 개수. 0.08 m x 500 = 약 40 m
        self.declare_parameter("trail_max_points", 500)

        # 이 시간보다 오래된 TF 는 쓰지 않는다 [s]
        self.declare_parameter("tf_timeout", 0.30)

        # /map 을 /main/map 으로 전달할지
        self.declare_parameter("relay_map", True)

        p = self.get_parameter

        self.map_frame: str = str(p("map_frame").value)
        self.base_frame: str = str(p("base_frame").value)

        self.pose_rate: float = float(p("pose_rate").value)
        self.trajectory_rate: float = float(p("trajectory_rate").value)

        self.trail_spacing: float = float(p("trail_spacing").value)
        self.trail_max: int = int(p("trail_max_points").value)

        self.tf_timeout: float = float(p("tf_timeout").value)
        self.relay_map: bool = bool(p("relay_map").value)

        # ---------------- QoS ----------------

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        live = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # ---------------- pub / sub ----------------

        self.pose_pub = self.create_publisher(
            PoseStamped, "/main/pose", live
        )

        self.traj_pub = self.create_publisher(
            Path, "/main/trajectory", latched
        )

        self.map_pub: Optional[rclpy.publisher.Publisher] = None

        if self.relay_map:

            self.map_pub = self.create_publisher(
                OccupancyGrid, "/main/map", latched
            )

            self.create_subscription(
                OccupancyGrid, "/map", self.map_cb, latched
            )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- state ----------------

        # 궤적. (x, y, yaw) 를 PoseStamped 로 보관한다.
        self._trail: list[PoseStamped] = []

        self._last_trail_xy: Optional[Tuple[float, float]] = None

        self._had_tf = False
        self._warned = False

        self._n_pose = 0

        # ---------------- timers ----------------

        self.create_timer(1.0 / self.pose_rate, self.on_pose_timer)
        self.create_timer(1.0 / self.trajectory_rate, self.on_traj_timer)
        self.create_timer(5.0, self.on_diag_timer)

        self.get_logger().info(
            f"main_share_node: {self.map_frame} -> {self.base_frame}, "
            f"pose {self.pose_rate} Hz, "
            f"trail {self.trail_spacing} m x {self.trail_max}"
        )

    # ---------------------------------------------------------

    def map_cb(self, msg: OccupancyGrid) -> None:
        """받은 맵을 그대로 /main/map 으로 전달한다."""

        if self.map_pub is not None:
            self.map_pub.publish(msg)

    # ---------------------------------------------------------

    def lookup_pose(self) -> Optional[PoseStamped]:
        """
        map -> base_link 를 PoseStamped 로.

        rclpy.time.Time() (=0) 으로 조회하면 "가장 최근" 을 준다.
        그 뒤 실제 stamp 를 보고 너무 오래됐으면 버린다.
        stamp 는 반드시 TF 의 것을 그대로 쓴다.
        지금 시각을 찍으면 Follower 쪽 시간 정렬이 깨진다. (§10, §21)
        """

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
            )

        except Exception as e:

            if not self._warned:
                self.get_logger().warn(
                    f"TF {self.map_frame} -> {self.base_frame} 없음: {e}"
                )
                self._warned = True

            return None

        stamp = rclpy.time.Time.from_msg(tf.header.stamp)

        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9

        # stamp 가 0 이면 static 이거나 시각이 안 실린 것이다.
        if stamp.nanoseconds > 0 and age > self.tf_timeout:
            return None

        msg = PoseStamped()

        msg.header.stamp = tf.header.stamp
        msg.header.frame_id = self.map_frame

        msg.pose.position.x = tf.transform.translation.x
        msg.pose.position.y = tf.transform.translation.y
        msg.pose.position.z = tf.transform.translation.z

        msg.pose.orientation = tf.transform.rotation

        return msg

    # ---------------------------------------------------------

    def on_pose_timer(self) -> None:

        pose = self.lookup_pose()

        if pose is None:
            self._had_tf = False
            return

        if not self._had_tf:
            self.get_logger().info("TF 확보. /main/pose 발행 시작")
            self._had_tf = True
            self._warned = False

        self.pose_pub.publish(pose)

        self._n_pose += 1

        self.append_trail(pose)

    # ---------------------------------------------------------

    def append_trail(self, pose: PoseStamped) -> None:
        """일정 간격 이상 움직였을 때만 궤적에 남긴다."""

        x = pose.pose.position.x
        y = pose.pose.position.y

        if self._last_trail_xy is not None:

            dx = x - self._last_trail_xy[0]
            dy = y - self._last_trail_xy[1]

            if math.hypot(dx, dy) < self.trail_spacing:
                return

        self._trail.append(pose)

        self._last_trail_xy = (x, y)

        if len(self._trail) > self.trail_max:
            # 오래된 것부터 버린다
            del self._trail[:len(self._trail) - self.trail_max]

    # ---------------------------------------------------------

    def on_traj_timer(self) -> None:

        if not self._trail:
            return

        path = Path()

        path.header.frame_id = self.map_frame
        path.header.stamp = self._trail[-1].header.stamp

        path.poses = list(self._trail)

        self.traj_pub.publish(path)

    # ---------------------------------------------------------

    def on_diag_timer(self) -> None:

        self.get_logger().info(
            f"[diag] pose={'OK' if self._had_tf else 'NO_TF'} "
            f"n_pose={self._n_pose} trail={len(self._trail)}"
        )


def main(args=None) -> None:

    rclpy.init(args=args)

    node = MainShareNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
