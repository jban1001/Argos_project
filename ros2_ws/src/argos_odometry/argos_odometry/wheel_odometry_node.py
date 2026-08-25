#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class WheelOdometryNode(Node):

    def __init__(self):
        super().__init__('wheel_odometry_node')

        # ============================
        # ARGOS calibration parameters
        # ============================

        # 1m 이동했을 때 발생하는 encoder tick 수
        self.declare_parameter('left_ticks_per_meter', 1000.0)
        self.declare_parameter('right_ticks_per_meter', 1000.0)

        # 좌우 궤도 중심 사이 거리 [m]
        self.declare_parameter('track_width', 0.40)

        # encoder 방향 보정
        # 앞으로 움직였을 때 tick이 증가하면 +1
        # 감소하면 -1
        self.declare_parameter('left_sign', 1.0)
        self.declare_parameter('right_sign', 1.0)

        # ============================
        # 출력 경로 (EKF 융합용)
        # ============================
        #
        # 기본값은 기존과 완전히 동일하다.
        #   /odom 을 내고 odom -> base_link TF 도 직접 쏜다.
        #
        # robot_localization EKF 를 쓸 때는 launch 에서
        #   odom_topic:=/wheel/odom_raw
        #   publish_tf:=false
        # 로 바꾼다. 그러면 이 노드는 원시 측정만 내고
        # /odom 과 TF 는 EKF 가 책임진다.
        #
        # /odom 발행자와 odom->base_link TF 발행자는
        # 어느 모드에서든 정확히 하나여야 한다.

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # ============================
        # Covariance
        # ============================
        #
        # 기존 코드는 covariance 를 전부 0 으로 두고 있었다.
        # 혼자 쓸 때는 아무도 안 보지만, EKF 에 넣으면
        # robot_localization 이 "분산 0 = 무한 신뢰" 로 해석해서
        # 필터가 발산하거나 IMU 를 완전히 무시한다.
        #
        # skid-steer 는 제자리 회전에서 슬립이 크므로
        # yaw rate 쪽 분산을 넉넉히 잡는다. 그래야 EKF 가
        # 회전 중에는 gyro 를 더 믿는다. 이게 이번 작업의 핵심이다.

        self.declare_parameter('twist_var_vx', 1.0e-3)
        self.declare_parameter('twist_var_vy', 1.0e-3)
        self.declare_parameter('twist_var_wz', 5.0e-2)

        # pose 분산은 적분값이라 시간이 지나면 의미가 없다.
        # EKF 에서 pose 를 융합하지 않으므로 큰 값으로 표시만 해 둔다.
        self.declare_parameter('pose_var_xy', 1.0e-2)
        self.declare_parameter('pose_var_yaw', 5.0e-2)

        self.left_ticks_per_meter = float(
            self.get_parameter('left_ticks_per_meter').value
        )

        self.right_ticks_per_meter = float(
            self.get_parameter('right_ticks_per_meter').value
        )

        self.track_width = float(
            self.get_parameter('track_width').value
        )

        self.left_sign = float(
            self.get_parameter('left_sign').value
        )

        self.right_sign = float(
            self.get_parameter('right_sign').value
        )

        self.odom_topic = str(
            self.get_parameter('odom_topic').value
        )

        self.publish_tf = bool(
            self.get_parameter('publish_tf').value
        )

        self.odom_frame = str(
            self.get_parameter('odom_frame').value
        )

        self.base_frame = str(
            self.get_parameter('base_frame').value
        )

        self.twist_var_vx = float(
            self.get_parameter('twist_var_vx').value
        )

        self.twist_var_vy = float(
            self.get_parameter('twist_var_vy').value
        )

        self.twist_var_wz = float(
            self.get_parameter('twist_var_wz').value
        )

        self.pose_var_xy = float(
            self.get_parameter('pose_var_xy').value
        )

        self.pose_var_yaw = float(
            self.get_parameter('pose_var_yaw').value
        )

        # ============================
        # State
        # ============================

        self.left_ticks = None
        self.right_ticks = None

        self.prev_left_ticks = None
        self.prev_right_ticks = None

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()

        # ============================
        # ROS
        # ============================

        self.left_sub = self.create_subscription(
            Int64,
            '/wheel_ticks/left',
            self.left_callback,
            10
        )

        self.right_sub = self.create_subscription(
            Int64,
            '/wheel_ticks/right',
            self.right_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_topic,
            10
        )

        # publish_tf=false 면 broadcaster 자체를 만들지 않는다.
        # 만들어 두고 안 쓰면 나중에 실수로 쏘기 쉽다.
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_tf else None
        )

        # 50 Hz
        self.timer = self.create_timer(
            0.02,
            self.update_odometry
        )

        self.get_logger().info('ARGOS wheel odometry started')

        self.get_logger().info(
            f'left_ticks_per_meter = {self.left_ticks_per_meter}'
        )

        self.get_logger().info(
            f'right_ticks_per_meter = {self.right_ticks_per_meter}'
        )

        self.get_logger().info(
            f'track_width = {self.track_width} m'
        )

        self.get_logger().info(
            f'odom_topic = {self.odom_topic}, '
            f'publish_tf = {self.publish_tf}, '
            f'{self.odom_frame} -> {self.base_frame}'
        )

        if not self.publish_tf:
            self.get_logger().info(
                'publish_tf=false : odom -> base_link TF 는 '
                'robot_localization EKF 가 발행한다'
            )

    def left_callback(self, msg):
        self.left_ticks = msg.data

    def right_callback(self, msg):
        self.right_ticks = msg.data

    def update_odometry(self):

        # 아직 양쪽 encoder 값을 못 받았으면 대기
        if self.left_ticks is None or self.right_ticks is None:
            return

        # 최초 값 저장
        if self.prev_left_ticks is None or self.prev_right_ticks is None:

            self.prev_left_ticks = self.left_ticks
            self.prev_right_ticks = self.right_ticks

            self.last_time = self.get_clock().now()

            return

        now = self.get_clock().now()

        dt = (now - self.last_time).nanoseconds / 1e9

        if dt <= 0.0:
            return

        # ============================
        # Tick difference
        # ============================

        delta_left_ticks = (
            self.left_ticks - self.prev_left_ticks
        ) * self.left_sign

        delta_right_ticks = (
            self.right_ticks - self.prev_right_ticks
        ) * self.right_sign

        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks

        # ============================
        # Tick -> distance [m]
        # ============================

        delta_left = (
            delta_left_ticks /
            self.left_ticks_per_meter
        )

        delta_right = (
            delta_right_ticks /
            self.right_ticks_per_meter
        )

        # ============================
        # Differential-drive kinematics
        # ============================

        delta_s = (
            delta_right + delta_left
        ) / 2.0

        delta_theta = (
            delta_right - delta_left
        ) / self.track_width

        # midpoint integration
        theta_mid = (
            self.theta + delta_theta / 2.0
        )

        self.x += (
            delta_s * math.cos(theta_mid)
        )

        self.y += (
            delta_s * math.sin(theta_mid)
        )

        self.theta += delta_theta

        # angle normalization
        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta)
        )

        # ============================
        # Velocity
        # ============================

        linear_velocity = delta_s / dt
        angular_velocity = delta_theta / dt

        # yaw -> quaternion
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        stamp = now.to_msg()

        # ============================
        # nav_msgs/Odometry
        # ============================

        odom = Odometry()

        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame

        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_velocity

        # 6x6 행 우선. 대각선만 채운다.
        #   0=x 7=y 14=z 21=roll 28=pitch 35=yaw
        odom.pose.covariance[0] = self.pose_var_xy
        odom.pose.covariance[7] = self.pose_var_xy
        odom.pose.covariance[35] = self.pose_var_yaw

        odom.twist.covariance[0] = self.twist_var_vx
        odom.twist.covariance[7] = self.twist_var_vy
        odom.twist.covariance[35] = self.twist_var_wz

        # 2D 로봇이라 z / roll / pitch 는 관측되지 않는다.
        # 큰 값을 넣어 EKF 가 절대 쓰지 않게 한다.
        for i in (14, 21, 28):
            odom.pose.covariance[i] = 1.0e6
            odom.twist.covariance[i] = 1.0e6

        self.odom_pub.publish(odom)

        # ============================
        # TF odom -> base_link
        # ============================
        #
        # EKF 융합 모드에서는 EKF 가 이 TF 를 낸다.
        # 둘이 같이 쏘면 TF 가 깜빡이며 스캔 정합이 깨진다.

        if self.tf_broadcaster is None:
            self.last_time = now
            return

        transform = TransformStamped()

        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)

        self.last_time = now


def main(args=None):

    rclpy.init(args=args)

    node = WheelOdometryNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
