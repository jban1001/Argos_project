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
            '/odom',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

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
        odom.header.frame_id = 'odom'

        odom.child_frame_id = 'base_link'

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

        self.odom_pub.publish(odom)

        # ============================
        # TF odom -> base_link
        # ============================

        transform = TransformStamped()

        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

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
