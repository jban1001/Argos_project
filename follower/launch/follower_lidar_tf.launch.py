#!/usr/bin/env python3
"""Publish the measured follower_base_link -> follower_laser_frame transform.

X/Y and orientation were measured on 2026-08-30.  Z=0.120 m is an
approximate physical measurement supplied by the user; remeasure Z before
using this transform for height-sensitive 3-D work.  Z does not affect the
current planar AMCL/costmap workflow.
"""

import math

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="follower_lidar_static_tf",
            output="screen",
            arguments=[
                "--x", "-0.043",
                "--y", "0.018",
                "--z", "0.120",
                "--roll", str(math.pi),
                "--pitch", "0.0",
                "--yaw", str(math.radians(-2.77)),
                "--frame-id", "follower_base_link",
                "--child-frame-id", "follower_laser_frame",
            ],
        ),
    ])
