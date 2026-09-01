#!/usr/bin/env python3
"""Follower AMCL using the main robot's read-only /map topic.

This launch file intentionally does not start a map server.  Both robots must
localize against the same map, and only AMCL may publish map -> follower_odom.

The follower node and every robot-specific AMCL topic live below /follower.
Both robots share ROS_DOMAIN_ID=42, so the default /amcl node, /amcl_pose,
/particle_cloud and /initialpose names would otherwise collide with the main
robot.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_params = str(
        Path.home() / "lidar_overlay_ws" / "config" / "follower_amcl.yaml")

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="nav2_amcl",
            executable="amcl",
            namespace="follower",
            name="amcl",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=[
                ("scan", "/follower/scan"),
                ("map", "/map"),
                ("amcl_pose", "/follower/amcl_pose"),
                ("particle_cloud", "/follower/particle_cloud"),
                ("initialpose", "/follower/initialpose"),
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            namespace="follower",
            name="amcl_lifecycle_manager",
            output="screen",
            parameters=[{
                "autostart": True,
                # Relative to the manager's /follower namespace.  Using the
                # absolute name here makes the Nav2 bond ID disagree with the
                # lifecycle node even though the lifecycle services resolve.
                "node_names": ["amcl"],
                "use_sim_time": use_sim_time,
            }],
        ),
    ])


if __name__ == "__main__":
    generate_launch_description()
