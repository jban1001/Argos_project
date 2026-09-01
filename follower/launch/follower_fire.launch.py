#!/usr/bin/env python3
"""Fire supervisor with optional underlay follow controller and MCU bridge.

This launch intentionally does not start AMCL, LiDAR, camera, ArUco or VIO.
Those sensors have their own measured bring-up and prechecks.  Start this only
after their topics/TF are healthy.  The follow controller is forced to dry-run;
the supervisor is the sole publisher that may forward actuator commands.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    root = Path.home() / "lidar_overlay_ws"
    follow_share = Path(get_package_share_directory("follower_bringup"))

    params_file = LaunchConfiguration("params_file")
    follow_params_file = LaunchConfiguration("follow_params_file")
    enable_motion = LaunchConfiguration("enable_motion")
    enable_pump = LaunchConfiguration("enable_pump")
    initial_mode = LaunchConfiguration("initial_mode")
    nav2_action = LaunchConfiguration("nav2_action")

    arguments = [
        DeclareLaunchArgument(
            "params_file", default_value=str(root / "config" / "follower_fire.yaml")),
        DeclareLaunchArgument(
            "follow_params_file",
            default_value=str(follow_share / "config" / "follow.yaml")),
        DeclareLaunchArgument("start_follow_controller", default_value="true"),
        DeclareLaunchArgument("start_serial_bridge", default_value="false"),
        DeclareLaunchArgument("enable_motion", default_value="false"),
        DeclareLaunchArgument("enable_pump", default_value="false"),
        DeclareLaunchArgument("initial_mode", default_value="auto"),
        DeclareLaunchArgument(
            "nav2_action", default_value="/follower/navigate_to_pose"),
    ]

    follow = Node(
        package="follower_localization",
        executable="follow_controller_node",
        namespace="follower",
        name="follow_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_follow_controller")),
        parameters=[
            follow_params_file,
            {"publish_commands": False, "main_pose_topic": "/amcl_pose"},
        ],
    )

    bridge = Node(
        package="follower_serial_bridge",
        executable="serial_bridge_node",
        namespace="follower",
        name="serial_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_serial_bridge")),
        parameters=[params_file],
    )

    supervisor = Node(
        package="follower_fire_control",
        executable="fire_supervisor_node",
        namespace="follower",
        name="fire_supervisor",
        output="screen",
        parameters=[
            params_file,
            {
                "enable_motion": ParameterValue(enable_motion, value_type=bool),
                "enable_pump": ParameterValue(enable_pump, value_type=bool),
                "initial_mode": initial_mode,
                "nav2_action": nav2_action,
            },
        ],
    )

    return LaunchDescription([*arguments, follow, bridge, supervisor])


if __name__ == "__main__":
    generate_launch_description()
