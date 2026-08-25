#!/usr/bin/env python3

"""Launch ARGOS navigation with the MPPI controller profile.

The original argos_navigation.launch.py and nav2_params.yaml continue to use
Regulated Pure Pursuit, providing an immediate rollback path for hardware tests.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGOS_CONFIG = os.path.expanduser("~/argos_project/config")


def generate_launch_description():
    bringup_share = get_package_share_directory("argos_bringup")
    map_yaml = LaunchConfiguration("map")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_share,
                "launch",
                "argos_navigation.launch.py",
            )
        ),
        launch_arguments={
            "nav2_params_file": os.path.join(
                ARGOS_CONFIG,
                "nav2_params_mppi.yaml",
            ),
            "map": map_yaml,
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value=os.path.expanduser(
                "~/argos_project/maps/argos_lab.yaml"
            ),
            description="Navigation map yaml",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz with the Nav2 Goal tool",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                bringup_share,
                "rviz",
                "argos_navigation.rviz",
            ),
            description="RViz configuration file",
        ),
        navigation,
        rviz,
    ])
