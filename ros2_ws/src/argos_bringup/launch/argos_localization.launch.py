#!/usr/bin/env python3

"""
ARGOS localization (저장된 map 기반)

  argos_bringup.launch.py  (base driver + odometry + LiDAR + TF)
  nav2_map_server          저장된 map 발행
  nav2_amcl                map -> odom

TF chain:
  map -> odom                AMCL
  odom -> base_link          wheel_odometry_node
  base_link -> laser_frame   static (실측)

주의
----
slam_toolbox 와 동시에 실행하면 map -> odom 을 둘이 같이 쏴서
TF 가 깨진다. 반드시 SLAM 을 끄고 실행할 것.

map_server / amcl 은 lifecycle node 이므로
nav2_lifecycle_manager 로 configure/activate 시킨다.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGOS_CONFIG = os.path.expanduser("~/argos_project/config")


def generate_launch_description():

    bringup_share = get_package_share_directory("argos_bringup")

    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_share, "launch", "argos_bringup.launch.py"
            )
        )
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        emulate_tty=True,
        parameters=[
            params_file,
            {"yaml_filename": map_yaml},
        ],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        emulate_tty=True,
        parameters=[params_file],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "autostart": True,
            "node_names": ["map_server", "amcl"],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(ARGOS_CONFIG, "amcl.yaml"),
            description="map_server / amcl 파라미터 파일",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.expanduser(
                "~/argos_project/maps/argos_lab.yaml"
            ),
            description="사용할 map yaml",
        ),
        bringup,
        map_server,
        amcl,
        lifecycle_manager,
    ])
