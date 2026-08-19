#!/usr/bin/env python3

"""
ARGOS SLAM

  argos_bringup.launch.py  (base driver + odometry + LiDAR + TF)
  slam_toolbox             (online async mapping)

TF chain:
  map -> odom                slam_toolbox
  odom -> base_link          wheel_odometry_node
  base_link -> laser_frame   static (실측)

주의
----
Jazzy 의 slam_toolbox 는 LifecycleNode 다.
평범한 Node 로 띄우면 프로세스는 살아있지만 configure/activate 가
안 되어 /scan 구독조차 생기지 않고 /map 도 나오지 않는다.
따라서 lifecycle 전이를 처리해 주는 upstream 의
online_async_launch.py 를 그대로 include 한다.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


ARGOS_CONFIG = os.path.expanduser("~/argos_project/config")


def generate_launch_description():

    bringup_share = get_package_share_directory("argos_bringup")
    slam_share = get_package_share_directory("slam_toolbox")

    slam_params = LaunchConfiguration("slam_params_file")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_share, "launch", "argos_bringup.launch.py"
            )
        )
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                slam_share, "launch", "online_async_launch.py"
            )
        ),
        launch_arguments={
            "slam_params_file": slam_params,
            "use_sim_time": "false",
            "autostart": "true",
            "use_lifecycle_manager": "false",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=os.path.join(
                ARGOS_CONFIG, "slam_toolbox.yaml"
            ),
            description="slam_toolbox 파라미터 파일",
        ),
        bringup,
        slam,
    ])
