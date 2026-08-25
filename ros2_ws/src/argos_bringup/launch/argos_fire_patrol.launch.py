#!/usr/bin/env python3

"""ARGOS Nav2 자동 순찰 + 화재 접근 통합 실행."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGOS_ROOT = os.path.expanduser("~/argos_project")
ARGOS_CONFIG = os.path.join(ARGOS_ROOT, "config")


def generate_launch_description():

    bringup_share = get_package_share_directory("argos_bringup")

    map_yaml = LaunchConfiguration("map")
    nav2_params = LaunchConfiguration("nav2_params_file")
    fire_params = LaunchConfiguration("fire_params_file")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_imu = LaunchConfiguration("use_imu")
    use_ekf = LaunchConfiguration("use_ekf")

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_share,
                "launch",
                "argos_navigation.launch.py",
            )
        ),
        launch_arguments={
            "map": map_yaml,
            "nav2_params_file": nav2_params,
            "nav_cmd_vel_topic": "/cmd_vel_nav_auto",
            "use_imu": use_imu,
            "use_ekf": use_ekf,
        }.items(),
    )

    # venv 에 ultralytics / TensorRT 의존성이 있으므로 이 프로세스만
    # ~/.venv/bin/python 으로 실행한다. YOLO 폴더는 읽기만 한다.
    fire_patrol = ExecuteProcess(
        cmd=[
            os.path.expanduser("~/.venv/bin/python"),
            os.path.join(ARGOS_ROOT, "scripts", "fire_nav_patrol.py"),
            "--ros-args",
            "--params-file",
            fire_params,
        ],
        output="screen",
        emulate_tty=True,
        additional_env={"PYTHONUNBUFFERED": "1"},
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
            "use_imu",
            default_value="true",
            description="MPU6050 IMU 사용 여부",
        ),
        DeclareLaunchArgument(
            "use_ekf",
            default_value="true",
            description="wheel + gyro EKF 융합 사용 여부",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(
                ARGOS_ROOT,
                "maps",
                "argos_outdoor_v1.yaml",
            ),
            description="자동 순찰에 사용할 저장 맵",
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=os.path.join(ARGOS_CONFIG, "nav2_params.yaml"),
            description="Nav2 파라미터 파일 (기본 RPP)",
        ),
        DeclareLaunchArgument(
            "fire_params_file",
            default_value=os.path.join(
                ARGOS_CONFIG,
                "fire_nav_patrol.yaml",
            ),
            description="화재 순찰 파라미터 파일",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Nav2 RViz 실행",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                bringup_share,
                "rviz",
                "argos_navigation.rviz",
            ),
            description="RViz 설정",
        ),
        navigation,
        fire_patrol,
        rviz,
    ])
