#!/usr/bin/env python3

"""
ARGOS bringup

  argos_base_driver      : /cmd_vel -> 모터,  encoder -> /wheel_ticks/*
  wheel_odometry_node    : /wheel_ticks/* -> /odom + odom->base_link TF
  ydlidar_ros2_driver    : /scan
  static_transform_pub   : base_link -> laser_frame  (실측값)

주의
----
ydlidar_ros2_driver 의 ydlidar_launch.py 는
base_link -> laser_frame 을 (0, 0, 0.02, 회전 없음) 으로 발행한다.
ARGOS 의 실제 장착 위치와 다르므로 그 launch 를 사용하지 않고
여기서 드라이버 노드만 직접 실행하고 TF 는 실측값으로 발행한다.
"""

import os

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


ARGOS_CONFIG = os.path.expanduser("~/argos_project/config")

YDLIDAR_PARAMS = os.path.expanduser(
    "~/ydlidar_ws/src/ydlidar_ros2_driver/params/X4-Pro-ARGOS.yaml"
)


def load_lidar_mount():

    path = os.path.join(ARGOS_CONFIG, "lidar_mount.yaml")

    with open(path) as f:
        data = yaml.safe_load(f)

    m = data["base_link_to_laser"]

    return [
        "--x", str(m["x"]),
        "--y", str(m["y"]),
        "--z", str(m["z"]),
        "--roll", str(m["roll"]),
        "--pitch", str(m["pitch"]),
        "--yaw", str(m["yaw"]),
        "--frame-id", "base_link",
        "--child-frame-id", "laser_frame",
    ]


def generate_launch_description():

    use_lidar = LaunchConfiguration("use_lidar")

    base_driver = Node(
        package="argos_odometry",
        executable="argos_base_driver",
        name="argos_base_driver",
        output="screen",
        emulate_tty=True,
        parameters=[
            os.path.join(ARGOS_CONFIG, "base_driver.yaml")
        ],
    )

    wheel_odometry = Node(
        package="argos_odometry",
        executable="wheel_odometry_node",
        name="wheel_odometry_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            os.path.join(ARGOS_CONFIG, "odometry_calibration.yaml")
        ],
    )

    # 드라이버 출력은 /scan_raw 로 받고, scan_normalizer 가 /scan 을 낸다.
    # 이유는 scan_normalizer.py 상단 주석 참고.
    lidar = LifecycleNode(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node",
        namespace="/",
        output="screen",
        emulate_tty=True,
        parameters=[YDLIDAR_PARAMS],
        remappings=[("scan", "scan_raw")],
        condition=IfCondition(use_lidar),
    )

    scan_normalizer = Node(
        package="argos_odometry",
        executable="scan_normalizer",
        name="scan_normalizer",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "input_topic": "/scan_raw",
            "output_topic": "/scan",
            "num_bins": 1440,
            "stamp_at_midpoint": True,
        }],
        condition=IfCondition(use_lidar),
    )

    laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_link_to_laser",
        output="screen",
        arguments=load_lidar_mount(),
        condition=IfCondition(use_lidar),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_lidar",
            default_value="true",
            description="YDLIDAR 노드 실행 여부",
        ),
        base_driver,
        wheel_odometry,
        lidar,
        scan_normalizer,
        laser_tf,
    ])
