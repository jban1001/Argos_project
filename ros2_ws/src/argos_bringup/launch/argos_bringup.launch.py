#!/usr/bin/env python3

"""
ARGOS bringup

  argos_base_driver      : /cmd_vel -> 모터,  encoder -> /wheel_ticks/*
  wheel_odometry_node    : /wheel_ticks/* -> odometry
  mpu6050_node           : MPU6050 -> /imu/data_raw          (use_imu)
  ekf_filter_node        : wheel + gyro -> /odom + TF        (use_ekf)
  ydlidar_ros2_driver    : /scan_raw
  scan_normalizer        : /scan_raw -> /scan
  static_transform_pub   : base_link -> laser_frame  (실측값)
                           base_link -> imu_link     (use_imu)

두 가지 odometry 모드
---------------------
융합 모드 (use_imu:=true use_ekf:=true, 기본값)

    wheel encoder ─> /wheel/odom_raw ─┐
                                      ├─> EKF ─> /odom + odom->base_link
    MPU6050 ──────> /imu/data_raw ────┘

    wheel_odometry_node 는 TF 를 내지 않는다.

fallback 모드 (use_ekf:=false)

    wheel encoder ─> /odom + odom->base_link

    기존과 완전히 동일한 동작이다.

어느 모드에서도
    /odom 발행자            정확히 1개
    odom->base_link TF 발행자  정확히 1개

use_ekf:=true 인데 use_imu:=false 면?
------------------------------------
EKF 에 yaw rate 를 줄 센서가 없어진다. odom0 은 vx 만 융합하므로
yaw 가 영원히 갱신되지 않아 odometry 가 완전히 망가진다.
그래서 두 argument 를 AND 로 묶어, IMU 가 없으면 EKF 도 끄고
자동으로 fallback 으로 내려간다. 조용히 깨지는 것보다 낫다.

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
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LifecycleNode


ARGOS_CONFIG = os.path.expanduser("~/argos_project/config")

YDLIDAR_PARAMS = os.path.expanduser(
    "~/ydlidar_ws/src/ydlidar_ros2_driver/params/X4-Pro-ARGOS.yaml"
)


def _load_mount(filename, key, parent, child):
    """정적 TF 파라미터를 yaml 에서 읽어 CLI 인자로 만든다."""

    path = os.path.join(ARGOS_CONFIG, filename)

    with open(path) as f:
        data = yaml.safe_load(f)

    m = data[key]

    return [
        "--x", str(m["x"]),
        "--y", str(m["y"]),
        "--z", str(m["z"]),
        "--roll", str(m["roll"]),
        "--pitch", str(m["pitch"]),
        "--yaw", str(m["yaw"]),
        "--frame-id", parent,
        "--child-frame-id", child,
    ]


def load_lidar_mount():
    return _load_mount(
        "lidar_mount.yaml", "base_link_to_laser", "base_link", "laser_frame"
    )


def load_imu_mount():
    return _load_mount(
        "imu_mount.yaml", "base_link_to_imu", "base_link", "imu_link"
    )


def generate_launch_description():

    use_lidar = LaunchConfiguration("use_lidar")
    use_imu = LaunchConfiguration("use_imu")
    use_ekf = LaunchConfiguration("use_ekf")

    # EKF 는 IMU 가 있을 때만 의미가 있다. 둘을 AND 로 묶는다.
    fuse = PythonExpression(
        ["'", use_ekf, "' == 'true' and '", use_imu, "' == 'true'"]
    )

    odom_calibration = os.path.join(
        ARGOS_CONFIG, "odometry_calibration.yaml"
    )

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

    # ------------------------------------------------------------
    # wheel odometry : 모드에 따라 둘 중 하나만 뜬다
    # ------------------------------------------------------------
    #
    # 파라미터를 substitution 으로 바꿔 끼우는 것보다
    # 노드를 두 벌 선언하고 조건을 반대로 거는 편이
    # "지금 어느 모드인지" 가 훨씬 분명하다.

    wheel_odometry_fused = Node(
        package="argos_odometry",
        executable="wheel_odometry_node",
        name="wheel_odometry_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            odom_calibration,
            {
                "odom_topic": "/wheel/odom_raw",
                "publish_tf": False,
            },
        ],
        condition=IfCondition(fuse),
    )

    wheel_odometry_standalone = Node(
        package="argos_odometry",
        executable="wheel_odometry_node",
        name="wheel_odometry_node",
        output="screen",
        emulate_tty=True,
        parameters=[odom_calibration],
        condition=UnlessCondition(fuse),
    )

    # ------------------------------------------------------------
    # IMU
    # ------------------------------------------------------------

    imu = Node(
        package="argos_odometry",
        executable="mpu6050_node",
        name="mpu6050_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            os.path.join(ARGOS_CONFIG, "mpu6050.yaml")
        ],
        condition=IfCondition(use_imu),
    )

    imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_link_to_imu",
        output="screen",
        arguments=load_imu_mount(),
        condition=IfCondition(use_imu),
    )

    # ------------------------------------------------------------
    # EKF
    # ------------------------------------------------------------
    #
    # robot_localization 은 기본으로 odometry/filtered 를 낸다.
    # 기존 스택 전체가 /odom 을 보고 있으므로 그리로 remap 한다.

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            os.path.join(ARGOS_CONFIG, "ekf_imu.yaml"),
            {"use_sim_time": False},
        ],
        remappings=[("odometry/filtered", "/odom")],
        condition=IfCondition(fuse),
    )

    # ------------------------------------------------------------
    # LiDAR
    # ------------------------------------------------------------
    #
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
        DeclareLaunchArgument(
            "use_imu",
            default_value="true",
            description="MPU6050 IMU 노드와 base_link->imu_link TF 실행 여부",
        ),
        DeclareLaunchArgument(
            "use_ekf",
            default_value="true",
            description=(
                "robot_localization EKF 로 wheel + gyro 를 융합한다. "
                "false 면 wheel odometry 가 직접 /odom 과 TF 를 낸다. "
                "use_imu:=false 면 이 값과 무관하게 fallback 으로 내려간다."
            ),
        ),
        base_driver,
        wheel_odometry_fused,
        wheel_odometry_standalone,
        imu,
        imu_tf,
        ekf,
        lidar,
        scan_normalizer,
        laser_tf,
    ])
