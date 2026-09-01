#!/usr/bin/env python3
"""라이다 + IMU 기반 팔로워 위치추정.  카메라는 쓰지 않는다.

    scan_odom.py (라이다 키프레임 정합) ─┐
                                          ├─ EKF ─> follower_odom -> follower_base_link
    IMU (MPU6050 각속도)                ─┘

카메라(VIO)를 쓰지 않는 이유 (2026-08-31 실측)
---------------------------------------------
OpenVINS 는 calib_cam_* 를 끄자 발산(600 초 1,261 km)은 멈췄지만, 정지 300 초에
여전히 2.68 m 를 흘렀고 위치가 고정인 구간에서도 |v| 0.156 m/s 를 보고했다.
AMCL 은 오도메트리가 움직였다고 말할 때만 갱신하므로, 멈춘 로봇이 "움직였다"고
하면 없는 운동으로 파티클이 퍼진다. 화재 시나리오는 정지 구간이 필수다.
초기화에 사람이 "2 초 흔들고 5 초 정지"를 해줘야 하는 것도 운용이 안 된다.

rf2o 가 아니라 scan_odom.py 인 이유
-----------------------------------
rf2o 는 이 센서에서 실패했다 (정지 시 회전 추정 sigma 112 deg). 다만 원인은
라이다가 아니라 **연속 프레임 정합**이었다. 이 센서는 스캔마다 각도 격자가
밀리고(increment 1.691~1.723 deg), 1 bin = 1.73 deg 가 매 스텝 무작위 걸음으로
쌓인다.  1.73 x sqrt(3270) = 99 deg 로 실측과 맞는다.
scan_odom.py 는 키프레임에 맞춰 그 누적을 끊는다.

    정지 300 초 위치 오차
      VIO(calib 켬)         513,545 m
      VIO(calib 끔)             2.68 m
      스캔정합(연속 프레임)     1.14 m
      스캔정합(키프레임)        0.011 m   <- 이 구성

카메라는 ArUco 마커 추종에만 쓴다. 마커 직접 추종은 마커 하나로 끝나는
계산이라 VIO 가 필요 없다 (follower_ws/README.md 7 절).

먼저 떠 있어야 하는 것:
    ld08_driver              /follower/scan_raw
    lidar_mask.py            /follower/scan
    follower_lidar_tf        follower_base_link -> follower_laser_frame
    serial_bridge            /follower/imu/data_raw

scan_odom.py 는 TF 를 발행하지 않는다. follower_odom -> follower_base_link 는
EKF 가 단독으로 소유한다. 둘 다 내면 부모가 둘이 되어 트리가 깨진다.
"""

import math
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ekf_params = str(
        Path.home() / "lidar_overlay_ws" / "config" / "follower_ekf.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("ekf_params_file", default_value=ekf_params),

        # base_link -> IMU.  값은 follower_ws/config/follower_frames.yaml 의
        # 실측값(앞 -0.020, 좌 +0.015, 위 +0.080, yaw -87.20 deg)이다.
        # 그 파일은 읽기만 하고 고치지 않는다.  EKF 가 IMU 를 쓰려면 이
        # 프레임이 트리에 있어야 하는데, follower_ws 의 런치는 OpenVINS 용
        # 이름('imu')으로 다른 사슬을 만들므로 여기서 따로 낸다.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="follower_imu_static_tf",
            output="screen",
            arguments=[
                "--x", "-0.020",
                "--y", "0.015",
                "--z", "0.080",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", str(math.radians(-87.20)),
                "--frame-id", "follower_base_link",
                "--child-frame-id", "follower_imu_link",
            ],
        ),

        # 라이다 정합 오도메트리.  TF 는 EKF 가 소유하므로 여기서는 끈다.
        ExecuteProcess(
            cmd=["python3", str(Path.home() / "lidar_overlay_ws" / "scan_odom.py")],
            output="screen",
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[LaunchConfiguration("ekf_params_file")],
        ),
    ])


if __name__ == "__main__":
    generate_launch_description()
