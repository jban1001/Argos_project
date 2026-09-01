"""OpenVINS 와 프레임 이어붙이기.

Publishes:
    TF  follower_odom -> global -> imu -> follower_base_link
    /odomimu   nav_msgs/Odometry   (OpenVINS 원본)

왜 정적 변환이 필요한가
-----------------------
OpenVINS 는 프레임 이름이 소스에 하드코딩돼 있다 (ROS2Visualizer.cpp):

    global -> imu -> cam0

우리 설계는 follower_odom -> follower_base_link 다. 이름을 바꾸려면 소스를
고쳐야 하는데, 업스트림을 건드리면 다음 갱신 때 되돌아간다. 대신 정적
변환으로 이어붙인다:

    follower_odom -> global            항등 (같은 원점을 다르게 부르는 것)
    imu -> follower_base_link          base_to_imu 의 역

그러면 map -> follower_odom -> global -> imu -> follower_base_link 로 트리가
하나로 이어지고, 각 프레임의 부모가 하나씩이라 TF 규칙을 지킨다.

메인 로봇과는 이름이 겹치지 않는다 (그쪽은 map/odom/base_link/laser_frame).
같은 도메인을 쓰므로 이건 우연이 아니라 확인해야 하는 사항이다.

값은 config/follower_frames.yaml 에서 읽는다. 여기 숫자를 적으면 두 벌이
되어 갈라진다.
"""

import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPO = Path.home() / "follower_ws"
sys.path.insert(0, str(REPO / "src" / "follower_localization"))


def imu_to_base_arguments() -> list[str]:
    """imu -> follower_base_link 를 static_transform_publisher 인자로."""
    from follower_localization.frames import base_to_imu
    from follower_localization.transforms import inverse, split_transform

    translation, quaternion = split_transform(
        inverse(base_to_imu(REPO / "config" / "follower_frames.yaml")))
    return ["--x", f"{translation[0]:.6f}",
            "--y", f"{translation[1]:.6f}",
            "--z", f"{translation[2]:.6f}",
            "--qx", f"{quaternion[0]:.6f}",
            "--qy", f"{quaternion[1]:.6f}",
            "--qz", f"{quaternion[2]:.6f}",
            "--qw", f"{quaternion[3]:.6f}",
            "--frame-id", "imu",
            "--child-frame-id", "follower_base_link"]


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument(
            "config_path",
            default_value=str(REPO / "config" / "openvins" / "estimator_config.yaml"),
            description="OpenVINS estimator_config.yaml. scripts/22 로 생성한다"),
        DeclareLaunchArgument(
            # INFO 면 ZUPT/TIME 줄이 100 Hz 로 쏟아져 추종 상태 로그가 전부
            # 묻힌다. 실제로 그것 때문에 "왜 안 따라가는가" 를 며칠 못 봤다.
            # VIO 를 파야 할 때만 INFO 로 올릴 것: vio_verbosity:=INFO
            "verbosity", default_value="WARNING",
            description="OpenVINS 로그 수준"),
    ]

    openvins = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        name="ov_msckf",
        output="screen",
        parameters=[{
            "verbosity": LaunchConfiguration("verbosity"),
            "config_path": LaunchConfiguration("config_path"),
        }],
        emulate_tty=True,
    )

    # follower_odom 과 global 은 같은 원점을 다르게 부르는 것이다.
    odom_to_global = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="follower_odom_to_global",
        arguments=["--frame-id", "follower_odom", "--child-frame-id", "global"])

    imu_to_base = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="imu_to_follower_base_link",
        arguments=imu_to_base_arguments())

    return LaunchDescription([*arguments, openvins, odom_to_global, imu_to_base])
