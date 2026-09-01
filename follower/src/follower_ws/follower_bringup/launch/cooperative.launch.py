"""협조 위치추정: map -> follower_odom 보정.

Publishes:
    TF  map -> follower_odom

먼저 떠 있어야 하는 것:
    camera.launch.py       영상
    aruco_pose.launch.py   T_C_A (두 해 모두)
    VIO                    follower_odom -> follower_base_link
    메인 로봇              /amcl_pose 와 /map (읽기 전용)

config/main_robot.yaml 의 마커 장착값이 <CONFIGURE> 면 기동을 거부한다.
메인 스택을 켜고 scripts/25_resolve_marker_mount.py 로 채울 것.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    arguments = [
        # 메인 로봇의 자세 토픽. 네임스페이스가 없으므로 절대 이름이다.
        DeclareLaunchArgument("main_pose_topic", default_value="/amcl_pose"),
        # AMCL 은 메인 로봇이 움직일 때만 발행한다. 정지 중 묵은 자세를
        # 버리면 아무리 오래 서 있어도 출발을 못 하므로 기본은 제한 없음(0)이다.
        DeclareLaunchArgument("max_main_pose_age_s", default_value="0.0"),
    ]

    cooperative = Node(
        package="follower_localization",
        executable="cooperative_node",
        name="cooperative_localization",
        namespace="follower",
        output="screen",
        parameters=[{
            "main_pose_topic": LaunchConfiguration("main_pose_topic"),
            "max_main_pose_age_s": ParameterValue(
                LaunchConfiguration("max_main_pose_age_s"), value_type=float),
        }],
        emulate_tty=True,
    )

    return LaunchDescription([*arguments, cooperative])
