"""추종 제어 (Phase 8~9).

Publishes:
    /follower/motor_command   std_msgs/String   (config 기본값: 발행 안 함)
    /follower/follow/state    std_msgs/String (JSON)  상태와 만들어진 명령

config/follow.yaml 의 publish_commands 가 false 면 명령을 만들기만 하고
로그로 낸다. 바퀴를 돌릴 준비가 됐을 때만 켤 것:

    ros2 launch follower_bringup follow.launch.py publish_commands:=true
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("follower_bringup"))

    arguments = [
        DeclareLaunchArgument(
            "params_file", default_value=str(share / "config" / "follow.yaml")),
        DeclareLaunchArgument("publish_commands", default_value="false"),
        DeclareLaunchArgument("main_pose_topic", default_value="/amcl_pose"),
    ]

    controller = Node(
        package="follower_localization",
        executable="follow_controller_node",
        name="follow_controller",
        namespace="follower",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"publish_commands": LaunchConfiguration("publish_commands"),
             "main_pose_topic": LaunchConfiguration("main_pose_topic")},
        ],
        emulate_tty=True,
    )

    return LaunchDescription([*arguments, controller])
