"""마커 자세 발행 노드.

Publishes:
    /follower/aruco/pose      PoseWithCovarianceStamped  가장 잘 맞는 해
    /follower/aruco/pose_alt  PoseWithCovarianceStamped  두 번째 해
    /follower/aruco/status    std_msgs/String (JSON)
    TF  follower_camera -> main_marker

camera.launch.py 가 먼저 떠 있어야 한다 -- 내부 파라미터를 camera_info
토픽에서 받으므로, 카메라가 없으면 이미지가 와도 pose 를 내지 않는다.

config/aruco.yaml 의 임계값이 아직 <CONFIGURE> 면 노드가 기동을 거부한다.
scripts/26_characterize_aruco.py 로 실측한 뒤 채울 것.
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
            "params_file",
            default_value=str(share / "config" / "aruco.yaml")),
    ]

    aruco = Node(
        package="follower_localization",
        executable="aruco_pose_node",
        name="aruco_pose",
        namespace="follower",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        emulate_tty=True,
    )

    return LaunchDescription([*arguments, aruco])
