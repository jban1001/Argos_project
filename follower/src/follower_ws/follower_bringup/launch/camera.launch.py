"""Follower camera bring-up.

Publishes:
    /follower/camera/image_raw    sensor_msgs/Image        mono8, 640x480
    /follower/camera/camera_info  sensor_msgs/CameraInfo

The control lock runs a few seconds AFTER the camera node, because several
V4L2 controls only become writable once the device is streaming and their
"automatic" partner has been cleared.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("follower_bringup"))

    arguments = [
        DeclareLaunchArgument("params_file", default_value=str(share / "config" / "camera.yaml")),
        DeclareLaunchArgument("device", default_value="/dev/video0"),
        # Empty means "use whatever camera.yaml says". Only pass these to
        # override the YAML for a one-off experiment.
        DeclareLaunchArgument("focus", default_value=""),
        DeclareLaunchArgument("exposure", default_value=""),
    ]

    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera",
        namespace="follower",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"video_device": LaunchConfiguration("device")},
        ],
        # v4l2_camera publishes "image_raw"/"camera_info" relative to its
        # namespace, which would give /follower/image_raw. Group them under a
        # camera/ prefix so the follower's sensors stay tidy and so image
        # transport plugins find image_raw and camera_info side by side.
        remappings=[
            ("image_raw", "camera/image_raw"),
            ("camera_info", "camera/camera_info"),
            ("image_raw/compressed", "camera/image_raw/compressed"),
        ],
        emulate_tty=True,
    )

    lock = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "follower_bringup", "lock_camera_controls",
                    "--device", LaunchConfiguration("device"),
                    "--params-file", LaunchConfiguration("params_file"),
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([*arguments, camera, lock])
