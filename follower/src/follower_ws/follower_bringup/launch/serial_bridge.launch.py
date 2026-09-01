"""Follower MCU bridge bring-up.

Publishes:
    /follower/imu/data_raw   sensor_msgs/Imu   frame follower_imu_link, ~200 Hz

The namespace is not decoration. The node publishes "imu/data_raw" relative to
its namespace, so running it bare with `ros2 run` puts the IMU on
/imu/data_raw while the camera sits under /follower/. Everything downstream --
scripts/10_allan_variance.py, the OpenVINS config, the recording commands in
CALIBRATION.md -- is written against /follower/imu/data_raw.
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
            default_value=str(share / "config" / "serial_bridge.yaml")),
    ]

    bridge = Node(
        package="follower_serial_bridge",
        executable="serial_bridge_node",
        name="serial_bridge",
        namespace="follower",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        emulate_tty=True,
    )

    return LaunchDescription([*arguments, bridge])
