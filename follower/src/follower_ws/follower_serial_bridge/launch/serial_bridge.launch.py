"""Bring up the follower serial bridge inside the /follower namespace.

Namespacing is not cosmetic here: the main robot already owns /map, /odom and
base_link, and spec section 32 forbids the two robots sharing those names.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("follower_serial_bridge"))
    default_params = share / "config" / "serial_bridge.yaml"

    params_argument = DeclareLaunchArgument(
        "params_file",
        default_value=str(default_params),
        description="Parameter YAML for the serial bridge",
    )
    port_argument = DeclareLaunchArgument(
        "port", default_value="/dev/ttyACM0", description="Arduino serial port"
    )

    bridge = Node(
        package="follower_serial_bridge",
        executable="serial_bridge_node",
        name="follower_serial_bridge",
        namespace="follower",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"port": LaunchConfiguration("port")},
        ],
        emulate_tty=True,
    )

    return LaunchDescription([params_argument, port_argument, bridge])
