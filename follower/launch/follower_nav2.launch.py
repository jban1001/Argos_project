#!/usr/bin/env python3
"""팔로워 Nav2 -- 지도 좌표로 이동한다.

    /map (메인 로봇)  ─┐
    /follower/scan    ─┼─> costmaps ─> planner ─> controller ─> /cmd_vel
    AMCL (map->odom)  ─┘                                          │
                                                                  v
                                        velocity_smoother ─> /cmd_vel_smoothed
                                                                  │
                                                     cmd_vel_bridge
                                                                  │
                                                 /follower/motor_command  "C,pwm,dps"

여기서 띄우지 않는 것과 그 이유
------------------------------
map_server   메인 로봇이 /map 을 낸다.  여기서 또 띄우면 두 개가 경쟁한다.
AMCL         launch/follower_amcl.launch.py 가 소유한다.  프리체크가 따로 있다.
라이다/EKF   tools/start_localization_stack.sh 가 띄운다.
             이 런치는 그 위에 얹는 것이지 대체가 아니다.

주행 한계는 config/follower_nav2.yaml 에 있고 전부 이 차체 실측이다.
cmd_vel_bridge 의 PWM 대응도 실측이며, 미설정이면 다리가 주행 명령을
거부한다 (지어낸 기본값을 두지 않는다).
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# tools/measure_drive_scale.py, 2026-08-31 실측.
PWM_PER_MPS = 700.8
PWM_INTERCEPT = 22.9
MIN_MOVING_PWM = 80


def generate_launch_description() -> LaunchDescription:
    root = Path.home() / "lidar_overlay_ws"
    params = LaunchConfiguration("params_file")

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "velocity_smoother",
    ]

    servers = [
        Node(package="nav2_controller", executable="controller_server",
             namespace="follower", name="controller_server", output="screen", parameters=[params]),
        Node(package="nav2_planner", executable="planner_server",
             namespace="follower", name="planner_server", output="screen", parameters=[params]),
        Node(package="nav2_behaviors", executable="behavior_server",
             namespace="follower", name="behavior_server", output="screen", parameters=[params]),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             namespace="follower", name="bt_navigator", output="screen", parameters=[params]),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother",
             namespace="follower", name="velocity_smoother", output="screen", parameters=[params]),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=str(root / "config" / "follower_nav2.yaml")),
        DeclareLaunchArgument("autostart", default_value="true"),

        *servers,

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            namespace="follower",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "autostart": LaunchConfiguration("autostart"),
                "node_names": lifecycle_nodes,
                "bond_timeout": 10.0,
            }],
        ),

        # Nav2 의 속도 명령을 MCU 문법으로 옮긴다.
        # 이 스크립트는 ROS 패키지가 아니라 tools/ 의 단독 파일이라
        # Node 가 아니라 ExecuteProcess 로 띄운다.
        ExecuteProcess(
            name="cmd_vel_bridge",
            output="screen",
            cmd=[
                "python3",
                str(root / "tools" / "cmd_vel_bridge.py"),
                "--ros-args",
                "-p", "cmd_vel_topic:=/follower/cmd_vel_smoothed",
                "-p", "command_topic:=/follower/motor_command",
                # 타입(stamped 여부)은 다리가 알아서 둘 다 구독한다
                "-p", f"pwm_per_mps:={PWM_PER_MPS}",
                "-p", f"pwm_intercept:={PWM_INTERCEPT}",
                "-p", f"min_moving_pwm:={MIN_MOVING_PWM}",
                "-p", "max_pwm:=160",
                # 펌웨어는 +-90 deg/s 를 받지만 실측 최고가 60.7 이다.
                "-p", "max_yaw_rate_dps:=60.0",
            ],
        ),
    ])


if __name__ == "__main__":
    generate_launch_description()
