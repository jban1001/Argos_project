#!/usr/bin/env python3

"""
ARGOS Nav2 navigation

  argos_localization.launch.py  (bringup + map_server + AMCL)
  controller_server             Regulated Pure Pursuit
  planner_server                SmacPlannerLattice (diff 모델)
  smoother_server
  behavior_server               spin / backup / wait
  bt_navigator
  waypoint_follower
  velocity_smoother

cmd_vel 흐름
------------
  controller_server ─┐
  behavior_server   ─┴─> /cmd_vel_nav ─> velocity_smoother ─> /cmd_vel
                                                                 │
                                                     argos_base_driver

nav2 기본 구성은 velocity_smoother 출력을 collision_monitor 가 받아서
/cmd_vel 을 내지만, 지금 단계에서는 collision_monitor 를 쓰지 않으므로
velocity_smoother 의 출력을 바로 /cmd_vel 로 remap 한다.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGOS_CONFIG = os.path.expanduser("~/argos_project/config")

LIFECYCLE_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
]


def generate_launch_description():

    bringup_share = get_package_share_directory("argos_bringup")

    params_file = LaunchConfiguration("nav2_params_file")
    nav_cmd_vel_topic = LaunchConfiguration("nav_cmd_vel_topic")
    map_yaml = LaunchConfiguration("map")
    use_imu = LaunchConfiguration("use_imu")
    use_ekf = LaunchConfiguration("use_ekf")

    common = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # 기본값은 기존과 동일한 /cmd_vel_nav 이다. 화재 순찰 통합 launch 는
    # Nav2 출력을 /cmd_vel_nav_auto 로 분리한 뒤 우선순위 중재 노드가
    # /cmd_vel_nav 으로 전달한다.
    to_nav = common + [("cmd_vel", nav_cmd_vel_topic)]

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_share, "launch", "argos_localization.launch.py"
            )
        ),
        launch_arguments={
            "map": map_yaml,
            "use_imu": use_imu,
            "use_ekf": use_ekf,
        }.items(),
    )

    def nav_node(pkg, exe, name, remappings):

        return Node(
            package=pkg,
            executable=exe,
            name=name,
            output="screen",
            emulate_tty=True,
            parameters=[params_file, {"use_sim_time": False}],
            remappings=remappings,
        )

    nodes = [
        nav_node("nav2_controller", "controller_server",
                 "controller_server", to_nav),
        nav_node("nav2_smoother", "smoother_server",
                 "smoother_server", common),
        nav_node("nav2_planner", "planner_server",
                 "planner_server", common),
        nav_node("nav2_behaviors", "behavior_server",
                 "behavior_server", to_nav),
        nav_node("nav2_bt_navigator", "bt_navigator",
                 "bt_navigator", common),
        nav_node("nav2_waypoint_follower", "waypoint_follower",
                 "waypoint_follower", common),
        nav_node(
            "nav2_velocity_smoother", "velocity_smoother",
            "velocity_smoother",
            common + [
                ("cmd_vel", "cmd_vel_nav"),
                ("cmd_vel_smoothed", "cmd_vel"),
            ],
        ),
    ]

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "autostart": True,
            "node_names": LIFECYCLE_NODES,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_imu",
            default_value="true",
            description="MPU6050 IMU 사용 여부",
        ),
        DeclareLaunchArgument(
            "use_ekf",
            default_value="true",
            description="wheel + gyro EKF 융합 사용 여부",
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=os.path.join(
                ARGOS_CONFIG, "nav2_params.yaml"
            ),
            description="Nav2 파라미터 파일",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.expanduser(
                "~/argos_project/maps/argos_lab.yaml"
            ),
            description="localization / navigation 에 사용할 저장 맵",
        ),
        DeclareLaunchArgument(
            "nav_cmd_vel_topic",
            default_value="cmd_vel_nav",
            description="controller / behavior server 출력 속도 토픽",
        ),
        localization,
        *nodes,
        lifecycle_manager,
    ])
