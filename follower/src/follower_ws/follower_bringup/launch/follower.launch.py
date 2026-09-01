"""팔로워 전체 기동 (Phase 10).

기본은 센서만 띄운다. 나머지는 켤 때 명시적으로 켠다 -- 아직 실측이 끝나지
않은 노드들은 어차피 기동을 거부하고, 모터 명령은 준비됐을 때만 나가야 한다.

    ros2 launch follower_bringup follower.launch.py
    ros2 launch follower_bringup follower.launch.py vio:=true
    ros2 launch follower_bringup follower.launch.py vio:=true aruco:=true \\
        cooperative:=true follow:=true

무엇이 무엇을 필요로 하는가
---------------------------
    camera        (없음)
    serial_bridge (없음)
    vio           camera + serial_bridge
    aruco         camera            -- config/aruco.yaml 임계값 실측 필요
    cooperative   aruco + vio + 메인 로봇 /amcl_pose
                                    -- config/main_robot.yaml 마커 장착값 필요
    follow        cooperative + 메인 로봇 /amcl_pose

실측이 안 끝난 노드는 기동을 거부하고 무엇을 재야 하는지 알려준다. 기본값을
지어내고 도는 것보다 낫다 -- 그러면 시스템은 도는 것처럼 보이는데 결과가
조용히 틀린다.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("follower_bringup"))

    arguments = [
        DeclareLaunchArgument(
            "camera", default_value="true",
            description="Logitech BRIO. 노출/게인/초점을 잠그고 mono8 30 Hz 로 발행"),
        DeclareLaunchArgument(
            "serial_bridge", default_value="true",
            description="Arduino Mega. IMU 200 Hz 와 모터 명령 전달"),
        DeclareLaunchArgument(
            "vio", default_value="false",
            description="OpenVINS. follower_odom -> follower_base_link"),
        DeclareLaunchArgument(
            "aruco", default_value="false",
            description="마커 자세 T_C_A. aruco.yaml 임계값 실측이 필요하다"),
        DeclareLaunchArgument(
            "cooperative", default_value="false",
            description="map -> follower_odom 보정. 마커 장착값과 메인 로봇이 필요하다"),
        DeclareLaunchArgument(
            "follow", default_value="false",
            description="추종 제어. 명령 발행은 publish_commands 로 따로 켠다"),
        # 모터 명령은 기본적으로 발행하지 않는다. 실수로 바퀴가 도는 일이
        # 없어야 한다.
        DeclareLaunchArgument(
            "publish_commands", default_value="false",
            description="모터 명령을 실제로 발행할지. 기본은 로그만 낸다"),
        DeclareLaunchArgument(
            "vio_verbosity", default_value="WARNING",
            description="OpenVINS 로그 수준. INFO 로 올리면 추종 로그가 묻힌다"),
        DeclareLaunchArgument(
            "main_pose_topic", default_value="/amcl_pose",
            description="메인 로봇의 map 기준 자세 토픽"),
    ]

    def include(name: str, condition_arg: str, extra=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / f"{name}.launch.py")),
            condition=IfCondition(LaunchConfiguration(condition_arg)),
            launch_arguments=(extra or {}).items())

    # params_file 을 명시적으로 넘긴다. 넘기지 않으면 먼저 포함된 런치가
    # 선언한 같은 이름의 설정이 스코프에 남아, 뒤에 오는 런치의
    # DeclareLaunchArgument 기본값이 무시된다. 실제로 serial_bridge 와
    # aruco 가 camera.yaml 을 받고 있었다.
    sensors = [
        include("camera", "camera",
                {"params_file": str(share / "config" / "camera.yaml")}),
        include("serial_bridge", "serial_bridge",
                {"params_file": str(share / "config" / "serial_bridge.yaml")}),
    ]

    # VIO 는 camera_info 와 IMU 가 흐르기 시작한 뒤에 붙어야 한다. 먼저 뜨면
    # 초기화 구간에서 빈 큐를 보고 발산 상태로 시작할 수 있다.
    vio = TimerAction(period=6.0, actions=[
        include("vio", "vio",
                {"verbosity": LaunchConfiguration("vio_verbosity")})])

    # ArUco 도 camera_info 를 받아야 pose 를 낸다.
    # params_file 을 명시적으로 넘긴다. 넘기지 않으면 camera.launch.py 가
    # 먼저 선언한 같은 이름의 설정이 스코프에 남아, aruco_pose.launch.py 의
    # DeclareLaunchArgument 기본값이 무시되고 camera.yaml 이 들어간다.
    aruco = TimerAction(period=6.0, actions=[
        include("aruco_pose", "aruco",
                {"params_file": str(share / "config" / "aruco.yaml")})])

    # 협조 위치추정은 ArUco 와 VIO 가 다 떠야 의미가 있다.
    cooperative = TimerAction(period=10.0, actions=[
        include("cooperative", "cooperative",
                {"main_pose_topic": LaunchConfiguration("main_pose_topic")})])

    follow = TimerAction(period=12.0, actions=[
        include("follow", "follow",
                {"params_file": str(share / "config" / "follow.yaml"),
                 "publish_commands": LaunchConfiguration("publish_commands"),
                 "main_pose_topic": LaunchConfiguration("main_pose_topic")})])

    return LaunchDescription([*arguments, *sensors, vio, aruco, cooperative, follow])
