"""추종 제어 노드. 모터는 돌리지 않고 명령 문자열만 확인한다."""

import json
import math

import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.parameter import Parameter
from rclpy.time import Time

from follower_localization.cooperative_node import matrix_to_transform
from follower_localization.follow_controller_node import FollowController
from follower_localization.transforms import make_transform


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_node(**overrides):
    params = {
        "publish_commands": False,
        "max_throttle": 60,
        "control_rate_hz": 1000.0,        # 시험에서는 타이머를 직접 부른다
    }
    params.update(overrides)
    return FollowController(parameter_overrides=[
        Parameter(name, value=value) for name, value in params.items()])


def main_pose(x, y, seconds=100.0):
    message = PoseWithCovarianceStamped()
    message.header.stamp = Time(nanoseconds=int(seconds * 1e9)).to_msg()
    message.pose.pose.position.x = float(x)
    message.pose.pose.position.y = float(y)
    message.pose.pose.orientation.w = 1.0
    return message


def place_follower(node, x, y, yaw_deg=0.0):
    angle = math.radians(yaw_deg)
    matrix = make_transform([x, y, 0.0],
                            [0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)])
    stamp = node.get_clock().now().to_msg()
    node._buffer.set_transform(
        matrix_to_transform(matrix, stamp, "map", "follower_base_link"), "test")


def collect(node):
    commands, states = [], []
    node._command_pub.publish = lambda m: commands.append(m.data)
    node._state_pub.publish = lambda m: states.append(json.loads(m.data))
    return commands, states


def test_refuses_throttle_above_the_hardware_limit(ros):
    """설정 실수로 MCU 가 거부하는 명령을 계속 보내면 로봇은 멈춰 있고
    원인은 안 보인다."""
    with pytest.raises(ValueError) as caught:
        make_node(max_throttle=200)
    assert "180" in str(caught.value)


def test_refuses_yaw_rate_above_the_hardware_limit(ros):
    with pytest.raises(ValueError):
        make_node(max_yaw_rate_dps=120.0)


def test_does_not_publish_by_default(ros):
    """기본값은 발행 안 함. 실수로 바퀴가 도는 일이 없어야 한다."""
    node = make_node()
    commands, states = collect(node)
    node._tick()
    node.destroy_node()
    assert commands == [], "publish_commands 가 false 인데 발행했다"
    assert states, "상태는 발행돼야 한다"
    assert states[-1]["published"] is False


def test_publishes_when_explicitly_enabled(ros):
    node = make_node(publish_commands=True)
    commands, _ = collect(node)
    node._tick()
    node.destroy_node()
    assert commands, "켰는데 발행하지 않았다"


def test_stops_without_a_pose(ros):
    """자세를 모르면 조향할 수 없다. 정지가 유일하게 안전하다."""
    node = make_node(publish_commands=True)
    commands, _ = collect(node)
    node._tick()
    node.destroy_node()
    assert commands[-1] == "S"


def test_stops_when_the_trajectory_is_too_short(ros):
    node = make_node(publish_commands=True)
    commands, _ = collect(node)
    node._on_main_pose(main_pose(1.0, 0.0))
    place_follower(node, 0.0, 0.0)
    node._tick()
    node.destroy_node()
    assert commands[-1] == "S"


def build_trajectory(node, length_m=3.0, step=0.05):
    points = int(length_m / step)
    for index in range(points):
        node._on_main_pose(main_pose(index * step, 0.0))


def test_produces_a_drive_command_when_following(ros):
    """궤적이 쌓이고 자세를 알면 주행 명령이 나와야 한다."""
    node = make_node(publish_commands=True)
    commands, states = collect(node)
    build_trajectory(node)
    node._aruco_last_seen = node._now()
    place_follower(node, 0.5, 0.0)
    node._tick()
    node.destroy_node()

    assert commands, "명령이 없다"
    assert commands[-1].startswith("C,") or commands[-1] == "S", commands[-1]
    assert states[-1]["trajectory_points"] > 10


def test_command_is_only_ever_a_known_form(ros):
    """브리지가 검사하지만, 여기서 이상한 문자열이 나오면 안 된다."""
    node = make_node(publish_commands=True)
    commands, _ = collect(node)
    build_trajectory(node)
    node._aruco_last_seen = node._now()
    for x in (0.2, 0.8, 1.4, 2.0):
        place_follower(node, x, 0.1, yaw_deg=10.0)
        node._tick()
    node.destroy_node()

    for command in commands:
        assert command == "S" or command.startswith("C,"), command
        if command.startswith("C,"):
            _, throttle, yaw = command.split(",")
            assert abs(int(throttle)) <= 60, command
            assert abs(float(yaw)) <= 45.0, command


def test_trajectory_target_is_behind_the_main_robot(ros):
    """메인을 향해 직진하면 코너를 잘라 들어간다. 지나간 경로를 따라야 한다.

    궤적 추종은 마커를 놓쳤을 때의 보조 경로다. 마커가 보이면 직접 추종이
    먼저이므로, 여기서는 마커를 낡게 두어 보조 경로를 시험한다.
    """
    node = make_node()
    _, states = collect(node)
    build_trajectory(node, length_m=3.0)
    # 마커를 놓친 직후: aruco_fresh_s 는 넘고 aruco_stale_s 는 안 넘는 구간.
    node._aruco_last_seen = node._now() - 1.0
    node._global_last_correction = node._now()
    node._main_stamp = node._now()
    place_follower(node, 1.5, 0.0)
    node._tick()
    node.destroy_node()

    payload = states[-1]
    assert "target_xy" in payload, "목표가 만들어지지 않았다"
    # 궤적 끝은 x=2.95 근처, 추종거리 1.0 m 이므로 목표는 x=1.95 근처
    assert payload["target_xy"][0] == pytest.approx(1.95, abs=0.15), payload


def test_state_is_reported_even_when_stopped(ros):
    """조용히 멈춰 있는 것과 고장난 것을 구분할 수 있어야 한다."""
    node = make_node()
    _, states = collect(node)
    node._tick()
    node.destroy_node()
    assert states[-1]["state"]
    assert states[-1]["reason"]
    assert states[-1]["command"] == "S"
