"""협조 위치추정 노드를 mock 입력으로 검증한다.

기동 거부만 확인하고 정상 경로를 두면, 거부는 잘 되는데 아무것도 발행하지
않는 노드가 통과한다.
"""

import math
import textwrap
from pathlib import Path

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.parameter import Parameter
from rclpy.time import Time

from follower_localization.config import UnmeasuredValue
from follower_localization.cooperative_node import CooperativeLocalization
from follower_localization.transforms import make_transform, transform_distance

REPO = Path(__file__).resolve().parents[3]

GOOD_MOUNT = """
aruco:
  marker_id: 5
  marker_size_m: 0.115
  mount:
    forward_m: -0.195
    left_m: 0.0
    up_m: 0.055
"""


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def write_config(tmp_path, body=GOOD_MOUNT):
    path = tmp_path / "main_robot.yaml"
    path.write_text(textwrap.dedent(body), encoding="utf-8")
    return path


def make_node(config_path):
    return CooperativeLocalization(parameter_overrides=[
        Parameter("main_robot_config", value=str(config_path)),
        Parameter("frames_config", value=str(REPO / "config" / "follower_frames.yaml")),
        Parameter("cam_imu_config", value=str(REPO / "config" / "cam_imu.yaml")),
    ])


def pose_message(matrix, stamp) -> PoseWithCovarianceStamped:
    from follower_localization.transforms import split_transform
    message = PoseWithCovarianceStamped()
    message.header.stamp = stamp
    translation, quaternion = split_transform(matrix)
    message.pose.pose.position.x = float(translation[0])
    message.pose.pose.position.y = float(translation[1])
    message.pose.pose.position.z = float(translation[2])
    message.pose.pose.orientation.x = float(quaternion[0])
    message.pose.pose.orientation.y = float(quaternion[1])
    message.pose.pose.orientation.z = float(quaternion[2])
    message.pose.pose.orientation.w = float(quaternion[3])
    return message


def feed_odometry(node, matrix, stamp):
    """VIO 가 발행했을 follower_odom -> follower_base_link 를 버퍼에 넣는다."""
    from follower_localization.cooperative_node import matrix_to_transform
    transform = matrix_to_transform(matrix, stamp, "follower_odom", "follower_base_link")
    node._buffer.set_transform(transform, "test")


def collect_tf(node):
    captured = []
    node._broadcaster.sendTransform = lambda t: captured.append(t)
    return captured


def yaw_pose(x, y, degrees):
    angle = math.radians(degrees)
    return make_transform([x, y, 0.0],
                          [0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)])


def test_refuses_unmeasured_marker_mount(ros, tmp_path):
    body = GOOD_MOUNT.replace("forward_m: -0.195", "forward_m: <CONFIGURE>")
    with pytest.raises(UnmeasuredValue):
        make_node(write_config(tmp_path, body))


def test_publishes_map_to_odom(ros, tmp_path):
    """게이트가 연속 3 프레임을 요구하므로 그만큼 먹인다.

    한 프레임만 넣으면 warming_up 으로 기각되는데, 그건 게이트가 제 일을
    하는 것이지 노드가 고장난 게 아니다.
    """
    node = make_node(write_config(tmp_path))
    sent = collect_tf(node)

    t_map_main = yaw_pose(3.0, 0.0, 0.0)
    truth = yaw_pose(2.0, 0.0, 0.0)
    t_odom_base = yaw_pose(0.5, 0.0, 0.0)          # VIO 가 조금 흘렀다고 치자

    from follower_localization.marker_mount import expected_marker_in_camera
    observed = expected_marker_in_camera(
        t_map_main, node._t_main_marker, truth, node._t_base_cam)

    for index in range(5):
        stamp = Time(nanoseconds=int((100.0 + index * 0.1) * 1e9)).to_msg()
        feed_odometry(node, t_odom_base, stamp)
        node._on_main_pose(pose_message(t_map_main, stamp))
        node._on_observation(pose_message(observed, stamp))
    node.destroy_node()

    assert sent, "연속 관측을 넣었는데 TF 가 안 나갔다"
    assert sent[-1].header.frame_id == "map"
    assert sent[-1].child_frame_id == "follower_odom"


def test_a_single_observation_is_not_trusted(ros, tmp_path):
    """한 프레임만 보고 map 을 옮기면 오검출 한 번에 위치가 튄다."""
    node = make_node(write_config(tmp_path))
    sent = collect_tf(node)
    stamp = Time(seconds=100).to_msg()
    t_map_main = yaw_pose(3.0, 0.0, 0.0)
    feed_odometry(node, yaw_pose(0.5, 0.0, 0.0), stamp)

    from follower_localization.marker_mount import expected_marker_in_camera
    observed = expected_marker_in_camera(
        t_map_main, node._t_main_marker, yaw_pose(2.0, 0.0, 0.0), node._t_base_cam)
    node._on_main_pose(pose_message(t_map_main, stamp))
    node._on_observation(pose_message(observed, stamp))
    node.destroy_node()
    assert sent == []


def test_nothing_without_a_main_pose(ros, tmp_path):
    """메인 자세 없이 절대 위치를 낼 수 없다."""
    node = make_node(write_config(tmp_path))
    sent = collect_tf(node)
    stamp = Time(seconds=100).to_msg()
    feed_odometry(node, yaw_pose(0.0, 0.0, 0.0), stamp)
    node._on_observation(pose_message(yaw_pose(1.0, 0.0, 0.0), stamp))
    node.destroy_node()
    assert sent == []


def test_stale_main_pose_is_skipped(ros, tmp_path):
    """묵은 자세를 쓰면 그 사이 메인 로봇이 이동한 만큼 통째로 틀린다."""
    node = make_node(write_config(tmp_path))
    sent = collect_tf(node)
    old = Time(seconds=100).to_msg()
    now = Time(seconds=105).to_msg()          # 5 초 뒤, 한계 1 초

    node._on_main_pose(pose_message(yaw_pose(3.0, 0.0, 0.0), old))
    feed_odometry(node, yaw_pose(0.0, 0.0, 0.0), now)
    node._on_observation(pose_message(yaw_pose(1.0, 0.0, 0.0), now))
    node.destroy_node()
    assert sent == []


def test_missing_vio_transform_is_skipped(ros, tmp_path):
    """VIO 가 없으면 map -> odom 을 계산할 수 없다."""
    node = make_node(write_config(tmp_path))
    sent = collect_tf(node)
    stamp = Time(seconds=100).to_msg()
    node._on_main_pose(pose_message(yaw_pose(3.0, 0.0, 0.0), stamp))
    node._on_observation(pose_message(yaw_pose(1.0, 0.0, 0.0), stamp))
    node.destroy_node()
    assert sent == []


def test_alternative_solution_is_taken_when_it_matches_the_prior(ros, tmp_path):
    """평면 마커의 뒤집힌 해를 사전값으로 골라낼 수 있어야 한다."""
    node = make_node(write_config(tmp_path))
    stamp = Time(seconds=100).to_msg()
    t_map_main = yaw_pose(3.0, 0.0, 0.0)
    truth = yaw_pose(2.0, 0.0, 0.0)

    from follower_localization.marker_mount import expected_marker_in_camera
    correct = expected_marker_in_camera(
        t_map_main, node._t_main_marker, truth, node._t_base_cam)
    # 12 도 뒤집힌 가짜 해를 "가장 잘 맞는 해" 자리에 놓는다
    flip = make_transform([0.0, 0.0, 0.0],
                          [0.0, math.sin(math.radians(6.0)), 0.0,
                           math.cos(math.radians(6.0))])
    flipped = correct @ flip

    node._prior = truth
    node._alt = (Time.from_msg(stamp), correct)
    chosen = node._choose_solution(Time.from_msg(stamp), flipped, t_map_main)
    node.destroy_node()

    distance, degrees = transform_distance(chosen, correct)
    assert degrees < 1e-6, "사전값에 가까운 해를 고르지 못했다"


def test_best_solution_is_kept_when_it_already_matches(ros, tmp_path):
    """늘 두 번째 해를 고르면 판정이 아니라 뒤집기다."""
    node = make_node(write_config(tmp_path))
    stamp = Time(seconds=100).to_msg()
    t_map_main = yaw_pose(3.0, 0.0, 0.0)
    truth = yaw_pose(2.0, 0.0, 0.0)

    from follower_localization.marker_mount import expected_marker_in_camera
    correct = expected_marker_in_camera(
        t_map_main, node._t_main_marker, truth, node._t_base_cam)
    flip = make_transform([0.0, 0.0, 0.0],
                          [0.0, math.sin(math.radians(6.0)), 0.0,
                           math.cos(math.radians(6.0))])

    node._prior = truth
    node._alt = (Time.from_msg(stamp), correct @ flip)
    chosen = node._choose_solution(Time.from_msg(stamp), correct, t_map_main)
    node.destroy_node()
    _, degrees = transform_distance(chosen, correct)
    assert degrees < 1e-6


def test_alternative_from_a_different_frame_is_ignored(ros, tmp_path):
    """짝이 아닌 프레임의 두 번째 해를 섞으면 엉뚱한 자세를 고른다."""
    node = make_node(write_config(tmp_path))
    stamp = Time(seconds=100)
    other = Time(seconds=101)
    t_map_main = yaw_pose(3.0, 0.0, 0.0)
    truth = yaw_pose(2.0, 0.0, 0.0)

    from follower_localization.marker_mount import expected_marker_in_camera
    correct = expected_marker_in_camera(
        t_map_main, node._t_main_marker, truth, node._t_base_cam)
    node._prior = truth
    node._alt = (other, correct)
    observed = correct @ make_transform(
        [0.0, 0.0, 0.0], [0.0, math.sin(math.radians(6.0)), 0.0,
                          math.cos(math.radians(6.0))])
    chosen = node._choose_solution(stamp, observed, t_map_main)
    node.destroy_node()
    assert np.allclose(chosen, observed), "시각이 다른 짝을 채택했다"


def test_published_correction_puts_the_follower_at_the_truth(ros, tmp_path):
    """TF 가 나갔는지가 아니라 그 값이 맞는지.

        map -> follower_odom  (발행값)
      @ follower_odom -> follower_base_link  (VIO)
      =  참값이어야 한다

    GroundCorrection 이 속도 제한을 걸므로 한 번에 도달하지 않는다. 충분히
    먹여서 수렴하는지 본다 -- 수렴하지 않으면 부호나 합성 순서가 틀린 것이다.
    """
    node = make_node(write_config(tmp_path))
    sent = collect_tf(node)

    t_map_main = yaw_pose(3.0, 0.0, 0.0)
    truth = yaw_pose(2.0, 0.5, 20.0)
    t_odom_base = yaw_pose(0.5, 0.0, 20.0)

    from follower_localization.marker_mount import expected_marker_in_camera
    observed = expected_marker_in_camera(
        t_map_main, node._t_main_marker, truth, node._t_base_cam)

    for index in range(120):
        stamp = Time(nanoseconds=int((100.0 + index * 0.1) * 1e9)).to_msg()
        feed_odometry(node, t_odom_base, stamp)
        node._on_main_pose(pose_message(t_map_main, stamp))
        node._on_observation(pose_message(observed, stamp))
    node.destroy_node()

    assert sent, "TF 가 안 나갔다"
    from follower_localization.transforms import flatten_to_ground
    last = sent[-1]
    t_map_odom = make_transform(
        [last.transform.translation.x, last.transform.translation.y,
         last.transform.translation.z],
        [last.transform.rotation.x, last.transform.rotation.y,
         last.transform.rotation.z, last.transform.rotation.w])
    recovered = t_map_odom @ t_odom_base

    # 지면 보정이므로 z/roll/pitch 는 버려진다. 참값도 같은 처리로 비교한다.
    distance, degrees = transform_distance(flatten_to_ground(truth), recovered)
    assert distance < 0.02, f"위치가 {distance * 100:.1f} cm 어긋났다"
    assert degrees < 1.0, f"자세가 {degrees:.2f} 도 어긋났다"


def _observer(node, t_map_main, t_odom_base, sent):
    from follower_localization.marker_mount import expected_marker_in_camera

    def observe(follower_pose, index):
        stamp = Time(nanoseconds=int((100.0 + index * 0.1) * 1e9)).to_msg()
        feed_odometry(node, t_odom_base, stamp)
        node._on_main_pose(pose_message(t_map_main, stamp))
        node._on_observation(pose_message(expected_marker_in_camera(
            t_map_main, node._t_main_marker, follower_pose, node._t_base_cam), stamp))
    return observe


def test_a_jump_is_rejected_but_sustained_observation_re_acquires(ros, tmp_path):
    """게이트와 속도 제한은 다른 층이고, 게이트가 먼저 작동한다.

    max_position_jump_m 를 넘는 도약은 즉시 받아들이지 않는다 -- 오검출 한
    번이 map 을 옮기면 안 되기 때문이다. 다만 새 위치에서 관측이 계속
    일치하면 재획득한다. 그렇지 않으면 로봇이 실제로 옮겨졌을 때 영영
    복구하지 못한다. 두 성질을 다 확인한다.
    """
    node = make_node(write_config(tmp_path))
    sent = collect_tf(node)
    observe = _observer(node, yaw_pose(3.0, 0.0, 0.0), yaw_pose(0.0, 0.0, 0.0), sent)

    for index in range(40):
        observe(yaw_pose(2.0, 0.0, 0.0), index)
    settled = len(sent)

    # 도약 직후 한 프레임은 기각돼야 한다
    observe(yaw_pose(2.0, 1.0, 0.0), 40)
    assert len(sent) == settled, "도약을 한 프레임 만에 받아들였다"

    # 새 위치에서 계속 보이면 재획득한다
    for index in range(41, 50):
        observe(yaw_pose(2.0, 1.0, 0.0), index)
    node.destroy_node()
    assert len(sent) > settled, "새 위치에서 계속 보이는데 재획득하지 못했다"


def test_correction_is_rate_limited_after_the_first_fix(ros, tmp_path):
    """첫 관측은 그대로 받고(끌어갈 기준이 없다), 그 뒤 변화에 제한이 걸린다.

    게이트를 통과하는 크기(0.3 m)로 옮겨서 속도 제한만 보이게 한다.
    """
    node = make_node(write_config(tmp_path))
    sent = collect_tf(node)
    observe = _observer(node, yaw_pose(3.0, 0.0, 0.0), yaw_pose(0.0, 0.0, 0.0), sent)

    for index in range(40):
        observe(yaw_pose(2.0, 0.0, 0.0), index)
    assert sent, "수렴 단계에서 TF 가 안 나갔다"
    before = sent[-1]

    for index in range(40, 43):          # 0.3 m, 게이트 통과
        observe(yaw_pose(2.0, 0.3, 0.0), index)
    after = sent[-1]
    step = math.hypot(after.transform.translation.x - before.transform.translation.x,
                      after.transform.translation.y - before.transform.translation.y)
    node.destroy_node()

    # 0.30 m/s x 0.1 s = 0.03 m 이 한 걸음. 3 걸음이면 0.09 m 가 상한이다.
    assert step > 0.001, "전혀 따라가지 않았다"
    assert step < 0.12, f"0.3 m 이동을 {step:.3f} m 로 한 번에 따라갔다"
