"""마커 장착값과 사전 자세.

가장 중요한 것은 왕복이다: 참값 T_M_F 에서 "마커가 이렇게 보일 것" 을 만들고,
그것을 관측인 셈 치고 section 16 연쇄에 넣었을 때 원래 T_M_F 가 나와야 한다.
두 방향의 연쇄가 서로 역이 아니면 여기서 걸린다 -- 그 오류는 실물에서는
"위치가 조금 이상한데" 로만 보인다.
"""

import math
import textwrap

import numpy as np
import pytest

from follower_localization.config import UnmeasuredValue
from follower_localization.marker_mount import (
    expected_marker_in_camera,
    main_base_to_marker,
)
from follower_localization.transforms import (
    follower_pose_in_map,
    inverse,
    make_transform,
    transform_distance,
)

MOUNT = """
aruco:
  marker_id: 5
  marker_size_m: 0.115
  mount:
    forward_m: -0.195
    left_m: 0.0
    up_m: 0.055
"""


def write(tmp_path, body):
    path = tmp_path / "main_robot.yaml"
    path.write_text(textwrap.dedent(body), encoding="utf-8")
    return path


def test_unmeasured_mount_is_refused(tmp_path):
    body = MOUNT.replace("forward_m: -0.195", "forward_m: <CONFIGURE>")
    with pytest.raises(UnmeasuredValue):
        main_base_to_marker(write(tmp_path, body))


def test_missing_mount_section_explains_what_to_run(tmp_path):
    body = MOUNT[:MOUNT.index("  mount:")]
    with pytest.raises(UnmeasuredValue) as caught:
        main_base_to_marker(write(tmp_path, body))
    assert "25_resolve_marker_mount" in str(caught.value)


def test_translation_lands_on_the_right_axes(tmp_path):
    T = main_base_to_marker(write(tmp_path, MOUNT))
    assert np.allclose(T[:3, 3], [-0.195, 0.0, 0.055])


def test_marker_faces_backward(tmp_path):
    """마커 면 법선(+z)이 메인 로봇 뒤(-x)를 향해야 한다.

    앞을 향하게 놓으면 팔로워가 뒤에서 볼 때 마커가 안 보이고, 설령
    보이더라도 자세가 180 도 뒤집혀 위치가 통째로 틀린다.
    """
    T = main_base_to_marker(write(tmp_path, MOUNT))
    normal = T[:3, :3] @ np.array([0.0, 0.0, 1.0])
    assert np.allclose(normal, [-1.0, 0.0, 0.0], atol=1e-9), f"법선이 {normal}"


def test_marker_up_is_robot_up(tmp_path):
    T = main_base_to_marker(write(tmp_path, MOUNT))
    up = T[:3, :3] @ np.array([0.0, 1.0, 0.0])
    assert np.allclose(up, [0.0, 0.0, 1.0], atol=1e-9), f"마커 위쪽이 {up}"


def test_mount_rotation_is_orthonormal(tmp_path):
    R = main_base_to_marker(write(tmp_path, MOUNT))[:3, :3]
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
    assert np.isclose(np.linalg.det(R), 1.0)


def test_round_trip_recovers_the_follower_pose(tmp_path):
    """사전 자세를 만든 뒤 그것을 관측으로 되먹이면 참값이 돌아와야 한다."""
    t_main_marker = main_base_to_marker(write(tmp_path, MOUNT))

    # 메인 로봇은 map 에서 (3, 1) 에 45 도로 서 있다
    angle = math.radians(45.0)
    t_map_main = make_transform(
        [3.0, 1.0, 0.0], [0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)])

    # 팔로워는 그 뒤쪽 (2, 0) 에 30 도로
    follower_angle = math.radians(30.0)
    t_map_follower = make_transform(
        [2.0, 0.0, 0.0],
        [0.0, 0.0, math.sin(follower_angle / 2), math.cos(follower_angle / 2)])

    # 카메라는 base_link 기준 앞 0.115 위 0.27
    t_base_cam = make_transform([0.115, 0.015, 0.27], [0.0, 0.0, 0.0, 1.0])

    observed = expected_marker_in_camera(
        t_map_main, t_main_marker, t_map_follower, t_base_cam)
    recovered = follower_pose_in_map(
        t_map_main, t_main_marker, observed, t_base_cam)

    distance, degrees = transform_distance(t_map_follower, recovered)
    assert distance < 1e-9, f"위치가 {distance:.6f} m 어긋났다"
    assert degrees < 1e-6, f"자세가 {degrees:.6f} 도 어긋났다"


def test_expected_pose_puts_the_marker_in_front_of_the_camera(tmp_path):
    """뒤따라가는 배치에서 마커는 카메라 앞(+z)에 있어야 한다."""
    t_main_marker = main_base_to_marker(write(tmp_path, MOUNT))
    t_map_main = make_transform([3.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    t_map_follower = make_transform([2.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    t_base_cam = make_transform([0.115, 0.0, 0.27], [0.0, 0.0, 0.0, 1.0])

    expected = expected_marker_in_camera(
        t_map_main, t_main_marker, t_map_follower, t_base_cam)
    # 카메라 프레임에서 마커까지의 벡터. 광학 프레임이 아니라 base 정렬이므로
    # 앞은 +x 다 (t_base_cam 회전이 항등이므로).
    assert expected[0, 3] > 0.0, "마커가 카메라 뒤에 있다고 나왔다"


def test_a_wrong_prior_does_not_change_the_recovered_pose(tmp_path):
    """사전값은 두 해 중 고르는 데만 쓰인다. 복원 자체에 섞이면 안 된다."""
    t_main_marker = main_base_to_marker(write(tmp_path, MOUNT))
    t_map_main = make_transform([3.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    truth = make_transform([2.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    t_base_cam = make_transform([0.115, 0.0, 0.27], [0.0, 0.0, 0.0, 1.0])

    observed = expected_marker_in_camera(t_map_main, t_main_marker, truth, t_base_cam)
    recovered = follower_pose_in_map(t_map_main, t_main_marker, observed, t_base_cam)
    distance, _ = transform_distance(truth, recovered)
    assert distance < 1e-9
