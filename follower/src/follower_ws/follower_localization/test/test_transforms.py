"""협조 위치추정 수식 검증.

프레임 규약을 틀리는 것이 이 프로젝트에서 가장 비싼 실수다. 카메라-IMU
병진을 엉뚱한 축에 넣어 13.5 cm 를 틀렸던 적이 있고, 그때는 회전 측정이
축의 물리적 방향을 알려줘서 잡혔다. 여기서는 참값을 아는 mock 장면을
만들어, 관측에서 역산한 값이 참값과 일치하는지 확인한다.

    python3 src/follower_localization/test/test_transforms.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from follower_localization.transforms import (  # noqa: E402
    flatten_to_ground, follower_pose_in_map, inverse, make_transform,
    map_to_odom, matrix_to_quaternion, quaternion_to_matrix, split_transform,
    transform_distance, yaw_of)


def _rotation_about(axis, degrees: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    angle = math.radians(degrees)
    skew = np.array([[0, -axis[2], axis[1]],
                     [axis[2], 0, -axis[0]],
                     [-axis[1], axis[0], 0]])
    rotation = np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * skew @ skew
    result = np.eye(4)
    result[:3, :3] = rotation
    return result


def _pose(x, y, z, yaw_deg) -> np.ndarray:
    result = _rotation_about((0, 0, 1), yaw_deg)
    result[:3, 3] = [x, y, z]
    return result


# --------------------------------------------------------------------------
# 기초 연산
# --------------------------------------------------------------------------

def test_quaternion_round_trip() -> None:
    rng = np.random.default_rng(0)
    for _ in range(50):
        axis = rng.normal(size=3)
        rotation = _rotation_about(axis, rng.uniform(-180, 180))[:3, :3]
        rebuilt = quaternion_to_matrix(matrix_to_quaternion(rotation))
        assert np.abs(rebuilt - rotation).max() < 1e-9


def test_quaternion_sign_is_canonical() -> None:
    """q 와 -q 는 같은 회전이다. 부호가 흔들리면 두 자세 비교가 무너진다."""
    rng = np.random.default_rng(1)
    for _ in range(30):
        rotation = _rotation_about(rng.normal(size=3), rng.uniform(-180, 180))[:3, :3]
        assert matrix_to_quaternion(rotation)[3] >= 0.0


def test_inverse_is_exact() -> None:
    rng = np.random.default_rng(2)
    for _ in range(30):
        transform = _rotation_about(rng.normal(size=3), rng.uniform(-180, 180))
        transform[:3, 3] = rng.normal(size=3) * 3.0
        assert np.abs(inverse(transform) @ transform - np.eye(4)).max() < 1e-12


def test_make_and_split_round_trip() -> None:
    translation = np.array([1.5, -2.25, 0.75])
    quaternion = matrix_to_quaternion(_rotation_about((1, 2, 3), 42.0)[:3, :3])
    back_t, back_q = split_transform(make_transform(translation, quaternion))
    assert np.allclose(back_t, translation)
    assert np.allclose(back_q, quaternion)


def test_yaw_survives_steep_pitch() -> None:
    """오일러 분해가 무너지는 각도에서도 yaw 가 정의돼야 한다.

    카메라가 12.5 도 기울어 장착된 이 로봇에서는 ArUco 관측이 항상 약간의
    roll/pitch 를 만든다. yaw 추출이 거기서 무너지면 안 된다.
    """
    for pitch in (0.0, 45.0, 85.0, 89.9):
        transform = _rotation_about((0, 0, 1), 30.0) @ _rotation_about((0, 1, 0), pitch)
        assert abs(math.degrees(yaw_of(transform)) - 30.0) < 1e-6, pitch


# --------------------------------------------------------------------------
# 협조 위치추정 (spec section 16)
# --------------------------------------------------------------------------

def test_recovers_follower_pose_from_a_known_scene() -> None:
    """참값을 아는 장면을 만들고, 관측에서 역산한 값이 그것과 같은지."""
    # 참값
    t_map_main = _pose(4.0, 1.0, 0.0, 30.0)         # 메인 로봇의 map 기준 pose
    t_map_follower = _pose(2.5, 0.2, 0.0, 20.0)     # 팔로워의 참 pose (구하려는 것)
    t_main_aruco = _pose(-0.15, 0.0, 0.35, 180.0)   # 마커는 메인 뒤쪽, 뒤를 향함
    t_base_cam = make_transform([0.0, 0.135, 0.19],
                                matrix_to_quaternion(_rotation_about((1, 0, 0), -95.0)[:3, :3]))

    # 카메라가 실제로 보게 될 관측을 참값에서 만든다
    t_map_cam = t_map_follower @ t_base_cam
    t_cam_aruco = inverse(t_map_cam) @ t_map_main @ t_main_aruco

    recovered = follower_pose_in_map(t_map_main, t_main_aruco, t_cam_aruco, t_base_cam)
    distance, angle = transform_distance(t_map_follower, recovered)
    assert distance < 1e-9, distance
    assert angle < 1e-7, angle


def test_map_to_odom_keeps_the_chain_consistent() -> None:
    """map -> odom -> base 가 map -> base 와 일치해야 한다.

    이것이 무너지면 RViz 에서 로봇이 지도 위 엉뚱한 곳에 놓인다.
    """
    t_map_follower = _pose(2.5, 0.2, 0.0, 20.0)
    t_odom_follower = _pose(1.1, -0.4, 0.0, -7.0)    # VIO 가 주는 값
    t_map_odom = map_to_odom(t_map_follower, t_odom_follower)
    rebuilt = t_map_odom @ t_odom_follower
    assert np.abs(rebuilt - t_map_follower).max() < 1e-12


def test_full_pipeline_with_vio_drift() -> None:
    """VIO 원점이 어디에 있든 결과가 같아야 한다.

    VIO 는 자기가 켜진 자리를 원점으로 삼는다. 그 자리가 map 어디든
    map -> base 는 ArUco 로만 결정되어야 한다.
    """
    t_map_main = _pose(6.0, -2.0, 0.0, 115.0)
    t_map_follower = _pose(4.4, -2.9, 0.0, 100.0)
    t_main_aruco = _pose(-0.15, 0.0, 0.35, 180.0)
    t_base_cam = make_transform([0.0, 0.135, 0.19],
                                matrix_to_quaternion(_rotation_about((1, 0, 0), -95.0)[:3, :3]))
    t_cam_aruco = inverse(t_map_follower @ t_base_cam) @ t_map_main @ t_main_aruco

    for odom_origin in (_pose(0, 0, 0, 0), _pose(-30.0, 12.0, 0.0, 200.0),
                        _pose(0.01, 0.0, 0.0, 0.5)):
        t_odom_follower = inverse(odom_origin) @ t_map_follower
        t_map_base = follower_pose_in_map(t_map_main, t_main_aruco, t_cam_aruco, t_base_cam)
        t_map_odom = map_to_odom(t_map_base, t_odom_follower)
        rebuilt = t_map_odom @ t_odom_follower
        assert np.abs(rebuilt - t_map_follower).max() < 1e-9


def test_wrong_marker_mount_shows_up_as_a_pose_error() -> None:
    """T_L_A 를 틀리면 결과가 그만큼 틀려야 한다 (조용히 흡수되면 안 된다).

    마커가 main_base_link 기준 어디에 붙어있는지는 사람이 재서 넣는 값이고,
    Phase 6 에서 아직 못 받은 항목이다. 틀렸을 때 얼마나 틀리는지 알아야
    측정 정밀도를 정할 수 있다.
    """
    t_map_main = _pose(4.0, 1.0, 0.0, 30.0)
    t_map_follower = _pose(2.5, 0.2, 0.0, 20.0)
    t_main_aruco = _pose(-0.15, 0.0, 0.35, 180.0)
    t_base_cam = make_transform([0.0, 0.135, 0.19],
                                matrix_to_quaternion(_rotation_about((1, 0, 0), -95.0)[:3, :3]))
    t_cam_aruco = inverse(t_map_follower @ t_base_cam) @ t_map_main @ t_main_aruco

    # 마커 위치를 5 cm 잘못 알고 있는 경우
    wrong_mount = _pose(-0.15 + 0.05, 0.0, 0.35, 180.0)
    recovered = follower_pose_in_map(t_map_main, wrong_mount, t_cam_aruco, t_base_cam)
    distance, _ = transform_distance(t_map_follower, recovered)
    assert abs(distance - 0.05) < 1e-9, distance

    # 마커 방향을 10 도 잘못 알고 있는 경우: 마커에서 카메라까지의 거리에
    # 비례해 위치 오차가 생긴다.
    wrong_yaw = _pose(-0.15, 0.0, 0.35, 190.0)
    recovered = follower_pose_in_map(t_map_main, wrong_yaw, t_cam_aruco, t_base_cam)
    distance, angle = transform_distance(t_map_follower, recovered)
    assert abs(angle - 10.0) < 1e-6, angle
    assert distance > 0.1, distance


def test_flatten_to_ground_removes_tilt_but_keeps_yaw() -> None:
    transform = _pose(1.0, 2.0, 0.7, 40.0) @ _rotation_about((1, 0, 0), 6.0)
    flat = flatten_to_ground(transform)
    assert abs(math.degrees(yaw_of(flat)) - 40.0) < 1e-6
    assert abs(flat[2, 3]) < 1e-12
    assert np.abs(flat[:3, :3] @ np.array([0, 0, 1.0]) - np.array([0, 0, 1.0])).max() < 1e-12
    assert np.allclose(flat[:2, 3], [1.0, 2.0])


if __name__ == "__main__":
    for name, function in sorted(globals().items()):
        if name.startswith("test_"):
            function()
            print(f"PASS  {name}")
    print("\nall transform tests passed")
