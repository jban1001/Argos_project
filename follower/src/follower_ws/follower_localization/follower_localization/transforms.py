"""SE(3) 변환과 협조 위치추정 수식.

ROS 에 의존하지 않는다. 프레임 규약을 틀리는 것이 이 프로젝트에서 가장
비싼 실수이므로, 수식만 떼어 내서 mock 으로 검증할 수 있게 해 둔다.

프레임 표기
-----------
`T_A_B` 는 **B 프레임을 A 프레임에서 본 것**이다. 즉 B 좌표의 점을 A 좌표로
옮긴다:

    p_A = T_A_B @ p_B

따라서 연쇄는 안쪽 첨자가 맞물린다: `T_A_C = T_A_B @ T_B_C`.
역변환은 첨자를 뒤집는다: `inverse(T_A_B) = T_B_A`.

협조 위치추정 (spec section 16)
-------------------------------
알 수 있는 값:

    T_M_L   메인 로봇의 AMCL pose (map 에서 본 main_base_link)
    T_L_A   메인 로봇에 붙은 ArUco 의 고정 변환
    T_C_A   팔로워 카메라가 측정한 ArUco 상대 pose
    T_F_C   팔로워 카메라의 고정 변환

    T_M_F = T_M_L @ T_L_A @ inverse(T_C_A) @ inverse(T_F_C)

연쇄가 M <- L <- A <- C <- F 로 맞물린다. VIO 가 주는 `T_O_F` 와 합치면

    T_M_O = T_M_F @ inverse(T_O_F)

가 되어, 이것이 발행할 map -> follower_odom 이다.

오일러 각을 더하지 않는다. 전부 동차행렬과 쿼터니언으로 다룬다.
"""

from __future__ import annotations

import math

import numpy as np


def quaternion_to_matrix(quaternion) -> np.ndarray:
    """[x, y, z, w] -> 3x3 회전행렬."""
    x, y, z, w = (float(v) for v in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("영 쿼터니언은 회전이 아닙니다")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    """3x3 회전행렬 -> [x, y, z, w]. w >= 0 으로 정규화한다."""
    trace = float(np.trace(rotation))
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            w = (rotation[2, 1] - rotation[1, 2]) / scale
            x = 0.25 * scale
            y = (rotation[0, 1] + rotation[1, 0]) / scale
            z = (rotation[0, 2] + rotation[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            w = (rotation[0, 2] - rotation[2, 0]) / scale
            x = (rotation[0, 1] + rotation[1, 0]) / scale
            y = 0.25 * scale
            z = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            w = (rotation[1, 0] - rotation[0, 1]) / scale
            x = (rotation[0, 2] + rotation[2, 0]) / scale
            y = (rotation[1, 2] + rotation[2, 1]) / scale
            z = 0.25 * scale
    quaternion = np.array([x, y, z, w], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    # q 와 -q 는 같은 회전이다. 부호를 고정해야 두 자세를 비교할 때 헷갈리지 않는다.
    return -quaternion if quaternion[3] < 0 else quaternion


def make_transform(translation, quaternion) -> np.ndarray:
    """이동 + 쿼터니언 -> 4x4 동차행렬."""
    transform = np.eye(4)
    transform[:3, :3] = quaternion_to_matrix(quaternion)
    transform[:3, 3] = np.asarray(translation, dtype=float)
    return transform


def split_transform(transform: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """4x4 -> (이동, 쿼터니언)."""
    return transform[:3, 3].copy(), matrix_to_quaternion(transform[:3, :3])


def inverse(transform: np.ndarray) -> np.ndarray:
    """회전의 직교성을 이용한 역변환. 일반 역행렬보다 정확하고 빠르다."""
    rotation = transform[:3, :3]
    result = np.eye(4)
    result[:3, :3] = rotation.T
    result[:3, 3] = -rotation.T @ transform[:3, 3]
    return result


def yaw_of(transform: np.ndarray) -> float:
    """지면 로봇에서 의미 있는 회전 성분 [rad].

    x 축을 xy 평면에 투영해서 잰다. 오일러 분해와 달리 pitch 가 90 도에
    가까워도 정의가 무너지지 않는다.
    """
    forward = transform[:3, 0]
    return math.atan2(float(forward[1]), float(forward[0]))


def transform_distance(first: np.ndarray, second: np.ndarray) -> tuple[float, float]:
    """두 변환 사이의 (이동 거리 [m], 회전 각 [deg])."""
    delta = inverse(first) @ second
    translation = float(np.linalg.norm(delta[:3, 3]))
    cosine = (float(np.trace(delta[:3, :3])) - 1.0) / 2.0
    angle = math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
    return translation, angle


def follower_pose_in_map(t_map_main: np.ndarray, t_main_aruco: np.ndarray,
                         t_cam_aruco: np.ndarray, t_base_cam: np.ndarray) -> np.ndarray:
    """T_M_F -- ArUco 관측으로부터 팔로워의 map 기준 pose.

    T_M_F = T_M_L @ T_L_A @ inverse(T_C_A) @ inverse(T_F_C)
    """
    return t_map_main @ t_main_aruco @ inverse(t_cam_aruco) @ inverse(t_base_cam)


def map_to_odom(t_map_base: np.ndarray, t_odom_base: np.ndarray) -> np.ndarray:
    """T_M_O -- 발행할 map -> follower_odom.

    VIO 가 T_O_F 를 계속 갱신하므로, 이 변환만 ArUco 로 보정하면
    map -> follower_odom -> follower_base_link 가 일관되게 유지된다.
    """
    return t_map_base @ inverse(t_odom_base)


def flatten_to_ground(transform: np.ndarray) -> np.ndarray:
    """z, roll, pitch 를 버리고 x, y, yaw 만 남긴다.

    지상 주행 로봇의 map -> odom 보정에 쓴다. 카메라가 기울어져 장착돼
    있으면 ArUco 관측이 작은 roll/pitch 를 만들어 내는데, 그것을 그대로
    map 보정에 실으면 로봇이 지면에서 뜬 것처럼 된다.
    """
    yaw = yaw_of(transform)
    result = np.eye(4)
    result[0, 0] = math.cos(yaw)
    result[0, 1] = -math.sin(yaw)
    result[1, 0] = math.sin(yaw)
    result[1, 1] = math.cos(yaw)
    result[:2, 3] = transform[:2, 3]
    return result
