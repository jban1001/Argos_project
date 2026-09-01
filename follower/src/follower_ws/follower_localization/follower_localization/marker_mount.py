"""메인 로봇에 붙은 마커의 위치 T_L_A 를 설정에서 만든다.

spec section 16 의 연쇄:

    T_M_F = T_M_L @ T_L_A @ inverse(T_C_A) @ inverse(T_F_C)

여기서 T_L_A 가 이 모듈이 담당하는 부분이다. 마커 장착 위치를 5 cm 틀리면
팔로워 위치가 정확히 5 cm 틀리고, 방향을 10 도 틀리면 10 도 틀린다 -- 이
값에는 어떤 필터도 걸리지 않고 그대로 통과한다.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml

from follower_localization.config import UnmeasuredValue, require_measured
from follower_localization.transforms import inverse


def _yaw_matrix(degrees: float) -> np.ndarray:
    angle = math.radians(degrees)
    return np.array([[math.cos(angle), -math.sin(angle), 0.0],
                     [math.sin(angle), math.cos(angle), 0.0],
                     [0.0, 0.0, 1.0]])


def main_base_to_marker(config_file: Path | str) -> np.ndarray:
    """T_L_A -- 메인 로봇 base_link 에서 본 마커.

    마커 좌표계는 OpenCV 규약을 따른다: +z 가 마커 면 바깥(카메라 쪽),
    +y 가 마커의 위, +x 가 마커를 정면에서 봤을 때의 오른쪽.

    이 로봇의 마커는 메인 로봇 뒤를 향하므로, 마커의 +z 는 로봇 -x 방향이다.
    """
    document = yaml.safe_load(Path(config_file).read_text(encoding="utf-8"))
    section = (document or {}).get("aruco")
    if not section:
        raise UnmeasuredValue(f"{config_file} 에 aruco 항목이 없다")
    mount = section.get("mount")
    if not mount:
        raise UnmeasuredValue(
            f"{config_file} 에 aruco.mount 가 없다. LiDAR 기준 실측값은 있지만 "
            "base_link 기준으로 변환하려면 메인 로봇의 base_link -> laser_frame "
            "정적 변환이 필요하다: python3 scripts/25_resolve_marker_mount.py")

    hint = ("메인 로봇 스택을 켠 뒤 실행할 것 (LiDAR 기준 실측값을 "
            "base_link 기준으로 변환한다):\n"
            "    python3 scripts/25_resolve_marker_mount.py")
    forward = require_measured("aruco.mount.forward_m", mount.get("forward_m"), hint)
    left = require_measured("aruco.mount.left_m", mount.get("left_m"), hint)
    up = require_measured("aruco.mount.up_m", mount.get("up_m"), hint)
    yaw = float(mount.get("yaw_deg", 180.0))

    # 마커 면 법선(+z)이 로봇 뒤(-x)를 향하고, 마커의 위(+y)가 로봇 위(+z) 를
    # 향하도록 놓는다. 이 두 조건이 마커 좌표계를 결정한다.
    #     marker +x -> robot -y (오른쪽)   marker +y -> robot +z   marker +z -> robot -x
    base_rotation = np.array([[0.0, 0.0, -1.0],
                              [-1.0, 0.0, 0.0],
                              [0.0, 1.0, 0.0]])
    transform = np.eye(4)
    transform[:3, :3] = _yaw_matrix(yaw - 180.0) @ base_rotation
    transform[:3, 3] = [forward, left, up]
    return transform


def expected_marker_in_camera(t_map_main: np.ndarray,
                              t_main_marker: np.ndarray,
                              t_map_follower_prior: np.ndarray,
                              t_base_cam: np.ndarray) -> np.ndarray:
    """지금 마커가 카메라에 어떻게 보일 것이라 믿는가 (T_C_A 의 사전값).

        T_C_A = inv(T_F_C) @ inv(T_M_F) @ T_M_L @ T_L_A

    평면 마커의 두 PnP 해 중 어느 쪽을 고를지 판정하는 데 쓴다. 팔로워가
    마커 정면에서 따라가므로 두 해의 재투영 오차가 거의 같고, 그것만으로는
    고를 수 없다. 대신 메인 로봇 AMCL 자세를 알고 있으므로 이 사전값과
    가까운 쪽을 고른다.

    사전값은 팔로워의 직전 추정(VIO 로 전파된 값)에 의존한다. 그 값이 크게
    틀어져 있으면 이 판정도 틀리므로, 호출하는 쪽이 사전값의 신뢰도를 함께
    판단해야 한다.
    """
    return inverse(t_base_cam) @ inverse(t_map_follower_prior) @ t_map_main @ t_main_marker
