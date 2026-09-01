"""팔로워의 좌표계 사슬을 설정에서 만든다.

    follower_base_link -> follower_imu_link -> follower_camera

base_link -> IMU 는 config/follower_frames.yaml, IMU -> 카메라는
config/cam_imu.yaml 이 출처다. 같은 값을 두 군데 두지 않는다.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml

from follower_localization.config import UnmeasuredValue, require_measured
from follower_localization.transforms import make_transform, quaternion_to_matrix


def _yaw_matrix(degrees: float) -> np.ndarray:
    angle = math.radians(degrees)
    return np.array([[math.cos(angle), -math.sin(angle), 0.0],
                     [math.sin(angle), math.cos(angle), 0.0],
                     [0.0, 0.0, 1.0]])


def base_to_imu(frames_file: Path | str) -> np.ndarray:
    """T_F_I -- base_link 에서 본 IMU.

    로봇 좌표 규약은 x 정면, y 좌측, z 위다. 설정은 사람이 자로 재기 쉬운
    forward/left/up 으로 받고 여기서 축에 넣는다 -- 사람에게 축 부호를
    판단하게 하면 틀린다 (병진 13.5 cm 를 엉뚱한 축에 넣은 적이 있다).
    """
    document = yaml.safe_load(Path(frames_file).read_text(encoding="utf-8"))
    section = (document or {}).get("follower_frames")
    if not section:
        raise UnmeasuredValue(f"{frames_file} 에 follower_frames 항목이 없다")
    mount = section.get("base_to_imu")
    if not mount:
        raise UnmeasuredValue(f"{frames_file} 에 base_to_imu 항목이 없다")

    hint = ("바퀴 축 중점에서 MPU6050 칩까지 앞/왼쪽/위를 자로 재서 넣을 것:\n"
            "    python3 scripts/28_set_frames.py --base-to-imu <앞> <왼> <위> "
            "--imu-to-cam <앞> <우> <위>   (cm)")
    forward = require_measured("base_to_imu.forward_m", mount.get("forward_m"), hint)
    left = require_measured("base_to_imu.left_m", mount.get("left_m"), hint)
    up = require_measured("base_to_imu.up_m", mount.get("up_m"), hint)
    yaw = require_measured(
        "base_to_imu.yaw_deg", mount.get("yaw_deg"),
        "python3 scripts/27_check_imu_alignment.py 로 IMU-로봇 회전을 잴 것.")

    transform = np.eye(4)
    transform[:3, :3] = _yaw_matrix(yaw)
    transform[:3, 3] = [forward, left, up]
    return transform


def imu_to_camera(cam_imu_file: Path | str) -> np.ndarray:
    """T_I_C -- IMU 에서 본 카메라 광학 프레임."""
    document = yaml.safe_load(Path(cam_imu_file).read_text(encoding="utf-8"))
    if document is None:
        raise UnmeasuredValue(f"{cam_imu_file} 를 읽을 수 없다")

    # calibrated: false 는 "이 값은 지금 형상과 맞지 않는다" 는 뜻이다.
    # 주석으로만 적어 두면 아무도 보지 않고 그대로 쓰인다 -- 실제로 막는다.
    if document.get("calibrated") is False:
        raise UnmeasuredValue(
            f"{cam_imu_file} 이(가) calibrated: false 다. 카메라나 IMU 가 움직인 뒤로 "
            "재측정되지 않았다는 뜻이므로, 이 값으로 계산한 T_F_C 는 틀린다. "
            "scripts/18_estimate_cam_imu_rotation.py 로 회전을 다시 재고 병진도 "
            "다시 잰 뒤 calibrated: true 로 되돌릴 것.")

    section = document.get("T_imu_cam")
    if not section:
        raise UnmeasuredValue(f"{cam_imu_file} 에 T_imu_cam 항목이 없다")
    hint = ("scripts/18_estimate_cam_imu_rotation.py 로 회전을, "
            "scripts/19_set_translation.py 로 병진을 잴 것.")
    translation = [require_measured(f"T_imu_cam.translation[{i}]", v, hint)
                   for i, v in enumerate(section["translation"])]
    quaternion = [require_measured(f"T_imu_cam.rotation_xyzw[{i}]", v, hint)
                  for i, v in enumerate(section["rotation_xyzw"])]
    return make_transform(translation, quaternion)


def base_to_camera(frames_file: Path | str, cam_imu_file: Path | str) -> np.ndarray:
    """T_F_C -- spec section 16 연쇄가 쓰는 값."""
    return base_to_imu(frames_file) @ imu_to_camera(cam_imu_file)


def find_config_dir(marker: str = "follower_frames.yaml") -> Path:
    """워크스페이스의 config 디렉터리를 찾는다.

    Path(__file__).parents[N] 로 세어 올라가면 빌드 방식에 따라 달라진다.
    --symlink-install 이면 소스 트리를, 아니면 install/ 아래를 가리켜
    엉뚱한 경로가 나온다 (2026-08-29: install/.../lib/config 를 찾다 실패).
    세는 대신 실제로 있는 곳을 찾는다.
    """
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "config" / marker
        if candidate.exists():
            return parent / "config"
    # 못 찾으면 종래 규칙으로 되돌린다. 호출자가 파일 없음으로 실패한다.
    return here.parents[3] / "config"
