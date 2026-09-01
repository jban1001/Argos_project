"""좌표계 사슬. 축 부호는 눈으로 검토해서는 안 잡힌다."""

import math
import textwrap

import numpy as np
import pytest
import yaml

from follower_localization.config import UnmeasuredValue
from follower_localization.frames import base_to_camera, base_to_imu, imu_to_camera

FRAMES = """
follower_frames:
  base_to_imu:
    forward_m: 0.05
    left_m: 0.0
    up_m: 0.12
    yaw_deg: -90.0
"""

CAM_IMU = """
T_imu_cam:
  translation: [-0.0420, 0.1088, 0.2018]
  rotation_xyzw: [-0.659860, -0.089422, -0.014678, 0.745904]
"""


def write(tmp_path, name, body):
    path = tmp_path / name
    path.write_text(textwrap.dedent(body), encoding="utf-8")
    return path


def test_imu_axes_map_to_the_measured_physical_directions(tmp_path):
    """측정으로 확인된 관계: IMU x = 로봇 우측, IMU y = 로봇 정면, IMU z = 위."""
    T = base_to_imu(write(tmp_path, "f.yaml", FRAMES))
    R = T[:3, :3]

    # IMU 좌표계의 각 축을 로봇 좌표계로 옮긴다
    imu_x_in_robot = R @ np.array([1.0, 0.0, 0.0])
    imu_y_in_robot = R @ np.array([0.0, 1.0, 0.0])
    imu_z_in_robot = R @ np.array([0.0, 0.0, 1.0])

    assert np.allclose(imu_x_in_robot, [0.0, -1.0, 0.0], atol=1e-9), "IMU x 가 로봇 우측이 아니다"
    assert np.allclose(imu_y_in_robot, [1.0, 0.0, 0.0], atol=1e-9), "IMU y 가 로봇 정면이 아니다"
    assert np.allclose(imu_z_in_robot, [0.0, 0.0, 1.0], atol=1e-9), "IMU z 가 위가 아니다"


def test_translation_lands_on_the_right_axes(tmp_path):
    """사람은 forward/left/up 으로 재고, 축에 넣는 일은 코드가 한다."""
    T = base_to_imu(write(tmp_path, "f.yaml", FRAMES))
    assert np.allclose(T[:3, 3], [0.05, 0.0, 0.12])


def test_unmeasured_mount_is_refused(tmp_path):
    body = FRAMES.replace("forward_m: 0.05", "forward_m: <CONFIGURE>")
    with pytest.raises(UnmeasuredValue):
        base_to_imu(write(tmp_path, "f.yaml", body))


def test_rotation_is_orthonormal(tmp_path):
    T = base_to_camera(write(tmp_path, "f.yaml", FRAMES),
                       write(tmp_path, "c.yaml", CAM_IMU))
    R = T[:3, :3]
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
    assert np.isclose(np.linalg.det(R), 1.0, atol=1e-9)


def test_chain_composes_in_the_right_order(tmp_path):
    frames = write(tmp_path, "f.yaml", FRAMES)
    cam_imu = write(tmp_path, "c.yaml", CAM_IMU)
    assert np.allclose(base_to_camera(frames, cam_imu),
                       base_to_imu(frames) @ imu_to_camera(cam_imu))


def test_camera_distance_from_base_is_preserved(tmp_path):
    """방향이 미해결이어도 거리는 회전과 무관하게 보존된다.

    IMU->카메라 병진의 크기는 0.2331 m 로 확정돼 있다. 사슬을 합성했을 때
    base_link 에서 카메라까지의 거리가 그 값과 base_to_imu 만큼 떨어져 있으면
    합성 자체는 맞다는 뜻이다.
    """
    frames = write(tmp_path, "f.yaml", FRAMES)
    cam_imu = write(tmp_path, "c.yaml", CAM_IMU)
    T = base_to_camera(frames, cam_imu)
    imu_position = base_to_imu(frames)[:3, 3]
    camera_offset = T[:3, 3] - imu_position
    assert np.linalg.norm(camera_offset) == pytest.approx(0.2331, abs=0.001)


def test_the_stored_translation_disagrees_with_an_axis_aligned_imu(tmp_path):
    """미해결 사항을 시험으로 못박아 둔다.

    저장된 IMU 프레임 병진은, IMU 가 로봇과 축정렬이라는 가정에서 실측
    (앞 13.5, 우 0, 위 19 cm) 를 변환한 값과 12.5 도 어긋난다. 그 12.5 도는
    측정된 카메라 기울기와 같은 값이고, 원인은 19 번 스크립트가 카메라
    광학축을 로봇 정면으로 놓았기 때문이다.

    scripts/27_check_imu_alignment.py 로 IMU 축정렬 여부를 가른 뒤 둘 중
    하나를 확정하면, 이 시험은 그때 지운다. 지금 지우면 그 12.5 도가 조용히
    따라간다.
    """
    stored = np.array(yaml.safe_load(
        write(tmp_path, "c.yaml", CAM_IMU).read_text())["T_imu_cam"]["translation"])
    # IMU 축정렬 가정에서 실측값을 IMU 프레임으로
    to_imu = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    axis_aligned = to_imu @ np.array([0.135, 0.0, 0.19])

    assert np.linalg.norm(stored) == pytest.approx(np.linalg.norm(axis_aligned), abs=1e-3), \
        "크기가 다르면 같은 벡터의 회전 문제가 아니라 다른 오류다"
    cosine = float(stored @ axis_aligned /
                   (np.linalg.norm(stored) * np.linalg.norm(axis_aligned)))
    angle = math.degrees(math.acos(np.clip(cosine, -1.0, 1.0)))
    assert angle == pytest.approx(12.5, abs=1.0), f"어긋난 각도가 {angle:.2f} 도다"


def test_missing_section_is_refused(tmp_path):
    with pytest.raises(UnmeasuredValue):
        base_to_imu(write(tmp_path, "f.yaml", "other: {}\n"))


def test_stale_calibration_is_refused(tmp_path):
    """calibrated: false 를 무시하면 표시는 장식일 뿐이다."""
    body = "calibrated: false\n" + CAM_IMU
    with pytest.raises(UnmeasuredValue) as caught:
        imu_to_camera(write(tmp_path, "c.yaml", body))
    assert "18_estimate_cam_imu_rotation" in str(caught.value)


def test_calibrated_true_is_accepted(tmp_path):
    body = "calibrated: true\n" + CAM_IMU
    T = imu_to_camera(write(tmp_path, "c.yaml", body))
    assert T.shape == (4, 4)


def test_real_config_is_now_usable():
    """실제 파일이 쓸 수 있는 상태인지.

    앞서 카메라가 움직여 calibrated: false 였고, 그때는 이 자리에 "거부되는지"
    를 확인하는 시험이 있었다. 회전(3 세션 평균)과 병진이 모두 들어와 true 가
    됐으므로 반대 방향으로 지킨다 -- 다시 무효화되면 여기서 걸린다.
    """
    from pathlib import Path
    repo = Path(__file__).resolve().parents[3]
    T = imu_to_camera(repo / "config" / "cam_imu.yaml")
    assert T.shape == (4, 4)
    assert np.allclose(T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-6)
    # 병진이 0 이면 19 번을 돌리지 않은 것이다
    assert np.linalg.norm(T[:3, 3]) > 0.05, "병진이 비어 있다"
