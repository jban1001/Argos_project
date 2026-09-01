"""ArUco 게이팅과 점진 보정 검증.

이 두 가지가 하는 일은 "좋을 때 잘 동작하는 것"이 아니라 "나쁠 때 망가지지
않는 것"이다. 그래서 테스트도 오검출, 소실, 재획득 같은 나쁜 상황을 만든다.

    python3 src/follower_localization/test/test_gating.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from follower_localization.gating import (  # noqa: E402
    ArucoGate, GateLimits, GroundCorrection)
from follower_localization.transforms import yaw_of  # noqa: E402


def _pose(x=0.0, y=0.0, yaw_deg=0.0, z=0.0) -> np.ndarray:
    yaw = math.radians(yaw_deg)
    t = np.eye(4)
    t[0, 0] = math.cos(yaw); t[0, 1] = -math.sin(yaw)
    t[1, 0] = math.sin(yaw); t[1, 1] = math.cos(yaw)
    t[:3, 3] = [x, y, z]
    return t


def _good(gate: ArucoGate, pose, stamp, marker=7, distance=1.0, reproj=0.5):
    return gate.check(marker, pose, distance, reproj, stamp)


# --------------------------------------------------------------------------
# 게이팅
# --------------------------------------------------------------------------

def test_requires_consecutive_detections() -> None:
    """한 프레임만 믿고 전역 TF 를 옮기지 않는다 (spec section 18)."""
    gate = ArucoGate(GateLimits(marker_id=7, required_consecutive=3))
    results = [_good(gate, _pose(1, 1), 0.1 * i) for i in range(4)]
    assert [r.accepted for r in results] == [False, False, True, True]
    assert results[1].reason == "warming_up"


def test_rejects_wrong_marker() -> None:
    gate = ArucoGate(GateLimits(marker_id=7, required_consecutive=1))
    assert not _good(gate, _pose(), 0.0, marker=9).accepted
    assert gate.rejected["wrong_id"] == 1
    assert _good(gate, _pose(), 0.1, marker=7).accepted


def test_rejects_out_of_range_and_blurry() -> None:
    gate = ArucoGate(GateLimits(marker_id=7, required_consecutive=1,
                                min_distance_m=0.25, max_distance_m=4.0,
                                max_reprojection_px=3.0))
    assert not _good(gate, _pose(), 0.0, distance=0.1).accepted     # 너무 가까움
    assert not _good(gate, _pose(), 0.1, distance=9.0).accepted     # 너무 멂
    assert not _good(gate, _pose(), 0.2, reproj=8.0).accepted       # 검출 품질 나쁨
    assert set(gate.rejected) == {"distance", "reprojection"}


def test_rejects_non_finite() -> None:
    """NaN 이 전역 TF 에 들어가면 하위 전체가 조용히 망가진다."""
    gate = ArucoGate(GateLimits(marker_id=7, required_consecutive=1))
    bad = _pose(1, 1)
    bad[0, 3] = float("nan")
    assert not _good(gate, bad, 0.0).accepted
    assert gate.rejected["not_finite"] == 1


def test_rejects_a_single_outlier_but_keeps_tracking() -> None:
    """오검출 한 프레임 뒤에 정상이 이어지면 다시 받아들여야 한다."""
    gate = ArucoGate(GateLimits(marker_id=7, required_consecutive=2,
                                max_position_jump_m=0.5))
    for i in range(3):
        _good(gate, _pose(1.0 + 0.01 * i, 1.0), 0.1 * i)
    outlier = _good(gate, _pose(9.0, 9.0), 0.4)       # 8 m 점프
    assert not outlier.accepted and outlier.reason == "position_jump"
    # 이어지는 정상 관측들
    assert not _good(gate, _pose(9.02, 9.0), 0.5).accepted   # streak 재시작
    assert _good(gate, _pose(9.03, 9.0), 0.6).accepted


def test_rejects_yaw_jump() -> None:
    gate = ArucoGate(GateLimits(marker_id=7, required_consecutive=1,
                                max_yaw_jump_deg=20.0))
    _good(gate, _pose(yaw_deg=0.0), 0.0)
    result = _good(gate, _pose(yaw_deg=60.0), 0.1)
    assert not result.accepted and result.reason == "yaw_jump"


def test_yaw_jump_uses_shortest_angle() -> None:
    """359 도 -> 1 도 는 2 도 회전이지 358 도가 아니다."""
    gate = ArucoGate(GateLimits(marker_id=7, required_consecutive=1,
                                max_yaw_jump_deg=20.0))
    _good(gate, _pose(yaw_deg=179.0), 0.0)
    assert _good(gate, _pose(yaw_deg=-179.0), 0.1).accepted


def test_reacquisition_after_a_long_gap_is_allowed() -> None:
    """오래 못 본 뒤 첫 관측은 튀는 게 정상이다. 점프 검사를 적용하면
    영원히 재획득하지 못한다 (spec section 20)."""
    gate = ArucoGate(GateLimits(marker_id=7, required_consecutive=1,
                                max_position_jump_m=0.5, reacquire_after_s=1.0))
    assert _good(gate, _pose(0, 0), 0.0).accepted
    # 5 초 공백 뒤 3 m 떨어진 곳에서 재획득
    assert _good(gate, _pose(3.0, 0.0), 5.0).accepted
    assert "position_jump" not in gate.rejected


# --------------------------------------------------------------------------
# 점진 보정
# --------------------------------------------------------------------------

def test_first_correction_is_taken_whole() -> None:
    correction = GroundCorrection()
    result = correction.update(_pose(2.0, 1.0, 30.0), dt=0.1)
    assert np.allclose(result[:2, 3], [2.0, 1.0])
    assert abs(math.degrees(yaw_of(result)) - 30.0) < 1e-9


def test_later_corrections_are_rate_limited() -> None:
    """받아들인 관측이 조금 틀렸어도 로봇이 튀지 않아야 한다."""
    correction = GroundCorrection(max_speed_mps=0.3, max_yaw_rate_dps=20.0)
    correction.update(_pose(0, 0, 0), dt=0.1)
    result = correction.update(_pose(5.0, 0.0, 90.0), dt=0.1)
    assert abs(result[0, 3] - 0.03) < 1e-9, result[0, 3]          # 0.3 m/s * 0.1 s
    assert abs(math.degrees(yaw_of(result)) - 2.0) < 1e-6         # 20 deg/s * 0.1 s


def test_correction_converges_to_the_target() -> None:
    correction = GroundCorrection(max_speed_mps=0.3, max_yaw_rate_dps=20.0)
    target = _pose(1.0, -0.5, 25.0)
    correction.update(_pose(0, 0, 0), dt=0.1)
    for _ in range(200):
        result = correction.update(target, dt=0.1)
    assert np.linalg.norm(result[:2, 3] - target[:2, 3]) < 1e-9
    assert abs(math.degrees(yaw_of(result)) - 25.0) < 1e-6


def test_correction_flattens_tilt() -> None:
    """카메라가 12.5 도 기울어 장착돼 있어 관측에 roll/pitch 가 섞인다.
    그것이 map 보정에 실리면 로봇이 지면에서 뜬다."""
    correction = GroundCorrection()
    tilted = _pose(1.0, 2.0, 40.0, z=0.6)
    tilted[:3, :3] = tilted[:3, :3] @ np.array(
        [[1, 0, 0], [0, math.cos(0.2), -math.sin(0.2)], [0, math.sin(0.2), math.cos(0.2)]])
    result = correction.update(tilted, dt=0.1)
    assert abs(result[2, 3]) < 1e-12
    assert abs(math.degrees(yaw_of(result)) - 40.0) < 1e-6
    assert np.allclose(result[:3, :3] @ [0, 0, 1.0], [0, 0, 1.0], atol=1e-12)


def test_zero_dt_does_not_move() -> None:
    correction = GroundCorrection()
    correction.update(_pose(0, 0, 0), dt=0.1)
    result = correction.update(_pose(9, 9, 90), dt=0.0)
    assert np.allclose(result[:2, 3], [0, 0])


if __name__ == "__main__":
    for name, function in sorted(globals().items()):
        if name.startswith("test_"):
            function()
            print(f"PASS  {name}")
    print("\nall gating tests passed")
