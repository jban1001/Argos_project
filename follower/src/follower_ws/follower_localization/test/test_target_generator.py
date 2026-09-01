"""궤적 추종 목표 생성 검증 (spec section 22).

핵심 주장은 하나다: **메인의 현재 위치를 쫓으면 코너에서 대각선으로 지르고,
궤적을 따라가면 지르지 않는다.** 그것을 가상 궤적으로 직접 비교한다.

    python3 src/follower_localization/test/test_target_generator.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from follower_localization.target_generator import (  # noqa: E402
    TargetLimits, TrajectoryFollower, steering_command)


def _l_corner(step=0.05, first=3.0, second=3.0):
    """ㄱ 자 궤적: +x 로 first 만큼 갔다가 (first, 0) 에서 +y 로 second 만큼 꺾는다.

    second 를 짧게 잡으면 "메인이 방금 코너를 돌았다" 는 상황이 된다. 그때
    follow_distance 만큼 거슬러 올라가면 목표는 코너 반대편(첫 다리) 에 놓이고,
    현재 위치 추종과의 차이가 드러난다.
    """
    points = [(x, 0.0) for x in np.arange(0.0, first + 1e-9, step)]
    points += [(first, y) for y in np.arange(step, second + 1e-9, step)]
    return points


def _feed(points, limits=None) -> TrajectoryFollower:
    follower = TrajectoryFollower(limits)
    for x, y in points:
        follower.add(x, y)
    return follower


# --------------------------------------------------------------------------
# 기본 동작
# --------------------------------------------------------------------------

def test_target_lies_on_the_path_at_the_requested_distance() -> None:
    follower = _feed([(x, 0.0) for x in np.arange(0, 5.0, 0.05)])
    target = follower.target(1.0)
    assert target is not None and not target.clamped
    # 끝이 (4.95, 0) 이므로 1 m 뒤는 (3.95, 0)
    assert abs(target.position[0] - 3.95) < 1e-9
    assert abs(target.position[1]) < 1e-12
    assert abs(math.degrees(target.yaw)) < 1e-9      # 진행 방향 +x


def test_distance_is_measured_along_the_path_not_straight_line() -> None:
    """코너를 넘어갈 때 두 값이 다르다. 직선으로 재면 목표가 경로를 벗어난다.

    메인이 코너를 돈 지 0.5 m 밖에 안 됐는데 1 m 뒤를 찾으므로, 목표는
    코너 건너편 첫 다리에 놓인다.
    """
    follower = _feed(_l_corner(second=0.5))
    target = follower.target(1.0)
    assert target is not None
    # 목표는 경로 위에 있어야 한다: 두 다리 중 하나 위
    # 목표는 첫 다리 위에 있어야 한다 (코너를 넘어갔으므로)
    assert abs(target.position[1]) < 1e-9, target.position
    assert abs(target.position[0] - 2.5) < 1e-9, target.position
    # 끝점 (3.0, 0.5) 에서 목표 (2.5, 0) 까지 직선거리는 1 m 보다 훨씬 짧다
    end = follower.points[-1]
    straight = float(np.linalg.norm(end - target.position))
    assert abs(straight - math.hypot(0.5, 0.5)) < 1e-9, straight
    assert straight < 1.0 - 0.2, straight


def test_short_history_is_clamped_not_extrapolated() -> None:
    """궤적이 짧으면 없는 과거를 지어내지 않고 가장 오래된 점으로 자른다."""
    follower = _feed([(x, 0.0) for x in np.arange(0, 0.5, 0.05)])
    target = follower.target(2.0)
    assert target is not None and target.clamped
    assert np.allclose(target.position, [0.0, 0.0])


def test_no_target_before_enough_history() -> None:
    follower = TrajectoryFollower(TargetLimits(min_history_m=0.15))
    assert follower.target() is None
    follower.add(0.0, 0.0)
    assert follower.target() is None
    follower.add(0.05, 0.0)
    assert follower.target() is None      # 0.05 m 뿐
    for x in np.arange(0.10, 0.40, 0.05):
        follower.add(float(x), 0.0)
    assert follower.target() is not None


# --------------------------------------------------------------------------
# 핵심 주장: 코너 지르기
# --------------------------------------------------------------------------

def test_path_following_does_not_cut_the_corner() -> None:
    """현재 위치 추종과 궤적 추종을 같은 코너에서 비교한다.

    팔로워가 첫 다리 위에 있고 메인이 코너를 돌아 두 번째 다리로 간 순간,
    현재 위치를 쫓으면 조향이 코너 바깥(대각선)을 향하고, 궤적을 쫓으면
    코너를 향한다.
    """
    follower = _feed(_l_corner(second=0.5))
    main_now = follower.points[-1]                 # 메인은 코너를 막 돈 (3.0, 0.5)
    me = np.array([2.4, 0.0])                      # 팔로워는 코너 0.6 m 앞
    my_yaw = 0.0                                   # +x 를 향함

    def bearing(goal):
        d = np.asarray(goal) - me
        return math.degrees(math.atan2(d[1], d[0]))

    target = follower.target(1.0)
    assert target is not None
    chase_bearing = bearing(main_now)              # 현재 위치를 쫓을 때
    path_bearing = bearing(target.position)        # 궤적을 쫓을 때

    # 현재 위치를 쫓으면 벌써 코너 안쪽(왼쪽)으로 조향한다 -> 대각선으로 지른다
    assert chase_bearing > 30.0, chase_bearing
    # 궤적을 쫓으면 아직 정면이다 -- 코너에 도달한 뒤에 돈다
    assert abs(path_bearing) < 1e-6, path_bearing
    assert chase_bearing - path_bearing > 25.0


def test_target_walks_the_path_as_main_advances() -> None:
    """메인이 진행하면 목표도 경로를 따라 이동하고, 항상 경로 위에 있다."""
    follower = TrajectoryFollower(TargetLimits(follow_distance_m=1.0))
    positions = []
    for x, y in _l_corner():
        follower.add(x, y)
        target = follower.target()
        if target is not None and not target.clamped:
            positions.append(target.position.copy())
    assert len(positions) > 50
    for position in positions:
        on_first = abs(position[1]) < 1e-6 and -1e-6 <= position[0] <= 3.0 + 1e-6
        on_second = abs(position[0] - 3.0) < 1e-6 and position[1] >= -1e-6
        assert on_first or on_second, position
    # 목표가 뒤로 가지 않는다 (경로를 따라 단조 증가)
    steps = [float(np.linalg.norm(b - a)) for a, b in zip(positions, positions[1:])]
    assert max(steps) < 0.2, max(steps)


# --------------------------------------------------------------------------
# 메모리와 잡음
# --------------------------------------------------------------------------

def test_history_is_bounded() -> None:
    """무한정 쌓지 않는다 (spec section 22)."""
    follower = TrajectoryFollower(TargetLimits(history_length_m=2.0))
    for x in np.arange(0, 50.0, 0.05):
        follower.add(float(x), 0.0)
    assert follower.total_length <= 2.0 + 1e-6
    assert len(follower) < 60


def test_stationary_main_does_not_fill_memory() -> None:
    """메인이 멈춰 있어도 같은 자리가 무한히 쌓이면 안 된다."""
    follower = TrajectoryFollower(TargetLimits(min_spacing_m=0.02))
    follower.add(1.0, 1.0)
    added = sum(follower.add(1.0 + 0.001 * i, 1.0) for i in range(500))
    assert added < 30, added


def test_non_finite_points_are_rejected() -> None:
    follower = TrajectoryFollower()
    assert follower.add(0.0, 0.0)
    assert not follower.add(float("nan"), 0.0)
    assert not follower.add(1.0, float("inf"))
    assert len(follower) == 1


# --------------------------------------------------------------------------
# 명령 생성 (모터는 돌리지 않는다. 문자열만 확인한다.)
# --------------------------------------------------------------------------

_CMD = dict(max_throttle=140, min_throttle=90, distance_deadband_m=0.08,
            angle_deadband_deg=7.0, steering_gain_dps_per_deg=0.5,
            max_yaw_rate_dps=18.0, throttle_gain_per_m=160.0)


def test_command_stops_inside_the_deadband() -> None:
    from follower_localization.target_generator import Target
    target = Target(np.array([0.03, 0.0]), 0.0, 1.0, False)
    assert steering_command([0.0, 0.0], 0.0, target, **_CMD) == "S"


def test_command_is_within_mcu_limits() -> None:
    """브리지가 다시 검사하지만, 여기서부터 범위를 벗어나면 안 된다."""
    from follower_localization.target_generator import Target
    import re
    for dx, dy, yaw in [(1.0, 0.0, 0.0), (5.0, 5.0, 0.0), (0.5, -3.0, 2.0),
                        (-2.0, 0.1, 0.0), (0.2, 0.0, -3.0)]:
        target = Target(np.array([dx, dy]), 0.0, 1.0, False)
        command = steering_command([0.0, 0.0], yaw, target, **_CMD)
        if command == "S":
            continue
        match = re.fullmatch(r"C,(-?\d+),(-?\d+\.\d)", command)
        assert match, command
        throttle, rate = int(match.group(1)), float(match.group(2))
        assert 0 < throttle <= 140, command
        assert abs(rate) <= 18.0, command


def test_throttle_scale_applies() -> None:
    """VIO_DEAD_RECKONING 에서 감속이 실제로 명령에 반영되어야 한다."""
    from follower_localization.target_generator import Target
    target = Target(np.array([2.0, 0.0]), 0.0, 1.0, False)
    full = steering_command([0.0, 0.0], 0.0, target, **_CMD)
    half = steering_command([0.0, 0.0], 0.0, target, scale=0.5, **_CMD)
    assert int(full.split(",")[1]) > int(half.split(",")[1])


def test_steering_turns_toward_the_target() -> None:
    from follower_localization.target_generator import Target
    left = Target(np.array([1.0, 1.0]), 0.0, 1.0, False)
    right = Target(np.array([1.0, -1.0]), 0.0, 1.0, False)
    assert float(steering_command([0, 0], 0.0, left, **_CMD).split(",")[2]) > 0
    assert float(steering_command([0, 0], 0.0, right, **_CMD).split(",")[2]) < 0


if __name__ == "__main__":
    for name, function in sorted(globals().items()):
        if name.startswith("test_"):
            function()
            print(f"PASS  {name}")
    print("\nall target generator tests passed")


# --- 마커 직접 추종 -------------------------------------------------------
# 펌웨어는 좌우 PWM 을 throttle +- turnCorrection 으로 만든다. 주행 중 yaw 가
# 크면 느린 쪽이 궤도가 못 도는 값까지 떨어져 한쪽만 조금 돌다 만다.

from follower_localization.target_generator import direct_marker_command  # noqa: E402

_LIMITS = dict(follow_distance_m=1.0, max_throttle=160, min_throttle=140,
               distance_deadband_m=0.15, angle_deadband_deg=5.0,
               steering_gain_dps_per_deg=0.5, max_yaw_rate_dps=12.0,
               throttle_gain_per_m=60.0)


def test_large_bearing_pivots_in_place():
    """크게 틀어지면 전진하지 않는다. 차동으로 억지로 휘면 한쪽이 죽는다."""
    command = direct_marker_command(range_m=2.0, bearing_deg=40.0, **_LIMITS)
    assert command.startswith("C,0,"), command


def test_small_bearing_drives_with_gentle_yaw():
    command = direct_marker_command(range_m=2.0, bearing_deg=10.0, **_LIMITS)
    throttle, yaw = command.split(",")[1:]
    assert int(throttle) >= 140, command
    assert abs(float(yaw)) <= 12.0, command


def test_yaw_never_stalls_the_slower_track():
    """느린 쪽 PWM 이 궤도가 도는 값 아래로 내려가면 안 된다."""
    for bearing in (5.0, 10.0, 15.0, 19.0):
        command = direct_marker_command(range_m=3.0, bearing_deg=bearing, **_LIMITS)
        throttle, yaw = command.split(",")[1:]
        throttle, yaw = int(throttle), float(yaw)
        # 펌웨어: turnCorrection = YAW_KP(2.0)*yaw + 커브FF(min(2.0*|yaw|, 40))
        turn = 2.0 * abs(yaw) + min(2.0 * abs(yaw), 40.0)
        turn = min(turn, abs(throttle) * 0.85)
        assert throttle - turn >= 90, (command, throttle - turn)


def test_at_distance_but_off_angle_turns_in_place():
    command = direct_marker_command(range_m=1.0, bearing_deg=10.0, **_LIMITS)
    assert command.startswith("C,0,"), command


def test_on_target_stops():
    assert direct_marker_command(range_m=1.0, bearing_deg=0.0, **_LIMITS) == "S"


def test_too_close_does_not_reverse():
    command = direct_marker_command(range_m=0.3, bearing_deg=0.0, **_LIMITS)
    assert command == "S", command
