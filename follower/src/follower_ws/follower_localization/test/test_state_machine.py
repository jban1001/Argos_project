"""상태머신 검증 (spec section 20, 23).

이 테스트가 확인하는 것은 "정상일 때 잘 도는가"가 아니라 **나빠질 때 안전한
쪽으로 가는가**이다. 위치추정 실패가 모터 폭주로 이어지면 안 된다.

    python3 src/follower_localization/test/test_state_machine.py
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from follower_localization.state_machine import (  # noqa: E402
    Decision, FollowerStateMachine, FollowState, Inputs, StateLimits)


def _healthy(now: float, **overrides) -> Inputs:
    """모든 것이 정상인 입력. 테스트마다 한 가지만 망가뜨린다."""
    base = dict(now=now, aruco_last_seen=now - 0.05,
                global_last_correction=now - 0.1, main_pose_stamp=now - 0.1,
                vio_last_stamp=now - 0.01, vio_position_sigma_m=0.05,
                have_trajectory=True)
    base.update(overrides)
    return Inputs(**base)


def _run(machine: FollowerStateMachine, sequence) -> list[Decision]:
    return [machine.update(inputs) for inputs in sequence]


def test_marker_in_view_follows_the_marker_directly() -> None:
    """마커가 보이면 마커로 따라간다. 이것이 주 경로다."""
    machine = FollowerStateMachine()
    decision = machine.update(_healthy(10.0))
    assert decision.state is FollowState.ARUCO_LOCAL_FOLLOW
    assert decision.throttle_scale == 1.0
    assert decision.stop_command is None


def test_aruco_gap_coasts_on_vio_at_reduced_speed() -> None:
    """1~3 초 구간은 감속하며 VIO 로 버틴다 (spec section 20)."""
    machine = FollowerStateMachine(StateLimits(aruco_fresh_s=1.0, aruco_stale_s=3.0,
                                               slow_factor=0.5))
    machine.update(_healthy(10.0))
    # 마커를 놓쳤고 지도 궤적도 없다 -> 마지막 수단인 VIO 로 버틴다.
    decision = machine.update(_healthy(12.0, aruco_last_seen=10.0,
                                       have_trajectory=False))
    assert decision.state is FollowState.VIO_DEAD_RECKONING
    assert decision.throttle_scale == 0.5
    assert decision.stop_command is None


def test_long_aruco_loss_stops_the_robot() -> None:
    """3 초를 넘기면 정지하고 재획득한다. 정지 명령이 반드시 나와야 한다."""
    machine = FollowerStateMachine(StateLimits(aruco_stale_s=3.0))
    machine.update(_healthy(10.0))
    decision = machine.update(_healthy(14.0, aruco_last_seen=10.0))
    assert decision.state is FollowState.LOST
    assert decision.throttle_scale == 0.0
    assert decision.stop_command == "S"


def test_bad_vio_does_not_stop_us_while_the_marker_is_visible() -> None:
    """VIO 가 죽어도 마커가 보이면 계속 따라간다.

    전에는 반대였다: VIO 를 먼저 요구해서, 마커가 완벽히 보이는데도
    "vio unusable" 로 멈췄다 (2026-08-29 실측). 상대 추종은 마커 하나로
    끝나는 계산이므로 VIO 를 요구할 이유가 없다. 의존이 많은 경로를 주
    경로로 두면 부품이 다 멀쩡해도 전체가 자주 죽는다.
    """
    machine = FollowerStateMachine(StateLimits(vio_max_position_sigma_m=0.5))
    machine.update(_healthy(10.0))
    decision = machine.update(_healthy(10.1, vio_position_sigma_m=2.0))
    assert decision.state is FollowState.ARUCO_LOCAL_FOLLOW
    assert decision.stop_command is None

    machine2 = FollowerStateMachine(StateLimits(vio_max_age_s=0.5))
    machine2.update(_healthy(10.0))
    stale = machine2.update(_healthy(10.1, vio_last_stamp=8.0))
    assert stale.state is FollowState.ARUCO_LOCAL_FOLLOW


def test_marker_lost_and_vio_dead_stops() -> None:
    """마커도 놓치고 물러날 곳도 없으면 정지한다."""
    machine = FollowerStateMachine(StateLimits(aruco_fresh_s=1.0, aruco_stale_s=3.0,
                                               vio_max_position_sigma_m=0.5))
    machine.update(_healthy(10.0))
    decision = machine.update(_healthy(12.0, aruco_last_seen=10.0,
                                       have_trajectory=False,
                                       vio_position_sigma_m=2.0))
    assert decision.state is FollowState.LOST
    assert decision.stop_command == "S"


def test_stale_main_pose_falls_back_to_relative_follow() -> None:
    """메인 로봇 pose 가 끊기면 전역 추종은 못 하지만, ArUco 가 보이면
    상대 추종은 계속할 수 있다 (spec section 23, STATE 2)."""
    machine = FollowerStateMachine(StateLimits(main_pose_max_age_s=1.0))
    machine.update(_healthy(10.0))
    decision = machine.update(_healthy(10.1, main_pose_stamp=5.0))
    assert decision.state is FollowState.ARUCO_LOCAL_FOLLOW
    assert decision.throttle_scale == 1.0


def test_missing_trajectory_falls_back_to_relative_follow() -> None:
    machine = FollowerStateMachine()
    machine.update(_healthy(10.0))
    decision = machine.update(_healthy(10.1, have_trajectory=False))
    assert decision.state is FollowState.ARUCO_LOCAL_FOLLOW


def test_stale_map_correction_falls_back() -> None:
    machine = FollowerStateMachine(StateLimits(global_max_age_s=1.5))
    machine.update(_healthy(10.0))
    decision = machine.update(_healthy(10.1, global_last_correction=5.0))
    assert decision.state is FollowState.ARUCO_LOCAL_FOLLOW


def test_hysteresis_prevents_chattering_at_the_boundary() -> None:
    """경계에서 상태가 떨리면 감속과 가속을 반복해 추종이 불안정해진다.

    ArUco 나이가 문턱을 살짝 오르내릴 때, 나빠질 때보다 좋아질 때 더
    엄격해야 한 번 내려간 상태가 쉽게 되돌아오지 않는다.
    """
    limits = StateLimits(aruco_fresh_s=1.0, recover_margin=0.7)
    machine = FollowerStateMachine(limits)
    machine.update(_healthy(10.0))
    # 1.05 초 -> 마커를 놓친 것으로 보고 지도 궤적으로 물러난다
    assert machine.update(_healthy(11.05, aruco_last_seen=10.0)).state \
        is FollowState.GLOBAL_FOLLOW
    # 0.9 초로 좋아져도 아직 안 돌아온다 (문턱 0.7 초)
    assert machine.update(_healthy(11.9, aruco_last_seen=11.0)).state \
        is FollowState.GLOBAL_FOLLOW
    # 0.5 초까지 좋아지면 마커 직접 추종으로 돌아온다
    assert machine.update(_healthy(11.5, aruco_last_seen=11.0)).state \
        is FollowState.ARUCO_LOCAL_FOLLOW


def test_recovery_sequence_matches_the_spec() -> None:
    """보임 -> 짧은 끊김 -> 긴 끊김 -> 재획득 (spec section 20)."""
    machine = FollowerStateMachine(StateLimits(aruco_fresh_s=1.0, aruco_stale_s=3.0))
    seen = 100.0
    states = [
        machine.update(_healthy(100.0, aruco_last_seen=seen)).state,
        machine.update(_healthy(102.0, aruco_last_seen=seen)).state,
        machine.update(_healthy(104.0, aruco_last_seen=seen)).state,
        machine.update(_healthy(105.0, aruco_last_seen=105.0)).state,
    ]
    assert states == [FollowState.ARUCO_LOCAL_FOLLOW,
                      FollowState.GLOBAL_FOLLOW,
                      FollowState.LOST,
                      FollowState.ARUCO_LOCAL_FOLLOW]


def test_never_started_starts_in_lost() -> None:
    """부팅 직후에는 아무것도 모른다. 안전한 쪽에서 시작해야 한다."""
    machine = FollowerStateMachine()
    assert machine.state is FollowState.LOST
    decision = machine.update(Inputs(now=0.0))
    assert decision.state is FollowState.LOST
    assert decision.stop_command == "S"


def test_transition_count_and_timestamp() -> None:
    machine = FollowerStateMachine(StateLimits(aruco_stale_s=3.0))
    machine.update(_healthy(10.0))
    assert machine.entered_at == 10.0
    before = machine.transitions
    machine.update(_healthy(10.1))          # 같은 상태 -> 전이 없음
    assert machine.transitions == before
    machine.update(_healthy(20.0, aruco_last_seen=10.0))
    assert machine.transitions == before + 1
    assert machine.entered_at == 20.0


if __name__ == "__main__":
    for name, function in sorted(globals().items()):
        if name.startswith("test_"):
            function()
            print(f"PASS  {name}")
    print("\nall state machine tests passed")
