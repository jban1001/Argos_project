"""팔로워 상태머신 (spec section 20, 23).

네 상태가 있고, 무엇을 근거로 조향할지가 상태마다 다르다.

    GLOBAL_FOLLOW       map 위치추정이 살아 있다 -> 메인 궤적을 추종
    ARUCO_LOCAL_FOLLOW  전역은 못 믿지만 ArUco 는 잘 보인다 -> 상대 추종
    VIO_DEAD_RECKONING  ArUco 가 잠깐 끊겼다 -> VIO 로 버티되 감속
    LOST                오래 못 봤다 -> 정지하고 재획득

설계에서 중요한 것
------------------
**전이 조건은 시간과 품질 둘 다로 정한다.** ArUco 를 마지막으로 본 시각만
보면, 잘 보이는데 품질이 나쁜 경우(멀거나 비스듬한 마커)를 놓친다.

**LOST 로 갈 때는 반드시 정지 명령을 낸다.** 위치추정 실패가 모터 폭주로
이어지면 안 된다 (spec section 29). 이 클래스는 명령을 문자열로 만들 뿐
직접 보내지 않으므로, 상위에서 시리얼 브리지로 넘기면 브리지가 문법과
범위를 다시 검사한다.

**히스테리시스를 둔다.** 경계에서 상태가 떨리면 감속과 가속을 반복해서
추종이 오히려 불안정해진다. 좋아질 때의 문턱을 나빠질 때보다 엄격하게 둔다.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


# 예정: 화재 대응 인계 (README 10 절)
#
# 메인 로봇이 화재를 탐지하면 그 자리를 비우고, 팔로워가 그 좌표로 가서 정지한
# 뒤 물을 발사한다. 그 동작은 추종이 아니라 **지점 이동**이므로 여기 상태가
# 하나 더 필요하다 (예: GOTO_POINT). 그때 주의할 것 두 가지:
#
#   - 마커는 의미가 없다. 메인 로봇이 비켰으므로 따라갈 대상이 없고, 가야 할
#     곳은 지도 좌표다. 즉 ARUCO_LOCAL_FOLLOW 가 아니라 전역 경로가 필요하고,
#     지금 그 경로는 보조로 밀려 있어 실주행에서 검증된 적이 없다.
#   - 발사는 되돌릴 수 없다. LOST 에서 반드시 S 를 내는 것과 같은 이유로,
#     자기 위치를 모르는 상태에서는 발사 경로가 아예 막혀 있어야 한다.
#
# 팔로워에 라이다가 붙으면 팔로워가 자기 AMCL 로 전역 자세를 얻을 수 있어
# 이 구조가 훨씬 단순해진다. 그 전까지는 전역 자세가 메인 AMCL x 마커 x VIO
# 세 가지에 모두 의존한다.


class FollowState(Enum):
    GLOBAL_FOLLOW = "GLOBAL_FOLLOW"
    ARUCO_LOCAL_FOLLOW = "ARUCO_LOCAL_FOLLOW"
    VIO_DEAD_RECKONING = "VIO_DEAD_RECKONING"
    LOST = "LOST"


@dataclass
class StateLimits:
    """전부 파라미터 (spec section 23). 실측 전이라 값은 잠정이다."""

    # ArUco 를 마지막으로 본 뒤 경과 시간 [s]
    aruco_fresh_s: float = 1.0        # 이 안이면 정상 추종
    aruco_stale_s: float = 3.0        # 이 넘으면 LOST

    # 전역 위치추정이 살아 있다고 볼 조건
    global_max_age_s: float = 1.5     # map -> odom 보정이 이보다 오래되면 못 믿는다
    main_pose_max_age_s: float = 1.0  # 메인 로봇 pose 가 이보다 오래되면 못 믿는다

    # VIO 품질
    vio_max_age_s: float = 0.5
    vio_max_position_sigma_m: float = 0.5

    # 감속 (spec section 20: 1~3 초 구간은 감속)
    slow_factor: float = 0.5

    # 히스테리시스: 좋아질 때는 더 엄격하게 본다
    recover_margin: float = 0.7


@dataclass
class Inputs:
    """상태머신이 보는 세상. 전부 상위에서 채워 준다."""

    now: float
    aruco_last_seen: float | None = None
    global_last_correction: float | None = None
    main_pose_stamp: float | None = None
    vio_last_stamp: float | None = None
    vio_position_sigma_m: float = 0.0
    have_trajectory: bool = False

    def age(self, stamp: float | None) -> float:
        return float("inf") if stamp is None else max(0.0, self.now - stamp)


@dataclass
class Decision:
    state: FollowState
    reason: str
    throttle_scale: float
    stop_command: str | None = None


class FollowerStateMachine:
    def __init__(self, limits: StateLimits | None = None) -> None:
        self.limits = limits or StateLimits()
        self.state = FollowState.LOST
        self.entered_at: float | None = None
        self.transitions = 0

    def _threshold(self, base: float, improving: bool) -> float:
        """좋아지는 방향으로 갈 때는 문턱을 좁힌다."""
        return base * self.limits.recover_margin if improving else base

    def update(self, inputs: Inputs) -> Decision:
        limits = self.limits
        aruco_age = inputs.age(inputs.aruco_last_seen)
        global_age = inputs.age(inputs.global_last_correction)
        main_age = inputs.age(inputs.main_pose_stamp)
        vio_age = inputs.age(inputs.vio_last_stamp)

        vio_ok = (vio_age <= limits.vio_max_age_s
                  and inputs.vio_position_sigma_m <= limits.vio_max_position_sigma_m)

        # "좋은 상태" 는 마커를 직접 보고 있는 상태뿐이다. GLOBAL_FOLLOW 는
        # 마커를 이미 놓쳐 지도 궤적으로 물러난 상태이므로, 거기서 마커
        # 추종으로 돌아올 때는 더 엄격한 문턱을 써야 한 번 내려간 상태가
        # 경계에서 떨지 않는다.
        was_good = self.state is FollowState.ARUCO_LOCAL_FOLLOW
        fresh_limit = self._threshold(limits.aruco_fresh_s, improving=not was_good)

        # 마커가 보이면 그것으로 따라간다. 이것이 주 경로다.
        #
        # 전에는 여기서 VIO 를 먼저 요구했다 (not vio_ok -> LOST). 그래서
        # 마커가 완벽히 보이는데도 VIO 나 지도 보정이 어긋나면 멈췄다
        # (2026-08-29 실측: reason "vio unusable" 로 계속 LOST). 상대 추종은
        # 마커 하나만 있으면 되는 계산이므로 VIO 를 요구할 이유가 없다.
        # 의존이 많은 쪽을 주 경로로 두면 부품이 다 멀쩡해도 전체가 자주
        # 죽는다 -- 각 단이 90% 여도 4 단이면 65% 다.
        if aruco_age <= fresh_limit:
            return self._settle(FollowState.ARUCO_LOCAL_FOLLOW,
                                "marker in view", inputs.now)

        # 여기부터는 마커를 놓친 경우의 보조 수단이다.
        if aruco_age <= limits.aruco_stale_s:
            global_ok = (global_age <= limits.global_max_age_s
                         and main_age <= limits.main_pose_max_age_s
                         and inputs.have_trajectory)
            if global_ok:
                return self._settle(
                    FollowState.GLOBAL_FOLLOW,
                    f"marker {aruco_age:.2f}s old, following map trajectory",
                    inputs.now)
            if vio_ok:
                return self._settle(
                    FollowState.VIO_DEAD_RECKONING,
                    f"marker {aruco_age:.2f}s old, coasting on VIO", inputs.now)

        # 마커도 없고 물러날 곳도 없다.
        if aruco_age > limits.aruco_stale_s:
            reason = f"marker lost for {aruco_age:.2f}s"
        elif not vio_ok:
            reason = (f"marker {aruco_age:.2f}s old and vio unusable "
                      f"(age {vio_age:.2f}s, "
                      f"sigma {inputs.vio_position_sigma_m:.2f}m)")
        else:
            reason = f"marker {aruco_age:.2f}s old, no fallback available"
        return self._settle(FollowState.LOST, reason, inputs.now)

    def _settle(self, state: FollowState, reason: str, now: float) -> Decision:
        if state is not self.state:
            self.state = state
            self.entered_at = now
            self.transitions += 1

        if state is FollowState.LOST:
            # 위치추정 실패가 모터 폭주로 이어지면 안 된다 (spec section 29).
            return Decision(state, reason, 0.0, stop_command="S")
        if state is FollowState.VIO_DEAD_RECKONING:
            return Decision(state, reason, self.limits.slow_factor)
        return Decision(state, reason, 1.0)
