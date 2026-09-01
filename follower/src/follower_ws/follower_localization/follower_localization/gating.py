"""ArUco 관측 게이팅과 map -> odom 의 점진 보정.

왜 게이팅이 필요한가 (spec section 18)
--------------------------------------
ArUco 한 프레임의 오검출로 전역 TF 가 튀면, 그 순간 로봇은 지도 위 엉뚱한
곳으로 순간이동한다. 상위 제어는 그것을 실제 위치로 믿고 조향한다. 그래서
관측을 받아들이기 전에 거를 것을 거른다.

왜 EKF 가 아니라 점진 보정인가 (spec section 19)
------------------------------------------------
EKF 를 쓰면 프로세스 노이즈와 관측 노이즈를 둘 다 정해야 하는데, 이 시스템의
ArUco 오차는 거리와 각도에 따라 크게 변하고 아직 특성을 측정하지 않았다.
근거 없는 공분산을 넣는 것보다, 게이팅으로 이상치를 막고 남은 것을 천천히
반영하는 편이 낫다. 튜닝할 값이 "얼마나 빨리 따라갈 것인가" 하나뿐이고,
그 값은 물리적으로 해석된다.

지상 로봇이므로 보정은 x, y, yaw 에만 건다 (spec section 19). 카메라가
12.5 도 기울어 장착돼 있어 ArUco 관측에는 항상 약간의 roll/pitch 가 섞이는데,
그것을 map 보정에 실으면 로봇이 지면에서 뜬 것처럼 된다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .transforms import flatten_to_ground, inverse, yaw_of


@dataclass
class GateLimits:
    """전부 파라미터로 노출한다 (spec section 18). 값은 <CONFIGURE> 다."""

    marker_id: int = -1                 # -1 이면 아무 마커나. 실전에서는 반드시 지정
    min_distance_m: float = 0.25
    max_distance_m: float = 4.0
    max_reprojection_px: float = 3.0
    max_position_jump_m: float = 0.5
    max_yaw_jump_deg: float = 20.0
    required_consecutive: int = 3
    # 이 시간이 지나면 이전 관측과의 비교를 포기한다. 오래 못 본 뒤의 첫
    # 관측은 튀는 게 정상이므로 점프 검사를 적용하면 영원히 못 받아들인다.
    reacquire_after_s: float = 1.0


@dataclass
class GateResult:
    accepted: bool
    reason: str = ""
    consecutive: int = 0


class ArucoGate:
    """관측을 받아들일지 판정한다. 상태를 가지므로 인스턴스로 쓴다."""

    def __init__(self, limits: GateLimits | None = None) -> None:
        self.limits = limits or GateLimits()
        self._last_pose: np.ndarray | None = None
        self._last_time: float | None = None
        self._streak = 0
        self.rejected: dict[str, int] = {}

    def _reject(self, reason: str) -> GateResult:
        self.rejected[reason] = self.rejected.get(reason, 0) + 1
        self._streak = 0
        return GateResult(False, reason, 0)

    def check(self, marker_id: int, t_map_follower: np.ndarray, distance_m: float,
              reprojection_px: float, stamp: float) -> GateResult:
        limits = self.limits

        if limits.marker_id >= 0 and marker_id != limits.marker_id:
            return self._reject("wrong_id")
        if not np.isfinite(t_map_follower).all():
            return self._reject("not_finite")
        if not (limits.min_distance_m <= distance_m <= limits.max_distance_m):
            return self._reject("distance")
        if reprojection_px > limits.max_reprojection_px:
            return self._reject("reprojection")

        # 점프 검사는 직전 관측이 충분히 최근일 때만 의미가 있다.
        stale = (self._last_time is None
                 or stamp - self._last_time > limits.reacquire_after_s)
        if not stale and self._last_pose is not None:
            jump = float(np.linalg.norm(t_map_follower[:2, 3] - self._last_pose[:2, 3]))
            turn = abs(math.degrees(
                math.atan2(math.sin(yaw_of(t_map_follower) - yaw_of(self._last_pose)),
                           math.cos(yaw_of(t_map_follower) - yaw_of(self._last_pose)))))
            if jump > limits.max_position_jump_m:
                self._last_pose, self._last_time = t_map_follower.copy(), stamp
                return self._reject("position_jump")
            if turn > limits.max_yaw_jump_deg:
                self._last_pose, self._last_time = t_map_follower.copy(), stamp
                return self._reject("yaw_jump")

        self._last_pose = t_map_follower.copy()
        self._last_time = stamp
        self._streak += 1
        if self._streak < limits.required_consecutive:
            return GateResult(False, "warming_up", self._streak)
        return GateResult(True, "", self._streak)


class GroundCorrection:
    """map -> follower_odom 을 천천히 목표값으로 끌어간다.

    한 관측이 들어올 때마다 목표 변환으로 순간 이동시키지 않고, x, y 는
    최대 속도, yaw 는 최대 각속도로 제한해 따라간다. 제한을 두면 받아들인
    관측이 조금 틀렸더라도 로봇이 튀지 않고, 계속 같은 방향으로 틀렸을
    때만 결국 반영된다.
    """

    def __init__(self, max_speed_mps: float = 0.30,
                 max_yaw_rate_dps: float = 20.0) -> None:
        self.max_speed = max_speed_mps
        self.max_yaw_rate = math.radians(max_yaw_rate_dps)
        self._current: np.ndarray | None = None
        self.applied = 0

    @property
    def transform(self) -> np.ndarray | None:
        """현재 발행해야 할 map -> follower_odom. 아직 없으면 None."""
        return None if self._current is None else self._current.copy()

    def reset(self) -> None:
        self._current = None

    def update(self, target: np.ndarray, dt: float) -> np.ndarray:
        target = flatten_to_ground(target)
        if self._current is None:
            # 첫 관측은 그대로 받는다. 초기값이 없으면 끌어갈 대상도 없다.
            self._current = target
            self.applied += 1
            return self._current.copy()

        dt = max(dt, 0.0)
        delta = target[:2, 3] - self._current[:2, 3]
        distance = float(np.linalg.norm(delta))
        limit = self.max_speed * dt
        if distance > limit > 0.0:
            delta = delta * (limit / distance)
        elif limit <= 0.0:
            delta = np.zeros(2)

        turn = math.atan2(math.sin(yaw_of(target) - yaw_of(self._current)),
                          math.cos(yaw_of(target) - yaw_of(self._current)))
        yaw_limit = self.max_yaw_rate * dt
        turn = max(-yaw_limit, min(yaw_limit, turn))

        yaw = yaw_of(self._current) + turn
        result = np.eye(4)
        result[0, 0] = math.cos(yaw)
        result[0, 1] = -math.sin(yaw)
        result[1, 0] = math.sin(yaw)
        result[1, 1] = math.cos(yaw)
        result[:2, 3] = self._current[:2, 3] + delta
        self._current = result
        self.applied += 1
        return self._current.copy()
