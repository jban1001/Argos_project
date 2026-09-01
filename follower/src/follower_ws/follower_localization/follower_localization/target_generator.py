"""메인 로봇 궤적을 일정 거리 뒤에서 따라가는 목표점 생성 (spec section 22).

왜 현재 위치를 그냥 쫓지 않는가
-------------------------------
메인 로봇의 현재 위치를 목표로 삼으면, 코너에서 팔로워는 직선으로 질러간다.
메인이 ㄱ 자로 돌면 팔로워는 대각선으로 자르고, 그 사이에 벽이나 장애물이
있으면 부딪힌다. 메인이 **실제로 지나간 자리**를 따라가야 한다.

그래서 궤적을 쌓아 두고, 끝에서부터 경로를 따라 `follow_distance` 만큼
거슬러 올라간 지점을 목표로 준다. 그 지점은 항상 메인이 실제로 통과한
자리다.

거리는 직선거리가 아니라 **경로를 따라 잰 길이**다. 직선거리로 재면 코너
안쪽에서 목표가 경로를 벗어난다.

메모리
------
궤적을 무한정 쌓지 않는다 (spec section 22). 추종에 필요한 것은 최근 구간
뿐이므로, 경로 길이 기준으로 잘라낸다. 시간 기준으로 자르면 메인이 멈춰
있을 때 필요한 구간까지 지워진다.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass
class TargetLimits:
    follow_distance_m: float = 1.0
    # 이 거리보다 촘촘한 점은 버린다. 메인이 멈춰 있을 때 같은 자리가
    # 무한히 쌓이는 것을 막는다.
    min_spacing_m: float = 0.02
    # 보관할 경로 길이. follow_distance 의 몇 배는 있어야 목표를 찾는다.
    history_length_m: float = 10.0
    # 궤적이 이만큼도 안 쌓였으면 목표를 만들지 않는다.
    min_history_m: float = 0.15


@dataclass
class Target:
    position: np.ndarray          # map 기준 [x, y]
    yaw: float                    # 그 지점에서 경로의 진행 방향 [rad]
    along_path_m: float           # 끝에서부터 경로를 따라 잰 거리
    clamped: bool                 # 궤적이 짧아 가장 오래된 점으로 잘렸는가


class TrajectoryFollower:
    """메인 궤적을 받아 팔로워의 목표점을 낸다."""

    def __init__(self, limits: TargetLimits | None = None) -> None:
        self.limits = limits or TargetLimits()
        self._points: deque[np.ndarray] = deque()
        self._segment: deque[float] = deque()   # _points[i-1] -> _points[i] 길이
        self.total_length = 0.0

    def __len__(self) -> int:
        return len(self._points)

    @property
    def points(self) -> list[np.ndarray]:
        return [p.copy() for p in self._points]

    def clear(self) -> None:
        self._points.clear()
        self._segment.clear()
        self.total_length = 0.0

    def add(self, x: float, y: float) -> bool:
        """궤적 점을 추가한다. 너무 촘촘하면 버리고 False 를 돌려준다."""
        point = np.array([float(x), float(y)])
        if not np.isfinite(point).all():
            return False
        if self._points:
            step = float(np.linalg.norm(point - self._points[-1]))
            if step < self.limits.min_spacing_m:
                return False
            self._points.append(point)
            self._segment.append(step)
            self.total_length += step
        else:
            self._points.append(point)

        while self.total_length > self.limits.history_length_m and len(self._segment) > 1:
            self.total_length -= self._segment.popleft()
            self._points.popleft()
        return True

    def target(self, distance_m: float | None = None) -> Target | None:
        """끝에서부터 경로를 따라 distance_m 거슬러 올라간 지점."""
        wanted = self.limits.follow_distance_m if distance_m is None else distance_m
        if len(self._points) < 2 or self.total_length < self.limits.min_history_m:
            return None

        # 끝에서부터 구간 길이를 더해 가며 목표 구간을 찾는다.
        remaining = wanted
        points = list(self._points)
        segments = list(self._segment)
        index = len(points) - 1
        clamped = False
        while index > 0 and remaining > segments[index - 1]:
            remaining -= segments[index - 1]
            index -= 1
        if index == 0 and remaining > 0.0:
            # 궤적이 요청한 거리보다 짧다. 가장 오래된 점으로 자른다.
            clamped = True
            remaining = 0.0

        ahead = points[index]
        if index == 0:
            position = ahead.copy()
            direction = points[1] - points[0]
            along = self.total_length
        else:
            behind = points[index - 1]
            length = segments[index - 1]
            # ahead 에서 behind 쪽으로 remaining 만큼 되돌아간다.
            ratio = 0.0 if length <= 0.0 else remaining / length
            position = ahead + (behind - ahead) * ratio
            direction = ahead - behind
            along = wanted

        norm = float(np.linalg.norm(direction))
        yaw = math.atan2(float(direction[1]), float(direction[0])) if norm > 1e-9 else 0.0
        return Target(position=position, yaw=yaw, along_path_m=along, clamped=clamped)


def steering_command(follower_xy, follower_yaw: float, target: Target,
                     max_throttle: int, min_throttle: int,
                     distance_deadband_m: float, angle_deadband_deg: float,
                     steering_gain_dps_per_deg: float, max_yaw_rate_dps: float,
                     throttle_gain_per_m: float, scale: float = 1.0) -> str:
    """목표점에서 MCU 명령 문자열을 만든다.

    문자열만 만들고 보내지 않는다. 실제 전송은 시리얼 브리지가 하고, 브리지가
    문법과 범위를 한 번 더 검사한다 (같은 검사를 두 곳에서 하는 것이 아니라,
    여기서는 제어 논리를, 브리지에서는 하드웨어 한계를 본다).
    """
    delta = np.asarray(target.position, dtype=float) - np.asarray(follower_xy, dtype=float)
    distance = float(np.linalg.norm(delta))
    heading = math.atan2(float(delta[1]), float(delta[0]))
    error = math.atan2(math.sin(heading - follower_yaw), math.cos(heading - follower_yaw))
    error_deg = math.degrees(error)

    if distance <= distance_deadband_m:
        return "S"

    throttle = int(round(throttle_gain_per_m * distance * scale))
    throttle = max(int(round(min_throttle * scale)), min(int(round(max_throttle * scale)),
                                                        throttle))
    if throttle <= 0:
        return "S"

    yaw_rate = 0.0 if abs(error_deg) <= angle_deadband_deg else \
        steering_gain_dps_per_deg * error_deg
    yaw_rate = max(-max_yaw_rate_dps, min(max_yaw_rate_dps, yaw_rate))
    return f"C,{throttle},{yaw_rate:.1f}"


def direct_marker_command(range_m: float, bearing_deg: float,
                          follow_distance_m: float,
                          max_throttle: int, min_throttle: int,
                          distance_deadband_m: float, angle_deadband_deg: float,
                          steering_gain_dps_per_deg: float,
                          max_yaw_rate_dps: float,
                          throttle_gain_per_m: float, scale: float = 1.0,
                          pivot_above_deg: float = 20.0) -> str:
    """마커만 보고 명령을 만든다 -- 지도도 VIO 도 쓰지 않는다.

    이것이 주 추종 경로다. 마커가 보이는 동안은 지도 위치추정이 어떻든
    상관없이 따라갈 수 있어야 한다. steering_command 는 지도 좌표계 궤적을
    쓰므로 AMCL/VIO/TF 가 모두 살아 있어야 하고, 그 중 하나만 어긋나도
    마커가 완벽히 보이는데도 멈춘다. 그때 물러날 곳이 여기다.

    range_m          마커까지 거리
    bearing_deg      로봇 기준 마커 방위. 좌가 양, 우가 음 (base_link 규약).
    pivot_above_deg  이보다 크게 틀어지면 전진하지 않고 제자리에서 돈다.
    """
    gap = range_m - follow_distance_m
    turn_needed = abs(bearing_deg) > angle_deadband_deg

    yaw_rate = 0.0 if not turn_needed else steering_gain_dps_per_deg * bearing_deg
    yaw_rate = max(-max_yaw_rate_dps, min(max_yaw_rate_dps, yaw_rate))

    # 방위가 크면 전진하지 않고 제자리에서 돈다.
    #
    # 펌웨어는 좌우 PWM 을 throttle +- turnCorrection 으로 만들고, 주행 중
    # turnCorrection 을 |throttle| x 0.85 까지 허용한다. throttle 140 에
    # yaw 45 deg/s 를 얹으면 한쪽 180, 다른 쪽 21 이 되어 -- 21 은 궤도가
    # 못 움직이는 값이라 한쪽만 조금 돌다 윙 소리만 났다 (2026-08-29 실측).
    #
    # 제자리 회전은 펌웨어가 throttle 0 일 때 TURN_FEED_FORWARD 90 으로
    # 기동 킥을 주므로 실제로 돈다. 크게 틀어졌을 때는 돌고 나서 가는 편이
    # 차동으로 억지로 휘는 것보다 확실하다.
    if abs(bearing_deg) > pivot_above_deg:
        return f"C,0,{yaw_rate:.1f}"

    # 거리는 맞는데 방향만 틀리면 제자리에서 돈다. 펌웨어가 throttle 0 일 때
    # 이것을 피벗으로 처리한다 (followingbot_mega.ino 의 targetThrottle == 0).
    if abs(gap) <= distance_deadband_m:
        return "S" if not turn_needed else f"C,0,{yaw_rate:.1f}"

    # 너무 가까우면 후진하지 않고 선다. 메인 로봇이 물러나 오는 상황에서
    # 후진은 뒤를 보지 못한 채 움직이는 것이라 위험하다.
    if gap < 0.0:
        return "S" if not turn_needed else f"C,0,{yaw_rate:.1f}"

    throttle = int(round(throttle_gain_per_m * gap * scale))
    throttle = max(int(round(min_throttle * scale)),
                   min(int(round(max_throttle * scale)), throttle))
    if throttle <= 0:
        return "S"
    return f"C,{throttle},{yaw_rate:.1f}"
