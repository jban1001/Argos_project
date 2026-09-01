"""Pure fire-response mission state machine and point controller."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import math

from .protocol import FireDispatch


class MissionState(Enum):
    IDLE = "IDLE"
    WAIT_CLEARANCE = "WAIT_CLEARANCE"
    NAVIGATING = "NAVIGATING"
    SETTLING = "SETTLING"
    SPRAYING = "SPRAYING"
    RETURNING = "RETURNING"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"


@dataclass(frozen=True)
class MissionConfig:
    arrival_radius_m: float = 0.18
    yaw_tolerance_deg: float = 8.0
    pivot_above_deg: float = 20.0
    max_throttle: int = 150
    min_throttle: int = 140
    throttle_gain_per_m: float = 60.0
    max_yaw_rate_dps: float = 12.0
    steering_gain_dps_per_deg: float = 0.5
    pose_max_age_s: float = 0.5
    localization_grace_s: float = 1.0
    telemetry_max_age_s: float = 0.3
    settle_duration_s: float = 1.0
    obstacle_timeout_s: float = 10.0
    mission_timeout_s: float = 120.0
    spray_duration_s: float = 3.0
    pump_feedback_timeout_s: float = 0.6
    pump_enabled: bool = False
    # 임무를 마치면 출발 지점으로 돌아간다.  안 돌아가면 팔로워가 불이
    # 있던 자리에 그대로 서 있고, 메인봇이 그 근처를 순찰하다 부딪친다.
    return_home: bool = True
    home_arrival_radius_m: float = 0.30
    return_timeout_s: float = 120.0


@dataclass(frozen=True)
class MissionInputs:
    now: float
    pose: tuple[float, float, float] | None = None
    pose_age_s: float = float("inf")
    obstacle: bool = False
    mcu_stopped: bool = False
    telemetry_age_s: float = float("inf")
    pump_feedback: bool | None = None
    follow_command: str = "S"


@dataclass(frozen=True)
class MissionOutput:
    state: MissionState
    reason: str
    motor_command: str | None
    pump_command: str
    mission_id: str | None
    distance_m: float | None = None
    yaw_error_deg: float | None = None


def _angle(error: float) -> float:
    return math.atan2(math.sin(error), math.cos(error))


def point_command(pose: tuple[float, float, float], target: FireDispatch,
                  config: MissionConfig) -> tuple[str, float, float, bool]:
    """Return motor command, distance, final/heading error, and arrival flag."""
    x, y, yaw = pose
    dx, dy = target.x - x, target.y - y
    distance = math.hypot(dx, dy)
    if distance <= config.arrival_radius_m:
        final_error = _angle(target.yaw - yaw)
        final_error_deg = math.degrees(final_error)
        if abs(final_error_deg) <= config.yaw_tolerance_deg:
            return "S", distance, final_error_deg, True
        yaw_rate = max(
            -config.max_yaw_rate_dps,
            min(config.max_yaw_rate_dps,
                config.steering_gain_dps_per_deg * final_error_deg),
        )
        return f"C,0,{yaw_rate:.1f}", distance, final_error_deg, False

    heading_error = _angle(math.atan2(dy, dx) - yaw)
    heading_error_deg = math.degrees(heading_error)
    yaw_rate = max(
        -config.max_yaw_rate_dps,
        min(config.max_yaw_rate_dps,
            config.steering_gain_dps_per_deg * heading_error_deg),
    )
    if abs(heading_error_deg) > config.pivot_above_deg:
        return f"C,0,{yaw_rate:.1f}", distance, heading_error_deg, False

    throttle = int(round(config.throttle_gain_per_m * distance))
    throttle = max(config.min_throttle, min(config.max_throttle, throttle))
    return f"C,{throttle},{yaw_rate:.1f}", distance, heading_error_deg, False


@dataclass(frozen=True)
class _Waypoint:
    """point_command 가 요구하는 최소 표적.  복귀 지점을 담는다."""
    x: float
    y: float
    yaw: float


class MissionController:
    def __init__(self, config: MissionConfig | None = None) -> None:
        self.config = config or MissionConfig()
        self.state = MissionState.IDLE
        self.reason = "waiting for fire dispatch"
        self.dispatch: FireDispatch | None = None
        self.started_at: float | None = None
        self.localization_lost_at: float | None = None
        self.obstacle_since: float | None = None
        self.stopped_since: float | None = None
        self.spray_started_at: float | None = None
        # 지령을 받은 자리.  임무를 마치면 여기로 돌아온다.
        self.home: tuple[float, float, float] | None = None
        self.return_started_at: float | None = None

    def accept_dispatch(self, dispatch: FireDispatch, now: float) -> tuple[bool, str]:
        if self.dispatch is not None and dispatch.mission_id == self.dispatch.mission_id:
            same_target = (
                abs(dispatch.x - self.dispatch.x) < 1e-6
                and abs(dispatch.y - self.dispatch.y) < 1e-6
                and abs(_angle(dispatch.yaw - self.dispatch.yaw)) < 1e-6
            )
            if not same_target:
                return False, "same mission_id cannot change target"
            if self.state in (MissionState.COMPLETE, MissionState.FAILED):
                return False, f"mission is already {self.state.value}"
            if dispatch.main_cleared and not self.dispatch.main_cleared:
                self.dispatch = dispatch
                self.reason = "main robot clearance received"
            return True, "duplicate dispatch accepted idempotently"

        if self.state not in (MissionState.IDLE, MissionState.COMPLETE,
                              MissionState.FAILED):
            return False, f"busy with {self.dispatch.mission_id if self.dispatch else '?'}"

        self.dispatch = dispatch
        self.started_at = now
        self.localization_lost_at = None
        self.obstacle_since = None
        self.stopped_since = None
        self.spray_started_at = None
        self.home = None
        self.return_started_at = None
        if dispatch.main_cleared:
            self._set(MissionState.NAVIGATING, "dispatch accepted; main robot clear")
        else:
            self._set(MissionState.WAIT_CLEARANCE,
                      "target accepted; waiting for main robot clearance")
        return True, self.reason

    def cancel(self, mission_id: str) -> tuple[bool, str]:
        if self.dispatch is None or mission_id != self.dispatch.mission_id:
            return False, "cancel mission_id does not match active mission"
        if self.state in (MissionState.IDLE, MissionState.COMPLETE, MissionState.FAILED):
            return False, f"mission is already {self.state.value}"
        self._set(MissionState.FAILED, "mission canceled")
        return True, self.reason

    def reset(self) -> tuple[bool, str]:
        if self.state not in (MissionState.COMPLETE, MissionState.FAILED):
            return False, "only terminal missions can be reset"
        self.dispatch = None
        self.started_at = None
        self._set(MissionState.IDLE, "mission reset; follow mode restored")
        return True, self.reason

    def _set(self, state: MissionState, reason: str) -> None:
        self.state = state
        self.reason = reason

    def _fail(self, reason: str) -> MissionOutput:
        self._set(MissionState.FAILED, reason)
        return self._output("S", "P,0")

    def _output(self, motor: str | None, pump: str,
                distance: float | None = None,
                yaw_error: float | None = None) -> MissionOutput:
        return MissionOutput(
            state=self.state,
            reason=self.reason,
            motor_command=motor,
            pump_command=pump,
            mission_id=None if self.dispatch is None else self.dispatch.mission_id,
            distance_m=distance,
            yaw_error_deg=yaw_error,
        )

    def update(self, inputs: MissionInputs) -> MissionOutput:
        if self.state is MissionState.IDLE:
            return self._output(inputs.follow_command, "P,0")
        if self.state in (MissionState.COMPLETE, MissionState.FAILED):
            return self._output("S", "P,0")
        if self.dispatch is None or self.started_at is None:
            return self._fail("internal error: active state without dispatch")
        # 출발 지점을 처음 아는 순간에 기억해 둔다.  자세를 아직 못 잡았으면
        # 넘어가고 다음 tick 에서 다시 시도한다.
        if self.home is None and inputs.pose is not None:
            self.home = inputs.pose

        if self.state is MissionState.RETURNING:
            # 복귀는 started_at 이 아니라 복귀 시작 시각으로 잰다.  안 그러면
            # 왕복이 한 제한시간을 나눠 쓰게 되어 돌아오다 죽는다.
            if (self.return_started_at is not None
                    and inputs.now - self.return_started_at
                    > self.config.return_timeout_s):
                return self._fail("return timeout")
        elif inputs.now - self.started_at > self.config.mission_timeout_s:
            return self._fail("mission timeout")

        if self.state is MissionState.WAIT_CLEARANCE:
            if self.dispatch.main_cleared:
                self._set(MissionState.NAVIGATING, "main robot clearance received")
            else:
                return self._output("S", "P,0")

        pose_ok = (inputs.pose is not None
                   and inputs.pose_age_s <= self.config.pose_max_age_s)
        if self.state is MissionState.NAVIGATING:
            if not pose_ok:
                if self.localization_lost_at is None:
                    self.localization_lost_at = inputs.now
                if inputs.now - self.localization_lost_at > self.config.localization_grace_s:
                    return self._fail("localization unavailable")
                self.reason = "waiting for fresh map pose"
                return self._output("S", "P,0")
            self.localization_lost_at = None

            if inputs.obstacle:
                if self.obstacle_since is None:
                    self.obstacle_since = inputs.now
                if inputs.now - self.obstacle_since > self.config.obstacle_timeout_s:
                    return self._fail("obstacle persisted")
                self.reason = "LiDAR obstacle stop"
                return self._output("S", "P,0")
            self.obstacle_since = None

            command, distance, yaw_error, arrived = point_command(
                inputs.pose, self.dispatch, self.config)
            if not arrived:
                self.reason = "moving to fire target"
                return self._output(command, "P,0", distance, yaw_error)
            self.stopped_since = None
            self._set(MissionState.SETTLING, "target reached; confirming stop")

        if self.state is MissionState.SETTLING:
            # NAVIGATING 과 같은 유예를 준다.  막 멈춘 직후에는 AMCL 갱신이
            # 뜸해져 자세가 잠깐 낡는데, 그걸로 임무를 죽이면 도착해 놓고
            # 실패한다.
            if not pose_ok:
                if self.localization_lost_at is None:
                    self.localization_lost_at = inputs.now
                if (inputs.now - self.localization_lost_at
                        > self.config.localization_grace_s):
                    return self._fail(
                        "localization lost while confirming stop")
                self.reason = "waiting for fresh map pose"
                return self._output("S", "P,0")
            self.localization_lost_at = None
            _, distance, yaw_error, arrived = point_command(
                inputs.pose, self.dispatch, self.config)
            if not arrived:
                self.stopped_since = None
                self._set(MissionState.NAVIGATING, "pose moved outside arrival tolerance")
                return self._output("S", "P,0", distance, yaw_error)
            telemetry_ok = inputs.telemetry_age_s <= self.config.telemetry_max_age_s
            if not telemetry_ok or not inputs.mcu_stopped:
                self.stopped_since = None
                self.reason = "waiting for fresh MCU L:0,R:0 telemetry"
                return self._output("S", "P,0", distance, yaw_error)
            if self.stopped_since is None:
                self.stopped_since = inputs.now
            if inputs.now - self.stopped_since < self.config.settle_duration_s:
                self.reason = "MCU stopped; settling"
                return self._output("S", "P,0", distance, yaw_error)
            if not self.config.pump_enabled:
                return self._finish("dry-run complete; pump disabled",
                                    inputs.now, distance, yaw_error)
            self.spray_started_at = inputs.now
            self._set(MissionState.SPRAYING, "pump deadman started")

        if self.state is MissionState.SPRAYING:
            if not pose_ok:
                return self._fail("localization lost while spraying")
            _, distance, yaw_error, arrived = point_command(
                inputs.pose, self.dispatch, self.config)
            telemetry_ok = inputs.telemetry_age_s <= self.config.telemetry_max_age_s
            if not arrived or not telemetry_ok or not inputs.mcu_stopped:
                return self._fail("spray interlock lost (pose or motor stop)")
            if self.spray_started_at is None:
                return self._fail("internal error: spraying without start time")
            elapsed = inputs.now - self.spray_started_at
            if elapsed >= self.config.spray_duration_s:
                return self._finish("spray duration complete",
                                    inputs.now, distance, yaw_error)
            if (inputs.pump_feedback is not True
                    and elapsed > self.config.pump_feedback_timeout_s):
                return self._fail("MCU did not confirm pump on")
            return self._output(None, "P,1", distance, yaw_error)

        if self.state is MissionState.RETURNING:
            if self.home is None:
                self._set(MissionState.COMPLETE, "no home pose recorded")
                return self._output("S", "P,0")
            if not pose_ok:
                if self.localization_lost_at is None:
                    self.localization_lost_at = inputs.now
                if (inputs.now - self.localization_lost_at
                        > self.config.localization_grace_s):
                    return self._fail("localization lost while returning")
                self.reason = "waiting for fresh map pose"
                return self._output("S", "P,0")
            self.localization_lost_at = None

            # 복귀 중에는 장애물로 임무를 죽이지 않는다.  이미 살수를 마쳤고,
            # 여기서 실패시켜 봐야 불이 있던 자리에 서 있게 될 뿐이다.
            home = _Waypoint(*self.home)
            config = replace(self.config,
                             arrival_radius_m=self.config.home_arrival_radius_m)
            command, distance, yaw_error, arrived = point_command(
                inputs.pose, home, config)
            if not arrived:
                self.reason = "returning to start"
                return self._output(command, "P,0", distance, yaw_error)
            self._set(MissionState.COMPLETE, "returned to start")
            return self._output("S", "P,0", distance, yaw_error)

        return self._fail(f"unhandled state {self.state.value}")

    def _finish(self, reason: str, now: float, distance: float,
                yaw_error: float) -> MissionOutput:
        """불 앞에서 할 일이 끝났다.  돌아갈 수 있으면 돌아간다."""
        if not self.config.return_home or self.home is None:
            self._set(MissionState.COMPLETE, reason)
            return self._output("S", "P,0", distance, yaw_error)
        self.return_started_at = now
        self.localization_lost_at = None
        self.obstacle_since = None
        self.stopped_since = None
        self._set(MissionState.RETURNING, f"{reason}; returning to start")
        return self._output("S", "P,0", distance, yaw_error)
