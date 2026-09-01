from dataclasses import replace

from follower_fire_control.mission import (
    MissionConfig,
    MissionController,
    MissionInputs,
    MissionState,
    point_command,
)
from follower_fire_control.protocol import FireDispatch


TARGET = FireDispatch("fire-1", 1.0, 0.0, 0.0, True)
AT_TARGET = (1.0, 0.0, 0.0)


def inputs(now, **updates):
    base = MissionInputs(
        now=now,
        pose=AT_TARGET,
        pose_age_s=0.0,
        mcu_stopped=True,
        telemetry_age_s=0.0,
        pump_feedback=False,
        follow_command="C,140,0.0",
    )
    return replace(base, **updates)


def test_idle_only_forwards_fresh_follow_decision_given_by_node():
    controller = MissionController()
    output = controller.update(inputs(0.0))
    assert output.state is MissionState.IDLE
    assert output.motor_command == "C,140,0.0"
    assert output.pump_command == "P,0"


def test_dispatch_waits_for_explicit_main_clearance():
    controller = MissionController()
    waiting = replace(TARGET, main_cleared=False)
    assert controller.accept_dispatch(waiting, 1.0)[0]
    assert controller.state is MissionState.WAIT_CLEARANCE
    assert controller.update(inputs(1.1)).motor_command == "S"
    assert controller.accept_dispatch(TARGET, 1.2)[0]
    assert controller.state is MissionState.WAIT_CLEARANCE
    assert controller.update(inputs(1.3)).state is MissionState.SETTLING


def test_same_id_cannot_move_target_and_other_mission_cannot_preempt():
    controller = MissionController()
    controller.accept_dispatch(TARGET, 0.0)
    moved = replace(TARGET, x=2.0)
    assert controller.accept_dispatch(moved, 0.1)[0] is False
    other = replace(TARGET, mission_id="fire-2")
    assert controller.accept_dispatch(other, 0.1)[0] is False


def test_point_controller_pivots_then_drives_then_arrives():
    config = MissionConfig()
    command, distance, _, arrived = point_command((0.0, 0.0, 1.57), TARGET, config)
    assert command.startswith("C,0,")
    assert distance == 1.0 and not arrived
    command, _, _, arrived = point_command((0.0, 0.0, 0.0), TARGET, config)
    assert command.startswith("C,140,") and not arrived
    command, _, _, arrived = point_command(AT_TARGET, TARGET, config)
    assert command == "S" and arrived


def test_localization_and_obstacle_fail_closed():
    config = MissionConfig(localization_grace_s=0.5, obstacle_timeout_s=0.5)
    controller = MissionController(config)
    controller.accept_dispatch(TARGET, 0.0)
    assert controller.update(inputs(0.1, pose=None)).motor_command == "S"
    assert controller.update(inputs(0.7, pose=None)).state is MissionState.FAILED

    controller = MissionController(config)
    controller.accept_dispatch(TARGET, 0.0)
    assert controller.update(inputs(0.1, pose=(0.0, 0.0, 0.0), obstacle=True)).motor_command == "S"
    assert controller.update(inputs(0.7, pose=(0.0, 0.0, 0.0), obstacle=True)).state is MissionState.FAILED


def test_dry_run_completes_without_ever_requesting_pump_on():
    controller = MissionController(MissionConfig(
        settle_duration_s=1.0, pump_enabled=False, return_home=False))
    controller.accept_dispatch(TARGET, 0.0)
    first = controller.update(inputs(0.1))
    assert first.state is MissionState.SETTLING and first.pump_command == "P,0"
    done = controller.update(inputs(1.2))
    assert done.state is MissionState.COMPLETE
    assert done.pump_command == "P,0"


def test_pump_requires_settle_feedback_and_has_fixed_duration():
    controller = MissionController(MissionConfig(
        settle_duration_s=1.0,
        spray_duration_s=3.0,
        pump_feedback_timeout_s=0.6,
        pump_enabled=True,
        return_home=False))
    controller.accept_dispatch(TARGET, 0.0)
    controller.update(inputs(0.1))
    spraying = controller.update(inputs(1.2))
    assert spraying.state is MissionState.SPRAYING
    assert spraying.motor_command is None
    assert spraying.pump_command == "P,1"
    assert controller.update(inputs(1.4, pump_feedback=True)).state is MissionState.SPRAYING
    done = controller.update(inputs(4.3, pump_feedback=True))
    assert done.state is MissionState.COMPLETE
    assert done.pump_command == "P,0"


def test_missing_pump_feedback_aborts_and_turns_off():
    controller = MissionController(MissionConfig(
        settle_duration_s=0.1,
        pump_feedback_timeout_s=0.2,
        pump_enabled=True))
    controller.accept_dispatch(TARGET, 0.0)
    controller.update(inputs(0.1))
    controller.update(inputs(0.3))
    failed = controller.update(inputs(0.6, pump_feedback=False))
    assert failed.state is MissionState.FAILED
    assert failed.motor_command == "S"
    assert failed.pump_command == "P,0"
