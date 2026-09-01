"""camera.yaml must be the only place a locked control's value lives.

These guard a real defect: lock_camera_controls once read only focus and
exposure out of the YAML and hardcoded the rest, so raising `gain` in
camera.yaml was silently undone by the lock step -- which then printed
"[ok] gain = 0", reporting the wrong value as correct.
"""

import textwrap

from follower_bringup.lock_camera_controls import (
    LOCK_SEQUENCE,
    resolve_sequence,
)


def write_yaml(tmp_path, body: str) -> str:
    path = tmp_path / "camera.yaml"
    path.write_text(textwrap.dedent(body), encoding="utf-8")
    return str(path)


def test_yaml_wins_over_hardcoded_fallback(tmp_path):
    params = write_yaml(tmp_path, """
        /follower/camera:
          ros__parameters:
            gain: 60
            exposure_time_absolute: 50
            focus_absolute: 45
    """)
    values = dict(resolve_sequence(params))
    assert values["gain"] == 60
    assert values["exposure_time_absolute"] == 50
    assert values["focus_absolute"] == 45


def test_every_locked_control_is_yaml_overridable(tmp_path):
    """No control may be locked to a value the YAML cannot reach."""
    lines = "\n".join(f"            {name}: {value + 1}"
                      for name, value in LOCK_SEQUENCE)
    params = write_yaml(tmp_path, f"""
        /follower/camera:
          ros__parameters:
{lines}
    """)
    values = dict(resolve_sequence(params))
    for name, fallback in LOCK_SEQUENCE:
        assert values[name] == fallback + 1, f"{name} ignored the YAML"


def test_bool_controls_become_integers(tmp_path):
    """v4l2_camera types these as true/false; the device wants 1/0."""
    params = write_yaml(tmp_path, """
        /follower/camera:
          ros__parameters:
            focus_automatic_continuous: false
            white_balance_automatic: true
    """)
    values = dict(resolve_sequence(params))
    assert values["focus_automatic_continuous"] == 0
    assert values["white_balance_automatic"] == 1


def test_command_line_beats_yaml(tmp_path):
    params = write_yaml(tmp_path, """
        /follower/camera:
          ros__parameters:
            exposure_time_absolute: 50
            focus_absolute: 45
    """)
    values = dict(resolve_sequence(params, focus=10, exposure=333))
    assert values["exposure_time_absolute"] == 333
    assert values["focus_absolute"] == 10


def test_missing_yaml_falls_back_without_crashing(tmp_path):
    values = dict(resolve_sequence(str(tmp_path / "nope.yaml")))
    assert values == dict(LOCK_SEQUENCE)


def test_auto_flags_are_applied_before_manual_values():
    """Order matters: exposure_time_absolute is inactive while auto_exposure=3."""
    order = [name for name, _ in LOCK_SEQUENCE]
    assert order.index("auto_exposure") < order.index("exposure_time_absolute")
    assert order.index("focus_automatic_continuous") < order.index("focus_absolute")
    assert order.index("white_balance_automatic") < order.index("white_balance_temperature")
