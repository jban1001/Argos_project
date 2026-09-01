"""Nothing outside this grammar and these ranges may reach the motor driver.

Run standalone:
    python3 src/follower_serial_bridge/test/test_command_validation.py
"""

from __future__ import annotations

import sys
from pathlib import Path

_PKG = Path(__file__).resolve().parent.parent / "follower_serial_bridge"
sys.path.insert(0, str(_PKG))

from commands import validate_command  # noqa: E402

MAX_PWM = 180
MAX_YAW = 90.0

ACCEPTED = [
    "S", "s", "F", "B", "L", "R", "D",
    "I,0", "I,1",
    "C,0,0", "C,140,20", "C,-150,-20.5", "C,180,90", "C,-180,-90",
    "M,0,0", "M,120,60", "M,-180,180",
]

REJECTED = [
    "",                # empty
    "C,nan,0",         # NaN
    "C,inf,0",
    "C,1e9,0",         # exponent notation
    "C,181,0",         # over MAX_PWM
    "C,-181,0",
    "C,100,91",        # over the yaw clamp
    "M,999,0",         # over MAX_PWM but format-legal: the bug this test exists for
    "M,0,999",
    "rm -rf /",        # injection
    "S; rm -rf /",
    "X,1,2",           # unknown opcode
    "C,100",           # missing field
    "C,100,0,0",       # extra field
    "C , 100 , 0",     # whitespace
    "M,1.5,0",         # non-integer PWM
]


def test_accepted() -> None:
    for command in ACCEPTED:
        assert validate_command(command, MAX_PWM, MAX_YAW) == command, command


def test_rejected() -> None:
    for command in REJECTED:
        assert validate_command(command, MAX_PWM, MAX_YAW) is None, command


if __name__ == "__main__":
    for name, function in sorted(globals().items()):
        if name.startswith("test_"):
            function()
            print(f"PASS  {name}")
    print(f"\n{len(ACCEPTED)} accepted, {len(REJECTED)} rejected -- all as expected")


# --- 극성 반전 -------------------------------------------------------------
# 이 로봇은 모터 배선이 반대다. 뒤집는 것은 전후뿐이고 좌우는 그대로여야
# 한다 -- 좌우까지 뒤집으면 팔로워가 목표 주위를 원을 그리며 돈다.

from commands import invert_drive  # noqa: E402


def test_invert_drive_swaps_forward_and_back():
    assert invert_drive("F") == "B"
    assert invert_drive("B") == "F"


def test_invert_drive_leaves_turns_and_stop_alone():
    for command in ("S", "L", "R", "D", "I,1"):
        assert invert_drive(command) == command


def test_invert_drive_negates_throttle_but_not_yaw():
    assert invert_drive("C,60,15.5") == "C,-60,15.5"
    assert invert_drive("C,-60,-15.5") == "C,60,-15.5"


def test_invert_drive_negates_both_wheels():
    assert invert_drive("M,100,-80") == "M,-100,80"


def test_inverted_command_still_passes_validation():
    for command in ("F", "C,60,15.5", "M,100,-80"):
        flipped = invert_drive(command)
        assert validate_command(flipped, 180, 90.0) == flipped
