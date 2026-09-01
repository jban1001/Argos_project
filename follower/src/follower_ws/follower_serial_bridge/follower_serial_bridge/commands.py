"""Command grammar for the Arduino link.

Kept free of rclpy so it can be tested without a ROS environment: this is the
last gate before bytes reach a motor driver, and a gate that is awkward to test
does not get tested.
"""

from __future__ import annotations

import re

# Only these forms are ever written to the MCU. The grammar is deliberately
# strict rather than permissive: it is what keeps NaN, inf, exponent notation
# and injected shell text out of the motor driver.
_BARE = re.compile(r"^[SsFfBbLlRrDd]$")
_DRIVE = re.compile(r"^[Cc],(-?\d{1,3}),(-?\d{1,3}(?:\.\d{1,3})?)$")
_DIRECT = re.compile(r"^[Mm],(-?\d{1,3}),(-?\d{1,3})$")
_STREAM = re.compile(r"^[Ii],[01]$")


# 예정: 물 발사 명령 (README 10 절)
#
# 발사 명령을 추가할 때 기존 형식을 넓히지 말고 **새 정규식을 하나 더** 둘 것.
# 이 문법이 좁은 것이 목적이고, 넓히면 그 목적이 사라진다.
#
# 발사는 모터와 성격이 다르다. 모터는 명령이 끊기면 멈추면 되지만, 이미 나간
# 물은 되돌릴 수 없다. 그래서:
#
#   - 타임아웃이 모터(350 ms MCU / 0.5 s 브리지)보다 짧아야 한다
#   - 정지가 확인된 뒤에만 통과시켜야 한다 (이동 중 발사 금지)
#   - 상태 기계가 LOST 면 여기까지 오지 않아야 한다
#
# 마지막 두 개는 이 함수 혼자 판단할 수 없다 -- 호출자가 상태를 알고 있어야
# 하므로, 검사를 여기에 욱여넣지 말고 발사 경로를 따로 만들 것.


def validate_command(command: str, max_pwm: int, max_yaw_rate: float) -> str | None:
    """Return the command to send, or None with the reason logged by the caller.

    Matching the shape is not enough. The firmware clamps out-of-range values,
    so passing them on is not dangerous, but it hides controller bugs: a
    controller asking for PWM 999 is broken and should be told so here rather
    than silently getting 180. Range limits mirror MAX_PWM and the yaw clamp
    in followingbot_mega.ino and must be kept in step with them.
    """
    if _BARE.match(command) or _STREAM.match(command):
        return command

    match = _DRIVE.match(command)
    if match:
        throttle = int(match.group(1))
        yaw_rate = float(match.group(2))
        if abs(throttle) > max_pwm:
            return None
        if abs(yaw_rate) > max_yaw_rate:
            return None
        return command

    match = _DIRECT.match(command)
    if match:
        left, right = int(match.group(1)), int(match.group(2))
        if abs(left) > max_pwm or abs(right) > max_pwm:
            return None
        return command

    return None


def invert_drive(command: str) -> str:
    """앞뒤 명령의 부호를 뒤집는다.

    이 로봇은 모터 배선이 반대라 F 를 주면 뒤로 간다 (2026-08-29 실측:
    바퀴를 들고 F 를 주니 역방향으로 돌았다). 펌웨어를 고치지 않고 여기서
    한 번만 뒤집는 이유는, 여기가 MCU 와 맞닿는 마지막 지점이라 어느 경로로
    들어온 명령이든 (추종 제어기, 수동 발행, 시험 스크립트) 같은 규칙을
    받기 때문이다. 제어기 쪽에서 뒤집으면 수동 명령은 여전히 반대로 간다.

    좌우 회전(L, R)과 yaw_rate 는 건드리지 않는다. 뒤집힌 것은 전후뿐이다.
    좌우까지 뒤집혔다면 그것은 배선이 좌우로 바뀐 것이고, 소프트웨어로
    덮을 일이 아니다.

    검증(validate_command) 뒤에 적용할 것. 부호만 바꾸므로 범위는 그대로다.
    """
    if _BARE.match(command):
        flip = {"F": "B", "B": "F", "f": "b", "b": "f"}
        return flip.get(command, command)

    match = _DRIVE.match(command)
    if match:
        throttle = -int(match.group(1))
        return f"{command[0]},{throttle},{match.group(2)}"

    match = _DIRECT.match(command)
    if match:
        left, right = -int(match.group(1)), -int(match.group(2))
        return f"{command[0]},{left},{right}"

    return command
