#!/usr/bin/env python3

"""
fire_seeker 상태머신 회귀 테스트 (하드웨어 불필요)

실행:
    source /opt/ros/jazzy/setup.bash
    ~/.venv/bin/python ~/argos_project/scripts/test_fire_seeker.py

가짜 감지기 / 가짜 LiDAR 로 상황을 재현해 /cmd_vel 출력을 검증한다.
상수(PATROL_V, FIRE_STOP_DIST 등)를 고친 뒤 돌려보면
안전 동작이 깨지지 않았는지 확인할 수 있다.
"""

import io
import math
import time
import contextlib
import importlib.util
from pathlib import Path

import numpy as np

spec = importlib.util.spec_from_file_location(
    "fs", str(Path(__file__).resolve().parent / "fire_seeker.py")
)
fs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fs)

# 하드웨어 없이 돌리기 위한 설정
fs.REQUIRE_SENSOR_GATE = False
fs.SHOW_WINDOW = False
fs.MAX_RUN_TIME = 11.0
fs.CONFIRM_SECONDS = 0.3
fs.FIRE_STOP_PAUSE = 0.3
fs.LOST_SECONDS = 0.5
# 이 시나리오는 직진/회전(bounce) 상태머신의 회귀 테스트다.
# 기본 순찰 모드가 wall 로 바뀌어도 테스트 목적은 유지한다.
fs.PATROL_MODE = "bounce"


# 실제 spark 탐지값은 보존하면서 MLP 입력에만 가중치를 적용해야 한다.
sample_confs = {"spark": 0.20}
original_spark_weight = fs.MLP_SPARK_WEIGHT
fs.MLP_SPARK_WEIGHT = 0.0
assert fs.mlp_spark_feature(sample_confs) == 0.0
assert sample_confs["spark"] == 0.20
fs.MLP_SPARK_WEIGHT = 0.10
assert abs(fs.mlp_spark_feature(sample_confs) - 0.02) < 1e-9
fs.MLP_SPARK_WEIGHT = 1.0
assert fs.mlp_spark_feature(sample_confs) == 0.20
fs.MLP_SPARK_WEIGHT = original_spark_weight


# (bearing_deg, is_fire, front, left, right, scan_fresh)
def scenario(t):
    if t < 1.2:
        return None, False, 3.0, 3.0, 3.0, True    # 순찰 직진
    if t < 3.0:
        return None, False, 0.4, 2.0, 0.5, True    # 막힘 -> 왼쪽이 트임
    if t < 4.2:
        return None, False, 3.0, 3.0, 3.0, True    # 다시 직진
    if t < 5.4:
        return 25.0, True, 3.0, 3.0, 3.0, True     # 불 발견 -> 정지 -> 몸 돌림
    if t < 7.0:
        return 2.0, True, 3.0, 3.0, 3.0, True      # 정면 맞음 -> 접근
    if t < 8.2:
        return 2.0, True, 0.4, 3.0, 3.0, True      # 충분히 접근 -> 정지
    if t < 9.2:
        return 2.0, True, 3.0, 3.0, 3.0, False     # 스캔 끊김 -> 정지
    return None, False, 3.0, 3.0, 3.0, True        # 불 사라짐 -> 순찰 복귀


t0 = time.monotonic()


class FakeDetector:
    def __init__(self):
        self.frame = np.zeros((8, 8, 3), np.uint8)

    def step(self):
        b, f, *_ = scenario(time.monotonic() - t0)
        return {
            "ok": True,
            "frame": self.frame,
            "confs": {c: 0.0 for c in fs.TARGET_CLASSES},
            "bearing": None if b is None else math.radians(b),
            "bearing_box": None,
            "prob": 0.0,
            "sensor_ok": False,
            "gas": 0,
            "temp": 0.0,
            "is_fire": f,
        }

    def release(self):
        pass


fs.FireDetector = FakeDetector

CMDS = []


def fake_clearance(self, heading=0.0, half_angle=None, stat="min"):
    del half_angle, stat
    _, _, front, left, right, _ = scenario(time.monotonic() - t0)
    if abs(fs.wrap(heading - fs.SIDE_HEADING)) < 0.1:
        return left
    if abs(fs.wrap(heading + fs.SIDE_HEADING)) < 0.1:
        return right
    return front


fs.FireSeeker.publish = lambda self, v, w: CMDS.append(
    (round(time.monotonic() - t0, 2), round(v, 3), round(w, 3))
)
fs.FireSeeker.stop = lambda self: None
fs.FireSeeker.spin_once = lambda self, t=0.005: time.sleep(0.02)
fs.FireSeeker.wait_data = lambda self, timeout=15.0: True
fs.FireSeeker.clearance = fake_clearance
fs.FireSeeker.scan_fresh = lambda self: scenario(time.monotonic() - t0)[5]

buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    fs.main()
log = buf.getvalue()


def W(lo, hi):
    return [c for c in CMDS if lo <= c[0] < hi]


def describe(cs):
    if not cs:
        return "없음"
    vs = [c[1] for c in cs]
    ws = [c[2] for c in cs]
    return (f"v[{min(vs):+.2f}..{max(vs):+.2f}] "
            f"w[{min(ws):+.2f}..{max(ws):+.2f}] n={len(cs)}")


WINDOWS = [
    (0.1, 1.1, "순찰 직진"),
    (1.4, 2.9, "막힘->회전"),
    (3.3, 4.1, "순찰 재개"),
    (4.55, 4.75, "불발견 정지"),
    (4.95, 5.35, "몸 돌리기"),
    (5.7, 6.9, "접근"),
    (7.3, 8.1, "도착 정지"),
    (8.5, 9.1, "스캔끊김"),
    (9.9, 10.9, "순찰 복귀"),
]

print("=== 구간별 /cmd_vel ===")
for lo, hi, name in WINDOWS:
    print(f"  {name:12s} {describe(W(lo, hi))}")

print("\n=== 검증 ===")

c = W(0.1, 1.1)
assert c and all(v > 0 and w == 0 for _, v, w in c), "순찰은 직진해야"
print("  OK 순찰: 직진 (v>0, w=0)")

c = W(1.4, 2.9)
assert c and all(v == 0 for _, v, _ in c), "회전 중 전진 금지"
assert all(w > 0 for _, _, w in c), "왼쪽이 트였으면 CCW(+w)"
print("  OK 막힘: 제자리 회전, 트인 쪽(왼쪽=CCW)으로")

c = W(3.3, 4.1)
assert c and all(v > 0 and w == 0 for _, v, w in c), "앞이 트이면 순찰 재개"
print("  OK 재개: 다시 직진")

c = W(4.55, 4.75)
assert c and all(v == 0 and w == 0 for _, v, w in c), "불 발견 시 완전 정지"
print("  OK 불발견: 완전 정지 (FIRE_STOP)")

c = W(4.95, 5.35)
assert c and all(v == 0 for _, v, _ in c), "몸 돌릴 땐 전진 금지"
assert all(w > 0 for _, _, w in c), "불이 왼쪽(+25도)이면 CCW"
print("  OK 정렬: 제자리에서 몸만 불 쪽으로")

c = W(5.7, 6.9)
assert c and all(v > 0 for _, v, _ in c), "정면 맞으면 접근"
print("  OK 접근: 전진")

c = W(7.3, 8.1)
assert c and all(v == 0 and w == 0 for _, v, w in c), "가까워지면 정지"
print("  OK 도착: 완전 정지 (HOLD)")

c = W(8.5, 9.1)
assert c and all(v == 0 and w == 0 for _, v, w in c), "스캔 끊기면 정지"
assert "[경고]" in log
print("  OK 스캔끊김: 완전 정지 + 경고")

c = W(9.9, 10.9)
assert c and all(v > 0 for _, v, _ in c), "불 사라지면 순찰 복귀"
print("  OK 복귀: 순찰 재개")

order = [x.split("]")[0][1:] for x in log.splitlines() if x.startswith("[")]
for need in ["FIRE_STOP", "ALIGN", "APPROACH", "HOLD"]:
    assert need in order, f"{need} 상태를 거치지 않았다: {order}"
assert order.index("FIRE_STOP") < order.index("ALIGN") < order.index("APPROACH")
print("  OK 순서: FIRE_STOP -> ALIGN -> APPROACH -> HOLD")

print("\n상태 로그:")
for x in log.splitlines():
    if x.startswith("["):
        print("   ", x)
