#!/usr/bin/env python3

"""ARGOS 화재 인지 메인 (fire_perception_main)

인지팀 원본 ~/YOLO/new_main_robot_map.py 를 **수정하지 않고** 이 프로세스 안에서
그대로 실행시키고, 원본이 매 프레임 계산하는 결과를 ROS 2 토픽으로 발행한다.

  카메라 · YOLO · MLP · 아두이노 · 텔레그램  ->  전부 원본이 소유한다 (각 1개)
  이 파일이 하는 일                          ->  원본을 띄우고, 결과를 중계한다

왜 import 가 아니라 exec 인가
-----------------------------
원본에는 `if __name__ == "__main__"` 가드가 없다. 601 줄부터 끝까지가 전부
모듈 최상위 실행문이라서, `import new_main_robot_map` 은 그 자리에서
YOLO 로드 · 카메라 open · 시리얼 open · rclpy.init · while True 를 전부 실행하고
**영원히 반환되지 않는다**. 그래서 import 대신 별도 스레드에서 exec 한다.

exec 의 네임스페이스(ns)가 곧 원본의 모듈 전역이다. 원본 루프가 최상위에 있으므로
루프 안 변수(results, yolo_confs, fire_probability ...)가 전부 ns 에 그대로 보인다.
이게 이 통합의 유일한 hook 이다.

동기화
------
cv2.imshow 는 원본 루프의 마지막 문장이다. 이걸 우리 프로세스 안에서만 감싸서
"이번 프레임 계산 끝" 신호로 쓴다. 콜백이 원본 스레드 안에서 동기로 돌기 때문에
콜백이 도는 동안 원본은 그 자리에 멈춰 있다. 따라서 스냅샷에 서로 다른 프레임이
섞이지 않으며 락이 필요 없다.

텔레그램 전송 상태
------------------
원본은 사진 -> 지도 -> 메시지 순으로 requests.post 를 호출한다.
requests.post 를 감싸서 sendMessage 의 응답을 보면 "정상 전송" 을 알 수 있다.
그 결과를 토픽에 실어 보내면 Nav 이 "알림이 나간 뒤에 출발" 할 수 있다.

  ★ 원본 BOT_TOKEN 이 "--" 인 동안에는 전송이 항상 실패한다.
    Nav 쪽 alert_wait_timeout 이 없으면 로봇이 영원히 출발하지 않는다.

  MLP 성능 (training_result.txt, 2026-08-25 재학습, threshold 0.70)
    Precision 1.000  오경보 없음
    Recall    0.667  위험 상황의 약 1/3 은 알림이 나가지 않는다

실행
----
    source /opt/ros/jazzy/setup.bash
    source ~/argos_project/ros2_ws/install/setup.bash
    ~/.venv/bin/python ~/argos_project/scripts/fire_perception_main.py

    ~/.venv/bin/python 을 써야 ultralytics / sklearn 이 잡힌다.
"""

from __future__ import annotations

import json
import os
import threading
import time
import traceback
from pathlib import Path
from typing import Any, Optional

import cv2
import requests

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool, String


# 원본에서 반드시 있어야 하는 전역. 없으면 조용히 틀리지 않고 즉시 죽는다.
# 텔레그램 전송 직렬화용 락.
#
# 왜 필요한가
#   전송하는 주체가 둘이다.
#     (1) 인지팀 원본의 telegram_alert_worker 스레드
#         - 불을 발견한 순간 스스로 보낸다 (사진 -> 지도 -> 메시지)
#     (2) 우리 _send_fire_alert 스레드
#         - HOLD 지점에서 보낸다 (사진 -> 도면 -> 정확지도 -> 메시지)
#
#   둘이 동시에 send_telegram_photo 를 부르면 요청이 서로 끼어들어
#   수신함에서 사진 순서가 뒤섞인다. 실제로 시험에서 그렇게 나왔다.
#
#   RLock 인 이유: 우리 스레드가 시퀀스 전체를 잡은 채로 그 안에서
#   래핑된 전송 함수를 다시 부르기 때문이다.
TELEGRAM_LOCK = threading.RLock()


class _GatedQueue:
    """원본의 알림 큐 앞에 게이트를 단다. 원본 파일은 건드리지 않는다.

    막으려는 것
      원본은 조건이 유지되는 한 쿨다운마다 계속 알림을 보낸다. 그런데
      불을 발견하고 0.6 m 까지 다가가는 데 최악 60 초가 걸리고, 그 동안
      불은 계속 보이므로 조건도 계속 참이다. 결과적으로 같은 화재 하나로
      알림이 여러 번 간다.

      쿨다운 숫자를 접근 시간보다 길게 잡는 방법도 있지만, 접근 관련
      파라미터를 바꿀 때마다 같이 조정해야 해서 잘 깨진다. 그래서
      "화재를 처리하는 동안" 이라는 상태로 막는다.

    반드시 지키는 것
      한 화재 건에 최소 한 번은 통과시킨다. 원본 알림은 MLP 가 독립적으로
      띄우므로 주행 쪽이 FIRE_STOP 에 들어간 직후에 나올 수도 있다.
      그때 무조건 막으면 그 화재는 통보가 아예 안 간다. 공장이 타는 동안
      아무도 모르는 상황이 제일 나쁘다.
    """

    def __init__(self, real_queue, logger=None, ns=None, conf_gates=None):
        self._real = real_queue
        self._logger = logger
        self._lock = threading.Lock()

        # 원본 모듈 전역. _allow 는 원본 루프 스레드 안에서 동기로 돌기
        # 때문에, 이걸 읽는 시점의 yolo_confs 는 알림을 만든 바로 그
        # 프레임의 값이다.
        self._ns = ns
        self._conf_gates = dict(conf_gates or {})

        self.episode_active = False
        self.sent_in_episode = 0
        self.blocked = 0
        self.blocked_weak = 0

    # --- 주행 노드가 알려주는 화재 처리 구간 ---

    def set_episode(self, active: bool) -> None:
        with self._lock:
            if active == self.episode_active:
                return

            self.episode_active = active

            if active:
                self.sent_in_episode = 0
            elif self._logger is not None and self.blocked:
                self._logger(
                    f"[MAIN] 화재 처리 종료. 접근 중 차단한 중복 알림 "
                    f"{self.blocked}건"
                )
                self.blocked = 0

    # --- 원본 메인 루프가 부르는 부분 ---

    def put_nowait(self, item):
        if self._allow(item):
            return self._real.put_nowait(item)

    def put(self, item, *args, **kwargs):
        if self._allow(item):
            return self._real.put(item, *args, **kwargs)

    def _weak_class_only(self) -> str | None:
        """스파크/담배만 잡혔는데 둘 다 기준 미달이면 그 사유를 돌려준다.

        원본에는 클래스별 알림 기준이 없다. MLP 확률 하나로만 알림을 낸다.
        그런데 스파크와 담배꽁초는 오탐이 잦아서, 약한 신뢰도로 알림이
        나가면 사용자가 알림을 무시하게 된다.

        막는 범위를 좁게 잡는다. 불이나 연기가 조금이라도 잡혔으면 통과,
        YOLO 가 아무것도 못 잡은 가스/온도 단독 알림도 통과시킨다.
        오직 "스파크 또는 담배가 잡혔는데 둘 다 기준 미달" 일 때만 막는다.
        """
        if not self._conf_gates or self._ns is None:
            return None

        confs = self._ns.get("yolo_confs")
        if not isinstance(confs, dict):
            return None            # 확인이 안 되면 막지 않는다

        def conf(name):
            try:
                return float(confs.get(name, 0.0) or 0.0)
            except (TypeError, ValueError):
                return 0.0

        # 게이트 대상이 아닌 클래스가 하나라도 잡혔으면 판단하지 않는다.
        for name in confs:
            if name not in self._conf_gates and conf(name) > 0.0:
                return None

        seen = []
        for name, floor in self._conf_gates.items():
            value = conf(name)
            if value <= 0.0:
                continue
            if value >= floor:
                return None        # 하나라도 기준을 넘으면 통과
            seen.append(f"{name}={value:.3f}<{floor:.2f}")

        if not seen:
            return None            # 아무것도 안 잡힘 = 센서 단독 알림. 통과.
        return ", ".join(seen)

    def _allow(self, item) -> bool:
        if item is None:            # 원본 종료 신호는 항상 통과
            return True

        weak = self._weak_class_only()
        if weak is not None:
            with self._lock:
                self.blocked_weak += 1
                count = self.blocked_weak
            if self._logger is not None:
                self._logger(
                    f"[MAIN] 신뢰도 미달로 알림 차단 ({weak}) -- 누적 {count}건"
                )
            return False

        with self._lock:
            if not self.episode_active:
                return True         # 순찰 중 = 발견 직후. 이게 즉시 알림이다.

            if self.sent_in_episode == 0:
                self.sent_in_episode += 1
                return True         # 한 건당 최소 1회는 보장한다

            self.blocked += 1
            return False            # 접근하는 동안의 중복은 막는다

    # --- 워커가 쓰는 부분은 그대로 넘긴다 ---

    def get(self, *args, **kwargs):
        return self._real.get(*args, **kwargs)

    def task_done(self):
        return self._real.task_done()

    def join(self):
        return self._real.join()


REQUIRED_GLOBALS = ("model", "cap", "results")

# 없어도 동작하는 전역. 참고/로그 전용이라 없으면 null 로 발행한다.
OPTIONAL_GLOBALS = (
    "yolo_confs",
    "best_detections",
    "fire_probability",
    "sensor_connected",
    "gas_raw",
    "ir_temperature",
    "danger_duration",
    "last_alert_time",
    "FIRE_PROB_THRESHOLD",
    "ALERT_COOLDOWN_SECONDS",
)


def _f(value, default=0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def mlp_danger(ns: dict) -> Optional[bool]:
    """원본 MLP 가 지금 이 프레임을 "위험" 으로 보는지.

    원본 L894 와 같은 조건이다.

        sensor_connected and fire_probability >= FIRE_PROB_THRESHOLD

    MLP 입력은 fire / smoke / cigarette_butt / spark 4개 confidence 와
    온도 / 가스 / 변화량이다. 즉 불뿐 아니라 담배꽁초나 연기도 이 하나의
    판정에 함께 반영된다. 임계값은 원본 전역에서 읽으므로 인지팀이 바꾸면
    자동으로 따라간다.

    판단에 필요한 전역이 없으면 None.
    """

    threshold = ns.get("FIRE_PROB_THRESHOLD")
    probability = ns.get("fire_probability")
    sensor_ok = ns.get("sensor_connected")

    if None in (threshold, probability, sensor_ok):
        return None

    if not sensor_ok:
        return False

    return _f(probability) >= _f(threshold)


def alert_expected(ns: dict) -> Optional[bool]:
    """이번 화재가 원본의 텔레그램 대상이 될 수 있는지 미리 판단한다.

    원본 L894~932 의 발송 조건을 그대로 따라간다.

        sensor_connected
        fire_probability >= FIRE_PROB_THRESHOLD
        time.time() - last_alert_time >= ALERT_COOLDOWN_SECONDS

    임계값을 하드코딩하지 않고 원본 전역에서 읽으므로, 인지팀이 0.70 이나
    60 초를 바꾸면 자동으로 따라간다.

    danger_duration(연속 확인 1초)은 일부러 넣지 않는다. 그건 곧 해소되는
    타이밍 문제라서 넣으면 오히려 "알림 대상 아님" 오판이 난다.

    판단에 필요한 전역이 하나라도 없으면 None 을 돌려준다.
    그 경우 Nav 은 예전처럼 무조건 기다렸다가 타임아웃으로 출발한다.
    """

    danger = mlp_danger(ns)
    cooldown = ns.get("ALERT_COOLDOWN_SECONDS")

    if danger is None or cooldown is None:
        return None

    if not danger:
        return False

    last_alert = _f(ns.get("last_alert_time"))

    # 원본은 쿨다운을 time.time() 기준으로 잰다 (L925).
    if last_alert > 0.0 and time.time() - last_alert < _f(cooldown):
        return False

    return True


class TelegramWatcher:
    """원본의 requests.post 호출을 관찰해 텔레그램 전송 상태를 추적한다.

    원본 텔레그램 워커의 전송 순서는 사진 -> 지도 -> 메시지이므로
    sendMessage 응답이 곧 "이번 알림이 끝났다" 를 뜻한다.
    """

    IDLE = "idle"
    QUEUED = "queued"
    SENDING = "sending"
    SENT = "sent"
    FAILED = "failed"

    def __init__(self, logger):
        self._log = logger
        self._lock = threading.Lock()
        self._state = self.IDLE
        self._seq = 0
        self._changed_at = 0.0
        self._photo_ok = 0
        self._last_alert_time = 0.0

    # -- 알림이 큐에 "등록" 되는 시점을 잡는다 ------------------------------
    def note_enqueue(self, last_alert_time: float) -> None:
        """원본 전역 last_alert_time 이 바뀌면 새 알림이 큐에 등록된 것이다."""

        if last_alert_time <= self._last_alert_time:
            return

        self._last_alert_time = last_alert_time

        with self._lock:
            self._seq += 1
            self._state = self.QUEUED
            self._changed_at = time.time()
            self._photo_ok = 0

        self._log(f"[TG] 알림 #{self._seq} 큐 등록")

    # -- requests.post 래퍼 -----------------------------------------------
    def wrap_post(self, original_post):

        def patched_post(url, *args, **kwargs):
            kind = None

            if isinstance(url, str):
                if "/sendPhoto" in url:
                    kind = "photo"
                elif "/sendMessage" in url:
                    kind = "message"

            if kind is not None:
                with self._lock:
                    if self._state in (self.QUEUED, self.SENT, self.FAILED):
                        self._state = self.SENDING
                        self._changed_at = time.time()

            try:
                response = original_post(url, *args, **kwargs)
            except Exception:
                if kind == "message":
                    self._finish(False, "예외")
                raise

            if kind == "photo":
                if getattr(response, "ok", False):
                    with self._lock:
                        self._photo_ok += 1

            elif kind == "message":
                self._finish(
                    bool(getattr(response, "ok", False)),
                    f"HTTP {getattr(response, 'status_code', '?')}",
                )

            return response

        return patched_post

    def _finish(self, ok: bool, detail: str) -> None:
        with self._lock:
            self._state = self.SENT if ok else self.FAILED
            self._changed_at = time.time()
            seq = self._seq
            photos = self._photo_ok

        if ok:
            self._log(f"[TG] 알림 #{seq} 전송 완료 (사진 {photos}장)")
        else:
            self._log(f"[TG] 알림 #{seq} 전송 실패 — {detail}")

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "state": self._state,
                "seq": self._seq,
                "at": round(self._changed_at, 3),
                "photos_ok": self._photo_ok,
            }


def render_fire_map(map_yaml: str, robot_xy, fire_xy, out_path: str) -> bool:
    """Nav2 가 쓰는 지도 위에 로봇과 불 위치를 정확히 찍는다.

    인지팀 원본의 create_robot_map_image 는 건물 도면(nabil_map.png)에
    로봇을 올린다. 그 도면은 SLAM 지도를 따라 그린 것이 아니라서
    실측 결과 벽 일치율이 8% 였다. 사람이 보기에는 예쁘지만 위치는
    2~3 m 어긋난다.

    여기서는 Nav2 가 실제로 불러온 pgm 을 그대로 쓴다. 같은 파일이므로
    좌표 변환 오차가 원리적으로 0 이다. 도면은 도면대로 따로 보내고,
    이 그림으로 정확한 위치를 전한다.
    """

    try:
        import yaml as _yaml

        with open(map_yaml) as f:
            meta = _yaml.safe_load(f)

        pgm = meta["image"]
        if not os.path.isabs(pgm):
            pgm = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), pgm)

        resolution = float(meta["resolution"])
        ox, oy = float(meta["origin"][0]), float(meta["origin"][1])

        grid = cv2.imread(pgm, cv2.IMREAD_GRAYSCALE)
        if grid is None:
            raise FileNotFoundError(f"지도 이미지를 열 수 없다: {pgm}")

        height, width = grid.shape
        canvas = cv2.cvtColor(grid, cv2.COLOR_GRAY2BGR)

        # 작은 지도라 그대로 보내면 잘 안 보인다. 정수배로 키운다.
        scale = max(2, min(8, int(900 / max(width, height))))
        canvas = cv2.resize(
            canvas, (width * scale, height * scale),
            interpolation=cv2.INTER_NEAREST,
        )

        def to_px(x: float, y: float):
            gx = (x - ox) / resolution
            gy = (y - oy) / resolution
            # pgm 은 위아래가 뒤집혀 저장된다
            return int(gx * scale), int((height - 1.0 - gy) * scale)

        drawn = False

        # 마커는 지도 배율과 무관하게 고정 크기로 그린다.
        # 로봇과 불은 0.6 m 밖에 안 떨어져 있어서 배율에 비례시키면
        # 서로 겹쳐 버린다.
        R = 9
        BLUE = (255, 90, 0)
        RED = (0, 0, 255)

        rpx = to_px(*robot_xy) if robot_xy is not None else None
        fpx = to_px(*fire_xy) if fire_xy is not None else None

        def inside(pt):
            return (
                pt is not None
                and 0 <= pt[0] < canvas.shape[1]
                and 0 <= pt[1] < canvas.shape[0]
            )

        if inside(rpx) and inside(fpx):
            cv2.arrowedLine(canvas, rpx, fpx, (0, 165, 255), 3,
                            cv2.LINE_AA, tipLength=0.35)

        if inside(rpx):
            cv2.circle(canvas, rpx, R, BLUE, -1, cv2.LINE_AA)
            cv2.circle(canvas, rpx, R, (255, 255, 255), 2, cv2.LINE_AA)
            # 라벨은 아래쪽으로 빼서 불 라벨과 겹치지 않게 한다
            cv2.putText(canvas, "ROBOT", (rpx[0] - 26, rpx[1] + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 4,
                        cv2.LINE_AA)
            cv2.putText(canvas, "ROBOT", (rpx[0] - 26, rpx[1] + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, BLUE, 2, cv2.LINE_AA)
            drawn = True

        if inside(fpx):
            cv2.line(canvas, (fpx[0] - R, fpx[1] - R),
                     (fpx[0] + R, fpx[1] + R), RED, 4, cv2.LINE_AA)
            cv2.line(canvas, (fpx[0] - R, fpx[1] + R),
                     (fpx[0] + R, fpx[1] - R), RED, 4, cv2.LINE_AA)
            cv2.circle(canvas, fpx, R + 7, RED, 2, cv2.LINE_AA)
            cv2.putText(canvas, "FIRE", (fpx[0] - 20, fpx[1] - 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 4,
                        cv2.LINE_AA)
            cv2.putText(canvas, "FIRE", (fpx[0] - 20, fpx[1] - 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, RED, 2, cv2.LINE_AA)
            drawn = True

        if not drawn:
            return False

        return bool(cv2.imwrite(out_path, canvas))

    except Exception as error:  # noqa: BLE001
        print("[MAP] 정확 위치 지도 생성 실패:", repr(error))
        return False


def stack_map_images(plan_path, slam_path, out_path: str) -> bool:
    """도면과 SLAM 지도를 한 장으로 세로 결합한다.

    왜 합치는가
      원본의 telegram_alert_worker 는 지도 사진 슬롯이 하나뿐이다
      (카메라 -> 지도 -> 메시지). 두 장을 보내려면 원본 전송 흐름을
      고쳐야 하는데, 한 장으로 합치면 원본을 전혀 건드리지 않고
      두 가지 정보를 다 전달할 수 있다.

    위쪽 : 건물 도면. 사람이 "몇 번 구역인지" 를 바로 안다.
           단 SLAM 지도와 픽셀이 대응하지 않아 위치는 2~3 m 오차가 있다.
    아래쪽: Nav2 가 실제로 쓰는 지도. 로봇 위치가 정확하다.
    """

    try:
        images = []

        for path, label in ((plan_path, "PLAN (approx)"),
                            (slam_path, "SLAM MAP (exact)")):
            if path is None:
                continue

            img = cv2.imread(str(path))

            if img is None:
                continue

            images.append((img, label))

        if not images:
            return False

        if len(images) == 1:
            return bool(cv2.imwrite(out_path, images[0][0]))

        width = max(img.shape[1] for img, _ in images)
        width = min(width, 1400)

        panels = []

        for img, label in images:
            scale = width / img.shape[1]
            resized = cv2.resize(
                img, (width, max(1, int(img.shape[0] * scale))),
                interpolation=cv2.INTER_AREA,
            )

            band = 44
            panel = cv2.copyMakeBorder(
                resized, band, 8, 0, 0, cv2.BORDER_CONSTANT,
                value=(255, 255, 255),
            )
            cv2.putText(
                panel, label, (14, band - 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2, cv2.LINE_AA,
            )
            panels.append(panel)

        combined = cv2.vconcat(panels)

        return bool(cv2.imwrite(out_path, combined))

    except Exception as error:  # noqa: BLE001
        print("[MAP] 지도 결합 실패:", repr(error))
        return False


class FirePerceptionPublisher(Node):

    def __init__(self) -> None:
        super().__init__("argos_fire_perception")

        self.declare_parameter(
            "perception_script",
            str(Path.home() / "YOLO" / "new_main_robot_map.py"),
        )
        self.declare_parameter("detection_topic", "/argos/fire_detection")
        self.declare_parameter("show_window", True)
        self.declare_parameter("tick_timeout", 1.0)
        self.declare_parameter("poll_fallback_hz", 30.0)
        self.declare_parameter("log_period", 5.0)
        # 원본 ALERT_COOLDOWN_SECONDS(L104, 60초)를 실행 중에만 바꾼다.
        # 0 이하이면 원본 값을 그대로 쓴다.
        # 원본 L104 ALERT_COOLDOWN_SECONDS(60) 를 실행 중에만 덮어쓴다.
        # 원본 파일은 건드리지 않는다. 0 이하로 두면 원본 값을 그대로 쓴다.
        #
        # 왜 90 초인가
        #   원본은 조건이 유지되는 한 쿨다운마다 계속 보낸다. 그런데
        #   불을 발견하고 접근을 마치고 순찰로 돌아가기까지가 최악 62 초다.
        #     확정 1.0 + 정지 0.7 + 텔레그램대기 10.0
        #     + 정렬 약 10 + 접근 25.0 + HOLD 유지 15.0
        #   그 동안 불은 계속 보이므로 조건도 계속 참이다.
        #   쿨다운이 그보다 짧으면 접근하는 내내 같은 화재로 알림이
        #   반복해서 간다. 20 초면 한 건에 4 회다.
        #
        #   한 건에 한 번만 오게 하려면 쿨다운이 그 시간보다 길어야 한다.
        #   접근 관련 파라미터(approach_max_seconds, hold_max_seconds,
        #   alert_wait_timeout)를 늘리면 이 값도 같이 늘려야 한다.
        self.declare_parameter("telegram_cooldown_seconds", 90.0)

        # 원본이 스스로 보내는 알림(불을 발견한 그 순간)을 막을지 여부.
        #
        # 기본값이 false 인 이유 — 안전망이기 때문이다.
        #   우리 알림은 HOLD(불 0.6 m 앞) 에 도달해야만 나간다. 그런데
        #   장애물 때문에 접근에 실패하거나 approach_max_seconds 를
        #   넘기면 HOLD 에 못 간다. 그 경우 원본 알림까지 막아 두면
        #   화재를 보고도 사람에게 아무 통보가 가지 않는다.
        #
        #   그래서 둘 다 보낸다.
        #     원본 알림 : 발견 즉시. 좌표는 몇 미터 부정확하지만 빠르다.
        #     HOLD 알림 : 접근 후. 좌표가 정확하고 사진이 3장이다.
        #
        #   두 전송이 겹치면 사진 순서가 섞이므로 TELEGRAM_LOCK 이
        #   각 알림을 한 덩어리로 묶어 순서를 지킨다.
        self.declare_parameter("suppress_original_alert", False)

        # HOLD 지점에서 우리가 텔레그램을 보낼지 여부.
        #
        # 기본값이 false 인 이유
        #   알림은 한 번이면 된다. 원본이 불을 발견한 즉시 보내는 것이
        #   안전망으로서 더 중요하다. 접근에 실패해도 그건 이미 나갔다.
        #   여기까지 켜면 같은 화재로 알림이 두 번 가서 번거롭다.
        #
        #   끄더라도 /main/fire_target 토픽은 그대로 발행된다.
        #   팔로워봇은 텔레그램이 아니라 그 토픽으로 좌표를 받는다.
        self.declare_parameter("send_hold_alert", False)
        # 스파크/담배꽁초는 오탐이 잦다. 이 값 미만이면 알림을 막는다.
        # 불·연기는 게이트하지 않는다.
        self.declare_parameter("alert_conf_spark", 0.5)
        # 원본 L97 FIRE_PROB_THRESHOLD.  MLP 확률이 이 값 이상으로
        # ALERT_CONFIRM_SECONDS 동안 유지되면 위험으로 확정하고 알림을 낸다.
        # 0 이하로 두면 원본 값을 그대로 쓴다.
        self.declare_parameter("fire_prob_threshold", 0.80)
        self.declare_parameter("alert_conf_cigarette_butt", 0.5)

        self.script_path = Path(
            self.get_parameter("perception_script").value
        ).expanduser()

        self.tick_timeout = float(self.get_parameter("tick_timeout").value)
        self.poll_hz = float(self.get_parameter("poll_fallback_hz").value)
        self.log_period = float(self.get_parameter("log_period").value)
        self.telegram_cooldown = float(
            self.get_parameter("telegram_cooldown_seconds").value
        )
        self.suppress_original_alert = bool(
            self.get_parameter("suppress_original_alert").value
        )
        self.send_hold_alert = bool(
            self.get_parameter("send_hold_alert").value
        )
        self.fire_prob_threshold = float(
            self.get_parameter("fire_prob_threshold").value
        )
        self.alert_conf_gates = {
            "spark": float(self.get_parameter("alert_conf_spark").value),
            "cigarette_butt": float(
                self.get_parameter("alert_conf_cigarette_butt").value
            ),
        }

        # 정확한 화재 위치 지도를 그릴 때 쓰는 지도. Nav2 가 불러온 것과
        # 같은 파일이어야 좌표가 일치한다.
        self.declare_parameter(
            "map_yaml",
            os.path.expanduser(
                "~/argos_project/maps/argos_outdoor_imu_v1.yaml"
            ),
        )
        self.declare_parameter(
            "alert_request_topic", "/argos/fire_alert_request"
        )

        self.map_yaml = str(self.get_parameter("map_yaml").value)

        # PerceptionRunner 가 원본 네임스페이스를 확보한 뒤 채워 넣는다.
        # 그 전에 요청이 오면 조용히 버린다.
        self.alert_request_handler = None

        topic = str(self.get_parameter("detection_topic").value)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(String, topic, qos)

        self.get_logger().info(f"화재 인지 결과 발행: {topic}")

        alert_topic = str(self.get_parameter("alert_request_topic").value)

        self.create_subscription(
            String, alert_topic, self._on_alert_request, 10
        )

        # 주행 노드의 화재 처리 구간 신호. transient_local 로 발행되므로
        # 인지가 늦게 떠도 현재 상태를 바로 받는다.
        self.fire_episode_handler = None

        self.create_subscription(
            Bool,
            "/argos/fire_episode",
            self._on_fire_episode,
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
            ),
        )

        self.get_logger().info(f"화재 알림 요청 구독: {alert_topic}")

    def _on_fire_episode(self, msg: Bool) -> None:
        handler = self.fire_episode_handler

        if handler is not None:
            handler(bool(msg.data))

    def _on_alert_request(self, msg: String) -> None:
        """주행 노드가 HOLD 지점에서 보낸 알림 요청을 넘긴다."""

        handler = self.alert_request_handler

        if handler is None:
            self.get_logger().warn(
                "알림 요청을 받았으나 원본이 아직 준비되지 않았다"
            )
            return

        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError) as error:
            self.get_logger().error(f"알림 요청 해석 실패: {error}")
            return

        handler(payload)

    # ------------------------------------------------------------------
    def build_payload(self, ns: dict, telegram: dict, seq: int) -> Optional[dict]:
        """원본 네임스페이스에서 한 프레임 스냅샷을 만든다.

        원본 스레드 안에서 동기로 호출되므로 여기서는 락이 필요 없다.
        """

        results = ns.get("results")

        if not results:
            return None

        # 클래스 이름표. Results 에 없으면 원본이 들고 있는 model 에서 가져온다.
        names = getattr(results[0], "names", None)

        if not names:
            model = ns.get("model")
            names = getattr(model, "names", {}) if model is not None else {}

        # 프레임 크기. orig_shape 는 (h, w) 다.
        height = width = 0
        orig_shape = getattr(results[0], "orig_shape", None)

        if orig_shape and len(orig_shape) >= 2:
            height, width = int(orig_shape[0]), int(orig_shape[1])

        if width <= 0:
            frame = ns.get("frame")
            if frame is not None and getattr(frame, "shape", None):
                height, width = int(frame.shape[0]), int(frame.shape[1])

        if width <= 0 or height <= 0:
            return None

        # ---- 박스 추출 ------------------------------------------------
        # 원본 best_detections 는 DISPLAY_CONF(0.10) 로 이미 걸러진 표시용이다.
        # 여기서는 results(conf=0.01 원본)를 직접 읽어 필터링을 하지 않는다.
        # 클래스별 임계값은 Nav 쪽 파라미터가 정한다.
        confs: dict[str, float] = {}
        boxes: dict[str, dict] = {}

        for result in results:
            result_boxes = getattr(result, "boxes", None)

            if result_boxes is None:
                continue

            for box in result_boxes:
                try:
                    class_id = int(box.cls[0].item())
                    confidence = float(box.conf[0].item())
                except (AttributeError, IndexError, TypeError):
                    continue

                if isinstance(names, dict):
                    name = str(names.get(class_id, class_id))
                elif names is not None and 0 <= class_id < len(names):
                    name = str(names[class_id])
                else:
                    name = str(class_id)

                if confidence > confs.get(name, 0.0):
                    confs[name] = confidence

                if confidence <= boxes.get(name, {}).get("conf", -1.0):
                    continue

                try:
                    x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].tolist())
                except (AttributeError, IndexError, TypeError, ValueError):
                    continue

                center_x = 0.5 * (x1 + x2)

                boxes[name] = {
                    "conf": round(confidence, 4),
                    "x1": int(x1),
                    "y1": int(y1),
                    "x2": int(x2),
                    "y2": int(y2),
                    "cx": round(center_x, 1),
                    "cy": round(0.5 * (y1 + y2), 1),
                    # 화면 중앙 기준 -1(왼쪽) ~ +1(오른쪽).
                    # 카메라 HFOV 를 모르는 순수 기하값이므로
                    # 방위각 변환은 Nav 이 자기 파라미터로 한다.
                    "norm_x": round((center_x - 0.5 * width) / (0.5 * width), 4),
                }

        # ---- 참고용 필드 (주행에는 쓰지 않는다) -----------------------
        sensor_connected = ns.get("sensor_connected")

        sensor = None
        if sensor_connected is not None:
            sensor = {
                "ok": bool(sensor_connected),
                "temp": round(_f(ns.get("ir_temperature")), 2),
                "gas": int(_f(ns.get("gas_raw"))),
            }

        mlp_prob = ns.get("fire_probability")

        return {
            "seq": seq,
            "stamp": round(time.time(), 3),
            "frame": {"w": width, "h": height},
            "confs": {k: round(v, 4) for k, v in confs.items()},
            "boxes": boxes,
            "sensor": sensor,
            "mlp_prob": None if mlp_prob is None else round(_f(mlp_prob), 4),
            "danger_duration": round(_f(ns.get("danger_duration")), 2),
            "telegram": telegram,
            # 원본 MLP 의 위험 판정. 불뿐 아니라 담배꽁초/연기도 여기 포함된다.
            "mlp_danger": mlp_danger(ns),
            # true  = 원본이 이번 화재로 텔레그램을 보낼 조건이다
            # false = 보내지 않는다 (MLP 미달 / 센서 끊김 / 60초 쿨다운)
            # null  = 판단 불가. Nav 은 그냥 기다렸다가 타임아웃으로 출발한다.
            "alert_expected": alert_expected(ns),
        }

    def publish(self, payload: dict) -> None:
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.publisher.publish(message)


class PerceptionRunner:
    """원본 스크립트를 이 프로세스 안에서 실행하고 결과를 중계한다."""

    def __init__(self, node: FirePerceptionPublisher) -> None:
        self.node = node
        self.log = node.get_logger().info
        self.warn = node.get_logger().warn
        self.error = node.get_logger().error

        self.ns: dict[str, Any] = {}
        # 알림 전송이 겹치지 않게 한다. 사진 3장 + 메시지라
        # 한 번에 수 초가 걸린다.
        self._alert_busy = threading.Event()
        self.telegram = TelegramWatcher(self.log)

        self.seq = 0
        self.last_tick = 0.0
        self.last_log = 0.0

        self.finished = threading.Event()
        self.failure: Optional[BaseException] = None

        self.show_window = bool(node.get_parameter("show_window").value) and bool(
            os.environ.get("DISPLAY")
        )

        if not self.show_window:
            self.warn("DISPLAY 가 없어 원본 카메라 창을 띄우지 않습니다.")

        self._orig_imshow = cv2.imshow
        self._orig_waitkey = cv2.waitKey
        self._orig_post = requests.post
        self._orig_shutdown = rclpy.shutdown

    # ------------------------------------------------------------------
    def install_patches(self) -> None:
        """우리 프로세스 안에서만 세 가지를 감싼다. 원본 파일은 건드리지 않는다."""

        def patched_imshow(winname, mat, *args, **kwargs):
            self.on_frame_done()

            if self.show_window:
                return self._orig_imshow(winname, mat, *args, **kwargs)

            return None

        def patched_waitkey(delay=0):
            if self.show_window:
                return self._orig_waitkey(delay)

            return -1

        def patched_shutdown(*args, **kwargs):
            # 원본 finally 가 rclpy.shutdown() 을 부른다. 그대로 두면 우리
            # publisher 컨텍스트까지 같이 죽는다. 종료 신호로만 받고,
            # 실제 shutdown 은 이 파일의 main() 이 한다.
            self.warn("[MAIN] 원본이 종료를 요청했습니다.")
            self.finished.set()

        cv2.imshow = patched_imshow
        cv2.waitKey = patched_waitkey
        requests.post = self.telegram.wrap_post(self._orig_post)
        rclpy.shutdown = patched_shutdown

    def remove_patches(self) -> None:
        cv2.imshow = self._orig_imshow
        cv2.waitKey = self._orig_waitkey
        requests.post = self._orig_post
        rclpy.shutdown = self._orig_shutdown

    # ------------------------------------------------------------------
    def start(self) -> None:
        script = self.node.script_path

        if not script.is_file():
            raise FileNotFoundError(f"인지 원본 스크립트 없음: {script}")

        source = script.read_text(encoding="utf-8")
        code = compile(source, str(script), "exec")

        # __file__ 을 넣어야 원본 L39 BASE_DIR = Path(__file__).parent 가
        # 동작한다. 그래야 best.engine / fire_mlp.pkl / nabil_map.png 경로가
        # 전부 원본 폴더로 자동 해결되고, 경로를 밖에서 덮어쓸 필요가 없다.
        #
        # __name__ 은 "__main__" 이 아닌 값으로 둔다. 원본에 가드가 없어서
        # 실행 결과는 같지만, 의미상 라이브러리로 취급한다.
        self.ns.update(
            {
                "__name__": "argos_perception_main",
                "__file__": str(script),
            }
        )

        self.log(f"[MAIN] 원본 실행: {script}")
        self.log("[MAIN] TensorRT 첫 추론 워밍업에 6~7초 걸립니다.")

        thread = threading.Thread(
            target=self._run_original,
            args=(code,),
            name="perception_original",
            daemon=True,
        )
        thread.start()

    def _run_original(self, code) -> None:
        try:
            exec(code, self.ns)  # noqa: S102 — 원본을 수정하지 않기 위한 의도적 사용
        except BaseException as error:  # noqa: BLE001
            self.failure = error
            self.error(f"[MAIN] 원본 실행 중 오류: {error!r}")
            traceback.print_exc()
        finally:
            self.finished.set()

    # ------------------------------------------------------------------
    def verify_globals(self, timeout: float = 90.0) -> None:
        """원본이 필요한 전역을 만들 때까지 기다린다.

        인지팀이 변수 이름을 바꾸면 여기서 **즉시 명확하게** 죽는다.
        조용히 잘못된 값으로 주행하는 것보다 낫다.
        """

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            if self.failure is not None:
                raise RuntimeError("원본이 시작에 실패했습니다.") from self.failure

            missing = [k for k in REQUIRED_GLOBALS if k not in self.ns]

            if not missing:
                absent = [k for k in OPTIONAL_GLOBALS if k not in self.ns]

                if absent:
                    self.warn(
                        "[MAIN] 참고용 전역이 없습니다(주행에는 영향 없음): "
                        + ", ".join(absent)
                    )

                self.log("[MAIN] 원본 전역 확인 완료 — 발행을 시작합니다.")
                return

            time.sleep(0.2)

        raise RuntimeError(
            "원본에서 다음 전역을 찾지 못했습니다: "
            + ", ".join(k for k in REQUIRED_GLOBALS if k not in self.ns)
            + "\n인지팀이 변수 이름을 바꿨는지 확인하십시오. "
            "이 이름들은 fire_perception_main.py 의 REQUIRED_GLOBALS 에 있습니다."
        )

    # ------------------------------------------------------------------
    def install_map_upgrade(self) -> None:
        """원본이 부르는 지도 생성 함수를 감싼다. 원본 파일은 건드리지 않는다.

        원본은 create_robot_map_image 를 전역에서 매번 조회한다. 그래서
        ns 의 그 이름만 바꿔 끼우면 모든 알림(발견 즉시 포함)에 우리가
        만든 지도가 실린다.

        원본 도면만으로는 위치가 2~3 m 틀린다. nabil_map.png 가 SLAM
        지도를 따라 그린 것이 아니라 건물 도면이라서, 벽 일치율이 8% 다.
        그래서 Nav2 가 실제로 쓰는 지도를 아래에 덧붙인다.
        """

        key = "create_robot_map_image"
        original = self.ns.get(key)

        if not callable(original):
            self.error(
                f"[MAIN] 원본에 {key} 가 없어 지도를 개선하지 못했습니다."
            )
            return

        save_dir = self.ns.get("SAVE_DIR")
        map_yaml = self.node.map_yaml

        def wrapper(map_pose_node, timestamp, *args, **kwargs):
            plan_path, robot_xy = None, None

            try:
                plan_path, robot_xy = original(
                    map_pose_node, timestamp, *args, **kwargs
                )
            except Exception as error:  # noqa: BLE001
                self.warn(f"[MAP] 원본 도면 생성 실패: {error!r}")

            if save_dir is None or robot_xy is None:
                return plan_path, robot_xy

            slam_path = str(Path(save_dir) / f"slam_{timestamp}.png")

            # 이 시점에는 불까지의 거리를 모른다 (단안 카메라).
            # 로봇 위치만 정확히 찍는다. 불 위치는 HOLD 에서만 확정된다.
            if not render_fire_map(map_yaml, robot_xy, None, slam_path):
                return plan_path, robot_xy

            merged = str(Path(save_dir) / f"map_merged_{timestamp}.png")

            if not stack_map_images(plan_path, slam_path, merged):
                self._cleanup(slam_path)
                return plan_path, robot_xy

            # 합쳤으므로 재료는 지운다. 원본 워커는 반환한 경로만 지운다.
            self._cleanup(slam_path)
            self._cleanup(plan_path)

            return merged, robot_xy

        self.ns[key] = wrapper

        self.log(
            "[MAIN] 알림 지도 개선: 건물 도면 + 실제 SLAM 지도를 "
            "한 장으로 합쳐 보냅니다."
        )

    @staticmethod
    def _cleanup(path) -> None:
        if path is None:
            return

        try:
            os.remove(str(path))
        except OSError:
            pass

    def install_alert_gate(self) -> None:
        """원본 알림 큐 앞에 게이트를 단다. 원본 파일은 건드리지 않는다."""

        key = "telegram_queue"
        real = self.ns.get(key)

        if real is None:
            self.error(
                f"[MAIN] 원본에 {key} 가 없어 알림 게이트를 못 달았습니다."
            )
            return

        # 원본 워커 스레드는 시작할 때 "진짜 큐 객체" 를 인자로 붙잡았고,
        # 원본 메인 루프는 매번 전역 telegram_queue 를 다시 조회한다.
        # 그래서 전역만 게이트로 바꾸면
        #   메인 루프 -> 게이트 -> (통과 시) 진짜 큐 -> 워커
        # 가 되어 원본 파일을 고치지 않고 중간에 낄 수 있다.
        self._gate = _GatedQueue(
            real, self.warn,
            ns=self.ns,
            conf_gates=self.node.alert_conf_gates,
        )
        self.ns[key] = self._gate

        if self.node.suppress_original_alert:
            self._gate.episode_active = True
            self._gate.sent_in_episode = 1     # 첫 통과분도 미리 소진
            self.warn(
                "[MAIN] 원본 자체 알림을 완전히 차단했습니다 "
                "(suppress_original_alert:=true)"
            )
        else:
            self.log(
                "[MAIN] 알림 게이트 설치: 발견 즉시 1회만 보내고, "
                "접근하는 동안의 중복은 막습니다."
            )

        self.node.fire_episode_handler = self._on_fire_episode

    def _on_fire_episode(self, active: bool) -> None:
        gate = getattr(self, "_gate", None)

        if gate is None:
            return

        if self.node.suppress_original_alert:
            return          # 완전 차단 모드에서는 게이트를 열지 않는다

        gate.set_episode(active)

        self.log(
            "[MAIN] 화재 처리 " + ("시작" if active else "종료")
            + " — 원본 알림 " + ("제한" if active else "허용")
        )

    def serialize_telegram(self) -> None:
        """원본의 전송 함수를 락으로 감싼다. 원본 파일은 건드리지 않는다.

        이렇게 하면 우리 시퀀스가 락을 쥐고 있는 동안 원본 워커의 전송이
        대기하므로, 한 알림의 사진 순서가 중간에 깨지지 않는다.
        """

        for key in ("send_telegram_photo", "send_telegram_message"):
            original = self.ns.get(key)

            if not callable(original):
                self.error(
                    f"[MAIN] 원본에 {key} 가 없어 전송 직렬화를 못 했습니다."
                )
                continue

            def make_wrapper(func):
                def wrapper(*args, **kwargs):
                    with TELEGRAM_LOCK:
                        return func(*args, **kwargs)

                return wrapper

            self.ns[key] = make_wrapper(original)

        self.log("[MAIN] 텔레그램 전송을 직렬화했습니다 (사진 순서 고정).")

    def apply_overrides(self) -> None:
        """원본 파일을 고치지 않고 실행 중 상수 하나만 바꾼다.

        ALERT_COOLDOWN_SECONDS 는 원본 L104 의 모듈 전역이고, 원본 L930 이
        매 루프마다 다시 읽는다. 그래서 여기서 값을 바꾸면 즉시 반영된다.

        이건 예전 fire_nav_patrol.py 의 무분별한 전역 덮어쓰기와 다르다.

          - 바꾸는 값이 이 하나뿐이고 config 에 이름이 드러나 있다
          - 키가 실제로 있는지 먼저 확인한다. 없으면 새로 만들지 않고
            에러를 남긴다 (예전 방식은 오타가 나면 조용히 새 전역을 만들었다)
          - 바꾸기 전후 값을 로그로 남긴다

        원본 파일 자체는 여전히 1바이트도 바뀌지 않는다. 인지팀이 L104 를
        직접 고쳐주면 이 파라미터를 0 으로 두면 된다.
        """

        if self.node.telegram_cooldown <= 0.0:
            self.log("[MAIN] 텔레그램 쿨다운: 원본 값을 그대로 사용합니다.")
            return

        key = "ALERT_COOLDOWN_SECONDS"

        if key not in self.ns:
            self.error(
                f"[MAIN] 원본에 {key} 가 없어 쿨다운을 바꾸지 못했습니다. "
                "인지팀이 이름을 바꿨는지 확인하십시오."
            )
            return

        before = self.ns[key]
        self.ns[key] = self.node.telegram_cooldown

        self.warn(
            f"[MAIN] 텔레그램 쿨다운 변경: {before}s -> "
            f"{self.node.telegram_cooldown}s "
            "(원본 파일은 수정하지 않고 실행 중에만 적용)"
        )

        self.apply_threshold_override()

    def apply_threshold_override(self) -> None:
        """경보 기준(MLP 확률)을 실행 중에만 바꾼다.

        원본 L97 의 모듈 전역이고 L832 / L896 / L965 가 매 루프마다 다시
        읽는다.  텔레그램 본문의 "경보 기준" 표시도 같은 이름을 쓰므로
        여기 한 곳만 바꾸면 판정과 표시가 함께 따라온다.
        """
        if self.node.fire_prob_threshold <= 0.0:
            self.log("[MAIN] 경보 기준: 원본 값을 그대로 사용합니다.")
            return

        key = "FIRE_PROB_THRESHOLD"

        if key not in self.ns:
            self.error(
                f"[MAIN] 원본에 {key} 가 없어 경보 기준을 바꾸지 못했습니다. "
                "인지팀이 이름을 바꿨는지 확인하십시오."
            )
            return

        before = self.ns[key]
        self.ns[key] = self.node.fire_prob_threshold

        self.warn(
            f"[MAIN] 경보 기준 변경: {before:.2f} -> "
            f"{self.node.fire_prob_threshold:.2f} "
            "(원본 파일은 수정하지 않고 실행 중에만 적용)"
        )

    # ------------------------------------------------------------------
    def on_frame_done(self) -> None:
        """원본 루프가 imshow 를 부르는 순간, 원본 스레드 안에서 동기로 실행된다."""

        self.last_tick = time.monotonic()
        self._emit()

    def _emit(self) -> None:
        self.telegram.note_enqueue(_f(self.ns.get("last_alert_time")))

        self.seq += 1

        payload = self.node.build_payload(
            self.ns,
            self.telegram.snapshot(),
            self.seq,
        )

        if payload is None:
            return

        self.node.publish(payload)

        now = time.monotonic()

        if now - self.last_log >= self.node.log_period:
            self.last_log = now

            fire_conf = payload["confs"].get("fire", 0.0)
            box = payload["boxes"].get("fire")
            where = "none" if box is None else f"norm_x={box['norm_x']:+.2f}"
            tg = payload["telegram"]["state"]

            self.log(
                f"[MAIN] seq={payload['seq']} fire={fire_conf:.2f} {where} "
                f"mlp={payload['mlp_prob']} tg={tg}"
            )


    # ------------------------------------------------------------------
    def handle_alert_request(self, payload: dict) -> None:
        """주행 노드가 HOLD 지점에서 보낸 알림 요청을 처리한다.

        인지 루프를 막으면 안 되므로 별도 스레드에서 보낸다.
        원본의 전송 함수를 그대로 재사용한다. 원본은 손대지 않는다.
        """

        if not self.node.send_hold_alert:
            fire = payload.get("fire") or {}
            self.log(
                "[ALERT] HOLD 좌표 수신 "
                f"({fire.get('x')}, {fire.get('y')}) — "
                "텔레그램은 원본 알림으로 갈음합니다 "
                "(send_hold_alert:=true 로 켤 수 있음)"
            )
            return

        if self._alert_busy.is_set():
            self.warn("[ALERT] 이전 알림 전송이 아직 안 끝났다. 건너뛴다.")
            return

        self._alert_busy.set()

        threading.Thread(
            target=self._send_fire_alert,
            args=(payload,),
            name="fire_alert_send",
            daemon=True,
        ).start()

    def _send_fire_alert(self, payload: dict) -> None:
        try:
            ns = self.ns
            node = self.node

            # serialize_telegram() 이 감싼 버전을 쓴다.
            send_photo = ns.get("send_telegram_photo")
            send_text = ns.get("send_telegram_message")

            if send_photo is None or send_text is None:
                self.warn("[ALERT] 원본 텔레그램 함수를 찾지 못했다.")
                return

            save_dir = ns.get("SAVE_DIR")
            stamp = time.strftime("%Y%m%d_%H%M%S")

            fire = payload.get("fire") or {}
            robot = payload.get("robot") or {}

            fire_xy = (
                (float(fire["x"]), float(fire["y"]))
                if "x" in fire and "y" in fire
                else None
            )
            robot_xy = (
                (float(robot["x"]), float(robot["y"]))
                if "x" in robot and "y" in robot
                else None
            )

            made = []

            # 1) 현장 사진. 박스가 그려진 프레임을 우선 쓴다.
            frame = ns.get("annotated_frame")
            if frame is None:
                frame = ns.get("frame")

            if frame is not None and save_dir is not None:
                cam_path = str(Path(save_dir) / f"fire_cam_{stamp}.jpg")
                if cv2.imwrite(cam_path, frame):
                    made.append(("현장 사진", cam_path))

            # 2) 건물 도면. 사람이 어느 구역인지 파악하는 용도다.
            #    벽 일치율이 8% 라 위치는 대략치다. 정확한 건 3) 이다.
            create_map = ns.get("create_robot_map_image")
            pose_node = ns.get("map_pose_node")

            if create_map is not None and pose_node is not None:
                try:
                    plan_path, _ = create_map(pose_node, stamp)
                    if plan_path is not None:
                        made.append(("건물 도면(대략)", str(plan_path)))
                except Exception as error:  # noqa: BLE001
                    self.warn(f"[ALERT] 도면 생성 실패: {error!r}")

            # 3) Nav2 가 쓰는 실제 지도. 좌표가 정확하다.
            if save_dir is not None and (fire_xy or robot_xy):
                exact_path = str(Path(save_dir) / f"fire_map_{stamp}.png")
                if render_fire_map(
                    node.map_yaml, robot_xy, fire_xy, exact_path
                ):
                    made.append(("정확 위치 지도", exact_path))

            text = self._compose_alert_text(payload)

            sent_any = False

            # 시퀀스 전체를 한 번에 잡는다. 이걸 개별 전송마다 잡으면
            # 사진 사이사이로 원본 워커의 전송이 끼어들어 순서가 깨진다.
            #
            # 순서는 고정이다
            #   1) YOLO 현장 사진
            #   2) 건물 도면 (사람이 구역을 파악)
            #   3) 실제 SLAM 지도 (정확한 좌표)
            #   4) 텍스트 메시지
            with TELEGRAM_LOCK:
                for label, path in made:
                    try:
                        if send_photo(path, caption=""):
                            sent_any = True
                            self.log(f"[ALERT] {label} 전송 완료")
                        else:
                            self.warn(f"[ALERT] {label} 전송 실패")
                    except Exception as error:  # noqa: BLE001
                        self.warn(f"[ALERT] {label} 전송 오류: {error!r}")

                try:
                    if send_text(text):
                        sent_any = True
                except Exception as error:  # noqa: BLE001
                    self.warn(f"[ALERT] 메시지 전송 오류: {error!r}")

            self.log(
                "[ALERT] HOLD 지점 알림 "
                + ("전송 완료" if sent_any else "전송 실패")
            )

            for _, path in made:
                try:
                    os.remove(path)
                except OSError:
                    pass

        except Exception as error:  # noqa: BLE001
            self.warn(f"[ALERT] 알림 처리 실패: {error!r}")

        finally:
            self._alert_busy.clear()

    def _compose_alert_text(self, payload: dict) -> str:
        ns = self.ns
        fire = payload.get("fire") or {}
        robot = payload.get("robot") or {}

        lines = ["🔥 화재 확인 (메인봇 접근 완료)"]

        if "x" in fire:
            lines.append(
                f"\n📍 화재 위치 (map)\nX: {fire['x']:.2f} m\n"
                f"Y: {fire['y']:.2f} m"
            )

        if "x" in robot:
            lines.append(
                f"\n🤖 메인봇 위치 (map)\nX: {robot['x']:.2f} m\n"
                f"Y: {robot['y']:.2f} m"
            )

        dist = payload.get("distance_m")
        if dist is not None:
            lines.append(f"\n📏 전방 거리: {dist:.2f} m")

        prob = ns.get("fire_probability")
        if isinstance(prob, (int, float)):
            lines.append(f"\n🧠 위험 확률: {prob * 100:.1f}%")

        temp = ns.get("ir_temperature")
        gas = ns.get("gas_raw")
        if temp is not None and gas is not None:
            lines.append(f"\n🌡 온도: {temp}\n💨 가스: {gas}")

        lines.append(
            "\n\n두 번째 사진은 건물 도면(대략 위치), "
            "세 번째 사진이 실제 지도 기준 정확한 위치입니다."
        )

        return "".join(lines)

    # ------------------------------------------------------------------
    def poll_loop(self) -> None:
        """imshow tick 이 끊겼을 때를 위한 폴백.

        인지팀이 원본에서 imshow 를 지우면 tick 이 사라진다. 그때는
        주기적으로 네임스페이스를 읽어 계속 발행한다. 프레임 경계와
        정확히 맞지는 않지만 주행이 멈추지는 않는다.
        """

        period = 1.0 / max(1.0, self.node.poll_hz)
        warned = False

        while not self.finished.is_set():
            time.sleep(period)

            if self.last_tick == 0.0:
                continue

            if time.monotonic() - self.last_tick <= self.node.tick_timeout:
                continue

            if not warned:
                warned = True
                self.warn(
                    "[MAIN] imshow tick 이 끊겼습니다. 폴링 방식으로 전환합니다."
                )

            self._emit()


def main() -> int:
    rclpy.init()

    node = FirePerceptionPublisher()
    runner = PerceptionRunner(node)

    def _spin() -> None:
        # 종료할 때 rclpy.spin 은 ExternalShutdownException 을 던진다.
        # 데몬 스레드에서 그대로 터지면 Ctrl+C 마다 traceback 이 찍힌다.
        #
        # rclpy.spin(node) 을 쓰면 안 된다. executor 를 안 주면
        # 전역 executor 를 쓰는데, 인지팀 원본도
        #   new_main_robot_map.py:391  threading.Thread(target=rclpy.spin, ...)
        # 로 전역 executor 를 spin 한다. 먼저 잡은 쪽이 이기고 나중 쪽은
        #   RuntimeError: Executor is already spinning
        # 으로 죽는다. 실제로 원본의 fire_alert_map_pose 노드가
        # 콜백을 하나도 처리하지 못하는 상태였다.
        #
        # 전용 executor 를 쓰면 전역 executor 가 원본 몫으로 남는다.
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        try:
            executor.spin()
        except Exception:  # noqa: BLE001
            pass
        finally:
            try:
                executor.remove_node(node)
                executor.shutdown()
            except Exception:  # noqa: BLE001
                pass

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    exit_code = 0

    try:
        runner.install_patches()
        runner.start()
        runner.verify_globals()
        runner.apply_overrides()
        runner.install_map_upgrade()
        runner.install_alert_gate()
        runner.serialize_telegram()

        # 원본 네임스페이스가 확보된 뒤에 연결한다.
        # 그 전에 요청이 오면 노드가 조용히 버린다.
        node.alert_request_handler = runner.handle_alert_request
        node.get_logger().info(
            "HOLD 지점 알림 요청 수신 준비 완료"
        )

        poll_thread = threading.Thread(
            target=runner.poll_loop,
            name="perception_poll",
            daemon=True,
        )
        poll_thread.start()

        # 원본 루프가 끝날 때까지 기다린다.
        while not runner.finished.wait(timeout=0.5):
            pass

        if runner.failure is not None:
            exit_code = 1

    except KeyboardInterrupt:
        node.get_logger().info("[MAIN] 종료합니다.")

    except Exception as error:  # noqa: BLE001
        node.get_logger().error(f"[MAIN] 시작 실패: {error}")
        exit_code = 1

    finally:
        runner.remove_patches()

        # 순서가 중요하다. spin 스레드가 도는 중에 노드를 없애면
        # "publisher's context is invalid" 와 terminate 경고가 뜬다.
        # shutdown -> spin 반환 -> join -> destroy 순으로 내린다.
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass

        spin_thread.join(timeout=3.0)

        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
