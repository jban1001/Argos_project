"""화재 미션의 주행을 Nav2 에 맡기는 어댑터.

왜 필요한가
-----------
mission.py 의 점 제어기는 목표를 향해 직선으로 간다.  장애물이 없는 곳에서는
충분하지만, 실제 화재 대응은 물건이 널린 방을 지나가야 한다.  Nav2 는 지도와
라이다로 경로를 만들고 장애물을 피한다.

무엇을 바꾸지 않는가
--------------------
상태기계(mission.py)는 그대로 둔다.  그것은 순수 함수이고 시험이 붙어 있다.
도착 판정도 상태기계가 자세로 하므로, Nav2 가 로봇을 옮기면 NAVIGATING ->
SETTLING 전이는 저절로 일어난다.  여기서 하는 일은 셋뿐이다.

    NAVIGATING 에 들어가면   목표를 보낸다
    NAVIGATING 을 벗어나면   목표를 취소한다
    그 사이                  상태를 보고한다

안전
----
목표 전송은 `enable_motion` 이 참일 때만 한다.  이 플래그는 원래 상태기계의
모터 출력을 막는 것인데, Nav2 는 자기 배관으로 따로 명령을 내므로 여기서도
같은 플래그를 걸지 않으면 dry-run 이 dry-run 이 아니게 된다.

액션은 비동기로만 다룬다.  이 어댑터는 20 Hz 타이머 안에서 불리므로
어디서도 블로킹하지 않는다.
"""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node

try:
    from nav2_msgs.action import NavigateToPose
    _HAVE_NAV2 = True
except ImportError:                     # nav2 가 없는 환경에서도 import 는 되게
    NavigateToPose = None
    _HAVE_NAV2 = False


class Nav2Driver:
    """NavigateToPose 액션을 미션 수명주기에 맞춰 여닫는다."""

    def __init__(self, node: Node, action_name: str = "navigate_to_pose") -> None:
        self._node = node
        self._available = _HAVE_NAV2
        self._client = (ActionClient(node, NavigateToPose, action_name)
                        if _HAVE_NAV2 else None)
        self._goal_future = None
        self._result_future = None
        self._handle = None
        self._sent_key: tuple | None = None
        self._last_reason = "idle"
        self._accepted = False
        self._terminal = None           # "succeeded" | "aborted" | "canceled"
        # Async action replies may arrive after a mode change/cancel.  A
        # generation prevents such a stale reply from resurrecting a goal.
        self._generation = 0

    # ------------------------------------------------------------------
    @property
    def available(self) -> bool:
        return self._available

    @property
    def active(self) -> bool:
        return self._sent_key is not None and self._terminal is None

    def server_ready(self) -> bool:
        return bool(self._client and self._client.server_is_ready())

    # ------------------------------------------------------------------
    def ensure_goal(self, mission_id: str, x: float, y: float, yaw: float,
                    frame_id: str = "map") -> None:
        """이 목표가 아직 안 나갔으면 보낸다.  이미 나갔으면 아무 것도 안 한다."""
        if not self._available:
            self._last_reason = "nav2_msgs unavailable"
            return
        key = (mission_id, round(x, 3), round(y, 3), round(yaw, 3), frame_id)
        if key == self._sent_key:
            return
        if not self._client.server_is_ready():
            self._last_reason = "navigate_to_pose server not ready"
            return

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        # stamp 0 = "가장 최근 TF 를 써라".  화재 좌표는 정적이므로 굳이
        # 특정 시각에 묶지 않는다.  묶으면 TF 외삽 경고만 난다.
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = pose

        if self._handle is not None:
            self._handle.cancel_goal_async()
        self._generation += 1
        generation = self._generation
        self._reset_futures()
        self._sent_key = key
        self._terminal = None
        self._accepted = False
        self._last_reason = "goal sent"
        self._goal_future = self._client.send_goal_async(goal)
        self._goal_future.add_done_callback(
            lambda future: self._on_goal_response(future, generation))

    def cancel(self, reason: str = "cancelled") -> None:
        self._generation += 1
        if self._handle is not None:
            self._handle.cancel_goal_async()
        self._reset_futures()
        self._sent_key = None
        self._terminal = None
        self._accepted = False
        self._last_reason = reason

    # ------------------------------------------------------------------
    def _reset_futures(self) -> None:
        self._goal_future = None
        self._result_future = None
        self._handle = None

    def _on_goal_response(self, future, generation: int) -> None:
        try:
            handle = future.result()
        except Exception as exc:                      # noqa: BLE001
            if generation != self._generation:
                return
            self._last_reason = f"goal send failed: {exc}"
            self._sent_key = None
            return
        if generation != self._generation:
            # The user changed mode while Nav2 was accepting this request.
            # Cancel it as soon as the handle exists and ignore its result.
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        if handle is None or not handle.accepted:
            self._last_reason = "goal rejected by Nav2"
            self._sent_key = None
            return
        self._handle = handle
        self._accepted = True
        self._last_reason = "goal accepted"
        self._result_future = handle.get_result_async()
        self._result_future.add_done_callback(
            lambda future: self._on_result(future, generation))

    def _on_result(self, future, generation: int) -> None:
        if generation != self._generation:
            return
        try:
            status = future.result().status
        except Exception as exc:                      # noqa: BLE001
            self._terminal = "aborted"
            self._last_reason = f"result failed: {exc}"
            return
        # action_msgs/GoalStatus: 4 succeeded, 5 canceled, 6 aborted
        self._terminal = {4: "succeeded", 5: "canceled", 6: "aborted"}.get(
            status, f"status_{status}")
        self._last_reason = f"nav2 {self._terminal}"

    # ------------------------------------------------------------------
    def status(self) -> dict:
        return {
            "available": self._available,
            "server_ready": self.server_ready(),
            "goal_sent": self._sent_key is not None,
            "accepted": self._accepted,
            "terminal": self._terminal,
            "reason": self._last_reason,
        }
