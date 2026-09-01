#!/usr/bin/env python3
"""Select a follower mode or send a guarded coordinate fire mission.

This command deliberately cannot enable the motor or pump. Those remain local
launch-time choices on the follower robot.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import uuid


MODES = ("auto", "follow", "coordinate_fire", "standby")


def mode_payload(mode: str, request_id: str) -> str:
    if mode not in MODES:
        raise ValueError(f"unsupported mode: {mode}")
    return json.dumps(
        {"schema": 1, "request_id": request_id, "mode": mode},
        separators=(",", ":"),
    )


def dispatch_payload(mission_id: str, x: float, y: float, yaw_deg: float,
                     main_cleared: bool) -> str:
    values = (x, y, yaw_deg)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("x, y and yaw must be finite")
    return json.dumps({
        "schema": 1,
        "mission_id": mission_id,
        "frame_id": "map",
        "x": x,
        "y": y,
        "yaw": math.radians(yaw_deg),
        "main_cleared": main_cleared,
    }, separators=(",", ":"))


def request_id(prefix: str) -> str:
    return f"{prefix}-{int(time.time())}-{uuid.uuid4().hex[:8]}"


def dispatch_was_accepted(status: object, expected: dict) -> bool:
    """Correlate a status with the exact mission, not just a reused ID."""
    if not isinstance(status, dict):
        return False
    if status.get("mission_id") != expected["mission_id"]:
        return False
    if status.get("state") not in {
            "WAIT_CLEARANCE", "NAVIGATING", "SETTLING", "SPRAYING", "RETURNING"}:
        return False
    target = status.get("target")
    if not isinstance(target, dict):
        return False
    try:
        same_numbers = all(
            abs(float(target[name]) - float(expected[name])) < 1e-6
            for name in ("x", "y", "yaw"))
    except (KeyError, TypeError, ValueError):
        return False
    return (same_numbers
            and target.get("frame_id") == expected["frame_id"]
            and target.get("main_cleared") is expected["main_cleared"])


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description="ARGOS 팔로우봇 모드/좌표 화재 임무 제어")
    sub = result.add_subparsers(dest="command", required=True)
    sub.add_parser("status", help="현재 모드와 로컬 안전 설정 확인")

    set_parser = sub.add_parser("set", help="모드 변경")
    set_parser.add_argument("mode", choices=MODES)

    fire = sub.add_parser(
        "coordinate-fire", help="map 좌표로 이동하는 화재 임무 전송")
    fire.add_argument("--x", required=True, type=float, help="map X [m]")
    fire.add_argument("--y", required=True, type=float, help="map Y [m]")
    fire.add_argument("--yaw-deg", type=float, default=0.0,
                      help="도착 방향 [deg]")
    fire.add_argument("--mission-id", default=None)
    fire.add_argument(
        "--confirm-main-clear", action="store_true",
        help="메인봇/사람이 경로에서 비켰음을 확인하고 이동 허용")

    cancel = sub.add_parser("cancel", help="진행 중 화재 임무 취소")
    cancel.add_argument("--mission-id", required=True)
    sub.add_parser("reset", help="완료/실패 임무를 IDLE로 초기화")
    return result


def _run_ros(args: argparse.Namespace) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from std_msgs.msg import Empty, String

    class ModeClient(Node):
        def __init__(self) -> None:
            super().__init__("argos_mode_client")
            latched = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.mode_status = None
            self.fire_status = None
            self.mode_pub = self.create_publisher(String, "/follower/mode/set", 10)
            self.dispatch_pub = self.create_publisher(String, "/fire/dispatch", 10)
            self.cancel_pub = self.create_publisher(String, "/fire/cancel", 10)
            self.reset_pub = self.create_publisher(Empty, "/fire/reset", 10)
            self.create_subscription(
                String, "/follower/mode/status", self._mode_status, latched)
            self.create_subscription(
                String, "/follower/fire/status", self._fire_status, 10)

        def _mode_status(self, message: String) -> None:
            try:
                self.mode_status = json.loads(message.data)
            except ValueError:
                pass

        def _fire_status(self, message: String) -> None:
            try:
                self.fire_status = json.loads(message.data)
            except ValueError:
                pass

        @staticmethod
        def message(data: str) -> String:
            value = String()
            value.data = data
            return value

        def wait_for(self, predicate, timeout: float, publish=None) -> bool:
            deadline = time.monotonic() + timeout
            next_publish = 0.0
            while time.monotonic() < deadline:
                now = time.monotonic()
                if publish is not None and now >= next_publish:
                    publish()
                    next_publish = now + 0.5
                rclpy.spin_once(self, timeout_sec=0.1)
                if predicate():
                    return True
            return False

        def set_mode(self, mode: str, timeout: float = 8.0) -> bool:
            rid = request_id("mode")
            message = self.message(mode_payload(mode, rid))
            return self.wait_for(
                lambda: bool(self.mode_status)
                and self.mode_status.get("request_id") == rid
                and self.mode_status.get("mode") == mode,
                timeout,
                lambda: self.mode_pub.publish(message),
            )

    rclpy.init()
    node = ModeClient()
    try:
        if args.command == "status":
            if not node.wait_for(lambda: node.mode_status is not None, 5.0):
                print("오류: /follower/mode/status 응답이 없습니다.")
                return 2
            print(json.dumps(node.mode_status, ensure_ascii=False, indent=2))
            return 0

        if args.command == "set":
            if not node.set_mode(args.mode):
                print(f"오류: {args.mode} 모드 확인 응답이 없습니다.")
                return 2
            print(f"모드 전환 완료: {args.mode}")
            return 0

        if args.command == "coordinate-fire":
            if not node.set_mode("coordinate_fire"):
                print("오류: coordinate_fire 모드 전환에 실패했습니다.")
                return 2
            mission_id = args.mission_id or request_id("fire")
            payload = dispatch_payload(
                mission_id, args.x, args.y, args.yaw_deg,
                args.confirm_main_clear)
            expected = json.loads(payload)
            message = node.message(payload)
            accepted = node.wait_for(
                lambda: dispatch_was_accepted(node.fire_status, expected),
                8.0,
                lambda: node.dispatch_pub.publish(message),
            )
            if not accepted:
                print("오류: 화재 임무 수락 상태를 받지 못했습니다.")
                return 3
            state = node.fire_status.get("state")
            if not args.confirm_main_clear:
                print(f"임무 준비 완료: {mission_id} ({state})")
                print("이동하려면 같은 좌표/mission-id로 --confirm-main-clear를 붙여 다시 실행하세요.")
            else:
                print(f"좌표 화재 임무 시작: {mission_id} ({state})")
            if not node.mode_status.get("pump_enabled", False):
                print("주의: 팔로우봇의 로컬 pump_enabled=false라 실제 살수는 하지 않습니다.")
            return 0

        if args.command == "cancel":
            message = node.message(json.dumps(
                {"schema": 1, "mission_id": args.mission_id},
                separators=(",", ":")))
            for _ in range(4):
                node.cancel_pub.publish(message)
                rclpy.spin_once(node, timeout_sec=0.25)
            print(f"취소 요청 전송: {args.mission_id}")
            return 0

        if args.command == "reset":
            for _ in range(4):
                node.reset_pub.publish(Empty())
                rclpy.spin_once(node, timeout_sec=0.25)
            print("임무 초기화 요청 전송")
            return 0
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None) -> int:
    args = parser().parse_args(argv)
    try:
        return _run_ros(args)
    except (ImportError, ValueError) as exc:
        print(f"오류: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
