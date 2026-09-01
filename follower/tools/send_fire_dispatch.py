#!/usr/bin/env python3
"""메인 로봇 역할로 화재 좌표를 보내고 팔로워의 상태 전이를 지켜본다.

규약은 follower_fire_control/protocol.py 가 정하는 그대로다.  여기서 형식을
새로 만들지 않는다 -- 같은 형식을 두 군데 적으면 한쪽만 고쳐져서 갈라진다.

    /fire/dispatch  {"schema":1,"mission_id":...,"frame_id":"map",
                     "x":...,"y":...,"yaw":...,"main_cleared":true|false}
    /fire/cancel    {"schema":1,"mission_id":...}
    /follower/fire/status  (감독기가 내는 상태)

두 단계로 보내는 이유
---------------------
메인 로봇은 불을 발견한 순간에는 아직 그 자리에 있다.  그래서 먼저
main_cleared=false 로 좌표만 알리고(팔로워는 WAIT_CLEARANCE 에서 대기),
비켜난 뒤에 main_cleared=true 를 보낸다.  --two-stage 가 그 순서를 흉내낸다.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String


class Dispatcher(Node):
    def __init__(self):
        super().__init__("send_fire_dispatch")
        self._pose = None
        self._last_state = None
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(PoseWithCovarianceStamped,
                                 "/follower/amcl_pose", self._on_pose, qos)
        self.create_subscription(String, "/follower/fire/status",
                                 self._on_status, 10)
        self._dispatch = self.create_publisher(String, "/fire/dispatch", 10)
        self._cancel = self.create_publisher(String, "/fire/cancel", 10)

    def _on_pose(self, m):
        self._pose = m

    def _on_status(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        key = (d.get("state"), d.get("reason"), json.dumps(d.get("nav2")))
        if key == self._last_state:
            return
        self._last_state = key
        nav2 = d.get("nav2") or {}
        extra = ""
        if nav2:
            extra = (f"  [nav2 서버={nav2.get('server_ready')} "
                     f"전송={nav2.get('goal_sent')} 수락={nav2.get('accepted')} "
                     f"종료={nav2.get('terminal')} / {nav2.get('reason')}]")
        print(f"  상태 {d.get('state'):<15} 거리={d.get('distance_m')} "
              f"이유={d.get('reason')}{extra}", flush=True)

    def send(self, payload: dict, pub):
        m = String(); m.data = json.dumps(payload, separators=(",", ":"))
        pub.publish(m)
        print(f"  -> {m.data}", flush=True)

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_pose(self, timeout=15.0):
        end = time.time() + timeout
        while time.time() < end and self._pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._pose is not None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mission-id", default=None)
    ap.add_argument("--forward", type=float, default=0.25,
                    help="현재 팔로워 자세 기준 정면 몇 m 앞을 화재 지점으로 볼지")
    ap.add_argument("--x", type=float, default=None)
    ap.add_argument("--y", type=float, default=None)
    ap.add_argument("--yaw", type=float, default=None, help="도(deg)")
    ap.add_argument("--two-stage", action="store_true",
                    help="main_cleared=false 를 먼저 보내고 --clear-after 초 뒤 true")
    ap.add_argument("--clear-after", type=float, default=6.0)
    ap.add_argument("--watch", type=float, default=40.0)
    ap.add_argument("--cancel-at-end", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = Dispatcher()
    try:
        if not node.wait_pose():
            print("/follower/amcl_pose 가 오지 않는다. AMCL 을 먼저 띄워라.")
            return 1
        p = node._pose.pose.pose.position
        q = node._pose.pose.pose.orientation
        yaw0 = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if args.x is not None and args.y is not None:
            fx, fy = args.x, args.y
        else:
            fx = p.x + args.forward * math.cos(yaw0)
            fy = p.y + args.forward * math.sin(yaw0)
        fyaw = math.radians(args.yaw) if args.yaw is not None else yaw0
        mid = args.mission_id or f"fire-{int(time.time())}"

        print(f"팔로워 현재  x={p.x:+.3f} y={p.y:+.3f} yaw={math.degrees(yaw0):+.1f}")
        print(f"화재 좌표    x={fx:+.3f} y={fy:+.3f} yaw={math.degrees(fyaw):+.1f}")
        print(f"mission_id   {mid}\n")

        base = {"schema": 1, "mission_id": mid, "frame_id": "map",
                "x": round(fx, 4), "y": round(fy, 4), "yaw": round(fyaw, 4)}

        if args.two_stage:
            print("1단계 -- 메인이 아직 비키지 않았다 (main_cleared=false)")
            node.send({**base, "main_cleared": False}, node._dispatch)
            node.spin_for(args.clear_after)
            print("\n2단계 -- 메인이 비켰다 (main_cleared=true)")
            node.send({**base, "main_cleared": True}, node._dispatch)
        else:
            node.send({**base, "main_cleared": True}, node._dispatch)

        print()
        node.spin_for(args.watch)

        if args.cancel_at_end:
            print("\n취소를 보낸다")
            node.send({"schema": 1, "mission_id": mid}, node._cancel)
            node.spin_for(4.0)
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
