#!/usr/bin/env python3
"""Nav2 에 목표를 보내고 진행을 지켜본다.

현재 AMCL 자세를 기준으로 정면 `--forward` m 앞(또는 --x/--y 로 절대 좌표)을
목표로 준다.  진행 중 자세와 남은 거리를 찍고, 끝나면 실제 이동량을 낸다.

안전
----
  - Ctrl-C 나 예외로 끝나도 목표를 취소하고 정지 명령을 보낸다.
  - --timeout 을 넘기면 취소한다.
  - cmd_vel_bridge 가 0.5 초 워치독을 갖고 있어, Nav2 가 멈추면 차체도 멈춘다.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Sender(Node):
    def __init__(self):
        super().__init__("send_goal")
        self._pose = None
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(PoseWithCovarianceStamped,
                                 "/follower/amcl_pose", self._on_pose, qos)
        self._stop = self.create_publisher(String, "/follower/motor_command", 10)
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def _on_pose(self, m):
        self._pose = m

    def stop_motor(self):
        m = String(); m.data = "S"
        for _ in range(3):
            self._stop.publish(m)
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_pose(self, timeout=15.0):
        end = time.time() + timeout
        while time.time() < end and self._pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._pose is not None

    def xy_yaw(self):
        p = self._pose.pose.pose.position
        return p.x, p.y, yaw_of(self._pose.pose.pose.orientation)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--forward", type=float, default=0.30,
                    help="현재 자세 기준 정면 거리 (m)")
    ap.add_argument("--x", type=float, default=None, help="맵 좌표 x (절대)")
    ap.add_argument("--y", type=float, default=None, help="맵 좌표 y (절대)")
    ap.add_argument("--frame", default="map",
                    help="목표를 낼 프레임. follower_base_link 로 주면 AMCL yaw 를 "
                         "거치지 않고 '로봇 기준 앞'이 그대로 앞이 된다")
    ap.add_argument("--timeout", type=float, default=90.0)
    args = ap.parse_args()

    rclpy.init()
    node = Sender()
    code = 0
    handle = None
    try:
        if not node.wait_pose():
            print("/follower/amcl_pose 가 오지 않는다. AMCL 을 먼저 띄워라.")
            return 1
        x0, y0, yaw0 = node.xy_yaw()
        if args.x is not None and args.y is not None:
            gx, gy = args.x, args.y
        else:
            gx = x0 + args.forward * math.cos(yaw0)
            gy = y0 + args.forward * math.sin(yaw0)
        print(f"현재  x={x0:+.3f} y={y0:+.3f} yaw={math.degrees(yaw0):+.1f} deg")
        print(f"목표  x={gx:+.3f} y={gy:+.3f}   (거리 {math.hypot(gx-x0, gy-y0):.3f} m)\n")

        if not node._client.wait_for_server(timeout_sec=15.0):
            print("navigate_to_pose 액션 서버가 없다. Nav2 를 먼저 띄워라.")
            return 1

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = args.frame
        goal.pose.header.stamp = rclpy.time.Time().to_msg()   # 0 = 최신 TF 를 쓴다
        if args.frame == "map":
            goal.pose.pose.position.x = gx
            goal.pose.pose.position.y = gy
            goal.pose.pose.orientation.z = math.sin(yaw0 / 2)
            goal.pose.pose.orientation.w = math.cos(yaw0 / 2)
        else:
            # 로봇 프레임: 그냥 정면으로 forward m.  방위는 그대로 유지.
            goal.pose.pose.position.x = args.forward
            goal.pose.pose.position.y = 0.0
            goal.pose.pose.orientation.w = 1.0
            print(f"프레임 {args.frame} 기준 목표: x={args.forward:+.3f} y=0.000")

        fut = node._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            print("목표가 거부됐다.")
            return 2
        print("목표 수락됨. 진행:")

        result_fut = handle.get_result_async()
        start = time.time()
        last = 0.0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if result_fut.done():
                break
            now = time.time()
            if now - last >= 2.0:
                last = now
                x, y, yaw = node.xy_yaw()
                print(f"  {now-start:5.1f}s  x={x:+.3f} y={y:+.3f} "
                      f"yaw={math.degrees(yaw):+7.1f}  남은거리 {math.hypot(gx-x, gy-y):.3f} m")
            if now - start > args.timeout:
                print(f"\n{args.timeout} 초 초과 -- 취소한다")
                handle.cancel_goal_async()
                rclpy.spin_once(node, timeout_sec=2.0)
                code = 3
                break

        if result_fut.done():
            status = result_fut.result().status
            names = {GoalStatus.STATUS_SUCCEEDED: "성공",
                     GoalStatus.STATUS_CANCELED: "취소됨",
                     GoalStatus.STATUS_ABORTED: "중단됨"}
            print(f"\n결과: {names.get(status, status)}")
            if status != GoalStatus.STATUS_SUCCEEDED:
                code = 4

        x, y, yaw = node.xy_yaw()
        print(f"최종  x={x:+.3f} y={y:+.3f} yaw={math.degrees(yaw):+.1f} deg")
        print(f"실제 이동  {math.hypot(x-x0, y-y0):.3f} m   "
              f"목표까지 남은 거리 {math.hypot(gx-x, gy-y):.3f} m")
        return code
    except KeyboardInterrupt:
        print("\n중단")
        if handle is not None:
            handle.cancel_goal_async()
            rclpy.spin_once(node, timeout_sec=2.0)
        return 130
    finally:
        try:
            node.stop_motor()
        finally:
            node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
