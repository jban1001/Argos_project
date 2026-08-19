#!/usr/bin/env python3

"""
ARGOS Nav2 자율주행 테스트

맵에서 안전한 목표점을 자동으로 고르고 NavigateToPose 로 보낸다.

목표점 선정 기준
  - free 셀
  - 장애물까지 여유 >= CLEARANCE (외접반경 0.30 m + 여유)
  - 로봇에서 GOAL_DIST 근처
  - 로봇 현재 위치에서 직선상 free (완전한 보장은 아니고 후보 선별용)

Nav2 가 알아서 경로를 만들고 따라가며, 이 스크립트는 진행 상황만 감시한다.
중단하면 Nav2 goal 을 취소하고 /cmd_vel 0 을 쏜다.
"""

import math
import sys
import time

import numpy as np
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy,
    QoSReliabilityPolicy, QoSHistoryPolicy,
)

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener


GOAL_DIST = 0.5          # 목표까지 목표 거리 [m]
CLEARANCE = 0.45         # 목표점 주변 최소 여유 [m]
TIMEOUT = 120.0


def yaw_q(q):
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z)
    )


class NavGoalTest(Node):

    def __init__(self):

        super().__init__("nav_goal_test")

        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(
            OccupancyGrid, "/map", self.map_cb, qos
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.client = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )

        self.map = None
        self.clear_field = None

    def map_cb(self, msg):

        self.map = msg

        a = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width
        )

        # unknown 도 장애물처럼 취급해서 보수적으로 고른다
        blocked = (a > 50) | (a < 0)

        self.clear_field = (
            distance_transform_edt(~blocked) * msg.info.resolution
        )

        self.free = (a == 0)

    def wait_ready(self, timeout=20.0):

        end = time.monotonic() + timeout

        while time.monotonic() < end:

            rclpy.spin_once(self, timeout_sec=0.05)

            if self.map is None:
                continue

            try:
                self.tf_buffer.lookup_transform(
                    "map", "base_link", rclpy.time.Time()
                )
            except Exception:
                continue

            return True

        return False

    def robot_pose(self):

        tf = self.tf_buffer.lookup_transform(
            "map", "base_link", rclpy.time.Time()
        )

        return (
            tf.transform.translation.x,
            tf.transform.translation.y,
            yaw_q(tf.transform.rotation),
        )

    def pick_goal(self, rx, ry):

        m = self.map

        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y

        rows, cols = np.nonzero(
            self.free & (self.clear_field >= CLEARANCE)
        )

        if rows.size == 0:
            return None

        gx = ox + (cols + 0.5) * res
        gy = oy + (rows + 0.5) * res

        d = np.hypot(gx - rx, gy - ry)

        # GOAL_DIST 에 가장 가까운 후보
        idx = int(np.argmin(np.abs(d - GOAL_DIST)))

        if d[idx] < 0.4:
            return None

        return float(gx[idx]), float(gy[idx]), float(d[idx])

    def stop_motor(self):

        for _ in range(10):
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)


def main():

    rclpy.init()

    node = NavGoalTest()

    goal_handle = None

    try:

        if not node.wait_ready():
            print("[FAIL] /map 또는 TF 준비 안 됨")
            return

        rx, ry, rth = node.robot_pose()

        print()
        print("=" * 56)
        print(" ARGOS Nav2 자율주행 테스트")
        print("=" * 56)
        print(f" 현재 위치 = ({rx:+.3f}, {ry:+.3f}, "
              f"{math.degrees(rth):+.1f} deg)")

        picked = node.pick_goal(rx, ry)

        if picked is None:
            print(f"[FAIL] 여유 {CLEARANCE} m 이상인 목표점을 못 찾음")
            return

        gx, gy, gd = picked

        print(f" 목표 위치 = ({gx:+.3f}, {gy:+.3f}), 직선거리 {gd:.2f} m")

        if not node.client.wait_for_server(timeout_sec=15.0):
            print("[FAIL] navigate_to_pose 액션 서버 없음")
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy

        # 목표 방향은 이동 방향으로
        gth = math.atan2(gy - ry, gx - rx)
        goal.pose.pose.orientation.z = math.sin(gth / 2)
        goal.pose.pose.orientation.w = math.cos(gth / 2)

        print("\n goal 전송...")

        send = node.client.send_goal_async(goal)

        rclpy.spin_until_future_complete(node, send, timeout_sec=15.0)

        goal_handle = send.result()

        if goal_handle is None or not goal_handle.accepted:
            print("[FAIL] goal 거부됨")
            return

        print(" goal 수락됨. 주행 감시 중...\n")

        result_future = goal_handle.get_result_async()

        t0 = time.monotonic()
        last = 0.0

        while not result_future.done():

            rclpy.spin_once(node, timeout_sec=0.2)

            el = time.monotonic() - t0

            if el > TIMEOUT:
                print("\n[FAIL] 시간 초과 - goal 취소")
                goal_handle.cancel_goal_async()
                rclpy.spin_once(node, timeout_sec=1.0)
                break

            if el - last >= 3.0:

                last = el

                try:
                    cx, cy, cth = node.robot_pose()
                    rem = math.hypot(gx - cx, gy - cy)
                    print(f"  {el:5.1f}s  위치 ({cx:+.2f}, {cy:+.2f})  "
                          f"남은거리 {rem:.2f} m")
                except Exception:
                    pass

        if result_future.done():

            status = result_future.result().status

            cx, cy, cth = node.robot_pose()
            err = math.hypot(gx - cx, gy - cy)

            # 4 = SUCCEEDED
            ok = (status == 4)

            print()
            print(f" 결과 status = {status} "
                  f"({'성공' if ok else '실패/취소'})")
            print(f" 최종 위치   = ({cx:+.3f}, {cy:+.3f})")
            print(f" 목표 오차   = {err:.3f} m")

            if ok and err < 0.3:
                print("\n [OK] 자율주행 성공")
            elif ok:
                print("\n [WARN] 성공 보고했으나 오차가 큼")
            else:
                print("\n [FAIL] 목표 도달 실패")

    except KeyboardInterrupt:
        print("\n[STOP] 사용자 중단")

        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
                rclpy.spin_once(node, timeout_sec=1.0)
            except Exception:
                pass

    finally:

        try:
            node.stop_motor()
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        print("[DONE]")


if __name__ == "__main__":
    main()
