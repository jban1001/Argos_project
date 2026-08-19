#!/usr/bin/env python3

import csv
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


OUT = "/home/odyssey/argos_project/nav_control_debug.csv"


def yaw_from_q(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class DebugNode(Node):

    def __init__(self):
        super().__init__("argos_nav_control_debug")

        self.cmd_nav = Twist()
        self.cmd_out = Twist()
        self.rotating = False

        self.create_subscription(
            Twist,
            "/cmd_vel_nav",
            self.nav_cb,
            10
        )

        self.create_subscription(
            Twist,
            "/cmd_vel",
            self.out_cb,
            10
        )

        self.create_subscription(
            Bool,
            "/is_rotating_to_heading",
            self.rot_cb,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.f = open(OUT, "w", newline="")
        self.writer = csv.writer(self.f)

        self.writer.writerow([
            "time",
            "x",
            "y",
            "yaw_deg",
            "nav_v",
            "nav_w",
            "out_v",
            "out_w",
            "rotate_to_heading",
        ])

        self.t0 = time.monotonic()

        self.create_timer(
            0.2,
            self.tick
        )

        print(f"[LOG] {OUT}")


    def nav_cb(self, msg):
        self.cmd_nav = msg


    def out_cb(self, msg):
        self.cmd_out = msg


    def rot_cb(self, msg):
        self.rotating = msg.data


    def tick(self):

        try:
            tf = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time()
            )

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            yaw = math.degrees(
                yaw_from_q(tf.transform.rotation)
            )

        except Exception:
            return

        t = time.monotonic() - self.t0

        nv = self.cmd_nav.linear.x
        nw = self.cmd_nav.angular.z

        ov = self.cmd_out.linear.x
        ow = self.cmd_out.angular.z

        self.writer.writerow([
            f"{t:.3f}",
            f"{x:.4f}",
            f"{y:.4f}",
            f"{yaw:.2f}",
            f"{nv:.4f}",
            f"{nw:.4f}",
            f"{ov:.4f}",
            f"{ow:.4f}",
            int(self.rotating),
        ])

        self.f.flush()

        print(
            f"{t:6.1f}s "
            f"pose=({x:+.2f},{y:+.2f},{yaw:+6.1f}°) "
            f"NAV=({nv:+.3f},{nw:+.3f}) "
            f"OUT=({ov:+.3f},{ow:+.3f}) "
            f"ROT={int(self.rotating)}"
        )


    def close(self):
        self.f.close()


def main():

    rclpy.init()

    node = DebugNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.close()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
