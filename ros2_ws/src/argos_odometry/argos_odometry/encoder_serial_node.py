#!/usr/bin/env python3

import re
import serial

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64


class EncoderSerialNode(Node):

    def __init__(self):
        super().__init__('encoder_serial_node')

        # 현재 확인된 포트
        self.declare_parameter('left_port', '/dev/ttyCH341USB1')
        self.declare_parameter('right_port', '/dev/ttyCH341USB0')
        self.declare_parameter('baudrate', 115200)

        left_port = self.get_parameter('left_port').value
        right_port = self.get_parameter('right_port').value
        baudrate = self.get_parameter('baudrate').value

        self.left_pub = self.create_publisher(
            Int64,
            '/wheel_ticks/left',
            10
        )

        self.right_pub = self.create_publisher(
            Int64,
            '/wheel_ticks/right',
            10
        )

        try:
            self.left_serial = serial.Serial(
                left_port,
                baudrate,
                timeout=0
            )

            self.get_logger().info(
                f'LEFT encoder connected: {left_port}'
            )

        except Exception as e:
            self.get_logger().error(
                f'LEFT encoder connection failed: {e}'
            )
            self.left_serial = None

        try:
            self.right_serial = serial.Serial(
                right_port,
                baudrate,
                timeout=0
            )

            self.get_logger().info(
                f'RIGHT encoder connected: {right_port}'
            )

        except Exception as e:
            self.get_logger().error(
                f'RIGHT encoder connection failed: {e}'
            )
            self.right_serial = None

        self.timer = self.create_timer(
            0.01,
            self.timer_callback
        )

    def extract_count(self, text):
        """
        문자열에서 signed integer를 추출.

        예:
        '1234'     -> 1234
        'E,1234'   -> 1234
        'COUNT=-5' -> -5

        누적 encoder count를 보내는 형태를 기준으로 함.
        """

        numbers = re.findall(r'-?\d+', text)

        if not numbers:
            return None

        return int(numbers[-1])

    def read_encoder(self, serial_port, publisher, name):

        if serial_port is None:
            return

        try:
            while serial_port.in_waiting > 0:

                raw = serial_port.readline()

                text = raw.decode(
                    'utf-8',
                    errors='ignore'
                ).strip()

                if not text:
                    continue

                count = self.extract_count(text)

                if count is None:
                    self.get_logger().warning(
                        f'{name}: cannot parse "{text}"'
                    )
                    continue

                msg = Int64()
                msg.data = count

                publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(
                f'{name} serial error: {e}'
            )

    def timer_callback(self):

        self.read_encoder(
            self.left_serial,
            self.left_pub,
            'LEFT'
        )

        self.read_encoder(
            self.right_serial,
            self.right_pub,
            'RIGHT'
        )


def main(args=None):

    rclpy.init(args=args)

    node = EncoderSerialNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        if node.left_serial is not None:
            node.left_serial.close()

        if node.right_serial is not None:
            node.right_serial.close()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
