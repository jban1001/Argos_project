#!/usr/bin/env python3

import math
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64


class OdomCalibrator(Node):

    def __init__(self):
        super().__init__('odom_calibrator')

        self.left_ticks = None
        self.right_ticks = None

        self.create_subscription(
            Int64,
            '/wheel_ticks/left',
            self.left_callback,
            10
        )

        self.create_subscription(
            Int64,
            '/wheel_ticks/right',
            self.right_callback,
            10
        )

    def left_callback(self, msg):
        self.left_ticks = msg.data

    def right_callback(self, msg):
        self.right_ticks = msg.data

    def wait_for_encoders(self):

        print('\n[대기] 좌/우 엔코더 데이터를 기다리는 중...')

        start = time.time()

        while rclpy.ok():

            rclpy.spin_once(self, timeout_sec=0.1)

            if (
                self.left_ticks is not None
                and self.right_ticks is not None
            ):
                print('[OK] 양쪽 엔코더 데이터 수신 완료')
                print(f' LEFT  = {self.left_ticks}')
                print(f' RIGHT = {self.right_ticks}')
                return

            if time.time() - start > 10.0:
                raise RuntimeError(
                    '10초 동안 encoder topic을 받지 못했습니다.'
                )

    def refresh(self):

        # 최신 값을 여러 번 받아 안정화
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.03)

    def snapshot(self):

        self.refresh()

        return (
            int(self.left_ticks),
            int(self.right_ticks)
        )


def main(args=None):

    rclpy.init(args=args)

    node = OdomCalibrator()

    try:

        node.wait_for_encoders()

        print()
        print('==========================================')
        print('       ARGOS ODOMETRY AUTO CALIBRATION')
        print('==========================================')
        print()
        print('이 프로그램은 다음을 자동 계산합니다.')
        print()
        print('1. LEFT encoder 방향')
        print('2. RIGHT encoder 방향')
        print('3. LEFT ticks/m')
        print('4. RIGHT ticks/m')
        print('5. effective track width')
        print()

        # ==================================================
        # STEP 1 : 1 meter straight
        # ==================================================

        print('------------------------------------------')
        print('[STEP 1] 정확히 1.000 m 직진')
        print('------------------------------------------')
        print()
        print('1) 바닥에 시작 위치를 표시하세요.')
        print('2) 로봇을 정면 방향으로 맞추세요.')
        print('3) 아직 움직이지 마세요.')

        input('\n준비되면 ENTER > ')

        left_start, right_start = node.snapshot()

        print()
        print(f'[START] LEFT  = {left_start}')
        print(f'[START] RIGHT = {right_start}')

        print()
        print('이제 로봇을 정확히 1.000 m 앞으로 이동하세요.')
        print('수동으로 밀어도 되고 manual_drive를 사용해도 됩니다.')
        print('1 m 이동 후 로봇을 정지시키세요.')

        input('\n1 m 이동 완료 후 ENTER > ')

        left_end, right_end = node.snapshot()

        print()
        print(f'[END] LEFT  = {left_end}')
        print(f'[END] RIGHT = {right_end}')

        delta_left_raw = left_end - left_start
        delta_right_raw = right_end - right_start

        if delta_left_raw == 0 or delta_right_raw == 0:
            raise RuntimeError(
                'encoder tick 변화가 없습니다.'
            )

        # 앞으로 움직일 때의 방향 자동 판별
        left_sign = 1.0 if delta_left_raw > 0 else -1.0
        right_sign = 1.0 if delta_right_raw > 0 else -1.0

        left_ticks_per_meter = abs(delta_left_raw)
        right_ticks_per_meter = abs(delta_right_raw)

        print()
        print('========== 직진 보정 결과 ==========')

        print(
            f'LEFT raw delta       = {delta_left_raw}'
        )

        print(
            f'RIGHT raw delta      = {delta_right_raw}'
        )

        print(
            f'LEFT sign            = {left_sign:+.0f}'
        )

        print(
            f'RIGHT sign           = {right_sign:+.0f}'
        )

        print(
            f'LEFT ticks_per_meter = '
            f'{left_ticks_per_meter:.3f}'
        )

        print(
            f'RIGHT ticks_per_meter = '
            f'{right_ticks_per_meter:.3f}'
        )

        # ==================================================
        # STEP 2 : 360 degree CCW rotation
        # ==================================================

        print()
        print('------------------------------------------')
        print('[STEP 2] 제자리 반시계방향 360도 회전')
        print('------------------------------------------')
        print()
        print('로봇의 현재 방향을 바닥에 표시하세요.')
        print()
        print('다음 단계에서는:')
        print()
        print('       ↺ 반시계 방향')
        print()
        print('으로 정확히 한 바퀴 돌린 뒤')
        print('처음 방향으로 돌아오면 됩니다.')
        print()
        print('가능하면 제자리 회전으로 수행하세요.')

        input('\n회전 시작 준비되면 ENTER > ')

        rot_left_start, rot_right_start = node.snapshot()

        print()
        print(
            f'[ROT START] LEFT  = {rot_left_start}'
        )

        print(
            f'[ROT START] RIGHT = {rot_right_start}'
        )

        print()
        print('이제 반시계 방향으로 정확히 360° 회전하세요.')

        input('\n360도 회전 완료 후 ENTER > ')

        rot_left_end, rot_right_end = node.snapshot()

        rot_left_raw = (
            rot_left_end - rot_left_start
        )

        rot_right_raw = (
            rot_right_end - rot_right_start
        )

        # 직진 기준 sign을 적용해 실제 전/후진 방향으로 정규화
        rot_left_ticks = (
            rot_left_raw * left_sign
        )

        rot_right_ticks = (
            rot_right_raw * right_sign
        )

        delta_left_m = (
            rot_left_ticks /
            left_ticks_per_meter
        )

        delta_right_m = (
            rot_right_ticks /
            right_ticks_per_meter
        )

        # Differential-drive:
        # dtheta = (dR - dL) / track_width
        #
        # CCW 360deg = +2*pi
        track_width = (
            delta_right_m - delta_left_m
        ) / (2.0 * math.pi)

        # 혹시 회전 방향이 반대로 수행되어 음수가 나오더라도
        # 폭 자체는 양수
        track_width = abs(track_width)

        print()
        print('========== 회전 보정 결과 ==========')

        print(
            f'LEFT rotation distance  = '
            f'{delta_left_m:.5f} m'
        )

        print(
            f'RIGHT rotation distance = '
            f'{delta_right_m:.5f} m'
        )

        print()
        print(
            f'Effective track width   = '
            f'{track_width:.5f} m'
        )

        # sanity check
        if track_width < 0.10:
            print()
            print('[경고] track_width가 너무 작습니다.')
            print('360도 회전이 정확했는지 확인하세요.')

        elif track_width > 1.50:
            print()
            print('[경고] track_width가 너무 큽니다.')
            print('360도 회전 또는 tick 값을 확인하세요.')

        # ==================================================
        # Save YAML
        # ==================================================

        output_dir = os.path.expanduser(
            '~/argos_project/config'
        )

        os.makedirs(
            output_dir,
            exist_ok=True
        )

        output_file = os.path.join(
            output_dir,
            'odometry_calibration.yaml'
        )

        yaml_text = f"""wheel_odometry_node:
  ros__parameters:
    left_ticks_per_meter: {left_ticks_per_meter:.6f}
    right_ticks_per_meter: {right_ticks_per_meter:.6f}
    left_sign: {left_sign:.1f}
    right_sign: {right_sign:.1f}
    track_width: {track_width:.6f}
"""

        with open(
            output_file,
            'w',
            encoding='utf-8'
        ) as f:

            f.write(yaml_text)

        print()
        print('==========================================')
        print('            CALIBRATION COMPLETE')
        print('==========================================')

        print()
        print(
            f'LEFT ticks/m  : '
            f'{left_ticks_per_meter:.3f}'
        )

        print(
            f'RIGHT ticks/m : '
            f'{right_ticks_per_meter:.3f}'
        )

        print(
            f'LEFT sign     : '
            f'{left_sign:+.0f}'
        )

        print(
            f'RIGHT sign    : '
            f'{right_sign:+.0f}'
        )

        print(
            f'Track width   : '
            f'{track_width:.5f} m'
        )

        print()
        print('설정 파일 저장:')
        print(output_file)
        print()

    except KeyboardInterrupt:

        print('\nCalibration cancelled.')

    except Exception as e:

        print()
        print(f'[ERROR] {e}')

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
