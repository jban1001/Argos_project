#!/usr/bin/env python3

"""
ARGOS MPU6050 (MPU6500 호환 클론) IMU 드라이버

publish : /imu/data_raw   sensor_msgs/Imu   frame_id = imu_link
          /diagnostics    diagnostic_msgs/DiagnosticArray

하드웨어 (2026-08-26 실측)
--------------------------
  Jetson Orin Nano 40핀 헤더 27(SDA) / 28(SCL) = /dev/i2c-1
  주소 0x68  (AD0 = GND)

  WHO_AM_I = 0x72 이다. 정품 InvenSense MPU-6050 은 0x68 을 낸다.
  0x72 는 GY-521 보드에 흔히 올라가는 MPU-6500 계열 호환 다이다.

  온도 레지스터가 이를 확증한다. raw = 2099 일 때
      MPU6050 식  raw/340    + 36.53 = 42.7 C   (실온과 안 맞음)
      MPU6500 식  raw/333.87 + 21    = 27.3 C   (MLX90640 실측 27 C 와 일치)

  그래서 WHO_AM_I 를 0x68 로 검사하면 안 된다. 정상 장치를 거부하게 된다.
  여기서는 0x00 / 0xFF (버스 죽음) 만 걸러내고 실제 값은 로그로 남긴다.

  자이로/가속도 레지스터 맵과 감도는 두 계열이 같으므로
  (FS_SEL=0 -> 131 LSB/(deg/s), AFS_SEL=0 -> 16384 LSB/g)
  스케일링은 분기하지 않는다.

왜 smbus2 를 안 쓰는가
----------------------
python3-smbus2 가 설치돼 있지 않다. 시스템 패키지를 늘리지 않으려고
/dev/i2c-N 을 직접 열고 ioctl(I2C_RDWR) 로 접근한다.

I2C_RDWR 을 쓰는 이유는 repeated-start 때문이다.
i2c-1 은 온보드 전원 모니터(INA3221 0x40, 0x25)와 버스를 공유한다.
write 후 read 를 따로 하면 그 사이에 커널 드라이버가 끼어들어
엉뚱한 레지스터를 읽을 수 있다. 두 메시지를 한 번의 ioctl 로 넘기면
커널 i2c core 가 원자적으로 처리한다.

orientation 을 왜 안 내는가
---------------------------
MPU6050 에는 magnetometer 가 없다. 절대 yaw 기준이 없으므로
자체 적분한 yaw 는 반드시 드리프트한다.
EKF 가 orientation 을 믿게 만들면 오히려 해가 되므로
    orientation_covariance[0] = -1.0
로 "제공하지 않음" 을 명시한다. (REP-145)

초기 EKF 는 angular_velocity.z 하나만 쓴다.

I2C 오류 처리 원칙
------------------
읽기에 실패하면 그 주기는 "아무것도 발행하지 않는다".
마지막 정상 샘플을 새 timestamp 로 다시 내보내면
EKF 는 그것을 새 측정으로 믿고 회전이 멈춘 것으로 오해한다.
정지한 로봇과 고장난 센서를 구분할 수 없게 된다.
"""

import ctypes
import fcntl
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import (
    DiagnosticArray,
    DiagnosticStatus,
    KeyValue,
)


# ---------------------------------------------------------------
# Linux i2c-dev ioctl
# ---------------------------------------------------------------

I2C_RDWR = 0x0707
I2C_M_RD = 0x0001


class _I2cMsg(ctypes.Structure):
    # struct i2c_msg { __u16 addr; __u16 flags; __u16 len; __u8 *buf; }
    # 64bit 에서는 buf 정렬 때문에 len 뒤에 2 byte padding 이 들어간다.
    # ctypes 가 c_void_p 를 8 byte 경계에 맞추며 자동으로 넣어 준다.
    _fields_ = [
        ("addr", ctypes.c_uint16),
        ("flags", ctypes.c_uint16),
        ("len", ctypes.c_uint16),
        ("buf", ctypes.c_void_p),
    ]


class _I2cRdwrData(ctypes.Structure):
    _fields_ = [
        ("msgs", ctypes.POINTER(_I2cMsg)),
        ("nmsgs", ctypes.c_uint32),
    ]


class I2cError(Exception):
    pass


class I2cDevice:
    """ioctl(I2C_RDWR) 기반 최소 I2C 접근자."""

    def __init__(self, bus: int, address: int):
        self.bus = bus
        self.address = address
        self.path = "/dev/i2c-{}".format(bus)

        try:
            self.fd = os.open(self.path, os.O_RDWR)
        except OSError as exc:
            raise I2cError(
                "{} 를 열 수 없다: {}".format(self.path, exc)
            ) from exc

    def close(self):
        if getattr(self, "fd", None) is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None

    def _rdwr(self, msgs):
        arr = (_I2cMsg * len(msgs))(*msgs)

        data = _I2cRdwrData()
        data.msgs = arr
        data.nmsgs = len(msgs)

        try:
            fcntl.ioctl(self.fd, I2C_RDWR, data)
        except OSError as exc:
            raise I2cError("I2C 전송 실패: {}".format(exc)) from exc

    def write_reg(self, reg: int, value: int):
        buf = (ctypes.c_uint8 * 2)(reg & 0xFF, value & 0xFF)

        msg = _I2cMsg(
            addr=self.address,
            flags=0,
            len=2,
            buf=ctypes.cast(buf, ctypes.c_void_p),
        )

        self._rdwr([msg])

    def read_regs(self, reg: int, length: int) -> bytes:
        """repeated-start 로 reg 지정 후 length byte 를 원자적으로 읽는다."""

        out_buf = (ctypes.c_uint8 * 1)(reg & 0xFF)
        in_buf = (ctypes.c_uint8 * length)()

        write_msg = _I2cMsg(
            addr=self.address,
            flags=0,
            len=1,
            buf=ctypes.cast(out_buf, ctypes.c_void_p),
        )

        read_msg = _I2cMsg(
            addr=self.address,
            flags=I2C_M_RD,
            len=length,
            buf=ctypes.cast(in_buf, ctypes.c_void_p),
        )

        self._rdwr([write_msg, read_msg])

        return bytes(in_buf)


# ---------------------------------------------------------------
# MPU6050 / MPU6500 레지스터
# ---------------------------------------------------------------

REG_SMPLRT_DIV = 0x19
REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_CONFIG2 = 0x1D      # MPU6500 계열에만 있다. 6050 에서는 무시된다.
REG_ACCEL_XOUT_H = 0x3B
REG_PWR_MGMT_1 = 0x6B
REG_WHO_AM_I = 0x75

GRAVITY = 9.80665
DEG_TO_RAD = math.pi / 180.0

# FS_SEL / AFS_SEL 에 따른 감도. 두 계열이 동일하다.
GYRO_LSB = {0: 131.0, 1: 65.5, 2: 32.8, 3: 16.4}
ACCEL_LSB = {0: 16384.0, 1: 8192.0, 2: 4096.0, 3: 2048.0}


def _s16(hi: int, lo: int) -> int:
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


class Mpu6050Node(Node):

    def __init__(self):
        super().__init__("mpu6050_node")

        # ---------------- 파라미터 ----------------

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 0x68)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("topic", "/imu/data_raw")

        # 0 = +-250 dps. 실내 주행 최대 0.64 rad/s = 36 deg/s 이므로
        # 가장 민감한 범위를 써서 분해능을 확보한다.
        self.declare_parameter("gyro_range", 0)
        self.declare_parameter("accel_range", 0)

        # DLPF_CFG=3 -> 대역폭 42 Hz, 그룹지연 4.8 ms.
        # 100 Hz 샘플링의 Nyquist 50 Hz 아래라 에일리어싱 여유가 있다.
        self.declare_parameter("dlpf_config", 3)

        # 시작 시 gyro bias 측정 [s]. 0 이면 하지 않는다.
        self.declare_parameter("calibration_seconds", 7.0)

        # 측정한 bias 를 발행값에서 뺄지 여부.
        # EKF 만 쓸 거면 true 가 편하고, 원시값을 보고 싶으면 false.
        self.declare_parameter("apply_gyro_bias", True)

        # 수동으로 bias 를 주입할 때 사용 [rad/s]. NaN 이면 무시.
        self.declare_parameter("gyro_bias_x", float("nan"))
        self.declare_parameter("gyro_bias_y", float("nan"))
        self.declare_parameter("gyro_bias_z", float("nan"))

        # 분산 [ (rad/s)^2 ] 와 [ (m/s^2)^2 ].
        # calibration 단계에서 실측한 표준편차로 갱신할 것.
        self.declare_parameter("angular_velocity_variance", 4.0e-4)
        self.declare_parameter("linear_acceleration_variance", 4.0e-2)

        # 연속 실패가 이만큼이면 ERROR 로 올린다.
        self.declare_parameter("error_threshold", 10)

        self.bus = int(self.get_parameter("i2c_bus").value)
        self.address = int(self.get_parameter("i2c_address").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.rate = float(self.get_parameter("publish_rate").value)
        self.topic = str(self.get_parameter("topic").value)

        self.gyro_range = int(self.get_parameter("gyro_range").value)
        self.accel_range = int(self.get_parameter("accel_range").value)
        self.dlpf = int(self.get_parameter("dlpf_config").value)

        self.calibration_seconds = float(
            self.get_parameter("calibration_seconds").value
        )

        self.apply_bias = bool(
            self.get_parameter("apply_gyro_bias").value
        )

        self.gyro_var = float(
            self.get_parameter("angular_velocity_variance").value
        )

        self.accel_var = float(
            self.get_parameter("linear_acceleration_variance").value
        )

        self.error_threshold = int(
            self.get_parameter("error_threshold").value
        )

        if self.gyro_range not in GYRO_LSB:
            raise ValueError("gyro_range 는 0..3 이어야 한다")

        if self.accel_range not in ACCEL_LSB:
            raise ValueError("accel_range 는 0..3 이어야 한다")

        self.gyro_lsb = GYRO_LSB[self.gyro_range]
        self.accel_lsb = ACCEL_LSB[self.accel_range]

        # ---------------- 상태 ----------------

        self.bias = [0.0, 0.0, 0.0]
        self.bias_valid = False

        self.error_count = 0
        self.consecutive_errors = 0
        self.sample_count = 0

        self.last_stamp_ns = None
        self.jitter_max = 0.0
        self.last_error_message = ""

        # ---------------- 하드웨어 ----------------

        self.dev = I2cDevice(self.bus, self.address)
        self.who_am_i = self._setup_device()

        # 파라미터로 bias 를 직접 준 경우 calibration 보다 우선한다.
        manual = [
            float(self.get_parameter("gyro_bias_x").value),
            float(self.get_parameter("gyro_bias_y").value),
            float(self.get_parameter("gyro_bias_z").value),
        ]

        if all(math.isfinite(v) for v in manual):
            self.bias = manual
            self.bias_valid = True

            self.get_logger().info(
                "파라미터로 받은 gyro bias 사용: "
                "x={:.6f} y={:.6f} z={:.6f} rad/s".format(*self.bias)
            )

        elif self.calibration_seconds > 0.0:
            self._calibrate()

        # ---------------- ROS ----------------

        self.imu_pub = self.create_publisher(
            Imu, self.topic, qos_profile_sensor_data
        )

        self.diag_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )

        self.timer = self.create_timer(1.0 / self.rate, self._on_timer)

        self.diag_timer = self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info(
            "MPU6050 노드 시작: /dev/i2c-{} 0x{:02X} "
            "WHO_AM_I=0x{:02X} {:.0f} Hz -> {}".format(
                self.bus, self.address, self.who_am_i,
                self.rate, self.topic
            )
        )

    # -----------------------------------------------------------
    # 초기화
    # -----------------------------------------------------------

    def _setup_device(self) -> int:

        who = self.dev.read_regs(REG_WHO_AM_I, 1)[0]

        # 0x00 / 0xFF 는 버스가 죽었거나 장치가 없다는 뜻이다.
        # 그 밖의 값은 클론일 수 있으므로 통과시키고 로그만 남긴다.
        if who in (0x00, 0xFF):
            raise I2cError(
                "WHO_AM_I=0x{:02X} - 장치가 응답하지 않는다".format(who)
            )

        if who != 0x68:
            self.get_logger().warn(
                "WHO_AM_I=0x{:02X} 로 정품 MPU-6050(0x68) 이 아니다. "
                "MPU-6500 계열 호환 다이로 보이며 자이로/가속도 "
                "레지스터와 감도는 동일하므로 그대로 진행한다.".format(who)
            )

        # sleep 해제 + clock source = gyro X PLL.
        # 내부 8 MHz 오실레이터보다 드리프트가 작다.
        self.dev.write_reg(REG_PWR_MGMT_1, 0x01)
        time.sleep(0.05)

        self.dev.write_reg(REG_CONFIG, self.dlpf & 0x07)
        self.dev.write_reg(REG_GYRO_CONFIG, (self.gyro_range & 0x03) << 3)
        self.dev.write_reg(REG_ACCEL_CONFIG, (self.accel_range & 0x03) << 3)

        # MPU6500 계열의 accel DLPF. MPU6050 에서는 예약 레지스터라 무시된다.
        self.dev.write_reg(REG_ACCEL_CONFIG2, 0x03)

        # DLPF 를 켰으므로 내부 출력은 1 kHz 다.
        # SMPLRT_DIV = 1000/rate - 1
        div = int(round(1000.0 / self.rate)) - 1
        div = max(0, min(255, div))

        self.dev.write_reg(REG_SMPLRT_DIV, div)

        actual = 1000.0 / (1 + div)

        if abs(actual - self.rate) > 1.0:
            self.get_logger().warn(
                "요청 주기 {:.1f} Hz 는 1000/(1+N) 으로 정확히 "
                "표현되지 않는다. 센서 내부 주기는 {:.1f} Hz 다.".format(
                    self.rate, actual
                )
            )

        time.sleep(0.05)

        return who

    def _calibrate(self):
        """정지 상태 전제. gyro bias 와 잡음을 측정한다."""

        self.get_logger().info(
            "gyro bias 측정 {:.1f} 초. 로봇을 완전히 정지시킬 것.".format(
                self.calibration_seconds
            )
        )

        samples = [[], [], []]
        accel_samples = [[], [], []]

        failures = 0
        deadline = time.monotonic() + self.calibration_seconds
        period = 1.0 / self.rate

        while time.monotonic() < deadline:

            try:
                gyro, accel, _ = self._read_sample()

            except I2cError:
                failures += 1
                time.sleep(period)
                continue

            for i in range(3):
                samples[i].append(gyro[i])
                accel_samples[i].append(accel[i])

            time.sleep(period)

        n = len(samples[0])

        if n < 10:
            self.get_logger().error(
                "calibration 표본이 {} 개뿐이다. bias 를 쓰지 않는다.".format(n)
            )
            return

        axis_names = ("x", "y", "z")

        for i in range(3):
            mean = sum(samples[i]) / n

            var = sum((v - mean) ** 2 for v in samples[i]) / n
            std = math.sqrt(var)

            peak = max(abs(v - mean) for v in samples[i])

            self.bias[i] = mean

            self.get_logger().info(
                "gyro {}: mean={:+.6f} rad/s ({:+.4f} deg/s)  "
                "std={:.6f}  max_dev={:.6f} rad/s".format(
                    axis_names[i], mean, mean / DEG_TO_RAD, std, peak
                )
            )

        for i in range(3):
            mean = sum(accel_samples[i]) / n
            std = math.sqrt(
                sum((v - mean) ** 2 for v in accel_samples[i]) / n
            )

            self.get_logger().info(
                "accel {}: mean={:+.4f} m/s^2  std={:.4f}".format(
                    axis_names[i], mean, std
                )
            )

        # 측정한 z 축 분산을 covariance 기본값으로 승격한다.
        # 실측이 파라미터 추정치보다 낫다.
        z_var = sum(
            (v - self.bias[2]) ** 2 for v in samples[2]
        ) / n

        if z_var > 0.0:
            self.gyro_var = max(z_var, 1.0e-8)

        self.bias_valid = True

        self.get_logger().info(
            "calibration 완료: 표본 {} 개, 실패 {} 회, "
            "gyro z 분산 {:.3e} (rad/s)^2".format(n, failures, self.gyro_var)
        )

        if not self.apply_bias:
            self.get_logger().info(
                "apply_gyro_bias=false 이므로 측정만 하고 빼지 않는다."
            )

    # -----------------------------------------------------------
    # 읽기
    # -----------------------------------------------------------

    def _read_sample(self):
        """(gyro[3] rad/s, accel[3] m/s^2, temp_raw) 를 돌려준다."""

        raw = self.dev.read_regs(REG_ACCEL_XOUT_H, 14)

        ax = _s16(raw[0], raw[1])
        ay = _s16(raw[2], raw[3])
        az = _s16(raw[4], raw[5])

        temp_raw = _s16(raw[6], raw[7])

        gx = _s16(raw[8], raw[9])
        gy = _s16(raw[10], raw[11])
        gz = _s16(raw[12], raw[13])

        accel = [
            ax / self.accel_lsb * GRAVITY,
            ay / self.accel_lsb * GRAVITY,
            az / self.accel_lsb * GRAVITY,
        ]

        gyro = [
            gx / self.gyro_lsb * DEG_TO_RAD,
            gy / self.gyro_lsb * DEG_TO_RAD,
            gz / self.gyro_lsb * DEG_TO_RAD,
        ]

        return gyro, accel, temp_raw

    # -----------------------------------------------------------
    # 주기 콜백
    # -----------------------------------------------------------

    def _on_timer(self):

        try:
            gyro, accel, _ = self._read_sample()

        except I2cError as exc:
            self.error_count += 1
            self.consecutive_errors += 1
            self.last_error_message = str(exc)

            # 오래된 표본을 새 timestamp 로 재발행하지 않는다.
            # 그러면 EKF 가 "회전이 멈췄다" 고 잘못 믿는다.
            if self.consecutive_errors in (1, self.error_threshold):
                self.get_logger().error(
                    "I2C 읽기 실패 (연속 {}회, 누적 {}회): {}".format(
                        self.consecutive_errors, self.error_count, exc
                    )
                )

            return

        self.consecutive_errors = 0

        if self.apply_bias and self.bias_valid:
            gyro = [gyro[i] - self.bias[i] for i in range(3)]

        # NaN/Inf 를 절대 내보내지 않는다.
        if not all(math.isfinite(v) for v in gyro + accel):
            self.error_count += 1
            self.last_error_message = "비유한값 표본"

            self.get_logger().error(
                "NaN/Inf 표본을 버렸다: gyro={} accel={}".format(gyro, accel)
            )

            return

        now = self.get_clock().now()
        stamp_ns = now.nanoseconds

        if self.last_stamp_ns is not None:
            dt = (stamp_ns - self.last_stamp_ns) / 1e9
            jitter = abs(dt - 1.0 / self.rate)
            self.jitter_max = max(self.jitter_max, jitter)

        self.last_stamp_ns = stamp_ns

        msg = Imu()

        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id

        # MPU6050 에는 magnetometer 가 없다.
        # orientation 을 제공하지 않는다는 REP-145 규약.
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]

        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]

        for i in (0, 4, 8):
            msg.angular_velocity_covariance[i] = self.gyro_var
            msg.linear_acceleration_covariance[i] = self.accel_var

        self.imu_pub.publish(msg)

        self.sample_count += 1

    # -----------------------------------------------------------
    # diagnostics
    # -----------------------------------------------------------

    def _publish_diagnostics(self):

        status = DiagnosticStatus()
        status.name = "argos: MPU6050 IMU"
        status.hardware_id = "i2c-{}:0x{:02X}".format(self.bus, self.address)

        if self.consecutive_errors >= self.error_threshold:
            status.level = DiagnosticStatus.ERROR
            status.message = "I2C 연속 실패 {}회".format(
                self.consecutive_errors
            )

        elif self.consecutive_errors > 0:
            status.level = DiagnosticStatus.WARN
            status.message = "I2C 간헐 실패"

        elif not self.bias_valid:
            status.level = DiagnosticStatus.WARN
            status.message = "gyro bias 미측정"

        else:
            status.level = DiagnosticStatus.OK
            status.message = "정상"

        status.values = [
            KeyValue(key="who_am_i", value="0x{:02X}".format(self.who_am_i)),
            KeyValue(key="samples", value=str(self.sample_count)),
            KeyValue(key="errors_total", value=str(self.error_count)),
            KeyValue(
                key="errors_consecutive",
                value=str(self.consecutive_errors),
            ),
            KeyValue(
                key="gyro_bias_z_rad_s",
                value="{:.6f}".format(self.bias[2]),
            ),
            KeyValue(
                key="bias_applied",
                value=str(bool(self.apply_bias and self.bias_valid)),
            ),
            KeyValue(
                key="jitter_max_s", value="{:.4f}".format(self.jitter_max)
            ),
            KeyValue(key="rate_hz", value="{:.1f}".format(self.rate)),
            KeyValue(key="last_error", value=self.last_error_message),
        ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]

        self.diag_pub.publish(array)

        # jitter 는 창 단위로 본다. 누적 최대는 의미가 흐려진다.
        self.jitter_max = 0.0

    def destroy_node(self):
        try:
            self.dev.close()
        except Exception:
            pass

        return super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = Mpu6050Node()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except I2cError as exc:
        print("MPU6050 초기화 실패: {}".format(exc))

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
