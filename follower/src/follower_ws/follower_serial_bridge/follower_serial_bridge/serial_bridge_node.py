#!/usr/bin/env python3
"""follower_serial_bridge -- the ONLY process allowed to open the Arduino port.

Spec section 8 requires a single owner for /dev/ttyACM*. Two processes sharing
a serial port interleave bytes and silently corrupt both directions, so the
port is opened with an exclusive lock and every other node reaches the MCU by
publishing to this bridge instead.

INPUT
  ~motor_command   std_msgs/String   validated MCU command line (reliable)

OUTPUT
  ~imu/data_raw    sensor_msgs/Imu           frame follower_imu_link, ~200 Hz
  ~mcu/telemetry   std_msgs/String           MCU TEL lines, ~10 Hz
  /diagnostics     diagnostic_msgs/DiagnosticArray               1 Hz

Default remappings put these under /follower (see the launch file).

SAFETY (spec section 29)
  * any exception in the reader thread stops the motors
  * shutdown, SIGINT and destruction all send S
  * a command watchdog sends S when the controller goes quiet
  * a serial disconnect is reported and reconnected to; while disconnected the
    MCU's own 350 ms command timeout is what actually holds the motors stopped
  * commands are whitelisted, so a malformed or NaN value can never reach the
    motor driver
"""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path

import rclpy
import serial
import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from .commands import invert_drive, validate_command
from .mcu_clock import McuClockSync, MicrosUnwrapper

GRAVITY = 9.80665
DEG_TO_RAD = math.pi / 180.0

class SerialBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("follower_serial_bridge")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 230400)
        self.declare_parameter("imu_frame_id", "follower_imu_link")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("expected_imu_rate", 200.0)
        self.declare_parameter("command_timeout", 0.5)
        # Must match MAX_PWM and the yaw clamp in followingbot_mega.ino.
        # 이 로봇은 모터 배선이 반대다. 실측으로 확인한 하드웨어 사실이므로
        # serial_bridge.yaml 에서 켠다. 코드 기본값은 false 로 두어, 배선을
        # 바로잡았을 때 설정만 끄면 되도록 한다.
        self.declare_parameter("invert_drive", False)
        self.declare_parameter("max_pwm", 180)
        self.declare_parameter("max_yaw_rate", 90.0)
        self.declare_parameter("reconnect_period", 2.0)
        # 5000 ppm is the ceramic-resonator datasheet limit; this robot
        # measures 928 ppm. It only clamps the fitted slope against nonsense.
        self.declare_parameter("max_skew_ppm", 5000.0)
        self.declare_parameter("clock_window_s", 1.0)
        self.declare_parameter("clock_history", 60)
        # Variances for one sample. Defaults are measured on this unit with
        # scripts/02_check_imu_stream.py; re-measure if the DLPF setting or the
        # full-scale range changes.
        self.declare_parameter("gyro_noise_std_dps", 0.037)
        self.declare_parameter("accel_noise_std_ms2", 0.05)

        self._port_name = self.get_parameter("port").value
        self._baud = int(self.get_parameter("baud").value)
        self._frame_id = self.get_parameter("imu_frame_id").value
        self._expected_rate = float(self.get_parameter("expected_imu_rate").value)
        self._command_timeout = float(self.get_parameter("command_timeout").value)
        self._invert_drive = bool(self.get_parameter("invert_drive").value)
        self._max_pwm = int(self.get_parameter("max_pwm").value)
        self._max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)
        self._reconnect_period = float(self.get_parameter("reconnect_period").value)

        gyro_var = (float(self.get_parameter("gyro_noise_std_dps").value) * DEG_TO_RAD) ** 2
        accel_var = float(self.get_parameter("accel_noise_std_ms2").value) ** 2
        self._gyro_cov = [gyro_var, 0.0, 0.0, 0.0, gyro_var, 0.0, 0.0, 0.0, gyro_var]
        self._accel_cov = [accel_var, 0.0, 0.0, 0.0, accel_var, 0.0, 0.0, 0.0, accel_var]

        self._load_calibration(str(self.get_parameter("calibration_file").value))

        # NB: not self._clock -- rclpy.node.Node already uses that attribute
        # for its own ROS clock, and shadowing it breaks create_timer().
        self._unwrap = MicrosUnwrapper()
        self._mcu_clock = McuClockSync(
            window_s=float(self.get_parameter("clock_window_s").value),
            history=int(self.get_parameter("clock_history").value),
            max_skew_ppm=float(self.get_parameter("max_skew_ppm").value),
        )

        self._serial: serial.Serial | None = None
        self._write_lock = threading.Lock()
        self._running = True
        self._last_command_time = 0.0
        self._stop_sent = True
        self._sample_count = 0
        self._malformed = 0
        self._last_rate = 0.0
        self._last_rate_time = time.monotonic()
        self._last_imu_stamp = 0.0
        self._connected = False
        self._mcu_ok = False
        self._last_telemetry = ""
        # Excess latency (observed minus estimated offset) collected between
        # diagnostic ticks. Measuring this INSIDE the bridge is the only honest
        # place: anything measured downstream also contains DDS and subscriber
        # scheduling, which has nothing to do with clock recovery.
        self._latency_window: list[float] = []

        self._imu_pub = self.create_publisher(Imu, "imu/data_raw", qos_profile_sensor_data)
        self._tel_pub = self.create_publisher(String, "mcu/telemetry", 10)
        self._diag_pub = self.create_publisher(
            DiagnosticArray,
            "/diagnostics",
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5,
            ),
        )
        self._cmd_sub = self.create_subscription(
            String, "motor_command", self._on_command, 10
        )

        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()
        self.create_timer(1.0, self._publish_diagnostics)
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(
            f"bridge starting on {self._port_name} @ {self._baud}, "
            f"imu frame '{self._frame_id}'"
        )

    # -- calibration ---------------------------------------------------------
    def _load_calibration(self, path: str) -> None:
        """Static accel bias/scale. See scripts/04_calibrate_accel.py for why.

        Gyro is deliberately left raw: its bias drifts with temperature and the
        VIO estimates it online, so freezing a boot-time value would be worse
        than doing nothing.
        """
        self._accel_lsb_per_g = 16384.0
        self._gyro_lsb_per_dps = 131.0
        self._accel_bias = [0.0, 0.0, 0.0]
        self._accel_scale = [1.0, 1.0, 1.0]
        self._calibrated = False

        if not path:
            self.get_logger().warn(
                "no calibration_file set: accel is published with nominal scale. "
                "This unit reads |a| = 8.38 m/s^2 instead of 9.81, so run "
                "scripts/04_calibrate_accel.py before trusting VIO."
            )
            return
        candidate = Path(path)
        if not candidate.is_file():
            self.get_logger().error(f"calibration_file '{path}' not found; using nominal scale")
            return
        try:
            with candidate.open("r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
            self._accel_lsb_per_g = float(data.get("accel_lsb_per_g", 16384.0))
            self._gyro_lsb_per_dps = float(data.get("gyro_lsb_per_dps", 131.0))
            self._accel_bias = [float(v) for v in data["accel_bias_lsb"]]
            self._accel_scale = [float(v) for v in data["accel_scale"]]
            if not all(data.get("axes_solved", [True, True, True])):
                self.get_logger().warn(f"{path}: not every axis was solved")
            self._calibrated = True
            self.get_logger().info(f"accel calibration loaded from {path}")
        except (KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            self.get_logger().error(f"bad calibration file '{path}': {exc}")

    # -- serial --------------------------------------------------------------
    def _open_serial(self) -> bool:
        try:
            # exclusive=True takes an advisory lock, so a second bridge (or a
            # stray detect_aruco.py) fails loudly instead of stealing bytes.
            self._serial = serial.Serial(
                self._port_name, self._baud, timeout=1.0, write_timeout=0.5, exclusive=True
            )
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f"cannot open {self._port_name}: {exc}", throttle_duration_sec=5.0)
            self._serial = None
            self._connected = False
            return False

        # Opening the port resets the Mega; the stream is meaningless until
        # boot and gyro calibration finish.
        self._unwrap.reset()
        self._mcu_clock.reset()
        self.get_logger().info("port open, waiting for READY (boot + gyro calibration)")
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and self._running:
            try:
                line = self._serial.readline().decode("ascii", errors="replace").strip()
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f"read failed while waiting for READY: {exc}")
                return False
            if line and not line.startswith(("IMU,", "TEL,")):
                self.get_logger().info(f"MCU: {line}")
            if line == "READY":
                self._connected = True
                self._send_raw("S")   # known-safe state before anything else
                return True
        self.get_logger().error("no READY from MCU; is the v2.0 firmware flashed?")
        return False

    def _send_raw(self, command: str) -> bool:
        if self._serial is None:
            return False
        try:
            with self._write_lock:
                self._serial.write((command + "\n").encode("ascii"))
            return True
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f"serial write failed: {exc}", throttle_duration_sec=2.0)
            self._connected = False
            return False

    # -- reader thread -------------------------------------------------------
    def _reader_loop(self) -> None:
        while self._running:
            if self._serial is None or not self._connected:
                if not self._open_serial():
                    self._safe_stop()
                    time.sleep(self._reconnect_period)
                    continue
            try:
                raw = self._serial.readline()
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f"serial read failed: {exc}")
                self._connected = False
                self._close_serial()
                continue
            except Exception as exc:                      # noqa: BLE001
                # Never let an unexpected fault leave the motors running.
                self.get_logger().error(f"reader thread fault: {exc}")
                self._safe_stop()
                self._connected = False
                continue

            if not raw:
                continue
            host_s = self.get_clock().now().nanoseconds * 1e-9
            line = raw.decode("ascii", errors="replace").strip()
            if line.startswith("IMU,"):
                self._handle_imu(line, host_s)
            elif line.startswith("TEL,"):
                self._last_telemetry = line
                self._mcu_ok = ",OK:1" in line
                message = String()
                message.data = line
                self._tel_pub.publish(message)
            elif line == "READY":
                # An unexpected READY means the MCU reset under us.
                self.get_logger().warn("MCU reset detected; resynchronising clock")
                self._unwrap.reset()
                self._mcu_clock.reset()
                self._send_raw("S")
            elif line:
                self.get_logger().info(f"MCU: {line}")

    def _handle_imu(self, line: str, host_s: float) -> None:
        parts = line.split(",")
        if len(parts) != 8:
            self._malformed += 1
            return
        try:
            mcu_us = self._unwrap.update(int(parts[1]))
            raw = [int(v) for v in parts[2:8]]
        except ValueError:
            self._malformed += 1
            return

        stamp_s = self._mcu_clock.update(mcu_us, host_s)
        if len(self._latency_window) < 5000:
            self._latency_window.append(self._mcu_clock.last_latency_s)

        message = Imu()
        message.header.stamp.sec = int(stamp_s)
        message.header.stamp.nanosec = int(round((stamp_s - int(stamp_s)) * 1e9))
        message.header.frame_id = self._frame_id

        # MPU6050 has no magnetometer and no fusion, so there is no orientation
        # to report. REP-145: element 0 set to -1 means "not available", which
        # is the correct way to say it instead of inventing an identity
        # quaternion that consumers would trust.
        message.orientation_covariance = [-1.0] + [0.0] * 8

        for i in range(3):
            value = (raw[i] - self._accel_bias[i]) * self._accel_scale[i]
            value = value / self._accel_lsb_per_g * GRAVITY
            if i == 0:
                message.linear_acceleration.x = value
            elif i == 1:
                message.linear_acceleration.y = value
            else:
                message.linear_acceleration.z = value

        message.angular_velocity.x = raw[3] / self._gyro_lsb_per_dps * DEG_TO_RAD
        message.angular_velocity.y = raw[4] / self._gyro_lsb_per_dps * DEG_TO_RAD
        message.angular_velocity.z = raw[5] / self._gyro_lsb_per_dps * DEG_TO_RAD

        message.angular_velocity_covariance = self._gyro_cov
        message.linear_acceleration_covariance = self._accel_cov

        self._imu_pub.publish(message)
        self._sample_count += 1
        self._last_imu_stamp = host_s

    # -- commands ------------------------------------------------------------
    def _on_command(self, message: String) -> None:
        command = message.data.strip()
        checked = validate_command(command, self._max_pwm, self._max_yaw_rate)
        if checked is None:
            self.get_logger().warn(
                f"rejected motor command {command!r} "
                f"(grammar, or outside +/-{self._max_pwm} PWM / "
                f"+/-{self._max_yaw_rate:g} deg/s)"
            )
            return
        command = checked
        if self._invert_drive:
            command = invert_drive(command)
        if self._send_raw(command):
            self._last_command_time = time.monotonic()
            self._stop_sent = command.upper() == "S"

    def _watchdog(self) -> None:
        """Second line of defence behind the MCU's own 350 ms timeout."""
        if self._stop_sent or self._last_command_time == 0.0:
            return
        if time.monotonic() - self._last_command_time > self._command_timeout:
            self.get_logger().warn(
                f"no motor command for {self._command_timeout:.2f}s; sending S"
            )
            self._safe_stop()

    def _safe_stop(self) -> None:
        self._stop_sent = True
        self._send_raw("S")

    # -- diagnostics ---------------------------------------------------------
    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        elapsed = max(now - self._last_rate_time, 1e-6)
        self._last_rate = self._sample_count / elapsed
        self._sample_count = 0
        self._last_rate_time = now

        status = DiagnosticStatus()
        status.name = "follower: serial bridge"
        status.hardware_id = self._port_name

        age = (self.get_clock().now().nanoseconds * 1e-9) - self._last_imu_stamp
        if not self._connected:
            status.level = DiagnosticStatus.ERROR
            status.message = "serial disconnected"
        elif self._last_rate < self._expected_rate * 0.8:
            status.level = DiagnosticStatus.ERROR
            status.message = f"IMU rate {self._last_rate:.0f} Hz below expected"
        elif not self._mcu_ok:
            status.level = DiagnosticStatus.ERROR
            status.message = "MCU reports IMU failure"
        elif not self._calibrated:
            status.level = DiagnosticStatus.WARN
            status.message = "running with uncalibrated accelerometer"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "ok"

        window = self._latency_window
        self._latency_window = []
        if window:
            window.sort()
            latency_p50 = window[len(window) // 2]
            latency_p99 = window[int(len(window) * 0.99)]
            latency_max = window[-1]
        else:
            latency_p50 = latency_p99 = latency_max = 0.0

        offset = self._mcu_clock.offset
        status.values = [
            KeyValue(key="imu_rate_hz", value=f"{self._last_rate:.1f}"),
            KeyValue(key="imu_age_s", value=f"{age:.3f}"),
            KeyValue(key="malformed_lines", value=str(self._malformed)),
            KeyValue(key="clock_offset_s", value="n/a" if offset is None else f"{offset:.6f}"),
            KeyValue(key="clock_skew_ppm", value=f"{self._mcu_clock.skew_ppm:.1f}"),
            KeyValue(key="clock_stamp_clamps", value=str(self._mcu_clock.clamped)),
            KeyValue(key="clock_latency_p50_ms", value=f"{latency_p50 * 1e3:.2f}"),
            KeyValue(key="clock_latency_p99_ms", value=f"{latency_p99 * 1e3:.2f}"),
            KeyValue(key="clock_latency_max_ms", value=f"{latency_max * 1e3:.2f}"),
            KeyValue(key="clock_resets", value=str(self._mcu_clock.resets)),
            KeyValue(key="micros_wraps", value=str(self._unwrap.wraps)),
            KeyValue(key="mcu_reboots", value=str(self._unwrap.reboots)),
            KeyValue(key="accel_calibrated", value=str(self._calibrated)),
            KeyValue(key="mcu_telemetry", value=self._last_telemetry),
        ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diag_pub.publish(array)

    # -- teardown ------------------------------------------------------------
    def _close_serial(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:                              # noqa: BLE001
                pass
            self._serial = None

    def shutdown(self) -> None:
        self._running = False
        self._safe_stop()
        time.sleep(0.1)
        self._close_serial()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Motors must stop no matter how we got here.
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
