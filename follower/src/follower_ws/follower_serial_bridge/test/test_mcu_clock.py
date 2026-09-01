"""Tests for MCU clock recovery.

Run standalone (no ROS needed):
    python3 src/follower_serial_bridge/test/test_mcu_clock.py

The numbers here are not arbitrary: 928 ppm is the skew measured on this
robot's Mega 2560 against the Pi 5, and the chunked delivery models USB CDC
handing several IMU lines to the host at once.
"""

from __future__ import annotations

import random
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "follower_serial_bridge"))

from mcu_clock import MICROS_WRAP, McuClockSync, MicrosUnwrapper  # noqa: E402

MEASURED_SKEW_PPM = 928.0


def test_wrap() -> None:
    unwrap = MicrosUnwrapper()
    values = [MICROS_WRAP - 3000, MICROS_WRAP - 1000, 1000, 3000]
    out = [unwrap.update(v) for v in values]
    assert [b - a for a, b in zip(out, out[1:])] == [2000, 2000, 2000]
    assert unwrap.wraps == 1 and unwrap.reboots == 0


def test_reboot_is_not_a_wrap() -> None:
    """A mid-range backwards jump is a reset, not a 71.6 minute wrap."""
    unwrap = MicrosUnwrapper()
    for value in (1_000_000, 1_005_000, 1_010_000):
        unwrap.update(value)
    assert unwrap.update(4000) == 4000
    assert unwrap.reboots == 1 and unwrap.wraps == 0


def _simulate(skew_ppm: float, seconds: float = 300.0, chunk: int = 5):
    random.seed(11)
    sync = McuClockSync()
    offset = 1787591000.0
    errors: list[float] = []
    pending: list[tuple[int, float]] = []
    for i in range(int(seconds * 200)):
        mcu_us = i * 5000
        true_host = mcu_us * 1e-6 * (1 + skew_ppm * 1e-6) + offset
        pending.append((mcu_us, true_host))
        if len(pending) >= chunk:
            # USB CDC hands the whole chunk over at once.
            delivery = pending[-1][1] + 0.0012 + random.expovariate(1 / 0.0004)
            for micros, truth in pending:
                stamp = sync.update(micros, delivery)
                if i > 200 * 70:            # after the fit has converged
                    errors.append(stamp - truth)
            pending = []
    drift = statistics.fmean(errors[-2000:]) - statistics.fmean(errors[:2000])
    return statistics.pstdev(errors), drift, sync


def test_skew_recovery() -> None:
    """The absolute offset is unobservable; the SLOPE and the JITTER are not.

    A constant lag is absorbed by camera-IMU time calibration, so what has to
    be true is that the error does not drift and does not jitter.

    Thresholds come from what the consumer tolerates, not from what this
    implementation happens to score: a monocular VIO wants camera-IMU time
    agreement inside roughly 1 ms, so 200 us of drift and 150 us of jitter
    leave a factor of five in hand. At the skew actually measured on this
    robot (928 ppm) the drift is under 1 us; 4500 ppm is the datasheet worst
    case for a ceramic resonator and still lands near 60 us.
    """
    for skew in (MEASURED_SKEW_PPM, -1500.0, 0.0, 4500.0):
        sd, drift, sync = _simulate(skew)
        assert sd < 150e-6, f"{skew} ppm: jitter sd {sd * 1e6:.0f} us"
        assert abs(drift) < 200e-6, f"{skew} ppm: drift {drift * 1e6:.0f} us"
        assert abs(sync.skew_ppm - skew) < 30, f"{skew} ppm: estimated {sync.skew_ppm:.1f}"
        assert sync.clamped == 0, f"{skew} ppm: {sync.clamped} clamped stamps"


def test_host_clock_step_resets() -> None:
    sync = McuClockSync()
    for i in range(2000):
        sync.update(i * 5000, i * 5000 * 1e-6 + 100.0 + 0.0005)
    before = sync.resets
    for i in range(2000, 4000):
        sync.update(i * 5000, i * 5000 * 1e-6 + 105.0 + 0.0005)
    assert sync.resets == before + 1


if __name__ == "__main__":
    for name, function in sorted(globals().items()):
        if name.startswith("test_"):
            function()
            print(f"PASS  {name}")
    print("\nall mcu_clock tests passed")
