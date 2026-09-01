"""Recovering ROS time from the Arduino's free-running micros() clock.

THE PROBLEM
-----------
Four clocks exist in this system (spec section 10): Arduino micros(), the
Raspberry Pi clock, the webcam timestamp, and the main robot's ROS clock.
A monocular VIO is very sensitive to camera/IMU time consistency, so the IMU
stamp must describe when the sample was MEASURED, not when the host happened
to finish reading a serial line.

Using the host receive time directly is wrong by a variable amount: USB CDC
batches bytes, the kernel wakes the reader when it feels like it, and Python
adds its own scheduling delay. That noise would be injected straight into the
VIO as timestamp error.

THE MODEL
---------
    t_ros_receive = t_mcu + offset + latency,     latency >= 0

`offset` is what we want. Latency is strictly non-negative and one-sided, so
the MINIMUM observed value of (t_ros_receive - t_mcu) over a window is the
best available estimate of `offset`: it is the sample that suffered the least
delay. This is the same principle NTP uses when it keeps the lowest-round-trip
sample of a burst, and it needs no model of the delay distribution.

The two clocks also run at different RATES. Measured on this robot, the
Arduino's clock loses 928 us every second against the Pi -- 928 ppm. That is
entirely normal: the Mega 2560 is clocked by a 16 MHz ceramic resonator
(+/-0.5% = 5000 ppm typical) while the Pi runs off a crystal. Ignoring it is
not an option: 928 ppm is 56 ms of drift per minute, which would drag the IMU
timeline away from the camera timeline without bound.

So `offset` is not a constant to be tracked, it is a LINE:

    offset(t_mcu) = a + b * t_mcu          b = relative clock skew

Estimating it:

  1. Time is cut into windows of `window_s`. Within each window the MINIMUM
     observed (t_ros - t_mcu) is kept. With ~200 samples per window that
     minimum sits very close to the true lower envelope.
  2. A least-squares line is fitted through the last `history` window minima.
     Each minimum carries roughly the same small positive bias, so the bias
     cancels in the SLOPE, which is the quantity that actually matters.
  3. The fitted line gives the offset at any MCU time.

Fitting minima rather than raw samples is what makes this robust: transport
latency is one-sided, so raw samples are all above the truth by a variable
amount, while their per-window minima are all above it by almost the same
amount. `max_skew_ppm` now only clamps the fitted slope against nonsense.

WRAP-AROUND vs REBOOT
---------------------
micros() wraps every 2**32 us (~71.6 min). A reset also sends it back to ~0.
The two are told apart by where it came FROM: a wrap can only happen from the
very top of the range. A reboot is additionally announced by the firmware's
READY line, which callers should forward to `reset()`.
"""

from __future__ import annotations

MICROS_WRAP = 1 << 32
_WRAP_HIGH = int(MICROS_WRAP * 0.90)
_WRAP_LOW = int(MICROS_WRAP * 0.10)


class MicrosUnwrapper:
    """uint32 micros() -> monotonic microseconds."""

    def __init__(self) -> None:
        self._previous: int | None = None
        self._epoch = 0
        self.wraps = 0
        self.reboots = 0

    def reset(self) -> None:
        self._previous = None
        self._epoch = 0

    def update(self, raw: int) -> int:
        if self._previous is not None and raw < self._previous:
            if self._previous >= _WRAP_HIGH and raw <= _WRAP_LOW:
                # Genuine 2**32 wrap.
                self._epoch += 1
                self.wraps += 1
            else:
                # Went backwards from somewhere in the middle: the MCU reset.
                # Keeping the old epoch would silently add 71.6 minutes.
                self.reboots += 1
                self._epoch = 0
                self._previous = raw
                return raw
        self._previous = raw
        return raw + self._epoch * MICROS_WRAP


class McuClockSync:
    """Maps unwrapped MCU microseconds onto the host clock.

    Args:
        window_s: length of a minimum-collection window, in MCU seconds.
        history: how many window minima to fit. window_s * history is the
            memory of the skew estimate; 1 s x 60 tracks a resonator whose
            drift changes with temperature without being noisy.
        max_skew_ppm: hard clamp on the fitted slope. A ceramic resonator is
            specified to +/-5000 ppm, so anything beyond that is a bug, not a
            clock.
        reset_threshold_s: a residual this large means a clock was stepped
            (NTP) or the MCU reset, not that a packet was late.
        overshoot_tolerance_s: how far a stamp may land after its own receive
            time before it is treated as a fault instead of fit error.
    """

    def __init__(
        self,
        window_s: float = 1.0,
        history: int = 60,
        max_skew_ppm: float = 5000.0,
        reset_threshold_s: float = 2.0,
        overshoot_tolerance_s: float = 0.05,
    ) -> None:
        self._window_s = window_s
        self._history = history
        self._max_skew = max_skew_ppm * 1e-6
        self._reset_threshold = reset_threshold_s
        self._overshoot_tolerance_s = overshoot_tolerance_s
        # Cumulative diagnostics live outside the estimator state so that
        # resetting the fit does not erase the record of why it was reset.
        self.resets = 0
        self.clamped = 0
        self.last_latency_s = 0.0
        self.reset()

    def reset(self) -> None:
        """Discard the fit. Call this when the MCU reboots."""
        self._minima: list[tuple[float, float]] = []   # (mcu_s, min offset)
        self._window_start: float | None = None
        self._window_min: float | None = None
        self._window_min_t = 0.0
        self._intercept: float | None = None           # offset at _t_ref
        self._slope = 0.0
        self._t_ref = 0.0

    @property
    def offset(self) -> float | None:
        """Offset at the most recent sample, for diagnostics."""
        return self._intercept

    @property
    def skew_ppm(self) -> float:
        return self._slope * 1e6

    def _predict(self, mcu_s: float) -> float | None:
        if self._intercept is None:
            return None
        return self._intercept + self._slope * (mcu_s - self._t_ref)

    def _refit(self) -> None:
        points = self._minima
        if len(points) < 3:
            # Not enough spread to fit a slope; hold the latest minimum.
            self._t_ref, self._intercept = points[-1]
            self._slope = 0.0
            return
        t_ref = points[len(points) // 2][0]
        mean_t = sum(t - t_ref for t, _ in points) / len(points)
        mean_o = sum(o for _, o in points) / len(points)
        num = sum((t - t_ref - mean_t) * (o - mean_o) for t, o in points)
        den = sum((t - t_ref - mean_t) ** 2 for t, _ in points)
        slope = num / den if den > 0 else 0.0
        slope = max(-self._max_skew, min(self._max_skew, slope))
        self._slope = slope
        self._t_ref = t_ref
        # Fitted offset at t_ref (mean_t is measured relative to t_ref).
        self._intercept = mean_o - slope * mean_t

    def update(self, mcu_us: int, host_s: float) -> float:
        """Return the host-clock timestamp for an MCU measurement."""
        mcu_s = mcu_us * 1e-6
        observed = host_s - mcu_s

        predicted = self._predict(mcu_s)
        if predicted is not None and abs(observed - predicted) > self._reset_threshold:
            # A clock was stepped, or the MCU reset without us noticing.
            self.resets += 1
            self.reset()
            predicted = None

        # ---- accumulate the per-window minimum ----------------------------
        if self._window_start is None:
            self._window_start = mcu_s
            self._window_min = observed
            self._window_min_t = mcu_s
        elif observed < self._window_min:
            self._window_min = observed
            self._window_min_t = mcu_s

        if mcu_s - self._window_start >= self._window_s:
            self._minima.append((self._window_min_t, self._window_min))
            if len(self._minima) > self._history:
                self._minima.pop(0)
            self._refit()
            self._window_start = mcu_s
            self._window_min = observed
            self._window_min_t = mcu_s

        predicted = self._predict(mcu_s)
        if predicted is None:
            # First window: nothing fitted yet, use the running minimum.
            predicted = self._window_min if self._window_min is not None else observed

        self.last_latency_s = observed - predicted
        stamp = mcu_s + predicted

        # A stamp slightly after its own receive time is physically impossible,
        # but forcing it back to host_s would re-inject exactly the receive
        # jitter this class exists to remove -- and the fitted line sits within
        # a few hundred microseconds of the envelope, so small overshoots are
        # estimation error, not a fault. Only a gross violation is clamped.
        if stamp > host_s + self._overshoot_tolerance_s:
            self.clamped += 1
            return host_s
        return stamp
