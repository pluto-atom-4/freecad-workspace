#!/usr/bin/env python3
"""SerialImuHal: a Stage F Hal (inverted-pendulum-project/07_Simulation/hal/hal.py, contract v1)
fed by ESP32 IMU frame lines (dof_frame.py format).

MOCK-TESTED ONLY. PITCH_SIGN, WHEEL_DISPLAY_SIGN and the arm/stale thresholds are UNVERIFIED on
hardware; the human check is #377. Slow tilt (< 5 deg/s over the 1 s arm window) can be absorbed
into the bias and mis-tare by up to about 5 degrees.

Input: an iterable of str lines. A None item is a poll tick (no line available yet).
  - non-str items are skipped;
  - lines not starting with "IMU," are skipped and NOT counted (junk-prefixed frames are ignored
    by design);
  - a line starting with "IMU," that fails dof_frame.parse_line is counted in bad_frames;
  - every parsed frame goes through StreamStats.update; duplicates / backward seq are skipped
    (no tick); a device reset (StreamStats.resets grows) forces filter.reset() and a re-arm.

Arming (one window of ARM_FRAMES accepted frames, collected inside wait_next_tick() or arm()):
  - a device gap > stale_s inside the window restarts the window (attempt fails);
  - any gyro axis with span (max - min) > arm_span_dps (strict) fails the attempt ("moving");
  - source exhausted before the window is full: arm_error "window incomplete: n/N", wait returns -1.0;
  - on success: bias = mean gyro per axis; the window is replayed through ComplementaryFilter with
    bias-corrected gyro; tare = mean fused pitch over the replay. The sample is the last window frame
    at t_s = 0.0 and the first wait_next_tick after arming returns NOMINAL_DT_S without consuming
    a frame. A failed attempt returns NOMINAL_DT_S; the next wait_next_tick keeps collecting.
  - arm() raises HalFault on a failed attempt or an exhausted source. A poll tick during arm()
    raises HalFault (retry arm()); it never spins.

Per tick after arming (one accepted frame):
  - dt = StreamStats.last_dt_us / 1e6 (modular uint32 handled by StreamStats); a reset frame
    returns NOMINAL_DT_S and re-arms;
  - a device gap dt > stale_s: filter.reset() (re-initialised from this frame), bias and tare kept,
    stale_count += 1, read_imu raises HalFault for that tick only, the tick returns NOMINAL_DT_S;
  - a poll tick (None) with clock() - last accepted frame wall time > stale_s sets a wall-stale
    flag (stale_count += 1 on entry) that read_imu reports until the next accepted frame; the tick
    returns NOMINAL_DT_S so the loop keeps running; a plain poll tick repeats the last sample.
  - t_s advances by the real device dt of each accepted frame (NOMINAL_DT_S if unknown), so it is
    monotonic across t_us wrap and resets (a re-arm after a reset adds NOMINAL_DT_S).

read_imu(): roll_rad = fused roll (not tared); pitch_rad = pitch_sign * (fused_pitch - tare) +
reference_pitch_rad; yaw_rad = 0.0; gyro_rad_s = (rad(gx-bx), pitch_sign*rad(gy-by), rad(gz-bz)).
Raises HalFault when closed, unarmed (raises arm_error), or stale.

read_encoders(): always HalFault (no encoder measurement; none is faked).

write_wheel_velocity(left, right): each input clamped to +-WHEEL_CMD_LIMIT_RAD_S; non-numeric or
non-finite -> 0.0; never raises. Integrates wheel_display_sign * clamped_left * last_returned_dt into
wheel_angle_rad. LEFT ONLY (the right command is stored but not integrated); call once per tick,
after wait_next_tick(). After close() writes are no-ops and wait_next_tick() returns -1.0.

Imports: stdlib, hal.hal, dof_frame, dof_fusion, dof_stats only (no pyserial, numpy, scipy,
controller, dof_serial, dof_monitor, dof_publisher).
"""

import math
import numbers
import sys
import time
from collections import namedtuple
from pathlib import Path

from dof_frame import PREFIX, FrameError, parse_line
from dof_fusion import ComplementaryFilter
from dof_stats import StreamStats

_SIM_DIR = Path(__file__).resolve().parents[3] / "inverted-pendulum-project" / "07_Simulation"
if str(_SIM_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_DIR))

from hal.hal import HAL_CONTRACT_VERSION, Hal, HalFault, ImuSample  # noqa: E402

_FRAME_START = PREFIX + ","

PITCH_SIGN = 1.0            # UNVERIFIED on hardware
WHEEL_DISPLAY_SIGN = 1.0    # UNVERIFIED on hardware (GUI check, #377)
NOMINAL_DT_S = 0.02
ARM_FRAMES = 50
ARM_SPAN_DPS = 5.0
STALE_S = 0.2
WHEEL_CMD_LIMIT_RAD_S = 1.0

_GYRO_AXES = ("x", "y", "z")
_Entry = namedtuple("_Entry", "frame dt_us reset")


def _is_real(v):
    return isinstance(v, numbers.Real) and not isinstance(v, bool)


def _clamp_cmd(v):
    """Clamp a wheel command to +-WHEEL_CMD_LIMIT_RAD_S; non-numeric/non-finite -> 0.0."""
    try:
        if not _is_real(v):
            return 0.0
        x = float(v)
    except (TypeError, ValueError, OverflowError):
        return 0.0
    if not math.isfinite(x):
        return 0.0
    return max(-WHEEL_CMD_LIMIT_RAD_S, min(WHEEL_CMD_LIMIT_RAD_S, x))


class SerialImuHal(Hal):
    """Stage F Hal backed by an iterable of IMU frame lines. See module docstring for semantics."""

    CONTRACT_VERSION = HAL_CONTRACT_VERSION

    def __init__(self, lines, reference_pitch_rad=0.0, *, alpha=0.98, arm_frames=ARM_FRAMES,
                 arm_span_dps=ARM_SPAN_DPS, stale_s=STALE_S, pitch_sign=PITCH_SIGN,
                 wheel_display_sign=WHEEL_DISPLAY_SIGN, clock=time.monotonic):
        if not (_is_real(reference_pitch_rad) and math.isfinite(reference_pitch_rad)):
            raise ValueError(f"reference_pitch_rad must be finite, got {reference_pitch_rad!r}")
        if isinstance(arm_frames, bool) or not isinstance(arm_frames, int) or arm_frames < 2:
            raise ValueError(f"arm_frames must be int >= 2, got {arm_frames!r}")
        if not (_is_real(arm_span_dps) and math.isfinite(arm_span_dps) and arm_span_dps >= 0):
            raise ValueError(f"arm_span_dps must be finite >= 0, got {arm_span_dps!r}")
        if not (_is_real(stale_s) and math.isfinite(stale_s) and stale_s > 0):
            raise ValueError(f"stale_s must be finite > 0, got {stale_s!r}")
        if pitch_sign not in (1.0, -1.0) or not _is_real(pitch_sign):
            raise ValueError(f"pitch_sign must be +1.0 or -1.0, got {pitch_sign!r}")
        if wheel_display_sign not in (1.0, -1.0) or not _is_real(wheel_display_sign):
            raise ValueError(f"wheel_display_sign must be +1.0 or -1.0, got {wheel_display_sign!r}")
        if not callable(clock):
            raise ValueError("clock must be callable")
        try:
            src = iter(lines)
        except TypeError as exc:
            raise ValueError(f"lines must be iterable: {exc}") from exc

        self._src = src
        self._reference = float(reference_pitch_rad)
        self._arm_frames = arm_frames
        self._arm_span = float(arm_span_dps)
        self._stale_s = float(stale_s)
        self._pitch_sign = float(pitch_sign)
        self._wheel_sign = float(wheel_display_sign)
        self._clock = clock

        self._filter = ComplementaryFilter(alpha=alpha)
        self._stats = StreamStats()
        self._last_seq = None
        self._last_wall = None

        self._armed = False
        self._has_armed = False
        self._arm_error = None
        self._win = []
        self._finished = False
        self._hold_first = False
        self._bias = (0.0, 0.0, 0.0)
        self._tare = 0.0
        self._cur = None           # _Entry of the current sample
        self._fused = (0.0, 0.0)   # (roll, pitch) of the current sample, radians
        self._dev_stale = False
        self._wall_stale = False
        self._stale_count = 0
        self._rearm_count = 0
        self._t_dev = 0.0
        self._closed = False
        self._last_dt = 0.0
        self._wheel_left = 0.0
        self._wheel_right = 0.0
        self._wheel_angle = 0.0

    # ----- read-only properties -------------------------------------------------------------

    @property
    def armed(self):
        return self._armed

    @property
    def arm_error(self):
        return self._arm_error

    @property
    def tared_pitch_rad(self):
        if not self._armed:
            return 0.0
        return self._pitch_sign * (self._fused[1] - self._tare)

    @property
    def wheel_angle_rad(self):
        return self._wheel_angle

    @property
    def stats(self):
        return self._stats

    @property
    def bad_frames(self):
        return self._stats.bad_frames

    @property
    def stale_count(self):
        return self._stale_count

    @property
    def rearm_count(self):
        return self._rearm_count

    @property
    def last_seq(self):
        return self._last_seq

    # ----- Hal interface --------------------------------------------------------------------

    def arm(self):
        """Explicit arming attempt. Raises HalFault if the attempt fails or the source is exhausted."""
        if self._closed:
            raise HalFault("SerialImuHal is closed")
        if self._armed:
            return
        status = self._arm_attempt()
        if status == "armed":
            self._hold_first = True   # explicit arm(): the next wait returns nominal without a frame
            return
        if status == "poll":
            raise HalFault("arm window incomplete: no frame yet, retry arm()")
        raise HalFault(self._arm_error or "arm failed")

    def read_imu(self):
        if self._closed:
            raise HalFault("SerialImuHal is closed")
        if not self._armed:
            raise HalFault(self._arm_error or "not armed: call wait_next_tick() or arm() first")
        if self._dev_stale:
            raise HalFault("stale IMU: device frame gap")
        if self._wall_stale:
            raise HalFault("stale IMU: no frame within stale_s")
        roll, pitch = self._fused
        bx, by, bz = self._bias
        gx, gy, gz = self._cur.frame.gyro
        return ImuSample(
            t_s=self._t_dev,
            roll_rad=roll,
            pitch_rad=self._pitch_sign * (pitch - self._tare) + self._reference,
            yaw_rad=0.0,
            gyro_rad_s=(math.radians(gx - bx),
                        self._pitch_sign * math.radians(gy - by),
                        math.radians(gz - bz)),
        )

    def read_encoders(self):
        raise HalFault("no encoders: SerialImuHal has no wheel measurement")

    def write_wheel_velocity(self, left_rad_s, right_rad_s):
        if self._closed:
            return
        self._wheel_left = _clamp_cmd(left_rad_s)
        self._wheel_right = _clamp_cmd(right_rad_s)
        self._wheel_angle += self._wheel_sign * self._wheel_left * self._last_dt

    def now_s(self):
        return self._t_dev

    def wait_next_tick(self):
        if self._closed or self._finished:
            self._last_dt = 0.0
            return -1.0
        self._dev_stale = False
        dt = self._tick()
        self._last_dt = dt if dt > 0 else 0.0
        return dt

    def close(self):
        if self._closed:
            return
        self._closed = True
        close = getattr(self._src, "close", None)
        if callable(close):
            try:
                close()
            except Exception:
                pass

    # ----- internals ------------------------------------------------------------------------

    def _tick(self):
        if not self._armed:
            status = self._arm_attempt()
            return -1.0 if status == "end" else NOMINAL_DT_S
        if self._hold_first:
            self._hold_first = False
            return NOMINAL_DT_S
        kind, entry = self._pull()
        if kind == "end":
            self._finished = True
            return -1.0
        if kind == "poll":
            if (self._last_wall is not None
                    and self._clock() - self._last_wall > self._stale_s):
                if not self._wall_stale:
                    self._stale_count += 1
                self._wall_stale = True
            return NOMINAL_DT_S
        if entry.reset:
            self._reset_state()
            self._win = [entry]
            status = self._arm_attempt()
            return -1.0 if status == "end" else NOMINAL_DT_S
        return self._run_frame(entry)

    def _run_frame(self, entry):
        """Armed: fuse one accepted frame and return its dt (NOMINAL when stale)."""
        dt = entry.dt_us / 1e6 if entry.dt_us is not None else NOMINAL_DT_S
        self._t_dev += dt
        self._cur = entry
        accel = entry.frame.accel
        gyro = self._corrected_gyro(entry.frame.gyro)
        if dt > self._stale_s:
            self._stale_count += 1
            self._dev_stale = True
            self._filter.reset()
            roll, pitch, _ = self._filter.update(accel, gyro, 0.0)
            self._fused = (roll, pitch)
            return NOMINAL_DT_S
        roll, pitch, _ = self._filter.update(accel, gyro, dt)
        self._fused = (roll, pitch)
        return dt

    def _corrected_gyro(self, gyro):
        bx, by, bz = self._bias
        return (gyro[0] - bx, gyro[1] - by, gyro[2] - bz)

    def _reset_state(self):
        self._filter.reset()
        self._armed = False
        self._hold_first = False
        self._win = []
        self._cur = None
        self._fused = (0.0, 0.0)
        self._dev_stale = False
        self._wall_stale = False
        self._arm_error = None
        self._rearm_count += 1

    def _arm_attempt(self):
        """Consume frames into the arm window. Returns 'armed', 'failed', 'poll' or 'end'."""
        while True:
            kind, entry = self._pull()
            if kind == "end":
                self._finished = True
                self._arm_error = f"window incomplete: {len(self._win)}/{self._arm_frames}"
                return "end"
            if kind == "poll":
                return "poll"
            if entry.reset:
                self._reset_state()
                self._win = [entry]
                continue
            if self._win and entry.dt_us is not None and entry.dt_us / 1e6 > self._stale_s:
                self._arm_error = (f"window gap: device gap {entry.dt_us / 1e6:.3f} s > "
                                   f"{self._stale_s:g} s inside arm window")
                self._win = [entry]
                return "failed"
            self._win.append(entry)
            if len(self._win) >= self._arm_frames:
                return self._finish_window()

    def _finish_window(self):
        win = self._win
        self._win = []
        n = len(win)
        gyros = [e.frame.gyro for e in win]
        for axis, name in enumerate(_GYRO_AXES):
            vals = [g[axis] for g in gyros]
            span = max(vals) - min(vals)
            if span > self._arm_span:
                self._arm_error = (f"board moving: gyro {name} span {span:.2f} dps > "
                                   f"{self._arm_span:g} dps")
                return "failed"
        bias = tuple(sum(g[i] for g in gyros) / n for i in range(3))
        self._bias = bias
        self._filter.reset()
        pitches = []
        roll = pitch = 0.0
        for i, e in enumerate(win):
            dt = 0.0 if i == 0 else (e.dt_us / 1e6 if e.dt_us is not None else NOMINAL_DT_S)
            roll, pitch, _ = self._filter.update(e.frame.accel, self._corrected_gyro(e.frame.gyro), dt)
            pitches.append(pitch)
        self._tare = sum(pitches) / n
        self._fused = (roll, pitch)
        self._cur = win[-1]
        if self._has_armed:
            self._t_dev += NOMINAL_DT_S          # re-arm after reset keeps t_s monotonic
        self._has_armed = True
        self._armed = True
        self._arm_error = None
        self._dev_stale = False
        self._wall_stale = False
        return "armed"

    def _pull(self):
        """Read items until one accepted frame, a poll tick, or the end of the source."""
        while True:
            try:
                item = next(self._src)
            except StopIteration:
                return ("end", None)
            except Exception as exc:  # source failure ends the stream; never escapes wait_next_tick
                if not self._armed:
                    self._arm_error = f"source error: {exc!r}"
                return ("end", None)
            if item is None:
                return ("poll", None)
            if not isinstance(item, str) or not item.startswith(_FRAME_START):
                continue
            try:
                frame = parse_line(item)
            except FrameError:
                self._stats.note_bad()
                continue
            resets_before = self._stats.resets
            if not self._stats.update(frame.seq, frame.t_us):
                continue
            self._last_seq = frame.seq
            self._last_wall = self._clock()
            self._wall_stale = False
            reset = self._stats.resets != resets_before
            return ("frame", _Entry(frame, self._stats.last_dt_us, reset))


__all__ = ["SerialImuHal", "PITCH_SIGN", "WHEEL_DISPLAY_SIGN", "NOMINAL_DT_S", "ARM_FRAMES",
           "ARM_SPAN_DPS", "STALE_S", "WHEEL_CMD_LIMIT_RAD_S"]
