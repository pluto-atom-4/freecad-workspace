"""
Spike/SMOKE: emulates float32 numerics only, NOT timing and NOT C3 soft-float cost
(C3 no-FPU claim unverified, BRC#23). Plant is a linearized smoke model.
PID stability is untested (#359).

The C++ port (BRC#25) must mirror the operation order below. The constants below
are what issue #347 reads.

Measured on 2026-10-05, numpy 2.4.6 (max |float32 - float64| over each run; F.4 linear
plant, alternating 16/32 ms, 2600 ticks = 62.4 s unless noted):
  LQR closed loop (theta0 0.02 / 0.05): cmd 5.3e-7, pitch 6.9e-9 rad, gyro 1.2e-7 rad/s
  LQR arithmetic only (f64 states replayed through the f32 core): cmd 1.3e-7
  PID scripted 50 steps: cmd 3.8e-8; PID stress (saturation + integral clamp, 300 ticks):
    cmd 1.1e-7, raw 1.2e-7; PID long (2640 ticks = 63.4 s, small error): cmd 1.5e-6,
    integral term drift 1.5e-6
Tolerances below are ~10x the worst measured value, rounded up. Agreement numbers only:
this says nothing about C3 speed, soft-float cost, sensor noise or hardware behaviour.
"""

import sys
from pathlib import Path
from typing import NamedTuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hal.hal import ImuSample
from hal.control_core import ControlOutput, DEFAULT_PID_KP, DEFAULT_PID_KI, DEFAULT_PID_KD, DEFAULT_OUT_LIMIT, DEFAULT_THETA_REF_RAD, LqrBalance


# Module-level constants for float32 emulation tolerances
F32 = np.float32
F32_TOL_ABS = 6e-6             # #347 reads this as tol_abs (worst measured 5.3e-7)
F32_TOL_PID_LONG_ABS = 2e-5    # PID run >= 60 s (worst measured 1.5e-6)
F32_TOL_STATE_PITCH_ABS = 7e-8   # rad (worst measured 6.9e-9)
F32_TOL_STATE_GYRO_ABS = 2e-6    # rad/s (worst measured 1.2e-7)
REL_FLOOR = 1e-3


class LoopRecord(NamedTuple):
    """One tick's telemetry: time, pitch, gyro y-rate, command, raw command, debug tuple."""
    t_s: float
    pitch_rad: float
    gyro_y_rad_s: float
    cmd: float
    raw: float
    debug: tuple


def quantize_f32(x: float) -> float:
    """Cast a Python float to float32 and back to Python float.

    Emulates the numeric loss when storing in a 32-bit float.
    """
    return float(np.float32(x))


def quantize_imu(sample: ImuSample) -> ImuSample:
    """Quantize IMU sample to float32 precision.

    Each field (t_s, roll, pitch, yaw, gyro components) is cast through F32.
    gyro remains a 3-tuple.
    """
    return ImuSample(
        t_s=quantize_f32(sample.t_s),
        roll_rad=quantize_f32(sample.roll_rad),
        pitch_rad=quantize_f32(sample.pitch_rad),
        yaw_rad=quantize_f32(sample.yaw_rad),
        gyro_rad_s=(
            quantize_f32(sample.gyro_rad_s[0]),
            quantize_f32(sample.gyro_rad_s[1]),
            quantize_f32(sample.gyro_rad_s[2]),
        ),
    )


class Float32PidBalance:
    """Discrete PID controller for pitch balance, using float32 numerics only.

    Emulates operation order and numeric precision of float32 on all state,
    constants, inputs, and intermediate values.

    Note:
        error = -pitch (same as plant_pid.py and control_core.py; sign UNVERIFIED).
        Op order ((kd*(e-prev))/dt ; (p+i)+d ; integral clamp BEFORE ki) is the
        directive the C++ port (BRC#25) must copy.
    """

    def __init__(self, kp=DEFAULT_PID_KP, ki=DEFAULT_PID_KI, kd=DEFAULT_PID_KD,
                 out_limit=DEFAULT_OUT_LIMIT):
        """Initialize PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            out_limit: Output saturation limit (±out_limit)

        Raises:
            ValueError: if out_limit <= 0
        """
        if out_limit <= 0:
            raise ValueError("out_limit must be positive")

        self.kp = F32(kp)
        self.ki = F32(ki)
        self.kd = F32(kd)
        self.out_limit = F32(out_limit)

        self._integral = F32(0.0)
        self._prev = None

    def step(self, imu: ImuSample, dt_s: float) -> ControlOutput:
        """Perform one PID step with float32 quantization.

        Args:
            imu: IMU sample with pitch_rad
            dt_s: Time delta since last call, in seconds

        Returns:
            ControlOutput with clamped command and debug info
        """
        pitch = F32(imu.pitch_rad)
        dt = F32(dt_s)
        e = F32(-pitch)
        L = self.out_limit

        # Handle negative dt: fault condition, no state change
        if float(dt) < 0:
            return ControlOutput(0.0, 0.0, (float(e), 0.0, 0.0, 0.0), True)

        # Handle zero dt: P-only output, no state change
        if float(dt) == 0:
            raw = self.kp * e
            cmd = F32(max(-L, min(L, raw)))
            debug = (float(e), float(raw), 0.0, 0.0)
            return ControlOutput(float(cmd), float(raw), debug, False)

        # Normal case: dt > 0, full PID update
        # P term
        p = self.kp * e

        # I term: accumulate, clamp, then scale
        self._integral = self._integral + e * dt
        self._integral = F32(max(-L, min(L, self._integral)))
        i = self.ki * self._integral

        # D term: derivative on error
        if self._prev is None:
            d = F32(0.0)
        else:
            d = (self.kd * (e - self._prev)) / dt

        # Sum and saturate
        raw = (p + i) + d
        cmd = F32(max(-L, min(L, raw)))

        # Update state for next call
        self._prev = e

        debug = (float(e), float(p), float(i), float(d))
        return ControlOutput(float(cmd), float(raw), debug, False)

    def reset(self) -> None:
        """Reset controller state (integral and previous error)."""
        self._integral = F32(0.0)
        self._prev = None


class Float32LqrBalance:
    """LQR state feedback controller for pitch and pitch-rate, using float32 numerics.

    Stateless; dt is ignored. dt < 0 still returns fault=True.
    """

    def __init__(self, K=None, theta_ref_rad=DEFAULT_THETA_REF_RAD,
                 out_limit=DEFAULT_OUT_LIMIT):
        """Initialize LQR controller.

        Args:
            K: Gain vector [k0, k1] for [theta, theta_dot]. If None, use default.
            theta_ref_rad: Reference pitch angle in radians
            out_limit: Output saturation limit (±out_limit)

        Raises:
            ValueError: if out_limit <= 0 or len(K) != 2 (if K is provided)
        """
        if out_limit <= 0:
            raise ValueError("out_limit must be positive")

        if K is None:
            K = LqrBalance().K

        if len(K) != 2:
            raise ValueError("K must be a sequence of length 2")

        self.K0 = F32(K[0])
        self.K1 = F32(K[1])
        self.theta_ref = F32(theta_ref_rad)
        self.out_limit = F32(out_limit)

    def step(self, imu: ImuSample, dt_s: float) -> ControlOutput:
        """Perform one LQR step with float32 quantization.

        Args:
            imu: IMU sample with pitch_rad and gyro_rad_s
            dt_s: Time delta (ignored for LQR; present for interface compatibility)

        Returns:
            ControlOutput with clamped command and debug info (theta, theta_dot)
        """
        theta = F32(imu.pitch_rad) - self.theta_ref
        td = F32(imu.gyro_rad_s[1])

        # Fault condition: dt < 0
        if float(dt_s) < 0:
            return ControlOutput(0.0, 0.0, (float(theta), float(td)), True)

        L = self.out_limit
        raw = -(self.K0 * theta + self.K1 * td)
        cmd = F32(max(-L, min(L, raw)))

        return ControlOutput(float(cmd), float(raw), (float(theta), float(td)), False)

    def reset(self) -> None:
        """No-op; LQR is stateless."""
        pass


def run_closed_loop(core, hal, n_ticks) -> list[LoopRecord]:
    """Run a closed-loop control simulation for n_ticks.

    Args:
        core: Control core object (Float32PidBalance or Float32LqrBalance)
        hal: HAL backend (e.g., FakeHal) that provides wait_next_tick(), read_imu(), write_wheel_velocity()
        n_ticks: Maximum number of control ticks to run

    Returns:
        List of LoopRecord telemetry, one per tick (may be shorter if hal signals stop)
    """
    records = []
    for _ in range(n_ticks):
        dt = hal.wait_next_tick()
        if dt < 0:
            break
        imu = hal.read_imu()
        out = core.step(imu, dt)
        hal.write_wheel_velocity(out.cmd_rad_s, out.cmd_rad_s)
        records.append(LoopRecord(imu.t_s, imu.pitch_rad, imu.gyro_rad_s[1],
                                  out.cmd_rad_s, out.raw_cmd, out.debug))
    return records


def compare_runs(ref, f32) -> dict:
    """Compare two runs (reference vs float32) and return statistics.

    Args:
        ref: List of LoopRecord from reference (Python float) run
        f32: List of LoopRecord from float32 emulation run

    Returns:
        Dict with keys: max_abs_cmd, max_abs_raw, max_rel_cmd, max_abs_pitch,
        max_abs_gyro, max_abs_i_term, n. Empty lists give all zeros with n=0.

    Raises:
        ValueError: If lengths differ.
    """
    if len(ref) != len(f32):
        raise ValueError(f"Run length mismatch: ref={len(ref)}, f32={len(f32)}")

    if len(ref) == 0:
        return {
            "max_abs_cmd": 0.0,
            "max_abs_raw": 0.0,
            "max_rel_cmd": 0.0,
            "max_abs_pitch": 0.0,
            "max_abs_gyro": 0.0,
            "max_abs_i_term": 0.0,
            "n": 0,
        }

    max_abs_cmd = 0.0
    max_abs_raw = 0.0
    max_rel_cmd = 0.0
    max_abs_pitch = 0.0
    max_abs_gyro = 0.0
    max_abs_i_term = 0.0

    for r_rec, f_rec in zip(ref, f32):
        # Command comparison
        abs_cmd_diff = abs(r_rec.cmd - f_rec.cmd)
        max_abs_cmd = max(max_abs_cmd, abs_cmd_diff)

        # Raw command comparison
        abs_raw_diff = abs(r_rec.raw - f_rec.raw)
        max_abs_raw = max(max_abs_raw, abs_raw_diff)

        # Relative command (only if reference cmd is significant)
        if r_rec.cmd != 0:
            rel_cmd = abs_cmd_diff / max(abs(r_rec.cmd), REL_FLOOR)
            max_rel_cmd = max(max_rel_cmd, rel_cmd)

        # Pitch comparison
        abs_pitch_diff = abs(r_rec.pitch_rad - f_rec.pitch_rad)
        max_abs_pitch = max(max_abs_pitch, abs_pitch_diff)

        # Gyro (Y rate) comparison
        abs_gyro_diff = abs(r_rec.gyro_y_rad_s - f_rec.gyro_y_rad_s)
        max_abs_gyro = max(max_abs_gyro, abs_gyro_diff)

        # I-term comparison (only if both debug tuples have len 4)
        if len(r_rec.debug) == 4 and len(f_rec.debug) == 4:
            abs_i_diff = abs(r_rec.debug[2] - f_rec.debug[2])
            max_abs_i_term = max(max_abs_i_term, abs_i_diff)

    return {
        "max_abs_cmd": max_abs_cmd,
        "max_abs_raw": max_abs_raw,
        "max_rel_cmd": max_rel_cmd,
        "max_abs_pitch": max_abs_pitch,
        "max_abs_gyro": max_abs_gyro,
        "max_abs_i_term": max_abs_i_term,
        "n": len(ref),
    }


def replay_imu(core, ref, periods_s=(0.016, 0.032)) -> list[LoopRecord]:
    """Replay a reference run's IMU data through a fresh core instance.

    Lazily imports FakeHal to avoid circular dependencies.

    Args:
        core: Control core object (Float32PidBalance or Float32LqrBalance)
        ref: List of LoopRecord from a reference run
        periods_s: Tuple of control tick periods (seconds) to cycle

    Returns:
        List of LoopRecord from replaying ref's IMU through core
    """
    from hal.fake_hal import FakeHal

    hal = FakeHal(
        tick_periods_s=periods_s,
        imu_script=[
            ImuSample(r.t_s, 0.0, r.pitch_rad, 0.0, (0.0, r.gyro_y_rad_s, 0.0))
            for r in ref
        ],
        max_ticks=len(ref),
    )
    return run_closed_loop(core, hal, len(ref))


__all__ = [
    "F32",
    "F32_TOL_ABS",
    "F32_TOL_PID_LONG_ABS",
    "F32_TOL_STATE_PITCH_ABS",
    "F32_TOL_STATE_GYRO_ABS",
    "REL_FLOOR",
    "LoopRecord",
    "quantize_f32",
    "quantize_imu",
    "Float32PidBalance",
    "Float32LqrBalance",
    "run_closed_loop",
    "compare_runs",
    "replay_imu",
]
