"""
Core control algorithms for the inverted pendulum: PID balance and LQR state feedback.

Pure Python, no Webots dependencies. Caller supplies measured dt per call.
HAL contract (hal.py): dt < 0 → output 0 and fault=True; dt==0 → P-only (PID) or normal (LQR).
"""

import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from plant_pid import PlantPID
from hal.hal import ImuSample


# Module constants
DEFAULT_PID_KP = 1.0
DEFAULT_PID_KI = 0.1
DEFAULT_PID_KD = 0.05
DEFAULT_OUT_LIMIT = 1.0
DEFAULT_THETA_REF_RAD = -0.1086


@dataclass(frozen=True)
class ControlOutput:
    """Control output from PID or LQR controller.

    fault=True means dt<0; cmd and raw are 0.0; internal state is not touched (HAL contract, hal.py).
    """
    cmd_rad_s: float
    raw_cmd: float
    debug: tuple
    fault: bool = False


class PidBalance:
    """Discrete PID controller for pitch balance.

    dt==0 is P-only with no state update; the core clamps that branch (PlantPID does not);
    first-call derivative is 0 as in PlantPID.
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

        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.out_limit = float(out_limit)

        self._pid = PlantPID(self.kp, self.ki, self.kd, -self.out_limit, self.out_limit)

    def step(self, imu: ImuSample, dt_s: float) -> ControlOutput:
        """Perform one PID step.

        Args:
            imu: IMU sample with pitch_rad and other fields
            dt_s: Time delta since last call, in seconds

        Returns:
            ControlOutput with clamped command and debug info

        Note:
            error = -pitch (carried over unchanged from the PID controller's inline code before #346, see git history; do NOT change; sign UNVERIFIED, #359)
        """
        pitch = float(imu.pitch_rad)
        dt = float(dt_s)
        error = -pitch

        if dt < 0:
            return ControlOutput(0.0, 0.0, (error, 0.0, 0.0, 0.0), True)

        out = self._pid.step(error, dt)

        if dt == 0:
            raw = self.kp * error
            debug = (error, raw, 0.0, 0.0)
        else:
            p, i, d = self._pid.last_components()
            raw = p + i + d
            debug = (error, p, i, d)

        L = self.out_limit
        cmd = max(-L, min(L, out))

        return ControlOutput(float(cmd), float(raw), debug, False)

    def reset(self) -> None:
        """Reset controller state (integral and previous error)."""
        self._pid = PlantPID(self.kp, self.ki, self.kd, -self.out_limit, self.out_limit)


class LqrBalance:
    """LQR state feedback controller for pitch and pitch-rate (theta, theta_dot).

    Stateless; dt is ignored. dt < 0 still returns fault=True.
    """

    def __init__(self, K=None, theta_ref_rad=DEFAULT_THETA_REF_RAD,
                 out_limit=DEFAULT_OUT_LIMIT):
        """Initialize LQR controller.

        Args:
            K: Gain vector [kp, kd] for [theta, theta_dot]. If None, use default_gain().
            theta_ref_rad: Reference pitch angle in radians
            out_limit: Output saturation limit (±out_limit)

        Raises:
            ValueError: if out_limit <= 0 or len(K) != 2 (if K is provided)
        """
        if out_limit <= 0:
            raise ValueError("out_limit must be positive")

        if K is None:
            from plant_lqr import default_gain
            g = default_gain()
            K = (float(g[0][0]), float(g[0][1]))
        else:
            if len(K) != 2:
                raise ValueError("K must be a sequence of length 2")
            K = (float(K[0]), float(K[1]))

        self.K = K
        self.theta_ref_rad = float(theta_ref_rad)
        self.out_limit = float(out_limit)

    def step(self, imu, dt_s) -> ControlOutput:
        """Perform one LQR step.

        Args:
            imu: IMU sample with pitch_rad and gyro_rad_s
            dt_s: Time delta (ignored for LQR; present for interface compatibility)

        Returns:
            ControlOutput with clamped command and debug info (theta, theta_dot)
        """
        theta = float(imu.pitch_rad) - self.theta_ref_rad
        theta_dot = float(imu.gyro_rad_s[1])

        if float(dt_s) < 0:
            return ControlOutput(0.0, 0.0, (theta, theta_dot), True)

        raw = -(self.K[0] * theta + self.K[1] * theta_dot)
        L = self.out_limit
        cmd = max(-L, min(L, raw))

        return ControlOutput(float(cmd), float(raw), (theta, theta_dot), False)

    def reset(self) -> None:
        """No-op; LQR is stateless."""
        pass


__all__ = [
    "DEFAULT_PID_KP",
    "DEFAULT_PID_KI",
    "DEFAULT_PID_KD",
    "DEFAULT_OUT_LIMIT",
    "DEFAULT_THETA_REF_RAD",
    "ControlOutput",
    "PidBalance",
    "LqrBalance",
]
