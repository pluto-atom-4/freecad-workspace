"""HAL (Hardware Abstraction Layer) contract for the inverted pendulum controller.

HAL_CONTRACT_VERSION: bump on ANY change to units, pitch sign, field order, dt semantics or THETA_REF handling. Must equal hal::kContractVersion in balancing-robot-controller lib/hal_iface/hal_iface.h (checked by export_cpp.py --check, #353, and the C++ stamp test).

All angles in radians, all rates in rad/s, all times in seconds. NOT degrees. (Degrees live only inside a backend, e.g. MCU lib/tilt, and are converted in the backend.)

pitch_rad has the same sign convention as Webots InertialUnit getRollPitchYaw()[1]. The Webots backend passes it through unchanged. The MCU backend applies exactly ONE sign constant (kPitchSign, balancing-robot-controller#30); that sign is verified only in hardware-in-the-loop, not in this repo.

gyro_rad_s is rad/s in body axes; index 1 is pitch rate d(pitch)/dt. Webots Gyro is rad/s per the Webots reference manual (NOT checked locally); LqrBalance.step (hal/control_core.py) feeds imu.gyro_rad_s[1] unconverted into K (the Webots LQR controller did the same inline before #346). Sign agreement of gyro_y with d(pitch)/dt is not independently verified.

Gyro/IMU bias calibration (risk R7) is applied INSIDE the backend before an ImuSample is returned; callers never see raw biased data. A backend must not return samples (HalFault) until calibration succeeded.

This module and everything it imports must not import Webots `controller`; only webots_hal.py may (leak guard, #343).

dt_s is measured seconds from wait_next_tick(). dt == 0 -> core output is P-only / no state update (PID; PlantPID does this, plant_pid.py:72-74); stateless LQR gives its normal output. dt < 0 -> core outputs 0 and raises a fault flag (NOTE: PlantPID itself raises ValueError for dt<0, so cores must pre-check). wait_next_tick() returning < 0 means STOP the loop (sim quit / backend shutdown); it never raises for sim quit.

The first wait_next_tick() after construction returns the nominal control period (e.g. 0.02), not 0.

Python raises HalFault; C++ returns false from read_imu/read_encoders. close() is Python-only.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Tuple

HAL_CONTRACT_VERSION = 1


class HalFault(Exception):
    """Raised by a backend on bad/None/unavailable sensor data, uncalibrated IMU, missing device, or exhausted script."""
    pass


@dataclass(frozen=True)
class ImuSample:
    """t_s = backend clock seconds (same clock as Hal.now_s()); gyro index 1 = pitch rate; unit rad/s."""
    t_s: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    gyro_rad_s: Tuple[float, float, float]


@dataclass(frozen=True)
class EncoderSample:
    """Cumulative wheel angle in rad; a backend with no encoders raises HalFault."""
    left_rad: float
    right_rad: float


class Hal(ABC):
    """Abstract HAL interface."""

    @abstractmethod
    def read_imu(self) -> ImuSample:
        """Raises HalFault."""
        pass

    @abstractmethod
    def read_encoders(self) -> EncoderSample:
        """Raises HalFault."""
        pass

    @abstractmethod
    def write_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> None:
        """rad/s; callers clamp, backend may additionally clamp for safety; must not raise on the hot path."""
        pass

    @abstractmethod
    def now_s(self) -> float:
        """Backend clock seconds."""
        pass

    @abstractmethod
    def wait_next_tick(self) -> float:
        """Blocks to next control tick; returns dt seconds, or -1.0 to stop."""
        pass

    def close(self) -> None:
        """Python-only; release resources."""
        pass


__all__ = ["HAL_CONTRACT_VERSION", "HalFault", "ImuSample", "EncoderSample", "Hal"]
