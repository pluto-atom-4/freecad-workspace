"""WebotsHal wraps a Webots Robot/Supervisor object passed in by the caller (it imports nothing from `controller`).

Gyro rad/s unit and sign agreement are UNVERIFIED locally; PID stability is untested (#359) so the HAL
assumes no controller; Webots behavior (step()==-1 on quit, getDevice None for a missing name, NaN before
first step, setVelocity above maxVelocity) is unverified here.
"""

import math
import sys
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hal.hal import Hal, HalFault, ImuSample, EncoderSample


class WebotsHal(Hal):
    """HAL backend wrapping a Webots Robot or Supervisor object."""

    def __init__(
        self,
        robot,
        sensor_names: dict,
        motor_names: dict,
        control_period_ms: int = 20,
        timestep_ms: Optional[int] = None,
    ):
        """Initialize WebotsHal with a Webots robot object and device names.

        Args:
            robot: Webots Robot or Supervisor instance passed by caller.
            sensor_names: Dict mapping keys to device names. Must include "imu", "gyro",
                          "wheel_left", "wheel_right".
            motor_names: Dict mapping keys to device names. Must include "wheel_left", "wheel_right".
            control_period_ms: Control loop period in milliseconds (default 20).
            timestep_ms: Webots basic time step in milliseconds. If None, read from robot.getBasicTimeStep().

        Raises:
            HalFault: If timestep or control_period is invalid, devices are missing, or
                      getDevice returns None for any device.
        """
        # Step 1: Store robot and time parameters
        self._robot = robot
        self._timestep_ms = int(robot.getBasicTimeStep()) if timestep_ms is None else int(timestep_ms)
        self._control_period_ms = int(control_period_ms)

        # Step 2: Validate time parameters
        if self._timestep_ms <= 0 or self._control_period_ms <= 0:
            raise HalFault(
                f"timestep_ms and control_period_ms must be > 0, got "
                f"timestep_ms={self._timestep_ms}, control_period_ms={self._control_period_ms}"
            )

        if self._timestep_ms >= self._control_period_ms:
            ts = self._timestep_ms
            cp = self._control_period_ms
            raise HalFault(f"basicTimeStep ({ts}ms) >= control period ({cp}ms)")

        # Step 3: Validate sensor and motor names
        required_sensors = {"imu", "gyro", "wheel_left", "wheel_right"}
        if not required_sensors.issubset(sensor_names.keys()):
            missing = required_sensors - set(sensor_names.keys())
            raise HalFault(f"sensor_names missing required keys: {missing}")

        required_motors = {"wheel_left", "wheel_right"}
        if not required_motors.issubset(motor_names.keys()):
            missing = required_motors - set(motor_names.keys())
            raise HalFault(f"motor_names missing required keys: {missing}")

        # Step 4: Load and enable all sensors
        self._sensors = {}
        for key, name in sensor_names.items():
            dev = robot.getDevice(name)
            if dev is None:
                raise HalFault(f"sensor key='{key}' device_name='{name}': getDevice returned None")
            dev.enable(self._timestep_ms)
            self._sensors[key] = dev

        # Step 5: Load all motors and set to velocity control mode
        self._motors = {}
        for key, name in motor_names.items():
            dev = robot.getDevice(name)
            if dev is None:
                raise HalFault(f"motor key='{key}' device_name='{name}': getDevice returned None")
            self._motors[key] = dev
            dev.setPosition(float('inf'))
            dev.setVelocity(0.0)

        # Step 6: Get and cache max velocities for wheel motors
        self._max_vel = {}
        for key in ("wheel_left", "wheel_right"):
            v = float(self._motors[key].getMaxVelocity())
            self._max_vel[key] = v if (math.isfinite(v) and v > 0) else None

        # Step 7: Initialize accumulators and state
        self._accum_ms = 0.0
        self._prev_fire_t = None
        self._stopped = False
        self._closed = False

    @property
    def robot(self):
        """Return the wrapped Webots Robot/Supervisor object."""
        return self._robot

    @property
    def timestep_ms(self) -> int:
        """Return the Webots basic time step in milliseconds."""
        return self._timestep_ms

    def sensor(self, key):
        """Return the sensor device object for the given key."""
        return self._sensors[key]

    def now_s(self) -> float:
        """Return current simulation time in seconds."""
        return float(self._robot.getTime())

    def wait_next_tick(self) -> float:
        """Block until next control tick; return dt or -1 to stop.

        Implements fixed-rate control loop via accumulator pattern. Loops stepping
        the robot until accumulated time reaches control_period_ms, then fires once
        per period. Returns nominal period on first call (HAL contract).

        Returns:
            dt in seconds (> 0), or -1.0 if robot.step() returned -1 (Webots quit).
        """
        if self._stopped:
            return -1.0

        while True:
            if self._robot.step(self._timestep_ms) == -1:
                self._stopped = True
                return -1.0

            self._accum_ms += self._timestep_ms

            if self._accum_ms >= self._control_period_ms:
                self._accum_ms -= self._control_period_ms
                break

        t = float(self._robot.getTime())
        if self._prev_fire_t is None:
            dt = self._control_period_ms / 1000.0
        else:
            dt = t - self._prev_fire_t
        self._prev_fire_t = t
        return dt

    def read_imu(self) -> ImuSample:
        """Read IMU (InertialUnit) and Gyro sensor data.

        Raises:
            HalFault: If either sensor returns None, has wrong length, contains None elements,
                      or any element is not finite.

        Returns:
            ImuSample with t_s=now_s(), roll/pitch/yaw from InertialUnit, gyro_rad_s from Gyro.
        """
        rpy = self._sensors["imu"].getRollPitchYaw()
        g = self._sensors["gyro"].getValues()

        if rpy is None or len(rpy) != 3 or any(x is None for x in rpy):
            raise HalFault("IMU getRollPitchYaw() returned None, wrong length, or None elements")

        if g is None or len(g) != 3 or any(x is None for x in g):
            raise HalFault("Gyro getValues() returned None, wrong length, or None elements")

        if not all(math.isfinite(float(x)) for x in rpy):
            raise HalFault("IMU getRollPitchYaw() contains non-finite values")

        if not all(math.isfinite(float(x)) for x in g):
            raise HalFault("Gyro getValues() contains non-finite values")

        return ImuSample(
            t_s=self.now_s(),
            roll_rad=float(rpy[0]),
            pitch_rad=float(rpy[1]),
            yaw_rad=float(rpy[2]),
            gyro_rad_s=(float(g[0]), float(g[1]), float(g[2])),
        )

    def read_encoders(self) -> EncoderSample:
        """Read cumulative wheel encoder positions.

        Raises:
            HalFault: If either encoder returns None or non-finite value.

        Returns:
            EncoderSample with left and right wheel angles in radians.
        """
        l = self._sensors["wheel_left"].getValue()
        r = self._sensors["wheel_right"].getValue()

        if l is None or r is None:
            raise HalFault("Encoder getValue() returned None")

        if not math.isfinite(float(l)) or not math.isfinite(float(r)):
            raise HalFault("Encoder getValue() returned non-finite value")

        return EncoderSample(float(l), float(r))

    def write_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> None:
        """Write wheel velocity commands in rad/s.

        Converts values to float, clamps to max velocity if known, and silently clamps
        non-finite values to zero. Never raises on the hot path.

        Args:
            left_rad_s: Left wheel velocity command (rad/s).
            right_rad_s: Right wheel velocity command (rad/s).
        """
        if self._closed:
            return

        for key, value in (("wheel_left", left_rad_s), ("wheel_right", right_rad_s)):
            try:
                v = float(value)
            except (TypeError, ValueError):
                v = 0.0

            if not math.isfinite(v):
                v = 0.0

            mv = self._max_vel[key]
            if mv is not None:
                v = max(-mv, min(mv, v))

            self._motors[key].setVelocity(v)

    def close(self) -> None:
        """Release resources and stop motors.

        Idempotent; safe to call multiple times. Sets both wheel motors to zero velocity.
        Does not call robot.simulationQuit().
        """
        if not self._closed:
            for key in ("wheel_left", "wheel_right"):
                try:
                    self._motors[key].setVelocity(0.0)
                except Exception:
                    pass
            self._closed = True
            self._stopped = True
