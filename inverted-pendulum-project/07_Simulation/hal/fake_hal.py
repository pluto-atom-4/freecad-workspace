"""SMOKE model of inverted pendulum plant and mock HAL backend.

LinearPitchPlant is a small-angle linearization derived from plant_lqr.py (lines 55-63);
it simulates the pendulum dynamics via continuous-time state space (A, B) integrated with
RK4 over discrete control ticks. Not a replacement for Webots validation.

FakeHal is a mock Hal backend that replays a pre-recorded IMU script or drives the plant
with commanded wheel velocities (zero-order hold per tick). Command u = (left+right)/2 is
interpreted as a cart-acceleration proxy (not true rad/s). Pitch sign follows HAL contract.
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hal.hal import Hal, HalFault, ImuSample, EncoderSample


class LinearPitchPlant:
    """Small-angle linearized inverted pendulum state-space plant.

    SMOKE model only; derived from plant_lqr.py (lines 55-63). Integrates continuous-time
    dynamics x_dot = A x + B u via RK4. State is [theta_rad, theta_dot_rad_s]; input u is
    interpreted as cart acceleration (proxy for wheel command, not true rad/s). Command
    is held constant across each advance() call (zero-order hold).

    Not a replacement for Webots validation.
    """

    def __init__(self, A, B, substep_s=0.002):
        """Initialize plant with state-space matrices.

        Args:
            A: 2x2 state transition matrix (numpy array or nested list).
            B: 2x1 input matrix (numpy array or nested list); B[i][0] is accessed.
            substep_s: RK4 substep duration in seconds. Must be > 0.

        Raises:
            ValueError: If substep_s <= 0.
        """
        if substep_s <= 0:
            raise ValueError(f"substep_s must be > 0, got {substep_s}")

        # Store A as 2x2 nested list of floats
        self._a = [[float(A[i][j]) for j in range(2)] for i in range(2)]

        # Store B as 2x1 nested list of floats
        self._b = [[float(B[i][0])] for i in range(2)]

        self._substep_s = float(substep_s)

    @classmethod
    def from_plant_lqr(cls, substep_s=0.002):
        """Construct plant from build_state_space() in plant_lqr.

        Lazily imports plant_lqr to avoid circular dependencies and numpy load overhead.

        Args:
            substep_s: RK4 substep duration in seconds.

        Returns:
            LinearPitchPlant instance initialized with A, B from build_state_space().
        """
        from plant_lqr import build_state_space

        A, B = build_state_space()
        return cls(A, B, substep_s)

    def advance(self, x, u, dt):
        """Integrate plant state via RK4 over dt seconds.

        Pure function; returns new state without modifying x.

        Dynamics: x_dot = A x + B u (continuous-time linear system).
        State x = [theta_rad, theta_dot_rad_s].
        Input u is scalar (cart acceleration proxy).
        Command u is held constant across the integration interval (zero-order hold).

        If dt <= 0, state is returned unchanged.

        Args:
            x: State tuple/list (theta, theta_dot).
            u: Scalar control input.
            dt: Time step in seconds.

        Returns:
            Tuple (new_theta, new_theta_dot) as floats.
        """
        if dt <= 0:
            return (float(x[0]), float(x[1]))

        # Number of RK4 substeps
        n = max(1, int(math.ceil(dt / self._substep_s - 1e-9)))
        h = dt / n

        # Current state
        theta, theta_dot = float(x[0]), float(x[1])

        for _ in range(n):
            # RK4: k1, k2, k3, k4 are [dtheta, dtheta_dot] pairs
            # dx = [theta_dot, A[1][0]*theta + A[1][1]*theta_dot + B[1][0]*u]

            # k1 at current state
            k1_0 = theta_dot
            k1_1 = self._a[1][0] * theta + self._a[1][1] * theta_dot + self._b[1][0] * u

            # k2 at state + h/2 * k1
            th_k2 = theta + 0.5 * h * k1_0
            thd_k2 = theta_dot + 0.5 * h * k1_1
            k2_0 = thd_k2
            k2_1 = self._a[1][0] * th_k2 + self._a[1][1] * thd_k2 + self._b[1][0] * u

            # k3 at state + h/2 * k2
            th_k3 = theta + 0.5 * h * k2_0
            thd_k3 = theta_dot + 0.5 * h * k2_1
            k3_0 = thd_k3
            k3_1 = self._a[1][0] * th_k3 + self._a[1][1] * thd_k3 + self._b[1][0] * u

            # k4 at state + h * k3
            th_k4 = theta + h * k3_0
            thd_k4 = theta_dot + h * k3_1
            k4_0 = thd_k4
            k4_1 = self._a[1][0] * th_k4 + self._a[1][1] * thd_k4 + self._b[1][0] * u

            # Weighted average
            theta += (h / 6.0) * (k1_0 + 2 * k2_0 + 2 * k3_0 + k4_0)
            theta_dot += (h / 6.0) * (k1_1 + 2 * k2_1 + 2 * k3_1 + k4_1)

        return (theta, theta_dot)


class FakeHal(Hal):
    """Mock Hal backend for simulation.

    Supports two modes (mutually exclusive):
      1. Script mode: replays a sequence of pre-recorded ImuSample objects.
      2. Plant mode: integrates LinearPitchPlant dynamics driven by wheel commands.

    SMOKE model only; u = (left+right)/2 is a cart-acceleration proxy even though the
    real wheel command is rad/s. Not a replacement for Webots validation.

    Wheel command is held constant across each wait_next_tick() interval (zero-order hold).
    Pitch sign follows HAL contract (radians, same convention as Webots InertialUnit).

    First call to wait_next_tick() returns periods[0] (nonzero), satisfying the HAL contract's
    first-tick rule.
    """

    def __init__(
        self,
        tick_periods_s=(0.016, 0.032),
        imu_script=None,
        plant=None,
        theta0_rad=0.05,
        max_ticks=None,
        theta_ref_rad=-0.1086,
    ):
        """Initialize FakeHal backend.

        Args:
            tick_periods_s: Tuple of control tick periods (seconds). Cycled; each wait_next_tick()
                advances by periods[tick_count % len(periods)]. Empty or any p <= 0 raises ValueError.
            imu_script: Optional list of ImuSample objects or 5-tuples (t_s, roll, pitch, yaw, gyro_tuple)
                coerced via ImuSample(*tup). If provided, plant must be None.
            plant: Optional LinearPitchPlant instance. If provided, imu_script must be None.
                State advanced via plant.advance(x, u, dt).
            theta0_rad: Initial pitch angle (radians), used only if plant is given.
            max_ticks: Optional maximum tick count. After tick_count >= max_ticks,
                wait_next_tick() returns -1.0 (stopping signal) without advancing clock or state.
            theta_ref_rad: Nominal/reference pitch angle (radians). Subtracted from plant pitch
                when reporting IMU; ImuSample.pitch_rad = plant_theta + theta_ref_rad.

        Raises:
            ValueError: If tick_periods_s is empty, contains p <= 0, or both imu_script
                and plant are provided.
        """
        self._periods = tuple(float(p) for p in tick_periods_s)

        if not self._periods or any(p <= 0 for p in self._periods):
            raise ValueError(
                f"tick_periods_s must be non-empty and all > 0, got {tick_periods_s}"
            )

        if imu_script is not None and plant is not None:
            raise ValueError("imu_script and plant are mutually exclusive")

        # Coerce imu_script entries to ImuSample if needed
        if imu_script is not None:
            self._imu_script = []
            for entry in imu_script:
                if isinstance(entry, ImuSample):
                    self._imu_script.append(entry)
                else:
                    # Assume 5-tuple: (t_s, roll, pitch, yaw, gyro_tuple)
                    self._imu_script.append(ImuSample(*entry))
        else:
            self._imu_script = None

        self._plant = plant
        self._theta_ref_rad = float(theta_ref_rad)
        self._max_ticks = max_ticks

        # State
        self._t = 0.0
        self.tick_count = 0
        self.commands = []  # Public; list of (t_s, left, right) tuples

        if plant is not None:
            self._x = (float(theta0_rad), 0.0)
        else:
            self._x = None

        self._u = 0.0  # Current (held) wheel velocity command
        self._imu_index = 0  # Index into _imu_script

    def read_imu(self) -> ImuSample:
        """Read next IMU sample.

        Script mode: return next entry from imu_script; raise HalFault when exhausted.
        Plant mode: return synthetic ImuSample with pitch = plant_theta + theta_ref_rad,
                    pitch_rate = plant_theta_dot, other fields zero.
        No source: raise HalFault.

        Returns:
            ImuSample with t_s = self.now_s().

        Raises:
            HalFault: If script exhausted or no source configured.
        """
        if self._imu_script is not None:
            if self._imu_index >= len(self._imu_script):
                raise HalFault("imu script exhausted")
            sample = self._imu_script[self._imu_index]
            self._imu_index += 1
            return sample

        if self._plant is not None:
            theta, theta_dot = self._x
            return ImuSample(
                t_s=self._t,
                roll_rad=0.0,
                pitch_rad=theta + self._theta_ref_rad,
                yaw_rad=0.0,
                gyro_rad_s=(0.0, theta_dot, 0.0),
            )

        raise HalFault("FakeHal has no imu source")

    def read_encoders(self) -> EncoderSample:
        """FakeHal has no encoder support.

        Raises:
            HalFault: Always.
        """
        raise HalFault("FakeHal has no encoders")

    def write_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> None:
        """Record wheel command and hold it for next tick.

        Command is zero-order held across the tick duration returned by the next
        wait_next_tick() call. u = (left + right) / 2 is interpreted as cart
        acceleration proxy (not true rad/s).

        Never raises, never clamps (unlike MCU backend).

        Args:
            left_rad_s: Left wheel velocity command (rad/s).
            right_rad_s: Right wheel velocity command (rad/s).
        """
        self.commands.append((self._t, float(left_rad_s), float(right_rad_s)))
        self._u = (float(left_rad_s) + float(right_rad_s)) / 2.0

    def now_s(self) -> float:
        """Return current backend clock time in seconds."""
        return self._t

    def wait_next_tick(self) -> float:
        """Block (trivially, synchronously) until next control tick.

        If max_ticks is set and tick_count >= max_ticks, return -1.0 immediately
        without advancing time or state (stopping signal). Multiple calls then
        always return -1.0.

        Otherwise, advance time by periods[tick_count % len(periods)], increment
        tick_count, and if plant is active, integrate state via plant.advance().
        Return the dt in seconds.

        HAL contract: first call returns periods[0] (nonzero), satisfying the
        first-tick rule.

        Returns:
            dt in seconds (> 0), or -1.0 to stop.
        """
        if self._max_ticks is not None and self.tick_count >= self._max_ticks:
            return -1.0

        dt = self._periods[self.tick_count % len(self._periods)]
        self.tick_count += 1
        self._t += dt

        if self._plant is not None:
            self._x = self._plant.advance(self._x, self._u, dt)

        return dt
