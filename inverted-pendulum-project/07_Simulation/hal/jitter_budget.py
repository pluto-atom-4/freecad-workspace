"""SMOKE model of tick-timing tolerance of the LQR core on the F.4 linear plant.

Default runs include repeating pushes to stress-test jitter and dropout handling: at each push,
the pendulum angle receives an impulse of ±0.02 rad (alternating sign, every 1.0 s, with sign
flipping each time). Pushes stop 2 s before the horizon, leaving the final 1 s tail window
to verify settling. This test mimics external disturbances—a settled robot at ~0 rad is
insensitive to timing jitter, so pushes force the controller to track continuously. Disable
pushes (push_rad=0.0) to measure baseline tolerance with no active disturbances.

Criterion: bounded lean through every push AND settled in the tail.

Measured 2026-10-05, numpy 2.4.6, `python3 -m hal.jitter_budget` from 07_Simulation (LQR and its
float32 copy agree on every number): max stable constant period 27 ms (28 ms fails); real Webots
pattern (16,16,16,32 ms, mean 20 ms) settles; constant 32 ms does not; max consecutive missed
updates every 10 ticks: 7 (9 without repeating pushes), every 25 ticks: 10; iid uniform jitter:
no failure found up to +-18 ms (the sweep ceiling), so this is a lower bound on the tolerable
jitter, not a limit.

Budget (measured value times a safety factor): jitter +-9 ms around 20 ms (ceiling-limited;
factor 0.5), sustained period 21.5 ms (27 ms x 0.8; only 7.5% above the 20 ms nominal), up to
3 consecutive missed updates (7 // 2). Safety factors are named constants
(JITTER_SAFETY_FACTOR, PERIOD_SAFETY_FACTOR).

THIS MEANS: in a linearized small-angle model of the plant (F.4 LinearPitchPlant, a SMOKE model),
the default-gain LQR, with the command held between ticks and the measured dt passed to the core,
stays within 2x the initial lean through every repeating push and settles in the tail window,
under the timing patterns above.

THIS DOES NOT MEAN: it is not a measurement of ESP32-C3 loop jitter; it says nothing about the C3
clock, soft-float cost, I2C and Dynamixel bus latency, MPU-6050 noise or gyro bias, motor
dynamics or the Webots plant. Sensor-to-actuator compute delay is NOT modelled (the command
applies at the next interval); one extra tick of delay would very likely shrink the margin.
Jitter is iid uniform and mean-preserving, not adversarial. LQR only: PID stability is untested
(#359). Risk R3 (timing) stays OPEN until c3_probe / loop_stats on hardware
(balancing-robot-controller #43, BRC#26/#32/#34). The C++ repo is not touched by this module.

Note: the sustained-period margin is thin (1.35x nominal at the limit); recorded only, no follow-up
design work decided.
"""

import math
import random
import sys
from pathlib import Path
from typing import NamedTuple

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hal.fake_hal import FakeHal, LinearPitchPlant
from hal.control_core import LqrBalance, DEFAULT_THETA_REF_RAD


class PushedFakeHal(FakeHal):
    """FakeHal variant that applies repeating angular pushes to stress-test control.

    At regular intervals, applies an impulse to the pendulum angle (alternating sign)
    to continuously stress jitter and dropout handling. Stops pushing before the final
    tail window so settling behavior can be verified.
    """

    def __init__(self, *args, push_rad=0.02, push_every_s=1.0, push_stop_s=None, **kwargs):
        """Initialize PushedFakeHal with push parameters.

        Args:
            push_rad: Magnitude of each angular push (radians).
            push_every_s: Interval between pushes (seconds).
            push_stop_s: Time at which to stop pushing (seconds); None = never stop.
            *args, **kwargs: Passed to FakeHal.__init__.
        """
        super().__init__(*args, **kwargs)
        self._push_rad = float(push_rad)
        self._push_every_s = float(push_every_s)
        self._push_stop_s = float(push_stop_s) if push_stop_s is not None else None
        self._next_push_t = self._push_every_s
        self._push_sign = 1.0

    def wait_next_tick(self) -> float:
        """Advance tick and apply push if conditions are met.

        Applies at most one push per tick after plant state advancement.

        Returns:
            dt in seconds (> 0), or -1.0 to stop.
        """
        dt = super().wait_next_tick()

        # Apply push if conditions met: plant mode, not stopped, time reached
        if (dt >= 0 and self._plant is not None and
            (self._push_stop_s is None or self._t < self._push_stop_s) and
            self._t >= self._next_push_t):
            # Apply alternating push to theta; theta_dot unchanged
            self._x = (self._x[0] + self._push_sign * self._push_rad, self._x[1])
            self._push_sign = -self._push_sign
            self._next_push_t += self._push_every_s

        return dt


# Module constants
NOMINAL_PERIOD_S = 0.020
REQUIRED_JITTER_S = 0.002
MIN_PERIOD_S = 0.001
THETA0_RAD = 0.05
LEAN_LIMIT_FACTOR = 2.0
SETTLE_BAND_RAD = 1e-3
TAIL_WINDOW_S = 1.0
HORIZON_S = 10.0
SWEEP_SEEDS = (0, 1, 2, 3, 4)
VERIFY_SEEDS = tuple(range(100, 110))
WEBOTS_REAL_PATTERN_S = (0.016, 0.016, 0.016, 0.032)
WEBOTS_ALT_PATTERN_S = (0.016, 0.032)
JITTER_SAFETY_FACTOR = 0.5
PERIOD_SAFETY_FACTOR = 0.8
DROPOUT_EVERY = 10
GRID_STEP_S = 0.0005
PUSH_RAD = 0.02
PUSH_EVERY_S = 1.0
MEASURED_MAX_JITTER_S = 0.018            # sweep CEILING: no failure found up to +-18 ms iid uniform jitter; NOT a failure limit
MEASURED_JITTER_IS_CEILING = True
JITTER_BUDGET_S = 0.009                  # floor_to_0.5ms(MEASURED_MAX_JITTER_S * JITTER_SAFETY_FACTOR)
MAX_PERIOD_BUDGET_S = 0.029              # NOMINAL_PERIOD_S + JITTER_BUDGET_S (single periods only; cannot persist)
MEASURED_MAX_CONSTANT_PERIOD_S = 0.027   # sustained constant period; 0.028 fails
SUSTAINED_PERIOD_BUDGET_S = 0.0215       # floor_to_0.5ms(MEASURED_MAX_CONSTANT_PERIOD_S * PERIOD_SAFETY_FACTOR)
MEASURED_MAX_DROPPED_TICKS = 7           # consecutive missed updates, repeated every DROPOUT_EVERY=10 ticks, with pushes (9 without pushes)
MAX_DROPPED_TICKS_BUDGET = 3             # MEASURED_MAX_DROPPED_TICKS // 2


class JitterRun(NamedTuple):
    """Result of a jitter tolerance test run."""
    ok: bool
    reason: str
    max_abs_err: float
    tail_mean_abs_err: float
    ticks: int
    t_end_s: float


def make_jittered_periods(nominal_s, jitter_s, n, seed):
    """Generate n periods with uniform random jitter.

    Args:
        nominal_s: Nominal period in seconds.
        jitter_s: Jitter magnitude (±range) in seconds.
        n: Number of periods to generate.
        seed: Random seed.

    Returns:
        List of n floats representing jittered periods.

    Raises:
        ValueError: If constraints violated (nominal_s <= 0, jitter_s < 0, n < 1,
                    or nominal_s - jitter_s < MIN_PERIOD_S).
    """
    if nominal_s <= 0:
        raise ValueError(f"nominal_s must be > 0, got {nominal_s}")
    if jitter_s < 0:
        raise ValueError(f"jitter_s must be >= 0, got {jitter_s}")
    if n < 1:
        raise ValueError(f"n must be >= 1, got {n}")
    if nominal_s - jitter_s < MIN_PERIOD_S:
        raise ValueError(
            f"nominal_s - jitter_s = {nominal_s - jitter_s} < MIN_PERIOD_S {MIN_PERIOD_S}"
        )

    if jitter_s == 0.0:
        return [nominal_s] * n

    rng = random.Random(seed)
    periods = []
    for _ in range(n):
        period = nominal_s + jitter_s * rng.uniform(-1.0, 1.0)
        periods.append(float(period))

    return periods


def webots_fire_periods(n, timestep_ms=16, control_ms=20):
    """Reproduce Webots accumulator-based firing pattern.

    Emulates the accumulator in WebotsHal.wait_next_tick (hal/webots_hal.py; it was inline in the Webots controllers before #346):
    Fixed-rate control gating via accumulator pattern.

    Args:
        n: Number of fire events (periods) to generate.
        timestep_ms: Sensor/actuator update rate in milliseconds.
        control_ms: Target control rate in milliseconds.

    Returns:
        List of n floats representing time deltas (in seconds) between fires.

    Raises:
        ValueError: If timestep_ms >= control_ms, either <= 0, or n < 1.
    """
    if timestep_ms <= 0 or control_ms <= 0:
        raise ValueError(
            f"timestep_ms and control_ms must be > 0, got {timestep_ms}, {control_ms}"
        )
    if timestep_ms >= control_ms:
        raise ValueError(
            f"timestep_ms ({timestep_ms}) must be < control_ms ({control_ms})"
        )
    if n < 1:
        raise ValueError(f"n must be >= 1, got {n}")

    periods = []
    accum = 0
    t = 0
    prev_fire_t = 0

    while len(periods) < n:
        t += timestep_ms
        accum += timestep_ms

        if accum >= control_ms:
            accum -= control_ms
            dt = (t - prev_fire_t) / 1000.0
            periods.append(float(dt))
            prev_fire_t = t

    return periods


def make_dropout_periods(nominal_s, n, every, burst):
    """Generate n periods with dropout (longer) periods at intervals.

    Args:
        nominal_s: Nominal period in seconds.
        n: Number of periods to generate.
        every: Frequency of dropouts (every 'every'-th period is long).
        burst: Number of nominal periods omitted (i.e., long period = nominal_s * (burst+1)).

    Returns:
        List of n floats.

    Raises:
        ValueError: If constraints violated (every < 2, burst < 0, n < 1, nominal_s <= 0).
    """
    if every < 2:
        raise ValueError(f"every must be >= 2, got {every}")
    if burst < 0:
        raise ValueError(f"burst must be >= 0, got {burst}")
    if n < 1:
        raise ValueError(f"n must be >= 1, got {n}")
    if nominal_s <= 0:
        raise ValueError(f"nominal_s must be > 0, got {nominal_s}")

    periods = []
    for i in range(n):
        if i % every == every - 1:
            period = nominal_s * (burst + 1)
        else:
            period = nominal_s
        periods.append(float(period))

    return periods


def run_jitter_loop(core, periods_s, theta0=THETA0_RAD, horizon_s=HORIZON_S,
                    push_rad=PUSH_RAD, push_every_s=PUSH_EVERY_S):
    """Run closed-loop simulation with jittered periods and record tracking error.

    By default, applies repeating angular pushes (±0.02 rad every 1 s) to stress-test
    jitter and dropout handling. Pushes stop 2 s before horizon to allow the tail
    window to judge settling. Pass push_rad=0.0 to disable pushes and measure baseline
    tolerance. Criterion: bounded lean through every push AND settled in the tail.

    Args:
        core: Control core (LqrBalance, Float32LqrBalance, or similar).
        periods_s: List/tuple of control tick periods (seconds). Cycled by FakeHal.
        theta0: Initial pitch angle (radians).
        horizon_s: Simulation horizon (seconds).
        push_rad: Angular push magnitude (radians). Pass 0.0 to disable.
        push_every_s: Interval between pushes (seconds).

    Returns:
        JitterRun with ok, reason, max_abs_err, tail_mean_abs_err, ticks, t_end_s.

    Raises:
        ValueError: If horizon_s < 2*TAIL_WINDOW_S or theta0 <= 0.
    """
    if horizon_s < 2 * TAIL_WINDOW_S:
        raise ValueError(
            f"horizon_s ({horizon_s}) must be >= 2*TAIL_WINDOW_S ({2*TAIL_WINDOW_S})"
        )
    if theta0 <= 0:
        raise ValueError(f"theta0 must be > 0, got {theta0}")

    # Build HAL with or without pushes
    if push_rad > 0:
        hal = PushedFakeHal(
            tick_periods_s=tuple(periods_s),
            plant=LinearPitchPlant.from_plant_lqr(),
            theta0_rad=theta0,
            max_ticks=100000,
            push_rad=push_rad,
            push_every_s=push_every_s,
            push_stop_s=horizon_s - 2 * TAIL_WINDOW_S,
        )
    else:
        hal = FakeHal(
            tick_periods_s=tuple(periods_s),
            plant=LinearPitchPlant.from_plant_lqr(),
            theta0_rad=theta0,
            max_ticks=100000,
        )

    records = []  # (t_s, abs_err)

    while True:
        dt = hal.wait_next_tick()
        if dt < 0:
            break

        imu = hal.read_imu()
        out = core.step(imu, dt)
        hal.write_wheel_velocity(out.cmd_rad_s, out.cmd_rad_s)

        err = imu.pitch_rad - DEFAULT_THETA_REF_RAD

        if not math.isfinite(err):
            return JitterRun(
                ok=False,
                reason="nonfinite",
                max_abs_err=float('nan'),
                tail_mean_abs_err=float('nan'),
                ticks=hal.tick_count,
                t_end_s=hal.now_s(),
            )

        if abs(err) > LEAN_LIMIT_FACTOR * theta0:
            return JitterRun(
                ok=False,
                reason="diverged",
                max_abs_err=max(abs(e) for _, e in records) if records else abs(err),
                tail_mean_abs_err=float('nan'),
                ticks=hal.tick_count,
                t_end_s=hal.now_s(),
            )

        records.append((hal.now_s(), abs(err)))

        if hal.now_s() >= horizon_s:
            break

    t_end = hal.now_s()

    # Compute max error and tail mean
    max_abs_err = max(abs(e) for _, e in records) if records else 0.0

    tail_start = t_end - TAIL_WINDOW_S
    tail_errs = [e for t, e in records if t >= tail_start]
    tail_mean_abs_err = sum(tail_errs) / len(tail_errs) if tail_errs else 0.0

    ok = tail_mean_abs_err <= SETTLE_BAND_RAD
    reason = "ok" if ok else "not_settled"

    return JitterRun(
        ok=ok,
        reason=reason,
        max_abs_err=float(max_abs_err),
        tail_mean_abs_err=float(tail_mean_abs_err),
        ticks=hal.tick_count,
        t_end_s=float(t_end),
    )


def settle_ok(core, plant_periods, theta0=THETA0_RAD, horizon_s=HORIZON_S,
              push_rad=PUSH_RAD, push_every_s=PUSH_EVERY_S):
    """Quick settlement check: run_jitter_loop and return ok.

    Args:
        core: Control core.
        plant_periods: List of periods.
        theta0: Initial pitch angle (radians).
        horizon_s: Simulation horizon (seconds).
        push_rad: Angular push magnitude (radians).
        push_every_s: Interval between pushes (seconds).

    Returns:
        Boolean: True if settled within tolerance.
    """
    return run_jitter_loop(core, plant_periods, theta0, horizon_s,
                          push_rad, push_every_s).ok


def max_tolerable_jitter(
    core_factory,
    nominal_s=NOMINAL_PERIOD_S,
    hi_s=0.018,
    step_s=GRID_STEP_S,
    seeds=SWEEP_SEEDS,
    theta0=THETA0_RAD,
    horizon_s=HORIZON_S,
    push_rad=PUSH_RAD,
    push_every_s=PUSH_EVERY_S,
):
    """Binary-search-like sweep for maximum tolerable jitter magnitude.

    Args:
        core_factory: Callable that creates a fresh control core.
        nominal_s: Nominal period.
        hi_s: Upper bound on jitter to sweep.
        step_s: Grid resolution.
        seeds: Seeds to test at each jitter level.
        theta0: Initial pitch angle.
        horizon_s: Simulation horizon.
        push_rad: Angular push magnitude (radians).
        push_every_s: Interval between pushes (seconds).

    Returns:
        Maximum jitter magnitude (in seconds) where all seeds pass.

    Raises:
        ValueError: If nominal_s - hi_s < MIN_PERIOD_S.
        RuntimeError: If nominal (zero-jitter) period is unstable.
    """
    if nominal_s - hi_s < MIN_PERIOD_S:
        raise ValueError(
            f"nominal_s - hi_s = {nominal_s - hi_s} < MIN_PERIOD_S {MIN_PERIOD_S}"
        )

    # Verify nominal period passes
    n_nominal = int(math.ceil(horizon_s / nominal_s)) + 2
    if not settle_ok(core_factory(), [nominal_s] * n_nominal, theta0, horizon_s,
                     push_rad, push_every_s):
        raise RuntimeError("nominal period unstable")

    last_passing_J = 0.0

    k = 1
    while True:
        J = round(k * step_s, 9)

        if J > hi_s:
            break

        # Test this jitter level with all seeds
        n = int(math.ceil(horizon_s / (nominal_s - J))) + 2
        all_pass = True

        for seed in seeds:
            periods = make_jittered_periods(nominal_s, J, n, seed)
            if not settle_ok(core_factory(), periods, theta0, horizon_s,
                             push_rad, push_every_s):
                all_pass = False
                break

        if not all_pass:
            # First failure: return last passing
            return last_passing_J

        last_passing_J = J
        k += 1

    # All tested jitter levels passed
    return last_passing_J


def max_stable_constant_period(
    core_factory,
    lo_s=0.010,
    hi_s=0.060,
    step_s=GRID_STEP_S,
    theta0=THETA0_RAD,
    horizon_s=HORIZON_S,
    push_rad=PUSH_RAD,
    push_every_s=PUSH_EVERY_S,
):
    """Ascending sweep to find maximum stable constant period.

    Args:
        core_factory: Callable that creates a fresh control core.
        lo_s: Lower bound on period.
        hi_s: Upper bound on period.
        step_s: Grid resolution.
        theta0: Initial pitch angle.
        horizon_s: Simulation horizon.
        push_rad: Angular push magnitude (radians).
        push_every_s: Interval between pushes (seconds).

    Returns:
        Maximum period (in seconds) where control is stable.

    Raises:
        RuntimeError: If lo_s is already unstable.
    """
    # Test lo_s first
    n_lo = int(math.ceil(horizon_s / lo_s)) + 2
    if not settle_ok(core_factory(), [lo_s] * n_lo, theta0, horizon_s,
                     push_rad, push_every_s):
        raise RuntimeError(f"lo_s period {lo_s} is unstable")

    last_passing_T = lo_s

    k = 1
    while True:
        T = round(lo_s + k * step_s, 9)

        if T > hi_s:
            break

        n = int(math.ceil(horizon_s / T)) + 2
        if settle_ok(core_factory(), [T] * n, theta0, horizon_s,
                     push_rad, push_every_s):
            last_passing_T = T
        else:
            # First failure: return last passing
            break

        k += 1

    return last_passing_T


def max_dropped_ticks(
    core_factory,
    nominal_s=NOMINAL_PERIOD_S,
    every=DROPOUT_EVERY,
    max_burst=12,
    theta0=THETA0_RAD,
    horizon_s=HORIZON_S,
    push_rad=PUSH_RAD,
    push_every_s=PUSH_EVERY_S,
):
    """Find maximum burst size for dropout tolerance.

    Args:
        core_factory: Callable that creates a fresh control core.
        nominal_s: Nominal period.
        every: Frequency of dropouts.
        max_burst: Maximum burst size to test.
        theta0: Initial pitch angle.
        horizon_s: Simulation horizon.
        push_rad: Angular push magnitude (radians).
        push_every_s: Interval between pushes (seconds).

    Returns:
        Maximum burst size (number of dropped ticks) tolerated. -1 if even b=0 fails.
    """
    last_passing_b = -1

    for b in range(max_burst + 1):
        n = int(math.ceil(horizon_s / nominal_s)) + 2
        periods = make_dropout_periods(nominal_s, n, every, b)

        if settle_ok(core_factory(), periods, theta0, horizon_s,
                     push_rad, push_every_s):
            last_passing_b = b
        else:
            break

    return last_passing_b


def main():
    """Run jitter budget measurement suite and print results."""
    K = LqrBalance().K
    factory = lambda: LqrBalance(K=K)

    # LQR sweep measurements (with pushes, defaults apply)
    lqr_max_jitter = max_tolerable_jitter(factory)
    print(f"JITTER-MEASURED lqr-max-tolerable-jitter {lqr_max_jitter}")

    lqr_max_period = max_stable_constant_period(factory)
    print(f"JITTER-MEASURED lqr-max-stable-period {lqr_max_period}")

    lqr_max_dropout_10 = max_dropped_ticks(factory, every=10)
    print(f"JITTER-MEASURED lqr-max-dropped-every10 {lqr_max_dropout_10}")

    lqr_max_dropout_25 = max_dropped_ticks(factory, every=25)
    print(f"JITTER-MEASURED lqr-max-dropped-every25 {lqr_max_dropout_25}")

    # LQR baseline measurements without pushes
    lqr_nopush_max_jitter = max_tolerable_jitter(factory, push_rad=0.0)
    print(f"JITTER-MEASURED lqr-nopush-max-tolerable-jitter {lqr_nopush_max_jitter}")

    lqr_nopush_max_dropout_10 = max_dropped_ticks(factory, every=10, push_rad=0.0)
    print(f"JITTER-MEASURED lqr-nopush-max-dropped-every10 {lqr_nopush_max_dropout_10}")

    # LQR settlement checks
    settle_tests = [
        ("const-0.016", [0.016] * int(math.ceil(HORIZON_S / 0.016) + 2)),
        ("const-0.020", [0.020] * int(math.ceil(HORIZON_S / 0.020) + 2)),
        ("const-0.024", [0.024] * int(math.ceil(HORIZON_S / 0.024) + 2)),
        ("const-0.028", [0.028] * int(math.ceil(HORIZON_S / 0.028) + 2)),
        ("const-0.032", [0.032] * int(math.ceil(HORIZON_S / 0.032) + 2)),
        ("webots-real", WEBOTS_REAL_PATTERN_S * int(math.ceil(HORIZON_S / sum(WEBOTS_REAL_PATTERN_S)) + 2)),
        ("webots-alt", WEBOTS_ALT_PATTERN_S * int(math.ceil(HORIZON_S / sum(WEBOTS_ALT_PATTERN_S)) + 2)),
        ("webots-fire-600", webots_fire_periods(600)),
        ("jittered-nominal", make_jittered_periods(0.020, REQUIRED_JITTER_S, 600, 0)),
    ]

    for tag, periods in settle_tests:
        result = run_jitter_loop(factory(), periods)
        print(
            f"SETTLE lqr-{tag} ok={result.ok} reason={result.reason} "
            f"max_err={result.max_abs_err:.2e} tail_mean={result.tail_mean_abs_err:.2e}"
        )

    # Float32 sweep measurements
    from hal.f32_emulation import Float32LqrBalance

    factory32 = lambda: Float32LqrBalance(K=K)

    f32_max_jitter = max_tolerable_jitter(factory32)
    print(f"JITTER-MEASURED f32-max-tolerable-jitter {f32_max_jitter}")

    f32_max_period = max_stable_constant_period(factory32)
    print(f"JITTER-MEASURED f32-max-stable-period {f32_max_period}")

    f32_max_dropout_10 = max_dropped_ticks(factory32, every=10)
    print(f"JITTER-MEASURED f32-max-dropped-every10 {f32_max_dropout_10}")

    f32_max_dropout_25 = max_dropped_ticks(factory32, every=25)
    print(f"JITTER-MEASURED f32-max-dropped-every25 {f32_max_dropout_25}")

    # Float32 settlement checks
    for tag, periods in settle_tests:
        result = run_jitter_loop(factory32(), periods)
        print(
            f"SETTLE f32-{tag} ok={result.ok} reason={result.reason} "
            f"max_err={result.max_abs_err:.2e} tail_mean={result.tail_mean_abs_err:.2e}"
        )

    return 0


__all__ = [
    "NOMINAL_PERIOD_S",
    "REQUIRED_JITTER_S",
    "MIN_PERIOD_S",
    "THETA0_RAD",
    "LEAN_LIMIT_FACTOR",
    "SETTLE_BAND_RAD",
    "TAIL_WINDOW_S",
    "HORIZON_S",
    "SWEEP_SEEDS",
    "VERIFY_SEEDS",
    "WEBOTS_REAL_PATTERN_S",
    "WEBOTS_ALT_PATTERN_S",
    "JITTER_SAFETY_FACTOR",
    "PERIOD_SAFETY_FACTOR",
    "DROPOUT_EVERY",
    "GRID_STEP_S",
    "PUSH_RAD",
    "PUSH_EVERY_S",
    "MEASURED_MAX_JITTER_S",
    "MEASURED_JITTER_IS_CEILING",
    "JITTER_BUDGET_S",
    "MAX_PERIOD_BUDGET_S",
    "MEASURED_MAX_CONSTANT_PERIOD_S",
    "SUSTAINED_PERIOD_BUDGET_S",
    "MEASURED_MAX_DROPPED_TICKS",
    "MAX_DROPPED_TICKS_BUDGET",
    "JitterRun",
    "PushedFakeHal",
    "make_jittered_periods",
    "webots_fire_periods",
    "make_dropout_periods",
    "run_jitter_loop",
    "settle_ok",
    "max_tolerable_jitter",
    "max_stable_constant_period",
    "max_dropped_ticks",
    "main",
]


if __name__ == "__main__":
    sys.exit(main())
