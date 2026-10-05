"""Unit tests for jitter_budget.py module.

Validates tick-timing tolerance of LQR and float32 cores on F.4 linear plant.
Deterministic tests with measured values from 2026-10-05 (numpy 2.4.6).
See jitter_budget.py docstring for budget numbers and safety margins.

Usage:
    mamba run -n pendulum-tools python3 -m pytest -q inverted-pendulum-project/07_Simulation/hal/test_jitter_budget.py
"""

import ast
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import hal.jitter_budget as jb
from hal.jitter_budget import (
    NOMINAL_PERIOD_S,
    REQUIRED_JITTER_S,
    MIN_PERIOD_S,
    THETA0_RAD,
    LEAN_LIMIT_FACTOR,
    SETTLE_BAND_RAD,
    TAIL_WINDOW_S,
    HORIZON_S,
    SWEEP_SEEDS,
    VERIFY_SEEDS,
    WEBOTS_REAL_PATTERN_S,
    WEBOTS_ALT_PATTERN_S,
    JITTER_SAFETY_FACTOR,
    PERIOD_SAFETY_FACTOR,
    DROPOUT_EVERY,
    GRID_STEP_S,
    PUSH_RAD,
    PUSH_EVERY_S,
    MEASURED_MAX_JITTER_S,
    MEASURED_JITTER_IS_CEILING,
    JITTER_BUDGET_S,
    MAX_PERIOD_BUDGET_S,
    MEASURED_MAX_CONSTANT_PERIOD_S,
    SUSTAINED_PERIOD_BUDGET_S,
    MEASURED_MAX_DROPPED_TICKS,
    MAX_DROPPED_TICKS_BUDGET,
    JitterRun,
    PushedFakeHal,
    make_jittered_periods,
    webots_fire_periods,
    make_dropout_periods,
    run_jitter_loop,
    settle_ok,
    max_tolerable_jitter,
    max_stable_constant_period,
    max_dropped_ticks,
)
from hal.control_core import LqrBalance, PidBalance, DEFAULT_THETA_REF_RAD
from hal.fake_hal import FakeHal
from hal.hal import ImuSample
from hal.f32_emulation import Float32LqrBalance, run_closed_loop


# Shared factories
K = LqrBalance().K
lqr = lambda: LqrBalance(K=K)
lqr32 = lambda: Float32LqrBalance(K=K)


# ============================================================================
# PART A: Exploratory / Validation Tests
# ============================================================================

class TestA1MakeJitteredPeriods:
    """A1: make_jittered_periods: determinism, divergence, bounds, and zero-jitter."""

    def test_a1_same_seed_twice_equal(self):
        """Same seed yields identical results."""
        p1 = make_jittered_periods(0.020, 0.005, 100, seed=7)
        p2 = make_jittered_periods(0.020, 0.005, 100, seed=7)
        assert p1 == p2

    def test_a1_different_seeds_differ(self):
        """Seeds 7 vs 8 produce different sequences."""
        p7 = make_jittered_periods(0.020, 0.005, 100, seed=7)
        p8 = make_jittered_periods(0.020, 0.005, 100, seed=8)
        assert p7 != p8

    def test_a1_jitter_bounds(self):
        """J=0.005, n=1000: all in [0.015, 0.025], mean within 5e-4 of nominal."""
        periods = make_jittered_periods(0.020, 0.005, 1000, seed=42)
        assert all(0.015 <= p <= 0.025 for p in periods)
        mean = sum(periods) / len(periods)
        assert abs(mean - 0.020) <= 5e-4

    def test_a1_zero_jitter_returns_exact_nominal(self):
        """J=0.0 returns [nominal]*n exactly."""
        periods = make_jittered_periods(0.020, 0.0, 100, seed=1)
        assert periods == [0.020] * 100


class TestA2MakeJitteredPeriodValidation:
    """A2: make_jittered_periods validation: ValueError for invalid inputs."""

    def test_a2_negative_jitter(self):
        """jitter_s < 0 raises ValueError."""
        with pytest.raises(ValueError):
            make_jittered_periods(0.020, -0.001, 10, seed=1)

    def test_a2_zero_n(self):
        """n=0 raises ValueError."""
        with pytest.raises(ValueError):
            make_jittered_periods(0.020, 0.005, 0, seed=1)

    def test_a2_zero_nominal(self):
        """nominal_s=0 raises ValueError."""
        with pytest.raises(ValueError):
            make_jittered_periods(0.0, 0.005, 10, seed=1)

    def test_a2_jitter_exceeds_min_period(self):
        """nominal_s - jitter_s < MIN_PERIOD_S raises ValueError."""
        with pytest.raises(ValueError):
            make_jittered_periods(0.020, 0.020, 10, seed=1)


class TestA3WebotsFirePeriods:
    """A3: webots_fire_periods reproduces Webots accumulator pattern."""

    def test_a3_pattern_first_nine(self):
        """First 9 values match expected pattern approx [0.032,0.016,0.016,0.016,0.032,0.016,0.016,0.016,0.032]."""
        periods = webots_fire_periods(9)
        expected = [0.032, 0.016, 0.016, 0.016, 0.032, 0.016, 0.016, 0.016, 0.032]
        for p, e in zip(periods, expected):
            assert abs(p - e) < 1e-12

    def test_a3_longer_sum(self):
        """Sum of periods[1:1+4*50] ≈ 0.080*50 (within 1e-9)."""
        periods = webots_fire_periods(201)
        chunk_sum = sum(periods[1:1+4*50])
        expected = 0.080 * 50
        assert abs(chunk_sum - expected) < 1e-9

    def test_a3_invalid_timestep(self):
        """timestep_ms >= control_ms raises ValueError."""
        with pytest.raises(ValueError):
            webots_fire_periods(10, timestep_ms=20, control_ms=20)


class TestA4MakeDropoutPeriods:
    """A4: make_dropout_periods generates periodic longer periods."""

    def test_a4_every_five_burst_two(self):
        """every=5, burst=2, n=10: [0.02,0.02,0.02,0.02,0.06]*2 (approx)."""
        periods = make_dropout_periods(0.020, 10, every=5, burst=2)
        expected = [0.02, 0.02, 0.02, 0.02, 0.06] * 2
        for p, e in zip(periods, expected):
            assert abs(p - e) < 1e-12

    def test_a4_burst_zero(self):
        """burst=0: all periods are nominal."""
        periods = make_dropout_periods(0.020, 20, every=5, burst=0)
        assert all(p == 0.020 for p in periods)

    def test_a4_invalid_every(self):
        """every=1 raises ValueError."""
        with pytest.raises(ValueError):
            make_dropout_periods(0.020, 10, every=1, burst=2)


class TestA5SettleOkTrue:
    """A5: settle_ok returns True for various jitter patterns (parametrized, all MEASURED 2026-10-05)."""

    @pytest.mark.parametrize("period,label", [
        (0.016, "const-0.016"),
        (0.020, "const-0.020"),
        (0.024, "const-0.024"),
    ])
    def test_a5_constant_periods(self, period, label):
        """Constant periods settle."""
        N = 700
        periods = [period] * N
        assert settle_ok(lqr(), periods, theta0=0.05, horizon_s=HORIZON_S)

    def test_a5_webots_real_pattern(self):
        """WEBOTS_REAL_PATTERN_S settles (real 16/16/16/32 ms pattern)."""
        N = 700
        period_sum = sum(WEBOTS_REAL_PATTERN_S)
        reps = int(math.ceil(HORIZON_S / period_sum)) + 1
        periods = (WEBOTS_REAL_PATTERN_S * reps)[:N]
        assert settle_ok(lqr(), periods, theta0=0.05, horizon_s=HORIZON_S)

    def test_a5_webots_alt_pattern(self):
        """WEBOTS_ALT_PATTERN_S settles (stress case 16/32 ms)."""
        N = 700
        period_sum = sum(WEBOTS_ALT_PATTERN_S)
        reps = int(math.ceil(HORIZON_S / period_sum)) + 1
        periods = (WEBOTS_ALT_PATTERN_S * reps)[:N]
        assert settle_ok(lqr(), periods, theta0=0.05, horizon_s=HORIZON_S)

    def test_a5_webots_fire_periods(self):
        """webots_fire_periods(600) settles."""
        periods = webots_fire_periods(600)
        assert settle_ok(lqr(), periods, theta0=0.05, horizon_s=HORIZON_S)

    @pytest.mark.parametrize("seed", list(SWEEP_SEEDS))
    def test_a5_jittered_seeds(self, seed):
        """make_jittered_periods with nominal and low jitter settles for seeds 0..4."""
        periods = make_jittered_periods(0.020, REQUIRED_JITTER_S, 600, seed)
        assert settle_ok(lqr(), periods, theta0=0.05, horizon_s=HORIZON_S)


class TestA6SettleOkFalse:
    """A6: settle_ok returns False for periods too slow (constant 0.028, 0.032)."""

    def test_a6_constant_028(self):
        """Constant 0.028 does not settle; reason='not_settled'."""
        N = 700
        result = run_jitter_loop(lqr(), [0.028] * N, theta0=0.05, horizon_s=HORIZON_S)
        assert result.ok is False
        assert result.reason == "not_settled"

    def test_a6_constant_032(self):
        """Constant 0.032 does not settle; reason='not_settled'."""
        N = 700
        result = run_jitter_loop(lqr(), [0.032] * N, theta0=0.05, horizon_s=HORIZON_S)
        assert result.ok is False
        assert result.reason == "not_settled"

    def test_a6_constant_040_fails_no_reason_check(self):
        """Constant 0.040 must fail (not ok), but do not assert reason."""
        N = 700
        result = run_jitter_loop(lqr(), [0.040] * N, theta0=0.05, horizon_s=HORIZON_S)
        assert result.ok is False


class TestA7NonVacuity:
    """A7: run_jitter_loop produces non-trivial error; not a vacuous test."""

    def test_a7_max_error_and_ticks(self):
        """Constant 0.020 with pushes: 0.05 < max_abs_err < 0.06 and ticks > 400."""
        result = run_jitter_loop(lqr(), [0.020] * 700, theta0=0.05, horizon_s=HORIZON_S)
        assert 0.05 < result.max_abs_err < 0.06
        assert result.ticks > 400


class TestA8Determinism:
    """A8: run_jitter_loop is deterministic; identical calls yield identical results."""

    def test_a8_identical_runs(self):
        """Two identical run_jitter_loop calls return equal JitterRun tuples."""
        periods = make_jittered_periods(0.020, REQUIRED_JITTER_S, 600, seed=99)
        result1 = run_jitter_loop(lqr(), periods, theta0=0.05, horizon_s=HORIZON_S)
        result2 = run_jitter_loop(lqr(), periods, theta0=0.05, horizon_s=HORIZON_S)
        assert result1 == result2


class TestA9Float32Core:
    """A9: Float32LqrBalance settles on real pattern but not on 0.032."""

    def test_a9_settles_webots_real(self):
        """Float32LqrBalance settles on WEBOTS_REAL_PATTERN_S."""
        N = 700
        period_sum = sum(WEBOTS_REAL_PATTERN_S)
        reps = int(math.ceil(HORIZON_S / period_sum)) + 1
        periods = (WEBOTS_REAL_PATTERN_S * reps)[:N]
        assert settle_ok(lqr32(), periods, theta0=0.05, horizon_s=HORIZON_S)

    def test_a9_does_not_settle_constant_032(self):
        """Float32LqrBalance does not settle on constant 0.032."""
        N = 700
        assert not settle_ok(lqr32(), [0.032] * N, theta0=0.05, horizon_s=HORIZON_S)


class TestA10PushesMattering:
    """A10: Pushes matter; tail_mean_abs_err with default push_rad vs push_rad=0.0 differ."""

    def test_a10_push_effect(self):
        """Default pushes yield tail_mean > 1e-9; no pushes yield tail_mean < 1e-9."""
        periods = [0.020] * 700

        # With pushes (default)
        with_pushes = run_jitter_loop(lqr(), periods, theta0=0.05, horizon_s=HORIZON_S,
                                       push_rad=PUSH_RAD)
        assert with_pushes.tail_mean_abs_err > 1e-9

        # Without pushes
        no_pushes = run_jitter_loop(lqr(), periods, theta0=0.05, horizon_s=HORIZON_S,
                                     push_rad=0.0)
        assert no_pushes.tail_mean_abs_err < 1e-9


class TestA11PidNotStabilityClaim:
    """A11: PidBalance with scripted input is bounded and repeatable, not a stability claim."""

    def test_a11_pid_bounded_repeatable(self):
        """PidBalance on jittered script: all cmd finite and bounded; repeat gives same record."""
        # Generate 200 pitch values from sine
        pitches = [0.1 * math.sin(0.7 * k) - 0.02 for k in range(200)]
        periods = make_jittered_periods(0.020, 0.005, 200, seed=1)

        hal1 = FakeHal(
            tick_periods_s=tuple(periods),
            imu_script=[ImuSample(0.0, 0.0, p, 0.0, (0.0, 0.0, 0.0)) for p in pitches],
            max_ticks=200,
        )

        records1 = []
        pid = PidBalance()
        while True:
            dt = hal1.wait_next_tick()
            if dt < 0:
                break
            imu = hal1.read_imu()
            out = pid.step(imu, dt)
            records1.append(out.cmd_rad_s)
            assert math.isfinite(out.cmd_rad_s)
            assert abs(out.cmd_rad_s) <= 1.0

        # Repeat with fresh core
        hal2 = FakeHal(
            tick_periods_s=tuple(periods),
            imu_script=[ImuSample(0.0, 0.0, p, 0.0, (0.0, 0.0, 0.0)) for p in pitches],
            max_ticks=200,
        )

        records2 = []
        pid2 = PidBalance()
        while True:
            dt = hal2.wait_next_tick()
            if dt < 0:
                break
            imu = hal2.read_imu()
            out = pid2.step(imu, dt)
            records2.append(out.cmd_rad_s)

        assert records1 == records2
        # Docstring comment: "PID stability is untested (#359); bounded/repeatable only"


class TestA12Validation:
    """A12: run_jitter_loop validates horizon_s and theta0."""

    def test_a12_horizon_too_small(self):
        """horizon_s < 2*TAIL_WINDOW_S raises ValueError."""
        with pytest.raises(ValueError):
            run_jitter_loop(lqr(), [0.020] * 100, theta0=0.05, horizon_s=1.0)

    def test_a12_theta0_zero_or_negative(self):
        """theta0 <= 0 raises ValueError."""
        with pytest.raises(ValueError):
            run_jitter_loop(lqr(), [0.020] * 100, theta0=0.0, horizon_s=HORIZON_S)


class TestA13AstImports:
    """A13: jitter_budget.py and this test file: no top-level import/import-from module=='controller'."""

    def test_a13_jitter_budget_no_controller_import(self):
        """jitter_budget.py has no top-level import/import-from with module=='controller'."""
        jb_file = Path(__file__).resolve().parent / "jitter_budget.py"
        jb_text = jb_file.read_text()
        jb_ast = ast.parse(jb_text)

        for node in jb_ast.body:
            if isinstance(node, ast.Import):
                for alias in node.names:
                    assert alias.name != "controller"
            elif isinstance(node, ast.ImportFrom):
                if node.module:
                    assert node.module != "controller"

    def test_a13_test_file_no_controller_import(self):
        """This test file has no top-level import/import-from with module=='controller'."""
        test_file = Path(__file__).resolve()
        test_text = test_file.read_text()
        test_ast = ast.parse(test_text)

        for node in test_ast.body:
            if isinstance(node, ast.Import):
                for alias in node.names:
                    assert alias.name != "controller"
            elif isinstance(node, ast.ImportFrom):
                if node.module:
                    assert node.module != "controller"

    def test_a13_jitter_budget_no_pidbid_strings(self):
        """jitter_budget.py source text contains neither 'PidBalance' nor 'Float32PidBalance'."""
        jb_file = Path(__file__).resolve().parent / "jitter_budget.py"
        jb_text = jb_file.read_text()
        assert "PidBalance" not in jb_text
        assert "Float32PidBalance" not in jb_text


# ============================================================================
# PART B: Pinned-Budget Tests
# ============================================================================

class TestB1MaxTolerableJitter:
    """B1: max_tolerable_jitter(lqr) meets budget and measured value."""

    def test_b1_jitter_budget(self):
        """max_tolerable_jitter(lqr) >= JITTER_BUDGET_S and approx MEASURED_MAX_JITTER_S ± GRID_STEP_S."""
        result = max_tolerable_jitter(lqr)
        assert result >= JITTER_BUDGET_S
        assert result >= REQUIRED_JITTER_S
        assert abs(result - MEASURED_MAX_JITTER_S) <= GRID_STEP_S
        assert MEASURED_JITTER_IS_CEILING is True
        assert result >= 0.018 - 1e-9
        print(f"JITTER-MEASURED jitter {result}")


class TestB2MaxStableConstantPeriod:
    """B2: max_stable_constant_period(lqr) meets sustained budget."""

    def test_b2_sustained_period_budget(self):
        """max_stable_constant_period(lqr) >= SUSTAINED_PERIOD_BUDGET_S and approx MEASURED_MAX_CONSTANT_PERIOD_S ± GRID_STEP_S."""
        result = max_stable_constant_period(lqr)
        assert result >= SUSTAINED_PERIOD_BUDGET_S
        assert abs(result - MEASURED_MAX_CONSTANT_PERIOD_S) <= GRID_STEP_S
        print(f"JITTER-MEASURED max_stable_period {result}")


class TestB3MaxDroppedTicks:
    """B3: max_dropped_ticks(lqr) meets dropout budget."""

    def test_b3_dropped_ticks_budget(self):
        """max_dropped_ticks(lqr) >= MAX_DROPPED_TICKS_BUDGET and within 1 of MEASURED."""
        result = max_dropped_ticks(lqr)
        assert result >= MAX_DROPPED_TICKS_BUDGET
        assert abs(result - MEASURED_MAX_DROPPED_TICKS) <= 1
        print(f"JITTER-MEASURED max_dropped_ticks {result}")


class TestB4AtBudgetVerification:
    """B4: At-budget 60 s verification: both LQR and float32 settle under budget jitter."""

    def test_b4_at_budget_verification(self):
        """For seeds 100..109 and theta0 in (0.02, 0.05): settle_ok True for both lqr() and lqr32() at budget jitter, 60 s horizon."""
        n = int(math.ceil(60.0 / (NOMINAL_PERIOD_S - JITTER_BUDGET_S))) + 2

        for seed in VERIFY_SEEDS:
            for theta0 in (0.02, 0.05):
                periods = make_jittered_periods(NOMINAL_PERIOD_S, JITTER_BUDGET_S, n, seed)

                # LQR reference
                assert settle_ok(lqr(), periods, theta0, 60.0)

                # Float32 LQR
                assert settle_ok(lqr32(), periods, theta0, 60.0)


class TestB5Relationships:
    """B5: Budget constants satisfy documented relationships."""

    def test_b5_jitter_budget_relationship(self):
        """JITTER_BUDGET_S <= MEASURED_MAX_JITTER_S * JITTER_SAFETY_FACTOR + 1e-12."""
        assert JITTER_BUDGET_S <= MEASURED_MAX_JITTER_S * JITTER_SAFETY_FACTOR + 1e-12

    def test_b5_jitter_required(self):
        """JITTER_BUDGET_S >= REQUIRED_JITTER_S."""
        assert JITTER_BUDGET_S >= REQUIRED_JITTER_S

    def test_b5_period_budget_relationship(self):
        """SUSTAINED_PERIOD_BUDGET_S <= MEASURED_MAX_CONSTANT_PERIOD_S * PERIOD_SAFETY_FACTOR + 1e-12."""
        assert SUSTAINED_PERIOD_BUDGET_S <= MEASURED_MAX_CONSTANT_PERIOD_S * PERIOD_SAFETY_FACTOR + 1e-12

    def test_b5_max_dropped_ticks_relationship(self):
        """MAX_DROPPED_TICKS_BUDGET == MEASURED_MAX_DROPPED_TICKS // 2."""
        assert MAX_DROPPED_TICKS_BUDGET == MEASURED_MAX_DROPPED_TICKS // 2

    def test_b5_max_period_budget(self):
        """MAX_PERIOD_BUDGET_S == approx(NOMINAL_PERIOD_S + JITTER_BUDGET_S)."""
        assert MAX_PERIOD_BUDGET_S == pytest.approx(NOMINAL_PERIOD_S + JITTER_BUDGET_S)

    def test_b5_sustained_above_nominal(self):
        """SUSTAINED_PERIOD_BUDGET_S > NOMINAL_PERIOD_S."""
        assert SUSTAINED_PERIOD_BUDGET_S > NOMINAL_PERIOD_S


class TestB6DocstringCarriesNumbers:
    """B6: jitter_budget.py docstring contains documented values and disclaimers."""

    def test_b6_docstring_contains_numbers(self):
        """jb.__doc__ contains '27 ms', '21.5 ms', 'R3', 'PID stability is untested', 'not a measurement of ESP32-C3'."""
        doc = jb.__doc__
        assert doc is not None
        assert "27 ms" in doc
        assert "21.5 ms" in doc
        assert "R3" in doc
        assert "PID stability is untested" in doc
        assert "not a measurement of ESP32-C3" in doc


class TestB7Float32Agreement:
    """B7: Float32 core agrees with LQR core on max_stable_constant_period and max_dropped_ticks."""

    def test_b7_float32_period_agreement(self):
        """max_stable_constant_period(lqr32) == max_stable_constant_period(lqr)."""
        ref_period = max_stable_constant_period(lqr)
        f32_period = max_stable_constant_period(lqr32)
        assert f32_period == ref_period

    def test_b7_float32_dropout_agreement(self):
        """max_dropped_ticks(lqr32) == max_dropped_ticks(lqr)."""
        ref_dropout = max_dropped_ticks(lqr)
        f32_dropout = max_dropped_ticks(lqr32)
        assert f32_dropout == ref_dropout


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
