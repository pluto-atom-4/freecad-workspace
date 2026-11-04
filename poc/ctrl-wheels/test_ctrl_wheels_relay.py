#!/usr/bin/env python3
"""
Unit tests for the ctrl-wheels relay (ctrl_wheels_relay), issue FW #450.

The module under test is the flat module ctrl_wheels_relay in poc/ctrl-wheels.
Every test uses an injected fake clock, never real time. SIM ONLY. The sign
convention is UNVERIFIED.

Usage:
    cd poc/ctrl-wheels
    python3 -m pytest test_ctrl_wheels_relay.py -v
"""

import dataclasses
import inspect
import math
import sys
import time
from pathlib import Path

import pytest

# Add the script directory to path for imports
sys.path.insert(0, str(Path(__file__).resolve().parent))

from ctrl_wheels_msg import Whl
from ctrl_wheels_relay import (
    CLAMP_RAD_S,
    POLL_S,
    STALE_S,
    Relay,
    RelayOut,
)


class FakeClock:
    """Injected monotonic clock. Only moves when the test advances it."""

    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt
        return self.t


def _whl(model=0.5, state=0, seq=0):
    """Build a Whl sample (raw and t_us are not used by the relay)."""
    return Whl(seq=seq, t_us=seq * 20000, model_rad_s=model, raw=0, state=state)


class TestConstants:
    """Test: pinned module constants."""

    def test_constants_pinned(self):
        assert STALE_S == 0.200
        assert POLL_S == 0.020
        assert CLAMP_RAD_S == 3.0


class TestDataclass:
    """Test: RelayOut is a frozen value object with the pinned field order."""

    def test_field_order(self):
        names = [f.name for f in dataclasses.fields(RelayOut)]
        assert names == ["wheel_rad_s", "stale"]

    def test_frozen(self):
        out = RelayOut(wheel_rad_s=0.0, stale=1)
        with pytest.raises(dataclasses.FrozenInstanceError):
            out.stale = 0  # type: ignore[misc]


class TestConstructor:
    """Test: constructor defaults and argument validation."""

    def test_defaults_pinned(self):
        params = inspect.signature(Relay.__init__).parameters
        assert params["clock"].default is time.monotonic
        assert params["stale_s"].default == STALE_S
        assert params["clamp"].default == CLAMP_RAD_S
        assert params["sign"].default == 1

    @pytest.mark.parametrize("sign", [0, 2, -2, 0.5, 1.0, -1.0, True])
    def test_sign_not_plus_or_minus_one_int_rejected(self, sign):
        with pytest.raises(ValueError):
            Relay(clock=FakeClock(), sign=sign)

    @pytest.mark.parametrize("stale_s", [0.0, -0.1])
    def test_stale_s_must_be_positive(self, stale_s):
        with pytest.raises(ValueError):
            Relay(clock=FakeClock(), stale_s=stale_s)

    @pytest.mark.parametrize("clamp", [0.0, -3.0])
    def test_clamp_must_be_positive(self, clamp):
        with pytest.raises(ValueError):
            Relay(clock=FakeClock(), clamp=clamp)


class TestNoData:
    """Test: nothing is sent before the first WHL arrives."""

    def test_poll_before_first_whl_is_none(self):
        clk = FakeClock()
        relay = Relay(clock=clk)
        assert relay.poll() is None

    def test_poll_before_first_whl_stays_none_as_time_passes(self):
        clk = FakeClock()
        relay = Relay(clock=clk)
        clk.advance(1.0)
        assert relay.poll() is None


class TestOnWhlType:
    """Test: on_whl accepts only Whl objects."""

    @pytest.mark.parametrize(
        "bad",
        [None, 1.0, "WHL,0,0,0.000000,0,0*60\n", {"model_rad_s": 0.5}, (0, 0, 0.5, 0, 0)],
        ids=["none", "float", "line_str", "dict", "tuple"],
    )
    def test_non_whl_rejected_with_type_error(self, bad):
        relay = Relay(clock=FakeClock())
        with pytest.raises(TypeError):
            relay.on_whl(bad)

    def test_rejected_whl_does_not_change_state(self):
        clk = FakeClock()
        relay = Relay(clock=clk)
        relay.on_whl(_whl(model=0.5, state=0))
        with pytest.raises(TypeError):
            relay.on_whl("not a whl")
        assert relay.poll() == RelayOut(wheel_rad_s=0.5, stale=0)


class TestHoldState:
    """Test: state 1 (hold) and state 2 (bus down) force a stale zero output."""

    @pytest.mark.parametrize("state", [1, 2])
    def test_nonzero_state_gives_stale_zero(self, state):
        clk = FakeClock()
        relay = Relay(clock=clk)
        relay.on_whl(_whl(model=2.0, state=state))
        assert relay.poll() == RelayOut(wheel_rad_s=0.0, stale=1)

    @pytest.mark.parametrize("state", [1, 2])
    def test_nonzero_state_ignores_sign_and_clamp(self, state):
        clk = FakeClock()
        relay = Relay(clock=clk, sign=-1, clamp=1.5)
        relay.on_whl(_whl(model=-3.5, state=state))
        assert relay.poll() == RelayOut(wheel_rad_s=0.0, stale=1)

    def test_state_one_then_time_passes_still_stale(self):
        clk = FakeClock()
        relay = Relay(clock=clk)
        relay.on_whl(_whl(model=1.0, state=1))
        clk.advance(0.125)
        assert relay.poll() == RelayOut(wheel_rad_s=0.0, stale=1)
        clk.advance(0.25)
        assert relay.poll() == RelayOut(wheel_rad_s=0.0, stale=1)


class TestStaleBoundary:
    """Test: age >= stale_s is stale; the boundary is exact."""

    def test_age_0_199_not_stale_default(self):
        clk = FakeClock(0.0)
        relay = Relay(clock=clk)
        relay.on_whl(_whl(model=0.75, state=0))
        clk.t = 0.199
        assert relay.poll() == RelayOut(wheel_rad_s=0.75, stale=0)

    def test_age_0_200_stale_default(self):
        clk = FakeClock(0.0)
        relay = Relay(clock=clk)
        relay.on_whl(_whl(model=0.75, state=0))
        clk.t = 0.200
        assert relay.poll() == RelayOut(wheel_rad_s=0.0, stale=1)

    def test_exact_boundary_with_representable_values(self):
        clk = FakeClock(0.0)
        relay = Relay(clock=clk, stale_s=0.25)
        relay.on_whl(_whl(model=0.75, state=0))
        clk.advance(0.125)
        assert relay.poll() == RelayOut(wheel_rad_s=0.75, stale=0)
        clk.advance(0.125)  # total age 0.25 exactly
        assert relay.poll() == RelayOut(wheel_rad_s=0.0, stale=1)

    def test_age_measured_from_latest_on_whl(self):
        clk = FakeClock(0.0)
        relay = Relay(clock=clk, stale_s=0.25)
        relay.on_whl(_whl(model=0.5, seq=0))
        clk.advance(0.125)
        relay.on_whl(_whl(model=0.6, seq=1))  # re-arms the arrival time
        clk.advance(0.125)  # age of latest sample is 0.125, first is 0.25
        assert relay.poll() == RelayOut(wheel_rad_s=0.6, stale=0)

    def test_stale_output_is_zero_with_stale_flag(self):
        clk = FakeClock(0.0)
        relay = Relay(clock=clk, stale_s=0.25)
        relay.on_whl(_whl(model=2.5, state=0))
        clk.advance(0.5)
        out = relay.poll()
        assert out == RelayOut(wheel_rad_s=0.0, stale=1)


class TestClampAndSign:
    """Test: sign flips before the clamp; clamp is symmetric at +-CLAMP_RAD_S."""

    @pytest.mark.parametrize(
        "model,sign,expected",
        [
            (1.0, 1, 1.0),
            (3.0, 1, 3.0),
            (3.5, 1, 3.0),
            (-3.0, 1, -3.0),
            (-3.5, 1, -3.0),
            (1.0, -1, -1.0),
            (-3.5, -1, 3.0),
            (3.5, -1, -3.0),
        ],
    )
    def test_sign_then_clamp(self, model, sign, expected):
        clk = FakeClock()
        relay = Relay(clock=clk, sign=sign)
        relay.on_whl(_whl(model=model, state=0))
        assert relay.poll() == RelayOut(wheel_rad_s=expected, stale=0)

    def test_custom_clamp_used(self):
        clk = FakeClock()
        relay = Relay(clock=clk, clamp=1.5)
        relay.on_whl(_whl(model=2.0, state=0))
        assert relay.poll() == RelayOut(wheel_rad_s=1.5, stale=0)

    def test_model_zero_fresh_is_not_stale(self):
        clk = FakeClock()
        relay = Relay(clock=clk)
        relay.on_whl(_whl(model=0.0, state=0))
        assert relay.poll() == RelayOut(wheel_rad_s=0.0, stale=0)

    def test_model_zero_fresh_with_sign_minus_is_not_stale(self):
        clk = FakeClock()
        relay = Relay(clock=clk, sign=-1)
        relay.on_whl(_whl(model=0.0, state=0))
        assert relay.poll() == RelayOut(wheel_rad_s=0.0, stale=0)


class TestNonFiniteModel:
    """Test: a fresh state-0 sample with a non-finite model is treated as a fault."""

    @pytest.mark.parametrize(
        "model", [math.nan, math.inf, -math.inf], ids=["nan", "inf", "neg_inf"]
    )
    @pytest.mark.parametrize("sign", [1, -1])
    def test_non_finite_model_gives_stale_zero(self, model, sign):
        clk = FakeClock()
        relay = Relay(clock=clk, sign=sign)
        relay.on_whl(_whl(model=model, state=0))
        assert relay.poll() == RelayOut(wheel_rad_s=0.0, stale=1)


class TestRecovery:
    """Test: a fresh state-0 WHL after a stale or hold period resumes output."""

    def test_fresh_state0_after_stale_resumes(self):
        clk = FakeClock(0.0)
        relay = Relay(clock=clk)
        relay.on_whl(_whl(model=0.5, state=0))
        clk.advance(0.25)
        assert relay.poll() == RelayOut(wheel_rad_s=0.0, stale=1)
        relay.on_whl(_whl(model=0.8, state=0, seq=1))
        assert relay.poll() == RelayOut(wheel_rad_s=0.8, stale=0)

    def test_state1_then_state0_resumes(self):
        clk = FakeClock()
        relay = Relay(clock=clk)
        relay.on_whl(_whl(model=0.5, state=1))
        assert relay.poll() == RelayOut(wheel_rad_s=0.0, stale=1)
        relay.on_whl(_whl(model=0.5, state=0, seq=1))
        assert relay.poll() == RelayOut(wheel_rad_s=0.5, stale=0)

    def test_state2_then_state0_resumes(self):
        clk = FakeClock()
        relay = Relay(clock=clk)
        relay.on_whl(_whl(model=0.5, state=2))
        assert relay.poll() == RelayOut(wheel_rad_s=0.0, stale=1)
        relay.on_whl(_whl(model=-0.5, state=0, seq=1))
        assert relay.poll() == RelayOut(wheel_rad_s=-0.5, stale=0)

    def test_resume_applies_sign_and_clamp_again(self):
        clk = FakeClock()
        relay = Relay(clock=clk, sign=-1)
        relay.on_whl(_whl(model=9.0, state=2))
        assert relay.poll() == RelayOut(wheel_rad_s=0.0, stale=1)
        relay.on_whl(_whl(model=9.0, state=0, seq=1))
        assert relay.poll() == RelayOut(wheel_rad_s=-3.0, stale=0)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
