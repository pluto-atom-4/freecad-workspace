#!/usr/bin/env python3
"""
Unit tests for the floor clamp and apply_offset in odometry.py (issue #322,
sub-issue of #301).

Pure Python, no numpy. No Webots, no hardware, must NOT `import controller` or
`serial`.

Reference numbers (radius 0.03 m, limit 0.9 m): angle 0 -> 100 at yaw 0 would
move x by -3.0, so x is clamped to exactly -0.9. Reversing by -1.0 rad then moves
x by +0.03 to -0.87. At yaw pi/4 a delta of 40 rad moves each axis by
-1.2 * cos(pi/4) = -0.848528 (inside the box although the radius 1.2 > 0.9).

Usage:
    cd poc/servo-wheel-dof
    mamba run -n servo-wheel-dof python3 -m pytest -q \
        webots/controllers/servo_wheel_dof/test_swd_odometry_clamp.py
"""

from __future__ import annotations

import dataclasses
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from odometry import (  # noqa: E402
    OFFSET_LIMIT_M,
    OdomState,
    apply_offset,
    step_odometry,
)
from upright_pose import WHEEL_OFFSET, WHEEL_RADIUS, upright_pose  # noqa: E402

PI = math.pi
LIM = 0.9
BAD = [float("nan"), float("inf"), float("-inf")]
BAD_LIMITS = [0, 0.0, -0.9, -1, float("nan"), float("inf"), float("-inf")]


def _run(angles, yaw: float = 0.0, limit=LIM, state: OdomState | None = None):
    """Feed angles at a fixed yaw with a clamp limit, starting from `state`."""
    s = OdomState() if state is None else state
    for a in angles:
        s = step_odometry(s, a, yaw, limit=limit)
    return s


class TestDefaults:
    def test_offset_limit_constant(self):
        assert OFFSET_LIMIT_M == 0.9

    def test_clamped_is_last_field_default_false(self):
        fields = dataclasses.fields(OdomState)
        assert fields[-1].name == "clamped"
        assert fields[-1].default is False
        assert OdomState().clamped is False

    def test_limit_is_keyword_only(self):
        with pytest.raises(TypeError):
            step_odometry(OdomState(), 1.0, 0.0, WHEEL_RADIUS, 0.9)  # type: ignore


class TestClamp:
    def test_large_forward_clamps_x_exactly(self):
        s = _run([0.0, 100.0])
        assert s.x == -0.9
        assert s.y == 0.0
        assert s.clamped is True
        assert s.prev_angle == 100.0

    def test_default_constant_as_limit(self):
        s = _run([0.0, 100.0], limit=OFFSET_LIMIT_M)
        assert s.x == -0.9
        assert s.clamped is True

    def test_large_backward_clamps_to_plus_limit(self):
        s = _run([0.0, -100.0])
        assert s.x == 0.9
        assert s.clamped is True

    def test_inside_box_not_clamped(self):
        s = _run([0.0, 1.0])
        assert s.x == pytest.approx(-0.03)
        assert s.clamped is False

    def test_first_call_not_clamped(self):
        s = step_odometry(OdomState(), 50.0, 0.0, limit=LIM)
        assert s == OdomState(0.0, 0.0, 50.0)
        assert s.clamped is False

    def test_limit_none_never_clamps(self):
        s = _run([0.0, 100.0], limit=None)
        assert s.x == pytest.approx(-3.0)
        assert s.x < -0.9
        assert s.clamped is False

    def test_limit_none_is_default(self):
        s = step_odometry(OdomState(0.0, 0.0, 0.0), 100.0, 0.0)
        assert s.x == pytest.approx(-3.0)
        assert s.clamped is False

    def test_integer_limit_accepted(self):
        s = _run([0.0, 100.0], limit=1)
        assert s.x == -1.0
        assert s.clamped is True


class TestExactLimit:
    def test_landing_exactly_on_limit_is_not_clamped(self):
        # radius 0.5, delta 1 rad, yaw 0: x = 0.5 * 1 * (-1) = -0.5 exactly.
        s0 = OdomState(0.0, 0.0, 0.0)
        s = step_odometry(s0, 1.0, 0.0, radius=0.5, limit=0.5)
        assert s.x == -0.5
        assert s.clamped is False

    def test_past_the_limit_by_next_step_is_clamped(self):
        s0 = OdomState(0.0, 0.0, 0.0)
        s = step_odometry(s0, 1.0, 0.0, radius=0.5, limit=0.5)
        s = step_odometry(s, 2.0, 0.0, radius=0.5, limit=0.5)
        assert s.x == -0.5
        assert s.clamped is True

    def test_standing_on_the_limit_with_zero_delta_not_clamped(self):
        s = _run([0.0, 100.0])
        assert s.clamped is True
        s = step_odometry(s, 100.0, 0.0, limit=LIM)
        assert s.x == -0.9
        assert s.clamped is False


class TestNoWindup:
    def test_reversal_moves_back_immediately(self):
        s = _run([0.0, 100.0])
        assert s.x == -0.9
        s = step_odometry(s, 99.0, 0.0, limit=LIM)
        assert s.x == pytest.approx(-0.87)
        assert s.y == pytest.approx(0.0, abs=1e-12)
        assert s.clamped is False
        assert s.prev_angle == 99.0

    def test_prev_angle_follows_command_while_clamped(self):
        s = _run([0.0, 100.0, 200.0])
        assert s.prev_angle == 200.0
        assert s.x == -0.9
        assert s.clamped is True

    def test_reversal_after_long_overshoot(self):
        s = _run([0.0, 500.0, 499.0])
        assert s.x == pytest.approx(-0.87)
        assert s.clamped is False


class TestPerAxis:
    def test_diagonal_clamps_both_axes(self):
        s = _run([0.0, 100.0], yaw=PI / 4)
        assert s.x == -0.9
        assert s.y == -0.9
        assert s.clamped is True

    def test_box_not_circle(self):
        # radius * delta = 1.2 > 0.9 but each component is 0.8485 < 0.9.
        s = _run([0.0, 40.0], yaw=PI / 4)
        assert s.x == pytest.approx(-1.2 * math.cos(PI / 4))
        assert s.y == pytest.approx(-1.2 * math.sin(PI / 4))
        assert math.hypot(s.x, s.y) > 0.9
        assert s.clamped is False

    def test_only_x_clamped(self):
        s = _run([0.0, 40.0], yaw=0.3)
        assert s.x == -0.9
        assert s.y == pytest.approx(-1.2 * math.sin(0.3))
        assert s.clamped is True

    def test_only_y_clamped(self):
        s = _run([0.0, 40.0], yaw=PI / 2)
        assert s.x == pytest.approx(0.0, abs=1e-12)
        assert s.y == -0.9
        assert s.clamped is True

    @pytest.mark.parametrize("yaw", [-3.0, -PI / 4, 0.0, 0.3, PI / 4, 2.0, PI])
    @pytest.mark.parametrize("delta", [-500.0, -40.0, 40.0, 500.0])
    def test_never_leaves_box(self, yaw, delta):
        s = _run([0.0, delta, 0.0, delta], yaw=yaw)
        assert abs(s.x) <= 0.9
        assert abs(s.y) <= 0.9


class TestInvalidLimit:
    @pytest.mark.parametrize("bad", BAD_LIMITS)
    def test_bad_limit_raises_on_first_call(self, bad):
        with pytest.raises(ValueError):
            step_odometry(OdomState(), 1.0, 0.0, limit=bad)

    @pytest.mark.parametrize("bad", BAD_LIMITS)
    def test_bad_limit_raises_on_normal_step(self, bad):
        with pytest.raises(ValueError):
            step_odometry(OdomState(0.0, 0.0, 0.0), 1.0, 0.0, limit=bad)

    @pytest.mark.parametrize("bad", BAD_LIMITS)
    def test_bad_limit_raises_even_with_nan_angle(self, bad):
        with pytest.raises(ValueError):
            step_odometry(OdomState(), float("nan"), 0.0, limit=bad)


class TestNonFiniteInputs:
    @pytest.mark.parametrize("bad", BAD)
    def test_bad_angle_returns_equal_clamped_state(self, bad):
        s0 = _run([0.0, 100.0])
        assert s0.clamped is True
        s1 = step_odometry(s0, bad, 0.0, limit=LIM)
        assert s1 == s0
        assert s1.clamped is True

    @pytest.mark.parametrize("bad", BAD)
    def test_bad_yaw_returns_equal_state(self, bad):
        s0 = _run([0.0, 1.0])
        assert step_odometry(s0, 2.0, bad, limit=LIM) == s0

    @pytest.mark.parametrize("bad", BAD)
    def test_bad_input_before_first_call(self, bad):
        assert step_odometry(OdomState(), bad, 0.0, limit=LIM) == OdomState()
        assert step_odometry(OdomState(), 1.0, bad, limit=LIM) == OdomState()


class TestApplyOffset:
    def test_adds_xy_and_keeps_z(self):
        a, b = 0.5, -0.25
        out = apply_offset((a, b, 0.03), OdomState(x=0.1, y=-0.2))
        assert out == (a + 0.1, b - 0.2, 0.03)
        assert out[2] == 0.03

    def test_zero_state_is_identity(self):
        t = (0.12, -0.34, 0.03)
        assert apply_offset(t, OdomState()) == t

    def test_returns_tuple_of_three(self):
        out = apply_offset((0.0, 0.0, 0.03), OdomState(x=1.0, y=2.0))
        assert isinstance(out, tuple)
        assert len(out) == 3

    @pytest.mark.parametrize("pitch", [-0.5, 0.0, 0.7])
    @pytest.mark.parametrize("yaw", [-2.0, 0.0, 1.3])
    def test_rim_contact_invariant_with_real_upright_pose(self, pitch, yaw):
        t = upright_pose(pitch, yaw)[0]
        out = apply_offset(t, OdomState(x=0.4, y=-0.7))
        assert out[2] == t[2]
        assert out[0] == t[0] + 0.4
        assert out[1] == t[1] - 0.7

    @pytest.mark.parametrize("yaw", [-2.0, 0.0, 1.3, PI / 2])
    def test_wheel_centre_closed_form(self, yaw):
        # Derivation in upright_pose.py: T = c0 - R*a with
        # R*a = wo * (sin yaw, -cos yaw, 0) and c0 = (0, -wo, radius).
        wo = WHEEL_OFFSET
        tx = -wo * math.sin(yaw)
        ty = -wo + wo * math.cos(yaw)
        state = OdomState(x=0.31, y=-0.42)
        out = apply_offset((tx, ty, WHEEL_RADIUS), state)
        cx = out[0] + wo * math.sin(yaw)
        cy = out[1] - wo * math.cos(yaw)
        assert cx == pytest.approx(0.31, abs=1e-12)
        assert cy == pytest.approx(-wo - 0.42, abs=1e-12)
        assert out[2] == WHEEL_RADIUS

    @pytest.mark.parametrize("yaw", [-2.0, 0.0, 1.3])
    def test_closed_form_matches_real_upright_pose(self, yaw):
        t = upright_pose(0.2, yaw)[0]
        wo = WHEEL_OFFSET
        out = apply_offset(t, OdomState(x=0.31, y=-0.42))
        assert out[0] + wo * math.sin(yaw) == pytest.approx(0.31, abs=1e-12)
        assert out[1] - wo * math.cos(yaw) == pytest.approx(-wo - 0.42, abs=1e-12)
