#!/usr/bin/env python3
"""
Unit tests for odometry.py (issue #321, sub-issue of #301).

Pure Python, no numpy. No Webots, no hardware, must NOT `import controller` or
`serial`.

Reference numbers (radius 0.03 m): delta 1 rad at yaw 0 moves x by -0.03;
delta 2*pi moves x by -0.03 * 2 * pi = -0.18849556; delta 10 rad moves x by
-0.3. rim_direction(pi/2) = (-cos, -sin) = (~0, -1); rim_direction(pi) =
(1, ~0); rim_direction(-pi/2) = (~0, 1).

Usage:
    cd poc/servo-wheel-dof
    mamba run -n servo-wheel-dof python3 -m pytest -q \
        webots/controllers/servo_wheel_dof/test_swd_odometry.py
"""

from __future__ import annotations

import dataclasses
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from odometry import OdomState, rim_direction, step_odometry  # noqa: E402
from upright_pose import WHEEL_RADIUS  # noqa: E402

PI = math.pi
R = 0.03
BAD = [float("nan"), float("inf"), float("-inf")]


def _run(angles, yaw: float = 0.0, state: OdomState | None = None) -> OdomState:
    """Feed a sequence of angles at a fixed yaw, starting from `state`."""
    s = OdomState() if state is None else state
    for a in angles:
        s = step_odometry(s, a, yaw)
    return s


class TestDefaults:
    def test_wheel_radius_is_three_cm(self):
        assert WHEEL_RADIUS == R

    def test_default_state(self):
        s = OdomState()
        assert (s.x, s.y, s.prev_angle) == (0.0, 0.0, None)


class TestFirstCall:
    @pytest.mark.parametrize("angle", [0.0, 1.0, -2.5, 123.4])
    def test_first_call_never_moves(self, angle):
        s = step_odometry(OdomState(), angle, 0.7)
        assert s.x == 0.0
        assert s.y == 0.0
        assert s.prev_angle == angle


class TestStraightLine:
    def test_zero_to_one_radian(self):
        s = _run([0.0, 1.0])
        assert s.x == pytest.approx(-0.03)
        assert s.y == pytest.approx(0.0, abs=1e-12)
        assert s.prev_angle == 1.0

    def test_telescoping_sequence_same_total(self):
        whole = _run([0.0, 1.0])
        parts = _run([0.0, 0.5, 1.0])
        assert parts.x == pytest.approx(whole.x)
        assert parts.x == pytest.approx(-0.03)
        assert parts.y == pytest.approx(0.0, abs=1e-12)


class TestSign:
    def test_positive_delta_moves_x_negative(self):
        s = _run([0.0, 0.2])
        assert s.x < 0.0
        assert s.x == pytest.approx(-0.006)

    def test_negative_delta_moves_x_positive(self):
        s = _run([0.0, -0.2])
        assert s.x > 0.0
        assert s.x == pytest.approx(0.006)

    def test_there_and_back_returns_to_start(self):
        s = _run([0.0, 1.0, 0.0])
        assert s.x == pytest.approx(0.0, abs=1e-12)
        assert s.y == pytest.approx(0.0, abs=1e-12)


class TestHeading:
    def test_yaw_half_pi(self):
        s = _run([0.0, 1.0], yaw=PI / 2)
        assert s.x == pytest.approx(0.0, abs=1e-12)
        assert s.y == pytest.approx(-0.03)

    def test_yaw_pi(self):
        s = _run([0.0, 1.0], yaw=PI)
        assert s.x == pytest.approx(0.03)
        assert s.y == pytest.approx(0.0, abs=1e-12)

    def test_yaw_minus_half_pi(self):
        s = _run([0.0, 1.0], yaw=-PI / 2)
        assert s.x == pytest.approx(0.0, abs=1e-12)
        assert s.y == pytest.approx(0.03)

    def test_yaw_applies_to_each_step(self):
        s = _run([0.0, 1.0], yaw=0.0)
        s = _run([2.0], yaw=PI / 2, state=s)
        assert s.x == pytest.approx(-0.03)
        assert s.y == pytest.approx(-0.03)


class TestRimDirection:
    @pytest.mark.parametrize("yaw", [-3.0, -1.0, 0.0, 0.7, 2.5])
    def test_unit_and_perpendicular_to_hinge_axis(self, yaw):
        ux, uy = rim_direction(yaw)
        assert math.hypot(ux, uy) == pytest.approx(1.0, abs=1e-12)
        axis = (math.sin(yaw), -math.cos(yaw))
        assert ux * axis[0] + uy * axis[1] == pytest.approx(0.0, abs=1e-12)

    def test_values(self):
        assert rim_direction(0.0) == (-1.0, -0.0)
        ux, uy = rim_direction(PI / 2)
        assert ux == pytest.approx(0.0, abs=1e-12)
        assert uy == pytest.approx(-1.0)
        ux, uy = rim_direction(PI)
        assert ux == pytest.approx(1.0)
        assert uy == pytest.approx(0.0, abs=1e-12)

    def test_returns_tuple_of_two_floats(self):
        d = rim_direction(0.3)
        assert isinstance(d, tuple)
        assert len(d) == 2
        assert all(isinstance(v, float) for v in d)


class TestRest:
    def test_no_drift_with_constant_angle(self):
        start = _run([0.0, 1.0], yaw=0.4)
        assert start.x != 0.0
        s = start
        for _ in range(1000):
            s = step_odometry(s, 1.0, 0.4)
        assert s.x == start.x
        assert s.y == start.y
        assert s.prev_angle == 1.0


class TestNonFinite:
    @pytest.mark.parametrize("bad", BAD)
    def test_bad_wheel_angle_returns_equal_state(self, bad):
        s0 = _run([0.0, 1.0])
        assert step_odometry(s0, bad, 0.3) == s0

    @pytest.mark.parametrize("bad", BAD)
    def test_bad_yaw_returns_equal_state(self, bad):
        s0 = _run([0.0, 1.0])
        assert step_odometry(s0, 2.0, bad) == s0

    @pytest.mark.parametrize("bad", BAD)
    def test_bad_before_first_call_keeps_none(self, bad):
        s = step_odometry(OdomState(), bad, 0.0)
        assert s == OdomState()
        assert s.prev_angle is None
        s = step_odometry(OdomState(), 1.0, bad)
        assert s == OdomState()

    @pytest.mark.parametrize("bad", BAD)
    def test_delta_accumulates_over_bad_angle_gap(self, bad):
        s = _run([0.0])
        s = step_odometry(s, bad, PI)
        assert s.prev_angle == 0.0
        s = step_odometry(s, 2.0, PI)
        assert s.x == pytest.approx(0.06)
        assert s.y == pytest.approx(0.0, abs=1e-12)

    @pytest.mark.parametrize("bad", BAD)
    def test_gap_delta_uses_later_yaw(self, bad):
        s = _run([0.0])
        s = step_odometry(s, 0.5, bad)
        s = step_odometry(s, 1.0, PI / 2)
        assert s.x == pytest.approx(0.0, abs=1e-12)
        assert s.y == pytest.approx(-0.03)


class TestBigStep:
    def test_ten_radians_literal(self):
        s = _run([0.0, 10.0])
        assert s.x == pytest.approx(-0.3)
        assert s.y == pytest.approx(0.0, abs=1e-12)

    def test_two_pi_not_wrapped(self):
        s = _run([0.0, 2 * PI])
        assert s.x == pytest.approx(-0.18849556, abs=1e-8)
        assert s.x == pytest.approx(-R * 2 * PI)


class TestImmutability:
    def test_assignment_raises(self):
        s = OdomState()
        with pytest.raises(dataclasses.FrozenInstanceError):
            s.x = 1.0  # type: ignore[misc]

    def test_step_does_not_mutate_input(self):
        s0 = OdomState(0.0, 0.0, 0.0)
        s1 = step_odometry(s0, 1.0, 0.0)
        assert s0 == OdomState(0.0, 0.0, 0.0)
        assert s1 is not s0
        assert s1.prev_angle == 1.0


class TestRadius:
    def test_custom_radius_scales_result(self):
        s = OdomState(0.0, 0.0, 0.0)
        custom = step_odometry(s, 1.0, 0.0, radius=0.05)
        assert custom.x == pytest.approx(-0.05)
        default = step_odometry(s, 1.0, 0.0)
        assert default.x == pytest.approx(-0.03)

    def test_radius_is_keyword_only(self):
        with pytest.raises(TypeError):
            step_odometry(OdomState(), 1.0, 0.0, 0.05)  # type: ignore[misc]
