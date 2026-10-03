#!/usr/bin/env python3
"""
Replay integration test for the odometry pieces (issue #324, sub-issue of #301).

Pure Python, no numpy. No Webots, no hardware, must NOT `import controller` or
`serial`. Imports only odometry, upright_pose and alignment_metrics.

Frames: world X east, Y north, Z up. upright_pose returns Robot translation T and
an axis-angle for R = Rz(yaw) Ry(pitch) Rx(pi/2). The wheel centre is
T + R*(0, 0, 0.026). Column 2 of R is (sin yaw, -cos yaw, 0), so the centre is
c0 + (state.x, state.y) with c0 = (0, -0.026, 0.03). Orientations are 9 floats,
row-major, as Webots getOrientation().

Hand-checked numbers (radius 0.03 m):
- Ramp 0 -> 2.0 at yaw 0: offset (-0.06, 0). At yaw 0.8: 0.06 * (-cos 0.8,
  -sin 0.8) = (-0.041803, -0.043041). A downward ramp gives the negative.
- Sinusoid (mock): offset p = r * sum(dtheta_k * u_k), u = -(cos yaw, sin yaw).
  Over the 20 s window theta has 5 periods and yaw exactly 1, so the continuous
  integral is the 5th Fourier coefficient of u against cos: exactly 0
  (Jacobi-Anger: cos(0.8 sin) has only even harmonics, sin(0.8 sin) has
  harmonic 5 in sin(5 phi) form, orthogonal to cos(5 phi)). Sampling error is
  <= r * max|yaw'| * h * TV(theta) = 0.03 * 0.2513 * 0.02 * 30 = 0.0045, so the
  end is within 0.01 m (end yaw is ~2e-15, so the upright T equals its start).
- Robot distance from start: offset <= 0.093 (Abel summation bound) plus the
  upright part <= 0.0203, so <= 0.12 is provable. The issue's 0.06 is not
  provable for the Robot position T (estimate ~0.06 at t = 5 s).

Usage:
    cd poc/servo-wheel-dof
    mamba run -n servo-wheel-dof python3 -m pytest -q \
        webots/controllers/servo_wheel_dof/test_swd_odometry_replay.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from alignment_metrics import anchor_metrics, relative_position  # noqa: E402
from odometry import (  # noqa: E402
    OFFSET_LIMIT_M,
    OdomState,
    apply_offset,
    rim_direction,
    step_odometry,
)
from upright_pose import WHEEL_OFFSET, WHEEL_RADIUS, upright_pose  # noqa: E402

TWO_PI = 2.0 * math.pi
C0 = (0.0, -WHEEL_OFFSET)  # wheel centre start (x, y)


def _rot9(axis_angle) -> tuple[float, ...]:
    """Rodrigues: axis-angle (x, y, z, angle) -> 9 floats, row-major."""
    kx, ky, kz, phi = axis_angle
    c = math.cos(phi)
    s = math.sin(phi)
    v = 1.0 - c
    return (
        c + kx * kx * v,
        kx * ky * v - kz * s,
        kx * kz * v + ky * s,
        ky * kx * v + kz * s,
        c + ky * ky * v,
        ky * kz * v - kx * s,
        kz * kx * v - ky * s,
        kz * ky * v + kx * s,
        c + kz * kz * v,
    )


def _pose(pitch: float, yaw: float, state: OdomState):
    """Return (T, R9, wheel centre) with the odometry offset applied to T."""
    translation, axis_angle = upright_pose(pitch, yaw)
    t = apply_offset(translation, state)
    rot = _rot9(axis_angle)
    centre = tuple(
        t[i] + WHEEL_OFFSET * rot[3 * i + 2] for i in range(3)
    )
    return t, rot, centre


def _replay(samples, limit=OFFSET_LIMIT_M):
    """Feed (angle, pitch, yaw) samples; return a list of (state, T, R9, centre)."""
    out = []
    state = OdomState()
    for angle, pitch, yaw in samples:
        state = step_odometry(state, angle, yaw, limit=limit)
        t, rot, centre = _pose(pitch, yaw, state)
        out.append((state, t, rot, centre))
    return out


def _ramp(a0: float, a1: float, yaw: float, n: int = 100):
    """n equal steps from a0 to a1 (n + 1 samples), pitch 0, fixed yaw."""
    return [(a0 + (a1 - a0) * i / n, 0.0, yaw) for i in range(n + 1)]


def _mock(seconds: int = 20, rate: int = 50):
    """Mock-like samples: angle 1.5 sin, yaw 0.8 sin, pitch 0.3 sin."""
    out = []
    for k in range(seconds * rate + 1):
        t = k / float(rate)
        angle = 1.5 * math.sin(TWO_PI * 0.25 * t)
        yaw = 0.8 * math.sin(TWO_PI * 0.05 * t)
        pitch = 0.3 * math.sin(TWO_PI * 0.1 * t)
        out.append((angle, pitch, yaw))
    return out


def _check_anchor(pitch: float, yaw: float, state: OdomState) -> None:
    """Anchor invariants with odometry equal those without, and are healthy."""
    t, rot, centre = _pose(pitch, yaw, state)
    m = anchor_metrics(t, rot, centre, rot)
    assert m.deviation_m < 1e-9
    assert m.axis_dot == pytest.approx(1.0, abs=1e-12)
    assert m.warn is False
    rel = relative_position(t, rot, centre)
    assert rel == pytest.approx((0.0, 0.0, WHEEL_OFFSET), abs=1e-12)
    # Invariance: same pitch and yaw without odometry.
    t0, rot0, centre0 = _pose(pitch, yaw, OdomState())
    m0 = anchor_metrics(t0, rot0, centre0, rot0)
    assert m.deviation_m == pytest.approx(m0.deviation_m, abs=1e-12)
    assert m.axis_dot == pytest.approx(m0.axis_dot, abs=1e-12)
    assert m.warn == m0.warn
    rel0 = relative_position(t0, rot0, centre0)
    assert rel == pytest.approx(rel0, abs=1e-12)


class TestRodriguesHelper:
    def test_yaw0_pitch0_hinge_axis_is_minus_y(self):
        rot = _rot9(upright_pose(0.0, 0.0)[1])
        col2 = (rot[2], rot[5], rot[8])
        assert col2 == pytest.approx((0.0, -1.0, 0.0), abs=1e-12)

    @pytest.mark.parametrize("yaw", [-2.0, 0.0, 0.8, 3.0])
    @pytest.mark.parametrize("pitch", [-0.3, 0.0, 0.3])
    def test_column2_matches_closed_form(self, pitch, yaw):
        rot = _rot9(upright_pose(pitch, yaw)[1])
        col2 = (rot[2], rot[5], rot[8])
        expect = (math.sin(yaw), -math.cos(yaw), 0.0)
        assert col2 == pytest.approx(expect, abs=1e-12)


class TestRamp:
    def test_yaw0_forward_ramp_moves_minus_x(self):
        rows = _replay(_ramp(0.0, 2.0, 0.0))
        t0 = upright_pose(0.0, 0.0)[0]
        t = rows[-1][1]
        assert t[0] - t0[0] == pytest.approx(-0.06, abs=1e-12)
        assert t[1] - t0[1] == pytest.approx(0.0, abs=1e-12)
        for _, tt, _, _ in rows:
            assert tt[2] == 0.03

    def test_yaw0_ramp_is_monotonic_toward_minus_x(self):
        xs = [row[1][0] for row in _replay(_ramp(0.0, 2.0, 0.0))]
        assert all(b <= a + 1e-15 for a, b in zip(xs, xs[1:]))

    def test_yaw0_backward_ramp_moves_plus_x(self):
        rows = _replay(_ramp(2.0, 0.0, 0.0))
        t0 = upright_pose(0.0, 0.0)[0]
        assert rows[-1][1][0] - t0[0] == pytest.approx(0.06, abs=1e-12)
        assert rows[-1][1][1] - t0[1] == pytest.approx(0.0, abs=1e-12)

    def test_yaw08_displacement_along_rim_direction(self):
        yaw = 0.8
        rows = _replay(_ramp(0.0, 2.0, yaw))
        t0 = upright_pose(0.0, yaw)[0]
        t = rows[-1][1]
        dx, dy = t[0] - t0[0], t[1] - t0[1]
        ux, uy = rim_direction(yaw)
        assert math.hypot(dx, dy) == pytest.approx(0.06, abs=1e-12)
        assert dx == pytest.approx(0.06 * ux, abs=1e-12)
        assert dy == pytest.approx(0.06 * uy, abs=1e-12)

    def test_yaw08_downward_ramp_flips_sign(self):
        yaw = 0.8
        rows = _replay(_ramp(2.0, 0.0, yaw))
        t0 = upright_pose(0.0, yaw)[0]
        t = rows[-1][1]
        dx, dy = t[0] - t0[0], t[1] - t0[1]
        ux, uy = rim_direction(yaw)
        assert math.hypot(dx, dy) == pytest.approx(0.06, abs=1e-12)
        assert dx == pytest.approx(-0.06 * ux, abs=1e-12)
        assert dy == pytest.approx(-0.06 * uy, abs=1e-12)


class TestSinusoid:
    def test_ends_within_one_centimetre_of_start(self):
        # Provable: continuous integral is 0, sampling error <= 0.0045 m.
        rows = _replay(_mock())
        t_start = rows[0][1]
        t_end = rows[-1][1]
        dist = math.hypot(t_end[0] - t_start[0], t_end[1] - t_start[1])
        assert dist < 0.01

    def test_never_far_from_start(self):
        # Provable bound 0.113 (offset <= 0.093 + upright part <= 0.0203).
        rows = _replay(_mock())
        t_start = rows[0][1]
        for _, t, _, _ in rows:
            assert math.hypot(t[0] - t_start[0], t[1] - t_start[1]) <= 0.12

    def test_wheel_offset_stays_within_issue_bound(self):
        # Issue intent: |offset| <= 0.06. Estimated ~0.045; not rigorously
        # proven (rigorous bound is 0.093). Relax to 0.07 if this ever fails.
        for state, _, _, _ in _replay(_mock()):
            assert math.hypot(state.x, state.y) <= 0.06

    def test_not_clamped_and_inside_box(self):
        for state, _, _, _ in _replay(_mock()):
            assert state.clamped is False
            assert abs(state.x) < OFFSET_LIMIT_M
            assert abs(state.y) < OFFSET_LIMIT_M


class TestRimStaysOnFloor:
    @pytest.mark.parametrize(
        "samples",
        [
            _ramp(0.0, 2.0, 0.0),
            _ramp(0.0, 2.0, 0.8),
            _ramp(0.0, 100.0, 0.8),
            _mock(),
        ],
        ids=["ramp-yaw0", "ramp-yaw08", "ramp-clamped", "mock"],
    )
    def test_centre_on_floor_and_tracks_offset(self, samples):
        for state, t, _, centre in _replay(samples):
            assert t[2] == WHEEL_RADIUS
            assert centre[2] == pytest.approx(WHEEL_RADIUS, abs=1e-12)
            assert centre[0] == pytest.approx(C0[0] + state.x, abs=1e-12)
            assert centre[1] == pytest.approx(C0[1] + state.y, abs=1e-12)


class TestAnchorMetrics:
    @pytest.mark.parametrize("state", [OdomState(), OdomState(x=0.4, y=-0.7)])
    @pytest.mark.parametrize("yaw", [-2.0, 0.0, 0.8])
    @pytest.mark.parametrize("pitch", [-0.3, 0.0, 0.3])
    def test_fixed_poses(self, pitch, yaw, state):
        _check_anchor(pitch, yaw, state)

    def test_every_50th_mock_step(self):
        samples = _mock()
        rows = _replay(samples)
        for i in range(0, len(rows), 50):
            _, pitch, yaw = samples[i]
            _check_anchor(pitch, yaw, rows[i][0])

    def test_final_mock_step(self):
        samples = _mock()
        rows = _replay(samples)
        _, pitch, yaw = samples[-1]
        _check_anchor(pitch, yaw, rows[-1][0])


class TestIdle:
    def test_constant_angle_changing_yaw_leaves_zero_offset(self):
        samples = [(0.7, 0.0, 0.05 * i) for i in range(201)]
        rows = _replay(samples)
        for state, t, _, _ in rows:
            assert state.x == 0.0
            assert state.y == 0.0
            assert state.clamped is False
        for (_, pitch, yaw), (_, t, _, _) in zip(samples, rows):
            assert t == upright_pose(pitch, yaw)[0]

    def test_idle_anchor_invariants(self):
        samples = [(0.7, 0.0, 0.05 * i) for i in range(201)]
        rows = _replay(samples)
        _, pitch, yaw = samples[-1]
        _check_anchor(pitch, yaw, rows[-1][0])


class TestClamp:
    def test_100_rad_ramp_yaw0_ends_on_box(self):
        rows = _replay(_ramp(0.0, 100.0, 0.0))
        state = rows[-1][0]
        assert state.x == -0.9
        assert state.y == pytest.approx(0.0, abs=1e-12)
        assert state.clamped is True
        assert rows[-1][1][0] == pytest.approx(-0.9, abs=1e-12)
        for s, _, _, _ in rows:
            assert abs(s.x) <= 0.9
            assert abs(s.y) <= 0.9

    def test_100_rad_ramp_yaw08_clamps_both_axes(self):
        rows = _replay(_ramp(0.0, 100.0, 0.8))
        state = rows[-1][0]
        assert state.x == -0.9
        assert state.y == -0.9
        assert state.clamped is True

    @pytest.mark.parametrize("yaw", [0.0, 0.8])
    def test_anchor_invariants_hold_when_clamped(self, yaw):
        rows = _replay(_ramp(0.0, 100.0, yaw))
        _check_anchor(0.0, yaw, rows[-1][0])
        for i in (30, 60, 100):
            _check_anchor(0.0, yaw, rows[i][0])
