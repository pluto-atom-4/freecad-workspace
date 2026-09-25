#!/usr/bin/env python3
"""
Unit tests for 6-DOF IMU complementary filter fusion module.

Pure Python / pytest, no external dependencies beyond pytest.

Usage:
    cd poc/esp32-dof
    mamba run -n esp32-dof python3 -m pytest monitor/test_esp32dof_fusion.py -v
"""

import sys
from pathlib import Path

import math
import pytest

# Add the script directory to path for imports
sys.path.insert(0, str(Path(__file__).resolve().parent))

from dof_fusion import ComplementaryFilter


class TestStaticFlat:
    """Test: filter on flat, stationary device (zero gyro, level accel)."""

    def test_static_flat_converges(self):
        """100 calls with flat accel (0,0,1) and zero gyro → roll,pitch ≈ 0."""
        fuse = ComplementaryFilter(alpha=0.98)
        accel_g = (0.0, 0.0, 1.0)
        gyro_dps = (0.0, 0.0, 0.0)
        dt_s = 0.02

        for _ in range(100):
            roll, pitch, yaw = fuse.update(accel_g, gyro_dps, dt_s)

        assert abs(roll) < 1e-9
        assert abs(pitch) < 1e-9


class TestFirstCallInit:
    """Test: first call initialization from accelerometer."""

    def test_first_call_init_tilt(self):
        """Single call with tilted accel → roll ≈ arctan(ay/az)."""
        fuse = ComplementaryFilter(alpha=0.98)
        # Tilt 30°: ay = sin(30°) = 0.5, az = cos(30°) ≈ 0.8660254
        accel_g = (0.0, 0.5, 0.8660254)
        gyro_dps = (0.0, 0.0, 0.0)
        dt_s = 0.02

        roll, pitch, yaw = fuse.update(accel_g, gyro_dps, dt_s)

        # Expected: roll ≈ atan2(0.5, 0.8660254) ≈ 0.5235988 (30° in radians)
        assert abs(roll - 0.5235988) < 1e-6
        assert abs(pitch) < 1e-6
        assert abs(yaw) < 1e-6


class TestTiltConvergence:
    """Test: tilt convergence blending."""

    def test_tilt_convergence(self):
        """Flat init, then tilted accel (zero gyro). Check convergence over updates."""
        fuse = ComplementaryFilter(alpha=0.98)

        # First call: flat
        roll, pitch, yaw = fuse.update((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), 0.02)
        assert abs(roll) < 1e-9
        assert abs(pitch) < 1e-9

        # Apply tilted accel (30° tilt), zero gyro
        accel_tilted = (0.0, 0.5, 0.8660254)
        gyro_zero = (0.0, 0.0, 0.0)
        dt_s = 0.02

        # After first tilted update, roll should be small but > 0
        roll, pitch, yaw = fuse.update(accel_tilted, gyro_zero, dt_s)
        assert 0.0 < roll < 0.05

        # After 299 more tilted updates (300 total), roll should be close to 30°
        for _ in range(299):
            roll, pitch, yaw = fuse.update(accel_tilted, gyro_zero, dt_s)

        expected_roll = 0.5235988
        assert abs(roll - expected_roll) < 0.02


class TestPitchSign:
    """Test: pitch estimation sign and magnitude."""

    def test_pitch_sign_from_ax(self):
        """Negative ax → positive pitch. ax=-sin(20°), az=cos(20°) → pitch ≈ 0.34907."""
        fuse = ComplementaryFilter(alpha=0.98)
        # Tilt pitch by 20° forward: ax = -sin(20°) ≈ -0.342020, az = cos(20°) ≈ 0.939693
        accel_g = (-0.342020, 0.0, 0.939693)
        gyro_dps = (0.0, 0.0, 0.0)
        dt_s = 0.02

        roll, pitch, yaw = fuse.update(accel_g, gyro_dps, dt_s)

        # Expected: pitch ≈ atan2(-ax, sqrt(ay²+az²)) = atan2(0.342020, 0.939693) ≈ 0.34907
        assert abs(roll) < 1e-6
        assert abs(pitch - 0.34907) < 5e-6


class TestGyroMapping:
    """Test: gyroscope integration."""

    def test_gyro_roll_mapping(self):
        """Init flat, single update gx=30 dps dt=0.02 accel flat → roll ≈ 0.0102625."""
        fuse = ComplementaryFilter(alpha=0.98)
        # Init flat
        fuse.update((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), 0.02)

        # Single update: gx=30 dps, dt=0.02, accel flat
        # With alpha=0.98, roll = pred_roll * alpha (since roll_acc=0 for flat accel)
        roll_expected = math.radians(30.0) * 0.02 * 0.98
        roll, pitch, yaw = fuse.update((0.0, 0.0, 1.0), (30.0, 0.0, 0.0), 0.02)

        assert abs(roll - roll_expected) < 1e-10
        assert abs(pitch) < 1e-6

    def test_gyro_pitch_mapping(self):
        """Init flat, single update gy=30 dps dt=0.02 accel flat → pitch ≈ 0.0102625."""
        fuse = ComplementaryFilter(alpha=0.98)
        # Init flat
        fuse.update((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), 0.02)

        # Single update: gy=30 dps, dt=0.02, accel flat
        # With alpha=0.98, pitch = pred_pitch * alpha (since pitch_acc=0 for flat accel)
        pitch_expected = math.radians(30.0) * 0.02 * 0.98
        roll, pitch, yaw = fuse.update((0.0, 0.0, 1.0), (0.0, 30.0, 0.0), 0.02)

        assert abs(pitch - pitch_expected) < 1e-10
        assert abs(roll) < 1e-6


class TestYawIntegration:
    """Test: yaw gyro integration."""

    def test_yaw_integration_50_updates(self):
        """Init flat with gz=90 dps, then 49 more calls (50 total), dt=0.02 each.
        First call integrates yaw. Total: 50 * 90 * π/180 * 0.02 = π/2."""
        fuse = ComplementaryFilter(alpha=0.98)
        gyro_90 = (0.0, 0.0, 90.0)
        accel_flat = (0.0, 0.0, 1.0)
        dt_s = 0.02

        # 50 total updates
        roll, pitch, yaw = None, None, None
        for _ in range(50):
            roll, pitch, yaw = fuse.update(accel_flat, gyro_90, dt_s)

        # Expected: yaw ≈ 50 * π/180 * 90 * 0.02 = 50 * 0.031416 ≈ 1.5708 (π/2)
        expected_yaw = math.pi / 2.0
        assert abs(yaw - expected_yaw) < 0.02


class TestZeroDtAndNegativeDt:
    """Test: dt=0 and dt<0 after initialization."""

    def test_zero_dt_no_update(self):
        """dt_s=0 after init: returned tuple equals previous tuple."""
        fuse = ComplementaryFilter(alpha=0.98)
        # Init
        fuse.update((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), 0.02)
        state_before = fuse.update((0.0, 0.0, 1.0), (30.0, 0.0, 0.0), 0.02)
        # Call with dt=0
        state_after = fuse.update((0.0, 0.0, 1.0), (90.0, 0.0, 0.0), 0.0)

        assert state_after == state_before

    def test_negative_dt_no_update(self):
        """dt_s=-1 after init: returned tuple equals previous tuple."""
        fuse = ComplementaryFilter(alpha=0.98)
        # Init
        fuse.update((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), 0.02)
        state_before = fuse.update((0.0, 0.0, 1.0), (30.0, 0.0, 0.0), 0.02)
        # Call with dt < 0
        state_after = fuse.update((0.0, 0.0, 1.0), (90.0, 0.0, 0.0), -1.0)

        assert state_after == state_before

    def test_first_call_dt_zero_initializes_roll_pitch(self):
        """First call with dt=0: roll/pitch initialized from accel, yaw == 0."""
        fuse = ComplementaryFilter(alpha=0.98)
        accel_g = (0.0, 0.5, 0.8660254)
        gyro_dps = (0.0, 0.0, 90.0)  # nonzero gyro
        dt_s = 0.0  # dt = 0 on first call

        roll, pitch, yaw = fuse.update(accel_g, gyro_dps, dt_s)

        # Expected: roll from accel, yaw = 0 (no integration when dt<=0)
        assert abs(roll - 0.5235988) < 1e-6
        assert abs(pitch) < 1e-6
        assert yaw == 0.0


class TestReset:
    """Test: filter reset."""

    def test_reset_clears_state(self):
        """Tilt, reset(), then flat call → roll≈0, yaw==0."""
        fuse = ComplementaryFilter(alpha=0.98)
        # Tilt
        fuse.update((0.0, 0.5, 0.8660254), (0.0, 0.0, 0.0), 0.02)
        roll_tilted, pitch_tilted, yaw_tilted = fuse.update((0.0, 0.5, 0.8660254), (0.0, 0.0, 0.0), 0.02)
        assert roll_tilted > 0.1

        # Reset
        fuse.reset()
        # Flat call (re-init)
        roll, pitch, yaw = fuse.update((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), 0.02)

        assert abs(roll) < 1e-9
        assert abs(pitch) < 1e-9
        assert yaw == 0.0


class TestWrapAroundNearPi:
    """Test: angle wrapping near ±π."""

    def test_wrap_large_roll(self):
        """Accel tilt ≈ 179°: roll ≈ 3.1241. Alternating updates keep abs(roll) > 3.0."""
        fuse = ComplementaryFilter(alpha=0.98)
        # Tilt to ~179°: sin(179°) ≈ 0.017452, cos(179°) ≈ -0.999848
        accel_179 = (0.0, 0.017452, -0.999848)
        gyro_zero = (0.0, 0.0, 0.0)
        dt_s = 0.02

        # First call: initialize at 179°
        roll, pitch, yaw = fuse.update(accel_179, gyro_zero, dt_s)
        # Expected roll ≈ atan2(sin(179°), cos(179°)) ≈ 3.1241 or ≈ -3.1241
        # atan2 gives (−π, π], so atan2(+, −) is in (π/2, π]
        assert abs(roll) > 3.0

        # 50 alternating updates: flip ay sign (accel flips, stays ≈179° but opposite)
        for i in range(50):
            ay_flip = 0.017452 if i % 2 == 0 else -0.017452
            accel_alt = (0.0, ay_flip, -0.999848)
            roll, pitch, yaw = fuse.update(accel_alt, gyro_zero, dt_s)
            assert abs(roll) > 3.0


class TestNonFiniteHandling:
    """Test: non-finite input rejection."""

    def test_nan_in_accel(self):
        """NaN in accel after init → state unchanged, results finite."""
        fuse = ComplementaryFilter(alpha=0.98)
        # Init
        fuse.update((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), 0.02)
        state_before = fuse.update((0.0, 0.0, 1.0), (30.0, 0.0, 0.0), 0.02)

        # NaN in ax
        roll, pitch, yaw = fuse.update((math.nan, 0.0, 1.0), (0.0, 0.0, 0.0), 0.02)

        assert roll == state_before[0]
        assert pitch == state_before[1]
        assert yaw == state_before[2]
        assert all(math.isfinite(v) for v in [roll, pitch, yaw])

    def test_nan_in_gyro(self):
        """NaN in gyro after init → state unchanged, results finite."""
        fuse = ComplementaryFilter(alpha=0.98)
        # Init
        fuse.update((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), 0.02)
        state_before = fuse.update((0.0, 0.0, 1.0), (30.0, 0.0, 0.0), 0.02)

        # NaN in gy
        roll, pitch, yaw = fuse.update((0.0, 0.0, 1.0), (0.0, math.nan, 0.0), 0.02)

        assert roll == state_before[0]
        assert pitch == state_before[1]
        assert yaw == state_before[2]
        assert all(math.isfinite(v) for v in [roll, pitch, yaw])

    def test_nan_in_dt(self):
        """NaN in dt_s after init → state unchanged, results finite."""
        fuse = ComplementaryFilter(alpha=0.98)
        # Init
        fuse.update((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), 0.02)
        state_before = fuse.update((0.0, 0.0, 1.0), (30.0, 0.0, 0.0), 0.02)

        # NaN in dt
        roll, pitch, yaw = fuse.update((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), math.nan)

        assert roll == state_before[0]
        assert pitch == state_before[1]
        assert yaw == state_before[2]
        assert all(math.isfinite(v) for v in [roll, pitch, yaw])

    def test_following_normal_update_after_nan(self):
        """Normal update after NaN rejection stays finite and consistent."""
        fuse = ComplementaryFilter(alpha=0.98)
        # Init
        fuse.update((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), 0.02)
        fuse.update((0.0, 0.0, 1.0), (30.0, 0.0, 0.0), 0.02)

        # Reject NaN
        fuse.update((math.nan, 0.0, 1.0), (0.0, 0.0, 0.0), 0.02)

        # Normal update should work
        roll, pitch, yaw = fuse.update((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), 0.02)

        assert all(math.isfinite(v) for v in [roll, pitch, yaw])


class TestZeroAccelAfterInit:
    """Test: zero acceleration after initialization."""

    def test_zero_accel_gyro_only_integration(self):
        """Zero accel after init: no exception, gyro-only step exact."""
        fuse = ComplementaryFilter(alpha=0.98)
        # Init with normal accel
        fuse.update((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), 0.02)
        roll_before, pitch_before, yaw_before = fuse.update((0.0, 0.0, 1.0), (30.0, 0.0, 0.0), 0.02)

        # Zero accel (norm < eps)
        accel_zero = (1e-7, 1e-7, 1e-7)  # norm² < 1e-12
        roll, pitch, yaw = fuse.update(accel_zero, (30.0, 0.0, 0.0), 0.02)

        # Roll should be exactly: prev_roll + gx_rad * dt
        gx_rad = math.radians(30.0)
        expected_roll = _wrap(roll_before + gx_rad * 0.02)
        assert math.isfinite(roll)
        assert abs(roll - expected_roll) < 1e-10


def _wrap(a):
    """Helper wrap function from dof_fusion module."""
    return (a + math.pi) % (2 * math.pi) - math.pi


class TestAlphaValidation:
    """Test: alpha parameter validation."""

    def test_alpha_negative_raises(self):
        """alpha=-0.1 raises ValueError."""
        with pytest.raises(ValueError):
            ComplementaryFilter(alpha=-0.1)

    def test_alpha_over_one_raises(self):
        """alpha=1.1 raises ValueError."""
        with pytest.raises(ValueError):
            ComplementaryFilter(alpha=1.1)

    def test_alpha_nan_raises(self):
        """alpha=float('nan') raises ValueError."""
        with pytest.raises(ValueError):
            ComplementaryFilter(alpha=float("nan"))

    def test_alpha_zero_ok(self):
        """alpha=0.0 constructs OK (accel-only)."""
        fuse = ComplementaryFilter(alpha=0.0)
        assert fuse.alpha == 0.0

    def test_alpha_one_ok(self):
        """alpha=1.0 constructs OK (gyro-only)."""
        fuse = ComplementaryFilter(alpha=1.0)
        assert fuse.alpha == 1.0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
