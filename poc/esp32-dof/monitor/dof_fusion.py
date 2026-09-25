#!/usr/bin/env python3
"""
Complementary filter for 6-DOF IMU orientation fusion (accelerometer + gyroscope).

The filter fuses accelerometer gravity vector (estimating roll and pitch) with gyroscope
rotational velocity (gyro-z for yaw) to produce an estimate of the device orientation.

**Inputs and conventions:**
- Accelerometer: 3 values in units of *g* (Earth's gravity ≈ 9.81 m/s²)
- Gyroscope: 3 values in units of *degrees per second* (dps)
- Time step dt: in *seconds* (float); must be positive to integrate

**Output:**
- Roll, pitch, yaw: all in *radians* (Python floats)
- Rotation convention: ZYX (yaw-pitch-roll); composed as R = Rz(yaw) Ry(pitch) Rx(roll)
  - Roll: rotation about X-axis (±π), right-hand rule thumb along +X
  - Pitch: rotation about Y-axis [−π/2, π/2], right-hand rule thumb along +Y
  - Yaw: rotation about Z-axis (−π, π], right-hand rule thumb along +Z

**Fusion approach:**
- Roll and pitch: complementary filter blends gyro integration (prediction) with accelerometer
  estimation (correction), with blend weight alpha ∈ [0, 1].
  - alpha=1.0: gyro-only (fastest response, drifts over time).
  - alpha=0.0: accel-only (slow, accurate assuming no linear acceleration).
  - Default alpha=0.98: 98% gyro + 2% accel correction.
- Yaw: gyro-z integration only (no magnetometer); drifts unbounded without external reference.

**Limitations:**
- Yaw DRIFTS WITHOUT BOUND over time — no magnetometer or external reference available.
- Roll/pitch ill-conditioned near pitch ≈ ±90° (gimbal lock region); singularity handling not
  implemented (use Euler angles with care near poles or upgrade to quaternions for robustness).
- No free-fall or linear acceleration rejection — filter assumes +1g on Z-axis when device is flat
  and stationary; large linear accelerations will corrupt roll/pitch estimates.
- Euler-rate coupling ignored: gyro integration does not account for the Euler-angle rate
  transformation (ω = E⁻¹ * dθ/dt). This is accepted for a POC and yields small errors for
  modest motion; a production system should use quaternions or matrix exponentials.
- Assumes accelerometer frames and gyroscope frames are aligned and that +Z points upward
  when the device is flat with specific force = +1g (standard IMU convention).

Usage:
    import math
    from dof_fusion import ComplementaryFilter

    # Create a filter with default 98/2 blend (gyro/accel)
    fuse = ComplementaryFilter(alpha=0.98)

    # Update the filter at each IMU sample
    accel_g = (ax, ay, az)      # accelerometer in g
    gyro_dps = (gx, gy, gz)     # gyroscope in deg/s
    dt_s = 0.02                 # time step in seconds

    roll, pitch, yaw = fuse.update(accel_g, gyro_dps, dt_s)
    print(f"Orientation: roll={math.degrees(roll):.1f}°, "
          f"pitch={math.degrees(pitch):.1f}°, yaw={math.degrees(yaw):.1f}°")

    # Reset to zero orientation
    fuse.reset()

    # Re-enable adaptive updates
    roll, pitch, yaw = fuse.update(accel_g, gyro_dps, dt_s)
"""

import math

_EPS_A2 = 1e-12


def _wrap(a):
    """Wrap angle to range (−π, π]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


class ComplementaryFilter:
    """
    6-DOF complementary filter for IMU orientation fusion (accelerometer + gyroscope).

    Attributes:
        alpha: Blend weight [0, 1]; higher = more gyro (faster but drifts).
    """

    def __init__(self, alpha: float = 0.98):
        """
        Initialize the filter.

        Args:
            alpha: Blend weight for complementary filter, 0.0 <= alpha <= 1.0.
                   - 1.0: gyro-only (pure integration, drifts)
                   - 0.0: accel-only (slow, no drift from gyro alone)
                   - Default 0.98: 98% gyro prediction + 2% accel correction

        Raises:
            ValueError: if alpha is not in [0.0, 1.0] or is NaN.
        """
        if not isinstance(alpha, (int, float)) or math.isnan(alpha):
            raise ValueError(f"alpha must be a finite number, got {alpha!r}")
        if not (0.0 <= alpha <= 1.0):
            raise ValueError(f"alpha must be in [0.0, 1.0], got {alpha}")
        self.alpha = alpha
        self.reset()

    def reset(self):
        """Reset filter state to zero orientation (roll=pitch=yaw=0) and un-initialized."""
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._initialised = False

    def update(self, accel_g, gyro_dps, dt_s):
        """
        Update filter state with a new IMU measurement.

        Args:
            accel_g: Tuple (ax, ay, az) of accelerometer readings in g.
            gyro_dps: Tuple (gx, gy, gz) of gyroscope readings in deg/s.
            dt_s: Time step in seconds (float). Must be finite and >= 0.
                  dt <= 0 on first call still initializes roll/pitch from accel but keeps yaw=0.
                  dt <= 0 on subsequent calls returns state unchanged (no integration).

        Returns:
            Tuple (roll, pitch, yaw) in radians (Python floats).
            - Roll: rotation about X-axis (±π)
            - Pitch: rotation about Y-axis [−π/2, π/2]
            - Yaw: rotation about Z-axis (−π, π]

        Behavior on non-finite input:
            If any of accel_g, gyro_dps, or dt_s is non-finite (NaN or ±inf),
            the update is ignored and the current state tuple is returned unchanged.
            The internal state is not modified (including _initialised flag).
        """
        # Unpack and validate finiteness
        try:
            ax, ay, az = accel_g
            gx, gy, gz = gyro_dps
        except (TypeError, ValueError):
            return (self._roll, self._pitch, self._yaw)

        # Check all 6 sensor values and dt_s for finiteness
        if not all(
            math.isfinite(v)
            for v in [ax, ay, az, gx, gy, gz, dt_s]
        ):
            return (self._roll, self._pitch, self._yaw)

        # Compute acceleration magnitude squared
        norm2 = ax * ax + ay * ay + az * az
        zero_acc = norm2 < _EPS_A2

        # Estimate roll and pitch from accelerometer (gravity vector)
        roll_acc = 0.0
        pitch_acc = 0.0
        if not zero_acc:
            roll_acc = math.atan2(ay, az)
            pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        # Initialization on first call
        if not self._initialised:
            if zero_acc:
                self._roll = 0.0
                self._pitch = 0.0
            else:
                self._roll = roll_acc
                self._pitch = pitch_acc
            self._initialised = True
            # Integrate yaw if dt > 0
            if dt_s > 0.0:
                self._yaw = _wrap(self._yaw + math.radians(gz) * dt_s)
            return (self._roll, self._pitch, self._yaw)

        # Subsequent calls: ignore update if dt <= 0
        if not (dt_s > 0.0):
            return (self._roll, self._pitch, self._yaw)

        # Gyro integration: convert deg/s to rad/s
        gx_rad = math.radians(gx)
        gy_rad = math.radians(gy)
        gz_rad = math.radians(gz)

        # Gyro prediction
        pred_roll = self._roll + gx_rad * dt_s
        pred_pitch = self._pitch + gy_rad * dt_s

        # Complementary blend: pred + (1-alpha)*wrap(accel - pred)
        # This form is wrap-safe and avoids 2π jumps across the discontinuity
        if zero_acc:
            self._roll = _wrap(pred_roll)
            self._pitch = pred_pitch
        else:
            self._roll = _wrap(pred_roll + (1.0 - self.alpha) * _wrap(roll_acc - pred_roll))
            self._pitch = pred_pitch + (1.0 - self.alpha) * (pitch_acc - pred_pitch)

        # Yaw: gyro-z integration only (no magnetometer)
        self._yaw = _wrap(self._yaw + gz_rad * dt_s)

        return (self._roll, self._pitch, self._yaw)
