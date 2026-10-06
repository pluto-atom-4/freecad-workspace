"""Pure helpers shared by Webots controller entry points.

Keeps log lines byte-compatible with what webots/tests/*.sh grep for.
No numpy, no Webots dependencies.
"""

import sys
from pathlib import Path
from typing import List, Sequence, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hal.hal import HalFault, ImuSample, EncoderSample


# Module constants
SATURATION_FRACTION = 0.95


def is_saturated(value: float, limit: float = 1.0) -> bool:
    """Check if a value is saturated relative to a limit.

    Args:
        value: The value to check
        limit: The saturation limit (default 1.0)

    Returns:
        True if abs(value) >= SATURATION_FRACTION * limit
    """
    return abs(value) >= SATURATION_FRACTION * limit


def should_log_row(count: int, throttle: int) -> bool:
    """Check if a row should be logged based on count and throttle.

    Args:
        count: The current count
        throttle: The throttle interval (0 raises ZeroDivisionError)

    Returns:
        True if count % throttle == 0
    """
    return count % throttle == 0


def time_limit_reached(t_s: float, max_s: float) -> bool:
    """Check if a time limit has been reached.

    Args:
        t_s: The current time in seconds
        max_s: The maximum time in seconds (0 means no limit)

    Returns:
        True if max_s > 0 and t_s >= max_s
    """
    return max_s > 0 and t_s >= max_s


def require_value(value, name: str) -> float:
    """Convert a value to float, raising HalFault if None.

    Args:
        value: The value to convert (may be None)
        name: The name of the value for error messages

    Returns:
        The value as a float

    Raises:
        HalFault: if value is None
    """
    if value is None:
        raise HalFault(f"{name}: value is None")
    return float(value)


def format_startup_rate_line(control_rate_ms: int) -> str:
    """Format the control rate startup log line.

    Args:
        control_rate_ms: The control rate in milliseconds

    Returns:
        Formatted string like "Control rate: 20ms (50.0Hz)"
    """
    return f"Control rate: {control_rate_ms}ms ({1000.0/control_rate_ms:.1f}Hz)"


def format_wheel_motors_line(lv: float, lt: float, rv: float, rt: float) -> str:
    """Format the wheel motor startup log line.

    Args:
        lv: Left wheel maxVelocity
        lt: Left wheel maxTorque
        rv: Right wheel maxVelocity
        rt: Right wheel maxTorque

    Returns:
        Formatted string with motor specs
    """
    return f"Wheel motors: mode=velocity, wheel_left maxVelocity={lv:.2f} maxTorque={lt:.2f}, wheel_right maxVelocity={rv:.2f} maxTorque={rt:.2f}"


def format_data_row(t: float, imu: ImuSample, enc: EncoderSample, pivot_left: float, pivot_right: float, extras: Sequence[float]) -> str:
    """Format a sensor data row for logging.

    Args:
        t: Time in seconds
        imu: IMU sample with roll_rad, pitch_rad, yaw_rad fields
        enc: Encoder sample with left_rad, right_rad fields
        pivot_left: Left pivot angle in radians
        pivot_right: Right pivot angle in radians
        extras: Additional float values to append (one per control law component)

    Returns:
        Space-separated formatted data row
    """
    tokens = [
        f"{t:.3f}",
        f"{imu.roll_rad:.4f}",
        f"{imu.pitch_rad:.4f}",
        f"{imu.yaw_rad:.4f}",
        "0.0000",
        "0.0000",
        "0.0000",
        f"{enc.left_rad:.4f}",
        f"{enc.right_rad:.4f}",
        f"{pivot_left:.4f}",
        f"{pivot_right:.4f}",
    ]
    for x in extras:
        tokens.append(f"{x:.4f}")
    return " ".join(tokens)


def format_summary_lines(fire_times: Sequence[float], control_rate_ms: int, saturation_count: int, control_step_count: int) -> List[str]:
    """Format the control loop summary lines at shutdown.

    Args:
        fire_times: Sequence of control step fire times in seconds
        control_rate_ms: Target control rate in milliseconds
        saturation_count: Number of control steps with saturation
        control_step_count: Total number of control steps executed

    Returns:
        List of formatted summary strings
    """
    lines = []
    if len(fire_times) > 1:
        diffs = [b - a for a, b in zip(fire_times[:-1], fire_times[1:])]
        mean_period_ms = sum(diffs) * 1000.0 / len(diffs)
        mean_freq_hz = 1000.0 / mean_period_ms if mean_period_ms > 0 else 0.0
        min_ms = min(diffs) * 1000.0
        max_ms = max(diffs) * 1000.0
        max_dev = max(abs(d * 1000.0 - control_rate_ms) for d in diffs)
        lines.append(f"Control rate: {mean_freq_hz:.2f}Hz (target {1000.0/control_rate_ms:.2f}Hz), periods {min_ms:.1f}-{max_ms:.1f}ms (target {control_rate_ms}ms), max deviation {max_dev:.2f}ms")
    lines.append(f"Saturation events: {saturation_count} of {control_step_count} control steps")
    return lines


__all__ = [
    "SATURATION_FRACTION",
    "is_saturated",
    "should_log_row",
    "time_limit_reached",
    "require_value",
    "format_startup_rate_line",
    "format_wheel_motors_line",
    "format_data_row",
    "format_summary_lines",
]
