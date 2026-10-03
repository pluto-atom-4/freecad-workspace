#!/usr/bin/env python3
"""
Overlay label text for the servo-wheel-dof POC (issue #313, sub-issue of #303).

Pure Python: only the standard library; no Webots, serial or numpy dependency
(must NOT `import controller`). The controller passes the result to
Supervisor.setLabel (GUI only, read-only).

Conventions:
- parse_overlay_env reads SWD_OVERLAY (on: 1,true,yes,on; off: empty,0,false,
  no,off; case-insensitive, stripped; anything else is off plus one warning).
  Same rules as telemetry.parse_telemetry_env. Never raises.
- format_overlay returns 4 lines joined by "\\n":
      servo-wheel OK|WARN
      dev=<mm, 3 decimals> mm
      axis=<dot, 6 decimals>
      rel=(<x>, <y>, <z>) mm        (mm, 2 decimals)
  None, non-numeric or non-finite values become the text nan. A rel that is
  None or not 3 values becomes (nan, nan, nan). A negative zero such as -0.000
  is written without the minus sign. Never raises.
- overlay_color returns 0xRRGGBB: COLOR_WARN when warn is true, else COLOR_OK.
- overlay_every_steps(timestep_ms, hz) is ceil(1000 / (hz * timestep_ms)),
  at least 1; a bad timestep or hz gives 1.

Usage:
    from overlay_text import format_overlay, overlay_color

    robot.setLabel(0, format_overlay(dev, dot, rel, warn), 0.01, 0.01, 0.05,
                   overlay_color(warn), 0.0)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

ENV_OVERLAY = "SWD_OVERLAY"
DEFAULT_HZ = 4.0
COLOR_OK = 0x00C000
COLOR_WARN = 0xFF0000

_ON = frozenset(("1", "true", "yes", "on"))
_OFF = frozenset(("", "0", "false", "no", "off"))


@dataclass(frozen=True)
class OverlayConfig:
    """Result of parse_overlay_env."""

    enabled: bool
    warnings: tuple[str, ...]


def parse_overlay_env(environ: Mapping[str, str]) -> OverlayConfig:
    """Read SWD_OVERLAY from a mapping. Never raises; junk means off + warning."""
    raw = environ.get(ENV_OVERLAY)
    text = "" if raw is None else str(raw).strip().lower()
    if text in _ON:
        return OverlayConfig(True, ())
    if text in _OFF:
        return OverlayConfig(False, ())
    return OverlayConfig(
        False,
        (
            f"{ENV_OVERLAY}={raw!r} is not one of 1/true/yes/on or "
            "0/false/no/off; overlay stays off",
        ),
    )


def _fmt(value: Any, scale: float, decimals: int) -> str:
    """Return value*scale with fixed decimals; nan for None/non-finite; no -0."""
    try:
        number = float(value) * scale
    except (TypeError, ValueError, OverflowError):
        return "nan"
    if not math.isfinite(number):
        return "nan"
    text = "%.*f" % (decimals, number)
    if float(text) == 0.0:
        text = "%.*f" % (decimals, 0.0)
    return text


def format_overlay(
    deviation_m: Any,
    axis_dot: Any,
    rel: Sequence[Any] | None,
    warn: Any,
) -> str:
    """Return the 4-line overlay text (see module docstring)."""
    try:
        rx, ry, rz = rel  # type: ignore[misc]
    except (TypeError, ValueError):
        rx = ry = rz = None
    head = "servo-wheel WARN" if warn else "servo-wheel OK"
    return "\n".join(
        (
            head,
            f"dev={_fmt(deviation_m, 1000.0, 3)} mm",
            f"axis={_fmt(axis_dot, 1.0, 6)}",
            f"rel=({_fmt(rx, 1000.0, 2)}, {_fmt(ry, 1000.0, 2)}, "
            f"{_fmt(rz, 1000.0, 2)}) mm",
        )
    )


def overlay_color(warn: Any) -> int:
    """Return the 0xRRGGBB label colour: red when warn, else green."""
    return COLOR_WARN if warn else COLOR_OK


def overlay_every_steps(timestep_ms: Any, hz: float = DEFAULT_HZ) -> int:
    """Return how many steps between label updates (>= 1)."""
    try:
        step = float(timestep_ms)
        rate = float(hz)
    except (TypeError, ValueError, OverflowError):
        return 1
    if not (math.isfinite(step) and math.isfinite(rate)):
        return 1
    if step <= 0.0 or rate <= 0.0:
        return 1
    return max(1, math.ceil(1000.0 / (rate * step)))
