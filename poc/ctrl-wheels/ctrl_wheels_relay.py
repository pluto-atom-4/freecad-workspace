#!/usr/bin/env python3
"""Relay that holds the latest WHL sample and applies a stale and hold rule."""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass

from ctrl_wheels_msg import Whl


STALE_S = 0.200
POLL_S = 0.020
CLAMP_RAD_S = 3.0


@dataclass(frozen=True)
class RelayOut:
    """One relay output: wheel velocity in rad/s and a stale flag (1 = zeroed)."""

    wheel_rad_s: float
    stale: int


class Relay:
    """Holds the latest WHL sample and maps it to a RelayOut on poll()."""

    def __init__(
        self,
        clock: Callable[[], float] = time.monotonic,
        stale_s: float = STALE_S,
        clamp: float = CLAMP_RAD_S,
        sign: int = 1,
    ) -> None:
        if type(sign) is not int or sign not in (1, -1):
            raise ValueError(f"sign must be +1 or -1, got {sign!r}")
        if not (math.isfinite(stale_s) and stale_s > 0):
            raise ValueError(f"stale_s must be a positive finite number, got {stale_s!r}")
        if not (math.isfinite(clamp) and clamp > 0):
            raise ValueError(f"clamp must be a positive finite number, got {clamp!r}")
        self._clock = clock
        self._stale_s = stale_s
        self._clamp = clamp
        self._sign = sign
        self._whl: Whl | None = None
        self._t_arrival = 0.0

    def on_whl(self, whl: Whl) -> None:
        """Store the latest sample and re-arm its arrival time."""
        if not isinstance(whl, Whl):
            raise TypeError(f"on_whl expects Whl, got {type(whl).__name__}")
        self._whl = whl
        self._t_arrival = self._clock()

    def poll(self) -> RelayOut | None:
        """Return the current output, or None before the first sample."""
        whl = self._whl
        if whl is None:
            return None
        if whl.state != 0:
            return RelayOut(wheel_rad_s=0.0, stale=1)
        if self._clock() - self._t_arrival >= self._stale_s:
            return RelayOut(wheel_rad_s=0.0, stale=1)
        if not math.isfinite(whl.model_rad_s):
            return RelayOut(wheel_rad_s=0.0, stale=1)
        value = self._sign * whl.model_rad_s
        value = max(-self._clamp, min(self._clamp, value))
        return RelayOut(wheel_rad_s=value, stale=0)
