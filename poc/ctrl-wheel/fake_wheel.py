#!/usr/bin/env python3
"""
Headless fake of the Webots ctrl_wheel controller (issue FW #451).

SIM ONLY. Headless stand-in for webots/controllers/ctrl_wheel/ctrl_wheel.py
(stacked PRs #461-#465, not in this tree). It is not a Webots model. The wheel
sign convention is UNVERIFIED and the model values (tau, dt, clamp, stale
window) are guesses. For tests only. Nothing here validates a real wheel or a
balance loop.

Controller law imitated:
    target = 0 if no command yet, cmd.stale == 1, or simulated age since the
             last accepted command >= stale_s; else clamp(sign * wheel_rad_s).
    velocity += (target - velocity) * (dt / (tau + dt))     first-order lag
    angle += velocity * dt
    sim time += dt
Each step sends one echo to echo_addr: cmd_seq_echo (last accepted cmd_seq,
None before the first), sim_t_s, angle_rad, vel_rad_s (finite difference of
angle, None on the first sample).

Ports: cmd_port 5005-5008 is refused and echo_addr port 5006 is refused.
step() is not thread-safe: drive it from one thread only.

Usage (run from poc/ctrl-wheel):
    python3 -m pytest test_ctrl_wheel_fake.py -v
"""

from __future__ import annotations

import socket
import threading
import time

from ctrl_wheel_udp import (
    UdpMsgError,
    decode_cmd,
    drain_all,
    encode_echo,
    open_listener,
)
from ctrl_wheel_udp import clamp as _clamp_rad_s

_RESERVED_CMD_PORTS = frozenset(range(5005, 5009))
_RESERVED_ECHO_PORT = 5006


class FakeWheel:
    """SIM ONLY headless ctrl_wheel stand-in. UNVERIFIED sign and model values."""

    def __init__(
        self,
        echo_addr: tuple[str, int],
        cmd_port: int = 0,
        tau_s: float = 0.05,
        dt_s: float = 0.016,
        sign: int = 1,
        stale_s: float = 0.2,
        clamp: float = 3.0,
        host: str = "127.0.0.1",
    ) -> None:
        if cmd_port in _RESERVED_CMD_PORTS:
            raise ValueError(f"cmd_port {cmd_port} is reserved (5005-5008)")
        if echo_addr[1] == _RESERVED_ECHO_PORT:
            raise ValueError(f"echo_addr port {_RESERVED_ECHO_PORT} is reserved")
        self._echo_addr = echo_addr
        self._dt = dt_s
        self._lag = dt_s / (tau_s + dt_s)
        self._sign = sign
        self._stale_s = stale_s
        self._limit = clamp
        self._cmd_sock = open_listener(cmd_port, host)
        self._cmd_addr = self._cmd_sock.getsockname()[:2]
        self._echo_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._closed = False
        self._sim_t = 0.0
        self._angle = 0.0
        self._velocity = 0.0
        self._prev_angle: float | None = None
        self._cmd_seq: int | None = None
        self._cmd_wheel = 0.0
        self._cmd_stale = 0
        self._cmd_t = 0.0

    @property
    def cmd_addr(self) -> tuple[str, int]:
        return self._cmd_addr

    def _accept_commands(self) -> None:
        for data in drain_all(self._cmd_sock):
            try:
                cmd = decode_cmd(data)
            except (UdpMsgError, RecursionError):
                continue
            self._cmd_seq = cmd.cmd_seq
            self._cmd_wheel = cmd.wheel_rad_s
            self._cmd_stale = cmd.stale
            self._cmd_t = self._sim_t

    def _target(self) -> float:
        if self._cmd_seq is None or self._cmd_stale == 1:
            return 0.0
        if self._sim_t - self._cmd_t >= self._stale_s:
            return 0.0
        raw = _clamp_rad_s(self._sign * self._cmd_wheel)
        return max(-self._limit, min(self._limit, raw))

    def step(self) -> None:
        """Advance one dt_s tick and send one echo. Never raises."""
        self._accept_commands()
        self._velocity += (self._target() - self._velocity) * self._lag
        self._angle += self._velocity * self._dt
        self._sim_t += self._dt
        if self._prev_angle is None:
            vel: float | None = None
        else:
            vel = (self._angle - self._prev_angle) / self._dt
        self._prev_angle = self._angle
        try:
            payload = encode_echo(self._cmd_seq, self._sim_t, self._angle, vel)
            self._echo_tx.sendto(payload, self._echo_addr)
        except (OSError, UdpMsgError):
            pass

    def close(self) -> None:
        """Close both sockets. Safe to call more than once."""
        if self._closed:
            return
        self._closed = True
        self._cmd_sock.close()
        self._echo_tx.close()

    def run_background(self, stop_event: threading.Event) -> threading.Thread:
        """Step every dt_s of real time in a daemon thread until stop_event is set."""

        def _loop() -> None:
            while not stop_event.is_set():
                started = time.monotonic()
                self.step()
                remaining = self._dt - (time.monotonic() - started)
                stop_event.wait(max(0.0, remaining))

        thread = threading.Thread(target=_loop, name="fake_wheel", daemon=True)
        thread.start()
        return thread
