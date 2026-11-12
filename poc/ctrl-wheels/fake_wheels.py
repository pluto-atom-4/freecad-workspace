#!/usr/bin/env python3
"""Headless twin fake of the Webots ctrl_wheels controller."""

from __future__ import annotations

import socket
import threading
import time

from ctrl_wheels_udp import (
    WHEEL_CLAMP_RAD_S,
    UdpMsgError,
    decode_cmd,
    drain_all,
    encode_echo,
    open_listener,
)

_RESERVED_PORTS = frozenset({5005, 5006, 5007, 5008, 5017, 5018})
STALE_S = 0.200
_LOOPBACK_HOSTS = frozenset({"127.0.0.1", "localhost"})


class FakeWheel:
    """Headless twin ctrl_wheels stand-in that applies each wheel command as given."""

    def __init__(
        self,
        echo_addr: tuple[str, int],
        cmd_port: int = 0,
        tau_s: float = 0.05,
        dt_s: float = 0.016,
        stale_s: float = STALE_S,
        clamp: float = WHEEL_CLAMP_RAD_S,
        host: str = "127.0.0.1",
    ) -> None:
        for name, port in (("cmd_port", cmd_port), ("echo port", echo_addr[1])):
            if isinstance(port, bool) or not isinstance(port, int) or not (0 <= port <= 65535):
                raise ValueError(f"{name} {port!r} must be an int in 0..65535")
        if host not in _LOOPBACK_HOSTS:
            raise ValueError(f"host {host!r} must be one of {sorted(_LOOPBACK_HOSTS)}")
        if cmd_port in _RESERVED_PORTS:
            raise ValueError(f"cmd_port {cmd_port} is reserved (CONTRACT.md S4)")
        if echo_addr[1] in _RESERVED_PORTS:
            raise ValueError(f"echo_addr port {echo_addr[1]} is reserved (CONTRACT.md S4)")
        if not (0.0 < clamp <= WHEEL_CLAMP_RAD_S):
            raise ValueError(f"clamp {clamp} must be in (0, {WHEEL_CLAMP_RAD_S}]")
        if not (dt_s > 0.0 and dt_s != float("inf")):
            raise ValueError(f"dt_s {dt_s} must be finite and > 0")
        if not (tau_s > 0.0 and tau_s != float("inf")):
            raise ValueError(f"tau_s {tau_s} must be finite and > 0")
        if not (stale_s > 0.0 and stale_s != float("inf")):
            raise ValueError(f"stale_s {stale_s} must be finite and > 0")
        self._echo_addr = echo_addr
        self._dt = dt_s
        self._lag = dt_s / (tau_s + dt_s)
        self._stale_s = stale_s
        self._limit = clamp
        self._echo_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._cmd_sock = open_listener(cmd_port, host)
        except OSError:
            self._echo_tx.close()
            raise
        self._cmd_addr = self._cmd_sock.getsockname()[:2]
        self._closed = False
        self._sim_t = 0.0
        self._angle_l = 0.0
        self._angle_r = 0.0
        self._vel_l = 0.0
        self._vel_r = 0.0
        self._prev_angle_l: float | None = None
        self._prev_angle_r: float | None = None
        self._cmd_seq: int | None = None
        self._cmd_left = 0.0
        self._cmd_right = 0.0
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
            self._cmd_left = cmd.left_rad_s
            self._cmd_right = cmd.right_rad_s
            self._cmd_stale = cmd.stale
            self._cmd_t = self._sim_t

    def _stopped(self) -> bool:
        if self._cmd_seq is None or self._cmd_stale == 1:
            return True
        return self._sim_t - self._cmd_t >= self._stale_s

    def _limited(self, wheel_rad_s: float) -> float:
        return max(-self._limit, min(self._limit, wheel_rad_s))

    def step(self) -> None:
        """Advance one dt_s tick and send one echo; never raises."""
        self._accept_commands()
        if self._stopped():
            target_l = 0.0
            target_r = 0.0
        else:
            target_l = self._limited(self._cmd_left)
            target_r = self._limited(self._cmd_right)
        self._vel_l += (target_l - self._vel_l) * self._lag
        self._vel_r += (target_r - self._vel_r) * self._lag
        self._angle_l += self._vel_l * self._dt
        self._angle_r += self._vel_r * self._dt
        self._sim_t += self._dt
        if self._prev_angle_l is None or self._prev_angle_r is None:
            vel_l: float | None = None
            vel_r: float | None = None
        else:
            vel_l = (self._angle_l - self._prev_angle_l) / self._dt
            vel_r = (self._angle_r - self._prev_angle_r) / self._dt
        self._prev_angle_l = self._angle_l
        self._prev_angle_r = self._angle_r
        try:
            payload = encode_echo(
                self._cmd_seq,
                self._sim_t,
                self._angle_l,
                vel_l,
                self._angle_r,
                vel_r,
            )
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

        thread = threading.Thread(target=_loop, name="fake_wheels", daemon=True)
        thread.start()
        return thread
