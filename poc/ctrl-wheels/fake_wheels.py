#!/usr/bin/env python3
"""
Headless twin fake of the Webots ctrl_wheels controller.

SIM ONLY. Headless stand-in for the twin Webots controller at
poc/ctrl-wheels/webots/controllers/ctrl_wheels/ctrl_wheels.py, which is the real
counterpart. Per the header of ctrl_wheels_udp.py, that controller may still
speak the v1 wire format until its v2 version lands; this fake speaks v2 only,
and no run against the real controller is claimed. This file
is not a Webots model and makes no physics claim. The wheel sign convention is
UNCONFIRMED (CONTRACT.md S2) and the fake applies each wheel's command as given,
so any sign belongs to the bridge. The model values (tau, dt, clamp, stale
window) are guesses. For tests only. Nothing here validates a real wheel or a
balance loop.

Wire (CONTRACT.md S3, UDP v2, one compact JSON datagram per message):
    cmd  in  on the command port (default 5018 in the real setup):
         {"v":2,"cmd_seq":<int>,"wheels":{"left":<float>,"right":<float>},"stale":<0|1>}
    echo out to echo_addr (default port 5017 in the real setup):
         {"v":2,"cmd_seq_echo":<int or null>,"sim_t_s":<float>,
          "left":{"angle_rad":<float>,"vel_rad_s":<float or null>},
          "right":{"angle_rad":<float>,"vel_rad_s":<float or null>}}
    Decoding and encoding go through ctrl_wheels_udp. A datagram that fails to
    decode (including any v1 command) is skipped without a reply.

Two independent first-order wheels, left and right. Each step (dt_s of sim time):
    target = 0 if no command has been accepted yet, if the accepted command has
             stale == 1, or if the sim age since it was accepted is at least
             stale_s; otherwise the command for that wheel, limited to +-clamp.
    velocity += (target - velocity) * (dt_s / (tau_s + dt_s))    first-order lag
    angle    += velocity * dt_s
    sim time += dt_s
    vel_rad_s is the finite difference of angle, None on the first step (no
    previous angle yet). CONTRACT.md S3 allows a null vel_rad_s; the first-step
    null is this fake's choice.

The stale window uses sim time on purpose, so tests are deterministic. CONTRACT.md
S5 defines the age on the host clock, so this is a deliberate difference.

Ports (CONTRACT.md S4): the reserved set is the literal set {5005, 5006, 5007,
5008, 5017, 5018}. This module refuses to bind a reserved command port, and it
refuses a reserved echo port in echo_addr. It must not be derived from a range.
Ports must be ints in 0..65535. Port 0 (ephemeral) is meant for tests.
The command port is unauthenticated, so host must be "127.0.0.1" or "localhost"
(the default is 127.0.0.1); any other host raises ValueError before binding.

Usage (run from poc/ctrl-wheels):
    python3 -m pytest test_ctrl_wheels_fake.py -v

step() is not thread-safe: drive it from one thread only.
"""

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

# CONTRACT.md S4, written out as a literal set on purpose. Do not derive it from a range.
_RESERVED_PORTS = frozenset({5005, 5006, 5007, 5008, 5017, 5018})
# Stale window of CONTRACT.md S5, in sim seconds.
STALE_S = 0.200
# The command port is unauthenticated (CONTRACT.md S4 default is 127.0.0.1): loopback only.
_LOOPBACK_HOSTS = frozenset({"127.0.0.1", "localhost"})


class FakeWheel:
    """SIM ONLY headless twin ctrl_wheels stand-in. UNVERIFIED sign and model values."""

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
        """Advance one dt_s tick and send one echo. Never raises.

        An echo can be dropped if the encode or send fails.
        """
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
            # By design: step() never raises, a failed echo is dropped. The encode
            # cannot fail today: cmd_seq is the decoded int (not clamped), and
            # json encode and decode share the same int-to-text digit limit.
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
