#!/usr/bin/env python3
"""
UDP JSON messages of the ctrl-wheel bridge (ctrl_wheel_udp): encoders, strict
decoders, a loopback listener and a non-blocking drain helper.

SIM ONLY. The wheel sign convention and the model values are UNVERIFIED.
The wire format is not validated against any real device or receiver.

Wire contract with webots/controllers/ctrl_wheel/ctrl_wheel.py. That controller
cannot import this module, so the JSON key names are duplicated there on
purpose. Change a key name in both places or not at all.

Ports: 5007 (echo, sim -> host) and 5008 (command, host -> sim) only.
Port 5006 is never used by this module.

Messages (compact JSON, UTF-8, one datagram per message):
    cmd:  {"cmd_seq":<int >= 0>,"wheel_rad_s":<float, clamped +-3.0>,"stale":<0|1>}
    echo: {"cmd_seq_echo":<int >= 0 or null>,"sim_t_s":<float>,
           "angle_rad":<float>,"vel_rad_s":<float or null>}

Strict decode: exact key set, no duplicate keys, bool rejected for numbers,
NaN / Infinity tokens rejected, int fields must be JSON ints (not floats),
nulls allowed only for cmd_seq_echo and vel_rad_s.

Usage (run from poc/ctrl-wheel):
    python3 -m pytest test_ctrl_wheel_udp.py -v
"""

from __future__ import annotations

import json
import math
import socket
from dataclasses import dataclass


HOST = "127.0.0.1"
PORT_CMD = 5008
PORT_ECHO = 5007
# SIM ONLY. Wheel speed clamp in rad/s, applied on encode and on decode.
WHEEL_CLAMP_RAD_S = 3.0
STALE_SET = (0, 1)
RECV_BUFSIZE = 65535

CMD_KEYS = frozenset({"cmd_seq", "wheel_rad_s", "stale"})
ECHO_KEYS = frozenset({"cmd_seq_echo", "sim_t_s", "angle_rad", "vel_rad_s"})


class UdpMsgError(ValueError):
    """Raised when a ctrl-wheel UDP message fails to encode or decode."""


@dataclass(frozen=True)
class Cmd:
    """Host -> sim wheel command."""
    cmd_seq: int
    wheel_rad_s: float
    stale: int


@dataclass(frozen=True)
class Echo:
    """Sim -> host state echo."""
    cmd_seq_echo: int | None
    sim_t_s: float
    angle_rad: float
    vel_rad_s: float | None


def _finite_real(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise UdpMsgError(f"{name} must be a real number, got {value!r}")
    try:
        result = float(value)
    except OverflowError as exc:
        raise UdpMsgError(f"{name} out of float range, got {value!r}") from exc
    if not math.isfinite(result):
        raise UdpMsgError(f"{name} is not finite, got {value!r}")
    return result


def _check_seq(name: str, value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise UdpMsgError(f"{name} must be int, got {value!r}")
    if value < 0:
        raise UdpMsgError(f"{name} must be >= 0, got {value}")
    return value


def _check_stale(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value not in STALE_SET:
        raise UdpMsgError(f"stale must be one of {STALE_SET}, got {value!r}")
    return value


def clamp(value: float) -> float:
    """Limit a wheel speed to +-WHEEL_CLAMP_RAD_S. Rejects bool and non-finite input."""
    v = _finite_real("wheel_rad_s", value)
    return max(-WHEEL_CLAMP_RAD_S, min(WHEEL_CLAMP_RAD_S, v))


def _reject_constant(token: str) -> None:
    raise UdpMsgError(f"non-finite JSON token {token} not allowed")


def _no_duplicate_keys(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise UdpMsgError(f"duplicate key {key!r}")
        result[key] = value
    return result


def _parse_object(data: bytes, keys: frozenset[str]) -> dict[str, object]:
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise UdpMsgError(f"message is not valid UTF-8: {exc}") from exc
    try:
        obj = json.loads(
            text,
            parse_constant=_reject_constant,
            object_pairs_hook=_no_duplicate_keys,
        )
    except UdpMsgError:
        raise
    except ValueError as exc:
        raise UdpMsgError(f"message is not valid JSON: {exc}") from exc
    if not isinstance(obj, dict):
        raise UdpMsgError(f"message must be a JSON object, got {type(obj).__name__}")
    missing = keys - obj.keys()
    extra = obj.keys() - keys
    if missing or extra:
        raise UdpMsgError(f"key set mismatch: missing {sorted(missing)}, extra {sorted(extra)}")
    return obj


def encode_cmd(cmd_seq: int, wheel_rad_s: float, stale: int) -> bytes:
    """Encode a command datagram. wheel_rad_s is clamped to +-WHEEL_CLAMP_RAD_S."""
    payload = {
        "cmd_seq": _check_seq("cmd_seq", cmd_seq),
        "wheel_rad_s": clamp(wheel_rad_s),
        "stale": _check_stale(stale),
    }
    return json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")


def decode_cmd(data: bytes) -> Cmd:
    """Decode and validate a command datagram. wheel_rad_s is clamped on decode."""
    obj = _parse_object(data, CMD_KEYS)
    return Cmd(
        cmd_seq=_check_seq("cmd_seq", obj["cmd_seq"]),
        wheel_rad_s=clamp(obj["wheel_rad_s"]),  # type: ignore[arg-type]
        stale=_check_stale(obj["stale"]),
    )


def encode_echo(
    cmd_seq_echo: int | None,
    sim_t_s: float,
    angle_rad: float,
    vel_rad_s: float | None,
) -> bytes:
    """Encode an echo datagram. cmd_seq_echo and vel_rad_s may be None."""
    payload = {
        "cmd_seq_echo": None if cmd_seq_echo is None else _check_seq("cmd_seq_echo", cmd_seq_echo),
        "sim_t_s": _finite_real("sim_t_s", sim_t_s),
        "angle_rad": _finite_real("angle_rad", angle_rad),
        "vel_rad_s": None if vel_rad_s is None else _finite_real("vel_rad_s", vel_rad_s),
    }
    return json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")


def decode_echo(data: bytes) -> Echo:
    """Decode and validate an echo datagram."""
    obj = _parse_object(data, ECHO_KEYS)
    seq_raw = obj["cmd_seq_echo"]
    vel_raw = obj["vel_rad_s"]
    return Echo(
        cmd_seq_echo=None if seq_raw is None else _check_seq("cmd_seq_echo", seq_raw),
        sim_t_s=_finite_real("sim_t_s", obj["sim_t_s"]),
        angle_rad=_finite_real("angle_rad", obj["angle_rad"]),
        vel_rad_s=None if vel_raw is None else _finite_real("vel_rad_s", vel_raw),
    )


def open_listener(port: int, host: str = HOST) -> socket.socket:
    """Bind a non-blocking UDP socket. No SO_REUSEADDR: a second bind raises OSError."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((host, port))
        sock.setblocking(False)
    except OSError:
        sock.close()
        raise
    return sock


def drain_all(sock: socket.socket) -> list[bytes]:
    """Return every pending datagram payload in arrival order. Never blocks."""
    payloads: list[bytes] = []
    while True:
        try:
            data, _addr = sock.recvfrom(RECV_BUFSIZE)
        except BlockingIOError:
            break
        except OSError:
            break
        payloads.append(data)
    return payloads
