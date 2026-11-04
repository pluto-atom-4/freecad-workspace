#!/usr/bin/env python3
"""
UDP v2 JSON messages of the ctrl-wheels bridge (ctrl_wheels_udp): encoders, strict
decoders, a loopback listener and a non-blocking drain helper.

Sim-only twin of the single-wheel module. One command carries the speeds of two
wheels (left and right). The sim side echoes both wheels' angle and velocity.

SIM ONLY. The wheel sign convention and the model values are UNVERIFIED.
The wire format is not validated against any real device or receiver.

The Webots controller (planned: tasks F18 to F20, #502 to #504) cannot import
this module, so it duplicates the JSON key names on purpose. Change a key name
in both places or not at all. Today the copied controller,
poc/ctrl-wheels/webots/controllers/ctrl_wheels/ctrl_wheels.py, still speaks the
v1 wire format (cmd keys cmd_seq, wheel_rad_s and stale on port 5008); it is
rewritten by tasks F18 to F20. This module describes v2 only.

Ports: 5017 (echo, sim -> host) and 5018 (command, host -> sim) only.
Port 5006 is never used by this module. The reserved port set of CONTRACT.md
S4 is not checked here; it belongs to the code that binds the ports.

Messages (compact JSON, UTF-8, one datagram per message):
    cmd:  {"v":2,"cmd_seq":<int >= 0>,"wheels":{"left":<float, clamped +-3.0>,
           "right":<float, clamped +-3.0>},"stale":<0|1>}
    echo: {"v":2,"cmd_seq_echo":<int >= 0 or null>,"sim_t_s":<float>,
           "left":{"angle_rad":<float>,"vel_rad_s":<float or null>},
           "right":{"angle_rad":<float>,"vel_rad_s":<float or null>}}

Strict decode: exact key set at the top level and inside "wheels", "left" and
"right". The nested exact-key-set rule is NEW for v2; the single-wheel module
checked only the top level. The version key "v" must be the JSON integer 2
(not 2.0, not true). No duplicate keys at any depth. Bool rejected for numbers,
NaN / Infinity tokens rejected, int fields must be JSON ints (not floats), and
nulls allowed only for cmd_seq_echo and the two vel_rad_s. Single-wheel v1
bytes fail the key-set check, so they are refused.

Limits, as CONTRACT.md S3 states them: only the command wheels are clamped to
+-3.0 rad/s. The echo velocities and angles are NOT clamped; they pass through
unchanged. cmd_seq and cmd_seq_echo are non-negative ints with no explicit
upper bound. CONTRACT.md S3 says only "JSON integer, 0 or more" and is silent
on huge values. This module refuses a value past Python's int-to-text limit
with UdpMsgError on both encode and decode. That refusal is an implementation
choice, not a contract rule.

Usage (run from poc/ctrl-wheels):
    python3 -m pytest test_ctrl_wheels_udp.py -v
"""

from __future__ import annotations

import json
import math
import socket
from dataclasses import dataclass


HOST = "127.0.0.1"
# Contract constants (CONTRACT.md S4). Not referenced in this module; the bridge
# (task F14) imports them.
PORT_CMD = 5018
PORT_ECHO = 5017
# SIM ONLY. Per-wheel speed clamp in rad/s, applied on encode and on decode.
WHEEL_CLAMP_RAD_S = 3.0
STALE_SET = (0, 1)
VERSION = 2
RECV_BUFSIZE = 65535

CMD_KEYS = frozenset({"v", "cmd_seq", "wheels", "stale"})
WHEELS_KEYS = frozenset({"left", "right"})
ECHO_KEYS = frozenset({"v", "cmd_seq_echo", "sim_t_s", "left", "right"})
LEG_KEYS = frozenset({"angle_rad", "vel_rad_s"})


class UdpMsgError(ValueError):
    """Raised when a ctrl-wheels UDP message fails to encode or decode."""


@dataclass(frozen=True)
class Cmd:
    """Host -> sim command for the left and right wheel."""
    cmd_seq: int
    left_rad_s: float
    right_rad_s: float
    stale: int


@dataclass(frozen=True)
class Echo:
    """Sim -> host state echo for the left and right wheel."""
    cmd_seq_echo: int | None
    sim_t_s: float
    angle_rad_l: float
    vel_rad_s_l: float | None
    angle_rad_r: float
    vel_rad_s_r: float | None


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


def _check_version(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value != VERSION:
        raise UdpMsgError(f"v must be the JSON integer {VERSION}, got {value!r}")
    return value


def clamp(value: float, name: str = "wheel") -> float:
    """Limit one wheel speed to +-WHEEL_CLAMP_RAD_S. Rejects bool and non-finite input.

    name only labels the error message (for example "wheels.left").
    """
    v = _finite_real(name, value)
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


def _parse_json(data: bytes) -> object:
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise UdpMsgError(f"message is not valid UTF-8: {exc}") from exc
    try:
        return json.loads(
            text,
            parse_constant=_reject_constant,
            object_pairs_hook=_no_duplicate_keys,
        )
    except UdpMsgError:
        raise
    except (ValueError, RecursionError) as exc:
        raise UdpMsgError(f"message is not valid JSON: {exc}") from exc


def _check_keys(name: str, value: object, keys: frozenset[str]) -> dict[str, object]:
    if not isinstance(value, dict):
        raise UdpMsgError(f"{name} must be a JSON object, got {type(value).__name__}")
    missing = keys - value.keys()
    extra = value.keys() - keys
    if missing or extra:
        raise UdpMsgError(
            f"{name} key set mismatch: missing {sorted(missing)}, extra {sorted(extra)}"
        )
    return value


def _parse_object(data: bytes, keys: frozenset[str]) -> dict[str, object]:
    return _check_keys("message", _parse_json(data), keys)


def _dump(payload: dict[str, object]) -> bytes:
    try:
        text = json.dumps(payload, separators=(",", ":"), allow_nan=False)
    except (ValueError, OverflowError) as exc:
        # ValueError also covers CPython's int-to-text digit limit for huge ints.
        raise UdpMsgError(f"message cannot be encoded: {exc}") from exc
    return text.encode("utf-8")


def _encode_leg(side: str, angle_rad: float, vel_rad_s: float | None) -> dict[str, object]:
    return {
        "angle_rad": _finite_real(f"{side}.angle_rad", angle_rad),
        "vel_rad_s": None if vel_rad_s is None else _finite_real(f"{side}.vel_rad_s", vel_rad_s),
    }


def _decode_leg(side: str, value: object) -> tuple[float, float | None]:
    leg = _check_keys(side, value, LEG_KEYS)
    angle = _finite_real(f"{side}.angle_rad", leg["angle_rad"])
    vel_raw = leg["vel_rad_s"]
    vel = None if vel_raw is None else _finite_real(f"{side}.vel_rad_s", vel_raw)
    return angle, vel


def encode_cmd(cmd_seq: int, left_rad_s: float, right_rad_s: float, stale: int) -> bytes:
    """Encode a command datagram. Each wheel is clamped to +-WHEEL_CLAMP_RAD_S."""
    payload = {
        "v": VERSION,
        "cmd_seq": _check_seq("cmd_seq", cmd_seq),
        "wheels": {
            "left": clamp(left_rad_s, "wheels.left"),
            "right": clamp(right_rad_s, "wheels.right"),
        },
        "stale": _check_stale(stale),
    }
    return _dump(payload)


def decode_cmd(data: bytes) -> Cmd:
    """Decode and validate a command datagram. Each wheel is clamped on decode."""
    obj = _parse_object(data, CMD_KEYS)
    _check_version(obj["v"])
    wheels = _check_keys("wheels", obj["wheels"], WHEELS_KEYS)
    return Cmd(
        cmd_seq=_check_seq("cmd_seq", obj["cmd_seq"]),
        left_rad_s=clamp(wheels["left"], "wheels.left"),
        right_rad_s=clamp(wheels["right"], "wheels.right"),
        stale=_check_stale(obj["stale"]),
    )


def encode_echo(
    cmd_seq_echo: int | None,
    sim_t_s: float,
    angle_rad_l: float,
    vel_rad_s_l: float | None,
    angle_rad_r: float,
    vel_rad_s_r: float | None,
) -> bytes:
    """Encode an echo datagram. cmd_seq_echo and both vel_rad_s may be None."""
    payload = {
        "v": VERSION,
        "cmd_seq_echo": None if cmd_seq_echo is None else _check_seq("cmd_seq_echo", cmd_seq_echo),
        "sim_t_s": _finite_real("sim_t_s", sim_t_s),
        "left": _encode_leg("left", angle_rad_l, vel_rad_s_l),
        "right": _encode_leg("right", angle_rad_r, vel_rad_s_r),
    }
    return _dump(payload)


def decode_echo(data: bytes) -> Echo:
    """Decode and validate an echo datagram."""
    obj = _parse_object(data, ECHO_KEYS)
    _check_version(obj["v"])
    seq_raw = obj["cmd_seq_echo"]
    angle_l, vel_l = _decode_leg("left", obj["left"])
    angle_r, vel_r = _decode_leg("right", obj["right"])
    return Echo(
        cmd_seq_echo=None if seq_raw is None else _check_seq("cmd_seq_echo", seq_raw),
        sim_t_s=_finite_real("sim_t_s", obj["sim_t_s"]),
        angle_rad_l=angle_l,
        vel_rad_s_l=vel_l,
        angle_rad_r=angle_r,
        vel_rad_s_r=vel_r,
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
        except OSError:
            # Covers BlockingIOError (no datagram pending on a non-blocking socket)
            # and any other receive error: stop draining, never block.
            break
        payloads.append(data)
    return payloads
