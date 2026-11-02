#!/usr/bin/env python3
"""
Unit tests for the UDP JSON messages of the ctrl-wheel bridge (ctrl_wheel_udp).

Pure Python / pytest, stdlib only. The module under test is a flat module in
poc/ctrl-wheel and is imported by bare name. Tests bind only 127.0.0.1 port 0
(ephemeral); the production ports 5007/5008 are asserted as values, never bound.

Usage:
    cd poc/ctrl-wheel
    python3 -m pytest test_ctrl_wheel_udp.py -v
"""

import dataclasses
import json
import select
import socket
import sys
from pathlib import Path

import pytest

# Add the script directory to path for imports
sys.path.insert(0, str(Path(__file__).resolve().parent))

import ctrl_wheel_udp
from ctrl_wheel_udp import (
    HOST,
    PORT_CMD,
    PORT_ECHO,
    WHEEL_CLAMP_RAD_S,
    Cmd,
    Echo,
    UdpMsgError,
    clamp,
    decode_cmd,
    decode_echo,
    drain_all,
    encode_cmd,
    encode_echo,
    open_listener,
)

CMD_KEYS = {"cmd_seq", "wheel_rad_s", "stale"}
ECHO_KEYS = {"cmd_seq_echo", "sim_t_s", "angle_rad", "vel_rad_s"}


@pytest.fixture
def listener():
    sock = open_listener(0)
    try:
        yield sock
    finally:
        sock.close()


@pytest.fixture
def sender():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        yield sock
    finally:
        sock.close()


def _deliver(sender, sock, payloads):
    """Send payloads to the listener over loopback and wait until readable."""
    port = sock.getsockname()[1]
    for payload in payloads:
        sender.sendto(payload, (HOST, port))
    readable, _, _ = select.select([sock], [], [], 1.0)
    assert readable, "loopback datagram not delivered within 1 s"


class TestConstants:
    """Test: pinned module constants and error type."""

    def test_host(self):
        assert HOST == "127.0.0.1"

    def test_port_values(self):
        assert PORT_CMD == 5008
        assert PORT_ECHO == 5007

    def test_wheel_clamp(self):
        assert WHEEL_CLAMP_RAD_S == 3.0

    def test_udp_msg_error_is_value_error(self):
        assert issubclass(UdpMsgError, ValueError)


class TestDataclass:
    """Test: Cmd and Echo are frozen value objects with pinned field order."""

    def test_cmd_field_order(self):
        names = [f.name for f in dataclasses.fields(Cmd)]
        assert names == ["cmd_seq", "wheel_rad_s", "stale"]

    def test_echo_field_order(self):
        names = [f.name for f in dataclasses.fields(Echo)]
        assert names == ["cmd_seq_echo", "sim_t_s", "angle_rad", "vel_rad_s"]

    def test_cmd_frozen(self):
        c = Cmd(1, 0.5, 0)
        with pytest.raises(dataclasses.FrozenInstanceError):
            c.cmd_seq = 2  # type: ignore[misc]

    def test_echo_frozen(self):
        e = Echo(1, 0.0, 0.0, None)
        with pytest.raises(dataclasses.FrozenInstanceError):
            e.sim_t_s = 2.0  # type: ignore[misc]


class TestClamp:
    """Test: clamp limits wheel speed to +-WHEEL_CLAMP_RAD_S and rejects bad input."""

    def test_plus_limit_unchanged(self):
        assert clamp(3.0) == 3.0

    def test_minus_limit_unchanged(self):
        assert clamp(-3.0) == -3.0

    def test_just_over_plus_limit(self):
        assert clamp(3.0000001) == 3.0

    def test_far_negative_clamped(self):
        assert clamp(-9) == -3.0

    def test_default_limit_is_wheel_clamp(self):
        assert clamp(100.0) == WHEEL_CLAMP_RAD_S

    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
    def test_non_finite_rejected(self, bad):
        with pytest.raises(UdpMsgError):
            clamp(bad)

    def test_bool_rejected(self):
        with pytest.raises(UdpMsgError):
            clamp(True)


class TestEncodeCmd:
    """Test: encode_cmd produces compact JSON with the pinned keys and rules."""

    def test_golden_bytes(self):
        assert encode_cmd(1, 0.5, 0) == b'{"cmd_seq":1,"wheel_rad_s":0.5,"stale":0}'

    def test_key_names_pinned(self):
        assert set(json.loads(encode_cmd(0, 0.0, 0)).keys()) == CMD_KEYS

    def test_wheel_clamped_on_encode(self):
        assert json.loads(encode_cmd(4, 9.0, 1))["wheel_rad_s"] == 3.0

    def test_stale_one_accepted(self):
        assert json.loads(encode_cmd(4, 0.0, 1))["stale"] == 1

    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
    def test_non_finite_wheel_rejected(self, bad):
        with pytest.raises(UdpMsgError):
            encode_cmd(1, bad, 0)

    def test_bool_cmd_seq_rejected(self):
        with pytest.raises(UdpMsgError):
            encode_cmd(True, 0.0, 0)

    def test_negative_cmd_seq_rejected(self):
        with pytest.raises(UdpMsgError):
            encode_cmd(-1, 0.0, 0)

    @pytest.mark.parametrize("bad", [2, -1, True])
    def test_stale_out_of_set_rejected(self, bad):
        with pytest.raises(UdpMsgError):
            encode_cmd(1, 0.0, bad)


class TestDecodeCmd:
    """Test: decode_cmd is strict about keys, types, tokens and JSON shape."""

    def test_golden_decode(self):
        data = b'{"cmd_seq":1,"wheel_rad_s":0.5,"stale":0}'
        assert decode_cmd(data) == Cmd(1, 0.5, 0)

    def test_wheel_clamped_on_decode(self):
        data = b'{"cmd_seq":1,"wheel_rad_s":-9.0,"stale":0}'
        assert decode_cmd(data).wheel_rad_s == -3.0

    @pytest.mark.parametrize(
        "token",
        [b"NaN", b"Infinity", b"-Infinity"],
        ids=["nan", "inf", "neg_inf"],
    )
    def test_non_finite_token_rejected(self, token):
        data = b'{"cmd_seq":1,"wheel_rad_s":' + token + b',"stale":0}'
        with pytest.raises(UdpMsgError):
            decode_cmd(data)

    def test_missing_key_rejected(self):
        with pytest.raises(UdpMsgError):
            decode_cmd(b'{"cmd_seq":1,"wheel_rad_s":0.5}')

    def test_extra_key_rejected(self):
        with pytest.raises(UdpMsgError):
            decode_cmd(b'{"cmd_seq":1,"wheel_rad_s":0.5,"stale":0,"x":1}')

    @pytest.mark.parametrize(
        "data",
        [
            b'{"cmd_seq":true,"wheel_rad_s":0.5,"stale":0}',
            b'{"cmd_seq":1,"wheel_rad_s":0.5,"stale":true}',
            b'{"cmd_seq":1,"wheel_rad_s":true,"stale":0}',
            b'{"cmd_seq":1.0,"wheel_rad_s":0.5,"stale":0}',
            b'{"cmd_seq":"1","wheel_rad_s":0.5,"stale":0}',
            b'{"cmd_seq":null,"wheel_rad_s":0.5,"stale":0}',
            b'{"cmd_seq":1,"wheel_rad_s":"0.5","stale":0}',
        ],
        ids=[
            "bool_cmd_seq",
            "bool_stale",
            "bool_wheel",
            "float_cmd_seq",
            "str_cmd_seq",
            "null_cmd_seq",
            "str_wheel",
        ],
    )
    def test_wrong_type_rejected(self, data):
        with pytest.raises(UdpMsgError):
            decode_cmd(data)

    @pytest.mark.parametrize("stale", [2, -1])
    def test_stale_out_of_set_rejected(self, stale):
        data = json.dumps({"cmd_seq": 1, "wheel_rad_s": 0.5, "stale": stale}).encode()
        with pytest.raises(UdpMsgError):
            decode_cmd(data)

    def test_negative_cmd_seq_rejected(self):
        with pytest.raises(UdpMsgError):
            decode_cmd(b'{"cmd_seq":-1,"wheel_rad_s":0.5,"stale":0}')

    def test_invalid_utf8_rejected(self):
        with pytest.raises(UdpMsgError):
            decode_cmd(b'\xff\xfe{"cmd_seq":1}')

    def test_invalid_json_rejected(self):
        with pytest.raises(UdpMsgError):
            decode_cmd(b'{"cmd_seq":1,')

    @pytest.mark.parametrize("data", [b"[1,2,3]", b'"cmd"', b"null", b"1"])
    def test_non_object_rejected(self, data):
        with pytest.raises(UdpMsgError):
            decode_cmd(data)


class TestEncodeEcho:
    """Test: encode_echo produces compact JSON with the pinned keys and nulls."""

    def test_golden_bytes(self):
        assert encode_echo(None, 1.5, 0.25, None) == (
            b'{"cmd_seq_echo":null,"sim_t_s":1.5,"angle_rad":0.25,"vel_rad_s":null}'
        )

    def test_key_names_pinned(self):
        assert set(json.loads(encode_echo(3, 0.0, 0.0, 0.0)).keys()) == ECHO_KEYS

    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
    def test_non_finite_rejected(self, bad):
        with pytest.raises(UdpMsgError):
            encode_echo(1, 0.0, bad, 0.0)


class TestDecodeEcho:
    """Test: decode_echo is strict; nulls allowed only for cmd_seq_echo and vel_rad_s."""

    def test_golden_decode(self):
        data = b'{"cmd_seq_echo":2,"sim_t_s":1.5,"angle_rad":0.25,"vel_rad_s":-0.5}'
        assert decode_echo(data) == Echo(2, 1.5, 0.25, -0.5)

    @pytest.mark.parametrize(
        "token",
        [b"NaN", b"Infinity", b"-Infinity"],
        ids=["nan", "inf", "neg_inf"],
    )
    def test_non_finite_token_rejected(self, token):
        data = b'{"cmd_seq_echo":1,"sim_t_s":' + token + b',"angle_rad":0.0,"vel_rad_s":0.0}'
        with pytest.raises(UdpMsgError):
            decode_echo(data)

    def test_missing_key_rejected(self):
        with pytest.raises(UdpMsgError):
            decode_echo(b'{"cmd_seq_echo":1,"sim_t_s":1.0,"angle_rad":0.0}')

    def test_extra_key_rejected(self):
        with pytest.raises(UdpMsgError):
            decode_echo(
                b'{"cmd_seq_echo":1,"sim_t_s":1.0,"angle_rad":0.0,"vel_rad_s":0.0,"x":1}'
            )

    @pytest.mark.parametrize(
        "data",
        [
            b'{"cmd_seq_echo":1,"sim_t_s":null,"angle_rad":0.0,"vel_rad_s":0.0}',
            b'{"cmd_seq_echo":1,"sim_t_s":1.0,"angle_rad":null,"vel_rad_s":0.0}',
            b'{"cmd_seq_echo":true,"sim_t_s":1.0,"angle_rad":0.0,"vel_rad_s":0.0}',
            b'{"cmd_seq_echo":"1","sim_t_s":1.0,"angle_rad":0.0,"vel_rad_s":0.0}',
            b'{"cmd_seq_echo":1,"sim_t_s":true,"angle_rad":0.0,"vel_rad_s":0.0}',
            b'{"cmd_seq_echo":1,"sim_t_s":1.0,"angle_rad":"0.0","vel_rad_s":0.0}',
            b'{"cmd_seq_echo":1,"sim_t_s":1.0,"angle_rad":0.0,"vel_rad_s":true}',
        ],
        ids=[
            "null_sim_t",
            "null_angle",
            "bool_cmd_seq_echo",
            "str_cmd_seq_echo",
            "bool_sim_t",
            "str_angle",
            "bool_vel",
        ],
    )
    def test_wrong_type_or_disallowed_null_rejected(self, data):
        with pytest.raises(UdpMsgError):
            decode_echo(data)

    def test_invalid_utf8_rejected(self):
        with pytest.raises(UdpMsgError):
            decode_echo(b'\xff{"cmd_seq_echo":1}')

    def test_invalid_json_rejected(self):
        with pytest.raises(UdpMsgError):
            decode_echo(b'{"cmd_seq_echo":1,"sim_t_s":')

    @pytest.mark.parametrize("data", [b"[1,2,3,4]", b'"echo"', b"null", b"1.5"])
    def test_non_object_rejected(self, data):
        with pytest.raises(UdpMsgError):
            decode_echo(data)


class TestRoundTrip:
    """Test: encode -> real loopback UDP -> drain_all -> decode, on port 0."""

    def test_cmd_round_trip(self, listener, sender):
        payload = encode_cmd(7, -1.25, 1)
        _deliver(sender, listener, [payload])
        got = drain_all(listener)
        assert len(got) == 1
        assert decode_cmd(got[0]) == Cmd(7, -1.25, 1)

    def test_echo_round_trip(self, listener, sender):
        payload = encode_echo(3, 12.5, -0.1086, 0.75)
        _deliver(sender, listener, [payload])
        got = drain_all(listener)
        assert len(got) == 1
        assert decode_echo(got[0]) == Echo(3, 12.5, -0.1086, 0.75)

    def test_echo_null_vel_round_trip(self, listener, sender):
        payload = encode_echo(5, 1.0, 0.5, None)
        _deliver(sender, listener, [payload])
        got = drain_all(listener)
        assert len(got) == 1
        echo = decode_echo(got[0])
        assert echo == Echo(5, 1.0, 0.5, None)
        assert echo.vel_rad_s is None

    def test_echo_null_cmd_seq_echo_round_trip(self, listener, sender):
        payload = encode_echo(None, 2.0, -0.5, 0.0)
        _deliver(sender, listener, [payload])
        got = drain_all(listener)
        assert decode_echo(got[0]) == Echo(None, 2.0, -0.5, 0.0)


class TestOpenListener:
    """Test: open_listener is non-blocking UDP with no SO_REUSEADDR."""

    def test_non_blocking(self, listener):
        assert listener.getblocking() is False

    def test_bound_to_loopback(self, listener):
        host, port = listener.getsockname()
        assert host == HOST
        assert port != 0

    def test_second_listener_on_same_port_raises(self, listener):
        port = listener.getsockname()[1]
        with pytest.raises(OSError):
            open_listener(port)


class TestDrainAll:
    """Test: drain_all returns every pending datagram in arrival order."""

    def test_empty_socket_returns_empty_list(self, listener):
        assert drain_all(listener) == []

    def test_three_datagrams_in_order_then_empty(self, listener, sender):
        payloads = [encode_cmd(seq, 0.1 * seq, 0) for seq in (0, 1, 2)]
        _deliver(sender, listener, payloads)
        got = drain_all(listener)
        assert got == payloads
        assert [decode_cmd(d).cmd_seq for d in got] == [0, 1, 2]
        assert drain_all(listener) == []


class TestPinnedKeys:
    """Test: key names are a literal contract with the Webots controller."""

    def test_cmd_wire_keys_literal(self):
        assert set(json.loads(encode_cmd(1, 0.0, 0))) == {"cmd_seq", "wheel_rad_s", "stale"}

    def test_echo_wire_keys_literal(self):
        keys = set(json.loads(encode_echo(1, 0.0, 0.0, 0.0)))
        assert keys == {"cmd_seq_echo", "sim_t_s", "angle_rad", "vel_rad_s"}


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
