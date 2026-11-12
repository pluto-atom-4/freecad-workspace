#!/usr/bin/env python3
"""Unit tests for the UDP v2 JSON messages of the bridge."""

import dataclasses
import json
import select
import socket
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from ctrl_wheels_udp import (
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

CMD_KEYS = {"v", "cmd_seq", "wheels", "stale"}
WHEELS_KEYS = {"left", "right"}
ECHO_KEYS = {"v", "cmd_seq_echo", "sim_t_s", "left", "right"}
LEG_KEYS = {"angle_rad", "vel_rad_s"}

NON_FINITE_TOKENS = [b"NaN", b"Infinity", b"-Infinity"]
HUGE_INT = b"1" + b"0" * 400

INT_DIGIT_LIMIT = sys.get_int_max_str_digits() if hasattr(sys, "get_int_max_str_digits") else 0
skip_without_int_limit = pytest.mark.skipif(
    INT_DIGIT_LIMIT == 0,
    reason="interpreter has no int-to-text digit limit, so no over-limit value exists",
)

V_BAD = [b"1", b"3", b"0", b"-2", b'"2"', b"2.0", b"true", b"false", b"null"]
CMD_SEQ_BAD = [b"-1", b"true", b"false", b"1.0", b"2.0", b'"1"', b"null"]
STALE_BAD = [b"2", b"-1", b"true", b"false", b"1.0", b"0.0", b'"0"', b"null"]
WHEEL_BAD = [b"true", b"false", b'"0.5"', b"null"] + NON_FINITE_TOKENS


def _raw_cmd(v=b"2", cmd_seq=b"1", left=b"0.5", right=b"-0.5", stale=b"0"):
    """Build command bytes from raw JSON tokens, so a test can inject any token."""
    return (
        b'{"v":' + v
        + b',"cmd_seq":' + cmd_seq
        + b',"wheels":{"left":' + left + b',"right":' + right + b"}"
        + b',"stale":' + stale + b"}"
    )


def _raw_echo(v=b"2", seq=b"1", sim_t=b"1.0", left_angle=b"0.0", left_vel=b"0.0",
              right_angle=b"0.0", right_vel=b"0.0"):
    """Build echo bytes from raw JSON tokens, so a test can inject any token."""
    return (
        b'{"v":' + v
        + b',"cmd_seq_echo":' + seq
        + b',"sim_t_s":' + sim_t
        + b',"left":{"angle_rad":' + left_angle + b',"vel_rad_s":' + left_vel + b"}"
        + b',"right":{"angle_rad":' + right_angle + b',"vel_rad_s":' + right_vel + b"}"
        + b"}"
    )


def _cmd_dict():
    """A valid command as a dict, for key-set tests (edit it, then _wire it)."""
    return {"v": 2, "cmd_seq": 1, "wheels": {"left": 0.5, "right": -0.5}, "stale": 0}


def _echo_dict():
    """A valid echo as a dict, for key-set tests (edit it, then _wire it)."""
    return {
        "v": 2,
        "cmd_seq_echo": 1,
        "sim_t_s": 1.0,
        "left": {"angle_rad": 0.0, "vel_rad_s": 0.0},
        "right": {"angle_rad": 0.0, "vel_rad_s": 0.0},
    }


def _wire(obj):
    """Serialise a dict to compact UTF-8 bytes. NaN tokens may appear if the dict has them."""
    return json.dumps(obj, separators=(",", ":")).encode("utf-8")


def _echo_kwargs(**overrides):
    """Keyword arguments for encode_echo with valid defaults, overridden per test."""
    kwargs = dict(
        cmd_seq_echo=1,
        sim_t_s=0.0,
        angle_rad_l=0.0,
        vel_rad_s_l=0.0,
        angle_rad_r=0.0,
        vel_rad_s_r=0.0,
    )
    kwargs.update(overrides)
    return kwargs


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
    """Test: pinned module constants (S3 clamp, S4 ports) and the error type."""

    def test_host(self):
        assert HOST == "127.0.0.1"

    def test_port_cmd_is_5018(self):
        assert PORT_CMD == 5018

    def test_port_echo_is_5017(self):
        assert PORT_ECHO == 5017

    def test_wheel_clamp(self):
        assert WHEEL_CLAMP_RAD_S == 3.0

    def test_udp_msg_error_is_value_error(self):
        assert issubclass(UdpMsgError, ValueError)


class TestDataclass:
    """Test: Cmd and Echo are frozen value objects with the pinned field order."""

    def test_cmd_field_order(self):
        names = [f.name for f in dataclasses.fields(Cmd)]
        assert names == ["cmd_seq", "left_rad_s", "right_rad_s", "stale"]

    def test_echo_field_order(self):
        names = [f.name for f in dataclasses.fields(Echo)]
        assert names == [
            "cmd_seq_echo",
            "sim_t_s",
            "angle_rad_l",
            "vel_rad_s_l",
            "angle_rad_r",
            "vel_rad_s_r",
        ]

    def test_cmd_frozen(self):
        c = Cmd(1, 0.5, -0.5, 0)
        with pytest.raises(dataclasses.FrozenInstanceError):
            c.cmd_seq = 2  # type: ignore[misc]

    def test_echo_frozen(self):
        e = Echo(1, 0.0, 0.0, None, 0.0, None)
        with pytest.raises(dataclasses.FrozenInstanceError):
            e.sim_t_s = 2.0  # type: ignore[misc]


class TestClamp:
    """Test: clamp limits one wheel speed to +-WHEEL_CLAMP_RAD_S and rejects bad input."""

    def test_plus_limit_unchanged(self):
        assert clamp(3.0) == 3.0

    def test_minus_limit_unchanged(self):
        assert clamp(-3.0) == -3.0

    def test_just_over_plus_limit(self):
        assert clamp(3.0000001) == 3.0

    def test_far_negative_clamped(self):
        assert clamp(-9) == -3.0

    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
    def test_non_finite_rejected(self, bad):
        with pytest.raises(UdpMsgError):
            clamp(bad)

    def test_bool_rejected(self):
        with pytest.raises(UdpMsgError):
            clamp(True)

    def test_string_rejected(self):
        with pytest.raises(UdpMsgError):
            clamp("0.5")


class TestGolden:
    """Test: hand-written golden bytes and literals (see the module docstring)."""

    def test_cmd_bytes(self):
        assert encode_cmd(1, 0.5, -1.25, 0) == (
            b'{"v":2,"cmd_seq":1,"wheels":{"left":0.5,"right":-1.25},"stale":0}'
        )

    def test_cmd_clamped_bytes(self):
        assert encode_cmd(2, 9.0, -9.0, 1) == (
            b'{"v":2,"cmd_seq":2,"wheels":{"left":3.0,"right":-3.0},"stale":1}'
        )

    def test_echo_bytes(self):
        assert encode_echo(3, 12.5, -0.1086, 0.75, 0.25, None) == (
            b'{"v":2,"cmd_seq_echo":3,"sim_t_s":12.5,'
            b'"left":{"angle_rad":-0.1086,"vel_rad_s":0.75},'
            b'"right":{"angle_rad":0.25,"vel_rad_s":null}}'
        )

    def test_echo_null_seq_bytes(self):
        assert encode_echo(None, 0.0, 0.0, None, 0.0, None) == (
            b'{"v":2,"cmd_seq_echo":null,"sim_t_s":0.0,'
            b'"left":{"angle_rad":0.0,"vel_rad_s":null},'
            b'"right":{"angle_rad":0.0,"vel_rad_s":null}}'
        )

    def test_cmd_decode(self):
        data = b'{"v":2,"cmd_seq":1,"wheels":{"left":0.5,"right":-1.25},"stale":0}'
        assert decode_cmd(data) == Cmd(1, 0.5, -1.25, 0)

    def test_echo_decode(self):
        data = (
            b'{"v":2,"cmd_seq_echo":3,"sim_t_s":12.5,'
            b'"left":{"angle_rad":-0.1086,"vel_rad_s":0.75},'
            b'"right":{"angle_rad":0.25,"vel_rad_s":null}}'
        )
        assert decode_echo(data) == Echo(3, 12.5, -0.1086, 0.75, 0.25, None)

    def test_echo_null_seq_decode(self):
        data = (
            b'{"v":2,"cmd_seq_echo":null,"sim_t_s":0.0,'
            b'"left":{"angle_rad":0.0,"vel_rad_s":null},'
            b'"right":{"angle_rad":0.0,"vel_rad_s":null}}'
        )
        assert decode_echo(data) == Echo(None, 0.0, 0.0, None, 0.0, None)


class TestEncodeCmd:
    """Test: encode_cmd writes the pinned keys, clamps each wheel and refuses bad input."""

    def test_key_names_pinned(self):
        obj = json.loads(encode_cmd(0, 0.0, 0.0, 0))
        assert set(obj.keys()) == CMD_KEYS
        assert set(obj["wheels"].keys()) == WHEELS_KEYS

    def test_v_is_integer_two(self):
        obj = json.loads(encode_cmd(0, 0.0, 0.0, 0))
        assert type(obj["v"]) is int
        assert obj["v"] == 2

    def test_left_clamped_independently(self):
        obj = json.loads(encode_cmd(4, 9.0, 0.5, 0))
        assert obj["wheels"] == {"left": 3.0, "right": 0.5}

    def test_right_clamped_independently(self):
        obj = json.loads(encode_cmd(4, 0.5, -9.0, 0))
        assert obj["wheels"] == {"left": 0.5, "right": -3.0}

    def test_stale_one_accepted(self):
        assert json.loads(encode_cmd(4, 0.0, 0.0, 1))["stale"] == 1

    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
    def test_non_finite_left_rejected(self, bad):
        with pytest.raises(UdpMsgError):
            encode_cmd(1, bad, 0.0, 0)

    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
    def test_non_finite_right_rejected(self, bad):
        with pytest.raises(UdpMsgError):
            encode_cmd(1, 0.0, bad, 0)

    @pytest.mark.parametrize("side", ["left", "right"])
    def test_error_names_the_wheel(self, side):
        values = {"left": 0.0, "right": 0.0}
        values[side] = float("nan")
        with pytest.raises(UdpMsgError) as info:
            encode_cmd(1, values["left"], values["right"], 0)
        assert f"wheels.{side}" in str(info.value)
        other = "right" if side == "left" else "left"
        assert f"wheels.{other}" not in str(info.value)

    @pytest.mark.parametrize(
        "left, right",
        [(True, 0.0), (0.0, True), ("0.5", 0.0), (0.0, "0.5")],
        ids=["bool_left", "bool_right", "str_left", "str_right"],
    )
    def test_bool_and_string_wheel_rejected(self, left, right):
        with pytest.raises(UdpMsgError):
            encode_cmd(1, left, right, 0)

    def test_bool_cmd_seq_rejected(self):
        with pytest.raises(UdpMsgError):
            encode_cmd(True, 0.0, 0.0, 0)

    def test_float_cmd_seq_rejected(self):
        with pytest.raises(UdpMsgError):
            encode_cmd(1.0, 0.0, 0.0, 0)

    @skip_without_int_limit
    def test_cmd_seq_too_large_for_text_rejected(self):
        over_limit = 10 ** INT_DIGIT_LIMIT
        with pytest.raises(UdpMsgError):
            encode_cmd(over_limit, 0.0, 0.0, 0)

    @skip_without_int_limit
    def test_negative_huge_cmd_seq_rejected_with_udp_msg_error(self):
        with pytest.raises(UdpMsgError) as info:
            encode_cmd(-(10 ** INT_DIGIT_LIMIT), 0.0, 0.0, 0)
        assert type(info.value) is UdpMsgError

    @skip_without_int_limit
    def test_huge_stale_rejected_with_udp_msg_error(self):
        with pytest.raises(UdpMsgError) as info:
            encode_cmd(1, 0.0, 0.0, 10 ** INT_DIGIT_LIMIT)
        assert type(info.value) is UdpMsgError

    @skip_without_int_limit
    @pytest.mark.parametrize("sign", [1, -1], ids=["positive", "negative"])
    def test_huge_wheel_rejected_with_udp_msg_error(self, sign):
        with pytest.raises(UdpMsgError) as info:
            encode_cmd(1, sign * 10 ** INT_DIGIT_LIMIT, 0.0, 0)
        assert type(info.value) is UdpMsgError

    def test_negative_cmd_seq_rejected(self):
        with pytest.raises(UdpMsgError):
            encode_cmd(-1, 0.0, 0.0, 0)

    @pytest.mark.parametrize("bad", [2, -1, True, 1.0])
    def test_stale_out_of_set_rejected(self, bad):
        with pytest.raises(UdpMsgError):
            encode_cmd(1, 0.0, 0.0, bad)


class TestDecodeCmd:
    """Test: decode_cmd is strict about keys, types, tokens, duplicates and JSON shape."""

    def test_left_clamped_independently_on_decode(self):
        assert decode_cmd(_raw_cmd(left=b"-9.0", right=b"0.5")) == Cmd(1, -3.0, 0.5, 0)

    def test_right_clamped_independently_on_decode(self):
        assert decode_cmd(_raw_cmd(left=b"0.5", right=b"9.0")) == Cmd(1, 0.5, 3.0, 0)

    def test_exact_limits_unchanged_on_decode(self):
        assert decode_cmd(_raw_cmd(left=b"3.0", right=b"-3.0")) == Cmd(1, 3.0, -3.0, 0)

    def test_stale_one_accepted(self):
        assert decode_cmd(_raw_cmd(stale=b"1")).stale == 1

    @pytest.mark.parametrize("token", V_BAD)
    def test_v_not_integer_two_rejected(self, token):
        with pytest.raises(UdpMsgError):
            decode_cmd(_raw_cmd(v=token))

    @pytest.mark.parametrize("token", CMD_SEQ_BAD)
    def test_cmd_seq_wrong_type_or_negative_rejected(self, token):
        with pytest.raises(UdpMsgError):
            decode_cmd(_raw_cmd(cmd_seq=token))

    @pytest.mark.parametrize("token", STALE_BAD)
    def test_stale_out_of_set_or_wrong_type_rejected(self, token):
        with pytest.raises(UdpMsgError):
            decode_cmd(_raw_cmd(stale=token))

    @pytest.mark.parametrize("token", WHEEL_BAD)
    @pytest.mark.parametrize("side", ["left", "right"])
    def test_wheel_wrong_type_or_disallowed_null_rejected(self, side, token):
        with pytest.raises(UdpMsgError):
            decode_cmd(_raw_cmd(**{side: token}))

    def test_huge_int_wheel_rejected(self):
        with pytest.raises(UdpMsgError):
            decode_cmd(_raw_cmd(left=HUGE_INT))

    @pytest.mark.parametrize("missing", ["v", "cmd_seq", "wheels", "stale"])
    def test_missing_top_level_key_rejected(self, missing):
        obj = _cmd_dict()
        del obj[missing]
        with pytest.raises(UdpMsgError):
            decode_cmd(_wire(obj))

    @pytest.mark.parametrize("extra", ["x", "wheel_rad_s"])
    def test_extra_top_level_key_rejected(self, extra):
        obj = _cmd_dict()
        obj[extra] = 0.0
        with pytest.raises(UdpMsgError):
            decode_cmd(_wire(obj))

    @pytest.mark.parametrize("missing", ["left", "right"])
    def test_missing_key_inside_wheels_rejected(self, missing):
        obj = _cmd_dict()
        del obj["wheels"][missing]
        with pytest.raises(UdpMsgError):
            decode_cmd(_wire(obj))

    @pytest.mark.parametrize("extra", ["centre", "wheel_rad_s"])
    def test_extra_key_inside_wheels_rejected(self, extra):
        obj = _cmd_dict()
        obj["wheels"][extra] = 0.0
        with pytest.raises(UdpMsgError):
            decode_cmd(_wire(obj))

    def test_empty_wheels_rejected(self):
        obj = _cmd_dict()
        obj["wheels"] = {}
        with pytest.raises(UdpMsgError):
            decode_cmd(_wire(obj))

    @pytest.mark.parametrize("value", [[0.5, -0.5], 0.5, None])
    def test_wheels_not_an_object_rejected(self, value):
        obj = _cmd_dict()
        obj["wheels"] = value
        with pytest.raises(UdpMsgError):
            decode_cmd(_wire(obj))

    @pytest.mark.parametrize(
        "data",
        [
            b'{"v":2,"v":2,"cmd_seq":1,"wheels":{"left":0.5,"right":-0.5},"stale":0}',
            b'{"v":2,"cmd_seq":1,"cmd_seq":1,"wheels":{"left":0.5,"right":-0.5},"stale":0}',
            b'{"v":2,"cmd_seq":1,"wheels":{"left":0.5,"left":0.5,"right":-0.5},"stale":0}',
            b'{"v":2,"cmd_seq":1,"wheels":{"left":0.5,"right":-0.5,"right":-0.5},"stale":0}',
        ],
        ids=["dup_v", "dup_cmd_seq", "dup_nested_left", "dup_nested_right"],
    )
    def test_duplicate_keys_rejected(self, data):
        with pytest.raises(UdpMsgError):
            decode_cmd(data)

    def test_v1_single_wheel_bytes_rejected(self):
        with pytest.raises(UdpMsgError):
            decode_cmd(b'{"cmd_seq":1,"wheel_rad_s":0.5,"stale":0}')

    def test_invalid_utf8_rejected(self):
        with pytest.raises(UdpMsgError):
            decode_cmd(b'\xff\xfe{"v":2}')

    def test_invalid_json_rejected(self):
        with pytest.raises(UdpMsgError):
            decode_cmd(b'{"v":2,')

    @pytest.mark.parametrize("data", [b"[1,2,3]", b'"cmd"', b"null", b"1", b"true"])
    def test_non_object_rejected(self, data):
        with pytest.raises(UdpMsgError):
            decode_cmd(data)

    def test_deeply_nested_closed_json_rejected(self):
        with pytest.raises(UdpMsgError):
            decode_cmd(b"[" * 100000 + b"]" * 100000)

    def test_deeply_nested_wheels_value_rejected(self):
        data = (
            b'{"v":2,"cmd_seq":1,"wheels":'
            + b"[" * 100000 + b"]" * 100000
            + b',"stale":0}'
        )
        with pytest.raises(UdpMsgError):
            decode_cmd(data)

    @pytest.mark.parametrize("side", ["left", "right"])
    def test_error_names_the_wheel(self, side):
        with pytest.raises(UdpMsgError) as info:
            decode_cmd(_raw_cmd(**{side: HUGE_INT}))
        assert f"wheels.{side}" in str(info.value)
        other = "right" if side == "left" else "left"
        assert f"wheels.{other}" not in str(info.value)

    @skip_without_int_limit
    def test_cmd_seq_too_many_digits_rejected(self):
        with pytest.raises(UdpMsgError):
            decode_cmd(_raw_cmd(cmd_seq=b"1" + b"0" * INT_DIGIT_LIMIT))


class TestEncodeEcho:
    """Test: encode_echo writes the pinned keys with nested legs and refuses bad input."""

    def test_key_names_pinned(self):
        obj = json.loads(encode_echo(3, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert set(obj.keys()) == ECHO_KEYS
        assert set(obj["left"].keys()) == LEG_KEYS
        assert set(obj["right"].keys()) == LEG_KEYS

    def test_v_is_integer_two(self):
        obj = json.loads(encode_echo(3, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert type(obj["v"]) is int
        assert obj["v"] == 2

    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
    @pytest.mark.parametrize(
        "slot",
        ["sim_t_s", "angle_rad_l", "vel_rad_s_l", "angle_rad_r", "vel_rad_s_r"],
    )
    def test_non_finite_rejected(self, slot, bad):
        with pytest.raises(UdpMsgError):
            encode_echo(**_echo_kwargs(**{slot: bad}))

    @pytest.mark.parametrize("slot", ["sim_t_s", "angle_rad_l", "angle_rad_r"])
    def test_null_sim_t_and_angle_rejected(self, slot):
        with pytest.raises(UdpMsgError):
            encode_echo(**_echo_kwargs(**{slot: None}))

    @pytest.mark.parametrize(
        "slot",
        ["sim_t_s", "angle_rad_l", "vel_rad_s_l", "angle_rad_r", "vel_rad_s_r"],
    )
    def test_bool_rejected(self, slot):
        with pytest.raises(UdpMsgError):
            encode_echo(**_echo_kwargs(**{slot: True}))

    @pytest.mark.parametrize("bad", [True, -1, 1.0])
    def test_cmd_seq_echo_wrong_value_rejected(self, bad):
        with pytest.raises(UdpMsgError):
            encode_echo(**_echo_kwargs(cmd_seq_echo=bad))

    @skip_without_int_limit
    def test_cmd_seq_echo_too_large_for_text_rejected(self):
        over_limit = 10 ** INT_DIGIT_LIMIT
        with pytest.raises(UdpMsgError):
            encode_echo(**_echo_kwargs(cmd_seq_echo=over_limit))

    @skip_without_int_limit
    @pytest.mark.parametrize("slot", ["sim_t_s", "angle_rad_l"])
    def test_huge_echo_float_rejected_with_udp_msg_error(self, slot):
        over_limit = 10 ** INT_DIGIT_LIMIT
        with pytest.raises(UdpMsgError) as info:
            encode_echo(**_echo_kwargs(**{slot: over_limit}))
        assert type(info.value) is UdpMsgError


class TestDecodeEcho:
    """Test: decode_echo is strict; nulls allowed only for cmd_seq_echo and the two vel_rad_s."""

    def test_null_velocities_decode(self):
        echo = decode_echo(_raw_echo(left_vel=b"null", right_vel=b"null"))
        assert echo == Echo(1, 1.0, 0.0, None, 0.0, None)

    def test_null_cmd_seq_echo_decodes(self):
        assert decode_echo(_raw_echo(seq=b"null")).cmd_seq_echo is None

    @pytest.mark.parametrize("token", NON_FINITE_TOKENS, ids=["nan", "inf", "neg_inf"])
    @pytest.mark.parametrize(
        "field",
        ["sim_t", "left_angle", "left_vel", "right_angle", "right_vel"],
    )
    def test_non_finite_token_rejected(self, field, token):
        with pytest.raises(UdpMsgError):
            decode_echo(_raw_echo(**{field: token}))

    @pytest.mark.parametrize("field", ["sim_t", "left_angle", "right_angle"])
    def test_null_sim_t_and_angle_rejected(self, field):
        with pytest.raises(UdpMsgError):
            decode_echo(_raw_echo(**{field: b"null"}))

    @pytest.mark.parametrize("token", [b"true", b"false", b'"1"', b'"0.0"'])
    @pytest.mark.parametrize(
        "field",
        ["sim_t", "left_angle", "left_vel", "right_angle", "right_vel"],
    )
    def test_bool_or_string_number_rejected(self, field, token):
        with pytest.raises(UdpMsgError):
            decode_echo(_raw_echo(**{field: token}))

    @pytest.mark.parametrize("token", [b"-1", b"true", b"1.0", b'"1"'])
    def test_cmd_seq_echo_wrong_type_or_negative_rejected(self, token):
        with pytest.raises(UdpMsgError):
            decode_echo(_raw_echo(seq=token))

    @pytest.mark.parametrize("token", V_BAD)
    def test_v_not_integer_two_rejected(self, token):
        with pytest.raises(UdpMsgError):
            decode_echo(_raw_echo(v=token))

    @pytest.mark.parametrize("missing", ["v", "cmd_seq_echo", "sim_t_s", "left", "right"])
    def test_missing_top_level_key_rejected(self, missing):
        obj = _echo_dict()
        del obj[missing]
        with pytest.raises(UdpMsgError):
            decode_echo(_wire(obj))

    @pytest.mark.parametrize("extra", ["x", "vel_rad_s"])
    def test_extra_top_level_key_rejected(self, extra):
        obj = _echo_dict()
        obj[extra] = 0.0
        with pytest.raises(UdpMsgError):
            decode_echo(_wire(obj))

    @pytest.mark.parametrize("side", ["left", "right"])
    @pytest.mark.parametrize("missing", ["angle_rad", "vel_rad_s"])
    def test_missing_key_inside_leg_rejected(self, side, missing):
        obj = _echo_dict()
        del obj[side][missing]
        with pytest.raises(UdpMsgError):
            decode_echo(_wire(obj))

    @pytest.mark.parametrize("side", ["left", "right"])
    @pytest.mark.parametrize("extra", ["centre", "vel"])
    def test_extra_key_inside_leg_rejected(self, side, extra):
        obj = _echo_dict()
        obj[side][extra] = 0.0
        with pytest.raises(UdpMsgError):
            decode_echo(_wire(obj))

    @pytest.mark.parametrize("side", ["left", "right"])
    @pytest.mark.parametrize("value", [[0.0, 0.0], 0.0, None])
    def test_leg_not_an_object_rejected(self, side, value):
        obj = _echo_dict()
        obj[side] = value
        with pytest.raises(UdpMsgError):
            decode_echo(_wire(obj))

    @pytest.mark.parametrize(
        "data",
        [
            b'{"v":2,"v":2,"cmd_seq_echo":1,"sim_t_s":1.0,'
            b'"left":{"angle_rad":0.0,"vel_rad_s":0.0},"right":{"angle_rad":0.0,"vel_rad_s":0.0}}',
            b'{"v":2,"cmd_seq_echo":1,"sim_t_s":1.0,"sim_t_s":1.0,'
            b'"left":{"angle_rad":0.0,"vel_rad_s":0.0},"right":{"angle_rad":0.0,"vel_rad_s":0.0}}',
            b'{"v":2,"cmd_seq_echo":1,"sim_t_s":1.0,'
            b'"left":{"angle_rad":0.0,"angle_rad":0.0,"vel_rad_s":0.0},"right":{"angle_rad":0.0,"vel_rad_s":0.0}}',
            b'{"v":2,"cmd_seq_echo":1,"sim_t_s":1.0,'
            b'"left":{"angle_rad":0.0,"vel_rad_s":0.0},"right":{"angle_rad":0.0,"vel_rad_s":0.0,"vel_rad_s":0.0}}',
        ],
        ids=["dup_v", "dup_top_sim_t", "dup_nested_left", "dup_nested_right"],
    )
    def test_duplicate_keys_rejected(self, data):
        with pytest.raises(UdpMsgError):
            decode_echo(data)

    def test_huge_int_sim_t_rejected(self):
        with pytest.raises(UdpMsgError):
            decode_echo(_raw_echo(sim_t=HUGE_INT))

    def test_v1_echo_bytes_rejected(self):
        with pytest.raises(UdpMsgError):
            decode_echo(b'{"cmd_seq_echo":1,"sim_t_s":1.0,"angle_rad":0.0,"vel_rad_s":0.0}')

    def test_invalid_utf8_rejected(self):
        with pytest.raises(UdpMsgError):
            decode_echo(b'\xff{"v":2}')

    def test_invalid_json_rejected(self):
        with pytest.raises(UdpMsgError):
            decode_echo(b'{"v":2,"cmd_seq_echo":')

    @pytest.mark.parametrize("data", [b"[1,2,3,4]", b'"echo"', b"null", b"1.5", b"false"])
    def test_non_object_rejected(self, data):
        with pytest.raises(UdpMsgError):
            decode_echo(data)

    def test_deeply_nested_closed_json_rejected(self):
        with pytest.raises(UdpMsgError):
            decode_echo(b"[" * 100000 + b"]" * 100000)

    def test_deeply_nested_leg_value_rejected(self):
        data = (
            b'{"v":2,"cmd_seq_echo":1,"sim_t_s":1.0,"left":'
            + b"[" * 100000 + b"]" * 100000
            + b',"right":{"angle_rad":0.0,"vel_rad_s":0.0}}'
        )
        with pytest.raises(UdpMsgError):
            decode_echo(data)

    @skip_without_int_limit
    def test_cmd_seq_echo_too_many_digits_rejected(self):
        with pytest.raises(UdpMsgError):
            decode_echo(_raw_echo(seq=b"1" + b"0" * INT_DIGIT_LIMIT))


class TestRoundTrip:
    """Test: encode -> real loopback UDP -> drain_all -> decode, on port 0 only."""

    def test_cmd_round_trip(self, listener, sender):
        payload = encode_cmd(7, -1.25, 2.5, 1)
        _deliver(sender, listener, [payload])
        got = drain_all(listener)
        assert len(got) == 1
        assert decode_cmd(got[0]) == Cmd(7, -1.25, 2.5, 1)

    def test_cmd_clamped_round_trip(self, listener, sender):
        payload = encode_cmd(8, 9.0, -9.0, 0)
        _deliver(sender, listener, [payload])
        got = drain_all(listener)
        assert len(got) == 1
        assert decode_cmd(got[0]) == Cmd(8, 3.0, -3.0, 0)

    def test_echo_round_trip(self, listener, sender):
        payload = encode_echo(3, 12.5, -0.1086, 0.75, 0.25, -0.5)
        _deliver(sender, listener, [payload])
        got = drain_all(listener)
        assert len(got) == 1
        assert decode_echo(got[0]) == Echo(3, 12.5, -0.1086, 0.75, 0.25, -0.5)

    def test_echo_null_vel_round_trip(self, listener, sender):
        payload = encode_echo(5, 1.0, 0.5, None, -0.5, None)
        _deliver(sender, listener, [payload])
        got = drain_all(listener)
        assert len(got) == 1
        echo = decode_echo(got[0])
        assert echo == Echo(5, 1.0, 0.5, None, -0.5, None)
        assert echo.vel_rad_s_l is None
        assert echo.vel_rad_s_r is None

    def test_echo_null_cmd_seq_echo_round_trip(self, listener, sender):
        payload = encode_echo(None, 2.0, -0.5, 0.0, 0.0, 0.0)
        _deliver(sender, listener, [payload])
        got = drain_all(listener)
        assert decode_echo(got[0]) == Echo(None, 2.0, -0.5, 0.0, 0.0, 0.0)

    def test_echo_velocity_not_clamped_round_trip(self, listener, sender):
        payload = encode_echo(4, 1.0, 0.0, 12.5, 0.0, -12.5)
        _deliver(sender, listener, [payload])
        got = drain_all(listener)
        assert len(got) == 1
        assert decode_echo(got[0]) == Echo(4, 1.0, 0.0, 12.5, 0.0, -12.5)

    def test_cmd_seq_has_no_upper_bound_round_trip(self, listener, sender):
        payload = encode_cmd(2**40, 0.0, 0.0, 0)
        _deliver(sender, listener, [payload])
        got = drain_all(listener)
        assert len(got) == 1
        assert decode_cmd(got[0]) == Cmd(2**40, 0.0, 0.0, 0)


class TestOpenListener:
    """Test: open_listener is a non-blocking UDP socket with no SO_REUSEADDR."""

    def test_non_blocking(self, listener):
        assert listener.getblocking() is False

    def test_bound_to_loopback(self, listener):
        host, port = listener.getsockname()
        assert host == HOST
        assert port != 0

    def test_no_reuseaddr(self, listener):
        assert listener.getsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR) == 0

    def test_second_listener_on_same_port_raises(self, listener):
        port = listener.getsockname()[1]
        with pytest.raises(OSError):
            open_listener(port)


class TestDrainAll:
    """Test: drain_all returns every pending datagram in arrival order and never blocks."""

    def test_empty_socket_returns_empty_list(self, listener):
        assert drain_all(listener) == []

    def test_three_datagrams_in_order_then_empty(self, listener, sender):
        payloads = [encode_cmd(seq, 0.1 * seq, -0.1 * seq, 0) for seq in (0, 1, 2)]
        _deliver(sender, listener, payloads)
        got = drain_all(listener)
        assert got == payloads
        assert [decode_cmd(d).cmd_seq for d in got] == [0, 1, 2]
        assert drain_all(listener) == []


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
