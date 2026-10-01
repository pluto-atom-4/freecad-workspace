#!/usr/bin/env python3
"""
Unit tests for imu_wheel_msg.py (issue #282, sub-issue of #258).

Pure Python: no Webots, no hardware, must NOT import `controller` or `serial`.

Usage:
    cd poc/servo-wheel-dof
    mamba run -n servo-wheel-dof python3 -m pytest -q \
        webots/controllers/servo_wheel_dof/test_swd_imu_wheel_msg.py
"""
from __future__ import annotations


import json
import math
import sys
from pathlib import Path

import pytest

# Add the script directory to path for imports (this dir is not a package).
sys.path.insert(0, str(Path(__file__).resolve().parent))

from imu_wheel_msg import (  # noqa: E402
    ImuWheel,
    format_imu_wheel_msg,
    mock_imu_wheel,
    parse_imu_wheel_msg,
)

KEYS = ("roll", "pitch", "yaw", "wheel_angle")
VALID = {"roll": 0.1, "pitch": -0.2, "yaw": 0.3, "wheel_angle": 1.0}


def _encode(d: dict) -> bytes:
    return json.dumps(d).encode("utf-8")


def _raw(key: str, token: str) -> bytes:
    """JSON with `token` pasted verbatim as the value of `key`, others 0."""
    body = ",".join(f'"{k}":{token if k == key else "0"}' for k in KEYS)
    return ("{" + body + "}").encode("utf-8")


class TestMock:
    def test_returns_imuwheel(self):
        """mock_imu_wheel returns an ImuWheel."""
        assert isinstance(mock_imu_wheel(0.0), ImuWheel)

    def test_t0_all_zero(self):
        """sin(0) = 0 for every channel at t = 0."""
        m = mock_imu_wheel(0.0)
        for field in KEYS:
            assert getattr(m, field) == pytest.approx(0.0, abs=1e-9)

    def test_t_1_25(self):
        """Exact values at t = 1.25 s, built from the pinned amp/freq."""
        m = mock_imu_wheel(1.25)
        assert m.roll == pytest.approx(0.5 * math.sin(2 * math.pi * 0.2 * 1.25))
        assert m.pitch == pytest.approx(0.3 * math.sin(2 * math.pi * 0.1 * 1.25))
        assert m.yaw == pytest.approx(0.8 * math.sin(2 * math.pi * 0.05 * 1.25))
        assert m.wheel_angle == pytest.approx(
            1.5 * math.sin(2 * math.pi * 0.25 * 1.25)
        )

    def test_roll_peak_is_amplitude(self):
        """roll peaks at 0.5 when 2*pi*0.2*t = pi/2, i.e. t = 1.25 s."""
        assert mock_imu_wheel(1.25).roll == pytest.approx(0.5)


class TestRoundTrip:
    def test_format_then_parse(self):
        """parse(format(...)) returns the original message."""
        original = ImuWheel(0.1, -0.2, 0.3, 1.0)
        assert parse_imu_wheel_msg(format_imu_wheel_msg(5, original)) == original

    def test_format_returns_bytes_with_seq(self):
        """format returns bytes holding seq and all four keys."""
        msg = format_imu_wheel_msg(5, ImuWheel(0.1, -0.2, 0.3, 1.0))
        assert isinstance(msg, bytes)
        obj = json.loads(msg)
        assert obj["seq"] == 5
        for k in KEYS:
            assert k in obj

    def test_mock_round_trips(self):
        """A mock sample survives format then parse."""
        sample = mock_imu_wheel(1.25)
        assert parse_imu_wheel_msg(format_imu_wheel_msg(0, sample)) == sample

class TestParseAccepts:
    def test_valid(self):
        """A well-formed message parses to the matching ImuWheel."""
        assert parse_imu_wheel_msg(_encode(VALID)) == ImuWheel(0.1, -0.2, 0.3, 1.0)

    def test_ints_accepted(self):
        """JSON integers are accepted and become floats."""
        got = parse_imu_wheel_msg(b'{"roll":1,"pitch":0,"yaw":-1,"wheel_angle":2}')
        assert got == ImuWheel(1.0, 0.0, -1.0, 2.0)
        assert all(isinstance(getattr(got, k), float) for k in KEYS)

    def test_extra_keys_and_seq_ignored(self):
        """seq and unknown keys neither reject nor appear in the result."""
        obj = {**VALID, "seq": 12, "foo": "bar"}
        assert parse_imu_wheel_msg(_encode(obj)) == ImuWheel(0.1, -0.2, 0.3, 1.0)

    def test_seq_optional(self):
        """A message without seq is still valid."""
        assert parse_imu_wheel_msg(_encode(VALID)) is not None

class TestParseRejects:
    @pytest.mark.parametrize(
        "data", [b"{", b"", b"not json", b"\xff\xfe"], ids=["brace", "empty", "text", "badutf8"]
    )
    def test_malformed_json(self, data):
        """Malformed or undecodable input returns None, never raises."""
        assert parse_imu_wheel_msg(data) is None

    @pytest.mark.parametrize("data", [b"[1,2,3,4]", b"3", b"null", b'"str"'])
    def test_non_dict_top_level(self, data):
        """Top level must be a JSON object."""
        assert parse_imu_wheel_msg(data) is None

    @pytest.mark.parametrize("key", KEYS)
    def test_missing_key(self, key):
        """Each of the four keys is required, including wheel_angle."""
        obj = {k: v for k, v in VALID.items() if k != key}
        assert parse_imu_wheel_msg(_encode(obj)) is None

    @pytest.mark.parametrize("key", KEYS)
    def test_bool_rejected(self, key):
        """bool is an int subclass in Python; it must still be rejected."""
        assert parse_imu_wheel_msg(_raw(key, "true")) is None

    @pytest.mark.parametrize("key", KEYS)
    @pytest.mark.parametrize("token", ['"1.0"', "null", "[1]", '{"a":1}'])
    def test_wrong_type_rejected(self, key, token):
        """Strings, null, lists and objects are not numbers."""
        assert parse_imu_wheel_msg(_raw(key, token)) is None

    @pytest.mark.parametrize("key", KEYS)
    @pytest.mark.parametrize(
        "token", ["NaN", "Infinity", "-Infinity", "1e999", "1" + "0" * 400]
    )
    def test_non_finite_or_overflow_rejected(self, key, token):
        """NaN/Inf literals, float overflow and a huge int all give None."""
        assert parse_imu_wheel_msg(_raw(key, token)) is None

    def test_deep_nesting_rejected(self):
        """Pathological nesting returns None (RecursionError is caught)."""
        assert parse_imu_wheel_msg(b"[" * 100000) is None