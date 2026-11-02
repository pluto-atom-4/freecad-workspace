#!/usr/bin/env python3
"""
Unit tests for the WHL wheel-velocity line protocol (ctrl_wheel_msg).

Pure Python / pytest, stdlib only. The module under test is a flat module in
poc/ctrl-wheel and is imported by bare name.

Usage:
    cd poc/ctrl-wheel
    python3 -m pytest test_ctrl_wheel_msg.py -v
"""

import dataclasses
import math
import sys
from pathlib import Path

import pytest

# Add the script directory to path for imports
sys.path.insert(0, str(Path(__file__).resolve().parent))

import ctrl_wheel_msg
from ctrl_wheel_msg import (
    PREFIX,
    SEQ_MOD,
    T_US_MAX,
    Whl,
    WhlError,
    checksum,
    format_whl,
    main,
    parse_line,
    step_lines,
)


def _with_checksum(body: str) -> str:
    """Append a valid *HH and newline to a WHL body (test helper, XOR inline)."""
    csum = 0
    for c in body:
        csum ^= ord(c)
    return f"{body}*{csum:02X}\n"


# Hand-computed golden lines (checksums from the main session, not derived here).
GOLDEN = [
    (Whl(1, 20000, 0.5, 21, 0), "WHL,1,20000,0.500000,21,0*54\n"),
    (Whl(65535, 4294967295, -1.25, -125, 2), "WHL,65535,4294967295,-1.250000,-125,2*5E\n"),
    (Whl(0, 0, 0.0, 0, 1), "WHL,0,0,0.000000,0,1*60\n"),
    (Whl(7, 140000, -0.000001, 0, 0), "WHL,7,140000,-0.000001,0,0*7F\n"),
]


class TestDataclass:
    """Test: Whl is a frozen value object with the pinned field order."""

    def test_field_order(self):
        names = [f.name for f in dataclasses.fields(Whl)]
        assert names == ["seq", "t_us", "model_rad_s", "raw", "state"]

    def test_frozen(self):
        w = Whl(1, 20000, 0.5, 21, 0)
        with pytest.raises(dataclasses.FrozenInstanceError):
            w.seq = 2  # type: ignore[misc]

    def test_prefix_and_limits(self):
        assert PREFIX == "WHL"
        assert SEQ_MOD == 65536
        assert T_US_MAX == 2**32 - 1

    def test_whl_error_is_value_error(self):
        assert issubclass(WhlError, ValueError)


class TestChecksum:
    """Test: checksum is XOR of ord values, ASCII only."""

    def test_single_char(self):
        assert checksum("A") == 0x41

    def test_pair_cancels(self):
        assert checksum("AA") == 0

    def test_empty(self):
        assert checksum("") == 0

    def test_golden_body(self):
        # Body of the golden line Whl(0, 0, 0.0, 0, 1); expected HH is 60.
        assert checksum("WHL,0,0,0.000000,0,1") == 0x60

    def test_non_ascii_rejected(self):
        with pytest.raises(WhlError):
            checksum("é")


class TestFormat:
    """Test: format_whl output, normalization and validation."""

    @pytest.mark.parametrize("w,expected", GOLDEN, ids=["basic", "max_min", "zero", "neg_tiny"])
    def test_golden_lines(self, w, expected):
        assert format_whl(w) == expected

    def test_output_ends_with_newline(self):
        assert format_whl(Whl(0, 0, 0.0, 0, 0)).endswith("\n")

    def test_hex_uppercase(self):
        line = format_whl(Whl(0, 0, 0.0, 0, 0))
        hh = line[line.rfind("*") + 1:line.rfind("*") + 3]
        assert len(hh) == 2
        assert all(c in "0123456789ABCDEF" for c in hh)

    def test_negative_zero_normalized(self):
        line = format_whl(Whl(0, 0, -0.0, 0, 0))
        assert "-0.000000" not in line
        assert line.startswith("WHL,0,0,0.000000,0,0*")

    def test_tiny_negative_rounds_to_normalized_zero(self):
        line = format_whl(Whl(0, 0, -1e-9, 0, 0))
        assert "-0.000000" not in line
        assert parse_line(line).model_rad_s == 0.0

    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
    def test_non_finite_model_formats_as_zero(self, bad):
        line = format_whl(Whl(1, 0, bad, 0, 0))
        assert line.startswith("WHL,1,0,0.000000,0,0*")
        assert "nan" not in line.lower()
        assert "inf" not in line.lower()
        assert parse_line(line).model_rad_s == 0.0

    def test_seq_taken_mod_65536(self):
        line = format_whl(Whl(65536, 0, 0.0, 0, 0))
        assert parse_line(line).seq == 0

    def test_seq_65537_wraps_to_1(self):
        line = format_whl(Whl(65537, 0, 0.0, 0, 0))
        assert parse_line(line).seq == 1

    def test_t_us_max_accepted(self):
        line = format_whl(Whl(0, T_US_MAX, 0.0, 0, 0))
        assert parse_line(line).t_us == T_US_MAX

    def test_t_us_over_max_rejected(self):
        with pytest.raises(WhlError):
            format_whl(Whl(0, T_US_MAX + 1, 0.0, 0, 0))

    def test_t_us_negative_rejected(self):
        with pytest.raises(WhlError):
            format_whl(Whl(0, -1, 0.0, 0, 0))

    @pytest.mark.parametrize("state", [3, -1])
    def test_state_out_of_set_rejected(self, state):
        with pytest.raises(WhlError):
            format_whl(Whl(0, 0, 0.0, 0, state))

    @pytest.mark.parametrize("field", ["seq", "t_us", "raw", "state"])
    def test_bool_rejected_in_format(self, field):
        base = dict(seq=0, t_us=0, model_rad_s=0.0, raw=0, state=0)
        base[field] = True
        with pytest.raises(WhlError):
            format_whl(Whl(**base))


class TestParse:
    """Test: parse_line accepts valid lines and rejects malformed ones."""

    @pytest.mark.parametrize("w,expected", GOLDEN, ids=["basic", "max_min", "zero", "neg_tiny"])
    def test_golden_lines_parse(self, w, expected):
        assert parse_line(expected) == w

    def test_round_trip(self):
        w = Whl(seq=1234, t_us=987654321, model_rad_s=-0.5, raw=-21, state=1)
        assert parse_line(format_whl(w)) == w

    def test_round_trip_zero(self):
        w = Whl(seq=0, t_us=0, model_rad_s=0.0, raw=0, state=0)
        assert parse_line(format_whl(w)) == w

    def test_crlf_equal_to_lf(self):
        line_lf = GOLDEN[0][1]
        line_crlf = line_lf.rstrip("\n") + "\r\n"
        assert parse_line(line_crlf) == parse_line(line_lf)

    def test_bare_lf_parses(self):
        assert parse_line(GOLDEN[2][1]).state == 1

    def test_missing_star_rejected(self):
        with pytest.raises(WhlError):
            parse_line("WHL,0,0,0.000000,0,1\n")

    def test_checksum_mismatch_rejected(self):
        # Golden line 1 has HH 54; 55 is wrong.
        with pytest.raises(WhlError):
            parse_line("WHL,1,20000,0.500000,21,0*55\n")

    def test_lowercase_hex_rejected(self):
        # Golden line 2 has HH 5E; lowercase must be rejected.
        with pytest.raises(WhlError):
            parse_line("WHL,65535,4294967295,-1.250000,-125,2*5e\n")

    def test_one_digit_checksum_rejected(self):
        with pytest.raises(WhlError):
            parse_line("WHL,0,0,0.000000,0,1*6\n")

    def test_three_digit_checksum_rejected(self):
        with pytest.raises(WhlError):
            parse_line("WHL,0,0,0.000000,0,1*060\n")

    def test_non_hex_checksum_rejected(self):
        with pytest.raises(WhlError):
            parse_line("WHL,0,0,0.000000,0,1*GG\n")

    def test_bad_prefix_with_valid_checksum_rejected(self):
        body = "IMU,1,20000,0.500000,21,0"
        with pytest.raises(WhlError):
            parse_line(_with_checksum(body))

    def test_too_few_fields_rejected(self):
        body = "WHL,1,20000,0.500000,21"
        with pytest.raises(WhlError):
            parse_line(_with_checksum(body))

    def test_too_many_fields_rejected(self):
        body = "WHL,1,20000,0.500000,21,0,0"
        with pytest.raises(WhlError):
            parse_line(_with_checksum(body))

    @pytest.mark.parametrize("tok", ["nan", "NaN", "inf", "-inf", "Infinity"])
    def test_non_finite_model_rejected(self, tok):
        body = f"WHL,1,20000,{tok},21,0"
        with pytest.raises(WhlError):
            parse_line(_with_checksum(body))

    def test_seq_overflow_rejected(self):
        body = "WHL,65536,20000,0.500000,21,0"
        with pytest.raises(WhlError):
            parse_line(_with_checksum(body))

    def test_seq_negative_rejected(self):
        body = "WHL,-1,20000,0.500000,21,0"
        with pytest.raises(WhlError):
            parse_line(_with_checksum(body))

    def test_t_us_overflow_rejected(self):
        body = "WHL,1,4294967296,0.500000,21,0"
        with pytest.raises(WhlError):
            parse_line(_with_checksum(body))

    @pytest.mark.parametrize("state", ["3", "-1"])
    def test_state_out_of_set_rejected(self, state):
        body = f"WHL,1,20000,0.500000,21,{state}"
        with pytest.raises(WhlError):
            parse_line(_with_checksum(body))

    def test_space_in_float_token_rejected(self):
        body = "WHL,1,20000,0.5 00,21,0"
        with pytest.raises(WhlError):
            parse_line(_with_checksum(body))

    def test_underscore_in_float_token_rejected(self):
        body = "WHL,1,20000,0_5,21,0"
        with pytest.raises(WhlError):
            parse_line(_with_checksum(body))

    def test_underscore_in_int_token_rejected(self):
        # int("1_0") is valid Python; the receiver must still reject it.
        body = "WHL,1_0,20000,0.500000,21,0"
        with pytest.raises(WhlError):
            parse_line(_with_checksum(body))

    def test_space_in_int_token_rejected(self):
        body = "WHL,1 0,20000,0.500000,21,0"
        with pytest.raises(WhlError):
            parse_line(_with_checksum(body))

    def test_exponent_and_plus_accepted(self):
        body = "WHL,1,20000,5e-1,+21,0"
        w = parse_line(_with_checksum(body))
        assert w.model_rad_s == 0.5
        assert w.raw == 21

    def test_exponent_and_plus_in_model(self):
        body = "WHL,2,40000,+2.5E+0,-21,0"
        w = parse_line(_with_checksum(body))
        assert w.model_rad_s == 2.5


class TestSeqWrap:
    """Test: seq wraps at 65536."""

    def test_format_wraps_65536_to_zero(self):
        assert parse_line(format_whl(Whl(65536, 0, 0.0, 0, 0))).seq == 0

    def test_step_lines_wrap_from_65535(self):
        lines = list(step_lines(1.0, 0.1, rate_hz=50.0, realtime=False, start_seq=65535))
        seqs = [parse_line(line).seq for line in lines]
        assert seqs == [65535, 0, 1, 2, 3]


class TestStepLines:
    """Test: step_lines generator contents and pacing."""

    def test_count_five_seconds_at_50hz(self):
        lines = list(step_lines(1.0, 5.0, realtime=False))
        assert len(lines) == 250

    def test_count_rate_option(self):
        lines = list(step_lines(1.0, 0.1, rate_hz=100.0, realtime=False))
        assert len(lines) == 10

    def test_lines_parse_back(self):
        lines = list(step_lines(1.0, 5.0, realtime=False))
        for i, line in enumerate(lines):
            w = parse_line(line)
            assert w.seq == i
            assert w.t_us == i * 20000
            assert w.model_rad_s == 1.0
            assert w.raw == 42
            assert w.state == 0

    def test_t_us_step_at_rate(self):
        lines = list(step_lines(0.0, 0.1, rate_hz=100.0, realtime=False))
        t = [parse_line(line).t_us for line in lines]
        assert t == [i * 10000 for i in range(10)]

    def test_start_seq_offset(self):
        lines = list(step_lines(0.0, 0.1, realtime=False, start_seq=100))
        assert [parse_line(line).seq for line in lines] == [100, 101, 102, 103, 104]

    @pytest.mark.parametrize(
        "value,raw",
        [(1.0, 42), (0.5, 21), (-0.5, -21), (3.0, 125), (-3.0, -125), (100.0, 125)],
    )
    def test_raw_mapping_and_clamp(self, value, raw):
        lines = list(step_lines(value, 0.02, realtime=False))
        assert len(lines) == 1
        assert parse_line(lines[0]).raw == raw

    def test_realtime_false_never_sleeps(self):
        sleeps = []
        lines = list(
            step_lines(
                1.0,
                0.1,
                realtime=False,
                clock=lambda: 0.0,
                sleep=lambda s: sleeps.append(s),
            )
        )
        assert len(lines) == 5
        assert sleeps == []

    def test_realtime_true_calls_injected_sleep(self):
        sleeps = []
        lines = list(
            step_lines(
                1.0,
                0.1,
                realtime=True,
                clock=lambda: 0.0,
                sleep=lambda s: sleeps.append(s),
            )
        )
        assert len(lines) == 5
        assert len(sleeps) >= 1
        assert all(math.isfinite(s) for s in sleeps)


class TestMain:
    """Test: CLI entry point, --no-sleep used so nothing really sleeps."""

    def test_no_sleep_prints_lines_and_returns_zero(self, capsys):
        rc = main(["--step", "0.5", "--seconds", "0.1", "--no-sleep"])
        assert rc == 0
        out_lines = capsys.readouterr().out.splitlines(keepends=True)
        assert len(out_lines) == 5
        for line in out_lines:
            assert line.endswith("\n")
            assert parse_line(line).model_rad_s == 0.5
        assert parse_line(out_lines[0]).raw == 21

    def test_rate_option(self, capsys):
        rc = main(["--step", "1.0", "--seconds", "0.1", "--rate", "100", "--no-sleep"])
        assert rc == 0
        out_lines = capsys.readouterr().out.splitlines(keepends=True)
        assert len(out_lines) == 10
        assert parse_line(out_lines[1]).t_us == 10000

    def test_default_is_realtime(self, monkeypatch):
        calls = []

        def spy(*args, **kwargs):
            calls.append(kwargs)
            return iter(())

        monkeypatch.setattr(ctrl_wheel_msg, "step_lines", spy)
        rc = main(["--step", "1.0", "--seconds", "1"])
        assert rc == 0
        assert len(calls) == 1
        assert calls[0].get("realtime") is True

    def test_no_sleep_sets_realtime_false(self, monkeypatch):
        calls = []

        def spy(*args, **kwargs):
            calls.append(kwargs)
            return iter(())

        monkeypatch.setattr(ctrl_wheel_msg, "step_lines", spy)
        rc = main(["--step", "1.0", "--seconds", "1", "--no-sleep"])
        assert rc == 0
        assert len(calls) == 1
        assert calls[0].get("realtime") is False


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
