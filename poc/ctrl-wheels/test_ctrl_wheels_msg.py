#!/usr/bin/env python3
"""
Unit tests for the WHL2 twin-wheel line protocol in ctrl_wheels_msg.

SIM ONLY. The suite pins the API that ctrl_wheels_msg.py implements:
WhlError(ValueError), Whl (single-wheel, 5 fields), checksum(body), PREFIX
== "WHL2", FIELD_COUNT == 7, the frozen dataclass
Whl2(seq, t_us, model_rad_s, raw_l, raw_r, state), format_whl2(Whl2) -> str,
parse_line(str) -> Whl2 (WHL2 only) and split(Whl2) -> (Whl, Whl). It covers
format, parse, the reject rules and split. step_lines and main are covered by
test_ctrl_wheels_msg_steps.py.

Golden origin: the accept and reject lines below are copied as literals from
CONTRACT.md S7 (poc/ctrl-wheels). The "-0 model" case uses the real float
-0.0. Expected values are never taken from the implementation.

Signs are UNCONFIRMED: the rawR negation on the chip side, the Webots-side
+1/+1 direction owner-observed (2026-11-06, sim only; chip-side signs
UNCONFIRMED) and the forward-positive model definition (CONTRACT.md S2).
Nothing here is validated on real hardware.

The reverse-direction reject ("WHL2 line against the single-wheel WHL v1
parser") is planned in the BRC and FW follow-up. It is not tested in this
directory, because the single-wheel v1 parser is not present here.

Usage:
    cd poc/ctrl-wheels
    python3 -m pytest test_ctrl_wheels_msg.py -v
"""

import dataclasses
import sys
from pathlib import Path

import pytest

# Add the script directory to path for imports
sys.path.insert(0, str(Path(__file__).resolve().parent))

import ctrl_wheels_msg
from ctrl_wheels_msg import (
    Whl,
    Whl2,
    WhlError,
    checksum,
    format_whl,
    format_whl2,
    parse_line,
    split,
)


# Limits written out as test literals on purpose. They are not imported from
# the module, so a change there cannot silently move the expected values.
SEQ_LIMIT = 65536
T_US_LIMIT = 4294967295
INT32_MIN_LITERAL = -2147483648
INT32_MAX_LITERAL = 2147483647


def _with_checksum(body):
    """Return body + "*HH" + newline, with HH the XOR of the body bytes.

    The XOR is written out here on purpose, so these tests do not reuse the
    implementation's checksum(). It is used only for bodies that must pass the
    checksum and then fail a field rule.
    """
    csum = 0
    for c in body:
        csum ^= ord(c)
    return f"{body}*{csum:02X}\n"


# Accept goldens: (expected Whl2, exact line). Whl2 field order is
# seq, t_us, model_rad_s, raw_l, raw_r, state. Copied from CONTRACT.md S7.
GOLDEN_ACCEPT = [
    (Whl2(1, 1000000, 0.5, 10, 12, 0), "WHL2,1,1000000,0.500000,10,12,0*48\n"),
    (Whl2(2, 1020000, 0.5, 10, -10, 0), "WHL2,2,1020000,0.500000,10,-10,0*66\n"),
    (Whl2(3, 1040000, -0.5, -10, -10, 0), "WHL2,3,1040000,-0.500000,-10,-10,0*61\n"),
    (Whl2(4, 1060000, -0.0, 0, 0, 0), "WHL2,4,1060000,0.000000,0,0,0*4C\n"),
    (Whl2(5, 1080000, 1.25, 20, -20, 0), "WHL2,5,1080000,1.250000,20,-20,0*68\n"),
    (Whl2(6, 1100000, 0.0, 0, 0, 1), "WHL2,6,1100000,0.000000,0,0,1*48\n"),
    (Whl2(7, 1120000, 0.0, 0, 0, 2), "WHL2,7,1120000,0.000000,0,0,2*48\n"),
    (Whl2(8, 1140000, 0.75, 15, -15, 0), "WHL2,8,1140000,0.750000,15,-15,0*6C\n"),
    (Whl2(65535, 1160000, 0.25, 5, -5, 0), "WHL2,65535,1160000,0.250000,5,-5,0*63\n"),
    (Whl2(9, 4294967295, 0.25, 5, -5, 0), "WHL2,9,4294967295,0.250000,5,-5,0*51\n"),
    (
        Whl2(10, 1180000, 1.0, -2147483648, 2147483647, 0),
        "WHL2,10,1180000,1.000000,-2147483648,2147483647,0*55\n",
    ),
]

GOLDEN_ACCEPT_IDS = [
    "positive",
    "raw_l_pos_raw_r_neg",
    "both_raw_neg",
    "neg_zero_model",
    "state0_mixed",
    "state1",
    "state2",
    "upper_hex_letter_hh",
    "seq_65535",
    "t_us_max",
    "int32_extremes",
]

# Reject goldens that apply to the WHL2 parser. Four entries (six_fields,
# eight_fields, state_3, nan_model) are altered WHL2 bodies whose HH is correct
# over the altered body. bad_hh is the only entry with a wrong HH, on purpose:
# its body is the same as the reverse-direction case in CONTRACT.md S7, whose
# correct HH is 56, and this literal uses 57. whl_v1_line is a whole
# single-wheel WHL v1 line (prefix WHL, 6 fields) with its own HH, not an
# altered WHL2 body; it is refused by the prefix and field count.
# The checksum is compared first, so a wrong HH would hide the rule under test.
# The reverse case (WHL2 line against the single-wheel v1 parser) is not tested
# here; see the module docstring.
GOLDEN_REJECT = [
    ("six_fields", "WHL2,20,1200000,0.500000,10,-10*4A\n"),
    ("eight_fields", "WHL2,20,1200000,0.500000,10,-10,0,0*4A\n"),
    ("whl_v1_line", "WHL,5,6,0.000000,7,2*67\n"),
    ("bad_hh", "WHL2,20,1200000,0.500000,10,-10,0*57\n"),
    ("state_3", "WHL2,20,1200000,0.500000,10,-10,3*55\n"),
    ("nan_model", "WHL2,20,1200000,nan,10,-10,0*2C\n"),
]


class TestDataclass:
    """Whl2 and Whl are frozen value objects with the pinned field order."""

    def test_whl2_field_order(self):
        names = [f.name for f in dataclasses.fields(Whl2)]
        assert names == ["seq", "t_us", "model_rad_s", "raw_l", "raw_r", "state"]

    def test_whl2_frozen(self):
        w = Whl2(1, 1000000, 0.5, 10, 12, 0)
        with pytest.raises(dataclasses.FrozenInstanceError):
            w.raw_l = 2  # type: ignore[misc]

    def test_whl_v1_field_order(self):
        names = [f.name for f in dataclasses.fields(Whl)]
        assert names == ["seq", "t_us", "model_rad_s", "raw", "state"]

    def test_whl_v1_is_frozen(self):
        w = Whl(1, 20000, 0.5, 21, 0)
        with pytest.raises(dataclasses.FrozenInstanceError):
            w.seq = 2  # type: ignore[misc]

    def test_prefix_and_field_count(self):
        assert ctrl_wheels_msg.PREFIX == "WHL2"
        assert ctrl_wheels_msg.FIELD_COUNT == 7

    def test_whl_error_is_value_error(self):
        assert issubclass(WhlError, ValueError)


class TestChecksum:
    """checksum is the XOR of ord values of an ASCII body."""

    def test_single_char(self):
        assert checksum("A") == 0x41

    def test_pair_cancels(self):
        assert checksum("AA") == 0

    def test_empty(self):
        assert checksum("") == 0

    def test_golden_body(self):
        # Body of the accept golden "WHL2,1,1000000,0.500000,10,12,0*48".
        assert checksum("WHL2,1,1000000,0.500000,10,12,0") == 0x48

    def test_non_ascii_rejected(self):
        with pytest.raises(WhlError):
            checksum("é")


class TestFormat:
    """format_whl2 output, normalisation and validation."""

    @pytest.mark.parametrize("w,expected", GOLDEN_ACCEPT, ids=GOLDEN_ACCEPT_IDS)
    def test_golden_lines(self, w, expected):
        assert format_whl2(w) == expected

    def test_tiny_negative_prints_zero(self):
        line = format_whl2(Whl2(0, 0, -1e-9, 0, 0, 0))
        assert "-0.000000" not in line
        assert line.startswith("WHL2,0,0,0.000000,0,0,0*")
        assert parse_line(line).model_rad_s == 0.0

    @pytest.mark.parametrize(
        "bad",
        [float("nan"), float("inf"), float("-inf")],
        ids=["nan", "inf", "neg_inf"],
    )
    def test_non_finite_model_rejected(self, bad):
        with pytest.raises(WhlError):
            format_whl2(Whl2(seq=0, t_us=0, model_rad_s=bad, raw_l=0, raw_r=0, state=0))

    @pytest.mark.parametrize("bad", ["0.5", None])
    def test_non_real_model_rejected(self, bad):
        with pytest.raises(WhlError):
            format_whl2(Whl2(0, 0, bad, 0, 0, 0))

    def test_bool_model_refused(self):
        # Implementation: _model_to_float refuses bool before the float check.
        with pytest.raises(WhlError):
            format_whl2(Whl2(0, 0, True, 0, 0, 0))

    def test_int_model_accepted_as_float(self):
        # Implementation: an int model (not bool) is converted by float() and
        # printed with %.6f, so 5 prints as 5.000000.
        assert format_whl2(Whl2(0, 0, 5, 0, 0, 0)).startswith("WHL2,0,0,5.000000,0,0,0*")

    @pytest.mark.parametrize(
        "model",
        [-999999999999.0, 9999999999999.0],
        ids=["neg_20_chars", "pos_20_chars"],
    )
    def test_model_text_of_20_chars_round_trips(self, model):
        assert len(f"{model:.6f}") == 20
        line = format_whl2(Whl2(0, 0, model, 0, 0, 0))
        assert parse_line(line).model_rad_s == model

    @pytest.mark.parametrize(
        "model",
        [-1e12, 1e13],
        ids=["neg_21_chars", "pos_21_chars"],
    )
    def test_model_text_of_21_chars_refused(self, model):
        assert len(f"{model:.6f}") == 21
        with pytest.raises(WhlError):
            format_whl2(Whl2(0, 0, model, 0, 0, 0))

    def test_huge_model_refused_in_twin_format(self):
        with pytest.raises(WhlError):
            format_whl2(Whl2(0, 0, 1e300, 0, 0, 0))

    def test_huge_model_refused_in_v1_format(self):
        # format_whl shares _model_text, so the same length guard applies.
        with pytest.raises(WhlError):
            format_whl(Whl(0, 0, 1e300, 0, 0))

    @pytest.mark.parametrize("field", ["raw_l", "raw_r"])
    @pytest.mark.parametrize("bad", [True, False, 1.0, "5", None])
    def test_raw_non_int_rejected(self, field, bad):
        base = dict(seq=0, t_us=0, model_rad_s=0.0, raw_l=0, raw_r=0, state=0)
        base[field] = bad
        with pytest.raises(WhlError):
            format_whl2(Whl2(**base))

    @pytest.mark.parametrize("field", ["raw_l", "raw_r"])
    @pytest.mark.parametrize("bad", [INT32_MAX_LITERAL + 1, INT32_MIN_LITERAL - 1])
    def test_raw_outside_int32_rejected(self, field, bad):
        base = dict(seq=0, t_us=0, model_rad_s=0.0, raw_l=0, raw_r=0, state=0)
        base[field] = bad
        with pytest.raises(WhlError):
            format_whl2(Whl2(**base))

    @pytest.mark.parametrize("field", ["raw_l", "raw_r"])
    @pytest.mark.parametrize("ok", [INT32_MIN_LITERAL, INT32_MAX_LITERAL, -125, 125])
    def test_raw_int32_range_accepted(self, field, ok):
        base = dict(seq=0, t_us=0, model_rad_s=0.0, raw_l=0, raw_r=0, state=0)
        base[field] = ok
        line = format_whl2(Whl2(**base))
        assert getattr(parse_line(line), field) == ok

    @pytest.mark.parametrize("state", [3, -1])
    def test_state_out_of_set_rejected(self, state):
        with pytest.raises(WhlError):
            format_whl2(Whl2(0, 0, 0.0, 0, 0, state))

    @pytest.mark.parametrize("field", ["seq", "t_us", "raw_l", "raw_r", "state"])
    def test_bool_rejected(self, field):
        base = dict(seq=0, t_us=0, model_rad_s=0.0, raw_l=0, raw_r=0, state=0)
        base[field] = True
        with pytest.raises(WhlError):
            format_whl2(Whl2(**base))

    def test_t_us_max_accepted(self):
        line = format_whl2(Whl2(0, T_US_LIMIT, 0.0, 0, 0, 0))
        assert parse_line(line).t_us == T_US_LIMIT

    def test_t_us_over_max_rejected(self):
        with pytest.raises(WhlError):
            format_whl2(Whl2(0, T_US_LIMIT + 1, 0.0, 0, 0, 0))

    def test_t_us_negative_rejected(self):
        with pytest.raises(WhlError):
            format_whl2(Whl2(0, -1, 0.0, 0, 0, 0))

    def test_seq_65536_wraps_to_0(self):
        assert format_whl2(Whl2(SEQ_LIMIT, 0, 0.0, 0, 0, 0)).startswith("WHL2,0,0,")

    def test_seq_65537_wraps_to_1(self):
        assert format_whl2(Whl2(SEQ_LIMIT + 1, 0, 0.0, 0, 0, 0)).startswith("WHL2,1,0,")

    def test_negative_seq_wraps_to_65535(self):
        # Python -1 % 65536 == 65535, per the contract's "mod 65536".
        assert format_whl2(Whl2(-1, 0, 0.0, 0, 0, 0)).startswith("WHL2,65535,0,")

    def test_negative_raw_r_kept_as_written(self):
        line = format_whl2(Whl2(1, 0, 0.0, 7, -3, 0))
        assert line.startswith("WHL2,1,0,0.000000,7,-3,0*")
        parsed = parse_line(line)
        assert parsed.raw_l == 7
        assert parsed.raw_r == -3

    def test_tiny_positive_model_prints_zero(self):
        line = format_whl2(Whl2(1, 0, 1e-9, 0, 0, 0))
        assert line.startswith("WHL2,1,0,0.000000,0,0,0*")


class TestParse:
    """parse_line accepts WHL2 lines and rejects malformed ones."""

    @pytest.mark.parametrize("w,expected", GOLDEN_ACCEPT, ids=GOLDEN_ACCEPT_IDS)
    def test_golden_lines_parse(self, w, expected):
        assert parse_line(expected) == w

    @pytest.mark.parametrize("ending", ["\n", "\r\n", ""], ids=["lf", "crlf", "none"])
    @pytest.mark.parametrize("w,expected", GOLDEN_ACCEPT, ids=GOLDEN_ACCEPT_IDS)
    def test_line_endings_parse(self, w, expected, ending):
        line = expected.rstrip("\n") + ending
        assert parse_line(line) == w

    @pytest.mark.parametrize(
        "ending",
        ["\n\n", "\r\r\n", "\r"],
        ids=["double_lf", "double_cr_lf", "lone_cr"],
    )
    def test_non_standard_line_endings_rejected(self, ending):
        line = "WHL2,1,1000000,0.500000,10,12,0*48" + ending
        with pytest.raises(WhlError):
            parse_line(line)

    def test_round_trip_negative(self):
        w = Whl2(seq=1234, t_us=987654321, model_rad_s=-0.5, raw_l=-21, raw_r=21, state=1)
        assert parse_line(format_whl2(w)) == w

    def test_round_trip_int32_extremes(self):
        w = Whl2(
            seq=65535,
            t_us=T_US_LIMIT,
            model_rad_s=0.25,
            raw_l=INT32_MIN_LITERAL,
            raw_r=INT32_MAX_LITERAL,
            state=2,
        )
        assert parse_line(format_whl2(w)) == w

    def test_round_trip_zero(self):
        w = Whl2(seq=0, t_us=0, model_rad_s=0.0, raw_l=0, raw_r=0, state=0)
        assert parse_line(format_whl2(w)) == w

    @pytest.mark.parametrize("name,line", GOLDEN_REJECT, ids=[n for n, _ in GOLDEN_REJECT])
    def test_golden_reject_lines(self, name, line):
        with pytest.raises(WhlError):
            parse_line(line)

    def test_empty_string_rejected(self):
        with pytest.raises(WhlError):
            parse_line("")

    def test_missing_star_rejected(self):
        with pytest.raises(WhlError):
            parse_line("WHL2,1,1000000,0.500000,10,12,0\n")

    def test_two_star_markers_rejected(self):
        # The HH is correct over the body that still contains the first '*'.
        # The state token "0*48" is then refused by the digit pattern.
        inner = "WHL2,1,1000000,0.500000,10,12,0*48"
        with pytest.raises(WhlError):
            parse_line(_with_checksum(inner))

    def test_lowercase_hex_rejected(self):
        # Golden line 8 has HH 6C; lowercase must be rejected.
        with pytest.raises(WhlError):
            parse_line("WHL2,8,1140000,0.750000,15,-15,0*6c\n")

    def test_one_digit_checksum_rejected(self):
        with pytest.raises(WhlError):
            parse_line("WHL2,1,1000000,0.500000,10,12,0*4\n")

    def test_three_digit_checksum_rejected(self):
        with pytest.raises(WhlError):
            parse_line("WHL2,1,1000000,0.500000,10,12,0*048\n")

    def test_non_hex_checksum_rejected(self):
        with pytest.raises(WhlError):
            parse_line("WHL2,1,1000000,0.500000,10,12,0*GG\n")

    def test_wrong_hh_on_valid_body_rejected(self):
        # Golden line 1 has HH 48; 49 is wrong.
        with pytest.raises(WhlError):
            parse_line("WHL2,1,1000000,0.500000,10,12,0*49\n")

    @pytest.mark.parametrize(
        "body",
        [
            "WHL2,65536,1000000,0.500000,10,12,0",
            "WHL2,-1,1000000,0.500000,10,12,0",
        ],
        ids=["seq_65536", "seq_sign_refused_by_digit_pattern"],
    )
    def test_seq_out_of_range_rejected(self, body):
        with pytest.raises(WhlError):
            parse_line(_with_checksum(body))

    @pytest.mark.parametrize(
        "body",
        [
            "WHL2,1,4294967296,0.500000,10,12,0",
            "WHL2,1,-1,0.500000,10,12,0",
        ],
        ids=["t_us_overflow", "t_us_sign_refused_by_digit_pattern"],
    )
    def test_t_us_out_of_range_rejected(self, body):
        with pytest.raises(WhlError):
            parse_line(_with_checksum(body))

    @pytest.mark.parametrize(
        "body",
        [
            "WHL2,1,1000000,0.500000,2147483648,12,0",
            "WHL2,1,1000000,0.500000,-2147483649,12,0",
            "WHL2,1,1000000,0.500000,10,2147483648,0",
            "WHL2,1,1000000,0.500000,10,-2147483649,0",
        ],
        ids=["raw_l_high", "raw_l_low", "raw_r_high", "raw_r_low"],
    )
    def test_raw_outside_int32_rejected(self, body):
        with pytest.raises(WhlError):
            parse_line(_with_checksum(body))

    @pytest.mark.parametrize("state", ["3", "-1"])
    def test_state_out_of_set_rejected(self, state):
        body = f"WHL2,1,1000000,0.500000,10,12,{state}"
        with pytest.raises(WhlError):
            parse_line(_with_checksum(body))

    @pytest.mark.parametrize(
        "tok",
        ["inf", "-inf", "1e400"],
        ids=["inf", "neg_inf", "exponent_overflow"],
    )
    def test_infinite_model_token_rejected(self, tok):
        # "1e400" parses to inf in float(), so the finite check refuses it.
        with pytest.raises(WhlError):
            parse_line(_with_checksum(f"WHL2,1,1000000,{tok},10,12,0"))

    def test_model_token_over_20_chars_rejected(self):
        # A finite model whose token is 21 characters is refused by the length cap.
        tok = "-1000000000000.000000"
        assert len(tok) == 21
        with pytest.raises(WhlError):
            parse_line(_with_checksum(f"WHL2,1,1000000,{tok},10,12,0"))

    @pytest.mark.parametrize(
        "body",
        [
            "WHL2,1,1000000,,10,12,0",
            "WHL2,,1000000,0.500000,10,12,0",
        ],
        ids=["empty_model_token", "empty_seq_token"],
    )
    def test_empty_token_rejected(self, body):
        with pytest.raises(WhlError):
            parse_line(_with_checksum(body))

    @pytest.mark.parametrize("pos", ["seq", "t_us", "model", "raw_l", "raw_r"])
    def test_five_thousand_digit_token_raises_whl_error(self, pos):
        # A checksum-valid line with a 5000-digit token must raise WhlError,
        # not ValueError from int() or float(), and the message must not carry the digits.
        fields = {
            "seq": "1",
            "t_us": "1000000",
            "model": "0.500000",
            "raw_l": "10",
            "raw_r": "12",
        }
        fields[pos] = "9" * 5000
        body = (
            f"WHL2,{fields['seq']},{fields['t_us']},{fields['model']},"
            f"{fields['raw_l']},{fields['raw_r']},0"
        )
        with pytest.raises(WhlError) as excinfo:
            parse_line(_with_checksum(body))
        assert len(str(excinfo.value)) < 200

    @pytest.mark.parametrize(
        "body",
        [
            "WHL2,+1,1000000,0.500000,10,12,0",
            "WHL2,1,+1000000,0.500000,10,12,0",
            "WHL2,1,1000000,0.500000,10,12,+0",
        ],
        ids=["seq_plus_refused", "t_us_plus_refused", "state_plus_refused"],
    )
    def test_plus_sign_refused_on_seq_t_us_state(self, body):
        # Pinned current behaviour, inherited from the single-wheel parser:
        # a leading "+" is refused on seq, t_us and state. CONTRACT.md S8 leaves
        # the "+" question open; these tests document what the code does now.
        with pytest.raises(WhlError):
            parse_line(_with_checksum(body))

    def test_plus_sign_accepted_on_raw_and_model(self):
        # Pinned current behaviour, inherited from the single-wheel parser:
        # a leading "+" is accepted on raw_l, raw_r (signed integer pattern)
        # and on the model token (float()). CONTRACT.md S8 leaves this open.
        w = parse_line(_with_checksum("WHL2,1,1000000,+0.5,+5,+12,0"))
        assert w == Whl2(1, 1000000, 0.5, 5, 12, 0)

    @pytest.mark.parametrize(
        "body",
        [
            "WHL2,1,1000000,0.5 00,10,12,0",
            "WHL2,1,1000000,0_5,10,12,0",
            "WHL2,1_0,1000000,0.500000,10,12,0",
            "WHL2,1,1_000000,0.500000,10,12,0",
            "WHL2,1,1000000,0.500000,1_0,12,0",
            "WHL2,1,1000000,0.500000,10,1 2,0",
            "WHL2,1,1000000,0.500000,10,12, 0",
            "WHL2, 1,1000000,0.500000,10,12,0",
        ],
        ids=[
            "space_in_model",
            "underscore_in_model",
            "underscore_in_seq",
            "underscore_in_t_us",
            "underscore_in_raw_l",
            "space_in_raw_r",
            "space_in_state",
            "space_in_seq",
        ],
    )
    def test_whitespace_and_underscore_tokens_rejected(self, body):
        with pytest.raises(WhlError):
            parse_line(_with_checksum(body))


class TestSplit:
    """split(Whl2) returns (left Whl, right Whl) with shared fields."""

    @pytest.mark.parametrize(
        "w",
        [w for w, _ in GOLDEN_ACCEPT],
        ids=GOLDEN_ACCEPT_IDS,
    )
    def test_golden_split(self, w):
        left, right = split(w)
        assert left == Whl(seq=w.seq, t_us=w.t_us, model_rad_s=w.model_rad_s, raw=w.raw_l, state=w.state)
        assert right == Whl(seq=w.seq, t_us=w.t_us, model_rad_s=w.model_rad_s, raw=w.raw_r, state=w.state)

    def test_returns_pair_of_whl(self):
        result = split(Whl2(1, 0, 0.25, 7, -3, 0))
        assert isinstance(result, tuple)
        assert len(result) == 2
        assert all(isinstance(part, Whl) for part in result)

    def test_shared_fields_match(self):
        left, right = split(Whl2(9, 123456, 0.75, 4, -4, 1))
        assert left.seq == right.seq == 9
        assert left.t_us == right.t_us == 123456
        assert left.model_rad_s == right.model_rad_s == 0.75
        assert left.state == right.state == 1

    @pytest.mark.parametrize(
        "raw_l,raw_r",
        [(7, -3), (-125, 125), (200, -200), (INT32_MIN_LITERAL, INT32_MAX_LITERAL)],
        ids=["mixed_sign", "limits_125", "no_clamp", "int32_extremes"],
    )
    def test_no_clamp_no_abs(self, raw_l, raw_r):
        left, right = split(Whl2(1, 0, 0.0, raw_l, raw_r, 0))
        assert left.raw == raw_l
        assert right.raw == raw_r

    def test_negative_raw_r_stays_negative(self):
        _, right = split(Whl2(1, 0, 0.5, 10, -10, 0))
        assert right.raw == -10

    def test_returned_parts_are_frozen(self):
        left, right = split(Whl2(1, 0, 0.5, 10, 12, 0))
        with pytest.raises(dataclasses.FrozenInstanceError):
            left.seq = 2  # type: ignore[misc]
        with pytest.raises(dataclasses.FrozenInstanceError):
            right.raw = 2  # type: ignore[misc]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
