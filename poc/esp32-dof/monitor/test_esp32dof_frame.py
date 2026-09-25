#!/usr/bin/env python3
"""
Unit tests for ESP32 6-DOF IMU frame protocol parser and formatter.

Pure Python / pytest, no external dependencies beyond pytest.

Usage:
    cd poc/esp32-dof
    mamba run -n esp32-dof python3 -m pytest monitor/test_esp32dof_frame.py -v
"""

import sys
from pathlib import Path

import pytest

# Add the script directory to path for imports
sys.path.insert(0, str(Path(__file__).resolve().parent))

from dof_frame import Frame, FrameError, parse_line, format_frame


def _with_checksum(body: str) -> str:
    """
    Helper to append valid checksum to a frame body.

    Args:
        body: Frame body from IMU to * (exclusive), e.g., "IMU,0,0,0,0,0,0,0,0"

    Returns:
        str: Complete line with appended *HH\\n
    """
    csum = 0
    for c in body:
        csum ^= ord(c)
    return f"{body}*{csum:02X}\n"


class TestRoundTrip:
    """Test: parse and format with exact decimal preservation."""

    def test_round_trip_basic_values(self):
        """format_frame() and parse_line() preserve 6-decimal-precision values."""
        f_orig = Frame(
            seq=1000,
            t_us=1234567,
            accel=(0.5, -1.25, 9.806650),
            gyro=(0.1, -0.2, 0.3),
        )
        line = format_frame(f_orig)
        f_parsed = parse_line(line)

        assert f_parsed.seq == f_orig.seq
        assert f_parsed.t_us == f_orig.t_us
        assert f_parsed.accel == f_orig.accel
        assert f_parsed.gyro == f_orig.gyro

    def test_output_ends_with_newline(self):
        """format_frame() output ends with \\n."""
        f = Frame(seq=0, t_us=0, accel=(0, 0, 0), gyro=(0, 0, 0))
        line = format_frame(f)
        assert line.endswith("\n")

    def test_output_contains_star(self):
        """format_frame() output contains * before checksum."""
        f = Frame(seq=0, t_us=0, accel=(0, 0, 0), gyro=(0, 0, 0))
        line = format_frame(f)
        assert "*" in line
        star_pos = line.rfind("*")
        assert star_pos > 0
        assert line[star_pos + 1:star_pos + 3].upper() == line[star_pos + 1:star_pos + 3]

    def test_max_seq_and_t_us(self):
        """Round-trip with seq=65535 (max) and t_us=2^32-1 (max)."""
        f_orig = Frame(
            seq=65535,
            t_us=4294967295,
            accel=(1.0, 2.0, 3.0),
            gyro=(4.0, 5.0, 6.0),
        )
        line = format_frame(f_orig)
        f_parsed = parse_line(line)

        assert f_parsed.seq == 65535
        assert f_parsed.t_us == 4294967295

    def test_hex_uppercase(self):
        """format_frame() produces uppercase hex checksum."""
        f = Frame(seq=0, t_us=0, accel=(0, 0, 0), gyro=(0, 0, 0))
        line = format_frame(f)
        star_pos = line.rfind("*")
        hex_part = line[star_pos + 1:star_pos + 3]
        assert hex_part == hex_part.upper()
        assert all(c in "0123456789ABCDEF" for c in hex_part)


class TestBadChecksum:
    """Test: checksum validation rejection."""

    def test_checksum_mismatch(self):
        """parse_line() rejects mismatched checksum."""
        body = "IMU,0,0,0,0,0,0,0,0"
        line = body + "*FF\n"
        with pytest.raises(FrameError, match="checksum mismatch"):
            parse_line(line)

    def test_lowercase_hex_rejected(self):
        """parse_line() rejects lowercase hex checksum."""
        body = "IMU,0,0,0,0,0,0,0,0"
        line = body + "*ab\n"
        with pytest.raises(FrameError, match="uppercase hex"):
            parse_line(line)

    def test_missing_star(self):
        """parse_line() rejects line without * marker."""
        line = "IMU,0,0,0,0,0,0,0,0\n"
        with pytest.raises(FrameError, match="missing checksum marker"):
            parse_line(line)

    def test_one_digit_checksum(self):
        """parse_line() rejects checksum with 1 digit."""
        body = "IMU,0,0,0,0,0,0,0,0"
        # Compute real checksum, then truncate
        csum = 0
        for c in body:
            csum ^= ord(c)
        line = body + f"*{csum & 0xF:X}\n"
        with pytest.raises(FrameError, match="exactly 2 characters"):
            parse_line(line)

    def test_three_digit_checksum(self):
        """parse_line() rejects checksum with 3 digits."""
        body = "IMU,0,0,0,0,0,0,0,0"
        line = body + "*FFF\n"
        with pytest.raises(FrameError, match="exactly 2 characters"):
            parse_line(line)

    def test_non_hex_checksum(self):
        """parse_line() rejects checksum with non-hex characters."""
        line = "IMU,0,0,0,0,0,0,0,0*GG\n"
        with pytest.raises(FrameError, match="uppercase hex"):
            parse_line(line)


class TestBadPrefix:
    """Test: prefix validation."""

    def test_bad_prefix_with_valid_checksum(self):
        """parse_line() rejects non-IMU prefix even with correct checksum."""
        body = "XYZ,0,0,0,0,0,0,0,0"
        line = _with_checksum(body)
        with pytest.raises(FrameError, match="bad prefix"):
            parse_line(line)

    def test_empty_prefix(self):
        """parse_line() rejects missing prefix."""
        body = ",0,0,0,0,0,0,0,0"
        line = _with_checksum(body)
        with pytest.raises(FrameError, match="bad prefix"):
            parse_line(line)


class TestFieldCount:
    """Test: field count validation."""

    def test_too_few_fields(self):
        """parse_line() rejects too few fields."""
        body = "IMU,0,0,0,0,0"
        line = _with_checksum(body)
        with pytest.raises(FrameError, match="expected 9 fields"):
            parse_line(line)

    def test_too_many_fields(self):
        """parse_line() rejects too many fields."""
        body = "IMU,0,0,0,0,0,0,0,0,0,0"
        line = _with_checksum(body)
        with pytest.raises(FrameError, match="expected 9 fields"):
            parse_line(line)


class TestNonNumeric:
    """Test: numeric field validation."""

    def test_seq_non_numeric(self):
        """parse_line() rejects non-numeric seq."""
        body = "IMU,abc,0,0,0,0,0,0,0"
        line = _with_checksum(body)
        with pytest.raises(FrameError, match="decimal digits"):
            parse_line(line)

    def test_t_us_non_numeric(self):
        """parse_line() rejects non-numeric t_us."""
        body = "IMU,0,xyz,0,0,0,0,0,0"
        line = _with_checksum(body)
        with pytest.raises(FrameError, match="decimal digits"):
            parse_line(line)

    def test_accel_x_non_numeric(self):
        """parse_line() rejects non-numeric ax."""
        body = "IMU,0,0,abc,0,0,0,0,0"
        line = _with_checksum(body)
        with pytest.raises(FrameError, match="not a valid float"):
            parse_line(line)

    def test_nan_rejected(self):
        """parse_line() rejects NaN float."""
        body = "IMU,0,0,nan,0,0,0,0,0"
        line = _with_checksum(body)
        with pytest.raises(FrameError, match="not finite"):
            parse_line(line)

    def test_inf_rejected(self):
        """parse_line() rejects inf float."""
        body = "IMU,0,0,inf,0,0,0,0,0"
        line = _with_checksum(body)
        with pytest.raises(FrameError, match="not finite"):
            parse_line(line)

    def test_seq_negative(self):
        """parse_line() rejects negative seq."""
        body = "IMU,-1,0,0,0,0,0,0,0"
        line = _with_checksum(body)
        with pytest.raises(FrameError, match="decimal digits"):
            parse_line(line)

    def test_seq_overflow(self):
        """parse_line() rejects seq >= 65536."""
        body = "IMU,65536,0,0,0,0,0,0,0"
        line = _with_checksum(body)
        with pytest.raises(FrameError, match="out of range"):
            parse_line(line)

    def test_t_us_overflow(self):
        """parse_line() rejects t_us > 2^32-1."""
        body = "IMU,0,4294967296,0,0,0,0,0,0"
        line = _with_checksum(body)
        with pytest.raises(FrameError, match="out of range"):
            parse_line(line)

    def test_embedded_space_rejected(self):
        """parse_line() rejects float with embedded space."""
        body = "IMU,0,0,1 .0,0,0,0,0,0"
        line = _with_checksum(body)
        with pytest.raises(FrameError, match="embedded whitespace"):
            parse_line(line)

    def test_underscore_rejected(self):
        """parse_line() rejects float with underscore."""
        body = "IMU,0,0,1_000,0,0,0,0,0"
        line = _with_checksum(body)
        with pytest.raises(FrameError, match="underscore"):
            parse_line(line)


class TestCRLF:
    """Test: line terminator handling."""

    def test_parses_crlf_equal_to_lf(self):
        """parse_line() accepts \\r\\n and parses identically to \\n."""
        body = "IMU,123,456,0.1,0.2,0.3,0.4,0.5,0.6"
        line_lf = _with_checksum(body).rstrip("\n")
        line_crlf = line_lf + "\r\n"

        f_lf = parse_line(line_lf + "\n")
        f_crlf = parse_line(line_crlf)

        assert f_lf == f_crlf

    def test_bare_lf_parses(self):
        """parse_line() accepts bare \\n."""
        line = _with_checksum("IMU,0,0,0,0,0,0,0,0")
        f = parse_line(line)
        assert f.seq == 0

    def test_no_terminator_rejected(self):
        """parse_line() rejects missing terminator (if treated as missing *)."""
        # Note: A line without \n or \r\n but with * still has a *, so it parses.
        # Only if there's no * do we reject it.
        body = "IMU,0,0,0,0,0,0,0,0"
        line = body + "*51"  # no terminator
        # This should parse fine because * is present
        f = parse_line(line)
        assert f.seq == 0


class TestNegatives:
    """Test: negative float values."""

    def test_negative_accel_round_trip(self):
        """format_frame() and parse_line() preserve negative acceleration."""
        f_orig = Frame(seq=0, t_us=0, accel=(-1.5, -2.75, -0.1), gyro=(0, 0, 0))
        line = format_frame(f_orig)
        f_parsed = parse_line(line)

        assert f_parsed.accel == f_orig.accel

    def test_negative_gyro_round_trip(self):
        """format_frame() and parse_line() preserve negative gyroscope."""
        f_orig = Frame(seq=0, t_us=0, accel=(0, 0, 0), gyro=(-5.5, -10.2, -0.01))
        line = format_frame(f_orig)
        f_parsed = parse_line(line)

        assert f_parsed.gyro == f_orig.gyro

    def test_all_negative_round_trip(self):
        """format_frame() and parse_line() preserve all negative values."""
        f_orig = Frame(
            seq=0, t_us=0, accel=(-1.1, -2.2, -3.3), gyro=(-4.4, -5.5, -6.6)
        )
        line = format_frame(f_orig)
        f_parsed = parse_line(line)

        assert f_parsed.accel == f_orig.accel
        assert f_parsed.gyro == f_orig.gyro


class TestGolden:
    """Test: firmware parity vector (hard-coded golden line)."""

    def test_golden_line_parses(self):
        """Hard-coded golden line parses to expected Frame."""
        golden_line = "IMU,12345,1234567890,0.500000,-9.806650,1.250000,10.500000,-5.500000,0.000000*52\n"
        f = parse_line(golden_line)

        assert f.seq == 12345
        assert f.t_us == 1234567890
        assert f.accel == (0.5, -9.806650, 1.25)
        assert f.gyro == (10.5, -5.5, 0.0)

    def test_golden_line_format(self):
        """Frame formats to expected golden line."""
        f = Frame(
            seq=12345,
            t_us=1234567890,
            accel=(0.5, -9.806650, 1.25),
            gyro=(10.5, -5.5, 0.0),
        )
        line = format_frame(f)

        expected = "IMU,12345,1234567890,0.500000,-9.806650,1.250000,10.500000,-5.500000,0.000000*52\n"
        assert line == expected


class TestLenientFloats:
    """Test: parser accepts exponent notation and leading + signs."""

    def test_exponent_and_plus_notation(self):
        """parse_line() accepts 1e-3, +2.5, and 2E+2 float tokens."""
        body = "IMU,0,0,1e-3,+2.5,2E+2,1e-3,+2.5,2E+2"
        line = _with_checksum(body)
        f = parse_line(line)

        # Verify the floats parsed to expected values
        assert f.seq == 0
        assert f.t_us == 0
        assert f.accel == (0.001, 2.5, 200.0)
        assert f.gyro == (0.001, 2.5, 200.0)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
