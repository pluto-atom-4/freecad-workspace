#!/usr/bin/env python3
"""
Tests for the WHL2 twin step generator step_lines and the main CLI in
ctrl_wheels_msg.

SIM ONLY. These tests pin the WHL2 step generator (line format, counts, seq and
t_us counters, raw mapping and clamp, pacing), its sign and state defaults, its
refusals (WhlError for bad arguments) and the main() option handling.

The chip raw signs are UNCONFIRMED. The sign_l=+1 and sign_r=-1 defaults pinned
here are not verified against hardware (CONTRACT.md S2). The Webots +1/+1 direction
was owner-observed on 2026-11-06 (sim only); this file does not test it.

Expected values are written by hand from the contract and from _raw_for
(rad/s * 60 / (2 pi) / 0.229, clamped to +-125, then rounded). They are not
computed with the implementation.

Pure Python, needs pytest. The module under test is a flat module in
poc/ctrl-wheels and is imported by bare name.

Usage:
    cd poc/ctrl-wheels
    python3 -m pytest test_ctrl_wheels_msg_steps.py -v
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from ctrl_wheels_msg import Whl2, WhlError, main, parse_line, step_lines


SIGNS = {"sign_l": 1, "sign_r": -1}


def _run(value, seconds, **kwargs):
    """Collect step_lines output with the pinned sign defaults, overridable per call."""
    args = {**SIGNS, **kwargs}
    return list(step_lines(value, seconds, **args))


class TestStepLinesTwinFormat:
    """Test: every yielded line is a WHL2 line with the twin fields."""

    def test_count_five_seconds_at_50hz(self):
        lines = _run(1.0, 5.0)
        assert len(lines) == 250

    def test_count_rate_option(self):
        lines = _run(1.0, 0.1, rate_hz=100.0)
        assert len(lines) == 10

    def test_every_line_is_whl2_with_newline(self):
        lines = _run(1.0, 0.1)
        for line in lines:
            assert line.startswith("WHL2,")
            assert line.endswith("\n")
            assert isinstance(parse_line(line), Whl2)


class TestStepLinesTwinContents:
    """Test: field values of the yielded WHL2 lines."""

    def test_lines_parse_back_for_value_one(self):
        lines = _run(1.0, 5.0)
        for i, line in enumerate(lines):
            w = parse_line(line)
            assert w.seq == i
            assert w.t_us == i * 20000
            assert w.model_rad_s == 1.0
            assert w.raw_l == 42
            assert w.raw_r == -42
            assert w.state == 0

    @pytest.mark.parametrize(
        "value,raw_l,raw_r",
        [
            (0.5, 21, -21),
            (-0.5, -21, 21),
            (-1.0, -42, 42),
            (3.0, 125, -125),
            (-3.0, -125, 125),
            (100.0, 125, -125),
            (-100.0, -125, 125),
        ],
    )
    def test_raw_mapping_clamp_and_signs(self, value, raw_l, raw_r):
        lines = _run(value, 0.02)
        assert len(lines) == 1
        w = parse_line(lines[0])
        assert w.raw_l == raw_l
        assert w.raw_r == raw_r

    def test_model_is_not_clamped(self):
        lines = _run(100.0, 0.02)
        assert parse_line(lines[0]).model_rad_s == 100.0

    @pytest.mark.parametrize("value", [1e11, -1e11])
    def test_large_value_in_model_range_clamps_raw(self, value):
        expected_l = 125 if value > 0 else -125
        lines = _run(value, 0.02)
        w = parse_line(lines[0])
        assert w.raw_l == expected_l
        assert w.raw_r == -expected_l

    @pytest.mark.parametrize("value", [1e308, -1e308, 1e13, -1e12])
    def test_value_outside_model_range_is_refused(self, value):
        with pytest.raises(WhlError):
            _run(value, 0.02)

    @pytest.mark.parametrize(
        "sign_l,sign_r,raw_l,raw_r",
        [
            (1, 1, 42, 42),
            (-1, -1, -42, -42),
            (-1, 1, -42, 42),
        ],
    )
    def test_sign_parameters_apply_to_raw(self, sign_l, sign_r, raw_l, raw_r):
        lines = _run(1.0, 0.02, sign_l=sign_l, sign_r=sign_r)
        w = parse_line(lines[0])
        assert w.raw_l == raw_l
        assert w.raw_r == raw_r

    def test_default_signs_and_state_without_kwargs(self):
        lines = list(step_lines(1.0, 0.02))
        assert len(lines) == 1
        w = parse_line(lines[0])
        assert w.raw_l == 42
        assert w.raw_r == -42
        assert w.state == 0

    @pytest.mark.parametrize("state", [0, 1, 2])
    def test_state_on_every_line(self, state):
        lines = _run(1.0, 0.1, state=state)
        assert len(lines) == 5
        for line in lines:
            assert parse_line(line).state == state


class TestStepLinesTwinSeqAndTime:
    """Test: seq and t_us counters of step_lines."""

    def test_seq_wraps_from_65535(self):
        lines = _run(1.0, 0.1, rate_hz=50.0, start_seq=65535)
        seqs = [parse_line(line).seq for line in lines]
        assert seqs == [65535, 0, 1, 2, 3]

    def test_seq_start_offset(self):
        lines = _run(0.0, 0.1, start_seq=100)
        assert [parse_line(line).seq for line in lines] == [100, 101, 102, 103, 104]

    def test_t_us_step_at_rate(self):
        lines = _run(0.0, 0.1, rate_hz=100.0)
        t = [parse_line(line).t_us for line in lines]
        assert t == [i * 10000 for i in range(10)]

    def test_t_us_wraps_mod_2_32(self):
        lines = _run(0.0, 4296.0, rate_hz=1.0)
        assert len(lines) == 4296
        t = [parse_line(lines[i]).t_us for i in (4293, 4294, 4295)]
        assert t == [4293000000, 4294000000, 32704]


class TestStepLinesTwinPacing:
    """Test: realtime pacing with injected clock and sleep."""

    def test_realtime_false_never_sleeps(self):
        sleeps = []
        lines = _run(
            1.0,
            0.1,
            realtime=False,
            clock=lambda: 0.0,
            sleep=lambda s: sleeps.append(s),
        )
        assert len(lines) == 5
        assert sleeps == []

    def test_realtime_true_paces_with_injected_clock(self):
        sleeps = []
        lines = _run(
            1.0,
            0.1,
            rate_hz=50.0,
            realtime=True,
            clock=lambda: 0.0,
            sleep=sleeps.append,
        )
        assert len(lines) == 5
        assert sleeps == pytest.approx([0.02, 0.04, 0.06, 0.08])


class TestStepLinesTwinRefusals:
    """Test: invalid arguments are refused with WhlError before any line is yielded."""

    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
    def test_non_finite_value_refused(self, bad):
        with pytest.raises(WhlError):
            _run(bad, 0.1)

    @pytest.mark.parametrize("bad", [float("nan"), float("inf")])
    def test_non_finite_seconds_refused(self, bad):
        with pytest.raises(WhlError):
            _run(1.0, bad)

    def test_negative_seconds_refused(self):
        with pytest.raises(WhlError):
            _run(1.0, -0.1)

    @pytest.mark.parametrize("bad", [0.0, -50.0, float("nan")])
    def test_rate_not_positive_finite_refused(self, bad):
        with pytest.raises(WhlError):
            _run(1.0, 0.1, rate_hz=bad)

    @pytest.mark.parametrize("rate", [1e7, 1e300])
    def test_period_below_one_us_refused(self, rate):
        with pytest.raises(WhlError):
            _run(0.0, 0.1, rate_hz=rate)

    @pytest.mark.parametrize("start_seq", [True, 1.0, "0"])
    def test_start_seq_non_int_refused(self, start_seq):
        with pytest.raises(WhlError):
            _run(0.0, 0.1, start_seq=start_seq)

    @pytest.mark.parametrize("state", [3, -1])
    def test_state_out_of_range_refused(self, state):
        with pytest.raises(WhlError):
            _run(1.0, 0.1, state=state)

    @pytest.mark.parametrize("state", [True, 1.0, "0"])
    def test_state_non_int_refused(self, state):
        with pytest.raises(WhlError):
            _run(1.0, 0.1, state=state)

    @pytest.mark.parametrize("bad", [0, 2, -2, 1.0, -1.0, True, None, "1"])
    def test_sign_l_must_be_plus_or_minus_one_int(self, bad):
        with pytest.raises(WhlError):
            _run(1.0, 0.1, sign_l=bad)

    @pytest.mark.parametrize("bad", [0, 2, -2, 1.0, -1.0, True, None, "1"])
    def test_sign_r_must_be_plus_or_minus_one_int(self, bad):
        with pytest.raises(WhlError):
            _run(1.0, 0.1, sign_r=bad)


class TestMainCli:
    """Test: main(argv) option handling, with pacing off (--no-sleep)."""

    def test_default_run_prints_whl2_line(self, capsys):
        rc = main(["--step", "1.0", "--seconds", "0.02", "--no-sleep"])
        assert rc == 0
        out = capsys.readouterr().out
        lines = out.splitlines(keepends=True)
        assert len(lines) == 1
        w = parse_line(lines[0])
        assert w.raw_l == 42
        assert w.raw_r == -42
        assert w.state == 0

    def test_sign_options_swap_raws(self, capsys):
        rc = main(
            ["--step", "1.0", "--seconds", "0.02", "--no-sleep",
             "--sign-left", "-1", "--sign-right", "1"]
        )
        assert rc == 0
        w = parse_line(capsys.readouterr().out)
        assert w.raw_l == -42
        assert w.raw_r == 42

    def test_state_out_of_choices_exits_2(self):
        with pytest.raises(SystemExit) as exc:
            main(["--step", "1.0", "--seconds", "0.02", "--no-sleep", "--state", "3"])
        assert exc.value.code == 2

    def test_non_finite_step_returns_2_with_stderr(self, capsys):
        rc = main(["--step", "nan", "--seconds", "0.02", "--no-sleep"])
        assert rc == 2
        cap = capsys.readouterr()
        assert cap.out == ""
        assert cap.err.startswith("ctrl_wheels_msg:")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
