#!/usr/bin/env python3
"""
Unit tests for monitor CLI (dof_monitor).

Usage:
    cd poc/esp32-dof
    mamba run -n esp32-dof python3 -m pytest -q monitor/test_esp32dof_monitor.py
"""

import io
import re
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import dof_monitor
from dof_monitor import run, main, _format_row, _header, _limit_duration
from dof_frame import Frame, format_frame
from dof_fusion import ComplementaryFilter
from dof_serial import DofSerialError
from dof_sources import mock_lines as real_mock_lines


def fast(n_s=1.0, drop_every=0):
    """Fast mock source (non-realtime)."""
    return list(real_mock_lines(duration_s=n_s, realtime=False, drop_every=drop_every))


def rows(text):
    """Extract data rows (lines starting with a digit after split)."""
    return [l for l in text.splitlines() if l.split() and l.split()[0].isdigit()]


class TestRun:
    """Tests for run() function."""

    def test_run_mock_no_drops(self):
        """Test: run with mock source, no drops."""
        out = io.StringIO()
        stats = run(iter(fast()), out=out, quiet_every=1)
        assert stats.received == 50
        assert stats.dropped == 0
        assert stats.bad_frames == 0
        assert len(rows(out.getvalue())) == 50

    def test_run_mock_with_drops_exact(self):
        """Test: run with mock source, drops every 10 frames."""
        out = io.StringIO()
        stats = run(iter(fast(1.0, 10)), out=out)
        assert stats.received == 45
        assert stats.dropped == 4
        assert stats.out_of_order == 0
        assert stats.bad_frames == 0
        assert stats.avg_rate_hz == pytest.approx(45.8333, rel=1e-3)
        assert stats.rate_hz == pytest.approx(45.8333, rel=1e-3)

    def test_run_summary_text(self):
        """Test: summary contains expected fields."""
        out = io.StringIO()
        stats = run(iter(fast(1.0, 10)), out=out, quiet_every=1)
        text = out.getvalue()
        assert re.search(r"received\s*:\s*45\b", text)
        assert re.search(r"dropped\s*:\s*4\b", text)
        assert re.search(r"bad_frames\s*:\s*0\b", text)
        assert "avg rate" in text
        assert "--- summary ---" in text
        assert len(rows(text)) == 45

    def test_run_garbage_line_counts_bad(self):
        """Test: garbage lines are counted as bad."""
        lines = fast()
        lines.insert(10, "garbage")
        lines.insert(20, "IMU,1,2*ZZ")
        out = io.StringIO()
        stats = run(iter(lines), out=out)
        assert stats.bad_frames == 2
        assert stats.received == 50

    def test_run_quiet_every(self):
        """Test: quiet_every controls row frequency."""
        lines = fast()

        out1 = io.StringIO()
        run(iter(lines), out=out1, quiet_every=5)
        assert len(rows(out1.getvalue())) == 10

        out2 = io.StringIO()
        run(iter(lines), out=out2, quiet_every=1)
        assert len(rows(out2.getvalue())) == 50

        out3 = io.StringIO()
        run(iter(lines), out=out3, quiet_every=7)
        assert len(rows(out3.getvalue())) == 8

    def test_run_prints_header_first(self):
        """Test: header is printed first."""
        out = io.StringIO()
        stats = run(iter([]), out=out)
        lines = out.getvalue().splitlines()
        assert lines[0] == _header()
        assert stats.received == 0
        assert "--- summary ---" in out.getvalue()

    def test_run_keyboard_interrupt(self):
        """Test: KeyboardInterrupt triggers summary and returns."""
        def gen():
            yield from fast()[:3]
            raise KeyboardInterrupt

        out = io.StringIO()
        stats = run(gen(), out=out)
        text = out.getvalue()
        assert stats.received == 3
        assert "(interrupted)" in text
        # No exception escapes

    def test_run_source_error_propagates(self):
        """Test: source error propagates."""
        def gen():
            raise DofSerialError("boom")
            yield  # unreachable

        with pytest.raises(DofSerialError):
            run(gen(), out=io.StringIO())

    def test_run_default_out_is_stdout_at_call_time(self, capsys):
        """Test: default out is sys.stdout at call time."""
        run(iter(fast()[:5]), quiet_every=1)
        out, err = capsys.readouterr()
        assert "--- summary ---" in out

    def test_run_rejects_bad_quiet_every(self):
        """Test: quiet_every validation."""
        with pytest.raises(ValueError):
            run(iter([]), quiet_every=0)
        with pytest.raises(ValueError):
            run(iter([]), quiet_every=-1)
        with pytest.raises(ValueError):
            run(iter([]), quiet_every=True)
        with pytest.raises(ValueError):
            run(iter([]), quiet_every="5")


class TestFormatAndHeader:
    """Tests for _format_row and _header."""

    def test_format_row_tokens(self):
        """Test: format_row output structure."""
        frame = Frame(12, 240000, (0.5, -1.0, 1.25), (10.5, -5.5, 0.0))
        row = _format_row(frame, 50.0, 0)
        tokens = row.split()
        assert tokens == ["12", "240.0", "0.500", "-1.000", "1.250", "10.50", "-5.50", "0.00", "50.0", "0"]
        assert len(_header().split()) == 10

    def test_fused_header_and_row(self):
        """Test: fused header has 13 columns and row formatting."""
        frame = Frame(12, 240000, (0.5, -1.0, 1.25), (10.5, -5.5, 0.0))
        assert len(_header(True).split()) == 13
        assert _header(False) == _header()
        toks = _format_row(frame, 50.0, 0, (10.0, -5.0, 0.0)).split()
        assert len(toks) == 13
        assert toks[-3:] == ["10.00", "-5.00", "0.00"]
        assert _format_row(frame, 50.0, 0, None).split() == _format_row(frame, 50.0, 0).split()


class TestLimitDuration:
    """Tests for _limit_duration."""

    def test_limit_duration(self, monkeypatch):
        """Test: _limit_duration stops after deadline."""
        clock = iter([0.0, 0.5, 1.5, 2.5])
        monkeypatch.setattr(dof_monitor, "monotonic", lambda: next(clock))
        result = list(_limit_duration(iter(["a", "b", "c"]), 1.0))
        assert result == ["a"]


class TestMain:
    """Tests for main() function."""

    def test_main_mock(self, capsys, monkeypatch):
        """Test: main with --mock."""
        calls = []
        def fake(**kw):
            calls.append(kw)
            return iter(fast())
        monkeypatch.setattr(dof_monitor, "mock_lines", fake)
        rc = main(["--mock", "--duration", "1", "--quiet-every", "1"])
        out, err = capsys.readouterr()
        assert rc == 0
        assert calls == [{"duration_s": 1.0, "realtime": True}]
        assert len(rows(out)) == 50
        assert "received" in out
        assert err == ""

    def test_main_port_passes_duration_and_baud(self, capsys, monkeypatch):
        """Test: main with --port passes args correctly."""
        calls = []
        def fake(*a, **k):
            calls.append((a, k))
            return iter(fast())
        monkeypatch.setattr(dof_monitor, "serial_lines", fake)
        rc = main(["--port", "/dev/ttyFAKE", "--baud", "9600", "--duration", "2"])
        out, err = capsys.readouterr()
        assert rc == 0
        assert calls == [(("/dev/ttyFAKE", 9600), {"max_duration_s": 2.0})]

    def test_main_port_default_baud(self, capsys, monkeypatch):
        """Test: main with --port uses default baud."""
        calls = []
        def fake(*a, **k):
            calls.append((a, k))
            return iter(fast())
        monkeypatch.setattr(dof_monitor, "serial_lines", fake)
        main(["--port", "x"])
        assert calls[0][0] == ("x", 115200)
        assert calls[0][1] == {"max_duration_s": None}

    def test_main_port_open_failure_exits_2(self, capsys, monkeypatch):
        """Test: serial port open failure exits with code 2."""
        def fake(*a, **k):
            def gen():
                raise DofSerialError("cannot open serial port 'x': nope")
                yield  # unreachable
            return gen()
        monkeypatch.setattr(dof_monitor, "serial_lines", fake)
        rc = main(["--port", "x"])
        out, err = capsys.readouterr()
        assert rc == 2
        assert "cannot open serial port" in err
        assert "dof_monitor: error:" in err
        assert "Traceback" not in (out + err)

    def test_main_replay_passes_path_and_realtime(self, capsys, monkeypatch):
        """Test: main with --replay passes args correctly."""
        calls = []
        def fake(path, realtime):
            calls.append((path, realtime))
            return iter(fast())
        monkeypatch.setattr(dof_monitor, "csv_replay_lines", fake)
        rc = main(["--replay", "f.csv"])
        out, err = capsys.readouterr()
        assert rc == 0
        assert calls == [("f.csv", True)]

    def test_main_replay_duration_wrapper(self, capsys, monkeypatch):
        """Test: main with --replay and --duration wraps with _limit_duration."""
        def fake(path, realtime):
            return iter(fast())
        monkeypatch.setattr(dof_monitor, "csv_replay_lines", fake)

        clock = iter([0.0, 0.1, 0.2, 5.0, 6.0])
        monkeypatch.setattr(dof_monitor, "monotonic", lambda: next(clock))

        rc = main(["--replay", "f.csv", "--duration", "1", "--quiet-every", "1"])
        out, err = capsys.readouterr()
        assert rc == 0
        assert len(rows(out)) == 2

    def test_main_replay_bad_row_exits_2(self, capsys, monkeypatch):
        """Test: CSV parsing error exits with code 2."""
        def fake(path, realtime):
            def gen():
                raise ValueError("f.csv:3: bad row")
                yield  # unreachable
            return gen()
        monkeypatch.setattr(dof_monitor, "csv_replay_lines", fake)
        rc = main(["--replay", "f.csv"])
        out, err = capsys.readouterr()
        assert rc == 2
        assert "bad row" in err

    def test_main_replay_missing_file_exits_2(self, capsys, monkeypatch):
        """Test: FileNotFoundError exits with code 2."""
        def fake(path, realtime):
            def gen():
                raise FileNotFoundError("nope.csv")
                yield  # unreachable
            return gen()
        monkeypatch.setattr(dof_monitor, "csv_replay_lines", fake)
        rc = main(["--replay", "nope.csv"])
        out, err = capsys.readouterr()
        assert rc == 2

    def test_main_keyboard_interrupt_exit_0(self, capsys, monkeypatch):
        """Test: KeyboardInterrupt exits with code 0."""
        def fake(duration_s=None, realtime=True):
            def gen():
                yield from fast()[:3]
                raise KeyboardInterrupt
            return gen()
        monkeypatch.setattr(dof_monitor, "mock_lines", fake)
        rc = main(["--mock"])
        out, err = capsys.readouterr()
        assert rc == 0
        assert "(interrupted)" in out

    def test_main_list_ports(self, capsys, monkeypatch):
        """Test: --list-ports."""
        monkeypatch.setattr(dof_monitor, "list_ports",
                           lambda: ["/dev/ttyACM0", "/dev/ttyUSB0"])
        rc = main(["--list-ports"])
        out, err = capsys.readouterr()
        assert rc == 0
        assert out.splitlines() == ["/dev/ttyACM0", "/dev/ttyUSB0"]

        # Test empty list
        monkeypatch.setattr(dof_monitor, "list_ports", lambda: [])
        rc = main(["--list-ports"])
        out, err = capsys.readouterr()
        assert rc == 0
        assert "no serial ports" in out

    def test_main_no_source_is_usage_error(self):
        """Test: no source argument is an error."""
        with pytest.raises(SystemExit) as e:
            main([])
        assert e.value.code == 2

    def test_main_source_flags_mutually_exclusive(self):
        """Test: source flags are mutually exclusive."""
        with pytest.raises(SystemExit) as e:
            main(["--mock", "--replay", "x.csv"])
        assert e.value.code == 2

        with pytest.raises(SystemExit) as e:
            main(["--mock", "--port", "x"])
        assert e.value.code == 2

    @pytest.mark.parametrize("args", [
        ["--mock", "--duration", "0"],
        ["--mock", "--duration", "-1"],
        ["--mock", "--duration", "nan"],
        ["--mock", "--duration", "abc"],
        ["--mock", "--quiet-every", "0"],
        ["--port", "x", "--baud", "0"],
    ])
    def test_main_rejects_bad_numbers(self, args):
        """Test: invalid numeric arguments are rejected."""
        with pytest.raises(SystemExit) as e:
            main(args)
        assert e.value.code == 2

    def test_main_bad_serial_args_exit_2(self, capsys):
        """Test: bad serial arguments exit with code 2 from dof_serial eager validation."""
        rc = main(["--port", "   "])
        out, err = capsys.readouterr()
        assert rc == 2
        assert "port must be a non-empty str" in err

    @pytest.mark.parametrize("args", [
        ["--mock", "--fuse", "--alpha", "1.5"],
        ["--mock", "--fuse", "--alpha", "-0.1"],
        ["--mock", "--fuse", "--alpha", "nan"],
        ["--mock", "--fuse", "--alpha", "abc"],
        ["--mock", "--publish", "--udp-port", "0"],
        ["--mock", "--publish", "--udp-port", "65536"],
        ["--mock", "--publish", "--udp-port", "abc"],
    ])
    def test_main_rejects_bad_fuse_publish_args(self, args):
        """Test: invalid fuse/publish arguments are rejected."""
        with pytest.raises(SystemExit) as e:
            main(args)
        assert e.value.code == 2
