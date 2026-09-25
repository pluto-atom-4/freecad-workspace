#!/usr/bin/env python3
"""
Unit tests for pyserial line source (dof_serial).

Usage:
    cd poc/esp32-dof
    mamba run -n esp32-dof python3 -m pytest -q monitor/test_esp32dof_serial.py
"""

import sys
import itertools
import types
from pathlib import Path
from time import monotonic

import pytest
import serial
import serial.tools.list_ports as lp

sys.path.insert(0, str(Path(__file__).resolve().parent))

import dof_serial
from dof_serial import serial_lines, list_ports, DofSerialError
from dof_frame import parse_line, FrameError


class FakePort:
    def __init__(self, script, idle=False):
        self.script = list(script)
        self.idle = idle
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.script[0]) if self.script and isinstance(self.script[0], bytes) else 0

    def read(self, size=1):
        if self.closed:
            raise serial.PortNotOpenError()
        if not self.script:
            if self.idle:
                return b""
            raise AssertionError("fake script exhausted (test bug)")
        item = self.script.pop(0)
        if isinstance(item, BaseException):
            raise item
        return item

    def close(self):
        self.closed = True


@pytest.fixture
def delays(monkeypatch):
    rec = []
    monkeypatch.setattr(dof_serial, "sleep", rec.append)
    return rec


def install(monkeypatch, *items):
    """items: FakePort (successful open) or Exception instance (open failure), consumed in order."""
    queue, opened, calls = list(items), [], []
    def fake(url, *a, **k):
        calls.append((url, a, k))
        item = queue.pop(0)             # IndexError = unexpected extra open (test bug)
        if isinstance(item, BaseException):
            raise item
        opened.append(item)
        return item
    monkeypatch.setattr(serial, "serial_for_url", fake)
    return opened, calls


class TestBasicLineHandling:
    """Test basic line yielding and CRLF stripping."""

    def test_yields_lines_strips_crlf_and_opens_with_url_api(self, monkeypatch):
        """Test: opens port with serial_for_url, strips CRLF, yields lines."""
        p = FakePort([b"frag\n", b"IMU,1*00\r\n", b"IMU,2*00\n"])
        opened, calls = install(monkeypatch, p)
        g = serial_lines("/dev/ttyFAKE")
        line1 = next(g)
        line2 = next(g)
        assert line1 == "IMU,1*00"
        assert line2 == "IMU,2*00"
        assert not p.closed
        g.close()
        assert p.closed
        # Verify serial_for_url was called correctly
        assert len(calls) == 1
        assert calls[0] == ("/dev/ttyFAKE", (115200,), {"timeout": 1.0})

    def test_first_fragment_after_open_discarded(self, monkeypatch):
        """Test: first line after open is discarded (sync-discard)."""
        # Case 1: first fragment on same chunk
        p = FakePort([b"tial,frag*00\n", b"A\n"])
        opened, calls = install(monkeypatch, p)
        g = serial_lines("/dev/ttyFAKE")
        assert next(g) == "A"
        g.close()

        # Case 2: first fragment spans chunks
        p2 = FakePort([b"tial", b"frag\n", b"A\n"])
        opened2, calls2 = install(monkeypatch, p2)
        g2 = serial_lines("/dev/ttyFAKE")
        assert next(g2) == "A"
        g2.close()

    def test_partial_line_stitched_across_timeouts(self, monkeypatch):
        """Test: partial lines (no newline) accumulate until newline arrives."""
        p = FakePort([b"x\n", b"IMU,3,", b"", b"77*1", b"A\n"])
        opened, calls = install(monkeypatch, p)
        g = serial_lines("/dev/ttyFAKE")
        assert next(g) == "IMU,3,77*1A"
        g.close()

    def test_blank_lines_skipped(self, monkeypatch):
        """Test: empty lines (just \n or \r\n) are skipped."""
        p = FakePort([b"x\n", b"\n", b"\r\n", b"a\n", b"\n", b"b\r\n"])
        opened, calls = install(monkeypatch, p)
        g = serial_lines("/dev/ttyFAKE")
        assert next(g) == "a"
        assert next(g) == "b"
        g.close()

    def test_timeout_does_not_end_iterator(self, monkeypatch):
        """Test: empty read (timeout) is ignored; iterator keeps looping."""
        p = FakePort([b"x\n", b"", b"", b"", b"A\n"], idle=False)
        opened, calls = install(monkeypatch, p)
        g = serial_lines("/dev/ttyFAKE")
        assert next(g) == "A"
        g.close()

    def test_multiple_lines_in_one_chunk(self, monkeypatch):
        """Test: multiple lines in one chunk are yielded separately."""
        p = FakePort([b"x\nA\nB\nC", b"D\n"])
        opened, calls = install(monkeypatch, p)
        g = serial_lines("/dev/ttyFAKE")
        assert next(g) == "A"
        assert next(g) == "B"
        assert next(g) == "CD"
        g.close()


class TestInvalidUTF8:
    """Test UTF-8 error handling."""

    def test_invalid_utf8_replaced_and_rejected_by_parser(self, monkeypatch):
        """Test: invalid UTF-8 is replaced with replacement char; parser rejects it."""
        p = FakePort([b"x\n", b"IMU,\xff\xfe*00\n"])
        opened, calls = install(monkeypatch, p)
        g = serial_lines("/dev/ttyFAKE")
        line = next(g)
        # Line should contain replacement character
        assert "�" in line or "�" in line
        # Parser should reject it (FrameError due to checksum mismatch or format)
        with pytest.raises(FrameError):
            parse_line(line)
        g.close()


class TestNoiseCap:
    """Test buffer cap (4096 bytes without newline is dropped)."""

    def test_noise_run_over_cap_dropped_then_resync(self, monkeypatch):
        """Test: >4096 bytes without \\n is dropped; next good line syncs correctly."""
        p = FakePort([b"x\n", b"A"*3000, b"B"*3000, b"tail\n", b"good\n"])
        opened, calls = install(monkeypatch, p)
        g = serial_lines("/dev/ttyFAKE")
        assert next(g) == "good"
        g.close()


class TestReconnect:
    """Test reconnection on port error."""

    def test_reconnect_reopens_and_continues(self, monkeypatch, delays, capsys):
        """Test: on read error with reconnect=True, port is closed/reopened, iteration continues."""
        p1 = FakePort([b"x\n", b"A\n", serial.SerialException("device reports readiness to read but returned no data")])
        p2 = FakePort([b"frag\n", b"B\n"])
        opened, calls = install(monkeypatch, p1, p2)
        g = serial_lines("/dev/ttyFAKE", reconnect=True)
        assert next(g) == "A"
        assert next(g) == "B"
        assert opened == [p1, p2]
        assert p1.closed
        assert not p2.closed
        assert delays == [1.0]
        captured = capsys.readouterr()
        assert "reconnecting" in captured.err
        g.close()
        assert p2.closed

    def test_reconnect_retries_failed_reopen(self, monkeypatch, delays):
        """Test: failed reopens are retried with delay; eventually succeeds."""
        p1 = FakePort([b"x\n", serial.SerialException("gone")])
        reopen_fail = serial.SerialException("could not open port /dev/ttyFAKE: [Errno 2] No such file or directory")
        p2 = FakePort([b"x\n", b"A\n"])
        opened, calls = install(monkeypatch, p1, reopen_fail, p2)
        g = serial_lines("/dev/ttyFAKE", reconnect=True)
        assert next(g) == "A"
        assert delays == [1.0, 1.0]
        assert len(calls) == 3  # first open, reopen fail, reopen success
        g.close()

    def test_no_reconnect_reraises_original(self, monkeypatch, delays):
        """Test: with reconnect=False, OSError is re-raised."""
        p1 = FakePort([b"x\n", b"A\n", serial.SerialException("boom")])
        opened, calls = install(monkeypatch, p1)
        g = serial_lines("/dev/ttyFAKE", reconnect=False)
        assert next(g) == "A"
        with pytest.raises(serial.SerialException, match="boom"):
            next(g)
        assert p1.closed
        assert len(opened) == 1
        assert delays == []
        with pytest.raises(StopIteration):
            next(g)

    def test_raw_oserror_also_handled(self, monkeypatch, delays):
        """Test: raw OSError (not SerialException subclass) is caught and handled."""
        p1 = FakePort([b"x\n", OSError(5, "EIO")])
        p2 = FakePort([b"x\n", b"A\n"])
        opened, calls = install(monkeypatch, p1, p2)
        g = serial_lines("/dev/ttyFAKE", reconnect=True)
        assert next(g) == "A"
        assert delays == [1.0]
        g.close()


class TestFirstOpenError:
    """Test first-open (lazy) error handling."""

    @pytest.mark.parametrize("errno_val,desc", [
        (13, "Permission denied"),
        (2, "No such file or directory"),
    ])
    def test_first_open_failure_raises_dof_error_even_with_reconnect(self, monkeypatch, delays, errno_val, desc):
        """Test: first-open failure raises DofSerialError even with reconnect=True."""
        orig = serial.SerialException(errno_val, f"could not open port /dev/ttyFAKE: [Errno {errno_val}] {desc}")
        opened, calls = install(monkeypatch, orig)
        g = serial_lines("/dev/ttyFAKE", reconnect=True)
        assert calls == []  # lazy: no open at call
        with pytest.raises(DofSerialError) as ei:
            next(g)
        msg = str(ei.value)
        assert "dialout" in msg
        assert "--list-ports" in msg
        assert "usermod -aG dialout" in msg
        assert ei.value.__cause__ is orig
        assert isinstance(ei.value, serial.SerialException)
        assert len(calls) == 1
        assert delays == []


class TestKeyboardInterrupt:
    """Test KeyboardInterrupt propagation."""

    def test_keyboard_interrupt_propagates_and_closes(self, monkeypatch):
        """Test: KeyboardInterrupt is not caught; port is still closed."""
        p = FakePort([b"x\n", KeyboardInterrupt()])
        opened, calls = install(monkeypatch, p)
        g = serial_lines("/dev/ttyFAKE")
        with pytest.raises(KeyboardInterrupt):
            next(g)
        assert p.closed


class TestMaxDuration:
    """Test max_duration_s parameter."""

    def test_max_duration_ends_iterator_cleanly(self, monkeypatch):
        """Test: max_duration_s causes iterator to end when deadline is reached."""
        p = FakePort([], idle=True)
        opened, calls = install(monkeypatch, p)
        clock = itertools.count(0.0, 0.4)
        monkeypatch.setattr(dof_serial, "monotonic", lambda: next(clock))
        g = serial_lines("/dev/ttyFAKE", max_duration_s=1.0)
        assert list(g) == []
        assert p.closed

    def test_max_duration_checked_during_reconnect(self, monkeypatch, delays):
        """Test: deadline is checked during reconnect loop; returns early if expired."""
        p1 = FakePort([b"x\n", serial.SerialException("gone")])
        # Provide several reopen failures, but deadline expires
        reopen_fail1 = serial.SerialException("reopen failed 1")
        reopen_fail2 = serial.SerialException("reopen failed 2")
        opened, calls = install(monkeypatch, p1, reopen_fail1, reopen_fail2)

        # Clock ticks: 0.0, 0.4, 0.8, 1.2, ...
        # deadline = 0.0 + 1.0 = 1.0
        # First read: 0.0, no deadline
        # OSError at 0.0, sleep(1.0), then check at 0.4 (deadline not expired)
        # Try reopen at 0.4, fails
        # sleep(1.0), then check at 0.8 (deadline not expired)
        # Try reopen at 0.8, fails
        # sleep(1.0), then check at 1.2 (deadline EXPIRED)
        clock = itertools.count(0.0, 0.4)
        monkeypatch.setattr(dof_serial, "monotonic", lambda: next(clock))
        g = serial_lines("/dev/ttyFAKE", max_duration_s=1.0, reconnect=True)
        assert list(g) == []
        assert p1.closed
        assert len(delays) <= 2  # two or fewer sleeps before deadline


class TestArgumentValidation:
    """Test eager argument validation."""

    @pytest.mark.parametrize("kwargs,desc", [
        ({"port": ""}, "empty port"),
        ({"port": "  "}, "whitespace-only port"),
        ({"port": 123}, "port as int"),
        ({"baud": 0}, "baud=0"),
        ({"baud": -9600}, "baud negative"),
        ({"baud": 1.5}, "baud as float"),
        ({"baud": True}, "baud as bool"),
        ({"timeout_s": 0}, "timeout_s=0"),
        ({"timeout_s": -1}, "timeout_s negative"),
        ({"timeout_s": float("nan")}, "timeout_s as NaN"),
        ({"timeout_s": float("inf")}, "timeout_s as Inf"),
        ({"reconnect": "yes"}, "reconnect as string"),
        ({"max_duration_s": 0}, "max_duration_s=0"),
        ({"max_duration_s": float("nan")}, "max_duration_s as NaN"),
        ({"max_duration_s": -1}, "max_duration_s negative"),
    ])
    def test_arguments_validated_eagerly(self, monkeypatch, kwargs, desc):
        """Test: all invalid arguments raise ValueError AT CALL TIME."""
        opened, calls = install(monkeypatch)  # NO items: any serial_for_url call -> IndexError (test bug)
        with pytest.raises(ValueError):
            # If 'port' is in kwargs, don't pass a positional port
            if 'port' in kwargs:
                serial_lines(**kwargs)
            else:
                serial_lines("/dev/ttyFAKE", **kwargs)
        # No open should have been attempted
        assert calls == []


class TestListPorts:
    """Test list_ports() function."""

    def test_list_ports_sorted_devices(self, monkeypatch):
        """Test: list_ports returns sorted device list."""
        def fake_comports():
            return [
                types.SimpleNamespace(device="/dev/ttyUSB1"),
                types.SimpleNamespace(device="/dev/ttyACM0"),
            ]
        monkeypatch.setattr(lp, "comports", fake_comports)
        result = list_ports()
        assert result == ["/dev/ttyACM0", "/dev/ttyUSB1"]

    def test_list_ports_empty(self, monkeypatch):
        """Test: list_ports returns empty list when no ports."""
        monkeypatch.setattr(lp, "comports", lambda: [])
        result = list_ports()
        assert result == []

    def test_list_ports_real_smoke(self):
        """Test: real list_ports() returns a list of strings (smoke test)."""
        result = list_ports()
        assert isinstance(result, list)
        assert all(isinstance(p, str) for p in result)


class TestRealLoopURL:
    """Test real pyserial loop:// URL."""

    def test_real_loop_url_path(self, monkeypatch):
        """Test: real loop:// URL with actual pyserial (prefilled buffer)."""
        real = serial.serial_for_url
        def prefilled(url, *a, **k):
            p = real(url, *a, **k)
            p.write(b"frag\nIMU,1*00\r\n\nIMU,2*00\nIMU,3,")
            return p
        monkeypatch.setattr(serial, "serial_for_url", prefilled)
        lines = list(serial_lines("loop://", timeout_s=0.05, reconnect=False, max_duration_s=0.3))
        assert lines == ["IMU,1*00", "IMU,2*00"]


if __name__ == "__main__":
    pytest.main([__file__, "-q"])
