#!/usr/bin/env python3
"""pyserial frame line source. Yields raw frame lines (Iterator[str]): non-empty, stripped of
trailing \r\n -- same contract as dof_sources.mock_lines/csv_replay_lines. Does NOT parse frames
(consumers call dof_frame.parse_line). Works with /dev/ttyACM0 and pyserial URLs (loop://)."""
import math
import sys
from time import monotonic, sleep
from typing import Iterator, List, Optional

import serial
from serial.tools import list_ports as _list_ports_mod

_MAX_BUF_BYTES = 4096       # carry-over cap: noise without "\n" is dropped
_READ_CHUNK = 4096
_RECONNECT_DELAY_S = 1.0


class DofSerialError(serial.SerialException):
    """First-open failure (missing device / permission). __cause__ = original error."""


def list_ports() -> List[str]:
    """Sorted device names from serial.tools.list_ports.comports() (no filtering)."""
    return sorted(p.device for p in _list_ports_mod.comports())


def serial_lines(port: str, baud: int = 115200, timeout_s: float = 1.0,
                 reconnect: bool = True, *, max_duration_s: Optional[float] = None) -> Iterator[str]:
    # EAGER validation, ValueError, message style f"... got {x!r}":
    #  port: str and non-empty (strip() != "")
    #  baud: int, NOT bool, > 0
    #  timeout_s: int/float (not bool), math.isfinite, > 0
    #  reconnect: bool
    #  max_duration_s: None or (int/float not bool, finite, > 0)
    if not isinstance(port, str) or port.strip() == "":
        raise ValueError(f"port must be a non-empty str, got {port!r}")
    if isinstance(baud, bool) or not isinstance(baud, int) or baud <= 0:
        raise ValueError(f"baud must be int > 0, got {baud!r}")
    if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)) or not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError(f"timeout_s must be finite > 0, got {timeout_s!r}")
    if not isinstance(reconnect, bool):
        raise ValueError(f"reconnect must be bool, got {reconnect!r}")
    if max_duration_s is not None:
        if isinstance(max_duration_s, bool) or not isinstance(max_duration_s, (int, float)) or not math.isfinite(max_duration_s) or max_duration_s <= 0:
            raise ValueError(f"max_duration_s must be None or finite > 0, got {max_duration_s!r}")
    return _serial_gen(port, int(baud), float(timeout_s), reconnect, max_duration_s)


def _open_first(port, baud, timeout_s):
    try:
        return serial.serial_for_url(port, baud, timeout=timeout_s)
    except OSError as exc:      # SerialException is an OSError
        raise DofSerialError(
            f"cannot open serial port {port!r}: {exc}. Check the device path (run with "
            f"--list-ports) and, on Linux, that you are in the 'dialout' group: "
            f"sudo usermod -aG dialout $USER (then log out and back in).") from exc


def _close_quietly(ser):
    try:
        ser.close()
    except OSError:
        pass


def _log(msg):
    print(f"[dof_serial] {msg}", file=sys.stderr, flush=True)


def _serial_gen(port, baud, timeout_s, reconnect, max_duration_s):
    deadline = None if max_duration_s is None else monotonic() + max_duration_s
    ser = _open_first(port, baud, timeout_s)        # raises DofSerialError; nothing to close
    buf = bytearray()
    synced = False                                   # discard through first "\n" after (re)open
    try:
        while True:
            if deadline is not None and monotonic() >= deadline:
                return
            try:
                data = ser.read(max(1, min(ser.in_waiting, _READ_CHUNK)))
            except OSError as exc:                   # SerialException/PortNotOpenError/raw OSError
                if not reconnect:
                    raise
                _log(f"{port}: {exc}; reconnecting in {_RECONNECT_DELAY_S:g}s")
                _close_quietly(ser)
                ser = None
                while ser is None:
                    if deadline is not None and monotonic() >= deadline:
                        return
                    sleep(_RECONNECT_DELAY_S)
                    try:
                        ser = serial.serial_for_url(port, baud, timeout=timeout_s)
                    except OSError as exc2:
                        _log(f"{port}: reopen failed: {exc2}")
                buf.clear()
                synced = False
                continue
            if not data:                             # read timeout: keep looping
                continue
            buf += data
            while True:
                i = buf.find(b"\n")
                if i < 0:
                    break
                raw = bytes(buf[:i])
                del buf[:i + 1]
                if not synced:                       # first fragment after open/overflow
                    synced = True
                    continue
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                if line:
                    yield line
            # After the extraction loop buf contains no "\n".
            if not synced:
                buf.clear()                # unsynced and no "\n" left -> pure junk
            elif len(buf) > _MAX_BUF_BYTES:
                buf.clear()
                synced = False             # noise run: drop, resync at next "\n"
    finally:
        if ser is not None:
            _close_quietly(ser)
