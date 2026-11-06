#!/usr/bin/env python3
"""
ctrl-wheels WHL2 twin bridge: reads WHL2 lines from a serial source, splits each
into a left and a right single-wheel sample, feeds one Relay per wheel, sends one
UDP v2 command (both wheels) per accepted line to the headless ctrl_wheels
stand-in and logs one CSV row per WHL2 line plus one per silence episode
(ctrl_wheels_log.COLUMNS).

The command uses the model velocity only, forward-positive on both wheels; the
raw columns of the line are logged but never drive a command. If either relay is
stale or the state is not 0, both wheels get (0.0, 0.0) with stale 1 (they stop
together). That stop is not a latch: a fresh state-0 line resumes. Each wheel is
clamped to +-3.0 silently. A line that is not WHL2 is a bad line.

Sequence rule (F22). A forward step (seq gap below half the 16-bit range, and the t_us
step is at most (missed + 1) x 40 ms plus 1 s) counts its gap as missed_whl. A seq that
goes back (gap at or above half) while t_us goes back by less than 1 s is a repeat or
reorder: missed 0, and the baseline stays at the highest seq seen. Anything else is a
firmware restart: missed 0, stats.restarts +1, one log line, and the baseline moves to the
new seq and t_us. Known limits (thresholds: _REORDER_BACK_US, _FWD_SLACK_US, _FRAME_US_MAX):
(i) a seq that goes back while t_us goes forward is read as a restart (both counters are
monotonic in the firmware);
(ii) a restart is read as a reorder only when the new seq is 1 to 32768 behind the baseline
(mod 65536) AND its first t_us is within 1 s BELOW the old t_us (a restart from a high old seq,
where the new seq looks like a forward wrap, or with a t_us above the old one, is a restart);
(iii) a restart that is read as a reorder (ii) is only noticed when t_us passes the old
baseline, up to 1 s of lines later, and those lines count missed 0;
(iv) a LATE frame more than 1 s of t_us behind the baseline, after a forward jump of 1 s or
more, is read as a restart and the next live frame then adds a false missed count (example:
frames 0..100, then seq 161 at +1.2 s, then the late seq 100: one false restart and a false
61 missed); this edge is accepted because the firmware stream is in order (only a replay or
relay could reorder), and a stricter rule (second consecutive line before accepting a
restart) was left out;
(v) a real outage longer than (missed + 1) x 40 ms plus 1 s is logged as a restart and its
missed frames are not counted (accepted for the sim CSV);
(vi) all thresholds are unmeasured guesses for the 50 Hz firmware.

Command line: build_parser() and main(argv) -> int. Run from poc/ctrl-wheels:
    python3 -m ctrl_wheels_msg --step 0.5 --seconds 5 | python3 ctrl_wheels_bridge.py --stdin --record ac1.csv
    python3 ctrl_wheels_bridge.py --port rfc2217://localhost:4002 --record ac1.csv
Exactly one of --port / --stdin. --record is required; an existing file is refused
unless --overwrite. Exit codes: 0 run finished (EOF or Ctrl-C), 2 usage error, a failed
source (at open, or mid-run: the 'line source stopped' line is logged BEFORE the summary)
or a --record error. A receive error stops that drain pass only; the next poll drains again, and the run still ends normally.
Ctrl-C while run_bridge is running ends the run through stop_event, prints the summary and
returns 0. Ctrl-C while waiting to open the source returns 0 without a summary. A KeyboardInterrupt
raised inside run_bridge's shutdown returns 0 without a summary. A Ctrl-C during startup (after the
source opens, before the SIGINT handler is installed) or during the final cleanup (after the handler
is restored) is not handled and ends with a traceback and a non-zero exit.
An unreachable URL with --duration returns 2 on purpose: nothing was ever opened, so the
run is a failed source, not a clean finish.

SIM ONLY. The bridge is a relay, not a control loop, and makes no balance claim.
The wheel signs are UNVERIFIED; they are flipped only through the Relay sign argument,
which the CLI exposes as --sign-left and --sign-right (the only sign flips in the CLI).
Latency is UNMEASURED: host_rtt_ms is wall time, not link latency.

The serial helpers (DofSerialError, _is_url, _open_first, _open_url_waiting,
_close_quietly and the read and sync loop) are copied, not imported, from
poc/esp32-dof/monitor/dof_serial.py in this repo, with no reconnect loop.
"""

from __future__ import annotations

import argparse
import math
import queue
import signal
import socket
import sys
import threading
import time
from collections.abc import Callable, Iterable, Iterator
from dataclasses import dataclass, fields

import serial

from ctrl_wheels_log import CsvLog
from ctrl_wheels_msg import SEQ_MOD, T_US_MAX, WhlError, parse_line, split
from ctrl_wheels_relay import POLL_S, Relay
from ctrl_wheels_udp import (
    PORT_CMD,
    PORT_ECHO,
    UdpMsgError,
    clamp,
    decode_echo,
    drain_all,
    encode_cmd,
    open_listener,
)

_MAX_BUF_BYTES = 4096
_READ_CHUNK = 4096
_RECONNECT_DELAY_S = 1.0
_WAIT_HEARTBEAT_S = 10.0
_URL_WAIT_CAP_S = 300.0
_SEQ_HALF = SEQ_MOD // 2
# Restart rule (F22). All three are unmeasured guesses for the 50 Hz firmware; see the module docstring.
_REORDER_BACK_US = 1_000_000  # a seq going back with t_us stepping back by less than this is a repeat or reorder
_FWD_SLACK_US = 1_000_000  # extra t_us allowed on a forward step, on top of the frame period
_FRAME_US_MAX = 40_000  # largest expected t_us step per frame


class DofSerialError(serial.SerialException):
    """Open or read failure of the line source. __cause__ = original error."""


@dataclass
class BridgeStats:
    whl_lines: int = 0
    bad_lines: int = 0
    cmds_sent: int = 0
    echoes_matched: int = 0
    unmatched_echoes: int = 0
    bad_echoes: int = 0
    missed_whl_total: int = 0
    stale_cmds: int = 0
    restarts: int = 0

    def __post_init__(self) -> None:
        # BridgeStats has no send-error or source-error field, so these are plain attributes.
        self.send_errors = 0
        self.source_error: str | None = None  # text of a line source error that ended the run
        # Set True in run_bridge's finally (every exit after the reader thread has started); a reader
        # failure after that is a consequence of shutdown (the caller closes the port next).
        self.closing = False
        self.closing_lock = threading.Lock()


@dataclass
class _Pending:
    sent_t: float
    row: dict | None


def _log(msg: str) -> None:
    print(f"[ctrl_wheels_bridge] {msg}", file=sys.stderr, flush=True)


def _is_url(port: str) -> bool:
    return "://" in port


def _close_quietly(ser) -> None:
    try:
        ser.close()
    except OSError:
        pass


def _open_first(port: str, baud: int, timeout_s: float):
    try:
        return serial.serial_for_url(port, baud, timeout=timeout_s)
    except OSError as exc:  # SerialException is an OSError
        raise DofSerialError(
            f"cannot open serial port {port!r}: {exc}. Check the device path and, on Linux, that you are in the 'dialout' group: "
            f"sudo usermod -aG dialout $USER (then log out and back in).") from exc


def _open_url_waiting(port: str, baud: int, timeout_s: float, deadline: float | None):
    # Returns an open port, or None when deadline passes first;
    # raises DofSerialError when _URL_WAIT_CAP_S passes first.
    cap_deadline = time.monotonic() + _URL_WAIT_CAP_S
    attempt = 0
    last_text = None
    last_log = None
    while True:
        if deadline is not None and time.monotonic() >= deadline:
            return None
        if time.monotonic() >= cap_deadline:
            raise DofSerialError(
                f"{port!r} did not open within {_URL_WAIT_CAP_S:g} s: {last_text}. "
                f"Start the simulator (or check the URL) and run again.")
        attempt += 1
        try:
            return serial.serial_for_url(port, baud, timeout=timeout_s)
        except OSError as exc:
            text = f"{type(exc).__name__}: {exc}"
            now = time.monotonic()
            if text != last_text or last_log is None or now - last_log >= _WAIT_HEARTBEAT_S:
                _log(f"waiting for {port} (attempt {attempt}): {text}; "
                     f"retrying every {_RECONNECT_DELAY_S:g}s")
                last_text, last_log = text, now
        now = time.monotonic()
        limit = cap_deadline if deadline is None else min(cap_deadline, deadline)
        time.sleep(max(0.0, min(_RECONNECT_DELAY_S, limit - now)))


def _open_serial(port: str, baud: int, timeout_s: float, deadline: float | None):
    # Opens now. Device paths raise DofSerialError at once. URL ports return None
    # when deadline passes first, and raise DofSerialError after _URL_WAIT_CAP_S.
    if _is_url(port):
        return _open_url_waiting(port, baud, timeout_s, deadline)
    return _open_first(port, baud, timeout_s)


def _read_serial(ser, port: str, deadline: float | None) -> Iterator[str]:
    # Owns ser: closed when the generator finishes or is closed.
    buf = bytearray()
    synced = False  # discard through first "\n" after open
    try:
        while True:
            if deadline is not None and time.monotonic() >= deadline:
                return
            try:
                data = ser.read(max(1, min(ser.in_waiting, _READ_CHUNK)))
            except OSError as exc:
                raise DofSerialError(f"{port!r}: read failed: {exc}") from exc
            if not data:  # read timeout: keep looping
                continue
            buf += data
            while True:
                i = buf.find(b"\n")
                if i < 0:
                    break
                raw = bytes(buf[:i])
                del buf[:i + 1]
                if not synced:  # first fragment after open
                    synced = True
                    continue
                line = raw.decode("ascii", errors="replace").strip()
                if line:
                    yield line
            if not synced:
                buf.clear()
            elif len(buf) > _MAX_BUF_BYTES:
                buf.clear()
                synced = False  # noise run: drop, resync at next "\n"
    finally:
        _close_quietly(ser)


def _serial_gen(port: str, baud: int, timeout_s: float, max_duration_s: float | None) -> Iterator[str]:
    deadline = None if max_duration_s is None else time.monotonic() + max_duration_s
    ser = _open_serial(port, baud, timeout_s, deadline)
    if ser is None:
        return
    yield from _read_serial(ser, port, deadline)


def open_serial_lines(
    port: str,
    baud: int = 115200,
    timeout_s: float = 0.05,
    max_duration_s: float | None = None,
) -> Iterator[str]:
    """Yield stripped non-empty lines from a serial port or pyserial URL.

    Device paths raise DofSerialError at the first next(). URL ports (containing
    "://") retry the first open every 1 s for up to 300 s or max_duration_s.
    """
    if not isinstance(port, str) or port.strip() == "":
        raise ValueError(f"port must be a non-empty str, got {port!r}")
    if isinstance(baud, bool) or not isinstance(baud, int) or baud <= 0:
        raise ValueError(f"baud must be int > 0, got {baud!r}")
    if (isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float))
            or not math.isfinite(timeout_s) or timeout_s <= 0):
        raise ValueError(f"timeout_s must be finite > 0, got {timeout_s!r}")
    if max_duration_s is not None and (
            isinstance(max_duration_s, bool) or not isinstance(max_duration_s, (int, float))
            or not math.isfinite(max_duration_s) or max_duration_s <= 0):
        raise ValueError(f"max_duration_s must be None or finite > 0, got {max_duration_s!r}")
    return _serial_gen(port, int(baud), float(timeout_s),
                       None if max_duration_s is None else float(max_duration_s))


def _read_lines(lines: Iterable[str], out: queue.Queue, finished: threading.Event,
                stats: BridgeStats) -> None:
    try:
        for line in lines:
            out.put(line)
    except KeyboardInterrupt:  # Ctrl-C ends the source like EOF
        pass
    except BaseException as exc:  # the line source ends the run; it does not crash the bridge
        with stats.closing_lock:
            if not stats.closing:
                stats.source_error = f"{type(exc).__name__}: {exc}"
                _log(f"line source stopped: {exc}")
    finally:
        finished.set()


def _row(host_t_s: float, seq: int, left: float, right: float, stale: int,
         w2=None, missed=None) -> dict:
    row = {
        "host_t_s": host_t_s,
        "cmd_seq": seq,
        "cmd_left_rad_s": left,
        "cmd_right_rad_s": right,
        "stale": stale,
        "missed_whl": missed,
    }
    if w2 is not None:
        row.update(
            whl_seq=w2.seq,
            whl_t_us=w2.t_us,
            whl_model_rad_s=w2.model_rad_s,
            whl_raw_l=w2.raw_l,
            whl_raw_r=w2.raw_r,
            whl_state=w2.state,
        )
    return row


class _Run:
    """State of one run_bridge call. Single-threaded use (main loop only)."""

    def __init__(self, relay_l, relay_r, cmd_addr, echo_sock, log, clock, echo_timeout_s, stats):
        self.relay_l = relay_l
        self.relay_r = relay_r
        self.cmd_addr = cmd_addr
        self.echo_sock = echo_sock
        self.log = log
        self.clock = clock
        self.echo_timeout_s = echo_timeout_s
        self.stats = stats
        self.cmd_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.next_cmd_seq = 0
        self.pending: dict[int, _Pending] = {}
        self.last_whl_seq: int | None = None
        self.last_whl_t_us: int | None = None
        self.last_cmd_stale: int | None = None
        self.stale_episode = False

    def _send(self, left_rad_s: float, right_rad_s: float, stale: int, *, w2=None, missed=None,
              log_row=True) -> None:
        seq = self.next_cmd_seq
        self.next_cmd_seq += 1
        now = self.clock()
        left = clamp(left_rad_s, "wheels.left")
        right = clamp(right_rad_s, "wheels.right")
        payload = encode_cmd(seq, left, right, stale)
        row = _row(now, seq, left, right, stale, w2, missed) if log_row else None
        try:
            self.cmd_tx.sendto(payload, self.cmd_addr)
        except OSError as exc:
            # Nothing reached the wire: no pending row, and last_cmd_stale is unchanged
            # so finish() still sends its stop.
            self.stats.send_errors += 1
            _log(f"command {seq} send failed: {exc}")
            return
        self.pending[seq] = _Pending(now, row)
        self.last_cmd_stale = stale
        self.stale_episode = stale == 1
        self.stats.cmds_sent += 1
        if stale == 1:
            self.stats.stale_cmds += 1

    def handle_line(self, line: str) -> None:
        try:
            w2 = parse_line(line)
        except WhlError:
            self.stats.bad_lines += 1
            return
        self.stats.whl_lines += 1
        missed = 0
        if self.last_whl_seq is None:
            self.last_whl_seq = w2.seq
            self.last_whl_t_us = w2.t_us
        else:
            gap = (w2.seq - self.last_whl_seq - 1) % SEQ_MOD
            dt = (w2.t_us - self.last_whl_t_us) % (T_US_MAX + 1)
            back = (self.last_whl_t_us - w2.t_us) % (T_US_MAX + 1)
            if gap < _SEQ_HALF and dt <= (gap + 1) * _FRAME_US_MAX + _FWD_SLACK_US:
                # forward: count the gap and move on
                missed = gap
                self.last_whl_seq = w2.seq
                self.last_whl_t_us = w2.t_us
            elif gap >= _SEQ_HALF and back < _REORDER_BACK_US:
                pass  # repeat or reorder: not a gap; the baseline keeps the highest seq and its t_us
            else:
                self.stats.restarts += 1
                _log(f"firmware restart detected (seq {self.last_whl_seq} -> {w2.seq})")
                self.last_whl_seq = w2.seq
                self.last_whl_t_us = w2.t_us
        self.stats.missed_whl_total += missed
        wl, wr = split(w2)
        self.relay_l.on_whl(wl)
        self.relay_r.on_whl(wr)
        out_l = self.relay_l.poll()
        out_r = self.relay_r.poll()
        stale = 1 if (out_l is None or out_r is None or out_l.stale or out_r.stale) else 0
        if stale:
            left, right = 0.0, 0.0
        else:
            left, right = out_l.wheel_rad_s, out_r.wheel_rad_s
        self._send(left, right, stale, w2=w2, missed=missed)

    def check_silence(self) -> None:
        # One stale command per episode when either relay goes stale without a new line.
        out_l = self.relay_l.poll()
        out_r = self.relay_r.poll()
        either = (out_l is not None and out_l.stale == 1) or (out_r is not None and out_r.stale == 1)
        if either and not self.stale_episode:
            self._send(0.0, 0.0, 1)

    def drain_echoes(self) -> None:
        for data in drain_all(self.echo_sock):
            try:
                echo = decode_echo(data)
            except (UdpMsgError, RecursionError):
                self.stats.bad_echoes += 1
                continue
            seq = echo.cmd_seq_echo
            pend = self.pending.pop(seq, None) if seq is not None else None
            if pend is None:
                self.stats.unmatched_echoes += 1
                continue
            self.stats.echoes_matched += 1
            if pend.row is not None:
                pend.row.update(
                    echo_cmd_seq=seq,
                    sim_t_s=echo.sim_t_s,
                    angle_rad_l=echo.angle_rad_l,
                    angle_rad_r=echo.angle_rad_r,
                    vel_rad_s_l=echo.vel_rad_s_l,
                    vel_rad_s_r=echo.vel_rad_s_r,
                    host_rtt_ms=(self.clock() - pend.sent_t) * 1000.0,
                )
                self.log.write_row(pend.row)

    def flush_timeouts(self) -> None:
        now = self.clock()
        expired = [s for s, p in self.pending.items() if now - p.sent_t >= self.echo_timeout_s]
        for seq in expired:
            pend = self.pending.pop(seq)
            if pend.row is not None:
                self.log.write_row(pend.row)

    def finish(self) -> None:
        if self.last_cmd_stale != 1:
            self._send(0.0, 0.0, 1, log_row=False)

    def settle(self, poll_s: float, sleep: Callable[[float], object]) -> None:
        while self.pending:
            self.drain_echoes()
            self.flush_timeouts()
            if self.pending:
                sleep(poll_s)


def run_bridge(
    lines: Iterable[str],
    relay_l,
    relay_r,
    cmd_addr: tuple[str, int],
    echo_sock: socket.socket,
    log,
    *,
    duration_s: float | None = None,
    poll_s: float = POLL_S,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], object] = time.sleep,
    echo_timeout_s: float = 0.5,
    stop_event: threading.Event | None = None,
) -> BridgeStats:
    """Relay WHL2 lines to the wheels over UDP and log one CSV row per WHL2 line (plus one per silence episode).

    relay_l and relay_r are single-wheel Relay instances. One command (both wheels)
    per WHL2 line. A line with state != 0, or with either relay stale, sends
    (0.0, 0.0) with stale 1 and its own row: one per line, not one per episode.
    The stop is not a latch; a fresh state-0 line resumes. A stale command from
    silence (no new line) is sent once per silence episode. Raw columns are logged
    only.
    Rows are written on echo match or after echo_timeout_s with no echo. The run
    ends on duration_s, stop_event, or an exhausted line source (after draining).
    A line source that raises (other than KeyboardInterrupt) also ends the run; its
    error text is kept in BridgeStats.source_error. Once the run loop has ended (including
    the final settle wait, and after run_bridge returns or raises) a failure of the line source is
    ignored (neither logged nor recorded); a failure before that still sets
    stats.source_error and is logged.
    A final stale command is sent unless the last command was already stale. It is
    not logged as a CSV row, so cmds_sent can exceed the row count by one.
    UDP send errors are counted in BridgeStats.send_errors, not raised. A command whose
    send failed is not counted in cmds_sent, gets no CSV row and does not change the
    stale state, so the final stop is still attempted.
    cmd_seq is an unbounded counter with no wrap (the Webots controller accepts any
    non-negative int).
    The injected clock drives echo timing (echo timeouts, host_rtt_ms) and duration_s
    only; Relay uses its own clock. A clock that never advances makes settle loop
    forever, so library callers must advance it.
    """
    if duration_s is not None and (
            isinstance(duration_s, bool) or not isinstance(duration_s, (int, float))
            or not math.isfinite(duration_s) or duration_s <= 0):
        raise ValueError(f"duration_s must be None or finite > 0, got {duration_s!r}")
    if isinstance(poll_s, bool) or not isinstance(poll_s, (int, float)) or not math.isfinite(poll_s) or poll_s <= 0:
        raise ValueError(f"poll_s must be finite > 0, got {poll_s!r}")
    if (isinstance(echo_timeout_s, bool) or not isinstance(echo_timeout_s, (int, float))
            or not math.isfinite(echo_timeout_s) or echo_timeout_s < 0):
        raise ValueError(f"echo_timeout_s must be finite >= 0, got {echo_timeout_s!r}")

    stats = BridgeStats()
    run = _Run(relay_l, relay_r, cmd_addr, echo_sock, log, clock, echo_timeout_s, stats)
    lines_q: queue.Queue = queue.Queue()
    finished = threading.Event()
    reader = threading.Thread(
        target=_read_lines, args=(lines, lines_q, finished, stats), name="ctrl_wheels_lines", daemon=True)
    started = clock()
    try:
        reader.start()
        while True:
            if stop_event is not None and stop_event.is_set():
                break
            if duration_s is not None and clock() - started >= duration_s:
                break
            while True:
                try:
                    line = lines_q.get_nowait()
                except queue.Empty:
                    break
                run.handle_line(line)
            if finished.is_set() and lines_q.empty():
                break
            run.check_silence()
            run.drain_echoes()
            run.flush_timeouts()
            sleep(poll_s)
    finally:
        with stats.closing_lock:
            stats.closing = True
        try:
            run.finish()
            run.settle(poll_s, sleep)
        finally:
            run.cmd_tx.close()
    return stats


# ---------------------------------------------------------------------------
# Command line. SIM ONLY. The sign flips are only --sign-left and --sign-right, through Relay.
# ---------------------------------------------------------------------------

# The CLI refuses only 5006 (the single-wheel servo_wheel_dof port), for either port.
# The full reserved set of CONTRACT.md S4 binds tests and the fake wheel, not this CLI;
# the bridge owns 5017/5018 by default and other ports are the caller's responsibility.
_RESERVED_PORT = 5006
_LOOPBACK = "127.0.0.1"
_SERIAL_READ_TIMEOUT_S = 0.05


def _stdin_lines() -> Iterator[str]:
    # sys.stdin is read when iteration starts, so a replaced stdin is honoured.
    # Ctrl-C while reading ends the source; main then sends the final stale command.
    try:
        for raw in sys.stdin:
            line = raw.strip()
            if line:
                yield line
    except KeyboardInterrupt:
        return


def _finite_float(text: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not a number: {text!r}") from None
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError(f"must be finite, got {text!r}")
    return value


def _positive_finite(text: str) -> float:
    value = _finite_float(text)
    if value <= 0:
        raise argparse.ArgumentTypeError(f"must be > 0, got {text!r}")
    return value


def _nonnegative_finite(text: str) -> float:
    value = _finite_float(text)
    if value < 0:
        raise argparse.ArgumentTypeError(f"must be >= 0, got {text!r}")
    return value


_ECHO_TIMEOUT_MAX_S = 10.0


def _echo_timeout(text: str) -> float:
    value = _nonnegative_finite(text)
    if value > _ECHO_TIMEOUT_MAX_S:
        raise argparse.ArgumentTypeError(f"must be <= {_ECHO_TIMEOUT_MAX_S:g}, got {text!r}")
    return value


def _port_number(text: str) -> int:
    try:
        value = int(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not an integer: {text!r}") from None
    if not 1 <= value <= 65535:
        raise argparse.ArgumentTypeError(f"must be 1..65535, got {value}")
    return value


def _baud(text: str) -> int:
    try:
        value = int(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not an integer: {text!r}") from None
    if value <= 0:
        raise argparse.ArgumentTypeError(f"must be > 0, got {value}")
    return value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Relay WHL2 lines from a serial port or stdin to the ctrl_wheels stand-in over UDP "
            "and log one CSV row per WHL2 line (plus one per silence episode). SIM ONLY. Sign UNVERIFIED, latency UNMEASURED, "
            "no balance claim. pyserial is imported at module load, so it is needed even "
            "for --stdin."),
        epilog=(
            "examples:\n"
            "  python3 -m ctrl_wheels_msg --step 0.5 --seconds 5 | "
            "python3 ctrl_wheels_bridge.py --stdin --record ac1.csv\n"
            "  python3 ctrl_wheels_bridge.py --port rfc2217://localhost:4002 --record ac1.csv\n"
            "exit codes: 0 run finished (EOF or Ctrl-C); 2 usage error, failed source (at open or "
            "mid-run) or --record error. A receive error stops that drain pass only; "
            "the next poll drains again, and the run still ends normally."),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument(
        "--port", metavar="PORT",
        help="serial device path or pyserial URL such as rfc2217://localhost:4002 "
             "(URL ports retry the first open for up to 300 s)")
    source.add_argument(
        "--stdin", action="store_true",
        help="read WHL2 lines from stdin until EOF")
    parser.add_argument(
        "--record", metavar="CSV", required=True,
        help="CSV log: one row per WHL2 line plus one per silence episode (ctrl_wheels_log columns)")
    parser.add_argument(
        "--overwrite", action="store_true",
        help="replace an existing --record file (default: refuse and leave it untouched)")
    parser.add_argument(
        "--sign-left", type=int, choices=(1, -1), default=1,
        help="Webots-side sign, 1 or -1 (default 1), applied to the model value for the left "
             "wheel (Relay). UNVERIFIED. Not the chip raw sign of the generator "
             "ctrl_wheels_msg (its --sign-left/--sign-right default +1/-1)")
    parser.add_argument(
        "--sign-right", type=int, choices=(1, -1), default=1,
        help="Webots-side sign, 1 or -1 (default 1), applied to the model value for the right "
             "wheel (Relay). UNVERIFIED. Not the chip raw sign of the generator "
             "ctrl_wheels_msg (its --sign-left/--sign-right default +1/-1)")
    parser.add_argument(
        "--duration", type=_positive_finite, default=None, metavar="SECONDS",
        help="stop after SECONDS (> 0); default: run until the source ends")
    parser.add_argument(
        "--cmd-port", type=_port_number, default=PORT_CMD, metavar="N",
        help=f"UDP port of the wheel stand-in for commands (default {PORT_CMD}). "
             "Only 5006 (the single-wheel servo_wheel_dof port) is refused; the other ports "
             "reserved in CONTRACT.md S4 bind tests and the fake wheel, not this CLI, "
             "so the port choice is yours")
    parser.add_argument(
        "--echo-port", type=_port_number, default=PORT_ECHO, metavar="N",
        help=f"UDP port this bridge listens on for echoes (default {PORT_ECHO}). "
             "Only 5006 (the single-wheel servo_wheel_dof port) is refused; the other ports "
             "reserved in CONTRACT.md S4 bind tests and the fake wheel, not this CLI, "
             "so the port choice is yours")
    parser.add_argument(
        "--baud", type=_baud, default=115200,
        help="serial baud rate (default 115200); not used by --stdin")
    parser.add_argument(
        "--echo-timeout", type=_echo_timeout, default=0.5, metavar="SECONDS",
        help=f"seconds to wait for an echo before the row is written without one, 0..{_ECHO_TIMEOUT_MAX_S:g} "
             "(default 0.5); 0 writes every row without waiting for an echo")
    return parser


def _check_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.port is not None and args.port.strip() == "":
        parser.error("--port must be a non-empty device path or URL")
    if _RESERVED_PORT in (args.cmd_port, args.echo_port):
        parser.error(f"port {_RESERVED_PORT} is reserved; choose another --cmd-port/--echo-port")
    if args.cmd_port == args.echo_port:
        parser.error("--cmd-port and --echo-port must be different")


def _open_source(args: argparse.Namespace, deadline: float | None):
    # Returns (lines, ser). ser is None for --stdin. Raises DofSerialError on failure.
    if args.stdin:
        return _stdin_lines(), None
    ser = _open_serial(args.port, args.baud, _SERIAL_READ_TIMEOUT_S, deadline)
    if ser is None:
        raise DofSerialError(f"{args.port!r} did not open before --duration elapsed")
    return _read_serial(ser, args.port, deadline), ser


def _route_sigint(stop: threading.Event) -> Callable[[], None]:
    # Ctrl-C ends the loop through stop_event, so the summary still prints.
    # Signals can only be set from the main thread; elsewhere this is a no-op.
    if threading.current_thread() is not threading.main_thread():
        return lambda: None
    previous = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, lambda _signum, _frame: stop.set())

    def restore() -> None:
        signal.signal(
            signal.SIGINT,
            previous if previous is not None else signal.default_int_handler)

    return restore


def _summary_text(stats: BridgeStats) -> str:
    parts = [f"{f.name}={getattr(stats, f.name)}" for f in fields(stats)]
    parts.append(f"send_errors={stats.send_errors}")
    return " ".join(parts)


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    _check_args(parser, args)
    deadline = None if args.duration is None else time.monotonic() + args.duration

    try:
        lines, ser = _open_source(args, deadline)
    except DofSerialError as exc:
        _log(str(exc))
        return 2
    except KeyboardInterrupt:
        _log("interrupted while opening the source")
        return 0

    try:
        try:
            sock = open_listener(args.echo_port)
        except OSError as exc:
            _log(f"cannot listen for echoes on UDP port {args.echo_port}: {exc}")
            return 2

        try:
            log = CsvLog(args.record, overwrite=args.overwrite)
        except OSError as exc:
            sock.close()
            hint = " (pass --overwrite to replace it)" if isinstance(exc, FileExistsError) else ""
            _log(f"cannot write --record {args.record!r}: {exc}{hint}")
            return 2

        relay_l = Relay(sign=args.sign_left)
        relay_r = Relay(sign=args.sign_right)
        stop = threading.Event()
        source_error = None
        close_error = None
        restore_sigint = _route_sigint(stop)
        try:
            stats = run_bridge(
                lines, relay_l, relay_r, (_LOOPBACK, args.cmd_port), sock, log,
                duration_s=args.duration, echo_timeout_s=args.echo_timeout, stop_event=stop)
            # Snapshot is taken before the port is closed below; stats.closing is the real guard
            # against the reader's late failure.
            source_error = stats.source_error
        except KeyboardInterrupt:
            _log("interrupted during shutdown; no summary")
            return 0
        except OSError as exc:
            # Normally the CSV write (for example disk full). An OSError from the command socket
            # creation in _Run.__init__ also escapes run_bridge and lands here, so it is reported
            # with this same "cannot write --record" text.
            _log(f"cannot write --record {args.record!r}: {exc}")
            return 2
        finally:
            restore_sigint()
            try:
                log.close()
            except OSError as exc:  # flush on close can fail (for example disk full)
                close_error = exc
            finally:
                sock.close()
    finally:
        if ser is not None:
            _close_quietly(ser)
    _log(f"summary {_summary_text(stats)}")
    if close_error is not None:
        _log(f"cannot finish --record {args.record!r}: {close_error}")
        return 2
    return 2 if source_error is not None else 0


if __name__ == "__main__":
    raise SystemExit(main())
