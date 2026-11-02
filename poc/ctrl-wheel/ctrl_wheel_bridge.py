#!/usr/bin/env python3
"""
ctrl-wheel bridge (issue FW #453, part 2): reads WHL lines from a serial source,
relays the wheel velocity to the headless ctrl_wheel stand-in over UDP and logs
one CSV row per WHL line plus one per silence episode (ctrl_wheel_log.COLUMNS).

Command line (FW #454): build_parser() and main(argv) -> int. Run from poc/ctrl-wheel:
    python3 -m ctrl_wheel_msg --step 0.5 --seconds 5 | python3 ctrl_wheel_bridge.py --stdin --record ac1.csv
    python3 ctrl_wheel_bridge.py --port rfc2217://localhost:4001 --record ac1.csv
Exactly one of --port / --stdin. --record is required; an existing file is refused
unless --overwrite. Exit codes: 0 run finished (EOF or Ctrl-C), 2 usage error, a failed
source (at open, or mid-run after the summary), a CSV write error or an echo socket error.
Ctrl-C ends the run through stop_event and prints the summary.
An unreachable URL with --duration returns 2 on purpose: nothing was ever opened, so the
run is a failed source, not a clean finish.

SIM ONLY. The bridge is a relay, not a control loop, and makes no balance claim.
The wheel sign is UNVERIFIED; it is flipped only through the Relay sign argument,
which the CLI exposes as --sign (the only sign flip in the CLI).
Latency is UNMEASURED: host_rtt_ms is wall time, not link latency.

Copied, not imported (owner decision: no import across directories). Source is
poc/esp32-dof/monitor/dof_serial.py in this repo:
- _MAX_BUF_BYTES, _READ_CHUNK, _RECONNECT_DELAY_S, _WAIT_HEARTBEAT_S,
  _URL_WAIT_CAP_S: lines 15-19
- DofSerialError: lines 22-23
- device-open error text (_open_first): lines 53-60
- _is_url: lines 63-64
- _open_url_waiting: lines 67-93
- _close_quietly: lines 96-100
- _log: lines 103-104, tag changed to [ctrl_wheel_bridge]
- read and sync loop of _serial_gen: lines 115-163, adapted: no reconnect loop
  (a read error raises DofSerialError), ascii decode with errors=replace
  (dof_serial uses utf-8), first fragment after open discarded as before.
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

from ctrl_wheel_log import CsvLog
from ctrl_wheel_msg import SEQ_MOD, WhlError, parse_line
from ctrl_wheel_relay import Relay
from ctrl_wheel_udp import (
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

    def __post_init__(self) -> None:
        # Not dataclass fields: the pinned field list has no send-error or source-error column.
        self.send_errors = 0
        self.source_error: str | None = None  # text of a line source error that ended the run


@dataclass
class _Pending:
    sent_t: float
    row: dict | None


def _log(msg: str) -> None:
    print(f"[ctrl_wheel_bridge] {msg}", file=sys.stderr, flush=True)


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
    # Returns an open port, or None when max_duration_s passes first;
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
        stats.source_error = f"{type(exc).__name__}: {exc}"
        _log(f"line source stopped: {exc}")
    finally:
        finished.set()


def _row(host_t_s: float, seq: int, wheel: float, stale: int, whl=None, missed=None) -> dict:
    row = {
        "host_t_s": host_t_s,
        "cmd_seq": seq,
        "cmd_rad_s": wheel,
        "stale": stale,
        "missed_whl": missed,
    }
    if whl is not None:
        row.update(
            whl_seq=whl.seq,
            whl_t_us=whl.t_us,
            whl_model_rad_s=whl.model_rad_s,
            whl_raw=whl.raw,
            whl_state=whl.state,
        )
    return row


class _Run:
    """State of one run_bridge call. Single-threaded use (main loop only)."""

    def __init__(self, relay, cmd_addr, echo_sock, log, clock, echo_timeout_s, stats):
        self.relay = relay
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
        self.last_cmd_stale: int | None = None
        self.stale_episode = False

    def _send(self, wheel_rad_s: float, stale: int, *, whl=None, missed=None, log_row=True) -> None:
        seq = self.next_cmd_seq
        self.next_cmd_seq += 1
        now = self.clock()
        wheel = clamp(wheel_rad_s)
        payload = encode_cmd(seq, wheel, stale)
        row = _row(now, seq, wheel, stale, whl, missed) if log_row else None
        self.pending[seq] = _Pending(now, row)
        self.last_cmd_stale = stale
        self.stale_episode = stale == 1
        try:
            self.cmd_tx.sendto(payload, self.cmd_addr)
        except OSError as exc:
            self.stats.send_errors += 1
            _log(f"command {seq} send failed: {exc}")
            return
        self.stats.cmds_sent += 1
        if stale == 1:
            self.stats.stale_cmds += 1

    def handle_line(self, line: str) -> None:
        try:
            whl = parse_line(line)
        except WhlError:
            self.stats.bad_lines += 1
            return
        self.stats.whl_lines += 1
        missed = 0
        if self.last_whl_seq is None:
            self.last_whl_seq = whl.seq
        else:
            gap = (whl.seq - self.last_whl_seq - 1) % SEQ_MOD
            if gap < _SEQ_HALF:  # forward: count the gap and move on
                missed = gap
                self.last_whl_seq = whl.seq
            # else a repeat or reorder: not a gap, and last_whl_seq keeps the highest seen
        self.stats.missed_whl_total += missed
        self.relay.on_whl(whl)
        out = self.relay.poll()
        self._send(out.wheel_rad_s, out.stale, whl=whl, missed=missed)

    def check_silence(self) -> None:
        # One stale command per episode when the relay goes stale without a new line.
        out = self.relay.poll()
        if out is not None and out.stale == 1 and not self.stale_episode:
            self._send(0.0, 1)

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
                    angle_rad=echo.angle_rad,
                    vel_rad_s=echo.vel_rad_s,
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
            self._send(0.0, 1, log_row=False)

    def settle(self, poll_s: float, sleep: Callable[[float], object]) -> None:
        while self.pending:
            self.drain_echoes()
            self.flush_timeouts()
            if self.pending:
                sleep(poll_s)


def run_bridge(
    lines: Iterable[str],
    relay,
    cmd_addr: tuple[str, int],
    echo_sock: socket.socket,
    log,
    *,
    duration_s: float | None = None,
    poll_s: float = 0.02,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], object] = time.sleep,
    echo_timeout_s: float = 0.5,
    stop_event: threading.Event | None = None,
) -> BridgeStats:
    """Relay WHL lines to the wheel over UDP and log one CSV row per WHL line (plus one per silence episode).

    One command per WHL line. A WHL line with state != 0 sends its own stale command
    and its own row: one per line, not one per episode. A stale command from silence
    (no new line) is sent once per silence episode.
    Rows are written on echo match or after echo_timeout_s with no echo. The run
    ends on duration_s, stop_event, or an exhausted line source (after draining).
    A line source that raises (other than KeyboardInterrupt) also ends the run; its
    error text is kept in BridgeStats.source_error.
    A final stale command is sent unless the last command was already stale. It is
    not logged as a CSV row, so cmds_sent can exceed the row count by one.
    UDP send errors are counted in BridgeStats.send_errors, not raised.
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
    run = _Run(relay, cmd_addr, echo_sock, log, clock, echo_timeout_s, stats)
    lines_q: queue.Queue = queue.Queue()
    finished = threading.Event()
    reader = threading.Thread(
        target=_read_lines, args=(lines, lines_q, finished, stats), name="ctrl_wheel_lines", daemon=True)
    reader.start()
    started = clock()
    try:
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
        try:
            run.finish()
            run.settle(poll_s, sleep)
        finally:
            run.cmd_tx.close()
    return stats


# ---------------------------------------------------------------------------
# Command line (FW #454). SIM ONLY. The sign flip is only --sign, through Relay.
# ---------------------------------------------------------------------------

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
            "Relay WHL lines from a serial port or stdin to the ctrl_wheel stand-in over UDP "
            "and log one CSV row per WHL line (plus one per silence episode). SIM ONLY. Sign UNVERIFIED, latency UNMEASURED, "
            "no balance claim. pyserial is imported at module load, so it is needed even "
            "for --stdin."),
        epilog=(
            "examples:\n"
            "  python3 -m ctrl_wheel_msg --step 0.5 --seconds 5 | "
            "python3 ctrl_wheel_bridge.py --stdin --record ac1.csv\n"
            "  python3 ctrl_wheel_bridge.py --port rfc2217://localhost:4001 --record ac1.csv\n"
            "exit codes: 0 run finished (EOF or Ctrl-C); 2 usage error, failed source (at open or "
            "mid-run), CSV write error or echo socket error."),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument(
        "--port", metavar="PORT",
        help="serial device path or pyserial URL such as rfc2217://localhost:4001 "
             "(URL ports retry the first open for up to 300 s)")
    source.add_argument(
        "--stdin", action="store_true",
        help="read WHL lines from stdin until EOF")
    parser.add_argument(
        "--record", metavar="CSV", required=True,
        help="CSV log, one row per command (ctrl_wheel_log columns)")
    parser.add_argument(
        "--overwrite", action="store_true",
        help="replace an existing --record file (default: refuse and leave it untouched)")
    parser.add_argument(
        "--sign", type=int, choices=(1, -1), default=1,
        help="wheel sign, 1 or -1 (default 1). This is the ONLY place the sign is flipped "
             "(Relay). UNVERIFIED.")
    parser.add_argument(
        "--duration", type=_positive_finite, default=None, metavar="SECONDS",
        help="stop after SECONDS (> 0); default: run until the source ends")
    parser.add_argument(
        "--cmd-port", type=_port_number, default=PORT_CMD, metavar="N",
        help=f"UDP port of the wheel stand-in for commands (default {PORT_CMD}); not 5006")
    parser.add_argument(
        "--echo-port", type=_port_number, default=PORT_ECHO, metavar="N",
        help=f"UDP port this bridge listens on for echoes (default {PORT_ECHO}); not 5006")
    parser.add_argument(
        "--baud", type=_baud, default=115200,
        help="serial baud rate (default 115200); not used by --stdin")
    parser.add_argument(
        "--echo-timeout", type=_echo_timeout, default=0.5, metavar="SECONDS",
        help=f"seconds to wait for an echo before the row is written without one, 0..{_ECHO_TIMEOUT_MAX_S:g} "
             f"(default 0.5)")
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

        relay = Relay(sign=args.sign)
        stop = threading.Event()
        source_error = None
        close_error = None
        restore_sigint = _route_sigint(stop)
        try:
            stats = run_bridge(
                lines, relay, (_LOOPBACK, args.cmd_port), sock, log,
                duration_s=args.duration, echo_timeout_s=args.echo_timeout, stop_event=stop)
            # Decide NOW, before the port is closed below: closing it makes the reader thread
            # fail its next read, and that late failure is a consequence of the shutdown.
            source_error = stats.source_error
        except KeyboardInterrupt:
            _log("interrupted during shutdown; no summary")
            return 0
        except OSError as exc:  # CSV write failure (for example disk full) from run_bridge
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
