#!/usr/bin/env python3
"""IMU frames -> SerialImuHal -> Stage F control core -> UDP wheel display (issue #375).

Chain: ESP32 frame lines (serial, --port; piped lines, --stdin) or synthetic/CSV frames (--mock, --replay)
-> dof_hal.SerialImuHal (arming, tare, complementary filter) -> hal.control_core
LqrBalance / PidBalance (or none) -> wheel command -> HAL wheel_angle integral -> one UDP JSON
datagram per armed tick to the servo-wheel-dof Webots display (127.0.0.1:5006 by default).

Display only: the real board never feels the simulated wheel. Nothing here claims the board
balances, and the wheel direction is unverified on hardware (human check #377).

UDP datagram (one per published tick, radians):
    {"seq": int, "roll": f, "pitch": f, "yaw": 0.0, "wheel_angle": f, "cmd_rad_s": f}
servo-wheel-dof's parser ignores seq and extra keys (cmd_rad_s). Only PITCH is tared: published
pitch is the HAL tared_pitch_rad (no LQR reference offset), so the start pose displays upright in
pitch. Roll is published UNTARED (the display ignores roll). yaw is always 0.0.
wheel_angle is the integral of the COMMANDED wheel velocity, not a measurement. cmd_rad_s is the
core's output before the HAL clamps it to +-1 rad/s (the HAL clamps only the integral).
--core none: the wheel_angle field and the record wheel_angle column carry the tared pitch, and
the HAL integral is not used.

Arming: ticks before the HAL is armed (the arm window, failed "moving" attempts, and a re-arm
after a device reset) are not published, not recorded and not counted as faults. Every fresh arm
(first arm, or re-arm after a device reset) resets the core, so its integrator starts clean.
--duration includes this arm window (about 1 s of device time), so a duration under about 1 s ends
before arming and exits 2.

Faults: once armed, a HalFault (stale frame gap or wall-stale) holds: write 0, core.reset(), the
last good roll/pitch are published with cmd 0 and the wheel angle unchanged.

Poll ticks: a source that yields None (serial stall) returns NOMINAL_DT_S without a new frame.
The bridge paces non-frame ticks to NOMINAL_DT_S of wall time, and --port polls at that same
cadence. The stall and hold contract is in README "Behaviour".

Firmware text lines (issue #387): every serial line not starting with "IMU," is a text line. By
default the bridge ignores them. --text-log PATH saves them (timestamped, fresh file per run) and
--echo-text prints them to stderr with a "[fw] " prefix; both are off by default and need --port or --stdin.

Wheel command out (issue #435): --cmd-out PATH (needs --stdin) writes one 'CMD <rad_s>' line per
armed tick, scaled by --cmd-scale (use -1 to flip the sign); off by default. A fault tick writes
0. The value is the controller output BEFORE the HAL clamp, so the receiver must clamp it. A
non-finite value (including an overflowing product) is written as 0.0. Every line is flushed. A
FIFO open blocks until a reader opens it, and --duration does not bound that wait. A reader that
stops reading eventually blocks the tick loop (the UDP publish and the record stop too). With '-',
standard output carries only CMD lines; all status goes to stderr. '-' is meant for debugging; if
the reader of standard output goes away early, the interpreter may print an 'Exception ignored ...
BrokenPipeError' line at exit and the process exit status can be 120; use a FIFO or a file for
pipes that may close early. After a vanished reader (EPIPE) writing stops with one stderr line
and the run continues; any other OS error ends the run with exit 2. No final zero line is written
on end of input, duration or Ctrl-C, so the receiver must treat silence (a stale command) as stop.
SIM-ONLY plumbing: the firmware reader is a separate change.

Usage:
    mamba run -n esp32-dof python3 monitor/dof_wheel_bridge.py --mock --duration 5
    mamba run -n esp32-dof python3 monitor/dof_wheel_bridge.py --port /dev/ttyACM0 --core pid
    mamba run -n esp32-dof python3 monitor/dof_wheel_bridge.py --replay raw.csv --record out.csv
    mamba run -n esp32-dof python3 monitor/dof_wheel_bridge.py --port /dev/ttyACM0 --text-log fw.txt --echo-text

Exit codes: 0 for input end, duration end, or Ctrl-C once the bridge has armed at least once
(including one that ends mid re-arm after a device reset; that case prints a "note:" line on
stderr); 2 for usage error, source error, never armed (not even once), or Ctrl-C before the
first arm. An unopenable --text-log path also exits 2 (one "--text-log:" error line). An OS error
writing --cmd-out other than EPIPE also exits 2.
"""

import argparse
import csv
import errno
import json
import math
import os
import queue
import socket
import sys
import threading
import time
from collections import namedtuple
from datetime import datetime
from pathlib import Path

_SIM_DIR = Path(__file__).resolve().parents[3] / "inverted-pendulum-project" / "07_Simulation"
if str(_SIM_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_DIR))

from hal.hal import HalFault  # noqa: E402
from hal.control_core import LqrBalance, PidBalance  # noqa: E402

from dof_frame import PREFIX, Frame, format_frame  # noqa: E402
from dof_hal import NOMINAL_DT_S, SerialImuHal  # noqa: E402
from dof_sources import csv_replay_lines, mock_frame, stdin_lines  # noqa: E402

GAINS_PATH = _SIM_DIR / "hal" / "parity_vectors.json"
HOST = "127.0.0.1"
DEFAULT_UDP_PORT = 5006
DEFAULT_ALPHA = 0.98
SEQ_MOD = 65536
T_US_MOD = 2 ** 32
CORES = ("lqr", "pid", "none")
RECORD_COLUMNS = ("seq", "t_us", "roll", "pitch", "yaw", "wheel_angle", "cmd_rad_s")
STATUS_PERIOD_S = 1.0
RECORD_FLUSH_EVERY = 50
IMU_LINE_PREFIX = PREFIX + ","    # serial lines with this prefix are frames, every other line is text
TEXT_LINE_MAX = 512               # sanitized output chars per text line
TEXT_ECHO_PREFIX = "[fw] "
_END = "end"
_ITEM = "item"
_ERR = "error"

# armed: HAL armed when the loop stopped. ended: "input" or "duration". ever_armed: armed at least
# once, never cleared (so a re-arm cut short by input or duration still shows True).
BridgeResult = namedtuple("BridgeResult",
                          "ticks published faults final_wheel_angle armed ended ever_armed")


# ----- gains and cores ------------------------------------------------------------------------

def _need(data, path, *keys):
    cur = data
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            raise ValueError(f"{path}: missing key {'.'.join(keys)}")
        cur = cur[k]
    return cur


def _num(value, where):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{where} must be a finite number, got {value!r}")
    return float(value)


def load_gains(path=None) -> dict:
    """Read LQR and PID gains from parity_vectors.json (stdlib json only).

    path defaults to the module-level GAINS_PATH, looked up at call time.

    Raises:
        ValueError: invalid JSON, missing key, or a non-finite / non-numeric value.
        OSError: the file cannot be read.
    """
    path = GAINS_PATH if path is None else path
    with open(path, encoding="utf-8") as fh:
        try:
            data = json.load(fh)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{path}: invalid JSON: {exc}") from None
    if not isinstance(data, dict):
        raise ValueError(f"{path}: top level must be a JSON object")
    k = _need(data, path, "lqr", "K")
    if not isinstance(k, list) or len(k) != 2:
        raise ValueError(f"{path}: lqr.K must be a list of length 2")
    return {
        "lqr": {
            "K": (_num(k[0], "lqr.K[0]"), _num(k[1], "lqr.K[1]")),
            "theta_ref_rad": _num(_need(data, path, "lqr", "theta_ref_rad"), "lqr.theta_ref_rad"),
            "out_limit": _num(_need(data, path, "lqr", "out_limit"), "lqr.out_limit"),
        },
        "pid": {
            "kp": _num(_need(data, path, "pid", "kp"), "pid.kp"),
            "ki": _num(_need(data, path, "pid", "ki"), "pid.ki"),
            "kd": _num(_need(data, path, "pid", "kd"), "pid.kd"),
            "out_limit": _num(_need(data, path, "pid", "out_limit"), "pid.out_limit"),
        },
    }


def build_core(name: str, gains):
    """Return (core or None, reference_pitch_rad). gains is load_gains() output (unused for none)."""
    if name == "lqr":
        g = gains["lqr"]
        core = LqrBalance(K=tuple(g["K"]), theta_ref_rad=g["theta_ref_rad"], out_limit=g["out_limit"])
        return core, g["theta_ref_rad"]
    if name == "pid":
        g = gains["pid"]
        core = PidBalance(kp=g["kp"], ki=g["ki"], kd=g["kd"], out_limit=g["out_limit"])
        return core, 0.0
    if name == "none":
        return None, 0.0
    raise ValueError(f"core must be one of {CORES}, got {name!r}")


# ----- UDP publisher --------------------------------------------------------------------------

def _finite_real(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


class WheelPublisher:
    """One UDP datagram per send(). Never raises on OSError (counted in errors).

    Counters: sent (successful sendto only), errors (swallowed OSError), skipped (refused values).
    """

    def __init__(self, host: str = HOST, port: int = DEFAULT_UDP_PORT):
        if not isinstance(host, str) or not host:
            raise ValueError(f"host must be a non-empty str, got {host!r}")
        if isinstance(port, bool) or not isinstance(port, int) or not (1 <= port <= 65535):
            raise ValueError(f"port must be int 1..65535, got {port!r}")
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sent = 0
        self.errors = 0
        self.skipped = 0

    def send(self, seq, roll, pitch, yaw, wheel_angle, cmd_rad_s=None) -> None:
        if self._sock is None:
            return
        if isinstance(seq, bool) or not isinstance(seq, int):
            self.skipped += 1
            return
        angles = [roll, pitch, yaw, wheel_angle]
        if cmd_rad_s is not None:
            angles.append(cmd_rad_s)
        if not all(_finite_real(a) for a in angles):
            self.skipped += 1
            return
        payload = {"seq": seq, "roll": float(roll), "pitch": float(pitch), "yaw": float(yaw),
                   "wheel_angle": float(wheel_angle)}
        if cmd_rad_s is not None:
            payload["cmd_rad_s"] = float(cmd_rad_s)
        try:
            self._sock.sendto(json.dumps(payload).encode("utf-8"), self._addr)
        except OSError:
            self.errors += 1
            return
        self.sent += 1

    def close(self) -> None:
        sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass


# ----- polled source --------------------------------------------------------------------------

class PolledSource:
    """Read an iterator on a daemon thread; __next__ returns None after poll_s with no line.

    An exception raised by the iterator is re-raised from __next__. End of the iterator is
    StopIteration (sticky). close() asks the reader thread to stop at its next put; the thread
    is a daemon and exits without joining.
    """

    def __init__(self, iterable, poll_s: float = 0.1, maxsize: int = 1024):
        if isinstance(poll_s, bool) or not isinstance(poll_s, (int, float)) \
                or not math.isfinite(poll_s) or poll_s <= 0:
            raise ValueError(f"poll_s must be finite > 0, got {poll_s!r}")
        self._it = iter(iterable)
        self._poll_s = float(poll_s)
        self._q = queue.Queue(maxsize=maxsize)
        self._stop = threading.Event()
        self._ended = False
        self._thread = threading.Thread(target=self._run, name="dof-serial-reader", daemon=True)
        self._thread.start()

    def _put(self, msg) -> bool:
        while not self._stop.is_set():
            try:
                self._q.put(msg, timeout=0.05)
                return True
            except queue.Full:
                continue
        return False

    def _run(self):
        try:
            for item in self._it:
                if not self._put((_ITEM, item)):
                    return
        except BaseException as exc:  # relayed to the consumer, which re-raises it
            self._put((_ERR, exc))
        self._put((_END, None))

    def __iter__(self):
        return self

    def __next__(self):
        if self._ended:
            raise StopIteration
        try:
            kind, val = self._q.get(timeout=self._poll_s)
        except queue.Empty:
            return None
        if kind == _ITEM:
            return val
        self._ended = True
        if kind == _ERR:
            raise val
        raise StopIteration

    def close(self) -> None:
        self._stop.set()


# ----- mock source with still lead-in ---------------------------------------------------------

def still_then_mock_lines(still_frames: int = 75, rate_hz: float = 50.0, realtime: bool = True,
                          duration_s=None):
    """Still lead-in (SerialImuHal arms on it), then mock motion with continuous seq and t_us.

    The first still_frames lines: accel (0,0,1), gyro (0,0,0), seq k, t_us round(k*1e6/rate).
    Following lines are mock_frame(j) with seq and t_us shifted by still_frames (no time jump).
    realtime=True paces on an absolute deadline. duration_s None is infinite.
    """
    if isinstance(still_frames, bool) or not isinstance(still_frames, int) or still_frames < 1:
        raise ValueError(f"still_frames must be int >= 1, got {still_frames!r}")
    if not _finite_real(rate_hz) or rate_hz <= 0:
        raise ValueError(f"rate_hz must be finite > 0, got {rate_hz!r}")
    if duration_s is not None and (not _finite_real(duration_s) or duration_s <= 0):
        raise ValueError(f"duration_s must be None or finite > 0, got {duration_s!r}")
    return _still_gen(still_frames, float(rate_hz), realtime, duration_s)


def _still_gen(still_frames, rate_hz, realtime, duration_s):
    n_total = None if duration_s is None else int(round(duration_s * rate_hz))
    start = time.monotonic()
    i = 0
    while n_total is None or i < n_total:
        if realtime:
            delay = start + i / rate_hz - time.monotonic()
            if delay > 0:
                time.sleep(delay)
        if i < still_frames:
            accel, gyro = (0.0, 0.0, 1.0), (0.0, 0.0, 0.0)
        else:
            m = mock_frame(i - still_frames, rate_hz)     # mock motion starts at j = 0 at the join
            accel, gyro = m.accel, m.gyro
        frame = Frame(seq=i % SEQ_MOD, t_us=round(i * 1_000_000 / rate_hz) % T_US_MOD,
                      accel=accel, gyro=gyro)
        yield format_frame(frame).rstrip("\r\n")
        i += 1


# ----- firmware text capture (#387) -----------------------------------------------------------

TEXT_TRUNCATED = "[truncated]"


def _escape_char(c: str) -> str:
    if c == "\\":
        return "\\\\"
    if c.isprintable() and c != "\ufffd":
        return c
    cp = ord(c)
    if cp <= 0xFF:
        return f"\\x{cp:02x}"
    if cp <= 0xFFFF:
        return f"\\u{cp:04x}"
    return f"\\U{cp:08x}"


def sanitize_text(s: str) -> str:
    """Make one serial text line safe to write or echo. Output is at most TEXT_LINE_MAX chars.

    A literal backslash is written as two backslashes, so text such as \\x41 cannot be read as an
    escape. Any other char that is not printable, and U+FFFD (a decode-error marker), is written
    as a fixed-width escape: \\xNN for code points up to 0xFF, \\uNNNN up to 0xFFFF, and
    \\UNNNNNNNN above that. A line longer than TEXT_LINE_MAX is cut and ends with "[truncated]",
    which counts inside the cap. The cut never splits an escape or a backslash pair.
    """
    pieces = []
    total = 0
    for c in s:
        piece = _escape_char(c)
        pieces.append(piece)
        total += len(piece)
        if total > TEXT_LINE_MAX:
            break
    if total <= TEXT_LINE_MAX:
        return "".join(pieces)
    budget = TEXT_LINE_MAX - len(TEXT_TRUNCATED)
    kept = []
    n = 0
    for piece in pieces:
        if n + len(piece) > budget:
            break
        kept.append(piece)
        n += len(piece)
    return "".join(kept) + TEXT_TRUNCATED


def _wall_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


class TextSink:
    """Firmware text lines -> optional file (path) and/or stderr (echo). Thread-safe.

    write() runs on the serial reader thread and never raises for any Exception. The file gets one
    row per write and is flushed after each row. Per-write flush is cheap at the rate of the BRC
    tilt_servo source, which prints [imu] every 100 ms (README Step D), and a tail line is never left
    in the buffer. The file is opened with mode "w", so it is fresh
    per run.

    The file and the echo are disabled independently. The first exception on the file path (open
    row write, flush, or the clock/timestamp calls) disables only the file capture. The first
    exception on the echo path (closed or broken stderr, or no stderr at all) disables only the
    echo. Each one prints at most one warning, best-effort. A sanitize_text() failure affects both
    paths, since neither can produce a line, so it prints one warning per path (two lines).

    Timestamps are clock() minus _t0, the clock() value taken at construction. main() builds the sink
    before WheelPublisher and SerialImuHal, and so before serial_lines() opens the port.

    close() is idempotent, and writes after close are ignored (no file write, no echo). The lock
    covers only the file write and the state change. The stderr echo runs outside the lock, so a
    stalled stderr cannot block close(). close() does not wait for an echo that is already past the
    lock, so that echo can run after close() returns. Accepted limit: a stalled stderr can still
    stall the reader thread.
    """

    def __init__(self, path=None, *, echo=False, clock=time.monotonic, wall_clock=None):
        self._clock = clock
        self._wall = _wall_iso if wall_clock is None else wall_clock
        self._echo = echo
        self._lock = threading.Lock()
        self._closed = False
        self._file_disabled = False
        self._echo_disabled = False
        self._t0 = clock()
        self._fh = None if path is None else open(path, "w", newline="", encoding="utf-8")

    @staticmethod
    def _warn(kind: str, exc) -> None:
        try:
            sys.stderr.write(f"dof_wheel_bridge: warning: firmware text {kind} disabled after "
                             f"error: {exc}\n")
            sys.stderr.flush()
        except Exception:
            pass

    def _echo_line(self, clean: str) -> None:
        try:
            sys.stderr.write(f"{TEXT_ECHO_PREFIX}{clean}\n")   # one write: no mid-line interleave
            sys.stderr.flush()
        except Exception as exc:
            with self._lock:
                first = not self._echo_disabled
                self._echo_disabled = True
            if first:
                self._warn("echo", exc)

    def write(self, line: str) -> None:
        file_warn = None
        echo_warn = None
        echo_text = None
        with self._lock:
            if self._closed:
                return
            file_wanted = self._fh is not None and not self._file_disabled
            echo_wanted = self._echo and not self._echo_disabled
            clean = None
            if file_wanted:
                try:
                    clean = sanitize_text(line)
                    self._fh.write(f"{self._clock() - self._t0:.3f}\t{self._wall()}\t{clean}\n")
                    self._fh.flush()
                except Exception as exc:
                    self._file_disabled = True
                    file_warn = exc
            if echo_wanted:
                try:
                    if clean is None:
                        clean = sanitize_text(line)
                    echo_text = clean
                except Exception as exc:
                    self._echo_disabled = True
                    echo_warn = exc
        if file_warn is not None:
            self._warn("file", file_warn)
        if echo_warn is not None:
            self._warn("echo", echo_warn)
        if echo_text is not None:
            self._echo_line(echo_text)

    def close(self) -> None:
        file_warn = None
        with self._lock:
            if self._closed:
                return
            self._closed = True
            fh, self._fh = self._fh, None
            if fh is not None:
                try:
                    fh.close()
                except Exception as exc:
                    if not self._file_disabled:
                        self._file_disabled = True
                        file_warn = exc
        if file_warn is not None:
            self._warn("file", file_warn)


def _tee_text(lines, sink):
    """Yield every line unchanged and in order; pass non-IMU lines to sink.write first.

    Runs on the reader thread. Only used when --text-log or --echo-text is given.
    """
    for line in lines:
        if not line.startswith(IMU_LINE_PREFIX):
            sink.write(line)
        yield line


def _same_file(first: str, second: str) -> bool:
    """True when both names are the same file: equal realpaths (symlinks followed), or, when both
    exist, the same inode (hard links). os.path.realpath does not raise on a symlink loop, unlike
    Path.resolve() on Python 3.11. Any error (bad path, loop, permission) gives False, and the
    later open reports the real problem.
    """
    try:
        if os.path.realpath(first) == os.path.realpath(second):
            return True
        if os.path.exists(first) and os.path.exists(second):
            return os.path.samefile(first, second)
        return False
    except Exception:
        return False


# ----- bridge loop ----------------------------------------------------------------------------

def _write_cmd(fh, value) -> bool:
    """Write one 'CMD <rad_s>' line and flush. False when the reader is gone (EPIPE).

    Any other OSError propagates; main reports it and exits 2.
    """
    try:
        fh.write(f"CMD {value:.4f}\n")
        fh.flush()
    except BrokenPipeError:
        return False
    except OSError as exc:
        if exc.errno == errno.EPIPE:
            return False
        raise
    return True


def run_bridge(hal, core, pub, *, duration_s=None, record=None,
               clock=time.monotonic, out=None, sleep=time.sleep, progress=None,
               cmd_out=None, cmd_scale=1.0) -> BridgeResult:
    """Run the tick loop until the HAL reports end of input (-1) or duration_s elapses.

    ticks counts every loop tick (armed or not). Unarmed ticks are not published, not recorded
    and not faults. Armed HalFault ticks hold (see module docstring). published counts sends the
    publisher actually accepted (pub.sent delta); record logs every armed tick, published or not.
    core None (--core none): wheel_angle = tared pitch (mirror), HAL command 0.
    ever_armed becomes True on the first armed tick and is never cleared. progress, if given, is
    a dict that gets progress["ever_armed"] = True at the same moment, so a caller can still read
    it after KeyboardInterrupt (which skips the return value).
    cmd_out, if given, gets one 'CMD <rad_s>' line per armed tick: cmd * cmd_scale, flushed (cmd is
    0.0 on a fault tick). After the reader is gone (EPIPE) writing stops for the rest of the run
    with one stderr line; the run itself continues.
    """
    out = sys.stderr if out is None else out
    cmd_fh = cmd_out
    writer = None
    if record is not None:
        writer = csv.writer(record)
        writer.writerow(RECORD_COLUMNS)
    ticks = published = faults = rows = 0
    seq = 0
    last_roll = last_pitch = 0.0
    final_wheel = 0.0
    start = clock()
    last_tick = start
    last_status = None
    prev_seq = hal.last_seq
    was_armed = False
    ever_armed = False
    last_rearm = None
    ended = "input"
    try:
        while True:
            dt = hal.wait_next_tick()
            if dt < 0:
                break
            now = clock()
            if duration_s is not None and now - start >= duration_s:
                ended = "duration"
                break
            ticks += 1
            new_frame = hal.last_seq != prev_seq
            prev_seq = hal.last_seq
            if not new_frame:
                idle = NOMINAL_DT_S - (now - last_tick)
                if idle > 0:
                    sleep(idle)
            last_tick = clock()

            if not hal.armed:
                was_armed = False
                last_status = _status(out, clock(), last_status, hal, ticks, faults, 0.0,
                                      hal.wheel_angle_rad, "arming")
                continue
            # Fresh arm: the first arm, or a device reset re-arm. A re-arm can finish inside one
            # wait_next_tick() call, so detect it from hal.rearm_count, not from armed going False.
            if not was_armed or hal.rearm_count != last_rearm:
                if core is not None:
                    core.reset()        # clean core state (integrator, previous error)
            was_armed = True
            ever_armed = True
            if progress is not None:
                progress["ever_armed"] = True
            last_rearm = hal.rearm_count

            try:
                imu = hal.read_imu()
            except HalFault as exc:
                faults += 1
                hal.write_wheel_velocity(0.0, 0.0)
                if core is not None:
                    core.reset()
                roll, pitch = last_roll, last_pitch
                cmd = 0.0
                wheel = last_pitch if core is None else hal.wheel_angle_rad
                last_status = _status(out, clock(), last_status, hal, ticks, faults, cmd,
                                      wheel, f"hold: {exc}")
            else:
                last_roll = imu.roll_rad
                last_pitch = hal.tared_pitch_rad
                roll, pitch = last_roll, last_pitch
                if core is None:
                    cmd = 0.0
                    hal.write_wheel_velocity(0.0, 0.0)
                    wheel = pitch
                else:
                    o = core.step(imu, dt)
                    cmd = 0.0 if o.fault else float(o.cmd_rad_s)
                    hal.write_wheel_velocity(cmd, cmd)
                    wheel = hal.wheel_angle_rad
                last_status = _status(out, clock(), last_status, hal, ticks, faults, cmd,
                                      wheel, None)

            sent_before = pub.sent
            pub.send(seq, roll, pitch, 0.0, wheel, cmd_rad_s=cmd)
            if pub.sent > sent_before:
                published += 1
            if cmd_fh is not None:
                v = cmd * cmd_scale
                if not math.isfinite(v):     # NaN, inf or an overflowing product never reaches the wire
                    v = 0.0
                v = round(v, 4) + 0.0        # -0.0 and tiny negatives (-1e-5) go out as 0.0000, not -0.0000
                if not _write_cmd(cmd_fh, v):
                    cmd_fh = None
                    print("dof_wheel_bridge: cmd-out closed, no longer writing", file=sys.stderr,
                          flush=True)
            if writer is not None:
                writer.writerow([seq, int(hal.now_s() * 1e6), roll, pitch, 0.0, wheel, cmd])
                rows += 1
                if rows % RECORD_FLUSH_EVERY == 0:
                    record.flush()
            seq = (seq + 1) % SEQ_MOD
            final_wheel = wheel
    finally:
        if record is not None:
            record.flush()
    return BridgeResult(ticks, published, faults, final_wheel, hal.armed, ended, ever_armed)


def _status(out, now, last, hal, ticks, faults, cmd, wheel, note):
    if last is not None and now - last < STATUS_PERIOD_S:
        return last
    tail = f" note={note}" if note else ""
    arm = "armed" if hal.armed else f"not armed (arm_error={hal.arm_error})"
    print(f"dof_wheel_bridge: t={hal.now_s():.2f}s {arm} ticks={ticks} faults={faults} "
          f"cmd={cmd:+.4f} wheel={wheel:+.4f}{tail}", file=out, flush=True)
    return now


# ----- CLI ------------------------------------------------------------------------------------

def _positive_float(text: str) -> float:
    try:
        v = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not a number: {text!r}")
    if not (math.isfinite(v) and v > 0):
        raise argparse.ArgumentTypeError(f"must be finite > 0, got {text!r}")
    return v


def _positive_int(text: str) -> int:
    try:
        v = int(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not an integer: {text!r}")
    if v < 1:
        raise argparse.ArgumentTypeError(f"must be >= 1, got {text!r}")
    return v


def _unit_float(text: str) -> float:
    try:
        v = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not a number: {text!r}")
    if not (math.isfinite(v) and 0.0 <= v <= 1.0):
        raise argparse.ArgumentTypeError(f"must be in [0, 1], got {text!r}")
    return v


def _finite_float(text: str) -> float:
    try:
        v = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not a number: {text!r}")
    if not math.isfinite(v):
        raise argparse.ArgumentTypeError(f"must be finite, got {text!r}")
    return v


def _port(text: str) -> int:
    try:
        v = int(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not an integer: {text!r}")
    if not (1 <= v <= 65535):
        raise argparse.ArgumentTypeError(f"must be 1..65535, got {text!r}")
    return v


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="dof_wheel_bridge",
                                description="IMU frames -> SerialImuHal -> LQR/PID -> UDP wheel display.")
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", metavar="PATH", help="serial device (e.g. /dev/ttyACM0) or pyserial URL (e.g. rfc2217://localhost:4000); a URL port waits up to 300 s for the port to open, ended earlier by --duration or Ctrl-C (needs pyserial)")
    src.add_argument("--mock", action="store_true", help="synthetic 50 Hz frames with a still lead-in")
    src.add_argument("--replay", metavar="CSV", help="raw-frame CSV (seq,t_us,ax,ay,az,gx,gy,gz)")
    src.add_argument("--stdin", action="store_true",
                     help="read frame lines from standard input, for example piped from wokwi-cli")
    p.add_argument("--baud", type=_positive_int, default=115200, help="serial baud (default 115200)")
    p.add_argument("--core", default="lqr", choices=CORES,
                   help="control core: lqr (default), pid, or none (display mirror of pitch)")
    p.add_argument("--udp-port", type=_port, default=DEFAULT_UDP_PORT, metavar="PORT",
                   help="UDP port of the servo-wheel-dof display (default 5006)")
    p.add_argument("--record", metavar="CSV",
                   help="write per-tick CSV seq,t_us,roll,pitch,yaw,wheel_angle,cmd_rad_s (armed ticks)")
    p.add_argument("--duration", type=_positive_float, default=None, metavar="SECONDS",
                   help="stop after SECONDS, counted from start including the ~1 s arm window; "
                        "under ~1 s exits 2 (duration expired before the arm window filled) "
                        "(default: until Ctrl-C / end of input)")
    p.add_argument("--alpha", type=_unit_float, default=DEFAULT_ALPHA, metavar="A",
                   help="complementary filter gyro weight 0..1 (default 0.98)")
    p.add_argument("--text-log", metavar="PATH", default=None,
                   help="save firmware text lines (lines not starting with 'IMU,') with timestamps "
                        "to PATH, a fresh file per run (needs --port or --stdin; default off; escaping and "
                        "[truncated] are in README Step D)")
    p.add_argument("--echo-text", action="store_true",
                   help="print firmware text lines to stderr with a '[fw] ' prefix "
                        "(needs --port or --stdin; default off; escaping and [truncated] are in README Step D)")
    p.add_argument("--cmd-out", metavar="PATH", default=None,
                   help="write the wheel command as 'CMD <rad_s>' lines to PATH (a FIFO or a file; '-' is "
                        "standard output, which then carries only CMD lines); needs --stdin. Opening a "
                        "FIFO waits for a reader, and --duration does not bound that wait. Every line is "
                        "flushed; a reader that stops reading blocks the loop. The value is before the HAL "
                        "clamp, so the receiver must clamp it; no final zero is written, so silence means "
                        "stop. SIM-ONLY (the firmware reader is a separate change). '-' is meant for "
                        "debugging; if the reader of standard output goes away early, the interpreter may "
                        "print an 'Exception ignored ... BrokenPipeError' line at exit and the process exit "
                        "status can be 120; use a FIFO or a file for pipes that may close early")
    p.add_argument("--cmd-scale", type=_finite_float, default=1.0, metavar="X",
                   help="factor applied to the command before writing it (use -1 to flip the sign; "
                        "a non-finite result is written as 0.0; the PD versus LQR sign is UNVERIFIED, "
                        "freecad-workspace#359)")
    return p


def _make_source(args, text_sink=None):
    if args.mock:
        return still_then_mock_lines(realtime=True)   # --duration is enforced by run_bridge
    if args.replay is not None:
        return csv_replay_lines(args.replay, realtime=True)
    if args.stdin:                           # no pyserial, baud or reconnect: plain text lines
        if sys.stdin is None:                    # fd 0 closed at start
            raise OSError("stdin is closed")
        if hasattr(sys.stdin, "reconfigure"):    # like the serial path: utf-8, bad bytes replaced, only \n ends a line
            sys.stdin.reconfigure(encoding="utf-8", errors="replace", newline="\n")
        lines = stdin_lines(sys.stdin)
        if text_sink is not None:
            lines = _tee_text(lines, text_sink)
        return PolledSource(lines, poll_s=NOMINAL_DT_S)
    try:
        from dof_serial import serial_lines  # lazy: pyserial only for --port
    except ImportError as exc:
        raise ImportError(f"--port needs pyserial in this environment: {exc}") from None
    lines = serial_lines(args.port, args.baud)
    if text_sink is not None:                # no flags: the iterator is passed through unwrapped
        lines = _tee_text(lines, text_sink)
    return PolledSource(lines, poll_s=NOMINAL_DT_S)


def main(argv=None, clock=time.monotonic, sleep=time.sleep) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    want_text = args.text_log is not None or args.echo_text
    if want_text and args.port is None and not args.stdin:
        parser.error("--text-log and --echo-text need --port or --stdin (not --mock or --replay)")
    if args.text_log is not None and args.record is not None \
            and _same_file(args.text_log, args.record):
        parser.error("--text-log and --record must not be the same file")
    if args.cmd_out not in (None, "-"):      # "-" is standard output, not a file
        if args.record is not None and _same_file(args.cmd_out, args.record):
            parser.error("--cmd-out and --record must not be the same file")
        if args.text_log is not None and _same_file(args.cmd_out, args.text_log):
            parser.error("--cmd-out and --text-log must not be the same file")
    if args.cmd_out is not None and not args.stdin:
        parser.error("--cmd-out needs --stdin")
    try:
        gains = load_gains() if args.core != "none" else None
        core, ref = build_core(args.core, gains)
    except (OSError, ValueError) as exc:
        print(f"dof_wheel_bridge: error: {exc}", file=sys.stderr, flush=True)
        return 2

    hal = pub = record_fh = text_sink = cmd_fh = None
    progress = {"ever_armed": False}
    try:
        if want_text:
            try:
                text_sink = TextSink(args.text_log, echo=args.echo_text, clock=clock)
            except (OSError, ValueError) as exc:     # ValueError: e.g. a NUL byte in the path
                print(f"dof_wheel_bridge: error: --text-log: cannot open {args.text_log!r}: "
                      f"{getattr(exc, 'strerror', None) or exc}", file=sys.stderr, flush=True)
                return 2
        # Open order: text sink, then --cmd-out, then --record. A bad --text-log or --cmd-out path
        # exits 2 before an existing --record file is truncated. A bad --record path comes after
        # --cmd-out, so the cmd file has already been created or truncated when it exits 2.
        if args.cmd_out is not None:
            # "-" is standard output: main never closes it (see finally).
            cmd_fh = sys.stdout if args.cmd_out == "-" else open(args.cmd_out, "w", encoding="utf-8")
        if args.record is not None:
            record_fh = open(args.record, "w", newline="", encoding="utf-8")
        pub = WheelPublisher(port=args.udp_port)
        hal = SerialImuHal(_make_source(args, text_sink=text_sink), reference_pitch_rad=ref,
                           alpha=args.alpha, clock=clock)
        result = run_bridge(hal, core, pub, duration_s=args.duration, record=record_fh,
                            clock=clock, sleep=sleep, progress=progress,
                            cmd_out=cmd_fh, cmd_scale=args.cmd_scale)
        if pub.errors or pub.skipped:
            print(f"dof_wheel_bridge: udp published={result.published} errors={pub.errors} "
                  f"skipped={pub.skipped}", file=sys.stderr, flush=True)
        if hal.source_error is not None:
            print(f"dof_wheel_bridge: error: source failed: {hal.source_error}",
                  file=sys.stderr, flush=True)
            return 2
        if not result.ever_armed or (result.ended == "duration" and result.published == 0):
            if result.ended == "duration" and not result.ever_armed:
                why = (f"duration expired before the arm window filled "
                       f"({hal.arm_error or 'no arm error yet'})")
            elif result.ended == "duration":
                why = "duration expired before the first armed tick (the arm window takes ~1 s)"
            else:
                why = f"never armed: {hal.arm_error or 'no arm error'}"
            print(f"dof_wheel_bridge: error: {why} (source_error={hal.source_error})",
                  file=sys.stderr, flush=True)
            return 2
        if not result.armed:
            # Armed earlier, then a device reset started a re-arm that input or duration ended.
            print(f"dof_wheel_bridge: note: {result.ended} ended during a re-arm "
                  f"({hal.arm_error or 'no arm error'}); the bridge had armed earlier and "
                  f"published {result.published} ticks", file=sys.stderr, flush=True)
        return 0
    except KeyboardInterrupt:
        armed = hal is not None and hal.armed
        if armed:
            return 0
        if progress["ever_armed"]:
            print("dof_wheel_bridge: note: interrupted during a re-arm; the bridge had armed "
                  "earlier", file=sys.stderr, flush=True)
            return 0
        print("dof_wheel_bridge: error: interrupted before armed", file=sys.stderr, flush=True)
        return 2
    except (ImportError, OSError, ValueError) as exc:
        print(f"dof_wheel_bridge: error: {exc}", file=sys.stderr, flush=True)
        return 2
    finally:
        if hal is not None:
            hal.close()
        if pub is not None:
            pub.close()
        try:
            if record_fh is not None:
                record_fh.close()
        except OSError as exc:
            print(f"dof_wheel_bridge: warning: closing --record failed: {exc}",
                  file=sys.stderr, flush=True)
        finally:
            try:
                if cmd_fh is not None and cmd_fh is not sys.stdout:   # never close standard output
                    cmd_fh.close()
            except OSError as exc:
                # EPIPE: the reader already went and was reported by run_bridge. The close re-flushes
                # the pending buffer, so it fails again here; stay silent so the user sees one line.
                if not isinstance(exc, BrokenPipeError) and exc.errno != errno.EPIPE:
                    print(f"dof_wheel_bridge: warning: closing --cmd-out failed: {exc}",
                          file=sys.stderr, flush=True)
            finally:
                if text_sink is not None:      # always closed, even if the record close failed
                    text_sink.close()


if __name__ == "__main__":
    sys.exit(main())
