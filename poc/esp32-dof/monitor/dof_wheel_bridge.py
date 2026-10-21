#!/usr/bin/env python3
"""IMU frames -> SerialImuHal -> Stage F control core -> UDP wheel display (issue #375).

Chain: ESP32 frame lines (serial, --port) or synthetic/CSV frames (--mock, --replay)
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

Usage:
    mamba run -n esp32-dof python3 monitor/dof_wheel_bridge.py --mock --duration 5
    mamba run -n esp32-dof python3 monitor/dof_wheel_bridge.py --port /dev/ttyACM0 --core pid
    mamba run -n esp32-dof python3 monitor/dof_wheel_bridge.py --replay raw.csv --record out.csv

Exit codes: 0 for input end, duration end, or Ctrl-C once the bridge has armed at least once
(including one that ends mid re-arm after a device reset; that case prints a "note:" line on
stderr); 2 for usage error, source error, never armed (not even once), or Ctrl-C before the
first arm.
"""

import argparse
import csv
import json
import math
import queue
import socket
import sys
import threading
import time
from collections import namedtuple
from pathlib import Path

_SIM_DIR = Path(__file__).resolve().parents[3] / "inverted-pendulum-project" / "07_Simulation"
if str(_SIM_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_DIR))

from hal.hal import HalFault  # noqa: E402
from hal.control_core import LqrBalance, PidBalance  # noqa: E402

from dof_frame import Frame, format_frame  # noqa: E402
from dof_hal import NOMINAL_DT_S, SerialImuHal  # noqa: E402
from dof_sources import csv_replay_lines, mock_frame  # noqa: E402

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


# ----- bridge loop ----------------------------------------------------------------------------

def run_bridge(hal, core, pub, *, duration_s=None, record=None,
               clock=time.monotonic, out=None, sleep=time.sleep, progress=None) -> BridgeResult:
    """Run the tick loop until the HAL reports end of input (-1) or duration_s elapses.

    ticks counts every loop tick (armed or not). Unarmed ticks are not published, not recorded
    and not faults. Armed HalFault ticks hold (see module docstring). published counts sends the
    publisher actually accepted (pub.sent delta); record logs every armed tick, published or not.
    core None (--core none): wheel_angle = tared pitch (mirror), HAL command 0.
    ever_armed becomes True on the first armed tick and is never cleared. progress, if given, is
    a dict that gets progress["ever_armed"] = True at the same moment, so a caller can still read
    it after KeyboardInterrupt (which skips the return value).
    """
    out = sys.stderr if out is None else out
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
    src.add_argument("--port", metavar="PATH", help="serial device, e.g. /dev/ttyACM0 (needs pyserial)")
    src.add_argument("--mock", action="store_true", help="synthetic 50 Hz frames with a still lead-in")
    src.add_argument("--replay", metavar="CSV", help="raw-frame CSV (seq,t_us,ax,ay,az,gx,gy,gz)")
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
    return p


def _make_source(args):
    if args.mock:
        return still_then_mock_lines(realtime=True)   # --duration is enforced by run_bridge
    if args.replay is not None:
        return csv_replay_lines(args.replay, realtime=True)
    try:
        from dof_serial import serial_lines  # lazy: pyserial only for --port
    except ImportError as exc:
        raise ImportError(f"--port needs pyserial in this environment: {exc}") from None
    return PolledSource(serial_lines(args.port, args.baud), poll_s=NOMINAL_DT_S)


def main(argv=None, clock=time.monotonic, sleep=time.sleep) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        gains = load_gains() if args.core != "none" else None
        core, ref = build_core(args.core, gains)
    except (OSError, ValueError) as exc:
        print(f"dof_wheel_bridge: error: {exc}", file=sys.stderr, flush=True)
        return 2

    hal = pub = record_fh = None
    progress = {"ever_armed": False}
    try:
        if args.record is not None:
            record_fh = open(args.record, "w", newline="", encoding="utf-8")
        pub = WheelPublisher(port=args.udp_port)
        hal = SerialImuHal(_make_source(args), reference_pitch_rad=ref, alpha=args.alpha,
                           clock=clock)
        result = run_bridge(hal, core, pub, duration_s=args.duration, record=record_fh,
                            clock=clock, sleep=sleep, progress=progress)
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
        if record_fh is not None:
            record_fh.close()


if __name__ == "__main__":
    sys.exit(main())
