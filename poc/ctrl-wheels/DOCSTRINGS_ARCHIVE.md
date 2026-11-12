# Docstrings archive (ctrl-wheels)
Full text of the multi-line docstrings replaced by one-liners (issue #565). Each block is the docstring value byte-for-byte. Line numbers are as of the commit before this change (the first line of the docstring in the old file). `webots/controllers/ctrl_wheels/sensor_read.py` is a hand-synced copy of a servo-wheel-dof file: after this change its docstrings differ from that source, which was not touched.

## `ctrl_wheels_bridge.py`

### `module` (line 2)

```

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
(ii) a restart is read as a reorder only when the new seq is 0 to 32767 behind the baseline
(mod 65536; 0 behind is a repeat) AND its first t_us is within 1 s BELOW the old t_us (a restart from a high old seq,
where the new seq looks like a forward wrap, or with a t_us above the old one, is a restart);
(iii) a restart that is read as a reorder (ii) is only noticed when t_us passes the old
baseline or the new seq passes the old baseline, whichever comes first, up to 1 s of lines
later, and those lines count missed 0;
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
With the bridge and Webots signs at their defaults +1/+1, both wheels rolled toward +x in an
owner-observed Webots run (2026-11-06, sim only); the chip-side signs are UNCONFIRMED.
The signs are flipped only through the Relay sign argument,
which the CLI exposes as --sign-left and --sign-right (the only sign flips in the CLI).
Latency is UNMEASURED: host_rtt_ms is wall time, not link latency.

The serial helpers (DofSerialError, _is_url, _open_first, _open_url_waiting,
_close_quietly and the read and sync loop) are copied, not imported, from
poc/esp32-dof/monitor/dof_serial.py in this repo, with no reconnect loop.

```

### `open_serial_lines` (line 241)

```
Yield stripped non-empty lines from a serial port or pyserial URL.

    Device paths raise DofSerialError at the first next(). URL ports (containing
    "://") retry the first open every 1 s for up to 300 s or max_duration_s.
    Test-covered helper, not on the CLI path (main() does not call it).
    
```

### `run_bridge` (line 448)

```
Relay WHL2 lines to the wheels over UDP and log one CSV row per WHL2 line (plus one per silence episode).

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
    
```

## `ctrl_wheels_log.py`

### `module` (line 1)

```

CSV log writer for the ctrl-wheels twin-wheel proof of concept (freecad-workspace
#494, task F10, under umbrella FW #482). Contract: poc/ctrl-wheels/CONTRACT.md, S6.

SIM ONLY. Not hardware-validated. Chip-side signs UNCONFIRMED; the
bridge/Webots default +1/+1 net direction was owner-observed 2026-11-06
(sim only). Latency UNMEASURED.

One row per host tick, 19 columns. Columns are fixed by COLUMNS and written in
that order, whatever the key order of the input mapping. A row is fully
validated before anything is written, so a rejected row leaves no partial line.

whl_raw_l and whl_raw_r are the raw goal velocities the planned firmware writes
to each chip, taken from the WHL2 line. They are not a servo readback. This log
does not range-check whl_raw_l or whl_raw_r to int32: it accepts any Python int
in an int column. The int32 range is enforced by the WHL2 parser,
ctrl_wheels_msg.parse_line (CONTRACT.md S1); this log does not repeat that check.

Cell rules:
- None or a missing key gives an empty cell.
- bool, NaN and inf are refused with ValueError.
- Float columns use "%.6f", except host_t_s and host_rtt_ms which use "%.3f".
- Integer columns are written as decimal integers.

host_rtt_ms: wall-clock milliseconds from sending the command with cmd_seq to
receiving the echo whose cmd_seq_echo equals that cmd_seq. It includes up to one
simulation step (16 ms in the world file and in fake_wheels, UNVERIFIED for Webots)
and one bridge poll, so it is NOT a pure link latency. Empty when no echo has
arrived.

echo_cmd_seq: the value of the echo JSON key cmd_seq_echo.

```

### `CsvLog` (line 130)

```
CSV writer for ctrl-wheels rows. The header is written on construction.

    A str or os.PathLike target is opened and owned: it is closed by close().
    With overwrite=False an existing path raises FileExistsError.
    Any other target is a caller-owned file object: it is flushed, never closed.
    
```

## `ctrl_wheels_msg.py`

### `module` (line 2)

```

WHL2 twin-wheel line protocol (ctrl_wheels_msg) for the ctrl-wheels POC:
formatter and parser for the two-wheel line, the single-wheel Whl type it
splits into, and a step-input generator (step_lines) that yields WHL2 lines via
format_whl2 (Whl and format_whl are kept for the bridge tests).

SIM ONLY. This is a sim-only twin. Nothing here is validated on real hardware
or against a real receiver. The sign conventions are UNCONFIRMED: the Webots net
direction at +1/+1 is owner-observed (2026-11-06, sim only), the chip-side
rawR negation is UNCONFIRMED (the planned firmware writes rawR already
negated), and "forward-positive" for model is the contract's definition
only (CONTRACT.md S2 in this directory).

Line format of the twin (WHL2, ASCII, one line per sample, CONTRACT.md S1):
    WHL2,<seq>,<t_us>,<model %.6f>,<rawL>,<rawR>,<state>*<HH>\n

Protocol details (WHL2):
- seq: uint16 sequence number, wraps at 65536 (format takes seq mod 65536);
  the parser rejects 65536 or more
- t_us: uint32 device microseconds (0 .. 2^32-1)
- model: model plant velocity in rad/s, %.6f. The formatter refuses a
  non-finite value (WhlError). The parser rejects any non-finite model token
  (nan, inf, -inf, an overflowing exponent such as 1e400).
  The formatter writes 0.000000 for a -0 model; the parser accepts -0.000000
- rawL, rawR: signed decimal integers, int32 range. The wire accepts any int32
  value; the +-125 cap (RAW_CAP) is a firmware step cap and is applied only by
  step_lines, not by the wire
- state: 0 run, 1 hold after a fall, 2 bus down; one state for the pair
- HH: 2-digit UPPERCASE hex XOR checksum of the body (from the leading W up to
  the *). The checksum is compared before the field count, prefix and tokens
- Line terminator: \n on output; the parser accepts exactly one trailing \n,
  \r\n, or no terminator

Supported range: every token is at most 20 characters (_MAX_TOKEN_LEN), and
the formatter refuses a model whose %.6f text is longer than that. So the
formatter and parser round-trip a model whose %.6f text is at most 20
characters: positive values up to 9999999999999.999999 and negative values down
to -999999999999.999999. A value whose text is longer (including one that
rounds up to a longer text) is refused by the formatter and by the parser.
This is a simulation wheel velocity in rad/s, far inside the range. The v1
formatter (format_whl) shares the guard; the single-wheel module in
poc/ctrl-wheel has no 20-character model-text guard.

Single-wheel names kept for the split: Whl (seq, t_us, model_rad_s, raw, state)
and format_whl, which writes the single-wheel WHL v1 line (prefix WHL, 6 fields).
Both stay because split() returns Whl and the bridge tests still use format_whl.
step_lines (the --step CLI) yields WHL2 lines through format_whl2.
Its left and right raws are the clamped raw with the sign applied: sign_l and
sign_r, default +1 and -1, SIM ONLY, UNCONFIRMED. parse_line accepts WHL2 only.

Usage (run from poc/ctrl-wheels):
    python3 -m ctrl_wheels_msg --step 1.0 --seconds 5
    python3 -m ctrl_wheels_msg --step 1.0 --seconds 5 --no-sleep
    python3 -m ctrl_wheels_msg --step 1.0 --seconds 5 --sign-left 1 --sign-right -1 --state 0

```

### `_model_text` (line 184)

```
Format a finite model with %.6f, normalising -0.000000 to 0.000000.

    Refuses a text longer than _MAX_TOKEN_LEN, because the parser would refuse
    that token. The message names the length, not the value.
    
```

### `parse_line` (line 251)

```

    Parse one WHL2 line. Accepts exactly one trailing \n, one trailing \r\n,
    or no terminator. Any other trailing characters (a doubled terminator or a
    lone \r) stay in the line and are refused by the checks below.

    The checksum is compared first, then the field count (7), the prefix
    (WHL2), then each token. A WHL v1 line (6 fields) is refused by the field
    count. Integer tokens are checked against an explicit pattern before int()
    is called, so "1_0" and "+" alone are refused. A leading "+" on raw and
    model is inherited from the single-wheel parser and is not decided
    (CONTRACT.md S1, S8). Float tokens are checked for
    whitespace and underscores, then float(), and must be finite. Every token
    is refused above 20 characters before any conversion. raw_l and raw_r must
    fit int32.

    Raises:
        WhlError: on any malformed, mis-checksummed or out-of-range line.
    
```

### `split` (line 326)

```
Split a twin sample into (left, right) single-wheel samples.

    seq, t_us, model_rad_s and state are shared. The left Whl takes raw_l and
    the right Whl takes raw_r, both as written (no clamp, no sign change).
    
```

### `step_lines` (line 361)

```

    Yield WHL2 twin lines holding a constant model value for `seconds` at `rate_hz`.

    Every line carries the same state (0, 1 or 2). seq counts up from start_seq
    (taken mod 65536). t_us steps by round(1e6 / rate_hz) and wraps at 2^32 like
    the device. A step below 1 us is refused (WhlError). model is value_rad_s
    as given. raw is the value converted to raw units, clamped to +-RAW_CAP
    before the sign is applied, then rounded. raw_l = sign_l * raw and
    raw_r = sign_r * raw. The signs are SIM ONLY and UNCONFIRMED.
    When realtime is True, each line is released against the injected clock
    by sleeping with the injected sleep.
    
```

## `ctrl_wheels_relay.py`

### `module` (line 2)

```

ctrl-wheels relay (issue FW #450): holds the latest WHL sample and decides
what wheel velocity to pass on, with a stale and hold rule.

SIM ONLY. This relay is not a control loop and makes no balance claim.
The net direction with the bridge and Webots signs at +1/+1 is owner-observed (2026-11-06,
sim only); the chip-side sign convention is UNCONFIRMED. The stale rule uses the host wall clock
(time.monotonic by default, injected in tests), not the device t_us.

Rules (pure logic, no sockets, no threads, no sleeping):
- poll() before the first on_whl() returns None (nothing to send).
- A sample with state != 0 (hold or bus down) gives RelayOut(0.0, 1).
- age >= stale_s since the latest on_whl() gives RelayOut(0.0, 1).
- A non-finite model_rad_s (nan, inf, -inf) on a fresh state-0 sample is a
  fault and gives RelayOut(0.0, 1). Without this, min() would pass nan to
  the clamp and return +clamp. Defence in depth: the formatter and parse_line
  both reject non-finite values, so this branch only triggers for a Whl built
  directly in code.
- Otherwise RelayOut(clamp(sign * model_rad_s), 0). Sign is applied before
  the symmetric clamp.

Usage: the caller feeds Whl samples with on_whl() and reads poll() at
POLL_S intervals. The caller owns the transport and the timing.

```

## `ctrl_wheels_udp.py`

### `module` (line 2)

```

UDP v2 JSON messages of the ctrl-wheels bridge (ctrl_wheels_udp): encoders, strict
decoders, a loopback listener and a non-blocking drain helper.

Sim-only twin of the single-wheel module. One command carries the speeds of two
wheels (left and right). The sim side echoes both wheels' angle and velocity.

SIM ONLY. The net direction with the bridge and Webots signs at +1/+1 is owner-observed
(2026-11-06); chip-side signs UNCONFIRMED; the model values are UNVERIFIED.
The wire format is not validated against any real device or receiver.

This is the v2 contract module (CONTRACT.md S3 to S6). The Webots controller
poc/ctrl-wheels/webots/controllers/ctrl_wheels/ctrl_wheels.py cannot import this
module, so it duplicates the JSON key names on purpose. Change a key name in both
places or not at all. This module describes v2 only.

Ports: 5017 (echo, sim -> host) and 5018 (command, host -> sim) only.
Port 5006 is never used by this module. The reserved port set of CONTRACT.md
S4 is not checked here; it belongs to the code that binds the ports.

Messages (compact JSON, UTF-8, one datagram per message):
    cmd:  {"v":2,"cmd_seq":<int >= 0>,"wheels":{"left":<float, clamped +-3.0>,
           "right":<float, clamped +-3.0>},"stale":<0|1>}
    echo: {"v":2,"cmd_seq_echo":<int >= 0 or null>,"sim_t_s":<float>,
           "left":{"angle_rad":<float>,"vel_rad_s":<float or null>},
           "right":{"angle_rad":<float>,"vel_rad_s":<float or null>}}

Strict decode: exact key set at the top level and inside "wheels", "left" and
"right". The nested exact-key-set rule is NEW for v2; the single-wheel module
checked only the top level. The version key "v" must be the JSON integer 2
(not 2.0, not true). No duplicate keys at any depth. Bool rejected for numbers,
NaN / Infinity tokens rejected, int fields must be JSON ints (not floats), and
nulls allowed only for cmd_seq_echo and the two vel_rad_s. Single-wheel v1
bytes fail the key-set check, so they are refused.

Limits, as CONTRACT.md S3 states them: only the command wheels are clamped to
+-3.0 rad/s. The echo velocities and angles are NOT clamped; they pass through
unchanged. cmd_seq and cmd_seq_echo are non-negative ints with no explicit
upper bound. CONTRACT.md S3 says only "JSON integer, 0 or more" and is silent
on huge values. This module refuses a value past Python's int-to-text limit
with UdpMsgError on both encode and decode. That refusal is an implementation
choice, not a contract rule.

Error messages never print a huge int in full: _show describes it by sign and
an estimated digit count, so formatting a bad value cannot raise a bare
ValueError past UdpMsgError.

Usage (run from poc/ctrl-wheels):
    python3 -m pytest test_ctrl_wheels_udp.py -v

```

### `_show` (line 103)

```
Describe a user value for an error message without ever raising.

    A huge int is described by its sign and an estimated digit count, because
    str() of it raises ValueError past the int-to-text limit. Other values use
    repr(), or only the type name when the value is neither a number nor a string.
    
```

### `clamp` (line 155)

```
Limit one wheel speed to +-WHEEL_CLAMP_RAD_S. Rejects bool and non-finite input.

    name only labels the error message (for example "wheels.left").
    
```

## `fake_wheels.py`

### `module` (line 2)

```

Headless twin fake of the Webots ctrl_wheels controller.

SIM ONLY. Headless stand-in for the twin Webots controller at
poc/ctrl-wheels/webots/controllers/ctrl_wheels/ctrl_wheels.py, which is the real
counterpart. The controller and this fake both speak the v2 wire format
(ctrl_wheels_udp.py); no run of this fake against the real controller is claimed. This file
is not a Webots model and makes no physics claim. The wheel sign convention is
UNCONFIRMED (CONTRACT.md S2) and the fake applies each wheel's command as given,
so any sign belongs to the bridge. The model values (tau, dt, clamp, stale
window) are guesses. For tests only. Nothing here validates a real wheel or a
balance loop.

Wire (CONTRACT.md S3, UDP v2, one compact JSON datagram per message):
    cmd  in  on the command port (default 5018 in the real setup):
         {"v":2,"cmd_seq":<int>,"wheels":{"left":<float>,"right":<float>},"stale":<0|1>}
    echo out to echo_addr (default port 5017 in the real setup):
         {"v":2,"cmd_seq_echo":<int or null>,"sim_t_s":<float>,
          "left":{"angle_rad":<float>,"vel_rad_s":<float or null>},
          "right":{"angle_rad":<float>,"vel_rad_s":<float or null>}}
    Decoding and encoding go through ctrl_wheels_udp. A datagram that fails to
    decode (including any v1 command) is skipped without a reply.

Two independent first-order wheels, left and right. Each step (dt_s of sim time):
    target = 0 if no command has been accepted yet, if the accepted command has
             stale == 1, or if the sim age since it was accepted is at least
             stale_s; otherwise the command for that wheel, limited to +-clamp.
    velocity += (target - velocity) * (dt_s / (tau_s + dt_s))    first-order lag
    angle    += velocity * dt_s
    sim time += dt_s
    vel_rad_s is the finite difference of angle, None on the first step (no
    previous angle yet). CONTRACT.md S3 allows a null vel_rad_s; the first-step
    null is this fake's choice.

The stale window uses sim time on purpose, so tests are deterministic. CONTRACT.md
S5 defines the age on the host clock, so this is a deliberate difference.

Ports (CONTRACT.md S4): the reserved set is the literal set {5005, 5006, 5007,
5008, 5017, 5018}. This module refuses to bind a reserved command port, and it
refuses a reserved echo port in echo_addr. It must not be derived from a range.
Ports must be ints in 0..65535. Port 0 (ephemeral) is meant for tests.
The command port is unauthenticated, so host must be "127.0.0.1" or "localhost"
(the default is 127.0.0.1); any other host raises ValueError before binding.

Usage (run from poc/ctrl-wheels):
    python3 -m pytest test_ctrl_wheels_fake.py -v

step() is not thread-safe: drive it from one thread only.

```

### `FakeWheel` (line 73)

```
SIM ONLY headless twin ctrl_wheels stand-in. Applies each wheel's command as given; any sign
    belongs to the bridge. Model values guessed.
```

### `FakeWheel.step` (line 154)

```
Advance one dt_s tick and send one echo. Never raises.

        An echo can be dropped if the encode or send fails.
        
```

## `test_ctrl_wheels_bridge.py`

### `module` (line 2)

```

Tests for the twin ctrl-wheels bridge (ctrl_wheels_bridge). The bridge module is
in this directory; these tests pin its API.

The bridge reads WHL2 lines, splits each into a left and a right single-wheel
sample (ctrl_wheels_msg.split), feeds one Relay per wheel, and sends one UDP v2
command per accepted line. It uses the model velocity only, forward-positive on
both wheels; the raw columns of the line are logged but never drive a command.
If either relay is stale, or the state is not 0, both wheels get (0.0, 0.0) with
stale 1 (they stop together). That stop is not a latch: a fresh state-0 sample
resumes. Each wheel is clamped to +-3.0 silently.

Integration tests use real loopback UDP sockets, a real FakeWheel in a thread
and the real clock, each well under 3 s. Every socket uses port 0 (ephemeral).
Ports 5005-5008, 5017 and 5018 are never bound here.

SIM ONLY. net direction with the bridge and Webots signs at +1/+1 owner-observed (2026-11-06, sim only);
chip-side signs UNCONFIRMED. The model values are UNVERIFIED. Nothing here
validates a real wheel, a balance loop or a serial device. open_serial_lines is
tested only for paths that need no hardware.

Usage (run from poc/ctrl-wheels):
    python3 -m pytest test_ctrl_wheels_bridge.py -v

```

### `_line` (line 68)

```
One stripped WHL2 line (no newline), as the bridge's reader passes it.

    t_us defaults to seq * 20000 (no restart, no wrap); pass it explicitly for restart and wrap tests.
    
```

## `test_ctrl_wheels_bridge_cli.py`

### `module` (line 2)

```

Tests for the ctrl-wheels bridge command line (ctrl_wheels_bridge.main and
build_parser), WHL2 twin form. These tests pin its contract.

CLI contract pinned here:
    exactly one of --port PORT / --stdin; --record CSV (required);
    --overwrite; --sign-left {1,-1} (default 1); --sign-right {1,-1} (default 1);
    --duration SECONDS (> 0);
    --cmd-port N (default 5018); --echo-port N (default 5017);
    --baud N (default 115200); --echo-timeout SECONDS (default 0.5).
--sign alone is refused: it is an ambiguous prefix of --sign-left and --sign-right.
--sign-left and --sign-right are bridge-side signs applied to the model velocity
(Relay sign). They are not the chip raw signs of the generator ctrl_wheels_msg
(--sign-left/--sign-right there default +1/-1). The bridge ignores the raw columns.
The CLI never latches: a fresh state-0 line resumes after a stop.
Parsed attribute names: port, stdin, record, overwrite, sign_left, sign_right,
duration, cmd_port, echo_port, baud, echo_timeout.

Ports: 5005-5008, 5017 and 5018 are never bound by a test. The CLI refuses only
5006; the defaults 5017/5018 are accepted by its checks. Ports used in tests are
ephemeral (bound to port 0, read back, closed). Refusal tests pass fixed
non-reserved numbers because usage errors happen before any socket is bound.

SIM ONLY. net direction with the bridge and Webots signs at +1/+1 owner-observed (2026-11-06, sim only); chip-side signs UNCONFIRMED; the model values are UNVERIFIED. Nothing
here validates a real wheel, a balance loop or a serial device.

Usage (run from poc/ctrl-wheels):
    python3 -m pytest test_ctrl_wheels_bridge_cli.py -v

```

### `test_old_sign_flag_refused_as_ambiguous_prefix` (line 224)

```
--sign is a prefix of both --sign-left and --sign-right, so argparse abbreviation
    matching refuses it as ambiguous; it is not a silent alias.
```

### `test_sign_flags_flip_each_echoed_wheel_velocity` (line 447)

```
--sign-left/--sign-right are bridge-side signs on the model velocity, SIM ONLY.
    Chip-side signs UNCONFIRMED; net direction at +1/+1 owner-observed. The input uses the
    generator's default raws (+1 left, -1 right); the bridge
    ignores raw, so each echoed wheel follows its own sign flag alone.
```

### `_sigint_when_reader_runs` (line 647)

```
Wait (bounded, 3 s) until the bridge's line reader thread runs, set seen, pause a fixed
    0.05 s and send SIGINT unless cancel is set. If the reader thread never appears, no SIGINT
    is sent and seen stays unset, so the test fails (main then ends on its --duration).
```

### `test_clean_ctrl_c_prints_nothing_after_the_summary` (line 664)

```
After a clean stop on a pyserial loop:// loopback source, the reader
    thread's failed read must not log 'line source stopped' after the summary. The SIGINT lands
    while the reader thread is live, so the port is closed mid-read.
```

### `test_midrun_source_failure_line_comes_before_the_summary` (line 703)

```
A real mid-run source failure is still logged,
    before the summary, and returns 2.
```

## `test_ctrl_wheels_fake.py`

### `module` (line 2)

```

Unit tests for fake_wheels.FakeWheel, the headless twin stand-in for the Webots
ctrl_wheels controller.

Pinned API:
    FakeWheel(echo_addr, cmd_port=0, tau_s=0.05, dt_s=0.016, stale_s=0.2,
              clamp=3.0, host="127.0.0.1")
    .cmd_addr, .step(), .close(), .run_background(stop_event) -> Thread

Contract: poc/ctrl-wheels/CONTRACT.md S3 (UDP v2 command and echo shapes) and
S4 (the reserved port set is the literal set {5005, 5006, 5007, 5008, 5017, 5018},
never a range). Tests bind only ephemeral ports (0) on the loopback address.

SIM ONLY. The sign convention is UNCONFIRMED (CONTRACT.md S2). The fake applies
each wheel's command as given, so any sign is the bridge's job, not the fake's.
The model values (tau, dt, stale window) are guessed values, not in CONTRACT.md;
retuning them breaks these tests. Nothing here validates a real
wheel or a balance loop.

Usage (run from poc/ctrl-wheels):
    python3 -m pytest test_ctrl_wheels_fake.py -v

```

### `_run` (line 114)

```
Model the real bridge: re-send the same v2 command before every step.

    Returns the decoded echoes, one per step.
    
```

### `_expected_path` (line 128)

```
Per-step velocity and angle for a list of per-step targets.

    Written from the guessed model, not from fake_wheels.py: velocity starts at
    0 and angle at 0; each step v += (target - v) * LAG, then angle += v * DT_S.
    
```

### `_assert_leg_path` (line 153)

```
Check one wheel of every echo against the first-order path.

    The velocity on the first step is None: CONTRACT.md S3 allows a null
    vel_rad_s, and the first-step null is this fake's choice (no previous
    angle). Index 0 checks for None and the rest compare numbers.
    
```

## `test_ctrl_wheels_log.py`

### `module` (line 2)

```

Unit tests for the ctrl-wheels CSV log writer (ctrl_wheels_log), task F09
(freecad-workspace #493, umbrella #482).

SIM ONLY. Not hardware-validated. Chip-side signs UNCONFIRMED; Webots +1/+1 owner-observed (2026-11-06, sim only).

This suite pins contract S6 of poc/ctrl-wheels/CONTRACT.md for ctrl_wheels_log:
the 19-column tuple, the column docs, the cell formatting rules (int columns,
float columns with %.6f and %.3f), the refusals (bool, NaN, inf, values out of
float range, unknown columns), the atomic-row rule, and the file and close
behaviour. The expected text is hand-written. The column tuple is duplicated
here by design. When the contract changes, edit the contract and this test
together.

Every test writes only under pytest's tmp_path, never to a real path, and
opens no network port.

Usage:
    cd poc/ctrl-wheels
    python3 -m pytest test_ctrl_wheels_log.py -v

```

### `_no_echo_row` (line 186)

```
Command with no echo.

    missing=True omits the NO_ECHO_KEYS; missing=False sets them to None.
    
```

### `TestUnknownKey` (line 406)

```
Test: an unknown key is refused and nothing is written.

    The single-wheel names are included as a drift guard: they must not be
    accepted by the twin writer.
    
```

## `test_ctrl_wheels_msg.py`

### `module` (line 2)

```

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

Chip-side signs are UNCONFIRMED (the rawR negation). The net direction with the bridge and
Webots signs at +1/+1 is owner-observed (2026-11-06, sim only); the forward-positive model
definition is in CONTRACT.md S2.
Nothing here is validated on real hardware.

The reverse-direction reject ("WHL2 line against the single-wheel WHL v1
parser") is planned in the BRC and FW follow-up. It is not tested in this
directory, because the single-wheel v1 parser is not present here.

Usage:
    cd poc/ctrl-wheels
    python3 -m pytest test_ctrl_wheels_msg.py -v

```

### `_with_checksum` (line 59)

```
Return body + "*HH" + newline, with HH the XOR of the body bytes.

    The XOR is written out here on purpose, so these tests do not reuse the
    implementation's checksum(). It is used only for bodies that must pass the
    checksum and then fail a field rule.
    
```

## `test_ctrl_wheels_msg_steps.py`

### `module` (line 2)

```

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

```

## `test_ctrl_wheels_relay.py`

### `module` (line 2)

```

Unit tests for the ctrl-wheels relay (ctrl_wheels_relay), issue FW #450.

The module under test is the flat module ctrl_wheels_relay in poc/ctrl-wheels.
Every test uses an injected fake clock, never real time. SIM ONLY. The net direction with the
bridge and Webots signs at +1/+1 is owner-observed (2026-11-06); chip-side signs UNCONFIRMED.

Usage:
    cd poc/ctrl-wheels
    python3 -m pytest test_ctrl_wheels_relay.py -v

```

## `test_ctrl_wheels_udp.py`

### `module` (line 2)

```

Unit tests for the UDP v2 JSON messages of the ctrl-wheels bridge (ctrl_wheels_udp).

SIM ONLY. Tests for the UDP v2 messages of ctrl_wheels_udp: encoders, strict
decoders, the loopback listener and the drain helper.

What the suite pins: poc/ctrl-wheels/CONTRACT.md, section S3 (the v2 command
and echo shapes, the exact-key-set rule at every level, the integer version
tag v == 2, the +-3.0 clamp on the command wheels only, the nulls allowed in
the echo, no explicit upper bound on cmd_seq) and section S4 (command port
5018, echo port 5017, asserted as values). The literal reserved port set of S4
belongs to the code that binds the ports, not to this module, so it is not
tested here.

CONTRACT.md S3 says only "JSON integer, 0 or more" and is silent on huge values.
The refusal of an int past Python's int-to-text limit with UdpMsgError, on
encode and on decode, is an implementation choice of ctrl_wheels_udp, pinned
here; it is not a contract rule.

Assumption: the four int-to-text tests size their inputs from
sys.get_int_max_str_digits() at import time (limit + 1 digits). They are
skipped when that limit is 0 (unlimited) or when the interpreter has no such
limit. The default CPython limit is 4300 digits.

Stdlib plus pytest. Tests bind only 127.0.0.1 port 0
(ephemeral). The production ports are asserted as values and never bound.

The golden byte literals in TestGolden are hand-written. They follow the key
order of the contract and the float text that json.dumps produces for a Python
float (repr, so 3.0 is written as "3.0" and 12.5 as "12.5"). They were not
produced by running the implementation, so a wrong encoder cannot pass them by
copying its own output.

Usage:
    cd poc/ctrl-wheels
    python3 -m pytest test_ctrl_wheels_udp.py -v

```

## `webots/controllers/ctrl_wheels/ctrl_wheels.py`

### `module` (line 2)

```

ctrl_wheels -- Webots controller for the twin-wheel ctrl-wheels relay simulation.

SIM ONLY. Nothing here is validated on hardware, and it makes no balance claim.
Webots side: with SIGN_LEFT/SIGN_RIGHT +1/+1 and the bridge signs +1/+1 the net
direction was owner-observed (2026-11-06, sim only; old +-0.026 track; repeated on ctrl_wheels.wbt 2026-11-08, owner-reported). The sign of each wheel against
the firmware (chip side) is UNCONFIRMED. Beyond that direction check, runtime
behaviour in Webots is NOT CHECKED.

Webots controller for the twin-wheel ctrl-wheels simulation. It speaks the v2 wire
format of poc/ctrl-wheels/CONTRACT.md (sections S3 and S4).
It binds the UDP command port CMD_PORT (127.0.0.1:5018) and drains it every step,
keeps the newest valid v2 command, and drives the two wheel motors with
setVelocity. It reads at most 1024 datagrams per step (MAX_DATAGRAMS_PER_STEP),
processes each in arrival order, and the newest valid one wins. A backlog deeper
than the bound is read on later steps (not discarded), so a sim-time watchdog
cannot see the wall-clock age of queued commands. The bound is a chosen value, not
measured; not verified in Webots. The watchdog age is measured in simulated
time from the step in which the newest command was processed. This differs from
CONTRACT.md S5 (host clock, bridge side) on purpose, same as the single-wheel
controller; not verified in Webots at faster-than-real-time.
Known limits (documented only, no code change, owner decision):
(a) A Webots pause stops simulated time, so the 0.2 s sim-time watchdog does not
fire during a pause.
(b) The command socket is a plain Linux UDP socket with no SO_RCVBUF tuning and no
timestamps. If its receive buffer fills, the kernel drops the NEWEST datagrams, so
a stop (stale=1) sent during a stall can be lost, and the controller may apply a
command from before the stall for up to STALE_SIM_S of simulated time after resume.
(c) The intended guard during a pause is the bridge's host-clock stale=1
(CONTRACT.md S5), not this controller. The single-wheel controller behaves the
same. Sim-only, the bound is unmeasured, not verified in Webots.
Both wheels get 0 rad/s when no valid command has arrived, when the
newest command carries stale=1, or when the newest valid command is STALE_SIM_S
or more old in simulated time. A failed or non-finite sensor read also stops both
motors and skips that step's log line, echo and angle update. Each normal step it
sends ONE echo datagram to 127.0.0.1:5017 (ECHO_ADDR). Send failures are ignored
and counted, never raised. The wire format is contract v2 in
poc/ctrl-wheels/CONTRACT.md (sections S3 and S4).

Only the Webots-specific import is `controller`; the sensor maths lives in
sensor_read.py so it stays importable outside Webots.

Device names (must match the twin world ctrl_wheels.wbt, case-sensitive, #510).
The names below match webots/worlds/ctrl_wheels.wbt:
    RotationalMotor: "wheel_motor_left", "wheel_motor_right"
    PositionSensor:  "wheel_sensor_left", "wheel_sensor_right"

```

### `_parse_cmd` (line 117)

```
Return (cmd_seq, left_rad_s, right_rad_s, stale) for a valid v2 command, else None.

    Checks match ctrl_wheels_udp.decode_cmd: exact key sets at the top level and
    inside "wheels", "v" must be the JSON integer 2, booleans are refused where
    numbers are expected, NaN and Infinity tokens are refused, ints must be JSON
    ints, duplicate keys are refused. The wheel values are not clamped here; the
    clamp is applied when the command is applied. Any malformed input returns None
    and never raises.
    
```

### `_open_socket` (line 160)

```
Bind a non-blocking UDP socket. Deliberately NO SO_REUSEADDR: on Linux it
    would let two listeners share the port and silently split datagrams.
```

## `webots/controllers/ctrl_wheels/sensor_read.py`

### `module` (line 2)

```

Pure sensor-readback module for the servo-wheel-dof Webots POC (issue #263).

Processes raw PositionSensor angle readings (radians, per Webots convention) into
validated (time, angle, angular-velocity) samples. Contains **no** Webots dependency
(must NOT `import controller`) so it stays importable and unit-testable outside Webots,
per this repo's dependency-injection convention (see poc/esp32-dof/webots/controllers/
esp32_dof_follower/dof_webots_math.py and poc/esp32-dof/CLAUDE.md's "Conventions"
section: math/logic modules stay controller-free; only the top-level Webots controller
script imports `controller`).

**Units:**
- Angle: radians, as returned directly by Webots' PositionSensor.getValue() (no unit
  conversion needed here, unlike a raw ADC/gyroscope reading).
- Time / dt: seconds, float.
- Angular velocity: radians/second, computed as a simple finite difference between two
  successive angle readings -- Webots' PositionSensor has no companion velocity sensor,
  so this is the only way to obtain a rate from it in this POC.

**Validation:**
- Readings must be finite (no NaN/+-inf). Non-finite input raises SensorReadError,
  mirroring poc/esp32-dof/monitor/dof_frame.py's FrameError pattern (a dedicated
  ValueError subclass) rather than silently substituting a default -- a bad readback
  here should surface immediately in the calling controller/tests, not be masked.
- dt_s must be > 0 to compute a velocity; dt_s <= 0 raises SensorReadError (mirrors
  poc/esp32-dof/monitor/dof_stats.py's guard against a non-positive time span, but
  raises instead of silently returning 0.0 -- an issue #265 controller calling this
  every timestep should never legitimately see dt <= 0, so treat it as a bug signal).

Usage:
    from sensor_read import angular_velocity_from_positions, read_sensor

    # Low-level: compute velocity from two raw angle samples
    velocity_rad_s = angular_velocity_from_positions(angle_prev_rad=0.10,
                                                       angle_curr_rad=0.12,
                                                       dt_s=0.02)

    # High-level: package a validated (t, angle, velocity) sample, injecting the raw
    # angle source as a callable or plain number (dependency-injection pattern, so no
    # running Webots instance / PositionSensor is required to exercise this):
    reading = read_sensor(t_s=1.02, angle_source=lambda: sensor.getValue(),
                           angle_prev_rad=0.10, t_prev_s=1.00)
    print(reading.angle_rad, reading.velocity_rad_s)

```

### `_validate_finite` (line 68)

```

    Validate that `value` is a finite float (not NaN/+-inf, not a non-numeric type).

    Args:
        value: Value to validate.
        name: Field name, used only in the error message.

    Returns:
        float: `value`, unchanged.

    Raises:
        SensorReadError: if `value` is not a finite real number.
    
```

### `angular_velocity_from_positions` (line 95)

```

    Compute angular velocity from two successive angle readings (finite difference).

    Webots' PositionSensor has no companion velocity sensor, so this is the standard
    way to derive a rate from it: velocity = (angle_curr - angle_prev) / dt.

    Args:
        angle_prev_rad: Previous angle reading, in radians.
        angle_curr_rad: Current angle reading, in radians.
        dt_s: Elapsed time between the two readings, in seconds. Must be > 0.

    Returns:
        float: Angular velocity in radians/second.

    Raises:
        SensorReadError: if either angle is non-finite, or dt_s is non-finite or <= 0.
    
```

### `read_sensor` (line 126)

```

    Read and validate one angle sample, packaging it with a finite-difference velocity.

    `angle_source` is injected rather than read from a live PositionSensor directly, so
    this stays testable without a running Webots instance (mirrors poc/esp32-dof's
    dependency-injection pattern of passing in a source rather than importing
    `controller`). Pass a plain number for a fixed/mock reading, or a zero-argument
    callable (e.g. `sensor.getValue`) for a live one -- the caller (issue #265's
    controller) decides which.

    Args:
        t_s: Current elapsed time in seconds (e.g. from Webots robot.getTime()).
        angle_source: Either a float (the raw angle reading, radians) or a
            zero-argument callable returning one.
        angle_prev_rad: Previous angle reading in radians, or None if this is the
            first sample (no velocity can be computed yet).
        t_prev_s: Timestamp of the previous reading in seconds, required (not None)
            whenever angle_prev_rad is not None.

    Returns:
        SensorReading: (t_s, angle_rad, velocity_rad_s). velocity_rad_s is None when
            angle_prev_rad is None (first sample); otherwise a finite-difference rate.

    Raises:
        SensorReadError: if the raw angle is non-finite, t_s/t_prev_s are non-finite,
            angle_prev_rad is given without t_prev_s (or vice versa), or the resulting
            dt_s is <= 0.
    
```
