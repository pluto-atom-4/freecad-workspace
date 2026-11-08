# ctrl-wheels (twin wheel pair)

SIM ONLY. This directory holds the bridge, relay, CSV log, fake wheels and Webots twin for the BRC firmware program `programs/c3_ctrl_wheels` (Wokwi, sim only). The firmware prints a WHL2 line per tick. The bridge reads that line, sends one UDP v2 command (both wheels) to the Webots controller per accepted line, and logs the Webots echo to a CSV. It is a relay, not a closed loop: the firmware receives nothing back. The model value is never mixed with the Webots angle or velocity; the CSV keeps them in separate columns. Not hardware-validated. No balance claim. The net direction at bridge +1/+1 with Webots +1/+1 is owner-observed (2026-11-06 on the old track, 2026-11-07 on the two-box world, earlier non-physics body, not re-run on the physics-root world, re-run pending #552); the chip-side sign is UNCONFIRMED and latency is UNMEASURED (`host_rtt_ms` is wall time, not link latency).

## Status

Ran (owner-reported, sim only; the author of this README did not run these):

- Webots twin run (owner-reported).
- Load of the earlier non-physics two-box version of `webots/worlds/ctrl_wheels.wbt` in Webots (owner-reported, 2026-11-07, sim only): no red error in the console; two body boxes (left reddish, right bluish) with a visible gap of about 8 mm; each wheel touches its box with no gap and no overlap; red marker on the left wheel, blue on the right; no flicker at the wheel and box faces; motor and sensor names as expected; angles and velocities 0 while idle (first velocity sample nan in the console). The console showed one WARNING: `DEF CTRL_WHEELS Robot: This node is controlled in kinematics mode but some Solid descendant nodes have physics and won't move along with this node.` It came from that earlier world, where only the wheel endPoints had Physics (#544). The physics-root world (PR #553) has not been loaded by the author; whether the warning is gone is pending #552.
- Direction run on the earlier non-physics two-box body (owner-reported, 2026-11-07, sim only, bridge and Webots signs +1/+1, one run per direction, `python3 -m ctrl_wheels_msg --step 1.0 --seconds 6 | mamba run -n esp32-dof python3 ctrl_wheels_bridge.py --stdin --record h1_pos.csv` and the same with `--step=-1.0` and `--record h1_neg.csv`): in Webots a positive command turned both wheels in the same sense and they rolled toward +x, a negative command rolled toward -x. The wheel sensors read +1/+1 and -1/-1 (figures in Sign convention). Chip-side signs, Wokwi, latency and hardware are not covered. Not re-run on the physics-root world; re-run pending #552.
- VS Code Wokwi run of `c3_ctrl_wheels-xiao_c3` into the bridge (`--port rfc2217://localhost:4002`, 60 s), with the Webots run on the other side. The owner reported that both wheels turned together and that the bridge exited with code 0. This run does not settle the direction of either wheel.

Not run:

- The firmware's 3-failed-tick latch.
- Bus-down state 2.
- A fall seen in WHL2 (state 1).
- A long run.
- Three RFC2217 listeners at once.
- Any hardware.

Open:

- Wheel sign: bridge and Webots signs at +1/+1 gave an owner-observed net direction in Webots (2026-11-06 on the old track, 2026-11-07 on the two-box world, sim only, earlier non-physics body, re-run pending #552); chip-side signs still UNCONFIRMED (see Sign convention).
- Latency: UNMEASURED. `host_rtt_ms` includes up to one simulation step and one bridge poll.
- Physics numbers in `ctrl_wheels.wbt` (mass, centerOfMass, skid size, skid clearance, floor friction) are guesses, not measured.

## Links

- Plan: BRC discussion 266, https://github.com/pluto-atom-4/balancing-robot-controller/discussions/266
- Earlier strategy: FW discussion 481, https://github.com/pluto-atom-4/freecad-workspace/discussions/481
- Umbrella: FW issue 482, https://github.com/pluto-atom-4/freecad-workspace/issues/482
- Wire and file contract: [CONTRACT.md](CONTRACT.md) in this directory
- Firmware program: `programs/c3_ctrl_wheels` in pluto-atom-4/balancing-robot-controller, https://github.com/pluto-atom-4/balancing-robot-controller/tree/main/programs/c3_ctrl_wheels. Its env is `c3_ctrl_wheels-xiao_c3` (`platformio.ini`), and its `wokwi.toml` sets `rfc2217ServerPort = 4002`.
- Sibling, single wheel: [`poc/ctrl-wheel`](../ctrl-wheel/README.md). This directory does not change it.

## Files

| File | What it does |
| ---- | ------------ |
| `README.md` | This file. |
| `CONTRACT.md` | Wire and file contract: WHL2 line (S1), sign convention (S2), UDP v2 (S3), ports (S4), stale and hold (S5), CSV columns (S6), golden lines and reject cases (S7), unverified items (S8). |
| `ctrl_wheels_msg.py` | WHL2 formatter and parser, `split()` into two single-wheel samples, and the `step_lines` step generator with its CLI (`python3 -m ctrl_wheels_msg`). The WHL v1 formatter is kept for the bridge tests. The checksum is the XOR of the body, two uppercase hex digits. A non-finite model is refused. The +-125 raw cap is applied by `step_lines` only, not on the wire. |
| `ctrl_wheels_udp.py` | UDP v2 JSON encoders and strict decoders (exact key sets at every level, no duplicate keys, no NaN or Infinity), clamp of the command wheels to +-3.0 rad/s, loopback listener and non-blocking drain. Constants `PORT_CMD` (5018) and `PORT_ECHO` (5017). Never uses 5006. |
| `ctrl_wheels_relay.py` | `Relay`, one per wheel, no sockets. A state other than 0 gives 0.0 and stale 1. An age of 200 ms or more on the host clock gives 0.0 and stale 1. The sign is applied before the symmetric clamp. |
| `ctrl_wheels_log.py` | CSV writer with 19 columns (`COLUMNS`) and `COLUMN_DOC`. Cell rules: None or a missing key gives an empty cell, bool, NaN and inf are refused, and a row is validated before anything is written. A path target is opened with mode `x` unless overwrite is set. |
| `ctrl_wheels_bridge.py` | Bridge. Reads WHL2 lines from a serial URL, a device path or stdin, runs one relay per wheel, sends one UDP command per accepted line and writes one CSV row per WHL2 line plus one per silence episode. CLI in `build_parser()` and `main()`. |
| `fake_wheels.py` | `FakeWheel`, a headless library stand-in for the Webots controller. First-order lag with guessed `tau_s` 0.05 and `dt_s` 0.016. Refuses a reserved `cmd_port` and a reserved `echo_addr` port (the literal set 5005, 5006, 5007, 5008, 5017, 5018). It binds only the command port; the echo is sent to `echo_addr`. Has no CLI; used by the tests only. |
| `test_ctrl_wheels_msg.py` | WHL2 format, parse, reject and split tests. Golden lines copied from CONTRACT.md S7. |
| `test_ctrl_wheels_msg_steps.py` | Tests for `step_lines` and the `ctrl_wheels_msg` CLI. |
| `test_ctrl_wheels_udp.py` | Tests for the UDP v2 encoders, decoders and listener. |
| `test_ctrl_wheels_relay.py` | Relay rules, with an injected fake clock. |
| `test_ctrl_wheels_log.py` | CSV log cell, column and file-behaviour tests. |
| `test_ctrl_wheels_fake.py` | `FakeWheel` tests, ephemeral ports only. |
| `test_ctrl_wheels_bridge.py` | Bridge integration tests over loopback UDP, with a real `FakeWheel` in a thread, ephemeral ports only. |
| `test_ctrl_wheels_bridge_cli.py` | Bridge command-line tests (`build_parser`, `main`). |
| `webots/controllers/ctrl_wheels/ctrl_wheels.py` | Webots controller. Binds 127.0.0.1:5018, keeps the newest valid v2 command and sets both motor velocities with fixed signs of +1. Both wheels stop when no valid command has arrived, when the newest command has stale 1, or when it is 0.2 s of sim time old. Sends one echo per step to 127.0.0.1:5017. |
| `webots/controllers/ctrl_wheels/sensor_read.py` | Sensor maths (finite angle check, finite-difference velocity). No Webots import. |
| `webots/worlds/ctrl_wheels.wbt` | Webots world: the Robot is the physics root (a free body with a guessed mass and centerOfMass; it rolls and may leave the 2 m floor), with two body boxes (`BODY_BOX_LEFT`, `BODY_BOX_RIGHT`, each 0.04 x 0.02 x 0.04 m at local z +-0.024, 8 mm gap) and two hinge wheels at local z +-0.050 (track 10.0 cm), motors `wheel_motor_left` and `wheel_motor_right`, sensors `wheel_sensor_left` and `wheel_sensor_right`. Direction with signs +1/+1: owner-observed OK in Webots (2026-11-06 on the old track, wheels at z +-0.026, 5.2 cm, one body box; 2026-11-07 on the earlier non-physics two-box body, not re-run on this world, re-run pending #552; see Sign convention and Status). Load and appearance of this world: owner-reported 2026-11-07, see Status. Motor limits are owner-chosen for this simulation and not checked here. |

## Origins

Copied and renamed from `poc/ctrl-wheel` (`ctrl_wheel_*` to `ctrl_wheels_*`, `fake_wheel` to `fake_wheels`), then changed for the two-wheel twin. What the headers and CONTRACT.md show:

- `ctrl_wheels_msg.py`: the header calls it the sim-only twin of the WHL2 protocol. It keeps the single-wheel `Whl` and `format_whl` for the bridge tests, and notes that the single-wheel module has no 20-character model-text guard.
- `ctrl_wheels_udp.py`: the header says "Sim-only twin of the single-wheel module". The nested exact-key rule is new for v2 (CONTRACT.md S3).
- `ctrl_wheels_relay.py`: CONTRACT.md S5 gives the single-wheel `ctrl_wheel_relay.py` rules. The twin applies one state and one stale flag to both wheels.
- `ctrl_wheels_log.py`: CONTRACT.md S6 gives the single-wheel column tuple and the twin's column changes.
- `ctrl_wheels_bridge.py`: the header says the serial helpers (`DofSerialError`, the URL open with retry, and the read and sync loop) are copied, not imported, from `poc/esp32-dof/monitor/dof_serial.py`. So the RFC2217 open and retry and the line reader come from there. The rest is the twin bridge.
- `fake_wheels.py`: the header calls it the headless twin fake of the Webots controller. CONTRACT.md S4 names the single-wheel fake wheel as the source of the reserved-port rule, which this twin writes out as a literal set.
- `webots/controllers/ctrl_wheels/ctrl_wheels.py`: the header says it speaks CONTRACT S3 and S4 and matches the single-wheel controller's stop behaviour. It drives two wheels.
- `webots/controllers/ctrl_wheels/sensor_read.py`: the header names the servo-wheel-dof POC (issue #263). The sibling README calls it a byte copy of the servo-wheel-dof file. That byte copy was not diffed here.
- `webots/worlds/ctrl_wheels.wbt`: the header says "Twin of ctrl_wheel.wbt". The two hinges sit at mirrored offsets (local z +-0.050, track 10.0 cm), the body is two boxes with an 8 mm gap, and the Robot is rotated -90 degrees about x (the single-wheel world uses +90, per its header). The track and the second box were changed after the first build of this world (PR #538). The Robot is now the physics root (a free body, PR #553).
- Test files: each header names the module it pins. They were written for the twin and were not diffed against the sibling tests.

## Sign convention

There are three sign places. They are independent, and nothing proves that they agree. The chip side is UNCONFIRMED; the Webots side is owner-observed (see below).

1. Chip side (BRC firmware `programs/c3_ctrl_wheels`, sim only, CONTRACT.md S2): `rawL = +raw` and `rawR = -raw`. The WHL2 line carries `rawR` already negated. The generator `python3 -m ctrl_wheels_msg` has `--sign-left` and `--sign-right` defaulting to +1 and -1. Those are the chip raw signs.
2. Bridge side: `--sign-left` and `--sign-right`, default +1 each, applied to the model value before the +-3.0 clamp. The raw columns of the line never reach the command; the bridge sends the model only.
3. Webots controller: `SIGN_LEFT = 1` and `SIGN_RIGHT = 1`, fixed constants in `ctrl_wheels.py`.

Flip one side only, and in one place. A bridge flip and a controller flip on the same wheel would cancel each other (the controller's own comment says so). The Webots defaults of +1/+1 follow the intended world design. The owner observed in the Webots 3D view (2026-11-06, sim only) that with +1/+1 a positive command turns both wheels about world +y in the same sense and they roll toward +x (viewed from each wheel's outside they look opposite; viewed from one side, same sense). That observation was made on the old world (wheels at local z +-0.026, one body box). The owner repeated it on the two-box world (wheels at +-0.050) on 2026-11-07 (sim only, earlier non-physics body, re-run pending #552, see Status): a positive command turned both wheels in the same sense and they rolled toward +x, a negative command rolled toward -x. Old-track CSVs sign_pos.csv (positive command run) and sign_neg.csv (negative command run, both signs +1; not the `--sign-right -1` run) (owner-local, not committed) show both wheel sensors +1.000/+1.000 and -1.000/-1.000, angle change about +9.45 and -9.47 rad over about 9.5 s of sim time. On the earlier non-physics two-box body (2026-11-07, owner-reported), h1_pos.csv and h1_neg.csv (owner-local, not committed, one run each, 300 rows each, 0 stale rows) show the same pattern: both wheel sensors +1.000/+1.000 and -1.000/-1.000, angle change +5.50 and -5.66 rad over about 5.5 s and 5.7 s of sim time, left and right identical (a non-physics figure; on the physics-root experiment copy ctrl_wheels_full.wbt, owner-reported in #544 and #545 and not on ctrl_wheels.wbt, left and right are no longer identical: left-right angle gap about 0.042 rad, velocity transients up to 3.96 rad/s). A run with `--sign-right -1` (commands +1/-1) gave erratic wheel velocities up to about 20 rad/s, above the 3.5 rad/s motor limit: physics instability, cause not investigated, not a recommended setting. That run was on the old world and is untested on the free body. Chip-side signs (rawL=+raw, rawR=-raw) stay UNCONFIRMED.

The only sign flags are `--sign-left` and `--sign-right`. No unsuffixed sign option exists in any CLI here.

## Run

All commands in this section run from the `poc/ctrl-wheels` directory. Run `cd poc/ctrl-wheels` once, then the commands below. The bridge commands use the `esp32-dof` env through `mamba run -n esp32-dof`, because pyserial is imported when the bridge loads.

Tests:

```
mamba run -n esp32-dof python3 -m pytest -q
```

Keep one pytest run to one poc directory. The module name `sensor_read` exists in more than one poc directory.

Stdin pipeline (no simulator needed for the bridge itself). The generator imports only the standard library, so it runs under the default `python3`:

```
python3 -m ctrl_wheels_msg --step 0.5 --seconds 5 | mamba run -n esp32-dof python3 ctrl_wheels_bridge.py --stdin --record ac1.csv
```

`ac1.csv` must not exist yet; add `--overwrite` to replace it. With no Webots running there is no echo, so each row is written after `--echo-timeout` (default 0.5 s) with empty echo cells. `--no-sleep` on the generator prints as fast as possible.

Webots world:

1. Open `webots/worlds/ctrl_wheels.wbt` in Webots and run it. The robot's controller is named `ctrl_wheels`. When it starts, the controller binds 127.0.0.1:5018 and sends echoes to 127.0.0.1:5017. The robot is free and rolls (about 0.17 m per 6 s run at 1.0 rad/s is a guess, not measured) and may leave the 2 m floor. Use File > Reload World between runs.

Live Wokwi run (the owner's route is the VS Code extension). Start order: Webots first, then the bridge, then the Wokwi simulation:

```
mamba run -n esp32-dof python3 ctrl_wheels_bridge.py --port rfc2217://localhost:4002 --record ac1.csv --duration 60
```

With a URL port the bridge retries the first open once per second for up to 300 s. With `--duration` the retry also stops at that limit, and the run exits 2 if the port never opened.

Bridge flags:

| Flag | Default | Meaning |
| ---- | ------- | ------- |
| `--port PORT` | one of `--port` or `--stdin` is required | Serial device path or pyserial URL, such as `rfc2217://localhost:4002`. |
| `--stdin` | off | Read WHL2 lines from stdin until EOF. |
| `--record CSV` | required | CSV log path. |
| `--overwrite` | off | Replace an existing `--record` file. |
| `--sign-left {1,-1}` | 1 | Bridge-side sign on the left model value, applied before the clamp. Net direction with this flag and the Webots constants at +1/+1 owner-observed 2026-11-06 on the old track and 2026-11-07 on the earlier non-physics two-box body, re-run pending #552 (sim only). |
| `--sign-right {1,-1}` | 1 | Bridge-side sign on the right model value, applied before the clamp. Net direction with this flag and the Webots constants at +1/+1 owner-observed 2026-11-06 on the old track and 2026-11-07 on the earlier non-physics two-box body, re-run pending #552 (sim only). |
| `--duration SECONDS` | run until the source ends | Stop after SECONDS (> 0). |
| `--cmd-port N` | 5018 | UDP port of the Webots controller for commands. Only 5006 is refused. |
| `--echo-port N` | 5017 | UDP port the bridge listens on for echoes. Only 5006 is refused. The two ports must differ. |
| `--baud N` | 115200 | Serial baud rate. Not used by `--stdin`. |
| `--echo-timeout SECONDS` | 0.5 | Wait for an echo before writing the row without one, 0 to 10. |

Exit codes: 0 when the run finishes (EOF, duration or Ctrl-C during the run). A Ctrl-C during startup (after the source opens, before the SIGINT handler is installed) or during the final cleanup (after the handler is restored) is not handled and ends with a traceback and a non-zero exit. 2 for a usage error, a failed source (at open or mid-run) or a `--record` error. 2 also covers a failed bind of the echo port (`--echo-port`), which the bridge logs as `cannot listen for echoes on UDP port N`; that bind is tried before the `--record` file is created.

`fake_wheels` is a library only. It has no CLI and is used by the tests.

## Pre-run checklist

- Nothing listening on TCP 4002: `ss -ltn`. A VS Code listener may stay open after a stop; that is observed for port 4000 only.
- Nothing listening on UDP 5017 or 5018: `ss -lun`. The bridge binds 5017 and the Webots controller binds 5018.
- No `ctrl_wheels_bridge.py` left from an earlier run (check with `ps`).
- No earlier Webots instance of this world is running; a second controller cannot bind 5018.
- Start order: Webots, then the bridge, then the simulation.
- One bridge instance only.
- Port 5006 is never used. The bridge refuses it for `--cmd-port` and `--echo-port`, the fake refuses it, and no module here binds it.

## Notes

- Firmware text on the serial stream: any line that is not WHL2 fails `parse_line`, is counted in `bad_lines` and does not stop the run. Such lines carry no WHL2 fields, so the count is benign.
- Firmware restart (code and unit tests only, not run in the simulator; the full `poc/ctrl-wheels` pytest run by the main session at the merge commit's parent 070a8ac gave 865 passed, 10 skipped): the bridge detects a firmware restart from WHL2 `seq` and `t_us` (PR #523, merged). Rule, with `gap = (seq - last_seq - 1) mod 65536` and `dt = (t_us - last_t_us) mod 2^32`:
  - Forward step, when `gap < 32768` and `dt <= (gap + 1) * 40 ms + 1 s`: `gap` is counted in `missed_whl`.
  - Repeat or reorder, when `gap >= 32768` and `t_us` went back by less than 1 s: missed 0, baseline stays at the highest seq.
  - Anything else is a restart: missed 0, the new `BridgeStats` field `restarts` (the last `BridgeStats` field; in the stderr summary line it comes just before `send_errors`) goes up by 1, one stderr log line is written, and the baseline moves to the new seq and `t_us`.
  - The CSV is unchanged (19 columns).
  - The thresholds (40 ms frame, 1 s slack, 1 s reorder) are UNMEASURED guesses for the 50 Hz firmware.
  - Known limits, one line each, detailed in the module docstring of `ctrl_wheels_bridge.py` (the list "Known limits", items (i) to (vi)):
    - (i) `seq` goes back while `t_us` goes forward: read as a restart.
    - (ii) a restart whose new seq is 0..32767 behind the baseline, with first `t_us` within 1 s below the old one, is read as a reorder.
    - (iii) that case is noticed only when `t_us` or `seq` passes the old baseline, within about 1 s of lines; missed stays 0 meanwhile.
    - (iv) a late frame more than 1 s behind, after a forward jump of 1 s or more, is read as a false restart plus a false missed count.
    - (v) a real outage longer than `(missed + 1) * 40 ms + 1 s` is logged as a restart and its missed frames are not counted.
    - (vi) the thresholds are unmeasured.
- One command per WHL2 line: each accepted line gives one UDP command to both wheels and one CSV row with its own `whl_*` cells.
- Stops are not latches. A state other than 0, or a stale relay, sends 0.0 to both wheels with stale 1. A fresh state-0 line resumes. The bridge's stale rule uses the host clock (200 ms). The Webots controller also stops both wheels after 0.2 s of sim time without a valid command. A Webots pause stops sim time, so that watchdog does not fire during a pause (the controller header, known limit (a)).
- `unmatched_echoes`: the sibling README expected a non-zero count from the echo cadence. That expectation has not been checked for this twin.
- CSV empty cells: an empty echo cell means no echo matched that row (`echo_cmd_seq`, `sim_t_s`, the angle and velocity cells, `host_rtt_ms`). `echo_cmd_seq` is also empty when the echo sends a null `cmd_seq_echo`, which is the case before the first command. `vel_rad_s_l` and `vel_rad_s_r` are empty when that wheel's velocity is null, which the controller sends on its first step.
- Webots world: the direction with signs +1/+1 is owner-observed OK (2026-11-06 on the old track, 2026-11-07 on the two-box world with track 10.0 cm, earlier non-physics body, not re-run on the physics-root world, re-run pending #552; see Sign convention). The motor limits (`maxVelocity 3.5`, `maxTorque 1e-3`) are owner-chosen for this simulation, per the world header, and were not checked here.
- Sim only. Not hardware-validated.
