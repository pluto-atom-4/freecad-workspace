# ctrl-wheel

SIM ONLY. Bridge and Webots wheel copy for the ctrl-wheel relay. It reads the `WHL` wheel-velocity line from the Wokwi program `c3_ctrl_wheel` (pluto-atom-4/balancing-robot-controller, `programs/c3_ctrl_wheel`, see `docs/CTRL_WHEEL.md` there), relays it as UDP JSON commands to a velocity-driven Webots wheel, and logs the Webots sensor echo to a CSV.

This is a RELAY, not a closed loop. The firmware receives nothing back. The firmware's `model` value (its lagged model velocity) and the `meas` values (Webots sensor angle and finite-difference velocity) are never mixed.

Not hardware-validated. No balance claim. The wheel sign is UNVERIFIED. Latency is UNMEASURED.

## Status

What ran:

- `pytest` in the `esp32-dof` mamba env, over the seven test modules in this directory: `test_ctrl_wheel_msg.py`, `test_ctrl_wheel_udp.py`, `test_ctrl_wheel_relay.py`, `test_ctrl_wheel_fake.py`, `test_ctrl_wheel_log.py`, `test_ctrl_wheel_bridge.py`, `test_ctrl_wheel_bridge_cli.py`.
- A scratch end-to-end run: `ctrl_wheel_msg` step input, the real `ctrl_wheel_bridge.py` and the other modules here, against a STUB `controller` plant (not Webots). Result: 150 rows, echo velocity 0.500, positive sign. The reported `host_rtt` median of 20.7 ms is the stub's pacing and is NOT a Webots number.
- Headless Wokwi run of `c3_ctrl_wheel`: two runs of 15 s and 12 s gave 608 and 458 WHL lines, all parsed with `ctrl_wheel_msg.parse_line`. The lines were read from stdout, not over RFC2217.

Not run:

- Webots (world, controller, `setPosition(inf)` then `setVelocity`, motor defaults).
- RFC2217 read (`--port rfc2217://localhost:4001`).
- Fall and bus-down states.
- Any hardware.

Open:

- Sign: UNVERIFIED. Only the owner's Webots GUI run settles AC1.
- Latency: UNMEASURED. `host_rtt_ms` is wall time, not link latency.
- The `webots/` directory (controller and world) is added by the separate stacked PRs #460 to #465 (issues #447, #455 to #458); merge those before this README so the rows below describe files that exist. They were checked on that branch (py_compile and a stub-controller smoke test), not in Webots.

## Links

- Plan: pluto-atom-4/balancing-robot-controller discussion 234 (https://github.com/pluto-atom-4/balancing-robot-controller/discussions/234)
- Parents: pluto-atom-4/freecad-workspace#446, pluto-atom-4/balancing-robot-controller#237
- Counterpart program: `programs/c3_ctrl_wheel` in pluto-atom-4/balancing-robot-controller

## Files

| File | Role |
| ---- | ---- |
| `ctrl_wheel_msg.py` | `WHL` line parse, format and step-input generator (`python3 -m ctrl_wheel_msg`). Checksum is XOR of the body, two uppercase hex digits. `RAW_CAP` (125) is applied by `step_lines` only, not by the wire. |
| `ctrl_wheel_udp.py` | JSON datagrams on 127.0.0.1: command to 5008, echo from 5007. Strict decode (exact keys, no duplicates, no NaN or Infinity). Clamp to +-3.0 rad/s. Port 5006 is never used. |
| `ctrl_wheel_relay.py` | Relay rules, no sockets: stale after 200 ms of wall time (not device `t_us`); `state != 0` gives zero and stale; non-finite model gives zero and stale; sign applied before the symmetric clamp. |
| `fake_wheel.py` | Headless stand-in for the Webots controller, tests and scratch runs only. First-order lag is a plant GUESS (`tau_s` 0.05, `dt_s` 0.016). No sensor-failure path: it always echoes. |
| `ctrl_wheel_log.py` | CSV writer, columns in this order: `host_t_s`, `whl_seq`, `whl_t_us`, `whl_model_rad_s`, `whl_raw`, `whl_state`, `cmd_seq`, `cmd_rad_s`, `stale`, `echo_cmd_seq`, `sim_t_s`, `angle_rad`, `vel_rad_s`, `host_rtt_ms`, `missed_whl`. `host_rtt_ms` is wall ms from sending `cmd_seq` to the echo with that `cmd_seq_echo`. It includes up to one simulation step and one bridge poll, so it is not link latency. Empty when no echo arrived. |
| `ctrl_wheel_bridge.py` | Read loop (serial URL, device path or stdin, one command per WHL line, one CSV row per command) and CLI (`build_parser`, `main`). Exit 0 on finish or Ctrl-C; exit 2 on usage error or a failed source, CSV or echo socket. `--sign` is the only sign flip. |
| `webots/controllers/ctrl_wheel/ctrl_wheel.py` | Webots controller. Skeleton from the servo-wheel copy, plus `setVelocity` and the echo. Reads ports 5008 and 5007 only. Sign is a fixed +1. Cannot import `ctrl_wheel_udp`, so the JSON key names are duplicated there on purpose. |
| `webots/controllers/ctrl_wheel/sensor_read.py` | Byte copy of the sensor-read module. The module name also exists in `poc/servo-wheel-dof`. |
| `webots/worlds/ctrl_wheel.wbt` | Webots world. Tick 16 ms. |

Origins of each copy:

- `webots/controllers/ctrl_wheel/ctrl_wheel.py`: from `poc/servo-wheel-dof/webots/controllers/servo_wheel_dof/servo_wheel_dof.py`, stripped to the skeleton, plus `setVelocity` and the echo.
- `webots/controllers/ctrl_wheel/sensor_read.py`: byte copy of the same directory's `sensor_read.py`.
- `webots/worlds/ctrl_wheel.wbt`: from `servo_wheel_dof.wbt`, text edits only (`maxVelocity`, `maxTorque`). The supervisor is untouched.
- RFC2217 open and retry, and the line reader in `ctrl_wheel_bridge.py`: copied from `poc/esp32-dof/monitor/dof_serial.py`. Line references are in the bridge docstring.
- `WHL` checksum and format rules: follow `poc/esp32-dof/monitor/dof_frame.py`.
- No cross-import between directories. `poc/esp32-dof` and `poc/servo-wheel-dof` are untouched.

## Run

Run from `poc/ctrl-wheel`, with the `esp32-dof` env. `pyserial` is needed even for `--stdin`, because the bridge imports `serial` at load time.

Tests:

```
mamba run -n esp32-dof python3 -m pytest -q
```

Do not run one pytest over both `poc/ctrl-wheel` and `poc/servo-wheel-dof`. The module name `sensor_read` exists in both.

AC1 pipeline (stdin, no simulator needed for the bridge itself):

```
python3 -m ctrl_wheel_msg --step 0.5 --seconds 5 | mamba run -n esp32-dof python3 ctrl_wheel_bridge.py --stdin --record ac1.csv
```

RFC2217 (live Wokwi run, port 4001):

```
mamba run -n esp32-dof python3 ctrl_wheel_bridge.py --port rfc2217://localhost:4001 --record ac1.csv
```

Sign: the controller has a fixed +1. Flip the sign ONLY with the bridge `--sign -1`. Never flip both.

## Pre-run checklist

- No listener on UDP 5007 or 5008: `ss -lunp`.
- No listener on TCP 4001 from an earlier run: `ss -ltnp`. A VS Code listener may stay open after a stop.
- No stray sender (old `ctrl_wheel_bridge.py` or `fake_wheel` process).
- Start order: Webots world first, then the bridge, then the simulation.
- Simulator tab visible.
- One bridge instance only.

## Notes

- The bridge summary field `unmatched_echoes` counts every echo that does not complete a pending row: a repeated echo of a command that already completed, an echo that arrives after `--echo-timeout`, and an echo with a null `cmd_seq_echo` (the controller before its first command). The controller is EXPECTED to echo every 16 ms step while commands arrive every 20 ms (Webots not run, so UNVERIFIED), so a non-zero count is expected in the scratch stub run (it gave 64 of 150).
- Rows: one per `WHL` line (a line with state 1 or 2 still gets its own stale command and its own row), plus one row per silence episode. The final stale command sent at shutdown is NOT logged, so `cmds_sent` can exceed the row count by one. `cmd_seq` is an unbounded counter (no wrap).
- Exit codes: 0 clean (EOF, duration or Ctrl-C), 2 for a usage error, an unusable CSV or port, a source that fails mid-run, or a CSV write error. An unreachable URL with `--duration` also returns 2 (nothing was opened).
- `pyserial` is imported at module load, so the bridge needs the esp32-dof env even with `--stdin`.
- The controller reads ports 5008 and 5007 only. Port 5006 belongs to `servo_wheel_dof`.
- The Webots tick is 16 ms in the world file.
- Test counts are not fixed here. Run pytest to see the current count.
