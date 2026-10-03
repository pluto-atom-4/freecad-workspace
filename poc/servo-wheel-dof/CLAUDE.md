# CLAUDE.md -- servo-wheel-dof POC (issue #258)

Guidance for Claude Code in this directory. Inherits `../../CLAUDE.md` (mamba-only).

## Commands

```bash
cd poc/servo-wheel-dof
mamba run -n servo-wheel-dof python3 -m pytest -q monitor webots/controllers/servo_wheel_dof
DRY_RUN=1 ./run_demo.sh --headless --mock    # needs WEBOTS_BIN to exist
./run_demo.sh --gui --mock                   # needs DISPLAY + Webots
```

Env: `servo-wheel-dof`. Create it with
`mamba env create -n servo-wheel-dof -f poc/servo-wheel-dof/mamba-envs.lock.yml`
(the custom-schema `mamba-envs.yaml` cannot be used with `-f`).

## Conventions

- **`.wbt` is edited as text only.** Never use Webots "Save World": it rewrites the
  file and drops the header comments. Keep the header sign convention and the
  README in sync.
- **`controller` is imported only in `servo_wheel_dof.py`.** Keep parsing, UDP and
  pose maths in the pure modules. Never import `controller` in a test.
- **Unique test basenames.** One pytest run covers `monitor/` and the controller
  dir: name new files `test_swd_<module>.py`, never reuse an existing basename.
- **No pyserial in Webots' Python.** The publisher and controller use stdlib only;
  nothing in this POC reads serial.
- Copied modules (`imu_wheel_msg.py`, `imu_udp_latest.py`, `imu_euler_math.py`)
  are hand-synced with `poc/esp32-dof`; mirror fixes by hand.
- Wire facts (UDP JSON, port 5006, radians, ZYX) live in `README.md`; keep code
  and README in sync.
- Do not hard-code test counts or user-specific paths in docs.
- **Telemetry stays in pure modules** (`telemetry.py`, `alignment_metrics.py`,
  `overlay_text.py`) and must never raise or stall the sim: read-only Supervisor calls,
  fail-soft, off by default. Keep the README column list equal to `telemetry.COLUMNS`
  and the WARN thresholds equal to `alignment_metrics` (`DEV_WARN_M`, `DOT_WARN`).

## Validation

- Only a human ticks verification boxes (issue #289). Headless runs prove only
  that the controller starts, receives UDP and computes poses.
- If a rotation sign looks wrong, fix `upright_pose` / the world with a unit test;
  no guessing.
- After editing `run_demo.sh` run `bash -n run_demo.sh` (and `shellcheck` if present).

## Gotchas

- Port 5006 is bound without SO_REUSEADDR; a second instance exits 1. The
  esp32-dof follower uses 5005.
- The simulation must be running (not paused) or the controller is not stepped.
- `run_demo.sh --mock` stops only the Webots process group it started.
- `mamba run` may buffer output; `run_demo.sh --mock` resolves the python once.
- Telemetry: `SWD_TELEMETRY=1` enables it; `SWD_TELEMETRY_FILE` alone does not. Use an
  absolute file path. Pose columns are `nan` without the wheel node. `SWD_OVERLAY=1`
  needs telemetry and is GUI-only (human check). Read a CSV with
  `python3 monitor/telemetry_summary.py <csv>`; details in README "Telemetry".
