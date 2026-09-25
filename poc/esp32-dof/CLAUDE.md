# CLAUDE.md -- ESP32 6-DOF IMU POC (issue #227)

Project-specific guidance for Claude Code when working in this directory. Inherits the workspace root's `../../CLAUDE.md` (mamba-only, no uv).

## Commands

```bash
# Tests (pure Python, no hardware, no display) -- from poc/esp32-dof
cd poc/esp32-dof
mamba run -n esp32-dof python3 -m pytest -q monitor webots/controllers/esp32_dof_follower

# Mock demo (needs DISPLAY + Webots) and Webots GUI alone
./run_mock_demo.sh [--duration N]
./webots/run_gui.sh            # WEBOTS_BIN overrides /usr/local/bin/webots
DOF_FOLLOWER_DEBUG=1 ./webots/run_gui.sh   # controller prints applied rotations

# Monitor
mamba run -n esp32-dof python3 monitor/dof_monitor.py --list-ports
mamba run -n esp32-dof python3 monitor/dof_monitor.py --mock --fuse --duration 3
mamba run -n esp32-dof python3 monitor/dof_monitor.py --port /dev/ttyACM0 --publish
```

Create the env with `mamba env create -n esp32-dof -f poc/esp32-dof/mamba-envs.lock.yml` (the custom-schema `mamba-envs.yaml` cannot be used with `-f`).

## Conventions

- **Unique test basenames.** All tests live in one pytest run across two directories, so every test file needs a globally unique basename: `test_esp32dof_<module>.py`. Never add a `test_*.py` whose basename already exists anywhere in this POC (pytest's default import mode would raise "import file mismatch").
- **`controller` is imported only in `webots/controllers/esp32_dof_follower/esp32_dof_follower.py`.** The Webots `controller` module does not exist outside Webots; keep all parsing, math and socket logic in `dof_webots_math.py` / `dof_udp_latest.py` so it stays unit-testable. Do not import `controller` in any test.
- Monitor modules are flat (`dof_*.py` import each other by name); run scripts by path or with `monitor/` on `sys.path`.
- Wire protocol facts (frame format, UDP JSON, ZYX Euler radians) are defined in `README.md`; keep code and README in sync.
- Do not hard-code test counts or user-specific paths (e.g. arduino-cli location) in docs.

## Validation workflow

- Prefer a human watching the Webots GUI (`./run_mock_demo.sh`) over headless batch runs for anything about how the box looks or which way it rotates. A headless run only proves the controller starts, receives UDP, and computes rotations.
- The firmware has been compiled but never flashed, and the GUI look / rotation handedness are unverified by automation. Do NOT claim the README's "Manual verification checklist (human)" items are done; only a human ticks them.
- If a rotation sign is wrong, fix `euler_to_axis_angle` / `setSFRotation` with a unit test; no guessing.
- After editing a shell script run `bash -n <script>` (and `shellcheck` if installed).

## Gotchas

- Port 5005 is bound by the controller without SO_REUSEADDR: a second Webots instance exits with "cannot bind UDP".
- The simulation must be running (not paused) or the controller is not stepped.
- `DOF_FOLLOWER_DEBUG=1` must be in Webots' own environment (inherited when launched via `run_gui.sh`).
- `mamba run` may buffer output; `run_mock_demo.sh` resolves the env's python once and runs it directly.
