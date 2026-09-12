# Stage A: Webots World Scaffolding (URDF Import + PROTO)

Issue: [#103](https://github.com/pluto-atom-4/freecad-workspace/issues/103) — Sub-issue of [#10](https://github.com/pluto-atom-4/freecad-workspace/issues/10) (Webots simulation pipeline).

## Overview

Stage A sets up a minimal Webots simulation environment:
- URDF import from the inverted pendulum robot design (`06_Exports/urdf/robot.urdf`).
- PROTO generation via `urdf2webots` for Webots asset representation.
- Webots world with plain offline nodes (Background, DirectionalLight, floor) — no networked PROTOs.
- Simple no-op `"<none>"` controller for the robot (real controllers come in later stages).

**Goal:** Provide a visual validation sandbox. A human can load the world in the Webots GUI and confirm the robot renders correctly (base plate, wheels, pendulum arms with servo mesh, correct scale).

## What's Included

```
07_Simulation/
├── webots/
│   ├── prepare_urdf_for_webots.sh    # Rewrite URDF package:// URIs → relative paths
│   ├── generate_proto.sh               # Generate InvertedPendulumRobot.proto via urdf2webots
│   ├── run_gui.sh                      # Launch Webots GUI (realtime, needs DISPLAY + human visual check)
│   ├── run_batch.sh                    # Headless smoke test (structural validation only)
│   ├── protos/                         # Generated PROTO files (gitignored)
│   ├── worlds/
│   │   └── pendulum_robot.wbt          # Stage A world (plain offline Background/Light/floor)
│   └── .generated/                     # Temporary generated artifacts (gitignored)
└── README.md                           # This file
```

## What's NOT Included (Out of Scope)

- **Physics/control simulation:** Stage A has a no-op `"<none>"` controller. Real controllers and physics tuning come later.
- **URDF mesh-path resolution:** The source URDF (`06_Exports/urdf/robot.urdf`) uses `package://inverted_pendulum_robot/meshes/...` URIs that `urdf2webots` cannot resolve. This stage's `prepare_urdf_for_webots.sh` script rewrites them to relative paths in a `.generated/` copy — the source URDF is never modified (that's issue #9 territory).
- **Networked standard-library PROTOs:** Stage A uses plain Webots nodes (Background, DirectionalLight, Solid) for a fully offline world. Future stages may add networked PROTOs (TexturedBackground, etc.) for visual polish.

## How to Run

### GUI Mode (Human Visual Validation)

```bash
cd inverted-pendulum-project/07_Simulation/webots/
export DISPLAY=:1  # Set to your active X display
./run_gui.sh
```

Expected behavior:
- Script auto-generates `protos/InvertedPendulumRobot.proto` if missing.
- Webots GUI opens with the robot visible.
- Robot should show:
  - A thin ~2.5mm base plate (flat, narrow).
  - Two wheels offset ~54mm above the base (correct per design, not a bug).
  - Two pendulum arms extending upward.
  - Servo motor mesh visible on each arm (the STL imported from CAD).
  - Correct scale relative to the floor (floor is 1m × 1m).
  - No missing geometry, no pink error materials, no visual warnings.

**⚠️ HARD MERGE GATE:** The human must visually confirm the above before this PR can be merged. Do not rely on any automated test to claim visual correctness — that's a visual inspection task only a human with Webots open can perform.

### Headless Smoke Test (Automated Structural Validation)

```bash
cd inverted-pendulum-project/07_Simulation/webots/
./run_batch.sh
```

Expected behavior:
- Script auto-generates the PROTO if needed.
- Webots runs in batch mode (no GUI, no rendering).
- Webots exits 0 if initialization succeeds (PROTO resolved, world loaded, no crash).

**Important:** This smoke test only validates the URDF/PROTO pipeline structure. It does **NOT** prove the robot looks correct visually or that physics behaves correctly. Those require human GUI inspection.

## The package:// URI Rewrite Problem

The source URDF references meshes via `package://inverted_pendulum_robot/meshes/...`, which assumes a ROS environment with `ROS_PACKAGE_PATH` set. Since `urdf2webots` runs outside ROS (just Python, no ROS), it cannot resolve those paths.

**Solution:** `prepare_urdf_for_webots.sh` copies `robot.urdf` to `.generated/robot_webots.urdf`, rewriting all `package://inverted_pendulum_robot/` → `../../../06_Exports/urdf/`. The relative path is from `.generated/` back to the actual mesh location. After rewrite, `urdf2webots` can resolve the relative mesh paths correctly.

**Note:** The source `06_Exports/urdf/robot.urdf` is never modified. This keeps issue #9 (URDF format/mesh paths) separate from this stage's PROTO generation task.

## Implementation Details

### prepare_urdf_for_webots.sh
- Validates source URDF and meshes exist and are non-empty.
- Copies URDF and rewrites `package://` URIs to relative paths.
- Fails loudly if any `package://` substring remains after rewrite.

### generate_proto.sh
- Calls `prepare_urdf_for_webots.sh`.
- Runs `mamba run -n pendulum-tools python3 -m urdf2webots.importer` with target R2025a.
- Validates output PROTO exists and is non-empty.
- **Validates** urdf2webots output for link/joint counts as a required sanity check (5 links, 4 joints expected; exits FATAL if mismatch).

### run_gui.sh / run_batch.sh
- Validate world file and WEBOTS_BIN.
- Auto-invoke `generate_proto.sh` if PROTO is missing or source URDF is newer.
- Launch Webots (GUI or batch respectively).

### pendulum_robot.wbt
- Hand-authored, committed.
- EXTERNPROTO references the generated PROTO.
- Plain Webots nodes: WorldInfo, Viewpoint, Background, DirectionalLight, Solid floor.
- Robot instance with `controller "<none>"` (built-in no-op — no controller code yet).
- Robot positioned with clearance above floor so it can settle under gravity.

## Troubleshooting

### "WEBOTS_BIN not found"
```bash
# Find your Webots binary (e.g. from AppImage or system install)
export WEBOTS_BIN=/path/to/webots
./run_gui.sh
```

### "mamba not found"
Ensure mamba is installed and on PATH. This is required to run `urdf2webots` in the `pendulum-tools` environment.

### "package:// URI found in output"
`prepare_urdf_for_webots.sh` failed the rewrite. Check that the `sed` pattern in the script matches the actual URDF paths.

### "PROTO generation succeeded, but robot is missing/invisible"
- Confirm the `.proto` file exists and is non-empty: `ls -lh protos/InvertedPendulumRobot.proto`
- Check the Webots console for errors (EXTERNPROTO resolution, mesh loading failures).
- If the PROTO file is stale, delete it and re-run `generate_proto.sh`.

### "Webots GUI shows pink error materials"
Likely a mesh path issue. Check:
1. Does `.generated/robot_webots.urdf` have correct relative paths? (should be `../../../06_Exports/urdf/meshes/...`)
2. Do the mesh files exist at that location relative to `.generated/`?

## Environment

- **OS:** Linux (tested on Debian/Ubuntu).
- **Webots:** R2025a or compatible (headless + GUI both supported).
- **Python env:** `pendulum-tools` mamba environment (includes urdf2webots).
- **DISPLAY:** Required for GUI mode; batch mode falls back to `xvfb-run` if available.

## References

- GitHub Issue: [#103](https://github.com/pluto-atom-4/freecad-workspace/issues/103)
- Parent Epic: [#10](https://github.com/pluto-atom-4/freecad-workspace/issues/10)
- Source URDF: `06_Exports/urdf/robot.urdf`
- POC reference: `poc/freecad-webots-pipeline/` (TurtleBot3 simulation pipeline)
