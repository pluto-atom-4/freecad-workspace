# TurtleBot3 POC Simulation with Webots: Integration Guide

## Overview

This document describes the integrated architecture of the TurtleBot3 Point-of-Concept (POC) simulation pipeline, which demonstrates how robot models are converted from URDF (Unified Robot Description Format) to Webots PROTO (Prototype) format and executed in a simulation environment.

The POC consists of three key components:
1. **URDF Definition** (`urdf/turtlebot3_poc.urdf`) — Robot structure and physics parameters
2. **Webots PROTO** (`webots/protos/TurtlebotPoc.proto`) — Converted Webots model definition
3. **Webots World** (`webots/worlds/turtlebot3_poc.wbt`) — Simulation environment and controller setup

---

## Part 1: File Descriptions

### 1.1 URDF Definition (`urdf/turtlebot3_poc.urdf`)

**Purpose:** Defines the robot structure, links, joints, and physics properties in standard URDF format (XML).

**Key Elements:**

- **Robot Name:** `turtlebot3_poc`
- **Materials:** Defines two materials (`light_black` and `dark`) for visual rendering

**Structure (Link & Joint Hierarchy):**

```
base_link (root link, robot body, 825g)
    ├─ wheel_left_link (28.5g, continuous joint)
    │   └─ left_tire_roundtrip.stl mesh (0.001 scale = mm→m conversion)
    ├─ wheel_right_link (28.5g, continuous joint)
    │   └─ right_tire_roundtrip.stl mesh (0.001 scale = mm→m conversion)
    └─ caster_back_link (fixed joint, 5g, no visual mesh)
        └─ collision box only
```

**Critical Details:**

- **Mesh Files:** Point to FreeCAD STEP-round-tripped re-exports:
  - `meshes/burger_base_roundtrip.stl` (4.6 MB)
  - `meshes/left_tire_roundtrip.stl` (1.1 MB)
  - `meshes/right_tire_roundtrip.stl` (1.1 MB)

- **Scale Factor:** All meshes use `0.001 0.001 0.001` to convert millimeters→meters (FreeCAD exports in mm)

- **Wheel Joints:** Type `continuous` (unbounded rotation) at:
  - Left: origin `xyz="0.0 0.08 0.023"` rpy=`"-1.57 0 0"` (90° rotation)
  - Right: origin `xyz="0.0 -0.080 0.023"` rpy=`"-1.57 0 0"`
  - Axis: `xyz="0 0 1"` in local joint frame (transformed to world Y-axis by rpy)

- **Caster:** Fixed rear contact point (no rotation), positioned at `xyz="-0.081 0 -0.004"`

- **Inertial Properties:** Mass, center-of-mass, and 6D inertia tensor for each link

### 1.2 Webots PROTO (`webots/protos/TurtlebotPoc.proto`)

**Purpose:** Defines the robot as a reusable PROTO object for instantiation in Webots worlds. Generated from URDF via `urdf2webots` conversion.

**PROTO Definition Fields:**

```proto
field SFVec3f     translation     0 0 0          # Initial position
field SFRotation  rotation        0 0 1 0        # Initial orientation
field SFString    name            "TurtlebotPoc" # Robot name
field SFString    controller      "void"         # Controller program
field MFString    controllerArgs  []             # Controller arguments
field SFString    customData      ""             # Custom metadata
field SFBool      supervisor      FALSE          # Supervisor mode flag
field SFBool      synchronization TRUE           # Timing sync flag
field SFBool      selfCollision   FALSE          # Self-collision detection
```

**Structure (Webots Hierarchy):**

The PROTO wraps a **Robot** node containing:

1. **Base Link (Solid)**
   - Visual: Transform with burger_base mesh at scale 0.001
   - Collision: Box (0.14×0.14×0.143 m)
   - Physics: Mass 0.8257 kg, inertia tensor

2. **Left Wheel (HingeJoint)**
   - Joint axis: `0 1 0.000796` (approximately world Y-axis)
   - Anchor: `0 0.08 0.023`
   - Devices:
     - `RotationalMotor` named `wheel_left_joint` (maxTorque 10000)
     - `PositionSensor` named `wheel_left_joint_sensor`
   - Solid: wheel_left_link with tire mesh + collision cylinder

3. **Right Wheel (HingeJoint)**
   - Joint axis: `0 1 0.000796`
   - Anchor: `0 -0.08 0.023`
   - Devices:
     - `RotationalMotor` named `wheel_right_joint`
     - `PositionSensor` named `wheel_right_joint_sensor`
   - Solid: wheel_right_link with tire mesh + collision cylinder

4. **Caster (Fixed Solid)**
   - Translation: `-0.081 0 -0.004`
   - Collision: Box only (no visual)
   - Physics: Mass 0.005 kg

**Key Conversion Details:**

- URDF `<joint>` → Webots `HingeJoint` (for continuous rotation)
- URDF `<link>` → Webots `Solid` node
- URDF `<collision>` → Webots `boundingObject` (Physics)
- URDF mesh paths adjusted to relative paths: `../../urdf/meshes/`
- Materials converted to `PBRAppearance` (Physical-Based Rendering)

### 1.3 Webots World (`webots/worlds/turtlebot3_poc.wbt`)

**Purpose:** Instantiates the TurtlebotPoc PROTO in a simulated environment with physics and control.

**World Configuration:**

| Component | Value | Purpose |
|-----------|-------|---------|
| **BasicTimeStep** | 32 ms | Simulation step duration |
| **Viewpoint** | orientation -0.3 0.3 0.9 1.2, position -0.6 -0.96 0.6 | Camera view |
| **TexturedBackground** | (Webots library PROTO, EXTERNPROTO) | Sky/environment texture |
| **TexturedBackgroundLight** | (Webots library PROTO, EXTERNPROTO) | Matching environment lighting |
| **RectangleArena** | floorSize 2 2, floorTileSize 0.2 0.2 (EXTERNPROTO) | Ground plane/floor |
| **TurtlebotPoc Instance** | translation 0 0 0.03, supervisor TRUE, controller "wheel_articulation_check" | Robot placement & control |

**Environment Setup:**

- Uses three Webots-library EXTERNPROTOs (issue #28): `TexturedBackground`, `TexturedBackgroundLight`, `RectangleArena`, all resolved from `cyberbotics/webots` R2025a on GitHub, in addition to the local `TurtlebotPoc.proto` EXTERNPROTO. This means the **first** Webots launch after a fresh checkout needs network access to resolve these three EXTERNPROTOs from `raw.githubusercontent.com`; Webots caches them locally afterward, so subsequent runs (including CI/headless) work offline.
- Floor is a `RectangleArena { floorSize 2 2  floorTileSize 0.2 0.2 }` — a flat, wall-less 2×2 m tiled plane at z=0 (no hand-rolled `Solid` box, no negative-z offset).
- Per this POC's `CLAUDE.md`, prefer a human visually confirming simulation behavior live via `run_gui.sh` over trusting `run_batch.sh`'s headless PASS/FAIL alone whenever a physics/kinematics-affecting change has been made — see Part 3.3 below.

---

## Part 2: Integration Flow

### 2.1 How They Work Together

```
┌─────────────────────────────────────────────────────────────┐
│  URDF: turtlebot3_poc.urdf                                  │
│  ├─ Defines robot structure (links, joints, meshes)         │
│  └─ References FreeCAD-generated meshes (mm scale)          │
└────────────────┬────────────────────────────────────────────┘
                 │
        urdf2webots converter
                 │
                 ↓
┌─────────────────────────────────────────────────────────────┐
│  PROTO: webots/protos/TurtlebotPoc.proto                    │
│  ├─ Converts URDF structure to Webots node hierarchy        │
│  ├─ Creates HingeJoints with RotationalMotors & Sensors     │
│  ├─ Adjusts mesh paths to Webots conventions                │
│  └─ Wraps Robot node with configurable fields              │
└────────────────┬────────────────────────────────────────────┘
                 │
         Instantiation in World
                 │
                 ↓
┌─────────────────────────────────────────────────────────────┐
│  WORLD: webots/worlds/turtlebot3_poc.wbt                    │
│  ├─ Defines simulation environment                          │
│  ├─ Instantiates TurtlebotPoc PROTO                         │
│  ├─ Attaches wheel_articulation_check controller            │
│  └─ Runs physics simulation                                 │
└────────────────┬────────────────────────────────────────────┘
                 │
                 ↓
┌─────────────────────────────────────────────────────────────┐
│  CONTROLLER: wheel_articulation_check.py                    │
│  ├─ Gets device handles (wheel motors & sensors)           │
│  ├─ Sets wheel velocity (2.0 rad/s both wheels)            │
│  ├─ Runs simulation for 300 steps (~9.6 seconds)            │
│  ├─ Prints telemetry every 25 steps                         │
│  └─ Verifies wheel rotation (PASS/FAIL verdict)            │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 Data Flow During Simulation

1. **Initialization:**
   - Webots loads world file
   - Instantiates TurtlebotPoc PROTO with base_link, wheels, caster
   - Loads wheel_articulation_check controller
   - Physics engine initialized with inertia & collision properties

2. **Simulation Loop (each 32 ms step):**
   - Controller gets device handles: `robot.getDevice("wheel_left_joint")`
   - Controller sets velocity: `motor.setVelocity(2.0 rad/s)`
   - Physics engine simulates dynamics:
     - Computes wheel torques from motor commands
     - Updates joint positions via ODE/Bullet solver
   - Position sensors read joint angles
   - Telemetry printed every 25 steps

3. **Termination (after 300 steps):**
   - Compare initial vs. final wheel joint positions (PASS requires `abs(delta) >= 1e-4` rad on **both** wheels)
   - Compare initial vs. final chassis position via `Supervisor.getSelf().getPosition()` (PASS requires `abs(dx) > 1e-3` or `abs(dy) > 1e-3`) — added in issue #32 specifically because the wheel-angle check alone cannot distinguish "chassis translating normally" from "wheels spinning in place while the chassis is welded to the static world" (the exact failure mode issue #32 fixed)
   - Overall verdict requires **both** checks to pass; print `VERDICT: PASS` or `VERDICT: FAIL/INCONCLUSIVE`
   - Exit with code 0 (success) or 1 (failure/inconclusive)

---

## Part 3: Practical Simulation Guide

### 3.1 Prerequisites

**Required Software:**

- **Webots 2025a** (or compatible)
- **Python 3** (Webots runtime includes its own Python for controllers)
- **urdf2webots** (if regenerating PROTO from URDF)

**Webots Installation:**

```bash
# On Linux:
# Download from https://cyberbotics.com/download or package manager
sudo apt install webots  # Ubuntu/Debian

# Verify installation:
webots --version
```

**Directory Structure:**

```
freecad-webots-pipeline/
├── urdf/
│   ├── turtlebot3_poc.urdf
│   └── meshes/
│       ├── burger_base_roundtrip.stl
│       ├── left_tire_roundtrip.stl
│       └── right_tire_roundtrip.stl
├── webots/
│   ├── protos/
│   │   └── TurtlebotPoc.proto
│   ├── worlds/
│   │   └── turtlebot3_poc.wbt
│   └── controllers/
│       └── wheel_articulation_check/
│           └── wheel_articulation_check.py
└── findings/
    └── FINDINGS.md
```

### 3.2 Running the Simulation (GUI Mode)

**Step 1: Open Webots**

```bash
webots &
```

**Step 2: Open the World File**

- File → Open World
- Navigate to `webots/worlds/turtlebot3_poc.wbt`
- Click Open

**Step 3: Run the Simulation**

- Click the Play button (▶) in the toolbar
- Observe:
  - Robot positioned on the floor
  - Wheels rotating smoothly
  - Telemetry printed to console every 1-2 seconds
  - Simulation runs for ~9.6 seconds (300 steps × 32 ms)

**Expected Output (Console):**

```
Found left motor device:  wheel_left_joint
Found right motor device: wheel_right_joint
  t= 0.80s step= 25  left pos=1.5995 rad observed_vel=1.9993 rad/s  |  right pos=1.5995 rad observed_vel=1.9993 rad/s
  t= 1.60s step= 50  left pos=3.1989 rad observed_vel=1.9993 rad/s  |  right pos=3.1989 rad observed_vel=1.9993 rad/s
  ...
  t= 9.60s step=300  left pos=19.1935 rad observed_vel=1.9993 rad/s  |  right pos=19.1935 rad observed_vel=1.9993 rad/s

Ran 300 simulation steps at 32ms each (9.60s simulated).
Commanded velocity: 2.0 rad/s on both wheels.
Chassis position: [1.1965216613138842e-06, -4.548118152253259e-09, 0.01995493542150951] -> [0.6274468493538085, 5.889092309206335e-08, 0.009646724583900543] (dx=0.6274, dy=0.0000)
Left wheel joint position:  0.0640 -> 19.2574 rad (delta 19.1934)
Right wheel joint position: 0.0640 -> 19.2574 rad (delta 19.1934)
PASS: both wheel joints rotated under commanded velocity.
PASS: chassis translated under commanded wheel velocity.

VERDICT: PASS
```

**Note:** the `observed_vel` figures are a *live* measurement (position-sensor delta since the previous print), not the commanded-velocity constant. The `Chassis position: ... dx=... dy=...` line (via `Supervisor.getSelf().getPosition()`) was added in issue #32 and is required, in addition to the wheel-position check, for an overall `PASS` — see Part 2.2 above.

**Step 4: Verify the Result**

- Final verdict should show `PASS`
- Both wheel position deltas should be ~19.2 rad (300 steps × 32 ms = 9.60 s simulated, not 10 s — the wheel angle is not exactly `20.0000 rad` because the initial reading is taken one warm-up step in, at `t≈0.03s`, not `t=0`); the chassis `dx`/`dy` should show real translation (nonzero `dx`, ~0 `dy` for straight-line forward motion)

### 3.3 Running in Batch Mode (Headless)

**Purpose:** Run simulation without GUI for automated testing/CI/CD

> **Validation policy note (this POC's `CLAUDE.md`):** batch mode's `VERDICT: PASS`/exit-code result is a useful structural/regression smoke signal (confirms EXTERNPROTO resolution, confirms the controller doesn't crash), but it is **not** sufficient on its own to confirm correct *physical* behavior after a physics/kinematics-affecting change. This exact gap is how the issue #32 bug shipped unnoticed: the original controller's batch-mode check only validated wheel-joint rotation and reported `PASS` while the chassis was completely welded to the static world. Prefer `./webots/run_gui.sh` with a human watching the simulation live for any change touching the URDF, PROTO regeneration, or world physics; reserve batch mode for CI-style smoke checks or when a human has explicitly asked for headless validation.

**Command:**

```bash
webots --batch webots/worlds/turtlebot3_poc.wbt --output=output.log
```

**Options Explained:**

- `--batch` — Disable GUI, run headless
- `--output=output.log` — Redirect stdout/stderr to file
- Add `--stop-time=10s` to set max runtime
- Add `--quiet` to suppress debug messages

**Check Results:**

```bash
cat output.log | grep "VERDICT"
echo $?  # Should be 0 for PASS, 1 for FAIL
```

### 3.4 Modifying the Simulation

#### 3.4.1 Change Wheel Velocity

Edit `webots/controllers/wheel_articulation_check/wheel_articulation_check.py`:

```python
# Line ~30: Change target velocity
TARGET_VELOCITY_RAD_S = 3.0  # Instead of 2.0
```

Re-run the simulation. Wheels should spin faster, travel farther in the same time.

#### 3.4.2 Change Simulation Duration

```python
# Line ~34: Change step count
SIM_STEPS = 600  # Instead of 300 (20 seconds instead of 10)
```

#### 3.4.3 Change Print Frequency

```python
# Line ~38: Print every N steps
PRINT_EVERY_STEPS = 50  # Instead of 25 (half as frequent)
```

#### 3.4.4 Modify Robot Initial Pose

Edit `webots/worlds/turtlebot3_poc.wbt`:

```wbt
TurtlebotPoc {
  translation 0.5 0.5 0.03  # Move to (0.5, 0.5) instead of origin — 0.03 is the baseline spawn z (issue #32; base_link's own frame, not the old base_footprint-relative 0.02)
  rotation 0 0 1 1.57        # Rotate 90° around Z-axis
  controller "wheel_articulation_check"
  supervisor TRUE            # Required for the controller's chassis-position (getSelf().getPosition()) check — see Part 2.2
}
```

#### 3.4.5 Add Environmental Obstacles

Add a box obstacle to `turtlebot3_poc.wbt`:

```wbt
Solid {
  translation 0.3 0 0.05
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 1 0 0  # Red
      }
      geometry Box {
        size 0.1 0.1 0.1
      }
    }
  ]
  boundingObject Box {
    size 0.1 0.1 0.1
  }
  physics Physics {
    density 500  # Heavy obstacle
  }
}
```

### 3.5 Hand-Crafted Demo Dataset

Four scenarios exist as complete, already-committed world + controller pairs (scenarios 2–4 were added this session; each is a real file, not a hypothetical edit walkthrough):

| Scenario | World file | Controller |
|----------|-----------|------------|
| 1. Forward Motion | `webots/worlds/turtlebot3_poc.wbt` | `wheel_articulation_check` |
| 2. Differential Steering | `webots/worlds/scenario_02_differential_steering.wbt` | `scenario_02_differential_steering` |
| 3. In-Place Rotation | `webots/worlds/scenario_03_inplace_rotation.wbt` | `scenario_03_inplace_rotation` |
| 4. Collision | `webots/worlds/scenario_04_collision.wbt` | `scenario_04_collision` |

Each world can be run either via GUI (`./webots/run_gui.sh webots/worlds/<world>.wbt`), headless (`./webots/run_batch.sh webots/worlds/<world>.wbt`), or directly (`webots webots/worlds/<world>.wbt`). All four worlds share the same `TexturedBackground`/`TexturedBackgroundLight`/`RectangleArena` environment setup described in Part 1.3, and spawn `TurtlebotPoc` at `translation 0 0 0.03` with `supervisor TRUE`.

> **Note:** `scenario_04_collision.py` was refactored to use a `Supervisor` with chassis x-position telemetry (`Supervisor.getSelf().getPosition()`) for its collision/plateau detection — the same issue #32 pattern used by `wheel_articulation_check.py` (see Scenario 4 below). Its wheel-joint telemetry is now recorded only as diagnostic output and no longer drives the PASS/FAIL verdict. `scenario_02_differential_steering.py` and `scenario_03_inplace_rotation.py` still use a plain `Robot()` with no `Supervisor`/chassis-position check — their PASS/FAIL remains based solely on wheel-joint-position telemetry, the same pattern that let the issue #32 chassis-welded bug ship unnoticed. Treat their PASS verdicts as confirming wheel articulation only, not confirmed chassis motion, until they're extended with the same check.

#### Scenario 1: Forward Motion (Baseline)

**Configuration:**

- Robot at origin, wheels spin forward
- Duration: ~9.6 s simulated (300 steps × 32 ms)
- Expected result: Robot moves forward in straight line

**Demo:**

```bash
# Use default configuration (as described in 3.2)
webots webots/worlds/turtlebot3_poc.wbt
```

**Expected Telemetry** (real captured values — see Part 3.2):

```
  t= 1.60s step= 50   left pos=3.1989 rad   right pos=3.1989 rad
  t= 3.20s step= 100  left pos=6.3978 rad   right pos=6.3978 rad
  t= 6.40s step= 200  left pos=12.7956 rad  right pos=12.7956 rad
  t= 9.60s step= 300  left pos=19.1935 rad  right pos=19.1935 rad
```

**Behavior:** Straight-line forward motion (equal wheel velocities); confirmed via both the wheel-position delta and the chassis `dx`/`dy` translation check (issue #32).

---

#### Scenario 2: Differential Steering

**Files:**
- World: `webots/worlds/scenario_02_differential_steering.wbt`
- Controller: `webots/controllers/scenario_02_differential_steering/scenario_02_differential_steering.py`

**Configuration (from the actual controller):**

```python
V_LEFT = 2.0    # rad/s (faster — left wheel)
V_RIGHT = 1.0   # rad/s (slower — right wheel)
```

Left wheel is commanded faster than the right, so per differential-drive kinematics the robot curves toward the slower (right) side. The controller's own docstring states the expected result explicitly: "Robot curves to the right in a smooth arc."

**Run:**

```bash
webots webots/worlds/scenario_02_differential_steering.wbt
# or headless:
./webots/run_batch.sh webots/worlds/scenario_02_differential_steering.wbt
```

**Telemetry:** printed roughly every 2 simulated seconds as `T=<t>s | L_pos=<rad> | R_pos=<rad>`, for up to `max_time = 10.0` seconds.

**Verdict logic (plain `Robot`, wheel-position telemetry only):** PASS requires the left wheel's rotation delta to exceed the right's, the ratio of left/right deltas to be within ±0.2 of the commanded 2.0 ratio, and both wheel positions to increase monotonically (no reversal) — it does **not** check chassis position/heading.

---

#### Scenario 3: In-Place Rotation

**Files:**
- World: `webots/worlds/scenario_03_inplace_rotation.wbt`
- Controller: `webots/controllers/scenario_03_inplace_rotation/scenario_03_inplace_rotation.py`

**Configuration (from the actual controller):**

```python
V_LEFT = 2.0     # rad/s (forward)
V_RIGHT = -2.0   # rad/s (backward, opposite direction)
```

Left wheel commanded forward, right wheel commanded backward at equal magnitude — the controller's docstring describes the expected result as "360° rotation without translation" (~360° every ~6.28 s), with X/Y position expected to stay near the origin.

**Run:**

```bash
webots webots/worlds/scenario_03_inplace_rotation.wbt
# or headless:
./webots/run_batch.sh webots/worlds/scenario_03_inplace_rotation.wbt
```

**Verdict logic (plain `Robot`, wheel-position telemetry only):** PASS requires the left wheel's delta to be positive and the right wheel's delta to be negative, the sum of the two deltas to be near zero (±0.2 rad, i.e. symmetric magnitudes), and each wheel to have rotated at least `2π` rad — it does **not** independently confirm the chassis stayed near the origin (no `Supervisor`/position check).

---

#### Scenario 4: Collision

**Files:**
- World: `webots/worlds/scenario_04_collision.wbt`
- Controller: `webots/controllers/scenario_04_collision/scenario_04_collision.py`

**Obstacle (from the actual world file):** a static `Solid` named implicitly by its position — `translation 0.5 0 0.05`, `rotation 0 0 1 0`, `geometry Box { size 0.1 0.5 0.1 }` (0.1 m × 0.5 m × 0.1 m, spanning the robot's path), `appearance PBRAppearance { baseColor 0.8 0.2 0.2 }` (dark red), `boundingObject Box { size 0.1 0.5 0.1 }`, `physics Physics { density 1000 }`. It sits 0.5 m ahead of the robot's spawn point along +X.

**Configuration (from the actual controller):** same forward-motion command as Scenario 1 — no obstacle-avoidance logic:

```python
V_LEFT = 2.0    # rad/s (forward)
V_RIGHT = 2.0   # rad/s (forward)
```

**Run:**

```bash
webots webots/worlds/scenario_04_collision.wbt
# or headless:
./webots/run_batch.sh webots/worlds/scenario_04_collision.wbt
```

**Collision-detection logic (refactored, `8d49a1e`):** the controller is now a `Supervisor` and watches the chassis's own x-position (`self_node.getPosition()[0]`, via `robot.getSelf()`) rather than wheel-joint rotation — this mirrors the `wheel_articulation_check.py` issue #32 fix, since wheel-encoder angle keeps climbing under a velocity-controlled motor regardless of whether the chassis is physically blocked, so an angle-based plateau check could never reliably fire. Once at least 10 chassis-position samples have been recorded, if the chassis x-position changed by less than `0.001` m (1 mm) over the last 5 samples, that moment is recorded as `collision_detected_time` (wheel position at that instant is also captured as `collision_detected_rotation`, but purely as a diagnostic annotation). Telemetry prints roughly every 2 simulated seconds as:

`T=<t>s [MOVING|COLLISION] | chassis_x=<m> m | dx(5steps)=<m> m | L_pos=<rad> rad | R_pos=<rad> rad (diagnostic only)`

**Verdict logic (`Supervisor`, chassis-position telemetry):** PASS requires the chassis to have moved forward initially (`positions_x[10] - positions_x[0] > 0`, checked via the first 10 samples) and a chassis-position plateau to have been detected (as above) before `max_time = 10.0` s. Wheel-joint telemetry (`positions_left`/`positions_right`) is still recorded and printed — including a final "wheels out of sync" warning if `|left_final - right_final| > 0.1` rad — but purely as diagnostic output; neither participates in the PASS/FAIL decision. A representative run: chassis `dx` holds around 8.45 mm/5-steps while moving, drops to 0.00 mm/5-steps once the robot contacts the 5 kg obstacle, collision detected at `t≈6.5s`, final chassis `x≈0.4127 m`, verdict `PASS`.

---

### 3.6 Common Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| **"could not find wheel motor devices"** | Wrong device names or PROTO mismatch | Check PROTO's RotationalMotor names match controller |
| **Wheels don't rotate** | Controller not running | Check Webots console for errors; verify controller field is not `"void"` |
| **Wheels rotate but robot doesn't move (chassis appears welded to the floor)** | `Robot` node has no `physics`/`boundingObject` of its own — typically caused by a massless dummy root link (e.g. `base_footprint`) with a fixed joint to the real, mass-bearing link; Webots implicitly welds a Physics-less Robot with no Physics-bearing Solid ancestor to the static world | Make the mass-bearing link (`base_link`) the URDF root directly, not a child of a massless reference-frame root — see issue #32. Regenerate the PROTO and confirm via `wheel_articulation_check.py`'s chassis-translation (`dx`/`dy`) check, not the wheel-angle check alone (the latter cannot detect this failure mode) |
| **Mesh files not found** | Path mismatch | Verify `../../urdf/meshes/` paths in TurtlebotPoc.proto exist |
| **Simulation too slow** | Graphics overhead | Use `--batch` mode (headless) for faster execution |
| **FAIL verdict** | Wheel position delta < 0.0001 rad | Wheels genuinely didn't move; check motor commands and physics |
| **"WARN: no position sensor devices"** | Sensors not enabled | PROTO or world might not expose PositionSensor devices |

### 3.7 Performance Notes

**Simulation Speed:**

- **GUI mode:** Real-time (~1× speed) with graphics rendering
- **Batch mode:** 5-10× faster (no rendering, optimal stepping)

**Resource Usage:**

- Memory: ~200-300 MB
- CPU: Single core (Webots step is sequential)
- Mesh size: ~6.8 MB total (STL files loaded once)

---

## Part 4: Architecture Decision Rationale

### 4.1 Why Three Files?

1. **URDF (`turtlebot3_poc.urdf`):** 
   - Industry standard, human-readable, tool-agnostic
   - Reusable for other simulators (Gazebo, ROS, etc.)

2. **PROTO (`TurtlebotPoc.proto`):** 
   - Webots-native format, optimized for simulation
   - Reusable instantiation in multiple worlds
   - Encapsulation (fields for position, controller, etc.)

3. **World (`turtlebot3_poc.wbt`):** 
   - Environment definition separate from robot model
   - Allows testing same robot in different scenarios
   - Controller attachment point

### 4.2 URDF → Webots Translation Decisions

- **Continuous Joints** → `HingeJoint` with infinite position range
- **Collision Geometry** → `boundingObject` (simpler than exact mesh collision)
- **Mesh Scale** → `0.001` (FreeCAD outputs mm; Webots expects m)
- **Materials** → `PBRAppearance` (modern rendering pipeline)

### 4.3 Why Hand-Authored URDF?

For this POC:
- Explicitly demonstrates FreeCAD mesh round-tripping (decision #4 in issue #24)
- Simplified structure (omitted LDS/IMU sensor mounts)
- Future: `urdf_builder.py` will auto-generate from CAD assemblies

### 4.4 Lesson Learned: URDF Root Link Must Carry Physics (Issue #32)

The original URDF (matching the ROBOTIS reference structure) used a massless `base_footprint` link as the root, connected via a fixed `base_joint` to `base_link` (which carries all real mass/inertia/collision). This is a common convention in ROS-world URDFs (it gives navigation stacks a stable ground-projected reference frame), but it does not translate safely into a Webots PROTO: `urdf2webots` maps the URDF root link directly onto the generated PROTO's top-level `Robot` node, and maps every other link one level down as a nested `Solid`. With `base_footprint` as the root, `physics`/`boundingObject` land on the nested `base_link` `Solid`, not on the `Robot` node itself — and Webots implicitly welds a `Physics`-less `Robot` node with no `Physics`-bearing `Solid` ancestor to the static world. The practical symptom: wheel joints rotate normally under commanded velocity (so a wheel-angle-only articulation check reports `PASS`), but the chassis never translates.

The fix (issue #32) was to drop `base_footprint`/`base_joint` from the URDF entirely and make `base_link` the URDF root directly, so the generated PROTO's `Robot` node carries `physics`/`boundingObject` itself. This is a URDF-authoring-level fix, not a `urdf2webots` flag — the installed `urdf2webots` 2025.0.0 does contain a dead code path apparently intended to auto-promote a dummy root out of the way (a `while rootLink in ['base_link', 'base_footprint']` loop in its importer comparing a `Link` object against string names, which is always `False` and never fires), but it does not actually do this, so restructuring the source URDF was the only viable fix.

**Takeaway for future URDF authoring in this pipeline (and for #9/#10):** if a URDF is destined for `urdf2webots` conversion, its root link must be the first mass/collision-bearing link, not a massless reference frame — regardless of what convention the source robot's "native" URDF uses.

---

## Part 5: Extending the Simulation

### 5.1 Adding Sensors

**Example: Add LiDAR range sensor**

Edit `TurtlebotPoc.proto`, add to `base_link` children:

```proto
DistanceSensor {
  name "lidar"
  translation 0 0 0.1
  rotation 0 0 1 0
  fieldOfView 1.57  # 90° FOV
  maxRange 3.0
  numberOfRays 360
}
```

**Controller Code:** 

```python
lidar = robot.getDevice("lidar")
lidar.enable(32)  # Enable at every timestep
ranges = lidar.getRangeImage()  # Get ray distances
```

### 5.2 Adding Joint Control Modes

**Velocity Control (current):** Already implemented

**Position Control (trajectory tracking):**

```python
motor.setPosition(target_angle_rad)  # Move to fixed angle
motor.setMaxVelocity(1.0)  # Rate limit
```

**Effort Control (torque):**

```python
motor.setTorque(force_nm)  # Direct torque command
```

### 5.3 Replacing with Real Controller

The wheel_articulation_check controller is minimal. Replace with full navigation stack:

```python
# Custom controller: navigate_to_goal.py
from controller import Robot
from nav_stack import MoveBaseClient

robot = Robot()
nav = MoveBaseClient()
nav.send_goal(target_x=1.0, target_y=1.0)  # Navigate 1m forward, 1m left

while nav.is_moving():
    # Update wheel commands based on nav feedback
    left_motor.setVelocity(nav.left_wheel_cmd)
    right_motor.setVelocity(nav.right_wheel_cmd)
    robot.step(32)
```

---

## Part 6: Reference Data

### 6.1 Robot Specifications

| Parameter | Value | Unit |
|-----------|-------|------|
| **Total Mass** | 0.8877 | kg |
| **Base Mass** | 0.8257 | kg |
| **Wheel Mass (each)** | 0.0285 | kg |
| **Caster Mass** | 0.005 | kg |
| **Wheel Radius** | 0.033 | m |
| **Wheel Separation** | 0.160 | m |
| **Wheelbase** | 0.160 | m |
| **Max Wheel Velocity** | 2.0 (demo) | rad/s |
| **Max Wheel Torque** | 10000 | N·m (Webots motor limit) |

### 6.2 Simulation Parameters

| Parameter | Value | Unit |
|-----------|-------|------|
| **Simulation Step** | 32 | ms |
| **Default Duration** | 300 steps | ~9.6 s |
| **Gravity** | 9.81 | m/s² |
| **Damping** | 0.0 (default) | — |
| **Friction** | Physics engine default | — |

### 6.3 File Sizes

| File | Size | Type |
|------|------|------|
| burger_base_roundtrip.stl | 4.6 MB | Binary mesh |
| left_tire_roundtrip.stl | 1.1 MB | Binary mesh |
| right_tire_roundtrip.stl | 1.1 MB | Binary mesh |
| turtlebot3_poc.urdf | ~4.5 KB | XML text |
| TurtlebotPoc.proto | ~5.2 KB | Text |
| turtlebot3_poc.wbt | ~2.0 KB | Text |

---

## Summary

The TurtleBot3 POC simulation demonstrates an integrated workflow:

1. **URDF** defines the robot structure once (tool-agnostic)
2. **PROTO** encapsulates the Webots-specific representation (reusable, configurable)
3. **World** ties robot to environment and controller (scenario-driven)
4. **Controller** executes behavior and reads back sensor data

This architecture enables:
- ✅ Reproducible, batch-runnable simulations
- ✅ Easy experimentation with different controllers and scenarios
- ✅ Validation of FreeCAD→Webots mesh pipeline
- ✅ Foundation for future autonomous navigation research

**Next Steps:**
- Extend with real ROS 2 control stack
- Add more sensor types (cameras, IMU, encoders)
- Implement obstacle avoidance controllers
- Scale to multi-robot scenarios

---

## Appendix: Quick Reference Commands

```bash
# GUI simulation
cd /home/pluto-atom-4/freecad-workspace/poc/freecad-webots-pipeline
webots webots/worlds/turtlebot3_poc.wbt

# Batch (headless) simulation
webots --batch webots/worlds/turtlebot3_poc.wbt --output=sim.log

# Verify controller output
tail -f sim.log | grep -E "(rad|PASS|FAIL|VERDICT)"

# Edit controller (after changes, re-run simulation)
nano webots/controllers/wheel_articulation_check/wheel_articulation_check.py

# Regenerate PROTO from URDF (if needed)
urdf2webots urdf/turtlebot3_poc.urdf --output webots/protos/TurtlebotPoc.proto

# Check mesh validity
file urdf/meshes/*.stl  # Should all be "data" (binary STL)
```

---

**Document Version:** 1.1  
**Created:** 2026-09-04  
**Last Updated:** 2026-09-05  
**Related Issues:** FreeCAD-Webots Pipeline POC (Issue #24), Issue #28 (world background/arena/viewpoint), Issue #32 (chassis-pinned-to-world fix)  
**References:**
- [Webots Documentation](https://cyberbotics.com/doc)
- [URDF Standard](http://wiki.ros.org/urdf)
- [urdf2webots Converter](https://github.com/omni-us/urdf2webots)
- FINDINGS.md — Technical discoveries and implementation notes
