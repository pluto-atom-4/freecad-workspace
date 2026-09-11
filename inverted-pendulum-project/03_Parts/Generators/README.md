# Part Generators

Python scripts for parametric FreeCAD model generation and servo motor integration.

---

## Servo Motor Integration (Phases 1-4)

Complete workflow for integrating Feetech STS3032 servo motor with three-plate assembly.

### Phase 1: STL to STEP Conversion

**Script:** `01_convert_servo_stl_to_step.py`

Converts servo motor STL mesh to STEP format for assembly integration.

**Requirements:**
- Headless FreeCAD binary (`freecadcmd`), invoked as a subprocess
- Resolved via the `FREECAD_BIN` environment variable, defaulting to
  `freecadcmd` on PATH if unset

**Usage:**
```bash
python3 01_convert_servo_stl_to_step.py
# or, pinning a specific FreeCAD build:
FREECAD_BIN=/home/pluto-atom-4/.local/opt/freecad-1.1.3/usr/bin/freecadcmd \
  python3 01_convert_servo_stl_to_step.py
# or manually:
freecadcmd -c "exec(open('01_convert_servo_stl_to_step_via_freecad.py').read())"
```

**Output:**
- `../Mechanical/feetech-STS3032.step` (36.13 MB)
- `../Mechanical/feetech-STS3032_conversion_report.json` (validation report)

**Time:** ~30-60 seconds

### Phase 2: Servo Motor Position Calculation

**Script:** `02_position_servo.py`

Calculates precise servo placement based on plate geometry (Edge26, Edge34).

**Requirements:**
- Headless FreeCAD binary (`freecadcmd`), resolved via `FREECAD_BIN` if you use that pattern
- `plates_assembled.FCStd` in current directory

**Usage:**
```bash
freecadcmd --python 02_position_servo.py
```

**Output:**
- `servo_placement.json` (placement matrix + validation data)
- Console: Detailed calculation logs

**Placement Data:**
- Position: X ≈ 10mm, Y ≈ 15mm, Z ≈ -11.25mm
- Rotation: Roll=0°, Pitch=90°, Yaw=0°
- Z-offset: 10mm below middle plate surface
- Validations: 6 checks (alignment, clearances)

**Time:** ~5-10 seconds

### Phase 3: External Servo Link

**Script:** `03_link_servo_to_assembly.py`

Creates external link to servo STEP file in assembly, applies placement.

**Requirements:**
- Headless FreeCAD binary (`freecadcmd`)
- Servo STEP file from Phase 1
- `servo_placement.json` from Phase 2

**Usage:**
```bash
freecadcmd --python 03_link_servo_to_assembly.py
```

**Output:**
- Updated `plates_assembled.FCStd` (< 20 KB with external link)
- `servo_link_config.json` (link configuration + validation data)
- Console: Link configuration logs

**Key Benefits:**
- Small assembly files (external link)
- Precise placement matrix
- 7+ validation checks

**Time:** ~5-10 seconds

### Phase 4: Export Merged Assembly

**Script:** `04_export_assembly_merged.py`

Merges plates + servo into single geometry, exports to multiple formats.

**Requirements:**
- Headless FreeCAD binary (`freecadcmd`)
- `plates_assembled.FCStd` (with servo link)
- Servo STEP file

**Usage:**
```bash
freecadcmd --python 04_export_assembly_merged.py
```

**Output:**
- `plates_assembled_with_servo.step` (2.0-2.5 MB STEP format)
- `plates_assembled_with_servo.stl` (1.8-2.0 MB STL mesh)
- `plates_assembled_with_servo.3mf` (optional, if supported)
- `export_metadata.json` (export statistics + validation)
- Console: Export metrics and validation results

**Use Cases:**
- CAD Software Import: Use STEP file
- 3D Printing: Use STL file
- Documentation: Use either format

**Time:** ~15-30 seconds

---

## Robot Assembly (Issue #9 Stages 1-5 + URDF Export Stages 4-5)

Complete workflow for creating a two-wheel self-balancing robot assembly from Stage 0's design parameters, configuring joints, computing mass properties, and exporting to URDF format for simulation.

### Phase 7-11 Overview (URDF Export Pipeline)

| Phase | Script | Output | Execution Mode | Status |
|-------|--------|--------|-----------------|--------|
| 7 | `07_create_body_and_wheels.py` | `robot_body_wheels.FCStd`, `07_body_wheels_metadata.json` | `freecadcmd -c` | ✅ |
| 8 | `08_configure_assembly_joints.py` | `robot_assembly.FCStd`, `joint_config.json` | **FreeCAD MCP bridge** (not plain freecadcmd) | ⚠️ |
| 9 | `09_compute_mass_properties.py` | `09_mass_properties.json` | `freecadcmd -c` | ✅ |
| 10 | `10_export_urdf.py` | `06_Exports/urdf/robot.urdf`, `06_Exports/urdf/meshes/`, `10_urdf_export_metadata.json` | `python3` | ✅ |
| 11 | `11_validate_inertia.py` | `11_inertia_validation_report.json` | `python3` | ✅ |

**Run phases 7, 9-11 end-to-end:** `./run_urdf_export.sh`  
**Run Phase 8 separately:** Requires FreeCAD MCP bridge (see Phase 8 section below for details)

### Joint & Link Naming Convention (for Webots/Simulator Import)

**Links (5 total):**

| Link Name | Type | Mass (kg) | Notes |
|-----------|------|-----------|-------|
| `Base_Link` | URDF root (no incoming joint) | 0.25 | Flat chassis plate (40×80×2.5 mm) |
| `Wheel_Left` | Cylinder | 0.03 | Left wheel (∅30 mm × 6 mm) |
| `Wheel_Right` | Cylinder | 0.03 | Right wheel (∅30 mm × 6 mm) |
| `Pendulum_Link` | Composite (plates + servo) | ~0.175 | 3-plate stack + servo left instance |
| `Pendulum_Link_Right` | Composite (plates + servo) | ~0.175 | 3-plate stack + servo right instance |

**Joints (4 total, all revolute, axis = [0 1 0]):**

| Joint Name | Type | Parent→Child | Axis | Limits (rad) | Notes |
|------------|------|-------------|------|--------------|-------|
| `wheel_left_joint` | revolute | `Base_Link` → `Wheel_Left` | [0, 1, 0] | [-∞, ∞] | Free rotation |
| `wheel_right_joint` | revolute | `Base_Link` → `Wheel_Right` | [0, 1, 0] | [-∞, ∞] | Free rotation |
| `pendulum_pivot_joint` | revolute | `Base_Link` → `Pendulum_Link` | [0, 1, 0] | [-∞, ∞] | Pivot axis (left servo) |
| `pendulum_pivot_right_joint` | revolute | `Base_Link` → `Pendulum_Link_Right` | [0, 1, 0] | [-∞, ∞] | Pivot axis (right servo) |

### Stage 1: Body + Wheel Geometry (Issue #9)

**Script:** `07_create_body_and_wheels.py`

Creates chassis (`Base_Link`), wheels (`Wheel_Left`, `Wheel_Right`), and reuses the existing 3-plate + servo linkage from Phase 3's `plates_servo_assembled.FCStd` as a `Pendulum_Link` subassembly.

**Requirements:**
- `plates_servo_assembled.FCStd` (output from Phase 3, must exist and be up-to-date)
- `robot_parameters.yaml` (Stage 0 design inputs)

**Usage:**
```bash
echo "exec(open('07_create_body_and_wheels.py').read())" | freecadcmd -c
```

**Output:**
- `robot_body_wheels.FCStd` (new document: Base_Link, Wheel_Left/Right, Pendulum_Link, Pendulum_Link_Right)
- `07_body_wheels_metadata.json` (per-link dimensions, placement, volume, triangle counts, validation results)
- Console: Geometry summary (box/cylinder/subassembly info)

**Key Design Decisions:**
- Copies (not links) the servo mesh objects into the new document for self-contained Stage 2+ input
- Chassis is a flat plate (not solid box), positioned as a deck plate near the pivot/servo level
- Pendulum_Link tilts 90° so its plates stand parallel to the wheel discs
- Pendulum_Link_Right built from human-tuned literal constants (not a geometric mirror, due to Mesh.transform() bugs)

**Time:** ~15-30 seconds

**Status:** 🔄 Redesign in progress (PR #52, `feat/issue-9-stage1-body-wheels`); see DESIGN.md for known transitional failures.

### Stage 2: Assembly Joints (Issue #9)

**Script:** `08_configure_assembly_joints.py` + `test_08_configure_assembly_joints.py`

Adds Assembly workbench joints (fixed + revolute) to configure the robot's kinematic structure.

⚠️ **KNOWN LIMITATION:** Phase 8 **CANNOT RUN in plain headless `freecadcmd`** due to a FreeCAD 1.1.3 bug where Assembly::JointGroup/Joint creation segfaults when `JointObject` module is imported. See the script's docstring (lines 76-93) for full details.

**Requirements:**
- `robot_body_wheels.FCStd` (output from Stage 1)
- `robot_parameters.yaml` (design parameters)
- FreeCAD MCP bridge (if running this script directly; see below)

**Usage (via FreeCAD MCP bridge, not plain freecadcmd):**

If you have the FreeCAD MCP bridge running (see root `CLAUDE.md`), use its `execute_python` tool:
```python
# In the bridge's Python console or via execute_python:
exec(compile(open('08_configure_assembly_joints.py').read(), 
             '08_configure_assembly_joints.py', 'exec'),
     {'__name__': '__main__', '__file__': '08_configure_assembly_joints.py'})
```

Or run the accompanying pytest test (pure Python, no FreeCAD required):
```bash
python3 -m pytest -q test_08_configure_assembly_joints.py
```

**Output:**
- `robot_assembly.FCStd` (new document with joints configured: wheel rotations, pendulum pivot, servo rotations)
- `joint_config.json` (joint configuration details, validation results)
- Console: Joint summary, validation results

**Time:** ~10-20 seconds

**Status:** 🔄 (see Issue #63 fix in PR #66 for Visibility recursion on App::Link)

### Stage 3: Servo Mass Properties (Issue #85)

**Script:** `09_compute_mass_properties.py` + `test_09_compute_mass_properties.py`

Computes mass properties for the two servo instances (left and right) embedded in the Pendulum_Link subassembly, using **mesh-native API** (never STEP round-trip, which loses fidelity catastrophically per FINDINGS.md).

**Requirements:**
- `robot_assembly.FCStd` (output from Stage 2)
- `robot_parameters.yaml` (including new `servo:` section with `target_mass_kg`)

**Usage:**
```bash
echo "exec(open('09_compute_mass_properties.py').read())" | freecadcmd -c
python3 -m pytest -q test_09_compute_mass_properties.py
```

**Output:**
- `09_mass_properties.json` (per-servo: volume_mm3, collision_proxy_volume_mm3, mass_kg, center_of_mass_mm, inertia_kg_mm2, validations)
- Console: Geometry summary (volumes, CoM, inertia tensor), validation results

**Key Design Decisions:**
- Volume and center-of-mass from mesh-native API (Mesh.Mesh.Volume, .CenterOfGravity), NOT STEP round-trip (60% volume loss + fragmentation per FINDINGS.md sec.3)
- Inertia tensor computed using bounding-box-inscribed ellipsoid approximation (I_xx=(m/20)*(dy²+dz²) cyclic, fed by mesh.BoundBox), NOT `Mesh.MatrixOfInertia` (which doesn't exist as an API on Mesh.Mesh)
- Mass comes from `robot_parameters.yaml`'s `servo.target_mass_kg` (datasheet value ~0.055 kg), not from density×volume
- Inertia tensor approximation scaled to match real mass
- Includes validation: both servos present, volumes nonzero, left/right symmetry, mass match, provisional geometric sanity check (±30% of estimated bounding box)

**Tests:**
- Both servos present in JSON
- Volumes nonzero and symmetric (left/right within 5%)
- Masses match datasheet target
- Inertia tensor structure valid (6 components: ixx, iyy, izz, ixy, ixz, iyz)
- Provisional geometric sanity check (±30% tolerance, pending Issue #8's hardware spike)

**Time:** ~5-10 seconds

**Status:** ✅ (Issue #85 complete)

### Stage 4: URDF Export with Collision Primitives (Issue #84)

**Script:** `10_export_urdf.py` + `test_10_export_urdf.py`

Exports the complete robot assembly to URDF format (standardized XML for simulators), using native URDF primitives (box + cylinder) for servo collision geometry and a high-fidelity mesh for visual display.

**Requirements:**
- `robot_assembly.FCStd` (output from Stage 2)
- `09_mass_properties.json` (output from Stage 3, containing servo mass/inertia/CoM)
- `07_body_wheels_metadata.json` (containing link dimensions/masses from Stage 1)
- `robot_parameters.yaml` (design parameters)
- `joint_config.json` (joint definitions from Stage 2)
- Visual mesh file: `../Mechanical/feetech-STS3032-visual-1.0mm.stl`

**Usage:**
```bash
echo "exec(open('10_export_urdf.py').read())" | freecadcmd -c
python3 -m pytest -q test_10_export_urdf.py
```

**Output:**
- `06_Exports/urdf/robot.urdf` (URDF XML, ~15-20 KB) containing:
  - 5 links: `Base_Link`, `Wheel_Left`, `Wheel_Right`, `Pendulum_Link`, `Pendulum_Link_Right`
  - 4 revolute joints: `wheel_left_joint`, `wheel_right_joint`, `pendulum_pivot_joint`, `pendulum_pivot_right_joint`
  - Masses and inertia tensors (SI units: kg, kg⋅m²)
  - Combined inertia for Pendulum_Link/Right (plate stack + servo, computed via parallel-axis theorem)
- `06_Exports/urdf/meshes/feetech-STS3032-visual.stl` (37,556 facets, copied from input)
- `10_urdf_export_metadata.json` (export log, collision primitive dimensions, inertia breakdown)

**Key Design Decisions:**
- **Collision geometry:** Servo represented as two URDF primitives (box + cylinder, NOT mesh) to keep physics simulator budget tight; each Pendulum_Link has 3 collision elements (plate box + servo box + servo cylinder)
- **Visual geometry:** High-fidelity STL mesh (1.0mm tolerance) for accurate 3D rendering; separately positioned from the plate geometry so both are visible
- **Inertia:** Combined via parallel-axis theorem, accounting for servo offset relative to plate stack; servo CoM computed by applying STS3032_Mount's Placement (both position and rotation) to the servo's local CoM from Stage 3
- **Plate CoM:** Approximated as center-of-bounding-box [7.995, 18.276, 4.5] mm (acceptable for ~3% tolerance; could be refined with true centroid computation if hardware measurement requires)

**Tests:**
- URDF file exists and is well-formed XML
- All 5 links present with valid names
- All 4 joints present and type='revolute'
- Joint parent/child references point to valid links
- Base_Link is root (no incoming joints)
- Every link has exactly 1 inertial element
- Pendulum_Link/Right combined masses correct (~0.175 kg = plate 0.12 + servo 0.055)
- Pendulum_Link/Right have exactly 2 visual elements (plate box + servo mesh) ✓ NEW
- Pendulum_Link/Right have exactly 3 collision elements (plate box + servo box + servo cylinder) ✓ NEW
- Servo collision primitives have correct dimensions (box 32×12×28 mm, cylinder r=4.65 mm × h=16.15 mm) ✓ NEW
- No mesh elements in collision geometry (collision uses only box/cylinder primitives)
- Visual mesh file exists and is referenced correctly
- Metadata documents all collision primitive dimensions and inertia details

**Time:** ~5-10 seconds

**Status:** ✅ (Issue #84 complete)

### Stage 5: Validate URDF Inertia (Issue #91)

**Script:** `11_validate_inertia.py` + `test_11_validate_inertia.py`

Compares calculated inertia properties (from `robot.urdf`, Stage 4 output) against real prototype measurements when available. Designed for hardware validation work (Issue #8's <2% inertia-error success criterion). Script is pure Python (no FreeCAD required).

**Requirements:**
- `06_Exports/urdf/robot.urdf` (output from Stage 4)
- `02_Design_Inputs/prototype_measurements.json` (optional, future hardware data)
  - See `02_Design_Inputs/prototype_measurements.schema.json` for expected JSON shape
  - See `02_Design_Inputs/prototype_measurements.example.json` for example data (fake values)

**Usage:**
```bash
python3 11_validate_inertia.py
python3 -m pytest -q test_11_validate_inertia.py
```

**Output:**
- `11_inertia_validation_report.json` — detailed per-link comparisons (if measurements present), overall pass/fail status, tolerance info
- Console: Summary table showing per-link comparison results and percent differences

**Key Design Decisions:**
- **Units:** Parses SI units from URDF (meters, kg·m²) and converts to project-native mm-based units for comparison (×1000 for length, ×1e6 for inertia)
- **Graceful absence:** Returns exit code 0 if `prototype_measurements.json` doesn't exist (no failure); validation is optional
- **Tolerance:** Compares mass and diagonal inertia terms (ixx, iyy, izz) at 30% default tolerance; skips off-diagonal terms (ixy, ixz, iyz) as near-zero/noisy
- **Null handling:** Unmeasured fields in prototype data are skipped, not counted as failures

**Tests:**
- URDF parsing: all 5 links present, units correct (mm-scale, not meter-scale)
- Prototype loading: missing file returns None, example file loads correctly
- Comparisons: within tolerance passes, exceeds tolerance fails, null fields skipped, off-diagonal terms skipped
- Report structure: correct fields, proper status values

**Exit Code:**
- 0: validation passed OR no prototype data available (graceful absence)
- 1: validation failed (measured values exceed tolerance)

**Time:** < 1 second (pure Python, no FreeCAD)

**Status:** ✅ (Issue #91 complete; prototype_measurements.json population pending Issue #8 hardware work)

---

## Testing & Validation (Phase 5, Legacy)

### Unit Tests (No FreeCAD Required)

**Script:** `test_05_integration.py`

Comprehensive test suite for all phases (validating JSON outputs, file existence, formats).

**Usage:**
```bash
python3 test_05_integration.py
```

**Output:**
- Console: Test summary (pass/fail counts, coverage)
- `test_results.json` (machine-readable results)

**Test Coverage:**
- Phase 1: 5 tests (file existence, format, size)
- Phase 2: 6 tests (JSON structure, values, tolerances)
- Phase 3: 5 tests (link config, validation counts)
- Phase 4: 7 tests (export files, metadata, sizes)
- **Total:** 23 tests

**Current Status:**
- Phase 1: ✓ PASSED (5/5)
- Phases 2-4: ⏳ Pending (require FreeCAD execution)

**Time:** ~2-5 seconds

### Live Integration Tests (FreeCAD Required)

**Script:** `test_05_integration_live.py`

End-to-end tests requiring FreeCAD Python environment.

**Usage:**
```bash
freecadcmd --python test_05_integration_live.py
```

**Tests:**
1. Assembly loading with servo link
2. Servo visibility verification
3. Servo position validation
4. STEP export performance
5. STL export performance
6. Performance benchmarks

**Time:** ~30-60 seconds

---

## Quick Workflow

### Phases 1-4: Servo Integration (Mechanical Assembly)

Run all phases sequentially:

```bash
# Phase 1: Convert STL → STEP (if not done)
python3 01_convert_servo_stl_to_step.py

# Phase 2: Calculate servo position
freecadcmd --python 02_position_servo.py

# Phase 3: Link servo to assembly
freecadcmd --python 03_link_servo_to_assembly.py

# Phase 4: Export merged assembly
freecadcmd --python 04_export_assembly_merged.py

# Run tests
python3 test_05_integration.py
```

**Time:** ~60-120 seconds

### Phases 7-11: URDF Export Pipeline (Robot Assembly + Simulation)

Run the complete URDF export pipeline in one command:

```bash
# Run all phases (7-11) end-to-end
./run_urdf_export.sh
```

Or run phases individually:

```bash
echo "exec(open('07_create_body_and_wheels.py').read())" | freecadcmd -c
echo "exec(open('08_configure_assembly_joints.py').read())" | freecadcmd -c
echo "exec(open('09_compute_mass_properties.py').read())" | freecadcmd -c
python3 10_export_urdf.py
python3 11_validate_inertia.py
```

**Time:** ~60-120 seconds (depends on FreeCAD startup)

---

## Legacy Scripts

### `simple_part.py`
Direct FreeCAD part generation using AppImage Python interpreter.

**Features:**
- 40×20×15mm base block with features
- 4mm through-hole
- 3mm fillet, 5mm chamfer
- Exports FCStd + STEP formats

**Usage:**
```bash
./run_part.sh
```

### `simple_bracket.py`
**Deprecated / legacy.** Support bracket generation via a FreeCAD MCP Bridge
(XML-RPC), which requires the separate `freecad-mcp-server` project running
its bridge — not part of the standard `pendulum-tools` workflow and no longer
a project dependency. Prefer `06_cadquery_parametric_brackets.py` (Phase 6,
CadQuery-based, no FreeCAD/bridge required) for new bracket generation.

**Usage (requires a running FreeCAD MCP Bridge, see `freecad-mcp-server/`):**
```bash
mamba run -n pendulum-tools python3 simple_bracket.py
```

---

## Known Issues & TODOs

### Issue #9 Decision #8: Hardcoded Servo STL Source Path

**Status:** ⏳ TODO (not blocking current pipeline, but impacts fresh-clone reproducibility)

The `01_convert_servo_stl_to_step_via_freecad.py` script hardcodes the vendor STL source path:
```python
DOCUMENTS_DIR = Path.home() / "Documents"
STL_SOURCE = DOCUMENTS_DIR / "feetech-STS3032_20190118_ASM.stl"
```

This assumes the vendor's original STL file is manually placed at `~/Documents/` on a specific machine and is not documented or automated. **Breaks fresh-clone reproducibility** — a new developer cloning this repo cannot regenerate the servo STEP file without first obtaining the STL separately and placing it at the hardcoded path.

**Follow-up (Issue #9 comment):** Not blocking the current pipeline (derived files already exist in the repo), but should eventually:
1. Accept the source path as a CLI argument or environment variable, or
2. Add a documented setup step (e.g., `SERVO_STL_PATH=~/path/to/file.stl python3 01_convert_servo_stl_to_step.py`)

---

## Directory Structure

```
Generators/
├── run_generator.sh                          # Phase 1-4 generator wrapper
├── run_export.sh                             # Phase 5 export wrapper (legacy)
├── run_urdf_export.sh                        # Phase 7-11 URDF export pipeline
├── 01_convert_servo_stl_to_step.py
├── 01_convert_servo_stl_to_step_via_freecad.py
├── 01_convert_servo_stl_to_step.sh
├── 02_position_servo.py
├── 03_link_servo_to_assembly.py
├── 04_export_assembly_merged.py
├── test_05_integration.py
├── test_05_integration_live.py
├── 06_cadquery_parametric_brackets.py        # Phase 6a: CadQuery-based brackets
├── 06_trimesh_merge_for_stl.py              # Phase 6b: Trimesh merging
├── 06_trimesh_mesh_validator.py             # Phase 6b: Mesh validation
├── test_06_phase6_tooling.py
├── 07_create_body_and_wheels.py              # Phase 7: Body + wheels (FreeCAD)
├── test_07_body_wheels_geometry.py
├── 08_configure_assembly_joints.py           # Phase 8: Assembly joints (FreeCAD)
├── test_08_configure_assembly_joints.py
├── 09_compute_mass_properties.py             # Phase 9: Mass properties (FreeCAD)
├── test_09_compute_mass_properties.py
├── 10_export_urdf.py                         # Phase 10: URDF export (pure Python)
├── test_10_export_urdf.py
├── 11_validate_inertia.py                    # Phase 11: Inertia validation (pure Python)
├── test_11_validate_inertia.py
├── robot_body_wheels.FCStd
├── 07_body_wheels_metadata.json
├── plates_assembled.FCStd
├── servo_placement.json
├── servo_link_config.json
├── export_metadata.json
├── test_results.json
├── simple_part.py
├── simple_bracket.py
├── run_part.sh
└── README.md
```

## Best Practices

- ✓ Type hints (Python 3.11+)
- ✓ Comprehensive docstrings
- ✓ FreeCAD best practices (recompute, cleanup)
- ✓ Proper error handling
- ✓ Structured logging
- ✓ Constants in UPPER_CASE

## Environment

Phase 1-5 scripts that need FreeCAD invoke it headlessly via `freecadcmd` as a
subprocess, controlled by the `FREECAD_BIN` environment variable (defaults to
`freecadcmd` on PATH). See `../../mamba-envs.yaml` and `../../README.md` for
the single `pendulum-tools` mamba environment used for everything else
(numpy/scipy/matplotlib and, for Phase 6, cadquery/trimesh).

## References

- [FreeCAD Part Module](https://wiki.freecadweb.org/Part_Module)
- [FreeCAD Scripting Basics](https://wiki.freecadweb.org/Scripting_basics)
