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

## Robot Assembly (Issue #9 Stages 1-3)

Complete workflow for creating a two-wheel self-balancing robot assembly from Stage 0's design parameters, configuring joints, and computing mass properties.

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

**Requirements:**
- `robot_body_wheels.FCStd` (output from Stage 1)
- `robot_parameters.yaml` (design parameters)

**Usage:**
```bash
echo "exec(open('08_configure_assembly_joints.py').read())" | freecadcmd -c
python3 -m pytest -q test_08_configure_assembly_joints.py
```

**Output:**
- `robot_assembly.FCStd` (new document with joints configured: wheel rotations, pendulum pivot, servo rotations)
- `08_assembly_joints_metadata.json` (joint configuration details, validation results)
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
- Uses FreeCAD `Mesh.MatrixOfInertia()` (mesh-native), NOT STEP round-trip (60% volume loss + fragmentation per FINDINGS.md sec.3)
- Mass comes from `robot_parameters.yaml`'s `servo.target_mass_kg` (datasheet value ~0.055 kg), not from density×volume
- Inertia tensor scaled to match real mass (computed for unit density, then scaled proportionally)
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

**Total Time:** ~60-120 seconds

---

## Robot Body + Wheel Geometry (Issue #9, Stage 1)

Stage 1 of Issue #9's "FreeCAD Mechanical Model with URDF Export" plan.
Builds the two-wheel robot chassis and wheels, and reuses the existing
3-plate pendulum linkage + servo (Issue #3's `plates_servo_assembled.FCStd`)
as a `Pendulum_Link` subassembly. Reads all dimensions from Stage 0's
`../../02_Design_Inputs/robot_parameters.yaml` (via `robot_parameters.py`)
rather than hardcoding them.

**Scope:** geometry only. Joint constraints (Stage 2), mass/inertia in SI
units (Stage 3), and URDF export (Stage 4) are separate, not-yet-implemented
stages -- see Issue #9's consolidated plan comment.

**Script:** `07_create_body_and_wheels.py`

**Requirements:**
- Headless FreeCAD binary (`freecadcmd`), resolved via `FREECAD_BIN` (see
  Environment section below)
- `plates_servo_assembled.FCStd` in this directory (source for the reused
  pendulum linkage; not modified)
- `../../02_Design_Inputs/robot_parameters.yaml` (read via
  `robot_parameters.py`)

**Usage:**
```bash
FREECAD_BIN=~/.local/opt/freecad-1.1.3/usr/bin/freecadcmd
"$FREECAD_BIN" -c "exec(open('07_create_body_and_wheels.py').read())"
```
Note: unlike Phases 1-5's documented `freecadcmd --python script.py` form,
this build of `freecadcmd` (1.1.3) does not set `__name__ == "__main__"`
for a plain positional script argument, so a script's
`if __name__ == "__main__":` guard never fires that way (verified
empirically). The `-c "exec(open(...).read())"` form (already used as the
Phase 1 fallback above) is what actually runs the script end-to-end.

**Output:**
- `robot_body_wheels.FCStd` -- new, self-contained document (does not modify
  `plates_servo_assembled.FCStd`), containing:
  - `Base_Link` (`Part::Feature`, flat plate, 40x80x2.5mm -- redesigned
    from an earlier 120x80x40mm solid box; see the module docstring and
    root `CLAUDE.md`/`DESIGN.md` for why)
  - `Wheel_Left` / `Wheel_Right` (`Part::Feature`, cylinders, dia 30mm x
    6mm wide) -- each mounted on its own `Pendulum_Link`/`Pendulum_Link_Right`
    `Bottom_Plate`'s real mounting hole, not on a body-centerline formula
  - `Pendulum_Link` (`App::Part`), containing copies of `PlateStack`
    (`Top_Plate`/`Middle_Plate`/`Bottom_Plate`) and `STS3032_Mount`
    (servo visual + collision-proxy meshes), tilted so its plates stand
    parallel to the wheel discs, positioned `pivot_height_mm` above
    `Base_Link`'s top
  - `Pendulum_Link_Right` (`App::Part`), a second copy built from literal,
    human-tuned constants (not a geometric mirror of `Pendulum_Link`) --
    see `build_pendulum_link_right()`
  - Object names are exact and case-sensitive -- Stage 2's joint config and
    Stage 4's URDF export consume them verbatim.
- `07_body_wheels_metadata.json` -- per-link dimensions/placement/volume,
  new-primitive triangle count (budget: <5000, per Issue #9's acceptance
  criteria), reused-mesh facet counts (reported separately, not counted
  against the budget -- see the JSON's own `reused_pendulum_mesh_note`),
  and validation results.

**Time:** ~10-20 seconds

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

## Directory Structure

```
Generators/
├── 01_convert_servo_stl_to_step.py
├── 01_convert_servo_stl_to_step_via_freecad.py
├── 02_position_servo.py
├── 03_link_servo_to_assembly.py
├── 04_export_assembly_merged.py
├── test_05_integration.py
├── test_05_integration_live.py
├── 07_create_body_and_wheels.py
├── test_07_body_wheels_geometry.py
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
