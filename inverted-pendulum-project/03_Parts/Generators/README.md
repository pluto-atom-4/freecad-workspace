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

## Testing & Validation (Phase 5)

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
