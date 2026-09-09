# Servo Motor Assembly & Artifact Production

## Overview

Integration of Feetech STS3032 servo motor into FreeCAD plates assembly (GitHub Issue #3).

**Key Metrics:**
- Servo geometry: 1.8 MB STL → 36.13 MB STEP
- Assembly file: 13 KB (base) → <20 KB (with servo link)
- Mechanical constraint: <1mm alignment with Middle_Plate edges
- Production artifacts: STEP (2.0-2.5 MB) + STL (1.0-1.5 MB, reduced-servo geometry as of the Phase 4 pipeline fix)

---

## Mechanical Assembly Architecture

### Assembly Components

**Three-Body System:**
1. **Middle_Plate** — servo mounting base with Edge26/Edge34 mounting points
2. **Servo_Motor** — external STEP reference with placement matrix
3. **Top/Bottom Plates** — clearance verification targets

### Assembly Constraints

| Constraint | Specification | Validation |
|-----------|---|---|
| Servo alignment | <1mm to Edge26/Edge34 midpoint | Phase 2 calculation |
| Servo orientation | 90° pitch (shaft perpendicular to plate) | Placement matrix |
| Servo clearance | >5mm to all plate surfaces | Clearance check |
| Servo centering | Positioned at edge intersection | Distance validation |

### Integration Mechanism: External Linking

**Why external link?**
- Keeps assembly .FCStd <20 KB (vs 36+ MB embedded)
- STEP format for cross-tool compatibility
- Enables parametric updates without re-linking
- Standard XDE (External Document Exchange) approach

**Link Structure:**
```json
{
  "body_name": "Servo_Motor",
  "reference_file": "../Mechanical/feetech-STS3032.step",
  "placement": {"position": [177.0, 167.5, -6.8], "rotation": [0, 90, 0]}
}
```

---

## Implementation Phases

### Phase 1: Geometric Preparation (Complete)

**Input:** `~/Documents/feetech-STS3032_20190118_ASM.stl` (1.8 MB)

**Process:**
- Load & validate STL mesh topology
- Convert to solid STEP geometry (ISO 10303 AP203)
- Export with validation report

**Output:**
- `03_Parts/Mechanical/feetech-STS3032.step` (36.13 MB)
- Geometry: 35,770 faces, 17,789 vertices, dimensions 32×12×31.9 mm
- Conversion report: `feetech-STS3032_conversion_report.json`

**Script:** `01_convert_servo_stl_to_step.py` (122 lines)

---

### Phase 2: Mechanical Alignment (Complete)

**Input:** `plates_assembled.FCStd` (base assembly)

**Workflow:**
1. Extract Middle_Plate edges (Edge26, Edge34)
2. Compute edge midpoints and geometric properties
3. Calculate servo placement matrix (6 DOF: x, y, z, roll, pitch, yaw)
4. Validate alignment and clearances
5. Export placement data to JSON

**Validation Checks:**
- Alignment tolerance: <1mm from edge geometry ✓
- Clearance to Middle_Plate: >5mm ✓
- Clearance to Top_Plate: >5mm ✓ (this Phase 2 script's own Z-only formula; see note below)
- Clearance to Bottom_Plate: >5mm ✓ (this Phase 2 script's own Z-only formula; see note below)
- Pitch angle: 90° (perpendicular) ✓

> **Note (issue #22):** `servo_placement.json`'s Z-only clearance formula does not
> account for each plate's independent rotation, and originally misreported
> Bottom_Plate's clearance as ~4.85mm (failing the 5.0mm minimum). Phase 3
> (`03_link_servo_to_assembly.py`) was corrected to measure real 3D shape
> distance (`Part.Shape.distToShape` against a servo bounding-box proxy)
> instead — the real measured clearance is ~17.1mm to Bottom_Plate and
> ~6.43mm to Top_Plate.

**Output:**
- `servo_placement.json` — placement matrix (x, y, z, roll, pitch, yaw)
- Test results: 6/6 unit tests passing

**Scripts:**
- `02_position_servo.py` (555 lines) — calculation engine
- `test_02_servo_position.py` (342 lines) — unit tests

---

### Phase 3: Assembly Integration (Complete)

**Input:**
- `plates_assembled.FCStd` (base assembly)
- `feetech-STS3032.step` (servo geometry)
- `servo_placement.json` (calculated placement)

**Implementation:**
1. Load assembly and verify plate bodies exist
2. Create Part::Body container "Servo_Motor"
3. Configure XDE external reference to STEP file
4. Apply placement matrix (position + rotation)
5. Validate 7 checks:
   - Link file resolution
   - STEP geometry loads
   - Placement matrix valid
   - Servo alignment <1mm
   - Clearances verified
   - No interference
   - Document integrity

**Output:**
- Updated `plates_assembled.FCStd` (<20 KB, with external servo link)
- `servo_link_config.json` (link config and validation metadata)

**Script:** `03_link_servo_to_assembly.py` (731 lines)

---

### Phase 4: Artifact Production (In Progress)

**Goal:** Export assembly with servo link to production formats

**Process:**
1. Load assembly and resolve external servo link
2. Merge all plate bodies + servo into single compound
3. Validate merged geometry (no gaps, no overlaps)
4. Export to multiple formats

**Formats & Outputs:**

| Format | File | Size | Use Case |
|--------|------|------|----------|
| STEP | `plates_assembled_with_servo.step` | 2.0-2.5 MB | CAD tool import |
| STL | `plates_assembled_with_servo.stl` | 1.0-1.5 MB | 3D printing |
| 3MF | `plates_assembled_with_servo.3mf` | ~0.8 MB | Advanced printing |

**Metadata Output:**
```json
{
  "export_timestamp": "2026-08-28T15:00:00Z",
  "total_vertices": 21989,
  "total_faces": 38170,
  "geometry_valid": true,
  "no_gaps": true,
  "no_overlaps": true,
  "exports": {
    "step": {"file": "plates_assembled_with_servo.step", "size_mb": 2.3},
    "stl": {"file": "plates_assembled_with_servo.stl", "size_mb": 1.26}
  }
}
```

**Script:** `04_export_assembly_merged.py` (planned)

---

## Artifact Validation Strategy

### Level 1: Format Compliance
- STEP: ISO 10303-21 compliant, parseable by CAD tools
- STL: Valid binary format, consistent triangle normals
- 3MF: Valid ZIP structure with correct XML

### Level 2: Geometry Integrity
- Closed surfaces (no gaps)
- Manifold geometry (suitable for 3D printing)
- No degenerate triangles
- Consistent face orientations

### Level 3: Dimensional Accuracy
- Bounding box matches design spec
- Component dimensions preserved
- Assembly alignment maintained
- Clearances within tolerance

### External Tool Validation

```bash
# FreeCAD: Open STEP with servo + plates
freecad plates_assembled_with_servo.step

# Fusion 360: CAD import validation
# https://www.autodesk.com/products/fusion-360

# Cura: 3D printing slicer (verify no warnings)
cura plates_assembled_with_servo.stl

# MeshLab: Geometry analysis
meshlab plates_assembled_with_servo.stl
```

---

## File Structure

```
03_Parts/
├── Mechanical/
│   ├── feetech-STS3032.step (36.13 MB) [Phase 1]
│   ├── feetech-STS3032_conversion_report.json
│   ├── plates_assembled_with_servo.step [Phase 4]
│   ├── plates_assembled_with_servo.stl [Phase 4]
│   └── export_metadata.json [Phase 4]
│
└── Generators/
    ├── plates_assembled.FCStd (input, 13 KB)
    ├── 01_convert_servo_stl_to_step.py (122 lines)
    ├── 02_position_servo.py (555 lines)
    ├── test_02_servo_position.py (342 lines)
    ├── servo_placement.json [Phase 2 output]
    ├── 03_link_servo_to_assembly.py (731 lines)
    ├── servo_link_config.json [Phase 3 output]
    └── 04_export_assembly_merged.py [Phase 4]
```

**Total Implementation:** 1,750 lines of Python (Phases 1-3)

---

## Usage

### Prerequisites
```bash
cd inverted-pendulum-project
mamba activate pendulum-tools   # for Phase 1's wrapper + Phase 5/6 tooling

# Headless FreeCAD binary for Phases 2-4 (they import FreeCAD directly).
# Defaults to "freecadcmd" on PATH if unset.
export FREECAD_BIN=~/.local/bin/freecadcmd1.1
```

### Run Phases

**Phase 1: Geometric Preparation** (plain python — wraps FreeCAD internally via subprocess)
```bash
cd 03_Parts/Generators
python3 01_convert_servo_stl_to_step.py
```

**Phase 2: Alignment Calculation** (runs inside FreeCAD's own interpreter)
```bash
"${FREECAD_BIN:-freecadcmd}" 02_position_servo.py
python3 test_02_servo_position.py  # Unit tests (plain python, no FreeCAD needed)
```

**Phase 3: Assembly Integration** (runs inside FreeCAD's own interpreter)
```bash
"${FREECAD_BIN:-freecadcmd}" 03_link_servo_to_assembly.py
```

**Phase 4: Export Artifacts** (runs inside FreeCAD's own interpreter)
```bash
"${FREECAD_BIN:-freecadcmd}" 04_export_assembly_merged.py
```

### Manual Verification
```bash
# Open assembly in FreeCAD
freecad 03_Parts/Generators/plates_assembled.FCStd

# In GUI:
# 1. Expand Model tree (left panel)
# 2. Verify "Servo_Motor" body with external link icon
# 3. Click servo to select in 3D view
# 4. View → Fit All to see complete assembly
```

---

## Key Design Decisions

| Decision | Rationale | Trade-off |
|----------|-----------|-----------|
| External STEP link | Lightweight assembly file | Depends on external file |
| 6 DOF placement | Full positioning control | Edge indexing geometry-dependent |
| Phase-based scripts | Testable, repeatable | Multiple files to manage |
| JSON metadata | Version control friendly | Not CAD-native format |
| Merge on export | Static geometry for tools | Loses assembly constraints |

---

## Status & Timeline

| Phase | Status | Completion | LOC |
|-------|--------|------------|-----|
| **1** | ✅ Complete | 2026-08-28 | 122 |
| **2** | ✅ Complete | 2026-08-28 | 555+342 |
| **3** | ✅ Complete | 2026-08-28 | 731 |
| **4** | 🔄 In Progress | 2026-08 | — |
| **5** | 📋 Planned | 2026-09 | — |

---

## Known Limitations & Mitigation

| Limitation | Impact | Mitigation |
|-----------|--------|-----------|
| External link breaks if servo.step moved | Assembly fails to load | Fixed path, relative links, documented |
| Phase 2 edge indexing brittle | Fails if plate changes | Include visualization helper script |
| STEP file 36 MB large | Slow transfer | Document optimization paths |
| Phase 4 not implemented | Cannot export yet | Schedule for implementation |
| Live GUI auto-tessellates shapes for display, contaminating `Shape.BoundBox` reads (a 70.00mm cylinder read ~69.90mm) | Dimensional validation can fail only when a generator script is run live through the FreeCAD MCP bridge, not headlessly | Validate dimensions against a true headless `freecadcmd` run; if a script tessellates internally, always tessellate a `.copy()` of the shape (see `07_create_body_and_wheels.py`'s `_tessellate_triangle_count()`) |
| Live bridge's `get_screenshot` broken (`freecad-mcp-workbench` 0.6.2) | No built-in visual-review screenshot | Workaround via `execute_python` + `FreeCADGui...ActiveView.saveImage()`, see root `CLAUDE.md`'s "FreeCAD Live Bridge — Known Limitations" |
| `freecadcmd` 1.1.3 doesn't set `__name__=="__main__"` for `--python`/positional invocation | Script silently no-ops, exits 0 | Invoke as `"$FREECAD_BIN" -c "exec(open('script.py').read())"` instead |
| Visibility/camera are GUI-only state | A headless-generated `.FCStd` opens with all objects invisible, no useful viewpoint | Guard visibility/camera-setting code with `if not getattr(App, "GuiUp", False): return`; only takes effect when driven through the live bridge (see `07_create_body_and_wheels.py`'s `_set_default_visibility()`/`_set_camera_framing()`) |

---

## Related Work: Issue #9 Stage 1 (Body + Wheel Geometry)

`03_Parts/Generators/07_create_body_and_wheels.py` (Issue #9, PR #52) builds `Base_Link`/
`Wheel_Left`/`Wheel_Right` as new primitives, sized from `02_Design_Inputs/robot_parameters.yaml`,
and reuses this document's `PlateStack`/`STS3032_Mount` (copied, not linked) as a `Pendulum_Link`
subassembly in a new `robot_body_wheels.FCStd`. It's a downstream consumer of this pipeline's
`plates_servo_assembled.FCStd`, not a new phase of the servo-integration work above — see Issue #9
for its own plan/decisions, and the "Known Limitations" row above for what a live-bridge human
review of its output actually found (GUI-only tessellation/visibility/camera caveats).

Separately: a human review of that Stage 1 artifact found it didn't match what was actually
pictured, despite passing every dimensional/structural check (see Issue #53 for the full
human-intent-vs-generated-artifact writeup and workflow recommendations) — worth reading before
starting Stage 2+ or any other new FreeCAD generation work in this repo.

---

## References

- GitHub Issue #3: Servo motor integration
- `../CLAUDE.md` — Development guide
- `README.md` — Quick start
- FreeCAD Part module documentation
- ISO 10303-21 (STEP), STL, 3MF specifications

---

**Status:** Design documented, Phases 1-3 complete, Phase 4 in progress. See "Related Work"
above for Issue #9 Stage 1's downstream use of this pipeline's output.
**Last Updated:** 2026-09-09
