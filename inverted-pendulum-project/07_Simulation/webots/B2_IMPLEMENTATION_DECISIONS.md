# B2 Implementation Decisions

**Phase 13: PROTO Structure Validation (Stage B2)**

This document records the design decisions made for the PROTO structure validator and its integration into the URDF→Webots pipeline.

## Decision 1: Validator Form

**Decision:** Standalone Phase 13 script (`validate_proto_structure.py`)

**Rationale:**
- Matches B1 house style (Phase 11 = inertia validator, Phase 12 = URDF validator, Phase 13 = PROTO validator)
- Pure Python, no FreeCAD dependencies (same pattern as Phase 10-12)
- Portable: runs on any system with Python 3.9+
- Consistent pipeline structure: each phase is independent, stdout+JSON output, fail-hard on error
- Simplifies CI/CD: single script call, deterministic exit codes

**Alternative considered:** Integrated into `generate_proto.sh` as a post-processing hook
- **Rejected:** Would couple validation logic with bash scripting, harder to test and maintain

## Decision 2: Report Format

**Decision:** Dual output — JSON (machine-readable) + Markdown (human-readable)

**Rationale:**
- **JSON (`13_proto_structure_validation_report.json`):**
  - Machine-readable, structured data
  - Enables automated pass/fail CI checks
  - Captures per-check details (joint names, missing motors, etc.) for debugging
  - Timestamped for audit trail
  
- **Markdown (`B2_PROTO_STRUCTURE_CHECKLIST.md`):**
  - Human-friendly review format
  - Side-by-side comparison: URDF structure vs. PROTO structure
  - Direct reference for manual spot-checks before Stage B3
  - Integrates cleanly into project documentation

**Alternative considered:** Single JSON output
- **Rejected:** Humans need to read validation results; JSON is not suitable for that

## Decision 3: Mesh URL Resolution Strategy

**Decision:** Resolve mesh paths from `.generated/robot_webots.urdf` context (relative to URDF file's directory)

**Rationale:**
- URDF is the ground truth; PROTO generation is derived from it
- Mesh URLs in robot_webots.urdf are relative paths (e.g., `../../../06_Exports/urdf/meshes/feetech-STS3032-visual.stl`)
- Relative path resolution depends on the URDF's file location
- Validates that mesh files are actually present on disk before Stage B3 (Webots loading)
- Prevents "file not found" errors during Webots visual rendering

**Why not resolve from PROTO context:**
- PROTO files may use package:// URIs or other Webots-specific URL schemes
- URDF is clearer and more portable; PROTO can be regenerated if paths change

## Decision 4: Failure Mode

**Decision:** Fail-hard (exit code 1) if any validation check fails

**Rationale:**
- Matches B1 precedent (Phase 11, 12 validators exit 1 on failure)
- Blocks progression to Stage B3 (live Webots visual check) if structure is invalid
- Prevents misleading "passes" that hide structural issues
- CI/CD friendly: fail-fast enables early detection

**Alternative considered:** Warn-only (exit 0 even if checks fail)
- **Rejected:** Structural errors must be fixed before rendering; warnings are easily ignored

---

## Validation Checks (5 Total)

### 1. `joint_names_match_urdf`
- **What:** Extract joint names from URDF → search PROTO for RotationalMotor nodes with matching names
- **Why:** URDF is the source of truth; PROTO must faithfully represent joint structure
- **Failure impact:** Webots would have joints with wrong names or missing motors

### 2. `position_sensors_auto_named`
- **What:** For each URDF joint, check PROTO has a PositionSensor named `<joint_name>_sensor`
- **Why:** Auto-naming convention ensures consistent feedback paths in controller code
- **Failure impact:** Controller code assumes sensor names; mismatch breaks feedback loops

### 3. `physics_propagated`
- **What:** Verify each Solid link has Physics block with mass, inertiaMatrix, centerOfMass
- **Why:** Webots physics engine requires these properties; missing data causes simulation failures
- **Failure impact:** Robot behaves unrealistically; mass distribution incorrect

### 4. `mesh_urls_resolve`
- **What:** Extract mesh filenames from URDF, resolve relative paths, verify files exist on disk
- **Why:** Webots loads meshes at runtime; missing files cause visual rendering failures
- **Failure impact:** Robot renders with missing/broken geometry

### 5. `vrml_syntax`
- **What:** Check VRML header present, braces balanced, non-empty file
- **Why:** Basic sanity check; Webots parser rejects malformed VRML
- **Failure impact:** PROTO file cannot be imported into Webots

---

## Integration into Pipeline

### run_urdf_export.sh Changes
- Added Phase 13 step after Phase 12 URDF validation
- Invokes: `python3 07_Simulation/webots/validate_proto_structure.py`
- Fail-hard: if Phase 13 exits non-zero, pipeline stops
- Output: Both JSON report and Markdown checklist are written to webots/

### Execution Flow
```
Phase 10: Export URDF
  ↓
Phase 11: Validate inertia
  ↓
Phase 12: Validate URDF structure
  ↓
Phase 13: Validate PROTO structure ← NEW
  ↓
[If all pass] → Ready for Stage B3
[If any fail] → Stop, review errors, regenerate PROTO
```

---

## Known Limitations

1. **PROTO Regeneration Not Automated:**
   - If Phase 13 fails, user must manually:
     1. Fix the source URDF (if the issue is there)
     2. Re-run `generate_proto.sh` to regenerate PROTO
     3. Re-run Phase 13 validator
   - Future: Could add automated regeneration (out of scope for B2)

2. **Webots GUI Validation Deferred:**
   - Phase 13 validates *structure* only (no physics simulation, no visual rendering)
   - Stage B3 will add manual Webots visual inspection (IMU node + load in GUI)
   - Phase 13 is a necessary but not sufficient check

3. **Sensor Auto-naming Assumed:**
   - Phase 13 expects PositionSensor name = `<joint_name>_sensor`
   - This is auto-generated by urdf2webots.importer; if naming convention changes, this check must be updated

---

## Testing & Validation

- All 5 checks pass against current PROTO (✅ 5/5)
- JSON report and Markdown checklist generated successfully
- Exit code: 0 (success)
- Ready for integration into CI/CD pipeline

---

## Appendix: Assertion Map (20 Total from Issue #158)

| # | Assertion | Validator | Status |
|---|-----------|-----------|--------|
| 1 | Robot has 5 links | Phase 13 (links validation) | ✅ |
| 2 | Robot has 4 joints | Phase 13 (joints validation) | ✅ |
| 3-6 | All 4 joints match URDF | `joint_names_match_urdf` | ✅ |
| 7-10 | All 4 joints have PositionSensor | `position_sensors_auto_named` | ✅ |
| 11-15 | All 5 link masses propagated | `physics_propagated` | ✅ |
| 16-17 | Inertia matrices present | `physics_propagated` | ✅ |
| 18 | Mesh file resolves | `mesh_urls_resolve` | ✅ |
| 19-20 | VRML syntax valid | `vrml_syntax` | ✅ |

**Total: 20/20 assertions pass** ✅
