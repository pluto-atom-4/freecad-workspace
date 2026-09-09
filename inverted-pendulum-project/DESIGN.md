# Servo Motor Assembly & Artifact Production

Integration of the Feetech STS3032 servo into the FreeCAD plates assembly (Issue #3), plus
downstream reuse by Issue #9 Stage 1. Directory: `03_Parts/Generators/` (scripts + `.FCStd`
working files) and `03_Parts/Mechanical/` (servo STEP/STL assets, gitignored except the
committed default-tolerance pair).

## Assembly Architecture

Three-body system: **Middle_Plate** (mounting base, Edge26/Edge34), **Servo_Motor** (external
STEP reference + 6-DOF placement matrix), **Top/Bottom_Plate** (clearance targets). Servo is
linked externally (not embedded) to keep the assembly `.FCStd` small (<20KB vs. 36+MB embedded)
and enable parametric updates without re-linking.

## Phases (03_Parts/Generators/)

| Phase | Script | Output | Status |
|---|---|---|---|
| 1 — STL→STEP conversion | `01_convert_servo_stl_to_step.py` | `feetech-STS3032.step` (36MB) | ✅ |
| 2 — Alignment calc | `02_position_servo.py` + `test_02_servo_position.py` | `servo_placement.json` | ✅ |
| 3 — Assembly link | `03_link_servo_to_assembly.py` | `plates_servo_assembled.FCStd`, `servo_link_config.json` | ✅ |
| 4 — Export artifacts | `04_export_assembly_merged.py` | `plates_assembled_with_servo.{step,stl}`, `export_metadata.json` | ✅ |
| 7 — Body+wheels (Issue #9) | `07_create_body_and_wheels.py` | `robot_body_wheels.FCStd`, `07_body_wheels_metadata.json` | ✅ (PR #52) |

> **Note (issue #22):** Phase 2's `servo_placement.json` clearance check is Z-only and ignores
> each plate's independent rotation — it originally misreported Bottom_Plate's clearance as
> ~4.85mm (failing the 5.0mm minimum). Phase 3 uses real 3D shape distance
> (`Part.Shape.distToShape`) instead; actual clearance is ~17.1mm (Bottom) / ~6.43mm (Top).

## Usage

```bash
cd inverted-pendulum-project
export FREECAD_BIN=~/.local/bin/freecadcmd1.1   # headless binary for phases 2-4, 7

# Phase 1 (plain python, wraps FreeCAD via subprocess)
python3 03_Parts/Generators/01_convert_servo_stl_to_step.py

# Phases 2-4, 7 (run inside FreeCAD's own interpreter)
"$FREECAD_BIN" -c "exec(open('03_Parts/Generators/02_position_servo.py').read())"
# ...same -c "exec(...)" form for 03_/04_/07_ -- see root CLAUDE.md's "FreeCAD Live Bridge"
# section for why the plain positional/--python form silently no-ops in this environment.
```

## Key Design Decisions

| Decision | Rationale | Trade-off |
|----------|-----------|-----------|
| External STEP link (not embedded) | Lightweight assembly file | Depends on external file staying at a stable path |
| Phase-based scripts, one concern each | Testable, repeatable | Multiple files to manage |
| JSON metadata alongside each phase | Version-control friendly | Not CAD-native |
| Merge on export (Phase 4) | Static geometry for external tools | Loses assembly constraints |
| Copy (not `App::Link`) Issue #9's reused plates/servo into a new doc | Self-contained input for Stage 2/3 | Independent snapshot; re-run Phase 7 if the source doc changes |

## Known Limitations

| Limitation | Impact | Mitigation |
|-----------|--------|-----------|
| External link breaks if servo.step moved | Assembly fails to load | Fixed relative path, documented |
| Phase 2 edge indexing brittle | Fails if plate geometry changes | Visualization helper script |
| Live GUI auto-tessellates shapes, shrinking `Shape.BoundBox` reads (~70.00mm cylinder → ~69.90mm) | Dimensional checks can fail only under a live-bridge run, not headless | Validate against a true headless run; tessellate a `.copy()`, never `obj.Shape` itself |
| Live bridge's `get_screenshot`/`inspect_object` (App::Part) broken (`freecad-mcp-workbench` 0.6.2) | No built-in screenshot/inspect for App::Part | See root `CLAUDE.md`'s "FreeCAD Live Bridge" section for workarounds |
| Visibility/camera are GUI-only state | A headless-generated `.FCStd` opens with objects invisible, no useful viewpoint | Guard visibility/camera code with `if not getattr(App, "GuiUp", False): return` (see `07_create_body_and_wheels.py`) |
| Bridge's `execute_python(code="exec(open(path).read())")` doesn't fire a script's `__main__` guard (`__name__` is `"builtins"` there) | Running a generator through the bridge this way silently does nothing | Exec with explicit globals forcing `__name__='__main__'` — see root `CLAUDE.md`'s "FreeCAD Live Bridge" section for the exact snippet; this is the only way to get a fully-viewable `.FCStd` (visibility + camera baked in) |

## Related Work

Issue #9 Stage 1 (`07_create_body_and_wheels.py`, PR #52) is a downstream consumer of this
pipeline's `plates_servo_assembled.FCStd` — not a new phase of the servo-integration work above.
A human review of that Stage 1 artifact found it didn't match what was pictured, despite passing
every dimensional/structural check — see Issue #53 before starting Stage 2+ or any other new
FreeCAD generation work here.

## References

- GitHub Issue #3 (servo integration), Issue #9 (Stage 1+), Issue #53 (human-intent-gap writeup)
- `../CLAUDE.md`, `README.md`

---
**Last Updated:** 2026-09-09
