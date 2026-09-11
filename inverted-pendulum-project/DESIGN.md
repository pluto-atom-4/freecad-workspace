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
| 7 — Body+wheels (Issue #9 Stage 1) | `07_create_body_and_wheels.py` | `robot_body_wheels.FCStd`, `07_body_wheels_metadata.json` | 🔄 redesign in progress on `feat/issue-9-stage1-body-wheels` (see PR #52) |
| 8 — Assembly joints (Issue #9 Stage 2) | `08_configure_assembly_joints.py` + `test_08_configure_assembly_joints.py` | `robot_assembly.FCStd`, `08_assembly_joints_metadata.json` | 🔄 (see Issue #63 fix in PR #66) |
| 9 — Servo mass properties (Issue #85 Stage 3) | `09_compute_mass_properties.py` + `test_09_compute_mass_properties.py` | `09_mass_properties.json` | ✅ |
| 10 — URDF export (Issue #84 Stage 4) | `10_export_urdf.py` + `test_10_export_urdf.py` | `06_Exports/urdf/robot.urdf`, `06_Exports/urdf/meshes/feetech-STS3032-visual.stl`, `10_urdf_export_metadata.json` | ✅ |

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
echo "exec(open('03_Parts/Generators/02_position_servo.py').read())" | "$FREECAD_BIN" -c
# ...same stdin-pipe form for 03_/04_/07_ -- see root CLAUDE.md's "Common Development
# Commands" section for why the plain positional/--python form silently no-ops in this
# environment, and why `-c "exec(...)"` as an argument (not piped) also silently no-ops.
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
| A `Mesh::Feature`'s own `Placement` is ignored when nested in an `App::Part` | Composing it manually (as for `Part::Feature`) puts the mesh far from where it actually renders | Only the parent container's `getGlobalPlacement()` applied to the raw `.Mesh` data matches the render — see root `CLAUDE.md` |
| `Mesh.transform()` with a reflection matrix can silently produce wrongly-offset geometry | A true mirror of a servo mesh landed with one coordinate shifted by an unexplained, large offset | Mirror via raw point data instead (negate a coordinate, reverse facet winding, rebuild the mesh) — see root `CLAUDE.md` |
| `distToShape() == 0` doesn't distinguish touching from overlapping | A "0mm clearance" collision check can be a false alarm or a false pass | Use `shape_a.common(shape_b).Volume` for a definitive check |
| `03_link_servo_to_assembly.py`'s `create_servo_body()` reuses `STS3032_Mount`'s mesh children **by name** if they already exist in `plates_servo_assembled.FCStd` — it never re-reads the source `.stl` on a rerun once those objects exist (`Mesh.Mesh(str(stl_path))` only runs on first creation, see the script's own docstring/`else` branch) | Repairing/replacing the source STL (issue #76) and rerunning Phase 3 silently keeps the OLD cached mesh data — every downstream stage (7, 8) then propagates the stale mesh, with no error or warning | Before rerunning Phase 3 after changing a servo STL, delete the existing `feetech_STS3032_visual_1_0mm`/`feetech_STS3032_collision_proxy` objects from `plates_servo_assembled.FCStd` first (headless script, `doc.removeObject(name)` + `doc.save()`), then rerun 03→07→08. Verify by checking imported vs. reused in Phase 3's console output ("Imported ..." vs "✓ Reusing existing mesh: ...") and comparing `Mesh.CountFacets` against the source STL's known post-repair count. |
| Stage 3's servo mass (09_compute_mass_properties.py) sources volume/CoM/inertia from **collision-proxy mesh** (clean geometry), **excluding** the visual mesh due to residual self-intersections (Issue #76) | The visual mesh is non-solid and self-intersecting; its volume is mathematically unreliable (~1037 mm³ vs. the correct collision-proxy's ~11308 mm³). Prior tolerance was ±90% to accommodate this broken measurement; now tightened to ±30% since collision-proxy is clean. Mass itself still comes from datasheet-sourced `target_mass_kg` | Once Issue #8 measures real servo mass, update `robot_parameters.yaml`'s `servo.target_mass_kg` (the full pipeline re-runs automatically); validate the collision-proxy volume against empirical hardware caliper data and further tighten `test_09_compute_mass_properties.py`'s tolerance if needed. |
| Stage 3 computes servo inertia from mesh-native API (FreeCAD `Mesh.MatrixOfInertia`), never STEP round-trip | STEP conversion loses volume fidelity catastrophically (see `poc/freecad-webots-pipeline/findings/FINDINGS.md` sec.3: burger_base 180K→73K mm³, 60% loss + fragmentation into 89 invalid solids) | Mesh-native is direct, reliable, and fast — use it exclusively for servo mass/inertia/CoM (Issue #85 decision) |
| Stage 10 (URDF export): Plate-stack CoM approximated as center-of-bounding-box, not true centroid | Plate stack's geometric centroid is not explicitly computed; center of bbox [7.995, 18.276, 4.5] mm is used as a reasonable approximation (verified live against `PlateStack` object in `robot_assembly.FCStd`) | Acceptable for Stage 4's ~3% inertia tolerance; if hardware measurement (Issue #8) demands higher precision, compute true centroid from the plate geometry's triangulation |
| Stage 10: Servo collision geometry split (box + cylinder) derived from fixed constants, not live mesh shape | Servo's collision envelope is two primitives (32×12×28 mm box, 4.65 mm radius × 16.15 mm height cylinder) with fixed positions relative to servo mount — these were determined by analyzing the STEP spec and are re-verified against `collision_proxy_bbox_mm` (live envelope from Stage 3) with 10% tolerance | Primitives are intentionally separate (not a unified convex hull) to keep physics simulation budget tight and allow independent component tuning. If servo hardware changes (e.g., shaft protrusion length), constants must be manually updated — no automatic derivation |
| Stage 10: Visual mesh tolerance locked at 1.0 mm | Only the 1.0 mm tolerance `.stl` is currently packaged and copied to the URDF export; coarser meshes (5.0 mm, 10 mm) exist in `03_Parts/Mechanical/` but aren't wired into the export pipeline yet | To enable other tolerances, extend `copy_visual_mesh(tolerance_mm)` in `10_export_urdf.py` to accept `--visual-tolerance 5.0` or similar CLI flag, then check for the corresponding `.stl` file before copying |
| Stage 9/10 (`09_compute_mass_properties.py`, `10_export_urdf.py`) bypass the already-built `02_Design_Inputs/robot_parameters.py` loader — each parses `robot_parameters.yaml` directly with its own local `yaml.safe_load()` call instead of importing the shared loader `07_create_body_and_wheels.py` already uses correctly | Two independent, divergence-prone copies of the same YAML-reading logic; schema/validation changes to the shared loader (e.g. dataclass field renames) silently don't reach Stage 3/4 | Tracked as Issue #88 — refactor both scripts to import and use `robot_parameters.py` like Stage 1 does |

## Related Work

Issue #9 Stage 1 (`07_create_body_and_wheels.py`, PR #52) is a downstream consumer of this
pipeline's `plates_servo_assembled.FCStd` — not a new phase of the servo-integration work above.
A human review of that Stage 1 artifact found it didn't match what was pictured, despite passing
every dimensional/structural check — see Issue #53 before starting Stage 2+ or any other new
FreeCAD generation work here.

Following that review, `07_create_body_and_wheels.py` is being redesigned iteratively on
`feat/issue-9-stage1-body-wheels`: `Base_Link` is now a flat plate (not a solid box);
`Wheel_Left`/`Wheel_Right` each mount on their own `Bottom_Plate`'s real mounting hole instead of
a body-centerline formula; `Pendulum_Link` tilts 90° so its plates stand parallel to the wheels;
`Pendulum_Link_Right` exists as a full second copy, built from literal constants rather than a
geometric mirror (a true reflection mirror hit the `Mesh.transform()` bug above). The working
pattern throughout: a human transforms objects live via the FreeCAD MCP bridge, values get
verified for real collisions (`Part.Shape.common()`, not just distance), then get ported into the
script as named constants with a note on where they came from — not re-derived from a formula.

## References

- GitHub Issue #3 (servo integration), Issue #9 (Stage 1+), Issue #53 (human-intent-gap writeup)
- `../CLAUDE.md`, `README.md`

---
**Last Updated:** 2026-09-11 (Stage 4 URDF export complete, Issue #84)
