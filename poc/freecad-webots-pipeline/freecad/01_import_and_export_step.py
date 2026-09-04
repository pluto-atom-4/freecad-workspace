#!/usr/bin/env python3
"""
Stage 1 — FreeCAD import & STEP export (headless).

For each of burger_base.stl, left_tire.stl, right_tire.stl fetched by
../00_fetch_turtlebot3_assets.sh into ../reference/meshes/, this script:
  1. Imports the STL as a Mesh.Mesh
  2. Converts the mesh to a Part.Shape (per-facet makePolygon -> Face ->
     makeShell -> makeSolid, same pattern as
     inverted-pendulum-project/03_Parts/Generators/01_convert_servo_stl_to_step_via_freecad.py)
  3. Exports the resulting solid to STEP in output/

Run via ./01_import_and_export_step.sh (sets FREECAD_BIN and invokes this
script inside freecadcmd), or directly:
    FREECAD_BIN=~/.local/opt/freecad-1.1.3/usr/bin/freecadcmd \
        "$FREECAD_BIN" -c "exec(open('01_import_and_export_step.py').read())"

Writes output/stage1_conversion_report.json (mirrors the
*_conversion_report.json schema convention used in Generators/).
"""

import json
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import FreeCAD as App
import Part
import Mesh

# ============================================================================
# CONFIGURATION
# ============================================================================

try:
    SCRIPT_DIR = Path(__file__).resolve().parent
except NameError:
    # freecadcmd -c "exec(open(path).read())" does not set __file__ inside
    # the exec'd code, and guessing it from cwd is fragile (a stale cwd
    # from an earlier `cd` silently produced a duplicated, wrong output
    # tree during POC development — see git history). Fail loudly instead:
    # the .sh wrapper injects __file__ explicitly, so this fallback should
    # not normally be hit.
    raise RuntimeError(
        "__file__ is not defined and no safe fallback is used. "
        "Run this script via 01_import_and_export_step.sh (which injects "
        "__file__), or invoke it as: "
        "freecadcmd -c \"__file__=r'/abs/path/to/this_script.py'; "
        "exec(open(__file__).read())\""
    )

POC_DIR = SCRIPT_DIR.parent
MESHES_DIR = POC_DIR / "reference" / "meshes"
OUTPUT_DIR = SCRIPT_DIR / "output"
REPORT_OUTPUT = OUTPUT_DIR / "stage1_conversion_report.json"

# (source_stl_filename, output_step_stem)
LINKS: List[str] = [
    "burger_base.stl",
    "left_tire.stl",
    "right_tire.stl",
]


# ============================================================================
# CONVERSION FUNCTIONS (adapted from Generators/01_convert_servo_stl_to_step_via_freecad.py)
# ============================================================================

def load_mesh(stl_path: Path) -> Optional["Mesh.Mesh"]:
    print(f"\nLoading STL mesh: {stl_path.name}")
    try:
        mesh = Mesh.Mesh(str(stl_path))
        print(f"  Vertices: {mesh.CountPoints}  Facets: {mesh.CountFacets}")
        return mesh
    except Exception as e:
        print(f"  FAILED to load: {e}")
        return None


def mesh_bbox_dict(mesh: "Mesh.Mesh") -> Dict[str, float]:
    bbox = mesh.BoundBox
    return {
        "min_x": bbox.XMin, "max_x": bbox.XMax,
        "min_y": bbox.YMin, "max_y": bbox.YMax,
        "min_z": bbox.ZMin, "max_z": bbox.ZMax,
        "width": bbox.XLength, "height": bbox.YLength, "depth": bbox.ZLength,
    }


def mesh_to_shape(mesh: "Mesh.Mesh", link_name: str) -> Optional[Part.Shape]:
    """Convert FreeCAD Mesh to Part Shape via per-facet polygon/face/shell/solid.

    Same algorithm as Generators/01_convert_servo_stl_to_step_via_freecad.py.
    burger_base.stl has a much higher facet count than the servo mesh that
    pattern was originally built for; timing/robustness at this scale is
    itself a Stage 4 finding, not something to silently work around here.
    """
    facet_count = mesh.CountFacets
    print(f"\nConverting {facet_count} facets to Part faces ({link_name})...")

    t_start = time.monotonic()
    faces = []
    failed_facets = 0
    for i, facet in enumerate(mesh.Facets):
        try:
            p1, p2, p3 = facet.Points[0], facet.Points[1], facet.Points[2]
            v1, v2, v3 = App.Vector(p1), App.Vector(p2), App.Vector(p3)
            wire = Part.makePolygon([v1, v2, v3, v1])
            face = Part.Face(wire)
            faces.append(face)
        except Exception as e:
            failed_facets += 1
            if failed_facets <= 5:
                print(f"  WARN: facet {i} failed: {e}")

        if (i + 1) % 5000 == 0:
            elapsed = time.monotonic() - t_start
            print(f"  {i + 1}/{facet_count} faces converted... ({elapsed:.1f}s elapsed)")

    faces_elapsed = time.monotonic() - t_start
    print(f"  Faces built: {len(faces)} ({failed_facets} failed) in {faces_elapsed:.1f}s")

    try:
        t_shell = time.monotonic()
        shell = Part.makeShell(faces)
        shell_elapsed = time.monotonic() - t_shell
        print(f"  Shell created in {shell_elapsed:.1f}s")

        t_solid = time.monotonic()
        solid = Part.makeSolid(shell)
        solid_elapsed = time.monotonic() - t_solid
        print(f"  Solid created ({solid.ShapeType}) in {solid_elapsed:.1f}s")

        total_elapsed = time.monotonic() - t_start
        return solid, {
            "facet_count": facet_count,
            "faces_built": len(faces),
            "failed_facets": failed_facets,
            "faces_build_seconds": round(faces_elapsed, 2),
            "shell_build_seconds": round(shell_elapsed, 2),
            "solid_build_seconds": round(solid_elapsed, 2),
            "total_seconds": round(total_elapsed, 2),
        }
    except Exception as e:
        print(f"  FAILED shell/solid conversion: {e}")
        import traceback
        traceback.print_exc()
        total_elapsed = time.monotonic() - t_start
        return None, {
            "facet_count": facet_count,
            "faces_built": len(faces),
            "failed_facets": failed_facets,
            "faces_build_seconds": round(faces_elapsed, 2),
            "total_seconds": round(total_elapsed, 2),
            "error": str(e),
        }


def export_step(shape: Part.Shape, step_path: Path, doc_name: str) -> bool:
    try:
        doc = App.newDocument(doc_name)
        obj = doc.addObject("Part::Feature", "Link_Shape")
        obj.Shape = shape
        doc.recompute()
        Part.export([obj], str(step_path))
        App.closeDocument(doc.Name)
        return step_path.exists()
    except Exception as e:
        print(f"  FAILED STEP export: {e}")
        import traceback
        traceback.print_exc()
        return False


# ============================================================================
# MAIN WORKFLOW
# ============================================================================

def run() -> bool:
    print("=" * 70)
    print("Stage 1: FreeCAD import & STEP export (TurtleBot3 Burger links)")
    print("=" * 70)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    report: Dict[str, Any] = {
        "stage": "stage1_import_and_export_step",
        "source_dir": str(MESHES_DIR),
        "output_dir": str(OUTPUT_DIR),
        "links": {},
        "status": "FAILED",
    }

    all_ok = True
    for filename in LINKS:
        link_name = Path(filename).stem
        stl_path = MESHES_DIR / filename
        step_path = OUTPUT_DIR / f"{link_name}.step"

        link_report: Dict[str, Any] = {
            "source_stl": str(stl_path),
            "output_step": str(step_path),
            "status": "FAILED",
        }

        if not stl_path.exists():
            print(f"\nFAILED: source STL not found: {stl_path}")
            link_report["error"] = "source STL not found"
            report["links"][link_name] = link_report
            all_ok = False
            continue

        mesh = load_mesh(stl_path)
        if mesh is None:
            link_report["error"] = "mesh load failed"
            report["links"][link_name] = link_report
            all_ok = False
            continue

        link_report["source_mesh"] = {
            "vertex_count": mesh.CountPoints,
            "facet_count": mesh.CountFacets,
            "surface_area_mm2": mesh.Area,
            "bounding_box_mm": mesh_bbox_dict(mesh),
        }

        shape, conversion_metrics = mesh_to_shape(mesh, link_name)
        link_report["conversion"] = conversion_metrics

        if shape is None:
            link_report["error"] = "mesh-to-shape conversion failed"
            report["links"][link_name] = link_report
            all_ok = False
            continue

        ok = export_step(shape, step_path, f"Stage1_{link_name}")
        if not ok:
            link_report["error"] = "STEP export failed"
            report["links"][link_name] = link_report
            all_ok = False
            continue

        shape_bbox = shape.BoundBox
        link_report["shape_shape_type"] = shape.ShapeType
        link_report["shape_bounding_box_mm"] = {
            "min_x": shape_bbox.XMin, "max_x": shape_bbox.XMax,
            "min_y": shape_bbox.YMin, "max_y": shape_bbox.YMax,
            "min_z": shape_bbox.ZMin, "max_z": shape_bbox.ZMax,
        }
        link_report["shape_volume_mm3"] = shape.Volume
        link_report["step_file_size_kb"] = round(step_path.stat().st_size / 1024, 1)
        link_report["status"] = "SUCCESS"
        print(f"  -> STEP exported: {step_path.name} ({link_report['step_file_size_kb']} KB)")

        report["links"][link_name] = link_report

    report["status"] = "SUCCESS" if all_ok else "PARTIAL_FAILURE"

    with open(REPORT_OUTPUT, "w") as f:
        json.dump(report, f, indent=2, default=str)
    print(f"\nReport written: {REPORT_OUTPUT}")
    print(f"\nStage 1 status: {report['status']}")

    return all_ok


if __name__ == "__main__":
    ok = run()
    import sys
    sys.exit(0 if ok else 1)
else:
    run()
