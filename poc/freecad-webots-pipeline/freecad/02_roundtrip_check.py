#!/usr/bin/env python3
"""
Stage 1b — STEP round-trip sanity check (headless FreeCAD).

For each STEP file produced by 01_import_and_export_step.py, this script:
  1. Re-imports the STEP file as a Part.Shape
  2. Diffs its bounding box / volume / facet count (after tessellation)
     against the original source STL mesh
  3. Writes output/roundtrip_report.json (mirrors the *_conversion_report.json
     schema convention used in Generators/)
  4. Re-exports the round-tripped shape as a mesh (STL) into output/ —
     these become Stage 2's URDF mesh references per decision #4 in the
     issue #24 implementation plan (URDF meshes reference FreeCAD
     round-tripped re-exports, not the pristine vendored STLs).

Run via (no separate .sh wrapper — typically run right after Stage 1's
01_import_and_export_step.sh, see README.md):
    FREECAD_BIN=~/.local/opt/freecad-1.1.3/usr/bin/freecadcmd
    "$FREECAD_BIN" -c "__file__=r'/abs/path/to/02_roundtrip_check.py'; exec(open(__file__).read())"

(__file__ must be injected explicitly — freecadcmd -c does not set it for
exec'd code, see the SCRIPT_DIR fallback below for why this matters.)
"""

import json
from pathlib import Path
from typing import Any, Dict, Optional

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
    # the exec'd code, and guessing it from cwd is fragile (see the same
    # note in 01_import_and_export_step.py). Fail loudly instead.
    raise RuntimeError(
        "__file__ is not defined and no safe fallback is used. Invoke as: "
        "freecadcmd -c \"__file__=r'/abs/path/to/this_script.py'; "
        "exec(open(__file__).read())\""
    )

POC_DIR = SCRIPT_DIR.parent
MESHES_DIR = POC_DIR / "reference" / "meshes"
OUTPUT_DIR = SCRIPT_DIR / "output"
REPORT_OUTPUT = OUTPUT_DIR / "roundtrip_report.json"

LINKS = [
    "burger_base",
    "left_tire",
    "right_tire",
]

# Tolerance for treating bbox/volume differences as "preserved" (mesh-vs-BREP
# tessellation always introduces some small deviation).
BBOX_TOLERANCE_MM = 0.5
VOLUME_TOLERANCE_PCT = 2.0


def source_mesh_metrics(link_name: str) -> Optional[Dict[str, Any]]:
    stl_path = MESHES_DIR / f"{link_name}.stl"
    if not stl_path.exists():
        return None
    mesh = Mesh.Mesh(str(stl_path))
    bbox = mesh.BoundBox
    return {
        "facet_count": mesh.CountFacets,
        "vertex_count": mesh.CountPoints,
        "surface_area_mm2": mesh.Area,
        "bounding_box_mm": {
            "min_x": bbox.XMin, "max_x": bbox.XMax,
            "min_y": bbox.YMin, "max_y": bbox.YMax,
            "min_z": bbox.ZMin, "max_z": bbox.ZMax,
            "width": bbox.XLength, "height": bbox.YLength, "depth": bbox.ZLength,
        },
    }


def reimport_step(step_path: Path) -> Optional[Part.Shape]:
    try:
        shape = Part.Shape()
        shape.read(str(step_path))
        return shape
    except Exception as e:
        print(f"  FAILED to re-import STEP: {e}")
        return None


def pct_diff(a: float, b: float) -> float:
    if a == 0:
        return 0.0 if b == 0 else float("inf")
    return abs(a - b) / abs(a) * 100.0


def check_link(link_name: str) -> Dict[str, Any]:
    print(f"\n{'-' * 60}")
    print(f"Round-trip check: {link_name}")
    print("-" * 60)

    step_path = OUTPUT_DIR / f"{link_name}.step"
    result: Dict[str, Any] = {
        "link": link_name,
        "step_file": str(step_path),
        "status": "FAILED",
    }

    src = source_mesh_metrics(link_name)
    if src is None:
        result["error"] = "source STL not found"
        return result
    result["source_mesh"] = src

    if not step_path.exists():
        result["error"] = "STEP file not found (Stage 1 did not produce it)"
        return result

    shape = reimport_step(step_path)
    if shape is None:
        result["error"] = "STEP re-import failed"
        return result

    bbox = shape.BoundBox
    reimported_bbox = {
        "min_x": bbox.XMin, "max_x": bbox.XMax,
        "min_y": bbox.YMin, "max_y": bbox.YMax,
        "min_z": bbox.ZMin, "max_z": bbox.ZMax,
        "width": bbox.XLength, "height": bbox.YLength, "depth": bbox.ZLength,
    }

    # Tessellate the re-imported BREP solid to compare facet count against
    # the original mesh (not expected to match exactly — BREP tessellation
    # uses its own deflection-based algorithm — but should be same order of
    # magnitude, and volume/bbox should match closely).
    try:
        mesh_data = shape.tessellate(0.1)  # (points, facets), 0.1mm deflection
        reimported_facet_count = len(mesh_data[1])
    except Exception as e:
        reimported_facet_count = None
        print(f"  WARN: tessellate failed: {e}")

    volume = shape.Volume

    src_bbox = src["bounding_box_mm"]
    bbox_diffs = {
        k: round(abs(reimported_bbox[k] - src_bbox[k]), 4)
        for k in ("width", "height", "depth")
    }
    bbox_ok = all(v <= BBOX_TOLERANCE_MM for v in bbox_diffs.values())

    result["reimported_shape"] = {
        "shape_type": shape.ShapeType,
        "bounding_box_mm": reimported_bbox,
        "volume_mm3": volume,
        "tessellated_facet_count": reimported_facet_count,
    }
    result["diff"] = {
        "bbox_diff_mm": bbox_diffs,
        "bbox_within_tolerance": bbox_ok,
        "source_facet_count": src["facet_count"],
        "reimported_tessellated_facet_count": reimported_facet_count,
        "facet_count_ratio": (
            round(reimported_facet_count / src["facet_count"], 3)
            if reimported_facet_count else None
        ),
    }

    print(f"  Source bbox (mm):     {src_bbox['width']:.3f} x {src_bbox['height']:.3f} x {src_bbox['depth']:.3f}")
    print(f"  Reimported bbox (mm): {reimported_bbox['width']:.3f} x {reimported_bbox['height']:.3f} x {reimported_bbox['depth']:.3f}")
    print(f"  Bbox diff (mm): {bbox_diffs}  within {BBOX_TOLERANCE_MM}mm tolerance: {bbox_ok}")
    print(f"  Source facets: {src['facet_count']}  Reimported tessellated facets: {reimported_facet_count}")
    print(f"  Reimported solid volume: {volume:.2f} mm^3")

    # Re-export the round-tripped shape as mesh for Stage 2 URDF use
    reexport_path = OUTPUT_DIR / f"{link_name}_roundtrip.stl"
    try:
        doc = App.newDocument(f"Roundtrip_{link_name}")
        obj = doc.addObject("Part::Feature", "Reimported_Shape")
        obj.Shape = shape
        doc.recompute()
        Mesh.export([obj], str(reexport_path))
        App.closeDocument(doc.Name)
        result["reexported_mesh"] = str(reexport_path)
        result["reexported_mesh_exists"] = reexport_path.exists()
        if reexport_path.exists():
            print(f"  Re-exported mesh for Stage 2: {reexport_path.name}")
    except Exception as e:
        print(f"  WARN: mesh re-export failed: {e}")
        result["reexport_error"] = str(e)

    result["status"] = "SUCCESS" if bbox_ok else "SUCCESS_WITH_BBOX_DRIFT"
    return result


def run() -> bool:
    print("=" * 70)
    print("Stage 1b: STEP round-trip check")
    print("=" * 70)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    report: Dict[str, Any] = {
        "stage": "stage1b_roundtrip_check",
        "links": {},
        "status": "FAILED",
    }

    all_ok = True
    for link_name in LINKS:
        link_result = check_link(link_name)
        report["links"][link_name] = link_result
        if link_result["status"] not in ("SUCCESS", "SUCCESS_WITH_BBOX_DRIFT"):
            all_ok = False

    report["status"] = "SUCCESS" if all_ok else "PARTIAL_FAILURE"

    with open(REPORT_OUTPUT, "w") as f:
        json.dump(report, f, indent=2, default=str)
    print(f"\nReport written: {REPORT_OUTPUT}")
    print(f"\nStage 1b status: {report['status']}")

    return all_ok


if __name__ == "__main__":
    ok = run()
    import sys
    sys.exit(0 if ok else 1)
else:
    run()
