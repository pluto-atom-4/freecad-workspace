#!/usr/bin/env python3
"""
One-off servo mesh repair utility using pymeshlab.

Targets:
  - feetech-STS3032-visual-1.0mm.stl
  - feetech-STS3032-collision-proxy.stl

Repairs self-intersections and non-manifold geometry per approved thresholds:
  - Self-intersections: significantly reduced + FreeCAD warning non-blocking
  - Non-manifolds: aim for hasNonManifolds() == False
  - Dimension drift: bbox delta < 0.05mm per axis

Does NOT commit — report generated, caller decides.
"""

import pymeshlab
from pymeshlab import PureValue
import json
from pathlib import Path


VISUAL_STL = Path(__file__).parent.parent / "Mechanical" / "feetech-STS3032-visual-1.0mm.stl"
COLLISION_STL = Path(__file__).parent.parent / "Mechanical" / "feetech-STS3032-collision-proxy.stl"

BBOX_TOLERANCE_MM = 0.05  # per axis, in mm


def bbox_to_dict(bbox):
    """Convert pymeshlab BoundingBox to dict with keys."""
    # BoundingBox.min() and max() return numpy arrays
    min_pt = bbox.min()
    max_pt = bbox.max()
    return {
        "min": tuple(min_pt),
        "max": tuple(max_pt),
    }


def bbox_delta(bbox_before, bbox_after):
    """Compute per-axis delta in mm. Returns dict and max delta."""
    deltas = {}
    for axis, idx in [("x", 0), ("y", 1), ("z", 2)]:
        delta = abs(bbox_after["max"][idx] - bbox_before["max"][idx]) + \
                abs(bbox_after["min"][idx] - bbox_before["min"][idx])
        deltas[axis] = delta
    max_delta = max(deltas.values())
    return deltas, max_delta


def repair_mesh(stl_path):
    """Repair one mesh. Return (success, report_dict)."""
    print(f"\n{'='*70}")
    print(f"REPAIRING: {stl_path.name}")
    print(f"{'='*70}")

    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(str(stl_path))

    # Record baseline
    baseline_vertices = ms.current_mesh().vertex_number()
    baseline_faces = ms.current_mesh().face_number()
    baseline_bbox = bbox_to_dict(ms.current_mesh().bounding_box())

    print(f"\nBaseline:")
    print(f"  Vertices: {baseline_vertices}")
    print(f"  Faces: {baseline_faces}")
    print(f"  BBox min: {baseline_bbox['min']}")
    print(f"  BBox max: {baseline_bbox['max']}")

    # Count self-intersections BEFORE
    ms.compute_selection_by_self_intersections_per_face()
    before_self_int_faces = ms.current_mesh().selected_face_number()
    print(f"  Self-intersecting faces (BEFORE): {before_self_int_faces}")

    # Remove selected self-intersecting faces
    if before_self_int_faces > 0:
        ms.meshing_remove_selected_faces()
        print(f"  → Removed {before_self_int_faces} self-intersecting faces")

    # Repair non-manifolds BEFORE closing holes (close_holes requires manifold)
    print(f"\n  Repairing non-manifold edges...")
    ms.meshing_repair_non_manifold_edges()
    print(f"  ✓ Non-manifold edges repaired")

    print(f"  Repairing non-manifold vertices...")
    ms.meshing_repair_non_manifold_vertices()
    print(f"  ✓ Non-manifold vertices repaired")

    # Close holes with real parameters from step 1
    print(f"\n  Closing holes...")
    ms.meshing_close_holes(
        maxholesize=30,
        selected=False,
        newfaceselected=True,
        selfintersection=True,
        refinehole=False,
        refineholeedgelen=PureValue(1.402515172958374)
    )
    print(f"  → Holes closed")

    # Cleanup
    print(f"\n  Cleaning up geometry...")
    ms.meshing_remove_duplicate_faces()
    print(f"  ✓ Duplicate faces removed")

    ms.meshing_remove_duplicate_vertices()
    print(f"  ✓ Duplicate vertices removed")

    ms.meshing_remove_null_faces()
    print(f"  ✓ Null faces removed")

    ms.meshing_remove_unreferenced_vertices()
    print(f"  ✓ Unreferenced vertices removed")

    # Re-orient (optional, requires fully manifold mesh)
    print(f"  Re-orienting faces coherently (if manifold)...")
    try:
        ms.meshing_re_orient_faces_coherently()
        print(f"  ✓ Faces re-oriented")
    except Exception as e:
        print(f"  ⚠ Re-orient skipped (requires full manifoldness): {type(e).__name__}")

    # Count self-intersections AFTER
    ms.compute_selection_by_self_intersections_per_face()
    after_self_int_faces = ms.current_mesh().selected_face_number()
    print(f"\n  Self-intersecting faces (AFTER): {after_self_int_faces}")

    # Record final
    final_vertices = ms.current_mesh().vertex_number()
    final_faces = ms.current_mesh().face_number()
    final_bbox = bbox_to_dict(ms.current_mesh().bounding_box())

    print(f"\nPost-repair:")
    print(f"  Vertices: {final_vertices}")
    print(f"  Faces: {final_faces}")
    print(f"  BBox min: {final_bbox['min']}")
    print(f"  BBox max: {final_bbox['max']}")

    # Check bbox delta
    deltas, max_delta = bbox_delta(baseline_bbox, final_bbox)
    print(f"\n  BBox delta per axis (mm):")
    print(f"    x: {deltas['x']:.6f}")
    print(f"    y: {deltas['y']:.6f}")
    print(f"    z: {deltas['z']:.6f}")
    print(f"    max: {max_delta:.6f} (tolerance: {BBOX_TOLERANCE_MM})")

    if max_delta > BBOX_TOLERANCE_MM:
        print(f"\n  ⚠ ERROR: BBox delta {max_delta:.6f}mm EXCEEDS {BBOX_TOLERANCE_MM}mm tolerance!")
        print(f"  ⚠ NOT saving this file.")
        return False, {
            "filename": stl_path.name,
            "status": "FAILED_BBOX_DELTA",
            "bbox_delta_exceeded": max_delta,
            "tolerance": BBOX_TOLERANCE_MM,
            "baseline": {
                "vertices": baseline_vertices,
                "faces": baseline_faces,
                "bbox": baseline_bbox,
                "self_int_faces": before_self_int_faces,
            },
            "final": {
                "vertices": final_vertices,
                "faces": final_faces,
                "bbox": final_bbox,
                "self_int_faces": after_self_int_faces,
            },
            "deltas": deltas,
        }

    # Save
    ms.save_current_mesh(str(stl_path))
    print(f"\n  ✓ Saved to {stl_path.name}")

    report = {
        "filename": stl_path.name,
        "status": "SUCCESS",
        "baseline": {
            "vertices": baseline_vertices,
            "faces": baseline_faces,
            "bbox": baseline_bbox,
            "self_int_faces": before_self_int_faces,
        },
        "final": {
            "vertices": final_vertices,
            "faces": final_faces,
            "bbox": final_bbox,
            "self_int_faces": after_self_int_faces,
        },
        "deltas": {
            "vertices": final_vertices - baseline_vertices,
            "faces": final_faces - baseline_faces,
            "bbox_per_axis_mm": deltas,
            "max_bbox_delta_mm": max_delta,
        },
        "self_int_reduction": before_self_int_faces - after_self_int_faces,
    }

    print(f"\n  Summary:")
    print(f"    Self-intersections: {before_self_int_faces} → {after_self_int_faces} (reduced by {report['self_int_reduction']})")
    print(f"    Vertices: {baseline_vertices} → {final_vertices} (Δ {report['deltas']['vertices']})")
    print(f"    Faces: {baseline_faces} → {final_faces} (Δ {report['deltas']['faces']})")
    print(f"    BBox delta: {max_delta:.6f}mm (✓ within {BBOX_TOLERANCE_MM}mm tolerance)")

    return True, report


def main():
    """Repair both meshes, report results."""
    print("\n" + "="*70)
    print("SERVO MESH REPAIR — pymeshlab")
    print("="*70)

    all_reports = []
    all_success = True

    for stl_path in [VISUAL_STL, COLLISION_STL]:
        if not stl_path.exists():
            print(f"\n⚠ ERROR: {stl_path} not found!")
            all_success = False
            continue

        success, report = repair_mesh(stl_path)
        all_reports.append(report)
        if not success:
            all_success = False

    # Final summary
    print(f"\n{'='*70}")
    print("FINAL SUMMARY")
    print(f"{'='*70}")

    for report in all_reports:
        if report["status"] == "SUCCESS":
            print(f"\n✓ {report['filename']}")
            print(f"    Self-int: {report['baseline']['self_int_faces']} → {report['final']['self_int_faces']}")
            print(f"    BBox delta: {report['deltas']['max_bbox_delta_mm']:.6f}mm")
        else:
            print(f"\n✗ {report['filename']}")
            print(f"    Status: {report['status']}")
            if "bbox_delta_exceeded" in report:
                print(f"    BBox delta exceeded: {report['bbox_delta_exceeded']:.6f}mm > {report['tolerance']}mm")

    # Save detailed report as JSON
    report_path = Path(__file__).parent / "_repair_servo_meshes_report.json"
    with open(report_path, "w") as f:
        json.dump(all_reports, f, indent=2)
    print(f"\n→ Detailed report: {report_path}")

    if not all_success:
        print(f"\n⚠ One or more repairs FAILED. See details above.")
        exit(1)

    print(f"\n✓ All repairs completed successfully.")


if __name__ == "__main__":
    main()
