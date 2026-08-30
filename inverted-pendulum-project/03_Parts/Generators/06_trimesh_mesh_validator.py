#!/usr/bin/env python3
"""
Phase 6: Mesh Validation and Repair using Trimesh

Validates STL mesh files and auto-repairs common issues.

Process:
1. Load STL mesh file using trimesh
2. Validate mesh properties (manifold, vertex count, bounds, volume)
3. Auto-repair if requested:
   - Merge duplicate vertices
   - Fill holes
   - Remove degenerate faces
4. Output comprehensive JSON report with validation and repair stats
5. Export repaired mesh if requested

Usage:
    python3 06_trimesh_mesh_validator.py --input model.stl [--auto-repair] [--output repaired.stl]
"""

import sys
import json
import argparse
import time
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any, List


def validate_imports():
    """Validate required imports are available."""
    try:
        import trimesh
        return trimesh
    except ImportError:
        print("ERROR: trimesh not available. Install with: pip install trimesh")
        sys.exit(1)


@dataclass
class ValidationResult:
    """Mesh validation result."""
    is_watertight: bool
    vertex_count_original: int
    vertex_count_repaired: Optional[int] = None
    face_count_original: int = 0
    face_count_repaired: Optional[int] = None
    open_edges: int = 0


@dataclass
class GeometryStats:
    """Mesh geometry statistics."""
    volume_mm3: float
    surface_area_mm2: float
    bounds: Dict[str, List[float]]
    centroid: Dict[str, float]
    edge_length_mean: float
    edge_length_min: float
    edge_length_max: float


class MeshValidator:
    """Validates and repairs 3D meshes using trimesh."""

    def __init__(self, input_path: str):
        """Initialize validator with mesh file."""
        self.input_path = Path(input_path)
        self.trimesh = validate_imports()

        if not self.input_path.exists():
            raise FileNotFoundError(f"Mesh file not found: {self.input_path}")

        self.mesh = None
        self.mesh_original = None
        self.repairs_applied: List[str] = []

    def load_mesh(self) -> bool:
        """Load mesh from file."""
        try:
            self.mesh = self.trimesh.load(str(self.input_path))
            self.mesh_original = self.mesh.copy()
            return True
        except Exception as e:
            print(f"ERROR: Failed to load mesh: {e}")
            return False

    def validate(self) -> ValidationResult:
        """Validate mesh properties."""
        if self.mesh is None:
            raise RuntimeError("Mesh not loaded. Call load_mesh() first.")

        # Count open edges (edges that belong to only one face)
        try:
            open_edges = len([e for e in self.mesh.edges_unique
                             if len(self.mesh.edges_face[e]) == 1])
        except Exception:
            open_edges = 0

        result = ValidationResult(
            is_watertight=self.mesh.is_watertight,
            vertex_count_original=len(self.mesh.vertices),
            face_count_original=len(self.mesh.faces),
            open_edges=open_edges,
        )

        return result

    def get_geometry_stats(self) -> GeometryStats:
        """Calculate mesh geometry statistics."""
        if self.mesh is None:
            raise RuntimeError("Mesh not loaded. Call load_mesh() first.")

        # Calculate edge lengths
        edges = self.mesh.edges_unique
        edge_vectors = (self.mesh.vertices[edges[:, 1]] -
                       self.mesh.vertices[edges[:, 0]])
        edge_lengths = self.trimesh.util.row_norm(edge_vectors)

        bounds_min = self.mesh.bounds[0]
        bounds_max = self.mesh.bounds[1]

        centroid = self.mesh.centroid

        return GeometryStats(
            volume_mm3=float(self.mesh.volume),
            surface_area_mm2=float(self.mesh.area),
            bounds={
                "x": [float(bounds_min[0]), float(bounds_max[0])],
                "y": [float(bounds_min[1]), float(bounds_max[1])],
                "z": [float(bounds_min[2]), float(bounds_max[2])],
            },
            centroid={
                "x": float(centroid[0]),
                "y": float(centroid[1]),
                "z": float(centroid[2]),
            },
            edge_length_mean=float(edge_lengths.mean()),
            edge_length_min=float(edge_lengths.min()),
            edge_length_max=float(edge_lengths.max()),
        )

    def repair(self) -> bool:
        """Auto-repair mesh issues."""
        if self.mesh is None:
            raise RuntimeError("Mesh not loaded. Call load_mesh() first.")

        before_vertices = len(self.mesh.vertices)
        before_faces = len(self.mesh.faces)

        try:
            # Remove degenerate faces (zero area)
            if len(self.mesh.faces) > 0:
                valid_mask = self.mesh.area_faces > 1e-8
                valid_count = valid_mask.sum()
                if valid_count < len(self.mesh.faces):
                    removed = len(self.mesh.faces) - valid_count
                    self.mesh.update_faces(valid_mask)
                    self.repairs_applied.append(f"removed_degenerate_faces: {removed}")

            # Merge duplicate/close vertices
            before_merge = len(self.mesh.vertices)
            self.mesh.merge_vertices()
            merged = before_merge - len(self.mesh.vertices)
            if merged > 0:
                self.repairs_applied.append(f"merged_duplicate_vertices: {merged}")

            # Fill holes (small holes only - max 10 edges)
            if not self.mesh.is_watertight:
                boundary_edges = self.mesh.outline()
                for loop in boundary_edges:
                    if len(loop.entities) <= 10:
                        try:
                            self.mesh.fill_holes()
                            self.repairs_applied.append("filled_holes: multiple")
                            break
                        except Exception:
                            pass

            # Remove unused vertices
            before_unused = len(self.mesh.vertices)
            self.mesh.remove_unreferenced_vertices()
            unused = before_unused - len(self.mesh.vertices)
            if unused > 0:
                self.repairs_applied.append(f"removed_unreferenced_vertices: {unused}")

            after_vertices = len(self.mesh.vertices)
            after_faces = len(self.mesh.faces)

            print(f"Repairs applied:")
            print(f"  Vertices: {before_vertices} -> {after_vertices}")
            print(f"  Faces: {before_faces} -> {after_faces}")
            for repair in self.repairs_applied:
                print(f"    - {repair}")

            return True

        except Exception as e:
            print(f"WARNING: Repair failed partially: {e}")
            return False

    def export_mesh(self, output_path: str) -> bool:
        """Export repaired mesh to file."""
        if self.mesh is None:
            raise RuntimeError("Mesh not loaded. Call load_mesh() first.")

        try:
            output_file = Path(output_path)
            output_file.parent.mkdir(parents=True, exist_ok=True)
            self.mesh.export(str(output_file))
            print(f"Exported repaired mesh to: {output_file}")
            return True
        except Exception as e:
            print(f"ERROR: Failed to export mesh: {e}")
            return False

    def generate_report(self, auto_repair: bool = False, output_path: Optional[str] = None) -> Dict[str, Any]:
        """Generate comprehensive JSON report."""
        validation = self.validate()
        geometry = self.get_geometry_stats()

        status = "valid"
        if not validation.is_watertight:
            status = "non_watertight"
        if auto_repair:
            self.repair()
            validation.vertex_count_repaired = len(self.mesh.vertices)
            validation.face_count_repaired = len(self.mesh.faces)
            status = "repaired"
            if output_path:
                self.export_mesh(output_path)

        # Update geometry stats if repaired
        if auto_repair:
            geometry = self.get_geometry_stats()

        report = {
            "status": status,
            "input_file": str(self.input_path),
            "validation": {
                "is_watertight": validation.is_watertight,
                "vertex_count_original": validation.vertex_count_original,
                "vertex_count_repaired": validation.vertex_count_repaired,
                "face_count_original": validation.face_count_original,
                "face_count_repaired": validation.face_count_repaired,
                "open_edges": validation.open_edges,
            },
            "geometry": asdict(geometry),
            "repairs_applied": self.repairs_applied if auto_repair else [],
        }

        return report


def main():
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Validate and repair 3D meshes using trimesh"
    )
    parser.add_argument(
        "--input", "-i",
        required=True,
        help="Input mesh file (STL, OBJ, etc.)"
    )
    parser.add_argument(
        "--auto-repair", "-r",
        action="store_true",
        help="Automatically repair common mesh issues"
    )
    parser.add_argument(
        "--output", "-o",
        help="Output path for repaired mesh (requires --auto-repair)"
    )
    parser.add_argument(
        "--report", "-j",
        help="JSON report output path"
    )

    args = parser.parse_args()

    # Validate args
    if args.output and not args.auto_repair:
        print("ERROR: --output requires --auto-repair flag")
        sys.exit(1)

    start_time = time.time()

    try:
        # Load and validate
        validator = MeshValidator(args.input)
        if not validator.load_mesh():
            sys.exit(1)

        # Generate report (with optional repair)
        report = validator.generate_report(
            auto_repair=args.auto_repair,
            output_path=args.output
        )

        # Add execution time
        execution_time = time.time() - start_time
        report["execution_time_seconds"] = round(execution_time, 2)

        # Print summary
        print(f"\nMesh Validation Report")
        print(f"=" * 50)
        print(f"Input: {args.input}")
        print(f"Status: {report['status'].upper()}")
        print(f"Vertices: {report['validation']['vertex_count_original']}", end="")
        if report['validation']['vertex_count_repaired']:
            print(f" -> {report['validation']['vertex_count_repaired']}", end="")
        print()
        print(f"Faces: {report['validation']['face_count_original']}", end="")
        if report['validation']['face_count_repaired']:
            print(f" -> {report['validation']['face_count_repaired']}", end="")
        print()
        print(f"Volume: {report['geometry']['volume_mm3']:.2f} mm³")
        print(f"Surface Area: {report['geometry']['surface_area_mm2']:.2f} mm²")
        print(f"Watertight: {report['validation']['is_watertight']}")
        print(f"Execution time: {report['execution_time_seconds']}s")

        # Save JSON report
        if args.report:
            report_path = Path(args.report)
            report_path.parent.mkdir(parents=True, exist_ok=True)
            with open(report_path, 'w') as f:
                json.dump(report, f, indent=2)
            print(f"\nReport saved to: {report_path}")

        return 0

    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
