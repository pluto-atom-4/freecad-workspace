#!/usr/bin/env python3
"""
Phase 6 Day 4: Multi-mesh Assembly Merger using Trimesh

Merges multiple STL meshes with optional per-mesh transforms into a single
composite STL file. Includes mesh composition tracking and validation.

Process:
1. Load configuration (mesh files and transforms)
2. Load each STL mesh with specified transforms (translation, rotation, scaling)
3. Apply transforms to each mesh
4. Merge all meshes into single composition
5. Validate merged result (watertight, bounds, geometry)
6. Export merged STL
7. Generate comprehensive JSON metadata report

Usage:
    python3 06_trimesh_merge_for_stl.py --config merge_config.json --output merged.stl
"""

import sys
import json
import argparse
import time
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any, List
import numpy as np


def validate_imports():
    """Validate required imports are available."""
    try:
        import trimesh
        return trimesh
    except ImportError:
        print("ERROR: trimesh not available. Install with: pip install trimesh")
        sys.exit(1)


@dataclass
class TransformSpec:
    """Transform specification for a mesh."""
    translate_mm: List[float]
    rotate_euler_deg: List[float]
    scale: float

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'TransformSpec':
        """Create from dictionary."""
        return cls(
            translate_mm=data.get('translate_mm', [0, 0, 0]),
            rotate_euler_deg=data.get('rotate_euler_deg', [0, 0, 0]),
            scale=data.get('scale', 1.0),
        )


@dataclass
class MeshInfo:
    """Information about a mesh in the assembly."""
    name: str
    file: str
    vertices_original: int
    vertices_in_merge: int
    transform: TransformSpec


@dataclass
class MergedGeometry:
    """Geometry statistics for merged mesh."""
    total_vertices: int
    total_faces: int
    volume_mm3: float
    surface_area_mm2: float
    bounds: Dict[str, List[float]]


class MeshMerger:
    """Merges multiple meshes with transforms."""

    def __init__(self):
        """Initialize mesh merger."""
        self.trimesh = validate_imports()
        self.meshes: List[tuple] = []  # (name, mesh, original_vertices)
        self.merged_mesh = None
        self.mesh_infos: List[MeshInfo] = []

    def load_mesh(self, file_path: str, name: str, transform_spec: TransformSpec) -> bool:
        """Load and transform a single mesh."""
        try:
            mesh_path = Path(file_path)
            if not mesh_path.exists():
                print(f"ERROR: Mesh file not found: {mesh_path}")
                return False

            # Load mesh
            mesh = self.trimesh.load(str(mesh_path))
            vertices_original = len(mesh.vertices)

            # Apply transforms
            self._apply_transform(mesh, transform_spec)

            # Store mesh and info
            self.meshes.append((name, mesh, vertices_original))
            print(f"Loaded {name}: {vertices_original} vertices from {file_path}")

            return True

        except Exception as e:
            print(f"ERROR: Failed to load mesh {file_path}: {e}")
            return False

    def _apply_transform(self, mesh, transform_spec: TransformSpec) -> None:
        """Apply translation, rotation, and scale to mesh."""
        # Scale
        if transform_spec.scale != 1.0:
            mesh.apply_scale(transform_spec.scale)

        # Rotation (Euler angles: roll, pitch, yaw)
        if any(a != 0 for a in transform_spec.rotate_euler_deg):
            angles_rad = np.array(transform_spec.rotate_euler_deg) * np.pi / 180.0
            # Create rotation matrix from Euler angles (ZYX order)
            rotation_matrix = self._euler_to_matrix(angles_rad)
            mesh.apply_transform(rotation_matrix)

        # Translation
        if any(t != 0 for t in transform_spec.translate_mm):
            mesh.apply_translation(transform_spec.translate_mm)

    @staticmethod
    def _euler_to_matrix(euler_rad: np.ndarray) -> np.ndarray:
        """Convert Euler angles (roll, pitch, yaw) to 4x4 transformation matrix."""
        roll, pitch, yaw = euler_rad

        # Rotation matrices for each axis
        Rx = np.array([
            [1, 0, 0, 0],
            [0, np.cos(roll), -np.sin(roll), 0],
            [0, np.sin(roll), np.cos(roll), 0],
            [0, 0, 0, 1]
        ])

        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch), 0],
            [0, 1, 0, 0],
            [-np.sin(pitch), 0, np.cos(pitch), 0],
            [0, 0, 0, 1]
        ])

        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0, 0],
            [np.sin(yaw), np.cos(yaw), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Combined rotation: Rz @ Ry @ Rx
        return Rz @ Ry @ Rx

    def merge(self) -> bool:
        """Merge all loaded meshes into single composition."""
        if not self.meshes:
            print("ERROR: No meshes loaded")
            return False

        try:
            # Use trimesh.util.concatenate to merge
            mesh_list = [mesh for _, mesh, _ in self.meshes]
            self.merged_mesh = self.trimesh.util.concatenate(mesh_list)

            # Build mesh info list
            for name, mesh, vertices_original in self.meshes:
                vertices_in_merge = len(mesh.vertices)
                transform_spec = None
                # We need to store transform specs during load_mesh
                self.mesh_infos.append(MeshInfo(
                    name=name,
                    file="",  # Will be set from config
                    vertices_original=vertices_original,
                    vertices_in_merge=vertices_in_merge,
                    transform=transform_spec
                ))

            print(f"Merged {len(self.meshes)} meshes into composition")
            print(f"  Total vertices: {len(self.merged_mesh.vertices)}")
            print(f"  Total faces: {len(self.merged_mesh.faces)}")

            return True

        except Exception as e:
            print(f"ERROR: Merge failed: {e}")
            import traceback
            traceback.print_exc()
            return False

    def validate(self) -> Dict[str, Any]:
        """Validate merged mesh properties."""
        if self.merged_mesh is None:
            raise RuntimeError("No merged mesh. Call merge() first.")

        validation = {
            "is_valid": True,
            "is_watertight": self.merged_mesh.is_watertight,
            "vertex_count": len(self.merged_mesh.vertices),
            "face_count": len(self.merged_mesh.faces),
        }

        try:
            # Check for issues
            if self.merged_mesh.volume < 0:
                validation["is_valid"] = False
                validation["warning"] = "Negative volume (inverted faces)"
        except Exception:
            pass

        return validation

    def get_geometry_stats(self) -> MergedGeometry:
        """Calculate merged mesh geometry statistics."""
        if self.merged_mesh is None:
            raise RuntimeError("No merged mesh. Call merge() first.")

        bounds_min = self.merged_mesh.bounds[0]
        bounds_max = self.merged_mesh.bounds[1]

        return MergedGeometry(
            total_vertices=len(self.merged_mesh.vertices),
            total_faces=len(self.merged_mesh.faces),
            volume_mm3=float(self.merged_mesh.volume),
            surface_area_mm2=float(self.merged_mesh.area),
            bounds={
                "x": [float(bounds_min[0]), float(bounds_max[0])],
                "y": [float(bounds_min[1]), float(bounds_max[1])],
                "z": [float(bounds_min[2]), float(bounds_max[2])],
            }
        )

    def export_merged(self, output_path: str) -> bool:
        """Export merged mesh to STL file."""
        if self.merged_mesh is None:
            raise RuntimeError("No merged mesh. Call merge() first.")

        try:
            output_file = Path(output_path)
            output_file.parent.mkdir(parents=True, exist_ok=True)
            self.merged_mesh.export(str(output_file))
            print(f"Exported merged mesh to: {output_file}")
            return True
        except Exception as e:
            print(f"ERROR: Failed to export mesh: {e}")
            return False

    def generate_metadata(self, merge_name: str, config: Dict[str, Any],
                         output_path: Optional[str] = None) -> Dict[str, Any]:
        """Generate JSON metadata report."""
        validation = self.validate()
        geometry = self.get_geometry_stats()

        # Build meshes_included list with transform info
        meshes_included = []
        for i, (name, mesh, vertices_original) in enumerate(self.meshes):
            transform_spec = config.get('meshes', [{}])[i].get('transform', {})
            mesh_entry = {
                "name": name,
                "file": config.get('meshes', [{}])[i].get('file', ''),
                "vertices_original": vertices_original,
                "vertices_in_merge": len(mesh.vertices),
                "transform": {
                    "translate": transform_spec.get('translate_mm', [0, 0, 0]),
                    "rotate": transform_spec.get('rotate_euler_deg', [0, 0, 0]),
                    "scale": transform_spec.get('scale', 1.0),
                }
            }
            meshes_included.append(mesh_entry)

        metadata = {
            "merge_name": merge_name,
            "status": "success",
            "mesh_count": len(self.meshes),
            "meshes_included": meshes_included,
            "merged_geometry": asdict(geometry),
            "validation": validation,
            "output_file": output_path or "",
        }

        return metadata


def load_config(config_path: str) -> Dict[str, Any]:
    """Load merge configuration from JSON file."""
    config_file = Path(config_path)
    if not config_file.exists():
        raise FileNotFoundError(f"Config file not found: {config_file}")

    with open(config_file, 'r') as f:
        config = json.load(f)

    # Validate config structure
    required_keys = ['merge_name', 'meshes']
    for key in required_keys:
        if key not in config:
            raise ValueError(f"Config missing required key: {key}")

    if not isinstance(config['meshes'], list) or len(config['meshes']) == 0:
        raise ValueError("Config 'meshes' must be non-empty list")

    return config


def main():
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Merge multiple STL meshes with per-mesh transforms"
    )
    parser.add_argument(
        "--config", "-c",
        required=True,
        help="Configuration JSON file with mesh list and transforms"
    )
    parser.add_argument(
        "--output", "-o",
        help="Output STL file path (overrides config if provided)"
    )
    parser.add_argument(
        "--metadata",
        help="Output JSON metadata file path"
    )

    args = parser.parse_args()

    start_time = time.time()

    try:
        # Load configuration
        config = load_config(args.config)
        merge_name = config.get('merge_name', 'merged_assembly')
        output_file = args.output or config.get('output_file', f'{merge_name}_merged.stl')

        print(f"Merge Configuration: {merge_name}")
        print(f"=" * 60)

        # Create merger
        merger = MeshMerger()

        # Load all meshes
        config_dir = Path(args.config).parent
        for mesh_config in config['meshes']:
            mesh_file = mesh_config['file']
            mesh_name = mesh_config['name']

            # Resolve relative paths
            if not Path(mesh_file).is_absolute():
                mesh_file = str(config_dir / mesh_file)

            # Get transform spec
            transform_dict = mesh_config.get('transform', {})
            transform_spec = TransformSpec.from_dict(transform_dict)

            if not merger.load_mesh(mesh_file, mesh_name, transform_spec):
                sys.exit(1)

        # Merge meshes
        if not merger.merge():
            sys.exit(1)

        # Validate
        validation = merger.validate()
        geometry = merger.get_geometry_stats()

        print(f"\nMerge Result Validation")
        print(f"=" * 60)
        print(f"Watertight: {validation['is_watertight']}")
        print(f"Valid: {validation['is_valid']}")
        print(f"Total Vertices: {validation['vertex_count']}")
        print(f"Total Faces: {validation['face_count']}")
        print(f"Volume: {geometry.volume_mm3:.2f} mm³")
        print(f"Surface Area: {geometry.surface_area_mm2:.2f} mm²")

        # Export
        if not merger.export_merged(output_file):
            sys.exit(1)

        # Generate metadata
        metadata = merger.generate_metadata(merge_name, config, output_file)

        # Add execution time
        execution_time = time.time() - start_time
        metadata["execution_time_seconds"] = round(execution_time, 2)

        # Save metadata
        metadata_file = args.metadata or f"{Path(output_file).stem}_metadata.json"
        metadata_path = Path(metadata_file)
        metadata_path.parent.mkdir(parents=True, exist_ok=True)
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)

        print(f"\nMetadata saved to: {metadata_path}")
        print(f"Execution time: {metadata['execution_time_seconds']}s")

        return 0

    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
