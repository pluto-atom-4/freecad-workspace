#!/usr/bin/env python3
"""
Unify URDF mesh path resolution and rewriting logic.

This module provides pure Python functions to:
  1. Resolve package:// URIs and relative paths to absolute paths on disk.
  2. Rewrite resolved paths to relative paths (for Webots consumption).

Used by:
  - 12_validate_urdf_export.py (validate_mesh_paths)
  - prepare_urdf_for_webots.sh (sed replacement via CLI)

Issue #161: Consolidate duplicated/hardcoded path logic.
"""

import sys
import argparse
import re
import os
from pathlib import Path
from typing import Optional


def resolve_mesh_path(filename: str, urdf_dir: Path) -> Path:
    """Resolve a mesh filename (package:// URI or relative path) to absolute path.

    Args:
        filename: Mesh filename from <mesh filename="..."> element.
                  Can be:
                    - package://inverted_pendulum_robot/meshes/feetech-STS3032-visual.stl
                    - meshes/feetech-STS3032-visual.stl
                    - ../../../06_Exports/urdf/meshes/...
        urdf_dir: Directory containing the URDF file (used as base for resolution).

    Returns:
        Absolute Path to the mesh file.

    Notes:
        - package:// URIs are resolved by finding the "meshes" token in the path
          and reconstructing from that point relative to urdf_dir.
        - If no "meshes" token, falls back to using the last 2-3 path components.
        - Relative paths are resolved directly relative to urdf_dir.
    """
    if filename.startswith('package://'):
        # package://robot_name/meshes/file.stl → urdf_dir/meshes/file.stl
        # or package://robot_name/path/to/file.stl → urdf_dir/path/to/file.stl (fallback)
        parts = filename.split('/')
        if len(parts) >= 3:
            try:
                # Find "meshes" token if present
                meshes_idx = parts.index('meshes')
                # Reconstruct from meshes onward
                relative_path = '/'.join(parts[meshes_idx:])
                mesh_path = urdf_dir / relative_path
            except ValueError:
                # No "meshes" directory in the path
                # Fallback: use last 2 path components if available
                if len(parts) >= 4:
                    relative_path = '/'.join(parts[-2:])
                else:
                    relative_path = parts[-1]
                mesh_path = urdf_dir / relative_path
        else:
            # Very short path; just use the filename
            mesh_path = urdf_dir / filename
    else:
        # Relative path: resolve relative to URDF directory
        mesh_path = urdf_dir / filename

    return mesh_path.resolve()


def rewrite_for_webots(filename: str, urdf_dir: Path, webots_generated_dir: Path) -> str:
    """Rewrite a mesh path to be relative to the Webots .generated directory.

    Args:
        filename: Original mesh filename (package:// URI or relative path).
        urdf_dir: Directory containing the source URDF.
        webots_generated_dir: Directory where the output URDF will live
                              (usually webots/.generated/).

    Returns:
        Relative path string suitable for the rewritten URDF (relative to webots_generated_dir).

    Example:
        Input: filename="package://inverted_pendulum_robot/meshes/feetech-STS3032-visual.stl"
               urdf_dir=Path("/workspace/06_Exports/urdf")
               webots_generated_dir=Path("/workspace/07_Simulation/webots/.generated")
        Output: "../../../06_Exports/urdf/meshes/feetech-STS3032-visual.stl"
    """
    # Resolve to absolute path
    resolved_abs = resolve_mesh_path(filename, urdf_dir)

    # Compute relative path from webots_generated_dir to the resolved file
    # Use os.path.relpath which works for any two paths (not just subpaths)
    relative = os.path.relpath(str(resolved_abs), str(webots_generated_dir.resolve()))
    return relative


def rewrite_urdf_file(input_path: Path, output_path: Path, urdf_dir: Path, webots_generated_dir: Path) -> None:
    """Rewrite all mesh paths in a URDF file.

    Replaces all <mesh filename="..."> attributes with rewritten paths.

    Args:
        input_path: Path to source URDF file.
        output_path: Path to output URDF file.
        urdf_dir: Directory containing the source URDF.
        webots_generated_dir: Target .generated directory for the output URDF.

    Raises:
        FileNotFoundError: If input_path does not exist.
    """
    if not input_path.exists():
        raise FileNotFoundError(f"Input URDF not found: {input_path}")

    with open(input_path, 'r') as f:
        content = f.read()

    # Replace all mesh filenames: <mesh filename="...">
    # Regex pattern: capture the filename value between quotes
    pattern = r'<mesh filename="([^"]+)"'

    def replace_mesh_filename(match):
        old_filename = match.group(1)
        try:
            new_filename = rewrite_for_webots(old_filename, urdf_dir, webots_generated_dir)
            return f'<mesh filename="{new_filename}"'
        except Exception as e:
            # If rewrite fails, log and keep original (will be caught by post-checks)
            print(f"WARNING: Failed to rewrite mesh path '{old_filename}': {e}", file=sys.stderr)
            return match.group(0)

    rewritten = re.sub(pattern, replace_mesh_filename, content)

    # Ensure output directory exists
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, 'w') as f:
        f.write(rewritten)


def main():
    """CLI interface for URDF rewriting."""
    parser = argparse.ArgumentParser(
        description='Rewrite mesh paths in URDF for Webots consumption.'
    )
    subparsers = parser.add_subparsers(dest='command', help='Command to run')

    # 'rewrite' subcommand
    rewrite_parser = subparsers.add_parser('rewrite', help='Rewrite mesh paths in URDF')
    rewrite_parser.add_argument('--input', type=Path, required=True, help='Input URDF file')
    rewrite_parser.add_argument('--output', type=Path, required=True, help='Output URDF file')
    rewrite_parser.add_argument('--urdf-dir', type=Path, required=True, help='Directory containing the source URDF')
    rewrite_parser.add_argument('--webots-dir', type=Path, required=True, help='Target .generated directory')

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        sys.exit(1)

    if args.command == 'rewrite':
        try:
            rewrite_urdf_file(args.input, args.output, args.urdf_dir, args.webots_dir)
            print(f"URDF rewritten successfully: {args.output}")

            # Post-check: ensure no package:// URIs remain
            with open(args.output, 'r') as f:
                output_content = f.read()
            if 'package://' in output_content:
                print("ERROR: package:// URI found in output after rewrite.", file=sys.stderr)
                remaining = [line for line in output_content.split('\n') if 'package://' in line]
                for line in remaining:
                    print(f"  {line}", file=sys.stderr)
                sys.exit(1)

            sys.exit(0)
        except Exception as e:
            print(f"ERROR: {e}", file=sys.stderr)
            sys.exit(1)


if __name__ == '__main__':
    main()
