#!/usr/bin/env python3
"""
Inject castShadows FALSE into Shape blocks referencing high-triangle-count meshes.

The feetech-STS3032-visual mesh has ~37556 triangles, exceeding Webots' 21845-triangle
limit. Webots warns about shadow casting on oversized meshes; this script injects
castShadows FALSE before closing braces in matching Shape blocks.

This script is idempotent: if castShadows already exists in the Shape, it skips.

Usage:
    python3 inject_cast_shadows.py --proto /path/to/InvertedPendulumRobot.proto

    Exit code 0: success (changes made or already patched)
    Exit code 1: fatal error (malformed PROTO or structure mismatch)
"""

import sys
import argparse
from pathlib import Path
from typing import List, Tuple

from vrml_lexer import find_matching_brace


def find_shape_blocks(content: str, mesh_name: str) -> List[Tuple[int, int]]:
    """
    Find all Shape { ... } blocks referencing mesh_name.

    Args:
        content: PROTO file content as string
        mesh_name: Mesh name to search for (e.g., "feetech-STS3032-visual")

    Returns:
        List of (start_pos, end_pos) tuples, where:
        - start_pos is the position of the '{' in 'Shape {'
        - end_pos is the position of the matching '}'
    """
    blocks = []
    search_start = 0

    while True:
        # Find next 'Shape {'
        shape_pos = content.find("Shape {", search_start)
        if shape_pos == -1:
            break

        # Open brace position is at 'Shape {'.rfind('{')
        open_brace_pos = content.rfind("{", shape_pos, shape_pos + 10)
        if open_brace_pos == -1:
            raise ValueError(f"Could not find opening brace for Shape at pos {shape_pos}")

        try:
            # Find matching close brace
            close_brace_pos = find_matching_brace(content, open_brace_pos)
        except ValueError as e:
            raise ValueError(f"Error finding matching brace for Shape at {shape_pos}: {e}")

        # Extract block content and check for mesh_name
        block_content = content[open_brace_pos : close_brace_pos + 1]
        if mesh_name in block_content:
            blocks.append((open_brace_pos, close_brace_pos))

        search_start = close_brace_pos + 1

    return blocks


def inject_cast_shadows_false(content: str, block_positions: List[Tuple[int, int]]) -> str:
    """
    Inject castShadows FALSE before closing } in each block.

    Args:
        content: PROTO file content as string
        block_positions: List of (start_pos, end_pos) tuples from find_shape_blocks()

    Returns:
        Modified content with castShadows FALSE injected

    Raises:
        ValueError: If a block already contains castShadows FALSE (already patched)
                    or if structure is invalid
    """
    if not block_positions:
        return content

    # Process blocks in reverse order to preserve positions during modification
    modified = content
    changes_made = 0

    for start_pos, end_pos in reversed(block_positions):
        # Adjust positions for any previous modifications
        block_content = modified[start_pos : end_pos + 1]

        if "castShadows FALSE" in block_content:
            # Already patched, skip
            continue

        # Find the closing brace within this block
        # We need the position in the modified string
        close_brace_offset = block_content.rfind("}")
        if close_brace_offset == -1:
            raise ValueError(
                f"Could not find closing brace in block at {start_pos}..{end_pos}"
            )

        close_brace_pos_in_modified = start_pos + close_brace_offset

        # Get indentation from the closing brace line
        line_start = modified.rfind("\n", 0, close_brace_pos_in_modified)
        if line_start == -1:
            line_start = 0
        else:
            line_start += 1

        indent_str = modified[line_start : close_brace_pos_in_modified]
        indent_len = len(indent_str) - len(indent_str.lstrip())

        # Inject castShadows FALSE with proper indentation
        indent = " " * (indent_len + 2)
        injection = f"\n{indent}castShadows FALSE\n{' ' * indent_len}"

        modified = (
            modified[:close_brace_pos_in_modified]
            + injection
            + modified[close_brace_pos_in_modified:]
        )
        changes_made += 1

    return modified, changes_made


def validate_vrml_braces(content: str) -> bool:
    """
    Validate VRML file has balanced braces.

    Args:
        content: PROTO file content as string

    Returns:
        True if braces are balanced, False otherwise
    """
    open_count = content.count("{")
    close_count = content.count("}")
    return open_count == close_count


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Inject castShadows FALSE into Shape blocks for high-triangle meshes"
    )
    parser.add_argument(
        "--proto",
        required=True,
        help="Path to PROTO file to process",
        type=str,
    )

    args = parser.parse_args()
    proto_path = Path(args.proto)

    # Validate file exists
    if not proto_path.exists():
        print(f"FATAL: PROTO file not found: {proto_path}", file=sys.stderr)
        return 1

    # Read file
    try:
        with open(proto_path, "r", encoding="utf-8") as f:
            content = f.read()
    except IOError as e:
        print(f"FATAL: Cannot read PROTO file {proto_path}: {e}", file=sys.stderr)
        return 1

    # Pre-validation: check braces balanced
    if not validate_vrml_braces(content):
        open_count = content.count("{")
        close_count = content.count("}")
        print(
            f"FATAL: Brace mismatch in input PROTO: {open_count} {{ vs {close_count} }}",
            file=sys.stderr,
        )
        return 1

    # Find Shape blocks
    mesh_name = "feetech-STS3032-visual"
    try:
        block_positions = find_shape_blocks(content, mesh_name)
    except ValueError as e:
        print(f"FATAL: {e}", file=sys.stderr)
        return 1

    # Expected: exactly 2 Shape blocks (DEF + USE)
    if len(block_positions) != 2:
        print(
            f"FATAL: Expected 2 Shape blocks referencing {mesh_name}, "
            f"found {len(block_positions)} — urdf2webots output shape may have changed, "
            f"castShadows injection cannot proceed safely.",
            file=sys.stderr,
        )
        return 1

    # Inject castShadows FALSE
    try:
        modified_content, changes_made = inject_cast_shadows_false(
            content, block_positions
        )
    except ValueError as e:
        print(f"FATAL: {e}", file=sys.stderr)
        return 1

    # Post-injection validation: check braces still balanced
    if not validate_vrml_braces(modified_content):
        open_count = modified_content.count("{")
        close_count = modified_content.count("}")
        print(
            f"FATAL: Brace mismatch after injection: {open_count} {{ vs {close_count} }}",
            file=sys.stderr,
        )
        return 1

    # Post-injection validation: verify castShadows FALSE is present in both blocks
    try:
        updated_blocks = find_shape_blocks(modified_content, mesh_name)
    except ValueError as e:
        print(f"FATAL: {e}", file=sys.stderr)
        return 1

    castShadows_count = 0
    for start_pos, end_pos in updated_blocks:
        block_content = modified_content[start_pos : end_pos + 1]
        if "castShadows FALSE" in block_content:
            castShadows_count += 1

    if castShadows_count != len(block_positions):
        print(
            f"FATAL: Post-injection validation failed: expected {len(block_positions)} "
            f"Shape blocks with castShadows FALSE, found {castShadows_count}.",
            file=sys.stderr,
        )
        return 1

    # Write back if changes were made
    if changes_made > 0:
        try:
            with open(proto_path, "w", encoding="utf-8") as f:
                f.write(modified_content)
            print(
                f"Injected castShadows FALSE into {changes_made} "
                f"{mesh_name} Shape block(s)."
            )
        except IOError as e:
            print(f"FATAL: Cannot write PROTO file {proto_path}: {e}", file=sys.stderr)
            return 1
    else:
        print(
            f"No changes needed (castShadows FALSE already present in all "
            f"{len(block_positions)} matching Shape blocks)."
        )

    return 0


if __name__ == "__main__":
    sys.exit(main())
