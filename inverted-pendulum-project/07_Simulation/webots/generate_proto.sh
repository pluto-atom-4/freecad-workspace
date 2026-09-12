#!/bin/bash
# Generate InvertedPendulumRobot.proto from prepared URDF via urdf2webots.
#
# This script:
# 1. Calls prepare_urdf_for_webots.sh to rewrite package:// URIs.
# 2. Runs urdf2webots to generate the PROTO file.
# 3. Validates output PROTO file exists and is non-empty.
# 4. Parses urdf2webots output to confirm link/joint counts (5 links, 4 joints).
#
# Usage: ./generate_proto.sh
#   (Run from the webots/ directory.)
#
# Exits 0 on success, non-zero with clear error message on any failure.

set -euo pipefail

# Initialize mamba in this shell session (required in non-interactive scripts).
# Find the mamba initialization script in standard conda/mamba installation locations.
MAMBA_INIT=""
if [ -f "$HOME/miniforge3/etc/profile.d/mamba.sh" ]; then
    MAMBA_INIT="$HOME/miniforge3/etc/profile.d/mamba.sh"
elif [ -f "$HOME/miniconda3/etc/profile.d/mamba.sh" ]; then
    MAMBA_INIT="$HOME/miniconda3/etc/profile.d/mamba.sh"
elif [ -f "$HOME/.conda/etc/profile.d/mamba.sh" ]; then
    MAMBA_INIT="$HOME/.conda/etc/profile.d/mamba.sh"
fi

if [ -n "$MAMBA_INIT" ]; then
    # Source mamba init (sets up the mamba function and PATH)
    source "$MAMBA_INIT" >/dev/null 2>&1 || true
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
URDF_INPUT="$SCRIPT_DIR/.generated/robot_webots.urdf"
PROTO_OUTPUT="$SCRIPT_DIR/protos/InvertedPendulumRobot.proto"

# Fail fast: mamba must be available (needed for pendulum-tools env).
if ! command -v mamba >/dev/null 2>&1; then
    echo "FATAL: mamba not found on PATH — cannot run urdf2webots in pendulum-tools env." >&2
    echo "Is mamba installed and on PATH?" >&2
    exit 1
fi

echo "Preparing URDF and generating PROTO..."

# Step 1: Prepare URDF (rewrite package:// URIs).
if ! "$SCRIPT_DIR/prepare_urdf_for_webots.sh"; then
    echo "FATAL: URDF preparation failed." >&2
    exit 1
fi

if [ ! -s "$URDF_INPUT" ]; then
    echo "FATAL: prepared URDF not found or empty: $URDF_INPUT" >&2
    exit 1
fi

echo ""
echo "Running urdf2webots (R2025a target)..."

# Step 2: Run urdf2webots in the pendulum-tools env.
# Capture output for link/joint count validation.
PROTO_OUTPUT_DIR="$(dirname "$PROTO_OUTPUT")"
mkdir -p "$PROTO_OUTPUT_DIR"

URDF2WEBOTS_OUTPUT=$(mamba run -n pendulum-tools python3 -m urdf2webots.importer \
    --input="$URDF_INPUT" \
    --output="$PROTO_OUTPUT" \
    --target=R2025a 2>&1) || {
    echo "FATAL: urdf2webots conversion failed." >&2
    echo "urdf2webots output:" >&2
    echo "$URDF2WEBOTS_OUTPUT" >&2
    echo "" >&2
    echo "Is urdf2webots installed in pendulum-tools? Install with:" >&2
    echo "  mamba install -n pendulum-tools -c conda-forge urdf2webots" >&2
    exit 1
}

# Step 3: Validate output PROTO file exists and is non-empty.
if [ ! -s "$PROTO_OUTPUT" ]; then
    echo "FATAL: urdf2webots completed but output PROTO not found or empty: $PROTO_OUTPUT" >&2
    exit 1
fi

# Step 4: Parse urdf2webots output to validate link/joint counts.
# Expected: 5 links (Base_Link, Wheel_Left, Wheel_Right, Pendulum_Link, Pendulum_Link_Right)
#           4 joints (wheel_left_joint, wheel_right_joint, pendulum_pivot_joint, pendulum_pivot_right_joint)
LINK_COUNT=$(echo "$URDF2WEBOTS_OUTPUT" | grep -oE "[0-9]+ links" | grep -oE "[0-9]+" | head -1 || true)
JOINT_COUNT=$(echo "$URDF2WEBOTS_OUTPUT" | grep -oE "[0-9]+ joints" | grep -oE "[0-9]+" | head -1 || true)

# Provide defaults if counts not found in output (urdf2webots may not always print them).
LINK_COUNT="${LINK_COUNT:-0}"
JOINT_COUNT="${JOINT_COUNT:-0}"

if [ "$LINK_COUNT" -eq 0 ]; then
    echo "FATAL: Failed to parse link count from urdf2webots output." >&2
    echo "urdf2webots output:" >&2
    echo "$URDF2WEBOTS_OUTPUT" >&2
    exit 1
fi

if [ "$JOINT_COUNT" -eq 0 ]; then
    echo "FATAL: Failed to parse joint count from urdf2webots output." >&2
    echo "urdf2webots output:" >&2
    echo "$URDF2WEBOTS_OUTPUT" >&2
    exit 1
fi

if [ "$LINK_COUNT" -ne 5 ]; then
    echo "FATAL: urdf2webots reported $LINK_COUNT links, expected 5." >&2
    echo "The source robot.urdf may have changed. Rebuild with 10_export_urdf.py." >&2
    exit 1
fi

if [ "$JOINT_COUNT" -ne 4 ]; then
    echo "FATAL: urdf2webots reported $JOINT_COUNT joints, expected 4." >&2
    echo "The source robot.urdf may have changed. Rebuild with 10_export_urdf.py." >&2
    exit 1
fi

# Step 5: Post-process PROTO to inject castShadows FALSE for high-triangle-count meshes.
# The feetech-STS3032-visual mesh has ~37556 triangles, exceeding Webots' 21845-triangle
# limit. Webots warns about shadow casting on oversized meshes; suppress with castShadows FALSE.
# This post-processor is idempotent: if castShadows already exists in the Shape, it skips.

echo ""
echo "Post-processing PROTO to suppress shadow-casting warnings..."

POST_PROCESS_RESULT=$(PROTO_OUTPUT_FILE="$PROTO_OUTPUT" python3 << 'PYTHON_EOF'
import re
import sys
import os

# proto_file passed as an environment variable from the bash script
proto_file = os.environ.get('PROTO_OUTPUT_FILE')
if not proto_file:
    print("FATAL: PROTO_OUTPUT_FILE environment variable not set", file=sys.stderr)
    sys.exit(1)

try:
    with open(proto_file, 'r') as f:
        content = f.read()
except IOError as e:
    print(f"FATAL: Cannot read PROTO file {proto_file}: {e}", file=sys.stderr)
    sys.exit(1)

def find_shape_blocks(text):
    """Find all Shape blocks and yield (shape_content, is_matching_shape)."""
    lines = text.split('\n')
    i = 0
    while i < len(lines):
        if 'Shape {' in lines[i]:
            shape_lines = [lines[i]]
            brace_count = lines[i].count('{') - lines[i].count('}')
            j = i + 1

            while j < len(lines) and brace_count > 0:
                shape_lines.append(lines[j])
                brace_count += lines[j].count('{') - lines[j].count('}')
                j += 1

            shape_content = '\n'.join(shape_lines)
            is_matching = 'feetech-STS3032-visual' in shape_content
            yield (shape_content, is_matching)
            i = j
        else:
            i += 1

# First pass: count matching Shape blocks (expected: 2 DEF + USE)
matching_count = sum(1 for _, is_matching in find_shape_blocks(content) if is_matching)

if matching_count != 2:
    print(f"FATAL: Expected 2 Shape blocks referencing feetech-STS3032-visual, found {matching_count} — urdf2webots output shape may have changed, castShadows injection cannot proceed safely.", file=sys.stderr)
    sys.exit(1)

# Second pass: inject castShadows where needed
changes_made = 0
already_patched = 0
lines = content.split('\n')
output_lines = []
i = 0

while i < len(lines):
    line = lines[i]
    output_lines.append(line)

    if 'Shape {' in line:
        shape_lines = [line]
        brace_count = line.count('{') - line.count('}')
        j = i + 1

        while j < len(lines) and brace_count > 0:
            shape_lines.append(lines[j])
            brace_count += lines[j].count('{') - lines[j].count('}')
            j += 1

        shape_content = '\n'.join(shape_lines)
        if 'feetech-STS3032-visual' in shape_content:
            if 'castShadows FALSE' not in shape_content:
                # Inject castShadows FALSE before the closing brace
                last_brace_idx = len(shape_lines) - 1
                injected = False

                while last_brace_idx >= 0:
                    if '}' in shape_lines[last_brace_idx]:
                        last_brace_line = shape_lines[last_brace_idx]

                        # Safety check: line must contain exactly one closing brace
                        brace_count_in_line = last_brace_line.count('}')
                        if brace_count_in_line != 1:
                            print(f"FATAL: Ambiguous brace placement: target line contains {brace_count_in_line} closing braces. Cannot safely inject castShadows.", file=sys.stderr)
                            sys.exit(1)

                        last_brace_pos = last_brace_line.rfind('}')
                        if last_brace_pos >= 0:
                            indent = len(last_brace_line) - len(last_brace_line.lstrip())
                            modified_line = (
                                last_brace_line[:last_brace_pos] +
                                '\n' + ' ' * (indent + 2) + 'castShadows FALSE\n' +
                                ' ' * indent + last_brace_line[last_brace_pos:]
                            )
                            shape_lines[last_brace_idx] = modified_line
                            changes_made += 1
                            injected = True
                        break
                    last_brace_idx -= 1

                if not injected:
                    print(f"FATAL: Could not find closing brace to inject castShadows in Shape block.", file=sys.stderr)
                    sys.exit(1)
            else:
                already_patched += 1

        for k in range(1, len(shape_lines)):
            output_lines.append(shape_lines[k])

        i = j
    else:
        i += 1

modified_content = '\n'.join(output_lines)

# Validate basic VRML syntax: check brace matching
open_braces = modified_content.count('{')
close_braces = modified_content.count('}')
if open_braces != close_braces:
    print(f"FATAL: Brace mismatch in modified PROTO: {open_braces} {{ vs {close_braces} }}", file=sys.stderr)
    sys.exit(1)

# Post-injection validation: verify each matching Shape block contains castShadows FALSE
castShadows_count = 0
for shape_content, is_matching in find_shape_blocks(modified_content):
    if is_matching:
        if 'castShadows FALSE' not in shape_content:
            print(f"FATAL: Post-injection validation failed: Shape block with feetech-STS3032-visual does not contain castShadows FALSE.", file=sys.stderr)
            sys.exit(1)
        castShadows_count += 1

if castShadows_count != matching_count:
    print(f"FATAL: Post-injection validation failed: expected {matching_count} Shape blocks with castShadows FALSE, found {castShadows_count}.", file=sys.stderr)
    sys.exit(1)

# Write back if changes were made
if changes_made > 0:
    try:
        with open(proto_file, 'w') as f:
            f.write(modified_content)
        print(f"Injected castShadows FALSE into {changes_made} feetech-STS3032-visual Shape block(s).")
    except IOError as e:
        print(f"FATAL: Cannot write PROTO file {proto_file}: {e}", file=sys.stderr)
        sys.exit(1)
elif already_patched == matching_count:
    print(f"No changes needed (castShadows FALSE already present in all {matching_count} matching Shape blocks).")
else:
    print(f"FATAL: Unexpected state: matching_count={matching_count}, already_patched={already_patched}, changes_made={changes_made}.", file=sys.stderr)
    sys.exit(1)

sys.exit(0)
PYTHON_EOF
) || {
    echo "FATAL: Post-processing failed." >&2
    exit 1
}

echo "$POST_PROCESS_RESULT"

echo ""
echo "=================================================================="
echo "PROTO generation complete."
echo "  Output: $PROTO_OUTPUT"
echo "  urdf2webots reported: $LINK_COUNT links, $JOINT_COUNT joints"
echo "=================================================================="
exit 0
