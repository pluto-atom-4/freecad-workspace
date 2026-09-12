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
LINK_COUNT=$(echo "$URDF2WEBOTS_OUTPUT" | grep -oE "[0-9]+ links" | grep -oE "[0-9]+" | head -1)
JOINT_COUNT=$(echo "$URDF2WEBOTS_OUTPUT" | grep -oE "[0-9]+ joints" | grep -oE "[0-9]+" | head -1)

# Provide defaults if counts not found in output (urdf2webots may not always print them).
LINK_COUNT="${LINK_COUNT:-0}"
JOINT_COUNT="${JOINT_COUNT:-0}"

if [ "$LINK_COUNT" -ne 0 ] && [ "$LINK_COUNT" -ne 5 ]; then
    echo "WARNING: urdf2webots reported $LINK_COUNT links, expected 5." >&2
    echo "The source robot.urdf may have changed. Please verify." >&2
fi

if [ "$JOINT_COUNT" -ne 0 ] && [ "$JOINT_COUNT" -ne 4 ]; then
    echo "WARNING: urdf2webots reported $JOINT_COUNT joints, expected 4." >&2
    echo "The source robot.urdf may have changed. Please verify." >&2
fi

echo ""
echo "=================================================================="
echo "PROTO generation complete."
echo "  Output: $PROTO_OUTPUT"
echo "  urdf2webots reported: $LINK_COUNT links, $JOINT_COUNT joints"
echo "=================================================================="
exit 0
