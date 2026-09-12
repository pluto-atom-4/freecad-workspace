#!/bin/bash
# Prepare inverted pendulum robot.urdf for Webots consumption via urdf2webots.
#
# URDF contains package://inverted_pendulum_robot/meshes/... URIs that urdf2webots
# cannot resolve (no ROS environment, no package.xml). This script:
# 1. Validates source URDF and all referenced mesh files exist and are non-empty.
# 2. Copies robot.urdf to .generated/robot_webots.urdf.
# 3. Rewrites package:// URIs to relative paths (relative to .generated/ dir).
# 4. Fails loudly if any package:// substring remains after rewrite.
#
# Usage: ./prepare_urdf_for_webots.sh
#   (Run from the webots/ directory, or set SCRIPT_DIR yourself.)
#
# Exits 0 on success, non-zero with clear error message on any failure.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
URDF_SOURCE="$SCRIPT_DIR/../../06_Exports/urdf/robot.urdf"
MESH_DIR="$SCRIPT_DIR/../../06_Exports/urdf/meshes"
URDF_OUTPUT="$SCRIPT_DIR/.generated/robot_webots.urdf"

# Verify source URDF exists and is non-empty.
if [ ! -s "$URDF_SOURCE" ]; then
    echo "FATAL: source URDF not found or empty: $URDF_SOURCE" >&2
    exit 1
fi

# Verify all referenced meshes exist and are non-empty.
# robot.urdf references: feetech-STS3032-visual.stl (used in two places)
MESH_FILE="$MESH_DIR/feetech-STS3032-visual.stl"
if [ ! -s "$MESH_FILE" ]; then
    echo "FATAL: referenced mesh not found or empty: $MESH_FILE" >&2
    exit 1
fi

echo "Preparing URDF for Webots..."
echo "  Source: $URDF_SOURCE"
echo "  Output: $URDF_OUTPUT"

# Ensure output directory exists.
mkdir -p "$(dirname "$URDF_OUTPUT")"

# Copy and rewrite package:// URIs to relative paths.
# package://inverted_pendulum_robot/ -> ../../../06_Exports/urdf/
# The relative path is from .generated/ (where the output URDF lives) back to
# the meshes directory: .generated/ is webots/.generated/, going up 3 levels
# through webots/ -> 07_Simulation/ -> inverted-pendulum-project/, then into 06_Exports/urdf/.
sed "s|package://inverted_pendulum_robot/|../../../06_Exports/urdf/|g" "$URDF_SOURCE" > "$URDF_OUTPUT"

# Fail loudly if any package:// substring remains (signals incompletely rewritten paths).
if grep -q "package://" "$URDF_OUTPUT"; then
    echo "FATAL: package:// URI found in output after rewrite — rewrite logic is incomplete." >&2
    echo "Remaining package:// lines:" >&2
    grep "package://" "$URDF_OUTPUT" >&2
    exit 1
fi

echo "URDF preparation complete. package:// URIs rewritten to relative paths."
exit 0
