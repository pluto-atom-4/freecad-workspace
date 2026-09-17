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
#
# Logging: All events logged to .generated/pipeline_debug.log (timestamps, path rewrites, mesh validation).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
URDF_SOURCE="$SCRIPT_DIR/../../06_Exports/urdf/robot.urdf"
MESH_DIR="$SCRIPT_DIR/../../06_Exports/urdf/meshes"
URDF_OUTPUT="$SCRIPT_DIR/.generated/robot_webots.urdf"
LOG_FILE="$SCRIPT_DIR/.generated/pipeline_debug.log"

# Log function: timestamp + message to both stdout and log file
log() {
    local msg="$1"
    local timestamp
    timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$timestamp] $msg" | tee -a "$LOG_FILE"
}

# Ensure output directory exists (and initialize log file)
mkdir -p "$(dirname "$URDF_OUTPUT")"

# Initialize log file with separator
log "========== PREPARE_URDF_FOR_WEBOTS START =========="

# Verify source URDF exists and is non-empty.
if [ ! -s "$URDF_SOURCE" ]; then
    log "ERROR: source URDF not found or empty: $URDF_SOURCE"
    echo "FATAL: source URDF not found or empty: $URDF_SOURCE" >&2
    exit 1
fi

log "✓ Source URDF exists: $URDF_SOURCE"

# Verify all referenced meshes exist and are non-empty.
# robot.urdf references: feetech-STS3032-visual.stl (used in two places)
MESH_FILE="$MESH_DIR/feetech-STS3032-visual.stl"
if [ ! -s "$MESH_FILE" ]; then
    log "ERROR: referenced mesh not found or empty: $MESH_FILE"
    echo "FATAL: referenced mesh not found or empty: $MESH_FILE" >&2
    exit 1
fi

log "✓ Mesh file exists: $MESH_FILE"

echo "Preparing URDF for Webots..."
echo "  Source: $URDF_SOURCE"
echo "  Output: $URDF_OUTPUT"

log "Starting URDF rewrite pipeline (source → $URDF_OUTPUT)"

# Copy and rewrite package:// URIs to relative paths.
# Use centralized Python resolver (Issue #161) instead of hardcoded sed.
# The relative path is computed from the resolved absolute path back to .generated/.
GENERATORS_DIR="$SCRIPT_DIR/../../03_Parts/Generators"
log "Invoking urdf_mesh_path_resolver.py (issue #161)..."

if python3 "${GENERATORS_DIR}/urdf_mesh_path_resolver.py" rewrite \
    --input "$URDF_SOURCE" \
    --output "$URDF_OUTPUT" \
    --urdf-dir "$(dirname "$URDF_SOURCE")" \
    --webots-dir "$(dirname "$URDF_OUTPUT")"; then
    log "✓ Resolver completed successfully"
else
    log "ERROR: Resolver failed with non-zero exit code"
    exit 1
fi

# Fail loudly if any package:// substring remains (signals incompletely rewritten paths).
if grep -q "package://" "$URDF_OUTPUT"; then
    log "ERROR: package:// URI found in output after rewrite — rewrite logic is incomplete."
    echo "FATAL: package:// URI found in output after rewrite — rewrite logic is incomplete." >&2
    echo "Remaining package:// lines:" >&2
    grep "package://" "$URDF_OUTPUT" >&2
    exit 1
fi

log "✓ package:// rewrite validation passed (no package:// URIs remaining)"
log "========== PREPARE_URDF_FOR_WEBOTS SUCCESS =========="

echo "URDF preparation complete. package:// URIs rewritten to relative paths."
exit 0
