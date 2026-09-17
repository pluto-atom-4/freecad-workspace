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
#
# Logging: All events logged to .generated/pipeline_debug.log (urdf2webots output, link/joint counts, injections, validation).

set -euo pipefail

# Log function: timestamp + message to both stdout and log file
log() {
    local msg="$1"
    local timestamp
    timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$timestamp] $msg" | tee -a "$SCRIPT_DIR/.generated/pipeline_debug.log"
}

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
LOG_FILE="$SCRIPT_DIR/.generated/pipeline_debug.log"

# Ensure log directory exists
mkdir -p "$(dirname "$LOG_FILE")"

# Fail fast: mamba must be available (needed for pendulum-tools env).
if ! command -v mamba >/dev/null 2>&1; then
    log "ERROR: mamba not found on PATH — cannot run urdf2webots in pendulum-tools env."
    echo "FATAL: mamba not found on PATH — cannot run urdf2webots in pendulum-tools env." >&2
    echo "Is mamba installed and on PATH?" >&2
    exit 1
fi

log "========== GENERATE_PROTO START =========="

echo "Preparing URDF and generating PROTO..."

# Step 1: Prepare URDF (rewrite package:// URIs).
log "Step 1: Calling prepare_urdf_for_webots.sh..."
if ! "$SCRIPT_DIR/prepare_urdf_for_webots.sh"; then
    log "ERROR: URDF preparation failed."
    echo "FATAL: URDF preparation failed." >&2
    exit 1
fi
log "✓ Step 1 complete: URDF preparation successful"

if [ ! -s "$URDF_INPUT" ]; then
    log "ERROR: prepared URDF not found or empty: $URDF_INPUT"
    echo "FATAL: prepared URDF not found or empty: $URDF_INPUT" >&2
    exit 1
fi

log "✓ Prepared URDF file exists: $URDF_INPUT"

echo ""
echo "Step 1.5: Pre-validating URDF structure via Phase 12 validator..."
log "Step 1.5: Running Phase 12 URDF validator..."

# Step 1.5: Pre-validate URDF via Phase 12 before tool invocation.
# Run full Phase 12 validator to catch structural issues early.
# Note: Validator requires numpy (via 10_export_urdf), so run in pendulum-tools env.
GENERATORS_DIR="$SCRIPT_DIR/../../03_Parts/Generators"
PHASE12_OUT="$SCRIPT_DIR/.generated/phase12_validator.out"

if ! mamba run -n pendulum-tools python3 "$GENERATORS_DIR/12_validate_urdf_export.py" > "$PHASE12_OUT" 2>&1; then
    log "ERROR: Phase 12 pre-validation failed (captured in phase12_validator.out)"
    echo "FATAL: Phase 12 pre-validation failed — URDF invalid, aborting urdf2webots." >&2
    exit 1
fi

log "✓ Phase 12 validator passed"

# Also assert exact link/joint counts before tool runs (defense-in-depth).
# Expected: 5 links (Base_Link, Wheel_Left, Wheel_Right, Pendulum_Link, Pendulum_Link_Right)
#           4 joints (wheel_left_joint, wheel_right_joint, pendulum_pivot_joint, pendulum_pivot_right_joint)
log "Step 1.5b: Running structure checker (5 links, 4 joints expected)..."
STRUCTURE_OUT="$SCRIPT_DIR/.generated/structure_checker.out"

if ! mamba run -n pendulum-tools python3 "$GENERATORS_DIR/urdf_structure_checker.py" \
    --urdf "$URDF_INPUT" --links 5 --joints 4 > "$STRUCTURE_OUT" 2>&1; then
    log "ERROR: URDF structure validation failed (captured in structure_checker.out)"
    echo "FATAL: URDF structure validation failed — link/joint counts incorrect, aborting urdf2webots." >&2
    exit 1
fi

log "✓ Structure checker passed (5 links, 4 joints confirmed)"

echo ""
echo "Running urdf2webots (R2025a target)..."
log "Step 2: Invoking urdf2webots.importer (R2025a target)..."

# Step 3: Run urdf2webots in the pendulum-tools env (pre-validated at Step 1.5).
# Capture output for link/joint count validation.
PROTO_OUTPUT_DIR="$(dirname "$PROTO_OUTPUT")"
mkdir -p "$PROTO_OUTPUT_DIR"

URDF2WEBOTS_OUTPUT=$(mamba run -n pendulum-tools python3 -m urdf2webots.importer \
    --input="$URDF_INPUT" \
    --output="$PROTO_OUTPUT" \
    --target=R2025a 2>&1) || {
    log "ERROR: urdf2webots conversion failed"
    echo "FATAL: urdf2webots conversion failed." >&2
    echo "urdf2webots output:" >&2
    echo "$URDF2WEBOTS_OUTPUT" >&2
    echo "" >&2
    echo "Is urdf2webots installed in pendulum-tools? Install with:" >&2
    echo "  mamba install -n pendulum-tools -c conda-forge urdf2webots" >&2
    exit 1
}

# Step 4: Validate output PROTO file exists and is non-empty.
if [ ! -s "$PROTO_OUTPUT" ]; then
    log "ERROR: urdf2webots completed but output PROTO not found or empty: $PROTO_OUTPUT"
    echo "FATAL: urdf2webots completed but output PROTO not found or empty: $PROTO_OUTPUT" >&2
    exit 1
fi

log "✓ PROTO file generated: $PROTO_OUTPUT"

# Step 5: Parse urdf2webots output to validate link/joint counts (defense-in-depth).
# Expected: 5 links (Base_Link, Wheel_Left, Wheel_Right, Pendulum_Link, Pendulum_Link_Right)
#           4 joints (wheel_left_joint, wheel_right_joint, pendulum_pivot_joint, pendulum_pivot_right_joint)
log "Step 3: Parsing urdf2webots output for link/joint counts..."

LINK_COUNT=$(echo "$URDF2WEBOTS_OUTPUT" | grep -oE "[0-9]+ links" | grep -oE "[0-9]+" | head -1 || true)
JOINT_COUNT=$(echo "$URDF2WEBOTS_OUTPUT" | grep -oE "[0-9]+ joints" | grep -oE "[0-9]+" | head -1 || true)

# Provide defaults if counts not found in output (urdf2webots may not always print them).
LINK_COUNT="${LINK_COUNT:-0}"
JOINT_COUNT="${JOINT_COUNT:-0}"

if [ "$LINK_COUNT" -eq 0 ]; then
    log "ERROR: Failed to parse link count from urdf2webots output"
    echo "FATAL: Failed to parse link count from urdf2webots output." >&2
    echo "urdf2webots output:" >&2
    echo "$URDF2WEBOTS_OUTPUT" >&2
    exit 1
fi

if [ "$JOINT_COUNT" -eq 0 ]; then
    log "ERROR: Failed to parse joint count from urdf2webots output"
    echo "FATAL: Failed to parse joint count from urdf2webots output." >&2
    echo "urdf2webots output:" >&2
    echo "$URDF2WEBOTS_OUTPUT" >&2
    exit 1
fi

if [ "$LINK_COUNT" -ne 5 ]; then
    log "ERROR: urdf2webots reported $LINK_COUNT links, expected 5"
    echo "FATAL: urdf2webots reported $LINK_COUNT links, expected 5." >&2
    echo "The source robot.urdf may have changed. Rebuild with 10_export_urdf.py." >&2
    exit 1
fi

if [ "$JOINT_COUNT" -ne 4 ]; then
    log "ERROR: urdf2webots reported $JOINT_COUNT joints, expected 4"
    echo "FATAL: urdf2webots reported $JOINT_COUNT joints, expected 4." >&2
    echo "The source robot.urdf may have changed. Rebuild with 10_export_urdf.py." >&2
    exit 1
fi

log "✓ Parsed counts: $LINK_COUNT links (regex match), $JOINT_COUNT joints (regex match)"

# Step 6: Post-process PROTO to inject castShadows FALSE for high-triangle-count meshes.
# The feetech-STS3032-visual mesh has ~37556 triangles, exceeding Webots' 21845-triangle
# limit. Webots warns about shadow casting on oversized meshes; suppress with castShadows FALSE.
# This post-processor is idempotent: if castShadows already exists in the Shape, it skips.

echo ""
echo "Post-processing PROTO to suppress shadow-casting warnings..."
log "Step 4: Invoking inject_cast_shadows.py (block injection for mesh optimization)..."

if ! python3 "$SCRIPT_DIR/inject_cast_shadows.py" --proto "$PROTO_OUTPUT"; then
    log "ERROR: Post-processing (castShadows injection) failed"
    echo "FATAL: Post-processing failed." >&2
    exit 1
fi

log "✓ inject_cast_shadows.py completed successfully"

echo ""
echo "=================================================================="
echo "PROTO generation complete."
echo "  Output: $PROTO_OUTPUT"
echo "  urdf2webots reported: $LINK_COUNT links, $JOINT_COUNT joints"
echo "=================================================================="

log "========== GENERATE_PROTO SUCCESS =========="
log "Final PROTO: $PROTO_OUTPUT (Links: $LINK_COUNT, Joints: $JOINT_COUNT)"

exit 0
