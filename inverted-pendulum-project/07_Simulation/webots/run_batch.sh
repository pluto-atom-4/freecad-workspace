#!/bin/bash
# Run the inverted pendulum robot Webots world in headless batch mode (smoke test).
#
# Usage: ./run_batch.sh [path/to/world.wbt]
#   Defaults to worlds/pendulum_robot.wbt
#
# This script:
# 1. Validates world file and WEBOTS_BIN.
# 2. Auto-generates the InvertedPendulumRobot PROTO if needed.
# 3. Runs Webots in batch mode (--batch --mode=fast --no-rendering --minimize).
#    Falls back to xvfb-run if DISPLAY-related errors occur.
#
# NOTE: This is a STRUCTURAL SMOKE TEST only. It validates:
#   - The world file is valid and loads.
#   - The PROTO resolves correctly (EXTERNPROTO).
#   - No crash occurs during import and initialization.
#
# It does NOT validate the robot's VISUAL CORRECTNESS or PHYSICS.
# A human must visually inspect the robot in the GUI (run_gui.sh) to confirm
# the mesh geometry, scale, joint limits, and visual appearance are correct.
#
# Exits 0 if Webots initializes and runs without crashing, non-zero otherwise.

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
WORLD="${1:-$SCRIPT_DIR/worlds/pendulum_robot.wbt}"
WEBOTS_BIN="${WEBOTS_BIN:-/usr/local/bin/webots}"
LOG_FILE="$SCRIPT_DIR/run_batch.log"
PROTO_FILE="$SCRIPT_DIR/protos/InvertedPendulumRobot.proto"
URDF_SOURCE="$SCRIPT_DIR/../../06_Exports/urdf/robot.urdf"

# Fail fast: world file must exist.
if [ ! -f "$WORLD" ]; then
    echo "FATAL: world file not found: $WORLD" >&2
    echo "Run from inverted-pendulum-project/07_Simulation/webots/ directory," >&2
    echo "or pass a valid world path: ./run_batch.sh /path/to/world.wbt" >&2
    exit 1
fi

# Fail fast: WEBOTS_BIN must exist and be executable.
if ! command -v "$WEBOTS_BIN" >/dev/null 2>&1 && [ ! -x "$WEBOTS_BIN" ]; then
    echo "FATAL: WEBOTS_BIN ($WEBOTS_BIN) not found or not executable." >&2
    echo "Set WEBOTS_BIN to a valid Webots binary, e.g.:" >&2
    echo "  export WEBOTS_BIN=/usr/local/bin/webots" >&2
    exit 1
fi

# Auto-generate PROTO if missing or source URDF is newer.
if [ ! -s "$PROTO_FILE" ] || [ "$URDF_SOURCE" -nt "$PROTO_FILE" ]; then
    echo "InvertedPendulumRobot.proto not found or out-of-date—auto-generating..."
    if ! "$SCRIPT_DIR/generate_proto.sh"; then
        echo "FATAL: PROTO generation failed." >&2
        exit 1
    fi
fi

run_webots() {
    "$WEBOTS_BIN" --batch --mode=fast --no-rendering --minimize --stdout "$WORLD"
}

echo "Running Webots (batch, headless, smoke test) on: $WORLD"
echo "Log: $LOG_FILE"
echo ""

if run_webots > "$LOG_FILE" 2>&1; then
    echo "SUCCESS: Webots exited 0 (direct invocation)."
    echo "Webots initialized and ran without crashing (structural smoke test passed)."
    echo ""
    echo "NOTE: This is a structural smoke test only. It does NOT validate visual"
    echo "correctness or physics. A human must visually inspect the robot in the"
    echo "GUI (run_gui.sh) to confirm the mesh geometry, scale, and appearance."
    echo ""
    tail -n 20 "$LOG_FILE"
    exit 0
fi

echo "Direct invocation failed or exited non-zero. Log tail:"
tail -n 20 "$LOG_FILE"

if grep -qiE "display|xcb|cannot open|EGL" "$LOG_FILE"; then
    echo ""
    echo "Detected a display-related failure — retrying with xvfb-run..."
    if command -v xvfb-run >/dev/null 2>&1; then
        if xvfb-run -a "$WEBOTS_BIN" --batch --mode=fast --no-rendering --minimize --stdout "$WORLD" > "$LOG_FILE.xvfb" 2>&1; then
            echo "SUCCESS: Webots exited 0 under xvfb-run."
            echo "Webots initialized and ran without crashing (structural smoke test passed)."
            echo ""
            echo "NOTE: This is a structural smoke test only. It does NOT validate visual"
            echo "correctness or physics. A human must visually inspect the robot in the"
            echo "GUI (run_gui.sh) to confirm the mesh geometry, scale, and appearance."
            echo ""
            tail -n 20 "$LOG_FILE.xvfb"
            exit 0
        else
            echo "xvfb-run attempt also failed. Log tail:"
            tail -n 20 "$LOG_FILE.xvfb"
            exit 1
        fi
    else
        echo "xvfb-run not available — cannot retry headless."
        exit 1
    fi
fi

exit 1
