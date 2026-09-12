#!/bin/bash
# Launch the inverted pendulum robot Webots world in GUI mode (realtime).
#
# Usage: ./run_gui.sh [path/to/world.wbt]
#   Defaults to worlds/pendulum_robot.wbt
#
# This script:
# 1. Validates world file and WEBOTS_BIN.
# 2. Auto-generates the InvertedPendulumRobot PROTO (via generate_proto.sh)
#    if the .proto file is missing or the source URDF is newer.
# 3. Launches Webots with realtime GUI (requires DISPLAY set).
#
# Exits non-zero with clear error messages on any failure.

set -uo pipefail

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
PROTO_FILE="$SCRIPT_DIR/protos/InvertedPendulumRobot.proto"
URDF_SOURCE="$SCRIPT_DIR/../../06_Exports/urdf/robot.urdf"

# Fail fast: world file must exist.
if [ ! -f "$WORLD" ]; then
    echo "FATAL: world file not found: $WORLD" >&2
    echo "Run from inverted-pendulum-project/07_Simulation/webots/ directory," >&2
    echo "or pass a valid world path: ./run_gui.sh /path/to/world.wbt" >&2
    exit 1
fi

# Fail fast: WEBOTS_BIN must exist and be executable.
if ! command -v "$WEBOTS_BIN" >/dev/null 2>&1 && [ ! -x "$WEBOTS_BIN" ]; then
    echo "FATAL: WEBOTS_BIN ($WEBOTS_BIN) not found or not executable." >&2
    echo "Set WEBOTS_BIN to a valid Webots binary, e.g.:" >&2
    echo "  export WEBOTS_BIN=/usr/local/bin/webots" >&2
    exit 1
fi

# Fail fast: DISPLAY must be set for GUI mode.
if [ -z "${DISPLAY:-}" ]; then
    echo "FATAL: \$DISPLAY is not set." >&2
    echo "run_gui.sh launches Webots with a visible window (--mode=realtime)" >&2
    echo "and requires a real X display — it will not fall back to xvfb-run." >&2
    echo "Set DISPLAY to your active X session (e.g. export DISPLAY=:1) and re-run." >&2
    exit 1
fi
echo "Using DISPLAY=$DISPLAY"

# Auto-generate PROTO if missing or source URDF is newer.
if [ ! -s "$PROTO_FILE" ] || [ "$URDF_SOURCE" -nt "$PROTO_FILE" ]; then
    echo ""
    echo "InvertedPendulumRobot.proto not found or out-of-date—auto-generating..."
    if ! "$SCRIPT_DIR/generate_proto.sh"; then
        echo "FATAL: PROTO generation failed." >&2
        exit 1
    fi
fi

echo ""
echo "Launching Webots (realtime, GUI) on: $WORLD"
echo "Press Play in the Webots window if the simulation starts paused."
exec "$WEBOTS_BIN" --mode=realtime "$WORLD"
