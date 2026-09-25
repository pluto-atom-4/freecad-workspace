#!/bin/bash
# Launch the esp32-dof Webots world live (GUI, realtime) so a human can watch
# the ESP32_BODY box follow the IMU orientation (issue #239, sub-issue of #227).
#
# Usage: ./run_gui.sh [path/to/world.wbt]
#   Defaults to worlds/esp32_dof.wbt (next to this script).
#
# Environment:
#   WEBOTS_BIN          Webots binary (default /usr/local/bin/webots)
#   DOF_FOLLOWER_DEBUG  Set to 1 to make the controller print the applied
#                       rotation. Webots is exec'd, so it inherits this shell's
#                       environment:  DOF_FOLLOWER_DEBUG=1 ./run_gui.sh
#
# Needs a real X display (DISPLAY). No xvfb fallback: a window nobody can see
# defeats the purpose. For a headless smoke test see ../README.md.
#
# The controller (controllers/esp32_dof_follower) is found by Webots relative
# to the world file (../controllers), so this script works from any cwd.
# --mode=realtime starts the simulation RUNNING; the controller is not stepped
# while the simulation is paused, so if the box does not move, press Play.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="${1:-$SCRIPT_DIR/worlds/esp32_dof.wbt}"
WEBOTS_BIN="${WEBOTS_BIN:-/usr/local/bin/webots}"

# Fail fast: world file must exist.
if [ ! -f "$WORLD" ]; then
    echo "FATAL: world file not found: $WORLD" >&2
    echo "Pass a valid world path: ./run_gui.sh /path/to/world.wbt" >&2
    exit 1
fi

# Fail fast: WEBOTS_BIN must be an executable file or a command on PATH.
if [ ! -x "$WEBOTS_BIN" ] && ! command -v "$WEBOTS_BIN" >/dev/null 2>&1; then
    echo "FATAL: WEBOTS_BIN ($WEBOTS_BIN) not found or not executable." >&2
    echo "Set WEBOTS_BIN to a valid Webots binary, e.g.:" >&2
    echo "  export WEBOTS_BIN=/usr/local/bin/webots" >&2
    exit 1
fi

# Fail fast: a visible window needs a real display.
if [ -z "${DISPLAY:-}" ]; then
    echo "FATAL: \$DISPLAY is not set." >&2
    echo "run_gui.sh launches Webots with a visible window (--mode=realtime)" >&2
    echo "and needs a real X display; it does not fall back to xvfb-run." >&2
    echo "Set DISPLAY to your active X session (e.g. export DISPLAY=:1) and re-run." >&2
    exit 1
fi
echo "Using DISPLAY=$DISPLAY"

echo ""
echo "Launching Webots (realtime, GUI) on: $WORLD"
echo "The simulation must be RUNNING for the box to move; press Play if it starts paused."
exec "$WEBOTS_BIN" --mode=realtime "$WORLD"
