#!/bin/bash
# Stage 3 (GUI) — launch the generated turtlebot3_poc Webots world live, in
# realtime mode, so a human can watch it (issue #26).
#
# Usage: ./run_gui.sh [path/to/world.wbt]
#   Defaults to worlds/turtlebot3_poc.wbt (the urdf2webots-generated world).
#
# Unlike run_batch.sh, this launches Webots with a visible window
# (--mode=realtime, no --batch/--no-rendering/--minimize) and requires a
# real DISPLAY — it deliberately does NOT fall back to xvfb-run, since
# Xvfb has no visible framebuffer and would silently defeat the point of
# this script.
#
# A fresh checkout of main lacks the gitignored generated assets this world
# needs (reference/, freecad/output/, urdf/meshes/, webots/protos/) — see
# ../README.md "Reproducing end to end". Rather than failing fast on a
# missing webots/protos/TurtlebotPoc.proto, this script auto-provisions
# them by running the existing Stage 0/1/1b/3 scripts in order, via the
# shared provisioning logic in _provision_assets.sh (also used by
# run_batch.sh — see issue #26 review finding #6). It only invokes those
# scripts — it does not duplicate their logic.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
POC_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORLD="${1:-$SCRIPT_DIR/worlds/turtlebot3_poc.wbt}"
WEBOTS_BIN="${WEBOTS_BIN:-/usr/local/bin/webots}"

# shellcheck source=./_provision_assets.sh
source "$SCRIPT_DIR/_provision_assets.sh"

if [ ! -f "$WORLD" ]; then
    echo "World file not found: $WORLD"
    echo "See ../README.md."
    exit 1
fi

# --- DISPLAY check --------------------------------------------------------
# This script's entire purpose is a visible window; unlike run_batch.sh, a
# headless/xvfb fallback here would silently defeat that. Fail loudly.
if [ -z "${DISPLAY:-}" ]; then
    echo "FATAL: \$DISPLAY is not set." >&2
    echo "run_gui.sh launches Webots with a visible window (--mode=realtime)" >&2
    echo "and requires a real X display — it will not fall back to xvfb-run" >&2
    echo "(Xvfb has no visible framebuffer, so that would defeat the point)." >&2
    echo "Set DISPLAY to your active X session (e.g. export DISPLAY=:1) and re-run." >&2
    exit 1
fi
echo "Using DISPLAY=$DISPLAY"

# --- Stage 0/1/1b/3 auto-provisioning -------------------------------------
# See _provision_assets.sh (shared with run_batch.sh) for the prerequisite
# chain and skip-check details.
provision_assets

echo ""
echo "Launching Webots (realtime, GUI) on: $WORLD"
echo "Press Play in the Webots window if the simulation starts paused."
exec "$WEBOTS_BIN" --mode=realtime "$WORLD"
