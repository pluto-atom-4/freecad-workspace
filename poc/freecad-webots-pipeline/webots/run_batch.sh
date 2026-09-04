#!/bin/bash
# Stage 3 — run the generated turtlebot3_poc Webots world headless/batch.
#
# Usage: ./run_batch.sh [path/to/world.wbt]
#   Defaults to worlds/turtlebot3_poc.wbt (the urdf2webots-generated world).
#
# Tries a direct invocation first (DISPLAY is expected to be set, e.g. :1).
# Falls back to xvfb-run automatically if the direct invocation fails with
# a display-related error.
#
# A fresh checkout of main lacks the gitignored generated assets this world
# needs (reference/, freecad/output/, urdf/meshes/, webots/protos/) — see
# ../README.md "Reproducing end to end". Like run_gui.sh, this script
# auto-provisions them via the shared logic in _provision_assets.sh (issue
# #26 review finding #6) before falling back to the old fail-fast behavior
# for anything auto-provisioning itself can't fix.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="${1:-$SCRIPT_DIR/worlds/turtlebot3_poc.wbt}"
WEBOTS_BIN="${WEBOTS_BIN:-/usr/local/bin/webots}"
LOG_FILE="$SCRIPT_DIR/run_batch.log"

# shellcheck source=./_provision_assets.sh
source "$SCRIPT_DIR/_provision_assets.sh"

# Fail fast on a bad/typo'd $WORLD, and on a missing/bad WEBOTS_BIN, before
# burning the (multi-minute) Stage 0/1/1b/3 auto-provisioning pipeline
# below — provisioning always targets the fixed
# webots/protos/TurtlebotPoc.proto regardless of $WORLD, so neither check
# can be fixed by provisioning anyway. Shared with run_gui.sh so both give
# the same message instead of two copies that drift (review finding #2,
# round 2; finding #3 and #7, round 3).
require_world_file "$WORLD"
require_webots_bin

# --- Stage 0/1/1b/3 auto-provisioning -------------------------------------
# See _provision_assets.sh (shared with run_gui.sh) for the prerequisite
# chain and skip-check details. Passing $WORLD lets provision_assets skip
# entirely if this world doesn't even reference the generated PROTO
# (review finding #5, round 3).
provision_assets "$WORLD"

run_webots() {
    "$WEBOTS_BIN" --batch --mode=fast --no-rendering --minimize --stdout "$WORLD"
}

echo "Running Webots on: $WORLD"
echo "Log: $LOG_FILE"

if run_webots > "$LOG_FILE" 2>&1; then
    echo "Webots exited 0 (direct invocation)."
    tail -n 40 "$LOG_FILE"
    exit 0
fi

echo "Direct invocation failed or exited non-zero; log tail:"
tail -n 40 "$LOG_FILE"

if grep -qiE "display|xcb|cannot open|EGL" "$LOG_FILE"; then
    echo ""
    echo "Detected a display-related failure — retrying with xvfb-run..."
    if command -v xvfb-run >/dev/null 2>&1; then
        if xvfb-run -a "$WEBOTS_BIN" --batch --mode=fast --no-rendering --minimize --stdout "$WORLD" > "$LOG_FILE.xvfb" 2>&1; then
            echo "Webots exited 0 under xvfb-run."
            tail -n 40 "$LOG_FILE.xvfb"
            exit 0
        else
            echo "xvfb-run attempt also failed; log tail:"
            tail -n 40 "$LOG_FILE.xvfb"
            exit 1
        fi
    else
        echo "xvfb-run not available — cannot retry headless."
        exit 1
    fi
fi

exit 1
