#!/bin/bash
# Stage 3 — run the generated turtlebot3_poc Webots world headless/batch.
#
# Usage: ./run_batch.sh [path/to/world.wbt]
#   Defaults to worlds/turtlebot3_poc.wbt (the urdf2webots-generated world).
#
# Tries a direct invocation first (DISPLAY is expected to be set, e.g. :1).
# Falls back to xvfb-run automatically if the direct invocation fails with
# a display-related error.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="${1:-$SCRIPT_DIR/worlds/turtlebot3_poc.wbt}"
WEBOTS_BIN="${WEBOTS_BIN:-/usr/local/bin/webots}"
LOG_FILE="$SCRIPT_DIR/run_batch.log"

if [ ! -f "$WORLD" ]; then
    echo "World file not found: $WORLD"
    echo "Run urdf2webots first (see ../README.md Stage 3)."
    exit 1
fi

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
