#!/bin/bash
# Headless/GUI demo runner for the servo-wheel-dof Webots world (issue #266,
# sub-issue of #258). Mirrors poc/esp32-dof/webots/run_gui.sh's conventions.
#
# Usage:
#   ./run_demo.sh                 # headless batch mode (default)
#   ./run_demo.sh --headless      # same, explicit
#   ./run_demo.sh --gui           # GUI realtime mode; needs $DISPLAY, no xvfb fallback
#   ./run_demo.sh --headless /path/to/other.wbt   # override world file
#
# Environment:
#   WEBOTS_BIN   Webots binary (default /usr/local/bin/webots)
#   TIMEOUT_S    Headless batch wall-clock timeout in seconds (default 30)
#   DRY_RUN      1 = print the resolved webots command and exit 0; no Webots launched
#
# Headless mode runs:
#   timeout "$TIMEOUT_S" "$WEBOTS_BIN" --batch --mode=realtime --no-rendering \
#     --minimize --stdout --stderr "$WORLD"
#
# GUI mode runs (needs a real X display; fails loudly if $DISPLAY is unset --
# there is no xvfb fallback, per poc/esp32-dof precedent):
#   "$WEBOTS_BIN" --mode=realtime "$WORLD"

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="$SCRIPT_DIR/webots/worlds/servo_wheel_dof.wbt"
WEBOTS_BIN="${WEBOTS_BIN:-/usr/local/bin/webots}"
TIMEOUT_S="${TIMEOUT_S:-30}"
MODE="headless"

usage() {
    echo "Usage: $0 [--headless|--gui] [/path/to/world.wbt]" >&2
    echo "  --headless   batch mode, no window, exits after \$TIMEOUT_S seconds (default)" >&2
    echo "  --gui        realtime GUI window; needs \$DISPLAY (no xvfb fallback)" >&2
    echo "Environment: WEBOTS_BIN (default /usr/local/bin/webots), TIMEOUT_S (default 30), DRY_RUN=1" >&2
}

while [ $# -gt 0 ]; do
    case "$1" in
        --headless)
            MODE="headless"
            shift
            ;;
        --gui)
            MODE="gui"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        -*)
            echo "FATAL: unknown option: $1" >&2
            usage
            exit 1
            ;;
        *)
            WORLD="$1"
            shift
            ;;
    esac
done

# Fail fast: world file must exist.
if [ ! -f "$WORLD" ]; then
    echo "FATAL: world file not found: $WORLD" >&2
    echo "Pass a valid world path: $0 [--headless|--gui] /path/to/world.wbt" >&2
    exit 1
fi

# Fail fast: WEBOTS_BIN must be an executable file or a command on PATH.
if [ ! -x "$WEBOTS_BIN" ] && ! command -v "$WEBOTS_BIN" >/dev/null 2>&1; then
    echo "FATAL: WEBOTS_BIN ($WEBOTS_BIN) not found or not executable." >&2
    echo "Set WEBOTS_BIN to a valid Webots binary, e.g.:" >&2
    echo "  export WEBOTS_BIN=/usr/local/bin/webots" >&2
    exit 1
fi

if [ "$MODE" = "gui" ]; then
    CMD=("$WEBOTS_BIN" --mode=realtime "$WORLD")
else
    CMD=(timeout "$TIMEOUT_S" "$WEBOTS_BIN" --batch --mode=realtime --no-rendering --minimize --stdout --stderr "$WORLD")
fi

# --- dry run (no side effects) ----------------------------------------------
if [ "${DRY_RUN:-0}" = "1" ]; then
    echo "[dry-run] mode   : $MODE"
    echo "[dry-run] world  : $WORLD"
    printf '[dry-run] command: %q ' "${CMD[@]}"
    echo
    exit 0
fi

if [ "$MODE" = "gui" ]; then
    # Fail fast: a visible window needs a real display.
    if [ -z "${DISPLAY:-}" ]; then
        echo "FATAL: \$DISPLAY is not set." >&2
        echo "--gui launches Webots with a visible window (--mode=realtime)" >&2
        echo "and needs a real X display; it does not fall back to xvfb-run." >&2
        echo "Set DISPLAY to your active X session (e.g. export DISPLAY=:1) and re-run," >&2
        echo "or use --headless for a windowless batch run." >&2
        exit 1
    fi
    echo "Using DISPLAY=$DISPLAY"
    echo "Launching Webots (realtime, GUI) on: $WORLD"
    echo "The simulation must be RUNNING for the controller to step; press Play if it starts paused."
    exec "${CMD[@]}"
fi

echo "Launching Webots (headless batch, timeout ${TIMEOUT_S}s) on: $WORLD"
exec "${CMD[@]}"
