#!/bin/bash
# Headless/GUI demo runner for the servo-wheel-dof Webots world (issue #266,
# sub-issue of #258). Mirrors poc/esp32-dof/webots/run_gui.sh's conventions.
# --mock (issue #287) also starts monitor/mock_publisher.py against the world,
# mirroring poc/esp32-dof/run_mock_demo.sh.
#
# Usage:
#   ./run_demo.sh                 # headless batch mode (default)
#   ./run_demo.sh --headless      # same, explicit
#   ./run_demo.sh --gui           # GUI realtime mode; needs $DISPLAY, no xvfb fallback
#   ./run_demo.sh --headless /path/to/other.wbt   # override world file
#   ./run_demo.sh --headless --mock   # + mock UDP publisher, stops when done
#   ./run_demo.sh --gui --mock        # + mock UDP publisher until Ctrl-C
#
# Environment:
#   WEBOTS_BIN   Webots binary (default /usr/local/bin/webots)
#   TIMEOUT_S    Headless batch wall-clock timeout in seconds (default 30)
#   DRY_RUN      1 = print the resolved webots command and exit 0; no Webots launched
#
# Environment (--mock only):
#   SWD_UDP_PORT   UDP port shared by controller and publisher (default 5006);
#                  exported to Webots. Exit 1 if it is already bound.
#   SWD_PYTHON     python that runs the publisher (default: python of the mamba
#                  env servo-wheel-dof, resolved once via `mamba run`)
#   SWD_WAIT_S     max seconds to wait for the controller to bind UDP (default 30)
#   MOCK_DURATION  publisher --duration in seconds. Default: headless =
#                  TIMEOUT_S - startup time - 3 s margin; gui = until Ctrl-C.
#                  TIMEOUT_S must be a positive integer with --mock.
#
# Headless mode runs:
#   timeout "$TIMEOUT_S" "$WEBOTS_BIN" --batch --mode=realtime --no-rendering \
#     --minimize --stdout --stderr "$WORLD"
#
# GUI mode runs (needs a real X display; fails loudly if $DISPLAY is unset --
# there is no xvfb fallback, per poc/esp32-dof precedent):
#   "$WEBOTS_BIN" --mode=realtime "$WORLD"
#
# With --mock the same Webots command runs in the background under `setsid`
# (own process group). The script waits until the controller binds
# 127.0.0.1:$SWD_UDP_PORT, runs the publisher, and on exit/INT/TERM kills ONLY
# that Webots process group (never pkill; a Webots you started yourself is left
# alone). Exit code = publisher's (130 after Ctrl-C, 143 after SIGTERM).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="$SCRIPT_DIR/webots/worlds/servo_wheel_dof.wbt"
WEBOTS_BIN="${WEBOTS_BIN:-/usr/local/bin/webots}"
TIMEOUT_S="${TIMEOUT_S:-30}"
MODE="headless"
MOCK=0
ENV_NAME="servo-wheel-dof"
PUBLISHER="$SCRIPT_DIR/monitor/mock_publisher.py"
UDP_PORT="${SWD_UDP_PORT:-5006}"
WAIT_S="${SWD_WAIT_S:-30}"
MARGIN_S=3

usage() {
    echo "Usage: $0 [--headless|--gui] [--mock] [/path/to/world.wbt]" >&2
    echo "  --headless   batch mode, no window, exits after \$TIMEOUT_S seconds (default)" >&2
    echo "  --gui        realtime GUI window; needs \$DISPLAY (no xvfb fallback)" >&2
    echo "  --mock       also run monitor/mock_publisher.py --mock on UDP \$SWD_UDP_PORT;" >&2
    echo "               Webots runs in the background and is stopped on exit" >&2
    echo "Environment: WEBOTS_BIN (default /usr/local/bin/webots), TIMEOUT_S (default 30), DRY_RUN=1" >&2
    echo "  --mock only: SWD_UDP_PORT (default 5006), SWD_PYTHON, SWD_WAIT_S (default 30)," >&2
    echo "               MOCK_DURATION (seconds; default: headless = until just before TIMEOUT_S," >&2
    echo "               gui = until Ctrl-C)" >&2
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
        --mock)
            MOCK=1
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

# --- --mock input validation (before dry-run so bad input is reported there too)
if [ "$MOCK" = "1" ]; then
    if ! [[ "$UDP_PORT" =~ ^[0-9]{1,5}$ ]] \
        || [ "$((10#$UDP_PORT))" -lt 1 ] || [ "$((10#$UDP_PORT))" -gt 65535 ]; then
        echo "FATAL: SWD_UDP_PORT must be an integer 1..65535, got: $UDP_PORT" >&2
        exit 1
    fi
    UDP_PORT=$((10#$UDP_PORT))
    if ! [[ "$WAIT_S" =~ ^[0-9]+$ ]]; then
        echo "FATAL: SWD_WAIT_S must be a non-negative integer, got: $WAIT_S" >&2
        exit 1
    fi
    if [ "$MODE" = "headless" ]; then
        if ! [[ "$TIMEOUT_S" =~ ^[0-9]+$ ]] || [ "$((10#$TIMEOUT_S))" -lt 1 ]; then
            echo "FATAL: with --mock, TIMEOUT_S must be a positive integer, got: $TIMEOUT_S" >&2
            exit 1
        fi
    fi
    if [ -n "${MOCK_DURATION:-}" ] && ! [[ "$MOCK_DURATION" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        echo "FATAL: MOCK_DURATION must be a number of seconds, got: $MOCK_DURATION" >&2
        exit 1
    fi
    if [ ! -f "$PUBLISHER" ]; then
        echo "FATAL: publisher not found: $PUBLISHER" >&2
        exit 1
    fi
fi

# --- dry run (no side effects) ----------------------------------------------
if [ "${DRY_RUN:-0}" = "1" ]; then
    echo "[dry-run] mode   : $MODE"
    echo "[dry-run] world  : $WORLD"
    printf '[dry-run] command: %q ' "${CMD[@]}"
    echo
    if [ "$MOCK" = "1" ]; then
        if [ -n "${SWD_PYTHON:-}" ]; then
            PY_SHOWN="$SWD_PYTHON"
        else
            PY_SHOWN="<python of mamba env '$ENV_NAME'>"
        fi
        echo "[dry-run] mock   : on (UDP 127.0.0.1:$UDP_PORT, exported as SWD_UDP_PORT)"
        echo "[dry-run] python : $PY_SHOWN"
        echo "[dry-run] check  : UDP port $UDP_PORT must be free (exit 1 if already bound)"
        printf '[dry-run] start  : setsid '
        printf '%q ' "${CMD[@]}"
        echo '&'
        echo "[dry-run] wait   : up to ${WAIT_S}s for the controller to bind UDP $UDP_PORT"
        echo "[dry-run] publish: $PY_SHOWN -u $PUBLISHER --mock --port $UDP_PORT"
        if [ -n "${MOCK_DURATION:-}" ]; then
            echo "[dry-run] length : --duration $MOCK_DURATION (MOCK_DURATION)"
        elif [ "$MODE" = "headless" ]; then
            echo "[dry-run] length : --duration TIMEOUT_S($TIMEOUT_S) - startup - ${MARGIN_S}s margin (min 1)"
        else
            echo "[dry-run] length : until Ctrl-C or the Webots window closes"
        fi
        echo "[dry-run] clean  : on EXIT/INT/TERM kill only that Webots process group (no pkill)"
    fi
    exit 0
fi

require_display() {
    # Fail fast: a visible window needs a real display.
    if [ -z "${DISPLAY:-}" ]; then
        echo "FATAL: \$DISPLAY is not set." >&2
        echo "--gui launches Webots with a visible window (--mode=realtime)" >&2
        echo "and needs a real X display; it does not fall back to xvfb-run." >&2
        echo "Set DISPLAY to your active X session (e.g. export DISPLAY=:1) and re-run," >&2
        echo "or use --headless for a windowless batch run." >&2
        exit 1
    fi
}

# ============================================================================
# --mock path: Webots in the background + mock publisher + cleanup trap.
# ============================================================================
if [ "$MOCK" = "1" ]; then
    if [ "$MODE" = "gui" ]; then
        require_display
        echo "Using DISPLAY=$DISPLAY"
    fi

    if ! command -v setsid >/dev/null 2>&1; then
        echo "FATAL: 'setsid' (util-linux) not found; it is needed to give Webots its own" >&2
        echo "process group so cleanup never touches other Webots instances." >&2
        exit 1
    fi

    # Resolve the interpreter once (avoids `mamba run` buffering / signal swallowing).
    if [ -n "${SWD_PYTHON:-}" ]; then
        PY="$(command -v -- "$SWD_PYTHON" 2>/dev/null || true)"
    else
        MAMBA="${MAMBA_EXE:-mamba}"
        PY="$("$MAMBA" run -n "$ENV_NAME" python -c 'import sys; print(sys.executable)' \
            2>/dev/null | tail -n 1)" || PY=""
    fi
    if [ -z "$PY" ] || [ ! -x "$PY" ]; then
        echo "FATAL: could not find python for mamba env '$ENV_NAME'." >&2
        echo "Create it:  mamba env create -n servo-wheel-dof -f poc/servo-wheel-dof/mamba-envs.lock.yml" >&2
        echo "or set SWD_PYTHON=/path/to/python (stdlib only is enough for the publisher)." >&2
        exit 1
    fi

    # True (0) if something already has the UDP port bound. awk reads all of
    # ss's output, so there is no SIGPIPE/pipefail false negative.
    port_in_use() {
        if command -v ss >/dev/null 2>&1; then
            ss -uln 2>/dev/null | awk -v p=":${UDP_PORT}" '
                { n = length($4); if (n >= length(p) && substr($4, n - length(p) + 1) == p) f = 1 }
                END { exit !f }'
        else
            "$PY" -c '
import socket, sys
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    s.bind(("127.0.0.1", int(sys.argv[1])))
except OSError:
    sys.exit(0)
sys.exit(1)' "$UDP_PORT"
        fi
    }

    if port_in_use; then
        echo "FATAL: UDP port $UDP_PORT is already in use." >&2
        echo "Another Webots world (or listener) holds it, and the controller would fail" >&2
        echo "with 'cannot bind UDP'. Close it, or pick a free port: SWD_UDP_PORT=5021 $0 ..." >&2
        echo "Find the owner with:  ss -uanp | grep :$UDP_PORT" >&2
        exit 1
    fi

    export SWD_UDP_PORT="$UDP_PORT"

    # --- start Webots in its own session; clean up only what we started -------
    WEBOTS_PID=""
    PUB_PID=""
    cleanup() {
        trap - EXIT INT TERM
        if [ -n "$PUB_PID" ]; then
            kill -TERM "$PUB_PID" 2>/dev/null || true
            wait "$PUB_PID" 2>/dev/null || true
        fi
        if [ -n "$WEBOTS_PID" ]; then
            # setsid made WEBOTS_PID the process-group id; the group can outlive
            # its leader, so test the group, not just the PID.
            if kill -0 -- "-$WEBOTS_PID" 2>/dev/null; then
                echo "Stopping Webots (process group $WEBOTS_PID)..."
                kill -TERM -- "-$WEBOTS_PID" 2>/dev/null || true
                for ((i = 0; i < 20; i++)); do
                    kill -0 -- "-$WEBOTS_PID" 2>/dev/null || break
                    sleep 0.25
                done
                kill -KILL -- "-$WEBOTS_PID" 2>/dev/null || true
            fi
            wait "$WEBOTS_PID" 2>/dev/null || true
        fi
    }
    trap cleanup EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM

    if [ "$MODE" = "gui" ]; then
        echo "Launching Webots (realtime, GUI, background) on: $WORLD"
        echo "The simulation must be RUNNING for the controller to step; press Play if it starts paused."
    else
        echo "Launching Webots (headless batch, timeout ${TIMEOUT_S}s, background) on: $WORLD"
    fi
    LAUNCH_AT=$SECONDS
    setsid "${CMD[@]}" &
    WEBOTS_PID=$!

    # --- wait until the controller has bound the UDP port ---------------------
    if command -v ss >/dev/null 2>&1; then
        echo "Waiting up to ${WAIT_S}s for the controller to listen on UDP $UDP_PORT..."
        ready=0
        while [ $((SECONDS - LAUNCH_AT)) -lt "$WAIT_S" ]; do
            if ! kill -0 "$WEBOTS_PID" 2>/dev/null; then
                echo "FATAL: Webots exited before the controller started (see messages above)." >&2
                exit 1
            fi
            if port_in_use; then
                ready=1
                break
            fi
            sleep 0.5
        done
        if [ "$ready" -ne 1 ]; then
            echo "FATAL: UDP $UDP_PORT still not bound after ${WAIT_S}s." >&2
            echo "Is the world loaded? Raise SWD_WAIT_S, or check the Webots output above." >&2
            exit 1
        fi
    else
        echo "WARNING: 'ss' not found; waiting a fixed 5 s instead of polling."
        sleep 5
        if ! kill -0 "$WEBOTS_PID" 2>/dev/null; then
            echo "FATAL: Webots exited before the controller started (see messages above)." >&2
            exit 1
        fi
    fi

    # --- publisher duration ---------------------------------------------------
    DURATION="${MOCK_DURATION:-}"
    if [ -z "$DURATION" ] && [ "$MODE" = "headless" ]; then
        DURATION=$(( $((10#$TIMEOUT_S)) - (SECONDS - LAUNCH_AT) - MARGIN_S ))
        if [ "$DURATION" -lt 1 ]; then
            echo "WARNING: TIMEOUT_S=${TIMEOUT_S} is almost used up by startup; publishing for 1 s." >&2
            DURATION=1
        fi
    fi
    PUB_CMD=("$PY" -u "$PUBLISHER" --mock --port "$UDP_PORT")
    if [ -n "$DURATION" ]; then
        PUB_CMD+=(--duration "$DURATION")
    fi

    # --- run the publisher in the background so INT/TERM traps fire at once ----
    echo "Running: ${PUB_CMD[*]}"
    if [ -z "$DURATION" ]; then
        echo "Ctrl-C to stop (exit code 130 after Ctrl-C is normal); closing Webots also stops it."
    fi
    "${PUB_CMD[@]}" &
    PUB_PID=$!

    webots_gone=0
    while kill -0 "$PUB_PID" 2>/dev/null; do
        if ! kill -0 "$WEBOTS_PID" 2>/dev/null; then
            webots_gone=1
            break
        fi
        sleep 0.5
    done
    if [ "$webots_gone" -eq 1 ]; then
        kill -TERM "$PUB_PID" 2>/dev/null || true
    fi
    rc=0
    wait "$PUB_PID" || rc=$?
    PUB_PID=""
    if [ "$webots_gone" -eq 1 ]; then
        if [ "$MODE" = "gui" ]; then
            echo "Webots exited; publisher stopped."
            rc=0
        else
            echo "FATAL: Webots exited before the publisher finished." >&2
            rc=1
        fi
    fi
    exit "$rc"
fi

# ============================================================================
# Non-mock path (unchanged behavior).
# ============================================================================
if [ "$MODE" = "gui" ]; then
    require_display
    echo "Using DISPLAY=$DISPLAY"
    echo "Launching Webots (realtime, GUI) on: $WORLD"
    echo "The simulation must be RUNNING for the controller to step; press Play if it starts paused."
    exec "${CMD[@]}"
fi

echo "Launching Webots (headless batch, timeout ${TIMEOUT_S}s) on: $WORLD"
exec "${CMD[@]}"
