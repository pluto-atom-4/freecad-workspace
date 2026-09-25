#!/bin/bash
# End-to-end mock demo (issue #239, sub-issue of #227):
#   Webots GUI (esp32_dof world)  <-- UDP 127.0.0.1:5005 <--  dof_monitor.py --mock --publish
#
# Usage: ./run_mock_demo.sh [extra dof_monitor.py args, e.g. --duration 30]
#   Runs until Ctrl-C unless --duration is given. Webots is closed on exit.
#
# Environment:
#   DOF_ENV             mamba env name (default esp32-dof)
#   DOF_PYTHON          python to run the monitor with (skips mamba lookup)
#   WEBOTS_BIN          Webots binary (see webots/run_gui.sh)
#   DOF_FOLLOWER_DEBUG  1 = controller prints applied rotations (inherited by Webots)
#   DOF_DEMO_WAIT_S     max seconds to wait for the controller to bind UDP (default 30)
#   DOF_DEMO_DRY_RUN    1 = print what would run and exit 0 (no Webots, no mamba)
#
# Needs a real X display (DISPLAY). Press nothing: the world starts in realtime.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_GUI="$SCRIPT_DIR/webots/run_gui.sh"
MONITOR="$SCRIPT_DIR/monitor/dof_monitor.py"
ENV_NAME="${DOF_ENV:-esp32-dof}"
UDP_PORT=5005
WAIT_S="${DOF_DEMO_WAIT_S:-30}"

# --- dry run (no side effects) ---------------------------------------------
if [ "${DOF_DEMO_DRY_RUN:-0}" = "1" ]; then
    echo "[dry-run] would check: DISPLAY set, UDP port $UDP_PORT free"
    echo "[dry-run] would start : setsid $RUN_GUI &   (Webots, realtime GUI)"
    echo "[dry-run] would wait  : up to ${WAIT_S}s for UDP $UDP_PORT to be bound"
    echo "[dry-run] would run   : <python of env '$ENV_NAME'> -u $MONITOR --mock --publish $*"
    echo "[dry-run] would clean : kill Webots process group on exit"
    exit 0
fi

# --- preconditions ----------------------------------------------------------
if [ -z "${DISPLAY:-}" ]; then
    echo "FATAL: \$DISPLAY is not set; the demo needs a real X display." >&2
    echo "Set DISPLAY (e.g. export DISPLAY=:1) and re-run." >&2
    exit 1
fi

port_in_use() {
    command -v ss >/dev/null 2>&1 || return 1
    ss -H -uln 2>/dev/null | awk '{print $4}' | grep -q ":${UDP_PORT}\$"
}

if port_in_use; then
    echo "FATAL: UDP port $UDP_PORT is already in use." >&2
    echo "A running Webots esp32_dof world (or another listener) holds it, and the" >&2
    echo "controller would fail with 'cannot bind UDP'. Close it and re-run." >&2
    echo "Find the owner with:  ss -uanp | grep :$UDP_PORT" >&2
    exit 1
fi

# Resolve the interpreter of the mamba env once (avoids `mamba run` output
# buffering / signal swallowing for the long-running foreground monitor).
if [ -n "${DOF_PYTHON:-}" ]; then
    PY="$DOF_PYTHON"
else
    MAMBA="${MAMBA_EXE:-mamba}"
    PY="$("$MAMBA" run -n "$ENV_NAME" python -c 'import sys; print(sys.executable)' 2>/dev/null | tail -n 1)" || PY=""
fi
if [ -z "$PY" ] || [ ! -x "$PY" ]; then
    echo "FATAL: could not find python for mamba env '$ENV_NAME'." >&2
    echo "Create it:  mamba env create -n esp32-dof -f poc/esp32-dof/mamba-envs.lock.yml" >&2
    echo "or set DOF_PYTHON=/path/to/python (needs numpy + pyserial)." >&2
    exit 1
fi

# --- start Webots in its own session, clean up on exit ----------------------
WEBOTS_PID=""
cleanup() {
    trap - EXIT INT TERM
    if [ -n "$WEBOTS_PID" ] && kill -0 "$WEBOTS_PID" 2>/dev/null; then
        echo "Stopping Webots (process group $WEBOTS_PID)..."
        kill -TERM -- "-$WEBOTS_PID" 2>/dev/null || kill -TERM "$WEBOTS_PID" 2>/dev/null || true
        for _ in 1 2 3 4 5 6 7 8 9 10; do
            kill -0 "$WEBOTS_PID" 2>/dev/null || break
            sleep 0.3
        done
        kill -KILL -- "-$WEBOTS_PID" 2>/dev/null || true
        wait "$WEBOTS_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "Starting Webots ($RUN_GUI)..."
setsid "$RUN_GUI" &
WEBOTS_PID=$!

# --- wait until the controller has bound the UDP port -----------------------
if command -v ss >/dev/null 2>&1; then
    echo "Waiting up to ${WAIT_S}s for the controller to listen on UDP $UDP_PORT..."
    ready=0
    elapsed=0
    while [ "$elapsed" -lt "$WAIT_S" ]; do
        if ! kill -0 "$WEBOTS_PID" 2>/dev/null; then
            echo "FATAL: Webots exited before the controller started (see messages above)." >&2
            exit 1
        fi
        if port_in_use; then
            ready=1
            break
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    if [ "$ready" -ne 1 ]; then
        echo "WARNING: UDP $UDP_PORT still not bound after ${WAIT_S}s; starting the monitor anyway." >&2
        echo "         (Is the world loaded and the simulation running? Press Play.)" >&2
    fi
else
    echo "WARNING: 'ss' not found; waiting a fixed 5 s instead of polling."
    sleep 5
fi

# --- run the monitor in the foreground --------------------------------------
echo "Running: $PY -u $MONITOR --mock --publish $*"
echo "Ctrl-C to stop (a summary is printed; exit code 130 after Ctrl-C is normal)."
rc=0
"$PY" -u "$MONITOR" --mock --publish "$@" || rc=$?
exit "$rc"
