#!/bin/bash
# Run the freecad-workspace DOF wheel bridge (dof_wheel_bridge.py) against a
# running Wokwi simulation's RFC2217 port, as a pipe check for the ttl_servo
# DOF frame stream (build with -D TTL_SERVO_DOF_FRAMES).
#
# SIM-ONLY, not hardware-validated. Webots wheel signs UNVERIFIED. This only
# shows that the pipe works (Wokwi RFC2217 port -> bridge -> Webots).
# Start order: the bridge may start first; it waits up to 300 s for the port.
# The simulator is not pre-checked by this script.
#
# stdout: --guide and --dry-run text; otherwise the bridge's stdout.
# stderr: FATAL, WARNING and status lines.
# Writes: none by this script. The only file the bridge writes is --record
# (--text-log is a reserved flag and is refused, so it is never written).
# Python runs in the mamba env "esp32-dof", from $FW_DIR/poc/esp32-dof/monitor.

set -u

# Ctrl+C always ends with exit 130 (same as wokwi-stream.sh).
trap 'exit 130' INT

PY_ENV=esp32-dof
DEFAULT_PORT_URL=rfc2217://localhost:4000
DEFAULT_CORE=lqr
DEFAULT_DURATION=120
DEFAULT_RECORD=/tmp/check_rfc.csv

case "${BASH_SOURCE[0]}" in
    */*) SCRIPT_DIR_RAW="${BASH_SOURCE[0]%/*}" ;;
    *) SCRIPT_DIR_RAW="." ;;
esac
SCRIPT_DIR="$(cd "$SCRIPT_DIR_RAW" 2>/dev/null && pwd)"
# Script lives in <FW>/poc/esp32-dof/tools: three levels up is the FW root.
DEFAULT_FW_DIR="$(cd "$SCRIPT_DIR/../../.." 2>/dev/null && pwd)"
DEFAULT_BRC_DIR="${HOME:-}/Documents/platform-io-workspace/balancing-robot-controller"

FW_DIR="${FW_DIR:-$DEFAULT_FW_DIR}"
BRC_DIR="${BRC_DIR:-$DEFAULT_BRC_DIR}"
MON_DIR="$FW_DIR/poc/esp32-dof/monitor"

PORT_URL="$DEFAULT_PORT_URL"
CORE="$DEFAULT_CORE"
DURATION="$DEFAULT_DURATION"
RECORD="$DEFAULT_RECORD"
OVERWRITE=0
GUIDE=0
DRY_RUN=0
PASSTHROUGH=()

# Colour only on a terminal stdout, and only when NO_COLOR is unset or empty.
COLOR=0
if [ -t 1 ] && [ -z "${NO_COLOR:-}" ]; then
    COLOR=1
fi

usage_short() {
    echo "Usage: wokwi-rfc2217-check.sh [--guide] [--dry-run] [--port URL] [--core lqr|pid|none]" >&2
    echo "         [--duration SECS] [--record PATH] [--overwrite] [-- EXTRA_BRIDGE_ARGS...]" >&2
    echo "Try 'wokwi-rfc2217-check.sh --help' for details." >&2
}

usage() {
    cat <<'EOF'
Usage: wokwi-rfc2217-check.sh [--guide] [--dry-run] [--port URL] [--core lqr|pid|none]
                              [--duration SECS] [--record PATH] [--overwrite]
                              [-h|--help] [-- EXTRA_BRIDGE_ARGS...]
Flags accept "--opt value" and "--opt=value".

Run the freecad-workspace DOF wheel bridge (dof_wheel_bridge.py) against the
RFC2217 port of a running Wokwi simulation (ttl_servo built with
-D TTL_SERVO_DOF_FRAMES). SIM-ONLY, not hardware-validated. Webots wheel signs
UNVERIFIED. The bridge may start first; it waits up to 300 s for the port.

Options:
  --guide           print the step-by-step guide and exit (runs nothing)
  --dry-run         print the exact bridge command and the checks the real run
                    does; --dry-run performs none of those checks and writes
                    nothing
  --port URL        RFC2217 URL (default: rfc2217://localhost:4000)
  --core CORE       controller core: lqr, pid or none (default: lqr)
  --duration SECS   bridge run length, positive integer (default: 120)
  --record PATH     CSV file the bridge writes (default: /tmp/check_rfc.csv)
  --overwrite       allow replacing an existing --record file
  -h, --help        show this help and exit
  -- ARGS...        pass ARGS to dof_wheel_bridge.py unchanged. Reserved flags
                    are refused (exit 2), as --flag, --flag=value, or any prefix
                    of a reserved flag (so --r is refused, as it matches
                    --record and --replay): --port --core --duration --record
                    --stdin --replay --mock --text-log. Other flags pass through.

Notes:
  Options accept "--opt value" and "--opt=value". --guide wins over --dry-run
  when both are given. --record is made absolute against the caller's cwd.
  The only file written is --record. An existing --record path is refused
  without --overwrite, including a dangling symlink.
  The advisory port check (ss -ltn) runs only for rfc2217 URLs on localhost or
  127.0.0.1 with an explicit port, and is skipped for bracketed IPv6 hosts and
  other hosts.

Environment:
  BRC_DIR   balancing-robot-controller checkout (used by --guide only)
            (default: $HOME/Documents/platform-io-workspace/balancing-robot-controller)
  FW_DIR    freecad-workspace checkout; the bridge runs from
            FW_DIR/poc/esp32-dof/monitor
            (default: three levels above this script)
  The mamba env is fixed to esp32-dof.

Exit codes: 0 ok; 1 FATAL before start (record exists, mamba missing, monitor
dir missing); 2 bad arguments; 130 Ctrl+C; otherwise the bridge's exit code.
EOF
}

fatal() {
    printf 'FATAL: %s\n' "$1" >&2
    exit 1
}

warn() {
    printf 'WARNING: %s\n' "$1" >&2
}

heading() {
    if [ "$COLOR" -eq 1 ]; then
        printf '\033[1;36m%s\033[0m\n' "$1"
    else
        printf '%s\n' "$1"
    fi
}

cmd_line() {
    if [ "$COLOR" -eq 1 ]; then
        printf '\033[32m%s\033[0m\n' "$1"
    else
        printf '%s\n' "$1"
    fi
}

# Set one option from a value. An empty value, or one starting with "--"
# (a missing value followed by another flag), is a bad argument.
set_opt() {
    local name="$1" val="$2"
    case "$val" in
        ""|--*)
            echo "wokwi-rfc2217-check: option $name needs a non-empty value (got '$val')" >&2
            usage_short
            exit 2
            ;;
    esac
    case "$name" in
        --port) PORT_URL="$val" ;;
        --core) CORE="$val" ;;
        --duration) DURATION="$val" ;;
        --record) RECORD="$val" ;;
    esac
}

print_guide() {
    printf '%s\n' "SIM-ONLY, not hardware-validated. Webots wheel signs UNVERIFIED. This only shows the pipe works."
    printf '\n'
    heading "1 Env"
    printf '%s\n' "BRC and FW below are shell variables for these steps; the script itself reads BRC_DIR and FW_DIR."
    cmd_line "export BRC=$(printf '%q' "$BRC_DIR")"
    cmd_line "export FW=$(printf '%q' "$FW_DIR")"
    cmd_line 'export PATH="$HOME/.platformio/penv/bin:$HOME/.wokwi/bin:$PATH"'
    cmd_line "export WOKWI_CLI_TOKEN='<your token>'"
    printf '\n'
    heading "2 Build (in BRC)"
    cmd_line 'cd "$BRC" && PLATFORMIO_BUILD_FLAGS="-D TTL_SERVO_DOF_FRAMES" pio run -e ttl_servo-xiao_c3'
    printf '\n'
    heading "3 Webots (in FW)"
    cmd_line 'cd "$FW/poc/servo-wheel-dof" && ./run_demo.sh --gui'
    printf '%s\n' "   UDP 5006, needs DISPLAY"
    printf '\n'
    heading "4 Bridge (this script)"
    cmd_line '"$FW/poc/esp32-dof/tools/wokwi-rfc2217-check.sh"'
    printf '%s\n' "   start it first, it waits up to 300 s for the port"
    printf '%s\n' "   add --overwrite if the --record file from an earlier run exists"
    printf '\n'
    heading "5 VS Code simulator"
    printf '%s\n' '   open programs/ttl_servo in $BRC and run "Wokwi: Start Simulator"'
    printf '%s\n' '   keep the tab visible, gyro Z slider at 0 during boot'
    printf '%s\n' '   stop the bridge with Ctrl+C or wait for --duration'
    printf '\n'
    heading "6 Rebuild WITHOUT the flag afterwards (shared .pio)"
    cmd_line 'cd "$BRC" && pio run -e ttl_servo-xiao_c3'
    printf '\n'
    printf '%s\n' "Note: if port 4000 is still listening after the simulator stops (VS Code keeps its listener open), close the tab or reload the VS Code window before a bridge-first run."
}

print_dry_run() {
    local cmd_str
    printf -v cmd_str '%q ' "${BRIDGE_ARGV[@]}"
    heading "Dry run: nothing is started and nothing is written."
    printf '%s\n' "Checks the real run does, in order (--dry-run performs none of them):"
    printf '  %s\n' \
        "1. $RECORD exists and no --overwrite: FATAL, exit 1" \
        "2. mamba on PATH: FATAL, exit 1 if missing" \
        "3. $MON_DIR exists: FATAL, exit 1 if missing" \
        "4. ss -ltn: WARNING if the port is listening; runs only for rfc2217 URLs on localhost or 127.0.0.1 with an explicit port, when ss exists (advisory, never gates)" \
        "5. reminder to stderr, then the bridge runs (it waits up to 300 s for the port)"
    heading "Command:"
    cmd_line "cd $(printf '%q' "$MON_DIR") && ${cmd_str% }"
}

# Advisory only: warn if the RFC2217 port already has a listener on localhost.
check_listener() {
    local rest hostport host port
    case "$PORT_URL" in
        rfc2217://*) rest="${PORT_URL#rfc2217://}" ;;
        *) return 0 ;;
    esac
    hostport="${rest%%/*}"
    case "$hostport" in
        *:*) ;;
        *) return 0 ;;
    esac
    host="${hostport%:*}"
    port="${hostport##*:}"
    case "$host" in
        localhost|127.0.0.1) ;;
        *) return 0 ;;
    esac
    [[ "$port" =~ ^[0-9]{1,5}$ ]] || return 0
    command -v ss >/dev/null 2>&1 || return 0
    if ss -ltn 2>/dev/null | grep -Eq ":${port}[[:space:]]"; then
        warn "port $port is already listening on $host. A VS Code listener may still hold it after the simulator stopped. Continuing."
    fi
}

# Parse flags. Accepts "--opt value" and "--opt=value". "--" ends flags.
while [ "$#" -gt 0 ]; do
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        --guide)
            GUIDE=1
            shift
            ;;
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        --overwrite)
            OVERWRITE=1
            shift
            ;;
        --)
            shift
            PASSTHROUGH=("$@")
            break
            ;;
        --port|--core|--duration|--record)
            if [ "$#" -lt 2 ]; then
                echo "wokwi-rfc2217-check: option $1 needs a value" >&2
                usage_short
                exit 2
            fi
            set_opt "$1" "$2"
            shift 2
            ;;
        --port=*|--core=*|--duration=*|--record=*)
            set_opt "${1%%=*}" "${1#*=}"
            shift
            ;;
        *)
            echo "wokwi-rfc2217-check: unknown argument: $1" >&2
            usage_short
            exit 2
            ;;
    esac
done

# Validate. Bad input exits 2 before any mamba or bridge work.
case "$CORE" in
    lqr|pid|none) ;;
    *)
        echo "wokwi-rfc2217-check: invalid --core '$CORE' (want lqr, pid or none)" >&2
        usage_short
        exit 2
        ;;
esac
if ! [[ "$DURATION" =~ ^[0-9]{1,9}$ ]] || [ "$((10#$DURATION))" -lt 1 ]; then
    echo "wokwi-rfc2217-check: invalid --duration '$DURATION' (want a positive integer of seconds)" >&2
    usage_short
    exit 2
fi
DURATION=$((10#$DURATION))

# Make the record path absolute now: the bridge runs after cd "$MON_DIR".
case "$RECORD" in
    /*) ;;
    *) RECORD="$PWD/$RECORD" ;;
esac

# Reserved bridge flags are set only by this script's own options. argparse is
# last-wins and accepts prefixes, so a passthrough token that is any prefix of a
# reserved flag (exact, as --flag=value, or shorter, e.g. --r) is refused.
RESERVED_FLAGS="--port --core --duration --record --stdin --replay --mock --text-log"
for tok in ${PASSTHROUGH[@]+"${PASSTHROUGH[@]}"}; do
    name="${tok%%=*}"
    case "$name" in
        --?*) ;;
        *) continue ;;
    esac
    for flag in $RESERVED_FLAGS; do
        case "$flag" in
            "$name"*)
                echo "wokwi-rfc2217-check: passthrough flag '$tok' is reserved (use the script option instead)" >&2
                usage_short
                exit 2
                ;;
        esac
    done
done

BRIDGE_ARGV=(mamba run -a "" -n "$PY_ENV" python3 dof_wheel_bridge.py \
    --port "$PORT_URL" --core "$CORE" --duration "$DURATION" \
    --record "$RECORD" --echo-text ${PASSTHROUGH[@]+"${PASSTHROUGH[@]}"})

if [ "$GUIDE" -eq 1 ]; then
    print_guide
    exit 0
fi

if [ "$DRY_RUN" -eq 1 ]; then
    print_dry_run
    exit 0
fi

# Order: record clobber -> mamba -> monitor dir -> advisory check -> run.
# [ -e ] follows symlinks, so a dangling symlink is caught by [ -L ] too.
if { [ -e "$RECORD" ] || [ -L "$RECORD" ]; } && [ "$OVERWRITE" -ne 1 ]; then
    fatal "record file exists: $RECORD (use --overwrite to replace it)"
fi
if ! command -v mamba >/dev/null 2>&1; then
    fatal "mamba not on PATH (it runs the env '$PY_ENV')"
fi
if [ ! -d "$MON_DIR" ]; then
    fatal "monitor dir not found: $MON_DIR (set FW_DIR to the freecad-workspace checkout)"
fi

check_listener

echo "wokwi-rfc2217-check: start order: the bridge may start first; it waits up to 300 s for $PORT_URL. Then start the simulator (--guide step 5)." >&2
echo "wokwi-rfc2217-check: running the bridge from $MON_DIR (duration ${DURATION} s, record $RECORD)" >&2

( cd "$MON_DIR" && "${BRIDGE_ARGV[@]}" )
rc=$?
exit "$rc"
