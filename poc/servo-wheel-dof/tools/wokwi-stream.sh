#!/bin/bash
# Stream lines from a running Wokwi simulation's RFC2217 serial port to stdout.
#
# stdout: data lines only (safe to pipe, e.g. into dof_wheel_bridge --stdin).
# stderr: status and error lines only.
# Python runs in the mamba env "esp32-dof" (pyserial 3.5); works from any cwd.

set -u

DEFAULT_HOST=localhost
DEFAULT_PORT=4000
DEFAULT_BAUD=115200
PY_ENV=esp32-dof

usage_short() {
    echo "Usage: wokwi-stream.sh [--host HOST] [--port PORT] [--baud BAUD]" >&2
    echo "Try 'wokwi-stream.sh --help' for details." >&2
}

usage() {
    cat <<'EOF'
Usage: wokwi-stream.sh [--host HOST] [--port PORT] [--baud BAUD]
       wokwi-stream.sh --help

Stream lines from a running Wokwi simulation (RFC2217 serial port) to stdout.
Status and errors go to stderr. Press Ctrl+C to stop.

Options:
  --host HOST   RFC2217 host (default: localhost)
  --port PORT   RFC2217 TCP port, 1-65535 (default: 4000)
  --baud BAUD   baud rate, positive integer (default: 115200)
  -h, --help    show this help and exit

Environment defaults (flags win over these):
  WOKWI_HOST, WOKWI_PORT, BAUD

Needs: a running Wokwi simulation exposing an RFC2217 serial port, and the
mamba env "esp32-dof" (pyserial). Exit codes: 0 ok, 1 connection or serial
error or missing env, 2 bad arguments, 130 stopped by Ctrl+C.
EOF
}

HOST="${WOKWI_HOST:-$DEFAULT_HOST}"
PORT="${WOKWI_PORT:-$DEFAULT_PORT}"
BAUD="${BAUD:-$DEFAULT_BAUD}"

# Parse flags. Accepts "--opt value" and "--opt=value".
while [ "$#" -gt 0 ]; do
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        --host|--port|--baud)
            if [ "$#" -lt 2 ]; then
                echo "wokwi-stream: option $1 needs a value" >&2
                usage_short
                exit 2
            fi
            case "$1" in
                --host) HOST="$2" ;;
                --port) PORT="$2" ;;
                --baud) BAUD="$2" ;;
            esac
            shift 2
            ;;
        --host=*) HOST="${1#--host=}"; shift ;;
        --port=*) PORT="${1#--port=}"; shift ;;
        --baud=*) BAUD="${1#--baud=}"; shift ;;
        *)
            echo "wokwi-stream: unknown argument: $1" >&2
            usage_short
            exit 2
            ;;
    esac
done

# Validate. Bad input exits 2 before any Python or mamba work.
if [ -z "$HOST" ]; then
    echo "wokwi-stream: host must not be empty" >&2
    usage_short
    exit 2
fi
if ! [[ "$PORT" =~ ^[0-9]{1,5}$ ]] || [ "$((10#$PORT))" -lt 1 ] || [ "$((10#$PORT))" -gt 65535 ]; then
    echo "wokwi-stream: invalid port '$PORT' (want an integer 1-65535)" >&2
    usage_short
    exit 2
fi
if ! [[ "$BAUD" =~ ^[0-9]{1,9}$ ]] || [ "$((10#$BAUD))" -lt 1 ]; then
    echo "wokwi-stream: invalid baud '$BAUD' (want a positive integer)" >&2
    usage_short
    exit 2
fi
PORT=$((10#$PORT))
BAUD=$((10#$BAUD))

if ! command -v mamba >/dev/null 2>&1; then
    echo "wokwi-stream: mamba not on PATH; it runs the pyserial env '$PY_ENV'" >&2
    exit 1
fi
if ! mamba run -n "$PY_ENV" python3 -c 'import serial' >/dev/null 2>&1; then
    echo "wokwi-stream: mamba env '$PY_ENV' with pyserial is missing or broken (create it from poc/esp32-dof/mamba-envs.lock.yml)" >&2
    exit 1
fi

# Python body, quoted heredoc: no shell expansion inside.
# argv: host port baud. "-a ''" makes mamba inherit stdio (no output capture),
# so lines reach the pipe as they arrive.
read -r -d '' PY_BODY <<'PYEOF'
import sys
import time


def status(msg):
    sys.stderr.write("wokwi-stream: " + msg + "\n")
    sys.stderr.flush()


def main():
    host = sys.argv[1]
    port = int(sys.argv[2])
    baud = int(sys.argv[3])
    where = "%s:%d" % (host, port)

    import serial

    ser = None
    try:
        try:
            ser = serial.serial_for_url("rfc2217://%s" % where, baudrate=baud, timeout=1)
        except (serial.SerialException, OSError) as e:
            status("cannot connect to %s (%s). Is the Wokwi simulation running "
                   "with its RFC2217 port open?" % (where, e))
            return 1

        status("connected to %s at %d baud. Press Ctrl+C to stop." % (where, baud))
        while True:
            data = ser.readline()
            if data:
                sys.stdout.write(data.decode("ascii", errors="replace"))
                sys.stdout.flush()
            else:
                time.sleep(0.01)
    except KeyboardInterrupt:
        status("stopped by user")
        return 130
    except BrokenPipeError:
        # Downstream reader closed the pipe: stop quietly.
        return 0
    except (serial.SerialException, OSError) as e:
        status("serial error on %s (%s)" % (where, e))
        return 1
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass


sys.exit(main())
PYEOF

mamba run -a "" -n "$PY_ENV" python3 -c "$PY_BODY" "$HOST" "$PORT" "$BAUD"
exit $?
