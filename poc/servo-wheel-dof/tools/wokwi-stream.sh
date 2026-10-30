#!/bin/bash
# Stream lines from a running Wokwi simulation's RFC2217 serial port to stdout.
#
# stdout: data lines, bytes copied as received, with two exceptions that each
#         add one newline: a pending partial line at exit (not after Ctrl+C),
#         and a newline-less run of 64 KiB that is split. Intended to be pipeable (for example
#         into dof_wheel_bridge --stdin); that pipe is not verified.
# stderr: status and error lines only.
# Python runs in the mamba env "esp32-dof" (pyserial 3.5); works from any cwd.

set -u

# Ctrl+C always ends with exit 130 (checked with a fake serial object only).
trap 'exit 130' INT

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
  --host HOST   RFC2217 host: a name or IPv4 address, or IPv6 as ::1 or [::1]
                (default: localhost). Put the port in --port, not in HOST.
  --port PORT   RFC2217 TCP port, 1-65535 (default: 4000)
  --baud BAUD   baud rate, positive integer (default: 115200)
  -h, --help    show this help and exit

Environment defaults (flags win over these):
  WOKWI_HOST, WOKWI_PORT, BAUD
  An empty value (for example WOKWI_PORT=) counts as unset.

Output: bytes as received, one line per output line. A newline is added to a
partial line left at exit (not after Ctrl+C), and to a run of 64 KiB without a
newline (split).

Needs: a running Wokwi simulation exposing an RFC2217 serial port, and the
mamba env "esp32-dof" (pyserial). Exit codes: 0 ok, 1 connection, serial,
stdout or env error, 2 bad arguments, 130 stopped by Ctrl+C.
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
host_error() {
    echo "wokwi-stream: $1" >&2
    usage_short
    exit 2
}
space_re='[[:space:]]'
bracketed_re='^\[[0-9A-Fa-f:.]+\]$'
bracketed_port_re='^\[.*\]:[0-9]+$'
bare_v6_re='^[0-9A-Fa-f:.]+$'
if [[ "$HOST" =~ $space_re ]]; then
    host_error "host must not contain whitespace"
fi
if [[ "$HOST" == *%* ]]; then
    host_error "IPv6 zone IDs are not supported (host '$HOST')"
elif [[ "$HOST" == \[* ]]; then
    if [[ "$HOST" =~ $bracketed_re ]]; then
        :
    elif [[ "$HOST" =~ $bracketed_port_re ]]; then
        host_error "host '$HOST' ends in a port; use --port for the port; IPv6 as [::1]"
    elif [[ "$HOST" == *\]* ]]; then
        host_error "invalid host '$HOST' (IPv6 must be written [::1])"
    else
        host_error "invalid host '$HOST' (unmatched '[')"
    fi
elif [[ "$HOST" == *:* ]]; then
    colons="${HOST//[^:]/}"
    if ! [[ "$HOST" =~ $bare_v6_re ]] || [ "${#colons}" -lt 2 ]; then
        host_error "host '$HOST' has a colon but is not IPv6; use --port for the port; IPv6 as [::1]"
    fi
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
# argv: host port baud. mamba run -a "" makes mamba inherit stdio (no output
# capture), so lines reach the pipe as they arrive.
read -r -d '' PY_BODY <<'PYEOF'
import os
import sys
import time

MAX_PENDING = 64 * 1024


class Downstream(Exception):
    """The reader of stdout closed the pipe (EPIPE)."""


class StdoutFailure(Exception):
    """Writing stdout failed for another reason (ENOSPC, EIO, ...)."""


def status(msg):
    sys.stderr.write("wokwi-stream: " + msg + "\n")
    sys.stderr.flush()


def emit(data):
    # Raw os.write, looped: each partial write returns to Python, so a Ctrl+C
    # is raised at once even while the pipe is full. (A buffered write keeps
    # going until the whole block is written, so Ctrl+C would wait.)
    view = memoryview(data)
    try:
        while view:
            view = view[os.write(1, view):]
    except BrokenPipeError:
        raise Downstream()
    except OSError as e:
        raise StdoutFailure(str(e))


def emit_consume(pending, length, add_newline):
    # Write the first `length` bytes of pending (plus a newline if asked), then
    # drop them from pending. The write is not masked: a Ctrl+C raises out of a
    # blocked write at once. After a Ctrl+C the exit flush is skipped (see main),
    # so a line is never written twice.
    data = bytes(pending[:length]) + (b"\n" if add_newline else b"")
    emit(data)
    del pending[:length]


def drain(pending, state):
    # Emit complete lines whole, whatever their length. A partial line waits
    # for its newline. A newline-less run is split every MAX_PENDING bytes
    # (exactly that many, plus a newline); the rest stays pending.
    while True:
        nl = pending.find(b"\n")
        if nl < 0:
            break
        emit_consume(pending, nl + 1, False)
    while len(pending) >= MAX_PENDING:
        emit_consume(pending, MAX_PENDING, True)
        if not state["noted"]:
            state["noted"] = True
            status("run of %d bytes without a newline: split" % MAX_PENDING)


def quiet_stdout():
    # Python docs recipe: after a stdout error (EPIPE or other), point stdout at
    # devnull so the exit-time flush cannot print "Exception ignored ...".
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, sys.stdout.fileno())


def main():
    host = sys.argv[1]
    port = int(sys.argv[2])
    baud = int(sys.argv[3])
    # A bare IPv6 literal must be bracketed in a URL (bash already checked it).
    shown = "[%s]" % host if ":" in host and not host.startswith("[") else host
    where = "%s:%d" % (shown, port)

    import serial

    ser = None
    pending = bytearray()
    state = {"noted": False}
    broken = False
    output_failed = False
    interrupted = False
    rc = 0
    try:
        try:
            ser = serial.serial_for_url("rfc2217://%s" % where, baudrate=baud, timeout=1)
        except (serial.SerialException, OSError) as e:
            status("cannot connect to %s (%s). Is the Wokwi simulation running "
                   "with its RFC2217 port open?" % (where, e))
            return 1

        status("connected to %s at %d baud. Press Ctrl+C to stop." % (where, baud))
        while True:
            # read_until returns at a newline, at MAX_PENDING bytes, or with the
            # partial bytes read so far after a timeout (b"" if none).
            chunk = ser.read_until(b"\n", MAX_PENDING)
            if chunk:
                pending += chunk
                drain(pending, state)
            else:
                time.sleep(0.01)
    except KeyboardInterrupt:
        status("stopped by user")
        interrupted = True
        rc = 130
    except Downstream:
        broken = True
        output_failed = True
        rc = 0
    except StdoutFailure as e:
        status("cannot write to stdout: %s" % e)
        broken = True
        output_failed = True
        rc = 1
    except (serial.SerialException, OSError) as e:
        status("serial error on %s (%s)" % (where, e))
        rc = 1
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        if pending and not output_failed and not interrupted:
            # Flush a partial line left at exit, with its newline. Not after
            # Ctrl+C: that would repeat a line already written and could block.
            try:
                emit(bytes(pending) + b"\n")
            except Downstream:
                broken = True
            except StdoutFailure as e:
                status("cannot write to stdout: %s" % e)
                broken = True
                rc = 1
            except KeyboardInterrupt:
                rc = 130  # a second Ctrl+C during the flush: quiet exit 130
        if broken:
            quiet_stdout()
    return rc


sys.exit(main())
PYEOF

mamba run -a "" -n "$PY_ENV" python3 -c "$PY_BODY" "$HOST" "$PORT" "$BAUD"
exit $?
