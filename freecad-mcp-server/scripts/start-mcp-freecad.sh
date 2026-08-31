#!/bin/bash

################################################################################
# FreeCAD MCP Server & Client Launcher
#
# Starts FreeCAD with Robust MCP Bridge and validates connection
# Supports: XML-RPC mode, Socket mode, interactive menu, command-line args
################################################################################

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
FREECAD_APPIMAGE="${FREECAD_APPIMAGE:-$HOME/.local/bin/FreeCAD_1.1.3-Linux-x86_64-py311.AppImage}"
MAMBA_ENV="freecad-mcp"
MAMBA_ACTIVATE="$HOME/miniforge3/bin/activate"
MAMBA_ENV_PREFIX="$HOME/miniforge3/envs/$MAMBA_ENV"
LOG_DIR="$PROJECT_DIR/logs"
LOG_FILE="$LOG_DIR/mcp-server.log"

# Default configuration
MODE="xmlrpc"  # xmlrpc or socket
XMLRPC_PORT=9875
SOCKET_PORT=9876
HOST="localhost"
TIMEOUT=10
START_FREECAD=true
VALIDATE=true

# State tracking
MCP_PID=""
FREECAD_PID=""
MAMBA_ENV_ACTIVATED=false

################################################################################
# Mamba Environment Setup
################################################################################

activate_mamba_env() {
    if [ -f "$MAMBA_ACTIVATE" ]; then
        # shellcheck disable=SC1090
        source "$MAMBA_ACTIVATE" "$MAMBA_ENV" 2>/dev/null
        if [ "${CONDA_DEFAULT_ENV:-}" = "$MAMBA_ENV" ]; then
            MAMBA_ENV_ACTIVATED=true
            return 0
        fi
        return 1
    else
        return 1
    fi
}

run_in_mamba_env() {
    local cmd="$1"
    if [ "$MAMBA_ENV_ACTIVATED" = true ]; then
        eval "$cmd"
    else
        # Fallback: run inside the mamba env without activating in this shell
        mamba run -n "$MAMBA_ENV" bash -c "$cmd"
    fi
}

################################################################################
# Helper Functions
################################################################################

log() {
    echo -e "${CYAN}[$(date '+%Y-%m-%d %H:%M:%S')]${NC} $*" | tee -a "$LOG_FILE"
}

error() {
    echo -e "${RED}[ERROR]${NC} $*" >&2 | tee -a "$LOG_FILE"
}

success() {
    echo -e "${GREEN}[✓]${NC} $*" | tee -a "$LOG_FILE"
}

warning() {
    echo -e "${YELLOW}[⚠]${NC} $*" | tee -a "$LOG_FILE"
}

info() {
    echo -e "${BLUE}[i]${NC} $*" | tee -a "$LOG_FILE"
}

print_header() {
    echo -e "\n${BLUE}════════════════════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}\n"
}

cleanup() {
    local exit_code=$?

    if [ -n "$MCP_PID" ]; then
        warning "Stopping MCP server (PID: $MCP_PID)..."
        kill "$MCP_PID" 2>/dev/null || true
    fi

    if [ -n "$FREECAD_PID" ]; then
        warning "Stopping FreeCAD (PID: $FREECAD_PID)..."
        kill "$FREECAD_PID" 2>/dev/null || true
    fi

    info "Cleanup complete. Log file: $LOG_FILE"
    exit $exit_code
}

trap cleanup EXIT INT TERM

################################################################################
# Validation & Diagnostics
################################################################################

check_dependencies() {
    print_header "Checking Dependencies"

    # Check Python
    if ! command -v python3 &> /dev/null; then
        error "Python 3 not found"
        return 1
    fi
    success "Python 3: $(python3 --version)"

    # Check FreeCAD AppImage
    if [ ! -f "$FREECAD_APPIMAGE" ]; then
        error "FreeCAD AppImage not found: $FREECAD_APPIMAGE"
        warning "Set FREECAD_APPIMAGE environment variable or place AppImage at: $FREECAD_APPIMAGE"
        return 1
    fi
    success "FreeCAD AppImage: $(basename "$FREECAD_APPIMAGE")"

    # Check mamba env
    if [ ! -d "$MAMBA_ENV_PREFIX" ]; then
        error "Mamba environment not found: $MAMBA_ENV"
        return 1
    fi
    success "Mamba environment: $MAMBA_ENV"

    return 0
}

check_freecad_mcp() {
    print_header "Checking FreeCAD MCP Installation"

    # Activate mamba env
    if ! activate_mamba_env; then
        error "Failed to activate mamba environment: $MAMBA_ENV"
        return 1
    fi

    success "Mamba environment activated: $MAMBA_ENV"

    # Check if freecad-robust-mcp is installed
    if ! python3 -c "import freecad_mcp" 2>/dev/null; then
        error "freecad-robust-mcp not installed in mamba env: $MAMBA_ENV"
        info "Run: cd $PROJECT_DIR && mamba env create -n $MAMBA_ENV -f mamba-envs.yaml"
        return 1
    fi

    success "freecad-robust-mcp installed"

    # Show Python info
    info "Python: $(python3 --version)"
    info "Python executable: $(which python3)"

    return 0
}

check_port_available() {
    local port=$1
    local name=$2

    if nc -z "$HOST" "$port" 2>/dev/null; then
        warning "Port $port ($name) already in use"
        return 1
    else
        success "Port $port ($name) available"
        return 0
    fi
}

check_mcp_connection() {
    local max_attempts=30
    local attempt=0

    print_header "Validating MCP Bridge Connection"

    while [ $attempt -lt $max_attempts ]; do
        if [ "$MODE" = "xmlrpc" ]; then
            if nc -z "$HOST" "$XMLRPC_PORT" 2>/dev/null; then
                success "MCP Bridge connected on XML-RPC: $HOST:$XMLRPC_PORT"
                return 0
            fi
        elif [ "$MODE" = "socket" ]; then
            if nc -z "$HOST" "$SOCKET_PORT" 2>/dev/null; then
                success "MCP Bridge connected on Socket: $HOST:$SOCKET_PORT"
                return 0
            fi
        fi

        echo -ne "\r  Attempting connection ($((attempt + 1))/$max_attempts)..."
        sleep 1
        ((attempt++))
    done

    echo
    error "Failed to connect to MCP Bridge after $max_attempts attempts"
    return 1
}

show_connection_info() {
    print_header "Connection Information"

    if [ "$MODE" = "xmlrpc" ]; then
        info "Mode: XML-RPC (recommended)"
        info "Endpoint: http://$HOST:$XMLRPC_PORT"
        info "Environment: FREECAD_MODE=xmlrpc"
    elif [ "$MODE" = "socket" ]; then
        info "Mode: JSON-RPC Socket"
        info "Endpoint: ws://$HOST:$SOCKET_PORT"
        info "Environment: FREECAD_MODE=socket"
    fi

    echo
    info "MCP Client Configuration (.mcp.json):"
    cat << EOF
{
  "mcpServers": {
    "freecad": {
      "command": "freecad-mcp",
      "env": {
        "FREECAD_MODE": "$MODE",
        "FREECAD_SOCKET_HOST": "$HOST",
        "FREECAD_XMLRPC_PORT": "$XMLRPC_PORT"
      }
    }
  }
}
EOF
    echo
}

show_status() {
    print_header "Service Status"

    # Check MCP server
    if [ -n "$MCP_PID" ] && kill -0 "$MCP_PID" 2>/dev/null; then
        success "MCP Server running (PID: $MCP_PID)"
    else
        error "MCP Server not running"
    fi

    # Check FreeCAD
    if [ -n "$FREECAD_PID" ] && kill -0 "$FREECAD_PID" 2>/dev/null; then
        success "FreeCAD running (PID: $FREECAD_PID)"
    else
        error "FreeCAD not running"
    fi

    # Check ports
    echo
    if nc -z "$HOST" "$XMLRPC_PORT" 2>/dev/null; then
        success "XML-RPC port $XMLRPC_PORT listening"
    else
        warning "XML-RPC port $XMLRPC_PORT not listening"
    fi

    if nc -z "$HOST" "$SOCKET_PORT" 2>/dev/null; then
        success "Socket port $SOCKET_PORT listening"
    else
        warning "Socket port $SOCKET_PORT not listening"
    fi
}

################################################################################
# Startup Functions
################################################################################

start_mcp_server() {
    print_header "Starting MCP Server"

    # Create log directory
    mkdir -p "$LOG_DIR"

    # Verify mamba env is activated
    if [ "$MAMBA_ENV_ACTIVATED" = false ]; then
        error "Mamba environment not activated"
        return 1
    fi

    info "Mode: $MODE"
    info "Host: $HOST"
    info "Mamba Environment: $MAMBA_ENV ($CONDA_PREFIX)"

    # Export configuration to environment
    export FREECAD_MODE="$MODE"
    export FREECAD_SOCKET_HOST="$HOST"
    export FREECAD_XMLRPC_PORT="$XMLRPC_PORT"
    export FREECAD_SOCKET_PORT="$SOCKET_PORT"

    log "Starting: freecad-mcp"
    log "Environment Variables:"
    log "  FREECAD_MODE=$FREECAD_MODE"
    log "  FREECAD_SOCKET_HOST=$FREECAD_SOCKET_HOST"
    log "  FREECAD_XMLRPC_PORT=$FREECAD_XMLRPC_PORT"
    log "  FREECAD_SOCKET_PORT=$FREECAD_SOCKET_PORT"
    log "  CONDA_PREFIX=$CONDA_PREFIX"

    # Start server in background with full mamba env binary path for reliability
    "$MAMBA_ENV_PREFIX/bin/freecad-mcp" >> "$LOG_FILE" 2>&1 &
    MCP_PID=$!

    # Verify process started
    sleep 1
    if ! kill -0 "$MCP_PID" 2>/dev/null; then
        error "Failed to start MCP server. Check logs:"
        tail -20 "$LOG_FILE"
        return 1
    fi

    success "MCP Server started (PID: $MCP_PID)"
    log "Command: $MAMBA_ENV_PREFIX/bin/freecad-mcp"
    log "Logs: $LOG_FILE"

    # Wait for server to fully initialize
    sleep 2

    return 0
}

start_freecad() {
    print_header "Starting FreeCAD"

    if [ ! -f "$FREECAD_APPIMAGE" ]; then
        error "FreeCAD AppImage not found: $FREECAD_APPIMAGE"
        return 1
    fi

    info "AppImage: $(basename "$FREECAD_APPIMAGE")"
    log "Launching: $FREECAD_APPIMAGE"

    # Start FreeCAD in background
    QT_QPA_PLATFORM=wayland "$FREECAD_APPIMAGE" >> "$LOG_FILE" 2>&1 &
    FREECAD_PID=$!

    success "FreeCAD started (PID: $FREECAD_PID)"
    info "Waiting for FreeCAD to initialize (15 seconds)..."
    sleep 15

    return 0
}

################################################################################
# Interactive Menu
################################################################################

show_menu() {
    echo
    echo -e "${CYAN}╔════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║   FreeCAD MCP Server Launcher              ║${NC}"
    echo -e "${CYAN}╚════════════════════════════════════════════╝${NC}"
    echo
    echo "1) Start MCP Server + FreeCAD (XML-RPC mode)"
    echo "2) Start MCP Server + FreeCAD (Socket mode)"
    echo "3) Start MCP Server only (XML-RPC)"
    echo "4) Start MCP Server only (Socket)"
    echo "5) Start FreeCAD only"
    echo "6) Check service status"
    echo "7) Show connection info"
    echo "8) View logs"
    echo "9) Exit"
    echo
}

interactive_menu() {
    print_header "FreeCAD MCP Server Launcher - Interactive Mode"

    while true; do
        show_menu
        read -p "Select option (1-9): " choice

        case $choice in
            1)
                MODE="xmlrpc"
                START_FREECAD=true
                check_dependencies && check_freecad_mcp && \
                    check_port_available "$XMLRPC_PORT" "XML-RPC" && \
                    start_mcp_server && start_freecad && \
                    VALIDATE=true
                ;;
            2)
                MODE="socket"
                START_FREECAD=true
                check_dependencies && check_freecad_mcp && \
                    check_port_available "$SOCKET_PORT" "Socket" && \
                    start_mcp_server && start_freecad && \
                    VALIDATE=true
                ;;
            3)
                MODE="xmlrpc"
                START_FREECAD=false
                check_dependencies && check_freecad_mcp && \
                    check_port_available "$XMLRPC_PORT" "XML-RPC" && \
                    start_mcp_server && \
                    VALIDATE=false
                ;;
            4)
                MODE="socket"
                START_FREECAD=false
                check_dependencies && check_freecad_mcp && \
                    check_port_available "$SOCKET_PORT" "Socket" && \
                    start_mcp_server && \
                    VALIDATE=false
                ;;
            5)
                START_FREECAD=true
                check_dependencies && start_freecad
                ;;
            6)
                show_status
                ;;
            7)
                show_connection_info
                ;;
            8)
                if [ -f "$LOG_FILE" ]; then
                    less "$LOG_FILE"
                else
                    warning "No log file found"
                fi
                ;;
            9)
                info "Exiting..."
                exit 0
                ;;
            *)
                error "Invalid option. Please select 1-9."
                ;;
        esac

        if [ "$VALIDATE" = true ]; then
            check_mcp_connection && show_connection_info
            VALIDATE=false
        fi

        echo -e "\n${YELLOW}Press Enter to continue...${NC}"
        read
    done
}

################################################################################
# Command-line Argument Parsing
################################################################################

show_help() {
    cat << EOF
Usage: $0 [OPTIONS]

Start FreeCAD with Robust MCP Bridge

OPTIONS:
    --help, -h              Show this help message
    --mode MODE             Connection mode: xmlrpc (default) or socket
    --host HOST             Host to bind to (default: localhost)
    --port PORT             Port for XMLRPC (default: 9875) or Socket (default: 9876)
    --freecad-only          Start only FreeCAD (no MCP server)
    --mcp-only              Start only MCP server (no FreeCAD)
    --no-validate           Skip connection validation
    --skip-freecad          Skip FreeCAD launch (start MCP only)
    --appimage PATH         Path to FreeCAD AppImage (default: ~/.local/bin/FreeCAD_1.1.3-Linux-x86_64-py311.AppImage)
    --interactive, -i       Show interactive menu (default if no args)
    --version               Show version info

EXAMPLES:
    # Interactive menu
    $0 --interactive

    # Start with XML-RPC (default)
    $0 --mode xmlrpc

    # Start with Socket mode
    $0 --mode socket

    # Start only MCP server
    $0 --mcp-only --mode xmlrpc

    # Start only FreeCAD
    $0 --freecad-only

    # Custom AppImage path
    $0 --appimage /path/to/FreeCAD.AppImage

EOF
}

parse_args() {
    local use_interactive=false

    # If no arguments, use interactive mode
    if [ $# -eq 0 ]; then
        use_interactive=true
    fi

    while [ $# -gt 0 ]; do
        case "$1" in
            --help|-h)
                show_help
                exit 0
                ;;
            --version)
                info "FreeCAD MCP Server Launcher v1.0.0"
                exit 0
                ;;
            --mode)
                MODE="$2"
                shift 2
                ;;
            --host)
                HOST="$2"
                shift 2
                ;;
            --port)
                if [ "$MODE" = "xmlrpc" ]; then
                    XMLRPC_PORT="$2"
                else
                    SOCKET_PORT="$2"
                fi
                shift 2
                ;;
            --freecad-only)
                START_FREECAD=true
                skip_mcp=true
                shift
                ;;
            --mcp-only)
                START_FREECAD=false
                shift
                ;;
            --skip-freecad)
                START_FREECAD=false
                shift
                ;;
            --no-validate)
                VALIDATE=false
                shift
                ;;
            --appimage)
                FREECAD_APPIMAGE="$2"
                shift 2
                ;;
            --interactive|-i)
                use_interactive=true
                shift
                ;;
            *)
                error "Unknown option: $1"
                show_help
                exit 1
                ;;
        esac
    done

    return 0
}

################################################################################
# Main Entry Point
################################################################################

main() {
    # Parse arguments
    parse_args "$@"

    # Show interactive menu if no specific options provided
    if [ $# -eq 0 ] || { [ $# -eq 1 ] && [[ "$1" == "--interactive" || "$1" == "-i" ]]; }; then
        interactive_menu
    fi

    # Print startup banner
    print_header "FreeCAD MCP Server Launcher"
    info "Project: $PROJECT_DIR"
    info "Mamba Environment: $MAMBA_ENV"
    info "Configuration Dir: $(pwd)"
    echo

    # Run checks
    check_dependencies || exit 1
    check_freecad_mcp || exit 1

    # Verify mamba env is activated
    if [ "$MAMBA_ENV_ACTIVATED" = false ]; then
        warning "Mamba environment not activated. Attempting to activate..."
        activate_mamba_env || {
            error "Failed to activate mamba environment: $MAMBA_ENV"
            exit 1
        }
    fi

    # Start services based on configuration
    if [ "$START_FREECAD" = false ]; then
        # MCP only
        if [ "$MODE" = "xmlrpc" ]; then
            check_port_available "$XMLRPC_PORT" "XML-RPC" || exit 1
        else
            check_port_available "$SOCKET_PORT" "Socket" || exit 1
        fi

        start_mcp_server

        if [ "$VALIDATE" = true ]; then
            check_mcp_connection || exit 1
            show_connection_info
        fi
    elif [ -v skip_mcp ]; then
        # FreeCAD only
        start_freecad
    else
        # Both MCP and FreeCAD
        if [ "$MODE" = "xmlrpc" ]; then
            check_port_available "$XMLRPC_PORT" "XML-RPC" || exit 1
        else
            check_port_available "$SOCKET_PORT" "Socket" || exit 1
        fi

        start_mcp_server
        start_freecad

        if [ "$VALIDATE" = true ]; then
            check_mcp_connection || exit 1
            show_connection_info
        fi
    fi

    # Keep running
    print_header "Services Running"
    info "Press Ctrl+C to stop all services"
    show_status

    # Wait for services
    while true; do
        sleep 5
    done
}

# Execute main
main "$@"
