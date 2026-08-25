# FreeCAD MCP Server Scripts

Utility scripts for managing FreeCAD MCP Server startup, validation, and diagnostics.

## Scripts

### start-mcp-freecad.sh

Main launcher script for FreeCAD with Robust MCP Bridge.

**Features:**
- ✅ Interactive menu mode (default)
- ✅ Command-line argument mode
- ✅ Start MCP server + FreeCAD together
- ✅ Start MCP server only
- ✅ Start FreeCAD only
- ✅ XML-RPC & Socket modes
- ✅ Connection validation & diagnostics
- ✅ Port availability checking
- ✅ Colored status output
- ✅ Automatic logging
- ✅ Service status monitoring
- ✅ **Virtual Environment Management** (early activation, venv verification)
- ✅ **Process Verification** (ensures services start successfully)
- ✅ **Environment Logging** (logs all configuration for debugging)

**Usage:**

#### Interactive Menu (Recommended)
```bash
./scripts/start-mcp-freecad.sh
# or
./scripts/start-mcp-freecad.sh --interactive
```

**Menu Options:**
1. Start MCP Server + FreeCAD (XML-RPC mode)
2. Start MCP Server + FreeCAD (Socket mode)
3. Start MCP Server only (XML-RPC)
4. Start MCP Server only (Socket)
5. Start FreeCAD only
6. Check service status
7. Show connection info
8. View logs
9. Exit

#### Command-Line Arguments

**Start both services (XML-RPC):**
```bash
./scripts/start-mcp-freecad.sh --mode xmlrpc
```

**Start both services (Socket mode):**
```bash
./scripts/start-mcp-freecad.sh --mode socket
```

**Start MCP server only:**
```bash
./scripts/start-mcp-freecad.sh --mcp-only --mode xmlrpc
```

**Start FreeCAD only:**
```bash
./scripts/start-mcp-freecad.sh --freecad-only
```

**Custom FreeCAD AppImage path:**
```bash
./scripts/start-mcp-freecad.sh --appimage /path/to/FreeCAD.AppImage --mode xmlrpc
```

**Custom port (XMLRPC):**
```bash
./scripts/start-mcp-freecad.sh --port 9999 --mode xmlrpc
```

**Skip connection validation:**
```bash
./scripts/start-mcp-freecad.sh --no-validate --mode xmlrpc
```

**Show help:**
```bash
./scripts/start-mcp-freecad.sh --help
```

#### Full Command-Line Options

| Option | Value | Description |
|--------|-------|-------------|
| `--help`, `-h` | - | Show help message |
| `--mode` | `xmlrpc` \| `socket` | Connection mode (default: xmlrpc) |
| `--host` | HOST | Host to bind (default: localhost) |
| `--port` | PORT | Port number (XMLRPC: 9875, Socket: 9876) |
| `--freecad-only` | - | Start only FreeCAD |
| `--mcp-only` | - | Start only MCP server |
| `--skip-freecad` | - | Start MCP only (same as --mcp-only) |
| `--no-validate` | - | Skip connection validation |
| `--appimage` | PATH | Path to FreeCAD AppImage |
| `--interactive`, `-i` | - | Show interactive menu |
| `--version` | - | Show version info |

### What the Script Does

#### Dependency Checking
- ✓ Python 3 available
- ✓ FreeCAD AppImage exists at configured path
- ✓ Virtual environment present
- ✓ freecad-robust-mcp package installed

#### Port Validation
- ✓ Checks if XML-RPC port (9875) is available
- ✓ Checks if Socket port (9876) is available
- ✓ Provides warning if ports already in use

#### Service Startup
1. **MCP Server:**
   - Sets environment variables (FREECAD_MODE, ports, host)
   - Activates virtual environment
   - Starts freecad-mcp command
   - Logs output to `logs/mcp-server.log`

2. **FreeCAD:**
   - Sets Wayland display backend
   - Launches AppImage in background
   - Waits 15 seconds for initialization
   - Logs output to same log file

#### Connection Validation
- Waits up to 30 seconds for bridge to respond
- Polls connection status
- Reports endpoint URL
- Shows MCP client configuration template

#### Status Monitoring
- Checks if processes are still running
- Verifies ports are listening
- Shows process IDs
- Reports connection state

### Virtual Environment Management

The script now includes robust virtual environment handling:

**Early Activation:**
- Virtual environment is activated early in execution flow
- Provides clear feedback on activation status
- Ensures all subprocess commands run with proper Python context

**Process Verification:**
- Uses full venv paths for reliability: `$VENV_PATH/bin/freecad-mcp`
- Verifies process successfully started before reporting success
- Prevents silent failures from background process launch

**Environment Logging:**
- All environment variables logged at startup:
  - `FREECAD_MODE`, `FREECAD_SOCKET_HOST`
  - `FREECAD_XMLRPC_PORT`, `FREECAD_SOCKET_PORT`
  - `VIRTUAL_ENV` (venv path)
- Python version and executable path displayed
- Full command paths shown in logs for debugging

**Enhanced Error Messages:**
- Detailed feedback if venv activation fails
- Process startup failures show log tail
- Python version and path validation

**Startup Example Output:**
```
[i] Project: /home/user/freecad-workspace/freecad-mcp-server
[i] Virtual Environment: /home/user/freecad-workspace/freecad-mcp-server/.venv

✓ Virtual environment activated: /home/user/freecad-workspace/freecad-mcp-server/.venv
✓ Python: Python 3.11.14
[i] Python executable: /home/user/freecad-workspace/freecad-mcp-server/.venv/bin/python3
✓ freecad-robust-mcp installed

[i] Mode: xmlrpc
[i] Host: localhost
[i] Virtual Environment: /home/user/freecad-workspace/freecad-mcp-server/.venv

[2026-08-24 16:25:52] Environment Variables:
  FREECAD_MODE=xmlrpc
  FREECAD_SOCKET_HOST=localhost
  FREECAD_XMLRPC_PORT=9875
  FREECAD_SOCKET_PORT=9876
  VIRTUAL_ENV=/home/user/freecad-workspace/freecad-mcp-server/.venv

✓ MCP Server started (PID: 1234567)
[2026-08-24 16:25:52] Command: /home/user/freecad-workspace/freecad-mcp-server/.venv/bin/freecad-mcp
```

### Logs

All output logged to: `./logs/mcp-server.log`

**View logs:**
```bash
tail -f ./logs/mcp-server.log
```

**From interactive menu:** Select option 8

### FreeCAD AppImage Configuration

Default path: `~/.local/bin/FreeCAD_1.1.3-Linux-x86_64-py311.AppImage`

**Override with environment variable:**
```bash
export FREECAD_APPIMAGE=/path/to/FreeCAD.AppImage
./scripts/start-mcp-freecad.sh
```

**Or pass as argument:**
```bash
./scripts/start-mcp-freecad.sh --appimage /path/to/FreeCAD.AppImage
```

### Connection Information

Once running, the script displays MCP client configuration:

**XML-RPC Mode:**
```json
{
  "mcpServers": {
    "freecad": {
      "command": "freecad-mcp",
      "env": {
        "FREECAD_MODE": "xmlrpc",
        "FREECAD_SOCKET_HOST": "localhost",
        "FREECAD_XMLRPC_PORT": "9875"
      }
    }
  }
}
```

**Socket Mode:**
```json
{
  "mcpServers": {
    "freecad": {
      "command": "freecad-mcp",
      "env": {
        "FREECAD_MODE": "socket",
        "FREECAD_SOCKET_HOST": "localhost",
        "FREECAD_SOCKET_PORT": "9876"
      }
    }
  }
}
```

### Stopping Services

Press `Ctrl+C` to stop all running services:
- MCP server process
- FreeCAD process
- Script execution

### Troubleshooting

**Script not executable:**
```bash
chmod +x ./scripts/start-mcp-freecad.sh
```

**"FreeCAD AppImage not found":**
- Verify path: `ls -l ~/.local/bin/FreeCAD_*.AppImage`
- Set correct path via `--appimage` or `FREECAD_APPIMAGE` env var

**"Port already in use":**
- Change port: `--port 9999`
- Or kill existing process: `lsof -i :9875`

**"freecad-robust-mcp not installed":**
```bash
cd ..
uv sync
```

**Connection validation timeout:**
- Check if FreeCAD MCP Bridge is running inside FreeCAD
- Workbench → Start MCP Bridge button
- Check FreeCAD console for errors

**No ports listening:**
- Verify MCP server is running: `ps aux | grep freecad-mcp`
- Check logs: `tail -f logs/mcp-server.log`
- Restart: `killall freecad-mcp && ./scripts/start-mcp-freecad.sh`

### Examples

#### Full workflow

```bash
# Start with interactive menu
./scripts/start-mcp-freecad.sh

# Select option 1: "Start MCP Server + FreeCAD (XML-RPC mode)"

# Wait for validation message
# Then open MCP client (Claude Code, etc.)

# Select option 6 from menu to check status
# Or Ctrl+C to stop
```

#### Scripted workflow

```bash
# Start services
./scripts/start-mcp-freecad.sh --mode xmlrpc &

# Wait for startup
sleep 20

# Check status in another terminal
./scripts/start-mcp-freecad.sh --help  # Shows configuration

# Stop
pkill -f start-mcp-freecad.sh
```

#### Development workflow

```bash
# Terminal 1: Start MCP + FreeCAD
cd ~/freecad-workspace/freecad-mcp-server
./scripts/start-mcp-freecad.sh --mode xmlrpc

# Terminal 2: Use MCP client (Claude Code, etc.)
# Configure to connect to localhost:9875

# Monitor logs in Terminal 3
tail -f logs/mcp-server.log
```

### Environment Variables

| Variable | Default | Purpose |
|----------|---------|---------|
| `FREECAD_APPIMAGE` | `~/.local/bin/FreeCAD_1.1.3-Linux-x86_64-py311.AppImage` | Path to FreeCAD AppImage |
| `FREECAD_MODE` | Set by script | `xmlrpc` or `socket` |
| `FREECAD_SOCKET_HOST` | Set by script | Server host (localhost) |
| `FREECAD_XMLRPC_PORT` | Set by script | XML-RPC port (9875) |
| `FREECAD_SOCKET_PORT` | Set by script | Socket port (9876) |
| `VIRTUAL_ENV` | Set by script | Path to active virtual environment |

### More Information

- [Robust MCP Server Docs](https://spkane.github.io/freecad-addon-robust-mcp-server/)
- [MCP Protocol](https://modelcontextprotocol.io/)
- [FreeCAD](https://www.freecadweb.org/)
