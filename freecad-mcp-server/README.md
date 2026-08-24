# FreeCAD MCP Server

Development environment for the [Robust MCP Server](https://github.com/spkane/freecad-addon-robust-mcp-server) — enabling AI assistants to interact with FreeCAD via MCP protocol.

## Quick Start

### 1. Install Dependencies

```bash
uv sync
```

### 2. Start FreeCAD with MCP Bridge

```bash
QT_QPA_PLATFORM=wayland ~/.local/bin/FreeCAD_1.1.3-Linux-x86_64-py311.AppImage
```

**Environment Variable Breakdown:**
- `QT_QPA_PLATFORM=wayland` — Forces Wayland display backend (required for Linux Wayland sessions)
- `FreeCAD_1.1.3-Linux-x86_64-py311.AppImage` — Python 3.11 compatible AppImage for 64-bit Linux

**Inside FreeCAD:**
1. Switch to "Robust MCP Bridge" workbench
2. Click **Start MCP Bridge** button (toolbar)
3. Check console for: `MCP Bridge started! XML-RPC: localhost:9875`

### 3. Configure MCP Client

Create `.mcp.json` in this directory:

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

Or configure globally in `~/.claude/claude_desktop_config.json`.

### 4. Use MCP Tools

Once bridge is running and MCP client configured, 150+ tools available:
- Create geometry (box, cylinder, sphere, cone, etc.)
- PartDesign sketching & patterns
- Export (STEP, STL, 3MF, OBJ)
- Macro management
- Document operations

## Development Workflow

```bash
# Activate environment
source .venv/bin/activate

# Or run directly with uv
uv run python3 <script.py>

# Sync after pyproject.toml changes
uv sync
```

## References

- [Robust MCP Server Docs](https://spkane.github.io/freecad-addon-robust-mcp-server/)
- [MCP Protocol](https://modelcontextprotocol.io/)
- [FreeCAD](https://www.freecadweb.org/)

See `../CLAUDE.md` for complete development guide.
