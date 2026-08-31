# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Workspace Structure

FreeCAD workspace with two independent projects, Python 3.11:

```
freecad-workspace/
├── freecad-mcp-server/        # FreeCAD MCP Server development (uv package manager)
│   ├── .venv/                 # Isolated Python environment
│   └── pyproject.toml         # uv project config
├── inverted-pendulum-project/ # Pendulum simulation & modeling (mamba/conda only)
│   ├── mamba-envs.yaml        # pendulum-tools env spec (recipe)
│   └── mamba-envs.lock.yml    # pendulum-tools env, pinned/reproducible
├── .gitignore
├── README.md
└── CLAUDE.md
```

**Note:** the two projects use different tooling. `freecad-mcp-server` uses `uv` +
`pyproject.toml`/`.venv`. `inverted-pendulum-project` has no `pyproject.toml`, `uv.lock`,
or `.venv` — it's managed entirely by a single mamba/conda environment (`pendulum-tools`),
with FreeCAD invoked externally as a subprocess (never installed into that environment).
Do not assume `uv sync`/`uv run` work in `inverted-pendulum-project` — they don't.

## Projects Overview

### FreeCAD MCP Server

**Purpose:** Integration bridge for AI assistants (Claude, GPT) to interact with FreeCAD via MCP protocol.

**Reference:** [Robust MCP Server Documentation](https://github.com/spkane/freecad-addon-robust-mcp-server)

**Key Architecture:**
- **Connection Modes:** XML-RPC (port 9875, recommended), JSON-RPC socket (port 9876), embedded (Linux only)
- **Tool Categories:** 150+ tools across execution, document management, object creation, PartDesign, sketching, view control, export/import, macro management
- **Plugin Structure:** Workbench-based plugin that starts the MCP bridge inside FreeCAD
- **Communication:** XML-RPC or socket protocol for bridging FreeCAD and external MCP clients

**Main Dependencies:** `freecad-robust-mcp` (PyPI package)

**Development Workflow:**
1. Start FreeCAD with MCP bridge running (via workbench or `just` commands from source)
2. Configure MCP client (.mcp.json or ~/.claude/claude_desktop_config.json)
3. Use 150+ available tools to manipulate FreeCAD documents, create geometries, manage macros

### Inverted Pendulum Project

**Purpose:** Simulation and numerical modeling of inverted pendulum dynamics with FreeCAD integration.

**Main Dependencies:** numpy, scipy, matplotlib, freecad-robust-mcp

**Key Features:**
- Numerical simulation of pendulum dynamics using scipy
- Data visualization with matplotlib
- Export simulation results to FreeCAD models (STEP format)
- Two FreeCAD integration modes: MCP (recommended) or direct Python bindings

**FreeCAD Integration Modes:**
1. **MCP Mode (Recommended)** — Connect to FreeCAD via Model Context Protocol
   - Works from any Python environment
   - Requires FreeCAD with MCP Bridge running (`./freecad-mcp-server/scripts/start-mcp-freecad.sh`)
   - Uses XML-RPC (port 9875) or Socket (port 9876)
   - Secure network communication, loose coupling

2. **Direct Bindings Mode (Advanced)** — Access FreeCAD Python API directly
   - Only works in FreeCAD's Python environment
   - Direct access to FreeCAD objects
   - No network overhead, tight coupling to FreeCAD

**Typical Workflow:**
- Define system dynamics using numpy/scipy
- Compute solutions numerically
- Visualize results with matplotlib
- (Optional) Export to FreeCAD for 3D model visualization

**See Also:**
- `freecad_integration_example.py` — Integration patterns and code examples
- `../freecad-mcp-server/scripts/start-mcp-freecad.sh` — Start FreeCAD with MCP Bridge

## Common Development Commands

### Environment Management

**freecad-mcp-server** (uv):
```bash
cd freecad-mcp-server/ && source .venv/bin/activate
# Or use uv directly (no activation needed)
cd freecad-mcp-server/ && uv run python3 <script.py>
# Sync dependencies (after pyproject.toml changes)
cd freecad-mcp-server/ && uv sync
```

**inverted-pendulum-project** (mamba/conda):
```bash
mamba activate pendulum-tools
cd inverted-pendulum-project/ && python3 <script.py>
```

### FreeCAD MCP Server

**From Source Development** (if working with freecad-addon-robust-mcp-server repo directly):

```bash
# Setup from source (requires mise/just)
git clone https://github.com/spkane/freecad-addon-robust-mcp-server.git
cd freecad-addon-robust-mcp-server
mise trust && mise install
just setup

# Start FreeCAD with MCP bridge
just freecad::run-gui        # GUI mode
just freecad::run-headless   # Headless mode

# Run tests
just testing::unit
just testing::cov
just testing::integration

# Code quality
just quality::lint
just quality::typecheck
just quality::format
just quality::check          # All pre-commit hooks
```

**Using PyPI Package:**

```bash
cd freecad-mcp-server
# Freecad-robust-mcp is already installed via uv sync
# Configure in MCP client settings to use the package

# Test connection
uv run python3 -c "import freecad_robust_mcp; print(freecad_robust_mcp.__version__)"
```

### Inverted Pendulum Project

No `uv`/`pyproject.toml` — uses the `pendulum-tools` mamba environment instead. See
`inverted-pendulum-project/README.md` for the full setup.

```bash
cd inverted-pendulum-project

# Activate the environment (create it first if needed — see mamba-envs.yaml)
mamba activate pendulum-tools

# Run simulation
python3 simulate.py

# FreeCAD integration: headless subprocess only (no MCP for this project)
export FREECAD_BIN=~/.local/bin/freecadcmd1.1   # or rely on "freecadcmd" from PATH
python3 freecad_integration_example.py direct

# Run tests
python3 -m pytest

# Open Python REPL
python3
```

## Available MCP Tools (FreeCAD)

**Major Categories (150+ tools total):**

| Category | Count | Examples |
|----------|-------|----------|
| Execution & Debugging | 5 | execute_python, get_freecad_version, get_connection_status |
| Document Management | 7 | create_document, open_document, save_document |
| Object Creation | 8 | create_box, create_cylinder, create_sphere |
| Object Management | 12 | edit_object, delete_object, boolean_operation |
| PartDesign Sketching | 14 | create_sketch, add_sketch_circle, pad_sketch, pocket_sketch |
| PartDesign Patterns | 5 | linear_pattern, polar_pattern, fillet_edges |
| View & Display | 11 | get_screenshot, set_view_angle, set_object_color |
| Undo/Redo | 3 | undo, redo, get_undo_redo_status |
| Export/Import | 7 | export_step, export_stl, import_step |
| Macro Management | 6 | create_macro, run_macro, delete_macro |
| Parts Library | 2 | list_parts_library, insert_part_from_library |

## MCP Client Configuration

### Claude Code / Claude Desktop

Create `.mcp.json` in project root or configure in `~/.claude/claude_desktop_config.json`:

**Using PyPI package:**
```json
{
  "mcpServers": {
    "freecad": {
      "command": "freecad-mcp",
      "env": {
        "FREECAD_MODE": "xmlrpc"
      }
    }
  }
}
```

**Using from source with uv:**
```json
{
  "mcpServers": {
    "freecad": {
      "command": "uv",
      "args": ["run", "--project", "/path/to/freecad-addon-robust-mcp-server", "freecad-mcp"],
      "env": {
        "FREECAD_MODE": "xmlrpc",
        "FREECAD_SOCKET_HOST": "localhost",
        "FREECAD_XMLRPC_PORT": "9875"
      }
    }
  }
}
```

## Development Patterns

### FreeCAD MCP Integration

1. **Start FreeCAD with bridge** → must run before MCP client connects
2. **Configure MCP client** → point to freecad-mcp command or uv wrapper
3. **Use MCP tools** → 150+ tools available through Claude
4. **Export models** → STEP, STL, 3MF, OBJ, IGES formats

### Typical Workflow

- Create geometry via MCP tools or through FreeCAD UI
- Use `export_step` or `export_stl` to export results
- Version control `.FCStd` files (FreeCAD native format) in git

### Testing & Validation

- Unit tests don't require FreeCAD running
- Integration tests require FreeCAD with MCP bridge
- Use `get_connection_status` to verify bridge connectivity

## Git & GitHub

- Repository: freecad-workspace on GitHub
- Each project is self-contained
- `.gitignore` covers Python environments, build artifacts, FreeCAD files

## Key Files

| File | Purpose |
|------|---------|
| `freecad-mcp-server/pyproject.toml` | uv project config (that project only) |
| `freecad-mcp-server/.venv/` | Python virtual environment (do not commit) |
| `inverted-pendulum-project/mamba-envs.yaml` | pendulum-tools env recipe (unpinned) |
| `inverted-pendulum-project/mamba-envs.lock.yml` | pendulum-tools env, pinned/reproducible |
| `.gitignore` | Excludes venv, __pycache__, .FCStd files |
| `.mcp.json` | MCP server configuration (project-level) |

## Troubleshooting

**MCP client can't connect to FreeCAD:**
- Verify FreeCAD has MCP bridge running (check console output for "MCP Bridge started!")
- Check port availability (9875 for XML-RPC, 9876 for socket)
- Ensure FREECAD_SOCKET_HOST matches (localhost vs remote)

**FreeCAD crashes in embedded mode:**
- Don't use `FREECAD_MODE=embedded` on macOS/Windows — use `xmlrpc` or `socket` instead

**Dependencies won't sync (freecad-mcp-server only):**
- Verify Python 3.11 is available: `python3 --version`
- Clear cache: `rm -rf .venv` then `uv sync`

**pendulum-tools env broken (inverted-pendulum-project):**
- Recreate from the pinned lock: `mamba env remove -n pendulum-tools -y && mamba env create -n pendulum-tools -f inverted-pendulum-project/mamba-envs.lock.yml`

## References

- [FreeCAD](https://www.freecadweb.org/)
- [Robust MCP Server Docs](https://spkane.github.io/freecad-addon-robust-mcp-server/)
- [MCP Protocol](https://modelcontextprotocol.io/)
- [uv Package Manager](https://docs.astral.sh/uv/)
