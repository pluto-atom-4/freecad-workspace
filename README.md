# FreeCAD Workspace

Integrated workspace for FreeCAD development projects with MCP (Model Context Protocol) server and inverted pendulum simulation.

## Projects

### 1. FreeCAD MCP Server (`freecad-mcp-server/`)

Development environment for the [Robust MCP Server](https://github.com/spkane/freecad-addon-robust-mcp-server) — an MCP server enabling AI assistants (Claude, GPT, etc.) to interact with FreeCAD.

**Key Features:**
- 150+ MCP tools for CAD operations
- Multiple connection modes (XML-RPC, JSON-RPC socket, embedded)
- GUI & headless support
- PartDesign sketching & patterns
- Macro development & export

**Setup & Usage:**
```bash
cd freecad-mcp-server
uv run --help  # List available commands via uv
```

### 2. Inverted Pendulum Project (`inverted-pendulum-project/`)

Simulation and modeling of inverted pendulum systems with numerical computation and visualization.

**Dependencies:**
- numpy: numerical computing
- scipy: scientific algorithms
- matplotlib: data visualization

**Setup & Usage:**
```bash
cd inverted-pendulum-project
uv run python3 <script.py>
```

## Environment Setup

Each project has its own isolated Python 3.11 virtual environment managed by `uv`.

```bash
# Initialize project environments
cd freecad-mcp-server && uv sync
cd ../inverted-pendulum-project && uv sync

# Activate a project environment
cd freecad-mcp-server && . .venv/bin/activate

# Or run directly with uv
uv run <command>
```

## Repository

GitHub: [freecad-workspace](https://github.com/username/freecad-workspace)

## License

See individual project licenses.
