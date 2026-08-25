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

## Quick Start: Using MCP to Create FreeCAD Parts

### 1. Start FreeCAD with MCP Bridge
```bash
cd freecad-mcp-server
./scripts/start-mcp-freecad.sh --mode xmlrpc
# Output: "MCP Bridge connected on XML-RPC: localhost:9875"
```

### 2. Use MCP Tools via Claude Code
Claude Code now has 150+ FreeCAD tools available:
- **Geometry creation:** `create_box`, `create_cylinder`, `create_sphere`, `create_cone`, `create_wedge`
- **Document ops:** `create_document`, `save_document`, `export_step`, `export_stl`
- **Object ops:** `delete_object`, `boolean_operation` (union, cut, intersect), `edit_object`
- **PartDesign:** `create_sketch`, `add_sketch_circle`, `pad_sketch`, `fillet_edges`, `create_pocket`
- **View:** `get_screenshot`, `set_view_angle`, `set_object_color`

### 3. Create a Part from Script
```bash
cd inverted-pendulum-project
# Option A: Interactive with Python
uv run python3 -c "
from freecad_integration_example import use_freecad_via_mcp
use_freecad_via_mcp()
"

# Option B: Run example part creator
uv run python3 03_Parts/simple_bracket.py
```

## Example: Simple Support Bracket via MCP

```python
# Create a simple part using MCP
from freecad_integration_example import use_freecad_via_mcp

def create_bracket():
    mcp = use_freecad_via_mcp()
    
    # Create document
    doc = mcp['create_document']()
    
    # Create base box (100x100x20mm)
    box = mcp['create_box'](length=100, width=100, height=20)
    
    # Create cylinder for hole (diameter 10mm)
    hole = mcp['create_cylinder'](radius=5, height=30)
    
    # Cut hole from box
    bracket = mcp['boolean_operation'](
        base_object=box,
        tool_object=hole,
        operation='cut'
    )
    
    # Fillet edges for smooth finish
    mcp['fillet_edges'](object=bracket, radius=2)
    
    # Export to STEP
    mcp['export_step'](
        object_name=bracket,
        filepath='03_Parts/support_bracket.step'
    )
    
    print("Bracket created: 03_Parts/support_bracket.step")

if __name__ == '__main__':
    create_bracket()
```

## Configuration

- `.mcp.json` — MCP server configuration (XML-RPC, localhost:9875)
- Each project has isolated Python 3.11 + uv environment
- FreeCAD AppImage path: `~/.local/bin/FreeCAD_1.1.3-Linux-x86_64-py311.AppImage`

## Repository

GitHub: [freecad-workspace](https://github.com/username/freecad-workspace)

## License

See individual project licenses.
