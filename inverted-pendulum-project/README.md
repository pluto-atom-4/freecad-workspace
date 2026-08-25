# Inverted Pendulum Project

Simulation and numerical modeling of inverted pendulum dynamics with FreeCAD integration.

## Features

- **Numerical Simulation:** Compute pendulum dynamics using numpy/scipy
- **Visualization:** Plot results with matplotlib
- **FreeCAD Integration:** Export models and visualizations to FreeCAD

## Setup

```bash
uv sync
```

## Usage

### Run Simulation

```bash
uv run python3 simulate.py
```

### FreeCAD Integration

Two approaches for using FreeCAD APIs:

#### 1. MCP Mode (Recommended)

Connect to FreeCAD via Model Context Protocol when MCP Bridge is running.

**Start FreeCAD with MCP Bridge:**
```bash
cd ../freecad-mcp-server
./scripts/start-mcp-freecad.sh --mode xmlrpc
```

**Use from Python:**
```python
from freecad_integration_example import use_freecad_via_mcp
use_freecad_via_mcp()
```

**Command line:**
```bash
uv run python3 freecad_integration_example.py mcp
```

**Benefits:**
- Works from any Python environment
- No tight coupling to FreeCAD
- Secure network communication

#### 2. Direct Bindings (Advanced)

Access FreeCAD Python API directly when running in FreeCAD's Python environment.

**In FreeCAD's Python console:**
```python
import sys
sys.path.insert(0, '/path/to/inverted-pendulum-project')
from freecad_integration_example import use_freecad_direct
use_freecad_direct()
```

**Or via command line (FreeCAD must be installed):**
```bash
freecad -c "
exec(open('freecad_integration_example.py').read())
use_freecad_direct()
"
```

**Benefits:**
- Direct access to FreeCAD objects
- No network communication overhead
- Full FreeCAD Python API available

### Create Parts with MCP Tools

Generate FreeCAD parts using 150+ available MCP tools.

**Simple Example:**
```bash
# Start MCP bridge first (in another terminal)
cd ../freecad-mcp-server
./scripts/start-mcp-freecad.sh --mode xmlrpc

# Then run the bracket creator
cd ../inverted-pendulum-project
uv run python3 03_Parts/simple_bracket.py
```

**Available MCP Tools:**
- Primitives: `create_box`, `create_cylinder`, `create_sphere`, `create_cone`, `create_wedge`
- Operations: `boolean_operation` (union/cut/intersect), `fillet_edges`, `create_sketch`, `pad_sketch`
- Export: `export_step`, `export_stl`, `export_iges`, `export_3mf`
- View: `get_screenshot`, `set_view_angle`, `set_object_color`

**Full reference:** `01_Documentation/MCP_TOOLS_REFERENCE.md`

### Export Simulation Results to FreeCAD

```python
from freecad_integration_example import export_simulation_to_freecad
import numpy as np

# Run simulation
# positions = run_simulation(duration=10)

# Visualize in FreeCAD
# export_simulation_to_freecad(positions, "pendulum_trajectory.step")
```

## Dependencies

- `numpy` — numerical computing
- `scipy` — scientific algorithms
- `matplotlib` — data visualization
- `freecad-robust-mcp` — FreeCAD integration via MCP

## Project Structure

```
inverted-pendulum-project/
├── pyproject.toml                      # Project config with dependencies
├── README.md                           # This file
├── freecad_integration_example.py      # FreeCAD integration examples
├── simulate.py                         # Pendulum simulation
├── .venv/                              # Virtual environment
├── uv.lock                             # Dependency lock file
├── 01_Documentation/
│   └── MCP_TOOLS_REFERENCE.md         # 150+ FreeCAD MCP tools guide
├── 02_Design_Inputs/                  # Design specifications & parameters
├── 03_Parts/                          # FreeCAD part files (.FCStd, .step)
│   └── simple_bracket.py              # Example: Create support bracket via MCP
├── 04_Assemblies/                     # Assembly definitions
├── 05_Drafts_Context/                 # Preliminary designs & concepts
└── 06_Exports/                        # Generated exports (STL, STEP, etc.)
```

## References

- [FreeCAD](https://www.freecadweb.org/)
- [Robust MCP Server](https://spkane.github.io/freecad-addon-robust-mcp-server/)
- [MCP Protocol](https://modelcontextprotocol.io/)
- [NumPy/SciPy Documentation](https://scipy.org/)

## See Also

- `../freecad-mcp-server/` — FreeCAD MCP Bridge setup
- `../CLAUDE.md` — Complete development guide
