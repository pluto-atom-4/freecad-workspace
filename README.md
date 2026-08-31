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
mamba env create -n freecad-mcp -f mamba-envs.yaml   # first time only
mamba activate freecad-mcp
./scripts/start-mcp-freecad.sh --help
```

### 2. Inverted Pendulum Project (`inverted-pendulum-project/`)

Simulation and modeling of inverted pendulum systems with numerical computation and visualization.

**Dependencies:**
- numpy: numerical computing
- scipy: scientific algorithms
- matplotlib: data visualization
- cadquery: parametric CAD generation (Phase 6)
- trimesh: mesh processing and repair (Phase 6)

**Setup & Usage:**
```bash
cd inverted-pendulum-project
mamba activate pendulum-tools
python3 <script.py>
```

## Environment Setup

Each project has its own isolated mamba/conda environment — no `uv`, no `pyproject.toml`,
no `.venv` anywhere in this workspace.

```bash
# Initialize project environments (first time only)
cd freecad-mcp-server && mamba env create -n freecad-mcp -f mamba-envs.yaml
cd ../inverted-pendulum-project && mamba env create -n pendulum-tools -f mamba-envs.yaml

# Activate a project environment
mamba activate freecad-mcp        # or: mamba activate pendulum-tools

# Or run a single command without activating
mamba run -n freecad-mcp <command>
```

Each project also ships a pinned `mamba-envs.lock.yml` for a reproducible install
(`mamba env create -n <name> -f mamba-envs.lock.yml`) alongside the unpinned recipe
in `mamba-envs.yaml`.

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

**Deprecated/legacy** — `simple_bracket.py` talks to the MCP Bridge over raw
`xmlrpc.client` and requires FreeCAD running with the bridge started (steps 1-2 above).
For new bracket generation prefer Phase 6's `06_cadquery_parametric_brackets.py`
(standalone, no FreeCAD/bridge required — see below).

```bash
cd inverted-pendulum-project
mamba activate pendulum-tools
python3 03_Parts/Generators/simple_bracket.py
```

## Phase 6: Parametric CAD & Mesh Tooling (Standalone)

Comprehensive tooling for **headless** bracket generation, mesh validation/repair, and multi-mesh assembly composition. No FreeCAD runtime required.

**Setup:**
```bash
mamba activate pendulum-tools

# Verify imports
python3 -c "import cadquery; import trimesh; print('Phase 6 ready')"
```

**3 Independent Tools:**

### Tool 1: CadQuery Parametric Bracket Generator
Generate support brackets (simple, L, corner types) with customizable dimensions:
```bash
cd inverted-pendulum-project

# Generate single bracket
python3 03_Parts/Generators/06_cadquery_parametric_brackets.py \
  --type simple \
  --length 100 --width 80 --thickness 10 \
  --hole-diameter 8 --fillet-radius 3

# Batch generation from config
python3 03_Parts/Generators/06_cadquery_parametric_brackets.py \
  --config 03_Parts/Generators/bracket_configs.json

# Output: STEP + STL + JSON metadata
```

### Tool 2: Trimesh Mesh Validator
Validate and repair STL meshes for 3D printing:
```bash
# Validation only
python3 03_Parts/Generators/06_trimesh_mesh_validator.py \
  --input servo.stl

# Auto-repair
python3 03_Parts/Generators/06_trimesh_mesh_validator.py \
  --input servo.stl \
  --auto-repair \
  --output servo_repaired.stl
```

### Tool 3: Trimesh Mesh Merger
Combine multiple meshes with spatial transforms into assembly:
```bash
python3 03_Parts/Generators/06_trimesh_merge_for_stl.py \
  --config 03_Parts/Generators/merge_config_example.json

# Output: Merged STL + composition metadata
```

**Full Documentation:** See `inverted-pendulum-project/03_Parts/Generators/README_PHASE6.md`

**Key Features:**
- Standalone execution (no FreeCAD, MCP, or Python embedded)
- Parametric design with CLI arguments or JSON configs
- Mesh validation, repair, and composition tracking
- STEP/STL export with metadata JSON reports
- Batch processing capability

## Configuration

- `.mcp.json` — MCP server configuration (XML-RPC, localhost:9875)
- Each project has its own isolated Python 3.11 mamba/conda environment (no `uv`,
  no `pyproject.toml`, no `.venv` anywhere in this workspace)
- FreeCAD AppImage path: `~/.local/bin/FreeCAD_1.1.3-Linux-x86_64-py311.AppImage`
  (headless CLI also available via the `~/.local/bin/freecadcmd1.1` symlink)

## Repository

GitHub: [freecad-workspace](https://github.com/pluto-atom-4/freecad-workspace)

## License

See individual project licenses.
