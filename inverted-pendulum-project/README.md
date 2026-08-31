# Inverted Pendulum Project

Simulation and numerical modeling of inverted pendulum dynamics with FreeCAD integration.

## Features

- **Numerical Simulation:** Compute pendulum dynamics using numpy/scipy
- **Visualization:** Plot results with matplotlib
- **Parametric CAD:** Generate brackets/parts with CadQuery, validate/repair meshes with trimesh
- **FreeCAD Integration:** Convert/export models via headless FreeCAD, invoked as a subprocess

## Setup

### Mamba environment (single env: `pendulum-tools`)

All numeric/CAD-authoring work (simulation, servo positioning, assembly linking,
CadQuery bracket generation, trimesh mesh processing) uses one mamba environment.
FreeCAD itself is never installed into or imported from this environment — it is
always invoked externally as a separate subprocess (see below), because FreeCAD
and CadQuery/OCP bundle different, incompatible OpenCASCADE builds.

```bash
# Create the environment (see mamba-envs.yaml for the full spec)
mamba create -n pendulum-tools -c conda-forge python=3.11 \
  cadquery trimesh numpy scipy matplotlib -y

# Activate
mamba activate pendulum-tools

# Verify
python -c "import cadquery, trimesh, numpy, scipy, matplotlib; print('OK')"
```

**Reproducible install:** `mamba-envs.yaml` is a recipe (unpinned minimum versions). For an
exact, reproducible environment matching what this project was tested against, use the
pinned lock file instead:

```bash
mamba env create -n pendulum-tools -f mamba-envs.lock.yml
```

Regenerate it after any env change with:
```bash
mamba env export --no-builds -n pendulum-tools > mamba-envs.lock.yml
```

## Usage

### Run Simulation

```bash
mamba run -n pendulum-tools python3 simulate.py
```

### FreeCAD Integration (headless subprocess)

FreeCAD is invoked headlessly via `freecadcmd`, in a separate process from any
CadQuery/trimesh code — never imported into the same Python process.

**Binary selection:** set the `FREECAD_BIN` environment variable to point at a
specific FreeCAD build. Defaults to `freecadcmd` (relies on PATH) if unset.

```bash
# Default: use freecadcmd from PATH
python3 freecad_integration_example.py direct

# Or pin to a specific FreeCAD build (e.g. the 1.1.3 AppImage extraction,
# available via the shorter ~/.local/bin/freecadcmd1.1 symlink)
export FREECAD_BIN=~/.local/bin/freecadcmd1.1
"$FREECAD_BIN" -c "
exec(open('freecad_integration_example.py').read())
use_freecad_direct()
"
```

**Benefits of the direct/subprocess pattern:**
- No network layer, no bridge process to keep running
- Full FreeCAD Python API available (Part, Mesh, App, …)
- FreeCAD process boundary keeps its OpenCASCADE build isolated from OCP/CadQuery

### Phase 1: Convert servo STL to STEP

```bash
# Uses FREECAD_BIN if set, otherwise falls back to "freecadcmd" on PATH
FREECAD_BIN=~/.local/bin/freecadcmd1.1 \
  mamba run -n pendulum-tools python3 03_Parts/Generators/01_convert_servo_stl_to_step.py
```

### Phase 6: Parametric CAD generation & mesh tooling (CadQuery/trimesh)

Runs directly in the `pendulum-tools` mamba env's own Python (never mixed with FreeCAD):

```bash
mamba run -n pendulum-tools python 03_Parts/Generators/06_cadquery_parametric_brackets.py --help
mamba run -n pendulum-tools python 03_Parts/Generators/test_06_phase6_tooling.py
```

See `03_Parts/Generators/README.md` and `03_Parts/Generators/README_PHASE6.md` for details.

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
- `cadquery` — parametric CAD generation (Phase 6)
- `trimesh` — mesh processing and repair (Phase 6)
- FreeCAD (external, not a Python dependency) — invoked headlessly via subprocess (`FREECAD_BIN`)

## Project Structure

```
inverted-pendulum-project/
├── mamba-envs.yaml                     # Single pendulum-tools mamba env spec (recipe)
├── mamba-envs.lock.yml                 # Pinned, reproducible env export
├── README.md                           # This file
├── freecad_integration_example.py      # FreeCAD integration example (direct/subprocess)
├── simulate.py                         # Pendulum simulation
├── 01_Documentation/
│   └── MCP_TOOLS_REFERENCE.md         # Deprecated MCP tool catalog (see note in file)
├── 02_Design_Inputs/                  # Design specifications & parameters
├── 03_Parts/                          # FreeCAD part files (.FCStd, .step)
│   └── Generators/                    # Phase 1-6 generator scripts
├── 04_Assemblies/                     # Assembly definitions
├── 05_Drafts_Context/                 # Preliminary designs & concepts
└── 06_Exports/                        # Generated exports (STL, STEP, etc.)
```

## Architecture note: FreeCAD vs CadQuery/OCP process boundary

FreeCAD and CadQuery/OCP must never be imported in the same Python process:
FreeCAD bundles its own OpenCASCADE build, and OCP (used by CadQuery) bundles a
different one — mixing them risks ABI/symbol conflicts. This project keeps them
as separate subprocess invocations:

- FreeCAD-only scripts run via `freecadcmd` as a subprocess (`FREECAD_BIN`).
- CadQuery/trimesh scripts run directly in the `pendulum-tools` mamba env's own Python.

## References

- [FreeCAD](https://www.freecadweb.org/)
- [CadQuery](https://cadquery.readthedocs.io/)
- [Trimesh](https://trimesh.org/)
- [NumPy/SciPy Documentation](https://scipy.org/)

## See Also

- `../CLAUDE.md` — Complete development guide
