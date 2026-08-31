#!/usr/bin/env python3
"""
Example: Using FreeCAD APIs from Inverted Pendulum Project

FreeCAD is integrated headlessly, as a separate subprocess, never imported
into the same Python process as CadQuery/OCP/trimesh (the two bundle
different, incompatible OpenCASCADE builds).

This module demonstrates "direct bindings" mode:
    - Runs inside FreeCAD's own Python environment (via a headless FreeCAD
      binary, e.g. `freecadcmd`), invoked as a subprocess from the shell or
      from another script.
    - Direct access to FreeCAD objects and the full Python API (Part, Mesh,
      App, ...).
    - The FreeCAD binary is resolved from the FREECAD_BIN environment
      variable, defaulting to "freecadcmd" (relies on PATH) if unset.
"""

import os
import sys
from typing import Optional


# ============================================================================
# FreeCAD binary configuration
# ============================================================================

# Resolve the headless FreeCAD binary to use for subprocess invocations.
# Defaults to "freecadcmd" (relies on PATH). To pin a specific build:
#   export FREECAD_BIN=/home/pluto-atom-4/.local/opt/freecad-1.1.3/usr/bin/freecadcmd
FREECAD_BIN = os.environ.get("FREECAD_BIN", "freecadcmd")


# ============================================================================
# Direct Python Bindings (sanctioned pattern)
# ============================================================================

def use_freecad_direct() -> Optional[object]:
    """
    Direct FreeCAD Python bindings — only available when this code runs
    inside FreeCAD's own Python interpreter (i.e. invoked via a headless
    FreeCAD binary as a subprocess, not the mamba/uv Python environment).

    Usage:
        1. In FreeCAD's Python console, run:
           >>> exec(open('freecad_integration_example.py').read())
           >>> use_freecad_direct()

    Or run via a headless FreeCAD binary as a subprocess:
        $FREECAD_BIN -c "
        import sys
        sys.path.insert(0, '/path/to/inverted-pendulum-project')
        from freecad_integration_example import use_freecad_direct
        use_freecad_direct()
        "

    Or from the command line of this script itself:
        python3 freecad_integration_example.py direct
        (this re-execs itself under $FREECAD_BIN as a subprocess)
    """
    try:
        import FreeCAD
        import Part

        print(f"✓ FreeCAD {FreeCAD.Version()}")

        # Create a new document
        doc = FreeCAD.newDocument("PendulumDirect")

        # Create a simple box (pendulum mass)
        box = Part.makeBox(10, 5, 2)
        obj = doc.addObject("Part::Feature", "PendulumMass")
        obj.Shape = box

        # Create a pivot point (sphere)
        sphere = Part.makeSphere(2)
        pivot = doc.addObject("Part::Feature", "Pivot")
        pivot.Shape = sphere
        pivot.Placement.Base.z = 15  # Position above

        doc.save("pendulum_model.FCStd")
        print("✓ Created model in FreeCAD: pendulum_model.FCStd")

        return doc

    except ImportError:
        print("✗ FreeCAD Python bindings not available.")
        print("  This function must run inside FreeCAD's Python environment.")
        print(f"  Run it via: {FREECAD_BIN} -c \"exec(open('freecad_integration_example.py').read()); use_freecad_direct()\"")
        return None


def run_direct_via_subprocess() -> int:
    """
    Convenience helper: re-invoke this script inside FreeCAD's own Python
    environment via subprocess, using the FREECAD_BIN binary. This is what
    `python3 freecad_integration_example.py direct` does when run from the
    mamba/uv environment (which does not have the `FreeCAD` module).
    """
    import subprocess

    script_path = os.path.abspath(__file__)
    cmd = [
        FREECAD_BIN,
        "-c",
        f"exec(open('{script_path}').read()); use_freecad_direct()",
    ]
    print(f"Invoking FreeCAD directly via subprocess: {' '.join(cmd)}")
    try:
        result = subprocess.run(cmd, timeout=120)
        return result.returncode
    except FileNotFoundError:
        print(f"✗ FreeCAD binary not found: {FREECAD_BIN}")
        print("  Set FREECAD_BIN to a valid headless FreeCAD binary, e.g.:")
        print("    export FREECAD_BIN=/home/pluto-atom-4/.local/opt/freecad-1.1.3/usr/bin/freecadcmd")
        return 1


# ============================================================================
# Integration with Simulation
# ============================================================================

def export_simulation_to_freecad(
    positions: list[float],
    filename: str = "pendulum_simulation.step"
):
    """
    Export simulation results as a 3D model, using FreeCAD's direct Python
    bindings. Must run inside FreeCAD's own Python environment (see
    use_freecad_direct() for how to invoke it via subprocess).

    Args:
        positions: List of (x, y, z) tuples from pendulum trajectory
        filename: Output STEP file name

    Example:
        >>> from simulate import run_simulation
        >>> positions = run_simulation(duration=10)
        >>> export_simulation_to_freecad(positions)
    """
    try:
        import FreeCAD
        import Part

        doc = FreeCAD.newDocument("SimulationVisualization")

        # Create a trail of spheres at each sampled position
        for i, pos in enumerate(positions[::10]):  # Sample every 10th point
            x, y, z = pos
            sphere = Part.makeSphere(0.5)
            obj = doc.addObject("Part::Feature", f"TrailPoint{i}")
            obj.Shape = sphere
            obj.Placement.Base = FreeCAD.Vector(x, y, z)

        doc.recompute()
        Part.export(doc.Objects, filename)
        print(f"✓ Exported simulation visualization: {filename}")

    except ImportError:
        print("✗ FreeCAD Python bindings not available.")
        print(f"  Run this inside FreeCAD's Python environment (see {FREECAD_BIN}).")


# ============================================================================
# CLI Demo
# ============================================================================

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "direct":
        # Try running directly first (works if we're already inside FreeCAD's
        # Python environment); otherwise re-exec via subprocess under FREECAD_BIN.
        try:
            import FreeCAD  # noqa: F401
            use_freecad_direct()
        except ImportError:
            sys.exit(run_direct_via_subprocess())
    else:
        print(__doc__)
        print("\nUsage:")
        print("  python3 freecad_integration_example.py direct  # Direct FreeCAD bindings (via subprocess, FREECAD_BIN)")
