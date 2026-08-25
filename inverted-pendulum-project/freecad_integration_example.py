#!/usr/bin/env python3
"""
Example: Using FreeCAD APIs from Inverted Pendulum Project

This module demonstrates how to integrate FreeCAD functionality into
the pendulum simulation project. There are two approaches:

1. MCP Client Mode (recommended)
   - Requires FreeCAD with MCP Bridge running
   - Communicates via XML-RPC (port 9875) or Socket (port 9876)
   - Works from any Python environment

2. Direct Bindings Mode
   - Requires running in FreeCAD's Python environment
   - Direct access to FreeCAD objects and APIs
   - More powerful but tightly coupled to FreeCAD
"""

import sys
from typing import Optional


# ============================================================================
# Approach 1: MCP Client Mode (Recommended)
# ============================================================================

def use_freecad_via_mcp():
    """
    Control FreeCAD via MCP when FreeCAD + MCP Bridge is running.

    Usage:
        1. Start FreeCAD with MCP Bridge:
           ./scripts/start-mcp-freecad.sh --mode xmlrpc

        2. Run this function:
           python3 -c "from freecad_integration_example import use_freecad_via_mcp; use_freecad_via_mcp()"
    """
    try:
        from freecad_robust_mcp import FreecadMCP

        # Connect to running FreeCAD MCP server
        mcp = FreecadMCP(
            mode="xmlrpc",
            host="localhost",
            port=9875
        )

        print("✓ Connected to FreeCAD via MCP")

        # Example: Create a document and add geometry
        mcp.create_document("PendulumStudy")
        mcp.create_box(length=10, width=5, height=2)
        mcp.export_step("pendulum_model.step")

        print("✓ Created geometry and exported to STEP")

    except ImportError:
        print("✗ freecad-robust-mcp not installed. Run: uv sync")
    except ConnectionError:
        print("✗ Cannot connect to FreeCAD MCP Bridge.")
        print("  Start FreeCAD with: ./scripts/start-mcp-freecad.sh --mode xmlrpc")


# ============================================================================
# Approach 2: Direct Python Bindings (Advanced)
# ============================================================================

def use_freecad_direct() -> Optional[object]:
    """
    Direct FreeCAD Python bindings (only in FreeCAD's Python environment).

    Usage:
        1. Start FreeCAD GUI with MCP Bridge workbench
        2. In FreeCAD's Python console, run:
           >>> exec(open('freecad_integration_example.py').read())
           >>> use_freecad_direct()

    Or run in FreeCAD's Python:
        freecad -c "
        import sys
        sys.path.insert(0, '/path/to/inverted-pendulum-project')
        from freecad_integration_example import use_freecad_direct
        use_freecad_direct()
        "
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
        print("  Run this script within FreeCAD's Python environment.")
        return None


# ============================================================================
# Integration with Simulation
# ============================================================================

def export_simulation_to_freecad(
    positions: list[float],
    filename: str = "pendulum_simulation.step"
):
    """
    Export simulation results as 3D model to FreeCAD.

    Args:
        positions: List of (x, y, z) tuples from pendulum trajectory
        filename: Output STEP file name

    Example:
        >>> from simulate import run_simulation
        >>> positions = run_simulation(duration=10)
        >>> export_simulation_to_freecad(positions)
    """
    try:
        from freecad_robust_mcp import FreecadMCP

        mcp = FreecadMCP(mode="xmlrpc", host="localhost", port=9875)
        mcp.create_document("SimulationVisualization")

        # Create a trail of spheres at each position
        for i, pos in enumerate(positions[::10]):  # Sample every 10th point
            x, y, z = pos
            mcp.create_sphere(
                radius=0.5,
                x=x, y=y, z=z
            )

        mcp.export_step(filename)
        print(f"✓ Exported simulation visualization: {filename}")

    except ConnectionError:
        print("✗ FreeCAD MCP server not running.")


# ============================================================================
# Configuration
# ============================================================================

# MCP connection settings (can override via environment)
MCP_CONFIG = {
    "mode": "xmlrpc",           # or "socket"
    "host": "localhost",
    "xmlrpc_port": 9875,        # XML-RPC port
    "socket_port": 9876,        # Socket port
}


# ============================================================================
# CLI Demo
# ============================================================================

if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "mcp":
        use_freecad_via_mcp()
    elif len(sys.argv) > 1 and sys.argv[1] == "direct":
        use_freecad_direct()
    else:
        print(__doc__)
        print("\nUsage:")
        print("  python3 freecad_integration_example.py mcp    # Use MCP mode (recommended)")
        print("  python3 freecad_integration_example.py direct # Use direct bindings (in FreeCAD)")
