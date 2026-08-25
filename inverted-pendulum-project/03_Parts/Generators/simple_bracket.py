#!/usr/bin/env python3
"""
Support Bracket Generator - FreeCAD MCP Integration Example

Creates a parametric support bracket via Model Context Protocol:
- 100×100×20mm base plate
- Ø10mm mounting hole (through)
- 2mm fillet on all edges for smooth finish
- Exports to FCStd and STEP formats

PREREQUISITES:
    FreeCAD MCP Bridge must be running:
        cd ../freecad-mcp-server
        ./scripts/start-mcp-freecad.sh --mode xmlrpc

USAGE:
    uv run python3 simple_bracket.py

OUTPUT:
    - 03_Parts/Mechanical/support_bracket.FCStd (FreeCAD native)
    - 03_Parts/Mechanical/support_bracket.step (3D model)

REFERENCES:
    - FreeCAD MCP Server: https://github.com/spkane/freecad-addon-robust-mcp-server
    - Model Context Protocol: https://modelcontextprotocol.io/
"""

import sys
import os
from pathlib import Path
from typing import Optional
import xmlrpc.client
import logging

# ============================================================================
# SETUP: Logging
# ============================================================================

logging.basicConfig(
    level=logging.INFO,
    format='%(levelname)s: %(message)s'
)
logger = logging.getLogger(__name__)

# ============================================================================
# CONSTANTS
# ============================================================================

# MCP Bridge connection
MCP_HOST = "localhost"
MCP_PORT = 9875
MCP_URL = f"http://{MCP_HOST}:{MCP_PORT}"

# Bracket dimensions (mm)
PLATE_LENGTH = 100.0
PLATE_WIDTH = 100.0
PLATE_HEIGHT = 20.0

# Hole dimensions
HOLE_RADIUS = 5.0  # Ø10mm = radius 5mm
HOLE_HEIGHT = 30.0

# Fillet radius
FILLET_RADIUS = 2.0

# Part naming
DOCUMENT_NAME = "SupportBracket"
BASE_PLATE_NAME = "BasePlate"
HOLE_NAME = "MountingHole"
BRACKET_NAME = "Bracket"
BRACKET_FINAL_NAME = "BracketFilleted"

# Output paths
SCRIPT_DIR = Path(__file__).parent
OUTPUT_DIR = SCRIPT_DIR.parent / "Mechanical"
FCSTD_FILE = OUTPUT_DIR / "support_bracket.FCStd"
STEP_FILE = OUTPUT_DIR / "support_bracket.step"


# ============================================================================
# MCP OPERATIONS
# ============================================================================

def verify_mcp_connection(mcp: xmlrpc.client.ServerProxy) -> bool:
    """Verify MCP bridge connectivity.

    Args:
        mcp: XML-RPC server proxy

    Returns:
        bool: True if connection successful

    Raises:
        Exception: Connection or method call errors
    """
    try:
        version = mcp.get_freecad_version()
        logger.info(f"✓ Connected to FreeCAD via MCP: {version}")
        return True
    except Exception as e:
        logger.error(f"Failed to verify MCP connection: {e}")
        raise


def create_bracket() -> bool:
    """Create support bracket using FreeCAD MCP tools.

    Returns:
        bool: True if successful, False otherwise
    """
    try:
        # Ensure output directory exists
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

        # Connect to MCP bridge
        logger.info("Connecting to FreeCAD MCP bridge...")
        mcp = xmlrpc.client.ServerProxy(MCP_URL, allow_none=True)
        verify_mcp_connection(mcp)

        # Create document
        logger.info(f"Creating FreeCAD document: {DOCUMENT_NAME}")
        doc_name = mcp.create_document(DOCUMENT_NAME)

        # Create base plate (100 × 100 × 20 mm)
        logger.info(f"Creating base plate ({PLATE_LENGTH}×{PLATE_WIDTH}×{PLATE_HEIGHT} mm)...")
        base_plate = mcp.create_box(
            PLATE_LENGTH, PLATE_WIDTH, PLATE_HEIGHT,
            BASE_PLATE_NAME
        )

        # Create mounting hole cylinder (Ø10mm, height 30mm)
        logger.info(f"Creating hole (Ø{HOLE_RADIUS*2} mm, height {HOLE_HEIGHT} mm)...")
        hole = mcp.create_cylinder(HOLE_RADIUS, HOLE_HEIGHT, HOLE_NAME)

        # Boolean cut: hole from base plate
        logger.info("Performing boolean cut operation...")
        bracket = mcp.boolean_operation(base_plate, hole, 'cut', BRACKET_NAME)

        # Fillet edges (2mm radius)
        logger.info(f"Filleting edges ({FILLET_RADIUS} mm radius)...")
        bracket_filleted = mcp.fillet_edges(bracket, FILLET_RADIUS, BRACKET_FINAL_NAME)

        # Export to STEP format
        logger.info(f"Exporting to STEP: {STEP_FILE}")
        step_result = mcp.export_step(bracket_filleted, str(STEP_FILE))
        logger.info(f"  Result: {step_result}")

        # Save FreeCAD document
        logger.info(f"Saving FreeCAD document: {FCSTD_FILE}")
        fcstd_result = mcp.save_document(str(FCSTD_FILE))
        logger.info(f"  Result: {fcstd_result}")

        # Print summary
        print("\n" + "=" * 60)
        print("✓ Support bracket created successfully")
        print("=" * 60)
        print(f"\nBracket specifications:")
        print(f"  Base plate: {PLATE_LENGTH}×{PLATE_WIDTH}×{PLATE_HEIGHT} mm")
        print(f"  Hole: Ø{HOLE_RADIUS*2} mm (height {HOLE_HEIGHT} mm)")
        print(f"  Fillet: {FILLET_RADIUS} mm radius")
        print(f"\nGenerated files:")
        print(f"  • {FCSTD_FILE}")
        print(f"  • {STEP_FILE}")
        print("\nNext steps:")
        print("  1. Open support_bracket.FCStd in FreeCAD")
        print("  2. Use support_bracket.step for CAM or 3D printing")

        return True

    except Exception as e:
        logger.error(f"Failed to create bracket: {e}")
        logger.error("\nTroubleshooting:")
        logger.error("  1. Verify FreeCAD MCP bridge is running:")
        logger.error("     cd ../freecad-mcp-server")
        logger.error("     ./scripts/start-mcp-freecad.sh --mode xmlrpc")
        logger.error("  2. Confirm .mcp.json configuration in project root")
        logger.error("  3. Check FREECAD_MODE=xmlrpc environment variable")
        import traceback
        traceback.print_exc()
        return False


if __name__ == '__main__':
    success = create_bracket()
    sys.exit(0 if success else 1)
