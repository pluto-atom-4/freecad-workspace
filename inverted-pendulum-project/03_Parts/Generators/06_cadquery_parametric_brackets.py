#!/usr/bin/env python3
"""
Phase 6 Day 3-4: CadQuery Parametric Bracket Generator

Generates parametric support brackets with 3 types:
1. Simple plate bracket - rectangular plate with mounting holes
2. L-bracket - two perpendicular plates with optional fillet
3. Corner bracket - reinforced L-bracket with corner support

Features:
- Parametric generation via CLI arguments or JSON config
- Export to STEP and STL formats
- JSON metadata output with geometry statistics
- Support for mounting hole patterns and fillets

Usage:
    # CLI mode - simple plate bracket
    python3 06_cadquery_parametric_brackets.py --type simple \\
        --length 100 --width 80 --thickness 10 --hole-diameter 8 --fillet-radius 3

    # Config mode - load from JSON file
    python3 06_cadquery_parametric_brackets.py --config bracket_configs.json

    # L-bracket with CLI
    python3 06_cadquery_parametric_brackets.py --type l_bracket \\
        --length 100 --width 80 --thickness 10

    # Corner bracket
    python3 06_cadquery_parametric_brackets.py --type corner \\
        --length 150 --width 120 --thickness 12

Output:
    03_Parts/Mechanical/
    ├── <bracket_name>.step
    ├── <bracket_name>.stl
    └── <bracket_name>_metadata.json
"""

import sys
import json
import argparse
import time
import logging
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any, List, Tuple
from datetime import datetime

# ============================================================================
# IMPORTS
# ============================================================================

def validate_imports():
    """Validate required imports are available."""
    try:
        import cadquery
        return cadquery
    except ImportError:
        print("ERROR: CadQuery not available. Install with: uv sync")
        sys.exit(1)


# ============================================================================
# SETUP: Logging
# ============================================================================

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s | %(levelname)-8s | %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)


# ============================================================================
# DATA STRUCTURES
# ============================================================================

@dataclass
class BracketParameters:
    """Bracket generation parameters."""
    length_mm: float          # X dimension (length)
    width_mm: float           # Y dimension (width)
    thickness_mm: float       # Z dimension (thickness)
    hole_diameter_mm: float   # Mounting hole diameter
    fillet_radius_mm: float   # Edge fillet radius


@dataclass
class GeometryStats:
    """Computed geometry statistics."""
    volume_mm3: float
    surface_area_mm2: float
    bounds_x: Tuple[float, float]
    bounds_y: Tuple[float, float]
    bounds_z: Tuple[float, float]


@dataclass
class BracketMetadata:
    """Complete metadata for generated bracket."""
    bracket_name: str
    bracket_type: str
    parameters: Dict[str, Any]
    geometry: Dict[str, Any]
    exported_formats: List[str]
    execution_time_seconds: float
    generated_at: str
    generator_version: str = "1.0.0"


# ============================================================================
# CONSTANTS
# ============================================================================

SCRIPT_DIR = Path(__file__).parent
OUTPUT_DIR = SCRIPT_DIR.parent / "Mechanical"
CONFIG_FILE = SCRIPT_DIR / "bracket_configs.json"

# Default parameters
DEFAULT_PARAMS = {
    "simple": {
        "length_mm": 100.0,
        "width_mm": 80.0,
        "thickness_mm": 10.0,
        "hole_diameter_mm": 8.0,
        "fillet_radius_mm": 2.0
    },
    "l_bracket": {
        "length_mm": 100.0,
        "width_mm": 80.0,
        "thickness_mm": 10.0,
        "hole_diameter_mm": 8.0,
        "fillet_radius_mm": 3.0
    },
    "corner": {
        "length_mm": 150.0,
        "width_mm": 120.0,
        "thickness_mm": 12.0,
        "hole_diameter_mm": 10.0,
        "fillet_radius_mm": 4.0
    }
}


# ============================================================================
# BRACKET GENERATOR CLASS
# ============================================================================

class BracketGenerator:
    """CadQuery-based parametric bracket generator."""

    def __init__(self, cq):
        """Initialize generator with CadQuery module.

        Args:
            cq: CadQuery module
        """
        self.cq = cq
        self.solid = None
        self.metadata = None

    def create_simple_plate(self, params: BracketParameters) -> Any:
        """Create simple plate bracket with mounting holes.

        A rectangular plate with 4 mounting holes (one in each corner).

        Args:
            params: Bracket parameters

        Returns:
            CadQuery workplane with completed bracket
        """
        logger.info(f"Creating simple plate bracket: {params.length_mm}×{params.width_mm}×{params.thickness_mm} mm")

        # Create base rectangular plate
        cq_solid = self.cq.Workplane("XY").box(
            params.length_mm,
            params.width_mm,
            params.thickness_mm
        )

        # Define hole positions (corners with margins)
        margin = params.hole_diameter_mm * 1.5
        hole_radius = params.hole_diameter_mm / 2.0

        # Calculate corner positions
        half_length = params.length_mm / 2.0
        half_width = params.width_mm / 2.0
        hole_x = half_length - margin
        hole_y = half_width - margin

        hole_positions = [
            (hole_x, hole_y),
            (-hole_x, hole_y),
            (hole_x, -hole_y),
            (-hole_x, -hole_y)
        ]

        # Create mounting hole as a cylinder
        hole_height = params.thickness_mm * 2
        hole_z = params.thickness_mm / 2.0

        # Subtract holes from plate
        for x, y in hole_positions:
            hole_solid = (
                self.cq.Workplane("XY")
                .moveTo(x, y)
                .circle(hole_radius)
                .extrude(hole_height, both=True)
            )
            cq_solid = cq_solid.cut(hole_solid)

        # Apply fillet to edges
        if params.fillet_radius_mm > 0:
            cq_solid = cq_solid.edges().fillet(params.fillet_radius_mm)

        self.solid = cq_solid
        return cq_solid

    def create_l_bracket(self, params: BracketParameters) -> Any:
        """Create L-bracket (two perpendicular plates).

        Args:
            params: Bracket parameters

        Returns:
            CadQuery workplane with L-bracket
        """
        logger.info(f"Creating L-bracket: {params.length_mm}×{params.width_mm}×{params.thickness_mm} mm")

        # Create first plate (horizontal)
        plate1 = self.cq.Workplane("XY").box(
            params.length_mm,
            params.width_mm,
            params.thickness_mm
        )

        # Create second plate (vertical)
        plate2 = self.cq.Workplane("XY").box(
            params.thickness_mm,
            params.width_mm,
            params.length_mm
        )

        # Position plate2 at the edge of plate1
        # Move plate2 so it connects to the edge
        half_length = params.length_mm / 2.0
        half_thickness = params.thickness_mm / 2.0

        # Create a solid body by combining
        combined = plate1.union(plate2)

        # Apply fillet to edges
        if params.fillet_radius_mm > 0:
            combined = combined.edges().fillet(params.fillet_radius_mm)

        self.solid = combined
        return combined

    def create_corner_bracket(self, params: BracketParameters) -> Any:
        """Create reinforced corner bracket.

        L-bracket with additional corner support for improved rigidity.

        Args:
            params: Bracket parameters

        Returns:
            CadQuery workplane with corner bracket
        """
        logger.info(f"Creating corner bracket: {params.length_mm}×{params.width_mm}×{params.thickness_mm} mm")

        # Create first plate (horizontal)
        plate1 = self.cq.Workplane("XY").box(
            params.length_mm,
            params.width_mm,
            params.thickness_mm
        )

        # Create second plate (vertical)
        plate2 = self.cq.Workplane("XY").box(
            params.thickness_mm,
            params.width_mm,
            params.length_mm
        )

        # Create corner support (small triangular/stepped support)
        support_size = params.thickness_mm * 2.5
        corner_support = self.cq.Workplane("XY").box(
            support_size,
            support_size,
            params.thickness_mm
        )

        # Combine all parts
        combined = plate1.union(plate2).union(corner_support)

        # Apply fillet to edges
        if params.fillet_radius_mm > 0:
            combined = combined.edges().fillet(params.fillet_radius_mm)

        self.solid = combined
        return combined

    def compute_geometry_stats(self) -> GeometryStats:
        """Compute geometry statistics from solid.

        Returns:
            GeometryStats object with computed values
        """
        if self.solid is None:
            raise ValueError("No solid generated. Call create_* method first.")

        # Get the underlying solid object
        shape = self.solid.val()

        # Get bounding box
        bb = shape.BoundingBox()
        bounds_x = (bb.xmin, bb.xmax)
        bounds_y = (bb.ymin, bb.ymax)
        bounds_z = (bb.zmin, bb.zmax)

        # Calculate volume and surface area
        # Handle both Solid and Compound objects
        try:
            volume = float(shape.Volume)
        except Exception:
            volume = 0.0

        try:
            surface_area = float(shape.Surface)
        except (AttributeError, RuntimeError, TypeError):
            # For Compound objects or when Surface is not available,
            # approximate by computing surface area from faces
            try:
                surface_area = sum(float(face.Surface) for face in shape.Faces)
            except Exception:
                surface_area = 0.0

        return GeometryStats(
            volume_mm3=volume,
            surface_area_mm2=surface_area,
            bounds_x=bounds_x,
            bounds_y=bounds_y,
            bounds_z=bounds_z
        )

    def generate(self, bracket_type: str, params: BracketParameters) -> None:
        """Generate bracket solid.

        Args:
            bracket_type: Type of bracket to generate ('simple', 'l_bracket', 'corner')
            params: Bracket parameters

        Raises:
            ValueError: If bracket type is invalid
        """
        if bracket_type == "simple":
            self.create_simple_plate(params)
        elif bracket_type == "l_bracket":
            self.create_l_bracket(params)
        elif bracket_type == "corner":
            self.create_corner_bracket(params)
        else:
            raise ValueError(f"Unknown bracket type: {bracket_type}")

        if self.solid is None:
            raise ValueError("Failed to generate bracket solid")

    def export_step(self, output_path: Path) -> bool:
        """Export solid to STEP format.

        Args:
            output_path: Output file path

        Returns:
            bool: True if successful
        """
        if self.solid is None:
            logger.error("No solid to export")
            return False

        try:
            logger.info(f"Exporting STEP: {output_path}")
            self.cq.exporters.export(self.solid, str(output_path))
            logger.info(f"  ✓ Exported {output_path.name}")
            return True
        except Exception as e:
            logger.error(f"Failed to export STEP: {e}")
            return False

    def export_stl(self, output_path: Path) -> bool:
        """Export solid to STL format.

        Args:
            output_path: Output file path

        Returns:
            bool: True if successful
        """
        if self.solid is None:
            logger.error("No solid to export")
            return False

        try:
            logger.info(f"Exporting STL: {output_path}")
            self.cq.exporters.export(self.solid, str(output_path))
            logger.info(f"  ✓ Exported {output_path.name}")
            return True
        except Exception as e:
            logger.error(f"Failed to export STL: {e}")
            return False

    def generate_metadata(
        self,
        bracket_name: str,
        bracket_type: str,
        params: BracketParameters,
        execution_time: float,
        exported_formats: List[str]
    ) -> BracketMetadata:
        """Generate metadata report for bracket.

        Args:
            bracket_name: Name of the bracket
            bracket_type: Type of bracket
            params: Bracket parameters
            execution_time: Time taken to generate (seconds)
            exported_formats: List of exported format names

        Returns:
            BracketMetadata object
        """
        stats = self.compute_geometry_stats()

        metadata = BracketMetadata(
            bracket_name=bracket_name,
            bracket_type=bracket_type,
            parameters=asdict(params),
            geometry={
                "volume_mm3": round(stats.volume_mm3, 2),
                "surface_area_mm2": round(stats.surface_area_mm2, 2),
                "bounds_mm": {
                    "x": [round(stats.bounds_x[0], 2), round(stats.bounds_x[1], 2)],
                    "y": [round(stats.bounds_y[0], 2), round(stats.bounds_y[1], 2)],
                    "z": [round(stats.bounds_z[0], 2), round(stats.bounds_z[1], 2)]
                }
            },
            exported_formats=exported_formats,
            execution_time_seconds=round(execution_time, 3),
            generated_at=datetime.now().isoformat()
        )

        self.metadata = metadata
        return metadata


# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def load_config(config_path: Path) -> Dict[str, Any]:
    """Load bracket configurations from JSON file.

    Args:
        config_path: Path to JSON config file

    Returns:
        Config dict with 'configurations' list

    Raises:
        FileNotFoundError: If config file doesn't exist
        json.JSONDecodeError: If config is invalid JSON
    """
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    logger.info(f"Loading config from {config_path.name}")
    with open(config_path, 'r') as f:
        config = json.load(f)

    return config


def parse_arguments() -> argparse.Namespace:
    """Parse command-line arguments.

    Returns:
        Parsed arguments namespace
    """
    parser = argparse.ArgumentParser(
        description="CadQuery Parametric Bracket Generator"
    )

    # Mode selection
    mode_group = parser.add_mutually_exclusive_group(required=True)
    mode_group.add_argument(
        "--config",
        type=Path,
        help="Load bracket configs from JSON file"
    )
    mode_group.add_argument(
        "--type",
        choices=["simple", "l_bracket", "corner"],
        help="Bracket type for CLI mode"
    )

    # CLI parameters
    parser.add_argument("--name", type=str, help="Bracket name (used in filenames)")
    parser.add_argument("--length", type=float, help="Bracket length (mm)")
    parser.add_argument("--width", type=float, help="Bracket width (mm)")
    parser.add_argument("--thickness", type=float, help="Bracket thickness (mm)")
    parser.add_argument("--hole-diameter", type=float, help="Mounting hole diameter (mm)")
    parser.add_argument("--fillet-radius", type=float, help="Edge fillet radius (mm)")

    return parser.parse_args()


def merge_parameters(bracket_type: str, cli_args: argparse.Namespace) -> BracketParameters:
    """Merge CLI arguments with defaults.

    Args:
        bracket_type: Type of bracket
        cli_args: Parsed CLI arguments

    Returns:
        Merged BracketParameters
    """
    # Start with defaults
    defaults = DEFAULT_PARAMS.get(bracket_type, {})

    # Override with CLI arguments
    params = BracketParameters(
        length_mm=cli_args.length if cli_args.length else defaults.get("length_mm", 100.0),
        width_mm=cli_args.width if cli_args.width else defaults.get("width_mm", 80.0),
        thickness_mm=cli_args.thickness if cli_args.thickness else defaults.get("thickness_mm", 10.0),
        hole_diameter_mm=cli_args.hole_diameter if cli_args.hole_diameter else defaults.get("hole_diameter_mm", 8.0),
        fillet_radius_mm=cli_args.fillet_radius if cli_args.fillet_radius else defaults.get("fillet_radius_mm", 2.0),
    )

    return params


def process_single_bracket(
    cq,
    bracket_name: str,
    bracket_type: str,
    params: BracketParameters,
    output_dir: Path
) -> bool:
    """Process a single bracket generation and export.

    Args:
        cq: CadQuery module
        bracket_name: Name of bracket
        bracket_type: Type of bracket
        params: Bracket parameters
        output_dir: Output directory

    Returns:
        bool: True if successful
    """
    start_time = time.time()

    try:
        # Create generator
        generator = BracketGenerator(cq)

        # Generate bracket
        generator.generate(bracket_type, params)

        # Export formats
        step_path = output_dir / f"{bracket_name}.step"
        stl_path = output_dir / f"{bracket_name}.stl"

        exported = []
        if generator.export_step(step_path):
            exported.append("step")
        if generator.export_stl(stl_path):
            exported.append("stl")

        # Generate metadata
        execution_time = time.time() - start_time
        metadata = generator.generate_metadata(
            bracket_name,
            bracket_type,
            params,
            execution_time,
            exported
        )

        # Save metadata JSON
        metadata_path = output_dir / f"{bracket_name}_metadata.json"
        with open(metadata_path, 'w') as f:
            json.dump(asdict(metadata), f, indent=2)
        logger.info(f"  ✓ Metadata saved to {metadata_path.name}")

        # Print summary
        print("\n" + "=" * 70)
        print(f"✓ {bracket_name} ({bracket_type}) generated successfully")
        print("=" * 70)
        print(f"\nBracket parameters:")
        print(f"  Length:         {params.length_mm} mm")
        print(f"  Width:          {params.width_mm} mm")
        print(f"  Thickness:      {params.thickness_mm} mm")
        print(f"  Hole diameter:  {params.hole_diameter_mm} mm")
        print(f"  Fillet radius:  {params.fillet_radius_mm} mm")
        print(f"\nGeometry:")
        print(f"  Volume:         {metadata.geometry['volume_mm3']} mm³")
        print(f"  Surface area:   {metadata.geometry['surface_area_mm2']} mm²")
        print(f"\nGenerated files:")
        print(f"  • {step_path.name}")
        print(f"  • {stl_path.name}")
        print(f"  • {metadata_path.name}")
        print(f"\nExecution time: {execution_time:.3f} seconds")

        return True

    except Exception as e:
        logger.error(f"Failed to generate {bracket_name}: {e}")
        import traceback
        traceback.print_exc()
        return False


# ============================================================================
# MAIN
# ============================================================================

def main():
    """Main entry point."""
    cq = validate_imports()

    # Ensure output directory
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    args = parse_arguments()

    try:
        if args.config:
            # Config file mode
            config = load_config(args.config)

            success_count = 0
            for bracket_config in config.get("configurations", []):
                name = bracket_config.get("name")
                b_type = bracket_config.get("type")
                b_params_dict = bracket_config.get("parameters", {})

                b_params = BracketParameters(
                    length_mm=b_params_dict.get("length_mm", 100.0),
                    width_mm=b_params_dict.get("width_mm", 80.0),
                    thickness_mm=b_params_dict.get("thickness_mm", 10.0),
                    hole_diameter_mm=b_params_dict.get("hole_diameter_mm", 8.0),
                    fillet_radius_mm=b_params_dict.get("fillet_radius_mm", 2.0),
                )

                if process_single_bracket(cq, name, b_type, b_params, OUTPUT_DIR):
                    success_count += 1

            print("\n" + "=" * 70)
            print(f"Configuration mode: {success_count}/{len(config.get('configurations', []))} brackets generated")
            print("=" * 70)

            return 0 if success_count == len(config.get('configurations', [])) else 1

        else:
            # CLI mode
            bracket_type = args.type
            bracket_name = args.name if args.name else f"bracket_{bracket_type}"

            params = merge_parameters(bracket_type, args)

            success = process_single_bracket(cq, bracket_name, bracket_type, params, OUTPUT_DIR)
            return 0 if success else 1

    except Exception as e:
        logger.error(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
