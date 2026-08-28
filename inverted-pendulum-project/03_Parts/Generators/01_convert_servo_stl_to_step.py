#!/usr/bin/env python3
"""
STL to STEP Conversion Wrapper: FeeTech STS3032 Servo Motor

Phase 1: Convert STL mesh to STEP format for servo motor integration.

This wrapper script executes the actual conversion logic (01_convert_servo_stl_to_step_via_freecad.py)
inside FreeCAD's Python environment, which provides access to the required
FreeCAD modules (Part, Mesh, App).

Features:
  - Validates STL file integrity (geometry, manifoldness)
  - Imports STL using FreeCAD Mesh module
  - Converts mesh faces to Part shape
  - Exports to STEP with dimensional precision
  - Validates coordinate system (servo shaft at origin)
  - Reports file sizes and geometry metrics

Usage:
  python3 01_convert_servo_stl_to_step.py

  Or via shell:
  ./01_convert_servo_stl_to_step.sh

  Or manually with freecad:
  freecad -c "exec(open('01_convert_servo_stl_to_step_via_freecad.py').read())"

Output:
  - inverted-pendulum-project/03_Parts/Mechanical/feetech-STS3032.step
  - inverted-pendulum-project/03_Parts/Mechanical/feetech-STS3032_conversion_report.json
  - Conversion report with validation results

Dependencies:
  - FreeCAD (system installation)
  - freecad command must be available in PATH
"""

import os
import sys
import subprocess
from pathlib import Path


def main():
    """Run the STL to STEP conversion via FreeCAD."""
    # Get the directory where this script is located
    script_dir = Path(__file__).parent
    conversion_script = script_dir / "01_convert_servo_stl_to_step_via_freecad.py"

    # Verify the conversion script exists
    if not conversion_script.exists():
        print(f"✗ Conversion script not found: {conversion_script}", file=sys.stderr)
        sys.exit(1)

    # Run the conversion via FreeCAD
    print("Starting STL to STEP conversion...")
    print(f"Script: {conversion_script}")
    print("")

    cmd = [
        "freecad",
        "-c",
        f"exec(open('{conversion_script}').read())"
    ]

    try:
        # Execute FreeCAD with the conversion script
        # Filter out GUI-related warnings
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=600)

        # Print output, filtering out Wayland/Qt/EGL warnings
        output_lines = result.stdout.split('\n')
        for line in output_lines:
            if not any(x in line for x in ['Wayland', 'Qt', 'EGL', 'Recompute']):
                if line.strip():
                    print(line)

        # Print stderr if there are actual errors (not just GUI warnings)
        if result.returncode != 0:
            error_lines = result.stderr.split('\n')
            for line in error_lines:
                if not any(x in line for x in ['Wayland', 'Qt', 'EGL']):
                    if line.strip():
                        print(line, file=sys.stderr)
            sys.exit(result.returncode)

        # Check if output was created
        output_step = script_dir.parent / "Mechanical" / "feetech-STS3032.step"
        report_json = script_dir.parent / "Mechanical" / "feetech-STS3032_conversion_report.json"

        if output_step.exists():
            print("")
            print("✓ Conversion completed successfully!")
            print(f"  Output: {output_step}")

            # Show file size
            size_mb = output_step.stat().st_size / (1024 * 1024)
            print(f"  Size: {size_mb:.2f} MB")
        else:
            print("")
            print("✗ Output file not created")
            sys.exit(1)

        if report_json.exists():
            print(f"  Report: {report_json}")

        return 0

    except subprocess.TimeoutExpired:
        print("✗ Conversion timed out after 10 minutes", file=sys.stderr)
        sys.exit(1)
    except FileNotFoundError:
        print("✗ FreeCAD not found. Is it installed?", file=sys.stderr)
        print("  Install with: sudo apt-get install freecad", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"✗ Error running conversion: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    sys.exit(main())
