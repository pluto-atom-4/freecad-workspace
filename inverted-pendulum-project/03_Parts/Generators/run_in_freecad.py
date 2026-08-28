#!/usr/bin/env python3
"""
Script to execute plate generator in running FreeCAD instance via IPC

This connects to FreeCAD (if running) and executes the generator script.
"""

import sys
import socket
import json
from pathlib import Path


def send_to_freecad_console(code: str, port: int = 9876) -> bool:
    """
    Send Python code to FreeCAD JSON-RPC interface

    Args:
        code: Python code to execute
        port: JSON-RPC port (default 9876)

    Returns:
        True if sent successfully
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        sock.connect(('localhost', port))

        request = {
            "jsonrpc": "2.0",
            "id": 1,
            "method": "execute",
            "params": {"code": code}
        }

        sock.send((json.dumps(request) + "\n").encode())

        response = sock.recv(4096).decode()
        print(f"Response: {response}")

        sock.close()
        return True

    except Exception as e:
        print(f"Error connecting to FreeCAD: {e}")
        return False


def main():
    """Main entry point"""
    print("FreeCAD Generator IPC Executor")
    print("-" * 60)
    print()

    # Check if FreeCAD is running
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1)
        result = sock.connect_ex(('localhost', 9876))
        sock.close()

        if result != 0:
            print("✗ FreeCAD MCP bridge not responding on port 9876")
            print("  Start FreeCAD with MCP Bridge first")
            return 1

        print("✓ FreeCAD MCP bridge detected")

    except Exception as e:
        print(f"✗ Error checking FreeCAD: {e}")
        return 1

    print()
    print("Executing plate generator in FreeCAD...")
    print()

    # Read the generator script
    script_path = Path(__file__).parent / "create_plates_simple.py"

    try:
        with open(script_path, 'r') as f:
            script_content = f.read()

        # Extract just the main logic (skip imports, focus on generation)
        injection_code = """
import sys
sys.path.insert(0, '/home/pluto-atom-4/freecad-workspace/inverted-pendulum-project/03_Parts/Generators')

from create_plates_simple import PlateAssembly

generator = PlateAssembly()
doc = generator.generate()
print("✓ Plate assembly generated successfully!")
"""

        sent = send_to_freecad_console(injection_code)

        if sent:
            print("✓ Code sent to FreeCAD")
        else:
            print("✗ Failed to send code to FreeCAD")
            return 1

    except Exception as e:
        print(f"✗ Error: {e}")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
