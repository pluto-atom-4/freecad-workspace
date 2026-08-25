# Part Generators

Python scripts for parametric FreeCAD model generation.

## Scripts

### `simple_part.py`
Direct FreeCAD part generation using AppImage Python interpreter.

**Features:**
- 40×20×15mm base block with features
- 4mm through-hole
- 3mm fillet, 5mm chamfer
- Exports FCStd + STEP formats

**Usage:**
```bash
# Via wrapper script (recommended)
./run_part.sh

# Direct execution
~/tmp/squashfs-root/usr/bin/python simple_part.py
```

**Output:**
- `../Mechanical/T101pwb01_02_Part.FCStd`
- `../Mechanical/T101pwb01_02_Part.step`

### `simple_bracket.py`
Support bracket generation via FreeCAD MCP Bridge.

**Requirements:**
- FreeCAD running with MCP Bridge on port 9875
- `freecad-robust-mcp` package

**Usage:**
```bash
# Start FreeCAD with MCP Bridge (in another terminal)
cd ../../freecad-mcp-server
./scripts/start-mcp-freecad.sh --mode xmlrpc

# Run script
uv run python3 simple_bracket.py
```

**Output:**
- `../Mechanical/support_bracket.FCStd`
- `../Mechanical/support_bracket.step`

## Directory Structure

```
Generators/
├── simple_part.py       # Direct AppImage generation
├── simple_bracket.py    # MCP bridge generation  
├── run_part.sh          # Wrapper script (simple_part.py)
└── README.md            # This file
```

## Best Practices

- ✓ Type hints (Python 3.11+)
- ✓ Comprehensive docstrings
- ✓ FreeCAD best practices (recompute, cleanup)
- ✓ Proper error handling
- ✓ Structured logging
- ✓ Constants in UPPER_CASE

## References

- [FreeCAD Part Module](https://wiki.freecadweb.org/Part_Module)
- [FreeCAD Scripting Basics](https://wiki.freecadweb.org/Scripting_basics)
- [FreeCAD MCP Server](https://github.com/spkane/freecad-addon-robust-mcp-server)
