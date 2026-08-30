# FreeCAD MCP Tools Reference

> **Deprecated.** This project no longer uses the `freecad-robust-mcp` package
> or an MCP bridge as its FreeCAD workflow. FreeCAD is now invoked headlessly
> via subprocess (`freecadcmd`, configured with the `FREECAD_BIN` environment
> variable) — see `../README.md` and `../03_Parts/Generators/README.md` for
> the current pattern. This file is kept only as a historical catalog of the
> MCP tool surface, for anyone still running the `freecad-mcp-server` bridge
> directly; it does not describe this project's active tooling.

Quick reference for 150+ FreeCAD tools available via Model Context Protocol.

## Setup

**Start FreeCAD with MCP Bridge:**
```bash
cd ../../freecad-mcp-server
./scripts/start-mcp-freecad.sh --mode xmlrpc
```

**Configuration:** `.mcp.json` in project root
- Protocol: XML-RPC (port 9875) — recommended
- Alternative: Socket mode (port 9876)

## Common Tool Categories

### Document Management

| Tool | Purpose | Example |
|------|---------|---------|
| `create_document` | Create new FreeCAD document | `create_document(name='MyPart')` |
| `open_document` | Open existing FCStd file | `open_document(filepath='model.FCStd')` |
| `save_document` | Save current document | `save_document(filepath='output.FCStd')` |
| `get_active_document` | Get current document name | N/A |

### Geometry Creation (Primitives)

| Tool | Parameters | Example |
|------|------------|---------|
| `create_box` | length, width, height | `create_box(length=100, width=50, height=20)` |
| `create_cylinder` | radius, height | `create_cylinder(radius=5, height=30)` |
| `create_sphere` | radius | `create_sphere(radius=10)` |
| `create_cone` | radius1, radius2, height | `create_cone(radius1=5, radius2=0, height=20)` |
| `create_wedge` | length, width, height | `create_wedge(length=100, width=50, height=20)` |
| `create_torus` | radius1, radius2 | `create_torus(radius1=20, radius2=5)` |

### Boolean Operations

| Operation | Result |
|-----------|--------|
| `union` | Combines two solids |
| `cut` | Removes tool from base |
| `intersect` | Keeps only overlapping volume |

**Usage:**
```python
mcp['boolean_operation'](
    base_object='Solid1',
    tool_object='Solid2',
    operation='cut',  # 'union', 'cut', or 'intersect'
    name='Result'
)
```

### Object Management

| Tool | Purpose |
|------|---------|
| `edit_object` | Modify object properties |
| `delete_object` | Remove object from document |
| `get_object_properties` | Query object parameters |
| `duplicate_object` | Create copy of object |

### PartDesign Tools (Sketching)

| Tool | Purpose | Use Case |
|------|---------|----------|
| `create_sketch` | Create 2D sketch | Foundation for Pad/Pocket |
| `add_sketch_circle` | Add circle to sketch | Define holes, edges |
| `add_sketch_rectangle` | Add rectangle | Define profiles |
| `add_sketch_line` | Add line segment | Draw geometry |
| `pad_sketch` | Extrude sketch to 3D | Create solid from profile |
| `pocket_sketch` | Subtract sketch volume | Cut away from solid |
| `create_fillet` | Round edges | Smooth transitions |
| `fillet_edges` | Fillet specific edges | Apply radius to edges |

### Patterns

| Tool | Purpose |
|------|---------|
| `linear_pattern` | Repeat object in line |
| `polar_pattern` | Repeat object in circle |
| `mirrored_pattern` | Mirror object |

### View & Display

| Tool | Purpose | Example |
|------|---------|---------|
| `get_screenshot` | Capture viewport image | `get_screenshot(filepath='view.png')` |
| `set_view_angle` | Change camera angle | `set_view_angle(view='top')` |
| `set_object_color` | Change object color | `set_object_color(object='Box', color=[1,0,0])` |
| `zoom_to_fit` | Auto-fit view | N/A |

### Export/Import

| Format | Tool |
|--------|------|
| STEP | `export_step(object_name, filepath)` |
| STL | `export_stl(object_name, filepath)` |
| IGES | `export_iges(object_name, filepath)` |
| 3MF | `export_3mf(object_name, filepath)` |
| OBJ | `export_obj(object_name, filepath)` |

### Macro Management

| Tool | Purpose |
|------|---------|
| `create_macro` | Create Python macro |
| `run_macro` | Execute macro |
| `delete_macro` | Remove macro |
| `list_macros` | List available macros |

## Example: Creating a Simple Part

```python
from freecad_integration_example import use_freecad_via_mcp

# Connect to FreeCAD
mcp = use_freecad_via_mcp()

# Create document
doc = mcp['create_document'](name='Part1')

# Create base box
box = mcp['create_box'](length=100, width=50, height=20, name='Base')

# Create cylinder for hole
hole = mcp['create_cylinder'](radius=5, height=30, name='Hole')

# Boolean cut
part = mcp['boolean_operation'](
    base_object=box,
    tool_object=hole,
    operation='cut',
    name='FinalPart'
)

# Fillet edges
mcp['fillet_edges'](object=part, radius=2)

# Export
mcp['export_step'](object_name=part, filepath='part.step')
mcp['save_document'](filepath='part.FCStd')
```

## Connection Status

**Test MCP bridge connectivity:**
```bash
python3 -c "from freecad_integration_example import use_freecad_via_mcp; mcp = use_freecad_via_mcp(); print('✓ Connected')"
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "Connection refused" | Start MCP bridge: `./freecad-mcp-server/scripts/start-mcp-freecad.sh` |
| Tool not found | This project no longer installs `freecad-robust-mcp`; install/update it separately if you are still using the MCP bridge standalone |
| Permission denied on export | Check file path and directory permissions |
| Slow operations | Close FreeCAD UI or use headless mode |

## References

- [FreeCAD Part Workbench](https://wiki.freecad.org/Part_Workbench)
- [FreeCAD PartDesign](https://wiki.freecad.org/PartDesign_Workbench)
- [Robust MCP Server Docs](https://spkane.github.io/freecad-addon-robust-mcp-server/)
- [MCP Protocol](https://modelcontextprotocol.io/)

## See Also

- `../03_Parts/Generators/simple_bracket.py` — Legacy MCP-based example (not part of the active `pendulum-tools` workflow)
- `../../CLAUDE.md` — Full development guide
