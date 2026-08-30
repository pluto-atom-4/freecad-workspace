# Phase 6: CadQuery Parametric Bracket Generation & Trimesh Mesh Handling

Phase 6 provides standalone, headless tooling for parametric bracket generation, mesh validation/repair, and multi-mesh assembly. No FreeCAD runtime required—ideal for CAD automation pipelines, 3D printing workflows, and programmatic part generation.

---

## Overview

### Purpose

Phase 6 enables three independent but complementary capabilities:

1. **Parametric Bracket Generation** — Generate support brackets (simple, L, corner types) with customizable dimensions
2. **Mesh Validation & Repair** — Validate STL meshes for 3D printing, auto-repair common issues
3. **Multi-mesh Assembly** — Combine multiple meshes with spatial transforms into composite STL files

### Key Capabilities

- **Standalone Execution** — No FreeCAD GUI, MCP bridge, or embedded Python required
- **Batch Processing** — JSON configuration files for generating multiple parts
- **Export Flexibility** — STEP (CAD) and STL (3D printing) formats
- **Metadata Tracking** — JSON reports with geometry stats, validation results, repair logs
- **Parametric Design** — CLI arguments or config files for dimension control
- **Mesh Health** — Detect watertight meshes, fix degeneracies, merge vertices

### Target Audience

- CAD designers building automated workflows
- Simulation engineers exporting geometry for analysis
- 3D printing operators validating and preparing models
- Anyone needing headless part generation without FreeCAD

---

## Quick Start

### Environment Setup

```bash
# Activate mamba Phase 6 environment
mamba activate pendulum-phase6

# Verify imports
python3 -c "import cadquery; import trimesh; print('OK')"
```

### First Run: Generate a Simple Bracket

```bash
# Create a 100×80×10mm simple plate bracket with 8mm mounting holes
python3 06_cadquery_parametric_brackets.py \
  --type simple \
  --length 100 \
  --width 80 \
  --thickness 10 \
  --hole-diameter 8 \
  --fillet-radius 3

# Output:
# ✓ Created: 03_Parts/Mechanical/simple_bracket.step (CAD format)
# ✓ Created: 03_Parts/Mechanical/simple_bracket.stl (3D printing)
# ✓ Created: 03_Parts/Mechanical/simple_bracket_metadata.json (geometry info)
```

---

## Tool 1: CadQuery Parametric Bracket Generator

**Script:** `06_cadquery_parametric_brackets.py`

**Purpose:** Generate parametric support brackets in 3 types with automatic STEP/STL export.

### Bracket Types

| Type | Description | Use Case |
|------|-------------|----------|
| **Simple** | Single rectangular plate with mounting holes | Basic mounting, flat surfaces |
| **L-bracket** | Two perpendicular plates (90° angle) | Corner reinforcement, wall mounts |
| **Corner** | Reinforced L-bracket with corner support | Heavy-duty, load-bearing applications |

### CLI Mode: Custom Parameters

```bash
# Simple plate bracket (most common)
python3 06_cadquery_parametric_brackets.py \
  --type simple \
  --length 100 \
  --width 80 \
  --thickness 10 \
  --hole-diameter 8 \
  --fillet-radius 3 \
  --output my_bracket

# L-bracket (two perpendicular plates)
python3 06_cadquery_parametric_brackets.py \
  --type l_bracket \
  --length 100 \
  --width 80 \
  --thickness 10 \
  --hole-diameter 8 \
  --fillet-radius 3

# Corner bracket (reinforced)
python3 06_cadquery_parametric_brackets.py \
  --type corner \
  --length 150 \
  --width 120 \
  --thickness 12 \
  --hole-diameter 10 \
  --fillet-radius 4

# List all parameters
python3 06_cadquery_parametric_brackets.py --help
```

### Config Mode: Batch Generation

Create `bracket_configs.json` (or use the provided template):

```json
{
  "configurations": [
    {
      "name": "small_bracket",
      "type": "simple",
      "parameters": {
        "length_mm": 60,
        "width_mm": 50,
        "thickness_mm": 8,
        "hole_diameter_mm": 6,
        "fillet_radius_mm": 2
      }
    },
    {
      "name": "medium_bracket",
      "type": "l_bracket",
      "parameters": {
        "length_mm": 100,
        "width_mm": 80,
        "thickness_mm": 10,
        "hole_diameter_mm": 8,
        "fillet_radius_mm": 3
      }
    },
    {
      "name": "large_bracket",
      "type": "corner",
      "parameters": {
        "length_mm": 150,
        "width_mm": 120,
        "thickness_mm": 12,
        "hole_diameter_mm": 10,
        "fillet_radius_mm": 4
      }
    }
  ]
}
```

Run batch generation:

```bash
python3 06_cadquery_parametric_brackets.py --config bracket_configs.json

# Output:
# ✓ Generated: small_bracket.step, small_bracket.stl
# ✓ Generated: medium_bracket.step, medium_bracket.stl
# ✓ Generated: large_bracket.step, large_bracket.stl
# ✓ Created: bracket_generation_report.json
```

### Parameter Reference

| Parameter | Type | Range | Default | Notes |
|-----------|------|-------|---------|-------|
| `length_mm` | float | 10–500 | — | X dimension (primary length) |
| `width_mm` | float | 10–500 | — | Y dimension (secondary length) |
| `thickness_mm` | float | 2–50 | — | Z dimension (mounting surface depth) |
| `hole_diameter_mm` | float | 2–20 | 8 | Mounting hole diameter (2x holes by default) |
| `fillet_radius_mm` | float | 0–10 | 2 | Edge fillet for smooth finish |

### Output Files

For a bracket named `my_bracket`:

```
03_Parts/Mechanical/
├── my_bracket.step                    # CAD format (use in FreeCAD, Fusion 360, etc.)
├── my_bracket.stl                     # STL mesh (3D printing)
└── my_bracket_metadata.json           # Geometry statistics
```

### Metadata JSON Example

```json
{
  "bracket_type": "simple",
  "generation_timestamp": "2026-08-30T09:15:42.123456",
  "generation_time_ms": 245,
  "parameters": {
    "length_mm": 100,
    "width_mm": 80,
    "thickness_mm": 10,
    "hole_diameter_mm": 8,
    "fillet_radius_mm": 3
  },
  "geometry_stats": {
    "volume_mm3": 7840.5,
    "surface_area_mm2": 2456.3,
    "bounds_x": [0.0, 100.0],
    "bounds_y": [0.0, 80.0],
    "bounds_z": [0.0, 10.0]
  },
  "export_formats": ["step", "stl"],
  "step_file": "my_bracket.step",
  "stl_file": "my_bracket.stl",
  "file_sizes_bytes": {
    "step": 45120,
    "stl": 28560
  }
}
```

### Export Format Selection

- **Use STEP** — Sharing with colleagues in CAD software, further design iteration
- **Use STL** — 3D printing preparation, mesh-based simulation tools

Both are generated by default.

---

## Tool 2: Trimesh Mesh Validator

**Script:** `06_trimesh_mesh_validator.py`

**Purpose:** Validate STL mesh files and automatically repair common 3D printing issues.

### What It Does

1. **Loads** — Parse STL mesh file
2. **Analyzes** — Check for watertight property, vertex/face count, geometry bounds
3. **Validates** — Detect non-manifold edges, open surfaces, degenerate faces
4. **Repairs** (optional) — Merge duplicate vertices, fill small holes, remove bad faces
5. **Reports** — JSON output with before/after statistics
6. **Exports** (optional) — Save repaired mesh to new STL file

### Validation-Only Mode

Check a mesh without modifying it:

```bash
# Validate servo.stl
python3 06_trimesh_mesh_validator.py --input ../Mechanical/servo.stl

# Output (console):
# Mesh validation results:
#   File: ../Mechanical/servo.stl
#   Vertices: 8,456
#   Faces: 16,912
#   Watertight: YES
#   Open edges: 0
#   Status: ✓ VALID (ready for 3D printing)
#
# Report written: servo_validation_report.json
```

### Auto-Repair Mode

Validate and repair a mesh:

```bash
# Auto-repair and output new file
python3 06_trimesh_mesh_validator.py \
  --input problematic.stl \
  --auto-repair \
  --output problematic_repaired.stl

# Output:
# Mesh validation results:
#   Original vertices: 10,245
#   Repaired vertices: 9,876 (removed 369 duplicates)
#   Original faces: 20,490
#   Repaired faces: 20,412 (removed 78 degenerate)
#   Watertight before: NO
#   Watertight after: YES (auto-repaired)
#   Status: ✓ REPAIRED
#
# Repaired mesh: problematic_repaired.stl
# Report: problematic_validation_report.json
```

### Report Generation

Generate detailed JSON report:

```bash
python3 06_trimesh_mesh_validator.py \
  --input model.stl \
  --report model_validation_report.json

# Creates detailed JSON with stats
```

### JSON Report Fields

```json
{
  "input_file": "servo.stl",
  "validation_timestamp": "2026-08-30T09:15:42",
  "validation_time_ms": 125,
  "mesh_info": {
    "file_size_bytes": 524288,
    "vertex_count_original": 8456,
    "vertex_count_repaired": 8456,
    "face_count_original": 16912,
    "face_count_repaired": 16912,
    "open_edges": 0
  },
  "validation_results": {
    "is_watertight": true,
    "has_degenerate_faces": false,
    "has_duplicate_vertices": false,
    "manifold": true,
    "self_intersecting": false
  },
  "geometry_stats": {
    "volume_mm3": 45280.5,
    "surface_area_mm2": 12456.3,
    "bounds": {
      "x_min": -50.2,
      "x_max": 50.2,
      "y_min": -32.1,
      "y_max": 32.1,
      "z_min": 0.0,
      "z_max": 84.3
    },
    "centroid": { "x": 0.0, "y": 0.0, "z": 42.1 },
    "edge_length_mean": 2.45,
    "edge_length_min": 0.02,
    "edge_length_max": 8.76
  },
  "repairs_applied": [],
  "status": "VALID"
}
```

### Common Issues & Solutions

| Issue | Symptom | Solution |
|-------|---------|----------|
| **Non-watertight** | Mesh has holes/gaps | Use `--auto-repair` flag |
| **Degenerate faces** | Very small/flat triangles | Use `--auto-repair` to remove |
| **Duplicate vertices** | Same position used multiple times | Auto-repair merges them |
| **Self-intersecting** | Faces overlap in space | Manually inspect and fix in CAD software |
| **Large file size** | Mesh too heavy to process | Reduce triangle count in original CAD |

### Practical Workflow

```bash
# 1. Check existing mesh
python3 06_trimesh_mesh_validator.py --input servo.stl

# If watertight = YES, you're done. If NO:

# 2. Auto-repair
python3 06_trimesh_mesh_validator.py \
  --input servo.stl \
  --auto-repair \
  --output servo_repaired.stl

# 3. Verify repair worked
python3 06_trimesh_mesh_validator.py --input servo_repaired.stl

# 4. Use repaired version for 3D printing
```

---

## Tool 3: Trimesh Mesh Merger

**Script:** `06_trimesh_merge_for_stl.py`

**Purpose:** Combine multiple STL meshes with spatial transforms into a single assembly STL.

### Transform Types

Three transform operations (applied in order: Rotate → Scale → Translate):

| Operation | Format | Example | Notes |
|-----------|--------|---------|-------|
| **Translation** | `[x, y, z]` (mm) | `[10, 5, 20]` | Moves mesh in 3D space |
| **Rotation** | `[roll, pitch, yaw]` (degrees) | `[90, 0, 0]` | Euler angles (ZYX order) |
| **Scaling** | float | `1.0` | 1.0 = no change, 2.0 = double size |

### Configuration File

Create or modify `merge_config_example.json`:

```json
{
  "merge_name": "servo_bracket_assembly",
  "output_file": "servo_bracket_assembly_merged.stl",
  "description": "Servo motor + support bracket assembly",
  "meshes": [
    {
      "file": "../Mechanical/feetech-STS3032.stl",
      "name": "servo_motor",
      "description": "Feetech STS3032 servo (no transform)",
      "transform": {
        "translate_mm": [0, 0, 0],
        "rotate_euler_deg": [0, 0, 0],
        "scale": 1.0
      }
    },
    {
      "file": "../Mechanical/small_bracket.stl",
      "name": "mounting_bracket",
      "description": "Support bracket (positioned relative to servo)",
      "transform": {
        "translate_mm": [10, 5, 20],
        "rotate_euler_deg": [90, 0, 0],
        "scale": 1.0
      }
    }
  ]
}
```

### CLI Usage

```bash
# Merge using config file
python3 06_trimesh_merge_for_stl.py --config merge_config_example.json

# Custom output file
python3 06_trimesh_merge_for_stl.py \
  --config merge_config_example.json \
  --output my_assembly.stl

# List help
python3 06_trimesh_merge_for_stl.py --help
```

### Output Files

For `merge_config_example.json`:

```
03_Parts/Mechanical/
├── servo_bracket_assembly_merged.stl      # Combined mesh
└── servo_bracket_assembly_merged_metadata.json  # Composition info
```

### Metadata JSON Example

```json
{
  "merge_name": "servo_bracket_assembly",
  "merge_timestamp": "2026-08-30T09:15:42.123456",
  "merge_time_ms": 312,
  "configuration_file": "merge_config_example.json",
  "meshes": [
    {
      "name": "servo_motor",
      "file": "../Mechanical/feetech-STS3032.stl",
      "vertices_original": 8456,
      "vertices_in_merge": 8456,
      "transform": {
        "translate_mm": [0, 0, 0],
        "rotate_euler_deg": [0, 0, 0],
        "scale": 1.0
      }
    },
    {
      "name": "mounting_bracket",
      "file": "../Mechanical/small_bracket.stl",
      "vertices_original": 1024,
      "vertices_in_merge": 1024,
      "transform": {
        "translate_mm": [10, 5, 20],
        "rotate_euler_deg": [90, 0, 0],
        "scale": 1.0
      }
    }
  ],
  "merged_geometry": {
    "total_vertices": 9480,
    "total_faces": 18912,
    "volume_mm3": 48520.8,
    "surface_area_mm2": 14256.5,
    "bounds": {
      "x_min": -50.2,
      "x_max": 100.0,
      "y_min": -32.1,
      "y_max": 80.0,
      "z_min": -10.0,
      "z_max": 94.3
    }
  },
  "output_file": "servo_bracket_assembly_merged.stl",
  "output_file_size_bytes": 632144,
  "status": "SUCCESS"
}
```

### Transform Order Explained

Transforms apply in fixed order: **Rotation → Scaling → Translation**

Example:
```json
{
  "rotate_euler_deg": [90, 0, 0],    // Step 1: Rotate 90° around X
  "scale": 2.0,                      // Step 2: Double size
  "translate_mm": [10, 5, 20]        // Step 3: Move to position
}
```

This is equivalent to:
1. Rotate the mesh 90° around its local X axis
2. Scale it 2x larger
3. Move it +10mm X, +5mm Y, +20mm Z from origin

### Practical Workflows

**Workflow 1: Servo + Bracket Assembly**

```bash
# 1. Validate servo mesh
python3 06_trimesh_mesh_validator.py --input servo.stl

# 2. Generate bracket (from Tool 2)
python3 06_cadquery_parametric_brackets.py \
  --type simple \
  --length 100 --width 80 --thickness 10 \
  --output support_bracket

# 3. Create merge config pointing to both STLs
# Edit merge_config_example.json with paths

# 4. Merge assemblies
python3 06_trimesh_merge_for_stl.py --config merge_config_example.json

# 5. Validate merged assembly
python3 06_trimesh_mesh_validator.py --input servo_bracket_assembly_merged.stl
```

**Workflow 2: Multi-Part Model**

```bash
# Create config with 3+ meshes
cat > multi_part_config.json << 'EOF'
{
  "merge_name": "complete_mechanism",
  "output_file": "mechanism_assembled.stl",
  "meshes": [
    {"file": "base_plate.stl", "name": "base", "transform": {...}},
    {"file": "arm.stl", "name": "arm", "transform": {...}},
    {"file": "joint.stl", "name": "joint", "transform": {...}},
    {"file": "servo.stl", "name": "servo", "transform": {...}}
  ]
}
EOF

python3 06_trimesh_merge_for_stl.py --config multi_part_config.json
```

---

## Workflow Examples

### Example 1: Single Bracket for Servo Mount

**Scenario:** You need a simple mounting bracket for a servo motor.

```bash
# Step 1: Generate bracket
python3 06_cadquery_parametric_brackets.py \
  --type simple \
  --length 100 \
  --width 80 \
  --thickness 10 \
  --hole-diameter 8 \
  --fillet-radius 3 \
  --output servo_mount

# Step 2: Check output
ls -lh 03_Parts/Mechanical/servo_mount.*
# servo_mount.step (50 KB) - Import into FreeCAD for further design
# servo_mount.stl (32 KB) - Send to 3D printer

# Done! The bracket is ready for CAD import or 3D printing.
```

### Example 2: Servo + Bracket Assembly

**Scenario:** Combine servo motor STL with a generated bracket into single assembly.

```bash
# Step 1: Validate servo mesh
python3 06_trimesh_mesh_validator.py --input ../Mechanical/servo.stl
# Output: watertight = YES, ready to use

# Step 2: Generate bracket
python3 06_cadquery_parametric_brackets.py \
  --type simple \
  --length 100 --width 80 --thickness 10

# Step 3: Create merge config
cat > servo_bracket_merge.json << 'EOF'
{
  "merge_name": "servo_with_bracket",
  "output_file": "servo_with_bracket.stl",
  "meshes": [
    {
      "file": "../Mechanical/servo.stl",
      "name": "servo",
      "transform": {
        "translate_mm": [0, 0, 0],
        "rotate_euler_deg": [0, 0, 0],
        "scale": 1.0
      }
    },
    {
      "file": "../Mechanical/simple_bracket.stl",
      "name": "bracket",
      "transform": {
        "translate_mm": [10, 5, 20],
        "rotate_euler_deg": [0, 90, 0],
        "scale": 1.0
      }
    }
  ]
}
EOF

# Step 4: Merge assemblies
python3 06_trimesh_merge_for_stl.py --config servo_bracket_merge.json

# Step 5: Validate merged result
python3 06_trimesh_mesh_validator.py --input servo_with_bracket.stl

# Result: servo_with_bracket.stl ready for 3D printing or simulation
```

### Example 3: Batch Bracket Production

**Scenario:** Generate 3 bracket sizes for inventory/options.

```bash
# Use provided bracket_configs.json with small/medium/large templates
python3 06_cadquery_parametric_brackets.py --config bracket_configs.json

# Output in 03_Parts/Mechanical/:
# ✓ small_bracket.step, small_bracket.stl
# ✓ medium_bracket.step, medium_bracket.stl
# ✓ large_bracket.step, large_bracket.stl

# All STLs ready for 3D printing, all STEP files ready for CAD import
```

---

## Testing & Validation

### Run Unit Tests

```bash
python3 test_06_phase6_tooling.py

# Output:
# ======== Phase 6 Tooling Tests ========
# 
# Tool 1: Bracket Generator
#   [PASS] CadQuery import
#   [PASS] Simple bracket generation
#   [PASS] L-bracket generation
#   [PASS] Corner bracket generation
#   [PASS] Config file parsing
#   [PASS] STEP export
#   [PASS] STL export
#
# Tool 2: Mesh Validator
#   [PASS] Trimesh import
#   [PASS] Mesh loading
#   [PASS] Watertight detection
#   [PASS] Repair functionality
#   [PASS] JSON report generation
#
# Tool 3: Mesh Merger
#   [PASS] Multi-mesh loading
#   [PASS] Transform application
#   [PASS] Merge composition
#   [PASS] Assembly export
#   [PASS] Metadata generation
#
# Results: 15/15 PASSED
# Report: test_06_results.json
```

### Interpret Test Results

Check `test_06_results.json`:

```json
{
  "test_timestamp": "2026-08-30T09:15:42",
  "total_tests": 15,
  "passed": 15,
  "failed": 0,
  "skipped": 0,
  "results": {
    "bracket_generator": {
      "tests": 7,
      "passed": 7,
      "status": "PASS"
    },
    "mesh_validator": {
      "tests": 5,
      "passed": 5,
      "status": "PASS"
    },
    "mesh_merger": {
      "tests": 3,
      "passed": 3,
      "status": "PASS"
    }
  }
}
```

---

## Integration with Phases 1-5

### Phase 1: STL to STEP Conversion

**Use Case:** Phase 1 outputs servo.step; Phase 6 validates the servo.stl source mesh.

```bash
# Phase 1 generated servo STEP file
# Phase 6 validates and can repair the source servo.stl

python3 06_trimesh_mesh_validator.py --input servo.stl --auto-repair --output servo_repaired.stl
# This repaired.stl ensures quality for any Phase 1 conversions
```

### Phase 4: Assembly Integration

**Use Case:** Phase 4 exports plates_assembled_with_servo.stl; Phase 6 validates and merges alternatives.

```bash
# Phase 4 created: plates_assembled_with_servo.stl
# Phase 6 can:
# 1. Validate the export quality
python3 06_trimesh_mesh_validator.py --input plates_assembled_with_servo.stl

# 2. Or generate alternative brackets and merge them
python3 06_cadquery_parametric_brackets.py --type corner --length 150 --width 120 --thickness 12
python3 06_trimesh_merge_for_stl.py --config merge_config_example.json
```

### Phase 6 as Independent Workflow

Phase 6 is **standalone** and doesn't require earlier phases:

```bash
# Generate brackets and assemblies without running Phases 1-4
python3 06_cadquery_parametric_brackets.py --config bracket_configs.json
python3 06_trimesh_merge_for_stl.py --config merge_config_example.json

# Useful for:
# - Parametric bracket design workflow
# - Batch part production
# - Mesh repair/validation pipelines
```

---

## Troubleshooting

### Issue: "Mesh not watertight" Error

**Symptom:**
```
Mesh validation results:
  Watertight: NO
  Status: INVALID (not suitable for 3D printing)
```

**Solution:**
```bash
# Use auto-repair
python3 06_trimesh_mesh_validator.py \
  --input problematic.stl \
  --auto-repair \
  --output problematic_repaired.stl

# If still not watertight:
# 1. Check original CAD file for gaps/holes
# 2. Regenerate STL from CAD with finer mesh settings
# 3. Use external mesh repair tool (Meshmixer, Netfabb)
```

### Issue: "CadQuery import error"

**Symptom:**
```
ERROR: CadQuery not available. Install with: uv sync
ModuleNotFoundError: No module named 'cadquery'
```

**Solution:**
```bash
# Ensure mamba environment activated
mamba activate pendulum-phase6

# Verify environment
python3 -c "import cadquery; print(cadquery.__version__)"

# If still fails, sync dependencies
cd /path/to/inverted-pendulum-project
uv sync
```

### Issue: "STEP export fails" or "File not found"

**Symptom:**
```
ERROR: Failed to export STEP
FileNotFoundError: [Errno 2] No such file or directory
```

**Solution:**
```bash
# Check output directory exists
mkdir -p 03_Parts/Mechanical

# Check permissions
ls -ld 03_Parts/Mechanical  # Should be writable

# Check disk space
df -h | grep "/$"  # Need at least 100MB free

# Try again with explicit path
python3 06_cadquery_parametric_brackets.py \
  --type simple \
  --length 100 --width 80 --thickness 10 \
  --output /absolute/path/bracket
```

### Issue: "Transform looks wrong" in merged mesh

**Symptom:**
```
Merged assembly has wrong orientation or position
Bracket is rotated 90° when it should be flat
```

**Solution:**
1. **Verify rotation order** — Euler angles are ZYX order
   ```json
   "rotate_euler_deg": [roll_x, pitch_y, yaw_z]
   // [90, 0, 0] rotates around X axis
   // [0, 90, 0] rotates around Y axis
   // [0, 0, 90] rotates around Z axis
   ```

2. **Test with single mesh**
   ```bash
   # Create simple test config with one mesh only
   cat > test_transform.json << 'EOF'
   {"merge_name": "test", "meshes": [{"file": "mesh.stl", "name": "test", "transform": {"translate_mm": [0, 0, 0], "rotate_euler_deg": [90, 0, 0], "scale": 1.0}}]}
   EOF
   python3 06_trimesh_merge_for_stl.py --config test_transform.json
   # View in viewer to verify
   ```

3. **Adjust incrementally**
   ```json
   // Start with no rotation
   "rotate_euler_deg": [0, 0, 0]
   // Apply one axis at a time
   // [90, 0, 0] → view → [0, 90, 0] → view
   ```

### Issue: Large file size or slow processing

**Symptom:**
```
Bracket generation took 5+ seconds
Merged mesh is >100 MB
```

**Solution:**
```bash
# For bracket generation: Reduce fillet radius
python3 06_cadquery_parametric_brackets.py \
  --type simple \
  --length 100 --width 80 --thickness 10 \
  --fillet-radius 0  # No fillet = faster

# For mesh merging: Reduce triangle count in source meshes
# (regenerate STLs in CAD with coarser mesh settings)

# Check mesh details
python3 06_trimesh_mesh_validator.py --input large_mesh.stl
# Look at vertex_count and face_count
```

---

## Performance & Optimization

### Bracket Generation Performance

| Bracket Type | Typical Time | Typical File Size |
|--------------|--------------|-------------------|
| Simple (no fillet) | 50-100 ms | 25-35 KB |
| Simple (with fillet) | 150-250 ms | 35-50 KB |
| L-bracket | 200-350 ms | 45-70 KB |
| Corner bracket | 300-500 ms | 60-90 KB |

**Optimization Tips:**
- Disable fillet for faster generation: `--fillet-radius 0`
- Use defaults for common sizes
- Batch generate with config file

### Mesh Validation Performance

| Mesh Size (vertices) | Typical Time | Report Output |
|----------------------|--------------|----------------|
| < 5,000 | 10-20 ms | Small JSON |
| 5,000-50,000 | 50-200 ms | Medium JSON |
| 50,000-500,000 | 200-1,000 ms | Large JSON |
| > 500,000 | 1-5 seconds | Very large JSON |

**Optimization Tips:**
- Validation is faster than repair (no mesh modification)
- Repair with `--auto-repair` adds 10-50% overhead
- Monitor disk space if exporting large repaired meshes

### Mesh Merger Performance

| Number of Meshes | Total Vertices | Typical Time |
|------------------|----------------|--------------|
| 2 | < 20,000 | 50-150 ms |
| 3 | 20,000-50,000 | 150-400 ms |
| 5 | 50,000-100,000 | 400-800 ms |

**Optimization Tips:**
- Process in sequence (don't parallelize—trimesh not thread-safe)
- Reduce triangle counts in source meshes if possible
- Use scaling to adjust part sizes instead of generating new geometry

---

## References & Links

### Official Documentation

- [CadQuery Documentation](https://cadquery.readthedocs.io/)
  - Parametric part design, feature generation, export formats

- [Trimesh Documentation](https://trimesh.org/)
  - Mesh validation, repair, composition, geometry analysis

### File Formats

- [STEP Format](https://en.wikipedia.org/wiki/ISO_10303-21)
  - ISO standard CAD exchange format, preserves design intent

- [STL Format](https://en.wikipedia.org/wiki/STL_(file_format))
  - Stereolithography format, triangle mesh representation

### Related Tools

- [FreeCAD](https://www.freecadweb.org/) — Import STEP, edit, re-export
- [Fusion 360](https://www.autodesk.com/products/fusion-360/) — Parametric CAD, STEP support
- [Meshmixer](https://www.meshmixer.com/) — Advanced mesh repair and manipulation
- [PrusaSlicer](https://www.prusa3d.com/en/page/prusaslicer_en/) — STL preparation, 3D printing

### Euler Angle References

- [Euler Angles (ZYX Order)](https://en.wikipedia.org/wiki/Euler_angles)
  - Convention: Roll (X), Pitch (Y), Yaw (Z)
  - Order: Z rotation first, then Y, then X

### Environment Management

- [Mamba Documentation](https://mamba.readthedocs.io/)
- [Python Virtual Environments](https://docs.python.org/3/tutorial/venv.html)

---

## Appendix: Common Workflows Summary

### Single Bracket Generation
```bash
python3 06_cadquery_parametric_brackets.py --type simple --length 100 --width 80 --thickness 10
```

### Batch Bracket Production
```bash
python3 06_cadquery_parametric_brackets.py --config bracket_configs.json
```

### Mesh Validation Only
```bash
python3 06_trimesh_mesh_validator.py --input model.stl
```

### Mesh Repair & Export
```bash
python3 06_trimesh_mesh_validator.py --input model.stl --auto-repair --output repaired.stl
```

### Assembly Merge
```bash
python3 06_trimesh_merge_for_stl.py --config merge_config_example.json
```

### Complete Workflow
```bash
# 1. Generate bracket
python3 06_cadquery_parametric_brackets.py --type simple --length 100 --width 80 --thickness 10

# 2. Validate servo mesh
python3 06_trimesh_mesh_validator.py --input servo.stl

# 3. Merge into assembly
python3 06_trimesh_merge_for_stl.py --config merge_config_example.json

# 4. Validate final assembly
python3 06_trimesh_mesh_validator.py --input servo_with_bracket.stl
```

---

**Last Updated:** 2026-08-30
**Phase 6 Status:** Complete (all 3 tools operational)
**Environment:** pendulum-phase6 mamba environment
