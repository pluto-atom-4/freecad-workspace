#!/usr/bin/env python3
"""
Stage 1: Body + Wheel Geometry (Issue #9)

Creates the chassis (`Base_Link`) and drive wheels (`Wheel_Left`/`Wheel_Right`)
as simple FreeCAD primitives (`Part.makeBox`/`Part.makeCylinder`), sized from
Stage 0's `robot_parameters.yaml` (via `02_Design_Inputs/robot_parameters.py`),
and reuses the existing 3-plate pendulum linkage + servo
(`plates_servo_assembled.FCStd`'s `PlateStack`/`STS3032_Mount` `App::Part`
groups, from Issue #3) as a `Pendulum_Link` subassembly.

Scope note: this script is Stage 1 ONLY. It does not configure joints
(Stage 2), compute mass/inertia in SI units (Stage 3), or export URDF
(Stage 4) -- see Issue #9's consolidated plan.

Link/copy decision (see `_copy_pendulum_link()` below for the mechanics):
this script COPIES `PlateStack`'s and `STS3032_Mount`'s objects (Shape/Mesh
data + Placement) into the new output document rather than using `App::Link`
to reference them externally. Reasoning:
  - The new document (`robot_body_wheels.FCStd`) is meant to be a
    self-contained input for Stage 2 (joint configuration) and Stage 3
    (mass/inertia) -- those scripts should not need `plates_servo_assembled.
    FCStd` to still exist at a stable relative path just to recompute this
    document's geometry.
  - `App::Link` cross-document references also require both documents to
    stay open simultaneously for shape access in some FreeCAD operations
    (tessellation, mass properties); a plain in-document copy avoids that
    entirely.
  - The source objects are simple `Part::Feature` (plates) and `Mesh::Feature`
    (servo meshes) -- both support a straightforward `.Shape.copy()` /
    `.Mesh.copy()` + `.Placement` copy into a freshly created object of the
    same type, with no parametric feature history to lose.
The cost of copying (vs. linking) is that `plates_servo_assembled.FCStd` and
`robot_body_wheels.FCStd` are independent snapshots from this point on -- if
the source document changes later, this script must be re-run to pick up the
change. That tradeoff favors a stable Stage 2+ input over always-fresh
geometry.

Object names created (exact, case-sensitive -- Stage 2's joint config and
Stage 4's URDF export consume these verbatim, per `robot_parameters.yaml`'s
`links:` mapping):
  - Base_Link      (Part::Feature, chassis box)
  - Wheel_Left      (Part::Feature, cylinder)
  - Wheel_Right     (Part::Feature, cylinder)
  - Pendulum_Link   (App::Part, containing PlateStack + STS3032_Mount
                     sub-groups, copied from the source document)

Usage:
    # This build of freecadcmd (1.1.3) does not set __name__ == "__main__"
    # for a plain positional or --python script argument -- the guard below
    # never fires that way and the process exits 0 having done nothing
    # (verified empirically). Use -c "exec(...)" instead:
    "${FREECAD_BIN:-freecadcmd}" -c "exec(open('07_create_body_and_wheels.py').read())"
    # or, pinning a specific FreeCAD build:
    FREECAD_BIN=~/.local/opt/freecad-1.1.3/usr/bin/freecadcmd
    "$FREECAD_BIN" -c "exec(open('07_create_body_and_wheels.py').read())"

Known limitation -- live GUI runs only (not headless): opening/recomputing
this script's objects under a live GUI (e.g. driven through the FreeCAD MCP
bridge's execute_python, as opposed to a true headless freecadcmd run) makes
FreeCAD's view providers auto-tessellate each shape for on-screen display,
which can shrink Shape.BoundBox reads by a small chordal-deviation amount --
observed as a 70.00mm wheel diameter reading ~69.90mm, enough to fail this
script's own tight dimensional validation. This is a different contamination
source than the one _tessellate_triangle_count() below guards against (that
one is self-inflicted by this script's own tessellation call; this one comes
from the GUI itself and is outside this script's control). Not fixed --
validate dimensions against a true headless run; see root CLAUDE.md's
"FreeCAD Live Bridge -- Known Limitations" section.

Redesign follow-up (Issue #9, live-bridge probe after PR #52) -- IN PROGRESS,
ONE-SIDED: Pendulum_Link is tilted 90 deg (PENDULUM_LINK_TILT_DEG) so its
plate faces stand parallel to the wheel discs, Wheel_Left is re-mounted on
Pendulum_Link's Bottom_Plate mounting hole instead of the old
body-centerline placement (see _mount_wheel_on_pendulum_plate()), and
Base_Link is now a flat plate (BASE_LINK_PLATE_THICKNESS_MM thick,
BASE_LINK_LENGTH_MM long) instead of a solid chassis.height_mm-tall box
(see build_base_link()), and repositioned near Bottom_Plate's top face --
a deck plate at the pivot/servo level -- instead of floating near the
ground at CHASSIS_GROUND_CLEARANCE_MM (see
_position_base_link_under_pendulum(); a human fine-tuned this live in the
FreeCAD GUI, ported back here). Wheel_Right
is NOT part of this redesign yet -- it has no mirrored Pendulum_Link_Right
to mount on, so it keeps its original body-centerline geometry and is
hidden by _set_default_visibility() rather than shown stale/disconnected.
Expect validate()'s track-width-symmetry, wheel-Z-match, and
no-interpenetration checks to FAIL until the mirrored right side is
implemented, and "Pendulum_Link positioned above Base_Link" to FAIL too --
Base_Link now sits mid-stack (flush with Bottom_Plate's top), not below
the whole assembly, so that check's original "chassis below pendulum"
assumption no longer holds by design. All known, transitional states, not
bugs; see
root CLAUDE.md / DESIGN.md for the open decisions.

Output:
    - robot_body_wheels.FCStd (new document, does not modify the source file)
    - 07_body_wheels_metadata.json (per-link dims/placement/volume, triangle
      counts, and validation results)
"""

import sys
import json
import functools
from pathlib import Path
from dataclasses import dataclass, field, asdict
from typing import Any, Dict, List, Optional
from datetime import datetime

# freecadcmd's embedded console buffers plain print() output such that it
# can be lost entirely if the process exits (even cleanly via sys.exit)
# before the buffer is flushed -- observed empirically running this script
# headlessly with stdout redirected to a file. Force every print() in this
# module to flush immediately so console/log output is reliable.
print = functools.partial(print, flush=True)  # noqa: A001

try:
    import FreeCAD as App
    import Part
    import Mesh
    import MeshPart
    from FreeCAD import Vector, Placement, Rotation
except ImportError:
    print("ERROR: FreeCAD Python modules not available.")
    print("This script must be run with FreeCAD's Python interpreter:")
    print("  freecadcmd --python 07_create_body_and_wheels.py")
    sys.exit(1)


# __file__ is not defined when this script is run via
# `freecadcmd -c "exec(open('07_create_body_and_wheels.py').read())"`
# (see 02_position_servo.py / 04_export_assembly_merged.py for the same
# fallback convention).
try:
    SCRIPT_DIR = Path(__file__).resolve().parent
except NameError:
    SCRIPT_DIR = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"

# Stage 0's design-input loader lives in a sibling directory
# (02_Design_Inputs), not next to this script -- no existing script in this
# directory reaches across directories like this yet, so this follows the
# cleanest reasonable pattern (sys.path.insert relative to this file).
_DESIGN_INPUTS_DIR = SCRIPT_DIR.parent.parent / "02_Design_Inputs"
sys.path.insert(0, str(_DESIGN_INPUTS_DIR))

from robot_parameters import load_robot_parameters, RobotParameters  # noqa: E402

SOURCE_DOC_FILENAME = "plates_servo_assembled.FCStd"
OUTPUT_DOC_NAME = "robot_body_wheels"
OUTPUT_FCSTD_FILENAME = "robot_body_wheels.FCStd"
METADATA_FILENAME = "07_body_wheels_metadata.json"

# Tessellation deflection for triangle-count reporting -- matches the
# project's established 1.0mm visual-mesh convention (see
# 04_export_assembly_merged.py's STL_LINEAR_DEFLECTION/STL_ANGULAR_DEFLECTION).
TESSELLATION_LINEAR_DEFLECTION_MM = 1.0
TESSELLATION_ANGULAR_DEFLECTION_RAD = 0.5

# Simulation efficiency budget from Issue #9's acceptance criteria
# ("Minimize polygon count for simulation efficiency (<5000 triangles)").
# Applied here to the NEW primitive geometry this stage creates (chassis +
# wheels). The reused Pendulum_Link content's servo visual mesh (~38.6k
# facets, inherited from Issue #3's plates_servo_assembled.FCStd) is
# reported separately and excluded from this budget -- per Issue #9
# Decision #10, the servo's simulation <collision> geometry will be emitted
# as URDF primitives (box + cylinder) in Stage 4, not this mesh, so this
# high-fidelity mesh is a visual-only asset that never needs to fit the
# collision/simulation triangle budget.
NEW_PRIMITIVE_TRIANGLE_BUDGET = 5000

# Ground plane is Z=0. Chassis sits with some clearance above it; wheels'
# bottom edge touches Z=0 (wheel center height = wheel radius).
CHASSIS_GROUND_CLEARANCE_MM = 10.0

# Redesign follow-up (Issue #9, live-bridge probe after PR #52): Wheel_Left
# is re-mounted directly on Pendulum_Link's Bottom_Plate mounting hole
# (Edge27/Edge37/Edge38 -- three arcs of one real bolt hole, confirmed via
# their shared Curve.Center) instead of the old body-centerline + track_mm
# placement. Values below are empirically chosen and live-bridge-verified:
# distToShape against the whole PlateStack/STS3032_Mount compound was 0 at
# every radius tried down to 5mm at the hole's own center (the hole center
# sits INSIDE the plate-stack solid, not just close to its surface), so
# shrinking the wheel alone cannot clear the interference -- an additional
# lateral offset along the plate's local normal (global Y, given
# PENDULUM_LINK_TILT_DEG below) is required. offset=6mm measured exactly
# 3.0mm of clearance.
PENDULUM_LINK_TILT_DEG = 90.0
WHEEL_ON_PLATE_RADIUS_MM = 15.0
WHEEL_ON_PLATE_WIDTH_MM = 6.0
WHEEL_ON_PLATE_CLEARANCE_OFFSET_MM = 6.0
WHEEL_ON_PLATE_HOLE_EDGE = "Edge27"

# Redesign follow-up: Base_Link becomes a flat plate instead of a solid
# chassis box (thickness no longer chassis.height_mm) -- matches this
# project's existing Top_Plate/Middle_Plate/Bottom_Plate thickness
# convention (2.5mm, empirically confirmed via their own Shape.BoundBox).
BASE_LINK_PLATE_THICKNESS_MM = 2.5

# Redesign follow-up: Base_Link's length (X dimension) shrunk from
# robot_parameters.yaml's chassis.length_mm (120mm) to 40mm -- width_mm
# (Y dimension, 80mm) still comes from robot_parameters.yaml, unchanged.
BASE_LINK_LENGTH_MM = 40.0

# Redesign follow-up: Base_Link's final position, human-tuned live in the
# FreeCAD GUI (three separate passes) then ported back here -- X and Y are
# both fixed absolute values now (neither centered on Bottom_Plate
# anymore), and Z is nudged BASE_LINK_Z_NUDGE_MM below
# flush-with-Bottom_Plate's-top (a small negative value embeds it slightly
# into Bottom_Plate rather than leaving a hairline gap). See
# _position_base_link_under_pendulum().
BASE_LINK_X_POSITION_MM = 15.40
BASE_LINK_Y_POSITION_MM = 19.00
BASE_LINK_Z_NUDGE_MM = -0.2


@dataclass
class ValidationResult:
    check_name: str
    passed: bool
    details: str
    value: Optional[float] = None
    tolerance: Optional[float] = None

    def to_dict(self) -> dict:
        result = {"check": self.check_name, "passed": self.passed, "details": self.details}
        if self.value is not None:
            result["value"] = round(self.value, 4) if isinstance(self.value, float) else self.value
        if self.tolerance is not None:
            result["tolerance"] = self.tolerance
        return result


@dataclass
class LinkRecord:
    """Per-link record for the JSON metadata output."""
    name: str
    kind: str  # "primitive_box" | "primitive_cylinder" | "reused_subassembly"
    dimensions_mm: Dict[str, float] = field(default_factory=dict)
    placement: Dict[str, Any] = field(default_factory=dict)
    bounding_box_mm: Dict[str, float] = field(default_factory=dict)
    volume_mm3: Optional[float] = None
    target_mass_kg: Optional[float] = None
    triangle_count: Optional[int] = None
    notes: Optional[str] = None

    def to_dict(self) -> dict:
        return {k: v for k, v in asdict(self).items() if v is not None}


def _placement_to_dict(placement: "Placement") -> Dict[str, Any]:
    ypr = placement.Rotation.getYawPitchRoll()
    return {
        "position": {
            "x": round(placement.Base.x, 4),
            "y": round(placement.Base.y, 4),
            "z": round(placement.Base.z, 4),
        },
        "rotation_ypr_deg": {
            "yaw": round(ypr[0], 4),
            "pitch": round(ypr[1], 4),
            "roll": round(ypr[2], 4),
        },
    }


def _bbox_to_dict(bbox) -> Dict[str, float]:
    return {
        "x_min": round(bbox.XMin, 4),
        "x_max": round(bbox.XMax, 4),
        "y_min": round(bbox.YMin, 4),
        "y_max": round(bbox.YMax, 4),
        "z_min": round(bbox.ZMin, 4),
        "z_max": round(bbox.ZMax, 4),
    }


class BodyWheelsGenerator:
    """Builds Base_Link/Wheel_Left/Wheel_Right and reuses the pendulum
    linkage as Pendulum_Link in a new, self-contained output document."""

    def __init__(self) -> None:
        self.params: Optional[RobotParameters] = None
        self.source_doc = None
        self.output_doc = None
        self.links: List[LinkRecord] = []
        self.validations: List[ValidationResult] = []
        self.reused_mesh_facet_counts: Dict[str, int] = {}
        self.new_primitive_triangle_count = 0
        self.total_volume_mm3 = 0.0

    # ---------------------------------------------------------------
    # Setup
    # ---------------------------------------------------------------

    def load_parameters(self) -> bool:
        try:
            self.params = load_robot_parameters()
            print(f"✓ Loaded robot_parameters.yaml (status={self.params.status})")
            print(f"  chassis: {self.params.chassis.length_mm}x{self.params.chassis.width_mm}x"
                  f"{self.params.chassis.height_mm} mm")
            print(f"  wheel:   dia={self.params.wheel.diameter_mm} mm, "
                  f"width={self.params.wheel.width_mm} mm, track={self.params.wheel.track_mm} mm")
            return True
        except Exception as e:
            print(f"ERROR loading robot_parameters.yaml: {e}")
            return False

    def open_source_document(self) -> bool:
        try:
            source_path = SCRIPT_DIR / SOURCE_DOC_FILENAME
            if not source_path.exists():
                print(f"ERROR: Source document not found: {source_path}")
                return False
            self.source_doc = App.openDocument(str(source_path))
            print(f"✓ Opened source document: {source_path.name}")
            return True
        except Exception as e:
            print(f"ERROR opening source document: {e}")
            return False

    def create_output_document(self) -> bool:
        try:
            self.output_doc = App.newDocument(OUTPUT_DOC_NAME)
            print(f"✓ Created output document: {OUTPUT_DOC_NAME}")
            return True
        except Exception as e:
            print(f"ERROR creating output document: {e}")
            return False

    # ---------------------------------------------------------------
    # Geometry: chassis + wheels
    # ---------------------------------------------------------------

    def build_base_link(self) -> bool:
        """Create Base_Link: a flat plate (BASE_LINK_PLATE_THICKNESS_MM
        thick, BASE_LINK_LENGTH_MM long), width from robot_parameters.yaml's
        chassis.width_mm, centered on X/Y at the origin, bottom face
        CHASSIS_GROUND_CLEARANCE_MM above the Z=0 ground plane.

        Redesign follow-up (Issue #9, live-bridge probe after PR #52):
        originally a solid chassis.length_mm x chassis.width_mm x
        chassis.height_mm box (a human review found it didn't match the
        "enclosure" the plan called for). Now a thin plate instead, and
        shrunk from chassis.length_mm (120mm) down to BASE_LINK_LENGTH_MM
        (40mm) -- width still from robot_parameters.yaml, unchanged."""
        try:
            chassis = self.params.chassis
            length, width = BASE_LINK_LENGTH_MM, chassis.width_mm
            thickness = BASE_LINK_PLATE_THICKNESS_MM

            bottom_z = CHASSIS_GROUND_CLEARANCE_MM
            base_pnt = Vector(-length / 2.0, -width / 2.0, bottom_z)
            shape = Part.makeBox(length, width, thickness, base_pnt)

            obj = self.output_doc.addObject("Part::Feature", "Base_Link")
            obj.Label = "Base_Link"
            obj.Shape = shape

            self._chassis_top_z = bottom_z + thickness
            self._chassis_bottom_z = bottom_z

            triangles = self._tessellate_triangle_count(shape)
            self.new_primitive_triangle_count += triangles
            self.total_volume_mm3 += shape.Volume

            self.links.append(LinkRecord(
                name="Base_Link",
                kind="primitive_plate",
                dimensions_mm={"length_mm": length, "width_mm": width, "thickness_mm": thickness},
                placement=_placement_to_dict(obj.Placement),
                bounding_box_mm=_bbox_to_dict(shape.BoundBox),
                volume_mm3=round(shape.Volume, 4),
                target_mass_kg=self.params.target_mass_for_link_kg("Base_Link"),
                triangle_count=triangles,
                notes=(
                    "Redesigned as a flat plate (was a solid "
                    f"chassis.length_mm={chassis.length_mm}mm x "
                    f"chassis.height_mm={chassis.height_mm}mm box) -- "
                    "length is now BASE_LINK_LENGTH_MM and thickness is now "
                    "BASE_LINK_PLATE_THICKNESS_MM, not robot_parameters."
                    "yaml's chassis.length_mm/chassis.height_mm."
                ),
            ))

            print(f"✓ Base_Link: {length}x{width}x{thickness} mm plate, "
                  f"bottom Z={bottom_z} mm, volume={shape.Volume:.2f} mm^3, "
                  f"{triangles} triangles")
            return True
        except Exception as e:
            print(f"ERROR building Base_Link: {e}")
            return False

    def build_wheel(self, name: str, side_sign: int) -> bool:
        """Create Wheel_Left/Wheel_Right: a cylinder, axis along Y (horizontal
        axle), positioned at +/-track/2 from the centerline, resting on the
        Z=0 ground plane (wheel center height = radius)."""
        try:
            wheel = self.params.wheel
            radius = wheel.diameter_mm / 2.0
            track_half = wheel.track_mm / 2.0

            center_y = side_sign * track_half
            base_y = center_y - wheel.width_mm / 2.0
            base_pnt = Vector(0.0, base_y, radius)
            axis_dir = Vector(0.0, 1.0, 0.0)

            shape = Part.makeCylinder(radius, wheel.width_mm, base_pnt, axis_dir)

            obj = self.output_doc.addObject("Part::Feature", name)
            obj.Label = name
            obj.Shape = shape

            triangles = self._tessellate_triangle_count(shape)
            self.new_primitive_triangle_count += triangles
            self.total_volume_mm3 += shape.Volume

            self.links.append(LinkRecord(
                name=name,
                kind="primitive_cylinder",
                dimensions_mm={
                    "diameter_mm": wheel.diameter_mm,
                    "radius_mm": radius,
                    "width_mm": wheel.width_mm,
                },
                placement=_placement_to_dict(obj.Placement),
                bounding_box_mm=_bbox_to_dict(shape.BoundBox),
                volume_mm3=round(shape.Volume, 4),
                target_mass_kg=self.params.target_mass_for_link_kg(name),
                triangle_count=triangles,
                notes=f"center_y_mm={center_y}, track_mm={wheel.track_mm}",
            ))

            print(f"✓ {name}: dia={wheel.diameter_mm} mm, width={wheel.width_mm} mm, "
                  f"center_y={center_y} mm, volume={shape.Volume:.2f} mm^3, "
                  f"{triangles} triangles")
            return True
        except Exception as e:
            print(f"ERROR building {name}: {e}")
            return False

    def _tessellate_triangle_count(self, shape) -> int:
        """Tessellate a COPY of the shape, not the shape itself.

        MeshPart.meshFromShape() caches triangulation data on the underlying
        TopoDS_Shape; if run against the same shape object assigned to
        obj.Shape, later reads of obj.Shape.BoundBox can return a
        triangulation-based (slightly smaller, chordal-deviation-shrunk)
        bounding box instead of the exact analytic one -- observed
        empirically as a ~0.5mm-short wheel diameter in this script's own
        dimension validation once tessellation ran first. Tessellating a
        copy avoids contaminating the shape actually stored on the object.
        """
        try:
            mesh = MeshPart.meshFromShape(
                Shape=shape.copy(),
                LinearDeflection=TESSELLATION_LINEAR_DEFLECTION_MM,
                AngularDeflection=TESSELLATION_ANGULAR_DEFLECTION_RAD,
            )
            return len(mesh.Facets)
        except Exception as e:
            print(f"  ⚠ WARNING: Could not tessellate shape for triangle count: {e}")
            return 0

    # ---------------------------------------------------------------
    # Pendulum_Link: reuse PlateStack + STS3032_Mount from the source doc
    # ---------------------------------------------------------------

    def build_pendulum_link(self) -> bool:
        """Copy PlateStack's plates and STS3032_Mount's servo meshes into a
        new Pendulum_Link App::Part in the output document, preserving their
        original relative placements to each other, then offset the whole
        container to sit above Base_Link (pivot_height_mm above the chassis
        top, centered in X/Y over the chassis footprint)."""
        try:
            source_plate_stack = self.source_doc.getObject("PlateStack")
            source_sts_mount = self.source_doc.getObject("STS3032_Mount")

            if source_plate_stack is None or source_sts_mount is None:
                print("ERROR: PlateStack or STS3032_Mount not found in source document")
                return False

            pendulum_link = self.output_doc.addObject("App::Part", "Pendulum_Link")
            pendulum_link.Label = "Pendulum_Link"

            plate_stack = self.output_doc.addObject("App::Part", "PlateStack")
            plate_stack.Label = "PlateStack"
            pendulum_link.addObject(plate_stack)

            sts_mount = self.output_doc.addObject("App::Part", "STS3032_Mount")
            sts_mount.Label = "STS3032_Mount"
            pendulum_link.addObject(sts_mount)

            # Copy the three plates (Part::Feature: Shape + Placement)
            plate_children = []
            for child in source_plate_stack.Group:
                if not hasattr(child, "Shape"):
                    continue
                new_obj = self.output_doc.addObject("Part::Feature", child.Name)
                new_obj.Label = child.Label
                new_obj.Shape = child.Shape.copy()
                new_obj.Placement = Placement(child.Placement)
                plate_stack.addObject(new_obj)
                plate_children.append(new_obj)
                print(f"  ✓ Copied {child.Name} into PlateStack")

            # Copy the servo meshes (Mesh::Feature: Mesh + Placement)
            mesh_children = []
            for child in source_sts_mount.Group:
                if not hasattr(child, "Mesh"):
                    continue
                new_obj = self.output_doc.addObject("Mesh::Feature", child.Name)
                new_obj.Label = child.Label
                new_obj.Mesh = child.Mesh.copy()
                new_obj.Placement = Placement(child.Placement)
                sts_mount.addObject(new_obj)
                mesh_children.append(new_obj)
                facets = new_obj.Mesh.CountFacets
                self.reused_mesh_facet_counts[child.Name] = facets
                print(f"  ✓ Copied {child.Name} into STS3032_Mount ({facets} facets)")

            if not plate_children:
                print("ERROR: No plate objects copied from PlateStack")
                return False

            # Combined bbox of the copied plates (pre-offset), used to center
            # Pendulum_Link over Base_Link and lift it pivot_height_mm above
            # the chassis top.
            plate_bbox = plate_children[0].Shape.BoundBox
            for obj in plate_children[1:]:
                plate_bbox.add(obj.Shape.BoundBox)

            plate_center_x = (plate_bbox.XMin + plate_bbox.XMax) / 2.0
            plate_center_y = (plate_bbox.YMin + plate_bbox.YMax) / 2.0

            pivot_height = self.params.pendulum.pivot_height_mm
            offset_x = -plate_center_x
            offset_y = -plate_center_y
            offset_z = (self._chassis_top_z + pivot_height) - plate_bbox.ZMin

            pendulum_link.Placement = Placement(
                Vector(offset_x, offset_y, offset_z), Rotation(Vector(0, 0, 1), 0)
            )

            # Redesign follow-up (Issue #9, live-bridge probe): tilt the
            # whole copied plate stack PENDULUM_LINK_TILT_DEG about local X,
            # in place (same Base, rotation only) -- plate faces go from
            # horizontal to parallel-with-the-wheel-discs. Wheel_Left's new
            # hole-mount position (_mount_wheel_on_pendulum_plate() below)
            # depends on this tilt already being applied.
            tilt = Rotation(Vector(1, 0, 0), PENDULUM_LINK_TILT_DEG)
            pendulum_link.Placement = Placement(
                pendulum_link.Placement.Base,
                tilt.multiply(pendulum_link.Placement.Rotation),
            )

            self.output_doc.recompute()

            # `plate_bbox` above already reflects each plate's own Placement
            # (Part::Feature.Shape auto-applies its object's Placement), but
            # NOT the PlateStack/Pendulum_Link container Placements above it
            # (App::Part-nested container placements don't fold into a
            # child's .Shape). Compose the container offset manually to get
            # the true global bbox for reporting.
            plate_bbox_global = plate_bbox.transformed(pendulum_link.Placement.toMatrix())

            volume = sum(obj.Shape.Volume for obj in plate_children)
            self.total_volume_mm3 += volume
            for name, facets in self.reused_mesh_facet_counts.items():
                mesh_obj = self.output_doc.getObject(name)
                try:
                    self.total_volume_mm3 += mesh_obj.Mesh.Volume
                except Exception:
                    pass  # not all meshes are guaranteed watertight

            self.links.append(LinkRecord(
                name="Pendulum_Link",
                kind="reused_subassembly",
                dimensions_mm={
                    "arm_length_mm": self.params.pendulum.arm_length_mm,
                    "pivot_height_mm": pivot_height,
                },
                placement=_placement_to_dict(pendulum_link.Placement),
                bounding_box_mm=_bbox_to_dict(plate_bbox_global),
                volume_mm3=round(volume, 4),
                target_mass_kg=self.params.target_mass_for_link_kg("Pendulum_Link"),
                notes=(
                    "Copied (not linked) from plates_servo_assembled.FCStd's "
                    "PlateStack + STS3032_Mount groups -- see module "
                    "docstring for the link-vs-copy rationale. Bounding box "
                    "and volume above cover the copied PlateStack plates "
                    "only (Part::Feature shapes); the copied STS3032_Mount "
                    "servo meshes retain their original relative placement "
                    "from the source document (see "
                    "reused_pendulum_mesh_facet_counts in geometry_stats for "
                    "their facet counts) and are not included in this "
                    "link's volume/bbox figures since Mesh::Feature "
                    "geometry isn't uniformly watertight/volumetric."
                ),
            ))

            print(f"✓ Pendulum_Link: offset=({offset_x:.2f}, {offset_y:.2f}, {offset_z:.2f}) mm, "
                  f"plates volume={volume:.2f} mm^3")
            return True
        except Exception as e:
            print(f"ERROR building Pendulum_Link: {e}")
            import traceback
            traceback.print_exc()
            return False

    def _mount_wheel_on_pendulum_plate(self, wheel_name: str) -> bool:
        """Redesign follow-up (Issue #9, live-bridge probe after PR #52):
        re-mount `wheel_name` on Pendulum_Link's Bottom_Plate, centered on
        its existing WHEEL_ON_PLATE_HOLE_EDGE mounting hole, replacing the
        original build_wheel() body-centerline placement.

        Must run after build_pendulum_link() (needs Bottom_Plate's final,
        tilted global placement) -- see run()'s call order.

        Only wired up for Wheel_Left today. Wheel_Right has no mirrored
        Pendulum_Link_Right/hole equivalent in this script yet (tracked as
        follow-up work); it keeps its original build_wheel() geometry and
        is hidden by _set_default_visibility() rather than left floating in
        a stale, disconnected position.
        """
        try:
            wheel_obj = self.output_doc.getObject(wheel_name)
            plate_obj = self.output_doc.getObject("Bottom_Plate")
            if wheel_obj is None or plate_obj is None:
                print(f"ERROR: {wheel_name} or Bottom_Plate not found for hole-mount")
                return False

            local_edge = Part.getShape(
                plate_obj, WHEEL_ON_PLATE_HOLE_EDGE, needSubElement=True, transform=False
            )
            local_center = Vector(*local_edge.Curve.Center)
            hole_center_global = plate_obj.getGlobalPlacement().multVec(local_center)
            new_center = hole_center_global + Vector(0.0, -WHEEL_ON_PLATE_CLEARANCE_OFFSET_MM, 0.0)

            radius = WHEEL_ON_PLATE_RADIUS_MM
            width = WHEEL_ON_PLATE_WIDTH_MM
            shape = Part.makeCylinder(radius, width, Vector(0.0, -width / 2.0, 0.0), Vector(0.0, 1.0, 0.0))

            old_volume = wheel_obj.Shape.Volume
            old_triangles = self._tessellate_triangle_count(wheel_obj.Shape)

            wheel_obj.Shape = shape
            wheel_obj.Placement = Placement(new_center, Rotation())

            triangles = self._tessellate_triangle_count(shape)
            self.total_volume_mm3 += shape.Volume - old_volume
            self.new_primitive_triangle_count += triangles - old_triangles

            for record in self.links:
                if record.name == wheel_name:
                    record.dimensions_mm = {
                        "diameter_mm": radius * 2.0, "radius_mm": radius, "width_mm": width,
                    }
                    record.placement = _placement_to_dict(wheel_obj.Placement)
                    record.bounding_box_mm = _bbox_to_dict(shape.BoundBox)
                    record.volume_mm3 = round(shape.Volume, 4)
                    record.triangle_count = triangles
                    record.notes = (
                        f"Re-mounted on Bottom_Plate's {WHEEL_ON_PLATE_HOLE_EDGE} hole "
                        f"(hole_center={tuple(round(v, 3) for v in hole_center_global)}), "
                        f"offset {WHEEL_ON_PLATE_CLEARANCE_OFFSET_MM}mm along -Y "
                        "(measured 3.0mm clearance from PlateStack/STS3032_Mount, "
                        "live-bridge verified)"
                    )
                    break

            print(f"✓ {wheel_name}: re-mounted on Bottom_Plate hole, radius={radius} mm, "
                  f"width={width} mm, center={new_center}")
            return True
        except Exception as e:
            print(f"ERROR re-mounting {wheel_name}: {e}")
            return False

    def _position_base_link_under_pendulum(self) -> bool:
        """Redesign follow-up (Issue #9, live-bridge probe, then manually
        fine-tuned live twice and ported back here): reposition Base_Link
        (translate only, same plate shape) to a deck-plate position near
        Bottom_Plate's top, at the pivot/servo level, instead of floating
        near the ground at CHASSIS_GROUND_CLEARANCE_MM or under the
        assembly's overall lowest point (earlier versions of this method
        targeted those instead).

        X and Y are both fixed absolute values now (BASE_LINK_X_POSITION_MM/
        BASE_LINK_Y_POSITION_MM), not centered on Bottom_Plate anymore -- a
        human moved it there live, across separate passes, and judged each
        one correct. Z is Bottom_Plate's own top face plus
        BASE_LINK_Z_NUDGE_MM (a small negative nudge, embedding Base_Link
        slightly into Bottom_Plate rather than leaving a hairline gap) --
        also a human live-tuned value, not derived from any other
        geometry.

        Must run after build_pendulum_link() (needs Bottom_Plate's final
        global placement). Independent of _mount_wheel_on_pendulum_plate()
        -- Wheel_Left's mount is relative to Bottom_Plate directly, not
        Base_Link, so this reposition doesn't disturb it.
        """
        try:
            base_link = self.output_doc.getObject("Base_Link")
            bottom_plate = self.output_doc.getObject("Bottom_Plate")
            if base_link is None or bottom_plate is None:
                print("ERROR: Base_Link or Bottom_Plate not found")
                return False

            # `.Shape` (direct property) already has Bottom_Plate's OWN
            # Placement baked in -- using it together with
            # getGlobalPlacement() (which ALSO includes that same own
            # Placement) would double-apply it. Part.getShape(...,
            # transform=False) gives the raw, unplaced shape instead --
            # same recipe _mount_wheel_on_pendulum_plate() uses for
            # Bottom_Plate's mounting hole.
            raw_local = Part.getShape(bottom_plate, "", needSubElement=False, transform=False)
            bp_bbox = raw_local.BoundBox.transformed(bottom_plate.getGlobalPlacement().toMatrix())

            target_center_x = BASE_LINK_X_POSITION_MM
            target_center_y = BASE_LINK_Y_POSITION_MM
            target_top_z = bp_bbox.ZMax + BASE_LINK_Z_NUDGE_MM

            local_bbox = base_link.Shape.BoundBox
            current_center_x = (local_bbox.XMin + local_bbox.XMax) / 2.0
            current_center_y = (local_bbox.YMin + local_bbox.YMax) / 2.0
            current_top_z = local_bbox.ZMax

            delta = Vector(
                target_center_x - current_center_x,
                target_center_y - current_center_y,
                target_top_z - current_top_z,
            )
            base_link.Placement = Placement(
                base_link.Placement.Base + delta, base_link.Placement.Rotation
            )

            self._chassis_top_z = target_top_z
            self._chassis_bottom_z = target_top_z - (local_bbox.ZMax - local_bbox.ZMin)

            for record in self.links:
                if record.name == "Base_Link":
                    record.placement = _placement_to_dict(base_link.Placement)
                    record.bounding_box_mm = _bbox_to_dict(base_link.Shape.BoundBox)
                    record.notes = (
                        (record.notes + " " if record.notes else "")
                        + f"Repositioned near Bottom_Plate's top (Z={target_top_z:.2f}mm, "
                        f"Bottom_Plate top + {BASE_LINK_Z_NUDGE_MM}mm), X={target_center_x}mm, "
                        f"Y={target_center_y}mm (human-tuned, fixed) -- instead of "
                        "CHASSIS_GROUND_CLEARANCE_MM above the ground plane."
                    )
                    break

            print(f"✓ Base_Link: repositioned near Bottom_Plate's top, "
                  f"top Z={target_top_z:.2f} mm, center=({target_center_x:.2f}, "
                  f"{target_center_y:.2f})")
            return True
        except Exception as e:
            print(f"ERROR repositioning Base_Link: {e}")
            return False

    # ---------------------------------------------------------------
    # Validation
    # ---------------------------------------------------------------

    def validate(self) -> None:
        chassis = self.params.chassis
        wheel = self.params.wheel

        base_link = self.output_doc.getObject("Base_Link")
        wheel_left = self.output_doc.getObject("Wheel_Left")
        wheel_right = self.output_doc.getObject("Wheel_Right")
        pendulum_link = self.output_doc.getObject("Pendulum_Link")

        # Object names present (case-sensitive, exact match)
        names_ok = all([base_link, wheel_left, wheel_right, pendulum_link])
        self.validations.append(ValidationResult(
            check_name="Required object names present",
            passed=names_ok,
            details="Base_Link, Wheel_Left, Wheel_Right, Pendulum_Link all found" if names_ok
                    else "One or more required objects missing",
        ))

        # Base_Link plate dimensions: length from BASE_LINK_LENGTH_MM,
        # width from robot_parameters.yaml's chassis.width_mm, thickness
        # from BASE_LINK_PLATE_THICKNESS_MM (not chassis.length_mm/
        # chassis.height_mm -- see build_base_link()'s redesign note).
        bbox = base_link.Shape.BoundBox
        chassis_ok = (
            abs(bbox.XLength - BASE_LINK_LENGTH_MM) < 0.01
            and abs(bbox.YLength - chassis.width_mm) < 0.01
            and abs(bbox.ZLength - BASE_LINK_PLATE_THICKNESS_MM) < 0.01
        )
        self.validations.append(ValidationResult(
            check_name="Base_Link plate dimensions",
            passed=chassis_ok,
            details=f"BBox {bbox.XLength:.2f}x{bbox.YLength:.2f}x{bbox.ZLength:.2f} mm vs "
                    f"expected {BASE_LINK_LENGTH_MM}x{chassis.width_mm}x{BASE_LINK_PLATE_THICKNESS_MM} mm",
        ))

        # Wheel dimensions
        wl_bbox = wheel_left.Shape.BoundBox
        wheel_dia_ok = abs(wl_bbox.XLength - wheel.diameter_mm) < 0.01
        wheel_width_ok = abs(wl_bbox.YLength - wheel.width_mm) < 0.01
        self.validations.append(ValidationResult(
            check_name="Wheel_Left dimensions match robot_parameters.yaml",
            passed=wheel_dia_ok and wheel_width_ok,
            details=f"BBox X(dia)={wl_bbox.XLength:.2f} mm, Y(width)={wl_bbox.YLength:.2f} mm "
                    f"vs expected dia={wheel.diameter_mm} mm, width={wheel.width_mm} mm",
        ))

        # Track width, symmetric about centerline
        wl_center_y = (wl_bbox.YMin + wl_bbox.YMax) / 2.0
        wr_bbox = wheel_right.Shape.BoundBox
        wr_center_y = (wr_bbox.YMin + wr_bbox.YMax) / 2.0
        track = wr_center_y - wl_center_y
        symmetric = abs(wl_center_y + wr_center_y) < 0.01  # mirrored about Y=0
        track_ok = abs(track - wheel.track_mm) < 0.01
        self.validations.append(ValidationResult(
            check_name="Wheel track width and symmetry",
            passed=track_ok and symmetric,
            details=f"track={track:.2f} mm (expected {wheel.track_mm} mm), "
                    f"centers=({wl_center_y:.2f}, {wr_center_y:.2f}) mm (symmetric about Y=0)",
            value=track,
            tolerance=wheel.track_mm,
        ))

        # Wheels share the same Z placement (ground clearance consistency)
        z_match = abs(wl_bbox.ZMin - wr_bbox.ZMin) < 0.01 and abs(wl_bbox.ZMax - wr_bbox.ZMax) < 0.01
        self.validations.append(ValidationResult(
            check_name="Wheels share identical Z placement",
            passed=z_match,
            details=f"Wheel_Left Z=[{wl_bbox.ZMin:.2f},{wl_bbox.ZMax:.2f}], "
                    f"Wheel_Right Z=[{wr_bbox.ZMin:.2f},{wr_bbox.ZMax:.2f}]",
        ))

        # Ground clearance: chassis bottom above Z=0, wheel bottom at/near Z=0
        ground_ok = bbox.ZMin > 0.0 and abs(min(wl_bbox.ZMin, wr_bbox.ZMin) - 0.0) < 0.01
        self.validations.append(ValidationResult(
            check_name="Ground clearance (chassis above Z=0, wheels resting on Z=0)",
            passed=ground_ok,
            details=f"Base_Link bottom Z={bbox.ZMin:.2f} mm, wheel bottom Z="
                    f"{min(wl_bbox.ZMin, wr_bbox.ZMin):.2f} mm",
        ))

        # No chassis/wheel interpenetration: chassis Y-range must not
        # overlap either wheel's Y-range.
        chassis_y = (bbox.YMin, bbox.YMax)
        no_overlap = not (
            (chassis_y[0] < wl_bbox.YMax and chassis_y[1] > wl_bbox.YMin)
            or (chassis_y[0] < wr_bbox.YMax and chassis_y[1] > wr_bbox.YMin)
        )
        self.validations.append(ValidationResult(
            check_name="No Base_Link / wheel interpenetration",
            passed=no_overlap,
            details=f"Base_Link Y=[{chassis_y[0]:.2f},{chassis_y[1]:.2f}], "
                    f"Wheel_Left Y=[{wl_bbox.YMin:.2f},{wl_bbox.YMax:.2f}], "
                    f"Wheel_Right Y=[{wr_bbox.YMin:.2f},{wr_bbox.YMax:.2f}]",
        ))

        # Pendulum sits above the chassis: PlateStack's global bottom Z
        # (computed and offset in build_pendulum_link) must clear Base_Link's
        # top Z. Re-derive both here from the live document rather than
        # trusting construction-time arithmetic.
        top_plate = self.output_doc.getObject("Top_Plate")
        pendulum_above = False
        pendulum_bottom_z = None
        if top_plate is not None and pendulum_link is not None:
            plate_names = ("Top_Plate", "Middle_Plate", "Bottom_Plate")
            plate_objs = [self.output_doc.getObject(n) for n in plate_names]
            plate_objs = [o for o in plate_objs if o is not None]
            local_bbox = plate_objs[0].Shape.BoundBox
            for o in plate_objs[1:]:
                local_bbox.add(o.Shape.BoundBox)
            global_bbox = local_bbox.transformed(pendulum_link.Placement.toMatrix())
            pendulum_bottom_z = global_bbox.ZMin
            pendulum_above = pendulum_bottom_z >= bbox.ZMax
        self.validations.append(ValidationResult(
            check_name="Pendulum_Link positioned above Base_Link",
            passed=pendulum_above,
            details=f"PlateStack global bottom Z={pendulum_bottom_z:.2f} mm vs "
                    f"Base_Link top Z={bbox.ZMax:.2f} mm" if pendulum_bottom_z is not None
                    else "Could not locate PlateStack plates for comparison",
        ))

        # Triangle budget (new primitives only -- see module docstring)
        budget_ok = self.new_primitive_triangle_count < NEW_PRIMITIVE_TRIANGLE_BUDGET
        self.validations.append(ValidationResult(
            check_name="New primitive geometry under triangle budget",
            passed=budget_ok,
            details=f"{self.new_primitive_triangle_count} triangles "
                    f"(budget: <{NEW_PRIMITIVE_TRIANGLE_BUDGET})",
            value=self.new_primitive_triangle_count,
            tolerance=NEW_PRIMITIVE_TRIANGLE_BUDGET,
        ))

    # ---------------------------------------------------------------
    # Output
    # ---------------------------------------------------------------

    def _set_default_visibility(self) -> None:
        """Mark every object visible so the saved file doesn't open blank.

        Plain-cylinder/box `Part::Feature` objects created headlessly have no
        `ViewObject` at all unless a GUI is up (`App.GuiUp`) -- there is no
        App-side "Visibility" property to set without one, so this is a
        no-op for a true headless `freecadcmd script.py` run (confirmed via
        Issue #9's live-bridge verification of PR #52: every object came out
        `Visibility=False` on disk from exactly such a run). It only takes
        effect when this script executes with a GUI available, e.g. driven
        through the live FreeCAD MCP bridge's `execute_python`.

        `self.output_doc.Objects` is FreeCAD's flat, document-wide object
        list -- it already includes objects nested under `Pendulum_Link`'s
        `App::Part` groups (`PlateStack`/`STS3032_Mount` and their plate/mesh
        children), regardless of App::Part grouping. Empirically re-verified
        live through the MCP bridge (Issue #9 follow-up, PR #52): with a GUI
        up, this loop already sets `Visibility = True` on every one of those
        nested objects -- there is no separate nested-visibility bug to fix
        here. The camera/viewpoint itself is GUI-only state and can never be
        embedded by a headless save regardless of this fix; see
        `_set_camera_framing()` below for the GUI-only camera addition.
        """
        if not getattr(App, "GuiUp", False):
            return
        for obj in self.output_doc.Objects:
            view_obj = getattr(obj, "ViewObject", None)
            if view_obj is not None:
                view_obj.Visibility = True

        # Redesign follow-up (Issue #9, live-bridge probe): Wheel_Right has
        # no mirrored Pendulum_Link_Right/hole to mount on yet (tracked as
        # follow-up work) -- hide it rather than show it floating, stale
        # and disconnected, at its old body-centerline position. Base_Link
        # (now a plate, see build_base_link()) stays visible.
        wheel_right = self.output_doc.getObject("Wheel_Right")
        view_obj = getattr(wheel_right, "ViewObject", None) if wheel_right is not None else None
        if view_obj is not None:
            view_obj.Visibility = False

    def _set_camera_framing(self) -> None:
        """Frame the 3D view on the whole assembly (isometric + fit-all)
        before saving, so the embedded camera in the .FCStd shows the full
        model instead of whatever viewpoint was last active.

        Like `_set_default_visibility()`, this is GUI-only: a headless
        `freecadcmd script.py` run has no `FreeCADGui` module at all (it is
        imported here, lazily, only inside this GUI-guarded branch -- an
        unconditional module-level `import FreeCADGui` would break the
        headless path this script is primarily run through), no 3D view, and
        therefore no camera to set -- this is a no-op headlessly, and a
        headless save embeds no camera/viewpoint at all, same as before this
        change. It only takes effect when this script executes with a GUI
        available, e.g. driven through the live FreeCAD MCP bridge's
        `execute_python`. Confirmed working live through the bridge (Issue
        #9 follow-up, PR #52): `view.viewIsometric()` followed by
        `view.fitAll()` on the output document's `ActiveView`.
        """
        if not getattr(App, "GuiUp", False):
            return
        try:
            import FreeCADGui as Gui
            gui_doc = Gui.getDocument(self.output_doc.Name)
            view = gui_doc.ActiveView if gui_doc is not None else None
            if view is None:
                print("  ⚠ WARNING: No ActiveView available; skipping camera framing")
                return
            view.viewIsometric()
            view.fitAll()
        except Exception as e:
            print(f"  ⚠ WARNING: Could not set camera framing: {e}")

    def save_document(self) -> bool:
        try:
            self._set_default_visibility()
            self._set_camera_framing()
            output_path = SCRIPT_DIR / OUTPUT_FCSTD_FILENAME
            self.output_doc.saveAs(str(output_path))
            print(f"✓ Saved output document: {output_path}")
            return True
        except Exception as e:
            print(f"ERROR saving output document: {e}")
            return False

    def save_metadata(self) -> bool:
        try:
            output_path = SCRIPT_DIR / METADATA_FILENAME
            metadata = {
                "phase": 7,
                "title": "Stage 1: Body + Wheel Geometry",
                "timestamp": datetime.now().isoformat(),
                "issue": 9,
                "source_document": SOURCE_DOC_FILENAME,
                "output_document": OUTPUT_FCSTD_FILENAME,
                "robot_parameters_source": str(_DESIGN_INPUTS_DIR / "robot_parameters.yaml"),
                "links": {link.name: link.to_dict() for link in self.links},
                "geometry_stats": {
                    "new_primitive_triangle_count": self.new_primitive_triangle_count,
                    "new_primitive_triangle_budget": NEW_PRIMITIVE_TRIANGLE_BUDGET,
                    "new_primitive_triangle_budget_ok":
                        self.new_primitive_triangle_count < NEW_PRIMITIVE_TRIANGLE_BUDGET,
                    "reused_pendulum_mesh_facet_counts": self.reused_mesh_facet_counts,
                    "reused_pendulum_mesh_note": (
                        "Inherited from Issue #3 (plates_servo_assembled.FCStd), "
                        "not new Stage 1 geometry. Excluded from the "
                        "<5000-triangle budget check above: per Issue #9 "
                        "Decision #10, the servo's simulation <collision> "
                        "geometry is emitted as URDF primitives (box + "
                        "cylinder) in Stage 4, not this mesh -- this "
                        "high-fidelity mesh is a visual-only asset."
                    ),
                    "total_volume_mm3": round(self.total_volume_mm3, 4),
                    "total_volume_note": (
                        "Sum of Base_Link + Wheel_Left + Wheel_Right + "
                        "copied PlateStack plate volumes + (where "
                        "watertight) copied STS3032_Mount mesh volumes. "
                        "For Stage 3's future sanity check (Issue #9 "
                        "Decision #12) against Phase 4's export_metadata.json "
                        "geometry_stats.volume_mm3 figure (22708.46 mm^3 for "
                        "plates+servo only, no wheels/chassis) -- NOT "
                        "expected to match exactly once wheel/chassis "
                        "volume is included; informational only."
                    ),
                },
                "validations": [v.to_dict() for v in self.validations],
                "all_validations_passed": all(v.passed for v in self.validations),
            }
            with open(output_path, "w") as f:
                json.dump(metadata, f, indent=2)
            print(f"✓ Saved metadata: {output_path}")
            return True
        except Exception as e:
            print(f"ERROR saving metadata: {e}")
            import traceback
            traceback.print_exc()
            return False

    # ---------------------------------------------------------------
    # Orchestration
    # ---------------------------------------------------------------

    def run(self) -> bool:
        print("=" * 70)
        print("STAGE 1: BODY + WHEEL GEOMETRY (Issue #9)")
        print("=" * 70)
        print()

        if not self.load_parameters():
            return False
        print()

        if not self.open_source_document():
            return False
        print()

        if not self.create_output_document():
            return False
        print()

        print("Building chassis and wheels...")
        print("-" * 70)
        if not self.build_base_link():
            return False
        if not self.build_wheel("Wheel_Left", side_sign=-1):
            return False
        if not self.build_wheel("Wheel_Right", side_sign=+1):
            return False
        print()

        print("Reusing pendulum linkage as Pendulum_Link...")
        print("-" * 70)
        if not self.build_pendulum_link():
            return False
        print()

        print("Re-mounting Wheel_Left on Pendulum_Link's Bottom_Plate...")
        print("-" * 70)
        if not self._mount_wheel_on_pendulum_plate("Wheel_Left"):
            return False
        print()

        print("Repositioning Base_Link under Pendulum_Link/STS3032_Mount...")
        print("-" * 70)
        if not self._position_base_link_under_pendulum():
            return False
        print()

        print("Validating...")
        print("-" * 70)
        self.validate()
        for v in self.validations:
            status = "✓" if v.passed else "✗"
            print(f"  {status} {v.check_name}: {v.details}")
        print()

        print("Saving output...")
        print("-" * 70)
        if not self.save_document():
            return False
        if not self.save_metadata():
            return False
        print()

        print("=" * 70)
        all_passed = all(v.passed for v in self.validations)
        if all_passed:
            print("✓ Stage 1 Complete -- all validations passed")
        else:
            print("⚠ Stage 1 Complete -- some validations failed, see above")
        print("=" * 70)

        return all_passed


def main() -> int:
    generator = BodyWheelsGenerator()
    success = generator.run()

    if generator.source_doc is not None:
        try:
            App.closeDocument(generator.source_doc.Name)
        except Exception:
            pass
    if generator.output_doc is not None:
        try:
            App.closeDocument(generator.output_doc.Name)
        except Exception:
            pass

    return 0 if success else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print(f"FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
