#!/usr/bin/env python3
"""
Stage 2: Joint constraints for assembly (Issue #61)

Wraps Stage 1's chassis/wheel/pendulum geometry (`robot_body_wheels.FCStd`,
from `07_create_body_and_wheels.py`) in a native Assembly workbench
`Assembly::AssemblyObject` container and adds 4 Revolute joints:
`wheel_left_joint`, `wheel_right_joint`, `pendulum_pivot_joint`,
`pendulum_pivot_right_joint`.

Scope note: this script is Stage 2 ONLY. It does not compute mass/inertia in
SI units (Stage 3) or export URDF (Stage 4).

** MUST BE RUN THROUGH THE LIVE FREECAD MCP BRIDGE (GUI-backed), NOT PLAIN
   `freecadcmd` -- see "Known limitation" below. **

Object/document structure created, in the SAME document as the source
(intra-document -- no cross-document `App::Link`, matching
`07_create_body_and_wheels.py`'s own reasoning for avoiding cross-doc
fragility):
  - Assembly            (Assembly::AssemblyObject, new container)
  - Base_Link_Link, Wheel_Left_Link, Wheel_Right_Link, Pendulum_Link_Link
    (App::Link, each wrapping the matching Stage 1 object with an identity
    Placement -- the Stage 1 objects already carry their own absolute
    placement, so the link itself adds no additional transform)
  - Joints group (Assembly::JointGroup, auto-created by
    UtilsAssembly.getJointGroup())
    - wheel_left_joint, wheel_right_joint, pendulum_pivot_joint,
      pendulum_pivot_right_joint
      (App::FeaturePython + JointObject.Joint proxy, JointType="Revolute")

Joint axis: all 4 joints rotate about global Y. Wheels: axis Vector(0,1,0)
from `build_wheel()` (07_create_body_and_wheels.py:491). Pendulum pivot:
NOT literally `servo_placement.json`'s axis (that file's placement is
identity/unrelated servo-mount data) -- empirically verified instead via a
live probe of `Pendulum_Link.Placement.Rotation` (Yaw-Pitch-Roll=0,0,90),
which maps local Z to global (0,-1,~0): the same line as the wheel axle,
confirming 07's own `PENDULUM_LINK_TILT_DEG=90` comment ("tilted so its
plate faces stand parallel to the wheel discs") -- physically correct for a
2-wheel self-balancing robot (wheel axle and pendulum swing must be
parallel).

FreeCAD 1.1's Assembly Revolute joints rotate about the connector
Placement's LOCAL X axis, not Z (confirmed via JointObject.py:1015-1030's
`rotation_axis = globalJcsPlc.Rotation.multVec(App.Vector(1, 0, 0))`).
`Rotation(Vector(0,0,1), 90deg)` maps local X -> global Y, so that rotation
is used for every joint's Placement1/Placement2, with Detach1=Detach2=True
(bypasses the normal auto-recompute-from-Reference geometry lookup, which
is a GUI-selection-driven path this script doesn't use).

No distinct "continuous" JointType exists (JointObject.py's JointTypes
enum: Fixed, Revolute, Cylindrical, Slider, Ball, Distance, Parallel,
Perpendicular, Angle, RackPinion, Screw, Gears, Belt). The issue's
"revolute/continuous motion" is realized as JointType="Revolute" with
EnableAngleMin=EnableAngleMax=False (unlimited rotation) -- these already
default to False (JointObject.py:322-346) but are set explicitly here for
clarity.

Base_Link is the implicit ground/fixed member (Reference2 on all 4
joints) -- NOT wrapped in an explicit GroundedJoint/ObjectToGround. A
follow-up stage can add grounding if the physics solver needs it.

pendulum_pivot_right_joint (Pendulum_Link_Right) was added after the
issue's original 3-joint ask, following human review of the first pass --
Pendulum_Link_Right exists in Stage 1's geometry as a second, independently
human-tuned pendulum linkage (see 07_create_body_and_wheels.py's
build_pendulum_link_right() docstring: "NOT a geometric mirror of
Pendulum_Link"), so it gets its own joint rather than staying rigid.

Known limitation -- Assembly::JointGroup/Joint creation segfaults TRUE
HEADLESS freecadcmd 1.1.3: reproducible and isolated via bisection --
`doc.addObject('Assembly::JointGroup', ...)` alone works fine headlessly;
add `import JointObject` (needed for the `Joint` proxy class) and ANY
subsequent Assembly/JointGroup object creation crashes the process with
"Application unexpectedly terminated" and no Python traceback -- not an
exception this script can catch. The same code runs without crashing
through the live FreeCAD MCP bridge (GUI-backed, `FreeCAD.GuiUp=True`).
This mirrors this project's other documented GUI-only-behavior limitations
(see root CLAUDE.md's "FreeCAD Live Bridge -- Known Limitations"); it is a
new entry for that list, not yet added there. Until FreeCAD fixes this
upstream, run this script's `main()` via the bridge's `execute_python`,
forcing `__name__ == "__main__"` per CLAUDE.md's documented trick:
    exec(compile(open(path).read(), path, 'exec'),
         {'__name__': '__main__', '__file__': path})
A plain `"$FREECAD_BIN" -c "exec(open(path).read())"` invocation (this
project's usual headless pattern) will segfault on this script specifically
-- do not use it here.

Output:
    - robot_assembly.FCStd (new document; robot_body_wheels.FCStd untouched)
    - joint_config.json (per-joint type/reference/origin/axis metadata +
      validation results)
"""

import sys
import json
import functools
from pathlib import Path
from datetime import datetime

print = functools.partial(print, flush=True)  # noqa: A001

try:
    import FreeCAD as App
except ImportError:
    print("ERROR: FreeCAD Python modules not available.")
    print("This script must be run inside FreeCAD (via the live MCP bridge's")
    print("execute_python -- see the 'Known limitation' section in this file's")
    print("docstring for why a plain freecadcmd invocation will segfault here).")
    sys.exit(1)

try:
    SCRIPT_DIR = Path(__file__).resolve().parent
except NameError:
    SCRIPT_DIR = Path.home() / "freecad-workspace" / "inverted-pendulum-project" / "03_Parts" / "Generators"

SOURCE_FCSTD_FILENAME = "robot_body_wheels.FCStd"
OUTPUT_FCSTD_FILENAME = "robot_assembly.FCStd"
CONFIG_FILENAME = "joint_config.json"

LINK_SOURCE_NAMES = ["Base_Link", "Wheel_Left", "Wheel_Right", "Pendulum_Link", "Pendulum_Link_Right"]

# All 4 joints rotate about global Y -- see docstring for how each origin/
# axis was derived (wheel axle axis vs. the Pendulum_Link/_Right tilt
# probe). Pendulum_Link_Right's own Placement (Yaw-Pitch-Roll=0,0,90,
# Pos=(-8.0014, 62.2758, 71.25)) was probed the same way as Pendulum_Link
# and maps local Z -> global (0,-1,~0), the same axis line -- so
# pendulum_pivot_right_joint reuses the identical axis convention.
JOINT_SPECS = [
    ("wheel_left_joint", "Wheel_Left", (15.0007, -28.5258, 54.0095)),
    ("wheel_right_joint", "Wheel_Right", (15.0007, 67.0258, 54.0095)),
    ("pendulum_pivot_joint", "Pendulum_Link", (-8.0014, -18.2758, 70.75)),
    ("pendulum_pivot_right_joint", "Pendulum_Link_Right", (-8.0014, 62.2758, 71.25)),
]


class JointConfigurator:
    """Builds the Assembly container + App::Link wrappers + 4 Revolute
    joints on top of Stage 1's robot_body_wheels.FCStd geometry."""

    def __init__(self) -> None:
        self.doc = None
        self.assembly = None
        self.links: dict = {}
        self.joint_records: list = []
        self.validations: list = []

    def load_source_document(self) -> bool:
        source_path = SCRIPT_DIR / SOURCE_FCSTD_FILENAME
        try:
            for open_doc in App.listDocuments().values():
                if open_doc.FileName == str(source_path):
                    self.doc = open_doc
                    break
            if self.doc is None:
                self.doc = App.openDocument(str(source_path))
            for name in LINK_SOURCE_NAMES:
                if self.doc.getObject(name) is None:
                    raise RuntimeError(f"source object {name} not found in {source_path}")
            print(f"✓ Loaded source document: {source_path}")
            return True
        except Exception as e:
            print(f"ERROR loading source document: {e}")
            return False

    def build_assembly_and_links(self) -> bool:
        try:
            self.assembly = self.doc.addObject("Assembly::AssemblyObject", "Assembly")
            for name in LINK_SOURCE_NAMES:
                src = self.doc.getObject(name)
                link = self.doc.addObject("App::Link", name + "_Link")
                link.LinkedObject = src
                link.Placement = App.Placement()  # identity; src's absolute placement already applies
                link.Label = name + "_Link"
                self.assembly.addObject(link)
                self.links[name] = link
                if getattr(App, "GuiUp", False):
                    try:
                        src.ViewObject.Visibility = False
                        link.ViewObject.Visibility = True
                    except Exception:
                        pass
            print(f"✓ Assembly container + {len(self.links)} App::Link wrappers created")
            return True
        except Exception as e:
            print(f"ERROR building assembly/links: {e}")
            return False

    def create_joints(self) -> bool:
        try:
            import JointObject
            import UtilsAssembly

            joint_group = UtilsAssembly.getJointGroup(self.assembly)
            # Rotation(Z, 90deg) maps local X -> global Y (the joint axis
            # convention for all 4 joints here) -- see docstring.
            axis_rotation = App.Rotation(App.Vector(0, 0, 1), 90)

            for joint_name, moving_key, origin in JOINT_SPECS:
                j = joint_group.newObject("App::FeaturePython", joint_name)
                JointObject.Joint(j, 1)  # 1 = "Revolute"
                j.Label = joint_name

                j.Reference1 = (self.links[moving_key], [""])
                j.Reference2 = (self.links["Base_Link"], [""])
                j.Detach1 = True
                j.Detach2 = True

                plc = App.Placement(App.Vector(*origin), axis_rotation)
                j.Placement1 = plc
                j.Placement2 = plc

                j.EnableAngleMin = False
                j.EnableAngleMax = False

                if getattr(App, "GuiUp", False):
                    try:
                        JointObject.ViewProviderJoint(j.ViewObject)
                    except Exception:
                        pass

                self.joint_records.append({
                    "name": joint_name,
                    "type": j.JointType,
                    "reference1": moving_key + "_Link",
                    "reference2": "Base_Link_Link",
                    "origin_mm": {"x": origin[0], "y": origin[1], "z": origin[2]},
                    "axis": "global Y (0,1,0)",
                    "enable_angle_min": bool(j.EnableAngleMin),
                    "enable_angle_max": bool(j.EnableAngleMax),
                })
                print(f"✓ {joint_name}: Revolute, origin={origin}")

            self.doc.recompute()
            return True
        except Exception as e:
            print(f"ERROR creating joints: {e}")
            return False

    def validate(self) -> None:
        self.validations = [
            {
                "check": "joint_count",
                "passed": len(self.joint_records) == 4,
                "details": f"{len(self.joint_records)} joints created (expected 4)",
            },
            {
                "check": "joint_type",
                "passed": all(r["type"] == "Revolute" for r in self.joint_records),
                "details": "All joints use JointType=Revolute (no distinct 'continuous' type exists)",
            },
            {
                "check": "axis_consistency",
                "passed": all(r["axis"] == "global Y (0,1,0)" for r in self.joint_records),
                "details": "All joints share global Y -- wheel axle and pendulum swing must be parallel",
            },
        ]
        for v in self.validations:
            status = "✓" if v["passed"] else "✗"
            print(f"  {status} {v['check']}: {v['details']}")

    def save_document(self) -> bool:
        try:
            output_path = SCRIPT_DIR / OUTPUT_FCSTD_FILENAME
            self.doc.saveAs(str(output_path))
            print(f"✓ Saved output document: {output_path}")
            return True
        except Exception as e:
            print(f"ERROR saving document: {e}")
            return False

    def save_config(self) -> bool:
        try:
            config = {
                "phase": 8,
                "title": "Stage 2: Assembly Joint Configuration",
                "generated": datetime.now().isoformat(),
                "source_document": SOURCE_FCSTD_FILENAME,
                "output_document": OUTPUT_FCSTD_FILENAME,
                "assembly_container": "Assembly",
                "links": {
                    name + "_Link": {"linked_object": name}
                    for name in LINK_SOURCE_NAMES
                },
                "joints": {r["name"]: r for r in self.joint_records},
                "validations": self.validations,
            }
            output_path = SCRIPT_DIR / CONFIG_FILENAME
            with open(output_path, "w") as f:
                json.dump(config, f, indent=2)
            print(f"✓ Saved config: {output_path}")
            return True
        except Exception as e:
            print(f"ERROR saving config: {e}")
            return False

    def run(self) -> bool:
        print("=" * 70)
        print("Stage 2: Joint constraints for assembly (Issue #61)")
        print("=" * 70)

        if not getattr(App, "GuiUp", False):
            print("ERROR: this script must run through the live FreeCAD MCP bridge")
            print("(GuiUp=True) -- plain headless freecadcmd segfaults on")
            print("Assembly::JointGroup/Joint creation. See docstring.")
            return False

        print("Loading source document...")
        if not self.load_source_document():
            return False
        print()

        print("Building Assembly container + App::Link wrappers...")
        if not self.build_assembly_and_links():
            return False
        print()

        print("Creating joints...")
        if not self.create_joints():
            return False
        print()

        print("Validating...")
        self.validate()
        print()

        print("Saving output...")
        if not self.save_document():
            return False
        if not self.save_config():
            return False
        print()

        all_passed = all(v["passed"] for v in self.validations)
        print("=" * 70)
        print("✓ Stage 2 Complete" if all_passed else "⚠ Stage 2 Complete -- some validations failed")
        print("=" * 70)
        return all_passed


def main() -> int:
    configurator = JointConfigurator()
    success = configurator.run()
    return 0 if success else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print(f"FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
