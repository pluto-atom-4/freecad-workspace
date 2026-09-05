# Findings: FreeCAD to Webots Pipeline POC (issue #24)

Written for whoever builds #9 Stage 4 (URDF export from this project's own
CAD) and #10 (Webots import of this project's own robot). Everything below
is a concrete, reproduced-on-this-machine data point, not speculation.

Environment: FreeCAD 1.1.3 headless (freecadcmd), Webots R2025a
(/usr/local/bin/webots), urdf2webots 2025.0.0 (pip, installed into the
pendulum-tools mamba env), TurtleBot3 Burger assets from
ROBOTIS-GIT/turtlebot3 pinned at fc817ce3073af1d6032397c64504134882af5e9a
(see ../NOTICE for the full provenance record).

## 1. Unit conversion: mm (FreeCAD/STL) vs m (URDF)

FreeCAD and the source STL meshes are in millimetres; URDF is defined in
metres. The ROBOTIS reference URDF (and this POC's turtlebot3_poc.urdf)
handles this entirely at the mesh tag with scale="0.001 0.001 0.001" - the
mesh geometry itself stays in mm, and URDF applies the mm-to-m scale factor
only on the visual/collision mesh reference. All numeric origin, axis,
inertial, and collision-primitive values elsewhere in the URDF (joint
offsets, box/cylinder sizes, inertia tensors) are already hand-authored in
metres in the source URDF - they are NOT mesh-scaled, and must be
independently correct in metres.

Concrete numbers from this pipeline:
- burger_base.stl bounding box (mm): 137.5 x 148.3 x 172.6
- Corresponding URDF collision box: box size="0.140 0.140 0.143" (m) - i.e.
  a rounded-down, hand-picked approximation of the mm bbox x 0.001, not a
  literal conversion of the mesh bbox. This is a gotcha: don't assume a
  collision primitive's dimensions are a mechanical 0.001-scale of the
  mesh's true bounding box - ROBOTIS's own reference URDF rounds/
  approximates it. A future exporter (#9) that does want an exact fit
  should compute the box from the actual (scaled) mesh bbox rather than
  copying this pattern verbatim.
- Joint origin xyz="0.0 0.08 0.023" for wheel_left_joint is already in
  metres and is not derived from any mesh coordinate - it's an
  independently specified assembly offset.

Takeaway for #9: any URDF exporter built on top of FreeCAD's own
(mm-native) Placement/Vector data must explicitly divide all translation
components by 1000 when writing origin xyz values. axis needs no scaling
(unit vector). Mesh scale="0.001 0.001 0.001" handles the mesh geometry
itself. These are two independent conversions - easy to do once and forget
the other.

## 2. Wheel joint axis/rotation convention: the rpy="-1.57 0 0" gotcha

This is the single easiest thing to get backwards when hand-authoring or
porting a URDF with drive wheels, so it gets called out explicitly per the
issue #24 plan.

    <joint name="wheel_left_joint" type="continuous">
      <origin xyz="0.0 0.08 0.023" rpy="-1.57 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>

Read naively, axis xyz="0 0 1" looks like "the wheel spins about Z" - but Z
here is the joint's own local (child) frame, which origin rpy="-1.57 0 0"
(a -90-degree rotation about X, applied when going from parent to child
frame) has already tilted so that the joint's local Z axis is coincident
with the parent's (base_link's) Y axis. Net effect: the wheel actually
rotates about the robot's Y axis (the left/right side axis), which is
exactly what a drive wheel needs - but this is invisible if you read axis
in isolation; you have to compose it with origin rpy.

Practical implications:
- Do not "simplify" a continuous wheel joint by rewriting it as
  origin rpy="0 0 0" plus axis xyz="0 1 0" assuming it is equivalent for
  simulation purposes. It changes the visual mesh's local frame too (URDF
  visual geometry is expressed in the same joint-child frame), and the
  wheel mesh's own visual origin rpy="1.57 0 0" (the opposite rotation,
  undoing the joint's tilt for rendering purposes) would then be wrong and
  the mesh would render sideways.
- If #9's exporter derives wheel joint axes from FreeCAD assembly
  constraints/placements, it must decompose the placement into an
  origin-rotation plus a local axis, not assume the axis is expressed in
  the parent/world frame.
- Webots' URDF importer (via urdf2webots) handled this correctly in this
  POC - see sections 5/6 below - so this convention round-trips fine once
  the URDF itself is correct; the risk is entirely in URDF authoring, not
  in Webots import.

## 3. STEP round-trip fidelity and performance (Stage 1 / Stage 1b)

Conversion pattern: per-facet Part.makePolygon -> Part.Face ->
Part.makeShell -> Part.makeSolid (same pattern as
inverted-pendulum-project/03_Parts/Generators/01_convert_servo_stl_to_step_via_freecad.py).

Reference point (from that script's own conversion report, the servo mesh
this pattern was originally built around): 35,770 facets, converted
successfully with no timing captured by that script.

This POC's timing (freecad/output/stage1_conversion_report.json, headless
freecadcmd on this development machine):

| Link | Facets | faces_build | shell_build | solid_build | Total | STEP size |
|---|---:|---:|---:|---:|---:|---:|
| burger_base | 96,524 | 13.7s | 58.2s | 2.2s | 74.1s | ~96.9 MB |
| left_tire | 21,672 | 3.6s | 16.3s | 0.6s | 20.4s | ~22.2 MB |
| right_tire | 21,672 | 3.3s | 14.5s | 0.5s | 18.3s | ~22.2 MB |

Zero failed facets across all three links - the naive per-facet approach is
robust at this scale, just slow, and Part.makeShell's sewing step dominates
total time (78-80% of it in every case). burger_base has 4.5x left_tire's
facet count but took roughly 3.6x the shell-build time and 3.6x the total
time, so the scaling here looks close to linear on this data point, not
quadratic - but at ~2.7x the facet count of the servo mesh this pattern was
built around, a 74s wall-clock conversion for a single link is already
enough that a #9 exporter processing many links back-to-back should budget
real time for this, not assume STL-to-STEP conversion is fast.

File size is the more surprising finding. The per-facet conversion produces
a STEP file with one planar face entity per source mesh triangle, so file
size scales with facet count, not with the underlying geometry's actual
complexity - burger_base.step is 97 MB for a robot base that would be a
handful of KB as a proper parametric/BREP solid. Confirmed by entity counts
in FreeCAD's own OCC transfer log during export: burger_base.step =
1,883,759 entities; left_tire.step / right_tire.step = 422,580 entities
each (for just 21,672 facets apiece). For #9, if source parts are ever this
facet-dense, this conversion pattern will produce STEP files that are
impractically large. Worth evaluating FreeCAD's shape-healing /
simplification options, or a coarser tessellation-refit, instead of a
literal 1:1 facet-to-face mapping, before applying this pattern to
higher-poly project geometry.

Round-trip fidelity (freecad/output/roundtrip_report.json, re-import of the
STEP files above and diff against the source mesh):

| Link | Source facets | Reimported tessellated facets | BBox diff (mm) | Pre-export shape | Pre-export volume (mm^3) | Post-reimport shape | Post-reimport volume (mm^3) |
|---|---:|---:|---:|---|---:|---|---:|
| burger_base | 96,524 | 96,834 | 0.0 / 0.0 / 0.0 | Solid | 180,331.4 | Compound (INVALID) | 73,604.6 |
| left_tire | 21,672 | 21,672 | 0.0 / 0.0 / 0.0 | Solid | 27,817.6 | Solid (valid) | 827.2 |
| right_tire | 21,672 | 21,672 | 0.0 / 0.0 / 0.0 | Solid | 27,817.6 | Solid (valid) | 827.2 |

Bounding box round-trips essentially perfectly in all three cases (sub-
micron diff, well inside any sane tolerance) - STEP faithfully preserves
the geometric extent. Volume and solid topology do not round-trip reliably,
and the failure mode is worse than a uniform scale error - it is structural:

- A follow-up diagnostic (Part.Shape.isValid(), per-solid breakdown) on the
  reimported STEP shapes found that burger_base.step reads back as a
  Compound of 89 separate solids (239 shells, still 96,524 faces total),
  and the compound itself fails isValid(). Several of the 89 individual
  solids also fail isValid() and/or report negative volumes (e.g. one
  reports -47,489 mm^3). The original in-memory shape, immediately after
  Part.makeSolid and before any STEP export, was a single valid Solid.
  STEP write/read did not preserve that single-solid topology - it
  silently fragmented into dozens of disconnected pieces, some
  self-intersecting or incorrectly oriented.
- left_tire and right_tire round-trip as a single valid Solid (not a
  Compound), yet their computed Volume is still ~33.6x smaller after
  reimport (827.2 mm^3 vs 27,817.6 mm^3) - identically for both wheels,
  which rules out random noise and points at a systematic face-orientation
  issue: .Volume is computed via the divergence theorem over oriented
  faces, so a reproducible subset of faces coming back flipped after the
  STEP round-trip would produce exactly this kind of large, consistent,
  but still "topologically valid" volume error.

Root cause (most likely, based on the above): a solid built by sewing many
thousands of independently-generated single-triangle planar faces
(Part.makeShell from separately constructed Part.Face objects, each with
its own copy of shared vertices rather than referencing common ones) is
consistent enough for FreeCAD's own in-memory makeSolid/.Volume at build
time, but STEP write/read does not reliably preserve face connectivity and
orientation for a solid assembled this way - especially at higher facet
density (burger_base, 96,524 facets, fragments into 89 solids) versus lower
density (each tire, 21,672 facets, stays one Solid but still mis-orients
enough faces to corrupt Volume).

Practical implication for #9: if a future URDF/STEP exporter (or anything
downstream that computes mass/inertia from FreeCAD shape volume) relies on
.Volume from a STEP file that was round-tripped through this per-facet
conversion pattern, the number will likely be wrong - by 30x or more, and
possibly split into dozens of spurious solids - even though the shape
"looks right" visually and its bounding box is correct. Any pipeline that
needs trustworthy mass properties post-STEP-round-trip should either
(a) compute mass properties from the pre-export in-memory shape and carry
them through separately rather than recomputing from a re-imported STEP, or
(b) run explicit shape-healing / re-orientation (ShapeFix_Solid, or
FreeCAD's own "Check geometry" / shape-healing tools) on the reimported
shape before trusting .Volume again. This is specific to the naive
per-facet conversion pattern, not a general STEP round-trip limitation - a
STEP file exported from a properly-authored native BREP solid (built from
sketches/pads, not raw mesh facets) would not be expected to show this,
since OCC's own solid-building maintains correct face orientation
throughout. This is therefore a data point specifically about the
01_convert_servo_stl_to_step_via_freecad.py-style mesh conversion pattern
this repo already uses in inverted-pendulum-project/03_Parts/Generators/ -
worth a follow-up look there too if that pipeline's STEP outputs are ever
re-imported and their volumes trusted downstream.

## 4. Webots baseline sanity check (bundled turtlebot3_burger.wbt)

    webots --batch --mode=fast --no-rendering --minimize --stdout \
      /usr/local/webots/projects/robots/robotis/turtlebot/worlds/turtlebot3_burger.wbt

ran cleanly: the bundled turtlebot3_ostacle_avoidance controller started and
terminated with no errors, and the world (which uses an EXTERNPROTO fetched
live from cyberbotics/webots) resolved and loaded correctly. This confirms
the local Webots R2025a install and its EXTERNPROTO resolution both work
correctly, independent of anything in this POC - so the Stage 3 result
below is attributable to this pipeline's own URDF/import path, not a broken
local Webots install.

## 5. urdf2webots behavior

`urdf2webots` 2025.0.0 (pip) converts a URDF to a Webots PROTO file via:

    python3 -m urdf2webots.importer \
      --input=urdf/turtlebot3_poc.urdf \
      --output=webots/protos/TurtlebotPoc.proto \
      --target=R2025a

This succeeded on the first attempt against turtlebot3_poc.urdf with no
errors or warnings (console output: "Root link: base_footprint" / "There
are 5 links, 4 joints and 0 sensors"). Notable behavior:

> **Addendum (2026-09-04, issue #32):** this `base_footprint` root was
> later identified as the root cause of a robot-doesn't-translate bug —
> with `base_footprint` (massless) as the URDF root and `base_link`
> (which carries all real mass/inertia/collision) attached one level
> down via a fixed `base_joint`, the generated PROTO's `Robot` node ends
> up with no `physics`/`boundingObject` of its own (they land on the
> nested `base_link` Solid instead), which Webots implicitly welds to
> the static world via an ODE fixed joint — the robot could spin its
> wheels but never translate. `urdf/turtlebot3_poc.urdf` was restructured
> to drop `base_footprint`/`base_joint` entirely; post-fix the URDF root
> link is `base_link` directly, and the regenerated PROTO's `Robot` node
> carries `physics`/`boundingObject` itself (console output now reads
> "Root link: base_link" / "There are 4 links, 3 joints and 0 sensors").

- Relative `<mesh filename="meshes/....stl">` paths in the URDF are
  resolved relative to the URDF file's own directory, and the generated
  PROTO's Mesh `url` fields are rewritten relative to the PROTO's own
  output location (`../../urdf/meshes/....stl` from
  `webots/protos/TurtlebotPoc.proto` back to `urdf/meshes/`). No manual
  path-rewriting was needed.
- Each `type="continuous"` joint became a Webots `HingeJoint` with a
  `RotationalMotor` device named identically to the URDF joint name
  (`wheel_left_joint`, `wheel_right_joint`) plus an auto-added
  `PositionSensor` device named `<joint_name>_sensor`. This predictable
  naming is what `wheel_articulation_check.py` relies on directly - no
  fallback name search was actually needed, though the controller keeps
  one for robustness (see `WHEEL_MOTOR_NAME_FALLBACKS` in that script).
- **The `origin rpy="-1.57 0 0"` + `axis xyz="0 0 1"` wheel joint
  convention (see section 2) was composed correctly**: the generated
  `HingeJoint`'s `jointParameters.axis` came out as
  `0.000000 1.000000 0.000796` - i.e. essentially the parent's Y axis
  (matching the intent), not the URDF's literal local `0 0 1`. This
  confirms urdf2webots does the correct rotation composition rather than a
  naive axis passthrough, and is a genuinely reassuring result for #9 - a
  correctly-authored URDF's rotated joint axes convert as expected.
- `type="fixed"` joints (`base_joint`, `caster_back_joint`) were folded
  directly into the parent `Solid` hierarchy rather than becoming
  Webots joints, as expected - no explicit fixed-joint node exists in the
  output.
- The `caster_back_link` (collision-only, no visual mesh in the URDF)
  converted with a `boundingObject Pose { children [ Box {...} ] }` and no
  `Shape`/geometry child under its own `Solid` - i.e. urdf2webots correctly
  did NOT invent a visual for a link that had none in the URDF.
- Per-link `<inertial>` mass/inertia values were carried through unchanged
  into each `Solid`'s `Physics { mass ... centerOfMass ... inertiaMatrix
  ... }` block.
- A `TurtlebotPoc_textures/` directory was created alongside the .proto
  file even though this URDF defines no textures (only flat `<material>`
  colors) - it was empty in this run; worth knowing it always creates this
  directory regardless of whether it is used.

## 6. Webots URDF import & articulation test results

Running the generated world:

    webots --batch --mode=fast --no-rendering --minimize --stdout \
      webots/worlds/turtlebot3_poc.wbt

**imported and ran successfully on the first attempt** - no import errors,
no missing-device errors, no PROTO resolution failures. The
`wheel_articulation_check` controller found both wheel motor devices under
their expected (URDF-matching) names on the first try:

    Found left motor device:  wheel_left_joint
    Found right motor device: wheel_right_joint

    Ran 20 simulation steps at 32ms each (0.64s simulated).
    Commanded velocity: 2.0 rad/s on both wheels.
    Left wheel joint position:  0.0640 -> 1.3435 rad (delta 1.2796)
    Right wheel joint position: 0.0640 -> 1.3435 rad (delta 1.2796)
    PASS: both wheel joints rotated under commanded velocity.

    VERDICT: PASS

Both wheel joints rotated in the same direction under an identical
commanded velocity, consistent with the axis convention discussed in
section 2 (both `HingeJoint`s share essentially the same world-Y-aligned
axis after urdf2webots's rotation composition). The measured rotation
(1.28 rad over ~0.6s at a commanded 2.0 rad/s, i.e. ~2.1 rad/s effective,
allowing for startup ramp and the coarse 32ms timestep) is in the expected
range for a velocity-controlled continuous joint - this is a real,
successful articulation result, not a false positive from a device that
merely accepted a command without moving.

**One minor Webots API quirk hit and worked around**: a `PositionSensor`'s
`getValue()` returns `nan` if read before the simulation has advanced past
its first `robot.step()` call following `enable()` - reading it
"immediately" after `motor.setPosition()/setVelocity()` and before any
`step()` call returns `nan`, not `0.0`. The controller now performs one
`robot.step()` before taking its "initial" reading to avoid this. This is
a general Webots controller-authoring gotcha, not specific to
urdf2webots-generated PROTOs, but worth flagging for #10's own controller
code.

No other import or articulation issues were encountered.

## 7. Stop-rule outcome

**The decision #6 timebox stop rule was NOT triggered.** Webots URDF
import (via the urdf2webots-generated PROTO) succeeded on the first
attempt, and the articulation test passed on its first run as well (after
the one PositionSensor NaN fix described above, which was a controller-
side correctness fix, not an import retry). Zero of the anticipated "3
genuinely distinct failed fixes" were needed for Stage 3.

## Summary for #9 / #10

- **Units**: FreeCAD/STL stays in mm throughout; URDF mesh tags apply
  `scale="0.001 0.001 0.001"`, but all other URDF numeric fields (origin
  xyz, inertial, collision primitives) must be independently authored/
  converted to metres - there is no single global scale knob.
- **Wheel axis convention**: `origin rpy="-1.57 0 0"` + `axis xyz="0 0 1"`
  is the correct, and non-obvious, way to express a Y-axis drive wheel;
  urdf2webots composes this correctly into Webots `HingeJoint.axis`.
  Section 2 has the full explanation - read it before hand- or
  auto-authoring any wheel joint.
- **STEP round-trip via the per-facet mesh-to-BREP pattern preserves
  bounding box but NOT volume or single-solid topology at higher facet
  counts.** This is the single most actionable finding for #9: do not
  trust `.Volume` (or solid count/topology) computed from a STEP file that
  was round-tripped through this conversion pattern - carry mass
  properties through from the pre-export in-memory shape instead, or run
  shape-healing after re-import. Also budget real wall-clock time
  (order of a minute per link at ~100k facets, dominated by
  `Part.makeShell`) and expect STEP file size to scale with facet count,
  not geometric complexity (a 97 MB STEP file for a small robot base is a
  direct consequence of this, not a fluke).
- **Webots/urdf2webots import path is solid**: given a correctly-authored
  URDF, urdf2webots converts joints, axes, meshes, inertials, and
  collision-only links (no invented visuals) all correctly and
  predictably, and the resulting PROTO imports and runs in Webots without
  manual fixup. The risk in this pipeline is concentrated in URDF
  authoring correctness (units, axis composition) and in the FreeCAD
  mesh-to-STEP-to-mesh round trip (section 3), not in the Webots import
  step itself.
- **Overall pipeline verdict for this POC: end-to-end PASS.** All stages
  (fetch, FreeCAD import/STEP export/round-trip, URDF authoring, Webots
  import, wheel articulation) completed successfully; the STEP round-trip
  volume/topology issue in section 3 is a real fidelity gap worth carrying
  into #9's design, but did not block this pipeline (URDF collision
  geometry uses primitive shapes, not the STEP-derived solids, so it was
  not exposed downstream in this POC - it would matter for anything that
  actually needs mass properties from the round-tripped shape).
