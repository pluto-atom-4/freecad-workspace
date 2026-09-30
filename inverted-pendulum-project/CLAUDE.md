# inverted-pendulum-project

Guidance loaded when working under this directory (moved from the root CLAUDE.md).

## FreeCAD Live Bridge — Known Limitations (freecad-mcp-workbench 0.6.2)

The live `mcp__freecad__*` bridge is occasionally borrowed by `inverted-pendulum-project` as an
ad-hoc human-review aid (its own pipeline stays headless-only). Found while doing so:

- **`get_screenshot` is broken** (`AttributeError: 'dict' object has no attribute '__name__'` on
  every call). Workaround: `execute_python` → `FreeCADGui.ActiveDocument.ActiveView.saveImage(path, w, h)`
  directly, then read the file back off disk.
- **`inspect_object` errors on `App::Part` container objects** (fine on `Part::Feature`/`Mesh::Feature`).
  Workaround: read `.Group`/`.Placement` etc. directly via `execute_python`.
- **Visibility and camera are GUI-only state** — no `ViewObject`/3D view exists in a true headless
  `freecadcmd` run. Guard any such code with `if not getattr(App, "GuiUp", False): return` (see
  `07_create_body_and_wheels.py`'s `_set_default_visibility()`/`_set_camera_framing()`). A camera
  set this way only embeds in the `.FCStd` if the save itself happens under a GUI.
- **A live GUI auto-tessellates shapes for display, which can shrink `Shape.BoundBox` reads** —
  observed a 70.00mm cylinder reading ~69.90mm. Validate dimensions against a true headless run,
  not a live-bridge session; when tessellating in a script, always tessellate a `.copy()` of the
  shape, never `obj.Shape` itself (same contamination happens self-inflicted, even headlessly).
- **To produce a fully-viewable `.FCStd` (visible objects + framed camera baked in), run the
  generator script itself through the bridge** — its own `App.GuiUp`-guarded code needs a GUI to
  fire. `execute_python(code="exec(open(path).read())")` alone silently no-ops: `__name__` in
  that context is `"builtins"`, not `"__main__"`, so the script's `if __name__ == "__main__":`
  guard never runs. Force it explicitly:
  `exec(compile(open(path).read(), path, 'exec'), {'__name__': '__main__', '__file__': path})`.
- **A `Mesh::Feature`'s own `Placement` is ignored when nested in an `App::Part`** — only the
  immediate parent container's `getGlobalPlacement()` applied directly to its raw `.Mesh` data
  matches what actually renders (verified by isolating one mesh + one plate, comparing to a
  screenshot). `Part::Feature` follows the normal convention (own `Placement` composes
  normally); only `Mesh::Feature` has this quirk. Not fully explained, but reproducible.
- **`Mesh.transform()` with a reflection matrix (e.g. `Matrix().scale(-1,1,1)`) can silently
  produce wrongly-offset geometry** on some meshes (observed: one coordinate shifted by a large,
  consistent, unexplained amount) — a real bug, not a math error. Mirror via raw point data
  instead: negate the coordinate on every vertex from `mesh.Topology`, reverse each facet's
  vertex order to fix normals, rebuild with `Mesh.Mesh((points, facets))`.
- **`distToShape() == 0` doesn't distinguish touching from overlapping** — for a real collision
  check, use `shape_a.common(shape_b).Volume`; nonzero means genuine interpenetration, zero means
  clear (even if `distToShape` also read 0).
- **`Part.Compound` has no `.CenterOfMass` or `.MatrixOfInertia` attributes** (raises `AttributeError`
  on either), even though it exposes `.Volume`. A `Part.Compound` typically wraps simpler solids
  (e.g. a single `Part.Solid`). Workaround: extract `.Solids[0]` (or iterate `.Solids` for multi-body
  compounds) and call `.CenterOfMass`/`.MatrixOfInertia` on the `Part.Solid` directly. Confirmed
  empirically (issue #96): a `Part.Solid`'s `.MatrixOfInertia` is already about its own `CenterOfMass`,
  no reverse parallel-axis shift needed when combining multiple solids via the parallel-axis theorem.
- **A generator script that reuses FreeCAD objects by name (an idempotency convenience, so reruns don't duplicate/auto-rename objects) can silently keep stale data after you change an on-disk input file it originally imported from** — a rerun that finds the object already present skips re-reading the source file entirely, no error or warning. Confirmed instance: `inverted-pendulum-project/03_Parts/Generators/03_link_servo_to_assembly.py`'s `create_servo_body()` reuses `STS3032_Mount`'s mesh children by name and never re-imports the `.stl` once they exist — repairing the source STL (issue #76) and rerunning the pipeline kept the old mesh data all the way through to the final assembly until the cached objects were explicitly deleted first. See `inverted-pendulum-project/DESIGN.md`'s Known Limitations table for the exact fix. General rule: after replacing/repairing any file a "reuse existing objects" script consumes, verify the NEXT run actually re-imports (check its console output for "reusing" vs "created"/"imported", or compare a concrete property like `Mesh.CountFacets`/`Shape.BoundBox` against the new source) rather than trusting a clean rerun + passing tests as proof the new data propagated.
- **Assembly::JointGroup/Joint object creation (native Assembly workbench, used by `03_Parts/Generators/08_configure_assembly_joints.py`) segfaults in true headless `freecadcmd` 1.1.3 whenever the `JointObject` module is imported** — reproducible and isolated via bisection: `doc.addObject('Assembly::JointGroup', ...)` alone works fine headlessly; add `import JointObject` (needed for the Joint proxy class) and any subsequent Assembly/JointGroup object creation crashes the process with "Application unexpectedly terminated" and no Python traceback — not an exception catchable in the script. The same code runs without crashing through the live FreeCAD MCP bridge (GUI-backed, `App.GuiUp=True`). Until FreeCAD fixes this upstream, any script creating Assembly workbench Joint objects must run via the bridge's `execute_python` (forcing `__name__ == "__main__"` per this file's documented compile/exec trick), not a plain `freecadcmd -c` invocation.
- **A live `.FCStd` document's own mesh data and a separately-regenerated/re-centered STL asset used for export (e.g., `regenerate_servo_visual_mesh_local_frame.py`'s output) are DIFFERENT vertex sets in different local frames** — an export composition origin correct for one is not automatically correct for the other. Example: Issue #148 (servo visual mesh origin fix) required recentering the `feetech-STS3032-visual.stl` file independently, yielding a new BoundBox center; composing with the live `.FCStd` mesh's center instead produced a visually-correct but numerically-wrong placement in the exported URDF. Always verify export logic against the actual regenerated asset's frame, not the document-embedded mesh.
