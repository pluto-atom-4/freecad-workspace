# CLAUDE.md — FreeCAD-Webots Pipeline POC

Project-specific guidance for Claude Code when working in this directory. Inherits the workspace root's `../../CLAUDE.md`.

## Simulation Validation Workflow

When validating a change to a Webots world/controller/URDF in this POC (e.g. after a robot-behavior fix), **prefer a human visually inspecting the simulation live in the Webots GUI over running headless batch mode**:

- Use `./webots/run_gui.sh webots/worlds/<world>.wbt` and ask the human to watch the simulation and report what they observe (does the robot move as expected, does it look physically correct, etc.) — do not assume a headless run's PASS/FAIL verdict alone is sufficient confirmation of correct *physical* behavior (chassis translation, visual framing, contact behavior). A controller's programmatic PASS/FAIL check (e.g. `wheel_articulation_check.py`) is a useful automated signal but has already been shown in this project's history to miss real bugs (e.g. it originally only checked wheel-joint rotation, not chassis translation, and passed while the robot was completely stuck in place — see issue #32).
- Only run `./webots/run_batch.sh webots/worlds/<world>.wbt` (headless) when the human has explicitly asked for headless/automated validation, or as a quick structural/regression smoke check (e.g. confirming EXTERNPROTO resolution, confirming the controller doesn't crash) — not as a substitute for a human's live visual confirmation of simulation *behavior* after a physics/kinematics-affecting change.
- When implementing a fix, do the file edits and (if needed) any non-simulation regeneration steps (e.g. `urdf2webots` PROTO regeneration, structural URDF tests) yourself, but leave the actual live-Webots run to the human unless they've said otherwise.
