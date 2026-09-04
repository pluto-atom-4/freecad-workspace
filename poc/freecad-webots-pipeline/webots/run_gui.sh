#!/bin/bash
# Stage 3 (GUI) — launch the generated turtlebot3_poc Webots world live, in
# realtime mode, so a human can watch it (issue #26).
#
# Usage: ./run_gui.sh [path/to/world.wbt]
#   Defaults to worlds/turtlebot3_poc.wbt (the urdf2webots-generated world).
#
# Unlike run_batch.sh, this launches Webots with a visible window
# (--mode=realtime, no --batch/--no-rendering/--minimize) and requires a
# real DISPLAY — it deliberately does NOT fall back to xvfb-run, since
# Xvfb has no visible framebuffer and would silently defeat the point of
# this script.
#
# A fresh checkout of main lacks the gitignored generated assets this world
# needs (reference/, freecad/output/, urdf/meshes/, webots/protos/) — see
# ../README.md "Reproducing end to end". Rather than failing fast on a
# missing webots/protos/TurtlebotPoc.proto, this script auto-provisions
# them by running the existing Stage 0/1/1b/3 scripts in order. It only
# invokes those scripts — it does not duplicate their logic.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
POC_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORLD="${1:-$SCRIPT_DIR/worlds/turtlebot3_poc.wbt}"
WEBOTS_BIN="${WEBOTS_BIN:-/usr/local/bin/webots}"

if [ ! -f "$WORLD" ]; then
    echo "World file not found: $WORLD"
    echo "See ../README.md."
    exit 1
fi

# --- DISPLAY check --------------------------------------------------------
# This script's entire purpose is a visible window; unlike run_batch.sh, a
# headless/xvfb fallback here would silently defeat that. Fail loudly.
if [ -z "${DISPLAY:-}" ]; then
    echo "FATAL: \$DISPLAY is not set." >&2
    echo "run_gui.sh launches Webots with a visible window (--mode=realtime)" >&2
    echo "and requires a real X display — it will not fall back to xvfb-run" >&2
    echo "(Xvfb has no visible framebuffer, so that would defeat the point)." >&2
    echo "Set DISPLAY to your active X session (e.g. export DISPLAY=:1) and re-run." >&2
    exit 1
fi
echo "Using DISPLAY=$DISPLAY"

# --- Stage 0/1/1b/3 auto-provisioning -------------------------------------
# Prerequisite chain, checked from the end backwards: the PROTO the world
# EXTERNPROTOs needs urdf/meshes/, which needs freecad/output/ (Stage 1/1b),
# which needs reference/ (Stage 0).
PROTO_FILE="$POC_DIR/webots/protos/TurtlebotPoc.proto"
REFERENCE_DIR="$POC_DIR/reference"
FREECAD_OUTPUT_DIR="$POC_DIR/freecad/output"
URDF_MESHES_DIR="$POC_DIR/urdf/meshes"

if [ -f "$PROTO_FILE" ]; then
    echo "Found $PROTO_FILE — assets already provisioned, skipping Stage 0/1/1b/3."
else
    echo ""
    echo "=================================================================="
    echo "webots/protos/TurtlebotPoc.proto not found — auto-provisioning"
    echo "generated assets before launch (Stage 0 -> 1 -> 1b -> 3)."
    echo "=================================================================="

    # Stage 0 — fetch TurtleBot3 reference assets.
    if [ -d "$REFERENCE_DIR/meshes" ] && [ -f "$REFERENCE_DIR/turtlebot3_burger.urdf" ]; then
        echo ""
        echo "-- Stage 0: reference assets already present, skipping fetch. --"
    else
        echo ""
        echo "-- Stage 0: fetching TurtleBot3 reference assets (00_fetch_turtlebot3_assets.sh)... --"
        if ! "$POC_DIR/00_fetch_turtlebot3_assets.sh"; then
            echo "FATAL: Stage 0 (asset fetch) failed." >&2
            exit 1
        fi
    fi

    # Stage 1 — FreeCAD Mesh -> Part.Shape -> STEP export.
    if [ -f "$FREECAD_OUTPUT_DIR/burger_base.step" ] \
        && [ -f "$FREECAD_OUTPUT_DIR/left_tire.step" ] \
        && [ -f "$FREECAD_OUTPUT_DIR/right_tire.step" ]; then
        echo ""
        echo "-- Stage 1: STEP files already present, skipping FreeCAD conversion. --"
    else
        FREECAD_BIN="${FREECAD_BIN:-freecadcmd}"
        echo ""
        echo "-- Stage 1: FreeCAD import & STEP export (FREECAD_BIN=$FREECAD_BIN). --"
        echo "   This is known-slow — ~74s for burger_base alone (see ../findings/FINDINGS.md)."
        if ! FREECAD_BIN="$FREECAD_BIN" "$POC_DIR/freecad/01_import_and_export_step.sh"; then
            echo "FATAL: Stage 1 (FreeCAD STEP export) failed." >&2
            echo "Is FREECAD_BIN set to a valid headless FreeCAD binary? e.g.:" >&2
            echo "  export FREECAD_BIN=~/.local/opt/freecad-1.1.3/usr/bin/freecadcmd" >&2
            exit 1
        fi
    fi

    # Stage 1b — STEP round-trip check, producing the mesh re-exports that
    # urdf/turtlebot3_poc.urdf actually references.
    if [ -f "$URDF_MESHES_DIR/burger_base_roundtrip.stl" ] \
        && [ -f "$URDF_MESHES_DIR/left_tire_roundtrip.stl" ] \
        && [ -f "$URDF_MESHES_DIR/right_tire_roundtrip.stl" ]; then
        echo ""
        echo "-- Stage 1b: round-tripped meshes already present in urdf/meshes/, skipping. --"
    else
        FREECAD_BIN="${FREECAD_BIN:-freecadcmd}"
        echo ""
        echo "-- Stage 1b: STEP round-trip check (FREECAD_BIN=$FREECAD_BIN). --"
        if ! "$FREECAD_BIN" -c "__file__=r'$POC_DIR/freecad/02_roundtrip_check.py'; exec(open(__file__).read())" 2>&1 | grep -v "Wayland\|Qt\|EGL"; then
            echo "FATAL: Stage 1b (STEP round-trip check) failed." >&2
            exit 1
        fi
        echo "-- Stage 1b: copying round-tripped meshes into urdf/meshes/ --"
        mkdir -p "$URDF_MESHES_DIR"
        cp "$FREECAD_OUTPUT_DIR/burger_base_roundtrip.stl" "$URDF_MESHES_DIR/"
        cp "$FREECAD_OUTPUT_DIR/left_tire_roundtrip.stl" "$URDF_MESHES_DIR/"
        cp "$FREECAD_OUTPUT_DIR/right_tire_roundtrip.stl" "$URDF_MESHES_DIR/"
    fi

    # Stage 3 — urdf2webots PROTO generation (pendulum-tools mamba env).
    echo ""
    echo "-- Stage 3: generating webots/protos/TurtlebotPoc.proto via urdf2webots (pendulum-tools env). --"
    if ! command -v mamba >/dev/null 2>&1; then
        echo "FATAL: mamba not found on PATH — cannot run urdf2webots in the pendulum-tools env." >&2
        echo "See ../README.md Stage 3 for the manual command." >&2
        exit 1
    fi
    mkdir -p "$POC_DIR/webots/protos"
    if ! mamba run -n pendulum-tools python3 -m urdf2webots.importer \
        --input="$POC_DIR/urdf/turtlebot3_poc.urdf" \
        --output="$PROTO_FILE" \
        --target=R2025a; then
        echo "FATAL: Stage 3 (urdf2webots conversion) failed." >&2
        echo "Is urdf2webots installed in pendulum-tools? See ../README.md Stage 3:" >&2
        echo "  mamba run -n pendulum-tools pip install urdf2webots" >&2
        exit 1
    fi

    if [ ! -f "$PROTO_FILE" ]; then
        echo "FATAL: auto-provisioning completed but $PROTO_FILE still missing." >&2
        exit 1
    fi

    echo ""
    echo "=================================================================="
    echo "Auto-provisioning complete."
    echo "=================================================================="
fi

echo ""
echo "Launching Webots (realtime, GUI) on: $WORLD"
echo "Press Play in the Webots window if the simulation starts paused."
exec "$WEBOTS_BIN" --mode=realtime "$WORLD"
