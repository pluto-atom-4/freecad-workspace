#!/bin/bash
# Shared Stage 0/1/1b/3 asset-provisioning logic (issue #26 review finding
# #6). Both run_gui.sh and run_batch.sh source this file and call
# provision_assets before their respective Webots invocations, so a fix to
# this logic (skip-check strength, error handling, etc.) only needs to be
# made once.
#
# Not meant to be executed directly — source it:
#   source "$SCRIPT_DIR/_provision_assets.sh"
#   provision_assets
#
# Expects the caller to already have `set -uo pipefail` (or stricter) in
# effect; this file relies on unset-variable and pipefail semantics but
# does not set them itself so it doesn't surprise a caller with different
# needs.

# WEBOTS_DIR/POC_DIR are derived from this file's own location so sourcing
# works the same regardless of which caller script sources it.
PROVISION_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
POC_DIR="$(cd "$PROVISION_SCRIPT_DIR/.." && pwd)"

# Declared once, here — the sole consumer of FREECAD_BIN is the
# provisioning logic below (Stage 1 / Stage 1b).
FREECAD_BIN="${FREECAD_BIN:-freecadcmd}"

# Prerequisite chain, checked from the end backwards: the PROTO the world
# EXTERNPROTOs needs urdf/meshes/, which needs freecad/output/ (Stage 1/1b),
# which needs reference/ (Stage 0).
PROTO_FILE="$POC_DIR/webots/protos/TurtlebotPoc.proto"
REFERENCE_DIR="$POC_DIR/reference"
FREECAD_OUTPUT_DIR="$POC_DIR/freecad/output"
URDF_MESHES_DIR="$POC_DIR/urdf/meshes"

# provision_assets — idempotent. Skips any stage whose expected output
# already exists AND is non-empty (a truncated/corrupt artifact from an
# interrupted prior run must not be mistaken for a finished one — review
# finding #3). Exits the calling shell with a FATAL message on any failure,
# matching this POC's existing FATAL-and-exit-1 convention.
provision_assets() {
    if [ -s "$PROTO_FILE" ]; then
        echo "Found $PROTO_FILE — assets already provisioned, skipping Stage 0/1/1b/3."
        return 0
    fi

    echo ""
    echo "=================================================================="
    echo "webots/protos/TurtlebotPoc.proto not found (or empty) —"
    echo "auto-provisioning generated assets before launch (Stage 0 -> 1 -> 1b -> 3)."
    echo "=================================================================="

    # Stage 0 — fetch TurtleBot3 reference assets. Check the URDF plus each
    # individual mesh STL that 00_fetch_turtlebot3_assets.sh fetches (see its
    # FILES array) rather than just the containing meshes/ directory's
    # existence — a directory can exist but be incomplete/empty after an
    # interrupted prior run (review finding #1).
    if [ -s "$REFERENCE_DIR/turtlebot3_burger.urdf" ] \
        && [ -s "$REFERENCE_DIR/meshes/burger_base.stl" ] \
        && [ -s "$REFERENCE_DIR/meshes/left_tire.stl" ] \
        && [ -s "$REFERENCE_DIR/meshes/right_tire.stl" ]; then
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
    if [ -s "$FREECAD_OUTPUT_DIR/burger_base.step" ] \
        && [ -s "$FREECAD_OUTPUT_DIR/left_tire.step" ] \
        && [ -s "$FREECAD_OUTPUT_DIR/right_tire.step" ]; then
        echo ""
        echo "-- Stage 1: STEP files already present, skipping FreeCAD conversion. --"
    else
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
    if [ -s "$URDF_MESHES_DIR/burger_base_roundtrip.stl" ] \
        && [ -s "$URDF_MESHES_DIR/left_tire_roundtrip.stl" ] \
        && [ -s "$URDF_MESHES_DIR/right_tire_roundtrip.stl" ]; then
        echo ""
        echo "-- Stage 1b: round-tripped meshes already present in urdf/meshes/, skipping. --"
    else
        echo ""
        echo "-- Stage 1b: STEP round-trip check (FREECAD_BIN=$FREECAD_BIN). --"
        # Run freecadcmd and filter its noisy Wayland/Qt/EGL warnings from
        # the displayed output, but capture freecadcmd's OWN exit code via
        # PIPESTATUS rather than the pipeline's overall exit status — under
        # `set -o pipefail`, `grep -v` exits 1 whenever every line matched
        # the filter (i.e. it selected zero lines), which would otherwise
        # make this check fail even when freecadcmd itself exited 0 (review
        # finding #2).
        "$FREECAD_BIN" -c "__file__=r'$POC_DIR/freecad/02_roundtrip_check.py'; exec(open(__file__).read())" 2>&1 \
            | grep -v "Wayland\|Qt\|EGL"
        freecad_exit_code="${PIPESTATUS[0]}"
        if [ "$freecad_exit_code" -ne 0 ]; then
            echo "FATAL: Stage 1b (STEP round-trip check) failed (freecadcmd exited $freecad_exit_code)." >&2
            exit 1
        fi

        echo "-- Stage 1b: copying round-tripped meshes into urdf/meshes/ --"
        mkdir -p "$URDF_MESHES_DIR"
        for link in burger_base left_tire right_tire; do
            src="$FREECAD_OUTPUT_DIR/${link}_roundtrip.stl"
            dst="$URDF_MESHES_DIR/${link}_roundtrip.stl"
            # Stage 1b's own mesh re-export is a try/except that only warns
            # on failure and does not flip its overall status (see
            # freecad/02_roundtrip_check.py) — so Stage 1b can report
            # SUCCESS above with a missing/incomplete re-exported mesh.
            # Verify the source actually exists (and is non-empty) before
            # copying, and verify the copy itself succeeded, rather than
            # silently falling through to launching Webots with an
            # incomplete mesh set (review finding #1).
            if [ ! -s "$src" ]; then
                echo "FATAL: expected round-tripped mesh not found (or empty): $src" >&2
                echo "Stage 1b reported success but its per-link mesh re-export for" >&2
                echo "'$link' appears to have failed — see the Stage 1b output above" >&2
                echo "and freecad/output/roundtrip_report.json for the reexport_error." >&2
                exit 1
            fi
            if ! cp "$src" "$dst"; then
                echo "FATAL: failed to copy $src -> $dst" >&2
                exit 1
            fi
        done
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

    if [ ! -s "$PROTO_FILE" ]; then
        echo "FATAL: auto-provisioning completed but $PROTO_FILE still missing (or empty)." >&2
        exit 1
    fi

    echo ""
    echo "=================================================================="
    echo "Auto-provisioning complete."
    echo "=================================================================="
}
