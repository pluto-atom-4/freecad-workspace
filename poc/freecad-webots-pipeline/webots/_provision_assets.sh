#!/bin/bash
# Shared Stage 0/1/1b/3 asset-provisioning logic (issue #26 review finding
# #6). Both run_gui.sh and run_batch.sh source this file and call
# provision_assets before their respective Webots invocations, so a fix to
# this logic (skip-check strength, error handling, etc.) only needs to be
# made once.
#
# Not meant to be executed directly — source it:
#   source "$SCRIPT_DIR/_provision_assets.sh"
#   require_webots_bin
#   require_world_file "$WORLD"
#   provision_assets "$WORLD"
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

# all_nonempty FILE... — true iff every given file exists and is
# non-empty. Single helper for the "already present, non-empty" skip-check
# pattern used by each provisioning stage below, so adding a future
# stage/asset means adding one call, not hand-duplicating another
# multi-line `[ -s ... ] && [ -s ... ] && ...` chain (review finding #6).
all_nonempty() {
    local f
    for f in "$@"; do
        [ -s "$f" ] || return 1
    done
    return 0
}

# require_webots_bin — fail fast if WEBOTS_BIN isn't runnable, before
# either caller spends a multi-minute provisioning pipeline on a Webots
# invocation that was always going to fail at the very last step (review
# finding #3).
require_webots_bin() {
    if ! command -v "$WEBOTS_BIN" >/dev/null 2>&1 && [ ! -x "$WEBOTS_BIN" ]; then
        echo "FATAL: WEBOTS_BIN ($WEBOTS_BIN) not found or not executable." >&2
        echo "Set WEBOTS_BIN to a valid Webots binary, e.g.:" >&2
        echo "  export WEBOTS_BIN=/usr/local/bin/webots" >&2
        exit 1
    fi
}

# require_world_file WORLD — fail fast on a missing/typo'd world path
# before provisioning runs. Shared so run_gui.sh and run_batch.sh give the
# same message instead of two hand-maintained copies that drift (review
# finding #7; the two copies had already diverged in wording).
require_world_file() {
    local world="$1"
    if [ ! -f "$world" ]; then
        echo "World file not found: $world" >&2
        echo "See ../README.md \"Reproducing end to end\" / \"Watching it live\"." >&2
        exit 1
    fi
}

# provision_assets [WORLD] — idempotent. Skips any stage whose expected
# output already exists AND is non-empty (a truncated/corrupt artifact
# from an interrupted prior run must not be mistaken for a finished one —
# review finding #3, round 1). Exits the calling shell with a FATAL
# message on any failure, matching this POC's existing FATAL-and-exit-1
# convention.
#
# If WORLD is given and does not reference the generated TurtlebotPoc
# PROTO (via its EXTERNPROTO declaration), provisioning is skipped
# entirely — the fixed webots/protos/TurtlebotPoc.proto target has
# nothing to do with an unrelated world, so there's no reason to run the
# multi-minute Stage 0/1/1b/3 pipeline just because that PROTO happens to
# be missing (review finding #5). With no WORLD argument, this relevance
# check is skipped and provisioning proceeds unconditionally (preserves
# behavior for any future caller that doesn't have a world path handy).
provision_assets() {
    local world="${1:-}"
    if [ -n "$world" ] && ! grep -q "TurtlebotPoc" "$world" 2>/dev/null; then
        echo "$world does not reference the generated TurtlebotPoc PROTO — skipping auto-provisioning."
        return 0
    fi

    if [ -s "$PROTO_FILE" ]; then
        echo "Found $PROTO_FILE — assets already provisioned, skipping Stage 0/1/1b/3."
        return 0
    fi

    # Checked up front, before Stage 0 runs: a missing `mamba` is only
    # needed by Stage 3 at the very end of this chain, but failing to
    # provision at all is knowable immediately — no reason to burn minutes
    # on Stage 0/1/1b first only to fail right before the finish line
    # (review finding #4).
    if ! command -v mamba >/dev/null 2>&1; then
        echo "FATAL: mamba not found on PATH — cannot run urdf2webots (Stage 3) in the pendulum-tools env." >&2
        echo "See ../README.md Stage 3 for the manual command." >&2
        exit 1
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
    # interrupted prior run (review finding #1, round 2).
    if all_nonempty \
        "$REFERENCE_DIR/turtlebot3_burger.urdf" \
        "$REFERENCE_DIR/meshes/burger_base.stl" \
        "$REFERENCE_DIR/meshes/left_tire.stl" \
        "$REFERENCE_DIR/meshes/right_tire.stl"; then
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
    if all_nonempty \
        "$FREECAD_OUTPUT_DIR/burger_base.step" \
        "$FREECAD_OUTPUT_DIR/left_tire.step" \
        "$FREECAD_OUTPUT_DIR/right_tire.step"; then
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
        # 01_import_and_export_step.sh's own success check only confirms its
        # JSON report was written, not that every expected .step file it
        # reports on actually landed — a partial FreeCAD failure with a
        # stale/incomplete report could still exit 0. Re-verify the actual
        # outputs this stage promises before trusting them, the same way
        # Stage 1b's mesh copies are verified below (review finding #2).
        if ! all_nonempty \
            "$FREECAD_OUTPUT_DIR/burger_base.step" \
            "$FREECAD_OUTPUT_DIR/left_tire.step" \
            "$FREECAD_OUTPUT_DIR/right_tire.step"; then
            echo "FATAL: Stage 1 reported success but one or more expected .step files" >&2
            echo "are missing (or empty) in $FREECAD_OUTPUT_DIR." >&2
            echo "See freecad/output/stage1_conversion_report.json for details." >&2
            exit 1
        fi
    fi

    # Stage 1b — STEP round-trip check, producing the mesh re-exports that
    # urdf/turtlebot3_poc.urdf actually references.
    if all_nonempty \
        "$URDF_MESHES_DIR/burger_base_roundtrip.stl" \
        "$URDF_MESHES_DIR/left_tire_roundtrip.stl" \
        "$URDF_MESHES_DIR/right_tire_roundtrip.stl"; then
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
        # finding #2, round 1).
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
            # incomplete mesh set (review finding #1, round 1).
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
    # (mamba's own presence was already checked up front, above.)
    echo ""
    echo "-- Stage 3: generating webots/protos/TurtlebotPoc.proto via urdf2webots (pendulum-tools env). --"
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
