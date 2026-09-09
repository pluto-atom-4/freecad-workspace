#!/usr/bin/env python3
"""
Tests for Stage 1's `07_create_body_and_wheels.py` (Issue #9).

Two layers, mirroring the existing dual-layer pattern in this directory
(`test_robot_parameters.py` for pure-Python; `test_05_integration_live.py`
for a `freecadcmd`-driven end-to-end check):

1. Pure-Python tests (no FreeCAD import, no FreeCAD process) -- verify that
   `07_create_body_and_wheels.py`'s hardcoded geometry logic (chassis/wheel
   placement math) is consistent with Stage 0's `robot_parameters.yaml`
   values, and that the script's own constants match its documented
   conventions. Always run.

2. `freecadcmd`-driven integration test -- actually runs
   `07_create_body_and_wheels.py` as a subprocess via a headless FreeCAD
   binary and asserts on the resulting `07_body_wheels_metadata.json`
   (object names present, dimensions within tolerance, triangle count under
   budget). This is a `pytest` test (unlike `test_05_integration_live.py`,
   which must itself run *inside* FreeCAD's Python and can't use pytest) --
   it shells out to `freecadcmd` from plain Python, so it can live in the
   `pendulum-tools` pytest suite and doesn't need a live GUI/MCP bridge for
   CI or automated verification (see Issue #9's Stage 1 Output Verification
   Plan comment, Section 5, on the live bridge not being authoritative for
   the triangle-count figure). Skipped (not failed) if no FreeCAD binary is
   available in this environment.

Usage:
    mamba run -n pendulum-tools python3 -m pytest -q test_07_body_wheels_geometry.py
    # or, with the env already active:
    python3 -m pytest -q test_07_body_wheels_geometry.py
"""

import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

SCRIPT_DIR = Path(__file__).resolve().parent
_DESIGN_INPUTS_DIR = SCRIPT_DIR.parent.parent / "02_Design_Inputs"
sys.path.insert(0, str(_DESIGN_INPUTS_DIR))

from robot_parameters import load_robot_parameters  # noqa: E402

GENERATOR_SCRIPT = SCRIPT_DIR / "07_create_body_and_wheels.py"
OUTPUT_FCSTD = SCRIPT_DIR / "robot_body_wheels.FCStd"
METADATA_JSON = SCRIPT_DIR / "07_body_wheels_metadata.json"
SOURCE_DOC = SCRIPT_DIR / "plates_servo_assembled.FCStd"

# All top-level objects 07_create_body_and_wheels.py creates -- what
# ends up in the metadata JSON's `links` and what the source text must
# literally mention.
GENERATED_OBJECT_NAMES = {"Base_Link", "Wheel_Left", "Wheel_Right", "Pendulum_Link", "Pendulum_Link_Right"}

# Subset robot_parameters.yaml's `links:` mapping must cover.
# Pendulum_Link_Right is deliberately excluded: Stage 0 predates it, and
# the generator reuses Pendulum_Link's own mapping for it instead of
# requiring a separate YAML entry (see build_pendulum_link_right()'s
# target_mass_kg handling).
REQUIRED_LINK_NAMES = {"Base_Link", "Wheel_Left", "Wheel_Right", "Pendulum_Link"}
TRIANGLE_BUDGET = 5000
DIMENSION_TOLERANCE_MM = 0.05


# ---------------------------------------------------------------------------
# Layer 1: Pure-Python tests -- no FreeCAD required
# ---------------------------------------------------------------------------


def test_robot_parameters_chassis_dims_are_placeholder_expected_values():
    """Sanity-pin the values 07_create_body_and_wheels.py's Base_Link will
    be built from, so a future robot_parameters.yaml edit that silently
    changes these is caught here too (not just in Stage 0's own tests)."""
    params = load_robot_parameters()
    assert params.chassis.length_mm == 120.0
    assert params.chassis.width_mm == 80.0
    assert params.chassis.height_mm == 40.0


def test_robot_parameters_wheel_dims_are_placeholder_expected_values():
    params = load_robot_parameters()
    assert params.wheel.diameter_mm == 70.0
    assert params.wheel.width_mm == 15.0
    assert params.wheel.track_mm == 110.0


def test_robot_parameters_links_mapping_covers_stage1_object_names():
    """07_create_body_and_wheels.py creates exactly these four link names;
    robot_parameters.yaml's `links:` mapping must know about all of them
    (component_for_link()/target_mass_for_link_kg() are called for each)."""
    params = load_robot_parameters()
    assert REQUIRED_LINK_NAMES.issubset(set(params.links.keys()))
    for name in REQUIRED_LINK_NAMES:
        # Must not raise -- component_for_link()/target_mass_for_link_kg()
        # are exactly what 07_create_body_and_wheels.py calls per link.
        params.component_for_link(name)
        mass = params.target_mass_for_link_kg(name)
        assert mass > 0


def test_wheel_track_and_radius_geometry_math():
    """Mirror 07_create_body_and_wheels.py's build_wheel() placement math in
    pure Python (no FreeCAD) so the wheel-center/track arithmetic is
    independently verified."""
    params = load_robot_parameters()
    wheel = params.wheel
    radius = wheel.diameter_mm / 2.0
    track_half = wheel.track_mm / 2.0

    left_center_y = -track_half
    right_center_y = +track_half

    assert radius == pytest.approx(35.0)
    assert left_center_y == pytest.approx(-55.0)
    assert right_center_y == pytest.approx(55.0)
    # Symmetric about the centerline
    assert left_center_y + right_center_y == pytest.approx(0.0)
    # Track (center-to-center) matches robot_parameters.yaml
    assert (right_center_y - left_center_y) == pytest.approx(wheel.track_mm)


def test_chassis_and_wheel_do_not_overlap_in_y():
    """Mirror the script's own "no interpenetration" validation in pure
    Python: chassis half-width must be narrower than the wheels' inner
    face, given the current geometry constants."""
    params = load_robot_parameters()
    chassis = params.chassis
    wheel = params.wheel

    chassis_half_width = chassis.width_mm / 2.0
    track_half = wheel.track_mm / 2.0
    wheel_inner_face_y = track_half - wheel.width_mm / 2.0

    assert chassis_half_width < wheel_inner_face_y, (
        f"chassis half-width {chassis_half_width} mm would overlap wheel "
        f"inner face at {wheel_inner_face_y} mm given current "
        f"robot_parameters.yaml values"
    )


def test_generator_script_exists_and_has_expected_object_names_in_source():
    """Cheap static check (no FreeCAD needed): the generator script's source
    text mentions the exact, case-sensitive object names it must create."""
    assert GENERATOR_SCRIPT.is_file()
    text = GENERATOR_SCRIPT.read_text()
    for name in GENERATED_OBJECT_NAMES:
        assert f'"{name}"' in text, f"{name!r} not found as a literal in {GENERATOR_SCRIPT.name}"


# ---------------------------------------------------------------------------
# Layer 2: freecadcmd-driven integration test
# ---------------------------------------------------------------------------


def _resolve_freecad_bin():
    """Same resolution order documented in mamba-envs.yaml / CLAUDE.md:
    FREECAD_BIN env var, else `freecadcmd` on PATH."""
    env_bin = os.environ.get("FREECAD_BIN")
    if env_bin:
        return env_bin if shutil.which(env_bin) or Path(env_bin).is_file() else None
    return shutil.which("freecadcmd")


FREECAD_BIN = _resolve_freecad_bin()

skip_reason = (
    "No FreeCAD binary available (set FREECAD_BIN or put freecadcmd on PATH) "
    "-- skipping freecadcmd integration test, per the dual-layer pattern "
    "(pure-Python tests above still run without FreeCAD)."
)


@pytest.mark.skipif(FREECAD_BIN is None, reason=skip_reason)
@pytest.mark.skipif(not SOURCE_DOC.is_file(), reason=f"{SOURCE_DOC.name} not found")
def test_freecadcmd_run_produces_valid_output():
    """Actually run 07_create_body_and_wheels.py via freecadcmd and assert
    on the resulting JSON metadata -- this is the authoritative source for
    the triangle-count figure (see Issue #9's Stage 1 Output Verification
    Plan, Section 5: the live MCP bridge is not authoritative for it)."""
    for path in (OUTPUT_FCSTD, METADATA_JSON):
        if path.exists():
            path.unlink()

    # NOTE: this FreeCAD 1.1.3 build's `freecadcmd` does not honor a plain
    # positional script argument's `__name__ == "__main__"` guard (verified
    # empirically: __name__ is set to the script's module name, not
    # "__main__", so `if __name__ == "__main__": sys.exit(main())` never
    # fires and the script silently no-ops). `-c "exec(open(...).read())"`
    # preserves `__name__ == "__main__"` and is what actually runs the
    # script end-to-end in this environment -- matching the alternative
    # invocation already documented for Phase 1 in README.md.
    cmd = [FREECAD_BIN, "-c", f"exec(open({str(GENERATOR_SCRIPT)!r}).read())"]
    result = subprocess.run(
        cmd,
        cwd=str(SCRIPT_DIR),
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        timeout=180,
    )

    assert result.returncode == 0, (
        f"07_create_body_and_wheels.py failed (exit {result.returncode}).\n"
        f"stdout:\n{result.stdout}\n\nstderr:\n{result.stderr}"
    )
    assert METADATA_JSON.is_file(), "07_body_wheels_metadata.json was not created"
    assert OUTPUT_FCSTD.is_file(), "robot_body_wheels.FCStd was not created"

    with open(METADATA_JSON) as f:
        metadata = json.load(f)

    assert metadata["phase"] == 7
    assert metadata["issue"] == 9
    assert metadata["all_validations_passed"] is True

    links = metadata["links"]
    assert GENERATED_OBJECT_NAMES == set(links.keys())

    params = load_robot_parameters()

    base_link_dims = links["Base_Link"]["dimensions_mm"]
    assert base_link_dims["length_mm"] == pytest.approx(params.chassis.length_mm, abs=DIMENSION_TOLERANCE_MM)
    assert base_link_dims["width_mm"] == pytest.approx(params.chassis.width_mm, abs=DIMENSION_TOLERANCE_MM)
    assert base_link_dims["height_mm"] == pytest.approx(params.chassis.height_mm, abs=DIMENSION_TOLERANCE_MM)

    for wheel_name in ("Wheel_Left", "Wheel_Right"):
        wheel_dims = links[wheel_name]["dimensions_mm"]
        assert wheel_dims["diameter_mm"] == pytest.approx(params.wheel.diameter_mm, abs=DIMENSION_TOLERANCE_MM)
        assert wheel_dims["width_mm"] == pytest.approx(params.wheel.width_mm, abs=DIMENSION_TOLERANCE_MM)

    wl_y = links["Wheel_Left"]["bounding_box_mm"]
    wr_y = links["Wheel_Right"]["bounding_box_mm"]
    wl_center_y = (wl_y["y_min"] + wl_y["y_max"]) / 2.0
    wr_center_y = (wr_y["y_min"] + wr_y["y_max"]) / 2.0
    assert (wr_center_y - wl_center_y) == pytest.approx(params.wheel.track_mm, abs=DIMENSION_TOLERANCE_MM)
    assert (wl_center_y + wr_center_y) == pytest.approx(0.0, abs=DIMENSION_TOLERANCE_MM)

    geometry_stats = metadata["geometry_stats"]
    assert geometry_stats["new_primitive_triangle_count"] < TRIANGLE_BUDGET
    assert geometry_stats["new_primitive_triangle_budget_ok"] is True

    assert links["Pendulum_Link"]["kind"] == "reused_subassembly"

    for v in metadata["validations"]:
        assert v["passed"] is True, f"Validation failed: {v['check']}: {v['details']}"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
