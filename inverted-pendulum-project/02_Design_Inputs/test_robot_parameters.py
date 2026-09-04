#!/usr/bin/env python3
"""
Tests for Stage 0's `robot_parameters.py` loader (Issue #9).

Pure Python / pytest, no FreeCAD dependency and no FreeCAD process required —
per CLAUDE.md, FreeCAD is never imported into the `pendulum-tools` mamba env.

Usage:
    mamba run -n pendulum-tools python3 -m pytest 02_Design_Inputs/test_robot_parameters.py -v
    # or, with the env already active:
    python3 -m pytest test_robot_parameters.py -v
"""

import sys
from pathlib import Path
from textwrap import dedent

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

from robot_parameters import (  # noqa: E402
    DEFAULT_YAML_PATH,
    SCHEMA_VERSION,
    ChassisSpec,
    PendulumSpec,
    RobotParameters,
    RobotParametersError,
    WheelSpec,
    load_robot_parameters,
)


# ---------------------------------------------------------------------------
# Loading the real, checked-in robot_parameters.yaml
# ---------------------------------------------------------------------------


def test_default_yaml_file_exists():
    assert DEFAULT_YAML_PATH.is_file()


def test_load_default_file_succeeds():
    params = load_robot_parameters()
    assert isinstance(params, RobotParameters)


def test_default_load_uses_default_path_by_default():
    """load_robot_parameters() with no args reads DEFAULT_YAML_PATH."""
    default_params = load_robot_parameters()
    explicit_params = load_robot_parameters(DEFAULT_YAML_PATH)
    assert default_params.to_dict() == explicit_params.to_dict()


def test_schema_version_matches_loader():
    params = load_robot_parameters()
    assert params.schema_version == SCHEMA_VERSION


def test_status_is_a_known_value():
    params = load_robot_parameters()
    assert params.status in {"PLACEHOLDER", "MEASURED"}


def test_status_is_currently_placeholder():
    """Stage 0 ships with provisional values only (Issue #9, Decision #3).
    This test is a deliberate tripwire: if this ever fails because someone
    flips status to MEASURED, that's a sign this test (and the range checks
    below, calibrated to *plausible placeholders*) should be revisited
    alongside real hardware numbers, not treated as a regression."""
    params = load_robot_parameters()
    assert params.status == "PLACEHOLDER"


# ---------------------------------------------------------------------------
# Schema / type shape
# ---------------------------------------------------------------------------


def test_component_types():
    params = load_robot_parameters()
    assert isinstance(params.chassis, ChassisSpec)
    assert isinstance(params.wheel, WheelSpec)
    assert isinstance(params.pendulum, PendulumSpec)


def test_links_reference_known_components():
    params = load_robot_parameters()
    assert params.links  # at least one mapping
    for link_name, component in params.links.items():
        assert component in {"chassis", "wheel", "pendulum"}, (
            f"{link_name} maps to unknown component {component!r}"
        )


def test_expected_link_names_present():
    """Stage 1 (Issue #9) is expected to create exactly these four links."""
    params = load_robot_parameters()
    assert set(params.links) == {"Base_Link", "Wheel_Left", "Wheel_Right", "Pendulum_Link"}
    assert params.links["Base_Link"] == "chassis"
    assert params.links["Wheel_Left"] == "wheel"
    assert params.links["Wheel_Right"] == "wheel"
    assert params.links["Pendulum_Link"] == "pendulum"


def test_component_for_link():
    params = load_robot_parameters()
    assert params.component_for_link("Base_Link") is params.chassis
    assert params.component_for_link("Wheel_Left") is params.wheel
    assert params.component_for_link("Wheel_Right") is params.wheel
    assert params.component_for_link("Pendulum_Link") is params.pendulum


def test_component_for_unknown_link_raises():
    params = load_robot_parameters()
    with pytest.raises(RobotParametersError):
        params.component_for_link("Nonexistent_Link")


def test_target_mass_for_link_kg():
    params = load_robot_parameters()
    assert params.target_mass_for_link_kg("Wheel_Left") == params.wheel.target_mass_kg


def test_to_dict_is_json_serializable():
    import json

    params = load_robot_parameters()
    data = params.to_dict()
    # Should not raise, and round-trip cleanly.
    reloaded = json.loads(json.dumps(data))
    assert reloaded["chassis"]["length_mm"] == params.chassis.length_mm


# ---------------------------------------------------------------------------
# Range / sanity checks on the placeholder values themselves
#
# Per Issue #9's consolidated plan, Decision #3: "~65-80mm wheel dia,
# ~120x80mm chassis". These bounds are intentionally generous around that
# guidance -- they exist to catch an accidental typo (e.g. a stray extra
# digit) or unit mistake (mm vs m/cm), not to pin exact numbers that are
# expected to change once real hardware is sourced.
# ---------------------------------------------------------------------------


def test_wheel_diameter_within_decision_3_placeholder_range():
    params = load_robot_parameters()
    assert 50.0 <= params.wheel.diameter_mm <= 100.0


def test_chassis_dimensions_within_decision_3_placeholder_range():
    params = load_robot_parameters()
    assert 80.0 <= params.chassis.length_mm <= 160.0
    assert 50.0 <= params.chassis.width_mm <= 110.0


def test_wheel_width_smaller_than_diameter():
    params = load_robot_parameters()
    assert params.wheel.width_mm < params.wheel.diameter_mm


def test_wheel_track_wider_than_chassis_is_plausible():
    """Track (wheel centre-to-centre) should be on the same order as the
    chassis width -- not a strict engineering constraint yet at Stage 0
    (no real axle/motor mount design exists), just a sanity bound against
    an obviously wrong placeholder (e.g. a 10x typo)."""
    params = load_robot_parameters()
    assert params.chassis.width_mm * 0.5 <= params.wheel.track_mm <= params.chassis.width_mm * 3.0


def test_all_target_masses_are_positive_and_small():
    """Placeholder masses should be in a plausible desktop-robot range
    (grams to a few hundred grams per component), not e.g. accidentally
    expressed in grams-as-kg."""
    params = load_robot_parameters()
    for name, spec in (
        ("chassis", params.chassis),
        ("wheel", params.wheel),
        ("pendulum", params.pendulum),
    ):
        assert 0.0 < spec.target_mass_kg < 2.0, f"{name}.target_mass_kg out of plausible range"


def test_all_densities_are_positive_and_plausible():
    """Loose bounds covering common 3D-printing plastics through light
    metals (kg/m^3) -- catches unit slips (e.g. g/cm^3 entered directly)."""
    params = load_robot_parameters()
    for name, spec in (
        ("chassis", params.chassis),
        ("wheel", params.wheel),
        ("pendulum", params.pendulum),
    ):
        assert 100.0 < spec.density_kg_m3 < 10000.0, f"{name}.density_kg_m3 out of plausible range"


# ---------------------------------------------------------------------------
# Round-trip: write a temp YAML, load it back, compare
# ---------------------------------------------------------------------------


def _valid_minimal_yaml_dict():
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "PLACEHOLDER",
        "chassis": {
            "length_mm": 100.0,
            "width_mm": 70.0,
            "height_mm": 30.0,
            "material": "PLA",
            "density_kg_m3": 1200.0,
            "target_mass_kg": 0.2,
        },
        "wheel": {
            "diameter_mm": 65.0,
            "width_mm": 12.0,
            "track_mm": 100.0,
            "material": "PLA",
            "density_kg_m3": 1200.0,
            "target_mass_kg": 0.025,
        },
        "pendulum": {
            "arm_length_mm": 150.0,
            "pivot_height_mm": 55.0,
            "material": "Aluminum_6061",
            "density_kg_m3": 2700.0,
            "target_mass_kg": 0.1,
        },
        "links": {
            "Base_Link": "chassis",
            "Wheel_Left": "wheel",
            "Wheel_Right": "wheel",
            "Pendulum_Link": "pendulum",
        },
    }


def test_round_trip_load(tmp_path):
    data = _valid_minimal_yaml_dict()
    yaml_path = tmp_path / "robot_parameters.yaml"
    yaml_path.write_text(yaml.safe_dump(data), encoding="utf-8")

    params = load_robot_parameters(yaml_path)

    assert params.schema_version == data["schema_version"]
    assert params.status == data["status"]
    assert params.chassis.length_mm == data["chassis"]["length_mm"]
    assert params.wheel.diameter_mm == data["wheel"]["diameter_mm"]
    assert params.pendulum.arm_length_mm == data["pendulum"]["arm_length_mm"]
    assert params.links == data["links"]


def test_round_trip_accepts_string_path(tmp_path):
    data = _valid_minimal_yaml_dict()
    yaml_path = tmp_path / "robot_parameters.yaml"
    yaml_path.write_text(yaml.safe_dump(data), encoding="utf-8")

    params = load_robot_parameters(str(yaml_path))
    assert isinstance(params, RobotParameters)


# ---------------------------------------------------------------------------
# Error handling
# ---------------------------------------------------------------------------


def test_missing_file_raises(tmp_path):
    missing = tmp_path / "does_not_exist.yaml"
    with pytest.raises(RobotParametersError):
        load_robot_parameters(missing)


def test_non_mapping_yaml_raises(tmp_path):
    yaml_path = tmp_path / "bad.yaml"
    yaml_path.write_text("- just\n- a\n- list\n", encoding="utf-8")
    with pytest.raises(RobotParametersError):
        load_robot_parameters(yaml_path)


def test_invalid_yaml_syntax_raises(tmp_path):
    yaml_path = tmp_path / "bad.yaml"
    yaml_path.write_text(
        dedent(
            """
            chassis: [unterminated
            """
        ),
        encoding="utf-8",
    )
    with pytest.raises(RobotParametersError):
        load_robot_parameters(yaml_path)


@pytest.mark.parametrize("missing_key", ["schema_version", "status", "chassis", "wheel", "pendulum"])
def test_missing_top_level_key_raises(tmp_path, missing_key):
    data = _valid_minimal_yaml_dict()
    del data[missing_key]
    yaml_path = tmp_path / "robot_parameters.yaml"
    yaml_path.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(RobotParametersError):
        load_robot_parameters(yaml_path)


@pytest.mark.parametrize("missing_key", ["length_mm", "width_mm", "height_mm", "material", "density_kg_m3", "target_mass_kg"])
def test_missing_chassis_key_raises(tmp_path, missing_key):
    data = _valid_minimal_yaml_dict()
    del data["chassis"][missing_key]
    yaml_path = tmp_path / "robot_parameters.yaml"
    yaml_path.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(RobotParametersError):
        load_robot_parameters(yaml_path)


def test_unknown_schema_version_raises(tmp_path):
    data = _valid_minimal_yaml_dict()
    data["schema_version"] = SCHEMA_VERSION + 999
    yaml_path = tmp_path / "robot_parameters.yaml"
    yaml_path.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(RobotParametersError):
        load_robot_parameters(yaml_path)


def test_unknown_status_raises(tmp_path):
    data = _valid_minimal_yaml_dict()
    data["status"] = "TOTALLY_MADE_UP"
    yaml_path = tmp_path / "robot_parameters.yaml"
    yaml_path.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(RobotParametersError):
        load_robot_parameters(yaml_path)


@pytest.mark.parametrize("bad_value", [0.0, -1.0])
def test_non_positive_density_raises(tmp_path, bad_value):
    data = _valid_minimal_yaml_dict()
    data["chassis"]["density_kg_m3"] = bad_value
    yaml_path = tmp_path / "robot_parameters.yaml"
    yaml_path.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(RobotParametersError):
        load_robot_parameters(yaml_path)


@pytest.mark.parametrize("bad_value", [0.0, -0.1])
def test_non_positive_target_mass_raises(tmp_path, bad_value):
    data = _valid_minimal_yaml_dict()
    data["wheel"]["target_mass_kg"] = bad_value
    yaml_path = tmp_path / "robot_parameters.yaml"
    yaml_path.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(RobotParametersError):
        load_robot_parameters(yaml_path)


def test_non_positive_dimension_raises(tmp_path):
    data = _valid_minimal_yaml_dict()
    data["wheel"]["diameter_mm"] = -5.0
    yaml_path = tmp_path / "robot_parameters.yaml"
    yaml_path.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(RobotParametersError):
        load_robot_parameters(yaml_path)


def test_empty_material_string_raises(tmp_path):
    data = _valid_minimal_yaml_dict()
    data["pendulum"]["material"] = "   "
    yaml_path = tmp_path / "robot_parameters.yaml"
    yaml_path.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(RobotParametersError):
        load_robot_parameters(yaml_path)


def test_link_referencing_unknown_component_raises(tmp_path):
    data = _valid_minimal_yaml_dict()
    data["links"]["Extra_Link"] = "not_a_real_component"
    yaml_path = tmp_path / "robot_parameters.yaml"
    yaml_path.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(RobotParametersError):
        load_robot_parameters(yaml_path)


def test_empty_links_mapping_raises(tmp_path):
    data = _valid_minimal_yaml_dict()
    data["links"] = {}
    yaml_path = tmp_path / "robot_parameters.yaml"
    yaml_path.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(RobotParametersError):
        load_robot_parameters(yaml_path)


def test_missing_links_key_is_allowed_but_then_fails_validation(tmp_path):
    """`links` is optional at the parse layer (defaults to {}) but empty
    links still fails RobotParameters.validate() -- Stage 1+ needs at least
    one link mapped to build anything."""
    data = _valid_minimal_yaml_dict()
    del data["links"]
    yaml_path = tmp_path / "robot_parameters.yaml"
    yaml_path.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(RobotParametersError):
        load_robot_parameters(yaml_path)


def test_chassis_not_a_mapping_raises(tmp_path):
    data = _valid_minimal_yaml_dict()
    data["chassis"] = "not-a-mapping"
    yaml_path = tmp_path / "robot_parameters.yaml"
    yaml_path.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(RobotParametersError):
        load_robot_parameters(yaml_path)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
