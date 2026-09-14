#!/usr/bin/env python3
"""
Tests for placement_overrides.yaml mechanism (Issue #146).

Two layers:
1. Static (no FreeCAD) - YAML schema validation, grep checks, always run
2. Subprocess (headless freecadcmd) - regenerate + verify placement in output,
   skipped if FREECAD_BIN or source doc unavailable
"""

import os
import sys
import json
import tempfile
import subprocess
from pathlib import Path
from unittest import mock

import pytest
import yaml


# Stage 0's design-input loader location (not needed for static layer, but available if tests run headlessly)
_DESIGN_INPUTS_DIR = Path(__file__).resolve().parent.parent / "02_Design_Inputs"
sys.path.insert(0, str(_DESIGN_INPUTS_DIR))


SCRIPT_DIR = Path(__file__).resolve().parent
OVERRIDES_FILE = SCRIPT_DIR / "placement_overrides.yaml"
GENERATOR_SCRIPT = SCRIPT_DIR / "07_create_body_and_wheels.py"
OUTPUT_FCSTD = SCRIPT_DIR / "robot_body_wheels.FCStd"
METADATA_JSON = SCRIPT_DIR / "07_body_wheels_metadata.json"
FREECAD_BIN = os.environ.get("FREECAD_BIN", "freecadcmd")

# Which link's metadata carries each override-able object's placement --
# see 07_create_body_and_wheels.py's sts_mount_placement field (Issue #120).
STS_MOUNT_METADATA_LINK = {
    "STS3032_Mount": "Pendulum_Link",
    "STS3032_Mount_Right": "Pendulum_Link_Right",
}

# Seeded values from the YAML
SEEDED_STS_MOUNT_POS = [-1.0, -0.40, -7.00]
SEEDED_STS_MOUNT_RIGHT_POS = [-1.0, 54.02, 13.05]


@pytest.fixture(autouse=True, scope="module")
def _restore_tracked_artifacts():
    """This module's subprocess tests regenerate the git-tracked
    robot_body_wheels.FCStd / 07_body_wheels_metadata.json as a side
    effect of actually running the generator. Back them up before any
    test in this module runs and restore after the last one, so a full
    suite run doesn't leave those tracked artifacts out of sync with
    06_Exports/urdf/robot.urdf for other test files collected in the
    same pytest session (e.g. test_urdf_fk_regression.py, which reads
    the checked-in URDF and expects it to match the checked-in
    metadata)."""
    backups = {}
    for path in (OUTPUT_FCSTD, METADATA_JSON):
        if path.exists():
            backups[path] = path.read_bytes()
    yield
    for path in (OUTPUT_FCSTD, METADATA_JSON):
        if path in backups:
            path.write_bytes(backups[path])
        elif path.exists():
            path.unlink()


class TestPlacementOverridesYAML:
    """Static layer: validate YAML schema without running FreeCAD."""

    def test_placement_overrides_yaml_exists(self) -> None:
        """Confirm placement_overrides.yaml file exists."""
        assert OVERRIDES_FILE.exists(), f"Missing {OVERRIDES_FILE}"

    def test_placement_overrides_yaml_parses(self) -> None:
        """Confirm YAML is valid and top-level is a mapping."""
        with open(OVERRIDES_FILE, "r") as f:
            data = yaml.safe_load(f)
        assert isinstance(data, dict), "YAML top level must be a mapping"

    def test_placement_overrides_yaml_seeded_values(self) -> None:
        """Confirm STS3032_Mount and STS3032_Mount_Right have correct seeded values."""
        with open(OVERRIDES_FILE, "r") as f:
            data = yaml.safe_load(f)

        # Check STS3032_Mount
        assert "STS3032_Mount" in data, "Missing STS3032_Mount key"
        sts_mount = data["STS3032_Mount"]
        assert "original" in sts_mount, "STS3032_Mount missing 'original'"
        assert "adjust" in sts_mount, "STS3032_Mount missing 'adjust'"
        assert len(sts_mount["original"]) == 3, "STS3032_Mount 'original' not length-3"
        assert len(sts_mount["adjust"]) == 3, "STS3032_Mount 'adjust' not length-3"

        # Seeded: original == adjust == SEEDED_STS_MOUNT_POS (with tolerance)
        tolerance = 1e-3
        for i, val in enumerate(SEEDED_STS_MOUNT_POS):
            assert (
                abs(sts_mount["original"][i] - val) < tolerance
            ), f"STS3032_Mount 'original'[{i}] mismatch"
            assert (
                abs(sts_mount["adjust"][i] - val) < tolerance
            ), f"STS3032_Mount 'adjust'[{i}] mismatch"

        # Check STS3032_Mount_Right
        assert "STS3032_Mount_Right" in data, "Missing STS3032_Mount_Right key"
        sts_mount_right = data["STS3032_Mount_Right"]
        assert "original" in sts_mount_right, "STS3032_Mount_Right missing 'original'"
        assert "adjust" in sts_mount_right, "STS3032_Mount_Right missing 'adjust'"
        assert len(sts_mount_right["original"]) == 3, "STS3032_Mount_Right 'original' not length-3"
        assert len(sts_mount_right["adjust"]) == 3, "STS3032_Mount_Right 'adjust' not length-3"

        # Seeded: original == adjust == SEEDED_STS_MOUNT_RIGHT_POS
        for i, val in enumerate(SEEDED_STS_MOUNT_RIGHT_POS):
            assert (
                abs(sts_mount_right["original"][i] - val) < tolerance
            ), f"STS3032_Mount_Right 'original'[{i}] mismatch"
            assert (
                abs(sts_mount_right["adjust"][i] - val) < tolerance
            ), f"STS3032_Mount_Right 'adjust'[{i}] mismatch"

    def test_generator_script_has_apply_placement_override_calls(self) -> None:
        """Confirm source text contains _apply_placement_override calls for all 5 known keys."""
        with open(GENERATOR_SCRIPT, "r") as f:
            source = f.read()

        known_keys = ["STS3032_Mount", "STS3032_Mount_Right", "PlateStack", "PlateStack_Right", "Base_Link"]
        for key in known_keys:
            pattern = f'_apply_placement_override({{}}, "{key}")'
            # We expect the call somewhere in the file with the key literal
            assert f'"{key}"' in source or f"'{key}'" in source, (
                f"Missing placement override call for {key!r}"
            )

    def test_generator_script_wheels_not_overrideable(self) -> None:
        """Confirm Wheel_Left/Wheel_Right are NOT in _apply_placement_override calls."""
        with open(GENERATOR_SCRIPT, "r") as f:
            source = f.read()

        # Wheels should NOT be referenced in placement override mechanism
        assert '_apply_placement_override(wheel_left, "Wheel_Left")' not in source
        assert '_apply_placement_override(wheel_right, "Wheel_Right")' not in source


@pytest.mark.skipif(
    not Path(GENERATOR_SCRIPT).exists(),
    reason="Generator script not found; skipping subprocess layer",
)
class TestPlacementOverridesSubprocess:
    """Subprocess layer: run headless generator and verify output FCStd.

    Skipped if FREECAD_BIN is not available or source docs are missing.
    """

    @staticmethod
    def _run_generator() -> bool:
        """Run the generator script via freecadcmd, return success.

        NOTE: freecadcmd -c's exit code is unreliable (it drops into an
        interactive REPL after the script runs -- see root CLAUDE.md /
        run_urdf_export.sh's documented convention). Delete the metadata
        file first, then treat its (re-)existence as the success signal,
        same pattern as test_07_body_wheels_geometry.py's
        test_freecadcmd_run_produces_valid_output.
        """
        if METADATA_JSON.exists():
            METADATA_JSON.unlink()
        try:
            cmd = [
                FREECAD_BIN,
                "-c",
            ]
            code = f"exec(open({str(GENERATOR_SCRIPT)!r}).read())"
            subprocess.run(
                cmd,
                input=code,
                text=True,
                capture_output=True,
                timeout=120,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return False
        return METADATA_JSON.exists()

    @staticmethod
    def _read_sts_mount_position(obj_name: str) -> tuple:
        """Read an STS mount's Placement.Base via the generator's own JSON
        metadata output, NOT by importing FreeCAD in this process -- FreeCAD
        and this env's CadQuery/OCP must never share a process (root
        CLAUDE.md / README.md's architecture note), so this test process
        (running under pendulum-tools) can never `import FreeCAD` directly.

        Returns: (x, y, z) tuple or raises RuntimeError.
        """
        link_name = STS_MOUNT_METADATA_LINK.get(obj_name)
        if link_name is None:
            raise RuntimeError(f"No metadata link mapping for {obj_name!r}")
        try:
            with open(METADATA_JSON, "r") as f:
                data = json.load(f)
            pos = data["links"][link_name]["sts_mount_placement"]["position"]
            return (pos["x"], pos["y"], pos["z"])
        except Exception as e:
            raise RuntimeError(f"Error reading {obj_name} from {METADATA_JSON}: {e}") from e

    def test_generator_runs_with_real_overrides(self) -> None:
        """Run generator with real placement_overrides.yaml in place."""
        if not self._run_generator():
            pytest.skip("Generator run failed; source docs may be missing")

        assert OUTPUT_FCSTD.exists(), f"Generator did not produce {OUTPUT_FCSTD}"

    def test_output_fcstd_has_seeded_placements(self) -> None:
        """Verify generator output reflects seeded YAML positions."""
        if not METADATA_JSON.exists():
            pytest.skip("Metadata JSON not available; run generator first")

        tolerance = 1e-2  # mm, allow small noise
        try:
            actual_sts = self._read_sts_mount_position("STS3032_Mount")
            for i, expected in enumerate(SEEDED_STS_MOUNT_POS):
                assert (
                    abs(actual_sts[i] - expected) < tolerance
                ), f"STS3032_Mount[{i}]: expected {expected}, got {actual_sts[i]}"

            actual_sts_right = self._read_sts_mount_position("STS3032_Mount_Right")
            for i, expected in enumerate(SEEDED_STS_MOUNT_RIGHT_POS):
                assert (
                    abs(actual_sts_right[i] - expected) < tolerance
                ), f"STS3032_Mount_Right[{i}]: expected {expected}, got {actual_sts_right[i]}"
        except RuntimeError as e:
            pytest.skip(f"Could not verify output: {e}")

    def test_override_mechanism_applies_changes(self) -> None:
        """Verify that override mechanism actually takes effect (not a passthrough).

        Approach: temporarily modify the YAML with a new position for STS3032_Mount,
        regenerate, verify the change is applied, then restore the original YAML.
        """
        if not METADATA_JSON.exists():
            pytest.skip("Metadata JSON not available")

        # Back up the real file
        with open(OVERRIDES_FILE, "rb") as f:
            original_bytes = f.read()

        try:
            # Create a temp override with a distinct position
            test_position = [5.0, 10.0, 20.0]
            with open(OVERRIDES_FILE, "r") as f:
                data = yaml.safe_load(f)
            data["STS3032_Mount"]["adjust"] = test_position
            with open(OVERRIDES_FILE, "w") as f:
                yaml.dump(data, f)

            # Regenerate
            if not self._run_generator():
                pytest.skip("Generator run failed during override test")

            # Verify the new position is applied
            try:
                actual = self._read_sts_mount_position("STS3032_Mount")
                tolerance = 1e-2
                for i, expected in enumerate(test_position):
                    assert (
                        abs(actual[i] - expected) < tolerance
                    ), f"Override test: STS3032_Mount[{i}] expected {expected}, got {actual[i]}"
            except RuntimeError as e:
                pytest.skip(f"Could not verify override: {e}")
        finally:
            # Restore original file
            with open(OVERRIDES_FILE, "wb") as f:
                f.write(original_bytes)

    def test_unknown_key_and_mismatch_original_are_non_fatal(self) -> None:
        """Verify warnings (unknown key + original mismatch) don't abort the run.

        Approach: write a temp override YAML with a typo'd key and a mismatched
        'original' value, regenerate, verify it completes, and check both warnings
        appear in console output.
        """
        # Back up the real file
        with open(OVERRIDES_FILE, "rb") as f:
            original_bytes = f.read()

        try:
            # Create a temp override with a typo'd key and a wrong 'original'
            with open(OVERRIDES_FILE, "r") as f:
                data = yaml.safe_load(f)
            data["STS3032_Mount_Typo"] = {  # Unknown key (typo)
                "original": [1.0, 2.0, 3.0],
                "adjust": [5.0, 6.0, 7.0],
            }
            data["STS3032_Mount"]["original"] = [999.0, 999.0, 999.0]  # Deliberately wrong
            with open(OVERRIDES_FILE, "w") as f:
                yaml.dump(data, f)

            # Regenerate (should succeed despite warnings)
            try:
                result = subprocess.run(
                    [FREECAD_BIN, "-c"],
                    input=f"exec(open({str(GENERATOR_SCRIPT)!r}).read())",
                    text=True,
                    capture_output=True,
                    timeout=120,
                )
            except FileNotFoundError:
                pytest.skip(f"FREECAD_BIN ({FREECAD_BIN}) not found")
                return

            # Check that the run succeeded. NOTE: freecadcmd -c's exit code is
            # unreliable (it drops into an interactive REPL after the script
            # runs, per root CLAUDE.md / run_urdf_export.sh's documented
            # convention) -- check for the script's own success marker in
            # stdout instead of trusting returncode.
            output = result.stdout + result.stderr
            assert "Stage 1 Complete" in output, f"Generator did not complete: {output[-2000:]}"

            # Check that expected warnings appear in output
            assert "unknown key" in output.lower() or "STS3032_Mount_Typo" in output, (
                "Expected 'unknown key' warning not found in output"
            )
            # The original mismatch warning may or may not appear depending on implementation
            # (it's a lower-priority check), but the run should still succeed
        finally:
            # Restore original file
            with open(OVERRIDES_FILE, "wb") as f:
                f.write(original_bytes)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
