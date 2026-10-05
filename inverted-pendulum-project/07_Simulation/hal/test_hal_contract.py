#!/usr/bin/env python3
"""
HAL contract tests: abstract base, frozen dataclasses, imports, version stability.

Verify Hal abstract interface, ImuSample/EncoderSample frozen dataclasses,
HalFault exception, HAL_CONTRACT_VERSION constant, no Webots controller imports,
and docstring contract pins.

Usage:
    cd inverted-pendulum-project
    mamba run -n pendulum-tools python3 -m pytest -q 07_Simulation/hal/test_hal_contract.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from hal.hal import (
    Hal,
    HalFault,
    ImuSample,
    EncoderSample,
    HAL_CONTRACT_VERSION,
)

import ast
import dataclasses
import re
import pytest
import hal.hal as hal_module

HAL_FILE = Path(__file__).resolve().parent / "hal.py"
INIT_FILE = Path(__file__).resolve().parent / "__init__.py"


class TestHalContract:
    """Test suite for HAL contract enforcement."""

    def test_hal_is_abstract(self):
        """Hal() instantiation must raise TypeError."""
        with pytest.raises(TypeError):
            Hal()

    def test_incomplete_subclass_rejected(self):
        """Subclass implementing only read_imu (not all abstract methods) raises TypeError."""
        class IncompleteHal(Hal):
            def read_imu(self):
                return ImuSample(0.0, 0.0, 0.0, 0.0, (0.0, 0.0, 0.0))

        with pytest.raises(TypeError):
            IncompleteHal()

    def test_minimal_subclass_works(self):
        """Minimal stub implementing all abstract methods works."""
        class StubHal(Hal):
            def __init__(self):
                self.velocities = []

            def read_imu(self):
                return ImuSample(0.0, 0.0, 0.1, 0.0, (0.0, 0.2, 0.0))

            def read_encoders(self):
                return EncoderSample(1.0, 2.0)

            def write_wheel_velocity(self, left_rad_s, right_rad_s):
                self.velocities.append((left_rad_s, right_rad_s))

            def now_s(self):
                return 0.0

            def wait_next_tick(self):
                return 0.02

        hal = StubHal()
        assert isinstance(hal, Hal)

        imu = hal.read_imu()
        assert imu.pitch_rad == 0.1
        assert imu.gyro_rad_s[1] == 0.2

        enc = hal.read_encoders()
        assert enc.left_rad == 1.0
        assert enc.right_rad == 2.0

        hal.write_wheel_velocity(0.5, 0.6)
        assert hal.velocities == [(0.5, 0.6)]

        assert hal.now_s() == 0.0
        assert hal.wait_next_tick() == 0.02

        result = hal.close()
        assert result is None

    def test_abstract_method_set(self):
        """Hal.__abstractmethods__ == frozenset of required methods."""
        expected = frozenset({
            "read_imu",
            "read_encoders",
            "write_wheel_velocity",
            "now_s",
            "wait_next_tick",
        })
        assert Hal.__abstractmethods__ == expected

    def test_dataclasses_frozen(self):
        """ImuSample and EncoderSample are frozen; assignment raises FrozenInstanceError."""
        imu = ImuSample(0.5, 0.01, -0.1086, 0.0, (0.0, 0.3, 0.0))
        with pytest.raises(dataclasses.FrozenInstanceError):
            imu.pitch_rad = 0.2

        enc = EncoderSample(1.5, 2.5)
        with pytest.raises(dataclasses.FrozenInstanceError):
            enc.left_rad = 3.0

    def test_field_order(self):
        """Dataclass field order matches contract."""
        imu_fields = [f.name for f in dataclasses.fields(ImuSample)]
        assert imu_fields == ["t_s", "roll_rad", "pitch_rad", "yaw_rad", "gyro_rad_s"]

        enc_fields = [f.name for f in dataclasses.fields(EncoderSample)]
        assert enc_fields == ["left_rad", "right_rad"]

    def test_imu_sample_positional_and_gyro_index(self):
        """ImuSample positional construction, gyro indexing, equality, hashing."""
        s = ImuSample(0.5, 0.01, -0.1086, 0.0, (0.0, 0.3, 0.0))

        assert s.t_s == 0.5
        assert s.roll_rad == 0.01
        assert s.pitch_rad == -0.1086
        assert s.yaw_rad == 0.0
        assert s.gyro_rad_s == (0.0, 0.3, 0.0)
        assert s.gyro_rad_s[1] == 0.3

        s2 = ImuSample(0.5, 0.01, -0.1086, 0.0, (0.0, 0.3, 0.0))
        assert s == s2
        assert hash(s) == hash(s2)

    def test_hal_fault_is_exception(self):
        """HalFault is an Exception subclass."""
        assert issubclass(HalFault, Exception)

        with pytest.raises(HalFault, match="boom"):
            raise HalFault("boom")

    def test_contract_version(self):
        """HAL_CONTRACT_VERSION is int, not bool, >= 1, == 1."""
        assert isinstance(HAL_CONTRACT_VERSION, int)
        assert not isinstance(HAL_CONTRACT_VERSION, bool)
        assert HAL_CONTRACT_VERSION >= 1
        assert HAL_CONTRACT_VERSION == 1

    def test_contract_version_regex_readable(self):
        """HAL_CONTRACT_VERSION literal in hal.py matches regex pattern."""
        text = HAL_FILE.read_text()
        m = re.search(r"^HAL_CONTRACT_VERSION\s*=\s*(\d+)\s*(#.*)?$", text, re.M)
        assert m is not None, "HAL_CONTRACT_VERSION not found with expected pattern"
        assert int(m.group(1)) == HAL_CONTRACT_VERSION

    def test_no_webots_import_ast(self):
        """hal.py imports only safe modules; __init__.py uses relative imports only."""
        hal_text = HAL_FILE.read_text()
        hal_ast = ast.parse(hal_text)

        # For hal.py: gather top-level imports
        hal_imports = set()
        for node in ast.walk(hal_ast):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    top_mod = alias.name.split(".")[0]
                    hal_imports.add(top_mod)
            elif isinstance(node, ast.ImportFrom):
                if node.level == 0 and node.module:
                    top_mod = node.module.split(".")[0]
                    hal_imports.add(top_mod)

        # hal.py should only import abc, dataclasses, typing
        allowed_hal = {"abc", "dataclasses", "typing"}
        assert hal_imports.issubset(allowed_hal), \
            f"hal.py imports disallowed modules: {hal_imports - allowed_hal}"

        # Check no 'controller' (Webots) in any import
        for node in ast.walk(hal_ast):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    assert not alias.name.startswith("controller"), \
                        f"hal.py imports controller: {alias.name}"
            elif isinstance(node, ast.ImportFrom):
                if node.module:
                    assert not node.module.startswith("controller"), \
                        f"hal.py imports from controller: {node.module}"

        # For __init__.py: only relative imports allowed (level > 0)
        if INIT_FILE.exists():
            init_text = INIT_FILE.read_text()
            init_ast = ast.parse(init_text)
            for node in ast.walk(init_ast):
                if isinstance(node, ast.ImportFrom):
                    assert node.level > 0, \
                        f"__init__.py has absolute import: {node.module}"

    def test_docstring_pins_contract(self):
        """Module docstring contains key contract pins."""
        doc = hal_module.__doc__
        assert doc is not None, "hal module has no docstring"

        required_substrings = [
            "rad/s",
            "radians",
            "InertialUnit",
            "kPitchSign",
            "calibration",
            "HAL_CONTRACT_VERSION",
            "dt == 0",
            "dt < 0",
            "returning < 0",
        ]

        for substring in required_substrings:
            assert substring in doc, \
                f"Module docstring missing required substring: {substring!r}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
