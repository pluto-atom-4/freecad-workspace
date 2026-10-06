"""
Comprehensive unit tests for gen_lqr_header: C++ gains header generation.

Tests cover canonical text, hashing, float32 formatting, header rendering,
CLI behavior, and import hygiene (no scipy/numpy/controller leaks).

Usage:
    mamba run -n pendulum-tools python3 -m pytest -q inverted-pendulum-project/07_Simulation/hal/test_gen_lqr_header.py
"""

import ast
import hashlib
import re
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from hal.hal import HAL_CONTRACT_VERSION
from hal.gen_lqr_header import (
    SCHEMA,
    f32,
    fmt_f32,
    canonical_text,
    gains_hash,
    format_hash,
    render_header,
    default_gains,
    main,
)
from hal.f32_emulation import quantize_f32
from hal.control_core import (
    DEFAULT_PID_KP,
    DEFAULT_PID_KI,
    DEFAULT_PID_KD,
    DEFAULT_OUT_LIMIT,
    DEFAULT_THETA_REF_RAD,
)


F = dict(
    K=(-23.5, -3.25),
    theta_ref=-0.125,
    out_limit=1.0,
    pid=(1.0, 0.5, 0.25, 1.0),
    contract_version=1,
)
FIXTURE_TEXT = "schema=1\ncontract_version=1\nK=-23.5,-3.25\ntheta_ref=-0.125\nout_limit=1\npid=1,0.5,0.25,1\n"


class TestCanonicalText:
    """Test canonical_text function."""

    def test_canonical_text_fixture(self):
        """Test canonical_text output matches expected format."""
        result = canonical_text(**F)
        assert result == FIXTURE_TEXT


class TestGainsHash:
    """Test gains_hash and format_hash functions."""

    def test_hash_matches_independent_sha(self):
        """Test hash matches independent SHA256 computation."""
        h = gains_hash(**F)
        expected_h = int(hashlib.sha256(FIXTURE_TEXT.encode("ascii")).hexdigest()[:8], 16)
        assert h == expected_h
        assert 0 <= h < 2**32
        assert re.match(r"^0x[0-9a-f]{8}$", format_hash(h))
        assert format_hash(gains_hash(**F)) == "0x757debdf"

    def test_hash_stable_and_quantize_invariant(self):
        """Test hash is stable and quantization-invariant."""
        h1 = gains_hash(**F)
        h2 = gains_hash(**F)
        assert h1 == h2

        K_f64 = (-23.81842951, -3.49402271)
        K_f32 = (f32(K_f64[0]), f32(K_f64[1]))
        h_f64 = gains_hash(K=K_f64, theta_ref=F["theta_ref"], out_limit=F["out_limit"], pid=F["pid"], contract_version=F["contract_version"])
        h_f32 = gains_hash(K=K_f32, theta_ref=F["theta_ref"], out_limit=F["out_limit"], pid=F["pid"], contract_version=F["contract_version"])
        assert h_f64 == h_f32

    def test_hash_changes_on_each_input(self):
        """Test hash changes when any input parameter changes."""
        base = gains_hash(**F)

        K0_variant = gains_hash(
            K=(F["K"][0] * 1.5, F["K"][1]),
            theta_ref=F["theta_ref"],
            out_limit=F["out_limit"],
            pid=F["pid"],
            contract_version=F["contract_version"],
        )
        K1_variant = gains_hash(
            K=(F["K"][0], F["K"][1] * 1.5),
            theta_ref=F["theta_ref"],
            out_limit=F["out_limit"],
            pid=F["pid"],
            contract_version=F["contract_version"],
        )
        theta_ref_variant = gains_hash(
            K=F["K"],
            theta_ref=F["theta_ref"] * 2,
            out_limit=F["out_limit"],
            pid=F["pid"],
            contract_version=F["contract_version"],
        )
        out_limit_variant = gains_hash(
            K=F["K"],
            theta_ref=F["theta_ref"],
            out_limit=F["out_limit"] * 2,
            pid=F["pid"],
            contract_version=F["contract_version"],
        )
        kp_variant = gains_hash(
            K=F["K"],
            theta_ref=F["theta_ref"],
            out_limit=F["out_limit"],
            pid=(F["pid"][0] * 2, F["pid"][1], F["pid"][2], F["pid"][3]),
            contract_version=F["contract_version"],
        )
        ki_variant = gains_hash(
            K=F["K"],
            theta_ref=F["theta_ref"],
            out_limit=F["out_limit"],
            pid=(F["pid"][0], F["pid"][1] * 2, F["pid"][2], F["pid"][3]),
            contract_version=F["contract_version"],
        )
        kd_variant = gains_hash(
            K=F["K"],
            theta_ref=F["theta_ref"],
            out_limit=F["out_limit"],
            pid=(F["pid"][0], F["pid"][1], F["pid"][2] * 2, F["pid"][3]),
            contract_version=F["contract_version"],
        )
        pid_out_limit_variant = gains_hash(
            K=F["K"],
            theta_ref=F["theta_ref"],
            out_limit=F["out_limit"],
            pid=(F["pid"][0], F["pid"][1], F["pid"][2], F["pid"][3] * 2),
            contract_version=F["contract_version"],
        )
        contract_version_variant = gains_hash(
            K=F["K"],
            theta_ref=F["theta_ref"],
            out_limit=F["out_limit"],
            pid=F["pid"],
            contract_version=F["contract_version"] + 1,
        )

        all_hashes = {
            base,
            K0_variant,
            K1_variant,
            theta_ref_variant,
            out_limit_variant,
            kp_variant,
            ki_variant,
            kd_variant,
            pid_out_limit_variant,
            contract_version_variant,
        }
        assert len(all_hashes) == 10


class TestFmtF32:
    """Test fmt_f32 formatting function."""

    def test_fmt_f32_basic(self):
        """Test basic fmt_f32 cases."""
        assert fmt_f32(1.0) == "1.0f"
        assert fmt_f32(0.0) == "0.0f"
        assert fmt_f32(-0.125) == "-0.125f"

    def test_fmt_f32_precision_roundtrip(self):
        """Test fmt_f32 precision and roundtrip."""
        for x in (0.1, -0.1086, 1e-5, 6e-6, 23.81842951):
            formatted = fmt_f32(x)
            assert formatted.endswith("f")
            value_str = formatted[:-1]
            roundtrip_value = float(np.float32(value_str))
            expected_value = f32(x)
            assert roundtrip_value == expected_value

    def test_fmt_f32_rejects_nonfinite(self):
        """Test fmt_f32 rejects non-finite values."""
        with pytest.raises(ValueError):
            fmt_f32(float("nan"))
        with pytest.raises(ValueError):
            fmt_f32(float("inf"))
        with pytest.raises(ValueError):
            fmt_f32(1e39)


class TestRenderHeader:
    """Test render_header C++ header generation."""

    def test_render_header_fixture_exact(self):
        """Test render_header output matches exact fixture."""
        h = format_hash(gains_hash(**F))
        expected = "\n".join([
            "// GENERATED by freecad-workspace export_cpp.py -- do not edit.",
            "// SIM-derived design values (LQR from the linearized plant model). Not tuned or validated for the real robot.",
            "#pragma once",
            "#include <cstdint>",
            "namespace balance_gains {",
            "constexpr uint32_t kSchema = 1;",
            "constexpr uint32_t kContractVersion = 1;",
            f"constexpr uint32_t kGainsHash = {h}u;",
            "constexpr float kLqrK[2] = { -23.5f, -3.25f };    // [theta, theta_dot]",
            "constexpr float kThetaRefRad = -0.125f;",
            "constexpr float kOutLimit = 1.0f;",
            "constexpr float kPidKp = 1.0f, kPidKi = 0.5f, kPidKd = 0.25f, kPidOutLimit = 1.0f;",
            "}  // namespace balance_gains",
        ]) + "\n"
        result = render_header(**F)
        assert result == expected

    def test_render_header_clean(self):
        """Test render_header output hygiene."""
        result = render_header(**F)
        lines = result.split("\n")
        assert lines[0] == "// GENERATED by freecad-workspace export_cpp.py -- do not edit."
        assert "scipy" not in result
        assert "numpy" not in result
        assert result.count("\n") >= 13
        assert result.endswith("\n")
        assert not result.endswith("\n\n")
        assert '"' not in result
        assert "kSchema = 1;" in result
        assert "kContractVersion = 1;" in result


class TestDefaultGains:
    """Test default_gains function."""

    def test_default_gains_values(self):
        """Test default_gains returns correct float32 values."""
        g = default_gains()

        K = g["K"]
        assert len(K) == 2
        assert abs(K[0] - (-23.81842951)) / abs(-23.81842951) < 1e-4
        assert abs(K[1] - (-3.49402271)) / abs(-3.49402271) < 1e-4

        assert g["theta_ref"] == f32(DEFAULT_THETA_REF_RAD)
        assert g["out_limit"] == f32(DEFAULT_OUT_LIMIT)

        pid = g["pid"]
        assert len(pid) == 4
        assert pid[0] == f32(DEFAULT_PID_KP)
        assert pid[1] == f32(DEFAULT_PID_KI)
        assert pid[2] == f32(DEFAULT_PID_KD)
        assert pid[3] == f32(DEFAULT_OUT_LIMIT)

        assert g["contract_version"] == HAL_CONTRACT_VERSION

        for v in [K[0], K[1], g["theta_ref"], g["out_limit"], pid[0], pid[1], pid[2], pid[3]]:
            assert float(np.float32(v)) == v


class TestHeaderKRoundtrip:
    """Test K value roundtrip through C++ header."""

    def test_header_k_roundtrip_exact(self):
        """Test K values can be extracted from header exactly."""
        g = default_gains()
        header = render_header(**g)

        match = re.search(r"kLqrK\[2\] = \{ ([^,]+), ([^ ]+) \};", header)
        assert match is not None

        k0_str = match.group(1).strip()
        k1_str = match.group(2).strip()

        assert k0_str.endswith("f")
        assert k1_str.endswith("f")

        k0_parsed = float(np.float32(k0_str[:-1]))
        k1_parsed = float(np.float32(k1_str[:-1]))

        assert k0_parsed == g["K"][0]
        assert k1_parsed == g["K"][1]


class TestCLI:
    """Test command-line interface."""

    def test_cli_matches_render(self, tmp_path):
        """Test CLI output matches render_header."""
        out = tmp_path / "h.h"
        result = main(["--out", str(out)])
        assert result == 0
        assert out.read_text() == render_header(**default_gains())

    def test_cli_requires_out(self):
        """Test CLI requires --out argument."""
        with pytest.raises(SystemExit):
            main([])


class TestImportHygiene:
    """Test gen_lqr_header module import structure."""

    def test_module_import_hygiene(self):
        """Test no top-level numpy/scipy/controller imports in gen_lqr_header."""
        module_path = Path(__file__).resolve().parent / "gen_lqr_header.py"
        tree = ast.parse(module_path.read_text())

        top_level_imports = set()
        for node in ast.walk(tree):
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                if node.col_offset == 0:
                    if isinstance(node, ast.Import):
                        for alias in node.names:
                            top_level_imports.add(alias.name)
                    elif isinstance(node, ast.ImportFrom):
                        if node.module:
                            top_level_imports.add(node.module)

        assert "numpy" not in top_level_imports
        assert "scipy" not in top_level_imports
        assert "controller" not in top_level_imports

        full_text = module_path.read_text()
        assert "import controller" not in full_text
        assert "from controller" not in full_text
        assert "import numpy" not in full_text or "hal.control_core" in full_text
        assert "import scipy" not in full_text


class TestF32Equivalence:
    """Test f32 equivalence with quantize_f32."""

    def test_f32_equals_quantize_f32(self):
        """Test f32 and quantize_f32 produce identical results."""
        for x in (0.1, -0.1086, 6e-6, 23.81842951, 1.0):
            assert f32(x) == quantize_f32(x)


class TestRealHeaderPins:
    """Test pinned values from real SIM-derived defaults."""

    def test_real_header_pins(self):
        """Test render_header with default_gains produces expected pinned values.

        These digits pin the current plant design. If the plant/LQR design changes
        deliberately, regenerate and update them (and the C++ copy via export_cpp.py).
        """
        header = render_header(**default_gains())

        assert "kGainsHash = 0x1e60b0fcu;" in header
        assert "kLqrK[2] = { -23.8184299f, -3.49402261f };" in header
        assert "kThetaRefRad = -0.108599998f;" in header
        assert "kPidKi = 0.100000001f" in header
        assert "kPidKd = 0.0500000007f" in header
