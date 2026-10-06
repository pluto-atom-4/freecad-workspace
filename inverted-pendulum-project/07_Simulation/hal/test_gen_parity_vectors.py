"""
Comprehensive unit tests for gen_parity_vectors: golden parity vector generation and validation.

Tests cover vector generation, counts, float32 exactness, hand-verified values, C++ include
rendering, determinism, CLI behavior, and import hygiene.

Usage:
    mamba run -n pendulum-tools python3 -m pytest -q inverted-pendulum-project/07_Simulation/hal/test_gen_parity_vectors.py
"""

import ast
import copy
import json
import re
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from hal.hal import HAL_CONTRACT_VERSION
from hal.f32_emulation import F32_TOL_ABS, quantize_f32
from hal.gen_lqr_header import gains_hash, format_hash, render_header, f32
from hal.gen_parity_vectors import (
    DEFAULT_JSON,
    build_vectors,
    render_json,
    load_vectors,
    validate_vectors,
    f32_agreement,
    gains_args,
    render_unity_inc,
    count_inc_rows,
    check,
    main,
)


V = build_vectors()


class TestCountsAndStamps:
    """Test vector counts and schema metadata."""

    def test_counts_and_stamps(self):
        """Test LQR/PID counts, schema, contract version, and validation."""
        assert len(V["lqr"]["cases"]) == 24
        assert len(V["pid"]["sequence"]) == 74
        assert V["schema"] == 1
        assert V["contract_version"] == HAL_CONTRACT_VERSION
        assert V["tol_abs"] == F32_TOL_ABS
        assert V["gains_hash"] == "0x1e60b0fc"
        problems = validate_vectors(V)
        assert problems == []


class TestInputsFloat32Exact:
    """Test all inputs and gains are float32-exact."""

    def test_inputs_float32_exact(self):
        """Test every K, theta_ref, out_limit, pid gains, and case/step inputs satisfy f32(x)==x."""
        # LQR gains
        for i, k in enumerate(V["lqr"]["K"]):
            assert quantize_f32(k) == k, f"K[{i}] = {k} not f32-exact"

        assert quantize_f32(V["lqr"]["theta_ref_rad"]) == V["lqr"]["theta_ref_rad"]
        assert quantize_f32(V["lqr"]["out_limit"]) == V["lqr"]["out_limit"]

        # PID gains
        assert quantize_f32(V["pid"]["kp"]) == V["pid"]["kp"]
        assert quantize_f32(V["pid"]["ki"]) == V["pid"]["ki"]
        assert quantize_f32(V["pid"]["kd"]) == V["pid"]["kd"]
        assert quantize_f32(V["pid"]["out_limit"]) == V["pid"]["out_limit"]

        # LQR case inputs
        for i, case in enumerate(V["lqr"]["cases"]):
            assert quantize_f32(case["pitch"]) == case["pitch"], f"lqr.cases[{i}].pitch not f32-exact"
            assert quantize_f32(case["gyro_y"]) == case["gyro_y"], f"lqr.cases[{i}].gyro_y not f32-exact"
            assert quantize_f32(case["dt"]) == case["dt"], f"lqr.cases[{i}].dt not f32-exact"

        # PID step inputs
        for i, step in enumerate(V["pid"]["sequence"]):
            assert quantize_f32(step["pitch"]) == step["pitch"], f"pid.sequence[{i}].pitch not f32-exact"
            assert quantize_f32(step["dt"]) == step["dt"], f"pid.sequence[{i}].dt not f32-exact"


class TestLqrHandValues:
    """Test hand-verified LQR case values."""

    def test_lqr_hand_values(self):
        """Test specific LQR cases for correctness and saturation behavior."""
        # Case 0: zero case
        assert V["lqr"]["cases"][0]["raw"] == 0.0
        assert V["lqr"]["cases"][0]["cmd"] == 0.0

        # Case 8: mid-range
        case_8 = V["lqr"]["cases"][8]
        assert case_8["raw"] == pytest.approx(0.58759, abs=2e-4)
        assert case_8["cmd"] == case_8["raw"]

        # Case 12: near low boundary
        case_12 = V["lqr"]["cases"][12]
        assert case_12["raw"] == pytest.approx(0.99085, abs=2e-3)
        assert case_12["cmd"] == case_12["raw"]

        # Case 13: saturates at +1.0
        case_13 = V["lqr"]["cases"][13]
        assert case_13["raw"] == pytest.approx(1.01467, abs=2e-3)
        assert case_13["cmd"] == 1.0

        # Case 16: saturates at +1.0
        case_16 = V["lqr"]["cases"][16]
        assert case_16["raw"] == pytest.approx(6.5107, abs=2e-3)
        assert case_16["cmd"] == 1.0

        # Case 17: saturates at -1.0
        case_17 = V["lqr"]["cases"][17]
        assert case_17["raw"] == pytest.approx(-7.62288, abs=2e-3)
        assert case_17["cmd"] == -1.0

        # Cases 18, 19, 20: same raw as case 8 (dt ignored by LQR)
        raw_8 = V["lqr"]["cases"][8]["raw"]
        assert V["lqr"]["cases"][18]["raw"] == raw_8
        assert V["lqr"]["cases"][19]["raw"] == raw_8
        assert V["lqr"]["cases"][20]["raw"] == raw_8

        # Cases 22, 23: fault cases with zero output
        assert V["lqr"]["cases"][22]["fault"] is True
        assert V["lqr"]["cases"][22]["cmd"] == 0.0
        assert V["lqr"]["cases"][22]["raw"] == 0.0
        assert V["lqr"]["cases"][23]["fault"] is True
        assert V["lqr"]["cases"][23]["cmd"] == 0.0
        assert V["lqr"]["cases"][23]["raw"] == 0.0

        # Cases 0..21: all non-fault
        for i in range(22):
            assert V["lqr"]["cases"][i]["fault"] is False


class TestPidHandValues:
    """Test hand-verified PID step values."""

    def test_pid_hand_values(self):
        """Test specific PID steps for correctness and reset behavior."""
        # Step 0: reset, zero output
        assert V["pid"]["sequence"][0]["reset"] is True
        assert V["pid"]["sequence"][0]["cmd"] == 0.0
        assert V["pid"]["sequence"][0]["raw"] == 0.0

        # Step 1: first output
        step_1 = V["pid"]["sequence"][1]
        assert step_1["raw"] == pytest.approx(0.041266, abs=1e-5)

        # Steps 10 and 11: dt==0 cases
        step_10 = V["pid"]["sequence"][10]
        step_11 = V["pid"]["sequence"][11]
        assert step_10["dt"] == 0.0
        assert step_10["raw"] == step_10["cmd"]
        assert step_10["raw"] == pytest.approx(0.05, abs=1e-8)
        assert step_11["dt"] == 0.0
        assert step_11["raw"] == step_11["cmd"]
        assert step_11["raw"] == pytest.approx(0.08, abs=1e-8)

        # Step 14: dt<0 fault
        step_14 = V["pid"]["sequence"][14]
        assert step_14["dt"] < 0.0
        assert step_14["fault"] is True
        assert step_14["cmd"] == 0.0
        assert step_14["raw"] == 0.0

        # Step 15: not a fault (dt>0 after fault)
        step_15 = V["pid"]["sequence"][15]
        assert step_15["fault"] is False

        # Exactly 2 resets: at indices 0 and 45
        reset_indices = [i for i, s in enumerate(V["pid"]["sequence"]) if s["reset"]]
        assert reset_indices == [0, 45]


class TestPidSaturationAndClamp:
    """Test PID saturation and clamping behavior."""

    def test_pid_saturation_and_clamp_present(self):
        """Test saturation both signs and float32 budget."""
        out_limit = V["pid"]["out_limit"]

        # Find positive and negative saturation
        has_pos_sat = any(
            s["raw"] > out_limit and s["cmd"] == out_limit
            for s in V["pid"]["sequence"]
        )
        has_neg_sat = any(
            s["raw"] < -out_limit and s["cmd"] == -out_limit
            for s in V["pid"]["sequence"]
        )
        assert has_pos_sat, "PID: no positive saturation found"
        assert has_neg_sat, "PID: no negative saturation found"

        # Check float32 budget (max abs raw <= 8.0)
        lqr_max_abs_raw = max(
            (abs(case["raw"]) for case in V["lqr"]["cases"]),
            default=0.0,
        )
        pid_max_abs_raw = max(
            (abs(step["raw"]) for step in V["pid"]["sequence"]),
            default=0.0,
        )
        overall_max = max(lqr_max_abs_raw, pid_max_abs_raw)
        assert overall_max <= 8.0, f"max abs raw {overall_max} > 8.0"


class TestF32Agreement:
    """Test float32 emulation agreement with reference."""

    def test_f32_agreement_within_tol(self):
        """Test float32 emulation stays within tolerance."""
        a = f32_agreement(V)

        assert a["lqr_max_abs_cmd"] <= F32_TOL_ABS
        assert a["lqr_max_abs_raw"] <= F32_TOL_ABS
        assert a["pid_max_abs_cmd"] <= F32_TOL_ABS
        assert a["pid_max_abs_raw"] <= F32_TOL_ABS

        # Non-vacuity: at least one agreement is non-zero
        max_agreement = max(a.values())
        assert max_agreement > 0.0, "float32 emulation identical to float64 (implausible)"


class TestIncCounts:
    """Test C++ include generation and formatting."""

    def test_inc_counts_and_format(self):
        """Test include header structure, counts, and format."""
        inc = render_unity_inc(V)

        # Row counts
        counts = count_inc_rows(inc)
        assert counts == (24, 74)

        # Format checks
        assert '"' not in inc, "include contains double-quote characters"
        assert "kVecGainsHash = 0x1e60b0fcu;" in inc
        assert "kVecSchema = 1" in inc
        assert inc.startswith("// GENERATED")
        assert "-0.0f" not in inc, "negative zero not normalized"
        assert "bool fault" in inc

        # Struct definitions
        assert "struct LqrCase { float pitch, gyro_y, dt, cmd, raw; bool fault; };" in inc
        assert "struct PidStep { float pitch, dt, cmd; bool reset; float raw; bool fault; };" in inc

        # Honesty text
        assert "NOT MCU or hardware evidence" in inc


class TestIncDeterministic:
    """Test C++ include generation determinism."""

    def test_inc_deterministic(self):
        """Test include rendering is deterministic."""
        inc1 = render_unity_inc(V)
        inc2 = render_unity_inc(V)
        assert inc1 == inc2

        v_copy = copy.deepcopy(V)
        inc3 = render_unity_inc(v_copy)
        assert inc1 == inc3


class TestCommittedJsonMatches:
    """Test regeneration matches committed JSON."""

    def test_committed_json_matches_regeneration(self):
        """Test regeneration produces exactly the committed JSON."""
        fresh = render_json(build_vectors())
        committed = DEFAULT_JSON.read_text(encoding="utf-8")
        assert fresh == committed

        # Also verify load_vectors matches
        loaded = load_vectors()
        assert loaded == V


class TestValidateTamper:
    """Test tamper detection."""

    def test_validate_detects_tamper(self):
        """Test validate_vectors detects each type of tampering."""
        # Tamper 1: change gains_hash
        v1 = copy.deepcopy(V)
        v1["gains_hash"] = "0x00000000"
        assert validate_vectors(v1) != []

        # Tamper 2: set LQR case pitch to non-f32 value
        v2 = copy.deepcopy(V)
        v2["lqr"]["cases"][0]["pitch"] = 0.1
        assert validate_vectors(v2) != []

        # Tamper 3: change K length
        v3 = copy.deepcopy(V)
        v3["lqr"]["K"].append(1.0)
        assert validate_vectors(v3) != []

        # Tamper 4: change contract_version
        v4 = copy.deepcopy(V)
        v4["contract_version"] = 2
        assert validate_vectors(v4) != []

        # Tamper 5: change schema
        v5 = copy.deepcopy(V)
        v5["schema"] = 2
        assert validate_vectors(v5) != []

        # Tamper 6: set non-fault LQR cmd to invalid value
        v6 = copy.deepcopy(V)
        v6["lqr"]["cases"][0]["cmd"] = 5.0
        assert validate_vectors(v6) != []

        # Tamper 7: set first PID step reset to False
        v7 = copy.deepcopy(V)
        v7["pid"]["sequence"][0]["reset"] = False
        assert validate_vectors(v7) != []

        # Tamper 8: remove most PID steps
        v8 = copy.deepcopy(V)
        v8["pid"]["sequence"] = v8["pid"]["sequence"][:10]
        assert validate_vectors(v8) != []


class TestHashEqualsHeader:
    """Test gains hash consistency."""

    def test_hash_equals_header_hash(self):
        """Test vector hash matches header generator hash."""
        v_hash = V["gains_hash"]
        computed_hash = format_hash(gains_hash(**gains_args(V)))
        assert v_hash == computed_hash

        # Also verify header contains the hash
        header = render_header(**gains_args(V))
        expected_line = "kGainsHash = " + v_hash + "u;"
        assert expected_line in header


class TestCheckCli:
    """Test --check CLI behavior."""

    def test_check_cli_exit(self, tmp_path):
        """Test check CLI exit codes."""
        # Check committed file passes
        result = check()
        assert result == 0

        # Check tampered file fails
        tmp_file = tmp_path / "v.json"
        committed_text = DEFAULT_JSON.read_text(encoding="utf-8")
        tampered = committed_text.replace('"schema": 1', '"schema": 2')
        tmp_file.write_text(tampered, encoding="utf-8")
        result = check(str(tmp_file))
        assert result != 0

        # Check missing file returns 2
        result = check(str(tmp_path / "missing.json"))
        assert result == 2


class TestWriteThenCheck:
    """Test write and check workflow."""

    def test_write_then_check_tmp(self, tmp_path):
        """Test writing and checking JSON."""
        p = tmp_path / "v.json"

        # Write
        result = main(["--json", str(p)])
        assert result == 0
        assert p.read_text(encoding="utf-8") == DEFAULT_JSON.read_text(encoding="utf-8")

        # Check
        result = check(str(p))
        assert result == 0


class TestEmitIncCli:
    """Test --emit-inc CLI."""

    def test_emit_inc_cli(self, tmp_path):
        """Test --emit-inc option."""
        out = tmp_path / "v.inc"
        result = main(["--emit-inc", str(out)])
        assert result == 0
        assert out.read_text(encoding="utf-8") == render_unity_inc(V)


class TestImportHygiene:
    """Test gen_parity_vectors module import structure."""

    def test_import_hygiene(self):
        """Test no top-level numpy/scipy/controller imports in gen_parity_vectors."""
        module_path = Path(__file__).resolve().parent / "gen_parity_vectors.py"
        tree = ast.parse(module_path.read_text())

        top_level_imports = set()
        lazy_import_modules = set()

        for node in ast.walk(tree):
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                # Top-level: col_offset == 0
                if node.col_offset == 0:
                    if isinstance(node, ast.Import):
                        for alias in node.names:
                            top_level_imports.add(alias.name)
                    elif isinstance(node, ast.ImportFrom):
                        if node.module:
                            top_level_imports.add(node.module)
                # Lazy imports inside functions
                else:
                    if isinstance(node, ast.ImportFrom):
                        if node.module:
                            lazy_import_modules.add(node.module)

        # Top-level: no numpy, scipy, or controller
        assert "numpy" not in top_level_imports
        assert "scipy" not in top_level_imports
        assert "controller" not in top_level_imports

        # hal.f32_emulation, hal.control_core, hal.hal ImuSample must NOT be top-level
        assert "hal.f32_emulation" not in top_level_imports
        assert "hal.control_core" not in top_level_imports

        # No real controller import anywhere
        full_text = module_path.read_text()
        assert "import controller" not in full_text
        assert "from controller" not in full_text


class TestJsonNotePresent:
    """Test JSON note field."""

    def test_json_note_present(self):
        """Test note field contains required disclaimers."""
        note = V["note"]
        assert "not a claim the controller balances" in note
        assert "#359" in note
