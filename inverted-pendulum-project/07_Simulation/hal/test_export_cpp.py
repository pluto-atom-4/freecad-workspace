"""
Comprehensive unit tests for export_cpp: C++ gains header and parity vectors export.

Tests cover file I/O, validation, C++ repo integration, contract versioning, diff detection,
CLI modes (write, dry-run, check), and import hygiene.

Usage:
    mamba run -n pendulum-tools python3 -m pytest -q inverted-pendulum-project/07_Simulation/hal/test_export_cpp.py
"""

import ast
import os
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import hal.export_cpp as export_cpp
from hal.export_cpp import (
    ExportError,
    HEADER_REL,
    INC_REL,
    HAL_IFACE_REL,
    ENV_VAR,
    MARKER,
    parse_header_stamps,
    parse_inc_stamps,
    render_targets,
    read_cpp_contract_version,
    diff_summary,
    main,
)
from hal.hal import HAL_CONTRACT_VERSION
from hal.gen_lqr_header import render_header
from hal.gen_parity_vectors import DEFAULT_JSON, load_vectors, gains_args, render_unity_inc


V = load_vectors()
EXPECTED_HEADER = render_header(**gains_args(V))
EXPECTED_INC = render_unity_inc(V)


def make_repo(tmp_path, contract=HAL_CONTRACT_VERSION, hal_iface=True):
    """Create a minimal PlatformIO repo structure in tmp_path / "cpp".

    Returns Path to repo root.
    """
    repo = tmp_path / "cpp"
    repo.mkdir(parents=True, exist_ok=True)

    # Write platformio.ini
    (repo / "platformio.ini").write_text("[platformio]\n")

    if hal_iface:
        hal_dir = repo / "lib" / "hal_iface"
        hal_dir.mkdir(parents=True, exist_ok=True)
        (hal_dir / "hal_iface.h").write_text(
            f"// kContractVersion MUST equal HAL_CONTRACT_VERSION\n"
            f"namespace hal {{\n"
            f"constexpr uint32_t kContractVersion = {contract};\n"
            f"}}\n"
        )

    return repo


def files_under(root):
    """Return set of relative POSIX paths of all files under root."""
    result = set()
    for path in root.rglob("*"):
        if path.is_file():
            result.add(path.relative_to(root).as_posix())
    return result


@pytest.fixture(autouse=True)
def clean_env(monkeypatch):
    """Ensure ENV_VAR is not set for each test."""
    monkeypatch.delenv(ENV_VAR, raising=False)


class TestWriteBasics:
    """Tests for write operation basics."""

    def test_write_creates_two_targets(self, tmp_path):
        """Write creates exactly two files with correct content and no temp files."""
        repo = make_repo(tmp_path)
        rc = main(["--cpp-repo", str(repo)])

        assert rc == 0

        hdr_path = repo / HEADER_REL
        inc_path = repo / INC_REL

        assert hdr_path.is_file()
        assert inc_path.is_file()

        hdr_bytes = hdr_path.read_bytes()
        inc_bytes = inc_path.read_bytes()

        assert hdr_bytes == EXPECTED_HEADER.encode("ascii")
        assert inc_bytes == EXPECTED_INC.encode("ascii")

        assert b"\r" not in hdr_bytes
        assert b"\r" not in inc_bytes

        assert hdr_bytes.endswith(b"\n")
        assert inc_bytes.endswith(b"\n")

        files = files_under(repo)
        assert all(not f.endswith(".tmp-export") for f in files)
        assert files == {"platformio.ini", HAL_IFACE_REL, HEADER_REL, INC_REL}

    def test_write_then_check_ok(self, tmp_path, capsys):
        """Write then check succeeds with OK in stdout."""
        repo = make_repo(tmp_path)

        rc = main(["--cpp-repo", str(repo)])
        assert rc == 0

        rc = main(["--cpp-repo", str(repo), "--check"])
        assert rc == 0

        out, err = capsys.readouterr()
        assert "OK" in out
        assert "NOT checked" not in out

    def test_check_detects_one_byte_edit(self, tmp_path, capsys):
        """Check detects single-byte edit in header file."""
        repo = make_repo(tmp_path)
        main(["--cpp-repo", str(repo)])

        hdr_path = repo / HEADER_REL
        hdr_bytes = hdr_path.read_bytes()
        edited = hdr_bytes.replace(b"kOutLimit = 1.0f", b"kOutLimit = 2.0f")
        hdr_path.write_bytes(edited)

        rc = main(["--cpp-repo", str(repo), "--check"])
        assert rc == 1

        out, err = capsys.readouterr()
        assert "DRIFT" in err
        assert "balance_gains_generated.h" in err
        assert "line" in err
        assert len(err) < 2500

    def test_check_missing_target(self, tmp_path, capsys):
        """Check detects missing .inc file."""
        repo = make_repo(tmp_path)
        main(["--cpp-repo", str(repo)])

        inc_path = repo / INC_REL
        inc_path.unlink()

        rc = main(["--cpp-repo", str(repo), "--check"])
        assert rc == 1

        out, err = capsys.readouterr()
        assert "MISSING" in err

    def test_check_crlf_detected(self, tmp_path, capsys):
        """Check detects CRLF line endings."""
        repo = make_repo(tmp_path)
        main(["--cpp-repo", str(repo)])

        hdr_path = repo / HEADER_REL
        hdr_bytes = hdr_path.read_bytes()
        crlf_bytes = hdr_bytes.replace(b"\n", b"\r\n")
        hdr_path.write_bytes(crlf_bytes)

        rc = main(["--cpp-repo", str(repo), "--check"])
        assert rc == 1


class TestPathValidation:
    """Tests for repository path validation."""

    def test_non_repo_path_refused(self, tmp_path, capsys):
        """Non-PlatformIO directory (no platformio.ini) is refused."""
        empty_dir = tmp_path / "empty"
        empty_dir.mkdir()

        rc = main(["--cpp-repo", str(empty_dir)])
        assert rc == 2

        rc = main(["--cpp-repo", str(empty_dir), "--check"])
        assert rc == 2

        assert files_under(empty_dir) == set()

    def test_nonexistent_path_refused(self, tmp_path, capsys):
        """Nonexistent path is refused."""
        nonexistent = tmp_path / "does_not_exist"

        rc = main(["--cpp-repo", str(nonexistent)])
        assert rc == 2

    def test_symlink_target_refused(self, tmp_path):
        """Symlink in target path is refused."""
        pytest.importorskip("os")
        if os.name == "nt":
            pytest.skip("symlinks not supported on Windows")

        repo = make_repo(tmp_path)
        link_target = tmp_path / "external"
        link_target.mkdir()

        lib_path = repo / "lib"
        lib_path.mkdir(exist_ok=True)
        symlink = lib_path / "balance_core"
        symlink.symlink_to(link_target)

        rc = main(["--cpp-repo", str(repo)])
        assert rc == 2


class TestContractVersion:
    """Tests for contract version checking."""

    def test_contract_mismatch_check(self, tmp_path, capsys):
        """Check detects contract version mismatch in C++ repo."""
        repo = make_repo(tmp_path)
        main(["--cpp-repo", str(repo)])

        hal_iface = repo / HAL_IFACE_REL
        hal_iface.write_text(
            f"// kContractVersion MUST equal HAL_CONTRACT_VERSION\n"
            f"namespace hal {{\n"
            f"constexpr uint32_t kContractVersion = {HAL_CONTRACT_VERSION + 1};\n"
            f"}}\n"
        )

        rc = main(["--cpp-repo", str(repo), "--check"])
        assert rc == 1

        out, err = capsys.readouterr()
        assert "CONTRACT" in err

    def test_write_refuses_contract_mismatch(self, tmp_path):
        """Write refuses if C++ repo has mismatched contract version."""
        repo = make_repo(tmp_path, contract=HAL_CONTRACT_VERSION + 1)

        rc = main(["--cpp-repo", str(repo)])
        assert rc == 1

        hdr_path = repo / HEADER_REL
        inc_path = repo / INC_REL
        assert not hdr_path.exists()
        assert not inc_path.exists()

    def test_hal_iface_absent(self, tmp_path, capsys):
        """Write succeeds and check reports contract NOT checked when hal_iface absent."""
        repo = make_repo(tmp_path, hal_iface=False)

        rc = main(["--cpp-repo", str(repo)])
        assert rc == 0

        rc = main(["--cpp-repo", str(repo), "--check"])
        assert rc == 0

        out, err = capsys.readouterr()
        assert "contract NOT checked" in out


class TestOverwriting:
    """Tests for file overwrite behavior."""

    def test_refuse_overwrite_non_generated(self, tmp_path):
        """Write refuses to overwrite non-generated (hand-written) file."""
        repo = make_repo(tmp_path)

        hdr_path = repo / HEADER_REL
        hdr_path.parent.mkdir(parents=True, exist_ok=True)
        hdr_path.write_text("hand written\n")

        rc = main(["--cpp-repo", str(repo)])
        assert rc == 2

        assert hdr_path.read_text() == "hand written\n"

        inc_path = repo / INC_REL
        assert not inc_path.exists()

    def test_overwrite_generated_ok(self, tmp_path):
        """Write overwrites existing generated file with marker."""
        repo = make_repo(tmp_path)

        hdr_path = repo / HEADER_REL
        hdr_path.parent.mkdir(parents=True, exist_ok=True)
        hdr_path.write_text("// GENERATED old\nx\n")

        rc = main(["--cpp-repo", str(repo)])
        assert rc == 0

        assert hdr_path.read_text() == EXPECTED_HEADER

    def test_write_idempotent(self, tmp_path, capsys):
        """Writing twice is idempotent; second run reports UNCHANGED."""
        repo = make_repo(tmp_path)

        rc = main(["--cpp-repo", str(repo)])
        assert rc == 0

        hdr_path = repo / HEADER_REL
        inc_path = repo / INC_REL
        hdr_mtime_1 = hdr_path.stat().st_mtime_ns
        inc_mtime_1 = inc_path.stat().st_mtime_ns

        rc = main(["--cpp-repo", str(repo)])
        assert rc == 0

        out, err = capsys.readouterr()
        assert "UNCHANGED" in out

        hdr_mtime_2 = hdr_path.stat().st_mtime_ns
        inc_mtime_2 = inc_path.stat().st_mtime_ns
        assert hdr_mtime_1 == hdr_mtime_2
        assert inc_mtime_1 == inc_mtime_2


class TestDryRun:
    """Tests for dry-run mode."""

    def test_dry_run_writes_nothing(self, tmp_path, capsys):
        """Dry-run preview does not write any files."""
        repo = make_repo(tmp_path)

        rc = main(["--cpp-repo", str(repo), "--dry-run"])
        assert rc == 0

        files = files_under(repo)
        assert files == {"platformio.ini", HAL_IFACE_REL}

        out, err = capsys.readouterr()
        assert "WOULD" in out

    def test_dry_run_with_check_is_usage_error(self, tmp_path):
        """Dry-run with --check is a usage error."""
        repo = make_repo(tmp_path)

        with pytest.raises(SystemExit) as exc_info:
            main(["--dry-run", "--check", "--cpp-repo", str(repo)])
        assert exc_info.value.code == 2


class TestCLIModes:
    """Tests for different CLI modes and argument validation."""

    def test_write_requires_cpp_repo(self, tmp_path):
        """Write mode requires explicit --cpp-repo (no env var fallback)."""
        with pytest.raises(SystemExit) as exc_info:
            main([])
        assert exc_info.value.code == 2

    def test_python_only_check_ok(self, tmp_path, capsys):
        """Python-only check (no --cpp-repo) succeeds and reports C++ repo NOT checked."""
        rc = main(["--check"])
        assert rc == 0

        out, err = capsys.readouterr()
        assert "C++ repo NOT checked" in out
        assert "verified" not in out.lower()

    def test_env_var_used_by_check(self, tmp_path, capsys, monkeypatch):
        """Check uses ENV_VAR fallback if set."""
        repo = make_repo(tmp_path)
        main(["--cpp-repo", str(repo)])

        monkeypatch.setenv(ENV_VAR, str(repo))

        rc = main(["--check"])
        assert rc == 0

        out, err = capsys.readouterr()
        assert "NOT checked" not in out


class TestVectorValidation:
    """Tests for parity vectors validation."""

    def test_invalid_vectors_refused(self, tmp_path, capsys):
        """Invalid vectors (bad hash) are refused in both write and check."""
        repo = make_repo(tmp_path)

        bad_json = tmp_path / "v.json"
        bad_json.write_text(DEFAULT_JSON.read_text().replace("0x1e60b0fc", "0xdeadbeef"))

        rc = main(["--cpp-repo", str(repo), "--json", str(bad_json)])
        assert rc == 1

        files = files_under(repo)
        assert files == {"platformio.ini", HAL_IFACE_REL}

        rc = main(["--check", "--json", str(bad_json)])
        assert rc == 1

    def test_stamp_mismatch_assertion(self, monkeypatch):
        """Stamp mismatch between header and inc raises ExportError."""
        original_render = export_cpp.render_unity_inc

        def bad_render(v):
            text = original_render(v)
            return text.replace(V["gains_hash"], "0xdeadbeef")

        monkeypatch.setattr(export_cpp, "render_unity_inc", bad_render)

        with pytest.raises(ExportError) as exc_info:
            render_targets(V)
        assert exc_info.value.code == 1


class TestStampParsing:
    """Tests for header and include stamp parsing."""

    def test_stamps_agree(self):
        """Header and inc stamps parse identically."""
        hdr_stamps = parse_header_stamps(EXPECTED_HEADER)
        inc_stamps = parse_inc_stamps(EXPECTED_INC)

        assert hdr_stamps == inc_stamps
        assert hdr_stamps == (V["schema"], HAL_CONTRACT_VERSION, V["gains_hash"])

    def test_read_cpp_contract_version(self, tmp_path):
        """Contract version parsing handles various formats."""
        # Test kContractVersion = 7;
        f1 = tmp_path / "f1.h"
        f1.write_text("kContractVersion = 7;")
        assert read_cpp_contract_version(tmp_path) is None  # no file at HAL_IFACE_REL

        hal_dir = tmp_path / "lib" / "hal_iface"
        hal_dir.mkdir(parents=True, exist_ok=True)

        hal_iface = hal_dir / "hal_iface.h"
        hal_iface.write_text("kContractVersion = 7;")
        assert read_cpp_contract_version(tmp_path) == 7

        # Test without space
        hal_iface.write_text("kContractVersion=7")
        assert read_cpp_contract_version(tmp_path) == 7

        # Test comment-only (no actual definition)
        hal_iface.write_text("// kContractVersion MUST equal ...")
        with pytest.raises(ExportError):
            read_cpp_contract_version(tmp_path)

        # Test two definitions
        hal_iface.write_text("kContractVersion = 7; kContractVersion = 8;")
        with pytest.raises(ExportError):
            read_cpp_contract_version(tmp_path)

        # Test no file
        hal_iface.unlink()
        assert read_cpp_contract_version(tmp_path) is None


class TestDiffSummary:
    """Tests for diff summary formatting."""

    def test_diff_summary_bounded(self):
        """Diff summary is bounded in length and reports line number."""
        exp = "\n".join("line " + str(i) for i in range(1000))
        act = "\n".join("line " + str(i) if i != 499 else "DIFFERENT" for i in range(1000))

        summary = diff_summary("test.h", exp, act)

        assert len(summary) < 600
        assert "500" in summary


class TestModuleHygiene:
    """Tests for import and usage restrictions."""

    def test_module_hygiene(self):
        """export_cpp.py has no forbidden imports or usage patterns."""
        source = Path(export_cpp.__file__).read_text()
        tree = ast.parse(source)

        forbidden_imports = {"subprocess", "socket", "urllib", "http", "requests", "numpy", "scipy", "shutil"}
        forbidden_usage = {"os.system", "control_core", "default_gains"}

        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    assert alias.name not in forbidden_imports
            elif isinstance(node, ast.ImportFrom):
                assert node.module not in forbidden_imports
                if node.module == "hal":
                    assert "control_core" not in [a.name for a in node.names]

        for usage in forbidden_usage:
            assert usage not in source

    def test_no_stray_json_write(self, tmp_path):
        """Write run does not modify DEFAULT_JSON."""
        repo = make_repo(tmp_path)

        json_before = DEFAULT_JSON.read_bytes()

        main(["--cpp-repo", str(repo)])

        json_after = DEFAULT_JSON.read_bytes()
        assert json_before == json_after

    def test_cli_subprocess_python_only(self, tmp_path):
        """Module runs as CLI script in subprocess (no ENV_VAR set)."""
        env = {k: v for k, v in os.environ.items() if k != ENV_VAR}

        result = subprocess.run(
            [sys.executable, str(Path(export_cpp.__file__)), "--check"],
            cwd=str(tmp_path),
            env=env,
            capture_output=True,
            text=True,
        )

        assert result.returncode == 0
        assert "C++ repo NOT checked" in result.stdout


class TestHandVerified:
    """Tests for hand-verified constant values."""

    def test_current_stamps_hand(self):
        """Hand-verified: gains_hash is 0x1e60b0fc and appears in generated output."""
        assert V["gains_hash"] == "0x1e60b0fc"

        assert "kGainsHash = 0x1e60b0fcu;" in EXPECTED_HEADER
        assert "kVecGainsHash = 0x1e60b0fcu;" in EXPECTED_INC
