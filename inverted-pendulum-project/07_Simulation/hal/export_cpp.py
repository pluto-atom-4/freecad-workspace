"""Writes C++ gains header and parity vectors include to the balancing-robot-controller repo.

Files only; no git, no network, no parity claim. Manual LOCAL pre-release step; there is no
cross-repo CI. Values are SIM-derived Python-core reference behaviour. PID sign/stability
untested (#359). The C++ pio test output is the only thing that can show parity.
"""

import argparse
import os
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hal.hal import HAL_CONTRACT_VERSION
from hal.gen_lqr_header import render_header
from hal.gen_parity_vectors import (
    DEFAULT_JSON,
    load_vectors,
    validate_vectors,
    gains_args,
    render_unity_inc,
)


HEADER_REL = "lib/balance_core/balance_gains_generated.h"
INC_REL = "test/test_balance_parity/vectors_generated.inc"
HAL_IFACE_REL = "lib/hal_iface/hal_iface.h"
MARKER = "// GENERATED"
ENV_VAR = "BALANCING_ROBOT_CONTROLLER_DIR"
CONTRACT_RE = re.compile(r"kContractVersion\s*=\s*(\d+)")
TMP_SUFFIX = ".tmp-export"

_HDR_STAMP_RES = [
    re.compile(r"kSchema = (\d+);"),
    re.compile(r"kContractVersion = (\d+);"),
    re.compile(r"kGainsHash = (0x[0-9a-f]{8})u;"),
]
_INC_STAMP_RE = re.compile(
    r"kVecSchema = (\d+), kVecContractVersion = (\d+), kVecGainsHash = (0x[0-9a-f]{8})u;"
)


class ExportError(Exception):
    """Exception raised during export; code 2 = refusal/usage, code 1 = content problem."""

    def __init__(self, msg, code=2):
        super().__init__(msg)
        self.code = code


def parse_header_stamps(text):
    """Parse header stamps from text. Returns tuple (schema, contract, hash).

    Raises ExportError(code=1) if any stamp is missing.
    """
    stamps = []
    for regex in _HDR_STAMP_RES:
        match = regex.search(text)
        if not match:
            raise ExportError("header stamp missing: " + regex.pattern, code=1)
        stamps.append(match.group(1))

    schema = int(stamps[0])
    contract = int(stamps[1])
    hash_str = stamps[2]

    return (schema, contract, hash_str)


def parse_inc_stamps(text):
    """Parse include stamps from text. Returns tuple (schema, contract, hash).

    Raises ExportError(code=1) if stamp is missing.
    """
    match = _INC_STAMP_RE.search(text)
    if not match:
        raise ExportError("include stamp missing", code=1)

    schema = int(match.group(1))
    contract = int(match.group(2))
    hash_str = match.group(3)

    return (schema, contract, hash_str)


def load_and_validate(json_path=None):
    """Load and validate parity vectors JSON.

    Returns dict. Raises ExportError on failure.
    """
    if json_path is None:
        json_path = DEFAULT_JSON

    json_path = Path(json_path)
    if not json_path.exists():
        raise ExportError(f"vectors file not found: {json_path}", code=2)

    v = load_vectors(json_path)
    problems = validate_vectors(v)

    if problems:
        msg = "; ".join(problems[:5])
        if len(problems) > 5:
            msg += f" (+{len(problems) - 5} more)"
        raise ExportError(msg, code=1)

    return v


def render_targets(v):
    """Render header and include from vectors. Returns dict mapping rel path -> text.

    Raises ExportError(code=1) on validation failure.
    """
    header = render_header(**gains_args(v))
    inc = render_unity_inc(v)

    # Validate stamps
    hdr_stamps = parse_header_stamps(header)
    inc_stamps = parse_inc_stamps(inc)

    if hdr_stamps != inc_stamps:
        raise ExportError(f"stamp mismatch: header {hdr_stamps} != inc {inc_stamps}", code=1)

    expected_stamps = (v["schema"], HAL_CONTRACT_VERSION, v["gains_hash"])
    if hdr_stamps != expected_stamps:
        raise ExportError(f"stamp mismatch: got {hdr_stamps}, expected {expected_stamps}", code=1)

    if v["contract_version"] != HAL_CONTRACT_VERSION:
        raise ExportError(
            f"contract_version {v['contract_version']} != HAL_CONTRACT_VERSION {HAL_CONTRACT_VERSION}",
            code=1,
        )

    if not header.startswith(MARKER):
        raise ExportError("header does not start with marker", code=1)
    if not inc.startswith(MARKER):
        raise ExportError("include does not start with marker", code=1)

    if not header.isascii():
        raise ExportError("header is not ASCII", code=1)
    if not inc.isascii():
        raise ExportError("include is not ASCII", code=1)

    if "\r" in header:
        raise ExportError("header contains carriage return", code=1)
    if "\r" in inc:
        raise ExportError("include contains carriage return", code=1)

    return {HEADER_REL: header, INC_REL: inc}


def check_repo_root(cpp_repo):
    """Validate and return normalized repo root path.

    Raises ExportError(code=2) if not a valid PlatformIO repo.
    """
    root = Path(cpp_repo).expanduser().resolve()

    if not root.is_dir():
        raise ExportError(f"not a directory: {root}", code=2)

    if not (root / "platformio.ini").is_file():
        raise ExportError(f"not a PlatformIO repo root: {root}", code=2)

    return root


def read_cpp_contract_version(cpp_repo):
    """Read C++ contract version from hal_iface.h. Returns int or None if file absent.

    Raises ExportError(code=1) on parse failure.
    """
    p = Path(cpp_repo) / HAL_IFACE_REL
    if not p.is_file():
        return None

    text = p.read_bytes().decode("utf-8")
    found = CONTRACT_RE.findall(text)

    if len(found) != 1:
        raise ExportError("cannot parse kContractVersion in hal_iface.h", code=1)

    return int(found[0])


def _guard_target(root, rel):
    """Validate target path; return Path. Raises ExportError(code=2) on symlink or escape.

    root: Path to repo root.
    rel: str relative path.
    """
    target = root / rel
    resolved = target.resolve()

    try:
        resolved.relative_to(root)
    except ValueError:
        raise ExportError(f"target escapes repo root: {rel}", code=2)

    # Walk existing components and check for symlinks
    parts = Path(rel).parts
    current = root
    for part in parts:
        current = current / part
        if current.exists() and current.is_symlink():
            raise ExportError(f"refusing symlink in path: {rel}", code=2)

    return target


def plan_outputs(cpp_repo, json_path=None):
    """Plan outputs: load, validate, render. Returns dict mapping Path -> text.

    Raises ExportError on failure. Reads only; never writes.
    """
    if json_path is None:
        json_path = DEFAULT_JSON

    root = check_repo_root(cpp_repo)
    v = load_and_validate(json_path)
    targets = render_targets(v)

    return {_guard_target(root, rel): text for rel, text in targets.items()}


def read_text_exact(path):
    """Read text from path. Never uses read_text()."""
    return path.read_bytes().decode("utf-8")


def is_generated_or_absent(path):
    """Check if path is absent or starts with marker.

    Returns True if absent or generated (first line is marker); False otherwise.
    """
    if not path.exists():
        return True

    try:
        text = read_text_exact(path)
        return text.startswith(MARKER)
    except UnicodeDecodeError:
        return False


def diff_summary(name, expected, actual):
    """Build diff summary string (bounded < 600 chars).

    Compares line-by-line, reports first difference and at most 3 pairs.
    """
    exp_lines = expected.split("\n")
    act_lines = actual.split("\n")

    k = 0
    while k < len(exp_lines) and k < len(act_lines):
        if exp_lines[k] != act_lines[k]:
            break
        k += 1

    summary = f"{name}: line counts exp={len(exp_lines)} act={len(act_lines)}; first difference at line {k + 1}"

    # Add up to 3 differing line pairs
    pairs = 0
    for i in range(k, min(len(exp_lines), len(act_lines))):
        if exp_lines[i] != act_lines[i]:
            e_repr = repr(exp_lines[i][:100])
            a_repr = repr(act_lines[i][:100])
            summary += f"; line {i + 1}: exp={e_repr} act={a_repr}"
            pairs += 1
            if pairs >= 3:
                break

    if len(summary) > 580:
        summary = summary[:580]

    return summary


def check_python_only(json_path=None):
    """Validate Python-only (no C++ repo). Returns list of problem strings."""
    if json_path is None:
        json_path = DEFAULT_JSON

    try:
        load_and_validate(json_path)
        return []
    except ExportError as e:
        return [str(e)]


def check_outputs(cpp_repo, json_path=None):
    """Check C++ outputs against fresh render. Returns (problems, notes).

    problems: list of strings (each a problem to report).
    notes: list of strings (informational).
    """
    if json_path is None:
        json_path = DEFAULT_JSON

    problems = []
    notes = []

    root = check_repo_root(cpp_repo)
    plan = plan_outputs(cpp_repo, json_path)

    for path, expected in plan.items():
        rel = path.relative_to(root).as_posix()

        if not path.exists():
            problems.append(f"MISSING: {rel}")
        else:
            actual = read_text_exact(path)
            if actual != expected:
                problems.append("DRIFT: " + diff_summary(rel, expected, actual))

    # Contract version check
    cv = read_cpp_contract_version(root)
    if cv is None:
        notes.append("contract NOT checked (lib/hal_iface/hal_iface.h absent)")
    elif cv != HAL_CONTRACT_VERSION:
        problems.append(
            f"CONTRACT: C++ kContractVersion={cv} != HAL_CONTRACT_VERSION={HAL_CONTRACT_VERSION}"
        )
    else:
        notes.append(f"C++ kContractVersion={cv} == HAL_CONTRACT_VERSION")

    return (problems, notes)


def write_outputs(plan, dry_run=False, root=None):
    """Write output files from plan dict.

    plan: dict mapping Path -> text.
    dry_run: if True, report what would be written.
    root: if provided, use for relative path reporting.

    Returns list of result lines.
    Raises ExportError(code=2) if any target is non-generated.
    """
    # Preflight: check all targets
    for path in plan.keys():
        if not is_generated_or_absent(path):
            name = path.relative_to(root).as_posix() if root else path.name
            raise ExportError(f"refusing to overwrite non-generated file: {name}", code=2)

    results = []

    for path, text in plan.items():
        name = path.relative_to(root).as_posix() if root else path.name
        new_bytes = text.encode("ascii")

        if path.exists() and path.read_bytes() == new_bytes:
            results.append(f"UNCHANGED {name}")
            continue

        if dry_run:
            if path.exists():
                results.append(f"WOULD WRITE {name}")
            else:
                results.append(f"WOULD CREATE {name}")
            continue

        # Write
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_name(path.name + TMP_SUFFIX)

        try:
            with open(tmp, "w", newline="\n", encoding="ascii") as f:
                f.write(text)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, path)
        finally:
            if tmp.exists():
                tmp.unlink()

        results.append(f"WROTE {name}")

    return results


def main(argv=None):
    """Main entry point.

    Modes:
      --check [--cpp-repo REPO]: validate JSON and optionally C++ outputs.
      --dry-run --cpp-repo REPO: preview writes.
      --cpp-repo REPO: write to repo.
    """
    parser = argparse.ArgumentParser(prog="export_cpp.py")
    parser.add_argument("--cpp-repo", type=str, help="Path to balancing-robot-controller repo")
    parser.add_argument(
        "--check", action="store_true", help="Validate outputs (no --dry-run allowed)"
    )
    parser.add_argument("--dry-run", action="store_true", help="Preview writes")
    parser.add_argument("--json", type=str, default=str(DEFAULT_JSON), help="Path to parity vectors JSON")

    args = parser.parse_args(argv)

    try:
        # Validate argument combinations
        if args.dry_run and args.check:
            parser.error("--dry-run cannot be combined with --check")

        if args.check:
            # Check mode
            repo = args.cpp_repo or os.environ.get(ENV_VAR)

            if not repo:
                # Python-only check
                problems = check_python_only(args.json)
                if problems:
                    for problem in problems:
                        print(problem, file=sys.stderr)
                    return 1

                v = load_and_validate(args.json)
                print(
                    f"OK: parity_vectors.json valid; header/inc stamps agree "
                    f"(schema={v['schema']} contract={v['contract_version']} hash={v['gains_hash']}). "
                    f"C++ repo NOT checked."
                )
                return 0
            else:
                # Check with repo
                problems, notes = check_outputs(repo, args.json)
                if problems:
                    for problem in problems:
                        print(problem, file=sys.stderr)
                    print(
                        f"drift detected: {len(problems)} problem(s); regenerate with --cpp-repo "
                        "(no --check) then review/commit in the C++ repo",
                        file=sys.stderr,
                    )
                    return 1

                print("OK: 2 generated files byte-identical to fresh output; " + "; ".join(notes))
                return 0

        else:
            # Write mode
            if not args.cpp_repo:
                parser.error("write mode requires an explicit --cpp-repo (no environment fallback)")

            root = check_repo_root(args.cpp_repo)
            cv = read_cpp_contract_version(root)

            if cv is not None and cv != HAL_CONTRACT_VERSION:
                raise ExportError(
                    f"C++ kContractVersion={cv} != HAL_CONTRACT_VERSION={HAL_CONTRACT_VERSION}; "
                    "mirror the bump in hal_iface.h first; nothing written",
                    code=1,
                )

            if cv is None:
                print(
                    "warning: lib/hal_iface/hal_iface.h absent; contract version not checked",
                    file=sys.stderr,
                )

            plan = plan_outputs(root, args.json)
            for line in write_outputs(plan, args.dry_run, root):
                print(line)

            v = load_and_validate(args.json)
            print(
                f"gains_hash={v['gains_hash']} contract_version={v['contract_version']}. "
                "Files only: nothing committed or pushed, C++ tests NOT run. "
                "Next (human): review the diff in the C++ repo, run pio test -e test, then commit there. "
                "Do not commit vectors_generated.inc alone: land it together with the #29 test_balance_parity.cpp "
                "(a test folder without a .cpp may break pio test; unverified)."
            )
            return 0

    except ExportError as e:
        print(f"error: {e}", file=sys.stderr)
        return e.code


__all__ = [
    "HEADER_REL",
    "INC_REL",
    "HAL_IFACE_REL",
    "MARKER",
    "ENV_VAR",
    "CONTRACT_RE",
    "TMP_SUFFIX",
    "ExportError",
    "parse_header_stamps",
    "parse_inc_stamps",
    "load_and_validate",
    "render_targets",
    "check_repo_root",
    "read_cpp_contract_version",
    "plan_outputs",
    "read_text_exact",
    "is_generated_or_absent",
    "diff_summary",
    "check_python_only",
    "check_outputs",
    "write_outputs",
    "main",
]


if __name__ == "__main__":
    sys.exit(main())
