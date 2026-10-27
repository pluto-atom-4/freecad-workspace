#!/usr/bin/env python3
"""
Mutation tests for the PROTO-vs-URDF check (#394, Part of #103 S4).

Proves the joint/link comparison is not vacuous: each mutation is applied to a
text copy of the real PROTO, written to tmp_path, parsed, and must be reported by
compare_joints / compare_links with the expected name and quantity. The real PROTO
is only read (an autouse module guard checks its bytes are unchanged). A missing
PROTO FAILS, it is not skipped (human decision). No Webots, no GUI, no hardware.

Usage:
    cd inverted-pendulum-project
    mamba run -n pendulum-tools python3 -m pytest 07_Simulation/webots/test_proto_urdf_mutation.py -q
"""

import re
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from proto_urdf_compare import (
    PROTO_PATH,
    URDF_PATH,
    compare_joints,
    compare_links,
    parse_proto,
    urdf_joints,
    urdf_links,
)

PENDULUM = ("pendulum_pivot_joint", "pendulum_pivot_right_joint")
AXIS_OLD = "axis 0.000000 1.000000 -0.000000"
PIVOT_AXIS = r"(axis )0\.000000 1\.000000 -0\.000000(\s+anchor -0\.008001 %s)"

# (id, old, new, expected count in real PROTO, regex?, {name: fragment its error must contain})
# Every mutant must produce errors naming exactly these keys (see test body).
MUTANTS = [
    ("pendulum_pivot_joint_axis_z", PIVOT_AXIS % r"-0\.018276 0\.070750", r"\g<1>0 0 1\g<2>", 1, True,
     {"pendulum_pivot_joint": "axis"}),
    ("pendulum_pivot_right_joint_axis_z", PIVOT_AXIS % r"0\.062276 0\.068250", r"\g<1>0 0 1\g<2>", 1, True,
     {"pendulum_pivot_right_joint": "axis"}),
    ("pendulum_end_rotation_zero", "rotation 1.000000 0.000000 0.000000 1.570796",
     "rotation 1.000000 0.000000 0.000000 0", 2, False, {j: "rotation" for j in PENDULUM}),
    ("pendulum_axis_sign_flip", AXIS_OLD, "axis 0.000000 -1.000000 -0.000000", 2, False,
     {j: "axis" for j in PENDULUM}),
    ("wheel_left_anchor_y", "anchor 0.014001 -0.028526 0.054010", "anchor 0.014001 -0.027526 0.054010",
     1, False, {"wheel_left_joint": "anchor"}),
    ("wheel_ixx_iyy_swap", "2.000000e-06 3.000000e-06 2.000000e-06", "3.000000e-06 2.000000e-06 2.000000e-06",
     2, False, {"Wheel_Left": "inertia", "Wheel_Right": "inertia"}),
    ("pendulum_link_ixz_iyz_swap", "4.500000e-05 -2.000000e-06 8.000000e-06",
     "4.500000e-05 8.000000e-06 -2.000000e-06", 1, False, {"Pendulum_Link": "inertia"}),
    # Key-set rows: a renamed joint key, and a renamed Solid (child + link key-set errors).
    ("joint_key_renamed_wheel_left", 'name "wheel_left_joint"', 'name "wheel_left_jointX"', 1, False,
     {"wheel_left_joint": "only in urdf", "wheel_left_jointX": "only in PROTO"}),
    ("solid_renamed_wheel_left", 'name "Wheel_Left"', 'name "Wheel_LeftX"', 1, False,
     {"wheel_left_joint": "child", "Wheel_Left": "only in urdf", "Wheel_LeftX": "only in PROTO"}),
    ("pendulum_link_mass", r"(mass )0\.175000(\s+centerOfMass \[ 0\.013010 0\.018879 0\.002540 \]"
     r"\s+inertiaMatrix \[\s+1\.110000e-04)", r"\g<1>0.176000\g<2>", 1, True, {"Pendulum_Link": "mass"}),
    ("pendulum_link_com_y", "centerOfMass [ 0.013010 0.018879 0.002540 ]",
     "centerOfMass [ 0.013010 0.019879 0.002540 ]", 1, False, {"Pendulum_Link": "com"}),
    ("pendulum_link_ixy_sign", "4.500000e-05 -2.000000e-06 8.000000e-06",
     "-4.500000e-05 -2.000000e-06 8.000000e-06", 1, False, {"Pendulum_Link": "inertia"}),
]


@pytest.fixture(scope="module", autouse=True)
def proto_file_unchanged():
    """The PROTO is read-only for this module: bytes must match after all tests run."""
    before = PROTO_PATH.read_bytes() if PROTO_PATH.exists() else None
    assert before is not None, f"PROTO missing: {PROTO_PATH} (tracked file must exist)"
    yield
    after = PROTO_PATH.read_bytes() if PROTO_PATH.exists() else None
    assert after == before, f"PROTO was modified by the tests: {PROTO_PATH}"


@pytest.fixture(scope="module")
def original():
    assert PROTO_PATH.exists(), f"PROTO missing: {PROTO_PATH} (tracked file must exist)"
    return PROTO_PATH.read_text()


@pytest.fixture(scope="module")
def urdf():
    assert URDF_PATH.exists(), f"URDF missing: {URDF_PATH}"
    return urdf_joints(URDF_PATH)


@pytest.fixture(scope="module")
def links():
    assert URDF_PATH.exists(), f"URDF missing: {URDF_PATH}"
    return urdf_links(URDF_PATH)


def mutate(original, old, new, expected_count, tmp_path, regex=False):
    """Apply one text mutation to the PROTO text, write a tmp copy, return the parsed result."""
    pattern = old if regex else re.escape(old)
    found = len(re.findall(pattern, original, flags=re.S))
    assert found == expected_count, f"{old!r}: found {found} times in PROTO, expected {expected_count}"
    repl = new if regex else (lambda _m: new)
    mutated, n = re.subn(pattern, repl, original, flags=re.S)
    assert n == expected_count and mutated != original, f"mutation {old!r} -> {new!r} was a no-op"
    path = tmp_path / "InvertedPendulumRobot.proto"
    path.write_text(mutated, encoding="utf-8")
    return parse_proto(path.read_text(encoding="utf-8"))


def all_errors(proto, urdf, links):
    return compare_joints(proto, urdf) + compare_links(proto, links)


def test_control_unmutated_copy_is_clean(original, urdf, links):
    # Guards against every mutation passing only because the parse or compare is broken.
    assert all_errors(parse_proto(original), urdf, links) == []


@pytest.mark.parametrize("mid,old,new,count,regex,expected", MUTANTS, ids=[m[0] for m in MUTANTS])
def test_mutation_is_detected(original, urdf, links, tmp_path, mid, old, new, count, regex, expected):
    proto = mutate(original, old, new, count, tmp_path, regex)
    errors = all_errors(proto, urdf, links)
    assert errors, f"{mid}: mutation not detected"
    for name, fragment in expected.items():
        assert any(e.startswith(f"{name}: ") and fragment in e for e in errors), (mid, name, errors)
    # Exactly the expected names are reported: no collateral errors on unrelated joints/links.
    assert {e.split(": ", 1)[0] for e in errors} == set(expected), (mid, errors)
