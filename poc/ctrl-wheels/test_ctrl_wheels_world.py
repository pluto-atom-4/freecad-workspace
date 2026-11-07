#!/usr/bin/env python3
"""
Text-parse test for the two-box geometry of webots/worlds/ctrl_wheels.wbt
(freecad-workspace #535, epic #533).

Reads the .wbt as TEXT and parses it with the stdlib (pytest is the runner); no
Webots is started. It pins the two DEF BODY_BOX_* Transforms (size 0.04 x 0.02
x 0.04 at local z +-0.024, an 8 mm gap), the box outer edge meeting the wheel
inner face (0.044), the HingeJoint anchor equal to the wheel endPoint
translation on each side, the mirrored wheels, the four device names, no
boundingObject or Physics in the two box blocks, and no leftover 0.026 in the
comment-stripped text.

SIM ONLY. This checks numbers in the text only. It does not prove the world
loads in Webots, and it says nothing about balance, wheel direction or sign.

Usage (run from poc/ctrl-wheels):
    python3 -m pytest test_ctrl_wheels_world.py -v
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest


WORLD = Path(__file__).resolve().parent / "webots" / "worlds" / "ctrl_wheels.wbt"
GAP = 0.008
BOX_SIZE = (0.04, 0.02, 0.04)
BOX_CENTRE = 0.024
WHEEL_CENTRE = 0.050
WHEEL_HEIGHT = 0.012
DEVICE_NAMES = ("wheel_motor_left", "wheel_motor_right", "wheel_sensor_left", "wheel_sensor_right")
SIDES = [("LEFT", "left", +1), ("RIGHT", "right", -1)]
ABS = 1e-9
NUM = r"([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)"


def _strip_comments(raw: str) -> str:
    return re.sub(r"#[^\n]*", "", raw)


def _vec(text: str, key: str) -> tuple[float, float, float]:
    """First `key x y z` triple in text, as floats."""
    m = re.search(r"\b" + key + r"\s+" + NUM + r"\s+" + NUM + r"\s+" + NUM, text)
    assert m, f"no '{key}' vector"
    return tuple(float(g) for g in m.groups())


def _block(text: str, open_re: str) -> str:
    """Inner text of the first {...} block whose header matches open_re (ends in a brace)."""
    m = re.search(open_re, text)
    assert m, f"block not found: {open_re}"
    start = m.end()
    depth = 1
    for i in range(start, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[start:i]
    raise AssertionError(f"unbalanced braces after: {open_re}")


@pytest.fixture(scope="module")
def world() -> str:
    return _strip_comments(WORLD.read_text(encoding="utf-8"))


def _box(world: str, side: str) -> str:
    return _block(world, r"DEF\s+BODY_BOX_" + side + r"\s+Transform\s*\{")


def _box_z(world: str, side: str) -> float:
    return _vec(_box(world, side), "translation")[2]


def _hinge(world: str, side_lower: str) -> str:
    want = f'name "wheel_motor_{side_lower}"'
    for m in re.finditer(r"HingeJoint\s*\{", world):
        block = _block(world[m.start():], r"HingeJoint\s*\{")
        if want in block:
            return block
    raise AssertionError(f"no HingeJoint holds {want}")


def _wheel(world: str, side_lower: str):
    """(anchor, endpoint translation, cylinder height) of one wheel."""
    hinge = _hinge(world, side_lower)
    anchor = _vec(hinge, "anchor")
    endpoint = _block(hinge, r"endPoint\s+Solid\s*\{")
    head = endpoint.split("children", 1)[0]
    translation = _vec(head, "translation")
    m = re.search(r"Cylinder\s*\{[^}]*?height\s+" + NUM, endpoint)
    assert m, f"no Cylinder height in {side_lower} wheel"
    return anchor, translation, float(m.group(1))


def test_two_body_box_transforms_exist(world):
    names = re.findall(r"DEF\s+BODY_BOX_(\w+)\s+Transform", world)
    assert len(names) == 2
    assert set(names) == {"LEFT", "RIGHT"}
    for side, _, _ in SIDES:
        x, y, _ = _vec(_box(world, side), "translation")
        assert x == pytest.approx(0.0, abs=ABS)
        assert y == pytest.approx(0.0, abs=ABS)


@pytest.mark.parametrize("side", ["LEFT", "RIGHT"])
def test_box_size(world, side):
    assert _vec(_box(world, side), "size") == pytest.approx(BOX_SIZE, abs=ABS)


def test_box_centres_mirrored(world):
    assert _box_z(world, "LEFT") == pytest.approx(BOX_CENTRE, abs=ABS)
    assert _box_z(world, "RIGHT") == pytest.approx(-BOX_CENTRE, abs=ABS)


def test_gap_is_8mm(world):
    left_z = _box_z(world, "LEFT")
    right_z = _box_z(world, "RIGHT")
    gap = (left_z - BOX_SIZE[2] / 2) - (right_z + BOX_SIZE[2] / 2)
    assert gap == pytest.approx(GAP, abs=ABS)


@pytest.mark.parametrize("side,low,sign", SIDES)
def test_box_outer_edge_meets_wheel_inner_face(world, side, low, sign):
    box_z = _box_z(world, side)
    size_z = _vec(_box(world, side), "size")[2]
    _, translation, height = _wheel(world, low)
    assert height == pytest.approx(WHEEL_HEIGHT, abs=ABS)
    wheel_z = translation[2]
    assert sign * box_z + size_z / 2 == pytest.approx(sign * wheel_z - height / 2, abs=ABS)
    assert sign * wheel_z - height / 2 == pytest.approx(0.044, abs=ABS)


@pytest.mark.parametrize("low,sign", [("left", +1), ("right", -1)])
def test_anchor_equals_endpoint_translation(world, low, sign):
    anchor, translation, _ = _wheel(world, low)
    assert anchor == pytest.approx(translation, abs=ABS)
    assert anchor == pytest.approx((0.0, 0.0, sign * WHEEL_CENTRE), abs=ABS)


def test_wheels_mirrored(world):
    _, left, _ = _wheel(world, "left")
    _, right, _ = _wheel(world, "right")
    assert left[2] == pytest.approx(-right[2], abs=ABS)
    for t in (left, right):
        assert t[0] == pytest.approx(0.0, abs=ABS)
        assert t[1] == pytest.approx(0.0, abs=ABS)


def test_four_device_names_present(world):
    for name in DEVICE_NAMES:
        assert world.count(f'name "{name}"') == 1, name
    for low in ("left", "right"):
        assert f'name "wheel_sensor_{low}"' in _hinge(world, low)


@pytest.mark.parametrize("side", ["LEFT", "RIGHT"])
def test_boxes_have_no_physics_or_bounding_object(world, side):
    block = _box(world, side)
    assert "boundingObject" not in block
    assert "Physics" not in block


def test_no_stale_026_in_code(world):
    assert "0.026" not in world
