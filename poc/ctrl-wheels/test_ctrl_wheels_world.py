#!/usr/bin/env python3
"""
Text-only parse of webots/worlds/ctrl_wheels.wbt, no Webots. SIM ONLY.
Pins body boxes, wheel anchors, device names, and the physics root (Robot mass/CoM,
bounding boxes, skids, Robot pose, 0.5 mm clearance, contact properties, name set,
wheel masses). Does not prove Webots loads the file, and says nothing about balance,
wheel direction or sign.
"""

from __future__ import annotations

import math
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
ROBOT_MASS = 0.064
ROBOT_COM = (0.0, 0.002, 0.0)
WHEEL_MASS = 0.05
WHEEL_RADIUS = 0.03
SKID_X = 0.02
SKID_Y = 0.0195
SKID_SIZE = (0.004, 0.020, 0.004)
ALL_NAMES = set(DEVICE_NAMES) | {"wheel_left", "wheel_right", "floor", "ctrl_wheels"}
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


# Strips the hinge `children [...]` so wheel nodes cannot match; Robot translation/rotation
# lookups still need canonical field order (translation before boundingObject).
def _robot_own(world: str) -> str:
    robot = _block(world, r"DEF\s+CTRL_WHEELS\s+Robot\s*\{")
    for m in re.finditer(r"\bchildren\s*\[", robot):
        depth = 1
        for i in range(m.end(), len(robot)):
            if robot[i] == "[":
                depth += 1
            elif robot[i] == "]":
                depth -= 1
                if depth == 0:
                    break
        else:
            raise AssertionError("unbalanced brackets in Robot children")
        if "HingeJoint" in robot[m.end():i]:
            return robot[:m.start()] + robot[i + 1:]
    raise AssertionError("no Robot children list holding the hinges")


def _group_items(world: str):
    group = _block(_robot_own(world), r"boundingObject\s+Group\s*\{")
    items = []
    for m in re.finditer(r"Transform\s*\{", group):
        inner = _block(group[m.start():], r"Transform\s*\{")
        items.append((_vec(inner, "translation"), _vec(inner, "size")))
    return items


def test_robot_is_physics_root(world):
    phys = _block(_robot_own(world), r"physics\s+Physics\s*\{")
    m = re.search(r"\bmass\s+" + NUM, phys)
    assert m, "no Robot mass"
    assert float(m.group(1)) == pytest.approx(ROBOT_MASS, abs=ABS)
    c = re.search(r"centerOfMass\s*\[\s*" + NUM + r"\s+" + NUM + r"\s+" + NUM + r"\s*\]", phys)
    assert c, "no Robot centerOfMass"
    assert tuple(float(g) for g in c.groups()) == pytest.approx(ROBOT_COM, abs=ABS)


def test_bounding_object_body_boxes_equal_visual_boxes(world):
    items = _group_items(world)
    assert len(items) == 4
    for side, _, _ in SIDES:
        visual = (_vec(_box(world, side), "translation"), _vec(_box(world, side), "size"))
        hits = [
            1 for t, s in items
            if t == pytest.approx(visual[0], abs=ABS) and s == pytest.approx(visual[1], abs=ABS)
        ]
        assert len(hits) == 1, f"no bounding Box equal to BODY_BOX_{side}"


def test_skids_mirrored_and_clear_floor(world):
    skids = [(t, s) for t, s in _group_items(world) if s == pytest.approx(SKID_SIZE, abs=ABS)]
    assert len(skids) == 2
    skids.sort(key=lambda ts: ts[0][0])
    assert skids[0][0] == pytest.approx((-SKID_X, SKID_Y, 0.0), abs=ABS)
    assert skids[1][0] == pytest.approx((+SKID_X, SKID_Y, 0.0), abs=ABS)
    own = _robot_own(world)
    # Robot rotation -90 deg about x puts Robot-frame y DOWN; axle height must equal wheel radius.
    rot = re.search(r"\brotation\s+" + NUM + r"\s+" + NUM + r"\s+" + NUM + r"\s+" + NUM, own)
    assert rot, "no Robot rotation"
    axis_angle = tuple(float(g) for g in rot.groups())
    assert axis_angle == pytest.approx((1.0, 0.0, 0.0, -math.pi / 2), abs=1e-6)
    assert _vec(own, "translation")[2] == pytest.approx(WHEEL_RADIUS, abs=ABS)
    floor = _block(world, r"(?m)^Solid\s*\{")
    assert 'name "floor"' in floor
    assert _vec(floor, "translation")[2] + _vec(floor, "size")[2] / 2 == pytest.approx(0.0, abs=ABS)
    bottom = SKID_Y + SKID_SIZE[1] / 2
    assert WHEEL_RADIUS - bottom == pytest.approx(0.0005, abs=ABS)
    for low in ("left", "right"):
        endpoint = _block(_hinge(world, low), r"endPoint\s+Solid\s*\{")
        r = re.search(r"Cylinder\s*\{[^}]*?radius\s+" + NUM, endpoint)
        assert r and float(r.group(1)) == pytest.approx(WHEEL_RADIUS, abs=ABS)


def test_contact_properties_and_contact_material(world):
    info = _block(world, r"WorldInfo\s*\{")
    cp = _block(info, r"ContactProperties\s*\{")
    assert re.search(r'material1\s+"body"', cp)
    assert re.search(r'material2\s+"default"', cp)
    assert re.search(r"coulombFriction\s*\[\s*0(?:\.0+)?\s*\]", cp)
    assert re.search(r'contactMaterial\s+"body"', _robot_own(world))


def test_name_strings_match_expected(world):
    names = re.findall(r'\bname\s+"([^"]*)"', world)
    assert set(names) == ALL_NAMES
    assert len(names) == len(ALL_NAMES), "duplicate name string"


@pytest.mark.parametrize("low", ["left", "right"])
def test_wheel_physics_mass(world, low):
    endpoint = _block(_hinge(world, low), r"endPoint\s+Solid\s*\{")
    phys = _block(endpoint, r"physics\s+Physics\s*\{")
    m = re.search(r"\bmass\s+" + NUM, phys)
    assert m and float(m.group(1)) == pytest.approx(WHEEL_MASS, abs=ABS)


def test_no_stale_026_in_code(world):
    assert "0.026" not in world
