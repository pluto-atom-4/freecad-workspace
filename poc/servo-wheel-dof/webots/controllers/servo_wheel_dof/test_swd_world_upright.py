#!/usr/bin/env python3
"""
Text-level guard for the upright rim-on-floor pose in servo_wheel_dof.wbt
(issue #292, sub-issue of #258).

Reads webots/worlds/servo_wheel_dof.wbt as TEXT and asserts the invariants that
keep the wheel standing on its rim: Robot rotation 1 0 0 1.57079633, Robot
translation z equal to the wheel Cylinder radius, hinge axis 0 0 1, hinge anchor
equal to the wheel translation, no rotation on the wheel Solid, boundingObject on
wheel and floor, supervisor TRUE, coordinateSystem "ENU" and the VRML header.
Comments are stripped and fields are read at brace depth 1 of the right node, so
header comments and nested nodes cannot give false matches. Regex and string
scanning only: no Webots, no hardware, must NOT `import controller`. It checks
the text, not the physics.

Mutation check (done by hand): point SWD_WORLD_PATH at a modified COPY of the
world (never edit the real file) and the matching test fails.

Usage:
    cd poc/servo-wheel-dof
    mamba run -n servo-wheel-dof python3 -m pytest -q \
        webots/controllers/servo_wheel_dof/test_swd_world_upright.py
"""

from __future__ import annotations

import os
import re
from functools import lru_cache
from pathlib import Path

import pytest

DEFAULT_WORLD = (
    Path(__file__).resolve().parents[2] / "worlds" / "servo_wheel_dof.wbt"
)
HEADER = "#VRML_SIM R2025a utf8"
ROBOT_PATTERN = r"\bDEF\s+SERVO_WHEEL_DOF\s+Robot\s*\{"
NUM = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"


def _world_path() -> Path:
    override = os.environ.get("SWD_WORLD_PATH")
    return Path(override).expanduser() if override else DEFAULT_WORLD


@lru_cache(maxsize=None)
def _world_text() -> str:
    path = _world_path()
    if not path.is_file():
        pytest.fail(f"world file not found: {path}", pytrace=False)
    return path.read_text(encoding="utf-8")


def _strip_comments(text: str) -> str:
    """Drop `#` comments (full-line and trailing), keeping `#` inside strings."""
    lines = []
    for line in text.splitlines():
        in_str = False
        cut = len(line)
        for i, ch in enumerate(line):
            if ch == '"':
                in_str = not in_str
            elif ch == "#" and not in_str:
                cut = i
                break
        lines.append(line[:cut])
    return "\n".join(lines)


def _match_close(text: str, open_idx: int) -> int:
    """Index of the `}` matching the `{` at open_idx; braces in strings ignored."""
    depth = 0
    in_str = False
    for i in range(open_idx, len(text)):
        ch = text[i]
        if ch == '"':
            in_str = not in_str
        elif in_str:
            continue
        elif ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return i
    raise ValueError(f"unbalanced braces from index {open_idx}")


def _blocks(text: str, pattern: str) -> list[str]:
    """Bodies (without the outer braces) of every node whose header matches.

    The pattern must end with `\\{` so the opening brace is the last match char.
    """
    bodies = []
    for m in re.finditer(pattern, text):
        open_idx = m.end() - 1
        if text[open_idx] != "{":
            raise ValueError(f"pattern must end with a brace: {pattern!r}")
        bodies.append(text[open_idx + 1 : _match_close(text, open_idx)])
    return bodies


def _block(text: str, pattern: str) -> str:
    found = _blocks(text, pattern)
    if len(found) != 1:
        raise AssertionError(
            f"expected exactly 1 node matching {pattern!r}, found {len(found)}"
        )
    return found[0]


def _depth0(body: str) -> str:
    """Body text with everything inside nested {} or [] removed (strings kept)."""
    out = []
    depth = 0
    in_str = False
    for ch in body:
        if in_str:
            if depth == 0:
                out.append(ch)
            in_str = ch != '"'
        elif ch == '"':
            in_str = True
            if depth == 0:
                out.append(ch)
        elif ch in "{[":
            if depth == 0:
                out.append(" ")
            depth += 1
        elif ch in "}]":
            depth -= 1
        elif depth == 0:
            out.append(ch)
    return "".join(out)


def _field_floats(body: str, name: str, count: int) -> list[float] | None:
    """`count` floats following field `name` at the node's own level, or None."""
    pat = rf"\b{name}\s+((?:{NUM}\s+){{{count - 1}}}{NUM})(?![\w.])"
    m = re.search(pat, _depth0(body))
    return [float(tok) for tok in m.group(1).split()] if m else None


def _field_word(body: str, name: str) -> str | None:
    m = re.search(rf"\b{name}\s+(\S+)", _depth0(body))
    return m.group(1) if m else None


def _field_str(body: str, name: str) -> str | None:
    m = re.search(rf'\b{name}\s+"([^"]*)"', _depth0(body))
    return m.group(1) if m else None


def _has_bounding_object(body: str) -> bool:
    return _field_word(body, "boundingObject") not in (None, "NULL")


def _solid_named(code: str, name: str) -> str:
    found = [
        b for b in _blocks(code, r"\bSolid\s*\{") if _field_str(b, "name") == name
    ]
    if len(found) != 1:
        raise AssertionError(f"expected 1 Solid named {name!r}, found {len(found)}")
    return found[0]


@pytest.fixture(scope="module")
def raw() -> str:
    return _world_text()


@pytest.fixture(scope="module")
def code(raw: str) -> str:
    return _strip_comments(raw)


@pytest.fixture(scope="module")
def robot(code: str) -> str:
    return _block(code, ROBOT_PATTERN)


@pytest.fixture(scope="module")
def wheel(code: str) -> str:
    return _solid_named(code, "wheel")


@pytest.fixture(scope="module")
def floor(code: str) -> str:
    return _solid_named(code, "floor")


class TestHelpers:
    """Self-tests of the parsing helpers on small inline VRML snippets."""

    def test_strip_comments_keeps_hash_inside_strings(self) -> None:
        text = 'a 1 # tail\n# full line\nb "x # y" # c\n'
        out = _strip_comments(text)
        assert "tail" not in out
        assert "full" not in out
        assert '"x # y"' in out
        assert out.count("#") == 1

    def test_comment_mentioning_rotation_is_not_a_field(self) -> None:
        text = "# rotation 1 0 0 1.57\nRobot {\n  translation 0 0 1\n}\n"
        body = _block(_strip_comments(text), r"\bRobot\s*\{")
        assert _field_floats(body, "rotation", 4) is None
        assert _field_floats(body, "translation", 3) == [0.0, 0.0, 1.0]

    def test_match_close_ignores_braces_in_strings(self) -> None:
        text = 'Foo { a "}" { b } c } tail'
        close = _match_close(text, text.index("{"))
        assert text[close + 1 :] == " tail"

    def test_match_close_unbalanced_raises(self) -> None:
        with pytest.raises(ValueError):
            _match_close("Foo { a { b }", 4)

    def test_depth0_hides_nested_fields(self) -> None:
        own = "rotation 0 1 0 1 children [ Solid { rotation 1 0 0 2 } ]"
        nested_only = "children [ Solid { rotation 1 0 0 2 } ]"
        assert _field_floats(own, "rotation", 4) == [0.0, 1.0, 0.0, 1.0]
        assert _field_floats(nested_only, "rotation", 4) is None
        assert _field_word(nested_only, "rotation") is None

    def test_float_spellings_parse_to_same_values(self) -> None:
        body = "translation .0 -0 3e-2"
        got = _field_floats(body, "translation", 3)
        assert got == pytest.approx([0.0, 0.0, 0.03])

    def test_block_requires_exactly_one_match(self) -> None:
        with pytest.raises(AssertionError):
            _block("A { }\nA { }\n", r"\bA\s*\{")
        with pytest.raises(AssertionError):
            _block("B { }\n", r"\bA\s*\{")

    def test_bounding_object_null_or_nested_does_not_count(self) -> None:
        assert _has_bounding_object("boundingObject Box { size 1 1 1 }")
        assert not _has_bounding_object("boundingObject NULL")
        assert not _has_bounding_object("name \"x\"")
        assert not _has_bounding_object("children [ Shape { boundingObject Box {} } ]")


class TestWorldUpright:
    """Invariants of the real world file (or the SWD_WORLD_PATH override)."""

    def test_vrml_header_is_first_line(self, raw: str) -> None:
        assert raw.splitlines()[:1] == [HEADER]

    def test_coordinate_system_is_enu(self, code: str) -> None:
        info = _block(code, r"\bWorldInfo\s*\{")
        assert _field_str(info, "coordinateSystem") == "ENU"

    def test_robot_rotation_is_plus_90_deg_about_x(self, robot: str) -> None:
        rot = _field_floats(robot, "rotation", 4)
        assert rot is not None, "Robot has no top-level rotation field"
        assert rot == pytest.approx([1.0, 0.0, 0.0, 1.57079633], abs=1e-8)

    def test_robot_translation_z_equals_wheel_radius(
        self, robot: str, wheel: str
    ) -> None:
        trans = _field_floats(robot, "translation", 3)
        assert trans is not None, "Robot has no top-level translation field"
        geom = _block(wheel, r"\bgeometry\s+Cylinder\s*\{")
        radius = _field_floats(geom, "radius", 1)
        assert radius is not None and radius[0] > 0
        assert trans[2] == pytest.approx(radius[0], abs=1e-9)

    def test_hinge_axis_is_local_z(self, code: str) -> None:
        hinge = _block(code, r"\bHingeJointParameters\s*\{")
        axis = _field_floats(hinge, "axis", 3)
        assert axis == pytest.approx([0.0, 0.0, 1.0])

    def test_hinge_anchor_equals_wheel_translation(
        self, code: str, wheel: str
    ) -> None:
        hinge = _block(code, r"\bHingeJointParameters\s*\{")
        anchor = _field_floats(hinge, "anchor", 3)
        trans = _field_floats(wheel, "translation", 3)
        assert anchor is not None and trans is not None
        assert anchor == pytest.approx(trans, abs=1e-9)

    def test_wheel_solid_has_no_rotation(self, wheel: str) -> None:
        assert _field_word(wheel, "rotation") is None

    @pytest.mark.parametrize("which", ["wheel", "floor"])
    def test_bounding_object_present(
        self, which: str, wheel: str, floor: str
    ) -> None:
        body = wheel if which == "wheel" else floor
        assert _has_bounding_object(body)

    def test_robot_is_supervisor(self, robot: str) -> None:
        assert _field_word(robot, "supervisor") == "TRUE"
