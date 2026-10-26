#!/usr/bin/env python3
"""
Pure-stdlib reader for a Webots PROTO (VRML text).

Extracts what a PROTO-vs-URDF comparison needs:
- root physics (mass, centerOfMass, inertiaMatrix) from the Robot node;
- one ProtoJoint per HingeJoint, keyed by the RotationalMotor name, with
  axis / anchor from HingeJointParameters and the endPoint Solid's name,
  pose (translation, rotation) and own physics.

No Webots, no third-party deps. Brace matching is delegated to
vrml_lexer.find_matching_brace.

Usage:
    from proto_urdf_compare import parse_proto
    robot = parse_proto(Path("InvertedPendulumRobot.proto").read_text())
"""

import math
import re
import sys
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from vrml_lexer import find_matching_brace  # noqa: E402

IDENTITY_ROTATION = (0.0, 0.0, 1.0, 0.0)

_COMMENT_RE = re.compile(r"#[^\n]*")
_NUMBER_RE = re.compile(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?")
_IDENT_RE = re.compile(r"[A-Za-z_]\w*")
_NODE_OPEN_RE = re.compile(r"([A-Z]\w*)\s*\{")
_DEF_PREFIX_RE = re.compile(r"(?:DEF[ \t]+[^\s{}\[\]\"#]+[ \t]+)?")
_VALUE_NODE_RE = re.compile(r"[A-Za-z_]\w*[ \t]*\{")
_HINGE_RE = re.compile(r"\bHingeJoint\s*\{")
_PROTO_RE = re.compile(r"\bPROTO\s+\w+\s*\[")
_ROBOT_RE = re.compile(r"\bRobot\s*\{")
_CLOSE_FOR = {"{": "}", "[": "]"}


@dataclass(frozen=True)
class Physics:
    mass: float
    com: tuple[float, ...]
    inertia: tuple[float, ...]


@dataclass(frozen=True)
class ProtoJoint:
    name: str
    axis: tuple[float, ...]
    anchor: tuple[float, ...]
    end_name: str
    end_translation: tuple[float, ...]
    end_rotation: tuple[float, ...]
    mass: float
    com: tuple[float, ...]
    inertia: tuple[float, ...]


@dataclass(frozen=True)
class ProtoRobot:
    root: Physics
    joints: dict[str, ProtoJoint]


def floats(text: str) -> tuple[float, ...]:
    """All numbers in text, ignoring '#' comments. Accepts '-0.000000' and '2.000000e-06'."""
    return tuple(float(tok) for tok in _NUMBER_RE.findall(_COMMENT_RE.sub("", text)))


def _eol(text: str, pos: int) -> int:
    """Index of the newline ending the line that contains pos (or len(text))."""
    nl = text.find("\n", pos)
    return len(text) if nl == -1 else nl


def _skip_string(text: str, pos: int) -> int:
    """pos is at an opening quote; return the index just past the closing quote."""
    i = pos + 1
    while i < len(text):
        c = text[i]
        if c == "\\":
            i += 2
            continue
        if c == '"':
            return i + 1
        i += 1
    raise ValueError(f"Unterminated string starting at position {pos}")


def _value_end_of_line(body: str, pos: int) -> int:
    """End of a scalar field value: first newline or '#' outside a string."""
    i = pos
    while i < len(body):
        c = body[i]
        if c == '"':
            i = _skip_string(body, i)
            continue
        if c in "#\n":
            return i
        i += 1
    return i


def _block_opener(body: str, pos: int) -> int | None:
    """
    If the field value starting at pos opens a block on its own line, return the
    index of that '{' or '['; otherwise None.

    The value must open the block: an optional "DEF x" prefix, then either a
    bracket ('[ ...') or a node type followed by '{' ('Physics { ...'). A scalar
    that merely precedes a bracket on the same line ('mass 2 centerOfMass [')
    is not a block value.
    """
    p = _DEF_PREFIX_RE.match(body, pos).end()
    if p < len(body) and body[p] in "{[":
        return p
    node = _VALUE_NODE_RE.match(body, p)
    if node is not None:
        return node.end() - 1
    return None


def _code_only(text: str) -> str:
    """Text with '#' comments and string literals removed (quotes kept as '""')."""
    out: list[str] = []
    i = 0
    while i < len(text):
        c = text[i]
        if c == '"':
            i = _skip_string(text, i)
            out.append('""')
            continue
        if c == "#":
            i = _eol(text, i)
            continue
        out.append(c)
        i += 1
    return "".join(out)


def top_level_fields(body: str) -> dict[str, str]:
    """
    Map field name to raw value text, depth 0 only.

    A value that opens '{' or '[' on its line is read through its matching
    close, so nested Pose/Solid fields never show up here. Other values run to
    end of line. On a duplicate name the first occurrence wins.
    """
    fields: dict[str, str] = {}
    n = len(body)
    i = 0
    while i < n:
        c = body[i]
        if c.isspace():
            i += 1
            continue
        if c == "#":
            i = _eol(body, i)
            continue
        if c == '"':
            i = _skip_string(body, i)
            continue
        m = _IDENT_RE.match(body, i)
        if m is None:
            i += 1
            continue
        name = m.group(0)
        start = m.end()
        while start < n and body[start] in " \t\r":
            start += 1
        open_pos = _block_opener(body, start)
        if open_pos is not None:
            close = _CLOSE_FOR[body[open_pos]]
            end = find_matching_brace(body, open_pos, body[open_pos], close) + 1
        else:
            end = _value_end_of_line(body, start)
        if name not in fields:
            fields[name] = body[start:end].strip()
        i = end
    return fields


def block_body(text: str, open_pos: int) -> str:
    """Text strictly inside the '{...}' or '[...]' block opening at open_pos."""
    if open_pos >= len(text) or text[open_pos] not in _CLOSE_FOR:
        raise ValueError(f"Expected '{{' or '[' at position {open_pos}")
    open_char = text[open_pos]
    end = find_matching_brace(text, open_pos, open_char, _CLOSE_FOR[open_char])
    return text[open_pos + 1:end]


def iter_nodes(list_body: str) -> Iterator[tuple[str, str]]:
    """Yield (TypeName, body) for each 'Type { ... }' at depth 0 of a children/device list."""
    n = len(list_body)
    i = 0
    while i < n:
        c = list_body[i]
        if c.isspace():
            i += 1
            continue
        if c == "#":
            i = _eol(list_body, i)
            continue
        if c == '"':
            i = _skip_string(list_body, i)
            continue
        m = _IDENT_RE.match(list_body, i)
        if m is None:
            i += 1
            continue
        node = _NODE_OPEN_RE.match(list_body, i)
        if node is None:
            i = m.end()
            continue
        open_pos = node.end() - 1
        end = find_matching_brace(list_body, open_pos)
        yield node.group(1), list_body[open_pos + 1:end]
        i = end + 1


def _need(fields: dict[str, str], key: str, where: str) -> str:
    if key not in fields:
        raise ValueError(f"{where}: missing field '{key}'")
    return fields[key]


def _block_of(value: str, where: str, open_char: str = "{") -> str:
    """Inside of the first `open_char` block in a field value."""
    pos = value.find(open_char)
    if pos == -1:
        raise ValueError(f"{where}: expected a '{open_char}' block, got {value[:60]!r}")
    return block_body(value, pos)


def _vec(text: str, n: int, where: str) -> tuple[float, ...]:
    vals = floats(text)
    if len(vals) != n or not all(math.isfinite(v) for v in vals):
        raise ValueError(f"{where}: expected {n} finite numbers, got {text!r}")
    return vals


def _unquote(value: str) -> str:
    s = value.strip()
    if len(s) >= 2 and s[0] == '"' and s[-1] == '"':
        return s[1:-1]
    return s


def _parse_physics(value: str, where: str) -> Physics:
    f = top_level_fields(_block_of(value, where))
    mass = _vec(_need(f, "mass", where), 1, f"{where}.mass")[0]
    com = _vec(_need(f, "centerOfMass", where), 3, f"{where}.centerOfMass")
    inertia = _vec(_need(f, "inertiaMatrix", where), 6, f"{where}.inertiaMatrix")
    return Physics(mass=mass, com=com, inertia=inertia)


def _motor_name(device_value: str, where: str) -> str:
    """Name of the RotationalMotor in a joint's device list (never a sensor's name)."""
    for node_type, node_body in iter_nodes(_block_of(device_value, where, "[")):
        if node_type == "RotationalMotor":
            mf = top_level_fields(node_body)
            return _unquote(_need(mf, "name", f"{where}.RotationalMotor"))
    raise ValueError(f"{where}: no RotationalMotor in device list")


def parse_proto(text: str) -> ProtoRobot:
    """Parse a Webots PROTO's Robot node into root physics and per-joint data."""
    m = _PROTO_RE.search(text)
    if m is None:
        raise ValueError("No 'PROTO <name> [' interface found")
    iface_end = find_matching_brace(text, m.end() - 1, "[", "]")

    rm = _ROBOT_RE.search(text, iface_end + 1)
    if rm is None:
        raise ValueError("No 'Robot {' node after the PROTO interface")
    rf = top_level_fields(block_body(text, rm.end() - 1))

    root = _parse_physics(_need(rf, "physics", "Robot"), "Robot.physics")

    children = _block_of(_need(rf, "children", "Robot"), "Robot.children", "[")
    joints: dict[str, ProtoJoint] = {}
    for node_type, node_body in iter_nodes(children):
        if node_type != "HingeJoint":
            # Only depth-0 children are read. A joint wrapped in another node
            # would be skipped silently, so refuse it instead.
            if _HINGE_RE.search(_code_only(node_body)):
                raise ValueError(
                    f"HingeJoint nested inside a {node_type} node is not supported; "
                    "joints must be direct children of Robot.children"
                )
            continue
        hf = top_level_fields(node_body)

        jp = top_level_fields(
            _block_of(_need(hf, "jointParameters", "HingeJoint"), "HingeJoint.jointParameters")
        )
        axis = _vec(_need(jp, "axis", "HingeJointParameters"), 3, "HingeJointParameters.axis")
        anchor = _vec(_need(jp, "anchor", "HingeJointParameters"), 3, "HingeJointParameters.anchor")

        name = _motor_name(_need(hf, "device", "HingeJoint"), "HingeJoint.device")
        if name in joints:
            raise ValueError(f"Duplicate joint name {name!r}")

        solid_body = _block_of(_need(hf, "endPoint", "HingeJoint"), f"{name}.endPoint")
        if _HINGE_RE.search(_code_only(solid_body)):
            raise ValueError(
                f"A HingeJoint is nested inside joint {name!r}'s endPoint; "
                "chained joints are not supported"
            )
        sf = top_level_fields(solid_body)
        where = f"{name}.endPoint"
        end_name = _unquote(_need(sf, "name", where))
        if "translation" in sf:
            end_translation = _vec(sf["translation"], 3, f"{where}.translation")
        else:
            end_translation = (0.0, 0.0, 0.0)
        if "rotation" in sf:
            end_rotation = _vec(sf["rotation"], 4, f"{where}.rotation")
        else:
            end_rotation = IDENTITY_ROTATION
        phys = _parse_physics(_need(sf, "physics", where), f"{where}.physics")

        joints[name] = ProtoJoint(
            name=name,
            axis=axis,
            anchor=anchor,
            end_name=end_name,
            end_translation=end_translation,
            end_rotation=end_rotation,
            mass=phys.mass,
            com=phys.com,
            inertia=phys.inertia,
        )

    return ProtoRobot(root=root, joints=joints)
