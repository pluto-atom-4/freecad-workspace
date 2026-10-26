#!/usr/bin/env python3
"""
Pure-stdlib reader and comparison for a Webots PROTO (VRML text) vs the URDF.

- parse_proto(text) -> ProtoRobot: root physics and one ProtoJoint per HingeJoint
  (keyed by RotationalMotor name), from the Robot node.
- urdf_joints(path) -> dict: per-joint parent/child/xyz/rpy/axis from the URDF.
- rpy_to_matrix, axis_angle_to_matrix, mat_vec, max_abs_diff, normalize: 3x3 helpers.
- compare_joints(proto, urdf, tol) -> list[str]: error strings, empty when they match.
  Never asserts.

No Webots, no third-party deps. Brace matching is delegated to
vrml_lexer.find_matching_brace.

Usage:
    from proto_urdf_compare import PROTO_PATH, URDF_PATH, compare_joints, parse_proto, urdf_joints
    errors = compare_joints(parse_proto(PROTO_PATH.read_text()), urdf_joints(URDF_PATH))
"""

import math
import re
import sys
import xml.etree.ElementTree as ET
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


# ---------------------------------------------------------------------------
# URDF reader and PROTO-vs-URDF joint comparison (pure stdlib, no numpy).
# ---------------------------------------------------------------------------

URDF_PATH = Path(__file__).resolve().parents[2] / "06_Exports" / "urdf" / "robot.urdf"
PROTO_PATH = Path(__file__).resolve().parent / "protos" / "InvertedPendulumRobot.proto"

Matrix3 = tuple[tuple[float, float, float], ...]


def _matmul(a: Matrix3, b: Matrix3) -> Matrix3:
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def _rot_x(t: float) -> Matrix3:
    c, s = math.cos(t), math.sin(t)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _rot_y(t: float) -> Matrix3:
    c, s = math.cos(t), math.sin(t)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _rot_z(t: float) -> Matrix3:
    c, s = math.cos(t), math.sin(t)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def rpy_to_matrix(rpy: tuple[float, ...]) -> Matrix3:
    """URDF fixed-axis rpy: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    roll, pitch, yaw = rpy
    return _matmul(_rot_z(yaw), _matmul(_rot_y(pitch), _rot_x(roll)))


def normalize(v: tuple[float, ...]) -> tuple[float, ...]:
    norm = math.sqrt(sum(x * x for x in v))
    if norm == 0.0:
        raise ValueError(f"cannot normalize zero-length vector {v!r}")
    return tuple(x / norm for x in v)


def axis_angle_to_matrix(aa: tuple[float, ...]) -> Matrix3:
    """Rodrigues rotation from (x, y, z, angle). Angle 0 gives identity."""
    angle = aa[3]
    if angle == 0.0:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    x, y, z = normalize(aa[:3])
    c, s = math.cos(angle), math.sin(angle)
    k = 1.0 - c
    return (
        (c + x * x * k, x * y * k - z * s, x * z * k + y * s),
        (y * x * k + z * s, c + y * y * k, y * z * k - x * s),
        (z * x * k - y * s, z * y * k + x * s, c + z * z * k),
    )


def mat_vec(r: Matrix3, v: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(sum(r[i][j] * v[j] for j in range(3)) for i in range(3))


def _flatten(x) -> list[float]:
    out: list[float] = []
    for item in x:
        if isinstance(item, (tuple, list)):
            out.extend(_flatten(item))
        else:
            out.append(float(item))
    return out


def max_abs_diff(a, b) -> float:
    """Largest element-wise |a - b| for vectors or matrices of equal shape."""
    fa, fb = _flatten(a), _flatten(b)
    if len(fa) != len(fb):
        raise ValueError(f"shape mismatch: {len(fa)} vs {len(fb)} elements")
    return max(abs(p - q) for p, q in zip(fa, fb))


def _urdf_vec(elem, attr: str, default: tuple[float, ...], where: str) -> tuple[float, ...]:
    if elem is None or attr not in elem.attrib:
        return default
    vals = tuple(float(tok) for tok in elem.attrib[attr].split())
    if len(vals) != len(default):
        raise ValueError(f"{where}: expected {len(default)} numbers in {attr!r}, got {elem.attrib[attr]!r}")
    return vals


def urdf_joints(path: Path) -> dict[str, dict]:
    """Map joint name to {parent, child, xyz, rpy, axis} from a URDF file."""
    root = ET.parse(path).getroot()
    joints: dict[str, dict] = {}
    for j in root.findall("joint"):
        name = j.get("name")
        if not name:
            raise ValueError(f"{path}: <joint> without a name")
        if name in joints:
            raise ValueError(f"{path}: duplicate joint name {name!r}")
        parent = j.find("parent")
        child = j.find("child")
        if parent is None or child is None:
            raise ValueError(f"{path}: joint {name!r} lacks <parent> or <child>")
        origin = j.find("origin")
        axis = j.find("axis")
        joints[name] = {
            "parent": parent.get("link"),
            "child": child.get("link"),
            "xyz": _urdf_vec(origin, "xyz", (0.0, 0.0, 0.0), name),
            "rpy": _urdf_vec(origin, "rpy", (0.0, 0.0, 0.0), name),
            "axis": _urdf_vec(axis, "xyz", (1.0, 0.0, 0.0), name),
        }
    return joints


def compare_joints(proto: ProtoRobot, urdf: dict, tol: float = 1e-6) -> list[str]:
    """
    Compare PROTO joints against URDF joints. Returns error strings; empty means match.

    The PROTO HingeJointParameters axis is in the parent (Base_Link) frame, not the
    world frame; this holds for the Robot at its default pose (identity). The URDF
    axis is in the joint frame after rpy (PR #157 re-expresses the same physical
    axis), so it is mapped into the parent frame as R(rpy) @ urdf_axis first.
    """
    errors: list[str] = []
    proto_names = set(proto.joints)
    urdf_names = set(urdf)
    for name in sorted(proto_names - urdf_names):
        errors.append(f"{name}: joint key only in PROTO, missing from urdf")
    for name in sorted(urdf_names - proto_names):
        errors.append(f"{name}: joint key only in urdf, missing from PROTO")

    for name in sorted(proto_names & urdf_names):
        pj = proto.joints[name]
        uj = urdf[name]
        d = max_abs_diff(pj.anchor, uj["xyz"])
        if d > tol:
            errors.append(f"{name}: anchor {pj.anchor} != urdf xyz {uj['xyz']} (max diff {d:.3g})")
        d = max_abs_diff(pj.end_translation, uj["xyz"])
        if d > tol:
            errors.append(
                f"{name}: end_translation {pj.end_translation} != urdf xyz {uj['xyz']} (max diff {d:.3g})"
            )
        r_urdf = rpy_to_matrix(uj["rpy"])
        # Tolerance margin (tol = 1e-6 is accepted, do not loosen silently): the real
        # pendulum axis residual is 3.27e-7, about 3x under tol. It comes from pi/2
        # written as 1.570796 (true pi/2 minus that is 3.7e-7) on the URDF side. The
        # rotation residual is exactly 0 because both sides use the same truncated
        # angle. A regeneration with coarser rounding could push the axis residual
        # past tol and flip the result; if so, loosen tol deliberately, not silently.
        try:
            d = max_abs_diff(axis_angle_to_matrix(pj.end_rotation), r_urdf)
            if d > tol:
                errors.append(f"{name}: rotation matrix from end_rotation differs from urdf rpy {uj['rpy']} (max diff {d:.3g})")
        except ValueError as exc:
            errors.append(f"{name}: rotation invalid: {exc}")
        try:
            proto_axis = normalize(pj.axis)
            world_axis = normalize(mat_vec(r_urdf, uj["axis"]))
            d = max_abs_diff(proto_axis, world_axis)
            if d > tol:
                errors.append(f"{name}: axis {proto_axis} != urdf world axis {world_axis} (max diff {d:.3g})")
        except ValueError as exc:
            errors.append(f"{name}: axis invalid: {exc}")
        if pj.end_name != uj["child"]:
            errors.append(f"{name}: child {pj.end_name!r} != urdf child {uj['child']!r}")
    return errors
