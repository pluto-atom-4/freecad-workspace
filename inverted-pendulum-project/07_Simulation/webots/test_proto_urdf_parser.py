#!/usr/bin/env python3
"""
Unit tests for proto_urdf_compare (PROTO text parser). Inline VRML strings only.

No Webots, no GUI, no real PROTO file.

Usage:
    cd inverted-pendulum-project
    mamba run -n pendulum-tools python3 -m pytest 07_Simulation/webots/test_proto_urdf_parser.py -q
"""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from proto_urdf_compare import (
    block_body,
    floats,
    iter_nodes,
    parse_proto,
    top_level_fields,
)

SAMPLE_PROTO = """#VRML_SIM R2023b utf8
PROTO InvertedPendulumRobot [
  field SFVec3f    translation  0 0 0   # } a comment with a brace
  field SFString   name         "robot"
]
{
Robot {
  translation IS translation
  children [
    HingeJoint {
      jointParameters HingeJointParameters {
        axis   0.000000 1.000000 -0.000000
        anchor 0.1 0 0.2
      }
      device [
        RotationalMotor {
          name "joint_a"
        }
        PositionSensor {
          name "joint_a_sensor"
        }
      ]
      endPoint Solid {
        translation 0.01 0.02 0.03
        rotation 1 0 0 1.5707963
        name "link_a"
        children [
          Pose {
            translation 9 9 9
          }
        ]
        physics Physics {
          mass 0.25
          centerOfMass [ 0.001 0.002 0.003 ]
          inertiaMatrix [
            1.0e-03 2.0e-03 3.0e-03
            2.000000e-06 0 0
          ]
        }
      }
    }
  ]
  name IS name
  physics Physics {
    mass 1.5
    centerOfMass [ 0 0 0.1 ]
    inertiaMatrix [ 0.1 0.2 0.3 0 0 0 ]
  }
}
}
"""

# Template for a one-joint PROTO. @@SOLID@@ is replaced by the endPoint Solid's
# field lines; @@DEVICE@@ by the device list contents.
TEMPLATE = """PROTO TestRobot [
  field SFVec3f translation 0 0 0
]
{
Robot {
  translation IS translation
  children [
    HingeJoint {
      jointParameters HingeJointParameters {
        axis 0 1 0
        anchor 0 0 0
      }
      device [
        @@DEVICE@@
      ]
      endPoint Solid {
@@SOLID@@
        physics Physics {
          mass 0.5
          centerOfMass [ 0 0 0 ]
          inertiaMatrix [ 1 1 1 0 0 0 ]
        }
      }
    }
  ]
  physics Physics {
    mass 2
    centerOfMass [ 0 0 0 ]
    inertiaMatrix [ 1 1 1 0 0 0 ]
  }
}
}
"""

MOTOR_J1 = 'RotationalMotor { name "j1" }'


def build_proto(solid_fields: str, device: str = MOTOR_J1) -> str:
    return TEMPLATE.replace("@@SOLID@@", solid_fields).replace("@@DEVICE@@", device)


# --- floats -----------------------------------------------------------------

def test_floats_accepts_negative_zero_and_exponent():
    assert floats("-0.000000 2.000000e-06") == (-0.0, 2e-06)


def test_floats_ignores_hash_comments():
    assert floats("1 2 # 3 4\n5") == (1.0, 2.0, 5.0)


# --- top_level_fields -------------------------------------------------------

def test_top_level_fields_ignores_nested_pose_fields():
    body = """
    children [
      Pose {
        translation 9 9 9
      }
    ]
    translation 1 2 3
    """
    fields = top_level_fields(body)
    assert fields["translation"] == "1 2 3"
    assert set(fields) == {"children", "translation"}


def test_top_level_fields_keeps_first_duplicate():
    body = "name \"first\"\nname \"second\"\n"
    assert top_level_fields(body)["name"] == '"first"'


def test_brace_in_comment_does_not_break_parsing():
    body = """
    # } this brace is only in a comment
    name "x"   # {
    translation 4 5 6
    """
    fields = top_level_fields(body)
    assert fields["name"] == '"x"'
    assert fields["translation"] == "4 5 6"


def test_brace_in_string_does_not_break_parsing():
    body = """
    name "a}b"
    translation 7 8 9
    """
    fields = top_level_fields(body)
    assert fields["name"] == '"a}b"'
    assert fields["translation"] == "7 8 9"


def test_hyphenated_def_name_does_not_leak_nested_fields():
    body = 'endPoint DEF my-end Solid {\n  name "nested"\n}\nname "outer"\n'
    fields = top_level_fields(body)
    assert set(fields) == {"endPoint", "name"}
    assert fields["name"] == '"outer"'
    assert fields["endPoint"].startswith("DEF my-end Solid {")


def test_unterminated_string_raises_value_error():
    with pytest.raises(ValueError, match="Unterminated string"):
        top_level_fields('name "abc\n')


def test_block_body_returns_inside_of_braces():
    text = "Foo { a 1 b [ 2 ] }"
    assert block_body(text, text.index("{")).strip() == "a 1 b [ 2 ]"


def test_block_body_unmatched_brace_raises_value_error():
    with pytest.raises(ValueError, match="Unmatched"):
        block_body("Foo { a 1", 4)


def test_block_body_rejects_non_block_position():
    with pytest.raises(ValueError):
        block_body("abc", 0)


def test_iter_nodes_yields_type_and_body():
    text = 'RotationalMotor { name "m" } PositionSensor { name "s" }'
    nodes = list(iter_nodes(text))
    assert [t for t, _ in nodes] == ["RotationalMotor", "PositionSensor"]
    assert 'name "m"' in nodes[0][1]


# --- parse_proto ------------------------------------------------------------

def test_parse_proto_sample_joint_and_root():
    robot = parse_proto(SAMPLE_PROTO)

    assert set(robot.joints) == {"joint_a"}
    joint = robot.joints["joint_a"]
    assert joint.axis == (0.0, 1.0, -0.0)
    # -0.000000 must keep its sign; plain == cannot tell -0.0 from 0.0.
    assert math.copysign(1.0, joint.axis[2]) == -1.0
    assert joint.anchor == (0.1, 0.0, 0.2)
    assert joint.end_name == "link_a"
    assert joint.end_translation == (0.01, 0.02, 0.03)
    assert joint.end_rotation == (1.0, 0.0, 0.0, 1.5707963)
    assert joint.mass == 0.25
    assert joint.com == (0.001, 0.002, 0.003)
    assert joint.inertia == (1e-03, 2e-03, 3e-03, 2e-06, 0.0, 0.0)

    assert robot.root.mass == 1.5
    assert robot.root.com == (0.0, 0.0, 0.1)
    assert robot.root.inertia == (0.1, 0.2, 0.3, 0.0, 0.0, 0.0)


def test_missing_rotation_defaults_to_identity():
    robot = parse_proto(build_proto('        name "l1"'))
    # Webots default axis-angle: no rotation (unit Z axis, angle 0).
    assert robot.joints["j1"].end_rotation == (0.0, 0.0, 1.0, 0.0)


def test_missing_translation_defaults_to_origin():
    robot = parse_proto(build_proto('        name "l1"'))
    assert robot.joints["j1"].end_translation == (0.0, 0.0, 0.0)


def test_solid_translation_not_taken_from_nested_pose():
    # The nested Pose translation comes FIRST on purpose: a parser that ignores
    # depth (first-wins) would pick 7 7 7 and fail this test.
    solid = """        name "l1"
        children [
          Pose {
            translation 7 7 7
          }
        ]
        translation 1 1 1"""
    robot = parse_proto(build_proto(solid))
    assert robot.joints["j1"].end_translation == (1.0, 1.0, 1.0)


def test_solid_without_translation_does_not_use_nested_pose():
    solid = """        name "l1"
        children [
          Pose {
            translation 7 7 7
          }
        ]"""
    robot = parse_proto(build_proto(solid))
    assert robot.joints["j1"].end_translation == (0.0, 0.0, 0.0)


def test_key_is_rotational_motor_name_not_sensor_name():
    device = (
        'PositionSensor { name "j1_sensor" }\n'
        '        RotationalMotor { name "j1" }\n'
        '        InertialUnit { name "imu" }'
    )
    robot = parse_proto(build_proto('        name "l1"', device=device))
    assert set(robot.joints) == {"j1"}


def test_zero_padded_negative_axis():
    text = build_proto('        name "l1"').replace(
        "axis 0 1 0", "axis -0.000000 -1.000000 0.000000"
    )
    axis = parse_proto(text).joints["j1"].axis
    assert axis[1] == -1.0
    # "-0.000000" on x must keep its sign; -0.0 == 0.0 under plain ==.
    assert math.copysign(1.0, axis[0]) == -1.0


def test_parse_proto_raises_value_error_on_missing_field():
    text = build_proto('        translation 1 2 3')  # no name field
    with pytest.raises(ValueError, match="missing field 'name'"):
        parse_proto(text)


def test_parse_proto_raises_value_error_without_robot_node():
    with pytest.raises(ValueError, match="Robot"):
        parse_proto("PROTO X [\n]\n{\n}\n")


def test_parse_proto_raises_value_error_without_rotational_motor():
    with pytest.raises(ValueError, match="RotationalMotor"):
        parse_proto(build_proto('        name "l1"', device='PositionSensor { name "s" }'))


def test_duplicate_joint_name_raises_value_error():
    text = build_proto('        name "l1"')
    start = text.index("    HingeJoint {")
    end = text.index("    }\n  ]", start) + len("    }\n")
    doubled = text[:end] + text[start:end] + text[end:]
    with pytest.raises(ValueError, match="Duplicate joint name 'j1'"):
        parse_proto(doubled)


def test_hinge_joint_wrapped_in_pose_raises_value_error():
    text = """PROTO TestRobot [
  field SFVec3f translation 0 0 0
]
{
Robot {
  children [
    Pose {
      children [
        HingeJoint {
          jointParameters HingeJointParameters {
            axis 0 1 0
            anchor 0 0 0
          }
          device [
            RotationalMotor { name "j1" }
          ]
          endPoint Solid {
            name "l1"
            physics Physics {
              mass 0.5
              centerOfMass [ 0 0 0 ]
              inertiaMatrix [ 1 1 1 0 0 0 ]
            }
          }
        }
      ]
    }
  ]
  physics Physics {
    mass 2
    centerOfMass [ 0 0 0 ]
    inertiaMatrix [ 1 1 1 0 0 0 ]
  }
}
}
"""
    with pytest.raises(ValueError, match="nested inside a Pose"):
        parse_proto(text)
