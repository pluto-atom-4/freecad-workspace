#!/usr/bin/env python3
"""
Test suite for inject_sensors.py

Tests:
1. Basic injection: Load sensors.yaml, inject into fresh PROTO, verify node present
2. Idempotency: Run injection twice, verify output is byte-identical
3. Multiple sensors: Add second sensor, verify both injected
4. Regression: Verify injected IMU node fields match config (xAxis, yAxis, zAxis, translation)
5. Schema validation: Reject invalid configs (duplicate names, unknown types, unsupported attach_to)
6. Error handling: Gracefully handle missing files, malformed YAML, VRML syntax errors
"""

import pytest
import os
import tempfile
import yaml
from pathlib import Path

from inject_sensors import (
    load_sensor_config,
    render_vrml_sensor_node,
    find_insertion_point,
    inject_sensors,
)


@pytest.fixture
def temp_dir():
    """Create temporary directory for test files."""
    with tempfile.TemporaryDirectory() as tmpdir:
        yield tmpdir


@pytest.fixture
def sample_proto(temp_dir):
    """Create a sample PROTO file with minimal structure."""
    proto_content = """PROTO InvertedPendulumRobot [
  field SFString name "InvertedPendulumRobot"
]
{
  Robot {
    translation 0 0 0.05
    controller "<none>"
    children [
      Pose {
        translation 0 0 -0.05
        rotation 0 0 1 0
        children [
          Shape {
            geometry Box { size 1 1 0.1 }
          }
        ]
      }
      HingeJoint {
        jointParameters HingeJointParameters {
          axis 0 1 0
          anchor 0 0 0
        }
        endPoint Solid {
          translation 0.5 0 0
          physics Physics {}
        }
      }
    ]
  }
}
"""
    proto_path = os.path.join(temp_dir, "InvertedPendulumRobot.proto")
    with open(proto_path, "w") as f:
        f.write(proto_content)
    return proto_path


@pytest.fixture
def sample_sensors_config(temp_dir):
    """Create a sample sensors.yaml config."""
    config = {
        "sensors": [
            {
                "type": "InertialUnit",
                "name": "imu",
                "attach_to": "robot",
                "description": "Body-fixed IMU",
                "fields": {
                    "translation": [0, 0, 0],
                    "rotation": [0, 0, 1, 0],
                    "xAxis": True,
                    "yAxis": True,
                    "zAxis": True,
                },
            }
        ]
    }
    config_path = os.path.join(temp_dir, "sensors.yaml")
    with open(config_path, "w") as f:
        yaml.dump(config, f)
    return config_path


# Test 1: Load config and inject into fresh PROTO
def test_inject_basic(sample_proto, sample_sensors_config, temp_dir):
    """Test basic injection: load config, inject into PROTO, verify node present."""
    assert inject_sensors(sample_proto, sample_sensors_config)

    with open(sample_proto, "r") as f:
        proto_content = f.read()

    # Verify InertialUnit node is present
    assert "InertialUnit {" in proto_content
    assert 'name "imu"' in proto_content
    assert "xAxis TRUE" in proto_content
    assert "yAxis TRUE" in proto_content
    assert "zAxis TRUE" in proto_content


# Test 2: Idempotency
def test_idempotency(sample_proto, sample_sensors_config):
    """Test idempotency: run injection twice, output must be byte-identical."""
    # First injection
    assert inject_sensors(sample_proto, sample_sensors_config)
    with open(sample_proto, "rb") as f:
        first_content = f.read()

    # Second injection (should skip due to idempotency check)
    assert inject_sensors(sample_proto, sample_sensors_config)
    with open(sample_proto, "rb") as f:
        second_content = f.read()

    # Verify byte-identical
    assert first_content == second_content, "Idempotency failed: second run changed file"


# Test 3: Multiple sensors
def test_multiple_sensors(sample_proto, temp_dir):
    """Test injecting multiple sensors."""
    config = {
        "sensors": [
            {
                "type": "InertialUnit",
                "name": "imu_body",
                "attach_to": "robot",
                "fields": {
                    "translation": [0, 0, 0],
                    "xAxis": True,
                    "yAxis": True,
                    "zAxis": True,
                },
            },
            {
                "type": "InertialUnit",
                "name": "imu_head",
                "attach_to": "robot",
                "fields": {
                    "translation": [0, 0, 0.5],
                    "xAxis": True,
                    "yAxis": False,
                    "zAxis": True,
                },
            },
        ]
    }
    config_path = os.path.join(temp_dir, "multi_sensors.yaml")
    with open(config_path, "w") as f:
        yaml.dump(config, f)

    assert inject_sensors(sample_proto, config_path)

    with open(sample_proto, "r") as f:
        proto_content = f.read()

    # Verify both sensors are present
    assert 'name "imu_body"' in proto_content
    assert 'name "imu_head"' in proto_content
    assert proto_content.count("InertialUnit {") == 2


# Test 4: Regression - verify IMU fields match config
def test_imu_fields_regression(sample_proto, sample_sensors_config):
    """Regression test: verify injected IMU fields match config."""
    assert inject_sensors(sample_proto, sample_sensors_config)

    with open(sample_proto, "r") as f:
        proto_content = f.read()

    # Verify exact field values from config
    assert "translation 0 0 0" in proto_content
    assert "rotation 0 0 1 0" in proto_content
    assert 'name "imu"' in proto_content
    assert "xAxis TRUE" in proto_content
    assert "yAxis TRUE" in proto_content
    assert "zAxis TRUE" in proto_content


# Test 5: Schema validation - duplicate names
def test_schema_duplicate_names(temp_dir):
    """Test that duplicate sensor names are rejected."""
    config = {
        "sensors": [
            {
                "type": "InertialUnit",
                "name": "imu",
                "attach_to": "robot",
            },
            {
                "type": "InertialUnit",
                "name": "imu",  # Duplicate
                "attach_to": "robot",
            },
        ]
    }
    config_path = os.path.join(temp_dir, "bad_config.yaml")
    with open(config_path, "w") as f:
        yaml.dump(config, f)

    with pytest.raises(ValueError, match="duplicate name"):
        load_sensor_config(config_path)


# Test 6: Schema validation - unknown sensor type
def test_schema_unknown_type(temp_dir):
    """Test that unknown sensor types are rejected."""
    config = {
        "sensors": [
            {
                "type": "GPS",  # Not supported yet
                "name": "gps",
                "attach_to": "robot",
            }
        ]
    }
    config_path = os.path.join(temp_dir, "bad_type.yaml")
    with open(config_path, "w") as f:
        yaml.dump(config, f)

    with pytest.raises(ValueError, match="unknown type"):
        load_sensor_config(config_path)


# Test 7: Schema validation - unsupported attach_to
def test_schema_unsupported_attach(temp_dir):
    """Test that unsupported attachment points are rejected."""
    config = {
        "sensors": [
            {
                "type": "InertialUnit",
                "name": "imu",
                "attach_to": "joint:pendulum_pivot",  # Deferred for future
            }
        ]
    }
    config_path = os.path.join(temp_dir, "bad_attach.yaml")
    with open(config_path, "w") as f:
        yaml.dump(config, f)

    with pytest.raises(ValueError, match="not supported"):
        load_sensor_config(config_path)


# Test 8: Missing config file graceful handling
def test_missing_config(sample_proto):
    """Test that missing config file is handled gracefully (returns False, not fatal; logs warning)."""
    nonexistent_config = "/nonexistent/sensors.yaml"
    # Returns False when config missing (main() handles gracefully in test context)
    result = inject_sensors(sample_proto, nonexistent_config)
    assert result is False


# Test 9: Malformed YAML
def test_malformed_yaml(sample_proto, temp_dir):
    """Test that malformed YAML is rejected."""
    config_path = os.path.join(temp_dir, "malformed.yaml")
    with open(config_path, "w") as f:
        f.write("sensors: {invalid yaml[")

    with pytest.raises(yaml.YAMLError):
        load_sensor_config(config_path)


# Test 10: Empty config (no sensors)
def test_empty_config(sample_proto, temp_dir):
    """Test that empty config is handled gracefully."""
    config = {"sensors": []}
    config_path = os.path.join(temp_dir, "empty.yaml")
    with open(config_path, "w") as f:
        yaml.dump(config, f)

    assert inject_sensors(sample_proto, config_path)

    # PROTO should be unchanged
    with open(sample_proto, "r") as f:
        proto_content = f.read()
    assert "InertialUnit" not in proto_content


# Test 11: Render VRML node with all fields
def test_render_vrml_all_fields():
    """Test VRML node rendering with all field types."""
    sensor = {
        "type": "InertialUnit",
        "name": "test_imu",
        "fields": {
            "translation": [1.0, 2.0, 3.0],
            "rotation": [0, 0, 1, 1.5707],  # 90 degrees
            "xAxis": True,
            "yAxis": False,
            "zAxis": True,
        },
    }

    vrml = render_vrml_sensor_node(sensor)

    assert "InertialUnit {" in vrml
    assert "translation 1.0 2.0 3.0" in vrml
    assert "rotation 0 0 1 1.5707" in vrml
    assert "xAxis TRUE" in vrml
    assert "yAxis FALSE" in vrml
    assert "zAxis TRUE" in vrml
    assert 'name "test_imu"' in vrml


# Test 12: Render VRML node minimal (name only)
def test_render_vrml_minimal():
    """Test VRML node rendering with minimal fields."""
    sensor = {
        "type": "InertialUnit",
        "name": "minimal_imu",
    }

    vrml = render_vrml_sensor_node(sensor)

    assert "InertialUnit {" in vrml
    assert 'name "minimal_imu"' in vrml
    # No translation, rotation, or axis fields
    assert "translation" not in vrml
    assert "rotation" not in vrml
    assert "xAxis" not in vrml


# Test 13: Find insertion point
def test_find_insertion_point(sample_proto):
    """Test finding the correct insertion point in PROTO."""
    with open(sample_proto, "r") as f:
        proto_content = f.read()

    pos = find_insertion_point(proto_content, attach_to="robot")

    # Insertion point should be after the first Pose block's closing }
    # and before the HingeJoint
    assert pos > 0
    assert proto_content[pos - 1] != "}"  # Not at the brace itself
    assert "HingeJoint" in proto_content[pos:]  # HingeJoint is after insertion point


# Test 14: Missing Pose anchor raises error
def test_missing_pose_anchor(temp_dir):
    """Test that missing Pose anchor raises informative error."""
    proto_path = os.path.join(temp_dir, "no_pose.proto")
    with open(proto_path, "w") as f:
        f.write("PROTO Robot { }")

    config = {
        "sensors": [
            {
                "type": "InertialUnit",
                "name": "imu",
                "attach_to": "robot",
            }
        ]
    }
    config_path = os.path.join(temp_dir, "sensors.yaml")
    with open(config_path, "w") as f:
        yaml.dump(config, f)

    assert not inject_sensors(proto_path, config_path)


# Test 15: VRML brace validation
def test_vrml_brace_validation(sample_proto, sample_sensors_config):
    """Test that VRML braces are validated before and after injection."""
    # Inject normally
    assert inject_sensors(sample_proto, sample_sensors_config)

    # Verify braces are still balanced
    with open(sample_proto, "r") as f:
        content = f.read()

    assert content.count("{") == content.count("}")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
