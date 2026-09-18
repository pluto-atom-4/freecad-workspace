#!/usr/bin/env python3
"""
Inject sensor nodes from YAML config into Webots PROTO file.

This script reads sensor definitions from a YAML configuration file and injects
them as VRML nodes into a Webots PROTO file. Injection is idempotent: if a sensor
with the same name already exists in the PROTO, it is skipped (logged as already injected).

Currently supports:
- InertialUnit sensor nodes only
- robot-attachment only (joint-attachment deferred for future work)

Usage:
    python3 inject_sensors.py --proto /path/to/InvertedPendulumRobot.proto \\
                              --config /path/to/sensors.yaml

    Exit code 0: success (sensors injected or already present)
    Exit code 1: fatal error (config validation, PROTO syntax, injection failure)

The --config argument defaults to sensors.yaml in the same directory as the PROTO
if not specified.
"""

import sys
import argparse
import os
import yaml
import logging
from pathlib import Path
from typing import List, Dict, Any

from vrml_lexer import find_matching_brace


def load_sensor_config(yaml_path: str) -> List[Dict[str, Any]]:
    """
    Load and validate sensor configuration from YAML file.

    Args:
        yaml_path: Path to sensors.yaml

    Returns:
        List of validated sensor dictionaries

    Raises:
        ValueError: If config is malformed or violates schema constraints
        FileNotFoundError: If yaml_path does not exist
        yaml.YAMLError: If YAML parsing fails
    """
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"Config file not found: {yaml_path}")

    with open(yaml_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    if config is None or "sensors" not in config:
        return []

    sensors = config.get("sensors", [])

    if not isinstance(sensors, list):
        raise ValueError(f"'sensors' must be a list, got {type(sensors)}")

    # Validate each sensor
    seen_names = set()
    for i, sensor in enumerate(sensors):
        if not isinstance(sensor, dict):
            raise ValueError(f"Sensor {i} must be a dict, got {type(sensor)}")

        # Required fields
        if "type" not in sensor:
            raise ValueError(f"Sensor {i}: missing required field 'type'")

        if "name" not in sensor:
            raise ValueError(f"Sensor {i}: missing required field 'name'")

        # Validate type
        sensor_type = sensor["type"]
        if sensor_type not in ["InertialUnit"]:
            raise ValueError(
                f"Sensor {i} ('{sensor['name']}'): unknown type '{sensor_type}'. "
                f"Supported types: InertialUnit"
            )

        # Validate name uniqueness
        sensor_name = sensor["name"]
        if sensor_name in seen_names:
            raise ValueError(
                f"Sensor {i}: duplicate name '{sensor_name}' (already defined)"
            )
        seen_names.add(sensor_name)

        # Validate attach_to (currently only "robot" supported)
        attach_to = sensor.get("attach_to", "robot")
        if attach_to != "robot":
            raise ValueError(
                f"Sensor {i} ('{sensor_name}'): attachment point '{attach_to}' not supported. "
                f"Only 'robot' (forward-compatible schema for 'joint:NAME' future work)"
            )

        # Validate fields if present
        if "fields" in sensor:
            if not isinstance(sensor["fields"], dict):
                raise ValueError(
                    f"Sensor {i} ('{sensor_name}'): 'fields' must be a dict, "
                    f"got {type(sensor['fields'])}"
                )

    return sensors


def render_vrml_sensor_node(sensor: Dict[str, Any]) -> str:
    """
    Render a VRML sensor node from sensor dict.

    Args:
        sensor: Sensor dictionary with type, name, fields, etc.

    Returns:
        Multi-line VRML node text (indented for insertion after Pose block)
    """
    node_type = sensor["type"]
    sensor_name = sensor["name"]
    fields = sensor.get("fields", {})

    lines = [f"      {node_type} {{"]

    # Emit translation first if present
    if "translation" in fields:
        val = fields["translation"]
        if isinstance(val, (list, tuple)) and len(val) == 3:
            val_str = " ".join(str(v) for v in val)
            lines.append(f"        translation {val_str}")
        else:
            raise ValueError(
                f"Sensor '{sensor_name}': translation must be [x, y, z], got {val}"
            )

    # Emit rotation second if present
    if "rotation" in fields:
        val = fields["rotation"]
        if isinstance(val, (list, tuple)) and len(val) == 4:
            val_str = " ".join(str(v) for v in val)
            lines.append(f"        rotation {val_str}")
        else:
            raise ValueError(
                f"Sensor '{sensor_name}': rotation must be [x, y, z, angle], got {val}"
            )

    # Emit boolean axis flags
    for axis in ["xAxis", "yAxis", "zAxis"]:
        if axis in fields:
            val = fields[axis]
            if isinstance(val, bool):
                lines.append(f"        {axis} {str(val).upper()}")
            else:
                raise ValueError(
                    f"Sensor '{sensor_name}': {axis} must be boolean, got {type(val)}"
                )

    # Emit name (always present and unique)
    lines.append(f'        name "{sensor_name}"')

    lines.append("      }")

    return "\n".join(lines)


def find_insertion_point(proto_content: str, attach_to: str = "robot") -> int:
    """
    Find the insertion point for a robot-attached sensor node in the PROTO.

    For robot-attached sensors, we insert after the first Pose block (base plate)
    and before the first HingeJoint.

    Args:
        proto_content: PROTO file content as string
        attach_to: Attachment point ("robot" only for now)

    Returns:
        Position after the first Pose block's closing }, ready for insertion

    Raises:
        ValueError: If anchor not found or braces unmatched
    """
    if attach_to != "robot":
        raise ValueError(f"Unsupported attachment: {attach_to}")

    # Find first 'Pose {' in the children array (typically the base plate)
    anchor_pattern = "Pose {"
    anchor_pos = proto_content.find(anchor_pattern)

    if anchor_pos == -1:
        raise ValueError(
            f"Could not find anchor '{anchor_pattern}' in PROTO — "
            "expected at least one Pose block in Robot children"
        )

    # Find the opening brace
    open_brace_pos = proto_content.rfind("{", anchor_pos, anchor_pos + len(anchor_pattern))
    if open_brace_pos == -1:
        raise ValueError(f"Could not find opening brace for Pose at pos {anchor_pos}")

    # Find matching closing brace
    try:
        close_brace_pos = find_matching_brace(proto_content, open_brace_pos)
    except ValueError as e:
        raise ValueError(
            f"Error finding matching brace for first Pose block: {e}"
        )

    # Find the newline after the closing brace (insertion point)
    insert_pos = proto_content.find("\n", close_brace_pos)
    if insert_pos == -1:
        raise ValueError(
            "Could not find newline after first Pose block — unexpected PROTO format"
        )

    return insert_pos + 1


def inject_sensors(proto_path: str, config_path: str) -> bool:
    """
    Main injection function: read PROTO, load sensors from YAML, inject into PROTO.

    Args:
        proto_path: Path to PROTO file to modify
        config_path: Path to sensors.yaml config file

    Returns:
        True if injection succeeded (or no sensors to inject), False on fatal error
    """
    # Read PROTO file
    if not os.path.exists(proto_path):
        logging.error(f"PROTO file not found: {proto_path}")
        return False

    try:
        with open(proto_path, "r", encoding="utf-8") as f:
            proto_content = f.read()
    except IOError as e:
        logging.error(f"Cannot read PROTO file {proto_path}: {e}")
        return False

    # Load sensor config
    try:
        sensors = load_sensor_config(config_path)
    except (FileNotFoundError, yaml.YAMLError, ValueError) as e:
        logging.error(f"Config validation failed: {e}")
        return False

    if not sensors:
        logging.info("No sensors to inject (config empty or missing)")
        return True

    # Pre-check: validate VRML braces before injection
    if proto_content.count("{") != proto_content.count("}"):
        logging.error(
            f"PROTO has mismatched braces before injection: "
            f"{proto_content.count('{')} {{ vs {proto_content.count('}')} }}"
        )
        return False

    # Find insertion point (once, shared for all robot-attached sensors)
    try:
        insert_pos = find_insertion_point(proto_content, attach_to="robot")
    except ValueError as e:
        logging.error(f"Could not find insertion point: {e}")
        return False

    # Inject sensors in order (track cumulative offset due to insertions)
    modified_content = proto_content
    offset = 0
    injected_count = 0

    for sensor in sensors:
        sensor_name = sensor["name"]

        # Idempotency check: skip if sensor name already present in PROTO
        if f'name "{sensor_name}"' in modified_content:
            logging.info(
                f"Sensor '{sensor_name}' already injected, skipping (idempotent)"
            )
            continue

        # Render sensor node
        try:
            sensor_node = render_vrml_sensor_node(sensor)
        except ValueError as e:
            logging.error(f"Failed to render sensor '{sensor_name}': {e}")
            return False

        # Insert at adjusted position (offset due to previous insertions)
        insert_at = insert_pos + offset
        sensor_node_with_newline = sensor_node + "\n"

        modified_content = (
            modified_content[:insert_at]
            + sensor_node_with_newline
            + modified_content[insert_at:]
        )

        offset += len(sensor_node_with_newline)
        injected_count += 1

    # Post-check: validate VRML braces after injection
    if modified_content.count("{") != modified_content.count("}"):
        logging.error(
            f"PROTO has mismatched braces after injection: "
            f"{modified_content.count('{')} {{ vs {modified_content.count('}')} }}"
        )
        return False

    # Write back to file
    try:
        with open(proto_path, "w", encoding="utf-8") as f:
            f.write(modified_content)
    except IOError as e:
        logging.error(f"Cannot write PROTO file {proto_path}: {e}")
        return False

    if injected_count > 0:
        injected_names = [
            s["name"]
            for s in sensors
            if f'name "{s["name"]}"' not in proto_content
        ]
        logging.info(
            f"Injected {injected_count} sensor(s) into PROTO: "
            f"{', '.join(injected_names)}"
        )
    else:
        logging.info(
            "All sensors already present in PROTO (idempotency: no changes made)"
        )

    return True


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Inject sensor nodes from YAML config into Webots PROTO"
    )
    parser.add_argument(
        "--proto",
        required=True,
        help="Path to PROTO file to inject sensors into",
        type=str,
    )
    parser.add_argument(
        "--config",
        help="Path to sensors.yaml config file (default: sensors.yaml in PROTO dir)",
        type=str,
        default=None,
    )

    args = parser.parse_args()

    # Set logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s: %(message)s",
    )

    # Resolve config path
    config_path = args.config
    if not config_path:
        config_path = os.path.join(os.path.dirname(args.proto), "sensors.yaml")

    # Check if config exists; if not, skip injection (not fatal)
    if not os.path.exists(config_path):
        logging.warning(f"Config file not found: {config_path}, skipping injection")
        return 0

    # Run injection
    if inject_sensors(args.proto, config_path):
        return 0
    else:
        return 1


if __name__ == "__main__":
    sys.exit(main())
