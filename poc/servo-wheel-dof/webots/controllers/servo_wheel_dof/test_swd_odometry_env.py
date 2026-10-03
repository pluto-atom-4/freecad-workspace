#!/usr/bin/env python3
"""
Unit tests for parse_odometry_env in odometry.py (issue #323, sub-issue of #301).

Pure Python; no Webots, no hardware, must NOT `import controller`.

Usage:
    cd poc/servo-wheel-dof
    mamba run -n servo-wheel-dof python3 -m pytest -q \
        webots/controllers/servo_wheel_dof/test_swd_odometry_env.py
"""

from __future__ import annotations

import dataclasses
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from odometry import (  # noqa: E402
    ENV_ENABLE,
    OdometryConfig,
    parse_odometry_env,
)


def test_env_enable_constant():
    assert ENV_ENABLE == "SWD_ODOMETRY"


@pytest.mark.parametrize(
    "value",
    ["1", "true", "yes", "on", "YES", "True", "ON", " On ", "\ttrue\n", " 1 "],
)
def test_on_values(value):
    cfg = parse_odometry_env({ENV_ENABLE: value})
    assert cfg == OdometryConfig(True, ())
    assert cfg.enabled is True
    assert cfg.warnings == ()


@pytest.mark.parametrize(
    "value",
    ["", "0", "false", "no", "off", "No", "OFF", "False", "  ", " 0 "],
)
def test_off_values_no_warning(value):
    cfg = parse_odometry_env({ENV_ENABLE: value})
    assert cfg == OdometryConfig(False, ())
    assert cfg.enabled is False
    assert cfg.warnings == ()


def test_missing_key_is_off_without_warning():
    cfg = parse_odometry_env({})
    assert cfg.enabled is False
    assert cfg.warnings == ()


@pytest.mark.parametrize("value", ["maybe", "2", "enable", "y", "10", "truee"])
def test_junk_is_off_with_exactly_one_warning(value):
    cfg = parse_odometry_env({ENV_ENABLE: value})
    assert cfg.enabled is False
    assert len(cfg.warnings) == 1
    assert ENV_ENABLE in cfg.warnings[0]


def test_warning_text_names_variable_value_and_effect():
    cfg = parse_odometry_env({ENV_ENABLE: "maybe"})
    (warning,) = cfg.warnings
    assert "SWD_ODOMETRY" in warning
    assert "maybe" in warning
    assert "odometry stays off" in warning


def test_warning_keeps_original_unstripped_value():
    cfg = parse_odometry_env({ENV_ENABLE: " Maybe "})
    (warning,) = cfg.warnings
    assert repr(" Maybe ") in warning


def test_none_value_is_unset_off_without_warning():
    # Same as parse_telemetry_env / parse_overlay_env: None counts as unset.
    cfg = parse_odometry_env({ENV_ENABLE: None})
    assert cfg.enabled is False
    assert cfg.warnings == ()


def test_non_string_junk_does_not_raise():
    cfg = parse_odometry_env({ENV_ENABLE: 2})
    assert cfg.enabled is False
    assert len(cfg.warnings) == 1
    assert ENV_ENABLE in cfg.warnings[0]


def test_non_string_on_value_is_on():
    assert parse_odometry_env({ENV_ENABLE: True}).enabled is True
    assert parse_odometry_env({ENV_ENABLE: 1}).enabled is True
    assert parse_odometry_env({ENV_ENABLE: 0}).enabled is False


def test_unrelated_variables_do_not_enable():
    cfg = parse_odometry_env(
        {
            "SWD_TELEMETRY": "1",
            "SWD_TELEMETRY_FILE": "/tmp/x.csv",
            "SWD_OVERLAY": "1",
            "SWD_ODOMETRY_X": "1",
        }
    )
    assert cfg == OdometryConfig(False, ())


def test_key_is_case_sensitive():
    cfg = parse_odometry_env({"swd_odometry": "1"})
    assert cfg == OdometryConfig(False, ())


def test_junk_value_beside_unrelated_on_values():
    cfg = parse_odometry_env({"SWD_TELEMETRY": "1", ENV_ENABLE: "maybe"})
    assert cfg.enabled is False
    assert len(cfg.warnings) == 1


def test_config_is_frozen():
    cfg = parse_odometry_env({ENV_ENABLE: "1"})
    with pytest.raises(dataclasses.FrozenInstanceError):
        cfg.enabled = False  # type: ignore[misc]
    with pytest.raises(dataclasses.FrozenInstanceError):
        cfg.warnings = ("x",)  # type: ignore[misc]


def test_warnings_is_tuple_of_str():
    cfg = parse_odometry_env({ENV_ENABLE: "maybe"})
    assert isinstance(cfg.warnings, tuple)
    assert all(isinstance(w, str) for w in cfg.warnings)
    assert parse_odometry_env({}).warnings == ()


def test_environ_is_not_modified():
    environ = {ENV_ENABLE: " On ", "SWD_TELEMETRY": "1"}
    snapshot = dict(environ)
    parse_odometry_env(environ)
    assert environ == snapshot
