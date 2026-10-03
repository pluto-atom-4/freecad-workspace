#!/usr/bin/env python3
"""
Unit tests for overlay_text.py (issue #313, sub-issue of #303).

Pure Python; no Webots, no hardware, must NOT `import controller`.

Usage:
    cd poc/servo-wheel-dof
    mamba run -n servo-wheel-dof python3 -m pytest -q \
        webots/controllers/servo_wheel_dof/test_swd_overlay_text.py
"""

from __future__ import annotations

import dataclasses
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from overlay_text import (  # noqa: E402
    COLOR_OK,
    COLOR_WARN,
    ENV_OVERLAY,
    OverlayConfig,
    format_overlay,
    overlay_color,
    overlay_every_steps,
    parse_overlay_env,
)

NAN = float("nan")
INF = float("inf")


def test_format_ok_exact():
    got = format_overlay(0.0, 1.0, (0.0, 0.0, 0.026), False)
    assert got == (
        "servo-wheel OK\n"
        "dev=0.000 mm\n"
        "axis=1.000000\n"
        "rel=(0.00, 0.00, 26.00) mm"
    )


def test_format_warn_exact():
    got = format_overlay(0.002, 0.99, (0.002, -0.001, 0.026), True)
    assert got == (
        "servo-wheel WARN\n"
        "dev=2.000 mm\n"
        "axis=0.990000\n"
        "rel=(2.00, -1.00, 26.00) mm"
    )


def test_format_small_deviation_decimals():
    got = format_overlay(0.0004, 0.9999, (0.0, 0.0, 0.026), False)
    assert got.split("\n")[1] == "dev=0.400 mm"
    assert got.split("\n")[2] == "axis=0.999900"


def test_format_all_nan_and_none_rel():
    got = format_overlay(NAN, None, None, True)
    assert got == (
        "servo-wheel WARN\n"
        "dev=nan mm\n"
        "axis=nan\n"
        "rel=(nan, nan, nan) mm"
    )


def test_format_inf_and_text_become_nan():
    got = format_overlay(INF, "abc", (NAN, INF, None), True)
    assert got == (
        "servo-wheel WARN\n"
        "dev=nan mm\n"
        "axis=nan\n"
        "rel=(nan, nan, nan) mm"
    )


def test_format_bad_rel_length_is_nan():
    for rel in ((1.0, 2.0), (1.0, 2.0, 3.0, 4.0), 5.0, "ab"):
        got = format_overlay(0.0, 1.0, rel, False)
        assert got.split("\n")[3] == "rel=(nan, nan, nan) mm"


def test_format_negative_zero_has_no_minus():
    got = format_overlay(-1e-9, 1.0, (-1e-9, 0.0, 0.026), False)
    assert got == (
        "servo-wheel OK\n"
        "dev=0.000 mm\n"
        "axis=1.000000\n"
        "rel=(0.00, 0.00, 26.00) mm"
    )


def test_format_has_four_lines():
    assert len(format_overlay(0.0, 1.0, (0, 0, 0.026), False).split("\n")) == 4


def test_color_by_warn():
    assert overlay_color(True) == COLOR_WARN == 0xFF0000
    assert overlay_color(False) == COLOR_OK == 0x00C000
    assert overlay_color(1) == COLOR_WARN
    assert overlay_color(0) == COLOR_OK
    assert 0 <= COLOR_OK <= 0xFFFFFF and 0 <= COLOR_WARN <= 0xFFFFFF


@pytest.mark.parametrize(
    "timestep, expected",
    [(16, 16), (32, 8), (20, 13), (250, 1), (1000, 1)],
)
def test_every_steps_default_hz(timestep, expected):
    assert overlay_every_steps(timestep) == expected


def test_every_steps_custom_hz():
    assert overlay_every_steps(10, hz=10.0) == 10


@pytest.mark.parametrize("timestep", [0, -5, NAN, INF, None, "x"])
def test_every_steps_bad_timestep_is_one(timestep):
    assert overlay_every_steps(timestep) == 1


@pytest.mark.parametrize("hz", [0, -1.0, NAN, INF, None])
def test_every_steps_bad_hz_is_one(hz):
    assert overlay_every_steps(16, hz=hz) == 1


@pytest.mark.parametrize("value", ["1", "true", "YES", " On ", "TrUe"])
def test_env_on(value):
    cfg = parse_overlay_env({ENV_OVERLAY: value})
    assert cfg == OverlayConfig(True, ())


@pytest.mark.parametrize("value", ["", "0", "false", "No", "OFF", "  "])
def test_env_off_no_warning(value):
    cfg = parse_overlay_env({ENV_OVERLAY: value})
    assert cfg == OverlayConfig(False, ())


def test_env_unset_is_off():
    assert parse_overlay_env({}) == OverlayConfig(False, ())


def test_env_junk_off_with_one_warning():
    cfg = parse_overlay_env({ENV_OVERLAY: "maybe"})
    assert cfg.enabled is False
    assert len(cfg.warnings) == 1
    assert "SWD_OVERLAY" in cfg.warnings[0]
    assert "'maybe'" in cfg.warnings[0]


def test_env_name_and_frozen():
    assert ENV_OVERLAY == "SWD_OVERLAY"
    cfg = parse_overlay_env({})
    with pytest.raises(dataclasses.FrozenInstanceError):
        cfg.enabled = True  # type: ignore[misc]
