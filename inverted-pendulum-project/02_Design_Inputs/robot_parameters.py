#!/usr/bin/env python3
"""
Stage 0: Design Input Loader (Issue #9)

Pure-Python loader/validator for `robot_parameters.yaml`. Deliberately has
ZERO FreeCAD dependency, per CLAUDE.md: FreeCAD is never imported into the
`pendulum-tools` mamba env, and scripts that need it invoke a headless
`freecadcmd` binary externally as a subprocess instead. This module must stay
import-safe in both worlds so the *same* numbers can be read by:

  - FreeCAD-side geometry scripts (Issue #9 Stage 1's
    `07_create_body_and_wheels.py` and later), run under `freecadcmd`;
  - the pure-Python URDF exporter (`urdf_builder.py`, Issue #9 Stage 4);
  - plain `python3 -m pytest` in `pendulum-tools`, no FreeCAD required.

All values loaded from `robot_parameters.yaml` are PROVISIONAL PLACEHOLDERS
per Issue #9's consolidated plan, Decision #3 — see that file's header for
the full rationale. Update the YAML in place once real hardware is sourced;
this loader's schema is meant to stay stable across that change.

Usage:
    from robot_parameters import load_robot_parameters

    params = load_robot_parameters()
    print(params.chassis.length_mm, params.wheel.diameter_mm)
    print(params.target_mass_for_link_kg("Wheel_Left"))

Can also be run standalone to sanity-check and pretty-print the parsed file:
    python3 robot_parameters.py
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from typing import Any, Dict, Optional, Union

import yaml

SCHEMA_VERSION = 1

# Directory containing this module — the default YAML lives right next to it.
_MODULE_DIR = Path(__file__).resolve().parent
DEFAULT_YAML_PATH = _MODULE_DIR / "robot_parameters.yaml"

VALID_STATUSES = {"PLACEHOLDER", "MEASURED"}


class RobotParametersError(ValueError):
    """Raised when robot_parameters.yaml is missing, malformed, or fails
    schema/range validation."""


def _require_keys(data: Dict[str, Any], keys: "tuple[str, ...]", context: str) -> None:
    missing = [key for key in keys if key not in data]
    if missing:
        raise RobotParametersError(
            f"{context}: missing required key(s): {missing}"
        )


def _require_positive(value: float, name: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise RobotParametersError(f"{name} must be numeric, got {value!r}")
    if not math.isfinite(value) or value <= 0:
        raise RobotParametersError(f"{name} must be a finite positive number, got {value}")
    return float(value)


def _require_nonempty_str(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise RobotParametersError(f"{name} must be a non-empty string, got {value!r}")
    return value


def _parse_component(
    raw: Any,
    context: str,
    cls: "type[ComponentSpec]",
    path: Optional[Union[str, Path]] = None,
) -> "ComponentSpec":
    if not isinstance(raw, dict):
        prefix = f"{path}: " if path is not None else ""
        raise RobotParametersError(f"{prefix}'{context}' must be a mapping")
    keys = tuple(f.name for f in fields(cls))
    _require_keys(raw, keys, context)
    return cls(**{key: raw[key] for key in keys})


@dataclass
class ComponentSpec:
    """Fields shared by every physical component (chassis/wheel/pendulum):
    a material label, its density, and a target mass for that component."""

    material: str
    density_kg_m3: float
    target_mass_kg: float

    def validate(self, name: str) -> None:
        _require_nonempty_str(self.material, f"{name}.material")
        _require_positive(self.density_kg_m3, f"{name}.density_kg_m3")
        _require_positive(self.target_mass_kg, f"{name}.target_mass_kg")


@dataclass
class ChassisSpec(ComponentSpec):
    """Base_Link (chassis/body) dimensions."""

    length_mm: float
    width_mm: float
    height_mm: float

    def validate(self, name: str = "chassis") -> None:
        super().validate(name)
        for attr in ("length_mm", "width_mm", "height_mm"):
            _require_positive(getattr(self, attr), f"{name}.{attr}")


@dataclass
class WheelSpec(ComponentSpec):
    """Wheel_Left / Wheel_Right dimensions (identical geometry, mirrored
    placement). `track_mm` is the wheel-centre-to-wheel-centre distance."""

    diameter_mm: float
    width_mm: float
    track_mm: float

    def validate(self, name: str = "wheel") -> None:
        super().validate(name)
        for attr in ("diameter_mm", "width_mm", "track_mm"):
            _require_positive(getattr(self, attr), f"{name}.{attr}")


@dataclass
class PendulumSpec(ComponentSpec):
    """Pendulum_Link envelope (the existing 3-plate + servo linkage from
    Issue #3, reused as a subassembly starting at Issue #9 Stage 1)."""

    arm_length_mm: float
    pivot_height_mm: float

    def validate(self, name: str = "pendulum") -> None:
        super().validate(name)
        for attr in ("arm_length_mm", "pivot_height_mm"):
            _require_positive(getattr(self, attr), f"{name}.{attr}")


@dataclass
class RobotParameters:
    """Top-level, validated view of `robot_parameters.yaml`."""

    schema_version: int
    status: str
    chassis: ChassisSpec
    wheel: WheelSpec
    pendulum: PendulumSpec
    links: Dict[str, str] = field(default_factory=dict)

    @property
    def _components(self) -> "Dict[str, ComponentSpec]":
        return {"chassis": self.chassis, "wheel": self.wheel, "pendulum": self.pendulum}

    def validate(self) -> None:
        if self.schema_version != SCHEMA_VERSION:
            raise RobotParametersError(
                f"schema_version {self.schema_version} is not supported "
                f"(expected {SCHEMA_VERSION})"
            )
        if self.status not in VALID_STATUSES:
            raise RobotParametersError(
                f"status must be one of {sorted(VALID_STATUSES)}, got {self.status!r}"
            )
        self.chassis.validate("chassis")
        self.wheel.validate("wheel")
        self.pendulum.validate("pendulum")

        if not self.links:
            raise RobotParametersError("links: must define at least one link mapping")
        for link_name, component in self.links.items():
            if component not in self._components:
                raise RobotParametersError(
                    f"links.{link_name}: unknown component {component!r} "
                    f"(expected one of {sorted(self._components)})"
                )

    def component_for_link(self, link_name: str) -> ComponentSpec:
        """Return the ComponentSpec (chassis/wheel/pendulum) backing a given
        link name, e.g. component_for_link("Wheel_Left") -> self.wheel."""
        try:
            component_key = self.links[link_name]
        except KeyError as exc:
            raise RobotParametersError(
                f"no component mapping for link {link_name!r} "
                f"(known links: {sorted(self.links)})"
            ) from exc
        return self._components[component_key]

    def target_mass_for_link_kg(self, link_name: str) -> float:
        """Convenience accessor: target mass (kg) for a given link name."""
        return self.component_for_link(link_name).target_mass_kg

    def to_dict(self) -> Dict[str, Any]:
        """JSON-serializable dict view."""
        return asdict(self)


def load_robot_parameters(path: Optional[Union[str, Path]] = None) -> RobotParameters:
    """Load, parse, and validate `robot_parameters.yaml`.

    Args:
        path: Optional override for the YAML file location. Defaults to
            `robot_parameters.yaml` next to this module.

    Returns:
        A fully validated RobotParameters instance.

    Raises:
        RobotParametersError: file missing, not a mapping, missing required
            keys, or any value fails schema/range validation.
    """
    yaml_path = Path(path) if path is not None else DEFAULT_YAML_PATH

    if not yaml_path.is_file():
        raise RobotParametersError(f"robot parameters file not found: {yaml_path}")

    with yaml_path.open("r", encoding="utf-8") as handle:
        try:
            raw = yaml.safe_load(handle)
        except yaml.YAMLError as exc:
            raise RobotParametersError(f"{yaml_path}: invalid YAML: {exc}") from exc

    if not isinstance(raw, dict):
        raise RobotParametersError(f"{yaml_path}: top level must be a mapping")

    _require_keys(raw, ("schema_version", "status", "chassis", "wheel", "pendulum"), str(yaml_path))

    chassis = _parse_component(raw["chassis"], "chassis", ChassisSpec, yaml_path)
    wheel = _parse_component(raw["wheel"], "wheel", WheelSpec, yaml_path)
    pendulum = _parse_component(raw["pendulum"], "pendulum", PendulumSpec, yaml_path)

    links_raw = raw.get("links", {})
    if not isinstance(links_raw, dict):
        raise RobotParametersError(f"{yaml_path}: 'links' must be a mapping if present")

    params = RobotParameters(
        schema_version=raw["schema_version"],
        status=raw["status"],
        chassis=chassis,
        wheel=wheel,
        pendulum=pendulum,
        links=dict(links_raw),
    )
    params.validate()
    return params


def main() -> int:
    """Load the default robot_parameters.yaml and pretty-print it as JSON."""
    try:
        params = load_robot_parameters()
    except RobotParametersError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print(json.dumps(params.to_dict(), indent=2))
    if params.status == "PLACEHOLDER":
        print(
            "\nNOTE: status=PLACEHOLDER — these are provisional values "
            "(Issue #9, Decision #3), not measured hardware.",
            file=sys.stderr,
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
