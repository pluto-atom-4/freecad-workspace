#!/usr/bin/env python3
"""
Scanner for Webots controller module leaks: verify no controller imports outside WebotsHal.

Ensure all HAL implementations (except WebotsHal class internals) have zero `import controller`
or `from controller import *` statements at any nesting level, preventing accidental Webots
dependency in other simulation backends or cross-platform code.

Usage:
    mamba run -n pendulum-tools python3 -m pytest -q inverted-pendulum-project/07_Simulation/hal/test_no_webots_leak.py
"""

import ast
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

HAL_DIR = Path(__file__).resolve().parent


def _controller_imports_in_node(node: ast.AST) -> list[int]:
    """Walk AST node (and all children) for controller imports; return line numbers."""
    linenos = []
    for n in ast.walk(node):
        # Check Import statements: import controller, import controller.x, import controller as c
        if isinstance(n, ast.Import):
            for alias in n.names:
                if alias.name == "controller" or alias.name.startswith("controller."):
                    linenos.append(n.lineno)
                    break

        # Check ImportFrom statements (only absolute: level == 0)
        # from controller import ..., from controller.x import ...
        elif isinstance(n, ast.ImportFrom):
            if n.level == 0 and n.module:
                if n.module == "controller" or n.module.startswith("controller."):
                    linenos.append(n.lineno)

        # Check __import__('controller') and importlib.import_module('controller')
        elif isinstance(n, ast.Call):
            func_is_dunder_import = isinstance(n.func, ast.Name) and n.func.id == "__import__"
            func_is_import_module = (
                (isinstance(n.func, ast.Name) and n.func.id == "import_module") or
                (isinstance(n.func, ast.Attribute) and n.func.attr == "import_module")
            )

            if (func_is_dunder_import or func_is_import_module) and n.args:
                first_arg = n.args[0]
                if isinstance(first_arg, ast.Constant) and isinstance(first_arg.value, str):
                    arg_str = first_arg.value
                    if arg_str == "controller" or arg_str.startswith("controller."):
                        linenos.append(n.lineno)

    return sorted(list(set(linenos)))


def _controller_import_linenos(source: str) -> list[int]:
    """Parse source code and return line numbers of all controller imports."""
    try:
        tree = ast.parse(source)
    except SyntaxError:
        return []
    return _controller_imports_in_node(tree)


class TestNoWebotsLeak:
    """Test suite to prevent Webots controller imports in non-Webots code."""

    def test_no_controller_import_in_hal_files(self):
        """All HAL files except webots_hal.py must have zero controller imports."""
        # Collect all .py files in HAL_DIR, excluding webots_hal.py and __pycache__
        files = sorted([
            p for p in HAL_DIR.rglob("*.py")
            if p.name != "webots_hal.py" and "__pycache__" not in p.parts
        ])

        # Verify non-empty and required files present
        assert files, "HAL_DIR contains no .py files"
        file_names = {p.name for p in files}
        assert "hal.py" in file_names, "hal.py not found in HAL_DIR"
        assert "__init__.py" in file_names, "__init__.py not found in HAL_DIR"

        # Check each file for controller imports
        for p in files:
            linenos = _controller_import_linenos(p.read_text(encoding="utf-8"))
            assert linenos == [], f"{p.name}: controller import detected at lines {linenos}"

    def test_webots_hal_no_module_level_controller_import(self):
        """WebotsHal must have zero module-level controller imports."""
        webots_file = HAL_DIR / "webots_hal.py"
        assert webots_file.exists(), f"webots_hal.py not found at {webots_file}"

        webots_source = webots_file.read_text(encoding="utf-8")

        # Check whole file
        linenos = _controller_import_linenos(webots_source)
        assert linenos == [], f"webots_hal.py: controller import at lines {linenos}"

        # Check each module-level non-function/class statement
        tree = ast.parse(webots_source)
        for stmt in tree.body:
            if isinstance(stmt, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                continue

            stmt_linenos = _controller_imports_in_node(stmt)
            assert stmt_linenos == [], \
                f"webots_hal.py: controller import in module-level statement at lines {stmt_linenos}"

    @pytest.mark.parametrize("source", [
        "import controller",
        "from controller import Robot",
        "import controller.x",
        "from controller.y import z",
        "def f():\n    from controller import Supervisor",
        "import controller as c",
        "__import__('controller')",
        "import importlib\nimportlib.import_module('controller')",
    ])
    def test_scanner_flags_leaks(self, source):
        """Scanner must detect all forms of controller imports."""
        linenos = _controller_import_linenos(source)
        assert linenos, f"Scanner failed to detect leak in: {source!r}"

    @pytest.mark.parametrize("source", [
        "from .controller import x",
        "import control_core",
        "from controller_utils import a",
        "x = 'import controller'",
        "# import controller",
    ])
    def test_scanner_ignores_non_leaks(self, source):
        """Scanner must not flag relative imports, similar names, or string literals."""
        linenos = _controller_import_linenos(source)
        assert linenos == [], f"Scanner false-positive on: {source!r}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
