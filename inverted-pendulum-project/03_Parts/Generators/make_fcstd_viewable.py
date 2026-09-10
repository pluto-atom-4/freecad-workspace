#!/usr/bin/env python3
"""
Utility: make a saved .FCStd viewable (visibility + camera framing baked in)

Small, standalone helper -- NOT a generator phase -- for turning an
already-built .FCStd into one that opens with its objects visible and the
camera framed on the whole assembly, without re-running whatever generator
script produced it.

Why this exists: FreeCAD generators in this project (e.g.
`07_create_body_and_wheels.py`) build and save headlessly by default
(`freecadcmd`), so visibility/camera -- both GUI-only concepts -- never get
set, let alone saved. Re-running an entire generator through the live
FreeCAD MCP bridge just to fix visibility rebuilds all its geometry from
scratch (slow, and re-copies any source document it reads from). This
script instead opens an EXISTING .FCStd, sets visibility/camera, and
re-saves -- no geometry rebuild.

Only meaningful with a GUI up (same `App.GuiUp` guard convention as a
generator's own `_set_default_visibility()`/`_set_camera_framing()`) -- run
it through the live FreeCAD MCP bridge's `execute_python`, not headless
`freecadcmd` (headlessly, `make_viewable()` just prints a warning and
returns False; it does not error).

Like the bridge's `execute_python(code="exec(open(path).read())")` gotcha
documented in root CLAUDE.md, a plain `exec(open(path).read())` there does
NOT fire this script's `if __name__ == "__main__":` guard (`__name__` is
`"builtins"` in that context, not `"__main__"`) -- force it explicitly:

    exec(compile(open(path).read(), path, 'exec'),
         {'__name__': '__main__', '__file__': path,
          'sys': __import__('sys')})
    sys.argv = ["make_fcstd_viewable.py", "/path/to/model.FCStd", "Base_Link", "Wheel_Right"]

Or, once the module has been exec'd once (defines make_viewable() in the
live bridge's namespace either way), call the function directly:

    make_viewable("/path/to/model.FCStd", hide=["Base_Link", "Wheel_Right"])
"""

import sys
from pathlib import Path

try:
    import FreeCAD as App
except ImportError:
    print("ERROR: FreeCAD Python modules not available.")
    sys.exit(1)


def make_viewable(path: str, hide=None, save: bool = True) -> bool:
    """Open (or reuse an already-open) document at `path`, show every
    object except those named in `hide`, frame the camera on the whole
    assembly (isometric + fit-all), and save.

    `hide` matches against each object's .Name (case-sensitive, exact --
    same convention as this project's generator scripts), not .Label.

    Returns False (does nothing else) if no GUI is up -- there is no
    ViewObject/3D view to set without one, same as every other GUI-guarded
    method in this project (see root CLAUDE.md's "FreeCAD Live Bridge"
    section).
    """
    if not getattr(App, "GuiUp", False):
        print("⚠ No GUI up (App.GuiUp is False) -- nothing to do. "
              "Run this through the live FreeCAD MCP bridge, not headless freecadcmd.")
        return False

    import FreeCADGui as Gui

    hide_names = set(hide or [])
    resolved_path = str(Path(path).resolve())

    doc = None
    for open_doc in App.listDocuments().values():
        if str(Path(open_doc.FileName).resolve()) == resolved_path:
            doc = open_doc
            break
    if doc is None:
        doc = App.openDocument(resolved_path)
        print(f"✓ Opened document: {resolved_path}")
    else:
        print(f"✓ Reusing already-open document: {doc.Name}")

    doc.recompute()

    shown, hidden = 0, 0
    for obj in doc.Objects:
        view_obj = getattr(obj, "ViewObject", None)
        if view_obj is None:
            continue
        if obj.Name in hide_names:
            view_obj.Visibility = False
            hidden += 1
        else:
            view_obj.Visibility = True
            shown += 1
    print(f"✓ Visibility set: {shown} shown, {hidden} hidden ({sorted(hide_names)})")

    gui_doc = Gui.getDocument(doc.Name)
    view = gui_doc.ActiveView if gui_doc is not None else None
    if view is None:
        print("⚠ WARNING: No ActiveView available; skipping camera framing")
    else:
        view.viewIsometric()
        view.fitAll()
        print("✓ Camera framed: isometric + fit-all")

    if save:
        doc.save()
        print(f"✓ Saved: {doc.FileName}")

    return True


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: make_fcstd_viewable.py <path-to-.FCStd> [object-name-to-hide ...]")
        return 1
    path = sys.argv[1]
    hide = sys.argv[2:]
    return 0 if make_viewable(path, hide=hide) else 1


if __name__ == "__main__":
    sys.exit(main())
