# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Workspace Structure

```
freecad-workspace/
├── freecad-mcp-server/        # FreeCAD MCP Server dev (mamba/conda only)
├── inverted-pendulum-project/ # Pendulum simulation & modeling (mamba/conda only)
├── .gitignore
├── README.md
└── CLAUDE.md
```

Both projects are **mamba-only** — no `uv`, `pyproject.toml`, `uv.lock`, or `.venv` anywhere in
this workspace. Each has its own env (`freecad-mcp`, `pendulum-tools`), its own
`mamba-envs.yaml` (recipe) + `mamba-envs.lock.yml` (pinned export). FreeCAD itself is never
installed into either env: `freecad-mcp-server` talks to it externally over XML-RPC/socket (GUI
AppImage); `inverted-pendulum-project` invokes it externally as a headless `freecadcmd`
subprocess, no MCP in its shipped pipeline. Don't assume `uv sync`/`uv run` work here.

## Projects Overview

**freecad-mcp-server** — MCP bridge for AI assistants to control FreeCAD. PyPI package
`freecad-robust-mcp` (import name `freecad_mcp`, not `freecad_robust_mcp`). Connection modes:
XML-RPC (port 9875, recommended), JSON-RPC socket (9876), embedded (Linux only, avoid on
macOS/Windows). Upstream source: [spkane/freecad-addon-robust-mcp-server](https://github.com/spkane/freecad-addon-robust-mcp-server)
(its own separate `uv`/`mise`/`just` tooling — unrelated to this workspace's mamba setup).

**inverted-pendulum-project** — Pendulum simulation (numpy/scipy/matplotlib) + parametric CAD
generation (cadquery → OCP, trimesh). FreeCAD integration is headless-subprocess-only via
`FREECAD_BIN` (see below), never imported into the `pendulum-tools` env — it bundles its own
OpenCASCADE build, which conflicts with OCP's if mixed in-process.

## Common Development Commands

```bash
# freecad-mcp-server
mamba run -n freecad-mcp python3 <script.py>
mamba run -n freecad-mcp python3 -c "import freecad_mcp; print(freecad_mcp.__version__)"
mamba run -n freecad-mcp freecad-mcp --version  # also confirms the required "mcp<2" pin is
  # intact -- a bare `pip install freecad-robust-mcp` pulls mcp 2.x, which crashes with
  # ModuleNotFoundError: No module named 'mcp.server.fastmcp'

# inverted-pendulum-project
cd inverted-pendulum-project
export FREECAD_BIN=~/.local/bin/freecadcmd1.1   # or rely on "freecadcmd" on PATH
mamba run -n pendulum-tools python3 -m pytest -q
```

**`freecadcmd` 1.1.3 in this environment does not set `__name__ == "__main__"` for a plain
positional or `--python` script argument** — a script's `if __name__ == "__main__":` guard
silently never fires, exits 0 having done nothing (verified empirically, not documented
upstream). Working invocation: `"$FREECAD_BIN" -c "exec(open('script.py').read())"`.

## MCP Client Configuration

`.mcp.json` (project root) configures the `freecad` MCP server. PyPI package usage:
`"command": "freecad-mcp"`, env `FREECAD_MODE=xmlrpc`. For an upstream source checkout instead,
`"command": "uv"` with `"args": ["run", "--project", "/path/to/freecad-addon-robust-mcp-server", "freecad-mcp"]`.
See [Robust MCP Server Docs](https://spkane.github.io/freecad-addon-robust-mcp-server/) for the
full tool catalog (150+ tools) rather than enumerating it here.

## Git & GitHub

- Repository: freecad-workspace on GitHub. Each project is self-contained.
- `.gitignore` excludes `.FCStd`/`.FCBak` files, Python envs, build artifacts — these live on
  disk locally only; regenerate via each project's `0N_*.py` scripts, don't expect them tracked.

## Key Files

| File | Purpose |
|------|---------|
| `<project>/mamba-envs.yaml` | env recipe (unpinned) |
| `<project>/mamba-envs.lock.yml` | env, pinned/reproducible export |
| `.mcp.json` | MCP server configuration (project-level) |

## Troubleshooting

**MCP client can't connect to FreeCAD:** verify FreeCAD's MCP bridge is running (console shows
"MCP Bridge started!"), check port (9875 XML-RPC / 9876 socket), `FREECAD_SOCKET_HOST` matches.

**freecad-mcp env broken:**
`mamba env remove -n freecad-mcp -y && mamba env create -n freecad-mcp -f freecad-mcp-server/mamba-envs.lock.yml`
(recreating from the unpinned recipe instead: keep the `mcp<2` pin in `pip_packages`.)

**pendulum-tools env broken:**
`mamba env remove -n pendulum-tools -y && mamba env create -n pendulum-tools -f inverted-pendulum-project/mamba-envs.lock.yml`

## FreeCAD Live Bridge — Known Limitations (freecad-mcp-workbench 0.6.2)

The live `mcp__freecad__*` bridge is occasionally borrowed by `inverted-pendulum-project` as an
ad-hoc human-review aid (its own pipeline stays headless-only). Found while doing so:

- **`get_screenshot` is broken** (`AttributeError: 'dict' object has no attribute '__name__'` on
  every call). Workaround: `execute_python` → `FreeCADGui.ActiveDocument.ActiveView.saveImage(path, w, h)`
  directly, then read the file back off disk.
- **`inspect_object` errors on `App::Part` container objects** (fine on `Part::Feature`/`Mesh::Feature`).
  Workaround: read `.Group`/`.Placement` etc. directly via `execute_python`.
- **Visibility and camera are GUI-only state** — no `ViewObject`/3D view exists in a true headless
  `freecadcmd` run. Guard any such code with `if not getattr(App, "GuiUp", False): return` (see
  `07_create_body_and_wheels.py`'s `_set_default_visibility()`/`_set_camera_framing()`). A camera
  set this way only embeds in the `.FCStd` if the save itself happens under a GUI.
- **A live GUI auto-tessellates shapes for display, which can shrink `Shape.BoundBox` reads** —
  observed a 70.00mm cylinder reading ~69.90mm. Validate dimensions against a true headless run,
  not a live-bridge session; when tessellating in a script, always tessellate a `.copy()` of the
  shape, never `obj.Shape` itself (same contamination happens self-inflicted, even headlessly).
- **To produce a fully-viewable `.FCStd` (visible objects + framed camera baked in), run the
  generator script itself through the bridge** — its own `App.GuiUp`-guarded code needs a GUI to
  fire. `execute_python(code="exec(open(path).read())")` alone silently no-ops: `__name__` in
  that context is `"builtins"`, not `"__main__"`, so the script's `if __name__ == "__main__":`
  guard never runs. Force it explicitly:
  `exec(compile(open(path).read(), path, 'exec'), {'__name__': '__main__', '__file__': path})`.
- **A `Mesh::Feature`'s own `Placement` is ignored when nested in an `App::Part`** — only the
  immediate parent container's `getGlobalPlacement()` applied directly to its raw `.Mesh` data
  matches what actually renders (verified by isolating one mesh + one plate, comparing to a
  screenshot). `Part::Feature` follows the normal convention (own `Placement` composes
  normally); only `Mesh::Feature` has this quirk. Not fully explained, but reproducible.
- **`Mesh.transform()` with a reflection matrix (e.g. `Matrix().scale(-1,1,1)`) can silently
  produce wrongly-offset geometry** on some meshes (observed: one coordinate shifted by a large,
  consistent, unexplained amount) — a real bug, not a math error. Mirror via raw point data
  instead: negate the coordinate on every vertex from `mesh.Topology`, reverse each facet's
  vertex order to fix normals, rebuild with `Mesh.Mesh((points, facets))`.
- **`distToShape() == 0` doesn't distinguish touching from overlapping** — for a real collision
  check, use `shape_a.common(shape_b).Volume`; nonzero means genuine interpenetration, zero means
  clear (even if `distToShape` also read 0).

## References

- [FreeCAD](https://www.freecadweb.org/)
- [Robust MCP Server Docs](https://spkane.github.io/freecad-addon-robust-mcp-server/)
- [MCP Protocol](https://modelcontextprotocol.io/)

<!-- code-review-graph MCP tools -->
## MCP Tools: code-review-graph

**This project has a knowledge graph. Start with the code-review-graph
MCP tools to narrow scope, then read the source.** The graph is cheaper than scanning files and
gives you structural context (callers, dependents, test coverage) that file search cannot.

### When to use graph tools FIRST

- **Exploring code**: `semantic_search_nodes_tool` or `query_graph_tool` instead of Grep
- **Understanding impact**: `get_impact_radius_tool` instead of manually tracing imports
- **Code review**: `detect_changes_tool` + `get_review_context_tool` instead of reading entire files
- **Finding relationships**: `query_graph_tool` with callers_of/callees_of/imports_of/tests_for
- **Architecture questions**: `get_architecture_overview_tool` + `list_communities_tool`

### Verify in the source

- Narrow scope with the graph, then read the source. Do not change code from graph output alone.
- For any non-trivial change, read the implementation and the relevant tests before concluding.
- Verify the exact source when touching behavior, database logic, migrations, retries, fallbacks,
  recovery, or compatibility code.
- When the graph and the source disagree, the source wins. The graph may be stale or may not
  model that relationship.
- An empty graph result can mean "not indexed" or "not statically visible", not "does not exist".

### Key Tools

| Tool | Use when |
| ------ | ---------- |
| `detect_changes_tool` | Reviewing code changes — gives risk-scored analysis |
| `get_review_context_tool` | Need source snippets for review — token-efficient |
| `get_impact_radius_tool` | Understanding blast radius of a change |
| `get_affected_flows_tool` | Finding which execution paths are impacted |
| `query_graph_tool` | Tracing callers, callees, imports, tests, dependencies |
| `semantic_search_nodes_tool` | Finding functions/classes by name or keyword |
| `get_architecture_overview_tool` | Understanding high-level codebase structure |
| `refactor_tool` | Planning renames, finding dead code |

### Workflow

1. The graph auto-updates on file changes (via hooks).
2. Use `detect_changes_tool` for code review.
3. Use `get_affected_flows_tool` to understand impact.
4. Use `query_graph_tool` pattern="tests_for" to check coverage.
<!-- /code-review-graph MCP tools -->

## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

Rules:
- For codebase questions, first run `graphify query "<question>"` when graphify-out/graph.json exists. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).
