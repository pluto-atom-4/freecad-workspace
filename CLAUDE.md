# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

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

**balancing-robot-controller** (separate repo, NOT part of this workspace) — C++/PlatformIO
firmware for the real robot (XIAO ESP32-C3, MPU-6050, Dynamixel wheels). Local checkout:
`~/Documents/platform-io-workspace/balancing-robot-controller`; GitHub:
[pluto-atom-4/balancing-robot-controller](https://github.com/pluto-atom-4/balancing-robot-controller).
Keep the split: **Python here, C++ there** — don't add C++ firmware sources to this repo.
Python is the source of truth; the C++ repo commits *generated* gains header + golden parity
vectors exported from here by `inverted-pendulum-project/07_Simulation/hal/export_cpp.py`
(#353; a manual LOCAL step: writes the two generated files into a local C++ checkout, no
git, no network, no cross-repo CI), so edit the Python generators (`gen_lqr_header.py` #348,
`gen_parity_vectors.py` #347; `hal/parity_vectors.json` is generated, never hand-edited),
never the generated C++ copies. Changes to the HAL contract (units, pitch sign,
`THETA_REF`, dt semantics) must bump `HAL_CONTRACT_VERSION` (#339) and be mirrored in the C++
repo's `lib/hal_iface`. Tracking: Stage F parent #338; C++ umbrella
balancing-robot-controller#22. Target board is ESP32-C3 only (S3 is a future enhancement).

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

# Stage F HAL, vectors and C++ exporter (inverted-pendulum-project/07_Simulation/hal)
mamba run -n pendulum-tools python3 -m pytest -q inverted-pendulum-project/07_Simulation/hal  # HAL tests
mamba run -n pendulum-tools python3 inverted-pendulum-project/07_Simulation/hal/gen_parity_vectors.py --check  # Python-only drift check; C++ copies NOT checked
mamba run -n pendulum-tools python3 inverted-pendulum-project/07_Simulation/hal/export_cpp.py --cpp-repo ~/Documents/platform-io-workspace/balancing-robot-controller --check  # byte-compare the C++ copies; omit --cpp-repo for a Python-only check
```

In `hal/` code import `from hal.hal import ...` with `07_Simulation` on `sys.path` (`sys.path.insert(0, str(Path(__file__).resolve().parents[1]))`); never put `hal/` itself on `sys.path` (the module `hal/hal.py` would shadow the package).

**`freecadcmd` 1.1.3 in this environment does not set `__name__ == "__main__"` for a plain
positional or `--python` script argument** — a script's `if __name__ == "__main__":` guard
silently never fires, exits 0 having done nothing (verified empirically, not documented
upstream). **`-c` is `--console` (a boolean flag, not `python -c CODE`)** — passing
`-c "exec(open('script.py').read())"` silently swallows the code string as an ignored
positional file argument and runs nothing (verified empirically 2026-09-10: no script output,
not even a bare `print()`, while exit code stays 0). Working invocation: pipe the code to
`-c`'s stdin REPL instead, where `__name__` is `"__main__"`:
```
echo "exec(open('script.py').read())" | "$FREECAD_BIN" -c
```

**`runpy.run_path('script.py', run_name='__main__')` (an alternative workaround to the above
when a script's own top-level code — not just its `__main__` guard — needs to run under
`freecadcmd`) sets `__file__` to whatever path string is passed in, unresolved** — pass a bare
relative filename and `Path(__file__).parent` resolves to `.`, silently breaking any
`script_dir.parent`-style sibling-directory lookup (observed: `03_link_servo_to_assembly.py`'s
`resolve_mechanical_dir()` wrote broken relative mesh paths into `servo_link_config.json`
instead of the correct absolute path). Always pass an absolute path to `run_path()`.

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

## Troubleshooting

**MCP client can't connect to FreeCAD:** verify FreeCAD's MCP bridge is running (console shows
"MCP Bridge started!"), check port (9875 XML-RPC / 9876 socket), `FREECAD_SOCKET_HOST` matches.

**freecad-mcp env broken:**
`mamba env remove -n freecad-mcp -y && mamba env create -n freecad-mcp -f freecad-mcp-server/mamba-envs.lock.yml`
(recreating from the unpinned recipe instead: keep the `mcp<2` pin in `pip_packages`.)

**pendulum-tools env broken:**
`mamba env remove -n pendulum-tools -y && mamba env create -n pendulum-tools -f inverted-pendulum-project/mamba-envs.lock.yml`

## FreeCAD Live Bridge

Known limitations of the live `mcp__freecad__*` bridge live in `inverted-pendulum-project/CLAUDE.md`

## GitHub Issue Workflow

Use the `investigate-plan-build` skill (`.claude/skills/investigate-plan-build/SKILL.md`, invoke as
`/investigate-plan-build`). Repo rules: the PAT can't merge PRs — a human always merges manually;
verify with `mcp__github__pull_request_read` (method `get`) before treating a PR as merged.

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
