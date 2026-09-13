---
name: investigate-plan-build
description: End-to-end GitHub issue workflow — file an issue, dispatch a read-only Architect-style agent to investigate and produce a plan plus genuine human decision-forks, surface those decisions via AskUserQuestion, dispatch a Builder-style agent to implement on a branch and open a PR, optionally run a capped iterative reviewer/Builder review cycle, verify merges before trusting them, regenerate downstream artifacts after a confirmed merge, and re-loop the whole cycle if a human's follow-up inspection finds the problem persists or surfaces something new. Use when a flaw/task has just been identified (human-reported or surfaced by prior investigation) and needs to go from observation to a merged, verified fix without re-deriving this process each time. Trigger phrases: "file this as an issue and work it", "run the issue workflow", "investigate and build a fix for this", "do the full issue-to-PR cycle".
---

# Investigate → Plan → Build (issue-to-PR workflow)

Codifies a repeatable cycle for turning an observed flaw or a lead from prior
investigation into a merged, verified fix, using subagents plus GitHub
issue/PR tooling. Works in any repo:
- If the repo has local `Architect`/`Builder` agents configured (e.g. in
  `.claude/agents/`), use those.
- Otherwise, fall back to whatever generic architect-style planning agent
  and generic implementation/builder agent are available in this
  environment (e.g. `Plan`/`Architect` and `Builder`/`general-purpose`), and
  note the substitution to the user.
- If a project's own CLAUDE.md (or equivalent guidance file) documents a
  different agent team or naming for this role, prefer that project's
  convention over the generic fallback.
- Similarly, the optional review step (step 5) uses whatever review-style
  agent is available (e.g. a `caveman:cavecrew-reviewer` plugin, a local
  `Reviewer` agent, or a generic reviewer) — substitute freely, same rule.

## Arguments

Invoked with:
- **issue description / observation** (required) — what was found broken,
  invalid, or worth tracking, in the invoker's own words or from a prior
  investigation's findings.
- **parent issue number** (optional) — if this is part of a larger tracked
  effort, the umbrella issue to file this as a sub-issue of.

If either is missing, ask the user before proceeding rather than guessing.

## Workflow

### 1. Create the GitHub issue

Use `mcp__github__issue_write` with a **short** body: root cause/observation,
scope, and why it matters — not a full essay. If a parent issue number was
given, file this as a sub-issue of it (`parent_issue_number` param on
`issue_write`, or `mcp__github__sub_issue_write` to link it under the parent
if the write tool doesn't accept that param directly). If this issue relates
to other open issues, cross-link them now in the body/a comment (e.g. "Part
of #N", "Blocks #N", "Related to #N") — keep a running index rather than
letting related issues float unlinked.

### 2. Dispatch an Architect-style agent to investigate and plan

Dispatch a read-only planning agent (no Bash, no write access) with the
issue number/description. It must:
- Read the actual source files/data — never guess or extrapolate from
  filenames alone.
- Produce a report with four **explicitly separated** sections:
  1. **Findings** — root-cause evidence with `file:line` references and real
     numbers (not "seems slow", but the actual measured/observed value).
  2. **Implementation plan** — concrete, ordered steps a builder agent can
     follow without re-deriving design decisions.
  3. **Decision forks** — a short list of ONLY genuine human-decision
     points, each phrased as an explicit question with the agent's own
     recommendation. Do not manufacture busywork questions when something is
     already unambiguous from the investigation — an empty list here is a
     valid and good outcome.
  4. **Risks/gotchas for the builder** — edge cases, known footguns in this
     codebase (check the project's CLAUDE.md / known-limitations notes),
     anything that looks obvious but isn't.

### 3. Surface decisions to the user

If the planning agent returned any decision forks, relay them via
`AskUserQuestion`. The tool rejects more than 4 questions per call — batch
them (multiple calls) if there are more than 4. For every question, list the
recommended option first and suffix its label with "(Recommended)". If there
were zero genuine decision forks, skip this step entirely and proceed with
the plan as-is.

### 4. Dispatch a Builder-style agent to implement

Dispatch a full-implementation agent with the plan and the locked-in
decisions from step 3. It must:
- Work on a new branch off `main` (`fix/issue-N-short-slug` or
  `feat/issue-N-short-slug`).
- Implement **exactly** per the plan and decisions — no scope creep, no
  silently overriding a locked-in decision.
- Write or update tests, and verify the fix actually addresses the root
  cause identified in step 2 (not just "it compiles" / "tests pass by
  coincidence").
- Run the full test suite.
- Open a PR against `main` with "Closes #N" in the body.
- End the commit message and the PR body with the **current session's own**
  attribution footer — pull the exact lines from this conversation's active
  system reminder about attribution (never hardcode a stale example from a
  past session):
  - Commit message ends with the `Co-Authored-By:` and `Claude-Session:`
    lines from that reminder.
  - PR body ends with the "🤖 Generated with [Claude Code]" line, a blank
    line, then the session URL — exactly as that reminder specifies.

### 5. Optional: iterative review cycle

When the user asks for it, or for a higher-stakes change, run a capped
review loop:
1. Dispatch a reviewer-style agent to review the PR's diff against `main`.
2. If it finds blocking issues, dispatch a builder-style agent to fix
   exactly those findings (same attribution rules as step 4).
3. Repeat, up to a small fixed cap (default: 4 rounds) — **or stop the
   moment a round comes back clean** ("no issues" / zero blocking
   findings). Never force additional rounds once it's already clean.

### 6. Verify merge before trusting it

Many bot/PAT credentials cannot merge PRs (`403 Resource not accessible`) —
if that's the case here, a human merges manually. **Never** treat a PR as
merged because the human said so verbally. Always call
`mcp__github__pull_request_read` (method `get`) and check its actual state
before proceeding to any work that depends on the merge having happened. A
verbal "I merged it" that turns out false is a real, previously-observed
failure mode — always re-verify regardless of who or what merges PRs in this
repo.

### 7. Regenerate downstream artifacts (post-merge)

Once the merge is confirmed, if the change affects generated downstream
artifacts (e.g. a regenerated file feeding a build, visualization, or
simulation pipeline), dispatch a builder-style agent to:
- Pull latest `main`.
- Regenerate the affected artifacts.
- Relaunch any relevant inspection/verification tooling in the background
  for the human to look at.

If that inspection is visual (a GUI, a rendered scene, a screenshot), the
agent must **never** claim to have visually verified correctness itself — it
cannot see GUI windows. It only confirms mechanical launch success (no
crash) and explicitly hands the actual visual judgment to the human.

### 8. Re-loop on fresh findings

If the human's fresh inspection shows the original issue persists, or
surfaces a **new** issue, repeat this entire cycle (steps 1-7 above) for the
new finding. Keep the original umbrella/tracking issue open until every
discovered sub-issue actually resolves the observed problem — closing the
umbrella early because a sub-fix merged is a false completion signal.

### 9. Correct mistakes explicitly, never silently

Keep issues cross-linked as this runs (step 1's index). If a fix or a
"not a bug" close later turns out wrong or incomplete, don't silently
reopen/reclose — post an explicit correction comment explaining what the
earlier assessment missed, then reopen/reclose as warranted.

## Quick checklist

- [ ] Issue filed (short body, linked to parent/related issues if any)
- [ ] Planning agent: findings (file:line) / plan / decision-forks / risks,
      kept separate
- [ ] Decision forks (if any) put to the user via AskUserQuestion, ≤4 per
      call, recommended option first and labeled
- [ ] Builder agent: new branch, tests updated, full suite run, root cause
      actually addressed, PR opened with "Closes #N"
- [ ] Commit + PR carry this session's real attribution footer (not a
      stale example)
- [ ] Optional capped review/fix loop, stopped early once clean
- [ ] Merge verified via `pull_request_read` (method `get`) — never trusted
      on say-so
- [ ] Downstream artifacts regenerated + inspection tooling relaunched
      (agent claims launch success only, never visual correctness, when the
      inspection is visual)
- [ ] Persisting/new findings re-loop from step 1; umbrella issue stays open
      until truly resolved
- [ ] Any reversed assessment gets an explicit correction comment
