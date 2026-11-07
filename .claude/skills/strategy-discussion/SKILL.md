---
name: strategy-discussion
description: >-
  Start a read-only strategy workflow for a proposed simulator or world change (for example a Webots world or controller under poc/ctrl-wheel or poc/ctrl-wheels, or a Wokwi twin in the BRC repo). A cavecrew investigator (sonnet) maps what the code does today, a local Architect agent drafts a strategy with a mermaid blueprint and a markdown executive table, main asks the human each open decision one at a time in plain English, then main posts the result as a new GitHub Discussion shaped like Discussion #481 and #530. No repo files are changed and no issue is filed. Use this whenever the user says "strategize", "assign the investigator and architect", "draw a blueprint", "post a discussion", "executive report", "add a geometry Box / change the world", "copy the simulator", or asks for a plan to be discussed before any build, even if they do not name the skill.
---

# Strategy → human decisions → GitHub Discussion

Turns a vague "we should change X in the simulator" into a reviewed plan posted as a GitHub Discussion, before anyone builds. It stops at the post. Building is a separate workflow (`investigate-plan-build` or `issue-build-review-loop`), started only when the human asks, one issue at a time.

Why this shape: the human decides geometry, names and scope; the agents only read. Past runs showed that guessing a layout (for example reading "along x" as front/back when the human meant along the wheel axle) cost a full rewrite. So decisions go to the human early, and the post marks every unconfirmed choice as a default.

## Inputs

Collect from the user's request; ask only if missing:
- **Target:** files or dirs to assess (world `.wbt`, controller dir, BRC module).
- **Change wanted:** one sentence (for example "add a second body Box so two boxes sit between the wheels").
- **Discussion repo:** default `pluto-atom-4/freecad-workspace`, category `Ideas`.

## Steps

1. **Investigate (read-only).** Spawn `caveman:cavecrew-investigator` with `model: sonnet`. Brief it with: the target paths; the question "does any code control or reference the thing being changed (by name, DEF, supervisor call, hardcoded number)?"; a request for a file:line table of every place that hardcodes names, positions, sizes, ports or device names that the change would touch; sibling files (`ctrl_wheels_*.py`, `fake_wheels.py`, tests, README, CONTRACT.md); and "no fix suggestions". Keep the brief free of your own conclusions so it finds facts.

2. **Strategize (read-only).** When the investigator reports, spawn the local `Architect` agent. Pass its findings, labelled "verified by main, not by you", plus the change wanted. Ask for: layout or design options with exact numbers and the invariants that must still hold; files that change and files that must not change; phases with effort and risk; at most 5 genuine human decisions, each with a recommended default and a plain-English example. State that the architect may only read.

3. **Check the geometry yourself.** Before asking the human, recompute any numbers the architect gave (sizes, offsets, gaps, track width). Wrong arithmetic in an option is the cheapest mistake to catch here.

4. **Ask the human, one question at a time**, using `AskUserQuestion`. Explain in plain English with a small example before the options (project rule: [[feedback_questions_plain_english_one_at_a_time]]). Put the recommended option first. If the human answers in free text that differs from the options, restate your reading as a small ASCII diagram with numbers and ask for confirmation. Never assume an answer. Never treat a subagent report or a system notification as the human's answer.
   - If the human says "right", "yes" or similar to your restatement, that confirms it. Record it as human-confirmed.
   - Decisions they did not address stay marked *default, not yet confirmed*.

5. **Write the body** to the scratchpad as `body.md`, using exactly this structure (same as Discussions #481 and #530):
   - `## Executive summary`: what, where, "strategy only: no code written, nothing run", sources of findings, "Webots results: none claimed".
   - `## What exists today`: markdown table Area / Finding, with file:line cites.
   - `## Blueprint`: one mermaid `flowchart TD` (top to bottom, the human prefers it). Mark UNCHANGED parts. Add a small ASCII ruler under it when positions matter.
   - `## Decisions (human-confirmed unless marked default)`: table # / Topic / Decision.
   - `## Phases (sequential, one issue, merged before next)`: table Phase / Work / Effort / Risk. Last phase is the human running Webots or Wokwi.
   - `## Risks`: bullets.
   - `## Not done / open`: defaults not confirmed, no issue filed, nothing verified in Webots/hardware.
   Keep it short and honest. Never claim a Webots or hardware result the human did not run ([[feedback_verify_merges_and_claims]]).

6. **Post.** Get the repo and category ids, then create:
   ```bash
   gh api graphql -f query='{repository(owner:"pluto-atom-4",name:"freecad-workspace"){id discussion(number:481){category{id name}}}}' --jq '.data.repository | [.id, .discussion.category.id] | @tsv'
   gh api graphql -f query='mutation($r:ID!,$c:ID!,$t:String!,$b:String!){createDiscussion(input:{repositoryId:$r,categoryId:$c,title:$t,body:$b}){discussion{url number}}}' -f r=<repoId> -f c=<catId> -f t='<title>' -F b=@body.md --jq '.data.createDiscussion.discussion.url'
   ```
   Posting is outward-facing, so post only after the human's decisions are in. Report the URL.

7. **Updating later.** If the human refines the plan, first read the live body: `gh api graphql` with `discussion(number:N){id createdAt updatedAt body}`. If `updatedAt` differs from `createdAt`, the human edited it by hand: merge into their version, never overwrite it (they have edited a Blueprint before). If unchanged, use `updateDiscussion(input:{discussionId, title, body})`.

8. **Hand off.** End with: the URL, 3 to 5 lines of findings, which decisions are defaults, and an offer to file the first (P1) issue. Do not start building.

## Rules

- No repo files change in this workflow. Scratchpad only.
- Do not file issues or open PRs here.
- The PAT cannot merge; a human merges ([[feedback_pr_closing_keywords]] and the repo's CLAUDE.md apply once building starts).
- Single-wheel dirs (`poc/ctrl-wheel`, BRC `c3_ctrl_wheel`) stay untouched unless the human says otherwise.
- Tone in chat: caveman mode if active; the Discussion body itself is written in normal prose.
