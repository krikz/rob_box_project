# Why this patch is disabled

## TL;DR

`hermes-agent-skill-validation.patch` was originally added in 2025 to backport
`force_scope` + scope-hint heuristics + `_validate_skills_for_assignee` into
Hermes Agent before upstream had them. As of `hermes-agent` commit
`180291162` (Sep 2 2026) and the working-tree state observed on 2026-09-09,
upstream has **not** yet merged these features — the patch is still
needed functionally, but it no longer applies cleanly because the
anchors have shifted and reverse-check now fails (i.e. the patch is no
longer applied either).

## Timeline

- **2025**: Patch added to backport `force_scope` / scope-hint / skill validation
  into hermes-agent. The patch was working: install.sh applied it idempotently.
- **Aug 2026**: `crp-schema`, `kanban-cost-attribution`, `auto-decompose-maintenance`
  patches were upstreamed into hermes-agent working tree (uncommitted). Those
  patches now report "patch already applied (reverse-check clean)" in
  install.sh but their forward-check fails — they are functionally equivalent
  to upstream changes but the anchors have drifted.
- **Sep 2 2026** (commit `180291162`): upstream merges `feat(telemetry): opt-in
  shared-metrics exporter (#95278)` with massive kanban_db.py / kanban.py
  changes (1083 insertions, 28 deletions in working tree from `git diff --stat`).
- **Sep 9 2026** (retro `t_8fba04b9`, task `t_9e0760b9`): drift-detect cron
  reports `failure_streak=14` because install.sh aborts with exit 1 on
  `hermes-agent-skill-validation.patch` not applying cleanly. Root cause: anchor
  shift — patch context lines (`@@ -400,6 +400,18 @@` in kanban.py and
  `@@ -84,6 +84,7 @@` in kanban_db.py) no longer match the new upstream code.

## What's in the patch that we still need

The patch adds:

- `force_scope` parameter on `hermes_cli.kanban_db.create_task` (CLI flag
  `--force-scope`) — escape hatch for ADR-0036 §4.1 mis-scope hint.
- `ARCHITECTURAL_KEYWORDS` / `_validate_scope_for_assignee` in
  `hermes_cli.kanban_db` — heuristics for catching implementation-profile +
  TDD-skill mis-scope loops (retros `t_aa585aa7`, `t_e2ae0c29`).
- `_validate_skills_for_assignee` / `_profile_skill_names` — reject per-task
  `skills=[...]` that don't match the assignee profile's installed skills
  (retro `t_6c6c98fb`).
- `--force-scope` CLI flag wiring in `hermes_cli.kanban.build_parser` and
  `_cmd_create`.

Without these, every kanban-create call from a devops/architect profile that
uses a TDD-shaped skill (e.g. test-driven-development on an ADR task) trips
the mis-scope hint and prints a warning to stderr — annoying but not blocking
because the scope-hint is hint-only. The skill-validation guard is more
critical: without it, a worker can request `skills=["devops"]` for an
`assignee=architect` task and the dispatcher will create the card; the worker
then crashes on spawn because `--skills devops` fails to resolve on the
architect profile. We **want this guard**.

## What needs to happen to re-enable this patch

1. Wait for upstream to merge the equivalent features natively. Track:
   - Upstream `hermes_cli/kanban.py` for `--force-scope` argument
   - Upstream `hermes_cli/kanban_db.py` for `_validate_skills_for_assignee`
     + `_profile_skill_names` + `ARCHITECTURAL_KEYWORDS`
2. Once upstream has them, regenerate this patch with:
   ```bash
   bash scripts/agent_flow/agent-flow-regen-vendor-patch.sh \
       scripts/agent_flow/vendor/hermes-agent-skill-validation.patch.DISABLED
   ```
   Then `git mv` it back to `.patch` and verify:
   ```bash
   bash scripts/agent_flow/install.sh  # should print OK patch already applied
   ```
3. If upstream still hasn't merged by the time we need this, the patch will
   have to be **manually** re-anchored against the new upstream code.
   That's a separate devops task — `t_<future>: regenerate skill-validation
   patch against current upstream`.

## Why we disabled instead of regenerated inline

The regenerate helper (`agent-flow-regen-vendor-patch.sh`) was last
updated to handle the **3-file** patch pattern
(`hermes_cli/kanban_db.py` + `hermes_cli/profiles.py` +
`tests/hermes_cli/test_kanban_db.py`). The current patch is **2-file**
(`hermes_cli/kanban.py` + `hermes_cli/kanban_db.py`) — different anchors,
different `+` line extraction. Extending the helper is non-trivial:

- kanban.py: `build_parser` has shifted by ~50 lines and the `--force-scope`
  argument must be inserted at a new anchor (likely before `--skill` or
  `--max-retries`, NOT at the previous `--force-scope` location).
- kanban_db.py: `_profile_skill_names` / `_validate_skills_for_assignee`
  need to be inserted before `create_task` (line ~3350 now, was 3341).
  The create_task body has grown by 200+ lines — the call site
  `_validate_skills_for_assignee(assignee, skills_list or [])` needs to
  land AFTER `skills_list = cleaned` at the new line ~3623+200=~3823.

Doing this safely requires:
- A live hermes-agent working tree state we can verify against
- Python helper to re-extract `+` hunks with the new anchor strategy
- Idempotency test (re-run install.sh twice, both should report "OK already applied")

That's a separate devops task (`t_<future>`), not part of `t_9e0760b9`.

## Acceptance for re-enabling

- [ ] Upstream commit landed that natively adds `force_scope` /
      `_validate_skills_for_assignee` / `_profile_skill_names` /
      `_validate_scope_for_assignee` / `ARCHITECTURAL_KEYWORDS` in
      `hermes_cli/kanban_db.py` AND `--force-scope` arg in
      `hermes_cli/kanban.py`.
- [ ] `git mv ... .patch` (drop `.DISABLED`).
- [ ] `bash scripts/agent_flow/install.sh` reports
      `OK patch already applied (reverse-check clean)` for this patch.
- [ ] `bash scripts/agent_flow/install.sh` exits 0.
- [ ] No `WARN patch failed` line in install.sh stdout for this patch.

## Related

- retro `t_8fba04b9` (issue #1977 stuck-open)
- task `t_9e0760b9` (this fix)
- retro `t_f00676f8` (original vendor-patch rationale)
- retro `t_49c2b63f` (regen helper origin)
- ADR-0036 §4.1 (scope-hint heuristic)
- retro `t_6c6c98fb` (skill-validation regression)
- retro `t_aa585aa7` (mis-scope cron-oversight)
