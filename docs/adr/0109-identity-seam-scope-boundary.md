# ADR-0109 — Identity seam: scope of "no `speaker_id[:8]`" contract (issue #2440)

> Previously numbered ADR-0097, commit `c1682495` (PR #2542). Renumbered 2026-09-15: see issue #2582 — sibling `0097-speaker-register-upsert-vs-update.md` merged earlier (`1ef642c0`, PR #2492) and kept 0097.

- **Status**: Accepted
- **Date**: 2026-09-15
- **Deciders**: architect (review #2471)
- **Source of truth for identity**: commit `8cb84bc9` (issue #2440, PR #2440)
- **Note**: ADR-0097 (renumbered to ADR-0109, issue #2582) was chosen at the time to avoid name clash with existing wip ADR-0096
  branches (`984542ab3 wip(adr): ADR-0096 «Встреча»`,
  `658e8daec wip(adr): ADR-0096 — MiniMax STT empty-text semantics`,
  `8439b0efe wip(adr): ADR-0096 vision_hailo launch-file decoupling`).
  Each ADR-0096 branch is a distinct topic (meeting, STT, vision); none
  cover this scope-boundary question.

## Context

Issue #2471 (review of `src/rob_box_voice`, window 2026-09-14) claims that
`DialogueNode._emit_backlog_diag_log` (commit `fefe3ba`, issue #2392) violates
the contract introduced by issue #2440 — "no `speaker_id[:8]` truncation" — and
asks to drop the `[:8]` slice.

Author of #2471 explicitly grounds the complaint in `8cb84bc9`:

```diff
-if sp_id:
-    lines.append(f"    <speaker_id>{sp_id[:8]}</speaker_id>")
+if sp_id:
+    lines.append(f"    <speaker_id>{sp_id}</speaker_id>")
```

with the comment: "Issue #2440 — полный id, без усечения до 8 символов".

## Finding

Raw evidence:

1. The change in `8cb84bc9` lives at `dialogue_node.py:3158`, inside
   `_build_system_context_xml` — the function that produces the
   `<system_context>` block sent to the LLM and surfaced to MCP tools.
2. The diag log in question lives at `dialogue_node.py:6430`, inside the
   helper `_emit_backlog_diag_log` (commit `fefe3ba`), which is purely
   server-side `self.get_logger().info(...)`. It is **not** part of any
   payload sent to the LLM or to MCP tools.
3. The same `8cb84bc9` commit explicitly says: "тот же id уходит в лог
   целиком" (the same id is logged in full elsewhere) — this is an
   observation about existing log lines, **not** an obligation to forbid
   any future `[:8]` in any new log line.
4. Repo-wide grep `speaker_id.{0,40}\[:8\]` under `src/rob_box_voice/`
   returns 8 hits total: 1 in `dialogue_node.py:6430` (this issue),
   7 pre-existing human-readable shortenings in `speaker_id_node.py`
   and `speaker_embeddings.py` ("Renamed → 'X' (id=abc12345)").
   None of the 7 are in the LLM/MCP context path.

The author's secondary claim — "the e2e flow breaks because it compares
truncated prefix against the full UUID in `voice_memory`" — is also
incorrect: e2e flow reads the full id from other log lines / from the
acquaintance object (see `DialogueNode._current_acquaintance()` introduced
in `8cb84bc9`, which uses `sid = str(sp.get('speaker_id') or "").strip()`
without truncation). The diag line is decorative for human triage.

## Decision

1. **Scope of "no `[:8]`" contract from issue #2440 is the
   `<system_context>` block** (`_build_system_context_xml`,
   `dialogue_node.py:3158`). It is **not** a repo-wide rule that forbids
   any truncation in any log statement.
2. **Truncation in human-readable log lines is allowed**, but only when
   the same log line also exposes the full id under a separate field
   (e.g. `speaker_id_full=…`). This keeps grep-friendly short prefix
   for human triage while preserving round-trippability to the database.
3. **Apply the fix in `_emit_backlog_diag_log`**: add `speaker_id_full`
   alongside `speaker_id_short` (which keeps the existing 8-char prefix
   for visual brevity). The short prefix is no longer the only
   representation, so any future e2e/grep script that wants the full id
   has it.

## Concrete change (proposed, single-file)

`src/rob_box_voice/rob_box_voice/dialogue_node.py:6425-6434`:

```python
self.get_logger().info(
    f"robot_log(step=backlog_diag): raw_current_speaker="
    f"is_known={sp.get('is_known')!r} "
    f"name={sp.get('name')!r} "
    f"confidence={sp.get('confidence')!r} "
    f"speaker_id_short={str(sp.get('speaker_id') or '')[:8]!r} "   # human-readable
    f"speaker_id_full={sp.get('speaker_id')!r} "                    # grep/DB-join
    f"tag_in={speaker_tag!r} "
    f"sanitized_name={sp_name!r} "
    f"text={text[:60]!r}"
)
```

Severity of issue #2471: **low** (as the author flagged). Not a
contract violation; the framing is incorrect.

## Out of scope

- Renaming `_emit_backlog_diag_log` or its call site — the helper is
  already extracted (ADR-0021 R1 compliance) and review n302/n303/n308
  triaging depends on the exact `step=backlog_diag` marker.
- Removing the 7 pre-existing `[:8]` shortenings in
  `speaker_id_node.py` / `speaker_embeddings.py` — they are
  intentionally compact for operator triage and never enter the LLM
  payload. They follow the same rule as above (short prefix is
  decorative; full id lives in adjacent log lines / DB).
- Writing a separate ADR for the identity seam itself — `8cb84bc9`
  is the canonical record (commit message + PR #2440 description).
  A future ADR can promote it to a full document if scope grows.

## Why this is an ADR and not just a comment

The author of #2471 mistook a localised log-truncation choice for a
contract violation and misattributed it to "ADR-0095" (which is actually
PR pollution detection, see `docs/adr/0095-pr-pollution-detection.md`).
We need a written record that:

- pins the **scope** of the no-`[:8]` rule to the `<system_context>`
  path so future reviewers don't reopen this debate every time they
  see `[:8]` in any log statement;
- avoids re-litigating the 7 pre-existing shortenings under
  `speaker_id_node.py` / `speaker_embeddings.py`;
- reminds contributors that "full id is already logged elsewhere"
  was the original justification, not a per-call-site requirement.

## References

- commit `8cb84bc9` (issue #2440 — единый шов «Знакомый»)
- commit `fefe3bad6` (issue #2392 — diag robot_log backlog path; 7-char
  short hash `fefe3ba` is what the issue body cites, full is `fefe3bad66`)
- `dialogue_node.py:3158` (`_build_system_context_xml`, full-id rule)
- `dialogue_node.py:6430` (`_emit_backlog_diag_log`, this fix)
- `dialogue_node.py:1984-1994` (`_current_acquaintance`, proves full id
  is propagated to operator-facing path)
- issue #2471 (this review)
- this ADR renumbered 0096→0097 because three wip branches
  (`#2454`, STT #2470, vision_hailo #2498) already used `0096-*.md`
  on distinct topics; renumber prevents doc-filename collision in
  `git mv`/PR review.