# MCP Tools Audit — BUG-14 (issue #813)

Audit of the 21+ MCP tool definitions to eliminate overlap and reduce the
per-request tool-definition overhead (~3150 tokens before this change).

## Measurement

Script: `src/rob_box_mcp_tools/scripts/audit_mcp_tools.py`

```bash
PYTHONPATH=src/rob_box_mcp_tools python3 src/rob_box_mcp_tools/scripts/audit_mcp_tools.py
```

The script instantiates every tool the MCPServer registers, serializes them in
the OpenAI Tool Calls format (exactly what gets published on `/mcp/tools` and
sent to the LLM on every request), and counts tokens with `tiktoken`.

| Metric | Before (develop) | After (this branch) | Δ |
|---|---|---|---|
| Tools | 43 | 43 | — |
| Full tool JSON | 10 916 tok | 8 172 tok | −25% |
| Descriptions only (LLM-facing) | 3 883 tok | 2 122 tok | −45% |

Acceptance: descriptions overhead < 2500 tokens → **MET** (2122).

## Overlap decision (play_sound / play_animation)

`play_sound_and_animation` does **not** exist in the codebase — the reported
overlap was between `play_sound` and `play_animation`. The two tools serve
**different channels** and are now explicitly disambiguated in their
descriptions:

- **play_sound** — audio channel: short sound effects (< 5 sec) for emotional
  accompaniment of speech. Full list via `get_sound_info()`. Music/loops are
  explicitly routed to `execute_music_code`.
- **play_animation** — visual channel: LED animation on the 381-LED matrix.
- When both are wanted during a phrase, `speak_text` already accepts an
  `animation` parameter; sounds can be played alongside via `play_sound`.

No merging needed — they are orthogonal (audio vs LED). Ambiguity resolved by
descriptions that state the channel and cross-reference the alternative.

## Removal candidates (< 5 calls/week)

Usage telemetry per tool is not wired in this repo, so "calls per week" cannot
be measured directly. Candidates below are flagged by structural redundancy
(prefix-overlap groups from the audit script); they are **not** removed in
this change because they map to distinct ROS topics/actions and removing them
would break existing voice commands:

- `get_current_time` (trivial, no ROS dependency) — candidate for removal or
  fold into `get_robot_status` if call telemetry confirms < 5/week.
- `estimate_tts_duration` (planning-only helper, overlaps `speak_text` timing).
- `continue_mapping` / `optimize_map` (mapping workflow steps rarely invoked
  standalone; usually reached via `start_mapping` / `finish_mapping`).

Re-evaluate after call-logging is available.

## Ambiguity check

The only historical "wrong tool" ambiguity was sound-vs-animation; the updated
descriptions state the media channel explicitly, and `speak_text` remains the
primary speech tool (others reference it). No remaining ambiguous pairs were
found by the prefix-overlap scan (`get_*`, `set_*`, `list_*`, `load_*` groups
are distinct domains: status vs time vs pose vs sound-info; volume vs pitch vs
speed vs vibe vs dj-mode; waypoints vs tracks; map vs track).
