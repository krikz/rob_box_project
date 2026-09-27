# voice-vr 10 — _on_json_cmd refactor: independent analysis

Task: `t_6f80e3e4` (branch `z-{agent}/2195-voice-vr-10-on-json-cmd-cc-107`,
HEAD `7ef2f98d`, target: decompose `WSSServer._on_json_cmd` into a table
dispatcher, terminal unknown-command error+warning with command/session_id,
explicit set_voice intent, each helper CC≤15).

Author: kanban worker `backend` (delegate-task analysis).
Date: 2026-09-08.

## What the target file looks like now

- `src/rob_box_quest/rob_box_quest/server/ws_server.py` is 2931 lines.
- `_on_json_cmd` spans lines **1608–2411** (804 lines, CC=107). Confirmed
  via full file read.
- Receive loop at **ws_server.py:2555–2563** still calls
  `self._on_json_cmd(ws, session, payload_obj)` directly. The
  `_dispatch_json_cmd` shim from voice-vr 16 (commit `e44b46c17`) was
  **reverted** on this branch — `async def _dispatch_json_cmd` is **not**
  present.
- `_ws_handler` was already refactored via FRAME_HANDLERS table
  (`e44b46c17`), and `_handle_supervisor_command` was already decomposed
  (`ebddc1f4f`, voice-vr 17). The pattern is in-tree.
- `t_a053cf04` (sibling voice-vr 09) added `ErrorCode.UNKNOWN_COMMAND` to
  `session.py` and a terminal `ERROR{UNKNOWN_COMMAND}` fallback in
  `_on_json_cmd` — but that branch is **NOT merged** into t_6f80e3e4.
  The terminal fallback is missing here.

## Command catalog (17 server-dispatched)

Sibling `t_2045029b/src/rob_box_core/rob_box_core/bridge_protocol.py`
already enumerates the 17 commands whose `server_dispatched=True` (lines
204–414). They map 1:1 to the if-chain in `_on_json_cmd`:

| # | cmd | lines | CC risk |
|---|-----|-------|---------|
| 1 | `ping` | 1627-1634 | trivial |
| 2 | `stream_list` | 1635-1645 | trivial |
| 3 | `stream_select` | 1886-1913 | includes topic validation |
| 4 | `teleop_twist` | 1646-1704 | floor-gate + heartbeat relay (~CC 11) |
| 5 | `teleop_heartbeat` | 1705-1741 | floor-gate (~CC 9) |
| 6 | `stop_emergency` | 1742-1750 | trivial |
| 7 | `voice_ptt_start` | 1751-1817 | arbiter + cache + dual-mode (~CC 12) |
| 8 | `voice_ptt_stop` | 1818-1846 | arbiter + cache + dual-mode (~CC 9) |
| 9 | `voice_mode` | 1847-1863 | trivial |
| 10 | `voice_listen_start` / `_stop` | 1864-1885 | one branch |
| 11 | `list_voices` | 2085-2109 | rate-limit + snapshot (~CC 5) |
| 12 | `set_voice` (AV-27 + AV-28 fork) | 2110-2276 | **CC ~20 — hardest** |
| 13 | `voice_pipeline` | 2277-2373 | rate-limit + default-fill + bridge (~CC 12) |
| 14 | `preview_voice` | 2374-2411 | field validation (~CC 7) |
| 15–18 | `supervisor_set_mode`, `_acquire_floor`, `_release_floor`, `_get_state` | 1917-2082 | 4 pre-guards + 3 branches (~CC 14 in aggregate, separable) |

Plus 6 `server_dispatched=False` aliases in the catalog: `avatar_set_mode`,
`avatar_acquire_floor`, `avatar_release_floor`, `set_panel_topic`, `ui_button`,
`admin_logs*`. None need a handler now; the terminal `else` should respond
with `UNKNOWN_COMMAND`. voice-vr 09 plans to add `avatar_*` aliases — out of
scope here.

## Recommended behavior-preserving design

**Mirror the existing `_ws_handler` and `_handle_supervisor_command` pattern.**
Three layers, all module-level except the dispatcher:

```python
# module-level table (ws_server.py ~line 1608, replacing the 800-line chain)
JSON_CMD_HANDLERS: dict[str, Callable[...]] = {
    "ping":                       _handle_cmd_ping,
    "stream_list":                _handle_cmd_stream_list,
    "stream_select":              _handle_cmd_stream_select,
    "teleop_twist":               _handle_cmd_teleop_twist,
    "teleop_heartbeat":           _handle_cmd_teleop_heartbeat,
    "stop_emergency":             _handle_cmd_stop_emergency,
    "voice_ptt_start":            _handle_cmd_voice_ptt_start,
    "voice_ptt_stop":             _handle_cmd_voice_ptt_stop,
    "voice_mode":                 _handle_cmd_voice_mode,
    "voice_listen_start":         _handle_cmd_voice_listen,   # also _stop
    "voice_listen_stop":          _handle_cmd_voice_listen,
    "supervisor_set_mode":        _handle_cmd_supervisor,
    "supervisor_acquire_floor":   _handle_cmd_supervisor,
    "supervisor_release_floor":   _handle_cmd_supervisor,
    "supervisor_get_state":       _handle_cmd_supervisor,
    "list_voices":                _handle_cmd_list_voices,
    "set_voice":                  _handle_cmd_set_voice,       # AV-27/AV-28 fork
    "voice_pipeline":             _handle_cmd_voice_pipeline,
    "preview_voice":              _handle_cmd_preview_voice,
}

# class WSSServer:
async def _on_json_cmd(self, ws, session, payload_obj) -> None:
    """Thin dispatcher. CC≈3 (ebddc1f4f precedent)."""
    cmd = payload_obj.get("cmd")
    handler = JSON_CMD_HANDLERS.get(cmd)
    if handler is None:
        log.warning(
            "quest: unknown JSON_CMD received: cmd=%r session_id=%s (issue #2194)",
            cmd, session.session_id,
        )
        await self._send_error(
            ws, 0, ErrorCode.UNKNOWN_COMMAND, f"unknown JSON_CMD: {cmd!r}",
        )
        return
    try:
        await handler(self, ws, session, payload_obj)
    except Exception as exc:                              # noqa: BLE001
        log.exception("quest: handler %s crashed: %s", handler.__name__, exc)
        await self._send_error(ws, 0, ErrorCode.INTERNAL, f"{cmd}: {exc}")
```

### Why this is behavior-preserving

- Each `cmd` keeps its existing event/error contract. Tests in
  `test_ws_server_voice.py` and `test_ws_server_v2.py` pin exact payload
  shapes; helpers move as units, not rewritten.
- Rate-limit slots (`VOICE_LIST_MIN_INTERVAL_S` / `VOICE_SET_MIN_INTERVAL_S` /
  `VOICE_STYLE_MIN_INTERVAL_S` / `VOICE_PREVIEW_MIN_INTERVAL_S`) and
  `feed_client_alive` calls live on `self`. Handlers take `server` as first
  arg and call `server._voice_rate_limit_check(...)` /
  `server.bridge.feed_client_alive()`.
- `_should_send_floor_held_error` and `_avatar_arbiter` stay `self.X` lookups;
  `teleop_*` helpers retain all gate logic verbatim.

### Module-level helper layout (each CC ≤ 15, matches `_handle_set_mode`)

- `_handle_cmd_ping` — 6 lines.
- `_handle_cmd_stream_list` — 8 lines.
- `_handle_cmd_stream_select` — 18 lines (incl. `_send_error{TOPIC_UNKNOWN}`).
- `_handle_cmd_teleop_twist` — 40 lines; uses `_parse_ts_seq(payload)` helper
  to drop the `ts_ms/seq` parsing duplication with `teleop_heartbeat`
  (3-line helper).
- `_handle_cmd_teleop_heartbeat` — 25 lines.
- `_handle_cmd_stop_emergency` — 5 lines.
- `_handle_cmd_voice_ptt_start` / `_handle_cmd_voice_ptt_stop` — split each,
  ~35 lines.
- `_handle_cmd_voice_mode` — 12 lines.
- `_handle_cmd_voice_listen` — 18 lines (one function; `cmd.endswith("_start")`
  decides active).
- `_handle_cmd_supervisor` — 50 lines for pre-guards + dispatch to
  `_handle_cmd_supervisor_get_state` / `_handle_cmd_supervisor_set_mode` /
  `_handle_cmd_supervisor_floor_op` (CC ≤ 6 each).
- `_handle_cmd_list_voices` — 18 lines.
- **`_handle_cmd_set_voice` (the hardest)** — split into:
  - `_handle_cmd_set_voice_av28` (~40 lines: rate-limit → `_validate_voice_set_payload`
    → branch to `bridge.set_voice_preset` / `bridge.set_voice_language` → ack)
  - `_handle_cmd_set_voice_av27` (~30 lines: rate-limit → `voice_id` validation
    → `bridge.set_voice` → ack/nack)
  - Outer dispatcher picks by `is_av28_request` flag (same predicate as today
    at ws_server.py:2156).
- `_handle_cmd_voice_pipeline` — 50 lines.
- `_handle_cmd_preview_voice` — 25 lines.

### Tests to add (mirror `test_ws_server_supervisor_dispatch.py`)

- `test_json_cmd_handlers_table_covers_dispatched_commands` — assert
  `set(JSON_CMD_HANDLERS.keys()) == set(bridge_protocol.client_dispatched_commands())`.
  Requires bridge_protocol to be merged first; otherwise assert against a
  hardcoded list.
- `test_unknown_json_cmd_returns_unknown_command_error` — assert
  `ERROR{UNKNOWN_COMMAND}` and WARNING with `cmd=... session_id=...`.
- `test_set_voice_dispatches_to_av28_when_preset_is_style_id` (already exists
  as `test_set_voice_routes_preset_and_language_to_bridge`, just retarget
  handler).
- `test_set_voice_dispatches_to_av27_when_preset_is_provider_preset` (already
  exists as `test_set_voice_success_sends_ack`).
- `test_handler_crash_sends_internal_error_and_does_not_drop_session`
  (mirrors issue #2099 regression at test_ws_server.py:645).

## Hidden contract expectations the design MUST preserve

### Per-cmd ordering invariants

- **`teleop_twist` gate-block** (lines 1677–1691): floor-held path calls
  `feed_client_alive()` BUT NOT `publish_quest`; success path calls both.
- **`voice_ptt_start`** (lines 1751–1817):
  `_avatar_arbiter.try_acquire_voice` first, then
  `_voice_cache.update(LISTENING, ...)`, then `publish_voice_*`, then
  `_send_voice_state(listening)`. Reordering breaks FSM observers
  (test_ws_server_voice_floor_edge_cases.py).
- **`voice_ptt_stop`** (lines 1818–1846):
  `_avatar_arbiter.release_voice` first; `publish_voice_*` always (idempotent);
  `voice_state(idle)` only if `was_holder`.
- **`set_voice` rate-limit slots** (lines 2164-2172 vs 2222-2237):
  AV-28 uses `"set_voice_style"` slot; AV-27 uses `"set_voice"` slot — they
  do NOT share. Tests at `test_ws_server_voice.py:502` and `:589` assert
  distinct slots.
- **`voice_pipeline`** (line 2298-2300) shares the `"set_voice_style"` slot —
  do not move it.
- **`stop_emergency`** (lines 1742-1750) skips all floor gates (ADR-0028 §4.4);
  no `feed_client_alive` required (test_ws_server_av19.py).
- **`teleop_heartbeat` gate-block** (lines 1729-1735): still calls
  `feed_client_alive` even when floor is held (different from `teleop_twist`).

### Error-code reuse

`FLOOR_HELD` rate-limit on `teleop_twist` must use
`_should_send_floor_held_error` (line 1680) — don't let the helper duplicate
that throttling.

### `set_voice` predicate `is_av28_request`

The discriminator at ws_server.py:2156-2160 is exact. Moving to a separate
helper loses nothing if the boolean logic is identical. The
`style_without_voice` case (preset ∈ `VOICE_PRESET_IDS` but missing
`voice_id`) is AV-28 with `nack_reason="invalid_voice_preset"` and MUST NOT
trigger `BAD_PAYLOAD{voice_id required}` (the bugfix at
ws_server.py:2151-2162).

### Terminal fallback wording

Log line is
`"quest: unknown JSON_CMD received: cmd=%r session_id=%s (see issue #2194)"`,
error message is `f"unknown JSON_CMD: {cmd!r}"`. The sibling voice-vr 09
(`t_a053cf04`) pinned these exact strings. Match them — divergence creates
merge conflicts in tests when both cards land.

## Risks from the unmerged `bridge_protocol` (sibling `t_2045029b`)

### `ErrorCode.UNKNOWN_COMMAND` collision — HIGH

voice-vr 09 (`t_a053cf04`) adds `UNKNOWN_COMMAND = "UNKNOWN_COMMAND"` to
`session.py:ErrorCode`. t_6f80e3e4 currently doesn't have it. The terminal
fallback you add here will reference `ErrorCode.UNKNOWN_COMMAND`.

- **If both cards merge, fine.**
- **If voice-vr 09 merges first and you rebase, the constant already exists** —
  no action needed, just rebase.
- The bridge_protocol sibling's `ERRORS` tuple at
  `t_2045029b/.../bridge_protocol.py:751-770` does NOT include
  `UNKNOWN_COMMAND`. Coordinate with voice-vr 09 to add it there too.
- Recommendation: import `bridge_protocol.ERRORS` and check
  `"UNKNOWN_COMMAND" in ERRORS` at module load, OR add `UNKNOWN_COMMAND` to
  `session.py:ErrorCode` as a sibling change.

### `client_dispatched_commands()` test coupling — MEDIUM

A clean test is
`assert set(JSON_CMD_HANDLERS) == set(bridge_protocol.client_dispatched_commands())`.
That requires `rob_box_core.bridge_protocol` to be importable from
`rob_box_quest` server tests. `ws_server.py` currently does NOT import
`rob_box_core`; but `quest_node.py:58` and tests already do
(`from rob_box_core.avatar_command import ...`). New dep is acceptable but
`bridge_protocol` must be merged first; otherwise fall back to a hardcoded
list.

### `voice_pipeline` whitelist drift — MEDIUM

bridge_protocol re-exports `VOICE_PRESET_IDS` (line 825), `VOICE_LANGUAGES`
(line 839), `VOICE_PIPELINE_DEFAULT_LANGUAGE` (line 846). If voice-vr 10
changes those constants and bridge_protocol also ships new whitelists, the
`set_voice` rate-limit helpers (`_validate_voice_set_payload`,
`_validate_voice_pipeline_payload`) should import from `bridge_protocol` for
consistency. The local constants at ws_server.py:49-167 must be kept
identical to the bridge_protocol ones — drift is the failure mode.

- `test_voice_pipeline_payload_matches_supervisor_contract` (line 1185-1262)
  checks `SERVER_PRESETS` and `SERVER_LANGUAGES` against client whitelists
  and YAML. Will keep passing only if values stay identical.
- Recommendation: defer the import-swap to a follow-up; this card should
  keep local constants and just decompose dispatch.

### Sibling PRs #2209 / #2212 — LOW-MEDIUM

These aren't in this repo's local clones — only mentioned in the task
context. They refactor adjacent methods on separate, unmerged branches.
Risk: rebase against `develop` will pick up 4 new commits; t_6f80e3e4 is
`behind 'origin/develop' by 4 commits`. None of those 4 touch
`_on_json_cmd`, so the decomposition is safe. Add a rebase-test step
before pushing.

### `voice-vr 09` (`t_a053cf04`) merge-order race — HIGH

If voice-vr 09 merges first (it adds `avatar_*` aliases + `UNKNOWN_COMMAND`),
then voice-vr 10's `JSON_CMD_HANDLERS` must include
`avatar_set_mode` / `avatar_acquire_floor` / `avatar_release_floor` /
`avatar_get_state` — else those become the terminal `UNKNOWN_COMMAND` case.

- **Recommendation (a)**: extend `JSON_CMD_HANDLERS` to cover `avatar_*`
  aliases immediately and route them to `_handle_cmd_supervisor` with
  payload `kind → floor` normalization (matches voice-vr 09's plan).
- **Recommendation (b)**: wait for voice-vr 09 to merge first.
- The bridge_protocol catalog at `t_2045029b/.../bridge_protocol.py:339-413`
  already lists `server_dispatched=False` for those names — flipping those
  to `True` requires both this card AND voice-vr 09's alias logic in
  lockstep.

### `_ws_handler` JSON_CMD dispatch path — LOW

Currently `_ws_handler` (line 2555-2563) calls `self._on_json_cmd(...)`
directly. After the refactor `_on_json_cmd` becomes a 6-line dispatcher —
no `_dispatch_json_cmd` shim needed (unlike `_ws_handler`). Don't introduce
one; keeps CC budget tight.

### `bridge_protocol` import-time cost — LOW

bridge_protocol's docstring (line 50-51) says "Импорт стоит 0 времени и
безопасен в conftest" — verified: pure dataclass/enum declarations, no
ROS/msgpack/aiohttp. Safe to import unconditionally in ws_server.py.

## Concrete file/line recommendations

- New table + dispatcher at ws_server.py lines **85-87** (module-level,
  before `def _pack_msgpack`) and **1608-1640** (replacing the head of
  `_on_json_cmd`).
- Module-level `_handle_cmd_*` helpers at the bottom of ws_server.py
  (after `_handle_supervisor_command`, around line **2900+**).
- Keep existing helpers (`_voice_rate_limit_check`,
  `_should_send_floor_held_error`, `_validate_voice_set_payload`,
  `_validate_voice_pipeline_payload`) as instance methods; module handlers
  call them via `server._voice_rate_limit_check(...)`.
- New test file
  `src/rob_box_quest/test/unit/server/test_ws_server_json_cmd_dispatch.py`
  mirroring `test_ws_server_supervisor_dispatch.py` (created by voice-vr 17
  / `ebddc1f4f`).
- Do **NOT** modify `protocol/topics.py`, `streams/registry.py`, or
  `session.py:ErrorCode` in this card (bridge_protocol and voice-vr 09
  own those).

## Summary of findings

The refactor is mechanically straightforward and the precedent pattern
(`_handle_supervisor_command` decomposition) is already merged. The main
risks are merge-order with `t_a053cf04` (voice-vr 09, `UNKNOWN_COMMAND` +
`avatar_*` aliases) and the unmerged `bridge_protocol` import surface in
the test layer. None block implementation; both warrant a comment in the
final PR description so the reviewer knows about them.
