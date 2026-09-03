# TTS picker — ROS path: quest_node → TTS provider

Issue: #1919 (AV-27) · Card: `t_5b9d5d0c` · ADR parent: ADR-0027 / ADR-0028
Owner: architect (recon only — no production code)

## TL;DR — decision

**Mechanism for `list_voices`:** direct in-process call from `tts_node` to the
**built-in static catalogue** at
`src/rob_box_voice/rob_box_voice/tts_voice_registry.py:40-67`
(`PROVIDER_VOICES` at `:40` / `DEFAULT_VOICES` at `:62`). tts_node exposes the answer to the
ROS graph as a new **latched topic** `/voice/tts/voices` published with
`DurabilityPolicy.TRANSIENT_LOCAL` + `depth=1` (one-shot publish on
startup + on every `set_provider` change). quest_node subscribes to that
topic and caches; no new ROS service, no synchronous RPC.

**Mechanism for `set_voice`:** publish a JSON request to a new topic
`/avatar/set_voice` and let the **supervisor** apply it to `tts_node` via a
**second SetParameters client** on `/tts_node/set_parameters` (mirrors the
existing `_set_dialogue_param` at
`src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py:414-451`,
which calls `/dialogue_node/set_parameters` at line 424). The voice parameter
(`yandex_voice` / `minimax_voice` / `silero_speaker`) lives on `tts_node`
(`tts_node.py:677,691,716` and `config/tts_node.yaml:25,62,77`), not on
`dialogue_node`, so we add `_set_tts_voice_param` next to
`_set_dialogue_param`. quest_node never touches tts_node parameters
directly (ADR-0028 S5, S12).

**`preview_voice`** is a separate concern (synthesize + stream back) and is
out of scope for this recon; documented here for context only.

## Why not a ROS service?

We considered three options. Trade-offs:

| Option | Cost | Benefit | Verdict |
|---|---|---|---|
| **ROS service** (`/voice/tts/list_voices` srv) | requires new `rob_box_msgs` IDL; synchronous RPC; quest_node must block; failure → timeout chain through aiohttp | request/response contract; "fresh" on demand | ❌ rejected — adds a new IDL surface and synchronous coupling for a result that changes at most once per provider switch. |
| **Topic** (latched `/voice/tts/voices`) | new publisher/subscriber wiring; quest_node must remember to (re)read on cache miss | cheap; works across nodes; survives quest_node restart (latched QoS); no IDL needed | ✅ chosen. |
| **Direct cross-package call** (quest_node imports `rob_box_voice.tts_voice_registry`) | works (registry is pure Python, already shared with `rob_box_mcp_tools`) but breaks ADR-0028 layering (quest_node bypasses supervisor) | zero ROS plumbing | ❌ rejected for `set_voice` (must go via supervisor); accepted as **internal-to-tts_node** source only. |

The chosen topic pattern uses **transient-local durability** so quest_node
gets the last-published payload on (re)connect. Note this is *not* the same
QoS as `/voice/tts/state` (default depth-10 volatile — `tts_node.py:1137`;
the only explicit `durability=` in the file is `VOLATILE` for audio at
`tts_node.py:1134`) — for
`/voice/tts/voices` we explicitly want latched to survive quest WS
reconnects without a 5-min TTL wait.

## Source of truth for the catalogue

Two candidates exist. They are **not equivalent**:

1. **`rob_box_voice.tts_voice_registry`** (provider names: `yandex|minimax|silero`,
   `tts_voice_registry.py:40-67`). Pure-Python dict, importable from any
   package, **already used** by:
   - `ListTtsVoicesTool` for LLM context
     (`src/rob_box_mcp_tools/rob_box_mcp_tools/tools/dialogue.py:1449,1523`),
   - `SetVoiceTool` for validation
     (`src/rob_box_mcp_tools/rob_box_mcp_tools/tools/dialogue.py:1042,1205`),
   - `dialogue_guards.py` prompt that tells the LLM to call `list_voices`
     (`src/rob_box_voice/rob_box_voice/core/dialogue_guards.py:673-674`).
2. **`BaseTTSProvider.list_voices()`**
   (`src/rob_box_llm/rob_box_llm/tts_provider_base.py:225-227`). Default
   returns `[]` (honest no-op). Only `MiniMaxTTSProvider` overrides it
   (`src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:684`), and
   the override returns a **static built-in list** (`_BUILTIN_VOICES` at
   `minimax_tts.py:447`) — no upstream call.
   Yandex and Silero providers don't subclass `BaseTTSProvider` in
   `tts_node` at all (tts_node imports only `MiniMaxTTSProvider` from
   `rob_box_llm`; `tts_node.py:112`).

**Decision:** the registry is the SoT for the wire payload. Rationale:

- It is the **only source that knows all three providers** in one place.
- It already matches the actual LLM/tool behaviour (validation, prompt
  context, default-voice fallback via `default_voice_for` at
  `tts_voice_registry.py:74-76`).
- `BaseTTSProvider.list_voices()` is async and currently yields no
  benefit over the synchronous dict for built-in catalogues — but is the
  right hook the day a provider adds an account-specific
  `/v1/voices` endpoint (see the override-point comment in
  `minimax_tts.py:684-696`). The
  **registry may be enriched by an async override later**; for now both
  answers coincide and we pick the synchronous one.
- Per issue #1919 acceptance criterion: "не выдумывать список в quest-ноде"
  — we don't: tts_node reads the registry, publishes it.

## Wire payload (outbound `voice_list`)

Aligns with the existing client type at
`src/rob_box_quest/webxr_client/src/wire/messages.ts:125-133` (`VoiceInfo`).
We extend with `provider` (needed for cross-provider UI; not in current
type — add it):

```json
{
  "type": "voice_list",
  "voices": [
    {
      "voice_id": "alena",
      "display_name": "Алёна",
      "language": "ru-RU",
      "gender": "female",
      "provider": "yandex",
      "description": "",
      "presets": ["standard", "friendly"]
    }
  ],
  "active_provider": "yandex",
  "active_voice": "alena",
  "ts_ms": 1234567890
}
```

`active_provider` / `active_voice` come from `/voice/tts/provider_state`
(publisher declared at `tts_node.py:1146-1148`, payload built in
`_publish_provider_state` at `tts_node.py:2310` — dict at `:2337`,
publish at `:2347`).
quest_node subscribes to **both** topics and merges on receipt.

> Naming note: the wire type is `voice_list` (singular), not
> `voices_list`. The current `messages.ts:145` defines `voice_list`, and
> `meta-quest-api.md:237` documents the same spelling.
> Issue body mentions `voices_list` — that was a typo; we follow the
> client type.

## `list_voices` cache — location and TTL

Cache lives on **quest_node side**, not tts_node side. Reasoning:

- tts_node publishes latched; quest_node receives on connect + on every
  provider_state change → already gets updates for free. **No TTL needed
  on tts_node side**: latched QoS + republish-on-change is its invalidation
  rule.
- quest_node still keeps a 5-minute in-memory TTL (issue #1919 acceptance
  criterion) to handle **transient WS reconnects** where the quest might
  briefly miss a republish. Invalidation rules:
  - any `/voice/tts/voices` message → cache replaced, TTL reset to 5 min;
  - any `/voice/tts/provider_state` message with `provider != cache.active_provider`
    → trigger re-fetch (issue #1765 — provider may have changed without
    list update being sent yet);
  - 5 min idle → next quest `list_voices` request forces a fresh publish
    from tts_node (tts_node re-publishes on demand via a small
    `~/list_voices` trigger topic, or — simpler — tts_node republishes
    `/voice/tts/voices` on every `set_provider` event, which we
    already do).

Default TTL exposed as a quest-node config (`voices_cache_ttl_sec`,
default `300`).

## Failure modes

| Scenario | Behaviour |
|---|---|
| Provider returns empty `voices` (e.g. registry has no entry for the name) | tts_node publishes `voice_list` with `voices: []`; UI shows "провайдер не отдаёт список голосов" (per issue #1919 acceptance). **No hardcoded fallback** in quest-node. |
| `/voice/tts/voices` topic missing (e.g. tts_node down) | quest_node cache stale after TTL; UI shows "голосовой пайплайн недоступен"; subsequent `list_voices` returns cached (if any) or `voice_list` with `voices: []` and `error: "tts_unreachable"`. |
| `/voice/tts/provider_state` mismatch with `/voice/tts/voices` | quest_node trusts `provider_state` as SoT for `active_provider`; flags the voices list as `stale: true` until next republish lands within TTL. |
| `set_voice` to unknown voice | supervisor validates against the same registry (`voices_for` at `tts_voice_registry.py:69-71`), exactly as `SetVoiceTool` already does (`dialogue.py:1205,1210,1213` → `voice_unavailable` + `available` list); returns `voice_set_nack{reason: "voice_unavailable", available: [...]}`; **no change applied** to tts_node parameters. |

## File:line evidence (every claim above)

All references below re-verified against `origin/develop` @ `19e759f4`
(rebase point of this branch, 03.09.2026). Line numbers drift with develop —
the anchors are the symbol names, the numbers are a snapshot.

- `BaseTTSProvider.list_voices` default (`return []`) → `src/rob_box_llm/rob_box_llm/tts_provider_base.py:225-227`; contract listed as "optional, default empty list" at `:181`
- `TTSProviderRegistry` class → `src/rob_box_llm/rob_box_llm/tts_provider_registry.py:53`
- `TTSProviderFactory` + process-lifetime `_cache` dict → `tts_provider_registry.py:98,108,126-136` (`reset_cache` at `:134`)
- `register_builtin_tts_providers` → `tts_provider_registry.py:139`
- `MiniMaxTTSProvider.list_voices` override (static `_BUILTIN_VOICES`, **no upstream call** — comment names it the future HTTP override point) → `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:684`, catalogue at `:447`
- Built-in registry (yandex/minimax/silero) → `src/rob_box_voice/rob_box_voice/tts_voice_registry.py:40` (`PROVIDER_VOICES`), `:62` (`DEFAULT_VOICES`), `:69` (`voices_for`), `:74` (`default_voice_for`), `:79` (`resolve_voice`)
- tts_node imports `MiniMaxTTSProvider` (proves BaseTTSProvider reachable in tts_node) → `src/rob_box_voice/rob_box_voice/tts_node.py:112`; guarded import, `None` fallback at `:123`
- tts_node publishes `/voice/tts/state` → `tts_node.py:1137`; `/voice/tts/provider_state` → `tts_node.py:1146-1148`
- **No transient-local publisher exists in tts_node yet**: the only `durability=` in the file is `DurabilityPolicy.VOLATILE` for audio → `tts_node.py:1134`
- tts_node subscribes to `/voice/tts/set_provider` → `tts_node.py:1117`
- `_publish_provider_state` builds the payload (`payload = {...}` at `:2337`, `pub.publish` at `:2347`) → `tts_node.py:2310`; called on startup at `:1221` and after provider change at `:1672`
- supervisor `_on_set_voice_mode` → `src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py:386`; `_apply_voice_mode` → `:398`; `_set_dialogue_param` → `:414-451` (client created at `:424`, `call_async` at `:440`, done-callback at `:442-451`)
- supervisor `/avatar/set_voice_mode` topic constant → `supervisor_node.py:105`, subscription → `:181`
- `set_voice_mode` end-to-end: quest bridge publish → `src/rob_box_quest/rob_box_quest/quest_node.py:372-386` (publisher created at `:561`, wired at `:628`); ws_server cmd handler → `src/rob_box_quest/rob_box_quest/server/ws_server.py:489-501` (`voice_mode_ack` at `:499`); bridge protocol stub → `ws_server.py:104-105,161`
- Client wire types: `ListVoicesCmd` → `src/rob_box_quest/webxr_client/src/wire/messages.ts:78`; `VoiceInfo` → `:125-133`; `JsonEvent` union → `:135`; `voice_list` event → `:145`; `voice_set_ack`/`voice_set_nack` → `:146-147`
- Server API doc §4.1 (`list_voices` → `voice_list`) → `docs/architecture/meta-quest-api.md:231-239`; §4.3 (`set_voice` → ack/nack) → `:217,226-227`; rate limits (`set_voice` 1/2s, `list_voices` 1/10s) → `:444-445`
- MCP `SetVoiceTool` (validates against `voices_for`, returns `voice_unavailable` + `available`) → `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/dialogue.py:1042,1205,1210,1213`; publishes `/voice/tts/set_provider` → `:1086-1087,1241`
- MCP `ListTtsVoicesTool` reads the same registry → `dialogue.py:1449,1523`
- LLM guard prompt tells the model to call `list_voices` → `src/rob_box_voice/rob_box_voice/core/dialogue_guards.py:673-674` (test pin: `src/rob_box_voice/test/unit/core/test_dialogue_guards.py:953`)
- **Negative result — no existing ROS service/topic serves a voice inventory today**: `grep -rn 'list_voices\|get_voices' src/rob_box_voice/ src/rob_box_quest/rob_box_quest/` (excluding tests) matches only the two prompt strings in `dialogue_guards.py:673-674`; no `.srv` file in `src/` mentions voices. Hence a **new** mechanism is required — this is the gap issue #1919 calls "доки врут — фичи нет".
- Issue acceptance criteria → https://github.com/krikz/rob_box_project/issues/1919

## What this design does NOT cover (out of scope per task body)

- `preview_voice` — synthesize + stream audio back; separate card.
- 3D-voice-picker UI on the client — separate card (style reference:
  `stream_menu.ts`).
- ADR-0028 §5.1 extension — if supervisor eventually moves to a
  `SetVoice` service (custom IDL), this design's topic is a clean
  intermediate step: same data, just transport swap.
- Editing `tts_node.yaml` to declare the new `voices_cache_ttl_sec`
  parameter on the quest side — that's the implementation card.
- Adding `provider` to the existing client `VoiceInfo` type — minor type
  bump, one-line TS change.

## Acceptance check (vs. issue #1919 §1)

- ✅ ROS path mapped: quest_node ← topic ← tts_node ← static registry.
- ✅ Source of truth = provider's catalogue (registry = provider catalogue
  in current implementation; `BaseTTSProvider.list_voices()` is the
  long-term hook for runtime providers).
- ✅ Empty list path honest: provider returns `[]` → wire returns `[]`,
  no hardcoded fallback.
- ✅ Cache TTL = 5 min, configurable, invalidation on `provider_state`.
- ✅ `set_voice` must go through supervisor (mirrors `voice_mode`).
- ⏳ Server-side implementation, UI, tests — separate cards.

## Verification log

Re-verified on 03.09.2026 against `origin/develop` @ `19e759f4` (this branch
was rebased onto it; `git rev-list --left-right --count origin/develop...HEAD`
→ `0  2`). Every reference in the evidence section above was re-read from
the file on disk, not carried over from the prior revision's prose.

Findings that changed the design (not just line numbers):

1. **`/voice/tts/state` is NOT latched.** The earlier revision of this doc
   claimed the new topic would "mirror `/voice/tts/state` (latched)". It is a
   plain depth-10 publisher (`tts_node.py:1137`), and the single explicit
   `durability=` in the whole file is `DurabilityPolicy.VOLATILE` for the
   audio topic (`tts_node.py:1134`). So `/voice/tts/voices` will be the
   **first** `TRANSIENT_LOCAL` publisher in `tts_node` — the implementation
   card must add the QoS profile, it cannot copy an existing one.
2. **`set_voice` needs a second SetParameters client, not a reuse.** The
   supervisor's existing client targets `/dialogue_node/set_parameters`
   (`supervisor_node.py:424`), but the voice parameters are declared on
   `tts_node` (`yandex_voice` `:677`, `silero_speaker` `:691`,
   `minimax_voice` `:716`). A new `_set_tts_voice_param` on
   `/tts_node/set_parameters` is required.
3. **No voice-inventory service or topic exists today.** Grep across
   `src/rob_box_voice/` and `src/rob_box_quest/rob_box_quest/` for
   `list_voices|get_voices` (tests excluded) returns only two LLM prompt
   strings (`dialogue_guards.py:673-674`); no `.srv` in `src/` mentions
   voices. The client type (`messages.ts:78,145`) and the API doc
   (`meta-quest-api.md:231-239`) both already describe the contract — the
   ROS side is what's missing. This is precisely the "доки врут — фичи нет"
   of issue #1919.
4. **`TTSProviderFactory` caches per `(provider, ...)` key for the process
   lifetime** (`tts_provider_registry.py:108,126-136`), so "the active
   provider" is a process-local singleton per key, not a per-request object.
   A `list_voices` answer is therefore stable between `set_provider` events —
   which is what makes latched-publish-on-change sufficient and a TTL on the
   tts_node side unnecessary.

CI note: at the time of this commit the `Unit Tests (ROS2 Humble)` job is red
on **develop itself** (`19e759f4`, run 33731415831) with
`RuntimeError: test_voice_bench_scoring: scripts/voice_bench/score.py not found`
— a pre-existing collection error in `rob_box_voice`, unrelated to this
markdown-only change (this branch's diff touches exactly one `.md` file).
