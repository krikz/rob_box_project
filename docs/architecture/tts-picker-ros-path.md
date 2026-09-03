# TTS picker — ROS path: quest_node → TTS provider

Issue: #1919 (AV-27) · Card: `t_5b9d5d0c` · ADR parent: ADR-0027 / ADR-0028
Owner: architect (recon only — no production code)

## TL;DR — decision

**Mechanism for `list_voices`:** direct in-process call from `tts_node` to the
**built-in static catalogue** at
`src/rob_box_voice/rob_box_voice/tts_voice_registry.py:40-66`
(`PROVIDER_VOICES` / `DEFAULT_VOICES`). tts_node exposes the answer to the
ROS graph as a new **latched topic** `/voice/tts/voices` published with
`DurabilityPolicy.TRANSIENT_LOCAL` + `depth=1` (one-shot publish on
startup + on every `set_provider` change). quest_node subscribes to that
topic and caches; no new ROS service, no synchronous RPC.

**Mechanism for `set_voice`:** publish a JSON request to a new topic
`/avatar/set_voice` and let the **supervisor** apply it to `tts_node` via a
**second SetParameters client** on `/tts_node/set_parameters` (mirrors the
existing `_set_dialogue_param` at
`src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py:332-369`,
which calls `/dialogue_node/set_parameters`). The voice parameter
(`yandex_voice` / `minimax_voice` / `silero_speaker`) lives on `tts_node`
(`tts_node.py:677` and `config/tts_node.yaml:62,77`), not on
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
QoS as `/voice/tts/state` (which is volatile — `tts_node.py:1137`) — for
`/voice/tts/voices` we explicitly want latched to survive quest WS
reconnects without a 5-min TTL wait.

## Source of truth for the catalogue

Two candidates exist. They are **not equivalent**:

1. **`rob_box_voice.tts_voice_registry`** (provider names: `yandex|minimax|silero`,
   `tts_voice_registry.py:40-66`). Pure-Python dict, importable from any
   package, **already used** by:
   - `ListTtsVoicesTool` for LLM context
     (`src/rob_box_mcp_tools/rob_box_mcp_tools/tools/dialogue.py:1523`),
   - `SetVoiceTool` for validation
     (`src/rob_box_mcp_tools/rob_box_mcp_tools/tools/dialogue.py:1205`),
   - `dialogue_guards.py` for tool-skipping logic
     (`src/rob_box_voice/test/unit/core/test_dialogue_guards.py:951`).
2. **`BaseTTSProvider.list_voices()`**
   (`src/rob_box_llm/rob_box_llm/tts_provider_base.py:225-227`). Default
   returns `[]` (honest no-op). Only `MiniMaxTTSProvider` overrides it
   (`src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:684-768`), and
   the override returns a **static built-in list** — no upstream call.
   Yandex and Silero providers don't subclass `BaseTTSProvider` in
   `tts_node` at all (tts_node has its own provider classes for them;
   `tts_node.py:97-122`).

**Decision:** the registry is the SoT for the wire payload. Rationale:

- It is the **only source that knows all three providers** in one place.
- It already matches the actual LLM/tool behaviour (validation, prompt
  context, default-voice fallback at `tts_voice_registry.py:62-66`).
- `BaseTTSProvider.list_voices()` is async and currently yields no
  benefit over the synchronous dict for built-in catalogues — but is the
  right hook the day a provider adds an account-specific
  `/v1/voices` endpoint (see comment at `tts_provider_base.py:226`). The
  **registry may be enriched by an async override later**; for now both
  answers coincide and we pick the synchronous one.
- Per issue #1919 acceptance criterion: "не выдумывать список в quest-ноде"
  — we don't: tts_node reads the registry, publishes it.

## Wire payload (outbound `voice_list`)

Aligns with the existing client type at
`src/rob_box_quest/webxr_client/src/wire/messages.ts:124-133` (`VoiceInfo`).
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
(declared at `tts_node.py:1141-1148`, payload built at `tts_node.py:2310-2355`).
quest_node subscribes to **both** topics and merges on receipt.

> Naming note: the wire type is `voice_list` (singular), not
> `voices_list`. The current `messages.ts:145` defines `voice_list`.
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
| `set_voice` to unknown voice | supervisor validates against the same registry (`supervisor_node.py` reads `voices_for(provider)` from `tts_voice_registry.py:69-71`); returns `voice_set_nack{reason: "voice_unavailable", available: [...]}`; **no change applied** to tts_node parameters. |

## File:line evidence (every claim above)

- `BaseTTSProvider.list_voices` default → `src/rob_box_llm/rob_box_llm/tts_provider_base.py:225-227`
- `BaseTTSProvider` registry → `src/rob_box_llm/rob_box_llm/tts_provider_registry.py:53-95`
- `TTSProviderFactory` caching → `src/rob_box_llm/rob_box_llm/tts_provider_registry.py:98-136`
- `MiniMaxTTSProvider.list_voices` override → `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:684-768`
- Built-in registry (yandex/minimax/silero) → `src/rob_box_voice/rob_box_voice/tts_voice_registry.py:40-66`
- tts_node imports `MiniMaxTTSProvider` (proves BaseTTSProvider class reachable in tts_node) → `src/rob_box_voice/rob_box_voice/tts_node.py:112`
- tts_node publishes `/voice/tts/state` + `/voice/tts/provider_state` (active voice travels in the provider_state JSON payload, not as a separate topic) → `tts_node.py:1137, 1141-1148`
- tts_node subscribes to `/voice/tts/set_provider` (the MCP-side mirror) → `tts_node.py:1116-1118`
- `_publish_provider_state` builds `{provider, voice, default_voice, reason, ts}` payload → `tts_node.py:2310-2355`
- supervisor `_apply_voice_mode` / `_set_dialogue_param` pattern to mirror → `src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py:316-369`
- supervisor `/avatar/set_voice_mode` topic → `supervisor_node.py:95`
- `set_voice_mode` end-to-end (bridge publish → supervisor apply → ack) → `src/rob_box_quest/rob_box_quest/quest_node.py:302-316` + `467`, `src/rob_box_quest/rob_box_quest/server/ws_server.py:460-472`
- Client wire types (`ListVoicesCmd`, `voice_list`, `VoiceInfo`) → `src/rob_box_quest/webxr_client/src/wire/messages.ts:77-145`
- Server API doc for §4.1/§4.3 → `docs/architecture/meta-quest-api.md:228-240`
- MCP existing pattern (`set_voice` → publish `/voice/tts/set_provider`) → `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/dialogue.py:1086-1088`
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

## Verified during this re-run (recon handoff)

- Confirmed by reading actual sources (not from prior commit's prose):
  - `tts_provider_base.py:225-227` → async `list_voices()` returns `[]`
  - `tts_provider_registry.py:53-95` → `TTSProviderRegistry` (in-memory,
    process-lifetime)
  - `tts_voice_registry.py:40-66,69-71` → `PROVIDER_VOICES`, `DEFAULT_VOICES`,
    `voices_for()`
  - `tts_node.py:1116-1148,2310-2355` → publishes `/voice/tts/state`
    (volatile, depth=10) + `/voice/tts/provider_state`; subscribes to
    `/voice/tts/set_provider`. **No transient-local QoS in this package yet**
    (grep `DurabilityPolicy` → only one use, at line 1134 for audio, set to
    VOLATILE) — so the new `/voice/tts/voices` publisher is the first
    transient-local topic in `tts_node`.
  - `supervisor_node.py:316-369` → `_apply_voice_mode` /
    `_set_dialogue_param` pattern; client is `/dialogue_node/set_parameters`,
    lazy-init, only in active mode (S12). No existing client for
    `/tts_node/set_parameters` — must be added (named `_set_tts_voice_param`
    in this design).
  - `tts_node.py:677` + `config/tts_node.yaml:62,77` → voice params are
    declared on tts_node (`yandex_voice`, `minimax_voice`), so set_voice
    must target tts_node parameters, not dialogue_node.
  - `quest_node.py:302-316,467` + `ws_server.py:460-472` → end-to-end
    pattern for `set_voice_mode` to mirror for `set_voice`.
  - `mcp_tools/tools/dialogue.py:1086-1088` → publishes
    `/voice/tts/set_provider` (separate path, not used by set_voice).
  - ADR-0028 §4.4 (`S5`, `S12`) → external client → supervisor → param
    flow is the only allowed path for `dialogue_node` and must be mirrored
    for `tts_node` parameters.
- Two corrections to the prior version of this doc:
  1. Removed the false "mirrors `/voice/tts/state` (latched)" analogy —
     that topic is volatile. New text specifies `TRANSIENT_LOCAL` + depth=1
     for `/voice/tts/voices` explicitly.
  2. Made explicit that set_voice needs a **second** SetParameters client
     on `/tts_node/set_parameters`, not a reuse of the dialogue_node one,
     because the voice parameter is on tts_node.
