# PR #907 — Provider Contract Review (multimodal, TTS)

- **PR:** https://github.com/krikz/rob_box_project/pull/907
- **Head SHA:** `6e75eb39` (`docs: resolve CHANGELOG conflict + add voice/llm guides`)
- **Reviewer:** `architect` (Hermes Agent)
- **Scope:** the new `src/rob_box_llm/` package's provider **contracts** (LLM + TTS)
  and the **multimodal** surface (`TextPart`/`ImagePart`/`MessageContent`),
  plus the contract ↔ implementation ↔ ROS-consumer ↔ yaml-config matrix.
  Cross-cutting concerns (Dockerfiles, test mocks, audio wrapping) are
  covered by the `pr-reviewer` review at `analysis/pr-907-review.md`.
- **Verdict:** **APPROVE-WITH-FOLLOWUPS.** The contracts are sound, the
  implementations follow them faithfully, the conformance suite passes
  locally (102/102 in scope). Six concrete followups before merge.

---

## 1. Scope and method

| Layer | Contract (file) | Implementations in PR | Pre-existing consumer |
|-------|-----------------|------------------------|-----------------------|
| LLM   | `provider.py:202` — `LLMProvider` ABC + value objects | `providers/deepseek.py` (DeepSeek + MiMo + MiniMax via `_OpenAICompatibleProvider`), `providers/minimax.py` (MiniMax-only subclass), `providers/fake.py` | `rob_box_voice/llm/provider_manager.py` — **NOT YET** migrated to `rob_box_llm` (this PR is additive; migration is P1) |
| TTS   | `tts.py:121` — `TTSProvider` ABC + value objects | `providers/minimax_tts.py` (concrete), `tts.py:185` (`FakeTTSProvider` in-line) | `rob_box_voice/tts_node.py:1118-1403` — `MiniMaxTTSProvider` lazy-imported |
| TTS extension | `tts_provider_base.py:160` — `BaseTTSProvider(TTSProvider)` | Same `MiniMaxTTSProvider` subclasses it | **NOT YET** consumed — `tts_node` uses direct construction, not `TTSProviderFactory` |
| Multimodal (LLM) | `provider.py:34-65` — `TextPart` / `ImagePart` / `MessageContent` | `providers/deepseek.py:76-94` (translation), `providers/minimax.py:130-148` (size-cap) | None — `dialogue_node.py` still uses SDK-shape messages |

Local verification:

```
$ PYTHONPATH=src/rob_box_llm:src/rob_box_llm/test pytest \
    src/rob_box_llm/test/test_tts_provider_contract.py \
    src/rob_box_llm/test/test_tts_conformance.py \
    src/rob_box_llm/test/test_provider_value_objects.py \
    src/rob_box_llm/test/test_minimax_provider.py
collected 102 items
... 102 passed in 0.86s
```

---

## 2. Contract ↔ implementation matrix

### 2.1 LLM contract (`LLMProvider`)

| Contract method | `DeepSeekProvider` | `MiMoProvider` | `MiniMaxProvider` | `FakeLLMProvider` |
|-----------------|--------------------|----------------|-------------------|---------------------|
| `complete(messages, *, tools=(), settings=None) -> LLMResponse` | `deepseek.py:308` | inherited | inherited (with image-cap pre-flight) | `fake.py:125` |
| `stream(messages, *, tools=(), settings=None) -> AsyncIterator[LLMChunk]` | `deepseek.py:347` | inherited | inherited | `fake.py:145` |
| `aclose() -> None` | `deepseek.py:372` (`self._client.close()`) | inherited | inherited | **not overridden** — falls through to ABC no-op |
| `capabilities -> ProviderCapabilities` | `_OpenAICompatibleProvider:206` (image_input=False) | inherited | `minimax.py:208` (image_input=True) | inherits default `ProviderCapabilities()` |
| `capabilities_for(model) -> ProviderCapabilities` | inherits default | inherits default | `minimax.py:244` (narrows on vision model token) | inherits default |
| Errors raised | `RateLimitError`/`TimeoutError`/`AuthError`/`ContentFilterError`/`ProviderError`/`CapabilityUnavailableError` (per `_map_exception` `deepseek.py:139`) | inherited | adds `base_resp.status_code != 0` envelope check (`minimax.py:159`) | n/a — explicit `IndexError` when scripts exhausted (test-only contract) |

**Findings:**

- ✅ **Signatures match.** All concrete providers use the same `(messages, *, tools=(), settings=None)` keyword-only shape with `Iterable[LLMMessage]`.
- ✅ **Error hierarchy is consistent.** LLM-side uses `ProviderError` and 5 subclasses; TTS-side uses parallel `TTSError` tree. No cross-contamination: `rob_box_llm.errors.TimeoutError` is a `ProviderError` (LLM); `rob_box_llm.errors.TTSTimeoutError` is a `TTSError` (TTS).
- ⚠️ **P0 finding 1 — `_apply_thinking_policy` in `minimax.py:301-324` rebuilds `LLMSettings` field-by-field.** A new field added to `LLMSettings` (`provider.py:155`) silently drops out of the rebuilt settings. Fix: use `dataclasses.replace(settings, extra=merged_extra)`. One-line change; no behaviour delta today. (Not blocking — there are no unmodelled fields yet — but a latent foot-gun for the next contributor who adds one.)
- ⚠️ **P0 finding 2 — `_require_capability_for_messages` and `_build_kwargs` in `deepseek.py:211-282` iterate `messages` twice.** If a caller passes a generator it gets exhausted by the first iteration and the second sees an empty sequence → `messages=[]` request to OpenAI SDK. The previous reviewer flagged this; seconded here. Cheap fix: freeze at the public boundary (`messages = tuple(messages)` in `complete`/`stream`).
- ⚠️ **P0 finding 3 — `_validate_image_bytes` (`minimax.py:130-148`) only enforces the 10 MB cap on `bytes` sources.** URL and `data:` sources pass through unbounded. Document or enforce.

### 2.2 Multimodal content contract

`TextPart` (frozen, `text: str`) and `ImagePart` (frozen, `source: Union[str, bytes]`, `media_type: str = "image/jpeg"`, `detail: str = "default"`) at `provider.py:34-65`.

`MessageContent = Union[str, tuple[MessagePart, ...]]` at `provider.py:69-70`. Plain `str` remains valid — text-only callers are not broken.

| Surface | `ImagePart` | `TextPart` | `MessageContent` translation |
|---------|-------------|------------|-------------------------------|
| `DeepSeekProvider` (OpenAI-compat) | rendered as `{"type": "image_url", "image_url": {"url": "data:...;base64,...", "detail": ...}}` (`deepseek.py:56`) | `{"type": "text", "text": ...}` | tuple → list of dicts; `str` passes through (`deepseek.py:76-94`) |
| `MiniMaxProvider` | same OpenAI wire-format (inherits from `_OpenAICompatibleProvider`) | same | same + `_validate_image_bytes` pre-flight (`minimax.py:130-148`) |

**Findings:**

- ✅ **Backwards compat preserved.** `MessageContent = Union[str, tuple[MessagePart, ...]]` — every pre-existing call site that passes a plain string keeps working (the previous reviewer verified 357/357 tests in `src/rob_box_llm/test/`).
- ✅ **Validation is tight.** `ImagePart.__post_init__` (`provider.py:60-64`) refuses non-`image/*` media types and unknown `detail` levels — fail-fast.
- ✅ **Image size cap is provider-local** (`MINIMAX_MAX_IMAGE_BYTES = 10 MB`, `minimax.py:75`). Good — provider-local constants are the right place (OpenAI may publish a different cap; we should not couple them).

### 2.3 TTS contract (`TTSProvider`)

| Contract method | `MiniMaxTTSProvider` | `FakeTTSProvider` |
|-----------------|----------------------|---------------------|
| `synthesize(text, *, settings=None) -> TTSAudio` | `minimax_tts.py:731` | `tts.py:202` |
| `stream(text, *, settings=None) -> AsyncIterator[TTSChunk]` | `minimax_tts.py:765` (SSE-buffered: yields one audio chunk then a terminal `finish_reason="stop"`) | `tts.py:230` (single terminal chunk) |
| `aclose() -> None` | `minimax_tts.py:913` (idempotent, only closes owned clients) | `tts.py:244` (no-op) |
| `name: str` | `"minimax"` (`minimax_tts.py:437`) | `"fake"` (`tts.py:197`) |
| Errors raised | `TTSBadRequestError` (volume range, empty text, invalid extra), `TTSAuthError`, `TTSRateLimitError`, `TTSTimeoutError`, `TTSError` (per `_map_exception` `minimax_tts.py:96`) | `TTSBadRequestError` for empty text |

**Findings:**

- ✅ **Container format is honest.** `synthesize` (`minimax_tts.py:752`) reports the actual format on `TTSAudio.format` even when the user asked for `OGG` (MiniMax doesn't ship OGG → falls back to MP3). The caller never sees a lie.
- ✅ **`stream()` honours the "raise before yield" invariant.** `minimax_tts.py:880-896` re-raises pre-yield exceptions and converts post-yield ones to a terminal `TTSChunk(finish_reason="error")`.
- ⚠️ **P1 finding 4 — `MiniMaxTTSProvider.stream()` returns at most 2 chunks (audio + terminal "stop"), not true chunk-per-frame.** This is documented as intentional (`tts.py:160-166` and `minimax_tts.py:771-781`), but consumers may assume multi-chunk. Tests at `test/unit/tts/test_minimax_integration.py:383` enforce single-chunk streaming, which is correct under the current contract — just keep the doc honest.
- ⚠️ **P1 finding 5 — `FakeTTSProvider.synthesize` (`tts.py:222`) packs `n_samples = max(len(text.encode("utf-8")), 1)` and writes constant value `1`.** This is fine for tests that check byte length or count, but consumers that interpret `samples` as a real audio waveform will see constant DC offset. Document or expose a knob.
- ℹ️ **TTS streaming latency win is deferred to WebSocket (M5/M6).** Already in ADR-0003 §2.4. No contract change needed.

### 2.4 TTS extension contract (`BaseTTSProvider`)

Layered on top of `TTSProvider` (`tts_provider_base.py:160`); every `BaseTTSProvider` IS-A `TTSProvider`, so existing call sites that type-annotate `TTSProvider` keep working.

| Extension hook | `MiniMaxTTSProvider` | Notes |
|----------------|----------------------|-------|
| `capabilities() -> TTSCapabilities` | `minimax_tts.py:467` (`streaming=True, voice_cloning=True, audio_format_pcm=True, audio_format_mp3=True`, others False) | static; default impl returns all-False (honest no-op) |
| `list_voices() -> list[TTSVoice]` | `minimax_tts.py:495` (inline list of 6 voices) | default: `[]` |
| `healthcheck() -> TTSHealth` | `minimax_tts.py:549` (validates credentials only, no upstream call) | default: `TTSHealth(ok=True)` |
| `_build_request_payload(text, settings, voice_meta) -> dict` (mandatory) | `minimax_tts.py:583` (delegates to module-level `_build_payload`, `stream=False` hardcoded — see P2 finding) | `BaseTTSProvider:228` marks it `@abc.abstractmethod` |
| `_http_client_factory() -> httpx.AsyncClient` | `minimax_tts.py:573` (`httpx.AsyncClient(timeout=self._timeout)`) | `BaseTTSProvider:241` default matches |

**Findings:**

- ✅ **Subclassing is conservative.** `BaseTTSProvider` IS-A `TTSProvider`; `isinstance(x, TTSProvider)` still works.
- ⚠️ **P2 finding 6 — `_build_request_payload` (mandatory override) hardcodes `stream=False` (`minimax_tts.py:613`).** This means the hook cannot be used by the streaming path. The hook is therefore "test-only", which contradicts the docstring at `tts_provider_base.py:170` ("Pure mapping; tested in isolation"). Either pass `stream` as a parameter to the hook (and `synthesize`/`stream` pass it accordingly), or document the hook as a non-stream-only inspection point.
- ⚠️ **P2 finding 7 — `_BUILTIN_VOICES` (`minimax_tts.py:312-355`) is dead code.** The runtime `list_voices()` at line 504-547 returns a different inline list. Same finding from the previous reviewer; seconded. Recommend deleting `_BUILTIN_VOICES` (its docstring references are now incorrect) and the second copy of the catalogue.
- ℹ️ **`TTSProviderRegistry` / `TTSProviderFactory` (`tts_provider_registry.py`) is exported from `__init__.py` but no caller uses it.** `tts_node` still lazy-instantiates `MiniMaxTTSProvider` directly (`tts_node.py:1149, 1377`). This is intentional for the PR (migration is P1+), but it means the extension architecture is currently a pure addition with no consumer. Tests cover it (`test_tts_extension_points.py`) so the contract is exercised — good.
- ℹ️ **`MiniMaxTTSProvider.stream()` (`minimax_tts.py:765-911`) is ~150 lines of SSE parsing.** The contract says "raise before yield, terminal chunk with `finish_reason`" — the implementation honours it, but the helper could be split out for readability. Post-merge cleanup.

### 2.5 Config consistency

#### ROS-side (`src/rob_box_voice/config/voice_assistant.yaml`)

**Status:** the legacy config under `src/` is fine and documents MiniMax TTS fields (`provider: minimax`, `minimax_speed`, `minimax_max_retries`, …). No `tts_node` provider for `piper`.

#### Deployment-side (`docker/vision/config/voice_assistant/voice_assistant.yaml`)

| Field | Value at line | Reality | Verdict |
|-------|---------------|---------|---------|
| `/** ros__parameters.provider` (LLM) | `15: provider: "mimo"` | OK | ✅ |
| `/** ros__parameters.model` | `16: model: "mimo-v2.5-pro"` | OK (matches `MiMoProvider.DEFAULT_MODEL` at `providers/mimo.py:27`) | ✅ |
| `/** ros__parameters.fallback_model` | `17: fallback_model: "mimo-v2.5"` | OK | ✅ |
| `/** ros__parameters.enable_fallback` | `18: enable_fallback: false` | OK | ✅ |
| `tts_node.provider` | `129: provider: "piper"` | **No `piper` provider exists.** `tts_node.py:229-233` only accepts `{"yandex", "silero", "minimax"}` → **startup `ValueError`** | 🚫 **P1 finding 8** |
| `tts_node.piper.model_path` | `133` | would be unused if piper were wired | n/a |
| `tts_node.silero.*` / `tts_node.yandex.*` | `140-160` | OK (matches `tts_node.py:175-185, 236-238`) | ✅ |
| `dialogue_node.provider` | `172: provider: "mimo"` | OK | ✅ |
| `dialogue_node.model` | `173: model: "mimo-v2.5-pro"` | OK | ✅ |
| `dialogue_node.deepseek.model` | `190: model: "deepseek-v4-flash"` | OK (matches `DeepSeekProvider.DEFAULT_MODEL = "deepseek-chat"` override) | ✅ |
| Missing `tts_node.minimax_*` block | n/a | deployment config does NOT include the `minimax:` sub-block — opt-in requires `docs/guides/examples/minimax_tts.yaml` to be merged in or a manual edit | ℹ️ not blocking (opt-in) |

#### Test-side (`docker/vision/test/config/voice_assistant_test.yaml`)

- `/dialogue_node.provider: "deepseek"` — OK (`dialogue_node.py:81-87` recognises `deepseek`).
- `model: "deepseek-v4-flash"` — same observation; OK.

#### Docs examples

- `docs/guides/examples/minimax_tts.yaml` — duplicates every `minimax_*` knob from `tts_node.py`. Good for copy-paste ops. Consistent with `tts_node.py:189-210`.
- `docs/guides/examples/minimax_llm.yaml` — `mimo.model: MiMo-7B` (line 60) **disagrees** with the code default `mimo-v2.5-pro` (`providers/mimo.py:27`) and the live config `mimo-v2.5-pro` (`src/rob_box_voice/config/voice_assistant.yaml:118`). Same drift appears in `src/rob_box_llm/README.md:25` and `docs/guides/MINIMAX.md:46`.

---

## 3. Findings (consolidated, in priority order)

### P0 — should be fixed in this PR (block merge)

#### F1. `_apply_thinking_policy` rebuilds `LLMSettings` field-by-field

- **File:** `src/rob_box_llm/rob_box_llm/providers/minimax.py:301-324`
- **Risk:** Adding any new field to `LLMSettings` (`provider.py:155-164`) silently drops it from `MiniMaxProvider` responses.
- **Fix:** one-line — `return dataclasses.replace(settings, extra=merged_extra)`.
- **Why this matters:** the contract explicitly enumerates the `LLMSettings` shape; a provider silently losing a field is a contract violation by omission.

#### F2. `_OpenAICompatibleProvider.complete/stream` iterate `messages` twice without freezing

- **File:** `src/rob_box_llm/rob_box_llm/providers/deepseek.py:211-254, 308-371`
- **Risk:** `Iterable[LLMMessage]` accepts a generator. After the first iteration it's empty; the second passes `messages=[]` to the OpenAI SDK → confusing 400 / empty response.
- **Fix:** `messages = tuple(messages)` at the top of both `complete()` and `stream()`.
- **Cost:** O(n) memory, one-time per call; eliminates the class of bug.

#### F3. `voice_assistant.yaml` references non-existent `piper` provider

- **File:** `docker/vision/config/voice_assistant/voice_assistant.yaml:129`
- **Risk:** Anyone who launches with this config hits `ValueError: provider must be one of: yandex, silero, minimax` (`tts_node.py:230-233`).
- **Fix:** change to `provider: "yandex"` (matches `tts_node.py` defaults and the existing Yandex voice "anton").

### P1 — fix in this PR or as an immediate follow-up

#### F4. `_BUILTIN_VOICES` is dead code

- **File:** `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:312-355`
- **Verdict:** never referenced at runtime. The runtime `list_voices()` (`minimax_tts.py:495-547`) defines its own inline list. Docstring at line 374 claims `:data:_BUILTIN_VOICES` is the source — false.
- **Fix:** delete the module-level `_BUILTIN_VOICES` constant and its 44 lines of dead duplicate voices; rename the inline list to `_LOCAL_CATALOGUE` if you want to keep the static-source-of-truth semantics. Alternatively, point `list_voices()` at the constant and delete the inline copy — pick one source.

#### F5. `MiniMaxTTSProvider._build_request_payload` hardcodes `stream=False`

- **File:** `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:610-616`
- **Issue:** the override is mandatory per `BaseTTSProvider:227`, but it ignores the streaming-vs-non-streaming distinction. The `synthesize` and `stream` paths therefore both call `_build_payload` directly with the right flag (`minimax_tts.py:743, 795`) and bypass the extension hook for anything but inspection.
- **Fix:** extend the hook to accept a `stream: bool` parameter and have `synthesize`/`stream` pass it through. Then the hook is the single source of truth for the wire format.

#### F6. `MiniMaxProvider._validate_image_bytes` does not enforce URL or `data:` size cap

- **File:** `src/rob_box_llm/rob_box_llm/providers/minimax.py:130-148`
- **Issue:** only `bytes` sources are capped at 10 MB. A 50 MB `data:` URL or a CDN URL with a large image passes through. MiniMax's API will reject server-side; user sees a 400 instead of a clear `CapabilityUnavailableError`.
- **Fix:** document the bypass, OR fetch the URL (HEAD) and check `Content-Length`, OR extract the size from `data:image/...;base64,...` and cap. Pick the cheapest that catches the common case.

### P2 — post-merge cleanup (not blockers)

#### F7. `_OpenAICompatibleProvider._require_capability_for_messages` nested generator expression is hard to read

- **File:** `src/rob_box_llm/rob_box_llm/providers/deepseek.py:229-231`
- Same finding as the previous reviewer's S4; seconded.

#### F8. `MiniMaxRedactedLogFilter.filter` clears `record.args = ()` on hit

- **File:** `src/rob_box_llm/rob_box_llm/providers/minimax.py:121`
- Same finding as the previous reviewer's S6; seconded.

#### F9. Docs/code drift on MiMo default model

- `docs/guides/examples/minimax_llm.yaml:60` and `docs/guides/MINIMAX.md:46` say `MiMo-7B`; code (`providers/mimo.py:27`) and `src/rob_box_voice/config/voice_assistant.yaml:118` say `mimo-v2.5-pro`. Update the docs (the code is the source of truth — `mimo-v2.5-pro` is also what `dialogue_node.py:90` uses).

#### F10. `tts_node._synthesize_minimax_async` lazy-init duplicated

- Same finding as the previous reviewer's S7; seconded. Extract `_ensure_minimax_provider()` helper.

#### F11. `MiniMaxTTSProvider.stream()` is ~150 lines of SSE parsing

- Same finding as the previous reviewer's S8; seconded. Extract `_iter_minimax_sse()`.

#### F12. Two TTS contract layers (`TTSProvider` and `BaseTTSProvider`) coexist

- This is intentional per ADR-0008. Just keep the doc honest: every entry in `__init__.py`'s `__all__` related to TTS comes from exactly one of two modules, and the migration plan to retire direct `MiniMaxTTSProvider(...)` construction in `tts_node` should be tracked.

---

## 4. What's good (highlights worth preserving)

- **Secrets hygiene is exemplary.** `MINIMAX_API_KEY` / `MINIMAX_GROUP_ID` come from ENV at construction time (`minimax_tts.py:439-440`) and are scrubbed from logs and exception messages by `_RedactGroupIdFilter` (`minimax_tts.py:278-300`) and `MiniMaxRedactedLogFilter` (`minimax.py:88-122`). 11 dedicated leak-guard tests in `test_minimax_tts_request_params_and_leak_guard.py` cover every leak path.
- **`_map_exception` is centralised** (`deepseek.py:139` for LLM, `minimax_tts.py:96` for TTS, `minimax.py:159` for the MiniMax `base_resp` envelope). All three locations use the same error hierarchy with the same `provider=` kwarg.
- **`ProviderCapabilities` is per-model on `MiniMaxProvider`** (`minimax.py:244-261`). Vision-capable models are gated at `_require_capability_for_messages` BEFORE the network call, so a wrong-model request fails fast with a typed `CapabilityUnavailableError`.
- **Capability defaults are conservative** (`provider.py:78-92`) — every flag is `False` except `text=True`. An unconfigured provider is always capability-honest.
- **`FakeLLMProvider`** (`providers/fake.py`) and **`FakeTTSProvider`** (`tts.py:185`) — both fully implement the contracts, both are deterministic, both record calls. Tests for the new package run entirely without network.
- **OpenAI-compatible wire format is shared** via `_OpenAICompatibleProvider`. `DeepSeekProvider` and `MiMoProvider` are 25-line wrappers; `MiniMaxProvider` adds ~80 lines for the `base_resp` envelope, image-cap pre-flight, and thinking-policy merge. Future OpenAI-compat providers are one-line subclasses.
- **TTS container honesty.** `synthesize` (`minimax_tts.py:752`) reports the actual container format (`OGG` request → `MP3` response) on the returned `TTSAudio`, so the transcoder never dispatches on a lie.
- **TTL-safe `aclose()`.** `MiniMaxTTSProvider.aclose` (`minimax_tts.py:913-920`) is idempotent and only closes clients it owns — caller-owned clients (injected for tests with `MockTransport`) are respected.

---

## 5. Verdict

**APPROVE-WITH-FOLLOWUPS.** The new package's provider contracts are sound, the implementations follow them faithfully, and the conformance suite is solid. The LLM ↔ TTS contract separation (separate `ProviderError` / `TTSError` hierarchies, separate value objects, separate ABCs) is the right call.

Six followups before merge: **F1**, **F2**, **F3** (block merge); **F4**, **F5**, **F6** (fix in this PR or as an immediate follow-up). The remaining F7–F12 are post-merge cleanups.

The PR also delivers:
- A real extension port for TTS providers (`BaseTTSProvider` + `TTSProviderRegistry` + `TTSProviderFactory`).
- A typed error hierarchy that matches the rest of rob_box (no `openai.APIError` / `httpx` leaking to callers).
- A capability-introspection surface that future fallback / factory code can use to pick the right adapter without making a network call.
- An honest multimodal content model that doesn't break text-only callers.

— architect (Hermes Agent)