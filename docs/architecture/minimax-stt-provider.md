# MiniMax STT provider — operator notes (issue #2365, ADR-0091)

> **Audience.** Operator / integrator running the `rob_box_voice` STT
> chain and wanting to know when MiniMax STT kicks in, what env
> variables / keys it needs, and how to toggle it on or off.
>
> **Status.** Phase 1 PoC merged via PR #2369 (commit `490918d1f`,
> Sep 2026). Phase 2 wiring (`stt_node._recognize_with_fallback`,
> ROS parameters) is **not** in the merge — see §4.

Related documents:

* ADR-0091 — `docs/adr/0091-minimax-stt-provider.md`
* STT provider contract — `docs/architecture/stt-provider-contract.md`
* LLM-side MiniMax operator guide — `docs/guides/MINIMAX.md`
* Config reference — `src/rob_box_voice/config/stt_chain.yaml`
* Unit tests — `src/rob_box_voice/test/unit/stt/test_minimax_provider.py`

## 1. What MiniMax brings to the chain

After ADR-0091 the STT chain has **three** providers. The Phase 2 order
is `vosk → minimax → yandex` (offline-first, then cloud, then primary):

| Position | Provider | Class / wrapper | Endpoint / mode | What it gives |
|----------|----------|-----------------|------------------|---------------|
| 1 (offline) | Vosk | `stt_node._recognize_vosk` | local CPU, no network | Fastest (~0.3–0.8 s), no quotas, no diarization |
| 2 (cloud + diarization) | **MiniMax** | `MiniMaxSTTProvider` | `POST https://api.minimax.io/v1/speech_to_text` (HTTPS, multipart) | Streaming + speaker diarization (`segments[*].speaker`) |
| 3 (primary cloud) | Yandex gRPC v3 | `stt_node._recognize_yandex` | Yandex Cloud STT streaming | Highest quality for `ru-RU`, soft-timeout 12 s |

The current Phase 1 deployment still uses the pre-ADR order (Yandex
primary + Vosk fallback). The MiniMax class is **shipped but not yet
inserted into the chain** — flipping the chain is Phase 2.

## 2. Configuration

### 2.1 Required environment variables

| Variable | Required? | Purpose |
|----------|-----------|---------|
| `MINIMAX_API_KEY` | Yes, to enable | Bearer token for `POST /v1/speech_to_text` |
| `MINIMAX_API_BASE_URL` | No | Override `https://api.minimax.io` (tests only) |

If `MINIMAX_API_KEY` is **unset**, `MiniMaxSTTProvider.maybe_from_env()`
returns `None` and the chain skips MiniMax silently — no warning, no
retry, no ROS noise. This is intentional so that pre-ADR chains keep
running without touching configuration.

### 2.2 Optional parameters

The constructor in `stt_providers/minimax_provider.py` accepts:

* `base_url` — defaults to `https://api.minimax.io`.
* `model` — defaults to `asr-1.0` (MiniMax-M3 STT).
* `language` — defaults to `"ru"` for rob_box's Russian-first chain;
  set to `None` to let MiniMax auto-detect.
* `timeout` — default `connect=5.0, read=15.0, write=10.0, pool=5.0`.

All values used today are in module-level constants and overridable
through the constructor or `maybe_from_env(**kwargs)`.

### 2.3 ROS parameters (Phase 2, not live yet)

Per `docs/adr/0091-minimax-stt-provider.md` §3 and issue #1004, Phase 2
will declare these ROS parameters in `stt_node` via `declare_parameter`:

* `minimax_stt_enabled` (bool, default `False` until parity proven)
* `minimax_stt_api_key_env` (string, default `"MINIMAX_API_KEY"`)
* `minimax_stt_position` (string, default `"between"`)

`src/rob_box_voice/config/stt_node.yaml` already carries a comment
placeholder for these flags; the keys themselves are intentionally
**not** declared in Phase 1 (issue #1004 forbids YAML keys that aren't
declared in `declare_parameter`).

## 3. Capability summary (when to prefer MiniMax)

Mirrors the `MiniMaxSTTProvider` class docstring; kept here so an
operator can decide without diving into Python.

Prefer MiniMax when **one or more** of these hold:

* **Cloud is acceptable** — outbound HTTPS + `MINIMAX_API_KEY`.
* **Speaker diarization needed** (issues #2346, #2348) — MiniMax returns
  per-utterance `speaker` labels inside `segments[*]`. Vosk returns
  none; Yandex gives a single `speaker_tag` per utterance.
* **Latency-sensitive barge-in** but Vosk is too noisy — MiniMax comes
  in at ~800–1300 ms per call (see probe in `stt_fallback.py`), much
  faster than Yandex's 1500–4000 ms under load.
* **Cost-sensitive cloud path** — cheaper than Yandex for short
  utterances (see `asr-1.0` pricing).

Avoid MiniMax when:

* Hot path requires strict offline operation → use Vosk first.
* Only Russian-language recognition is needed and Yandex parity
  numbers exist → Yandex remains primary.

## 4. How to toggle MiniMax on / off

Today (Phase 1) the toggle is **environment-level only**:

```bash
# Enable MiniMax STT in the chain (Phase 2 wiring picks it up)
export MINIMAX_API_KEY="sk-..."

# Disable without touching code — leave the variable empty
unset MINIMAX_API_KEY        # or: export MINIMAX_API_KEY=""
```

After Phase 2 lands, the same toggle will be exposed via ROS parameter
(see §2.3). Until then, `maybe_from_env()` and `stt_chain.yaml` are the
canonical knobs.

To disable permanently (eg. an air-gapped robot), simply do not export
`MINIMAX_API_KEY`. The constructor refuses to instantiate without the
key, so the chain falls back to Vosk / Yandex as before.

## 5. Tests

Unit tests live in `src/rob_box_voice/test/unit/stt/test_minimax_provider.py`
(43 cases per PR #2369). They are pure-Python, no network, no ROS2,
no audio hardware:

```bash
# From src/rob_box_voice (uses package pytest.ini)
pytest test/unit/stt/test_minimax_provider.py -v

# Whole unit directory (fast, CI-safe)
pytest test/unit -v

# Coverage of the new module
pytest test/unit/stt/test_minimax_provider.py \
    --cov=rob_box_voice.stt_providers.minimax_provider \
    --cov-report=term-missing
```

What the suite covers:

* `recognize()` returns text on 200/JSON; returns `None` on 401 / 403 /
  429 / 5xx / timeout / non-JSON / missing `text` field.
* `MiniMaxSTTProvider.maybe_from_env()` returns `None` without the
  env-var key.
* Empty / oversized audio_bytes are rejected before any HTTP call.
* `_extract_text` understands both response shapes
  (`{"text": ...}` and `{"data": {"text": ...}}`).
* `name` is stable (`PROVIDER_NAME == "minimax"`) so dashboards
  keyed on it don't break.

If you need a fake HTTP server for an end-to-end check (outside the
unit suite), see `tools/mock_minimax_server.py` — a minimal aiohttp
app that returns canned `verbose_json` payloads.

## 6. Troubleshooting

| Symptom | First thing to check |
|---------|----------------------|
| `MiniMaxSTTProvider.maybe_from_env()` returns `None` | `echo "$MINIMAX_API_KEY"` — empty means it returns `None` (intentional, not a bug). |
| Construction raises `MiniMaxSTTUnavailableError` | Same root cause as above. |
| 401/403 in logs | Key has been revoked or restricted. Rotate in MiniMax console; restart `stt_node`. |
| 429 / latency spikes | Yandex / MiniMax quotas may be sharing a project. Back off, or temporarily disable the provider (`unset MINIMAX_API_KEY`). |
| `MINIMAX_API_KEY=eyJh...` appears in logs | Should never happen — `MiniMaxSTTRedactedLogFilter` is attached to the module + `httpx` loggers. If it does, file an issue and roll the key. |
| Phase 2 wiring questions (chain order, ROS params) | Re-read `docs/adr/0091-minimax-stt-provider.md`; do not edit `stt_node.yaml` until Phase 2 lands (issue #1004). |

## 7. What is **not** in this doc

This file is intentionally operator-focused. The following live
elsewhere and are linked at the top:

* STTResult / STTSegment value objects and the extended
  `STTProvider` Protocol — `docs/architecture/stt-provider-contract.md`.
* The ADR that fixed the chain order and the diarization side-channel
  invariant — `docs/adr/0091-minimax-stt-provider.md`.
* LLM-side MiniMax (text + vision + tools) — `docs/guides/MINIMAX.md`.

## 8. References

* MiniMax Speech-to-Text API reference:
  <https://platform.minimax.io/docs/api-reference/speech-to-text>
* Issue #2365 — original feature request for MiniMax STT.
* ADR-0002 — capability-segregated MiniMax design (LLM-side).
* ADR-0091 — MiniMax STT chain design and choice of provider order.
* PR #2369 — Phase 1 implementation (43 unit tests, env-driven
  factory, redaction filter, chain config stub).
