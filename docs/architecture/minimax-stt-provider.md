# MiniMax STT provider — operator notes (issue #2365, ADR-0124)

> **Audience.** Operator / integrator running the `rob_box_voice` STT
> chain: when MiniMax STT kicks in, what it needs, how to toggle it, and
> what its failures look like in the log.
>
> **Status.** **Live.** Phase 1 (the adapter) merged via PR #2369; Phase 2
> (chain wiring, per-provider budgets, dead-provider cache) via PR #2706,
> with follow-up fixes in PR #2708. Verified on Vision Pi 21.09.2026 —
> see §6.

Related documents:

* **ADR-0124** — `docs/adr/0124-stt-provider-chain-priority.md`
  (chain order, dead cache, config SSoT). **Supersedes ADR-0091
  §2.2/§2.3/§5.**
* ADR-0091 — `docs/adr/0091-minimax-stt-provider.md` (provider contract,
  diarization side-channel; chain sections superseded)
* ADR-0108 — empty-text semantics
* Config — `src/rob_box_voice/config/stt_node.yaml` and
  `docker/vision/config/voice_assistant/stt_node.yaml`.
  There is **no** `stt_chain.yaml` — it was deleted in ADR-0124 §2.6.
* LLM-side MiniMax operator guide — `docs/guides/MINIMAX.md`
* Unit tests — `src/rob_box_voice/test/unit/stt/test_minimax_provider.py`

## 1. Where MiniMax sits in the chain

Order is `minimax → yandex → vosk` — cloud first for quality, local last
as the thing that always works:

| # | Provider | Wrapper | Mode | Timeout / retries | Notes |
|---|----------|---------|------|-------------------|-------|
| 1 | **MiniMax** | `MiniMaxSTTProvider` | `POST https://api.minimax.io/v1/speech_to_text` (HTTPS multipart) | 5.0 s / 1 | Punctuated, capitalised text; diarization available but not yet consumed |
| 2 | Yandex gRPC v3 | `stt_node._recognize_yandex` | Yandex Cloud STT streaming | 12.0 s / 1 | Gives `speaker_tag` (issue #1077) |
| 3 | Vosk | `stt_node._recognize_vosk` | local CPU, no network | — / 0 | Offline last resort, no quotas, no diarization |

`vosk` is **always** forced to the end of the chain regardless of
configuration (`_normalize_provider_chain`): it is the only provider that
works with no network and no money, so no config change may make the
robot deaf.

## 2. Configuration

All keys are ROS parameters declared in `stt_node.py` and mirrored in
`stt_node.yaml`. Issue #1004 forbids YAML keys that aren't declared, and
ADR-0124 §2.6 forbids a second YAML source for the same value.

### 2.1 Chain

```yaml
stt_provider_chain: [minimax, yandex, vosk]
```

### 2.2 MiniMax

| Parameter | Default | Purpose |
|---|---|---|
| `minimax_stt_enabled` | `true` | Hard off-switch |
| `minimax_stt_api_key_env` | `MINIMAX_API_KEY` | Which env var holds the key |
| `minimax_stt_api_key` | `""` | Fallback if the env var is unset (ENV wins) |
| `minimax_stt_base_url` | `https://api.minimax.io` | Override for tests |
| `minimax_stt_model` | `asr-1.0` | **Plan-dependent** — see §6 |
| `minimax_stt_language` | `ru` | Empty → server auto-detects |
| `minimax_stt_timeout_s` | `5.0` | Soft timeout per call |
| `minimax_stt_max_retries` | `1` | Retries on transient failure |

Without a key the provider is skipped silently — no warning, no retry.
That is the intended way to run a robot without MiniMax.

### 2.3 Dead-provider cache

Shared behaviour with TTS (issue #1083) and LLM (issue #1082): a failing
provider is skipped until its TTL expires, instead of costing a timeout
on every phrase.

| Parameter | Default | Applies to |
|---|---|---|
| `provider_dead_ttl_s` | `300.0` | Quota / plan / key (`2056`, `2061`, 401/403/429, gRPC `PERMISSION_DENIED`, `RESOURCE_EXHAUSTED`) |
| `provider_dead_ttl_transient_s` | `30.0` | Network, 5xx, timeout, `DEADLINE_EXCEEDED` |
| `provider_state_file` | `/data/stt_provider_state.json` | Survives node restarts; `""` disables |

Rules worth knowing as an operator:

* A **successful** recognition clears the mark — top up the balance and
  the provider returns on its own, no restart needed (observed live, §6).
* `empty` and `low_confidence` do **not** mark a provider dead: the cloud
  answered, it just had nothing to say.
* If **every** provider is marked dead, the cache is ignored and the full
  chain runs. A deaf robot is worse than a slow one.

## 3. Reading the log

One summary line plus one line per attempt:

```
[stt_attempt] minimax:ok(3159ms 'Робот, расскажи анек') -> accepted 'Робот, расскажи анекдот.'
[stt_attempt_metric] provider=minimax reason=ok latency_ms=3159 attempt=0 text='Робот, расскажи анекдот.'
```

When a cloud is down:

```
[stt_attempt] minimax:dead(0ms)->yandex:dead(0ms)->vosk:ok(1848ms 'провод ты меня слыши') -> accepted
[stt_attempt_metric] provider=minimax reason=dead latency_ms=0 attempt=0 text=- error='dead 279s more: STTQuotaError(...)'
```

`reason` values: `ok`, `empty`, `low_confidence`, `timeout`, `error`,
`dead` (skipped by the cache). **`error=` always carries the provider's
own message** — read it before guessing at the cause.

The effective provider is logged whenever it changes, and mirrored in the
state file:

```
🎧 STT provider → 'vosk' (chain=['minimax','yandex','vosk'],
   dead={'minimax': 293.5, 'yandex': 294.5}, reason=recognize, last_attempt=vosk)
```

```bash
docker exec voice-assistant cat /data/stt_provider_state.json
```

There is deliberately **no** `/voice/stt/provider_state` topic: nothing
subscribes to one, and an unconsumed topic is what the
`seam_without_consumer` guard (issue #2118) exists to catch. ADR-0124
§2.5 has the payload shape ready if a consumer ever appears.

## 4. Toggling MiniMax

```bash
# Off, keeping the key (ROS parameter):
#   minimax_stt_enabled: false   in stt_node.yaml
#
# Off by removing the key — the chain skips it silently:
unset MINIMAX_API_KEY

# Reorder without touching code:
#   stt_provider_chain: [yandex, minimax, vosk]
```

`stt_provider_chain` accepts any order; unknown names are dropped with a
warning, duplicates collapse, and `vosk` is moved to the end. A chain of
exactly `[vosk]` is a legitimate offline-only mode.

## 5. Tests

54 pure-Python cases, no network, no ROS2, no audio hardware:

```bash
# From src/rob_box_voice
pytest test/unit/stt/test_minimax_provider.py -v

# Chain, dead cache and error mapping
pytest test/test_stt_dead_cache.py test/test_stt_node_fallback.py -v
```

Coverage includes the **real response bodies captured from the robot**
(`TestQuotaInResponseBody.PLAN_500` / `QUOTA_200`), the chain-order
invariants, dead-cache TTL classification, and gRPC status mapping.

`tools/mock_minimax_server.py` provides a canned HTTP server for
end-to-end checks outside the unit suite.

## 6. Troubleshooting

MiniMax reports **application errors in the response body, not in the
HTTP status**. Both shapes below are parsed and classified as a plan /
quota problem (`QUOTA_HINTS` in `minimax_provider.py`):

| What you see | Meaning | What to do |
|---|---|---|
| HTTP 500, `"your current token plan not support model, asr-1.0 (2061)"` | The **plan** does not include that model. Topping up the balance may not be enough. | Upgrade the plan, or point `minimax_stt_model` at a model the plan includes (ROS parameter — no rebuild). |
| HTTP 200, `base_resp.status_code=2056` "Token Plan usage limit reached" | Quota exhausted | Top up. The provider returns by itself within `provider_dead_ttl_s`. |
| `reason=dead` for minutes on end | Working as designed — the cache is skipping a known-dead cloud | `error=` in the same line names the original failure. |
| `minimax:timeout` | Phrase exceeded `minimax_stt_timeout_s` | Live latency is ~3.1 s on a normal phrase against a 5 s budget; long phrases can exceed it. Raise `minimax_stt_timeout_s`. |
| `MiniMaxSTTProvider.maybe_from_env()` → `None` | `MINIMAX_API_KEY` empty | Intentional, not a bug. |
| API key visible in logs | Should never happen — `MiniMaxSTTRedactedLogFilter` covers the module and `httpx` loggers | File an issue and roll the key. |

Probing the endpoint directly from the robot is the fastest way to
separate "our code" from "their service":

```bash
docker exec voice-assistant python3 - <<'PY'
import io, os, wave, httpx
buf = io.BytesIO()
with wave.open(buf, "wb") as w:
    w.setnchannels(1); w.setsampwidth(2); w.setframerate(16000)
    w.writeframes(b"\x00\x00" * 16000)
r = httpx.post("https://api.minimax.io/v1/speech_to_text",
               headers={"Authorization": "Bearer " + os.environ["MINIMAX_API_KEY"]},
               files={"file": ("audio.wav", io.BytesIO(buf.getvalue()), "audio/wav")},
               data={"model": "asr-1.0", "response_format": "json", "language": "ru"},
               timeout=20.0)
print(r.status_code, r.text[:400])
PY
```

A healthy answer for silence is `200 {"text":"","duration":1,...}`.

### 6.1 Verified on Vision Pi, 21.09.2026

With the plan disabled, then re-enabled mid-session, the chain behaved as
designed with no restart:

```
17:01  minimax reason=error  error='STTQuotaError(...not support model, asr-1.0 (2061))'
17:01  minimax reason=dead   error='dead 293s more: ...'
       ... TTL expired, provider re-probed ...
17:10  minimax reason=ok     latency_ms=3093
```

The state file went back to `{"provider": "minimax", "dead_providers": {}}`
on the first success. Wake-word routing works on MiniMax's punctuated,
capitalised output (`🎯 Wake word detected: "Робот, расскажи анекдот."`).

Note that Yandex was simultaneously returning `PERMISSION_DENIED` on
folder `b1gfmjogjodcgff82pjd` (archived) for both STT and TTS — so the
middle of the chain was empty and the fallback went straight to Vosk.

## 7. What is **not** in this doc

* `STTResult` / `STTSegment` and the extended Protocol —
  `docs/architecture/stt-provider-contract.md` (not implemented; the
  chain did not need them).
* Diarization / `/voice/stt/segments` — ADR-0091 §2.4, no consumer yet.
* LLM-side MiniMax (text + vision + tools) — `docs/guides/MINIMAX.md`.
* Unifying the STT / TTS / LLM health caches — issue #2702.

## 8. References

* MiniMax Speech-to-Text API:
  <https://platform.minimax.io/docs/api-reference/speech-to-text>
* Issue #2365 — original feature request
* ADR-0002 — capability-segregated MiniMax design (LLM-side)
* ADR-0091 — provider contract (chain sections superseded by ADR-0124)
* ADR-0124 — chain order, per-provider budgets, dead cache
* PR #2369 — Phase 1 adapter; PR #2706 — Phase 2 wiring;
  PR #2708 — body-level error classification and gRPC stream fix
