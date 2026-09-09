# TTS interface — current as-is snapshot (PR #907, MiniMax T2A v2 integration)

**Project:** rob_box_project (`krikz/rob_box_project`)
**Document type:** Technical reference / as-is snapshot (input for the architect)
**Branch under review:** `feature/harness-p0-foundation` (`@ fa3f8409` and onward through `9bc7219b`)
**Authors' premise:** reverse-engineer the **current** TTS interface so the next planning round can decide what to keep / replace / refactor.
**Author:** developer (Hermes Agent) — Kanban task `t_6185f757`.
**Source-tree exact path:** `/home/builder/hermes-share/rob_box_project/.worktrees/t_ac5f796b`.
**Re-validates against:**
- ADR `0002-minimax-provider.md` (LLM/image scope)
- ADR `0003-minimax-tts-architecture.md` (TTS-specific architecture)
- `docs/architecture/minimax-tts-architecture.md` (implementation detaill)
- `docs/analysis/nodes-current-state.md` and `docs/analysis/current-nodes.md` (cross-node narrative)

> **Reading order.** §1 = where the contract lives and how it's split.
> §2 = how providers are wired in. §3 = the four ROS touch-points inside the node.
> §4 = audio shapes and transports. §5 = config. §6 = what's *not* there yet (open questions for the architect).

---

## 1. `TTSProvider` contract — base class + value objects

### 1.1 Where it lives

| Symbol | File | Lines |
|---|---|---|
| `TTSProvider` (ABC) + `TTSFormat` enum | `src/rob_box_llm/rob_box_llm/tts.py` | 33-44, 121-177 |
| Value objects `TTSSettings`, `TTSAudio`, `TTSChunk` | same | 52-113 |
| `FakeTTSProvider` (test in-memory impl) | same | 185-245 |
| Public re-exports | `src/rob_box_llm/rob_box_llm/__init__.py` | 56-63 |
| Errors | `src/rob_box_llm/rob_box_llm/errors.py` | 63-105 |

`tts.py:1-22` (module docstring) is the contract blueprint; the
following fragments distil what's enforceable.

### 1.2 Abstract surface — two methods, async-only

```python
# src/rob_box_llm/rob_box_llm/tts.py:121-177
class TTSProvider(abc.ABC):
    name: str = "abstract"

    @abc.abstractmethod
    async def synthesize(self, text: str, *, settings: TTSSettings | None = None) -> TTSAudio: ...

    @abc.abstractmethod
    async def stream(self, text: str, *, settings: TTSSettings | None = None) -> AsyncIterator[TTSChunk]: ...

    async def aclose(self) -> None: ...   # default no-op
```

The contract rules baked into docstrings:

- **Fail-fast before the first chunk.** Pre-yield errors raise a
  `TTSError` subclass; post-yield errors become a terminal
  `TTSChunk(finish_reason="error")` (`tts.py:19-22`, `tts.py:151-167`).
- **Always emit a terminal chunk.** Every `stream()` ends with a
  `TTSChunk` carrying `finish_reason="stop"` (or `"error"`) so callers
  can detect end-of-stream unambiguously (`tts.py:103-113`,
  `tts.py:155-167`).
- **v1 streaming accepts one terminal chunk.** Real fixed-size frame
  streaming would require MiniMax's T2A WebSocket endpoint (out of
  scope here; tracked in ADR-0003 §2.4 as M5/M6 — `tts.py:160-167`).

### 1.3 Value objects — frozen dataclasses

```python
# src/rob_box_llm/rob_box_llm/tts.py:52-113
@dataclass(frozen=True)
class TTSSettings:
    model:            Optional[str]   = None   # "speech-02-hd" | "speech-02-turbo"
    voice:            Optional[str]   = None   # provider-specific id
    language:         Optional[str]   = None   # BCP-47 ("ru","en") or full ("Russian")
    speed:            Optional[float] = None   # 0.5 – 2.0
    volume:           Optional[float] = None   # provider-specific scale
    pitch:            Optional[int]   = None   # semitone offset
    emotion:          Optional[str]   = None   # "happy" | "neutral" | "sad"
    sample_rate:      Optional[int]   = None   # Hz — provider default if None
    format:           TTSFormat       = TTSFormat.PCM
    text_normalization: Optional[bool]= None
    extra:            Mapping[str, Any] = field(default_factory=dict)
                                                    # frozen as MappingProxyType

@dataclass(frozen=True)
class TTSAudio:
    samples:     bytes                  # int16 LE unless format != PCM
    sample_rate: int                    # Hz
    format:      TTSFormat = TTSFormat.PCM
    raw:         Any | None = None      # original provider response

@dataclass(frozen=True)
class TTSChunk:
    samples:       bytes = b""
    sample_rate:   int   = 0
    format:        TTSFormat = TTSFormat.PCM
    finish_reason: Optional[str] = None  # "stop" | "error"
```

`TTSFormat` is a `str` enum (`tts.py:33-44`) with members `PCM | WAV | MP3 | OGG`. It
inherits from `str` so it serialises cleanly to JSON and round-trips
through `TTSFormat(value)`.

`TTSAudio.duration_s` returns `len(samples)/(sample_rate*2)` only for
PCM (`tts.py:91-99`); compressed formats yield `0.0` and must be
decoded externally.

### 1.4 Implementation inventory shipped today

| Provider | Module | Notes |
|---|---|---|
| `MiniMaxTTSProvider` | `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py` (673 LOC) | HTTP/SSE to `https://api.minimax.io/v1/t2a_v2`; Bearer auth + `GroupId` query; hex-encoded audio in JSON envelope |
| `FakeTTSProvider` | same file `tts.py:185-245` | Deterministic in-memory PCM buffer (length proportional to input bytes, constant value 1); used by unit tests |

Exported through `rob_box_llm/__init__.py:69-70, 96-97` and
`rob_box_llm/providers/__init__.py:12, 23`. **There is no provider
registry / factory function** — consumers `import MiniMaxTTSProvider`
directly. This is the only registration mechanism today; matching
`LLMProvider` analogues (`docs/analysis/current-nodes.md` §2.1, smell D1/X-1) — the duplication is structural, not accidental.

### 1.5 The MiniMax-specific HTTP shaping

Key bits worth highlighting because they constrain what *any* TTS
provider must do for this project:

| Aspect | Value | Source |
|---|---|---|
| Endpoint | `POST {base_url}/v1/t2a_v2` (`base_url` = `https://api.minimax.io` default) | `minimax_tts.py:327, 392, 555` |
| Auth | `Authorization: Bearer <MINIMAX_API_KEY>` header **and** `GroupId=<MINIMAX_GROUP_ID>` query | `minimax_tts.py:372-389` |
| Request body | `{model, text, stream, voice_setting{voice_id,speed,vol,pitch,emotion,language}, audio_setting{sample_rate,bitrate,format,channel}, text_normalization?}` | `_build_payload()` at `minimax_tts.py:131-233` |
| Response envelope | `{data:{audio:<hex>, audio_sample_rate}, base_resp:{status_code, status_msg}}`; `status_code != 0` ⇒ API-level error | `minimax_tts.py:428-449, 589-609` |
| Audio payload | hex-encoded; the provider decodes to raw `bytes` and reports `TTSAudio.samples` | `_decode_audio()` at `minimax_tts.py:457-475` |
| `OGG` requested | silently downgraded to `"mp3"` on the wire and re-flagged in `TTSAudio.format` so downstream decoders never dispatch on a false marker | `minimax_tts.py:186-188, 502, 624, 659` |

Two security/observability details a future provider should copy:

- **Secret redaction.** A `_RedactGroupIdFilter` is attached to the
  `httpx` logger so the `GroupId` query parameter is masked on the way
  through access logs (`minimax_tts.py:271-296`). Plus
  `_redact_sensitive_text` blanks `api_key` / `group_id` from any
  exception string before it leaves the provider
  (`minimax_tts.py:81-86, 421-435, 583-594`).
- **Allow-list + reserved set on `settings.extra`.**
  `_ALLOWED_EXTRA_KEYS` permits 9 documented forward-compat keys; `_RESERVED_PAYLOAD_KEYS` (6 keys) raises `TTSBadRequestError` to
  prevent callers from overwriting typed top-level fields via
  untrusted `extra` (`minimax_tts.py:210-232, 241-268`).
- **Fail-fast validation.** `volume ∉ [0.0, 10.0]` raises
  `TTSBadRequestError` rather than letting MiniMax return a 400
  (`minimax_tts.py:160-170`).

### 1.6 Error hierarchy

```text
TTSError (base, .provider)
├── TTSAuthError        (401/403 + heuristic "auth/key/token" in base_resp.status_msg)
├── TTSBadRequestError  (other 4xx + heuristic "invalid/param/voice" + local validation failures)
├── TTSRateLimitError   (429 + heuristic "quota/rate/limit")
└── TTSTimeoutError     (httpx.TimeoutException / ConnectError)
```

Defined in `src/rob_box_llm/rob_box_llm/errors.py:63-90`. Mapping from
exceptions lives in `minimax_tts.py:_map_exception`, 89-128, plus the
heuristic on `base_resp.status_msg` at `minimax_tts.py:441-449`.

---

## 2. Where the providers live in the "fabric" — registration today

> **TL;DR.** There is *no* registry, factory, or DI container. Each
> consumer `import`s and instantiates the concrete class directly.

Concrete call sites in the repo today:

| Site | Import | Use |
|---|---|---|
| `src/rob_box_voice/rob_box_voice/tts_node.py:59-77` | `from rob_box_llm import MiniMaxTTSProvider, TTSSettings, TTSFormat` (lazy `try/except ImportError`) | Wraps provider for opt-in ROS-params mode |
| `src/rob_box_llm/test/test_tts_conformance.py` | `from rob_box_llm import FakeTTSProvider` | Conformance suite — exercises both providers against the same protocol |
| `src/rob_box_llm/test/test_minimax_tts*.py` | `from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider` | Provider-level coverage with `httpx.MockTransport` |

See `docs/analysis/current-nodes.md` §2.1 / D1 for the broader
duplication argument and ADR-0001 for why P0 chose a tiny ABC over a
factory. Practical consequence: adding a 3rd provider requires touching
*both* `__init__.py` re-exports (`rob_box_llm/__init__.py` and
`providers/__init__.py`) **and** the consumer in `tts_node.py`.

---

## 3. ROS touch-points inside `tts_node`

`tts_node` is the only TTS-aware ROS node in the codebase
(`src/rob_box_voice/rob_box_voice/tts_node.py`, 1526 LOC). ROS surface
is symmetrical to the other persistent nodes (see
`docs/analysis/current-nodes.md` row 3 for the same table).

### 3.1 Subscribers (`tts_node.py:322-336`)

| Topic | Type | QoS | Callback |
|---|---|---|---|
| `/voice/dialogue/response` | `std_msgs/String` | depth 10 | `dialogue_callback` (chunk dispatcher) |
| `/voice/tts/request` | `std_msgs/String` | depth 10 | same callback (alias used by `reflection_node`/external triggerers) |
| `/voice/tts/control` | `std_msgs/String` | depth 10 | `control_callback` (`"STOP"` interrupts playback) |
| `/voice/current_dialogue_id` | `std_msgs/String` | depth 1 (KEEP_LAST) | `_on_new_dialogue_id` (cancels stale TTS after barge-in) |

### 3.2 Publishers (`tts_node.py:339-358`)

| Topic | Type | QoS |
|---|---|---|
| `/voice/audio/speech` | `audio_common_msgs/AudioData` | `KEEP_LAST depth=audio_qos_depth`; `audio_qos_reliability` ∈ `{best_effort, reliable}` (default `best_effort`); `VOLATILE` durability |
| `/voice/tts/state` | `std_msgs/String` | depth 10; values: `ready` / `synthesizing` / `playing` / `stopped` |
| `/voice/tts/finished` | `std_msgs/String` | depth 10; JSON `{speech_id, success, error?}` for MCP tools / `animation_player` |

> **AudioData carries no metadata.** `uint8[] data` is the entire
> message; sample rate and channel count are out-of-band
> (`audio_output_sample_rate` ROS param, `channels = 1` hard-coded).
> See ADR `minimax-tts-architecture.md` §4.3.

### 3.3 Provider factory — switch happens in `_synthesize_and_play`

```python
# src/rob_box_voice/rob_box_voice/tts_node.py:740-756
result = {}
if self.provider == "minimax":
    self.publish_state("synthesizing")
    if self.minimax_streaming:
        result = self._synthesize_minimax_streaming_publish(text, ssml_attributes)
    else:
        result = self._synthesize_minimax(text, ssml_attributes)
    audio_np = result["audio_np"]; sample_rate = result["sample_rate"]
elif self.yandex_stub:           # Yandex gRPC path (primary for ROBBOX voice)
    ...
# Silero v5 fallback only fires when both yandex+silero paths are picked
# AND yandex is unavailable; MiniMax path NEVER falls back to Silero —
# the caller chose MiniMax explicitly.
```

So the four entry points from a callback to an audio buffer are:

```
dialogue_callback(msg: String)            # tts_node.py:524-613
  └── _run_synthesis_worker(...)          # tts_node.py:689-710  (offloads to Thread, holds _synthesis_lock)
        └── _synthesize_and_play(...)      # tts_node.py:712-983
              ├── self.provider == "minimax"
              │     ├── minimax_streaming == False
              │     │     └── _synthesize_minimax(...)     # tts_node.py:1329-1352
              │     │           └── _synthesize_minimax_with_retry(...)   # tts_node.py:1201-1258
              │     │                 └── _synthesize_minimax_async(...)  # tts_node.py:1118-1199
              │     │                       └── MiniMaxTTSProvider.synthesize(text, settings)
              │     │                       └── _decode_minimax_audio(...)# tts_node.py:1081-1098
              │     │                             └── utils.audio_transcode.to_pcm_int16(...)
              │     │                       └── TTSAudio → np.float32 mono [-1, 1]
              │     └── minimax_streaming == True
              │           └── _synthesize_minimax_streaming_publish(...) # tts_node.py:1260-1327
              │                 └── _stream_minimax_chunks(...)           # tts_node.py:1354-1403 (async gen)
              │                       └── MiniMaxTTSProvider.stream(text, settings)
              │                              (publishes each chunk to /voice/audio/speech,
              │                               then concatenates for local playback)
              ├── self.yandex_stub and provider=="yandex"
              │     └── _synthesize_yandex(...) # tts_node.py:985-1060  (gRPC, anton voice)
              └── provider=="silero" or yandex failed
                    └── silero_model.apply_tts(...)  # tts_node.py:801-815
```

### 3.4 Audio post-processing & playback

After the provider returns a `np.float32` mono buffer, the same
post-pipeline runs for every branch:

```
audio_np @ sample_rate  ─►  _prepare_audio_for_topic()   # tts_node.py:1405-1419
                            resample → audio_output_sample_rate (default 16000)
                          ─►  _publish_audio()             # tts_node.py:1421-1433
                            float32 → int16 LE bytes
                            → AudioData().data = list(bytes)
                            → publish(/voice/audio/speech, qos)
                          ─►  chipmunk_mode on?
                              ├─ True:  resample sr → sr/(2 * pitch_shift) → resample → 16000
                              │        (reproduces the 44100/22050 = 2x ROBBOX chipmunk)
                              └─ False: resample sr → 16000
                          ─►  apply volume_gain (10^(volume_db/20))
                          ─►  mono → stereo  np.column_stack([x, x])
                          ─►  AudioPlaybackManager.play_audio(..., blocking=True, timeout=5.0)
```

`_run_synthesis_worker` offloads the above onto a daemon
`threading.Thread` and serialises everything behind
`self._synthesis_lock` so the ROS executor's `dialogue_callback` /
`control_callback` stay responsive to barge-in
(`tts_node.py:365-365, 603-608, 689-710`).

`cleanup_playback_noise` (`tts_node.py:1449-1473`) ends each playback
with `sd.stop()` + a 100 ms sleep to drain the ReSpeaker USB buffer
(USB Audio Class 1.0 quirk noted in the docstring).

---

## 4. Audio formats — what is on the wire

| Layer | Container | Bit-depth | Channels | Sample rate | Notes |
|---|---|---|---|---|---|
| **MiniMax HTTP response** | hex-encoded PCM / WAV / MP3 / (OGG→MP3 fallback) | int16 | mono (1) | 8k / 16k / 22.05k / 24k / 32k / 44.1k (requested; default 32k) | `minimax_tts.py:182-189, 457-475` |
| **`TTSAudio.samples`** | raw `bytes`, container as requested | int16 LE PCM unless `format != PCM` | depends on provider | provider's reported `sample_rate` | frozen dataclass — `tts.py:80-99` |
| **`utils/audio_transcode.DecodedAudio.pcm`** | int16 LE PCM, no header | 16 | mono (downmixed if WAV had >1 ch) | from container if present, else the caller's `default_sample_rate` | `utils/audio_transcode.py:60-128` |
| **`np.float32 audio_np`** | floats in [-1, 1] | 32 | mono | provider SR | `tts_node.py:1063-1079` |
| **`/voice/audio/speech` AudioData** | `uint8[] data` carrying int16 LE PCM | 16 | mono (1) — `audio_channels=1` hard-coded | fixed by `audio_output_sample_rate` (default 16000); metadata out-of-band | `_publish_audio()` at `tts_node.py:1421-1433` |
| **ReSpeaker USB sink** | int16 LE | 16 | **stereo (2)** | **16000** only | reproduced on `tts_node.py:836-893`; chipmunk mode may resample via `effective_rate` before the final 16 kHz stage |

Container handling matrix — `utils/audio_transcode.py`:

| `TTSFormat` | Decoder strategy |
|---|---|
| `PCM` | Pass-through (alignment to 16-bit is validated; no channel strip) — `audio_transcode.py:105-121` |
| `WAV` | `wave.open` (stdlib); supports `sampwidth ∈ {1,2,3,4}` with bit-conversion down to int16; downmix >2 ch to mono — `audio_transcode.py:123-190, 193-207` |
| `MP3`, `OGG` | Lazy `pydub.AudioSegment.from_file` then `set_channels(1).set_sample_width(2)` → fallback to `ffmpeg -f <fmt> -i pipe:0 -f s16le -acodec pcm_s16le -ac 1 -ar 44100 pipe:1` subprocess (30 s timeout) — `audio_transcode.py:210-281` |

`package.xml` `exec_depend`s `ffmpeg` but **not** `pydub` — voice image
guarantees the binary, minimal developer envs only hit PCM/WAV paths.
The transcoder is intentionally dependency-light
(`audio_transcode.py:8-12`) so `rob_box_voice` stays importable on
slim containers.

Conversion validation surfaces as `AudioTranscodeError(fmt=..., reason=...)`
(`audio_transcode.py:48-57`); the seven failure tags callers can branch on are:
`empty`, `unaligned_pcm`, `bad_wav`, `unaligned_channels`,
`unsupported_sampwidth`, `no_decoder`, `ffmpeg_failed`,
`ffmpeg_timeout`, `unsupported_format` (last is the defensive guard at
the bottom of `to_pcm_int16`).

### 4.1 Streaming wire shape

`MiniMaxTTSProvider.stream()` consumes SSE lines (`minimax_tts.py:573-588`):

```text
data:{"data":{"audio":"<hex>", "audio_sample_rate":32000}, "base_resp":{"status_code":0, ...}}\n
\n
data:...                                  # one frame per event
\n
```

Each `data:` payload becomes a `TTSChunk(samples=bytes.fromhex(...), sample_rate=...)`. The provider always yields an **empty terminal chunk** with `finish_reason="stop"` after the last audio frame (`minimax_tts.py:656-661`); mid-stream failures after a yield become `TTSChunk(finish_reason="error")` (`minimax_tts.py:606-609, 633-637`).

> **Caveat for the architect.** Today the SSE source buffers the
> complete response server-side (MiniMax T2A v2 SSE), so the provider
> yields exactly **one** terminal chunk in practice. Real chunk-per-frame
> streaming waits on the WebSocket endpoint (ADR-0003 §2.4, M5/M6).
> The downstream `_synthesize_minimax_streaming_publish` therefore
> publishes one `AudioData` per request — *equivalent* in latency to
> the non-streaming path. Both paths share `_decode_minimax_audio` and
> the `_publish_audio` int16 writer
> (`tts_node.py:1283-1302, 1421-1433`).

---

## 5. Configuration today (env / ROS-params / defaults)

### 5.1 ROS parameters declared in `tts_node.__init__`

| Param | Type | Default | Mutable via `parameters_callback`? | Where used |
|---|---|---|---|---|
| `provider` | string | `"yandex"` | no | gate at `tts_node.py:228-233` |
| `yandex_api_key` / `yandex_voice` / `yandex_speed` | string/string/float | `""` / `"anton"` / `1.0` | last one yes | gRPC path |
| `silero_speaker` / `silero_sample_rate` / `silero_put_accent` / `silero_put_yo` / `silero_put_stress_homo` / `silero_put_yo_homo` | string/int/bool×4 | `"baya"` / `48000` / `True`×4 | no | offline fallback |
| `minimax_api_key` / `minimax_group_id` | string | `""` | no (env fallback) | `MiniMaxTTSProvider(...)` ctor at `tts_node.py:1149-1155` |
| `minimax_voice` / `minimax_model` / `minimax_language` | string/string/string | `"male-qn-qingse"` / `"speech-02-hd"` / `"ru"` | no | `TTSSettings(...)` |
| `minimax_speed` (float) / `minimax_sample_rate` (int) / `minimax_timeout` (float) | 1.0 / 32000 / 30.0 | no (or via SSML `rate` at runtime) |
| `minimax_format` | string in `{pcm, wav, mp3, ogg}` | `"pcm"` | no | `_parse_format()` validation → `TTSSettings.format` |
| `minimax_max_retries` (0..3) / `minimax_retry_backoff_ms` | int/int | 2 / 500 | yes (`max_retries` only) | retry loop at `tts_node.py:1201-1258` |
| `minimax_streaming` | bool | `False` | yes | branches `_synthesize_minimax` vs `_synthesize_minimax_streaming_publish` |
| `audio_topic` | string | `"/voice/audio/speech"` | no | publisher topic |
| `audio_output_sample_rate` | int | `16000` | no | `_prepare_audio_for_topic` target |
| `audio_qos_reliability` (∈ `best_effort, reliable`) / `audio_qos_depth` | string/int | `best_effort` / `10` | no | `QoSProfile(...)` |
| `chipmunk_mode` / `pitch_shift` / `normalize_text` / `volume_db` | bool/float/bool/float | True / 1.0 / True / -3.0 | yes (all four) | playback post-pipeline |

30 `declare_parameter` calls live at `tts_node.py:168-225`; their
`get_parameter` reads span `tts_node.py:228-285`. The
`add_on_set_parameters_callback` hook at `tts_node.py:289` is wired to
`parameters_callback` at `tts_node.py:1481-1508` and currently whitelists
only `volume_db`, `pitch_shift`, `chipmunk_mode`, `yandex_speed`,
`minimax_max_retries`, `minimax_streaming` for live edits.

### 5.2 Configuration sources, in priority order

The MiniMax-subset is the only one with a layered scheme (per ADR
`minimax-tts-architecture.md` §2.3 + §3):

```
launch YAML → ROS-params (defaults via `declare_parameter`) → ENV-fallback → secrets file
```

- **Secrets only** (`api_key`, `group_id`) honour ENV (`MINIMAX_API_KEY`,
  `MINIMAX_GROUP_ID`); the path is `value or os.getenv(...)` at
  `tts_node.py:251-252`. Final fallback for those two is `docker/vision/.env.secrets`
  (read-only mount referenced in the architecture doc).
- All other `minimax_*` fields are ROS-only by design (see ADR
  rationale — "ENV fallback for the other 7 parameters was rejected to
  avoid two sources of truth").
- `minimax_speed` admits one runtime override: SSML `<prosody rate=...>`,
  parsed in `_parse_ssml_attributes` at `tts_node.py:623-687`, applied
  in `_synthesize_minimax_async` at `tts_node.py:1158`.

### 5.3 Validation already in place

- `provider ∈ {yandex, silero, minimax}` enforced at node ctor
  (`tts_node.py:228-233`).
- `audio_output_sample_rate > 0` (`tts_node.py:271-272`).
- `audio_qos_reliability ∈ {best_effort, reliable}`
  (`tts_node.py:339-347`); other values raise.
- `minimax_format` parsed through `TTSFormat(value.lower().strip())`
  in `_parse_format` (`tts_node.py:1100-1116`); unknown values raise
  `ValueError` with the allowed-list.
- `minimax_max_retries` clamped to `[0, 3]`, `minimax_retry_backoff_ms`
  clamped to `>= 0` (`tts_node.py:260-263`).
- Provider-side: `volume ∈ [0.0, 10.0]` and reserved-key collision in
  `settings.extra` raise `TTSBadRequestError` *before* the HTTP call —
  fail-fast (`minimax_tts.py:160-170, 226-232`).

### 5.4 Env-vars the node reads explicitly

| Variable | Where read | Notes |
|---|---|---|
| `YANDEX_API_KEY` | `tts_node.py:236` | falls back if `yandex_api_key` ROS-param empty |
| `MINIMAX_API_KEY` | `tts_node.py:251` | falls back if `minimax_api_key` ROS-param empty |
| `MINIMAX_GROUP_ID` | `tts_node.py:252` | falls back if `minimax_group_id` ROS-param empty |

The provider's own ctor (`minimax_tts.py:332-353`) re-reads `os.getenv("MINIMAX_API_KEY")` / `MINIMAX_GROUP_ID` if no keyword argument was passed — that's a **second** layer of fallback that only fires when callers omit the kwargs, which `tts_node` never does.

---

## 6. Open items for the architect (what's *not* there yet)

These came up while reading the code and are not invented — they are
genuine gaps in the current interface:

1. **No `TTSProvider` registry / factory.** Adding a 3rd provider
   means editing both `__init__.py` re-exports and the `provider ==`
   chain in `_synthesize_and_play`. There is no analogue to the LLM
   `PROVIDERS` table either (`docs/analysis/current-nodes.md` smell
   D1/X-1).
2. **No chunk-per-frame stream in practice.** The contract allows it,
   the `stream()` path publishes each chunk to `/voice/audio/speech`,
   but MiniMax SSE always delivers the full payload → one terminal
   chunk. Documented as M5/M6 future work (ADR-0003 §2.4).
3. **No normalised "voice list" capability.** `TTSSettings.voice`
   accepts arbitrary strings; MiniMax `/v1/voices` isn't yet wired
   (ADR section "Alternatives not chosen", §2.3).
4. **No voice-id ↔ human-name mapping layer for MiniMax.** `minimax_voice`
   is passed straight to the API (`tts_node.py:253`,
   `minimax_tts.py:150`); there's no helper like `_LANGUAGE_ALIASES`
   that exists for language.
5. **Two different env-fallback layers.** `tts_node` reads ENV,
   *then* constructs `MiniMaxTTSProvider(api_key=..., group_id=...)`;
   the provider re-reads ENV *only if both kwargs are `None`*. The
   redundancy is intentional but worth flagging.
6. **Retry policy lives in the ROS layer, not the provider.**
   `_synthesize_minimax_with_retry` (`tts_node.py:1201-1258`) consumes
   up to 3 attempts (configurable via `minimax_max_retries`); the
   provider raises immediately. This is deliberate (per ADR-0003 §5)
   but means any new consumer (e.g. `mcp_tools`, a CLI) has to rebuild
   the loop or accept one-shot failures.
7. **No common `aclose()` lifecycle hook for the ROS pool.**
   `close_minimax_provider` (`tts_node.py:1435-1447`) wraps the
   provider's `aclose()` in `asyncio.run` on shutdown; if a 2nd
   opt-in provider is added later, the shutdown path must be
   hand-extended.
8. **AudioData has no metadata.** `channels=1` is hard-coded; sample
   rate is fixed at `audio_output_sample_rate`. A multi-channel
   microphone stack (e.g. ReSpeaker 4-mic) cannot ride the same
   topic — already flagged in ADR `minimax-tts-architecture.md` §4.4.
9. **`stream()` chunk-rate limit absent.** No max-bytes-per-msg, no
   ROS message-too-large guard; relies on MiniMax SSE frame sizes. New
   providers would need to honour this contract surface explicitly.
10. **Tests assume `httpx.MockTransport` + `conftest.py`.** See
    `src/rob_box_llm/test/test_tts_conformance.py` — the conformance
    suite already exercises both `MiniMaxTTSProvider` and
    `FakeTTSProvider` against `TTSProvider` ABC. Adding a 3rd provider
    *must* pass this conformance suite (existing pattern; not invented
    here).

---

## Appendix A — Quick file / line index

| Concern | File | Lines |
|---|---|---|
| `TTSProvider` ABC + value objects | `src/rob_box_llm/rob_box_llm/tts.py` | 33-44, 52-113, 121-177, 185-245 |
| TTS error hierarchy | `src/rob_box_llm/rob_box_llm/errors.py` | 63-105 |
| Public re-exports | `src/rob_box_llm/rob_box_llm/__init__.py` | 56-70 |
| Provider module | `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py` | 1-673 (whole file) |
| Language alias table | `minimax_tts.py` | 53-66 |
| `extra` allow-list / reserved set | `minimax_tts.py` | 241-268 |
| Volume / format / stream shaping | `minimax_tts.py` | 131-189, 481-513, 515-661 |
| Logger redaction filter | `minimax_tts.py` | 271-296 |
| Exception → domain-error mapping | `minimax_tts.py` | 89-128 |
| ROS node entry points | `src/rob_box_voice/rob_box_voice/tts_node.py` | subs 322-336, pubs 339-358, params 166-285 |
| Provider branching | `tts_node.py` | 740-756 |
| MiniMax provider construction | `tts_node.py` | 1148-1155 |
| `TTSSettings` assembly | `tts_node.py` | 1163-1170 |
| Retry wrapper | `tts_node.py` | 1201-1258 |
| Streaming publish wrapper | `tts_node.py` | 1260-1327 |
| Transcoding helpers | `src/rob_box_voice/rob_box_voice/utils/audio_transcode.py` | 48-128 (entrypoints) + fallbacks 210-281 |
| `package.xml` deps for TTS path | `src/rob_box_voice/package.xml` | 23-32 (`httpx`, `ffmpeg`, `rob_box_llm`) |
| ADR for the integration | `docs/adr/0003-minimax-tts-architecture.md` | whole |
| Implementation detaill | `docs/architecture/minimax-tts-architecture.md` | whole |
| Cross-node context | `docs/analysis/current-nodes.md` | persistent row (§1 table) |
