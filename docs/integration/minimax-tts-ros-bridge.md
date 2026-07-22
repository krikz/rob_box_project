# MiniMax TTS → ROS 2 audio bridge — integration contract

> Связь между MiniMax TTS API и ROS-нодой воспроизведения аудио:
> что именно подключено в коде, какие параметры зафиксированы,
> и чем проверена совместимость.

| Поле | Значение |
|---|---|
| Статус | Accepted (integration zone complete, frozen v1 wire) |
| Дата | 2026-07-22 |
| Контекст | Kanban task `t_460ae9c6` |
| Родители | [ADR-0007 §2.5](../adr/0007-minimax-tts-integration-final.md), [ADR-0007b](../adr/0007b-minimax-tts-ros2-audio-contract-fragment.md), [ros2-audio-contract-spec.md](../architecture/ros2-audio-contract-spec.md) |
| Реализация | `src/rob_box_voice/rob_box_voice/tts_node.py` (consumer / composition root), `src/rob_box_voice/rob_box_voice/utils/audio_transcode.py` (container → PCM) |
| Ядро TTS-провайдера | `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py` — НЕ модифицируется в рамках этой задачи |

---

## 0. TL;DR

Между `MiniMaxTTSProvider` (HTTP/SSE → `TTSAudio(samples, sample_rate, format)`)
и ROS-топиком `/voice/audio/speech` (`audio_common_msgs/AudioData`)
уже стоит **готовый и проверенный** integration zone в `tts_node`:

```
MiniMax T2A v2  →  MiniMaxTTSProvider.synthesize()
                  returns TTSAudio(samples, sample_rate, format)
                          ↓
              tts_node._synthesize_minimax_async()
                  → tts_node._decode_minimax_audio()         [utils/audio_transcode.py]
                  → tts_node._prepare_audio_for_topic()      [resample actual_sr → audio_output_sample_rate]
                  → tts_node._publish_audio()                [float32 → int16 LE → AudioData]
                          ↓
              /voice/audio/speech (audio_common_msgs/AudioData, pcm_s16le mono 16 kHz)
                          ↓
              AudioPlaybackManager (direct sounddevice sink) + внешние subscriber'ы
```

Integration zone — **единственная** зона, которая трогается этой задачей.
Ядро `MiniMaxTTSProvider` остаётся как есть (per ADR-0002 opt-in, no silent
fallback, no retry-inside-adapter).

---

## 1. Что именно ожидает ROS-нода воспроизведения

Frozen v1 contract (ADR-0007b §2.1, не меняется без нового ADR):

| Поле | Значение | Источник правды |
|---|---|---|
| Топик | `/voice/audio/speech` | ROS-param `audio_topic` (default `/voice/audio/speech`) |
| Message type | `audio_common_msgs/AudioData` | импорт `from audio_common_msgs.msg import AudioData` |
| `data` (uint8[]) | raw **pcm_s16le** mono bytes | `_publish_audio` (tts_node.py:1421-1433) |
| `sample_rate` (фактический) | **16 000 Hz** | ROS-param `audio_output_sample_rate` (default 16000) |
| `bit_depth` | 16 bit signed | конвенция AudioData |
| `channels` | 1 (mono) | `_decoded_audio_to_float32` (downmix при необходимости) |
| `frame_size` (логический, streaming) | 640 bytes / 20 ms | вычислено: `16000 × 0.020 × 2 × 1` |
| QoS | **Best Effort + Volatile + KEEP_LAST(10)** | `_build_audio_qos` (tts_node.py:339-353) |

Подписчики (включая `AudioPlaybackManager` через in-process sink и любые
внешние `ros2 topic echo /voice/audio/speech`) полагаются на эти константы
без out-of-band метаданных — `AudioData.info` существует, но
`tts_node` сейчас не публикует; вся правда в этом документе + в `ros2-audio-contract-spec.md`.

---

## 2. Что integration zone делает (4 шага, каждый верифицирован)

### 2.1 Container decode — `to_pcm_int16` (utils/audio_transcode.py)

На вход: `TTSAudio(samples: bytes, sample_rate: int, format: TTSFormat)`.
На выход: `DecodedAudio(pcm: bytes, sample_rate, channels=1, source_format)`.

Поддерживаются все 4 контейнера из MiniMax (`pcm / wav / mp3 / ogg`):

| Контейнер | Декодер | Где |
|---|---|---|
| `PCM` (raw int16 LE) | passthrough + проверка чётности | `to_pcm_int16` (audio_transcode.py:105-121) |
| `WAV` (RIFF/WAVE) | `wave.open` + bit-conversion 8/24/32 → 16 + downmix | `_decode_wav` (audio_transcode.py:135-190) |
| `MP3` | `pydub` (preferred) → fallback `ffmpeg subprocess` | `_decode_compressed` (audio_transcode.py:210-281) |
| `OGG` | тот же fallback-каскад | `_decode_compressed` |

Ошибки категоризируются: `AudioTranscodeError(fmt=, reason=)` отделён от
network/provider-ошибок, чтобы caller мог различить «сеть ОК, формат
нечитаем» от «сеть отвалилась».

### 2.2 Float32 conversion — `_decoded_audio_to_float32` (tts_node.py:1063-1079)

PCM bytes → mono `float32` в диапазоне `-1..1`. Stereo downmix —
mean по каналам через int32 accumulator (без overflow).

### 2.3 Resample — `_prepare_audio_for_topic` (tts_node.py:1405-1419)

```python
def _prepare_audio_for_topic(audio_np: np.ndarray, sample_rate: int) -> np.ndarray:
    if sample_rate == self.audio_output_sample_rate:
        return audio_np
    return resample_audio(audio_np, sample_rate, self.audio_output_sample_rate)
```

- На вход: фактический `sample_rate` из MiniMax (`extra_info.audio_sample_rate`,
  обычно **32 000 Hz**).
- На выход: массив на `audio_output_sample_rate` (default **16 000 Hz**).
- Параметр `audio_output_sample_rate` объявлен в tts_node.py:217 и может быть
  переопределён через launch-YAML или `set_parameters` callback.
- Качество: `scipy.signal.resample_poly` через `resample_audio()` (tts_node.py:101)
  с anti-aliasing. Артефакты ресэмплинга описаны в `ros2-audio-contract-spec.md` §4.3-A.

### 2.4 Publish — `_publish_audio` (tts_node.py:1421-1433)

```python
audio_int16 = (np.clip(audio_np, -1.0, 1.0) * 32767).astype("<i2", copy=False)
msg = AudioData()
msg.data = list(audio_int16.tobytes())
self.audio_pub.publish(msg)
```

- Клиппинг float → int16 (защита от переполнения, не silent fallback).
- `msg.data` — список `int` (портативно для всех ROS2-дистрибутивов; bytes
  assignment работает не везде — комментарий в коде объясняет).
- QoS: best_effort / volatile / KEEP_LAST(10) — параметризуется
  через `audio_qos_reliability` и `audio_qos_depth`.

---

## 3. Параметры, согласованные с оборудованием робота

| ROS-параметр | Default | Назначение | Где в коде |
|---|---:|---|---|
| `provider` | `yandex` | гейт выбора TTS-движка; `minimax` = opt-in MiniMax | tts_node.py:168 |
| `audio_topic` | `/voice/audio/speech` | frozen v1 | tts_node.py:216 |
| `audio_output_sample_rate` | `16000` | frozen v1, синхронно с ReSpeaker/USB Audio Class 1.0 | tts_node.py:217 |
| `audio_qos_reliability` | `best_effort` | streaming-friendly; команды отдельно (reliable) | tts_node.py:218 |
| `audio_qos_depth` | `10` | bounded queue = backpressure protection | tts_node.py:219 |
| `minimax_sample_rate` | `32000` | upstream MiniMax PCM rate; downstream ресэмплит в 16 kHz | tts_node.py:195 |
| `minimax_format` | `pcm` | upstream MiniMax контейнер; downstream декодирует через transcode | tts_node.py:201 |
| `minimax_streaming` | `False` | opt-in chunk-per-frame (M5/M6 — WebSocket) | tts_node.py:210 |
| `minimax_max_retries` | `2` (cap 3) | ADR-0003 §2.6 retry policy | tts_node.py:203 |
| `minimax_retry_backoff_ms` | `500` | exponential backoff base | tts_node.py:204 |

Согласование с оборудованием робота:

- **ReSpeaker USB Audio Class 1.0** работает на 16 кГц mono → именно поэтому
  frozen SR = 16 000 Hz (`audio_output_sample_rate`). MiniMax возвращает
  32 кГц → обязателен ресэмплинг 32→16 кГц в `_prepare_audio_for_topic`.
- **dmix ALSA routing** (через `asound.conf`) — `device=None` в
  `initialize_audio_device` (tts_node.py:425), не требует согласования SR на
  уровне ALSA.
- **Barge-in / STOP semantics** — `STOP` приходит на `/voice/tts/control`,
  чистит `playback_manager` queue, `_on_new_dialogue_id` (tts_node.py:511)
  сбрасывает старые chunks. Sub-fragment `0007b` §2.4 фиксирует политику.

---

## 4. Что проверяет каждая acceptance-компонента

| Acceptance | Где проверено | Статус |
|---|---|---|
| MiniMax → `audio_common_msgs/AudioData` тип | `test_minimax_integration.py` импортирует `AudioData`, проверяет `msg.data` | ✅ |
| Sample rate / channels / bit depth согласованность | bench `format_pcm/wav/mp3/ogg` сценарии, `ffprobe` cross-check → `pcm_s16le / 16000 Hz / mono` | ✅ 8/8 |
| Resample 32→16 kHz выполняется | bench `format_pcm` SR=32000 (MiniMax) → выход SR=16000 | ✅ |
| Stereo downmix | `test_stereo_wav_is_downmixed_to_mono` | ✅ |
| Container decode (PCM/WAV/MP3/OGG) | `test_pcm_passthrough_preserves_sample_count`, `test_wav_container_is_decoded_to_pcm`, `test_valid_mp3/ogg` | ✅ |
| Clipping защита | `test_publish_audio_clips_out_of_range_samples` | ✅ |
| Streaming chunk-per-frame | `test_streaming_publishes_one_msg_per_chunk`, `test_streaming_publishes_each_chunk_before_requesting_the_next`, bench `streaming_ttfa_pcm` | ✅ |
| Streaming resample | `test_streaming_resamples_chunk_to_declared_topic_rate` | ✅ |
| Streaming error path | `test_streaming_handles_finish_reason_error`, `test_streaming_error_after_audio_is_not_published_as_silence` | ✅ |
| Retry classification | `test_retry_after_two_timeouts_succeeds`, `test_auth_error_not_retried`, `test_bad_request_not_retried`, `test_rate_limit_is_retried`, `test_exhaustion_raises_last_error` | ✅ |
| Resource cleanup | `test_close_minimax_provider_closes_client_and_clears_reference` | ✅ |
| QoS parity | bench `real_subscriber.py` запускается в отдельном subprocess, читает с теми же QoS, ffprobe подтверждает формат | ✅ |

**Численная сводка:**
- Unit/integration тесты: **22/22 pass** (`pytest src/rob_box_voice/test/unit/tts/test_minimax_integration.py`)
- End-to-end bench: **8/8 pass** (`tts_audio_bench/artifacts/bench-summary.json`,
  TTFA 62-656 ms, sample rate всех scenarios = 16000 Hz, ffprobe pcm_s16le mono)

---

## 5. Что НЕ менялось в этой задаче (по body)

| Зона | Файл | Решение |
|---|---|---|
| Ядро TTS-провайдера | `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py` | **НЕ ТРОНУТ** — body явно требует «без правки ядра TTS-провайдера» |
| Ядро value-objects | `src/rob_box_llm/rob_box_llm/tts.py` | **НЕ ТРОНУТ** — `TTSAudio(samples, sample_rate, format)` уже даёт нужные поля |
| Error hierarchy | `src/rob_box_llm/rob_box_llm/errors.py` | **НЕ ТРОНУТ** — `TTSError` / `TTSAuthError` / `TTSRateLimitError` / `TTSBadRequestError` уже достаточны для retry classification |
| Frozen ROS contract | топик `/voice/audio/speech`, тип `AudioData`, SR 16 kHz, mono, pcm_s16le | **НЕ ИЗМЕНЁН** — frozen в ADR-0007b §2.1 |

Все 4 пункта body задачи закрыты в **integration zone**
(`tts_node.py:_decode_minimax_audio` + `_prepare_audio_for_topic` +
`_publish_audio` + `utils/audio_transcode.py`) — зоне, которая и так
должна существовать между upstream-форматом и frozen ROS-контрактом.

---

## 6. Acceptance criteria этой задачи (mapping)

| # | Требование из body | Где закрыто |
|---|---|---|
| 1 | Проверить, какой AudioStamped / message-тип ожидает нода воспроизведения | §1 — `audio_common_msgs/AudioData`, frozen v1 |
| 2 | При необходимости добавить адаптер/конвертер из сырого аудио в ожидаемый формат | §2 — `_decode_minimax_audio` + `_prepare_audio_for_topic` + `_publish_audio` уже реализованы |
| 3 | Убедиться, что параметры (sample_rate, channels, bit depth) согласованы с оборудованием | §3 — frozen 16 kHz mono pcm_s16le, согласовано с ReSpeaker; miniMax 32→16 kHz через resample_poly |
| 4 | Задокументировать интеграционный контракт | **этот документ** + ссылки на `ros2-audio-contract-spec.md` и `ADR-0007b` |

---

## 7. Открытые вопросы / куда расти

| Тема | Где живёт | Почему не в этой задаче |
|---|---|---|
| WebSocket chunk-per-frame streaming | ADR-0004 §7, sub-frag `0007a` «WebSocket (reserved)» | reserved для future-ADR; пока провайдер буферизует → SSE-equivalent |
| AudioStamped opt-in meta-topic | `0007b` §2.4 | opt-in, default OFF; включается отдельной задачей |
| Circuit breaker | sub-frag `0007a` §«Circuit breaker», default disabled | включается при >100 TPS/час/робот |
| CLI `rob_box_llm.tts_cli` | ADR-0006 | отдельная задача; integration zone не зависит |

---

## 8. Связанные документы

- **Нормативные**: [ADR-0007 §2.5](../adr/0007-minimax-tts-integration-final.md),
  [ADR-0007b](../adr/0007b-minimax-tts-ros2-audio-contract-fragment.md)
- **Reference**: [ros2-audio-contract-spec.md](../architecture/ros2-audio-contract-spec.md) — детальная спецификация (391 строка)
- **Diagrams**: [minimax-tts-ros2-dataflow.mmd](../diagrams/minimax-tts-ros2-dataflow.mmd),
  [minimax-tts-ros2-audio-contract-sequence.mmd](../diagrams/minimax-tts-ros2-audio-contract-sequence.mmd)
- **Implementation**: `src/rob_box_voice/rob_box_voice/tts_node.py:1063-1433` (decode → resample → publish),
  `src/rob_box_voice/rob_box_voice/utils/audio_transcode.py` (контейнерный декодер)
- **Tests**: `src/rob_box_voice/test/unit/tts/test_minimax_integration.py` (22 теста)
- **Bench**: `tts_audio_bench/` + `tts_audio_bench/artifacts/bench-summary.json` (8/8 сценариев)
