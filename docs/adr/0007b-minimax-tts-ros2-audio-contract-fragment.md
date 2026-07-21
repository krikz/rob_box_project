# Sub-fragment 0007b к ADR-0007: контракт MiniMax TTS ↔ ROS 2 аудиостэк

| Поле | Значение |
|---|---|
| Статус | Sub-fragment к ADR-0007 (Proposed) |
| Дата | 2026-07-21 |
| Автор | architect (Hermes Agent) |
| Контекст | Kanban task `t_3ff1d7f5` |
| Родитель | [ADR-0007](0007-minimax-tts-integration-final.md) §2.5 |
| Родители (по теме) | [ADR-0002](0002-minimax-provider.md), [ADR-0003](0003-minimax-tts-architecture.md), [ADR-0004](0004-minimax-tts-integration-design.md) |
| Связанный код | `src/rob_box_llm/rob_box_llm/tts.py`, `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py`, `src/rob_box_voice/rob_box_voice/tts_node.py` |
| Диаграмма | [`../diagrams/minimax-tts-ros2-audio-contract-sequence.mmd`](../diagrams/minimax-tts-ros2-audio-contract-sequence.mmd) |
| Сопутствующий sub-fragment | [0007a — Reliability](0007a-minimax-tts-reliability-fragment.md) |

## 1. Контекст и границы

MiniMax возвращает hex-кодированный аудиобуфер и фактические параметры в `extra_info`. ROS 2 `audio_common_msgs/AudioData` содержит только `uint8[] data`, поэтому sample rate, число каналов и формат нельзя надёжно угадать на стороне подписчика. Этот ADR фиксирует границу `MiniMaxTTSProvider → tts_node → ROS 2`, не меняя существующие имена топиков.

В проекте **нет отдельной ноды `audio_play_node`**: публикация выполняется `tts_node`, а воспроизведение — локально через `AudioPlaybackManager`/`sounddevice`. В диаграмме имя `audio_play_node` обозначает логический downstream audio-playback consumer; при выделении его в отдельную ноду топик и сообщение остаются теми же.

## 2. Frozen audio contract (v1)

### 2.1 Параметры PCM

| Поле | Значение v1 | Правило проверки |
|---|---:|---|
| `sample_rate` | 16 000 Hz на ROS-выходе | MiniMax обычно запрашивается на 32 000 Hz; при отличии от 16 kHz обязательна ресэмплинг-конверсия в `tts_node` |
| `bit_depth` | 16 bit | только signed PCM; 24/32 bit не публикуются в `AudioData` v1 |
| `format` | PCM signed little-endian (`pcm_s16le`) | `data` — raw bytes, не WAV-заголовок и не base64/hex |
| `channels` | 1 (mono) | TTS-выход mono; stereo создаётся только в локальном playback path для ReSpeaker |
| `frame_size` | 640 bytes для 20 ms | `16000 × 0.020 × 2 × 1 = 640`; frame size не является полем `AudioData`, это размер логического чанка |

Провайдер возвращает `TTSAudio(samples: bytes, sample_rate=<actual>)`. `tts_node` декодирует hex, интерпретирует PCM как `int16` little-endian, приводит к float32, ресэмплирует в `audio_output_sample_rate` (default 16 kHz), затем сериализует `int16` LE. На границе ROS принимаются только размеры, кратные двум; значение sample rate передаётся out-of-band параметром `audio_output_sample_rate`.

Если API вернул другой sample rate (например, 24 kHz), запрещено просто менять метаданные: это изменит скорость/тональность. Сначала выполнить качественный ресэмплинг (целевой 16 kHz), затем опубликовать PCM. Ошибка декодирования, несовместимый формат или невозможность ресэмплинга — typed TTS error и состояние `error`, без молчаливого fallback.

### 2.2 ROS-сообщение и топики

Текущий совместимый контракт:

```text
audio_common_msgs/AudioData
  data: uint8[]  # raw pcm_s16le, mono, 16 kHz
```

| Направление | Топик | Тип | Назначение |
|---|---|---|---|
| caller → tts_node | `/voice/dialogue/response` | `std_msgs/String` | основной текст/JSON-запрос |
| caller → tts_node | `/voice/tts/request` | `std_msgs/String` | альтернативный TTS-запрос |
| tts_node → audio consumer | `/voice/audio/speech` | `audio_common_msgs/AudioData` | озвучивание MiniMax/Yandex/Silero; имя и тип frozen |
| tts_node → caller | `/voice/tts/state` | `std_msgs/String` | `ready`, `synthesizing`, `playing`, `error` |
| tts_node → caller | `/voice/tts/finished` | `std_msgs/String` | завершение произнесения |

`/audio/audio` и `/audio/speech_audio` — существующие **входные** топики `audio_node` (микрофон), не выход MiniMax; их нельзя переиспользовать для TTS.

### 2.3 QoS

* Команды и управление (`/voice/dialogue/response`, `/voice/tts/request`, `/voice/tts/control`, state/finished): `Reliable`, `Volatile`, `KEEP_LAST`, depth 10. Потеря команды недопустима; STOP должен доходить.
* Аудиопоток `/voice/audio/speech`: профиль, эквивалентный `qos_profile_sensor_data`: `Best Effort`, `Volatile`, `KEEP_LAST`, bounded depth (default 10). Старые аудиочанки не должны воспроизводиться после barge-in, а медленный consumer не должен блокировать synthesis.
* Все endpoints должны использовать совместимые reliability/durability. `Reliable` publisher и `Best Effort` subscriber не образуют ожидаемую пару во всех DDS-конфигурациях; профиль выбирается одинаково на publisher и consumer.

Текущая реализация `tts_node` уже параметризует `audio_qos_reliability` (default `best_effort`), `audio_qos_depth` (default 10) и `audio_topic` (default `/voice/audio/speech`). При включении потокового режима downstream обязан выбрать тот же профиль.

### 2.4 Публикация и backpressure

Основной sync-путь сохраняет обратную совместимость: одна `AudioData` на весь синтез. Для streaming/WebSocket-пути контрактом является фиксированный PCM chunk длительностью 20 ms (640 bytes при v1), публикуемый последовательно с timestamp/sequence в расширенном сообщении.

`AudioData` не имеет timestamp и sequence. Поэтому для фиксированных чанков v1 используется только для legacy consumer; новый мета-топик включается opt-in:

```text
/voice/audio/speech_meta  # AudioDataStamped-подобный custom message
header.stamp, header.frame_id="voice_out"
sample_rate=16000, channels=1, format="pcm_s16le", samples=uint8[]
```

Ограничения и политика:

1. Размер сообщения ограничен DDS/RMW и транспортом; не отправлять многосекундный поток одним сообщением в streaming path. 20 ms chunks дают bounded payload и избегают DDS fragmentation.
2. Publisher не должен бесконечно накапливать очередь. Использовать KEEP_LAST с bounded depth; при Best Effort допустимо отбрасывать старые chunks при переполнении.
3. Consumer обязан обнаруживать пропуск sequence/timestamp и сбрасывать неполный playback buffer, а не повторять устаревшие samples.
4. Для Reliable команд разрешён bounded blocking/timeout и явная ошибка; нельзя блокировать ROS executor синхронным ожиданием медленного аудиопотребителя.
5. При barge-in `STOP` очищает очередь playback и отбрасывает chunks старого `dialogue_id`. Синтез и публикация проверяют cancellation между chunks.

## 3. Принятое решение и последствия

Сохраняем `/voice/audio/speech` + `AudioData` как v1 для совместимости с текущими подписчиками. MiniMax-specific conversion выполняется до ROS publish; провайдер не знает о DDS и не публикует сам. Fixed-size chunking и timestamped message являются opt-in расширением для настоящего streaming и не должны тихо менять legacy path.

Это предотвращает QoS mismatch, неконтролируемый backpressure и ошибочное воспроизведение 24 kHz как 16 kHz. Цена решения — out-of-band metadata в legacy topic и необходимость отдельного custom message для timestamped streaming.

## 4. Acceptance criteria

* `sample_rate=16000`, `int16 LE`, mono, byte length even are validated before publish.
* 24 kHz fixture becomes 16 kHz PCM via resampling, not metadata rewrite.
* QoS is explicit: Reliable for commands, sensor-data Best Effort/Volatile for audio.
* Legacy `/voice/audio/speech` remains available; `/audio/audio` is not used for TTS.
* Streaming design states 20 ms / 640-byte frames, bounded depth, drop/cancel behavior.
* Sequence diagram reflects `caller → provider → topic → audio_play_node` and the actual in-process playback implementation.
