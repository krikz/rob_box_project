# ROS 2 Audio Contract — MiniMax TTS → Speaker

> **Спецификация** (не ADR) контракта аудиопотока от MiniMax TTS API
> до физического динамика ROS 2-узла. Зафиксированные параметры
> контракта (PCM-формат, топики, QoS) задокументированы в
> [ADR-0007b](../adr/0007b-minimax-tts-ros2-audio-contract-fragment.md);
> этот документ дополняет их и сводит в один reference-документ
> всё, что нужно реализатору и интегратору.

| Поле | Значение |
|---|---|
| Статус | Proposed (готов к ревью) |
| Дата | 2026-07-21 |
| Автор | ros2-engineer (Hermes Agent) |
| Контекст | Kanban task `t_9bb85faf` |
| Родитель | [ADR-0007](../adr/0007-minimax-tts-integration-final.md) §2.5 |
| Смежные | [ADR-0007b](../adr/0007b-minimax-tts-ros2-audio-contract-fragment.md) (frozen contract v1), [ADR-0007a](../adr/0007a-minimax-tts-reliability-fragment.md) (retry/CB), [architecture/minimax-tts-architecture.md](./minimax-tts-architecture.md) |
| Связанный код | `src/rob_box_llm/rob_box_llm/tts.py`, `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py`, `src/rob_box_voice/rob_box_voice/tts_node.py`, `src/rob_box_voice/rob_box_voice/audio_playback_manager.py`, `src/rob_box_voice/rob_box_voice/utils/audio_transcode.py` |
| Диаграмма | [`../diagrams/minimax-tts-ros2-dataflow.mmd`](../diagrams/minimax-tts-ros2-dataflow.mmd) |

---

## 0. TL;DR

- **MiniMax TTS → ROS 2 контракт v1** (frozen): одно `audio_common_msgs/AudioData`-сообщение
  на весь синтез, raw `int16` little-endian PCM, mono, **16 кГц**, топик
  `/voice/audio/speech`, QoS sensor-data (Best Effort, Volatile, KEEP_LAST(10)).
- **Параметры MiniMax** по умолчанию: `sample_rate=32000`, `channel=1`,
  `format="pcm"`. Контракт ROS-стороны требует обязательной **ресэмплинг-конверсии**
  MiniMax 32 кГц → 16 кГц в `tts_node` до публикации.
- **Зона ответственности**: `MiniMaxTTSProvider` декодирует hex/контейнер
  и возвращает PCM+`sample_rate`; `tts_node` ресэмплирует и публикует;
  in-process playback через `AudioPlaybackManager`/sounddevice — без отдельной
  `audio_play_node` и без `sound_play`.
- **Статичные vs варьирующиеся параметры**: PCM-контракт и QoS — статичные;
  `voice_id`, `model`, `language`, `speed`, `volume`, `pitch`, `emotion` —
  варьируются от выбранного голоса или диалога; `format` MiniMax — фиксирован
  на `pcm`, чтобы избежать неоднозначности декодера.
- **При несовпадении формата** — ответственность за ресэмплинг/конверсию
  лежит на `tts_node._prepare_audio_for_topic` (см. §4). Никаких silent
  fallback'ов: ошибка декодирования → `TTSError` → состояние `error`
  на `/voice/tts/state`.

---

## 1. Параметры аудио на каждой границе

### 1.1 Frozen PCM-контракт (v1)

Параметры PCM на границе ROS-топика — **заморожены** на одни значения
для всех TTS-провайдеров (`minimax`, `yandex`, `silero`), чтобы
подписчики (`audio_play_node` или внешние интеграторы) могли полагаться
на константу без метаданных.

| Поле | Значение v1 | Правило проверки | Источник правды |
|---|---:|---|---|
| `sample_rate` | **16 000 Hz** | MiniMax обычно 32 000 Hz → обязательная ресэмплинг-конверсия | ROS-параметр `audio_output_sample_rate` (default 16000) |
| `bit_depth` | **16 bit** | только signed PCM; 24/32 не публикуются | конвенция `audio_common_msgs/AudioData` v1 |
| `format` | **PCM signed little-endian** (`pcm_s16le`) | `data` — raw bytes, **не** WAV-заголовок и **не** base64/hex | формат AudioData + §4 |
| `channels` | **1 (mono)** | stereo создаётся только в локальном playback path для ReSpeaker (`tts_node.py:893`) | конвенция v1 |
| `frame_size` | 640 bytes (20 ms) | `16000 × 0.020 × 2 × 1`; логический чанк для streaming, **не** поле AudioData | вычислено |

> **Out-of-band метаданные** sample_rate/channels/format не передаются
> через `AudioData` (нет полей); значения передаются ROS-параметром
> `audio_output_sample_rate` + этой спецификацией.

### 1.2 Параметры на стороне MiniMax API

Значения, которые **запрашиваем у API** через `MiniMaxTTSProvider._build_payload`
(см. `providers/minimax_tts.py:143-189`):

| Поле MiniMax | Наш default | MiniMax диапазон | Источник |
|---|---|---|---|
| `audio_setting.sample_rate` | **32000 Hz** | 8000/16000/22050/24000/32000/44100 | ROS-параметр `minimax_sample_rate` |
| `audio_setting.bitrate` | 128000 (хардкод) | 32000/64000/128000/256000 (только для mp3) | `minimax_tts.py` |
| `audio_setting.format` | **`"pcm"`** | `mp3 / pcm / flac / wav / pcmu_raw / pcmu_wav / opus` | `minimax_format` → `_build_payload` |
| `audio_setting.channel` | **1 (mono, хардкод)** | 1/2 | `minimax_tts.py:188` |
| `voice_setting.voice_id` | `"male-qn-qingse"` | ~40 системных + клоны | ROS-параметр `minimax_voice` |
| `voice_setting.speed` | 1.0 | [0.5, 2] | ROS-параметр `minimax_speed` |
| `voice_setting.vol` | 1.0 | (0, 10] ⚠️ ноль недопустим | ROS-параметр `volume_db` маппится в vol |
| `voice_setting.pitch` | 0 | [-12, 12] semitones | (не вынесено в ROS-параметр) |
| `voice_setting.emotion` | авто | 9 значений (`happy/sad/.../fluent/whisper`) | (не вынесено в ROS-параметр) |
| `model` | `"speech-02-hd"` | 8 моделей (2.8/2.6/02/01 × hd/turbo) | ROS-параметр `minimax_model` |

### 1.3 Что MiniMax реально возвращает (`extra_info`)

В ответе синка `extra_info` содержит **фактические** параметры, которые
могут отличаться от запрошенных (особенно `sample_rate` при ошибке
или fallback):

```jsonc
{
  "extra_info": {
    "audio_length": 11124,         // ms
    "audio_sample_rate": 32000,    // фактический SR; ≠ 16000 → обязателен ресэмплинг
    "audio_size": 179926,          // bytes (hex-декодированный)
    "bitrate": 128000,
    "word_count": 163,
    "usage_characters": 163,       // биллинг
    "audio_format": "pcm",
    "audio_channel": 1
  }
}
```

**Критично**: провайдер возвращает `TTSAudio(samples: bytes, sample_rate=<actual>)`,
и `tts_node` обязан использовать **фактический** SR, а не наш default.

---

## 2. Статичные vs варьирующиеся параметры (явная таблица)

Это раздел, прямо запрошенный в body задачи `t_9bb85faf`.

### 2.1 Статичные (конфиг, фиксируются на старте ноды)

Меняются только через ROS-параметры / YAML / ENV при запуске или по
команде `set_parameters`. **Не зависят** от выбранного голоса MiniMax.

| Параметр | Где задаётся | Default | Что определяет | Почему статичный |
|---|---|---:|---|---|
| `sample_rate` (ROS-выход) | ROS-param `audio_output_sample_rate` | 16000 Hz | PCM на `/voice/audio/speech` | контракт frozen v1; подписчики не знают другой SR |
| `format` (MiniMax-вход) | ROS-param `minimax_format` | `"pcm"` | что запрашиваем у API | `pcm` минимизирует транскодинг (int16 LE уже готов для AudioData) |
| `channels` (MiniMax-вход) | хардкод | 1 (mono) | MiniMax-выход | v1 contract: mono only |
| `audio_qos_reliability` | ROS-param | `best_effort` | QoS audio-топика | streaming-friendly; команды отдельно (reliable) |
| `audio_qos_depth` | ROS-param | 10 | bounded queue audio-топика | backpressure защита |
| `audio_topic` | ROS-param | `/voice/audio/speech` | имя топика | контракт frozen v1 |
| `provider` (гейт) | ROS-param | `yandex` | какой TTS-движок | маршрутизация в `tts_node` |
| `chipmunk_mode` | ROS-param | `True` | эффект голоса | пост-обработка playback, не контракт |
| `volume_db` | ROS-param | -3.0 | пост-обработка | playback gain, не контракт |
| `audio_playback_mode` | ROS-param (declaration сейчас отсутствует, см. ниже) | `direct_sink` | какой sink | в проекте — всегда direct sink; `sound_play` не используется |

> **Замечание для интегратора**: `audio_playback_mode` сейчас не вынесен в
> явный параметр, потому что `tts_node` всегда использует
> `AudioPlaybackManager` (direct sounddevice sink). Если появится требование
> публиковать в `sound_play` или отдельный sink-node, параметр нужно
> объявить. До тех пор — implicit `direct_sink` (см. §5).

### 2.2 Варьирующиеся (per-utterance, от голоса или команды)

Меняются при каждом синтезе в зависимости от того, какой голос выбран
для ответа, от LLM-prompt'а или от runtime-параметров диалога.

| Параметр | Откуда берётся | Кто меняет | Диапазон | Что определяет |
|---|---|---|---|---|
| `voice_id` | ROS-param `minimax_voice` ИЛИ override в `/voice/dialogue/response` (если поддерживается) | оператор / dialogue_node | ~40 ID + клоны | какой MiniMax-голос |
| `model` | ROS-param `minimax_model` ИЛИ override в `/voice/dialogue/response` | оператор | 8 моделей | качество / латентность / эмоции |
| `language` | ROS-param `minimax_language` ИЛИ override | оператор | `ru/en/zh/...` | язык произношения |
| `speed` | ROS-param `minimax_speed` ИЛИ SSML `<prosody rate>` в диалоге | оператор / SSML | [0.5, 2] | темп |
| `volume` (`vol`) | производный от `volume_db` + runtime gain | dialogue_node | (0, 10] | громкость синтеза |
| `pitch` | (не вынесено в ROS-параметр; только через `settings.extra` или SSML) | SSML | [-12, 12] semitones | высота голоса |
| `emotion` | (не вынесено; только через `settings.extra`) | SSML / явный вызов | 9 enum | эмоциональная окраска |
| `text` (utterance) | payload `/voice/dialogue/response` | dialogue_node | ≤ 10000 симв | **собственно текст для синтеза** |
| `actual_sample_rate` (MiniMax-выход) | `extra_info.audio_sample_rate` | MiniMax API | 8000–44100 | **фактический SR**, всегда требует проверки |
| `audio_length` | `extra_info.audio_length` | MiniMax API | ms | длительность utterance |

> **Правило варьирования**: всё, что касается **содержания** голоса
> (кто говорит, как, на каком языке, что говорит) — варьируется.
> Всё, что касается **формы передачи** по ROS-шине (SR, формат,
> число каналов, QoS, топик) — заморожено.

### 2.3 Сводка: что меняется на каждый utterance

```
utterance = synthesize(text, voice_id, model, language, speed, ...)
   ↓
MiniMax API → extra_info{actual_sample_rate, audio_length, audio_format}
   ↓
TTSAudio(samples, actual_sample_rate, format)
   ↓
[static convert] → /voice/audio/speech (16 kHz mono pcm_s16le, best_effort)
```

PCM-контейнер на ROS-шине **никогда** не меняется. Меняется только
содержимое `data: uint8[]` и параметры upstream.

---

## 3. ROS-топики, сообщения и QoS

### 3.1 Сообщение

```text
# audio_common_msgs/AudioData
audio_common_msgs/AudioData
  uint8[] data   # raw pcm_s16le mono 16 kHz; длина всегда кратна 2
```

Пример одного сообщения (Python-репрезентация):

```python
from audio_common_msgs.msg import AudioData

# ~20 ms голоса "Привет" после 32 kHz → 16 kHz PCM
msg = AudioData()
msg.data = bytes.fromhex(
    "00ff7f00ff00007f00ffff007e..."  # int16 LE, 16 kHz mono
)
# len(msg.data) = 640 bytes для 20 ms; для полного utterance — кратно 640
```

Для больших utterance (например, 3 секунды) — это **одно** AudioData-сообщение,
потому что MiniMax sync-эндпоинт возвращает весь payload целиком. Для
streaming (WebSocket, будущее) — серия `AudioData` по 640 байт + opt-in
мета-топик `/voice/audio/speech_meta` (см. ADR-0007b §2.4).

### 3.2 Карта топиков

| Направление | Топик | Тип | Назначение | Контракт |
|---|---|---|---|---|
| caller → tts_node | `/voice/dialogue/response` | `std_msgs/String` | основной текст/JSON-запрос | frozen |
| caller → tts_node | `/voice/tts/request` | `std_msgs/String` | альтернативный TTS-запрос | frozen |
| tts_node → audio consumer | `/voice/audio/speech` | `audio_common_msgs/AudioData` | озвучивание MiniMax/Yandex/Silero | frozen v1 |
| tts_node → caller | `/voice/tts/state` | `std_msgs/String` | `ready/synthesizing/playing/stopped/error` | frozen |
| tts_node → caller | `/voice/tts/finished` | `std_msgs/String` | завершение utterance | frozen |
| (opt-in, streaming) | `/voice/audio/speech_meta` | custom AudioDataStamped | header + SR + формат + PCM | opt-in, ADR-0007b §2.4 |

### 3.3 QoS-профили

| Топик | Reliability | Durability | History | Depth | Обоснование |
|---|---|---|---|---:|---|
| `/voice/audio/speech` | **Best Effort** | Volatile | KEEP_LAST | 10 | sensor-data профиль; старые chunks не нужны после barge-in; медленный consumer не должен блокировать synthesis |
| `/voice/dialogue/response` | **Reliable** | Volatile | KEEP_LAST | 10 | потеря команды недопустима |
| `/voice/tts/request` | Reliable | Volatile | KEEP_LAST | 10 | то же |
| `/voice/tts/state` | Reliable | Volatile | KEEP_LAST | 10 | состояние должно доходить |
| `/voice/tts/finished` | Reliable | Volatile | KEEP_LAST | 10 | сигнал завершения |
| `/voice/audio/speech_meta` (opt-in) | Best Effort | Volatile | KEEP_LAST | 10 | согласовано с основным audio-топиком |

> **Правило**: `Reliable`-publisher и `Best Effort`-subscriber не образуют
> ожидаемую пару во всех DDS-конфигурациях. Publisher и consumer должны
> выбирать **одинаковый** профиль. Текущий `tts_node` уже параметризует
> `audio_qos_reliability` (default `best_effort`).

---

## 4. Ответственность за конверсию и ресэмплинг (happy path + mismatch)

Это acceptance-критерий из body: «кто отвечает за конверсию/ресемплирование».

### 4.1 Конвейер обработки и зоны ответственности

```
┌─────────────────┐   ┌──────────────────┐   ┌──────────────────┐   ┌──────────────┐
│ MiniMaxTTSProvider│  │  tts_node        │   │ ROS topic        │   │ audio_play_  │
│ (HTTP/SSE)       │  │  (_prepare_audio │   │ /voice/audio/    │   │ node (sink)  │
│                  │  │   _for_topic)    │   │ speech           │   │              │
│ - запрашивает    │  │ - декодирует     │   │ frozen contract: │   │ - читает     │
│   sample_rate,   │  │   hex/int16 LE   │   │ pcm_s16le mono   │   │   AudioData  │
│   format,        │  │ - приводит к     │   │ 16 kHz           │   │ - воспроиз-  │
│   channel у API  │  │   float32        │   │                  │   │   водит      │
│ - декодирует hex │  │ - ресэмплит      │   │                  │   │              │
│   / контейнер    │  │   actual_sr →    │   │                  │   │              │
│ - возвращает     │  │   target_sr      │   │                  │   │              │
│   TTSAudio       │  │ - сериализует    │   │                  │   │              │
│                  │  │   обратно int16  │   │                  │   │              │
└─────────────────┘   └──────────────────┘   └──────────────────┘   └──────────────┘
```

### 4.2 Таблица ответственности (happy + mismatch)

| Шаг | Кто отвечает | Где в коде | Поведение при ошибке |
|---|---|---|---|
| 1. Запросить у API нужный `audio_setting` (SR, format, channel) | `MiniMaxTTSProvider` | `minimax_tts.py:_build_payload` (143–189) | Валидация полей → `TTSBadRequestError` |
| 2. Декодировать hex/контейнер (`pcm/wav/mp3/ogg`) → PCM float32 | `MiniMaxTTSProvider` | `minimax_tts.py:_decode_audio` (501) | Неизвестный формат / битый hex → `TTSError` |
| 3. Прочитать фактический `sample_rate` из `extra_info` | `MiniMaxTTSProvider` | `minimax_tts.py:470` | fallback на `32000` если поле отсутствует |
| 4. **Ресэмплинг** actual_sr → `audio_output_sample_rate` (16 kHz) | `tts_node._prepare_audio_for_topic` | `tts_node.py:_normalize_audio_to_topic_sr` (1410) | Ошибка ресэмплинга → `TTSError` (ни в коем случае не metadata-rewrite!) |
| 5. Сериализация float32 → int16 LE bytes | `tts_node._prepare_audio_for_topic` | `tts_node.py:1423-1428` | Нечётная длина → `TTSError` (только кратные двум) |
| 6. Публикация `AudioData{data=raw_bytes}` | `tts_node._publish_audio` | `tts_node.py:822` | QoS-mismatch → DDS warning, но не критично |
| 7. Чтение и воспроизведение | `AudioPlaybackManager` (direct sounddevice sink, в-process) | `tts_node.py:_synthesize_and_play` → `audio_playback_manager.py` | Девайс не доступен → `RuntimeError` → состояние `error` на `/voice/tts/state` |

### 4.3 Сценарии несовпадения формата (acceptance criteria)

#### Сценарий A: MiniMax вернул sample_rate ≠ 16 kHz (например 24 kHz)

| Что делаем | Кто делает | Результат |
|---|---|---|
| **Ресэмплинг** 24 kHz → 16 kHz через scipy/samplerate | `tts_node._normalize_audio_to_topic_sr` | Голос сохраняет тембр и длительность |
| ❌ НЕ делаем: меняем только метаданные | — | ЗАПРЕЩЕНО (изменит скорость/тональность) |
| ❌ НЕ делаем: silent fallback на другой провайдер | — | ЗАПРЕЩЕНО (пользователь явно выбрал MiniMax) |
| ❌ НЕ делаем: публикация `data` как есть с метаданными | — | AudioData не имеет поля sample_rate — невозможно |

**Итог**: `tts_node` обязан выполнить качественный ресэмплинг до
публикации. Нарушение — typed TTS error + `error`-state.

#### Сценарий B: MiniMax вернул MP3 (а не PCM)

| Что делаем | Кто делает | Результат |
|---|---|---|
| Транскодирование MP3 → float32 PCM через `utils/audio_transcode.py` | `MiniMaxTTSProvider._decode_audio` | Получаем PCM + actual_sr |
| Если декодер не справился | `MiniMaxTTSProvider` | `TTSError` (не silent fallback на PCM-плейсхолдер) |

#### Сценарий C: MiniMax вернул WAV с заголовком (но провайдер запрашивал PCM)

| Что делаем | Кто делает | Результат |
|---|---|---|
| Детектировать WAV-заголовок (`RIFF....WAVE`) | `MiniMaxTTSProvider._decode_audio` | Пропустить заголовок, интерпретировать payload как int16 LE |
| ❌ НЕ делаем: silent fallback на PCM (теряем заголовок → мусор в AudioData) | — | ЗАПРЕЩЕНО |

#### Сценарий D: MiniMax вернул `format=OGG` (намеренный fallback в провайдере)

| Что делаем | Кто делает | Результат |
|---|---|---|
| Провайдер уже маппит OGG → MP3 на этапе `_build_payload` (`minimax_tts.py:186-187`) | `MiniMaxTTSProvider` | Фактически получаем MP3 → переход в сценарий B |
| ❌ НЕ делаем: выдавать MP3 пользователю с маркировкой `TTSAudio.format=OGG` | — | Без warning — известный UX-баг (см. research §4.1) |

#### Сценарий E: декодирование успешно, но `len(data) % 2 != 0`

| Что делаем | Кто делает | Результат |
|---|---|---|
| Валидация `len(data) % 2 == 0` перед публикацией | `tts_node._publish_audio` | Нечётная длина → `TTSError` (`TTSBadRequestError`) |
| ❌ НЕ делаем: опубликовать как есть | — | ЗАПРЕЩЕНО (битый int16 на крае, помехи в динамике) |

#### Сценарий F: QoS-mismatch между publisher и subscriber

| Что делаем | Кто делает | Результат |
|---|---|---|
| Publisher (tts_node) и consumer оба используют `best_effort` или оба `reliable` | конфигурация (launch YAML) | Совместимая пара |
| ❌ Publisher `best_effort`, subscriber `reliable` | — | В некоторых DDS — без ошибки, но не ожидаемая семантика |

---

## 5. Sink и воспроизведение

### 5.1 Текущая архитектура (как реализовано в проекте)

В проекте **нет отдельной ноды `audio_play_node`**, и `sound_play`
**не используется**. Воспроизведение выполняется **внутри `tts_node`**
через `AudioPlaybackManager` (singleton), который держит sounddevice
sink напрямую (`src/rob_box_voice/rob_box_voice/audio_playback_manager.py`,
`src/rob_box_voice/rob_box_voice/utils/audio_utils.py`).

```python
# tts_node.py:319
self.playback_manager = AudioPlaybackManager.get_instance()

# tts_node.py:906
# Используем AudioPlaybackManager для синхронизированного доступа
...
self.finished_pub.publish(finished_msg)
```

Публикация на `/voice/audio/speech` — для **внешних** подписчиков
(логирование, телеметрия, record-нода). Сам `tts_node` эту публикацию
не «слушает» для playback.

### 5.2 Альтернативы (когда понадобятся)

| Sink | Когда использовать | Зависимости |
|---|---|---|
| **Direct sounddevice sink** (текущий) | одиночный потребитель в одном процессе | `python3-pyaudio` (через pip) + ALSA/dev |
| **`sound_play` (`sound_play_node`)** | когда нужен std ROS-пакет play builtin звуков (`.wav` файлов из пакета) | `ros-humble-sound-play` |
| **Отдельная `audio_play_node`** | когда TTS должен изолироваться от playback (например, sink на отдельной машине через Zenoh-мост) | новый пакет `rob_box_audio_sink` |

### 5.3 Требования к латентности (sink-сторона)

| Метрика | Целевое | Жёсткий предел | Что делаем, если не укладываемся |
|---|---:|---:|---|
| **TTFA** (time-to-first-audio от старта синтеза) | ≤ 800 ms | ≤ 2000 ms | sync-эндпоинт MiniMax возвращает весь payload одним блоком → TTFA = длительность synthesis + network. Для снижения — переход на WebSocket (`/v1/t2a_ws_v2`) в будущем ADR |
| **Jitter между чанками** (streaming) | ≤ 10 ms | ≤ 50 ms | bounded buffer `audio_qos_depth=10`; KEEP_LAST отбрасывает старые |
| **CPU usage playback** (sounddevice) | ≤ 5% одного ядра Pi4 | ≤ 20% | уменьшить `block_size` или перейти на отдельный sink-поток |
| **Время на декодинг MP3/WAV/OGG** | ≤ 50 ms на 1s utterance | ≤ 200 ms | ffmpeg pipeline overhead; кеширование decoder'а |

---

## 6. Acceptance criteria (mapping к body задачи)

| # | Требование из body | Где в этой спеке |
|---|---|---|
| 1 | Частота дискретизации, битность, число каналов, формат контейнера, рекомендуемые дефолты под audio_common | §1.1 (frozen v1) + §1.2 (MiniMax defaults) |
| 2 | Топики/сообщения ROS2 для публикации аудио, QoS-профиль | §3.1, §3.2, §3.3 |
| 3 | Требования к латентности и буферизации (sound_play или прямые sink) | §5.3 |
| 4 | Статичные vs варьирующиеся от голоса параметры | §2.1, §2.2, §2.3 |
| 5 | Mermaid-диаграмма **потока данных** от API MiniMax до динамика ROS2-ноды | [`../diagrams/minimax-tts-ros2-dataflow.mmd`](../diagrams/minimax-tts-ros2-dataflow.mmd) |
| 6 | Пример ROS2-сообщения | §3.1 (Python-код) |
| 7 | Happy path + случай несовпадения формата (кто отвечает) | §4.1 (конвейер) + §4.2 (таблица ответственности) + §4.3 (6 сценариев) |

---

## 7. Связанные документы

| Документ | Что даёт | Связь |
|---|---|---|
| [ADR-0007](../adr/0007-minimax-tts-integration-final.md) | финальный архитектурный контракт интеграции | parent §2.5 |
| [ADR-0007b](../adr/0007b-minimax-tts-ros2-audio-contract-fragment.md) | frozen PCM-контракт v1, QoS, opt-in streaming | основной документ для §1.1 и §3 |
| [ADR-0007a](../adr/0007a-minimax-tts-reliability-fragment.md) | retry/CB, error mapping | ссылка на §4.3 сценарии ошибок |
| [ADR-0004](../adr/0004-minimax-tts-integration-design.md) | порт, registry, retry | базовая архитектура |
| [ADR-0012](../adr/0012-respeaker-6ch-channel-map-and-mixer.md) | **ReSpeaker 6-канальный режим — карта каналов и `mix_channels` канонический дефолт** | cross-reference: контракт **выходного** потока STT → откуда берётся (`audio_node` capture) |
| [architecture/minimax-tts-architecture.md](./minimax-tts-architecture.md) | реализационная детализация TTS | детальный маппинг полей TTSSettings → T2A body |
| [analysis/tts-current-interface.md](../analysis/tts-current-interface.md) | as-is снапшот PR #907 | первоисточник по current state |
| [research/tts-integration-research.md](../research/tts-integration-research.md) | сводный research-отчёт | первоисточник по MiniMax API |
| [diagrams/minimax-tts-ros2-dataflow.mmd](../diagrams/minimax-tts-ros2-dataflow.mmd) | **Mermaid dataflow-диаграмма** | сопутствует этой спеке |
| [diagrams/minimax-tts-ros2-audio-contract-sequence.mmd](../diagrams/minimax-tts-ros2-audio-contract-sequence.mmd) | sequence-диаграмма (ADR-0007b §2) | sequence-вариант; не дубликат |

> **Cross-reference (capture-сторона).** Эта спецификация описывает **выходной** PCM-контракт от TTS до динамика. **Входной** capture-контракт (микрофон ReSpeaker → `/audio/audio`) живёт в [ADR-0012](../adr/0012-respeaker-6ch-channel-map-and-mixer.md). Новому разработчику: если вы видите 6-канальный capture и параметр `mix_channels` в `audio_node.py` — это **не баг** «лишние каналы». Ch5/Ch6 = playback-референс (несёт чистый цифровой сигнал фразы), A/B-валидированный дефолт `[0,1,2,3,4,5]` (PR #1093). Исключение Ch5-6 (`[0,1,2,3]`) ломает Yandex STT — откачено в `826fc128`.
