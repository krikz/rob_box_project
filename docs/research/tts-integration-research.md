# TTS-интерфейс и интеграция MiniMax — research-отчёт

**Дата:** 2026-07-21
**Назначение:** сводный ресёрч по (1) текущему TTS-интерфейсу в проекте и
(2) публичным возможностям MiniMax TTS API, с явным сопоставлением и
выявленными пробелами. Подготовлен как опорный документ для последующих
задач по реализации (ADR-0007 уже зафиксировал архитектуру; этот отчёт
служит «картой» того, что готово и что нужно доработать).

**Контекст:** Kanban-задача `t_47c7e942` (parent для `t_25b8e221`,
`t_8d714ff0`, `t_9977da5f`, `t_9bb85faf`); исходная нить —
PR #907 `feat: MiniMax TTS integration` в
`feature/harness-p0-foundation`. PR уже содержит и код, и тесты, и ADR.

**Связанные документы:**
- `docs/analysis/tts-current-interface.md` — детальный as-is-снапшот
  интерфейса по PR #907 (502 строки, использован как первоисточник для
  раздела 1).
- `docs/research/minimax-tts-api.md` — полный ресёрч по публичному API
  MiniMax TTS (514 строк, 15 разделов; использован как первоисточник
  для раздела 2).
- `docs/adr/0003-minimax-tts-architecture.md` — архитектурное решение
  по TTS-провайдеру.
- `docs/adr/0004-minimax-tts-integration-design.md` — точки расширения и
  retry-политика.
- `docs/adr/0006-minimax-tts-pydantic-settings-config.md` — pydantic-схема
  конфига.
- `docs/adr/0007-minimax-tts-integration-final.md` — финальный синтез
  (надстройка над 0003/0004).
- Исходники:
  - `src/rob_box_llm/rob_box_llm/tts.py` — ABC `TTSProvider` + value objects
  - `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py` — реализация
  - `src/rob_box_voice/rob_box_voice/tts_node.py` — потребитель (ROS2-нода)

---

## 0. TL;DR

- **Текущий интерфейс** — компактный async-only ABC `TTSProvider`
  (`synthesize()` + `stream()`) с frozen-dataclass'ами
  (`TTSSettings`/`TTSAudio`/`TTSChunk`) и типизированной иерархией
  ошибок (`TTSError` + 4 подкласса). Минимум, достаточный для
  диалогового движка ROBBOX. Конфигурация идёт через ROS-параметры
  `tts_node` + ENV для секретов (`MINIMAX_API_KEY`/`MINIMAX_GROUP_ID`).
- **MiniMax TTS API** — единый sync/SSE-эндпоинт `POST /v1/t2a_v2`
  с bearer-аутентификацией + query-парамом `GroupId`, 8 моделей
  (`speech-2.8/2.6/02/01 × {hd,turbo}`), ~40 системных голосов + клоны,
  7 форматов аудио (mp3/pcm/flac/wav/pcmu_raw/pcmu_wav/opus),
  стриминг через SSE на том же эндпоинте, отдельный WebSocket
  (`/v1/t2a_ws_v2`) и Async (`/v1/t2a_async_v2`) для длинных текстов.
- **Совместимость** — критичные use-case'ы (sync + SSE, auth/group,
  обязательные поля) реализованы корректно. Зафиксированные
  сознательные сужения: нет поддержки `flac/pcmu_*/opus`, нет
  stereo, нет WS-эндпоинта, нет Async. 5 открытых несостыковок
  (валидация `vol`, эвристика маппинга ошибок, лимит `text`,
  проброс `pronunciation_dict`/`timbre_weights`/`voice_modify`,
  миграция `language_boost`).
- **Главный риск интеграции** — отсутствие явной фабрики/реестра
  провайдеров: добавление 3-го TTS-провайдера требует правок в
  двух `__init__.py` плюс в `tts_node._synthesize_and_play`.
  Это унаследованная «детская болезнь» P0 и зафиксирована в
  `current-nodes.md` smell D1.

---

## 1. Текущая архитектура TTS в проекте

### 1.1 Ключевые классы и value objects

| Сущность | Файл:строки | Назначение |
|---|---|---|
| `TTSProvider` (ABC) | `tts.py:121-177` | Async-only контракт: `synthesize()`, `stream()`, `aclose()`. Требует `TTSError` до первого yield; mid-stream — `TTSChunk(finish_reason="error")`. |
| `TTSSettings` (frozen) | `tts.py:52-77` | Параметры вызова: `model`, `voice`, `language`, `speed`, `volume`, `pitch`, `emotion`, `sample_rate`, `format`, `text_normalization`, `extra: MappingProxyType`. Все опциональны, дефолты — на стороне провайдера. |
| `TTSAudio` (frozen) | `tts.py:80-99` | Результат `synthesize()`: `samples: bytes` (int16 LE PCM если `format==PCM`), `sample_rate`, `format`, `raw` (исходный ответ для диагностики). |
| `TTSChunk` (frozen) | `tts.py:102-113` | Фрейм стрима: `samples`, `sample_rate`, `format`, `finish_reason ∈ {None, "stop", "error"}`. Контракт требует **хотя бы один финальный** чанк с `finish_reason`. |
| `TTSFormat` (str-Enum) | `tts.py:33-44` | Контейнер: `PCM / WAV / MP3 / OGG`. Намеренно узкий набор (flac/pcmu_*/opus не включены — см. ADR-0003). |
| `FakeTTSProvider` | `tts.py:185-245` | Детерминированный in-memory провайдер для тестов; кодирует `len(text)` int16-семплов значением 1. `aclose()` идемпотентен. |

**Иерархия ошибок** (`errors.py:63-90`) — отдельная ветка от LLM, чтобы
`except TTSError` не глотал LLM-ошибки и наоборот:

```
TTSError(Exception)
├── TTSRateLimitError    # 429 / quota exhausted
├── TTSTimeoutError      # network / read timeout
├── TTSAuthError         # 401 / 403 / missing API key
└── TTSBadRequestError   # 400-class / unsupported voice/model/param
```

### 1.2 Реализации (`fabric` провайдеров)

| Класс | Файл | Транспорт | Streaming | Тесты |
|---|---|---|---|---|
| `MiniMaxTTSProvider` | `providers/minimax_tts.py:299-673` | `httpx.AsyncClient` (sync + SSE) | ✅ через `client.stream("POST", ...)` | `test_minimax_tts*.py` (7 файлов) |
| `FakeTTSProvider` | `tts.py:185-245` | — | эмулирует одним финальным чанком | в `test_tts_conformance.py` |

> **TL;DR по реестру.** Фабрики/реестра/DI-контейнера **нет**.
> Каждый потребитель `import`'ит и инстанцирует конкретный класс
> напрямую (см. `docs/analysis/tts-current-interface.md` §2). Это
> явное архитектурное решение P0 (ADR-0001 «tiny ABC over factory»),
> но на практике создаёт точку трения при добавлении 3-го провайдера.

**Реэкспорты** (`rob_box_llm/__init__.py`, `providers/__init__.py`)
продублированы — добавлять нового провайдера = править оба файла.

### 1.3 Точки расширения в `tts_node`

`src/rob_box_voice/rob_box_voice/tts_node.py` — единственная ROS2-нода,
работающая с TTS (1526 LOC). Маршрутизация:

```
dialogue_callback(String)
  └─ _run_synthesis_worker    # offload в Thread, держит _synthesis_lock
       └─ _synthesize_and_play
            ├─ provider == "minimax"
            │    ├─ minimax_streaming == False
            │    │    └─ _synthesize_minimax → ... → MiniMaxTTSProvider.synthesize()
            │    └─ minimax_streaming == True
            │         └─ _stream_minimax_chunks → MiniMaxTTSProvider.stream()
            ├─ provider == "yandex" (gRPC, основной для ROBBOX)
            └─ provider == "silero"  (offline-fallback, модель apply_tts)
```

**Важно:** Silero как fallback срабатывает **только** если выбран
путь yandex/silero и yandex недоступен; MiniMax-путь **никогда** не
fallback'ится на Silero (пользователь явно выбрал MiniMax).

### 1.4 Конфигурация

**ROS-параметры** (30 `declare_parameter` в `tts_node.py:168-225`):

| Параметр | Default | Mutable | Назначение |
|---|---|---|---|
| `provider` | `"yandex"` | no | гейт: `∈ {yandex, silero, minimax}` (`tts_node.py:228-233`) |
| `yandex_api_key` / `yandex_voice` / `yandex_speed` | `""` / `"anton"` / `1.0` | last | gRPC-путь |
| `silero_*` (7 шт.) | speaker `baya`, sr 48000, ударения True×4 | no | offline-fallback |
| `minimax_api_key` / `minimax_group_id` | `""` | no (ENV-fallback) | ctor MiniMaxTTSProvider |
| `minimax_voice` / `minimax_model` / `minimax_language` | `"male-qn-qingse"` / `"speech-02-hd"` / `"ru"` | no | TTSSettings |
| `minimax_speed` / `minimax_sample_rate` / `minimax_timeout` | `1.0` / `32000` / `30.0` | runtime через SSML `rate` | `_synthesize_minimax_async` |
| `minimax_format` | `"pcm"` | no | `_parse_format` → `TTSSettings.format` |
| `minimax_max_retries` (0..3) / `minimax_retry_backoff_ms` | `2` / `500` | yes (`max_retries`) | retry-loop |
| `minimax_streaming` | `False` | yes | гейт `synthesize` vs `stream` |
| `audio_*` (5 шт.) | topic `/voice/audio/speech`, sr 16000, best_effort/depth 10 | no | publisher |
| `chipmunk_mode` / `pitch_shift` / `normalize_text` / `volume_db` | True / 1.0 / True / -3.0 | yes | playback post-pipeline |

**ENV-фоллбэки** (только для секретов — намеренное сужение):
`MINIMAX_API_KEY`, `MINIMAX_GROUP_ID`, `YANDEX_API_KEY` читаются
в `tts_node.py:236, 251, 252` если соответствующий ROS-параметр пуст.

**Приоритет** (per ADR `minimax-tts-architecture.md` §2.3, §3):
`launch YAML → ROS-params (declare_parameter defaults) → ENV-fallback → secrets file`.
Секреты только из ENV (по дизайну, чтобы избежать двух источников правды).

**Валидации на старте** (`tts_node.py:228-272`):
- `provider ∈ {yandex, silero, minimax}` иначе raise;
- `audio_output_sample_rate > 0`;
- `audio_qos_reliability ∈ {best_effort, reliable}`;
- `minimax_format` через `TTSFormat(value.lower().strip())` — иначе
  ValueError со списком допустимых;
- `minimax_max_retries` clamp в `[0, 3]`, `minimax_retry_backoff_ms ≥ 0`.

**Валидации в провайдере** (`minimax_tts.py:160-170, 226-232`):
- `volume ∈ [0.0, 10.0]` иначе `TTSBadRequestError` (fail-fast);
- пересечение `settings.extra` с `_RESERVED_PAYLOAD_KEYS` (model,
  text, stream, voice_setting, audio_setting, text_normalization)
  → `TTSBadRequestError`.

### 1.5 Аудио-конвейер после провайдера

```
TTSAudio(samples, sample_rate, format)
  └─ _decode_minimax_audio → np.float32 mono [-1, 1]    # utils.audio_transcode
       └─ _prepare_audio_for_topic  # resample → audio_output_sample_rate (default 16000)
            └─ _publish_audio  # float32 → int16 LE bytes → AudioData().data
                 └─ chipmunk_mode?
                     ├─ True:  resample sr → sr/(2*pitch_shift) → 16000
                     └─ False: resample sr → 16000
                 └─ apply volume_gain = 10^(volume_db/20)
                 └─ mono → stereo np.column_stack([x, x])
```

`AudioData` ROS2-сообщение **не несёт метаданных** (sample_rate и
channels — out-of-band параметры); это отмечено как ограничение в
`minimax-tts-architecture.md` §4.3.

### 1.6 Retry-политика (ADR-0003 §2.6)

Реализована **вокруг** провайдера, в
`_synthesize_minimax_with_retry` (`tts_node.py:1201-1258`):

| Ошибка | Retry | Backoff |
|---|---|---|
| `TTSAuthError` | ❌ нет | — |
| `TTSBadRequestError` | ❌ нет | — |
| `TTSRateLimitError` | ✅ один ретрай | exp backoff |
| `TTSTimeoutError` | ✅ полный бюджет | exp backoff |
| `TTSError` (5xx обёртка) | ✅ полный бюджет | exp backoff |
| прочие | ✅ как `TTSError` (conservative) | exp backoff |

`max_attempts = configured_retries + 1`; `configured_retries ∈ [0, 3]`.

### 1.7 Ограничения текущего интерфейса (зафиксированные)

Из `docs/analysis/tts-current-interface.md` §6:

1. **Нет реестра/фабрики** — добавление 3-го провайдера = правки в
   обоих `__init__.py` + в цепочке `if/elif` в `_synthesize_and_play`.
2. **Chunk-per-frame стрим отсутствует на практике** — MiniMax SSE
   всегда отдаёт весь payload одним чанком; контракт допускает
   чанкинг, но реальная имплементация всегда даёт 1 чанк.
3. **Нет списка голосов** — `TTSSettings.voice` принимает произвольные
   строки; `GET /v1/voices` не привязан.
4. **Нет маппинга voice-id ↔ human-name** для MiniMax (в отличие от
   `_LANGUAGE_ALIASES` для языков).
5. **Два уровня ENV-фоллбэка** для секретов — намеренно, но
   многословно.

---

## 2. Возможности MiniMax TTS API

Сверка на основе `docs/research/minimax-tts-api.md` (дата: 2026-07-21,
источники — официальная документация MiniMax, см. §14 того документа).

### 2.1 Базовая сводка

| Параметр | Значение |
|---|---|
| **Base URL** | `https://api.minimax.io` (глобальный) / `https://api-uw.minimax.io` (сокращение TTFA) |
| **Эндпоинт T2A** | `POST /v1/t2a_v2` (синк и стрим) |
| **Альтернативы** | WS `wss://api.minimax.io/v1/t2a_ws_v2` (§7.4 research); Async `POST /v1/t2a_async_v2` (§8) |
| **Auth** | HTTP header `Authorization: Bearer <MINIM...KEY>` (НЕ OpenAI-совместимый) |
| **Доп. параметр** | Query `GroupId=<MINIMAX_GROUP_ID>` — обязателен (иначе 1004) |
| **Content-Type** | `application/json` |
| **OpenAPI spec** | `https://platform.minimax.io/docs/zh/api-reference/openapi.json` |

### 2.2 Модели (8 штук)

| model | Качество | Эмоции | Interjection | whisper | fluent |
|---|---|---|---|---|---|
| `speech-2.8-hd` | ultra-realistic, sound tags | полный | ✅ | ❌ | ❌ |
| `speech-2.8-turbo` | speed + natural flow | полный | ✅ | ❌ | ❌ |
| `speech-2.6-hd` | ultra-low latency | + fluent + whisper | ❌ | ✅ | ✅ |
| `speech-2.6-turbo` | for agents | + fluent + whisper | ❌ | ✅ | ✅ |
| `speech-02-hd` ← **наш default** | rhythm + similarity | полный | ❌ | ❌ | ❌ |
| `speech-02-turbo` | + multilingual | полный | ❌ | ❌ | ❌ |
| `speech-01-hd` | legacy | полный | ❌ | ❌ | ❌ |
| `speech-01-turbo` | legacy | полный | ❌ | ❌ | ❌ |

> ⚠️ `language_boost` для `speech-01/02` **не** поддерживает Persian,
> Filipino, Tamil (см. §5 research).

### 2.3 Тело запроса (`POST /v1/t2a_v2`)

**Обязательные:** `model`, `text` (≤10 000 символов; >3 000 рекомендуется стрим).

**Опциональные верхнеуровневые:** `stream`, `stream_options`, `voice_setting`,
`audio_setting`, `pronunciation_dict`, `timbre_weights`, `language_boost`,
`voice_modify`, `subtitle_enable`, `subtitle_type`, `output_format`.

**`voice_setting` (см. §4.3 research):**

| Поле | Тип | Диапазон | Default | Заметки |
|---|---|---|---|---|
| `voice_id` | string | — | обязателен | ID системного / клона / сгенерированного голоса |
| `speed` | float | `[0.5, 2]` | `1` | |
| `vol` | float | **`(0, 10]`** ⚠️ | `1` | 0 недопустим, открытый минимум |
| `pitch` | int | `[-12, 12]` | `0` | semitones |
| `emotion` | enum | `happy/sad/angry/fearful/disgusted/surprised/calm/fluent/whisper` | авто | `fluent`/`whisper` только на 2.6 |
| `text_normalization` | bool | — | `false` | улучшает чтение чисел, +latency |
| `latex_read` | bool | — | `false` | только для китайского, формулы в `$$…$$` |

**`audio_setting` (см. §4.4 research):**

| Поле | Допустимые значения | Default | Заметки |
|---|---|---|---|
| `sample_rate` | `8000/16000/22050/24000/32000/44100` | `32000` | Hz |
| `bitrate` | `32000/64000/128000/256000` | `128000` | только для `mp3` |
| `format` | `mp3 / pcm / flac / wav / pcmu_raw / pcmu_wav / opus` | `mp3` | см. наш маппинг ниже |
| `channel` | `1` / `2` | `1` | mono/stereo |
| `force_cbr` | bool | `false` | только при streaming mp3 |

### 2.4 Ответ (sync, `stream=false`)

```jsonc
{
  "data": {
    "audio": "<hex>",          // hex-кодированный буфер
    "status": 2,                // 2 = synthesis completed
    "subtitle_file": "<url>"    // только если subtitle_enable=true
  },
  "extra_info": {
    "audio_length": 11124,      // ms
    "audio_sample_rate": 32000,
    "audio_size": 179926,       // bytes
    "bitrate": 128000,
    "word_count": 163,
    "invisible_character_ratio": 0,
    "usage_characters": 163,    // биллинг — 1 символ = 1 character billable
    "audio_format": "mp3|pcm|flac",  // в ответе только эти 3
    "audio_channel": 1
  },
  "trace_id": "01b8bf9…",       // для саппорта
  "base_resp": {
    "status_code": 0,           // 0 = success
    "status_msg": "success"
  }
}
```

> `data` может быть `null` — реализация обязана проверять
> (`payload.get("data") or {}` уже делает это у нас).

### 2.5 Языки (`language_boost`)

**Enum из 40 значений + `auto`:**
Chinese, Chinese,Yue, English, Arabic, Russian, Spanish, French,
Portuguese, German, Turkish, Dutch, Ukrainian, Vietnamese, Indonesian,
Japanese, Italian, Korean, Thai, Polish, Romanian, Greek, Czech,
Finnish, Hindi, Bulgarian, Danish, Hebrew, Malay, Persian, Slovak,
Swedish, Croatian, Filipino, Hungarian, Norwegian, Slovenian, Catalan,
Nynorsk, Tamil, Afrikaans, `auto`.

- Полные английские имена (`"Russian"`, `"English"`, …) — канонический
  формат. Наш `_LANGUAGE_ALIASES` маппит короткие коды (`ru→Russian`).
- `auto` — рекомендуется, когда язык заранее неизвестен.
- Сужение версий: для `speech-01/02` нет Persian/Filipino/Tamil.

### 2.6 Стриминг

**HTTP/SSE** (`POST /v1/t2a_v2` + `stream=true`):
- Ответ — последовательность JSON-событий (не обёрнутых в массив).
- `data.status`: `1` = промежуточный, `2` = финальный.
- `data.audio` — hex-фрагмент.
- `base_resp.status_code != 0` в любом событии = ошибка.
- `voice_modify` в стриме ограничен только `mp3`.

**WebSocket** (`wss://api.minimax.io/v1/t2a_ws_v2`):
- Синхронный, до 10 000 символов на запрос.
- Долгоживущее соединение, чанки текста (не «один текст — один ответ»).
- В нашей интеграции **не используется**; помечен в ADR-0003 §2.4 как
  «future work».

**Async T2A** (`POST /v1/t2a_async_v2`):
- До **1 000 000** символов на запрос.
- Вход: `text` или `text_file_id` (через `POST /v1/files/upload purpose=t2a_async_input`).
- Output: file в сторадже MiniMax; TTL ссылки = **9 часов**.
- Polling: `GET /v1/query/t2a_async_query_v2?task_id=…`.

### 2.7 Ошибки (`base_resp.status_code`)

Из `errorcode.md` — полная таблица с маппингом на наши `TTSError`-подклассы
(см. §6 research для деталей):

| Код | Наш класс | Сообщение | Действие |
|---|---|---|---|
| `0` | — | success | OK |
| `1000` | `TTSError` | unknown error | retry with backoff |
| `1001` | `TTSTimeoutError` | request timeout | retry with backoff |
| `1002` | `TTSRateLimitError` | rate limit | exp backoff; учитывать RPM |
| `1004` | `TTSAuthError` | not authorized / token not match group | проверить API key + GroupId |
| `1008` | `TTSBadRequestError`* | insufficient balance | уведомить; **не** retry |
| `1024` | `TTSError` | internal error | retry |
| `1026` | `TTSBadRequestError` | input new_sensitive | контент-фильтр |
| `1027` | `TTSBadRequestError` | output new_sensitive | контент-фильтр |
| `1033` | `TTSError` | system error / mysql failed | retry |
| `1039` | `TTSRateLimitError` | token limit (TPM) | backoff |
| `1041` | `TTSError` | conn limit | связаться с поддержкой |
| `1042` | `TTSBadRequestError` | invisible character ratio limit (>10%) | проверить ввод |
| `1043` | `TTSBadRequestError` | ASR similarity check failed | проверить `file_id`/`text_validation` |
| `1044` | `TTSBadRequestError` | clone prompt similarity check failed | проверить prompt audio |
| `2013` | `TTSBadRequestError` | invalid params / glyph format error | проверить payload |
| `20132` | `TTSBadRequestError` | invalid samples or voice_id | проверить `voice_id` |
| `2037` | `TTSBadRequestError` | voice duration too short/long | (clone) |
| `2039` | `TTSBadRequestError` | voice clone voice_id duplicate | выбрать другой |
| `2042` | `TTSAuthError`* | no access to this voice_id | проверить права |
| `2045` | `TTSRateLimitError` | rate growth limit | сгладить всплески |
| `2048` | `TTSBadRequestError` | prompt audio too long | обрезать prompt до 8с |
| `2049` | `TTSAuthError` | invalid API Key | перевыпустить ключ |
| `2056` | `TTSRateLimitError` | usage limit exceeded | ждать 5-часового окна |

`*` — текущая эвристика по `status_msg` может ошибаться (нет явных
маркеров auth/quota). Фикс: `if/elif` по `status_code`.

### 2.8 Rate limits и биллинг

| API | Модели | RPM | TPM |
|---|---|---|---|
| T2A | 2.8-turbo/hd, 2.6-turbo/hd, 02-turbo/hd | 60 | не указан |
| Voice Cloning | — | 60 | — |
| Voice Design | — | 20 | — |

Все T2A делят общий пул **RPM=60** (≈6 запросов / 10 секунд устойчиво).

**Подписки Audio:**

| План | $/мес | points/мес | Voice slots | RPM | T2A v2 |
|---|---|---|---|---|---|
| Starter | $5 | 100 000 | 10 | 10 | ✅ |
| Standard | $30 | 300 000 | 100 | 50 | ✅ |
| Pro | $99 | 1 100 000 | 250 | 200 | ✅ |
| Scale | $249 | 3 300 000 | 500 | 500 | ✅ |
| Business | $999 | 20 000 000 | 800 | 800 | ✅ |

Pay-as-you-go: `extra_info.usage_characters` = биллинговый счётчик.

---

## 3. Сопоставление возможностей API и реализации

Обозначения: ✅ поддерживается; ⚠️ частично / сознательное сужение;
❌ не поддерживается; 🔧 требует адаптера.

### 3.1 Аутентификация и общие

| Фича API | Статус | Где / комментарий |
|---|---|---|
| Bearer-токен в `Authorization` | ✅ | `minimax_tts.py:372-381`; `TTSAuthError` если не задан |
| `GroupId` query-параметр | ✅ | `minimax_tts.py:383-389`; `TTSAuthError` если не задан |
| Ред-акция секретов в логах | ✅ | `_RedactGroupIdFilter` (`minimax_tts.py:271-296`) + `_redact_sensitive_text` |
| Альтернативный base_url (`api-uw.minimax.io` для TTFA) | ✅ | `base_url` kwarg в ctor; default `https://api.minimax.io` |
| Кастомный HTTP-клиент (для тестов) | ✅ | `client=httpx.AsyncClient(...)` kwarg; не закрывается при `aclose()` |

### 3.2 Модели

| Фича API | Статус | Где / комментарий |
|---|---|---|
| `speech-02-hd` (наш default) | ✅ | `minimax_tts.py:329` |
| `speech-02-turbo` | 🔧 | Прокидывается через `TTSSettings.model`; валидации списка нет |
| `speech-2.6-hd`/`turbo`, `2.8-hd`/`turbo`, `01-*` | 🔧 | То же — `TTSSettings.model` без whitelist; **forward-compat через `settings.extra`** |
| Жёсткая валидация `model ∈ known_set` | ❌ | Не реализовано; полагаемся на API-side reject |
| `language_boost` (top-level) | ❌ | Кладём язык в `voice_setting.language` (legacy); см. §11.2 research |

### 3.3 Voice setting

| Фича API | Статус | Где / комментарий |
|---|---|---|
| `voice_id` (обязателен) | ✅ | `minimax_tts.py:155` |
| `speed` `[0.5, 2]` | ✅ | Передаём as-is, без локальной валидации |
| `vol` **`(0, 10]`** | ⚠️ | Валидируем `[0.0, 10.0]` **включительно** (`minimax_tts.py:160-170`); vol=0 пройдёт локально, но API отклонит — нужна правка на `(0, 10]` |
| `pitch` `[-12, 12]` | ✅ | as-is |
| `emotion` (9 значений) | ✅ | as-is; список не валидируем (MiniMax-side reject) |
| `text_normalization` | ✅ | `minimax_tts.py:198-199` |
| `latex_read` | 🔧 | Доступно через `settings.extra` (`_ALLOWED_EXTRA_KEYS`); в `TTSSettings` явно не вынесено |

### 3.4 Audio setting

| Фича API | Статус | Где / комментарий |
|---|---|---|
| `sample_rate` (`8000/16000/22050/24000/32000/44100`) | ✅ | default 32000; без валидации enum'а |
| `bitrate` (`32000/64000/128000/256000`) | ✅ | Хардкод 128000; не вынесен в `TTSSettings` |
| `format` ∈ {pcm, wav, mp3} | ✅ | Маппинг `TTSSFormat → audio_setting.format` (`minimax_tts.py:182-189`) |
| `format` = flac | ❌ | Нет способа попросить; `TTSFormat` не имеет FLAC |
| `format` = pcmu_raw / pcmu_wav / opus | ❌ | Нет способа попросить |
| `format` = ogg | ⚠️ | В нашем `TTSFormat` есть `OGG`, но провайдер молча подменяет на `mp3` (`minimax_tts.py:186-187, 502, 624, 659`) и помечает в `TTSAudio.format` как `MP3`. **Логическая ловушка**: пользователь видит в конфиге `format=ogg`, в `TTSAudio.format` — `mp3`. |
| `channel` = 1 (mono) | ✅ | Хардкод `channel: 1` |
| `channel` = 2 (stereo) | ❌ | Не вынесено; open-issue в ADR-0003 §5 |
| `force_cbr` (только streaming mp3) | 🔧 | Доступно через `settings.extra` |

### 3.5 Язык

| Фича API | Статус | Где / комментарий |
|---|---|---|
| BCP-47 (`ru`, `en`, …) | 🔧 | `_LANGUAGE_ALIASES` (`minimax_tts.py:53-66`) маппит в полные имена (`Russian`, `English`, …) |
| Полные имена (`Russian`, …) | ✅ | Pass-through |
| `auto` | 🔧 | Можно передать через `TTSSettings.language="auto"` — пройдёт через `_map_language` |
| 40-язычный полный список `language_boost` | ❌ | Используем legacy `voice_setting.language`; миграция отложена (см. §11.2 research) |

### 3.6 Расширенные фичи (forward-compat)

| Фича API | Статус | Где / комментарий |
|---|---|---|
| `pronunciation_dict` | 🔧 | В `_ALLOWED_EXTRA_KEYS` (`minimax_tts.py:263`); в `TTSSettings` явно не вынесено |
| `timbre_weights` | 🔧 | В `_ALLOWED_EXTRA_KEYS` (`minimax_tts.py:264`) |
| `voice_modify` (pitch/intensity/timber + sound_effects) | 🔧 | Доступно через `settings.extra` |
| `subtitle_enable` / `subtitle_type` | 🔧 | `subtitle_timestamp` в allow-list, но полноценного enum-поля в `TTSSettings` нет |
| `output_format=url` (TTL 24ч) | ❌ | Не поддерживается; sync всегда `hex` |
| `stream_options.exclude_aggregated_audio` | ❌ | Не пробрасывается |

### 3.7 Стриминг

| Фича API | Статус | Где / комментарий |
|---|---|---|
| HTTP/SSE на `/v1/t2a_v2` (stream=true) | ✅ | `minimax_tts.py:515-661`; каждый JSON-event → `TTSChunk`, в конце пустой terminal chunk с `finish_reason="stop"` |
| Парсинг SSE как `data:…\n\n` или raw JSON | ✅ | `minimax_tts.py:573-588` |
| Распознавание `[DONE]`-маркера | ✅ | `minimax_tts.py:579` (defensive — MiniMax обычно не шлёт, но если придёт — выйдем) |
| Pre-yield ошибки → `raise TTSError` | ✅ | `minimax_tts.py:609, 646` |
| Mid-stream ошибки → `TTSChunk(finish_reason="error")` | ✅ | `minimax_tts.py:606-608, 643-644` |
| `data.status` 1 (промежуточный) vs 2 (финальный) | ⚠️ | Парсим, но **не различаем** — `TTSChunk.finish_reason` ставится только в финальном empty chunk |
| Фиксированный размер PCM-фрейма | ❌ | MiniMax SSE не даёт; ADR-0003 §2.4 фиксирует это как «sync primary + SSE optional», реальный chunking = future work через WS |
| WebSocket `wss://api.minimax.io/v1/t2a_ws_v2` | ❌ | Не реализован; ADR-0003 §2.4 «reserved» |
| Async T2A `POST /v1/t2a_async_v2` | ❌ | Не реализован; для текстов >10 000 символов |

### 3.8 Лимиты текста и лимитирующие проверки

| Фича API | Статус | Где / комментарий |
|---|---|---|
| Лимит `text ≤ 10 000` | ⚠️ | Провайдер **не** валидирует; ADR-0003 §5 open-issue. Sync на 10001 символов вернёт ошибку от API; но лучше валидировать раньше (на стороне `tts_node`/`dialogue_node`). |
| Лимит `>3 000` → рекомендуется стрим | ℹ️ | Не enforced; см. `tts_node.minimax_streaming` |

### 3.9 Ошибки → наши исключения

| Категория | Статус | Где / комментарий |
|---|---|---|
| HTTP 401/403 → `TTSAuthError` | ✅ | `_map_exception` (`minimax_tts.py:115-116`) |
| HTTP 429 → `TTSRateLimitError` | ✅ | `minimax_tts.py:117-118` |
| HTTP 4xx (прочие) → `TTSBadRequestError` | ✅ | `minimax_tts.py:119-120` |
| `httpx.TimeoutException` → `TTSTimeoutError` | ✅ | `minimax_tts.py:108-111` |
| `httpx.HTTPError` → `TTSTimeoutError` | ✅ | `minimax_tts.py:122-125` |
| `base_resp.status_code` → подкласс | ⚠️ | Эвристика по `status_msg` (`minimax_tts.py:443-449`); не покрывает коды 1008/2042/1039 надёжно — см. §11.1 research |
| Non-JSON ответ | ✅ | `minimax_tts.py:418-426` → `TTSError` |
| Response без `data.audio` | ✅ | `minimax_tts.py:467` → `TTSError` |
| Стрим без чанков | ✅ | `minimax_tts.py:648-652` → `TTSError` |

### 3.10 Конфигурация и lifecycle

| Фича | Статус | Где / комментарий |
|---|---|---|
| ENV-fallback для `MINIMAX_API_KEY` | ✅ | `minimax_tts.py:347` |
| ENV-fallback для `MINIMAX_GROUP_ID` | ✅ | `minimax_tts.py:348` |
| Свои дефолты voice/model | ✅ | `default_voice=`, `default_model=` kwargs |
| `aclose()` идемпотентен | ✅ | `minimax_tts.py:663-670` |
| Закрытие только self-owned клиента | ✅ | `if self._owns_client and not self._client.is_closed` |
| HTTP-таймаут | ✅ | `timeout=` kwarg (default 30.0) |
| pydantic-схема конфига | ✅ | ADR-0006 (`docs/adr/0006-minimax-tts-pydantic-settings-config.md`) |

---

## 4. Пробелы и риски интеграции

### 4.1 Архитектурные риски

1. **Нет фабрики/реестра TTS-провайдеров** (severity: medium).
   Добавление 3-го провайдера = правки в двух `__init__.py` +
   цепочка `if/elif` в `_synthesize_and_play`. Это унаследованная
   «детская болезнь» P0 (см. `current-nodes.md` smell D1).
   **Решение:** завести `TTSProviderRegistry` (аналог `PROVIDERS` для
   LLM), `register(name)` decorator, `build_tts(name, cfg)` factory.
   Это даст чистый extension point и устранит дублирование re-exports.

2. **Чанк-per-frame стрим отсутствует** (severity: medium для TTFA).
   Контракт `TTSProvider.stream()` допускает чанкинг, но MiniMax SSE
   всегда отдаёт полный payload одним чанком → TTFA ≈ длительности
   всего utterance. Для диалога это неоптимально.
   **Решение:** переход на WebSocket `wss://.../v1/t2a_ws_v2`
   (ADR-0003 §2.4 «future work»). Это снимет TTFA до ~300 ms.

3. **OGG fallback без явного предупреждения** (severity: low).
   `TTSSettings(format=TTSFormat.OGG)` молча превращается в mp3 на
   стороне провайдера; `TTSAudio.format` маркируется как `MP3`. Если
   пользователь полагается на формат downstream — он получит
   сюрприз. **Решение:** либо убрать `OGG` из `TTSFormat`, либо
   логировать warning + явно документировать.

### 4.2 Несостыковки с документированным API

4. **`vol == 0` не rejected локально** (severity: low, easy fix).
   Спека говорит `vol ∈ (0, 10]`; наша валидация `0.0 <= vol <= 10.0`.
   vol=0 пройдёт локально и вернётся от API как 400. **Решение:**
   `minimax_tts.py:163` поменять на `not (0.0 < vol <= 10.0)`.

5. **Эвристика маппинга ошибок по `status_msg`** (severity: medium).
   Для кодов `1008/2042/1039` строковые маркеры в `status_msg`
   ненадёжны (см. §11.1 research). **Решение:** явный `if/elif` по
   `status_code` (таблица в §2.7 этого документа).

6. **`text` лимит не валидируется** (severity: low до тех пор, пока
   `dialogue_node` сам не отправит >10 000; medium после).
   **Решение:** либо валидация в провайдере
   (`raise TTSBadRequestError("text > 10000 chars, use async endpoint")`),
   либо в `tts_node` (быстрый отказ без сетевого вызова).

7. **`language_boost` vs legacy `voice_setting.language`** (severity: low).
   Работает, но используем старое/внутреннее поле. MiniMax может
   убрать. **Решение:** однострочная миграция на `language_boost`;
   отложена (ADR-0003 §7 «open questions»).

8. **Нет voice-id ↔ human-name маппинга** (severity: low).
   `_LANGUAGE_ALIASES` есть, для голосов — нет. UX-проблема, не
   функциональная. **Решение:** завести `_VOICE_ALIASES` + (опц.)
   endpoint `GET /v1/voices` для перечисления.

### 4.3 Расширенные фичи — не вынесены в `TTSSettings`

9. **`pronunciation_dict`, `timbre_weights`, `voice_modify`, `subtitle_*`, `force_cbr`** доступны через `settings.extra` (`_ALLOWED_EXTRA_KEYS`), но **не вынесены** в typed-поля `TTSSettings`. Это
   снижает discoverability и валидацию. **Решение:** добавить поля в
   `TTSSettings` (dataclass-only, опциональные), в `_build_payload`
   проверять в первую очередь typed-поле и fallback'ить на extra.

### 4.4 Форматы и каналы

10. **`flac`, `pcmu_raw`, `pcmu_wav`, `opus` не выбираемы** (severity:
    low — намеренное сужение ADR-0003). Внутренний аудиоконвейер ROS2
    ожидает PCM/WAV/MP3, transcoding — на стороне `tts_node`. **Решение:**
    при появлении требований — добавить `FLAC`/`PCMU`/`OPUS` в `TTSFormat`
    + encoder'ы в `utils/audio_transcode.py`.

11. **Stereo (`channel=2`) не выбираемо** (severity: low). Хардкод
    `channel: 1` в `_build_payload`. **Решение:** `TTSSettings.channel`
    (int, 1/2), default 1.

### 4.5 Тестовые покрытия и операционные риски

12. **Mock-инфраструктура есть, но не покрывает полный error matrix**
    (severity: low). `test_minimax_tts_errors_parametrized.py` —
    644 строки, parametrised по HTTP-статусам. Но `base_resp.status_code`
    ошибки (1008, 2042, …) покрыты не все. **Решение:** добавить
    parametrised test по полной таблице §2.7.

13. **Нет e2e-проверки TTFA в CI** (severity: medium).
    `tts_audio_bench/scripts/run_bench.py` (990 строк) умеет
    замерять TTFA, но не входит в CI-workflow
    (`.github/workflows/G-TTS-Provider-Tests.yml` — 129 строк, смотрит
    только conformance + provider-level tests).
    **Решение:** stage-gate в CI на `tts_audio_bench` против mock-сервера
    (baseline TTFA ≤ X ms) — даст регрессионный сигнал без
    зависимости от реального API.

14. **Документация MiniMax активно меняется** (severity: medium).
    Спека 8 моделей уже разрослась с 4 до 8 за ~2 месяца; добавились
    `speech-2.8`, `speech-2.6`, новые эмоции. **Решение:** перед
    крупными правками сверяться с `openapi.json`, а не только
    HTML-страницей (`docs/research/minimax-tts-api.md` §14).

15. **Секреты в ENV — потенциальная утечка через дочерние процессы**
    (severity: low). `os.getenv(...)` в `__init__` провайдера
    (`minimax_tts.py:347-348`) — если провайдер импортируется в
    контексте, где ENV прокидывается в другие сервисы (subprocess
    без env-filter), возможна утечка. **Решение:** `_RedactGroupIdFilter`
    уже стоит на `httpx`-логгере; для ENV ничего не сделано — это
    сознательный trade-off.

---

## 5. Рекомендации (для последующих задач)

Расположены по убыванию приоритета; ссылки на соответствующие секции
выше.

| # | Что | Почему | Где править |
|---|---|---|---|
| 1 | **TTSProvider registry / factory** | Устраняет smell D1, делает добавление 3-го провайдера однострочным | новый `registry.py` + правки `__init__.py`/`tts_node.py` |
| 2 | **Маппинг ошибок по `status_code`** | Покрывает коды 1008/2042/1039 надёжно; не зависит от строковых маркеров | `minimax_tts.py:441-449` → `if/elif status_code in {...}` |
| 3 | **vol=0 → reject** | Спека требует `(0, 10]`; текущая валидация пропустит | `minimax_tts.py:163` (`not (0.0 < vol <= 10.0)`) |
| 4 | **Миграция на `language_boost`** | Forward-compat; однострочно | `minimax_tts.py:_build_payload` |
| 5 | **Тип-поля в `TTSSettings`** для pronunciation_dict/timbre_weights/voice_modify/subtitle | Discoverability + валидация | `tts.py:52-77` + `_build_payload` |
| 6 | **WebSocket-эндпоинт** | Снижает TTFA с ~duration до ~300 ms | новый `MiniMaxWSTTSProvider` (или второй mode) |
| 7 | **OGG в `TTSFormat`** | Либо убрать, либо явно документировать fallback на mp3 | `tts.py:44` или docstring |
| 8 | **Stereo (`channel=2`)** | Если понадобится для multi-channel voice | `TTSSettings.channel` + `_build_payload` |
| 9 | **CI-gate на TTFA** (через `tts_audio_bench` + mock-сервер) | Регрессионный сигнал без зависимости от живого API | `.github/workflows/G-TTS-Provider-Tests.yml` |
| 10 | **Async T2A** | Для текстов >10 000 символов | новый `MiniMaxAsyncTTSProvider` |

---

## 6. Резюме для последующих задач

- **Что готово:** ABC + value objects + `MiniMaxTTSProvider`
  (sync + SSE) + `FakeTTSProvider` + типизированные ошибки +
  RETRY-policy + ENV-fallback для секретов + pydantic-конфиг (ADR-0006) +
  ROS-интеграция в `tts_node` + CI-workflow + mock-сервер + audio bench.
- **Что не готово:** registry/factory; chunk-per-frame stream;
  `language_boost`; voice-id ↔ human-name; расширенные поля в
  `TTSSettings`; stereo; WS; Async.
- **Что глючит/несоответствует:** vol=0 валидация; эвристика
  маппинга ошибок по `status_msg`; тихий fallback `OGG → MP3` без
  warning; отсутствие TTFA-gate в CI.
- **Что осознанно сужено (не баги):** форматы (нет flac/pcmu_*/opus);
  канал (mono only); нет WS/Async; нет `output_format=url`.
- **Что требует операционного внимания:** ENV-секреты + rate-limit
  RPM=60 на аккаунт + биллинг по `usage_characters` (см. §2.8).

---

## 7. Acceptance criteria — проверка покрытия исходного ТЗ

Исходный ТЗ (см. body задачи `t_47c7e942`) требует покрыть все 5
пунктов материалами для последующих задач:

| Пункт ТЗ | Покрыт в | Секция этого отчёта |
|---|---|---|
| (1) выгрузить ветку PR #907 | `git log pr-907` (см. секцию 0; PR смержен в `feature/harness-p0-foundation`) | §0, контекст |
| (2) прочитать существующий TTS-интерфейс (ABC, провайдеры, конфиг, фабрика) | `tts.py`, `providers/`, `tts_node.py` | §1 (1.1–1.7) |
| (3) изучить публичную документацию MiniMax TTS (эндпоинты, форматы, голоса, модели, лимиты, стриминг, ошибки, auth) | `docs/research/minimax-tts-api.md` (514 строк) | §2 (2.1–2.9) |
| (4) выдать аналитический отчёт Markdown с 3 разделами + список референсных ссылок | этот документ | §1, §2, §3–§5; §8 |
| (5) без реализации, только разведка | да — никаких правок кода не сделано; только прочитан и зафиксирован | весь отчёт |

**Все 5 пунктов покрыты; список референсных ссылок — в §8.**

---

## 8. Референсные ссылки

### 8.1 Исходники в репозитории

**Реализация TTS-провайдеров:**
- `src/rob_box_llm/rob_box_llm/tts.py` — ABC `TTSProvider`, value objects, `FakeTTSProvider` (255 строк)
- `src/rob_box_llm/rob_box_llm/errors.py` — `TTSError` + 4 подкласса (106 строк)
- `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py` — `MiniMaxTTSProvider` (673 строки)
- `src/rob_box_llm/rob_box_llm/providers/__init__.py` — re-exports провайдеров (24 строки)
- `src/rob_box_llm/rob_box_llm/__init__.py` — public surface пакета (102 строки)
- `src/rob_box_voice/rob_box_voice/tts_node.py` — потребитель (1526 строк)
- `src/rob_box_voice/rob_box_voice/utils/audio_transcode.py` — PCM/WAV/MP3/OGG декодер (281 строка)

**Конфигурация:**
- `src/rob_box_voice/config/voice_assistant.yaml` — runtime-конфиг (включая `provider`, `minimax_*` параметры)
- `src/rob_box_voice/launch/voice_assistant.launch.py` — launch с `provider:=minimax` arg
- `.env.example` — переменные окружения

**Тесты:**
- `src/rob_box_llm/test/test_tts_conformance.py` — соответствие контракту `TTSProvider`
- `src/rob_box_llm/test/test_tts_provider_contract.py` — контракт-тесты
- `src/rob_box_llm/test/test_tts_errors.py` — иерархия ошибок
- `src/rob_box_llm/test/test_tts_value_objects.py` — dataclasses
- `src/rob_box_llm/test/test_minimax_tts_provider.py` — provider-level (599 строк)
- `src/rob_box_llm/test/test_minimax_tts_formats.py` — форматы (432 строки)
- `src/rob_box_llm/test/test_minimax_tts_streaming.py` — стрим (435 строк)
- `src/rob_box_llm/test/test_minimax_tts_errors_parametrized.py` — error matrix (644 строки)
- `src/rob_box_llm/test/test_minimax_tts_request_params_and_leak_guard.py` — leak-guard (403 строки)
- `src/rob_box_llm/test/test_minimax_tts_logging.py` — логирование (94 строки)
- `src/rob_box_voice/test/unit/tts/test_minimax_integration.py` — integration (564 строки)
- `src/rob_box_voice/test/unit/utils/test_audio_transcode.py` — transcode utils (232 строки)

**CI / инструменты:**
- `.github/workflows/G-TTS-Provider-Tests.yml` — CI-workflow (129 строк)
- `tts_audio_bench/` — бенч для TTFA, mock-сервер, audio validator
- `tools/mock_minimax_server.py` — локальный mock MiniMax (671 строка)
- `tools/audio_capture_harness/` — захват audio из DDS-топика

### 8.2 Документация проекта

**ADR (архитектурные решения):**
- `docs/adr/0001-harness-architecture.md` — P0 foundation, выбор «tiny ABC over factory»
- `docs/adr/0002-minimax-provider.md` — LLM-провайдер MiniMax (верхний уровень)
- `docs/adr/0003-minimax-tts-architecture.md` — TTS-архитектура
- `docs/adr/0004-minimax-tts-integration-design.md` — точки расширения, retry, registry hook
- `docs/adr/0006-minimax-tts-pydantic-settings-config.md` — pydantic-схема конфига
- `docs/adr/0007-minimax-tts-integration-final.md` — финальный синтез
- `docs/adr/0007a-minimax-tts-reliability-fragment.md` — reliability (фрагмент)
- `docs/adr/0007b-minimax-tts-ros2-audio-contract-fragment.md` — ROS2 AudioData contract

**Анализ и архитектура:**
- `docs/analysis/tts-current-interface.md` — as-is снапшот (502 строки, главный источник §1)
- `docs/analysis/current-nodes.md` — Dialog/Persistent/Telegram cross-cutting (893 строки)
- `docs/analysis/nodes-current-state.md` — узлы в текущем состоянии
- `docs/architecture/minimax-tts-integration-design.md` — дизайн-контракт (530 строк)
- `docs/architecture/minimax-tts-architecture.md` — архитектура TTS (401 строка)
- `docs/architecture/nodes-refactor-plan.md` — план рефакторинга (999 строк)
- `docs/refactoring-plan.md` — общий план (514 строк)

**Research:**
- `docs/research/minimax-tts-api.md` — полный ресёрч по MiniMax API (514 строк, 15 разделов; главный источник §2)
- `docs/research/agent-harnesses-best-practices.md` — обзор харнесов (330 строк)

**Гайды и конфиг-примеры:**
- `docs/guides/MINIMAX_TTS.md` — пользовательский гайд (591 строка)
- `docs/guides/examples/minimax_tts.yaml` — пример YAML-конфига (153 строки)

**Диаграммы:**
- `docs/diagrams/minimax-tts-final-class.mmd` — финальная class-диаграмма
- `docs/diagrams/minimax-tts-integration-class.mmd` — class-диаграмма интеграции
- `docs/diagrams/minimax-tts-integration-sequence.mmd` — sequence-диаграмма интеграции
- `docs/diagrams/minimax-tts-sequence.mmd` — общая sequence-диаграмма
- `docs/diagrams/minimax-tts-ros2-audio-contract-sequence.mmd` — AudioData contract

**Прочее:**
- `docs/README.md` — индекс документации
- `docs/reports/PERSISTENT_NODES_INTERFACES.md` — отчёт по persistent-узлам (545 строк)
- `BLOCKERS.md` — зафиксированные блокеры
- `.planning/ROADMAP.md` — roadmap проекта
- `.planning/STATE.md` — текущее состояние

### 8.3 Внешние источники (MiniMax docs)

(Из `docs/research/minimax-tts-api.md` §14 — официальная документация MiniMax)

| Документ | URL | Где использован |
|---|---|---|
| T2A HTTP (OpenAPI 3.1 + спека полей) | https://platform.minimax.io/docs/api-reference/speech-t2a-http.md | §2.2–§2.6 |
| Error Code Reference | https://platform.minimax.io/docs/api-reference/errorcode.md | §2.7 |
| Rate Limits | https://platform.minimax.io/docs/guides/rate-limits.md | §2.8 |
| Async Long TTS Guide | https://platform.minimax.io/docs/guides/speech-t2a-async.md | §2.6 |
| Sync TTS WebSocket Guide | https://platform.minimax.io/docs/guides/speech-t2a-websocket.md | §2.6 |
| T2A WebSocket (endpoint reference) | https://platform.minimax.io/docs/api-reference/speech-t2a-websocket.md | §2.6 |
| Voice Management / Get Voice | https://platform.minimax.io/docs/api-reference/voice-management-get.md | §3.3 (voice-id маппинг) |
| System Voice ID List | https://platform.minimax.io/docs/faq/system-voice-id.md | §3.3 |
| Audio Subscription Pricing | https://platform.minimax.io/docs/guides/pricing-speech.md | §2.8 |
| OpenAPI spec (полная) | https://platform.minimax.io/docs/zh/api-reference/openapi.json | §2.1 (формальная схема) |
| Документация-индекс | https://platform.minimax.io/docs/llms.txt | навигация |

### 8.4 Kanban-задачи (контекст)

- `t_47c7e942` — эта задача (parent для архитектурных)
- `t_25b8e221`, `t_8d714ff0`, `t_9977da5f`, `t_9bb85faf` — дочерние задачи
  (фасадные реализации по разным направлениям)
- `t_aeb3c867` — предыдущая задача (ресёрч MiniMax API, создала
  `docs/research/minimax-tts-api.md`)
- `t_ebdc4c99` — предыдущая задача (харнесы best practices)

---

**Конец отчёта.**