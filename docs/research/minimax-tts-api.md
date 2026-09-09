# MiniMax TTS API — публичный контракт

**Дата:** 2026-07-21
**Статус:** Draft (v1) — research-реферат, источник сверялся с официальной документацией
**Автор:** analyst (Hermes Agent)
**Контекст:** Kanban-задача `t_aeb3c867` (parent для архитектурных задач `t_25b8e221`, `t_b4cb8948`).
**Связанные документы:**
- `docs/adr/0003-minimax-tts-architecture.md` — ADR, описывающее уже принятую интеграцию
- `docs/adr/0002-minimax-provider.md` — ADR верхнего уровня (Accepted)
- `docs/architecture/minimax-provider.md` — разбор PR #907, as-is → target
- `docs/architecture/minimax-tts-architecture.md` — reference по нашему `MiniMaxTTSProvider`
- `docs/research/agent-harnesses-best-practices.md` — общий research по харнесам
- Исходники референсной реализации (уже в `feature/harness-p0-foundation`):
  - `src/rob_box_llm/rob_box_llm/tts.py` — ABC `TTSProvider`, value objects (`TTSSettings`, `TTSAudio`, `TTSChunk`, `TTSFormat`)
  - `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py` — `MiniMaxTTSProvider` (sync + SSE-стрим)
  - `src/rob_box_llm/test/conftest.py` + `test_minimax_*` — тесты против `respx`-моков

**Назначение:** зафиксировать то, что говорит о себе сам MiniMax API, и сопоставить с уже существующим в проекте кодом. Это research-реферат, не дизайн-документ — правки контракта в реализации здесь не предлагаются; для архитектурных решений есть ADR-0002/0003.

---

## 0. Зачем этот документ

В проекте уже есть рабочая интеграция MiniMax TTS (см. `MiniMaxTTSProvider`), и ряд архитектурных решений зафиксирован в ADR-0002 / ADR-0003. Этот документ **не дублирует их**, а собирает компактный технический справочник по **самому публичному API MiniMax** на дату исследования (21 июля 2026): эндпоинты, модели, голоса, форматы, стриминг, ошибки, rate-limit, биллинг — то, что нужно знать разработчику/агенту, чтобы понимать и безопасно расширять существующую интеграцию.

Все цитаты схемы — из официальной `platform.minimax.io/docs/api-reference/speech-t2a-http.md` (OpenAPI 3.1) и сопутствующих страниц. Где наша реализация и документация расходятся — это явно отмечено в разделе «Сверка с реализацией».

---

## 1. Базовые параметры

| Параметр        | Значение                                                                          |
| --------------- | --------------------------------------------------------------------------------- |
| Base URL        | `https://api.minimax.io` (глобальный) / `https://api-uw.minimax.io` (сокращение TTFA) |
| Эндпоинт T2A    | `POST /v1/t2a_v2` (синхронный, синк/стрим)                                        |
| Аутентификация  | HTTP `Authorization: Bearer <MINIM…KEY>` — не OpenAI-совместимо                  |
| Доп. параметр   | Query `GroupId=<MINIM…GROUP_ID>` — обязателен                                    |
| Content-Type    | `application/json`                                                                |
| OpenAPI spec    | `https://platform.minimax.io/docs/zh/api-reference/openapi.json`                  |
| Альтернативы    | WebSocket (`/v1/t2a_ws_v2`?) — см. §7; Async T2A (`/v1/t2a_async_v2`) — §8       |

> Эндпоинт `/v1/t2a_v2` имеет две ноды: `api.minimax.io` и `api-uw.minimax.io`. Вторая описана как «Alternative Endpoint, Reduced Time to First Audio (TTFA)» — для нашего `tts_node` это кандидат на уменьшение задержки первого звука.

---

## 2. Аутентификация и синтез-пара (Endpoints 1–2)

### 2.1. Аутентификация

| Поле          | Где                                              | Описание                                                            |
| ------------- | ------------------------------------------------ | ------------------------------------------------------------------- |
| `Authorization` | HTTP header, `Bearer <MINIM…KEY>`                | API key. Получить: Account Management → API Keys.                  |
| `GroupId`     | Query-параметр                                   | Идентификатор аккаунта/организации. Без него API вернёт 1004 (`not authorized`). |

### 2.2. Сценарии, в которых API отказывает в auth/quota

Документированные коды в `base_resp.status_code` (см. §6):

- `1004` — `not authorized / token not match group / cookie is missing`. **Решение:** проверить API key, убедиться, что `GroupId` совпадает с тем, на котором выпущен ключ.
- `2049` — `invalid API Key`. **Решение:** перевыпустить ключ.
- `1008` — `insufficient balance`. **Решение:** пополнить баланс.
- `2056` — `usage limit exceeded`. **Решение:** дождаться 5-часового окна.

---

## 3. Модели (`model`)

Список допустимых `model` (см. §9 о нашей реализации — там жёстко зашит `speech-02-hd`):

| model              | Описание                                                        | Эмоции                     | Interjection-теги | `whisper` |
| ------------------ | --------------------------------------------------------------- | -------------------------- | ----------------- | --------- |
| `speech-2.8-hd`    | Ultra-realistic quality featuring sound tags.                   | да (полный список)         | **да**            | нет       |
| `speech-2.8-turbo` | Seamless speed meets natural flow.                              | да                         | **да**            | нет       |
| `speech-2.6-hd`    | Ultra-low latency, intelligence parsing, naturalness.           | да + `fluent`, `whisper`   | нет               | **да**    |
| `speech-2.6-turbo` | Faster, ideal for agents.                                       | да + `fluent`, `whisper`   | нет               | **да**    |
| `speech-02-hd`     | Superior rhythm + similarity (наш дефолт).                       | да                         | нет               | нет       |
| `speech-02-turbo`  | То же + улучшенная мультиязычность.                             | да                         | нет               | нет       |
| `speech-01-hd`     | Старая версия.                                                  | да                         | нет               | нет       |
| `speech-01-turbo`  | Старая версия.                                                  | да                         | нет               | нет       |

> ⚠️ `language_boost` для speech-01/02 **не поддерживает** Persian, Filipino, Tamil — это документировано в спеке.

---

## 4. Тело запроса (`POST /v1/t2a_v2`)

### 4.1. Обязательные поля

```json
{
  "model": "speech-2.8-hd",
  "text": "<= 10000 символов; для >3000 рекомендуется стриминг>"
}
```

### 4.2. Опциональные верхнеуровневые поля

| Поле               | Тип        | Назначение                                                                                                  |
| ------------------ | ---------- | ----------------------------------------------------------------------------------------------------------- |
| `stream`           | bool       | Включить SSE-стриминг. По умолчанию `false`.                                                                |
| `stream_options`   | object     | `exclude_aggregated_audio`: если `true`, последний чанк не содержит полной склейки.                         |
| `voice_setting`    | object     | см. §4.3                                                                                                    |
| `audio_setting`    | object     | см. §4.4                                                                                                    |
| `pronunciation_dict` | object  | `tone: ["orig/replacement", …]` — IPA, пиньинь с тоном, кантонез ёутпинг, хирагана/катакана, текст.        |
| `timbre_weights`   | array      | Микс до 4 голосов, `weight ∈ [1,100]`. Используется вместо `voice_id` (legacy-поле).                       |
| `language_boost`   | string     | Полное имя языка или `auto` (см. §5).                                                                      |
| `voice_modify`     | object     | pitch/intensity/timber `[-100,100]` + `sound_effects` ∈ `spacious_echo`, `auditorium_echo`, `lofi_telephone`, `robotic`. |
| `subtitle_enable`  | bool       | Включить субтитры. По умолчанию `false`.                                                                    |
| `subtitle_type`    | enum       | `sentence` / `word` / `word_streaming` (последнее только при `stream=true`).                                |
| `output_format`    | enum       | `hex` (по умолчанию) или `url` (только sync, URL живёт **24 часа**). В стриме — только `hex`.              |
| `language_boost`   | enum       | см. §5.                                                                                                     |

### 4.3. `voice_setting`

| Поле                | Тип    | Диапазон                  | По умолчанию | Назначение                                   |
| ------------------- | ------ | ------------------------- | ------------ | -------------------------------------------- |
| `voice_id`          | string | —                         | обязателен   | ID голоса (системного, клона, сгенерированного). |
| `speed`             | float  | `[0.5, 2]`                | `1`          | Скорость речи.                               |
| `vol`               | float  | `(0, 10]`                 | `1`          | Громкость. **0 недопустим, открытый минимум.** |
| `pitch`             | int    | `[-12, 12]`               | `0`          | Сдвиг тона в полутонах.                      |
| `emotion`           | enum   | `happy/sad/angry/fearful/disgusted/surprised/calm/fluent/whisper` | авто | Контроль эмоций. По умолчанию модель сама выбирает. |
| `text_normalization`| bool   | —                         | `false`      | Улучшает чтение чисел; +latency.             |
| `latex_read`        | bool   | —                         | `false`      | Чтение LaTeX-формул (только китайский; формулы в `$$…$$`). |

> Эмоции `fluent` и `whisper` доступны только на `speech-2.6-hd` / `speech-2.6-turbo`. На `speech-2.8-*` `whisper` отключён.

### 4.4. `audio_setting`

| Поле         | Допустимые значения (документированные)                              | По умолчанию | Примечание |
| ------------ | ------------------------------------------------------------------- | ------------ | ---------- |
| `sample_rate`| `8000 / 16000 / 22050 / 24000 / 32000 / 44100`                      | `32000`      | Гц.        |
| `bitrate`    | `32000 / 64000 / 128000 / 256000`                                   | `128000`     | Только для `mp3`. |
| `format`     | `mp3 / pcm / flac / wav / pcmu_raw / pcmu_wav / opus`               | `mp3`        | `pcmu_*` — G.711 µ-law 8 kHz; `opus` — Ogg/Opus. |
| `channel`    | `1` (mono) / `2` (stereo)                                           | `1`          |            |
| `force_cbr`  | bool                                                                | `false`      | Только при **streaming mp3**.       |

> ⚠️ MiniMax в `extra_info.audio_format` теперь возвращает `mp3 | pcm | flac` (см. §4.6), а `format` в `audio_setting` поддерживает уже 7 значений. **Несостыковка** реализации: наш `_build_payload` маппит `TTSFormat.OGG → "mp3"`, что соответствует спеке; но `TTSFormat` в коде имеет `WAV/MP3/PCM/OGG` — у нас нет способа попросить `flac`, `pcmu_*`, `opus`. Это сознательное сужение (см. ADR-0003).

### 4.5. Pause control и inline-произношение в `text`

| Фича                       | Синтаксис                              | Ограничения                     |
| -------------------------- | -------------------------------------- | ------------------------------- |
| Pause (`<#x#>`)            | `<#1.5#>` — пауза 1.5 сек              | `[0.01, 99.99]`, 2 знака, не подряд |
| Inline IPA                 | `(lɪv)`                                | IPA в скобках                   |
| Inline пиньинь + тон       | `(he2)` (китайский, тон 1–5)          |                                 |
| Inline кантонез ёутпинг   | `(sung3)` (кантонез, тон 1–6)          | Требует `language_boost=Chinese,Yue`. |
| Interjection-теги (2.8)    | `(laughs)`, `(sighs)` и т. д. (см. §3) | Только `speech-2.8-*`           |

### 4.6. Ответ (sync, `stream=false`)

```jsonc
{
  "data": {
    "audio": "<hex>",          // hex-кодированный аудиобуфер выбранного формата
    "status": 2,                // 2 = synthesis completed
    "subtitle_file": "<url>"    // только если subtitle_enable=true
  },
  "extra_info": {
    "audio_length": 11124,      // миллисекунды
    "audio_sample_rate": 32000, // Гц
    "audio_size": 179926,       // байты
    "bitrate": 128000,
    "word_count": 163,
    "invisible_character_ratio": 0,
    "usage_characters": 163,    // биллинг — по этому счётчику
    "audio_format": "mp3",      // ← ВНИМАНИЕ: только mp3/pcm/flac
    "audio_channel": 1
  },
  "trace_id": "01b8bf9…",       // для саппорта
  "base_resp": {
    "status_code": 0,           // 0 = success
    "status_msg": "success"
  }
}
```

> `data` может быть `null` — реализация обязана это проверять (`payload.get("data") or {}` уже делает это у нас).

---

## 5. Языки (`language_boost`)

Документированный enum включает **40 значений** плюс `auto`:

Chinese, Chinese,Yue, English, Arabic, Russian, Spanish, French, Portuguese, German, Turkish, Dutch, Ukrainian, Vietnamese, Indonesian, Japanese, Italian, Korean, Thai, Polish, Romanian, Greek, Czech, Finnish, Hindi, Bulgarian, Danish, Hebrew, Malay, Persian, Slovak, Swedish, Croatian, Filipino, Hungarian, Norwegian, Slovenian, Catalan, Nynorsk, Tamil, Afrikaans, `auto`.

Замечания:

1. Полные английские имена (`"Russian"`, `"English"`, …) — это канонический формат MiniMax. Наш `_LANGUAGE_ALIASES` маппит короткие коды (`ru` → `Russian`) и пробрасывает уже-полные имена.
2. `auto` — рекомендуется, когда язык заранее неизвестен; иначе при пропуске поля (`null`) MiniMax не делает «усиления» конкретного языка.
3. **Сужение версий:** для `speech-01-*` / `speech-02-*` нет Persian/Filipino/Tamil. Для `speech-2.6+` всё 40 поддержано.

---

## 6. Коды ошибок и их маппинг

Полный список из `errorcode.md`. Используйте это для маппинга в наши `TTSError`-подклассы (`TTSAuthError`, `TTSRateLimitError`, `TTSBadRequestError`, `TTSError`, `TTSTimeoutError`).

| Код      | Категория в нашей иерархии | Сообщение                                                            | Действие                                          |
| -------- | -------------------------- | -------------------------------------------------------------------- | ------------------------------------------------- |
| `0`      | —                          | success                                                              | OK                                                |
| `1000`   | `TTSError`                 | unknown error                                                        | retry with backoff                                |
| `1001`   | `TTSTimeoutError`          | request timeout                                                      | retry with backoff                                |
| `1002`   | `TTSRateLimitError`        | rate limit                                                           | exponential backoff; учитывать RPM (см. §10)      |
| `1004`   | `TTSAuthError`             | not authorized / token not match group                               | проверить `MINIM…KEY` + `GroupId`; см. §2.1       |
| `1008`   | `TTSBadRequestError`*      | insufficient balance                                                 | уведомить пользователя; не retry                  |
| `1024`   | `TTSError`                 | internal error                                                       | retry                                            |
| `1026`   | `TTSBadRequestError`       | input new_sensitive                                                  | изменить ввод (контент-фильтр)                   |
| `1027`   | `TTSBadRequestError`       | output new_sensitive                                                 | изменить ввод                                    |
| `1033`   | `TTSError`                 | system error / mysql failed                                          | retry                                            |
| `1039`   | `TTSRateLimitError`        | token limit (TPM)                                                    | backoff                                          |
| `1041`   | `TTSError`                 | conn limit                                                           | связаться с поддержкой                            |
| `1042`   | `TTSBadRequestError`       | invisible character ratio limit (>10%)                               | проверить ввод на невидимые символы              |
| `1043`   | `TTSBadRequestError`       | The asr similarity check failed                                      | проверить `file_id`/`text_validation` (для clone) |
| `1044`   | `TTSBadRequestError`       | clone prompt similarity check failed                                 | проверить prompt audio                            |
| `2013`   | `TTSBadRequestError`       | invalid params / glyph definition format error                       | проверить payload                                 |
| `20132`  | `TTSBadRequestError`       | invalid samples or voice_id                                          | проверить `voice_id`                              |
| `2037`   | `TTSBadRequestError`       | voice duration too short / too long                                  | (только для clone)                                |
| `2039`   | `TTSBadRequestError`       | voice clone voice id duplicate                                       | выбрать другой `voice_id`                         |
| `2042`   | `TTSAuthError`*            | no access to this voice_id                                           | проверить права                                   |
| `2045`   | `TTSRateLimitError`        | rate growth limit                                                    | сгладить всплески                                 |
| `2048`   | `TTSBadRequestError`       | prompt audio too long                                                | обрезать prompt до 8с                            |
| `2049`   | `TTSAuthError`             | invalid API Key                                                      | перевыпустить ключ                                |
| `2056`   | `TTSRateLimitError`        | usage limit exceeded                                                 | ждать 5-часового окна                            |

`*` — наш текущий эвристический маппер по строке `status_msg` может неправильно классифицировать эти коды, потому что `status_msg` в этих случаях не содержит явного `auth`/`quota`/`rate`/`limit`/`invalid`/`param`/`voice` маркера. См. §11.1.

> **Маппинг-эвристика** в существующей реализации (`minimax_tts.py:443–448`):
> ```
> "auth" | "key" | "token"      → TTSAuthError
> "quota" | "rate" | "limit"    → TTSRateLimitError
> "invalid" | "param" | "voice" → TTSBadRequestError
> иначе                          → TTSError
> ```
> Работает в большинстве случаев, но для `1008`, `1024`, `1039`, `1041`, `1042`, `2037`, `2045`, `2048`, `2056` строковые маркеры в `status_msg` могут быть неочевидны. Желательно расширить маппинг до точного соответствия по `status_code` (см. таблицу выше) — это заметка для follow-up ADR.

---

## 7. Стриминг (SSE) и WebSocket

### 7.1. HTTP/SSE (тот же эндпоинт `POST /v1/t2a_v2`, `stream=true`)

- Тело запроса то же, что у sync.
- Ответ приходит **последовательностью JSON-объектов** (не обёрнутых в массив), либо по `text/event-stream`.
- `data.status`:
  - `1` — `synthesizing` (промежуточный чанк);
  - `2` — `synthesis completed` (финальный чанк, в нём же `extra_info`).
- `data.audio` — hex-кодированный фрагмент аудио.
- `base_resp.status_code != 0` в любом событии — ошибка. По контракту нашей реализации: если первый чанк уже ушёл — отдаём `TTSChunk(finish_reason="error")`; иначе `raise TTSError(...)`.
- `voice_modify` в стриме **ограничен только `mp3`** (см. §4.2).

### 7.2. Альтернативы стриму через тот же HTTP

- `output_format=url` (только sync) — вернёт ссылку, валидную 24 часа. Удобно для больших аудио, чтобы не тащить большие base16-данные.
- `exclude_aggregated_audio=true` — последний чанк не будет содержать полной склейки (экономия трафика, но усложняет аккумуляцию буфера на клиенте).

### 7.3. Наша реализация стрима

`MiniMaxTTSProvider.stream()` корректно:

1. использует `client.stream("POST", …)`;
2. парсит строки SSE как `data:…` JSON (или сырой JSON);
3. различает `[DONE]`-маркер (у MiniMax он есть, но в примерах из доки не виден — обработка остаётся как defensive);
4. доставляет каждый чанк как `TTSChunk`;
5. в конце выпускает пустой `TTSChunk(finish_reason="stop")`.

Известное ограничение (зафиксировано в ADR-0003 §2.4 как «sync primary + SSE optional»): MiniMax HTTP/SSE stream **не даёт фиксированного размера чанка** — это чанки по времени вывода модели. Для TTFA-критичных сценариев (диалоговое озвучивание) в MiniMax есть отдельный WebSocket-эндпоинт, описанный в `speech-t2a-websocket.md`. На дату исследования мы его не используем — это «future work».

### 7.4. WebSocket (`wss://api.minimax.io/v1/t2a_ws_v2`)

- Синхронный WebSocket, до 10 000 символов на запрос (см. `speech-t2a-websocket.md`).
- Документированное отличие: handshake быстрее, можно держать долгоживущее соединение и слать чанки текста, а не «один текст — один ответ».
- В нашей интеграции **не используется**; помечено в ADR-0003 §2.4 как «reserved».

---

## 8. Async T2A (long-form)

`/v1/t2a_async_v2` — для **длинных текстов** (книги, длинные README, подкасты).

| Поле          | Значение                                                       |
| ------------- | -------------------------------------------------------------- |
| Лимит текста  | до 1 000 000 символов на запрос                                |
| Вход          | либо `text` напрямую, либо `text_file_id` (после `POST /v1/files/upload` с `purpose=t2a_async_input`) |
| Output        | файл в сторадже MiniMax; доступен по `file_id` через `/v1/files/retrieve_content` |
| TTL ссылки    | 9 часов (32 400 секунд) — после этого файл пропадает           |
| Статусы       | `task_id` через `GET /v1/query/t2a_async_query_v2?task_id=…`   |

> Async — это единственный путь для >10 000 символов. Синхронные эндпоинты ограничены 10 000 символами. Для нашего `tts_node` это значит: если `len(text) > 10000`, нужно либо резать на чанки, либо поднимать Async-путь, которого сейчас нет.

---

## 9. Сверка с нашей реализацией (`MiniMaxTTSProvider`)

| Что проверял                                          | Документация MiniMax                                              | Наша реализация (`minimax_tts.py`)                                             | OK? |
| ------------------------------------------------------ | ----------------------------------------------------------------- | -------------------------------------------------------------------------------- | --- |
| Endpoint path                                         | `POST /v1/t2a_v2`                                                 | `f"{self._base_url}/v1/t2a_v2"`                                                  | ✅  |
| `Authorization: Bearer …`                              | обязателен                                                        | есть; `TTSAuthError` если `MINIM…KEY` не задан                                  | ✅  |
| `GroupId` query                                       | обязателен                                                        | есть; `TTSAuthError` если `MINIM…GROUP_ID` не задан                              | ✅  |
| `model` enum                                          | 8 значений (см. §3)                                               | default `"speech-02-hd"`; allow-list `_ALLOWED_EXTRA_KEYS` для forward-compat   | ✅ (заужено до speech-02/01) |
| `text` лимит                                          | < 10 000 символов; > 3 000 рекомендуется стрим                    | Не валидируем в провайдере — полагаемся на стороне tts_node/dialogue_node       | ⚠️ |
| `voice_setting.voice_id` обязателен                   | да                                                                | `voice_setting: {"voice_id": voice}` всегда                                     | ✅  |
| `voice_setting.speed` диапазон                        | `[0.5, 2]`                                                        | передаём как есть (валидация только на стороне API)                              | ✅  |
| `voice_setting.vol` диапазон                          | `(0, 10]`                                                         | валидируем `[0.0, 10.0]` включительно (включая 0)                                | ⚠️ спека говорит **«exclusiveMinimum: 0»** для `vol`; наша проверка `0.0 <= vol <= 10.0` пропустит vol=0, что API отклонит. Нужно поправить на `0.0 < vol <= 10.0` или ослабить валидацию до «soft». |
| `voice_setting.pitch` диапазон                        | `[-12, 12]`                                                       | передаём как есть                                                              | ✅  |
| `voice_setting.emotion` enum                          | 9 значений                                                        | передаём как есть                                                               | ✅  |
| `voice_setting.text_normalization`                     | bool                                                              | `payload["text_normalization"] = bool(settings.text_normalization)`             | ✅  |
| `audio_setting.sample_rate`                           | 8000/16000/22050/24000/32000/44100                                | default 32 000; без валидации                                                   | ✅  |
| `audio_setting.bitrate`                               | 32000/64000/128000/256000, только mp3                             | жёстко 128 000                                                                  | ✅  |
| `audio_setting.format`                                | mp3/pcm/flac/wav/pcmu_raw/pcmu_wav/opus                           | маппим `PCM/WAV/MP3 → сами; OGG → "mp3"`; `flac/pcmu_*/opus` не выбираем        | ⚠️ сознательное сужение (не баг) |
| `audio_setting.channel`                               | 1 или 2                                                           | всегда 1                                                                        | ⚠️ сознательное сужение; см. open-issue в ADR-0003 §5 |
| `audio_setting.force_cbr` (только streaming mp3)       | bool                                                              | не передаётся                                                                   | ⚠️ не критично; можно пробрасывать через `settings.extra` |
| `pronunciation_dict.tone`                             | список строк `orig/replacement`                                   | в `settings.extra` allow-list есть, но не пробрасываем напрямую                  | ⚠️ |
| `timbre_weights`                                      | массив до 4 голосов                                                | в allow-list `extra`                                                            | ⚠️ |
| `language_boost`                                      | enum (40 + auto)                                                  | Не используем; ставим `voice_setting.language` (старое поле, см. §11.2)         | ⚠️ функциональное расхождение |
| `voice_modify`                                        | pitch/intensity/timber `[-100,100]` + sound_effects               | Не используем                                                                   | ℹ️  |
| `subtitle_enable` / `subtitle_type`                   | bool/enum                                                         | Не используем                                                                   | ℹ️  |
| `output_format=url`                                   | вернёт URL, 24 ч                                                  | Никогда не отдаём `output_format` (жёстко `hex`)                                | ℹ️  |
| Sync response: `data.audio` (hex), `data.status=2`    | да                                                                | `payload.get("audio")`; статус не проверяем (полагаемся на `base_resp`)          | ✅  |
| Sync response: `extra_info` поля                      | 9 полей                                                           | `payload.get("audio_sample_rate")`; остальное игнорируем                         | ✅ (минимально достаточно) |
| Stream: chunk events, `data.status` 1/2               | см. §7.1                                                          | Корректно парсим; `finish_reason="stop"`/`"error"` по контракту                 | ✅  |
| Stream: поддержка `voice_modify`                      | только mp3                                                        | не релевантно (не используем)                                                   | ✅  |
| Ошибка HTTP 401/403 → `TTSAuthError`                  | документировано                                                   | есть (status 401/403)                                                           | ✅  |
| Ошибка HTTP 429 → `TTSRateLimitError`                 | документировано                                                   | есть                                                                            | ✅  |
| Ошибка `base_resp.status_code == 1004` → `TTSAuthError` | да                                                              | Эвристика по `status_msg` (см. §6)                                              | ⚠️ см. §6 |
| Ошибка `1008/1039/2056` → quota/usage лимит           | да                                                                | Эвристика по `status_msg`                                                       | ⚠️ |
| Retry рекомендации                                    | см. §6 (таймаут, internal, rate)                                   | ADR-0003 §2.6 фиксирует таблицу retry; **внутри провайдера ретраев нет**        | ⚠️ соответствует ADR |

### 9.1. Итог сверки

- **OK**: всё критичное для основного use-case (sync + SSE с образами чанков, auth/group, обязательные/часть опциональных полей) реализовано корректно.
- **Сознательные сужения (ADR-0003)**: `format`, `channel`, отсутствие `output_format=url`, отсутствие `language_boost`.
- **Зафиксированные несостыковки (открыть issues)**:
  1. `vol` принимает `0`, а спека требует `vol > 0`. Либо поправить валидацию, либо ослабить до `soft fail`.
  2. Эвристика маппинга `base_resp.status_code → TTSError` подкласс по строке `status_msg` ненадёжна для кодов `1008/1039/2056/2042/1004/2049`.
  3. `text` лимит (10 000) не валидируется на стороне провайдера.
  4. `pronunciation_dict`, `timbre_weights`, `voice_modify`, `subtitle_*`, `force_cbr` доступны через `settings.extra`, но **не протащены в наш `TTSSettings`** явно.
  5. Streaming `pcm`/`wav` (по спеке ограничен только `mp3`) — наш `_decode_audio` ничего об этом не знает; полагаемся на «прозрачный проброс байтов».

---

## 10. Rate limits и квоты

Из `rate-limits.md`:

| API              | Модели                                                          | RPM  | TPM             |
| ---------------- | --------------------------------------------------------------- | ---- | --------------- |
| T2A              | speech-2.8-turbo/hd, speech-2.6-turbo/hd, speech-02-turbo/hd   | 60   | не указан       |
| Voice Cloning    | —                                                               | 60   | —               |
| Voice Design     | —                                                               | 20   | —               |

Все T2A-модели делят общий пул RPM=60 (кроме `speech-01-*`, которые в таблице не указаны и, по косвенным источникам, делят тот же бюджет). На одном аккаунте можно держать **6 запросов за 10 секунд** устойчиво.

> ⚠️ В отличие от LLM (TPM), MiniMax для T2A считает **только RPM** — отдельного символьного/токенового rate-limit'а в публичной доке нет. Это не значит, что его нет: см. коды `1039` (`token limit`) и `2056` (`usage limit exceeded 5-hour window`) — это **TPM/usage-cap**, который срабатывает при превышении, но без публичной формулы.

### 10.1. Биллинг (по `audio-subscription.md`)

Подписки Audio:

| План      | $/мес | points/мес | Voice slots | RPM | T2A v2 / Large v2 | Все модели | Fast gen |
| --------- | ----- | ---------- | ----------- | --- | ----------------- | ---------- | -------- |
| Starter   | $5    | 100 000    | 10          | 10  | да                | да         | да       |
| Standard  | $30   | 300 000    | 100         | 50  | да                | да         | да       |
| Pro       | $99   | 1 100 000  | 250         | 200 | да                | да         | да       |
| Scale     | $249  | 3 300 000  | 500         | 500 | да                | да         | да       |
| Business  | $999  | 20 000 000 | 800         | 800 | да                | да         | да       |
| Customer  | —     | as needed  | —           | ∞   | да                | priority   |          |

Подписка включает ускоренную генерацию. Pay-as-you-go тарификация — `usage_characters` из `extra_info` (см. §4.6): 1 символ входного текста = 1 character billable.

---

## 11. Сценарии «что может пойти не так»

### 11.1. Классификация ошибок через `status_msg`-эвристику

Текущий код (строки `minimax_tts.py:443–448`) опирается на подстроки `auth/key/token`, `quota/rate/limit`, `invalid/param/voice`. Примеры риска:

| Код   | Типичный `status_msg`                             | Наш маппинг                  | Что должно быть  |
| ----- | ------------------------------------------------- | ---------------------------- | ---------------- |
| 1008  | `insufficient balance`                            | → `TTSError` (нет маркера!)  | `TTSBadRequestError` (или новый `TTSQuotaError`) |
| 1004  | `not authorized / token not match group`          | → `TTSAuthError` ✅          | OK               |
| 1039  | `token limit`                                     | → `TTSRateLimitError` ✅     | OK               |
| 2013  | `invalid params / glyph definition format error`  | → `TTSBadRequestError` ✅    | OK               |
| 2042  | `You don't have access to this voice_id`          | → `TTSError` (нет маркера «auth») | `TTSAuthError` |
| 2049  | `invalid API Key`                                 | → `TTSAuthError` ✅ (маркер «key») | OK              |
| 2056  | `usage limit exceeded`                            | → `TTSRateLimitError` ✅ (маркер «limit») | OK |

> Фикс: явный `if-elif` по `status_code`, а не подстроке — тривиальная правка в `_map_exception` (см. таблицу в §6).

### 11.2. `language_boost` vs `voice_setting.language`

Документация MiniMax (платформа) оперирует **только** верхнеуровневым `language_boost`. В нашей реализации язык кладётся в `voice_setting.language` — это **старое/внутреннее** поле, которое MiniMax всё ещё принимает (поэтому работает), но в актуальной спеке не документировано. Риск: в будущем MiniMax может это убрать. Миграция на `language_boost` — однострочная правка, отложенная (см. ADR-0003 §7 «open questions»).

### 11.3. Что НЕ покрыто реализацией (deliberate non-goals)

| Возможность                                  | Почему не покрыто                                            |
| -------------------------------------------- | ------------------------------------------------------------ |
| WebSocket (`/v1/t2a_ws_v2`)                  | Требует другого транспорта; в ADR-0003 помечен как «future work». |
| Async T2A (`/v1/t2a_async_v2`)               | Вне use-case нашего `tts_node`; тексты >10 000 сейчас не ожидаются. |
| `output_format=url`                          | Возвращали бы внешнюю ссылку с TTL 24 ч; неудобно для стриминга. |
| `subtitle_enable` + word-level таймкоды     | Не востребовано текущим `tts_node`.                          |
| `voice_modify` (deepen/brighten, effects)    | Не запрошено. У нас есть `emotion`.                          |
| `flac / pcmu_raw / pcmu_wav / opus` форматы  | Внутренний конвейер ROS2 AudioData ожидает PCM/WAV/MP3; transcoding — на стороне `tts_node`. |
| Кастомные голоса (Voice Clone, Voice Design) | Через `_ALLOWED_EXTRA_KEYS.voice_id` можно подставить свой `voice_id`, но без явного API для `POST /v1/voice_clone`. |

---

## 12. Минимальный пример (cURL, sync)

```bash
curl -sS -X POST https://api.minimax.io/v1/t2a_v2 \
  -H "Authorization: Bearer ${MINIMAX_API_KEY}" \
  -H "Content-Type: application/json" \
  -G --data-urlencode "GroupId=${MINIMAX_GROUP_ID}" \
  --data '{
    "model": "speech-2.8-hd",
    "text": "Привет, это пример синтеза MiniMax TTS.",
    "stream": false,
    "language_boost": "Russian",
    "voice_setting": {
      "voice_id": "Russian_male_v1",
      "speed": 1.0,
      "vol": 1.0,
      "pitch": 0,
      "emotion": "neutral"
    },
    "audio_setting": {
      "sample_rate": 32000,
      "bitrate": 128000,
      "format": "mp3",
      "channel": 1
    },
    "pronunciation_dict": {"tone": []}
  }'
```

Ответ: JSON с `data.audio` (hex), `extra_info.usage_characters` (для биллинга), `trace_id` (для саппорта), `base_resp`.

## 13. Минимальный пример (Python, sync) — без SDK

```python
import json, os
import httpx

API_KEY = os.environ["MINIMAX_API_KEY"]
GROUP_ID = os.environ["MINIMAX_GROUP_ID"]

payload = {
    "model": "speech-2.8-hd",
    "text": "Привет",
    "stream": False,
    "language_boost": "Russian",
    "voice_setting": {
        "voice_id": "Russian_male_v1",
        "speed": 1, "vol": 1, "pitch": 0, "emotion": "neutral"
    },
    "audio_setting": {
        "sample_rate": 32000, "bitrate": 128000, "format": "mp3", "channel": 1
    },
}

resp = httpx.post(
    "https://api.minimax.io/v1/t2a_v2",
    params={"GroupId": GROUP_ID},
    headers={"Authorization": f"Bearer {API_KEY}", "Content-Type": "application/json"},
    content=json.dumps(payload),
    timeout=30.0,
)
resp.raise_for_status()
data = resp.json()
if data["base_resp"]["status_code"] != 0:
    raise RuntimeError(data["base_resp"]["status_msg"])
mp3_bytes = bytes.fromhex(data["data"]["audio"])
open("hello.mp3", "wb").write(mp3_bytes)
print(f"usage_characters={data['extra_info']['usage_characters']}, "
      f"audio_length={data['extra_info']['audio_length']}ms")
```

> Официальных SDK от MiniMax для TTS именно T2A v2 нет — есть OpenAI/Anthropic-совместимые SDK для текстовых моделей, и есть `fal.ai`/`replicate` обёртки. Поэтому проект использует «голый» `httpx` — это явный и переносимый выбор, зафиксирован в ADR-0003.

---

## 14. Использованные источники

| Документ                                                | URL                                                                                  | Зачем                                          |
| ------------------------------------------------------- | ------------------------------------------------------------------------------------ | ---------------------------------------------- |
| T2A HTTP (OpenAPI 3.1 + спека полей)                    | https://platform.minimax.io/docs/api-reference/speech-t2a-http.md                    | §3, §4, §5, §6, §11                            |
| Error Code Reference                                    | https://platform.minimax.io/docs/api-reference/errorcode.md                          | §6, §11.1                                      |
| Rate Limits                                             | https://platform.minimax.io/docs/guides/rate-limits.md                               | §10                                            |
| Async Long TTS Guide                                    | https://platform.minimax.io/docs/guides/speech-t2a-async.md                           | §8                                             |
| Sync TTS WebSocket Guide                                | https://platform.minimax.io/docs/guides/speech-t2a-websocket.md                      | §7.4                                           |
| T2A WebSocket (endpoint reference)                      | https://platform.minimax.io/docs/api-reference/speech-t2a-websocket.md               | §7.4                                           |
| Voice Management / Get Voice                            | https://platform.minimax.io/docs/api-reference/voice-management-get.md               | §11.3 (для подтягивания списка системных голосов) |
| System Voice ID List                                    | https://platform.minimax.io/docs/faq/system-voice-id.md                              | §11.3                                          |
| Audio Subscription Pricing                              | https://platform.minimax.io/docs/guides/pricing-speech.md                            | §10.1                                          |
| OpenAPI spec (полная)                                   | https://platform.minimax.io/docs/zh/api-reference/openapi.json                       | §1, формальная схема                            |
| Документация-индекс                                     | https://platform.minimax.io/docs/llms.txt                                            | навигация                                       |

> Документация MiniMax активно развивается (см. раздел «API Release Notes»); перед крупными изменениями интеграции стоит перепроверять схему против `openapi.json`, а не только HTML-страницу.

---

## 15. Резюме для агента/разработчика

1. **Эндпоинт один**: `POST https://api.minimax.io/v1/t2a_v2`, `Authorization: Bearer`, `?GroupId=…`. Альтернативы — WS (для TTFA), Async (для >10 000 символов).
2. **Модели 8 штук** (`speech-2.8/2.6/02/01 -hd/-turbo`); interjection-теги только на `speech-2.8`; `whisper`/`fluent` только на `speech-2.6`.
3. **Голоса**: обязателен `voice_id`; системные + клоны + сгенерированные; микс до 4 через `timbre_weights`.
4. **Форматы аудио**: 7 (`mp3/pcm/flac/wav/pcmu_raw/pcmu_wav/opus`); биллинг по символам входа.
5. **Стрим**: HTTP/SSE — последовательность JSON-событий с `data.audio` (hex). WebSocket — отдельный путь, не реализован у нас.
6. **Ошибки**: 17 кодов в `base_resp.status_code`; маппинг на наши `TTSError`-подклассы сейчас эвристический по `status_msg` — желательно переводить на `status_code`.
7. **Rate-limit**: общий RPM=60 на аккаунт для всех T2A; подписки от $5/мес.
8. **Биллинг**: счётчик `extra_info.usage_characters`; URL-режим (TTL 24 ч) и Async (TTL 9 ч) — альтернативы для больших работ.
9. **Наш код**: основные use-cases (sync + SSE с правильным контрактом `TTSChunk`) — корректные; есть список сознательных сужений (формат, каналы, нет WS/Async); есть 5 открытых несостыковок для follow-up (vol=0, маппинг ошибок, text-лимит, проброс `pronunciation_dict`/`timbre_weights`/`voice_modify`, миграция на `language_boost`).
