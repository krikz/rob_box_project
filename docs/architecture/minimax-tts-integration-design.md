# MiniMax TTS — design-контракт интеграции и точки расширения

> **Инженерный справочник** к [ADR-0004](../adr/0004-minimax-tts-integration-design.md).
> Документ описывает **как** реализовать и расширять MiniMax-TTS
> интеграцию в `rob_box_llm` и `rob_box_voice`. Реализационная
> детализация конкретного `MiniMaxTTSProvider` (маппинг полей,
> цепочка форматов) живёт в [`minimax-tts-architecture.md`](./minimax-tts-architecture.md).

| Поле         | Значение                                                                |
|--------------|-------------------------------------------------------------------------|
| Статус       | Accepted (после одобрения ADR-0004)                                     |
| Дата         | 2026-07-21                                                             |
| Контекст     | Kanban task `t_b4cb8948`, PR #907, потомки `t_25b8e221`                |
| Родители     | [ADR-0002](../adr/0002-minimax-provider.md), [ADR-0003](../adr/0003-minimax-tts-architecture.md), ADR-0004 |
| AS-IS снапшот | [`../analysis/tts-current-interface.md`](../analysis/tts-current-interface.md) |
| API-реферат   | [`../research/minimax-tts-api.md`](../research/minimax-tts-api.md)      |

---

## 1. Назначение документа

Этот документ — **практический справочник** для:

1. **Реализатора MiniMax-TTS** (он уже есть; этот документ фиксирует
   его поведение как reference).
2. **Реализатора второго TTS-провайдера** (ElevenLabs, Azure Speech,
   Yandex v4) — как шаблон.
3. **Реализатора CLI/cron-инструмента**, который вызывает TTS без
   `rclpy`.
4. **Реализатора multi-channel mic stack** (когда появится
   AudioStamped-топик).

Он **НЕ дублирует**:

- Реализационные детали MiniMax (маппинг полей, hex-декодирование,
  редекс фильтры) — это в [`minimax-tts-architecture.md`](./minimax-tts-architecture.md).
- AS-IS снапшот текущего кода — [`tts-current-interface.md`](../analysis/tts-current-interface.md).
- Что говорит о себе MiniMax API — [`minimax-tts-api.md`](../research/minimax-tts-api.md).

---

## 2. Структура класса `MiniMaxTTSProvider`

```
+---------------------------------------------------+
|  TTSProvider (ABC, rob_box_llm/tts.py:121-177)    |
|  + synthesize(text, *, settings) -> TTSAudio      |
|  + stream(text, *, settings) -> AsyncIterator      |
|  + aclose() -> None                                |
+---------------------------------------------------+
                       ^
                       | implements (наследует напрямую, не Base)
                       |
+---------------------------------------------------+
|  MiniMaxTTSProvider                               |
|  (rob_box_llm/providers/minimax_tts.py:299-673)   |
|                                                   |
|  - DEFAULT_BASE_URL = "https://api.minimax.io"    |
|  - DEFAULT_VOICE = "male-qn-qingse"                |
|  - DEFAULT_MODEL = "speech-02-hd"                  |
|  - DEFAULT_TIMEOUT = 30.0                          |
|                                                   |
|  __init__(api_key=None, group_id=None,             |
|           base_url=, default_voice=,               |
|           default_model=, timeout=, client=None)   |
|                                                   |
|  - _headers() / _params() / _post(payload)         |
|  - _build_payload(text, settings, stream=)         |
|  - _decode_audio(data, fmt) -> (bytes, int)        |
|  + synthesize(text, *, settings) -> TTSAudio      |
|  + stream(text, *, settings) -> AsyncIterator      |
|  + aclose() -> None  (idempotent)                  |
+---------------------------------------------------+
                       |
                       v
+---------------------------------------------------+
|  Module-level helpers                              |
|  - _LANGUAGE_ALIASES: dict[str, str]                |
|  - _map_language(lang) -> str|None                  |
|  - _redact_sensitive_text(text, *, secrets)         |
|  - _map_exception(exc, *, provider, secrets)        |
|  - _build_payload(text, settings, *, stream,       |
|                   default_voice, default_model)     |
|  - _RESERVED_PAYLOAD_KEYS: frozenset[str] (6 keys)  |
|  - _ALLOWED_EXTRA_KEYS: frozenset[str] (9 keys)     |
|  - _RedactGroupIdFilter (logging.Filter)            |
|  - _HTTPX_GROUP_ID_FILTER (singleton instance)      |
+---------------------------------------------------+
```

**Ключевые характеристики:**

- **Stateless поверх `httpx.AsyncClient`.** `_owns_client` флаг
  различает "создал сам" vs "передали извне" (для тестов).
- **Ленивый attach `_RedactGroupIdFilter`** — защищает `httpx`
  logger от утечки `GroupId` query-параметра в access-логи.
- **Fail-fast валидация** — `vol ∉ [0.0, 10.0]` бросает
  `TTSBadRequestError` ДО HTTP-вызова; `extra` содержит reserved
  keys — тоже до вызова.
- **`_build_payload` всегда возвращает полный payload** — не дропает
  unset-поля, иначе MiniMax применит свои дефолты.

### 2.1 Что НЕ наследует `MiniMaxTTSProvider`

- **НЕ наследует `BaseTTSProvider`** (этот класс обозначен в ADR-0004
  §2.1, но не реализован — `MiniMaxTTSProvider` остаётся прямым
  наследником `TTSProvider`).
- **НЕ имеет собственного retry-loop** (per ADR-0003 §5.2, retry
  реализуется в `tts_node.py:1201-1258`).
- **НЕ имеет circuit breaker** (per ADR-0004 §2.7, обозначен,
  не реализован).

---

## 3. Схема конфигурации

### 3.1 Источники (high → low priority)

```
runtime ROS-параметры (set_parameters callback)
   ↓ если unset
ENV (только для api_key/group_id)
   ↓ если unset
YAML-файл /etc/rob_box/tts.yaml (если существует; НЕ для секретов)
   ↓ если unset
hardcoded defaults в коде
```

### 3.2 Полная таблица параметров MiniMax-провайдера

| ROS-параметр       | ENV                    | YAML-key         | Default                  | Где используется               |
|--------------------|------------------------|------------------|--------------------------|--------------------------------|
| `minimax_api_key`  | `MINIMAX_API_KEY`      | `api_key`        | (нет — обязателен)        | `MiniMaxTTSProvider(api_key=)` |
| `minimax_group_id` | `MINIMAX_GROUP_ID`     | `group_id`       | (нет — обязателен)        | `MiniMaxTTSProvider(group_id=)`|
| `minimax_voice`    | —                      | `voice`          | `male-qn-qingse`          | `TTSSettings.voice`            |
| `minimax_model`    | —                      | `model`          | `speech-02-hd`            | `TTSSettings.model`            |
| `minimax_language` | —                      | `language`       | `ru`                      | `TTSSettings.language`         |
| `minimax_speed`    | — (SSML override)      | `speed`          | `1.0`                     | `TTSSettings.speed`            |
| `minimax_sample_rate` | —                   | `sample_rate`    | `32000`                   | `TTSSettings.sample_rate`      |
| `minimax_timeout`  | —                      | `timeout`        | `30.0`                    | `MiniMaxTTSProvider(timeout=)` |
| `minimax_format`   | —                      | `format`         | `"pcm"`                   | `TTSSettings.format`           |
| `minimax_max_retries` | —                   | `max_retries`    | `2`                       | `_synthesize_minimax_with_retry` |
| `minimax_retry_backoff_ms` | —             | `retry_backoff_ms` | `500`                  | `_synthesize_minimax_with_retry` |
| `minimax_streaming` | —                     | `streaming`      | `False`                   | ветка в `_synthesize_and_play` |
| `provider`         | —                      | `provider`       | `"yandex"` (для tts_node) | гейт                            |
| `audio_output_sample_rate` | —              | `audio_output_sample_rate` | `16000`     | `_prepare_audio_for_topic`     |
| `audio_topic`      | —                      | `audio_topic`    | `/voice/audio/speech`     | publisher                       |

### 3.3 YAML-схема (пример)

```yaml
# /etc/rob_box/tts.yaml — опциональный override для multi-robot
provider: minimax              # НЕ секрет, можно тут
api_key: ""                    # Секрет — оставить пустым; берётся из ENV
group_id: ""                   # Секрет — оставить пустым; берётся из ENV
voice: "Russian_male_v1"
model: "speech-02-hd"
language: "ru"
speed: 1.0
sample_rate: 32000
timeout: 30.0
format: "pcm"
max_retries: 2
retry_backoff_ms: 500
streaming: false
audio_output_sample_rate: 16000
audio_topic: "/voice/audio/speech"
```

**Парсер** — `tts_node` опционально читает этот файл при старте
(`pathlib.Path("/etc/rob_box/tts.yaml").exists()`); если файла нет
— продолжает с ROS-параметрами. Парсер **НЕ валидирует секреты**
(они остаются ENV-only).

### 3.4 Почему НЕ pydantic-settings в базовом пайплайне

- `rob_box_llm` сейчас не зависит от `pydantic` (только от
  `httpx`). Добавление pydantic-settings = +1 hard dep, +
  потенциальные конфликты версий с `pydantic>=2`.
- ROS-параметры уже дают typed access (`get_parameter("name").value`
  с автоматическим type-coercion); YAML override —
  `dict[str, Any]` для обратной совместимости.
- pydantic-settings полезен для **standalone CLI** (где нет
  ROS-launch), но это отдельный use-case (ADR-0004 §2.4) — и
  реализуется в отдельном модуле, если потребуется.

---

## 4. Контракт выходных данных с аудиостэком

### 4.1 Текущий контракт (frozen)

```
Топик:     /voice/audio/speech (default)
Тип:       audio_common_msgs/AudioData
Поле:      uint8[] data
Содержит:  int16 little-endian PCM
Каналы:    1 (mono) — жёстко зашито в tts_node
Частота:   audio_output_sample_rate (default 16000 Hz)
Метаданные: out-of-band (ROS-параметры audio_output_sample_rate, audio_channels)
QoS:       KEEP_LAST depth=audio_qos_depth (default 10)
           reliability ∈ {best_effort, reliable} (default best_effort)
           durability = VOLATILE
```

**Implication:** каждый подписчик должен заранее знать
`sample_rate` (через ROS-параметр или out-of-band соглашение) и
`channels = 1`. Это **документированное ограничение**, не баг.

### 4.2 AudioStamped как opt-in forward-compat

```
Топик:     /voice/audio/speech_meta (default пустая строка = OFF)
Тип:       audio_common_msgs/AudioDataStamped  (новая msg, см. ADR-0004 §2.3.2)
Поля:      header (std_msgs/Header)
           sample_rate (int32)
           channels (int8)
           format (string) — "pcm_s16le" | "pcm_s32le" | "mp3" | "wav"
           samples (uint8[])
Частота:   встроена в сообщение
Каналы:    встроены в сообщение
```

**Активация:** только если `audio_meta_topic != ""` (ROS-параметр).

**Состояние на 2026-07-21:** msg-тип **не определён**. Перед активацией
нужно либо:

- Использовать существующий `audio_common_msgs/AudioDataStamped`
  (есть в `ros-humble-audio-common` — проверить), либо
- Определить свой в `rob_box_msgs/AudioDataStamped`.

**Текущая рекомендация:** **не активировать** без явного use-case
(подписчик, которому нужны метаданные).

### 4.3 Конвейер в tts_node (после MiniMax-ответа)

```
MiniMax response: hex PCM int16 LE @ 32 kHz mono
  │
  ├─ TTSAudio(samples: bytes, sample_rate: 32000, format=PCM)
  │
  ├─ _decode_minimax_audio: utils.audio_transcode.to_pcm_int16
  │   (только если format != PCM; для PCM — pass-through)
  │
  ├─ np.frombuffer(int16).astype(float32) / 32768
  │   → audio_np (float32 mono [-1, 1])
  │
  ├─ _prepare_audio_for_topic: resample → audio_output_sample_rate
  │
  ├─ chipmunk_mode on?
  │     ├─ True:  resample sr → sr/(2*pitch_shift) → resample → 16 kHz
  │     └─ False: resample sr → 16 kHz
  │
  ├─ apply volume_gain = 10^(volume_db / 20)
  │
  ├─ mono → stereo: np.column_stack([x, x])  # для ReSpeaker
  │
  ├─ AudioPlaybackManager.play_audio(blocking=True, timeout=5.0)
  │   (локальное воспроизведение)
  │
  ├─ int16 = (audio_np * 32767).astype(np.int16).tobytes()
  │   → AudioData().data = list(bytes)
  │
  └─ publish(/voice/audio/speech, qos=audio_qos_*)        # ROS publish
```

> **Для реализатора следующего провайдера:** контракт `TTSAudio` —
> всё, что вам нужно. Всё, что после — задача `tts_node`, не
> провайдера.

---

## 5. Стратегия стриминга

### 5.1 Три уровня

| Уровень | Endpoint                             | Yield                       | Latency-выгода                | Когда использовать |
|---------|--------------------------------------|-----------------------------|--------------------------------|---------------------|
| Sync    | `POST /v1/t2a_v2` (`stream=false`)   | 1 `TTSAudio` после полного  | — (baseline 2-3 s TTFA)        | Default             |
| SSE     | `POST /v1/t2a_v2` (`stream=true`)    | 1+ `TTSChunk`, terminal stop | 0% (MiniMax SSE буферизует)    | Тесты, forward-compat |
| WebSocket | `wss://api.minimax.io/v1/t2a_ws_v2` | chunks каждые ~50-200 ms    | ~50-200 ms TTFA win            | M5/M6               |

### 5.2 Где живёт стриминг

```
tts_node._synthesize_and_play()         (tts_node.py:712-983)
   ├── provider == "minimax"
   │     ├── minimax_streaming == False
   │     │     └── _synthesize_minimax(...)           (tts_node.py:1329-1352)
   │     │           └── _synthesize_minimax_with_retry(...)  (tts_node.py:1201-1258)
   │     │                 └── _synthesize_minimax_async(...)  (tts_node.py:1118-1199)
   │     │                       └── MiniMaxTTSProvider.synthesize(...)
   │     │                              → TTSAudio
   │     │                       └── _decode_minimax_audio() → audio_np float32
   │     │
   │     └── minimax_streaming == True
   │           └── _synthesize_minimax_streaming_publish(...)  (tts_node.py:1260-1327)
   │                 └── _stream_minimax_chunks()             (tts_node.py:1354-1403)
   │                       └── MiniMaxTTSProvider.stream(...)
   │                              → AsyncIterator[TTSChunk]
   │                       └── публикует каждый chunk как AudioData msg
   │                              + собирает в общий буфер для playback
```

### 5.3 Контракт `TTSChunk` (frozen в ABC)

```python
@dataclass(frozen=True)
class TTSChunk:
    samples: bytes = b""
    sample_rate: int = 0
    format: TTSFormat = TTSFormat.PCM
    finish_reason: Optional[str] = None  # "stop" | "error"
```

**Правила:**

1. Провайдер ОБЯЗАН эмитить минимум один терминальный chunk
   (`finish_reason != None`) — `tts.py:103-113`.
2. Pre-yield ошибки → `raise TTSError`.
3. Post-yield ошибки → terminal `TTSChunk(finish_reason="error")`.
4. Успех → terminal `TTSChunk(finish_reason="stop")`.

**Текущая MiniMax-реализация:** эмитит ровно один terminal chunk
(полный буферизованный ответ), даже в `stream()` режиме. Это
**намеренно** для v1 — true chunk-per-frame ждёт WebSocket
(ADR-0003 §2.4).

---

## 6. Retry и обработка ошибок

### 6.1 Классификация ошибок

| Источник                                  | Тип `TTSError`              | Retry? | Кол-во попыток      |
|-------------------------------------------|-----------------------------|--------|---------------------|
| `httpx.TimeoutException`                   | `TTSTimeoutError`           | Да     | ≤ `max_retries`     |
| `httpx.ConnectError` (DNS/TCP/TLS)        | `TTSTimeoutError`           | Да     | ≤ `max_retries`     |
| HTTP 401/403                               | `TTSAuthError`              | **Нет**| —                   |
| HTTP 429                                   | `TTSRateLimitError`         | Да     | ≤ 1                 |
| HTTP 4xx (кроме 401/403/429)               | `TTSBadRequestError`        | **Нет**| —                   |
| HTTP 5xx                                   | `TTSError`                  | Да     | ≤ `max_retries`     |
| `base_resp.status_code != 0` (бизнес)      | по эвристике `status_msg`   | **Нет**| —                   |
| `JSONDecodeError`                          | `TTSError`                  | **Нет**| —                   |
| `vol ∉ [0.0, 10.0]` (локальная)            | `TTSBadRequestError`        | **Нет**| — (fail-fast)       |
| `extra` содержит reserved keys             | `TTSBadRequestError`        | **Нет**| — (fail-fast)       |

### 6.2 Retry-алгоритм (универсальный)

```python
async def with_retry(callable_, policy: RetryPolicy) -> T:
    attempt = 0
    while True:
        try:
            return await callable_()
        except (TTSAuthError, TTSBadRequestError):
            raise                       # NO RETRY
        except Exception as exc:
            if not policy.should_retry(exc, attempt):
                raise
            delay = policy.backoff_seconds(attempt)
            _log.warning("retry %d/%d after %.2fs: %s",
                         attempt + 1, policy.max_retries, delay, exc)
            await asyncio.sleep(delay)
            attempt += 1
```

**Backoff:** `delay_n = (backoff_ms / 1000.0) * (2 ** n)`
(экспоненциальный, default 0.5s, 1.0s, 2.0s для max_retries=2).

### 6.3 Реализация в tts_node

В файле `tts_node.py:1201-1258`:

```python
async def _synthesize_minimax_with_retry(self, text, ssml_attributes):
    configured_retries = min(max(0, int(self.minimax_max_retries)), 3)
    max_attempts = configured_retries + 1
    backoff_ms = self.minimax_retry_backoff_ms
    last_exc = None

    for attempt in range(max_attempts):
        try:
            return await self._synthesize_minimax_async(text, ssml_attributes)
        except MiniMaxTTSAuthError:
            raise
        except MiniMaxTTSBadRequestError:
            raise
        except Exception as exc:
            retry_budget = 1 if isinstance(exc, MiniMaxTTSRateLimitError) \
                           else configured_retries
            last_exc = exc
            if attempt >= retry_budget:
                raise
            delay = (backoff_ms / 1000.0) * (2 ** attempt)
            await asyncio.sleep(delay)
    raise last_exc
```

> **Совместимо** с ADR-0004 §2.6 `RetryPolicy.should_retry()`:
> при переносе в value-object поведение не меняется.

### 6.4 Circuit breaker (отложен)

Текущее состояние: **не реализован**. ADR-0004 §2.7 обозначил
форму (`CircuitBreaker.disabled()` как default). Триггер для
реализации: >100 TTS-вызовов/час/робот или multi-robot fleet.

---

## 7. Точки расширения

### 7.1 Что такое "точка расширения"

Точка расширения = место в коде, где **можно добавить нового
провайдера / поведение без правки существующих файлов**.

| Точка                          | Как расширять                                       | Сложность |
|--------------------------------|------------------------------------------------------|-----------|
| Новый TTS-провайдер            | Реализовать `TTSProvider` + зарегистрировать в `TTSProviderRegistry` (после миграции) | Низкая |
| Новый формат аудио             | Расширить `TTSFormat` + добавить decoder в `utils/audio_transcode.py` | Низкая |
| Новый retry-класс              | Создать подкласс или инстанс `RetryPolicy`            | Тривиальная |
| Новый error-класс              | Подкласс `TTSError`, добавить в иерархию              | Тривиальная |
| Новый lifecycle hook           | Переопределить `aclose()` (idempotent)               | Тривиальная |
| Новый transport (WebSocket)    | Создать `MiniMaxStreamingTTSProvider(MiniMaxTTSProvider)` с persistent connection | Средняя |

### 7.2 Как добавить второго TTS-провайдера

Пошаговый чеклист:

1. **Реализовать `TTSProvider`** в
   `src/rob_box_llm/rob_box_llm/providers/<name>_tts.py`:
   - `synthesize(text, *, settings) -> TTSAudio` — async
   - `stream(text, *, settings) -> AsyncIterator[TTSChunk]` — async
   - `aclose() -> None` — idempotent

2. **Добавить value-object**, если нужны специфичные настройки:
   - Наследовать от `TTSSettings` или создать отдельный
     `ProviderSettings` (только для forward-compat).
   - Использовать `settings.extra` для простых случаев.

3. **Добавить error-mapping** в стиле `_map_exception`:
   - 401/403 → `TTSAuthError`
   - 429 → `TTSRateLimitError`
   - 4xx → `TTSBadRequestError`
   - 5xx, timeout, network → `TTSError` / `TTSTimeoutError`

4. **Зарегистрировать** в `rob_box_llm/__init__.py`:
   ```python
   from rob_box_llm.providers.<name>_tts import <Name>TTSProvider
   __all__ += ["<Name>TTSProvider"]
   ```

5. **Добавить ROS-параметры** в `tts_node.py:__init__`:
   - `<name>_api_key` (или другой секрет) → ENV-fallback
   - `<name>_voice`, `<name>_model`, etc.

6. **Добавить ветку** в `tts_node._synthesize_and_play` (до
   миграции на registry). После миграции — registry берёт на себя.

7. **Покрыть тестами**:
   - Conformance suite (`test_tts_conformance.py`) — обязателен.
   - Provider-level тесты с `httpx.MockTransport` или эквивалентом.

### 7.3 Что НЕ делать при расширении

- **НЕ** трогать `TTSProvider` ABC — frozen.
- **НЕ** менять `/voice/audio/speech` топик.
- **НЕ** добавлять автоматический fallback между провайдерами.
- **НЕ** дублировать retry-loop в провайдере.
- **НЕ** игнорировать `_ALLOWED_EXTRA_KEYS` — это security boundary.

---

## 8. Зависимости и лицензии

| Зависимость | Где используется | Лицензия |
|-------------|------------------|----------|
| `httpx>=0.27` | `MiniMaxTTSProvider` HTTP клиент | BSD-3-Clause |
| `pydub` (lazy) | `utils.audio_transcode` для MP3/OGG | MIT (lazy — pакет опционален) |
| `ffmpeg` (system) | `utils.audio_transcode` fallback для MP3/OGG | LGPL/GPL (system binary) |
| `python3-httpx` (apt) | ROS `package.xml` `exec_depend` | Debian |

**НЕ добавлять в `rob_box_llm`:**

- `pydantic` / `pydantic-settings` — отдельный CLI-модуль если потребуется.
- `websockets` — для будущего WS-провайдера (M5/M6).
- `respx` (только тесты).

---

## 9. Acceptance criteria

Этот документ считается зафиксированным, когда:

1. ✅ ADR-0004 — accepted.
2. ⏳ Реализован `RetryPolicy` value-object (отдельная задача,
   не блокирует принятие ADR).
3. ⏳ Реализован `TTSProviderRegistry` (отдельная задача).
4. ⏳ Существующий `MiniMaxTTSProvider` остаётся без изменений
   (регрессия ≥ 95% pass rate).
5. ⏳ Conformance suite (`test_tts_conformance.py`) — pass.

---

## Приложение A. Sequence-диаграмма (расширенная)

См. [`../diagrams/minimax-tts-integration-sequence.mmd`](../diagrams/minimax-tts-integration-sequence.mmd).

Включает:
- Sync happy-path (synthesize → TTSAudio → audio_np → AudioData).
- Retry-loop (timeout / 429 → повтор с backoff).
- Error path (TTSAuthError / TTSBadRequestError → raise, no retry).
- Streaming hook (`minimax_streaming=true` → `TTSProvider.stream()`).
- Registry resolution (`provider="<name>"` → `TTSProviderRegistry.create()`).

---

## Приложение B. Class-диаграмма

См. [`../diagrams/minimax-tts-integration-class.mmd`](../diagrams/minimax-tts-integration-class.mmd).

Включает:
- `TTSProvider` ABC + value-объекты.
- `MiniMaxTTSProvider` (текущая реализация).
- `BaseTTSProvider` (обозначен, не реализован).
- `TTSProviderRegistry` (обозначен, не реализован).
- `RetryPolicy` (обозначен, не реализован).
- `CircuitBreaker` (обозначен, не реализован).
- Точки расширения (interfaces + factory methods).