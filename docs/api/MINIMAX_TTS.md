# `MiniMaxTTSProvider` — API reference

> **Пакет:** `rob_box_llm` ≥ 0.2.1
> **Модуль:** `rob_box_llm.providers.minimax_tts`
> **Транспорт:** HTTP `POST https://api.minimax.io/v1/t2a_v2` (MiniMax T2A v2)
> **Альтернативы по base_url:** `https://api-uw.minimax.io` (Reduced TTFA), провайдерская нода для Китая
> **Стиль:** Markdown с блоками OpenAPI-стиля. Для архитектурных решений см. [ADR-0003](../adr/0003-minimax-tts-architecture.md); для пользовательского сценария — [Getting Started](../guides/MINIMAX_TTS_GETTING_STARTED.md).

---

## Содержание

1. [Класс `MiniMaxTTSProvider`](#1-класс-minimaxttsprovider)
2. [Конструктор](#2-конструктор)
3. [Метод `synthesize()`](#3-метод-synthesize)
4. [Метод `stream()`](#4-метод-stream)
5. [Метод `aclose()`](#5-метод-aclose)
6. [Метод `capabilities()`](#6-метод-capabilities)
7. [Метод `list_voices()`](#7-метод-list_voices)
8. [Метод `healthcheck()`](#8-метод-healthcheck)
9. [Value-objects (`TTSSettings`, `TTSAudio`, `TTSChunk`, `TTSFormat`)](#9-value-objects)
10. [Исключения](#10-исключения)
11. [Bytes API и переменные окружения](#11-bytes-api-и-переменные-окружения)
12. [Ограничения и не-поддерживаемое](#12-ограничения)

---

## 1. Класс `MiniMaxTTSProvider`

```python
from rob_box_llm import MiniMaxTTSProvider
from rob_box_llm.tts_provider_base import (
    BaseTTSProvider, TTSCapabilities, TTSHealth, TTSVoice, ProviderBuilder,
)
from rob_box_llm.tts_provider_registry import (
    TTSProviderRegistry, TTSProviderFactory, register_builtin_tts_providers,
)
```

| Свойство | Значение |
|---|---|
| Базовый класс | `rob_box_llm.tts_provider_base.BaseTTSProvider` |
| Также наследует | `rob_box_llm.tts.TTSProvider` (backward-compat) |
| `name` (канонический id) | `"minimax"` |
| Регистрируется в реестре | `register_builtin_tts_providers()` → `TTSProviderRegistry.register("minimax", builder)` |
| Потокобезопасность | Конструктор — да; метод `synthesize`/`stream`/`healthcheck` — корректны при параллельных вызовах; общий `httpx.AsyncClient` шарится между ними |
| Async-only | Все методы `synthesize`, `stream`, `list_voices`, `healthcheck` — `async` |

---

## 2. Конструктор

```python
MiniMaxTTSProvider(
    *,
    api_key: Optional[str] = None,
    group_id: Optional[str] = None,
    base_url: Optional[str] = None,
    default_voice: str = "male-qn-qingse",
    default_model: str = "speech-02-hd",
    timeout: float = 30.0,
    client: Optional[httpx.AsyncClient] = None,
    max_attempts: int = 3,
    retry_base_delay: float = 0.5,
    retry_jitter: float = 0.25,
    max_concurrency: int = 1,
    cache_size: int = 128,
) -> None
```

| Параметр | Тип | Default | Описание |
|---|---|---|---|
| `api_key` | `Optional[str]` | ENV `MINIMAX_API_KEY` | MiniMax API key. Без префикса `Bearer`. Префикс добавляет провайдер. Если не задан ни параметром, ни ENV — **первый вызов** `synthesize`/`stream` поднимет `TTSAuthError`. |
| `group_id` | `Optional[str]` | ENV `MINIMAX_GROUP_ID` | MiniMax account/group id. Передаётся как query-параметр `GroupId=…` в каждый запрос. Обязателен. |
| `base_url` | `Optional[str]` | ENV `MINIMAX_TTS_BASE_URL` → `"https://api.minimax.io"` | Корень API. Аргумент конструктора имеет приоритет над ENV. Для тестов можно указать локальный mock endpoint. |
| `default_voice` | `str` | `"male-qn-qingse"` | Voice id, который подставляется в payload, если `TTSSettings.voice is None`. |
| `default_model` | `str` | `"speech-02-hd"` | Model, который подставляется, если `TTSSettings.model is None`. Допустимые: `speech-02-hd`, `speech-02-turbo`, `speech-01-hd`, `speech-01-turbo`. |
| `timeout` | `float` | `30.0` | httpx timeout в секундах на каждый HTTP-запрос (sync и streaming). |
| `client` | `Optional[httpx.AsyncClient]` | `None` | Инжектированный клиент (для тестов с `httpx.MockTransport`). Если задан — провайдер **не закрывает** его в `aclose()` (ответственность на вызывающем). |
| `max_attempts` | `int` | `3` | Максимум попыток в `synthesize_bytes()` для retryable ошибок. |
| `retry_base_delay` | `float` | `0.5` | Начальная задержка exponential backoff, секунды. |
| `retry_jitter` | `float` | `0.25` | Максимальный случайный jitter, секунды. |
| `max_concurrency` | `int` | `1` | Локальный лимит одновременных запросов `synthesize_bytes()`. |
| `cache_size` | `int` | `128` | Максимум результатов во встроенном LRU-кэше; `0` отключает completed-result cache. |

**Возвращает:** `None`.

**Исключения:** не бросает; валидация credentials отложена до первого `synthesize`/`stream`.

**Побочные эффекты:**

- Прикрепляет к `logging.getLogger("httpx")` фильтр `_RedactGroupIdFilter`, который редактирует query-параметр `GroupId` в URL httpx access-логов (`<redacted>`), даже если сторонний код поднимет уровень логгера до INFO/DEBUG.
- Если `client is None`, создаёт собственный `httpx.AsyncClient(timeout=self._timeout)` через `_http_client_factory()` и **закроет его** в `aclose()`.

**Минимальный пример:**

```python
import os
from rob_box_llm import MiniMaxTTSProvider

provider = MiniMaxTTSProvider(
    api_key=os.environ["MINIMAX_API_KEY"],
    group_id=os.environ["MINIMAX_GROUP_ID"],
)
```

---

## 3. Метод `synthesize()`

```python
async def synthesize(
    text: str,
    *,
    settings: Optional[TTSSettings] = None,
) -> TTSAudio
```

Синхронный (one-shot) синтез: один POST → один JSON с `data.audio` (hex-encoded PCM/MP3/WAV) → `TTSAudio`.

| Параметр | Тип | Default | Описание |
|---|---|---|---|
| `text` | `str` | — обязательный | Текст для синтеза. Документированный лимит MiniMax: ≤ 10 000 символов; > 3 000 рекомендуется стриминг (см. `stream()`). Провайдер **не** валидирует длину; ожидается валидация на стороне вызывающего (dialogue_node / tts_node). Пустая строка или только пробелы → `TTSBadRequestError("text is empty")`. |
| `settings` | `Optional[TTSSettings]` | `TTSSettings()` | Per-call параметры. См. §9. |

**Возвращает:** [`TTSAudio`](#9-value-objects).

**Исключения:**

| Исключение | Условие |
|---|---|
| `TTSBadRequestError` | `text` пуст после `.strip()`; `settings.volume` вне `[0.0, 10.0]` (fail-fast до HTTP); `settings.extra` пересекается с reserved keys (`model`, `text`, `stream`, `voice_setting`, `audio_setting`, `text_normalization`) |
| `TTSAuthError` | `MINIMAX_API_KEY` или `MINIMAX_GROUP_ID` пустые; HTTP 401/403; `base_resp.status_msg` содержит `auth`/`key`/`token` |
| `TTSRateLimitError` | HTTP 429; `base_resp.status_msg` содержит `quota`/`rate`/`limit` |
| `TTSBadRequestError` (через `_map_exception`) | HTTP 400-499 (кроме 401/403/429) |
| `TTSTimeoutError` | `httpx.TimeoutException` или любой другой `httpx.HTTPError` |
| `TTSError` | HTTP 5xx; `resp.json()` не парсится; `data.audio` отсутствует; `bytes.fromhex(data.audio)` упал; `base_resp.status_code != 0` с неопознанной категорией |

**Гарантии:**

- Провайдер НЕ делает ретраев внутри себя (retry — на стороне `tts_node` через параметры `minimax_max_retries`).
- Учётные данные (`api_key`, `group_id`) **редактируются** (`<redacted>`) в любых сообщениях исключений и в логах, если туда попал URL с `GroupId=`.

**Пример:**

```python
from rob_box_llm import MiniMaxTTSProvider, TTSSettings, TTSFormat

provider = MiniMaxTTSProvider()
try:
    audio = await provider.synthesize(
        "Привет, мир!",
        settings=TTSSettings(
            voice="male-qn-qingse",
            language="ru",
            speed=1.0,
            sample_rate=32000,
            format=TTSFormat.PCM,
        ),
    )
    print(f"{len(audio.samples)} bytes @ {audio.sample_rate} Hz, "
          f"{audio.duration_s:.2f}s, {audio.format.value}")
finally:
    await provider.aclose()
```

---

## 4. Метод `stream()`

```python
async def stream(
    text: str,
    *,
    settings: Optional[TTSSettings] = None,
) -> AsyncIterator[TTSChunk]
```

SSE-стриминг: `stream=true` в payload, чтение `aiter_lines()`, эмиссия по `TTSChunk` для каждого аудио-события + терминальный `TTSChunk(finish_reason="stop")` или `"error"`.

| Параметр | Тип | Default | Описание |
|---|---|---|---|
| `text` | `str` | — обязательный | То же, что и в `synthesize`. |
| `settings` | `Optional[TTSSettings]` | `TTSSettings()` | То же, что и в `synthesize`. `settings.format` влияет на маркировку `TTSChunk.format`. |

**Возвращает:** `AsyncIterator[TTSChunk]`.

**Контракт эмиссии:**

1. **До первого `yield`** — при любой ошибке бросает исключение (типы — см. `synthesize`).
2. **После первого `yield`** — mid-stream ошибки конвертируются в финальный `TTSChunk(finish_reason="error")` и стрим завершается.
3. **При успехе** — после последнего аудио-чанка эмитируется пустой `TTSChunk(samples=b"", finish_reason="stop")`.
4. **Если ни одного аудио-чанка не пришло** — `TTSError("minimax stream returned no audio chunks")`.
5. **`[DONE]`-маркер** в потоке MiniMax распознаётся как `break` (дефенсив — в публичной документации MiniMax он не документирован, но реальные ответы его содержат).

**Известное ограничение:** на 2026-07-22 MiniMax HTTP/SSE не даёт фиксированного размера чанка — это чанки по времени вывода модели. Для TTFA-критичных сценариев в MiniMax есть отдельный WebSocket-эндпоинт `/v1/t2a_ws_v2`, который **не реализован** в этом провайдере (out of scope, см. ADR-0003 §2.4 «future work»).

**Пример:**

```python
async for chunk in provider.stream("Длинный текст...", settings=TTSSettings(model="speech-02-turbo")):
    if chunk.finish_reason == "stop":
        print("stream finished")
    elif chunk.finish_reason == "error":
        print("stream error mid-flight")
        break
    else:
        # chunk.samples — bytes, chunk.sample_rate — Hz, chunk.format — TTSFormat
        play(chunk.samples, chunk.sample_rate)
```

---

## 5. Метод `aclose()`

```python
async def aclose() -> None
```

Освобождение ресурсов. **Идемпотентен** — безопасно вызывать из `finally` без отдельного гарда.

**Поведение:**

- Если `client` был создан провайдером (`_owns_client=True`) и ещё не закрыт — `await self._client.aclose()`.
- Если `client` был инжектирован через параметр конструктора — провайдер **не** закрывает его (ответственность на вызывающем).
- Повторные вызовы — no-op.

**Возвращает:** `None`. **Не бросает.**

**Когда вызывать:**

- В `finally` блоке `async with` / try-finally в скриптах и тестах.
- В `destroy_node()` ROS2-ноды.
- В shutdown-хук fastapi/litestar/aiogram-приложения, если провайдер живёт дольше одного запроса.

**Когда НЕ вызывать:**

- Если провайдер живёт ровно один запрос — `httpx.AsyncClient` закроется при сборке мусора; явный `aclose` всё равно рекомендуется.

---

## 6. Метод `capabilities()`

```python
def capabilities(self) -> TTSCapabilities  # sync, не async
```

Декларативное описание того, что провайдер умеет. Возвращает `TTSCapabilities` (`frozen=True` dataclass).

| Поле | Значение | Почему |
|---|---|---|
| `streaming` | `True` | SSE через `stream=true` |
| `voice_cloning` | `True` | Через `settings.extra["timbre_weights"]` (см. [§12](#12-ограничения)) |
| `ssml` | `False` | MiniMax не парсит SSML |
| `pronunciation_dict` | `False` | Поле `pronunciation_dict` существует, но не вынесено в `TTSSettings` — оставлено как `extra` |
| `audio_format_pcm` | `True` | Документировано |
| `audio_format_mp3` | `True` | Документировано |
| `audio_format_ogg` | `False` | MiniMax не поддерживает OGG; `TTSFormat.OGG` в `settings.format` прозрачно фоллбэчится на MP3 (см. §12) |
| `custom_endpoint` | `False` | Это не on-prem провайдер |

**Используется:** registry / composition root / fallback selector.

---

## 7. Метод `list_voices()`

```python
async def list_voices(self) -> list[TTSVoice]
```

Возвращает встроенный каталог из 6 голосов (см. `_BUILTIN_VOICES` в `minimax_tts.py`). Не делает HTTP-вызова. MiniMax на 2026-07-22 **не предоставляет** публичный `/v1/voices` эндпоинт в T2A v2.

Возвращаемые `TTSVoice`:

| `id` | `language` | `gender` | `supports_cloning` |
|---|---|---|---|
| `male-qn-qingse` | `zh` | `male` | `False` |
| `female-shaonv` | `zh` | `female` | `False` |
| `Calm_Woman` | `en` | `female` | `False` |
| `English_PassionateWarrior` | `en` | `male` | `False` |
| `Russian_DeepVoice` | `ru` | `male` | `False` |
| `Russian_CalmWoman` | `ru` | `female` | `False` |

**Не бросает.** Полный актуальный каталог — в [документации MiniMax](https://platform.minimaxi.com/docs/api-reference/voice-cloning/voice-cloning-1).

---

## 8. Метод `healthcheck()`

```python
async def healthcheck(self) -> TTSHealth  # frozen dataclass
```

Лёгкая pre-flight проверка. **Не делает HTTP-запроса** — только валидирует, что credentials сконфигурированы.

| Поле `TTSHealth` | Когда какое значение |
|---|---|
| `ok=True`, `latency_ms=0.0` | `MINIMAX_API_KEY` и `MINIMAX_GROUP_ID` оба непустые |
| `ok=False`, `reason="MINIMAX_API_KEY missing"` | `api_key` пуст |
| `ok=False`, `reason="MINIMAX_GROUP_ID missing"` | `group_id` пуст |

**Пример:**

```python
health = await provider.healthcheck()
if not health.ok:
    print(f"TTS provider {health.provider} unhealthy: {health.reason}")
```

Для реальной сетевой проверки используйте отдельный `ping_minimax.py` (см. `docs/guides/MINIMAX_TTS.md` §7) — он не входит в публичный API.

---

## 9. Value-objects

Все из `rob_box_llm.tts` (`frozen=True`).

### `TTSSettings`

| Поле | Тип | Default | Маппинг в MiniMax T2A v2 | Допустимые значения |
|---|---|---|---|---|
| `model` | `Optional[str]` | `None` → `default_voice_model` | `model` (top-level) | `"speech-02-hd"`, `"speech-02-turbo"`, `"speech-01-hd"`, `"speech-01-turbo"` (плюс новые см. research §3) |
| `voice` | `Optional[str]` | `None` → `default_voice` | `voice_setting.voice_id` | Любой `voice_id` из каталога MiniMax |
| `language` | `Optional[str]` | `None` | `voice_setting.language` (после алиаса) | Короткий BCP-47 (`ru`, `en`, `zh`, …) или полное имя (`Russian`, `English`, …) |
| `speed` | `Optional[float]` | `None` | `voice_setting.speed` | `[0.5, 2.0]` (валидация на стороне MiniMax) |
| `volume` | `Optional[float]` | `None` | `voice_setting.vol` | `[0.0, 10.0]` (см. ⚠ ниже) |
| `pitch` | `Optional[int]` | `None` | `voice_setting.pitch` | `[-12, 12]` semitones |
| `emotion` | `Optional[str]` | `None` | `voice_setting.emotion` | `happy`, `sad`, `angry`, `fearful`, `disgusted`, `surprised`, `calm`, `fluent`, `whisper` |
| `sample_rate` | `Optional[int]` | `None` → `32000` | `audio_setting.sample_rate` | `8000`, `16000`, `22050`, `24000`, `32000`, `44100` |
| `format` | `TTSFormat` | `TTSFormat.PCM` | `audio_setting.format` | `PCM`, `WAV`, `MP3`, `OGG` (OGG → MP3 fallback) |
| `text_normalization` | `Optional[bool]` | `None` | `text_normalization` (top-level) | bool |
| `extra` | `Mapping[str, Any]` | `{}` | top-level keys, прошедшие allow-list | см. ниже |

**Allow-list для `settings.extra`** (`_ALLOWED_EXTRA_KEYS`):

```
voice_id, speed, pitch, vol, emotion,
subtitle_timestamp, pronunciation_dict, timbre_weights, audio_output_format
```

**Reserved keys** (если присутствуют в `extra` → `TTSBadRequestError`): `model`, `text`, `stream`, `voice_setting`, `audio_setting`, `text_normalization`.

**Language alias map** (`_LANGUAGE_ALIASES`): `ru → Russian`, `en → English`, `zh → Chinese`, `ja → Japanese`, `ko → Korean`, `es → Spanish`, `fr → French`, `de → German`, `pt → Portuguese`, `it → Italian`, `ar → Arabic`, `hi → Hindi`. Полные имена (`Russian`, `English`, …) передаются as-is.

> ⚠️ `volume=0.0` сейчас принимается валидацией (`0.0 <= vol <= 10.0`), но MiniMax спеке требует `vol > 0` (`exclusiveMinimum: 0`). Это зафиксированная несостыковка — провайдер пропустит `vol=0`, API вернёт 400 → `TTSBadRequestError`. См. `docs/research/minimax-tts-api.md` §11.1.

### `TTSAudio`

| Поле | Тип | Описание |
|---|---|---|
| `samples` | `bytes` | int16 little-endian для PCM; для MP3/WAV — закодированный контейнер |
| `sample_rate` | `int` | Hz (из `extra_info.audio_sample_rate` MiniMax ответа) |
| `format` | `TTSFormat` | Фактический формат, **который вернул MiniMax** (для OGG → подменяется на MP3) |
| `raw` | `Any` | Полный JSON-ответ MiniMax (для диагностики; может содержать `extra_info.usage_characters` для биллинга) |
| `duration_s` (property) | `float` | `len(samples) / (sample_rate * 2)` для PCM; `0.0` для сжатых форматов |

### `TTSChunk`

| Поле | Тип | Описание |
|---|---|---|
| `samples` | `bytes` | Аудио-фрагмент (пустой для терминального чанка) |
| `sample_rate` | `int` | Hz |
| `format` | `TTSFormat` | Формат контейнера |
| `finish_reason` | `Optional[str]` | `None` для аудио-чанков; `"stop"` (успех) или `"error"` (mid-stream) для терминального |

### `TTSFormat`

Enum: `PCM`, `WAV`, `MP3`, `OGG`. Значение `OGG` в `TTSSettings.format` прозрачно фоллбэчится на `MP3` в MiniMax (см. §12).

---

## 10. Исключения

Все из `rob_box_llm.errors` (`TTSError` — базовый; `provider="minimax"` всегда установлен):

| Исключение | HTTP-статус | MiniMax `base_resp.status_msg` keyword | Retry? |
|---|---|---|---|
| `TTSAuthError` | 401, 403 | `auth`, `key`, `token` | ❌ Не retry — нет смысла с теми же кредами |
| `TTSRateLimitError` | 429 | `quota`, `rate`, `limit` | ✅ С exponential backoff (на стороне `tts_node`) |
| `TTSBadRequestError` | 400-499 (кроме auth/rate) | `invalid`, `param`, `voice` | ❌ Не retry — нужно менять вход |
| `TTSTimeoutError` | — (httpx) | — | ✅ С backoff |
| `TTSError` (базовый) | 5xx, JSONDecode, missing `data.audio`, `bytes.fromhex` упал | `base_resp.status_code != 0` (неизвестная категория) | ⚠️ Зависит от контекста |

> **Эвристика классификации** (см. `minimax_tts.py:_map_exception` и `_post`): провайдер сейчас маппит по подстрокам в `status_msg`. Для кодов `1004`, `1008`, `1039`, `2042`, `2056` это надёжно; для кода `2049` (`invalid API Key`) — надёжно; для кодов `1008`, `2042` — могут быть неточности. Зафиксировано в `docs/research/minimax-tts-api.md` §11.1 как открытая несостыковка.

**Сообщения исключений** редактируют credential-строки: если upstream proxy вернул `Authorization: Bearer eyJhbG…` в теле ошибки, эта подстрока заменится на `<redacted>` в `str(exc)`.

---

## 11. Bytes API и переменные окружения

Для коротких интеграций используйте компактный метод:

```python
async def synthesize_bytes(
    text: str,
    voice: Optional[str] = None,
    **opts: object,
) -> bytes
```

Поддерживаемые форматы:

| `format` | Возвращаемые bytes | Sample rate |
|---|---|---|
| `pcm_22050` | raw signed 16-bit little-endian mono PCM | 22 050 Hz |
| `pcm_24000` | raw signed 16-bit little-endian mono PCM | 24 000 Hz |
| `wav` | RIFF/WAVE контейнер; raw PCM оборачивается модулем `wave`, если API не вернул контейнер | 24 000 Hz |

Метод кэширует успешные результаты, объединяет одинаковые in-flight запросы,
ограничивает конкурентность и повторяет retryable ошибки. `TTSAuthError` и
`TTSBadRequestError` не повторяются.

### Переменные окружения

| ENV | Когда читается | Default | Обязательно? |
|---|---|---|---|
| `MINIMAX_API_KEY` | Конструктор, если `api_key=None` | — | Да, для реального синтеза. `healthcheck()` вернёт `ok=False` |
| `MINIMAX_GROUP_ID` | Конструктор, если `group_id=None` | — | Да, для реального синтеза. `healthcheck()` вернёт `ok=False` |
| `MINIMAX_TTS_BASE_URL` | Конструктор, если `base_url=None` | `https://api.minimax.io` | Нет. Укажите только для proxy, тестового или регионального endpoint. |

Приоритет: **аргумент конструктора** > ENV > встроенный default. Лимит
конкурентности не читается провайдером из ENV: передайте `max_concurrency`
явно. В готовом примере переменная `MINIMAX_TTS_MAX_CONCURRENCY` преобразуется
в этот аргумент на уровне приложения.

---

## 12. Ограничения

Что **НЕ поддерживается** MiniMax-провайдером осознанно (см. ADR-0003 §2.4, §5):

| Ограничение | Что вместо |
|---|---|
| `TTSFormat.OGG` (OGG/Opus) | Прозрачный fallback на MP3; `TTSAudio.format` помечается как `MP3`. Декодер на стороне `tts_node` не получит «фальшивый» маркер |
| MiniMax WebSocket `/v1/t2a_ws_v2` (для TTFA) | Используется sync HTTP. Для уменьшения TTFA — переключите `base_url="https://api-uw.minimax.io"` |
| Async T2A `/v1/t2a_async_v2` (для >10 000 символов) | Не реализовано. Если текст > 10 000 символов — режьте на стороне вызывающего |
| `output_format=url` | Не запрашивается; аудио всегда приходит hex-encoded inline |
| SSML | MiniMax не парсит SSML. Inline-паузы/IPA/пиньинь работают через специальный синтаксис в `text` (см. research §4.5) |
| Stereo (`channel=2`) | Всегда mono (`channel=1`) |
| `language_boost` (top-level) | Используется устаревшее `voice_setting.language` (работает, но не документировано в актуальной спеке) |
| Валидация `len(text) <= 10000` | На стороне вызывающего |
| Валидация `speed ∈ [0.5, 2.0]` | На стороне MiniMax; провайдер передаёт как есть |
| `vol > 0` (спека) | Провайдер принимает `0.0`, MiniMax отвергнет — зафиксированная несостыковка |

**Лимиты MiniMax API** (см. `docs/research/minimax-tts-api.md` §10):

- Общий пул **RPM = 60** для всех T2A-моделей на аккаунт.
- `max_concurrency` ограничивает число параллельных запросов, но не RPM. Если
  приложение может превысить квоту аккаунта, добавьте rate limiter перед
  вызовом провайдера.
- Текст ≤ 10 000 символов на запрос.
- Биллинг — по `extra_info.usage_characters` (1 символ входа = 1 character).

---

## Связанные документы

- [docs/guides/MINIMAX_TTS.md](../guides/MINIMAX_TTS.md) — пользовательский гайд с примерами ROS2
- [docs/guides/MINIMAX_TTS_GETTING_STARTED.md](../guides/MINIMAX_TTS_GETTING_STARTED.md) — минимальный путь от нуля до публикации в ROS2
- [`examples/tts_minimax_example.py`](../../examples/tts_minimax_example.py) — исполняемый registry/factory → PCM → WAV пример
- [ADR-0001](../adr/0001-harness-architecture.md) — базовая архитектура harness и capability registry
- [docs/research/minimax-tts-api.md](../research/minimax-tts-api.md) — research-реферат публичного API MiniMax
- [docs/architecture/minimax-tts-architecture.md](../architecture/minimax-tts-architecture.md) — реализационный контракт
- [docs/adr/0003-minimax-tts-architecture.md](../adr/0003-minimax-tts-architecture.md) — ADR с обоснованием
- [docs/adr/0008-tts-provider-extension-points-landed.md](../adr/0008-tts-provider-extension-points-landed.md) — точки расширения (`BaseTTSProvider`)

**Версия документа:** 1.1 (2026-07-24) · **Покрывает:** `rob_box_llm>=0.2.1`, `MiniMaxTTSProvider` поверх MiniMax T2A v2 HTTP.
