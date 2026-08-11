# PR #907 — Ревью HTTP-клиента MiniMax: ошибки и таймауты

- **PR:** https://github.com/krikz/rob_box_project/pull/907
- **Scope:** только HTTP-клиенты MiniMax (LLM + TTS), обработка сетевых ошибок, таймауты, защита от зависаний, идемпотентность, async-корректность.
- **Head SHA:** `6e75eb39`
- **Файлы, попавшие в этот срез ревью:**
  - `src/rob_box_llm/rob_box_llm/errors.py` (иерархия доменных ошибок)
  - `src/rob_box_llm/rob_box_llm/providers/minimax.py` (LLM-провайдер MiniMax, OpenAI-compatible)
  - `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py` (TTS-провайдер MiniMax, httpx)
  - `src/rob_box_llm/rob_box_llm/providers/deepseek.py` (`_OpenAICompatibleProvider`, общая база для LLM)
  - `src/rob_box_llm/rob_box_llm/tts_provider_base.py` (`BaseTTSProvider`, фабрика клиента)
  - `src/rob_box_voice/rob_box_voice/tts_node.py` (retry-обёртка над MiniMax TTS)

## TL;DR

| Подсистема | Общая оценка | Блокер? |
| --- | --- | --- |
| Маппинг сетевых/HTTP-исключений в доменные ошибки | Хорошо (покрыто параметризованными тестами) | — |
| Таймауты (connect/read/total) | **Плохо:** один общий `timeout: float`, нет разделения фаз | ⚠️ |
| Ретраи с backoff | **Частично:** есть в `tts_node`, нет jitter; нет в провайдере; LLM-провайдер — без ретраев | ⚠️ |
| Не-2xx ответы / JSON-ошибки | **Хорошо:** `_post_process_response` для LLM, `_post` для TTS разбирают envelope | — |
| Защита от зависаний (cancellation, лимиты) | **Слабо:** нет `asyncio.CancelledError`, нет явных лимитов на размер ответа, нет `Limits(...)` | ⚠️ |
| Async-корректность (блокирующие вызовы, пул коннектов) | **OK:** нет `time.sleep`, нет `requests`, `aclose` идемпотентный | — |
| Утечка секретов в логах | **OK:** `MiniMaxRedactedLogFilter`, `_redact_sensitive_text`, `_RedactGroupIdFilter` | — |

**Вердикт: REQUEST_CHANGES** — основные блокеры по таймаутам и защите от зависаний. Сетевые ошибки обрабатываются честно (никаких `except: pass`), и в TTS-тестах это реально проверено.

---

## 1. Маппинг сетевых ошибок

### 1.1 LLM-провайдер (`_OpenAICompatibleProvider._map_exception`, `deepseek.py:139-161`)

Покрытие классов OpenAI SDK:

- `AuthenticationError` → `AuthError` ✅
- `APITimeoutError` → `TimeoutError` ✅
- `APIConnectionError` → `TimeoutError` (с пометкой "connection-level failure — try again later") ⚠️ спорно (см. ниже)
- `APIStatusError` → `RateLimitError` для 429/403, `ContentFilterError` для контент-фильтра, иначе `ProviderError` ✅
- Неизвестные исключения → `ProviderError(str(exc))` ✅

Покрытие в тестах (`test_minimax_provider.py:555-592`, `test_deepseek_provider.py:310-410`) — параметризованные кейсы для каждой ветки.

**Замечание (Warning):**
`APIConnectionError` от OpenAI SDK — это семейство, которое включает в себя и ECONNREFUSED, и DNS resolve failures, и TLS handshake failures, и "Connection reset by peer". Маппинг всего этого в `TimeoutError` — не идеален семантически. Можно использовать `TimeoutError` как safe-default (retry с тем же промптом), но в сообщении `TimeoutError` теряется тип исходной проблемы, что затрудняет диагностику. Более честный путь: создать `ConnectionError(ProviderError)` параллельно `TimeoutError` (по ADR-0005 уже обсуждалось), либо сохранять `exc.__class__.__name__` в строке ошибки. Сейчас строка — это `str(exc)`, и в ней есть тип, но фактически в `str(APIConnectionError)` обычно попадает само сообщение OpenAI SDK, а не имя класса. Это **минорное** замечание, не блокер.

### 1.2 TTS-провайдер (`MiniMaxTTSProvider._map_exception`, `minimax_tts.py:96-135`)

Покрытие классов httpx:

- `httpx.TimeoutException` → `TTSTimeoutError` ✅
- `httpx.HTTPStatusError` → 401/403 → `TTSAuthError`, 429 → `TTSRateLimitError`, 400-499 → `TTSBadRequestError`, ≥500 → `TTSError` ✅
- прочий `httpx.HTTPError` → `TTSTimeoutError` (conservative) ⚠️
- не-TTSError / не-httpx → `TTSError` ✅

Покрытие в тестах (`test_minimax_tts_errors_parametrized.py:312-405`) — параметризованный набор из 5 транспортных классов (`ConnectTimeout`, `ReadTimeout`, `PoolTimeout`, `ConnectError`, `NetworkError`) + `asyncio.TimeoutError` отдельно. Это сильное покрытие.

**Замечание (Warning):** та же история, что и в 1.1 — `ConnectError`/`NetworkError` идут в `TTSTimeoutError`, хотя семантически это не таймаут. Тест `test_minimax_tts_errors_parametrized.py:334-370` это документирует (`expected_exc=TTSTimeoutError` явно), но если в будущем ввести `TTSConnectionError` — нужно будет обновить и тест, иначе будет silent semantic change. Текущее решение — осознанный trade-off "retry-or-not" важнее диагностики, и в комментарии к тесту это явно сказано.

### 1.3 JSON-envelope в HTTP 200 (`minimax.py:159-183`, `minimax_tts.py:678-699`)

MiniMax возвращает HTTP 200 даже на quota / auth / safety ошибки, оборачивая их в `base_resp.status_code`. Оба провайдера это корректно обрабатывают:

- LLM: `_raise_for_base_resp` разбирает по ключевым словам (`auth/key/token` → `AuthError`, `quota/balance/billing` → `RateLimitError`, `safe/content/policy` → `ContentFilterError`), иначе — общий `ProviderError`.
- TTS: разбор по словам внутри `_post` (после 200 OK) и в `stream` (для каждого SSE-события).

Это правильный подход. **Покрытие:** косвенно через `test_minimax_provider.py:tests _post_process_response`, и прямо через параметризованные тесты TTS.

**Замечание (Info):** тесты на `_raise_for_base_resp` именно LLM-провайдера я не нашёл — поищите в `test_minimax_provider.py` строки `base_resp`. Это легко добавить (`status_code != 0` → raises), но если этого нет, рассмотрите как post-merge cleanup.

---

## 2. Таймауты

### 2.1 LLM-провайдер (`minimax.py:222, 230-237`, `deepseek.py:188, 197-201`)

```python
self._client = AsyncOpenAI(
    base_url=base_url,
    api_key=api_key or "no-key-configured",
    timeout=timeout,   # float, default 30.0
)
```

`timeout` передаётся как **единственное число** в OpenAI SDK. OpenAI SDK под капотом использует `httpx.Timeout(timeout)`, что эквивалентно:

```python
httpx.Timeout(timeout)  # connect=timeout, read=timeout, write=timeout, pool=timeout
```

**Проблема:** один таймаут на все фазы запроса. Это означает:

- TCP handshake (connect): 30 секунд (на самом деле в норме < 1 секунды)
- TLS handshake: 30 секунд (входит в connect, но если сервер не отвечает на TLS — wait полные 30 секунд)
- Чтение тела ответа: 30 секунд
- Получение соединения из пула: 30 секунд

Для latency-sensitive путей (голос, perception) это слишком щедро на connect phase. Если DNS или TLS зависнут, пользователь ждёт 30 секунд на каждый вызов. Это типичный anti-pattern.

**Fix (блокер):** принять `httpx.Timeout` или кортеж `(connect, read, write, pool)` и прокидывать в OpenAI SDK:

```python
from openai import AsyncOpenAI
import httpx

# В MiniMaxProvider.__init__:
timeout: float | httpx.Timeout = timeout  # принимаем либо число, либо Timeout
self._client = AsyncOpenAI(
    base_url=base_url,
    api_key=api_key or "no-key-configured",
    timeout=timeout,
)

# В провайдере: дефолты для MiniMax
DEFAULT_TIMEOUT = httpx.Timeout(connect=5.0, read=20.0, write=10.0, pool=5.0)
```

OpenAI SDK принимает любой из: `float`, `httpx.Timeout`, объект с полями `connect/read/write/pool`.

### 2.2 TTS-провайдер (`minimax_tts.py:432, 443, 581`, `tts_provider_base.py:241-252`)

Та же история:

```python
return httpx.AsyncClient(timeout=self._timeout)  # float, default 30.0
```

В `tts_provider_base.py:183` (docstring) даже упомянуто `httpx.Timeout(self._timeout)`, но в реальном коде (строки 251-252) — `httpx.AsyncClient(timeout=timeout)` где `timeout` это float.

**Fix (блокер):** то же — принять `httpx.Timeout` или кортеж. Особенно критично для streaming SSE: если MiniMax начнёт отдавать поток с задержкой между чанками, текущие 30 секунд на `read` могут быть недостаточны, а те же 30 секунд на `connect` — избыточны.

### 2.3 Ретраи (`tts_node.py:1201-1258`)

```python
delay = (backoff_ms / 1000.0) * (2**attempt)
await _asyncio.sleep(delay)
```

- ✅ Есть exponential backoff (`backoff_ms * 2**attempt`)
- ✅ Различаются ретраибельные и не-ретраибельные ошибки (AuthError/BadRequestError — без ретрая, RateLimitError — 1 ретрай per ADR-0003, остальные — полный бюджет)
- ✅ Лимит попыток (`configured_retries + 1`, capped at 3)
- ⚠️ **Нет jitter** — при одновременном отказе у нескольких клиентов они все ретраят в один и тот же момент, создавая thundering herd
- ⚠️ **Нет учёта `Retry-After`** от MiniMax (429 обычно приходит с этим заголовком)
- ⚠️ **Не идемпотентно:** TTS synthesize возвращает свежий аудио-блоб, ретрай безопасен (idempotent по тексту), но LLM `complete()` с `tools=True` НЕ идемпотентен — если сервер принял запрос, но сеть упала до получения ответа, ретрай может исполнить инструмент дважды. В LLM-провайдере ретраев вообще нет (`_map_exception` сразу пробрасывает), так что проблема сейчас не возникает — но если ретраи добавят, это станет блокером.

**Fix (Warning, не блокер):**

```python
import random
delay = (backoff_ms / 1000.0) * (2**attempt)
jittered = delay * (0.5 + random.random())  # ±50%
await _asyncio.sleep(jittered)
```

И отдельная ветка для 429: если в `exc` доступен `Retry-After`, использовать его вместо backoff.

### 2.4 LLM-провайдер — нет ретраев вообще

`_map_exception` пробрасывает сразу. Это осознанный выбор (ADR-0005?), и retry ответственность ложится на caller (например, `AgentSession`). Однако в репозитории я не нашёл общего retry-декоратора для LLM-вызовов — проверьте, что caller действительно реализует retry с учётом идемпотентности. Если нет — стоит добавить лёгкий retry-decorator в `rob_box_llm` (например, `with_retry(exceptions=(RateLimitError, TimeoutError), max_attempts=3, base=0.5, jitter=True)`) и применить его в MiniMaxProvider / DeepSeekProvider / MiMoProvider.

---

## 3. Не-2xx ответы и JSON-ошибки

Покрыто в п. 1.3. Дополнительно:

### 3.1 TTS `_post` (`minimax_tts.py:641-705`)

Сценарий "HTTP 200, но тело не JSON" обработан:

```python
try:
    data = resp.json()
except json.JSONDecodeError as exc:
    response_text = _redact_sensitive_text(
        resp.text[:200], secrets=(self._api_key, self._group_id)
    )
    raise TTSError(
        f"Non-JSON response: {response_text}", provider=self.name
    ) from exc
```

✅ Правильно: обрезаем до 200 символов (защита от огромного тела), редактируем секреты, raise с typed error. Никакого `except: pass`.

### 3.2 TTS `stream` (`minimax_tts.py:823-859`)

SSE-события, которые не парсятся как JSON, **не падают** — `_log.debug("ignoring non-JSON SSE line")` и `continue`. Это правильно для SSE: keep-alive комментарии, мусор от прокси и т.п. приходят как строки без `data:`. **Но:** если ВСЕ строки не-JSON, цикл завершится без `yield` и финальный блок `if not yielded_audio: raise TTSError("minimax stream returned no audio chunks")` сработает. ✅ Корректно.

**Замечание (Info):** в этом же цикле есть ветка, которая пришла как error envelope с `base_resp.status_code != 0`, но `yielded_audio=True` (строка 856-858). Тогда возвращается `TTSChunk(finish_reason="error")` без `raise`. Caller в `tts_node.py:1280` это ловит (`raise Exception("MiniMax stream reported error: finish_reason=error")`), что приводит к двойной обёртке ошибки. Не блокер, но если будете чистить — стоит привести к единому формату.

---

## 4. Защита от зависаний

### 4.1 Лимиты на размер ответа

Не нашёл ни в LLM, ни в TTS провайдерах **явного ограничения** на максимальный размер ответа.

- LLM: ответ от OpenAI-совместимого API парсится как объект, поэтому если MiniMax вернёт 100 МБ — клиент съест всю RAM.
- TTS: `resp.json()` загружает весь JSON, включая `data.audio` (hex-encoded audio). Для синтеза длинного текста это может быть 50+ МБ hex → ~25 МБ PCM.

**Fix (Warning):** задать `Limits(max_content_size=...)` при создании клиента:

```python
return httpx.AsyncClient(
    timeout=self._timeout,
    limits=httpx.Limits(max_connections=10, max_keepalive_connections=5),
)
```

Это ограничит и пул коннектов (сейчас дефолт 100/100, что для единичного провайдера избыточно), и размер ответа.

### 4.2 Cancellation

**Не реализовано.** Ни в одном из async-методов провайдеров нет:

- `asyncio.current_task()`
- обработки `asyncio.CancelledError`
- явной передачи контекста

OpenAI SDK и httpx поддерживают cancellation "из коробки" (оба нативно используют asyncio), но если ROS-нода завершается с активным `await self.minimax_provider.synthesize(...)`, `CancelledError` пробросится через провайдер без cleanup. `aclose()` в этот момент не вызовется, и клиент не закроет соединения.

**Fix (Warning):**

```python
async def synthesize(self, text, *, settings=None):
    ...
    try:
        ...
        data = await self._post(payload)
    except asyncio.CancelledError:
        await self.aclose()  # best-effort cleanup
        raise
    ...
```

И в `aclose`: явно `await self._client.aclose()` с `try/except Exception`. Сейчас `aclose` уже делает это (`minimax_tts.py:913-920`, `deepseek.py:372-373`), но только при штатном вызове.

### 4.3 Защита от одновременных вызовов

Не нашёл `asyncio.Lock` или `asyncio.Semaphore` ни в одном провайдере. Если caller запустит 100 параллельных `synthesize` (например, в batch-processing), httpx пул выдержит (дефолт 100), но MiniMax может упереться в rate limit и не отвечать. Это скорее caller-side concern, но стоит хотя бы задокументировать в docstring.

---

## 5. Async-корректность

### 5.1 Блокирующие вызовы

Поиск `time.sleep|requests\.|urllib\.|open\(` в `minimax.py`, `minimax_tts.py`, `deepseek.py` — **ничего не найдено**. В async-путях нет ни одного блокирующего I/O вызова. ✅

### 5.2 Пул коннектов и его переиспользование

Используется `httpx.AsyncClient(timeout=...)` без явного `Limits(...)` — дефолт 100 keepalive. Для единичного MiniMax-провайдера это ОК, но в ROS-ноде, которая перезапускается на каждый запрос (что **не делается** в `_synthesize_minimax_async` — там lazy init на ноду), это не проблема. ✅

### 5.3 `aclose` идемпотентность

- `MiniMaxTTSProvider.aclose` (`minimax_tts.py:913-920`): проверяет `self._owns_client and not self._client.is_closed`. ✅ идемпотентен.
- `_OpenAICompatibleProvider.aclose` (`deepseek.py:372-373`): `await self._client.close()`. ⚠️ без проверки `is_closed`. Если caller уже закрыл клиент — `RuntimeError: Client has not been opened` или аналогичное.

**Fix (минор):** добавить `if not self._client.is_closed` перед `close()` в `deepseek.py:372`.

---

## 6. Утечка секретов

### 6.1 LLM

`MiniMaxRedactedLogFilter` (`minimax.py:88-122`) заменяет API key в любом лог-сообщении на `***`. Это правильный паттерн (defense in depth).

### 6.2 TTS

- `_redact_sensitive_text` (`minimax_tts.py:88-93`) редактирует и API key, и GroupId из произвольных строк (используется в `_post` и `stream`).
- `_RedactGroupIdFilter` (`minimax_tts.py:278-300`) патчит URL-параметры httpx access log.
- Применяется в `__init__`: `if not any(isinstance(item, _RedactGroupIdFilter) for item in _httpx_logger.filters): _httpx_logger.addFilter(_HTTPX_GROUP_ID_FILTER)` — с проверкой, чтобы не добавлять дубль. ✅

**Замечание (минор):** `_redact_sensitive_text` использует простое `str.replace` — если ключ короче 4 символов, может редактировать куски слов. Это типичный trade-off, и в тестах (`test_minimax_tts_errors_parametrized.py:368-370`) проверяется, что sentinel-значения длинные и не ломают classification. На реальных production-ключах (~50 hex символов) — безопасно.

### 6.3 Что не покрыто

**MiniMaxProvider (LLM)** не имеет редактирования URL/headers в логах OpenAI SDK. OpenAI SDK логирует URL через httpx, и если кто-то включит INFO на `httpx` логгере, URL `https://api.minimax.io/v1/chat/completions` утечёт — это не страшно (нет секретов в URL), но `Authorization: Bearer <key>` заголовок httpx не логирует по умолчанию. ✅ достаточно.

---

## 7. Резюме замечаний

### Блокеры (нужно править до merge)

1. **Таймауты — нет разделения фаз.** Оба провайдера передают `timeout` как `float` (по умолчанию 30 секунд на всё). Принять `httpx.Timeout` или кортеж `(connect, read, write, pool)`.
   - `minimax.py:230-237` (LLM, через OpenAI SDK)
   - `minimax_tts.py:573-581` (TTS, прямой httpx)
   - `tts_provider_base.py:241-252` (базовая фабрика клиента)
2. **Размер ответа не ограничен.** Добавить `httpx.Limits(max_content_size=...)` (и заодно `max_connections`, `max_keepalive_connections`).
   - `minimax_tts.py:581`
   - `tts_provider_base.py:252`
3. **`aclose` в LLM-провайдере не идемпотентен** (`deepseek.py:372-373`) — добавить guard `is_closed`.

### Warnings (править после merge или в этом PR, если есть время)

4. **APIConnectionError → TimeoutError** семантически нечестно. Рассмотреть отдельный `ConnectionError(ProviderError)` / `TTSConnectionError`.
5. **Retry без jitter** (`tts_node.py:1247`) — добавить `random.random()` множитель.
6. **Retry не учитывает `Retry-After`** для 429.
7. **Нет cancellation cleanup** — добавить `try/except asyncio.CancelledError: await self.aclose(); raise` в synthesize/stream.
8. **Нет общего retry-decorator для LLM-провайдеров** — caller ответственен за retry, но явной инфраструктуры в `rob_box_llm` нет.

### Info (можно не трогать)

9. Тест `_raise_for_base_resp` для LLM — стоит добавить, если отсутствует.
10. `TTSChunk(finish_reason="error")` vs `raise` после `yielded_audio=True` — единый формат.
11. Документировать expected concurrency в docstring MiniMax-провайдеров.

---

## Конкретные предложения фиксов

### Fix 1: разделение timeout-фаз

В `src/rob_box_llm/rob_box_llm/providers/deepseek.py:181-201`:

```python
import httpx
from openai import AsyncOpenAI

def __init__(
    self,
    *,
    name: str,
    base_url: str,
    default_model: str,
    api_key: Optional[str] = None,
    timeout: float | httpx.Timeout = 30.0,
    client: Optional[AsyncOpenAI] = None,
) -> None:
    self.name = name
    self._base_url = base_url
    self._default_model = default_model
    if client is not None:
        self._client = client
    else:
        # OpenAI SDK принимает любой из float/Timeout; httpx.Timeout
        # даёт нам независимые connect/read/write/pool лимиты.
        self._client = AsyncOpenAI(
            base_url=base_url,
            api_key=api_key or "no-key-configured",
            timeout=timeout,
        )
```

В `src/rob_box_llm/rob_box_llm/providers/minimax.py:216-237`:

```python
DEFAULT_TIMEOUT = httpx.Timeout(connect=5.0, read=20.0, write=10.0, pool=5.0)

def __init__(
    self,
    *,
    base_url: str = DEFAULT_BASE_URL,
    api_key: Optional[str] = None,
    model: str = DEFAULT_MODEL,
    timeout: float | httpx.Timeout = DEFAULT_TIMEOUT,
    ...
):
    super().__init__(
        ...
        timeout=timeout,
        ...
    )
```

В `src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:424-448`:

```python
import httpx

DEFAULT_TIMEOUT = httpx.Timeout(connect=5.0, read=20.0, write=10.0, pool=5.0)

def __init__(
    self,
    ...,
    timeout: float | httpx.Timeout = DEFAULT_TIMEOUT,
    ...
):
    self._timeout = timeout
    ...
```

В `src/rob_box_llm/rob_box_llm/tts_provider_base.py:241-252`:

```python
def _http_client_factory(self) -> "httpx.AsyncClient":
    import httpx

    timeout: float | httpx.Timeout = getattr(self, "_timeout", 30.0)
    return httpx.AsyncClient(
        timeout=timeout,
        limits=httpx.Limits(
            max_connections=10,
            max_keepalive_connections=5,
            max_content_size=50 * 1024 * 1024,  # 50 MB
        ),
    )
```

### Fix 2: aclose идемпотентность для LLM

`src/rob_box_llm/rob_box_llm/providers/deepseek.py:372-373`:

```python
async def aclose(self) -> None:
    if not self._client.is_closed:
        await self._client.close()
```

### Fix 3: jitter в retry

`src/rob_box_voice/rob_box_voice/tts_node.py:1247`:

```python
import random
delay = (backoff_ms / 1000.0) * (2**attempt)
jittered_delay = delay * (0.5 + random.random())  # ±50%
self.get_logger().warn(
    f"⏳ MiniMax retry {attempt + 1}/{retry_budget} after {jittered_delay:.2f}s "
    f"(base {delay:.2f}s, {type(exc).__name__}: {exc})"
)
import asyncio as _asyncio
await _asyncio.sleep(jittered_delay)
```

### Fix 4: cancellation cleanup (опционально)

`src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:731-763`:

```python
async def synthesize(self, text, *, settings=None):
    ...
    try:
        data = await self._post(payload)
    except asyncio.CancelledError:
        # Best-effort cleanup before propagating cancellation.
        await self.aclose()
        raise
    samples, sample_rate = self._decode_audio(data, s.format)
    ...
```

(аналогично в `stream`).

---

## Что **не** нашёл (и почему это OK)

- `except: pass` — нигде. ✅
- `requests` в async-пути — нет. ✅
- `time.sleep` в async-пути — нет (только `asyncio.sleep` в retry-обёртке). ✅
- Утечка секретов в обычных логах — нет (всё редактируется). ✅
- Hardcoded credentials — нет (всё через env / kwarg). ✅