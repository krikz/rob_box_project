# MiniMax (LLM) — руководство пользователя

> Гайд по подключению `MiniMaxProvider` (text + vision) к потребителям
> `rob_box_llm`. Архитектурный контракт и решения — в
> [`architecture/minimax-provider.md`](../../architecture/minimax-provider.md)
> и [ADR-0002](../adr/0002-minimax-provider.md). Этот документ — практическая
> инструкция: API key, env, factory-конфиг, capabilities, image input,
> типовые сценарии и troubleshooting.
>
> Гайд по **TTS** через MiniMax (отдельный HTTP endpoint `t2a_v2`) —
> [`docs/guides/MINIMAX_TTS.md`](MINIMAX_TTS.md). Не путайте: это два
> разных адаптера, два разных endpoint'а, общий только секрет `MINIMAX_API_KEY`.

---

## Содержание

1. [Что такое MiniMax в контексте `rob_box_llm`](#1-что-такое-minimax-в-контексте-rob_box_llm)
2. [Получение API ключа](#2-получение-api-ключа)
3. [Настройка переменных окружения](#3-настройка-переменных-окружения)
4. [Конфигурация через provider registry / factory](#4-конфигурация-через-provider-registry--factory)
5. [Поддерживаемые модели и capability-флаги](#5-поддерживаемые-модели-и-capability-флаги)
6. [Примеры вызова](#6-примеры-вызова)
   - 6.1 [Из Python-кода (text)](#61-из-python-кода-text)
   - 6.2 [Передача изображений (vision)](#62-передача-изображений-vision)
   - 6.3 [Tool calling и thinking policy](#63-tool-calling-и-thinking-policy)
   - 6.4 [Потоковая генерация `stream()`](#64-потоковая-генерация-stream)
7. [Ограничения: rate limits, размер изображений, токены](#7-ограничения-rate-limits-размер-изображений-токены)
8. [Troubleshooting](#8-troubleshooting)
9. [Связанные документы](#9-связанные-документы)

---

## 1. Что такое MiniMax в контексте `rob_box_llm`

`MiniMaxProvider` — адаптер существующего абстрактного порта `LLMProvider`
(см. [`src/rob_box_llm/rob_box_llm/provider.py`](../../src/rob_box_llm/rob_box_llm/provider.py))
поверх OpenAI-compatible endpoint `https://api.minimax.io/v1/chat/completions`.
Модель по умолчанию — **`MiniMax-M3`** (vision-capable, text + tools + streaming).

Это **четвёртый** провайдер в общем LLM-слое:

| Провайдер | Endpoint | Default model | Назначение |
|-----------|----------|---------------|------------|
| `DeepSeekProvider` | OpenAI-compatible | `deepseek-chat` | Уже работающий прод-дефолт для dialogue/Telegram |
| `MiMoProvider` | `api.xiaomimimo.com` | `MiMo-7B` | Опциональный Xiaomi-интегратор |
| **`MiniMaxProvider`** | `api.minimax.io/v1` | **`MiniMax-M3`** | **Opt-in: единый MiniMax для chat + vision (PR #907)** |
| `FakeLLMProvider` | — | — | In-memory стенд для тестов |

> **MiniMax ≠ Xiaomi MiMo.** Несмотря на похожие имена и OpenAI-совместимый
> формат, это разные вендоры и разные endpoint'ы. `MiniMaxProvider` работает
> **только** с `api.minimax.io/v1`.

### Возможности `MiniMaxProvider`

| Capability | Поддержка | Примечание |
|------------|-----------|------------|
| `text` | ✅ | Все `MiniMax-*` модели |
| `streaming_text` | ✅ | `provider.stream()` без tools |
| `tools` | ✅ | Non-streaming вызовы; `complete()` с `tools=[...]` |
| `streaming_tools` | ❌ | Адаптер пока не агрегирует streaming tool-call deltas; см. §8.4 |
| `image_input` | ✅ | Только vision-capable модели (`*M3*`, `*M2-vision*`, `*vision*`) |

### Когда подключать MiniMax как LLM

- Нужна **vision** (анализ JPEG/PNG/WebP/GIF) — DeepSeek/MiMo этого не дают.
- Хотите один аккаунт / один счёт и для LLM, и для TTS-через-MiniMax.
- Движок MiniMax-M3 нужен в agent mode c tool-calls (рекомпиляция reasoning
  через `thinking` policy).

### Когда НЕ подключать

- **Не используйте** MiniMax для safety-critical perception (каждый кадр)
  — это облако с latency, квотой и privacy-риском; см.
  [`architecture/minimax-provider.md` §7.1](../../architecture/minimax-provider.md).
- Не пытайтесь заменить Yandex/DeepSeek на MiniMax без явного ADR —
  ROBBOX defaults охраняют backward-compatible поведение.

---

## 2. Получение API ключа

1. Зарегистрируйтесь на платформе MiniMax:
   <https://platform.minimaxi.com/user-center/basic-information/interface-key>.
2. В разделе **«Interface Key»** создайте новый ключ. MiniMax выдаёт строку
   вида `eyJhbGciOi...` (без префикса `Bearer ` — префикс добавляет
   адаптер автоматически через OpenAI SDK).
3. (Только для TTS-пути, **не для LLM**) — на той же странице скопируйте
   `MINIMAX_GROUP_ID`. Для text/vision он **не нужен**: Chat Completions не
   использует `GroupId=` query-параметр.
4. Пополните баланс: для chat тарификация обычно **по input/output токенам**;
   цены и пакеты — в личном кабинете.

> **Тестовая среда.** MiniMax даёт trial-кредиты новым аккаунтам; для
> smoke-теста хватает нескольких десятков запросов к `MiniMax-M3`.

> **Важно.** Один и тот же ключ используется и для chat, и для TTS.
> Если вы уже настраивали TTS-ноду (см. `MINIMAX_TTS.md`), ключ у вас
> есть — повторно создавать его не нужно.

---

## 3. Настройка переменных окружения

Секреты для text+vision провайдера:

| ENV | Обязательно | Назначение |
|-----|-------------|------------|
| `MINIMAX_API_KEY` | да, если MiniMax выбран | Bearer-токен для OpenAI-compatible API |
| `OPENAI_BASE_URL` | нет | Только если хотите переопределить `https://api.minimax.io/v1` (legacy escape hatch) |
| `MINIMAX_GROUP_ID` | **нет** | Нужен только для TTS-эндпоинта `t2a_v2`; chat completions его игнорируют |

Шаблон лежит в [`.env.example`](../../.env.example) (корневой):

```bash
# ============================================================
# MiniMax (LLM — Chat Completions, opt-in)
# ============================================================
# Использование: установить ROS-параметр llm_provider:=minimax
# или выбрать через factory/minimax.py. Без ключа MiniMax-провайдер
# НЕ активируется; нижестоящие fallback (deepseek, mimo) остаются.

# MiniMax API Key (Bearer-токен)
# Где взять: https://platform.minimaxi.com/user-center/basic-information/interface-key
# Формат: строка без префикса, например "eyJhbGciOi..."
MINIMAX_API_KEY=

# MiniMax base URL (опционально; по умолчанию https://api.minimax.io/v1)
# Менять только в тестовых окружениях или при проксировании через
# локальный mock-server.
MINIMAX_BASE_URL=

# MiniMax default model (опционально; по умолчанию MiniMax-M3, vision-capable)
# Допустимые значения см. в §5.
MINIMAX_MODEL=
```

В docker-compose (production / dev-стенд) секреты прокидываются
через `docker/vision/.env.secrets` (read-only mount). Сам файл в Git
не коммитится; в репозитории лежит только шаблон.

> **Безопасность ключа.** Ключ **никогда** не появляется в:
>
> * git (`.env.example`, YAML, launch-файлы);
> * ROS logs на INFO и выше;
> * `LLMResponse.raw`, исключениях, метриках (`llm_tokens_total`
>   содержит только totals);
> * stdout даже при exception (см. `MiniMaxRedactedLogFilter`).
>
> Подробнее — [ADR-0002 § «Секреты»](../adr/0002-minimax-provider.md).

---

## 4. Конфигурация через provider registry / factory

`MiniMaxProvider` ещё **не подключён в production defaults** — он
opt-in (см. ADR-0002 «Границы решения»). Чтобы активировать его в
конкретном потребителе, используйте registry/factory (M2 фаза PR #907,
в работе).

### 4.1 Прямая инициализация (composition root / тесты)

```python
# my_node/main.py
import os
from rob_box_llm import MiniMaxProvider


def build_provider():
    api_key = os.environ.get("MINIMAX_API_KEY")
    if not api_key:
        # Fallthrough к нижестоящему провайдеру — НЕ валимся здесь
        return None
    return MiniMaxProvider(
        api_key=api_key,
        # base_url и model имеют defaults, см. §5
    )
```

### 4.2 YAML-форма для registry (PR #907 M2)

Файл [`docs/guides/examples/minimax_llm.yaml`](examples/minimax_llm.yaml)
— копируемая заготовка для `composition root / factory`:

```yaml
llm:
  provider_order: [minimax, mimo, deepseek]   # MiniMax первым; fallback ниже
  providers:
    minimax:
      model: MiniMax-M3                       # vision-capable default
      base_url: https://api.minimax.io/v1     # опционально
      api_key_env: MINIMAX_API_KEY            # НЕ сам ключ!
      timeout_sec: 30
      thinking: disabled                      # latency-sensitive default
    mimo:
      model: MiMo-7B
      api_key_env: MIMO_API_KEY
    deepseek:
      model: deepseek-chat
      api_key_env: DEEPSEEK_API_KEY
```

> **Секреты.** Поле `api_key_env` хранит **имя ENV-переменной**, а не
> сам ключ. Это инвариантно всем конфигам `rob_box_llm`: ни один
> production-конфиг не принимает `api_key` напрямую. Для unit-тестов
> конструктор `MiniMaxProvider(api_key=...)` сохранён (явный in-test
> DI), но в YAML / launch так делать **нельзя**.

### 4.3 Запуск робота с новым провайдером

Переключение провайдера — это **только конфиг**, без правки кода:

```bash
# Текущее значение по умолчанию не поменялось: ROBBOX-голос на deepseek.
# Чтобы прогнать smoke-тест MiniMax-M3:
ros2 launch rob_box_voice voice_assistant.launch.py \
    llm_provider:=minimax
```

ROS-параметр `llm_provider` подхватывается `composition root` ноды;
новых топиков и payload schemas MiniMax-интеграция не вводит.

---

## 5. Поддерживаемые модели и capability-флаги

На дату 2026-07-22 `MiniMaxProvider` декларирует следующие capability:

| Поле `_CAPABILITIES` | Значение | Где проверяется |
|---------------------|----------|-----------------|
| `text` | `True` | Все `MiniMax-*` модели |
| `streaming_text` | `True` | Только без `tools` |
| `tools` | `True` | Non-streaming |
| `streaming_tools` | `False` | Отказываем с `CapabilityUnavailableError` до сети (см. §8.4) |
| `image_input` | `True` только если модель в `_MINIMAX_VISION_MODEL_TOKENS` | `_model_supports_vision()` |

### 5.1 Vision-capable модели

`capabilities_for(model)` сужает базовые capabilities до фактической
модели. Image input доступен только если имя модели матчится по
подстрокам (lowercase):

| Substring | Примеры | Vision |
|-----------|---------|--------|
| `minimax-m3` | `MiniMax-M3` | ✅ |
| `minimax-m2-vision` | `MiniMax-M2-vision` | ✅ |
| `minimax-vision` | любые `*vision*` | ✅ |

Любая другая модель (например, `MiniMax-M2.7`) получит
`ProviderCapabilities(image_input=False)`. Если потребитель вызовет
`complete([..., LLMMessage(content=[TextPart, ImagePart])])` для такой
модели, адаптер бросит **`CapabilityUnavailableError`** ещё до
сетевого вызова.

### 5.2 Список моделей MiniMax (text)

Актуальный каталог моделей — в
[документации MiniMax → Models](https://platform.minimax.io/docs/guides/models-intro).
Типовые (по состоянию на 2026-07-22):

| Model id | Назначение | Vision | Notes |
|----------|------------|--------|-------|
| `MiniMax-M3` | Default в `MiniMaxProvider` | ✅ | Универсальная: chat + image-input |
| `MiniMax-M3-chat` | Chat-only вариант | ❌ | Дешевле, если vision не нужна |
| `MiniMax-M2.7` | Бюджетная | ❌ | Минимальная стоимость токена |

Смену default делайте в своём `composition root`, **не** в
`MiniMaxProvider.DEFAULT_MODEL` — последний зафиксирован архитектурой
для parity между тестами и runtime.

---

## 6. Примеры вызова

### 6.1 Из Python-кода (text)

```python
import asyncio
from rob_box_llm import MiniMaxProvider, LLMMessage, LLMSettings


async def main() -> None:
    provider = MiniMaxProvider()  # ключ из MINIMAX_API_KEY
    try:
        resp = await provider.complete(
            [
                LLMMessage(role="system", content="Ты — голосовой ассистент робота."),
                LLMMessage(role="user", content="Привет! Который час?"),
            ],
            settings=LLMSettings(temperature=0.2, max_tokens=256),
        )
        print(f"OK: {resp.content!r} (usage: {resp.usage})")
    finally:
        await provider.aclose()


asyncio.run(main())
```

### 6.2 Передача изображений (vision)

```python
import asyncio
from pathlib import Path
from rob_box_llm import (
    MiniMaxProvider,
    TextPart,
    ImagePart,
    LLMMessage,
    LLMSettings,
)


async def caption_image(jpeg_path: Path) -> str:
    provider = MiniMaxProvider()
    try:
        resp = await provider.complete(
            [
                LLMMessage(
                    role="user",
                    content=(
                        TextPart(text="Что на изображении? Короткое описание, до 30 слов."),
                        ImagePart(
                            source=jpeg_path.read_bytes(),
                            media_type="image/jpeg",
                            detail="low",  # low | default | high
                        ),
                    ),
                ),
            ],
            settings=LLMSettings(
                model="MiniMax-M3",
                temperature=0.0,
                max_tokens=200,
            ),
        )
        return resp.content
    finally:
        await provider.aclose()


print(asyncio.run(caption_image(Path("/tmp/frame.jpg"))))
```

#### Что происходит под капотом

1. `_validate_image_bytes()` проверяет, что bytes ≤ `MINIMAX_MAX_IMAGE_BYTES`
   (10 MB); при превышении — `CapabilityUnavailableError` ещё до сети.
2. URL source передаётся как есть; bytes → base64-data-URL через общий
   `_image_part_to_openai()` в `_OpenAICompatibleProvider`.
3. Содержимое упорядочено: в OpenAI wire это
   `[{"type":"text",...}, {"type":"image_url",...}]`.
4. base64 и URL **никогда не пишутся в логи** (см. ADR-0002 §9).

#### Допустимые форматы `ImagePart.media_type`

`image/jpeg`, `image/png`, `image/webp`, `image/gif`. Другие типы
дают `ProviderError("unsupported media_type")`.

### 6.3 Tool calling и thinking policy

`MiniMaxProvider` поддерживает **non-streaming** tool calling. Для
agentic сценариев нужно переопределить thinking policy:

```python
from rob_box_llm import (
    MiniMaxProvider,
    LLMMessage,
    LLMSettings,
    ToolCall,
)


async def run_agent():
    provider = MiniMaxProvider(
        # Включить thinking для agent-mode; latency-sensitive путь
        # голоса должен оставаться с {"type": "disabled"} (default).
        thinking={"type": "enabled", "budget": 4096},
    )
    resp = await provider.complete(
        [LLMMessage(role="user", content="Найди вейпоинт 'кухня'")],
        settings=LLMSettings(max_tokens=512),
        tools=[
            {
                "type": "function",
                "function": {
                    "name": "find_waypoint",
                    "description": "Найти вейпоинт по имени",
                    "parameters": {
                        "type": "object",
                        "properties": {"name": {"type": "string"}},
                        "required": ["name"],
                    },
                },
            }
        ],
    )
    for call in resp.tool_calls:
        print(f"tool: {call.name}({call.arguments})")
```

> **Per-call override.** `settings.extra` принимает `thinking` — он
> перекрывает instance-политику на конкретный вызов, остаётся валидная
> типизация `LLMSettings` (frozen).

### 6.4 Потоковая генерация `stream()`

```python
async def stream_chat():
    provider = MiniMaxProvider()
    try:
        async for chunk in provider.stream(
            [LLMMessage(role="user", content="Расскажи короткий анекдот.")],
            settings=LLMSettings(temperature=0.7, max_tokens=200),
        ):
            if chunk.text_delta:
                print(chunk.text_delta, end="", flush=True)
            if chunk.finish_reason:
                print(f"\n[finish={chunk.finish_reason}]")
    finally:
        await provider.aclose()
```

> **Streaming + tools = отказ.** См. §8.4: `provider.stream(..., tools=...)`
> бросит `CapabilityUnavailableError` до сетевого запроса, потому что
> OpenAI-compatible wire-format adapter в P0 не агрегирует streaming
> tool-call deltas. Для tool-calls используйте `complete()`.

---

## 7. Ограничения: rate limits, размер изображений, токены

| Параметр | Значение | Где |
|----------|----------|-----|
| **HTTP timeout** | 30 секунд (default в `MiniMaxProvider.__init__`) | Конструктор, можно переопределить |
| **Размер изображения** | ≤ **10 MB** на frame (`MINIMAX_MAX_IMAGE_BYTES`) | `_validate_image_bytes()` |
| **Длина текстового сообщения** | ≤ 1 MB (внутреннее ограничение OpenAI SDK; см. `openai.BadRequestError` при превышении) | Сеть |
| **Параллельные вызовы** | На стороне MiniMax — см. личный кабинет, поле «Rate Limit» | Сеть |
| **Токены** | MiniMax тарифицирует input + output токены отдельно | `LLMResponse.usage` |

### 7.1 Rate limits (документация)

MiniMax даёт **trial-кредиты** новым аккаунтам и снимает ограничения
постепенно — точные RPM/TPM см. в
[личном кабинете → Usage / Rate Limits](https://platform.minimaxi.com/user-center/basic-information/interface-key).
Адаптер мапит `429` и `base_resp.status_msg` содержащие «rate»/«limit»/«quota»/«balance»/«billing»
на `RateLimitError` (см. `_raise_for_base_resp` в `providers/minimax.py`).

### 7.2 Token limits

Точные значения по моделям (на 2026-07-22):

| Model | Context window | Output max |
|-------|----------------|------------|
| `MiniMax-M3` | до 1M tokens (см. docs MiniMax) | см. docs MiniMax |
| `MiniMax-M3-chat` | обычно ниже | обычно ниже |

Адаптер **не** ограничивает длину сверх OpenAI SDK; ограничения
конкретной модели см. в [MiniMax Models](https://platform.minimax.io/docs/guides/models-intro).

### 7.3 Размер изображений (`MINIMAX_MAX_IMAGE_BYTES`)

Зафиксирован в коде как **10 MB**:

```python
# src/rob_box_llm/rob_box_llm/providers/minimax.py
MINIMAX_MAX_IMAGE_BYTES: int = 10 * 1024 * 1024  # 10 MB
```

Если MiniMax изменит официальное значение — обновите **одну строку**
и прогоните unit-тесты (см. `tests/test_minimax_provider.py` → раздел
«image validation»).

---

## 8. Troubleshooting

### 8.1 `AuthError`: неверный API ключ

**Симптом:**

```
AuthError: minimax: 1004 invalid api key
```

**Решения:**

* `MINIMAX_API_KEY` не задан — `factory` пропустит MiniMax и попробует
  следующий в `provider_order`.
* Ключ отозван или удалён в личном кабинете MiniMax — перевыпустите
  ключ и перезапустите ноду.
* URL пробросили на `https://api.xiaomimimo.com` (MiMo!) или другой
  вендор — `base_url` в MiniMaxProvider идёт через `MINIMAX_BASE_URL`
  или жёсткий default `https://api.minimax.io/v1`.

### 8.2 `RateLimitError`: 429 или quota

**Симптом:**

```
RateLimitError: minimax: 1005 rate limit exceeded
```

**Решения:**

* Подождите 30–60 секунд и повторите через fallback-декоратор
  (`provider_order: [minimax, mimo, deepseek]`).
* В личном кабинете MiniMax проверьте баланс / лимиты.
* Снизьте частоту запросов (добавьте 100–500 мс джиттер между
  successive frame captions).

### 8.3 `ContentFilterError`: safety / content policy

**Решение:** проверьте prompt на токсичные категории. Для корректировки
prompt-pattern см. `safe_prompts.md` (опциональный guidance в
`docs/guides/`). Адаптер не retry.

### 8.4 `CapabilityUnavailableError`: streaming + tools

**Симптом:**

```
CapabilityUnavailableError: minimax: streaming tool calls are not supported,
                              use complete() instead
```

**Решение:** это **не** ошибка — это fail-fast gate (см. ADR-0002 §5.2 +
[architecture §10/M1](../../architecture/minimax-provider.md)). Поменяйте
`provider.stream(..., tools=...)` на `await provider.complete(..., tools=...)`.
Когда M2+/M3 добавит streaming tool-call aggregation, gate автоматически
переключится на permissive и появится новый ключ в `_CAPABILITIES`.

### 8.5 `CapabilityUnavailableError`: image too big

**Симптом:**

```
CapabilityUnavailableError: minimax: image payload 15728640 bytes exceeds limit 10485760
```

**Решения:**

* Сожмите JPEG до ≤ 10 MB перед отправкой (`-quality 75`).
* Для потоковой perception уменьшите resolution на этапе capture (типовой
  config: 640×480@5fps для OAK-D Lite → ~30–60 KB на кадр).
* Если MiniMax повысил лимит — обновите `MINIMAX_MAX_IMAGE_BYTES` в
  коде и перезапустите unit-тесты.

### 8.6 Видно `MINIMAX_API_KEY=eyJhbGci...` в логах

**Не должно происходить** — `MiniMaxRedactedLogFilter` затирает
API key в любой record после его разрешения из ENV. Если всё-таки
ключ утек:

1. **Немедленно отзовите** ключ в личном кабинете MiniMax и перевыпустите.
2. Проверьте, что фильтр навешен на корневой logger:
   ```python
   import logging
   from rob_box_llm.providers.minimax import MiniMaxRedactedLogFilter
   logging.getLogger().addFilter(MiniMaxRedactedLogFilter(api_key_env="MINIMAX_API_KEY"))
   ```
3. Если утечка идёт из **своего** logger (например, `myapp.foo`) —
   фильтр нужно добавить **именно туда** (см. ADR-0002 §9).

### 8.7 `ProviderError` с `base_resp.status_code` в логах

**Симптом:** HTTP 200 OK, но `provider.complete()` бросил
`ProviderError("minimax: 1xxx something went wrong")`.

**Что делать:** коды MiniMax `>= 1000` приходят **внутри** ответа.
`base_resp.status_msg` мапится на наши `AuthError`/`RateLimitError`/
`ContentFilterError`/`ProviderError` (см. `_raise_for_base_resp`).
Если код нестандартный — загляните в
[MiniMax API → Chat → Error Codes](https://platform.minimax.io/docs/api-reference/text-chat-openai)
и заведите issue с полным `status_msg`.

---

## 9. Связанные документы

* [`architecture/minimax-provider.md`](../../architecture/minimax-provider.md)
  — целевая архитектура, capability-граф, фазы M0–M6.
* [ADR-0002](../adr/0002-minimax-provider.md) — capability-segregated
  MiniMax-интеграция (Accepted).
* [ADR-0001](../adr/0001-harness-architecture.md) — целевая архитектура
  харнесов (P0 foundation).
* [`docs/guides/MINIMAX_TTS.md`](MINIMAX_TTS.md) — пользовательский гайд
  для **MiniMax TTS** (отдельный endpoint, общий API key).
* [`docs/guides/examples/minimax_llm.yaml`](examples/minimax_llm.yaml) —
  копируемый YAML-шаблон factory-конфига.
* [`docs/guides/examples/minimax_tts.yaml`](examples/minimax_tts.yaml) —
  yaml-шаблон для TTS-ноды.
* [MiniMax OpenAI SDK ref](https://platform.minimax.io/docs/api-reference/text-openai-api) —
  официальный reference для Chat Completions.
* [MiniMax Models](https://platform.minimax.io/docs/guides/models-intro) —
  актуальный список моделей и их capabilities.
