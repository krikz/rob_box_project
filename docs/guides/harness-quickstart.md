# Harness Quickstart — как создать свой харнес

> **Цель документа:** за 10 минут провести читателя от чистого workspace
> до работающего собственного харнеса поверх `rob_box_harness`, который
> дёргает `MiniMaxProvider` через env-ключ и реальный LLM.
>
> Это **минимальный** путь. Архитектурное обоснование — в
> [ADR-0001](../adr/0001-harness-architecture.md) (особенно §2.2, §2.5 и
> §2.6). Полный API-reference — в
> [`src/rob_box_harness/README.md`](../../src/rob_box_harness/README.md)
> и [`src/rob_box_harness/rob_box_harness/providers/README.md`](../../src/rob_box_harness/rob_box_harness/providers/README.md).

## Содержание

1. [Что понадобится](#1-что-понадобится)
2. [Шаг 1 — установить пакет](#2-шаг-1--установить-пакет)
3. [Шаг 2 — API ключ](#3-шаг-2--api-ключ)
4. [Шаг 3 — зарегистрировать свой харнес](#4-шаг-3--зарегистрировать-свой-харнес)
5. [Шаг 4 — вызвать `run_harness` (копируемый сниппет)](#5-шаг-4--вызвать-run_harness-копируемый-сниппет)
6. [Шаг 5 — YAML-конфиг и ENV](#6-шаг-5--yaml-конфиг-и-env)
7. [Что дальше](#7-что-дальше)

---

## 1. Что понадобится

- Python 3.10+ (используется ROS Humble в проде; локально для проверки достаточно 3.10+).
- Учётная запись [MiniMax](https://platform.minimaxi.com/user-center/basic-information/interface-key) с выпущенным API-ключом.
- Сетевой доступ до `api.minimax.io:443`.
- Рабочий workspace `rob_box_project` (ветка `feature/harness-p0-foundation` или новее).

Проверить, что пакет уже на месте:

```bash
ls src/rob_box_harness/rob_box_harness/
# Ожидаемо: __init__.py  clock.py  config.py  effects.py  errors.py  harness.py  ...
```

---

## 2. Шаг 1 — установить пакет

```bash
cd src/rob_box_harness
pip install -e .[dev]
pip install -e ../rob_box_llm
```

Sanity-check импортов:

```bash
python -c "from rob_box_harness import run_harness_sync, HarnessConfig; print('ok')"
# Ожидаемо: ok
```

---

## 3. Шаг 2 — API ключ

API-ключ MiniMax **обязан** приходить из ENV — YAML-литералы запрещены
(ADR-0001 §2.5.3, требование M7). Минимальная настройка:

```bash
export MINIMAX_API_KEY="eyJhbGciOi..."
```

Провайдер логирует все ошибки без утечки ключа. Если хочется
дополнительной защиты от случайных `Authorization`-ликов в логах,
прикрепите `MiniMaxRedactedLogFilter` к корневому логгеру:

```python
import logging
from rob_box_harness.providers.minimax import MiniMaxRedactedLogFilter

logging.getLogger().addFilter(MiniMaxRedactedLogFilter(api_key_env="MINIMAX_API_KEY"))
```

---

## 4. Шаг 3 — реализовать свой харнес

Харнес — это подкласс `Harness[StateT]` с двумя обязательными точками:

- атрибут `name` (строка-идентификатор, должен совпадать с `harness.kind` в конфиге),
- асинхронный метод `step(input_data) -> Any` (один «ход» харнеса).

`Harness[StateT]` уже берёт на себя lifecycle (`init` / `run` / `teardown`),
async-with, snapshot/restore и диспатч `EchoEffect` через `side_effect_bus`.
Самая тонкая часть — `step`, и для типового «user → LLM → текст» сценария
фреймворк даёт готовый помощник `run_request_response_loop`.

Минимальный пример (`my_harness.py`):

```python
"""MyHarness — харнес, который спрашивает LLM и возвращает ответ в верхнем регистре."""

from __future__ import annotations

from typing import Any, Mapping

from rob_box_harness.harness import Harness
from rob_box_harness.harnesses._base import run_request_response_loop


class MyHarness(Harness[Mapping[str, Any]]):
    """Демо-харнес: пропускает ответ LLM через ``str.upper``."""

    name = "my_harness"

    async def step(self, input_data: Any) -> str:
        return await run_request_response_loop(
            self,
            input_data,
            post_process=str.upper,
        )
```

Зарегистрируйте билдер в дефолтном реестре **один раз** при старте процесса
(например, в `__main__` или в точке входа вашего приложения):

```python
# bootstrap.py
from rob_box_harness import get_default_registry
from my_harness import MyHarness


def register_my_harness() -> None:
    """Добавить 'my_harness' к дефолтному реестру (вместе с 'echo'/'upper')."""
    registry = get_default_registry()
    registry.register("my_harness", lambda cfg: MyHarness(cfg))
```

> В тестах можно вызвать `reset_default_registry()`, чтобы получить чистый
> реестр; для приложения достаточно зарегистрировать билдер ровно один раз
> в `main()` до первого `run_harness()`.

---

## 5. Шаг 4 — вызвать `run_harness` (копируемый сниппет)

Это **главный сниппет** документа — копируйте целиком, должен
просто работать. Файл лежит рядом с `my_harness.py` и `bootstrap.py`:

```python
"""run_my_harness.py — минимальный end-to-end сценарий.

Запуск:   python run_my_harness.py
Требует:  переменную MINIMAX_API_KEY в окружении.
"""

import asyncio
import os

from rob_box_harness import HarnessConfig, run_harness
from rob_box_harness.providers.minimax import (
    MINIMAX_API_KEY_ENV,
    MiniMaxProvider,
)

# Регистрируем наш харнес один раз при импорте.
from bootstrap import register_my_harness

register_my_harness()


async def main() -> None:
    # 1. Конфиг харнеса. Значение 'kind' обязано совпадать с
    #    атрибутом MyHarness.name и с ключом регистрации.
    config = HarnessConfig.from_dict(
        {
            "harness": {"kind": "my_harness", "name": "demo"},
            "memory": {"backend": "in_memory"},
            "effects": {"bus": "noop"},
            "transport": {"kind": "fake"},
        },
    )

    # 2. Провайдер LLM. Ключ берётся строго из ENV (M7, ADR-0001 §2.5.3):
    #    если переменная не выставлена, MiniMaxProvider поднимет ConfigError
    #    с section="llm.api_key" — лучше, чем молчаливо упасть в проде.
    api_key = os.environ.get(MINIMAX_API_KEY_ENV)
    if not api_key:
        raise SystemExit(
            f"Set {MINIMAX_API_KEY_ENV} in your environment and re-run."
        )
    provider = MiniMaxProvider(api_key=api_key, model="MiniMax-M3")

    # 3. Строим харнес через фабрику и подменяем LLM-порт на свой провайдер.
    #    Дефолтный LLM у харнеса — DummyLLMProvider; init() не перетирает
    #    уже установленный llm, поэтому присваивание ДО async with — рабочее.
    from rob_box_harness import HarnessFactory, get_default_registry
    harness = HarnessFactory.create("my_harness", config, get_default_registry())
    harness.llm = provider

    # 4. Прогоняем init → run → teardown под async with.
    async with harness as h:
        result = await h.run("Привет, кто ты?")

    # 5. Печатаем результат.
    print("output:", result.output)


if __name__ == "__main__":
    asyncio.run(main())
```

Что происходит:

1. `HarnessConfig.from_dict` собирает конфиг (memory=in-memory, effects=noop, transport=fake) и валидирует `harness.kind` / `harness.name`.
2. `MiniMaxProvider(api_key=..., model="MiniMax-M3")` — единственный реальный сетевой клиент; auth идёт через `os.environ.get(MINIMAX_API_KEY_ENV)`.
3. `HarnessFactory.create("my_harness", config, get_default_registry())` находит зарегистрированный билдер, создаёт `MyHarness(config)` (с `llm=None`).
4. `harness.llm = provider` подменяет дефолтный `DummyLLMProvider` на наш `MiniMaxProvider` **до** `init()`, поэтому `init()` сохраняет наш выбор.
5. `async with harness as h: await h.run(...)` прогоняет `init → run → teardown`. Внутри `step()` дёргает `self.llm.complete(...)`, кладёт результат в `memory`, диспатчит `EchoEffect` через `side_effect_bus`, возвращает текст.

Ожидаемый вывод (зависит от модели; пример — для иллюстрации структуры):

```
output: ПРИВЕТ! Я ВИРТУАЛЬНЫЙ АССИСТЕНТ РОБОТА РОББОКС.
```

> Если у вас нет сети/ключа — замените `MiniMaxProvider(...)` на
> `DummyLLMProvider()` из `rob_box_harness.providers`. Для `DummyLLMProvider`
> вход `"Привет, кто ты?"` вернёт строку `"echo: Привет, кто ты?"`,
> `post_process=str.upper` превратит её в `"ECHO: ПРИВЕТ, КТО ТЫ?"`.

---

## 6. Шаг 5 — YAML-конфиг и ENV

Для прод-кейса переносим конфиг в YAML, секреты остаются в ENV:

```yaml
# harness.config.yaml
harness:
  kind: my_harness
  name: demo

memory:
  backend: in_memory

effects:
  bus: noop

transport:
  kind: fake

logging:
  level: INFO
  redact: [MINIMAX_API_KEY]
```

```python
import asyncio
import os

from rob_box_harness import HarnessConfig, HarnessFactory, get_default_registry
from rob_box_harness.providers.minimax import (
    MINIMAX_API_KEY_ENV,
    MiniMaxProvider,
    build_minimax_provider,
)

from bootstrap import register_my_harness

register_my_harness()


async def main() -> None:
    # 1. config — ${MINIMAX_API_KEY} подставится из os.environ через _interpolate_env.
    config = HarnessConfig.from_dict(
        {
            "harness": {"kind": "my_harness", "name": "demo"},
            "llm": {
                "provider": "minimax",
                "model": "MiniMax-M3",
                "api_key": "${MINIMAX_API_KEY}",
            },
            "memory": {"backend": "in_memory"},
            "effects": {"bus": "noop"},
            "transport": {"kind": "fake"},
        },
    )

    # 2. provider — фабрика сама читает config.llm и ENV.
    provider = build_minimax_provider(config.llm)

    # 3. Сборка харнеса + подмена LLM-порта.
    harness = HarnessFactory.create("my_harness", config, get_default_registry())
    harness.llm = provider

    async with harness as h:
        result = await h.run("Привет!")

    print("output:", result.output)


if __name__ == "__main__":
    asyncio.run(main())
```

Переменные окружения:

| Variable | Required | Purpose |
|----------|----------|---------|
| `MINIMAX_API_KEY` | yes | API-ключ для `https://api.minimax.io` |

Если ключ не выставлен, `build_minimax_provider` поднимет `ConfigError`
с `section="llm.api_key"` — намеренно, чтобы случайный YAML без секрета
не ушёл в прод.

---

## 7. Что дальше

- **Добавить tools** — `tools: {provider: mcp_bridge, endpoint: /mcp/execute}` и
  использовать `harness.tools.execute(call)` в `step()` (см.
  [`src/rob_box_harness/rob_box_harness/tools.py`](../../src/rob_box_harness/rob_box_harness/tools.py)).
- **Подключить real effects** — заменить `effects.bus: noop` на `composite`
  с `composite: [<bus-name>, ...]`, чтобы `EchoEffect`/`LogEffect` пошли в
  продакшен-каналы (имена шин добавляются в P1 вместе с адаптерами TTS/sound/LED).
- **Real transport** — заменить `transport.kind: fake` на `ros2` с
  топиками `/voice/stt/result`, `/audio/vad`, `/telegram/updates` (P1).
- **Persistent memory** — `memory.backend: sqlite` + `path: ~/.rob_box/voice.db`
  (требует `SQLiteVoiceMemory` — P1).
- **Тесты** — pytest-фикстуры из `src/rob_box_harness/test/conftest.py`
  подменяют LLM на `DummyLLMProvider` без сети, можно копировать их
  для своего харнеса 1-в-1.

## Связанные документы

- [ADR-0001 — Целевая архитектура харнесов](../adr/0001-harness-architecture.md)
- [ADR-0002 — MiniMax LLM-провайдер](../adr/0002-minimax-provider.md)
- [`src/rob_box_harness/README.md`](../../src/rob_box_harness/README.md) — API-reference фреймворка
- [`src/rob_box_harness/rob_box_harness/providers/README.md`](../../src/rob_box_harness/rob_box_harness/providers/README.md) — `MiniMaxProvider` API
- [MiniMax LLM user-guide](MINIMAX.md) — полный гайд по capabilities / vision / tools
- [`SPEC_CURRENT.md`](../../SPEC_CURRENT.md) — источник истины по состоянию P0
