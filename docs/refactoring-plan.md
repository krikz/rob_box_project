# План рефакторинга: целевые харнесы dialog / persistent / telegram

**Дата:** 2026-07-18
**Статус:** Proposed (приложение к ADR-0001)
**Автор:** architect
**Контекст:** `t_8c0ae7dd` — на основе ADR-0001, `docs/analysis/current-nodes.md` (t_0f7b815c) и `docs/research/harness-best-practices.md` (t_ebdc4c99).

---

## Содержание

1. [Текущее состояние (резюме анализа)](#1-текущее-состояние-резюме-анализа)
2. [Целевая архитектура (резюме ADR)](#2-целевая-архитектура-резюме-adr)
3. [Приоритеты и фазы](#3-приоритеты-и-фазы)
4. [P0 — Фундамент (Неделя 1–2)](#4-p0--фундамент-неделя-12)
5. [P1 — Диалоговый харнес + Telegram v1 (Неделя 3–5)](#5-p1--диалоговый-харнес--telegram-v1-неделя-35)
6. [P2 — Persistent + Telegram v2 + observability (Неделя 6–8)](#6-p2--persistent--telegram-v2--observability-неделя-68)
7. [Критерии приёмки](#7-критерии-приёмки)
8. [Риски и митигации](#8-риски-и-митигации)
9. [Зависимости между задачами](#9-зависимости-между-задачами)
10. [Не делаем](#10-не-делаем)

---

## 1. Текущее состояние (резюме анализа)

Из `docs/analysis/current-nodes.md` (45 code smells в трёх категориях):

**Dialog (DialogueNode, 2466 LOC, coverage 9%)** — самые тяжёлые проблемы:
- D1. God-object: один класс = state machine + LLM + 30 tools + 5 skills + streaming + memory + FAQ + VAD/wake + volume/pitch/mapping handlers.
- D3. Два параллельных пути стриминга (`_ask_llm_streaming` vs `_ask_llm_non_streaming`) с почти идентичной логикой.
- D5. Tool execution не имеет общего контракта: `_execute_tool_calls` и `LLMToolCallAdapter` дублируют retry/timeout.
- D8. Skill sub-agents живут в отдельном модуле, но shared `model` + `adapter` инжектятся вручную.
- D11. `voice_memory` импортируется через try/except — нет декларации зависимости.
- D12. `VoiceSettingsSkill` (volume/pitch/speed) — три почти одиначных handler'а.

**Persistent (audio/stt/tts/sound/led/command)**:
- P1. У каждой ноды свой lifecycle: connect/disconnect/error/restart — без общего шаблона.
- P3. `state`-топики имеют разные форматы (`ready`, `running`, `error_no_device`, `synthesizing`, `playing_<name>`…).
- P5. Каждая нода пишет свой `parameters_callback`, нет общего валидатора.
- P7. `command_node` и `dialogue_node` оба слушают `/voice/stt/result` — race condition.

**Telegram (TelegramNode + handlers, ~2300 LOC, coverage ~0%)**:
- T1. LLM-клиент дублируется в `telegram/llm_chat.py` и `voice/dialogue_node.py`.
- T2. `commands.py` — 25 handler-функций, 534 строки, никакой декларативности.
- T3. Нет моста к `VoiceMemory`/`FAQStore` — TG-юзер и голосовой юзер видят разный мир.
- T5. Side-effects разбросаны: `bot.send_message` напрямую + `publish_tts` + `cmd_vel_pub`.
- T7. `voice_processor.py` — отдельный STT для голосовых сообщений, без общего контракта с `STTNode`.
- T12. `MCPBridge` синхронный, а tool execution может быть асинхронным — race condition.

**Покрытие (coverage.json, актуально на момент анализа)**:

| Модуль | Statements | Покрыто | % |
|--------|-----------:|--------:|---:|
| `dialogue_node.py` | 1331 | 179 | 9% |
| `telegram_node.py` + handlers | ~1800 | 0 | 0% |
| `audio_node`, `stt_node`, `tts_node`, `sound_node`, `led_node`, `command_node` | среднее | 30–60% | — |

---

## 2. Целевая архитектура (резюме ADR)

Полная версия — в `docs/adr/0001-harness-architecture.md`. Кратко:

```
Adapters:    DialogHarness | PersistentHarness(s) | TelegramHarness
                                │
Harness:     AgentSession  ◄─── Ports: LLMProvider, ToolExecutor,
              ▲                    MemoryStore, SideEffectBus, Transport,
              │                    Clock, SnapshotStore
Core:        BaseSkill, DialogueStateMachine, ConversationHistory,
             CapabilityRegistry, Effect[T]
```

**Инварианты** (НЕ нарушать в фазах):
- ROS2-топики и их payload не меняются.
- Поведение, наблюдаемое снаружи (команды `/say`, `/goto`, wake word) не меняется в P0/P1.
- Существующие unit-тесты остаются зелёными до тех пор, пока соответствующий модуль не будет явно переписан в плане.

---

## 3. Приоритеты и фазы

| Приоритет | Фаза | Что | Когда | Риск |
|-----------|------|-----|-------|------|
| **P0** | 1 | Базовые порты + DialogHarness skeleton | Неделя 1–2 | низкий |
| **P0** | 2 | Вынос `DialogueManager` + `VoiceMemory`-обёртка | Неделя 2 | низкий |
| **P1** | 3 | `AgentSession` + `SideEffectBus` + `ToolExecutor` | Неделя 3 | средний |
| **P1** | 4 | DialogHarness v2: миграция `dialogue_node.py` | Неделя 4–5 | средний |
| **P1** | 5 | TelegramHarness v1: `commands.py` → `TelegramCommandRegistry` | Неделя 5 | средний |
| **P2** | 6 | PersistentHarness: lifecycle/state-publisher | Неделя 6 | низкий |
| **P2** | 7 | TelegramHarness v2: общий `AgentSession` с голосовым | Неделя 7 | средний |
| **P2** | 8 | Snapshot/observability + e2e тесты | Неделя 8 | средний |

**Принцип**: каждый шаг — работоспособный код (никаких «закоммитим скелет, допилим потом»). Если фаза не доделана — откатываем, не оставляем полу-мигрированный код.

---

## 4. P0 — Фундамент (Неделя 1–2)

### Задача P0.1 — Shared-модуль `rob_box_llm`

**Цель**: единая реализация LLM-клиента, переиспользуемая из voice и telegram.

**Действия**:
1. Создать `src/rob_box_llm/`:
   - `rob_box_llm/__init__.py`
   - `rob_box_llm/provider.py` — `LLMProvider` (ABC) + `LLMMessage`, `LLMResponse`, `LLMChunk`.
   - `rob_box_llm/providers/deepseek.py` — реализация через `openai.AsyncOpenAI` с `base_url`.
   - `rob_box_llm/providers/mimo.py` — то же, для MiMo.
   - `rob_box_llm/providers/fake.py` — для тестов (заданные ответы, проверка вызовов).
   - `rob_box_llm/errors.py` — `RateLimitError`, `TimeoutError`, `ContentFilterError`, `ProviderError`.
2. Перенести общий код из `telegram/llm_chat.py` и `dialogue_node._init_llm_client`.
3. Тесты:
   - `tests/test_provider_deepseek.py` (mock HTTP)
   - `tests/test_provider_mimo.py` (mock HTTP)
   - `tests/test_provider_fake.py`
   - `tests/test_provider_errors.py` (mapping ошибок)

**Что НЕ делаем**: не трогаем существующих потребителей, только добавляем новый модуль и тесты.

**Критерии приёмки**:
- `LLMProvider` ABC с методами `complete()` и `stream()` (async).
- `DeepSeekProvider` и `MiMoProvider` параметризуются через `base_url`, `api_key`, `model`.
- `FakeLLMProvider` принимает список «запланированных ответов» для тестов.
- Тесты зелёные; coverage нового модуля ≥ 90%.

**Риск**: низкий — это новый код, существующие ноды не задеты.

---

### Задача P0.2 — Shared-модуль `rob_box_core` (time, logging, params)

**Цель**: общие утилиты для всех нод, чтобы устранить копипаст.

**Действия**:
1. `src/rob_box_core/`:
   - `clock.py` — `Clock` (ABC) + `SystemClock` + `MockClock`.
   - `logger.py` — `get_node_logger(name)` с structured fields; обёртка над `rclpy.logging`.
   - `param_guard.py` — `declare_and_validate(node, schema)` — единый declare + type-check.
   - `lifecycle.py` — `NodeLifecycle` (ABC): `on_start / on_configure / on_cleanup / on_shutdown / on_error`.

2. Тесты для каждого.

**Критерии приёмки**:
- `MockClock` управляем (`advance(seconds)`, `set_now(t)`).
- `param_guard` падает с человекочитаемой ошибкой при неверном типе параметра.
- Coverage ≥ 85%.

**Риск**: низкий. Использование в P1, не в P0.

---

### Задача P0.3 — Извлечение `DialogueManager` в отдельный модуль

**Цель**: `DialogueManager` уже изолирован в `core/dialogue_manager.py`. Провести инвентаризацию и подготовить к переезду в shared.

**Действия**:
1. Добавить `DialogueStateMachine` (новый) — обёртка с явным набором transitions.
2. Добавить type hints на 100% методов `DialogueManager`.
3. Добавить unit-тесты:
   - `test_state_transitions.py` (все валидные переходы)
   - `test_invalid_transitions.py` (попытки нарушить — должны быть отвергнуты)
   - `test_silence_mode.py`
   - `test_timeout.py`
   - `test_query_accumulation.py`

**Критерии приёмки**:
- `DialogueStateMachine` имеет явный `Enum` валидных переходов.
- Coverage `dialogue_manager.py` ≥ 95%.
- Никаких изменений в `dialogue_node.py`.

**Риск**: низкий.

---

### Задача P0.4 — `VoiceMemory` → `MemoryStore` интерфейс

**Цель**: сделать `VoiceMemory` одной из реализаций порта `MemoryStore`.

**Действия**:
1. В `src/rob_box_core/memory.py`:
   ```python
   class MemoryStore(ABC):
       async def append_turn(self, role: str, content: str, **meta) -> None: ...
       async def load_recent(self, limit: int, scope: str = "default") -> list[Turn]: ...
       async def save_fact(self, fact: str, category: str = "general") -> int: ...
       async def search_facts(self, query: str, limit: int = 5) -> list[Fact]: ...
       async def search(self, query: str, limit: int = 5) -> list[MemoryHit]: ...
   ```
2. Обернуть `VoiceMemory` в `SQLiteVoiceMemory(MemoryStore)`.
3. `InMemoryStore(MemoryStore)` для тестов.
4. Тесты:
   - `test_sqlite_store.py` (текущее поведение VoiceMemory)
   - `test_in_memory_store.py`
   - `test_interface_compliance.py` (параметризованный — все реализации проходят)

**Критерии приёмки**:
- Существующие вызовы `VoiceMemory.save_turn(...)` etc. продолжают работать (через адаптер).
- `MemoryStore` интерфейс чистый (≤ 6 методов, все async).
- Coverage обоих реализаций ≥ 85%.

**Риск**: низкий. Это подготовка, не миграция.

---

## 5. P1 — Диалоговый харнес + Telegram v1 (Неделя 3–5)

### Задача P1.1 — `AgentSession` + `SideEffectBus`

**Цель**: ядро харнес-слоя. `AgentSession` координирует порты; `SideEffectBus` маршрутизирует эффекты.

**Действия**:
1. `src/rob_box_core/session.py`:
   - `AgentSession[StateT]` (Generic).
   - Конструктор принимает: `llm`, `tools`, `memory`, `effects`, `transport`, `clock`, `hooks`.
   - Метод `async on_user_input(text, source) → SessionResult`.
   - Метод `snapshot() → SessionSnapshot` (state + history + pending effects).
   - Метод `replay(snapshot) → None` (для тестов).

2. `src/rob_box_core/effects.py`:
   - `Effect` (TaggedUnion): `Speak(text, ssml)`, `PlaySound(name)`, `SetLED(pattern)`, `SendReply(channel, text)`, `Move(vel)`.
   - `SideEffectBus` (ABC): `dispatch(effect)`, `subscribe(handler)`.
   - Реализации:
     - `CompositeBus([TTSBus, SoundBus, LEDBus, TelegramBus])` — продакшен.
     - `NoopBus()` — тесты.
     - `RecordingBus()` — записывает все эффекты для replay.

3. Тесты:
   - `test_session_simple.py` — fake LLM, fake tools, NoopBus.
   - `test_session_streaming.py` — chunks → effects.
   - `test_session_replay.py` — snapshot → restore.
   - `test_effects_bus.py` — routing, fan-out, recording.

**Критерии приёмки**:
- `AgentSession` НЕ знает про ROS2, Telegram, wake-word — только про порты.
- `Snapshot` сериализуем в JSON.
- Coverage нового кода ≥ 85%.

**Риск**: средний — это сердце рефакторинга. Митигация: сначала тесты на Fake'ах, потом подключение к существующему коду через адаптер.

---

### Задача P1.2 — `ToolExecutor` порт + `MCPBridgeExecutor`

**Цель**: единая абстракция исполнения инструментов.

**Действия**:
1. `src/rob_box_core/tools.py`:
   - `ToolCall(name, args)`, `ToolResult(value, error)`.
   - `ToolExecutor(ABC)`: `execute(call, timeout=10.0) → ToolResult`, `execute_async(call) → Future`.
2. `src/rob_box_mcp_tools/executor.py` — `MCPBridgeExecutor(ToolExecutor)` на базе текущего `MCPBridge` + `LLMToolCallAdapter`.
3. `src/rob_box_core/tools/local.py` — `LocalSkillExecutor(ToolExecutor)` для прямых вызовов Python (skill'и).
4. `FakeToolExecutor` для тестов.

**Тесты**: каждый executor отдельно + `test_tool_executor_contract.py` (параметризованный).

**Критерии приёмки**: единый таймаут, единый формат ошибок, единый retry policy (через decorator).

**Риск**: средний.

---

### Задача P1.3 — `DialogHarness` v1 (тонкая нода)

**Цель**: выделить ROS2-обвязку `DialogueNode` в отдельный класс.

**Действия**:
1. `src/rob_box_voice/rob_box_voice/harness/dialog_harness.py`:
   ```python
   class DialogHarness(Node):
       """Thin ROS2 wrapper that creates AgentSession and bridges events."""
       def __init__(self):
           super().__init__("dialogue_node")
           self.session = AgentSession(...)
           self._setup_subs()  # /voice/stt/result, /audio/vad, /voice/tts/finished
           self._setup_pubs()  # /voice/dialogue/response, /voice/dialogue/state, /voice/sound/trigger
   ```
2. Перенести туда только ROS2-callback'и (`stt_callback`, `vad_callback`, `tts_finished_callback`).
3. `dialogue_node.py` теперь импортирует `DialogHarness` и делегирует ему.

**Критерии приёмки**:
- Поведение ROS2-топиков идентично (e2e тест с fake LLM).
- `dialogue_node.py` уменьшается на ≥ 30%.

**Риск**: средний. **Митигация**: feature flag `USE_HARNESS=true/false`, по умолчанию off на время P1.

---

### Задача P1.4 — Извлечение skill'ов из `dialogue_node.py`

**Цель**: `VoiceSettingsSkill` (volume/pitch/speed), `MappingSkill` (mapping/backup), `DJPlaylistSkill`.

**Действия**:
1. `src/rob_box_voice/rob_box_voice/skills/voice_settings_skill.py` — объединяет три обработчика в один skill с тремя tool'ами.
2. `src/rob_box_voice/rob_box_voice/skills/mapping_skill.py` — `_handle_mapping_command`, `_backup_rtabmap_db`, `_confirm_start_mapping`.
3. `src/rob_box_voice/rob_box_voice/skills/dj_playlist_skill.py` — `set_dj_mode`, `list_tracks`, `save_track`, `load_track`, `delete_track`.
4. Каждый skill — подкласс `BaseSkill`, имеет `@function_tool` методы.
5. Тесты: для каждого skill — happy path + 1–2 edge cases.

**Критерии приёмки**:
- Каждый skill в отдельном файле, ≤ 300 LOC.
- Coverage каждого skill ≥ 80%.

**Риск**: низкий (изоляция).

---

### Задача P1.5 — `TelegramCommandRegistry` + `TelegramHarness` v1

**Цель**: декларативная регистрация команд вместо 25 отдельных функций.

**Действия**:
1. `src/rob_box_telegram/rob_box_telegram/harness/command_registry.py`:
   ```python
   registry = TelegramCommandRegistry()
   @registry.command("say", description="Заставить робота сказать текст")
   async def cmd_say(update, context, args): ...
   ```
2. `TelegramHarness` собирает все зарегистрированные команды в `Application.add_handler`.
3. Перенос всех 25 команд из `commands.py` в `registry/`.
4. `messages.py` упрощается: `text_message_handler` → `AgentSession.on_user_input(text, source=TG)`.

**Критерии приёмки**:
- `commands.py` исчезает (или уменьшается до автогенерации).
- Каждая команда имеет docstring + тест.
- Coverage handlers ≥ 60%.

**Риск**: средний. Большой объём механической работы. Митигация: мигрируем по 5 команд за коммит, в конце — feature flag `USE_TG_HARNESS=true`.

---

## 6. P2 — Persistent + Telegram v2 + observability (Неделя 6–8)

### Задача P2.1 — `PersistentHarness` (lifecycle)

**Цель**: общий lifecycle для audio/stt/tts/sound/led/command.

**Действия**:
1. `src/rob_box_core/persistent.py`:
   - `PersistentHarness(ABC, Node)`:
     ```python
     class PersistentHarness(Node):
         def __init__(self, name, hardware_factory, **params):
             super().__init__(name)
             self.hardware = hardware_factory()
             self.lifecycle = NodeLifecycle(clock=SystemClock(), logger=...)
         def on_start(self): self.hardware.connect()
         def on_error(self, e): self.hardware.recover()
         def on_shutdown(self): self.hardware.disconnect()
     ```
2. Применить к `audio_node`, `stt_node`, `tts_node` (самые «железные»).
3. `led_node`, `sound_node`, `command_node` — на следующей итерации.

**Критерии приёмки**:
- `audio_node.py`, `stt_node.py`, `tts_node.py` уменьшаются на 20–30% (вынос lifecycle).
- `on_error` единый: log + state publish + recovery strategy (per-node configurable).

**Риск**: низкий. **Не** делаем для всех шести сразу — только для трёх «железных».

---

### Задача P2.2 — `StatePublisher` (унификация `/<node>/state`)

**Цель**: единый формат state-топиков.

**Действия**:
1. `src/rob_box_core/state.py`:
   ```python
   class NodeState:
       status: Literal["ready", "running", "degraded", "error"]
       last_error: str | None
       uptime_sec: float
       metrics: dict
   ```
2. `StatePublisher(node, topic)`: `publish(state)`, `publish_status("running")`.
3. Каждая Persistent-нода переходит на единый формат (сохраняя backward-compat для LED-топиков).

**Критерии приёмки**:
- Новый формат публикуется на дополнительный топик `/<node>/state_v2` (parallel publishing).
- Старый топик продолжает работать до окончательной миграции.

**Риск**: низкий. Делаем parallel publishing.

---

### Задача P2.3 — `STTForTelegramSkill` (общий STT для голосовых в TG)

**Цель**: TG-voice и voice-pipeline используют один STT-контракт.

**Действия**:
1. `src/rob_box_core/stt.py`:
   - `STTProvider(ABC)`: `async transcribe(audio_bytes, language) → str`.
   - `YandexSTTProvider`, `VoskSTTProvider`, `FakeSTTProvider`.
2. `STTForTelegramSkill` использует `STTProvider` (через DI).
3. `STTNode` (persistent) — постепенно переходит на тот же интерфейс.

**Критерии приёмки**:
- TG-voice → STT → `AgentSession.on_user_input(text, source=VoiceMessage)`.
- Тесты: TG-голосовое → текст → LLM → ответ (e2e).

**Риск**: средний.

---

### Задача P2.4 — TelegramHarness v2 (общий AgentSession с голосовым)

**Цель**: TG и голос делят `AgentSession`.

**Действия**:
1. `TelegramHarness` создаёт `AgentSession` с тем же набором skill'ов, что и `DialogHarness`.
2. `MemoryStore` параметризуется по `scope`: `voice` / `telegram` / `both`.
3. `CapabilityRegistry` скрывает/показывает skill'ы в зависимости от канала.
4. Команда `/say текст` → `AgentSession.on_user_input(text, source=TG)` → TTS.

**Критерии приёмки**:
- E2E сценарий: голос «робок, расскажи анекдот» → LLM → ответ и голосом, и в TG-чат.
- Memory общая (если юзер включил `share_context=true`).

**Риск**: средний. Это самый «политический» этап (где голос и TG встречаются). Митигация: feature flag `TG_SHARE_CONTEXT=false` по умолчанию.

---

### Задача P2.5 — Snapshot/observability hooks

**Цель**: `AgentSession.snapshot()` используется для дашборда и тестов.

**Действия**:
1. `Snapshot` сериализуется в JSON, доступен через ROS2-сервис `/<node>/snapshot`.
2. `hooks.on_response_chunk(chunk)` → отправляет в `/<node>/trace` (для визуализации в дашборде).
3. E2E тесты через `snapshot/replay`.

**Критерии приёмки**:
- `Snapshot` round-trip JSON.
- Один дашборд-прототип может подписаться на `/trace`.

**Риск**: низкий.

---

## 7. Критерии приёмки (общие для всего плана)

| Метрика | Сейчас | Цель | Как мерить |
|---------|-------:|-----:|------------|
| Coverage `dialogue_node.py` | 9% | ≥ 60% | `coverage report` |
| Coverage `telegram_node.py` + handlers | 0% | ≥ 60% | `coverage report` |
| Coverage `audio/stt/tts/sound` (отдельно) | 30–60% | ≥ 70% | `coverage report` |
| LOC `dialogue_node.py` | 2466 | ≤ 1200 | `wc -l` |
| LOC `commands.py` | 534 | ≤ 150 | `wc -l` |
| Кол-во копий LLM-клиента | 4 | 1 | `grep -r "AsyncOpenAI"` |
| E2E сценарий voice → TG | невозможен | работает | ручной + авто-тест |
| Время добавления нового skill | ~1 день | ≤ 2 часа | метрика команды |

**Definition of Done** (для каждой фазы):
- Все unit-тесты зелёные.
- Существующие e2e (если есть) — зелёные.
- Coverage нового/изменённого кода ≥ 80%.
- Никаких TODO в коде (`grep -rn "TODO" src/`).
- Документация обновлена (этот план + ADR при необходимости).

---

## 8. Риски и митигации

| # | Риск | Вероятность | Импакт | Митигация |
|---|------|-------------|--------|-----------|
| 1 | Поломаем ROS2-контракт топиков при выносе в harness | средняя | высокий | Параллельная публикация в P2; integration-тесты на топики. |
| 2 | LLM-порт не покрывает edge cases (rate limit, timeout, retries) | средняя | средний | Подробные тесты в P0.1; реальные provider'ы — через `respx`/`aioresponses`. |
| 3 | Skill-извлечение сломает wake-word / диалоговую логику | низкая | высокий | Feature flag `USE_HARNESS=true/false`; деплой по одному skill'у за раз. |
| 4 | Telegram v2 (общий контекст) поломает приватность пользователей | средняя | высокий | Default `TG_SHARE_CONTEXT=false`; явный opt-in в команде `/share_context`. |
| 5 | Persistent harness окажется слишком абстрактным для «железа» | средняя | средний | Не пытаемся вынести специфику устройств (ReSpeaker, ALSA, gRPC); только lifecycle/state. |
| 6 | Тесты на in-memory fakes не отражают реальное поведение LLM | средняя | средний | Раз в спринт — один «дымовой» e2e-тест с реальным LLM (в CI как optional). |
| 7 | Команда перегружена — план растягивается на >8 недель | высокая | средний | P0 — обязательный минимум; P1 и P2 — независимые потоки, можно брать параллельно. |
| 8 | Новые порты «протекают»: `AgentSession` начинает знать про ROS2 | средняя | средний | Code review фокус: любой импорт `rclpy` / `telegram` в `rob_box_core` = reject. |

---

## 9. Зависимости между задачами

```
P0.1 (LLMProvider) ──┐
                      ├──► P1.1 (AgentSession) ──┬──► P1.3 (DialogHarness v1) ──► P1.4 (Skills)
P0.2 (Core utils)  ──┤                           │
                      │                           ├──► P2.4 (TG v2 — общий session)
P0.3 (DialogueState) ─┤                           │
                      │                           └──► P2.5 (Snapshot)
P0.4 (MemoryStore)  ──┘
                      │
                      └──► P1.2 (ToolExecutor) ───► P1.5 (TelegramHarness v1) ──► P2.3 (TG STT)
                                                              │
                                                              └──► P2.4 (TG v2)

P2.1 (PersistentHarness) ──► P2.2 (StatePublisher) ──► P2.5 (Snapshot)
```

**Критический путь**: P0.1 → P1.1 → P1.3 → P2.4 (≈ 5–6 недель).
**Можно параллельно**: P1.5 (TG v1) после P1.2; P2.1 (Persistent) после P0.2.

---

## 10. Не делаем

Чтобы план не разрастался, явно фиксируем, что **вне scope**:

- Изменения формата ROS2-топиков (отдельная ADR, когда понадобится).
- Переход на другой LLM SDK (OpenAI Agents → LangGraph). Порты спроектированы так, чтобы это было возможно, но сам перенос — не сейчас.
- Мульти-юзер / мульти-робот. Адаптер модели для этого уже учтён в `AgentSession[StateT]`, но реализация — потом.
- Web-дашборд для диалогов. `Snapshot` готов, но UI — следующий этап.
- Event sourcing / CQRS (см. ADR §3.2, 3.3).
- Полная переделка skill-системы. P1.4 — только извлечение существующих; новая архитектура skill'ов — отдельная задача.

---

*Готово к ревью. После одобрения — задачи разбиваются на Kanban-карточки для implementation-профиля.*
