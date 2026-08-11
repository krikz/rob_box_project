# Phase 6: Harness P0 Finalization — Context (REVISED v2)

**Gathered:** 2026-07-28
**Status:** Ready for planning — **REVISED** (v1 «parallel implementation» archived → 06-CONTEXT-v1-parallel.md)

<domain>
## Phase Boundary

Фаза 6 финализирует ветку `feature/harness-p0-foundation` (PR #907) — **РЕАЛЬНАЯ замена старых нод** на harness-архитектуру.

**Коренное отличие от v1:** v1 делала «параллельные обёртки» (harness рядом с нодой, никто не включает). **v2 выкидывает старые ноды и заменяет их тонкими оболочками, использующими harness-порты.**

### Целевая архитектура

```
┌──────────────┐     ROS2 топики      ┌──────────────────┐     ROS2 топики      ┌──────────────┐
│  Telegram    │ ──────────────────→  │   DIALOGUE       │  ←────────────────── │  Perception  │
│  (тонкий     │   /voice/stt/result  │   (LLM-МОЗГ)     │   /sensors/data     │  (UART)      │
│   мост)      │                      │                  │                      │              │
│  409 строк   │   /voice/dialogue/   │   Harness-порты: │                      │  Без LLM!    │
│  → ~80 строк │     response ←────── │   • LLMProvider  │                      │  Своя прош.  │
│  НЕТ LLM!    │                      │   • ToolProvider │                      │              │
│  VPN в конт. │                      │   • MemoryStore  │                      │              │
└──────────────┘                      └──────────────────┘                      └──────────────┘
```

### Что переписываем (3 ноды)

| Нода | Сейчас | Будет | Суть изменений |
|------|--------|-------|----------------|
| **Dialogue** | `dialogue_node.py` 2181 строк | ~300 строк оболочка + harness-порты | LLM-логика, тулзы, DSM, memory → harness. ROS2 pub/sub → тонкая оболочка |
| **Telegram** | `telegram_node.py` 409 строк + handlers/ | ~80 строк мост | Без своего LLM! Только python-telegram-bot → ROS2 топики. VPN в контейнере |
| **Perception** | 5 нод, 3536 строк | ~200 строк UART-мост | Без LLM! micro-ROS выкидываем. Своя прошивка сенсор-борда. Perception = UART → топики |

### Что остаётся от текущего harness

**ПЕРЕИСПОЛЬЗУЕМ (порты правильные):**

| Файл | Строк | Назначение |
|------|-------|-----------|
| `harness.py` | 407 | `Harness[StateT]` — базовый класс |
| `config.py` | 612 | `HarnessConfig` |
| `lifecycle.py` | 126 | Lifecycle hooks |
| `providers/minimax.py` | 605 | `MiniMaxProvider` (ADR-0001 M1–M10) |
| `providers/dummy.py` | — | DummyProvider (тесты) |
| `providers/fake_llm.py` | — | FakeLLM (тесты) |
| `executors/ros_mcp.py` | 191 | `ROSMCPToolProvider` |
| `executors/local.py` | — | Local executor |
| `executors/mcp_bridge.py` | — | MCP bridge |
| `memory.py` + `memory/sqlite_voice.py` | — | `MemoryStore` порт + SQLite |
| `transport.py` + `transport/ros2_transport.py` | — | `Transport` порт + ROS2 impl |
| `tools.py` | — | Tool definitions |
| `effects.py` | — | `SideEffectBus` + `Effect` |
| `registry.py` | — | `HarnessRegistry` |
| `runner.py` | — | `run_harness()` |
| `clock.py` | — | `Clock` |
| `errors.py` | — | Error types |
| `tts/minimax_tts.py` + `tts/registry.py` | — | TTS provider + registry |

**ПЕРЕПИСЫВАЕМ / АДАПТИРУЕМ (v1 «параллельные обёртки» — неверная архитектура):**

| Файл | Строк | Проблема |
|------|-------|----------|
| `harnesses/dialog.py` | 439 | Parallel wrapper, never connected to dialogue_node |
| `harnesses/telegram.py` | 789 | Parallel wrapper, never connected to telegram_node |
| `harnesses/persistent.py` | 387 | Lifecycle wrapper for 6 voice nodes, не для perception |
| `core/dialogue_state_machine.py` | 316 | Может быть переиспользован после адаптации |

### Волновая структура (3 группы × N волн)

**Группа A: Dialogue — переписать dialogue_node под harness**
- W1: Выделить `LLMProvider` из `dialogue_node._build_agent()` → harness-порт
- W2: Выделить `ToolProvider` (29 тулзов + 5 skills) → harness-порт
- W3: Перенести `DialogueStateMachine` (IDLE/LISTENING/DIALOGUE/SILENCED) в harness
- W4: Перенести `MemoryStore` (VoiceMemory + FAQ) в harness
- W5: Переписать `dialogue_node.py` как тонкую оболочку (~300 строк)
- W6: Интеграционные тесты: dialogue_node + harness-порты

**Группа B: Telegram — переписать telegram_node как тонкий мост**
- W7: Удалить LLM-зависимости из telegram_node (LLMChat, MCPBridge — ВСЁ)
- W8: Переписать telegram_node как чистый ROS2-мост (~80 строк)
- W9: Интеграционные тесты: telegram → топики → dialogue

**Группа C: Perception — переписать под UART-сенсоры**
- W10: Удалить LLM-зависимости из perception (context_aggregator — весь LLM)
- W11: Переписать perception как UART-мост (~200 строк)
- W12: Интеграционные тесты: perception → топики → dialogue

**Вне скоупа Фазы 6:**
- Прошивка сенсор-борда (отдельная задача, не Python)
- Мерж PR #907 — делает пользователь
- P1-фичи (capability-фильтрация и т.д.)

</domain>

<decisions>
## Implementation Decisions

### D-01: Harness = чистый Python, тестируется без ROS2
- Harness-порты (LLMProvider, ToolProvider, MemoryStore, Transport, SideEffectBus, Clock) — **чистый Python**, без зависимости от `rclpy`
- Тесты harness — через `pytest` + `unittest.mock`, без поднятия ROS2
- Ноды = тонкие ROS2-оболочки (30–300 строк), композируют harness-порты
- **Причина:** возможность запускать и тестировать локально, без робота

### D-02: Порты переиспользуем, адаптеры переписываем
- Порты из `rob_box_harness` (LLMProvider, ToolProvider, MemoryStore, Transport, etc.) — **сохраняем**, они правильные
- Harness-адаптеры (dialog.py, telegram.py, persistent.py) — **переписываем**, они написаны под неверную архитектуру «parallel wrapper»
- DialogueStateMachine (core/dialogue_state_machine.py) — адаптируем, базовая логика правильная

### D-03: Dialogue = мозг (LLM живёт здесь)
- `dialogue_node.py` — единственная нода с LLM
- Telegram и Perception — **без LLM**, чистые мосты
- Все инструменты, skills, memory, FAQ — внутри диалоговой ноды через harness-порты

### D-04: Telegram = тонкий ROS2-мост
- `telegram_node.py` → ~80 строк: python-telegram-bot → ROS2 топики
- **Без своего LLM!** Только пересылка сообщений в dialogue_node через топики
- VPN в контейнере остаётся

### D-05: Perception = UART-мост, без LLM
- micro-ROS контейнер — **выкидываем**
- Сенсор-борд получает свою прошивку (вне скоупа, не Python)
- Perception нода = UART → ROS2 топики (~200 строк)
- Никакого LLM в perception (context_aggregator, reflection — удаляются)

### D-06: Старые ноды удаляются полностью
- `dialogue_node.py` (2181 строк) → заменяется тонкой оболочкой
- `telegram_node.py` (409 строк) → заменяется тонким мостом
- `context_aggregator_node.py` (745 строк) + 4 другие perception-ноды → заменяются UART-мостом
- Старый код не остаётся «на всякий случай» — это и есть смысл фазы

### D-07: Все три группы — параллельными волнами
- Dialogue (W1–W6), Telegram (W7–W9), Perception (W10–W12) — независимы
- Можно выполнять параллельно (разные файлы, разные пакеты)
- Интеграционные тесты (W6, W9, W12) — после своих групп

</decisions>

<interfaces>
## Key Interfaces

### Harness Ports (reusable, already in rob_box_harness)

```
LLMProvider     — chat(), stream(), model info
ToolProvider    — list_tools(), invoke(), register_tool()
MemoryStore     — append_turn(), load_recent(), save_fact(), search_facts()
Transport       — subscribe(), publish() (ROS2, fake for tests)
SideEffectBus   — emit(Effect), on(type, handler)
Clock           — now(), sleep()
```

### Node Shells (to be written)

```
DialogueShell(Node)      — pub/sub STT/VAD/dialogue, delegates to DialogCore(LLMProvider, ToolProvider, MemoryStore)
TelegramBridge(Node)     — pub/sub STT/dialogue, delegates to python-telegram-bot
PerceptionBridge(Node)   — pub/sub sensors, delegates to UART reader
```

### Topic Contract (unchanged)

```
/voice/stt/result        — String (recognised speech)
/voice/dialogue/response — String (JSON: {ssml, speech_id, emotion})
/voice/dialogue/state    — String (IDLE/LISTENING/DIALOGUE/SILENCED)
/audio/vad               — Bool (voice activity detection)
/sensors/data            — String (JSON: sensor readings) — NEW
```

</interfaces>

<risks>
## Risks

| Risk | Mitigation |
|------|-----------|
| dialogue_node переписывается с нуля — может сломаться DJ-режим, barge-in, fallback | W6: интеграционные тесты до/после. Сохранить эталонные тестовые сценарии |
| Telegram без своего LLM теряет «автономность» | Это осознанное решение. Telegram — мост, не мозг |
| Perception без micro-ROS требует новой прошивки | Прошивка — вне скоупа. Perception нода шлёт сырые данные в топики |
| Harness-порты могут не покрыть все use-case'ы dialogue_node | W1–W4: сначала порты, потом оболочка. Если порта не хватает — расширяем |

</risks>

<verification>
## Verification Gates

1. **W6 Gate:** `pytest src/rob_box_voice/test/ -k "dialogue" -v` — все диалоговые тесты проходят с новой оболочкой
2. **W9 Gate:** `pytest src/rob_box_telegram/test/ -v` — телеграм-мост шлёт/принимает топики
3. **W12 Gate:** `pytest src/rob_box_perception/test/ -v` — perception шлёт сенсорные данные
4. **Final Gate:** `mypy --strict src/rob_box_harness/ && black --check && isort --check && flake8`
5. **Final Gate:** диалоговые E2E тесты (wake word → STT → LLM → TTS) проходят без старых нод
</verification>
