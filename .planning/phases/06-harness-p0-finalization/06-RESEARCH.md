# Phase 6 v2: Harness P0 Finalization — Research

**Researched:** 2026-07-28
**Domain:** Реальная замена старых нод (dialogue/telegram/perception) на harness-архитектуру. Порты переиспользуем, адаптеры переписываем, старый код удаляем.
**Confidence:** HIGH

## Summary

Фаза 6 v2 коренным образом отличается от v1. Вместо «parallel wrappers» (harness рядом с нодой) — **полная замена** старых нод тонкими оболочками, использующими harness-порты.

**Ключевое:** Порты (`LLMProvider`, `ToolProvider`, `MemoryStore`, `Transport`, `SideEffectBus`, `Clock`) уже реализованы в `rob_box_harness` на 90%+. Основная работа — не писать новые порты, а **извлечь логику из старых нод** в эти порты, после чего заменить старые ноды тонкими ROS2-оболочками.

**Структура:** 3 группы × 12 волн. Группы независимы (разные пакеты), можно параллелить.

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|-------------|----------------|-----------|
| LLMProvider | `rob_box_harness/providers/minimax.py` (605 строк, ✅ готов) | Адаптировать DeepSeek/MiMo | MiniMaxProvider уже реализует ADR-0001 M1–M10 |
| ToolProvider | `rob_box_harness/executors/ros_mcp.py` (191 строка, ✅ готов) | `executors/mcp_bridge.py` | ROSMCPToolProvider оборачивает LLMToolCallAdapter |
| MemoryStore | `rob_box_harness/memory.py` + `sqlite_voice.py` (✅ готов) | VoiceMemory из `rob_box_voice` | SQLiteVoiceMemory — готовая реализация |
| DialogueStateMachine | `rob_box_harness/core/dialogue_state_machine.py` (316 строк, ⚠️ адаптировать) | `dialogue_manager.py` из `rob_box_voice` | Базовая DSM правильная, нужно довести |
| Transport | `rob_box_harness/transport.py` + `ros2_transport.py` (✅ готов) | ROS2 pub/sub | Абстрактный Transport + ROS2 реализация |
| Dialogue shell | `rob_box_voice/dialogue_node.py` (2181 строк → ~300) | `rob_box_harness` порты | Тонкая ROS2-оболочка, вся логика в портах |
| Telegram bridge | `rob_box_telegram/telegram_node.py` (409 строк → ~80) | handlers/ (916 строк) | Только python-telegram-bot → ROS2 топики, без LLM |
| Perception bridge | `rob_box_perception/` (5 нод, 3536 строк → ~200) | UART driver | Единый UART-мост, без LLM, без micro-ROS |

## Wave Dependency Graph

```
Group A (Dialogue):
  W1: LLMProvider extraction ──┐
  W2: ToolProvider extraction  ├──→ W5: Dialogue shell ──→ W6: Integration tests
  W3: DSM extraction          │
  W4: MemoryStore extraction  ┘

Group B (Telegram):
  W7: Remove LLM deps ──→ W8: Telegram bridge ──→ W9: Integration tests

Group C (Perception):
  W10: Remove LLM deps ──→ W11: Perception bridge ──→ W12: Integration tests

All three groups are independent (different packages, no shared state).
```

## Detailed Findings

### 1. Harness Ports — State Assessment

**✅ Production-ready (не трогаем):**

| Port | File | Lines | Tests | Status |
|------|------|-------|-------|--------|
| LLMProvider (ABC) | `harness.py` | 407 | — | Base class with StateT generic |
| MiniMaxProvider | `providers/minimax.py` | 605 | 56 tests, 95% cov | ADR-0001 M1–M10 implemented |
| DummyProvider | `providers/dummy.py` | — | — | For tests |
| FakeLLM | `providers/fake_llm.py` | — | — | For tests |
| ToolProvider (ABC) | `tools.py` | — | — | Abstract: list_tools, invoke, register_tool |
| ROSMCPToolProvider | `executors/ros_mcp.py` | 191 | — | Wraps LLMToolCallAdapter |
| MCPBridge | `executors/mcp_bridge.py` | — | — | MCP tool bridge |
| MemoryStore (ABC) | `memory.py` | — | — | Abstract: append_turn, load_recent, save_fact, search_facts |
| SQLiteVoiceMemory | `memory/sqlite_voice.py` | — | — | SQLite implementation |
| Transport (ABC) | `transport.py` | — | — | Abstract: subscribe, publish |
| ROS2Transport | `transport/ros2_transport.py` | — | — | ROS2 implementation |
| SideEffectBus (ABC) | `effects.py` | — | — | Abstract: emit, on |
| Clock | `clock.py` | — | — | now(), sleep() |
| HarnessRegistry | `registry.py` | — | — | Registry of harnesses |
| HarnessConfig | `config.py` | 612 | — | Config parsing |
| TTS provider + registry | `tts/minimax_tts.py`, `tts/registry.py` | — | 64 TTS tests | ADR-0008/0009 verified |

**⚠️ Нужна адаптация:**

| File | Lines | Что не так | Что делать |
|------|-------|-----------|------------|
| `harnesses/dialog.py` | 439 | Parallel wrapper, never connected | Переписать как DialogCore — чистый Python, без ROS2 |
| `harnesses/telegram.py` | 789 | Parallel wrapper, never connected | Удалить. Telegram не должен иметь harness |
| `harnesses/persistent.py` | 387 | Lifecycle for 6 voice nodes | Удалить. Perception = UART-мост |
| `core/dialogue_state_machine.py` | 316 | DSM правильная, но не интегрирована | Интегрировать в DialogCore |

### 2. Dialogue Node — Deep Dive

**Текущая структура (2181 строка):**

| Секция | Строки | Суть | Судьба |
|--------|--------|------|--------|
| `__init__` | 154–346 (193 строки) | ROS2 pub/sub, params, agent build | → thin shell (~100 строк) |
| Provider resolution | 397–435 (39 строк) | `_resolve_*`, `_provider_overrides` | → LLMProvider (уже готов) |
| Voice memory init | 437–520 (84 строки) | `_init_voice_memory`, FAQ, event profile | → MemoryStore (уже готов) |
| `_build_agent` | 524–590 (67 строк) | Agent + model construction | → LLMProvider + DialogCore |
| `_make_tools` | 591–1056 (466 строк) | 29 `@function_tool` декораторов | → ToolProvider (адаптировать) |
| `_make_output_tools` | 1058–1236 (179 строк) | 3 output + 3 DJ tools | → ToolProvider (адаптировать) |
| `_build_skills` | 1238–1372 (135 строк) | 5 skill sub-agents | → SkillRegistry (новый порт?) |
| ROS2 callbacks | 1390–1432 (43 строки) | `_on_vad`, `_on_stt` | → thin shell |
| `_agent_run` | async, ~150 строк | LLM run loop, history, retry | → DialogCore |
| `_run_agent_with_retry` | async, ~80 строк | Error taxonomy, fallback | → LLMProvider |
| DJ mode | ~200 строк | `_build_dj_prompt`, `_on_dj_mode_msg`, `_on_dj_tick_check` | → thin shell (ROS2 timing) |
| Barge-in / cancel | `_cancel_run` (~30 строк) | Async task cancel, TTS flush | → thin shell (ROS2-specific) |
| History management | `_trim_history`, `_strip_excluded_tools`, `_truncate_history_outputs` (~80 строк) | Conversation history | → MemoryStore |
| `main()` / lifecycle | `shutdown_asyncio_loop`, `destroy_node` (~40 строк) | ROS2 lifecycle | → thin shell |

**Что извлекаем → harness-порты (W1–W4):**
- LLM client creation + retry + fallback → LLMProvider (уже есть `MiniMaxProvider`, добавить DeepSeek/MiMo адаптеры)
- 29 tools + 5 skills → ToolProvider (адаптировать `ROSMCPToolProvider`)
- DSM (IDLE/LISTENING/DIALOGUE/SILENCED) → DialogueStateMachine
- VoiceMemory + FAQ + EventProfile → MemoryStore

**Что остаётся в оболочке (W5, ~300 строк):**
- ROS2 pub/sub (8 топиков)
- Async loop driver
- VAD/STT callbacks → делегируют DialogCore
- DJ mode (таймеры на ROS2, инжект контекста)
- Barge-in/cancel (async task management)
- Lifecycle (shutdown, destroy_node)

### 3. Telegram Node — Deep Dive

**Текущая структура (409 строк + 916 handlers):**

| Компонент | Строки | Судьба |
|-----------|--------|--------|
| `TelegramNode.__init__` | 92–225 (134 строки) | → тонкий мост (~40 строк) |
| LLM-зависимости | LLMChat, MCPBridge | **Удалить полностью** |
| Camera callbacks | `_on_camera_*` (14 строк) | Оставить (пересылка в топики) |
| `publish_tts` | 270–291 (22 строки) | → пересылка в `/voice/dialogue/response` |
| Telegram loop | `_run_telegram` async (324–387, 64 строки) | Оставить (python-telegram-bot) |
| `handlers/commands.py` | 552 строки, 25 handlers | Удалить LLM-зависимости, оставить пересылку в топики |
| `handlers/messages.py` | 199 строк | Удалить LLM-зависимости |
| `handlers/callbacks.py` | 165 строк | Удалить LLM-зависимости |

**W7:** Удалить `LLMChat`, `MCPBridge`, и все LLM-вызовы из handlers.
**W8:** Переписать telegram_node как чистый мост: Telegram message → `/voice/stt/result`, `/voice/dialogue/response` → Telegram reply.

### 4. Perception Node — Deep Dive

**Текущая структура (5 нод, 3536 строк):**

| Нода | Строки | LLM? | Судьба |
|------|--------|------|--------|
| `context_aggregator_node.py` | 745 | ✅ DeepSeek client, `_summarize_events()` | **Удалить LLM**, оставить сенсорные подписки |
| `health_monitor.py` | 163 | ❌ Нет LLM | Оставить как есть или консолидировать |
| `reflection_node.py` | — | Вероятно LLM | **Удалить** |
| `startup_greeting_node.py` | — | Вероятно нет | **Удалить** (не нужно) |
| `vision_stub_node.py` | — | Нет | **Удалить** (micro-ROS выкидываем) |

**W10:** Удалить весь LLM-код из context_aggregator (DeepSeek client, `_summarize_events`, `check_and_summarize`).
**W11:** Консолидировать 5 нод → 1 perception bridge (~200 строк): UART → `/sensors/data`.

### 5. Test Strategy

| Wave | Что тестируем | Как |
|------|--------------|-----|
| W1–W4 | Harness-порты | `pytest src/rob_box_harness/test/` — existing 88 tests must stay green |
| W5 | Dialogue shell | Интеграционные: shell + fake LLM/MCP/Memory |
| W6 | Dialogue E2E | Wake word → STT → LLM → TTS, сравнить с эталоном |
| W7–W8 | Telegram bridge | Интеграционные: telegram message → топик → ответ |
| W9 | Telegram E2E | Полный цикл telegram → dialogue → telegram |
| W10–W11 | Perception bridge | UART-симулятор → `/sensors/data` |
| W12 | Perception E2E | Сенсорные данные → топик → dialogue получает |

### 6. Risk Assessment

| Risk | Probability | Impact | Mitigation |
|------|-----------|--------|------------|
| DJ-режим сломается при извлечении логики | MEDIUM | HIGH | DJ-режим остаётся в оболочке (W5), не трогаем |
| Barge-in/cancel гонки при переносе в DialogCore | MEDIUM | HIGH | `_cancel_run` остаётся в оболочке (async loop-specific) |
| Telegram без своего LLM теряет фичи | LOW | MEDIUM | Осознанное решение. Telegram — мост, не мозг |
| Perception без micro-ROS требует новой прошивки | LOW | LOW | Прошивка — вне скоупа. Perception шлёт сырые данные |
| Harness-порты не покрывают все use-case'ы | LOW | HIGH | Порты уже покрывают 90%+. W1–W4: расширяем порты ДО оболочки |

## Recommendations

1. **Порты ПЕРВЫЕ (W1–W4).** Не писать оболочку, пока порты не готовы. Это гарантирует что оболочка будет тонкой.
2. **Dialogue shell ПОСЛЕДНЯЯ в группе A (W5).** W1–W4 можно параллелить между собой.
3. **Telegram и Perception НЕЗАВИСИМЫ от Dialogue (группы B и C).** Можно параллелить все три группы.
4. **Интеграционные тесты ПОСЛЕ каждой группы (W6, W9, W12).** Не ждать конца фазы.
5. **mypy strict-clean ПОСЛЕ всех изменений.** Добавить в финальную волну.

## RESEARCH COMPLETE
