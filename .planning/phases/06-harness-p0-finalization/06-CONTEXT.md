# Phase 6: Harness P0 Finalization — Context

**Gathered:** 2026-07-27
**Status:** Ready for planning

<domain>
## Phase Boundary

Фаза 6 финализирует ветку `feature/harness-p0-foundation` (PR #907) — **полная имплементация ADR-0001**: документация, Docker, харнесы для dialog/persistent/telegram, тесты, PR.

**Волновая структура — 22 атомарные волны (каждая = 1 задача):**

### Группа A: Документация (W1–W5)
| Wave | Задача | Источник |
|------|--------|----------|
| **W1** | Мерж `docs/architecture/minimax-tts-architecture.md` → `docs/adr/0003-minimax-tts-architecture.md` (401+325 строк) | Обсуждение D-01 |
| **W2** | Мерж `docs/architecture/minimax-tts-integration-design.md` → `docs/adr/0004-minimax-tts-integration-design.md` (530+659 строк) | Обсуждение D-01 |
| **W3** | Мерж фрагментов `0007a/b/c` (739 строк) → финальный `0007-minimax-tts-integration-final.md` | Обсуждение D-01 |
| **W4** | Удалить дубли из `docs/architecture/`, оставить перекрёстные ссылки на `adr/` | Обсуждение D-01 |
| **W5** | Обновить `SPEC_CURRENT.md`: P0→Done, описать P1, убрать гермесовские references | Обсуждение D-06 |

### Группа B: Docker (W6–W7)
| Wave | Задача | Источник |
|------|--------|----------|
| **W6** | Добавить `rob_box_harness` в `docker/vision/voice_assistant/Dockerfile` (+ зависимости) | Обсуждение D-04 |
| **W7** | Проверить сборку Docker-образа с harress (`docker build`) | Обсуждение D-04 |

### Группа C: DialogHarness (W8–W9)
| Wave | Задача | Источник |
|------|--------|----------|
| **W8** | `DialogHarness` адаптер: создать класс-обёртку над `Harness[StateT]`, LLM→LLMProvider, 30 tools→ToolExecutor, voice_memory→MemoryStore | ADR-0001 §2.7.1 |
| **W9** | `DialogueStateMachine`: мигрировать `DialogueManager` + IDLE/LISTENING/DIALOGUE/SILENCED в DSM | ADR-0001 §2.7.1 |

### Группа D: PersistentHarness (W10)
| Wave | Задача | Источник |
|------|--------|----------|
| **W10** | `PersistentHarness`: унификация 6 нод (audio/stt/tts/sound/led/cmd) — `HardwareLifecycle`, `StatePublisher`, `Clock`, `LoggerAdapter`, `ParameterGuard` | ADR-0001 §2.7.2 |

### Группа E: TelegramHarness (W11)
| Wave | Задача | Источник |
|------|--------|----------|
| **W11** | `TelegramHarness`: `LLMChat`→`LLMProvider`, `MCPBridge`→`ToolExecutor`, 25 handlers→`TelegramCommandRegistry`, `voice_processor`→skill, `camera_cache`→`SnapshotStore`, `auth`→middleware | ADR-0001 §2.7.3 |

### Группа F: Порты (W12–W13)
| Wave | Задача | Источник |
|------|--------|----------|
| **W12** | `ROS2Transport`: реальная реализация `Transport` для ROS2-топиков (subscribe/publish) | ADR-0001 §2.4.5 |
| **W13** | `SQLiteVoiceMemory`: реализация `MemoryStore` для persistent history (`append_turn`, `load_recent`, `save_fact`, `search_facts`) | ADR-0001 §2.4.3 |

### Группа G: Тесты (W14–W17)
| Wave | Задача | Источник |
|------|--------|----------|
| **W14** | `DialogueNode` test coverage: 9% → 80%+ | SPEC_CURRENT C1 |
| **W15** | `TelegramNode` test coverage: 0% → 50%+ | SPEC_CURRENT C2 |
| **W16** | MCP-инструменты test coverage: → 70%+ | SPEC_CURRENT C3 |
| **W17** | Интеграционные E2E тесты harress + реальные ноды (после W8–W13) | Обсуждение |

### Группа H: PR и аудит (W18–W22)
| Wave | Задача | Источник |
|------|--------|----------|
| **W18** | PR #907: опубликовать финальный сводный комментарий | SPEC_CURRENT A1 |
| **W19** | ADR-0008 аудит: проверить актуальность `tts-provider-extension-points-landed` | Обсуждение |
| **W20** | ADR-0009 аудит: проверить актуальность `integration-test-report` | Обсуждение |
| **W21** | `mypy strict-clean` на всём `rob_box_harness` | ADR-0001 §2.6.1 |
| **W22** | Линтеры: `black --line-length 120`, `isort --profile black`, `flake8` на всех изменённых файлах | Conventions |

**P0-код уже готов** (`rob_box_harness` — 88 тестов, `MiniMaxProvider` — 56 тестов, 90%+ coverage, mypy strict-clean).

**Вне скоупа Фазы 6:**
- Мерж PR #907 — делает пользователь после тестирования
- Capability-фильтрация в fallback wrapper — P1 (ADR-0001 §2.6.2)

</domain>

<decisions>
## Implementation Decisions

### D-01: Стратегия документации — единый источник правды
- **Все архитектурные документы → `docs/adr/`** как каноничный источник (Architecture Decision Records)
- **`docs/architecture/`** — только обзорные документы со ссылками на adr/ (НЕ дубликаты)
- **Дубли для мержа:**
  - `docs/architecture/minimax-tts-architecture.md` + `docs/adr/0003-minimax-tts-architecture.md` → смержить в `docs/adr/0003-minimax-tts-architecture.md`
  - `docs/architecture/minimax-tts-integration-design.md` + `docs/adr/0004-minimax-tts-integration-design.md` → смержить в `docs/adr/0004-minimax-tts-integration-design.md`
  - `docs/architecture/minimax-provider.md` (обзор) + `docs/adr/0002-minimax-provider.md` (ADR) → связать перекрёстными ссылками, сохранить оба (разные документы)
- **Фрагменты ADR-0007:** `0007a/b/c` → смержить в финальный `0007-minimax-tts-integration-final.md`, фрагменты удалить
- **Гайды MiniMax:** `MINIMAX.md` (LLM), `MINIMAX_TTS.md` (TTS), `MINIMAX_TTS_GETTING_STARTED.md` (quickstart), `api/MINIMAX_TTS.md` (API ref) — разные документы, НЕ дубли. Оставить.

### D-02: ADR-соответствие — проверка пройдена
- ✅ ADR-0001 M1–M10 все реализованы в `rob_box_harness/providers/minimax.py`
- ✅ 5 портов: LLMProvider, ToolProvider, MemoryStore, SideEffectBus, Transport + Clock
- ✅ Lifecycle: init() → run() → teardown() с идемпотентностью
- ✅ ENV-only auth (`MINIMAX_API_KEY`), redaction, retry, capabilities
- Расхождений между ADR и кодом не найдено

### D-03: Два MiniMax-провайдера — не дубли
- `rob_box_llm/providers/minimax.py` — production-grade upstream (LLMProvider контракт)
- `rob_box_harness/providers/minimax.py` — тонкая harness-обёртка (+env-auth enforcement, +chat(), +retry)
- `rob_box_harness/providers/minimax_tts.py` — TTS-провайдер (отдельный, не LLM)
- Оставить как есть. Осознанная архитектура.

### D-04: Docker-интеграция harress
- ❌ Сейчас `rob_box_harness` НЕ установлен ни в один Dockerfile
- ✅ Нужно добавить в `docker/vision/voice_assistant/Dockerfile` (или `voice_base`)
- Зависимость: `rob_box_llm>=0.2.1`, `PyYAML>=6.0` (из setup.py)

### D-05: Полная миграция нод на Harness (в рамках Фазы 6)
- **DialogHarness** (W8–W9): `DialogueNode` (~2466 строк, 9% coverage) → `DialogHarness` + `AgentSession`
  - LLM-клиент + fallback → `LLMProvider`
  - 30 инструментов / 5 skills → `ToolExecutor` + `SkillRegistry`
  - `DialogueManager` + состояния → `DialogueStateMachine`
  - `voice_memory`, `faq_store` → `MemoryStore`
- **PersistentHarness** (W10): унификация 6 нод через `HardwareLifecycle`, `StatePublisher`
- **TelegramHarness** (W11): `LLMChat`(469)+`MCPBridge`(137)+`commands.py`(534)→ports+skills
- **ROS2Transport** (W12): реальный Transport для ROS2
- **SQLiteVoiceMemory** (W13): MemoryStore для persistent history
- **Тесты** (W14–W17): DialogueNode 9→80%, TelegramNode 0→50%, MCP 70%+, E2E

### D-06: Docker-интеграция harress
- ❌ Сейчас `rob_box_harness` НЕ установлен ни в один Dockerfile
- ✅ W6: добавить в `docker/vision/voice_assistant/Dockerfile`
- ✅ W7: проверить сборку

### D-07: SPEC_CURRENT.md — обновить (W5)
- Пометить P0 как ✅ Done
- Отразить новую волновую структуру
- Убрать гермесовские references (kanban create, etc.)

### D-08: Мерж PR #907
- Делает пользователь САМ после тестирования
- Цель: `feature/harness-p0-foundation` → `develop`
- Статус: MERGEABLE, OPEN

### the agent's Discretion
- Порядок волн внутри групп: A→B→C→D→E→F→G→H (зависимости)
- W1–W5 можно параллельно (разные файлы)
- W8 должен быть перед W9 (DialogHarness → DialogueStateMachine)
- W12, W13 нужны для W8–W11 (порты используются харнесами)
- W14–W17 — после реализации (W8–W13)
- W21–W22 — последними (quality gates)
- Конкретный дизайн адаптеров — на усмотрение агента в рамках ADR-0001

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Архитектура и ADR
- `docs/adr/0001-harness-architecture.md` — Главный ADR: Harness[StateT], 5 портов, lifecycle, P0/P1 граница
- `docs/adr/0002-minimax-provider.md` — MiniMax LLM-провайдер (ADR, Accepted)
- `docs/adr/0007-minimax-tts-integration-final.md` — MiniMax TTS интеграция (финальный)
- `docs/adr/0009-harness-tts-contract.md` — TTSProvider контракт в harness

### Код
- `src/rob_box_harness/README.md` — Harness Framework API
- `src/rob_box_harness/rob_box_harness/providers/README.md` — MiniMax LLM provider API
- `src/rob_box_harness/setup.py` — Зависимости пакета
- `src/rob_box_voice/rob_box_voice/dialogue_node.py` — Текущий dialogue_node (цель миграции)

### Документация
- `SPEC_CURRENT.md` — Текущее состояние P0/P1 (источник истины)
- `docs/guides/harness-quickstart.md` — How-to: создать свой харнес
- `docs/guides/MINIMAX.md` — MiniMax LLM user-guide
- `docs/guides/MINIMAX_TTS_GETTING_STARTED.md` — MiniMax TTS quickstart

### Docker
- `docker/vision/voice_assistant/Dockerfile` — Целевой Dockerfile для harress
- `docker/vision/voice_base/Dockerfile` — Альтернатива (базовый образ)

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- `rob_box_harness.Harness[StateT]` — базовый класс для всех харнесов
- `rob_box_harness.run_harness()` — entry-point для запуска
- `rob_box_harness.providers.minimax.MiniMaxProvider` — harness-обёртка с retry/chat()
- `rob_box_harness.providers.dummy.DummyLLMProvider` — для тестов
- `EchoHarness`, `UpperHarness` — примеры харнесов (reference implementation)

### Established Patterns
- Lifecycle: `__init__` → `init()` → `run()` → `teardown()` (идемпотентный)
- Ports: ABC с явными методами (complete/stream/discover/execute/etc.)
- Config: YAML + ENV, секреты ТОЛЬКО через env
- Testing: pytest-asyncio, FakeTransport, DummyLLMProvider, RecordingBus

### Integration Points
- `docker/vision/voice_assistant/` — точка входа для Docker-интеграции
- `dialogue_node.py` — текущая реализация, требует адаптеров без поломки
- `rob_box_llm` — зависимость для LLMProvider

### Docker-стандарты проекта
- ❌ `COPY config/` в Dockerfile запрещён
- ❌ `COPY scripts/` в Dockerfile запрещён
- ✅ `network_mode: host`
- ✅ volumes: `./config:/config:ro`
</code_context>

<deferred>
## Deferred Ideas (вне Фазы 6)

- **Capability-фильтрация в fallback wrapper** — P1 (ADR-0001 §2.6.2)
- **RedisStore** (альтернатива SQLiteVoiceMemory) — будущая фаза
- **Multi-robot fleet координация** — Milestone 3+
- **AI HAT+ интеграция** — hardware не закуплено

</deferred>
