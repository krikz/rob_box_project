# Phase 6: Harness P0 Finalization — Context

**Gathered:** 2026-07-27
**Status:** Ready for planning

<domain>
## Phase Boundary

Фаза 6 финализирует ветку `feature/harness-p0-foundation` (PR #907) — документация, Docker-интеграция, подготовка к мержу в `develop`.

**Волновая структура (простые атомарные изменения):**
- **Wave 6.1** — Документация: смержить дубли в `adr/`, удалить фрагменты ADR-0007a/b/c, обновить `SPEC_CURRENT.md`
- **Wave 6.2** — Docker: добавить `rob_box_harness` в Docker-образы (voice_assistant)
- **Wave 6.3** — dialogue_node: начать миграцию на Harness (адаптеры)

**P0-код уже готов** (`rob_box_harness` — 88 тестов, `MiniMaxProvider` — 56 тестов, 90%+ coverage, mypy strict-clean). Фаза 6 НЕ меняет P0-код, только интегрирует его.

**Вне скоупа Фазы 6:**
- P1-харнесы (DialogHarness, PersistentHarness, TelegramHarness) — Фаза 7+
- ROS2Transport, SQLiteVoiceMemory — Фаза 7+
- Мерж PR #907 — делает пользователь после тестирования
- Полная миграция dialogue_node — Фаза 7 (здесь только начало)

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

### D-05: dialogue_node — начало миграции
- Текущий `dialogue_node.py`: 0 упоминаний harness, ~2466 строк, 9% coverage
- В Фазе 6: создать адаптеры/интерфейсы, не ломая существующий код
- Полная миграция → Фаза 7

### D-06: SPEC_CURRENT.md — обновить
- Пометить P0 как ✅ Done
- Чётко описать P1 (DialogHarness, PersistentHarness, TelegramHarness)
- Добавить ссылки на новые GSD-фазы (6, 7)

### D-07: Мерж PR #907
- Делает пользователь САМ после тестирования
- Цель: `feature/harness-p0-foundation` → `develop`
- Статус: MERGEABLE, OPEN

### the agent's Discretion
- Порядок волн: 6.1 (документация) → 6.2 (Docker) → 6.3 (dialogue_node)
- Конкретный дизайн dialogue_node-адаптеров — на усмотрение агента в рамках ADR-0001
- Docker-образ: выбрать между `voice_assistant` и `voice_base` на основе анализа зависимостей

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
## Deferred Ideas

- **P1-харнесы** (DialogHarness, PersistentHarness, TelegramHarness) — Фаза 7+
- **ROS2Transport** (реальный) — Фаза 7+
- **SQLiteVoiceMemory / RedisStore** — Фаза 7+
- **Полная миграция dialogue_node** — Фаза 7
- **Capability-фильтрация в fallback wrapper** — P1 (ADR-0001 §2.6.2)
- **ADR-фрагменты 0008** (tts-provider-extension-points-landed) — решить судьбу при доку-аудите
</deferred>
