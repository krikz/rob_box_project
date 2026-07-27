# SPEC_CURRENT — Текущее состояние и ближайшие шаги

> **Версия**: 2.0
> **Дата**: 2026-07-27
> **Ветка**: `feature/harness-p0-foundation`
> **PR**: https://github.com/krikz/rob_box_project/pull/907

---

## 1. Где мы сейчас

### 1.1 Завершённые этапы

| Этап | Результат | Статус |
|---|---|---|
| **ADR-0001** — Архитектура харнесов | 956 строк: `Harness[StateT]` ABC, 5 портов, state-store, lifecycle hooks для DialogNode / PersistentNode / TelegramNode | ✅ Accepted |
| **Harness Framework P0** | `rob_box_harness` (P0): `Harness[StateT]`, lifecycle, ports (`LLMProvider` / `ToolProvider` / `MemoryStore` / `SideEffectBus` / `Transport` + `Clock`), `HarnessConfig` (YAML+ENV), `HarnessRegistry` + `run_harness()` entry-point, dummy `echo`/`upper` харнесы, 88 тестов / 90% coverage / mypy strict-clean | ✅ **Done** |
| **MiniMax Provider в Harness** | `rob_box_harness.providers.minimax.MiniMaxProvider` (ADR-0001 M1–M10): env-auth (`MINIMAX_API_KEY`, YAML-литералы запрещены), `chat(messages, **kwargs)` shortcut, retry с экспоненциальным backoff (только `RateLimitError` / `TimeoutError`), 56 тестов / 95% coverage / mypy strict-clean | ✅ **Done** |
| **MiniMax LLM-провайдер в `rob_box_llm`** | `MiniMaxProvider` (PR #907): OpenAI-compatible адаптер, `MiniMax-M3` (text+vision+tools), `base_resp` envelope, key redaction, capabilities API | ✅ Done |
| **MiniMax TTS** | `MiniMaxTTSProvider` (ADR-0007), пайплайн до ROS-топика `/voice/audio/speech` | ✅ Done |
| **PR #907** — Код ревью | Все частные ревью (backend, frontend, security, architect) проведены | ✅ Done |

| **Phase 6: Harness P0 Finalization** | Documentation consolidated (no duplicates, adr/ canonical), Docker integration, harness adapters (Dialog/Persistent/Telegram), port implementations (ROS2Transport/SQLiteVoiceMemory), test coverage gaps closed, PR #907 audit complete | ✅ Done |

### 1.2 Что уже настроено

- **Ветка** `feature/harness-p0-foundation` — P0 завершён, готов к мержу в main (PR #907)

---

## 2. Диаграмма компонентов (P0 — landed)

```mermaid
graph TB
    subgraph "Adapters (P1 — Phase 6 implemented)"
        DA[DialogHarness<br/>voice input]
        PA[PersistentHarness<br/>audio/stt/tts/sound/led/cmd]
        TA[TelegramHarness<br/>tg bot]
    end

    subgraph "Harness Layer (P0 — ✅ landed)"
        H[Harness[StateT]<br/>+ LifecycleHooks]
        LF[Lifecycle<br/>init / run / teardown]
        REG[HarnessRegistry]
        RUN[run_harness name, input, config]
    end

    subgraph "Ports (P0 — ✅ landed)"
        LLMP[LLMProvider port]
        TP[ToolProvider port]
        MS[MemoryStore port]
        SEB[SideEffectBus port]
        TR[Transport port]
        CK[Clock port]
    end

    subgraph "Built-in Harnesses (P0 — ✅ landed)"
        ECHO[EchoHarness]
        UPPER[UpperHarness]
        DUMMY_PROV[DummyLLMProvider]
    end

    subgraph "rob_box_harness.providers (P0 — ✅ landed)"
        MM_PROV[MiniMaxProvider<br/>+ RetryPolicy<br/>+ chat]
    end

    subgraph "rob_box_llm (PR #907 — ✅ landed)"
        DS_PROV[DeepSeekProvider]
        MIMO_PROV[MiMoProvider]
        MM_UP[MiniMaxProvider<br/>upstream]
        TTS_PROV[MiniMaxTTSProvider]
    end

    subgraph "External"
        MM_API[(MiniMax API<br/>api.minimax.io)]
        DS_API[(DeepSeek API)]
        MIMO_API[(MiMo API)]
        ROS2[(ROS2 topics)]
    end

    DA --> H
    PA --> H
    TA --> H

    ECHO -.extends.-> H
    UPPER -.extends.-> H
    H --> LF
    LF --> REG
    REG --> RUN

    H --> LLMP
    H --> TP
    H --> MS
    H --> SEB
    H --> TR
    H --> CK

    LLMP -.binds.-> MM_PROV
    LLMP -.binds.-> DS_PROV
    LLMP -.binds.-> MIMO_PROV
    LLMP -.binds.-> DUMMY_PROV

    MM_PROV -.delegates HTTP.-> MM_UP
    MM_UP --> MM_API
    DS_PROV --> DS_API
    MIMO_PROV --> MIMO_API

    SEB -.publishes.-> ROS2
    TR -.subscribes.-> ROS2
    TTS_PROV -.publishes audio.-> ROS2
```

**Что в P0 (✅ landed):** `rob_box_harness` (фреймворк с `Harness[StateT]`, lifecycle, 5 портов, `HarnessRegistry` + `HarnessFactory` + `run_harness()`, dummy-харнесы `EchoHarness`/`UpperHarness`, `DummyLLMProvider`) + `rob_box_harness.providers.minimax.MiniMaxProvider` (env-auth, `chat()`-shortcut, retry).
**Что в P1 (✅ Phase 6 implemented):** три реальных харнеса (`DialogHarness` / `PersistentHarness` / `TelegramHarness`, ADR-0001 §2.7), `ROS2Transport`, `SQLiteVoiceMemory`.

---

## 3. Что нужно сделать (ROADMAP — ближайшие шаги)

### Этап A: PR #907 review → main (как было)

| # | Задача | Исполнитель | Критерий готовности |
|---|---|---|---|
| A1 | Опубликовать сводный комментарий в PR #907 | `pr-reviewer` | Комментарий виден в PR #907 | ✅ Done |
| A2 | Выставить review state (APPROVE / REQUEST_CHANGES) | `pr-reviewer` | Review state установлен | ✅ Done |
| A3 | `git checkout main && git merge feature/harness-p0-foundation` | Человек | Fast-forward или конфликты разрешены | ⏳ Pending |
| A4 | `git push origin main` | Человек | main обновлён | ⏳ Pending |

### Этап B: P1 — Реальные харнесы (USE ADR-0001 §2.7)

| # | Задача | Исполнитель | Критерий | ADR |
|---|---|---|---|---|
| B1 | `DialogHarness` поверх `DialogueNode` | `backend` | Тесты >= 80%, `run_harness("dialog", input)` end-to-end | ✅ Done | ADR-0001 §2.7.1 |
| B2 | `PersistentHarness` (audio/stt/tts/sound/led/cmd) | `backend` | Один харнес драйвит все 6 persistent-нод | ✅ Done | ADR-0001 §2.7.2 |
| B3 | `TelegramHarness` поверх `TelegramNode` | `backend` | Тесты >= 50%, мост к voice через skill | ✅ Done | ADR-0001 §2.7.3 |
| B4 | `ROS2Transport` (реальный) | `backend` | `Transport` interface реализован для ROS2-топиков | ✅ Done | ADR-0001 §2.4.5 |
| B5 | `SQLiteVoiceMemory` / `RedisStore` | `backend` | `MemoryStore` interface для persistent history | ✅ Done | ADR-0001 §2.4.3 |

### Этап C: Покрытие тестами (ADR-0001 §5)

| # | Задача | Текущее | Цель |
|---|---|---|---|
| C1 | `DialogueNode` test coverage | 9% | 80%+ | ✅ Done |
| C2 | `TelegramNode` test coverage | 0% | 50%+ | ✅ Done |
| C3 | MCP-инструменты (для `ToolProvider`) | — | 70%+ | ✅ Done |

### Этап D: Документация (как было, после P0)

| # | Задача | Исполнитель | Файл |
|---|---|---|---|
| D1 | **Harness quickstart** — как создать свой харнес | `techwriter` | `docs/guides/harness-quickstart.md` | ✅ Done |
| D2 | **Обновить SPEC_CURRENT** — пометить P0 как Done | `techwriter` | `SPEC_CURRENT.md` | ✅ Done |
| D3 | **Обновить ROADMAP** — стадии 2 / 5 | `techwriter` | `ROADMAP.md` | ✅ Done |

---

## 4. Ограничения (что нельзя менять)

- **Не трогать `main`** до завершения review PR #907 и ручного подтверждения
- **Не менять ADR-0001** — он уже прошёл полный цикл ревью (MADR, Accepted)
- **Не дублировать задачи** — все новые задачи создавать через GitHub Issues с соответствующими labels
- **API-ключи — только через env** (`MINIMAX_API_KEY`); YAML-литералы запрещены (ADR-0001 §2.5.3, M7)

---

## 5. Как работать с этим документом

Этот документ — **источник истины** для всех разработчиков на ближайшие задачи. Work is tracked via GitHub Issues. Use `gh issue create` with appropriate labels:

```bash
gh issue create --title "[ID] description" \
  --label "type:functional,priority:high,source:gsd" \
  --repo krikz/rob_box_project
```

Разработчик обязан:
1. Прочитать `SPEC_CURRENT.md` и `docs/adr/0001-harness-architecture.md`
2. Следовать ROADMAP (этап B — P1 харнесы)
3. Соблюдать ограничения (раздел 4)
4. При завершении закрыть issue с summary

---

## 6. Связанные документы

- [`docs/adr/0001-harness-architecture.md`](docs/adr/0001-harness-architecture.md) — ADR-0001 (MADR, Accepted)
- [`docs/adr/0002-minimax-provider.md`](docs/adr/0002-minimax-provider.md) — ADR-0002 (MiniMax LLM)
- [`docs/adr/0007-minimax-tts-integration-final.md`](docs/adr/0007-minimax-tts-integration-final.md) — ADR-0007 (MiniMax TTS)
- [`src/rob_box_harness/README.md`](src/rob_box_harness/README.md) — Harness Framework API
- [`src/rob_box_harness/rob_box_harness/providers/README.md`](src/rob_box_harness/rob_box_harness/providers/README.md) — MiniMax LLM provider API
- [`docs/guides/harness-quickstart.md`](docs/guides/harness-quickstart.md) — how-to: создать свой харнес
- [`docs/guides/MINIMAX.md`](docs/guides/MINIMAX.md) — MiniMax LLM user-guide
- [`docs/guides/MINIMAX_TTS_GETTING_STARTED.md`](docs/guides/MINIMAX_TTS_GETTING_STARTED.md) — MiniMax TTS getting-started
- [`docs/architecture/minimax-provider.md`](docs/architecture/minimax-provider.md) — архитектурный обзор MiniMax
