# Фаза 06: Harness P0 Finalization — конспект содержимого каталога

> Назначение: единый структурированный обзор всех файлов фазы. Основа для онбординга
> новых агентов и human-review перед стартом имплементации. Без кода — только ресёрч и конспект.
>
> Дата: 2026-07-28
> Источник: `.planning/phases/06-harness-p0-finalization/`

---

## 1. Полный список файлов с кратким описанием

### 1.1 Активные документы (v2 — node-replacement architecture)

| Файл | Размер / строк | Назначение (2-3 строки) |
|---|---|---|
| `06-CONTEXT.md` | 193 строки | Главный документ фазы v2 (REVISED 2026-07-28). Содержит `<domain>` (3 ноды → тонкие оболочки), `<decisions>` D-01…D-07, `<interfaces>` (порты и топики), `<risks>`, `<verification>`. Точка входа для любого агента. |
| `06-CONTEXT-v1-parallel.md` | 209 строк | Архивная v1 (2026-07-27): 22 атомарные волны под параллельные обёртки. Заменена v2 после фидбэка пользователя — сохранена как исторический snapshot решений про документацию, Docker, PR-стратегию. |
| `06-RESEARCH.md` | 178 строк | Детальный ресёрч архитектуры: production-ready порты vs адаптеры на перепись; deep-dive по dialogue_node (2181 строка) с маппингом секций в порты; deep-dive по telegram_node и 5 perception-нодам; test-strategy; risk assessment с митигациями. |
| `06-DISCUSSION-LOG.md` | 215 строк | Аудит-тред решений пользователя от 2026-07-27/28. Содержит free-text формулировки на русском и соответствующие решения D-01…D-08. Также v1-волновая структура и deferred ideas. |
| `06-VALIDATION.md` | 77 строк | Validation strategy: 22 требования (DOC-MERGE-01…LINT-22) → автоматизированные команды (grep, pytest, mypy); Wave 0 gaps checklist; sampling rate per task/wave/phase gate; feedback latency. |
| `PHASE-EXECUTION-PLAN.md` | 137 строк | Консолидированный план аналитика (задача `t_7cffa9de`): scope summary, task graph (3 группы × 12 волн), snapshot состояния на 2026-07-28 10:25 (W1 ✅, W2 ⚠ in-flight, W3-W12 not started), risk register R1-R5, рекомендованная декомпозиция на 11-12 child-карточек для kanban-create, коммуникационный протокол. |
| `06-01-PLAN.md` | 269 строк | **Plan 01** (выполнен ✅). 4 задачи: W1 DeepSeekProvider + MiMoProvider, W2 ToolRegistry (34 тула), W3 DialogCore + DSM, W4 MemoryStore extensions (waypoints/FAQ/EventProfile). Каждая задача с read_first/action/verify/done. |
| `06-01-SUMMARY.md` | 227 строк | **Plan 01 close-out.** Детальный: 391 test passed, 5 production commits (06dbd5a8, 43d0111d, 0b7b66c7, 900addaf, d8665a1c); 11 файлов модифицировано; 2 архитектурных отклонения (ROSMCPToolProvider не делегирует ToolRegistry; DialogCore hooks делегируют DSM); 2 verification gaps (mypy/async). |
| `06-02-PLAN.md` | 220 строк | **Plan 02** (выполнен ✅). 2 задачи: W5 rewrite dialogue_node.py (2181 → 357 строк) как thin shell композирующий DialogCore; W6 integration tests (13 tests). Содержит полный diff-план что остаётся/удаляется. |
| `06-02-SUMMARY.md` | 211 строк | **Plan 02 close-out.** W5 commit `2a0aee26`, W6 commit `1eec45df`, fix `f80cbeaf` (rclpy shim). LOC delta: -1824 строк (-83.6%). 13 integration tests pass; 24/24 в `test_dialogue_shell.py + test_dialogue_node.py`. WAKE_WORD-before-STT_RESULT gate. |
| `06-03-PLAN.md` | 234 строки | **Plan 03** (telegram). 3 задачи: W7 удалить LLM из telegram_node + handlers, W8 переписать как ≤100-строчный мост, W9 integration tests. Топик-контракт: telegram → `/voice/stt/result`, `/voice/dialogue/response` → telegram reply. |
| `06-04-PLAN.md` | 244 строки | **Plan 04** (perception). 3 задачи: W10 удалить LLM из context_aggregator + удалить reflection/startup_greeting/vision_stub, W11 создать `perception_bridge.py` (~200 строк) UART → `/sensors/data`, W12 integration tests. health_monitor сохранён. |

### 1.2 Архивные документы (v1 — parallel-wrappers architecture, deprecated)

Каталог `archive-v1/` содержит 22 PLAN/SUMMARY файла от v1-фазы (W1–W22), которая
была заменена 2026-07-28. Структура: 06-01…06-09 с парами PLAN.md/SUMMARY.md.
Содержание — старая 22-волновая декомпозиция (документация → Docker → DialogHarness →
PersistentHarness → TelegramHarness → порты → тесты → PR). **Не использовать как
источник истины для имплементации** — только для исторического контекста.

| Файл (v1) | Назначение (кратко) |
|---|---|
| `archive-v1/06-01-PLAN.md` (12 KB) + SUMMARY (3 KB) | v1 Plan 01 — документация (W1-W5) |
| `archive-v1/06-02-PLAN.md` (11 KB) + SUMMARY (3 KB) | v1 Plan 02 — Docker (W6-W7) |
| `archive-v1/06-03-PLAN.md` (12 KB) + SUMMARY (2 KB) | v1 Plan 03 — DialogHarness (W8-W9) |
| `archive-v1/06-04-PLAN.md` (14 KB) + SUMMARY (3 KB) | v1 Plan 04 — PersistentHarness (W10) |
| `archive-v1/06-05-PLAN.md` (18 KB) + SUMMARY (6 KB) | v1 Plan 05 — TelegramHarness (W11) |
| `archive-v1/06-06-PLAN.md` (16 KB) + SUMMARY (5 KB) | v1 Plan 06 — порты (W12-W13) |
| `archive-v1/06-07-PLAN.md` (15 KB) + SUMMARY (5 KB) | v1 Plan 07 — тесты (W14-W17) |
| `archive-v1/06-08-PLAN.md` (8 KB) + SUMMARY (8 KB) | v1 Plan 08 — PR + аудит (W18-W22) |
| `archive-v1/06-09-PLAN.md` (18 KB) | v1 Plan 09 (без SUMMARY — задача обрывается) |

### 1.3 Файлы, которые упоминались в задаче, но НЕ существуют

- `phase.md` — нет. Точка входа для фазы = `06-CONTEXT.md`.
- `plans/*.md` (подкаталог) — нет. Планы лежат плоско как `06-NN-PLAN.md`.

---

## 2. Извлечённый scope фазы

### 2.1 Phase boundary (что заменяем, что оставляем)

Фаза 6 v2 **финализирует** ветку `feature/harness-p0-foundation` (PR #907).
Коренное отличие от v1: **полная замена старых нод тонкими оболочками**, а не
«параллельные обёртки». Три ноды переписываются с нуля под harness-парадигму.

| Подсистема | Что сейчас | Что будет | Трансформация |
|---|---|---|---|
| **Dialogue** | `dialogue_node.py` 2181 строк + LLM-логика inline | `dialogue_node.py` ~300 строк shell + `DialogCore` | LLM-логика, тулзы, DSM, memory → harness-порты. ROS2 pub/sub → тонкая оболочка |
| **Telegram** | `telegram_node.py` 409 строк + `handlers/` 916 строк | `telegram_node.py` ~80 строк + handlers урезаны | Без своего LLM! Только python-telegram-bot ↔ ROS2 топики. VPN в контейнере остаётся |
| **Perception** | 5 нод, 3536 строк (`context_aggregator`, `reflection`, `startup_greeting`, `vision_stub`, `health_monitor`) | 1 нода `perception_bridge.py` ~200 строк | Без LLM! Без micro-ROS! UART → `/sensors/data`. Своя прошивка сенсор-борда (вне скоупа) |

### 2.2 Архитектурная диаграмма

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

### 2.3 Что переиспользуем vs что переписываем

**ПЕРЕИСПОЛЬЗУЕМ (порты правильные, не трогаем):**
`harness.py`, `config.py`, `lifecycle.py`, `providers/minimax.py` (605 строк, ADR-0001
M1-M10), `providers/dummy.py`, `providers/fake_llm.py`, `executors/ros_mcp.py`,
`executors/local.py`, `executors/mcp_bridge.py`, `memory.py` + `memory/sqlite_voice.py`,
`transport.py` + `transport/ros2_transport.py`, `tools.py`, `effects.py`, `registry.py`,
`runner.py`, `clock.py`, `errors.py`, `tts/minimax_tts.py` + `tts/registry.py`.

**ПЕРЕПИСЫВАЕМ / АДАПТИРУЕМ:**
`harnesses/dialog.py` (439 строк), `harnesses/telegram.py` (789 строк),
`harnesses/persistent.py` (387 строк), `core/dialogue_state_machine.py` (316 строк —
адаптировать).

### 2.4 Цели и Definition of Done

**Главная цель:** Ветка `feature/harness-p0-foundation` готова к мержу в `develop`
(мерж делает пользователь). Три ноды — тонкие ROS2-оболочки поверх harness-портов.
Все LLM-зависимости из telegram и perception удалены полностью.

**Definition of Done:**
1. `dialogue_node.py` ≤ 350 строк, shell-only, без `from openai` / `from agents` / `@function_tool`.
2. `telegram_node.py` ≤ 100 строк, мост telegram ↔ ROS2 топики, без `LLMChat`/`MCPBridge`.
3. `perception_bridge.py` ≤ 200 строк, UART → `/sensors/data`, без LLM-кода.
4. Все 4 порта (`LLMProvider`, `ToolProvider`, `MemoryStore`, `Transport`) — чистый Python, тестируются без ROS2.
5. Per-wave verification: `python3 -m pytest src/rob_box_harness/test -x -q` зелёный.
6. Final gate: `mypy --strict` + `black --check` + `isort --check` + `flake8`.
7. Working tree clean на каждой волне, отдельные коммиты на `feature/harness-p0-foundation`.
8. НЕ мержим в `main` (это делает пользователь после тестирования).

### 2.5 Вне скоупа

- Прошивка сенсор-борда (отдельный проект, не Python).
- Мерж PR #907 — пользователь, вручную.
- P1-фичи (capability-фильтрация, DialogHarness/PersistentHarness/TelegramHarness из ADR-0001 §2.7 — Фаза 7+).
- Docker / build-system updates для harness (DOCKER-06/DOCKER-07 в `06-VALIDATION.md`).
- AI HAT+ интеграция (hardware не закуплено).
- Multi-robot fleet координация (Milestone 3+).

---

## 3. Полный список задач / plans, упомянутых в фазе

### 3.1 Активная фаза v2: 4 плана (выполнение)

| Plan | Волны | Зависимости | Статус | Файл |
|---|---|---|---|---|
| **06-01** | W1 (DeepSeekProvider+MiMoProvider), W2 (ToolRegistry 34 тула), W3 (DialogCore+DSM), W3a (DSM timer), W4 (MemoryStore waypoints/FAQ/EventProfile) | — | ✅ **DONE** (5 commits: 06dbd5a8, 43d0111d, 0b7b66c7, 900addaf, d8665a1c) | [06-01-PLAN.md](06-01-PLAN.md) / [06-01-SUMMARY.md](06-01-SUMMARY.md) |
| **06-02** | W5 (dialogue_node shell rewrite 2181→357 строк), W6 (13 integration tests) | depends on 06-01 | ✅ **DONE** (commits: 2a0aee26, 1eec45df, fix f80cbeaf) | [06-02-PLAN.md](06-02-PLAN.md) / [06-02-SUMMARY.md](06-02-SUMMARY.md) |
| **06-03** | W7 (remove LLM from telegram), W8 (rewrite ≤100 строк мост), W9 (integration tests) | — (параллельно A) | ⚠ **W7+W8 сделаны вне плана** (commits 07dfc28a, b2ed9480 в `feature/t_*` worktrees), W9 (tests) — open | [06-03-PLAN.md](06-03-PLAN.md) |
| **06-04** | W10 (remove LLM from perception), W11 (create perception_bridge.py ≤200 строк), W12 (integration tests) | — (параллельно A и B) | ⏳ **NOT STARTED** | [06-04-PLAN.md](06-04-PLAN.md) |

### 3.2 Декомпозиция для дочерних kanban-карт (из `PHASE-EXECUTION-PLAN.md` §5)

Аналитик (задача `t_7cffa9de`) рекомендует следующие 12 child-карточек:

```
A-2   W2  ToolRegistry                     → backend, depends on nothing
A-3   W3  DialogCore + DSM integration     → backend, block on A-2
A-4   W4  MemoryStore extensions           → backend, block on nothing
A-5   W5  dialogue_node shell rewrite      → backend, block on A-2, A-3, A-4
A-6   W6  dialogue shell tests             → backend, block on A-5
B-7   W7  Remove LLM from telegram         → backend, depends on nothing
B-8   W8  telegram ROS2 bridge             → backend, block on B-7
B-9   W9  telegram bridge tests            → backend, block on B-8
C-10  W10 Remove LLM from perception       → backend, depends on nothing
C-11  W11 perception_bridge.py             → backend, block on C-10
C-12  W12 perception bridge tests          → backend, block on C-11
ENV-MYPY-21  fix harness pytest.ini asyncio_mode=auto  → backend, parallel
```

> Примечание: Фактическое состояние отличается от этого предложения (W1-W6 уже
> завершены дочерними карточками в worktrees `t_*`; W7-W8 сделаны, W9-W12 — open).

### 3.3 Verification gates (per `06-VALIDATION.md`)

| Gate | Команда | Что проверяет |
|---|---|---|
| Per-wave | `python3 -m pytest src/rob_box_harness/test -x -q` | Зелёный baseline 391 test pass |
| Per-group final | `pytest src/rob_box_harness/test tests/unit/harness --cov=rob_box_harness --cov-fail-under=85` | Полный suite + coverage ≥85% |
| Phase gate | `mypy --strict src/rob_box_harness/rob_box_harness/providers/deepseek.py src/rob_box_harness/rob_box_harness/providers/mimo.py` | mypy clean |
| Phase gate | `black --check && isort --check && flake8` | Linters clean |
| W6 specific | `pytest src/rob_box_voice/test/ -k "dialogue"` | dialogue tests с новой оболочкой |
| W9 specific | `pytest src/rob_box_telegram/test/ -v` | telegram bridge |
| W12 specific | `pytest src/rob_box_perception/test/ -v` | perception bridge |

### 3.4 Известные риски (R1-R5 из `PHASE-EXECUTION-PLAN.md` §4)

- **R1** Multi-package refactor ~6000 LOC может превысить time budget одного worker.
  Митигация: декомпозиция на 11 child-tasks, каждая ≤2 часа.
- **R2** `pytest-asyncio mode=auto` не настроен в harness `pytest.ini` → 28 async тестов
  падают pre-existing. Митигация: отдельная ENV-MYPY-21 задача или `@pytest.mark.asyncio`.
- **R3** GitHub push требует credentials, которых нет в docker-контейнере.
  Митигация: push с host-машины, orchestrator проверяет remote HEAD.
- **R4** Конкурентные writer'ы на `feature/harness-p0-foundation` → push-конфликты.
  Митигация: каждый child-task rebase на `origin/feature/harness-p0-foundation` перед push.
- **R5** W2 test файл существует без source → orphan test. Митигация: в acceptance W2
  включено «create source + verify import + run tests».

### 3.5 Communication protocol (per `PHASE-EXECUTION-PLAN.md` §8)

- Parent task `t_7cffa9de` остаётся в `running` пока все 11 child-волн не достигнут `done`.
- Каждый child обновляет `.planning/phases/06-harness-p0-finalization/progress.md`
  (wave id, commit SHA, push result, test summary).
- На любой failure требующей human input → `kanban_block(kind='needs_input')`.

---

## 4. Точка входа для следующего агента

1. Прочитай [`06-CONTEXT.md`](06-CONTEXT.md) — главный документ фазы.
2. Прочитай [`06-RESEARCH.md`](06-RESEARCH.md) — архитектурный deep-dive.
3. Прочитай [`PHASE-EXECUTION-PLAN.md`](PHASE-EXECUTION-PLAN.md) — текущий план аналитика.
4. Прочитай [`06-01-SUMMARY.md`](06-01-SUMMARY.md) + [`06-02-SUMMARY.md`](06-02-SUMMARY.md)
   — что уже сделано и какие отклонения от плана зафиксированы.
5. Прочитай [`06-03-PLAN.md`](06-03-PLAN.md) или [`06-04-PLAN.md`](06-04-PLAN.md)
   — задача для имплементации.
6. Сверься с [`06-VALIDATION.md`](06-VALIDATION.md) — verification-команды.

`06-DISCUSSION-LOG.md` и `archive-v1/` — только для исторического контекста,
не использовать как источник истины для имплементации.