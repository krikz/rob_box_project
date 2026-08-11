# Roadmap: Rob Box

## Overview

Трёхэтапный план подготовки кодовой базы к активной разработке: сначала документация приводится в соответствие с реальностью, затем проверяется структура Docker и пакетов, затем формируется полная картина технического долга. После завершения всех трёх фаз кодовая база готова к Milestone 2 — навигации и локализации.

## Phases

- [x] **Phase 1: Аудит документации** - Привести docs/ и README пакетов в соответствие с текущим состоянием проекта
- [x] **Phase 2: Ревью структуры** - Проверить Docker layout и пакеты на соответствие стандартам проекта
- [x] **Phase 3: Code Quality Review** - Разобрать tech debt, запустить статический анализ, задокументировать стратегию рефакторинга
- [ ] **Phase 4: GitHub Issues Integration** - Мигрировать tasks.json → GitHub Issues, настроить label taxonomy, обновить workflow агента
- [ ] **Phase 5: Миграция ROS 2 Humble → Lyrical Luth** - Перевести все Docker-сервисы на ROS 2 Lyrical (Ubuntu 26.04), включая решение блокеров Nav2 и DepthAI
- [~] **Phase 6: Harness P0 Finalization** - Финализация `feature/harness-p0-foundation`: реальная замена старых нод на harness-порты (Dialogue = мозг с LLM, Telegram = тонкий мост, Perception = UART-мост). Архитектура v2 «node replacement»; v1 «parallel wrappers» архивирован в `archive-v1/`

## Phase Details

### Phase 1: Аудит документации
**Goal**: Все файлы docs/ и README пакетов отражают реальное состояние проекта
**Depends on**: Nothing (first phase)
**Requirements**: DOCS-01, DOCS-02, DOCS-03, DOCS-04, DOCS-05, DOCS-06
**Success Criteria** (what must be TRUE):
  1. Новый разработчик может запустить робота только по документации (без дополнительных вопросов)
  2. docs/architecture/ содержит актуальную Docker-топологию (Main Pi / Vision Pi сервисы)
  3. Каждый пакет src/ имеет README с описанием нод, топиков и параметров
  4. README.md корректно описывает аппаратный стек и порядок запуска
**Plans**: TBD

Plans:
- [ ] 01-01: Аудит docs/ — найти устаревшее, задокументировать расхождения
- [ ] 01-02: Обновить docs/architecture/ под текущую Docker-топологию
- [ ] 01-03: Обновить README каждого пакета в src/ (ноды, топики, параметры)

### Phase 2: Ревью структуры
**Goal**: Docker layout и пакеты следуют единому стандарту проекта
**Depends on**: Phase 1
**Requirements**: STRUCT-01, STRUCT-02, STRUCT-03, STRUCT-04, STRUCT-05
**Success Criteria** (what must be TRUE):
  1. grep -r "COPY config" docker/ возвращает 0 результатов
  2. grep -r "COPY scripts" docker/ возвращает 0 результатов
  3. Все docker-сервисы имеют network_mode: host и depends_on: zenoh-router
  4. setup.py и package.xml всех пакетов не содержат TODO и имеют актуальные зависимости
**Plans**: TBD

Plans:
- [ ] 02-01: Аудит Dockerfile-ов — нет COPY config/, COPY scripts/; volumes везде
- [ ] 02-02: Аудит setup.py и package.xml — нет TODO, зависимости актуальны
- [ ] 02-03: Устранить найденные нарушения стандартов

### Phase 3: Code Quality Review
**Goal**: Все critical issues задокументированы, tech debt приоритизирован, стратегия рефакторинга определена
**Depends on**: Phase 2
**Requirements**: CQ-01, CQ-02, CQ-03, CQ-04, CQ-05, CQ-06
**Success Criteria** (what must be TRUE):
  1. Каждый пункт CONCERNS.md имеет severity (critical/high/medium/low) и disposition (fix/defer/accept)
  2. Отчёт flake8/black/isort с количеством нарушений задокументирован в .planning/
  3. Покрытие тестами задокументировано: модули < 50% выявлены и объяснены
  4. Все stub-реализации помечены # STUB: и добавлены в трекер
  5. dialogue_node.py имеет задокументированную стратегию рефакторинга для Milestone 3
**Plans**: TBD

Plans:
- [ ] 03-01: Разобрать CONCERNS.md — severity, disposition, добавить в tasks.json
- [ ] 03-02: Запустить flake8/black/isort, задокументировать отчёт
- [ ] 03-03: Актуализировать отчёт о покрытии тестами, выявить модули < 50%
- [ ] 03-04: Задокументировать стратегию рефакторинга dialogue_node.py
- [ ] 03-05: Пометить все stub-реализации # STUB:, добавить в трекер

### Phase 03.1: Research: OpenAI Agent SDK vs Anthropic Claude SDK - agentic features comparison (INSERTED)

**Goal:** Determine whether OpenAI Agents SDK or Anthropic Claude SDK better serves Rob Box agentic features, with DeepSeek API compatibility verification
**Requirements**: D-01, D-02, D-03, D-04, D-05, D-06, D-07, D-08, D-09, D-10, D-11, D-12
**Depends on:** Phase 3
**Decision:** Stay with OpenAI Agents SDK. Anthropic SDK skipped — Managed Agents is cloud-only, DeepSeek Anthropic API has critical gaps (no MCP, no images, no code execution). See 03.1-RESEARCH.md Section 7 for full decision matrix.
**Success Criteria** (what must be TRUE):
  1. RESEARCH.md covers all 12 D-XX decisions with DeepSeek compatibility verified for each
  2. All requirements.txt files have pinned openai>=2.37.0 and openai-agents>=0.17.0
  3. Decision recorded: stay with OpenAI Agents SDK, Anthropic SDK adds zero value for DeepSeek-only setup
**Plans:** 1 plan

Plans:
- [x] 03.1-01-PLAN.md — Validate research, pin versions, document decision

### Phase 03.1.1: Apply research findings: P0 error handling (APITimeoutError/RateLimitError/APIStatusError) + OPENAI_LOG debug in dialogue_node (INSERTED)

**Goal:** dialogue_node catches APITimeoutError, RateLimitError, APIStatusError with request_id logging; OPENAI_LOG=info enabled in voice-assistant container
**Requirements**: D-P0-1, D-P0-2, D-P0-3, D-P0-4, D-P1-1
**Depends on:** Phase 03.1
**Plans:** 1 plan

Plans:
- [ ] 03.1.1-01-PLAN.md — Add P0 error handling to _run_agent_with_retry() + OPENAI_LOG env var

### Phase 4: GitHub Issues Integration
**Goal**: GitHub Issues = единственный источник правды для задач/багов/tech-debt; tasks.json удалён; ИИ-агент работает через `gh` CLI
**Depends on**: Phase 3
**Requirements**: GH-01, GH-02, GH-03, GH-04, GH-05
**Success Criteria** (what must be TRUE):
  1. `gh issue list --label ai-generated` возвращает ≥20 issues
  2. `ls tasks.json` — файл не существует в репозитории
  3. `copilot-instructions.md` содержит инструкции по работе с `gh issue` вместо `tasks.json`
  4. Скилл `.agents/skills/github-issues-workflow/SKILL.md` создан и описывает полный цикл старт→ветка→PR→close
  5. Все `# STUB:` комментарии в коде содержат `#N` (номер GitHub Issue) вместо `TASK-05X`
**Plans**:
- [ ] 04-01: Обновить скиллы + copilot-instructions (GitHub Issues вместо tasks.json)
- [ ] 04-02: GitHub labels (17 шт.) + milestones (M1/M2/M3) через gh CLI
- [ ] 04-03: Migrate tasks.json + TECH_DEBT → Issues; STUB → #N; удалить tasks.json

### Phase 5: Миграция ROS 2 Humble → Lyrical Luth
**Goal**: Все Docker-сервисы робота работают на ROS 2 Lyrical Luth (Ubuntu 26.04 Resolute)
**Depends on**: Phase 4
**Requirements**: MIG-01, MIG-02, MIG-03, MIG-04, MIG-05
**Success Criteria** (what must be TRUE):
  1. Все Dockerfile'ы (кроме Nav2 и DepthAI) используют ros:lyrical-ros-base или lyrical-совместимые базовые образы
  2. CI/CD успешно собирает base-образы (ros2-zenoh, rtabmap, pcl) на lyrical
  3. Nav2 собран из исходников (ros-navigation/navigation2 main) и работает на роботе
  4. DepthAI собран из исходников (depthai-core main) и OAK-D камера публикует данные
  5. Все pip install используют --break-system-packages, python3-pip добавлен в зависимости
**Plans**: 2 волны (Wave 1 — механическая миграция, Wave 2 — Nav2/DepthAI source-build)

Plans:
- [ ] 05-01-PLAN.md — Wave 1: Механическая миграция (~200 файлов, всё кроме Nav2 и DepthAI)
- [ ] 05-02-PLAN.md — Wave 2: Nav2 source-build + DepthAI source-build (2 tasks: Nav2 Dockerfile source-build, DepthAI Dockerfile source-build + GitHub Issue)

### Phase 6: Harness P0 Finalization
**Goal**: Ветка `feature/harness-p0-foundation` готова к мержу в main: документация без дублей, все ADR-требования выполнены, оставшиеся изменения доделаны
**Depends on**: Phase 4 (инфраструктура GitHub Issues), ADR-0001, ADR-0002, ADR-0007, ADR-0009
**Branch**: `feature/harness-p0-foundation`
**PR**: https://github.com/krikz/rob_box_project/pull/907
**Canonical refs**:
  - [`SPEC_CURRENT.md`](SPEC_CURRENT.md) — источник истины для P0/P1
  - [`docs/adr/0001-harness-architecture.md`](docs/adr/0001-harness-architecture.md)
  - [`docs/adr/0002-minimax-provider.md`](docs/adr/0002-minimax-provider.md)
  - [`docs/adr/0007-minimax-tts-integration-final.md`](docs/adr/0007-minimax-tts-integration-final.md)
  - [`docs/adr/0009-harness-tts-contract.md`](docs/adr/0009-harness-tts-contract.md)
  - [`src/rob_box_harness/README.md`](src/rob_box_harness/README.md)
  - [`docs/guides/harness-quickstart.md`](docs/guides/harness-quickstart.md)
  - [`docs/guides/MINIMAX.md`](docs/guides/MINIMAX.md)
**Success Criteria** (what must be TRUE):
  1. `dialogue_node.py` — тонкая оболочка ~300 строк, вся LLM-логика в harness-портах
  2. `telegram_node.py` — тонкий ROS2-мост ~80 строк, без своего LLM
  3. `perception_*` — UART-мост ~200 строк, без LLM, собственная прошивка сенсор-борда
  4. Все harness-порты (LLMProvider, ToolProvider, MemoryStore) — pure Python, тестируются без ROS2
  5. ADR-0001 / ADR-0002 / ADR-0007 / ADR-0009 — соответствие, P1 deferred задокументирован
  6. SPEC_CURRENT.md обновлён — P0 помечен как завершённый
  7. Ветка готова к `git merge feature/harness-p0-foundation` → main
**Plans (v2)**: 4 plans in 4 waves (12 internal tasks W1–W12 distributed across the 4 plans)

> **Note:** Phase 6 v1 («parallel implementation», 9 plans in 5 waves) был заархивирован в `.planning/phases/06-harness-p0-finalization/archive-v1/` (commit `16913094`). v1's `feat(harness): W1..W4` commits сохранены и **составляют фактическую работу Plan 06-01 v2** (DeepSeek+MiMo providers, ToolRegistry, DialogCore+DSM, MemoryStore).

Plans:
- [x] 06-01-PLAN.md — **Harness ports foundation** (Wave 1): DeepSeek+MiMo providers, ToolRegistry(34), DialogCore+DSM, MemoryStore waypoints/FAQ/EventProfile. Closed-out 2026-07-28 via safe-resume gate. Production commits: `06dbd5a8`, `43d0111d`, `0b7b66c7`, `900addaf`, `d8665a1c` (merged via `72bab30a`, `f324ae83`). Requirements: HARNESS-LLM-01, HARNESS-TOOL-02, HARNESS-DSM-03, HARNESS-MEMORY-04. Summary: `06-01-SUMMARY.md`.
- [x] 06-02-PLAN.md — **Dialogue shell rewrite** (Wave 2, depends on 06-01): `dialogue_node.py` 2181 → 357 строк оболочка, композирующая DialogCore + 13 integration tests. Closed-out 2026-07-28 via safe-resume gate. Production commits: W5 `2a0aee26` (shell rewrite), W6 `1eec45df` (integration tests); post-merge fix `f80cbeaf` (rclpy shim unconditional); SUMMARY `6245e064`. Merges: `18ff45ce`, `2f8335f5`. Requirements: DIALOG-SHELL-05, DIALOG-TEST-06. Summary: `06-02-SUMMARY.md`. 13/13 integration tests PASS in 0.64s, no regression in existing tests (24/24 PASS in 0.72s).
- [x] 06-03-PLAN.md — **Telegram bridge** (Wave 3, depends on 06-01+06-02): `telegram_node.py` 409 → 99 строк pure ROS2 bridge. Closed-out 2026-07-28 via safe-resume gate. Production commits: W7 `07dfc28a` (LLM removal), W8 `b2ed9480` (bridge rewrite), W9 `493a2791` (integration tests); merges `2f8335f5`, `73eba425`, `8c65c364`. `llm_chat.py` (469 LOC) + `mcp_bridge.py` (304 LOC) deleted; lives in `rob_box_harness` now. 20/20 tests PASS, 1 VPN skipped per spec; no regression in Plans 06-02 (13/13) or 06-04 (13/13). Requirements: TG-LLM-REMOVE-07, TG-BRIDGE-08, TG-TEST-09. Summary: `06-03-SUMMARY.md`.
- [x] 06-04-PLAN.md — **Perception bridge** (Wave 4, depends on 06-01+06-02): 5 perception нод → 1 `perception_bridge.py` 198 строк UART-мост. Closed-out 2026-07-28 via safe-resume gate. Production commits: W10 `7552418a` (LLM removal + node deletion), W11 `85cfd62e` (perception_bridge.py creation), W12 `1200304c` (integration tests); merges `73eba425`, `762b4a83`. `context_aggregator_node.py` 745 → 523 LOC; 5 old files deleted (reflection_node 898 LOC, startup_greeting_node 181 LOC, vision_stub_node 160 LOC, prompt_formatter 180 LOC, long_term_memory 209 LOC). 13/13 integration tests PASS in 0.09s; no regression in Plans 06-02 (13/13) or 06-03 (20/20). Requirements: PERC-LLM-REMOVE-10, PERC-BRIDGE-11, PERC-TEST-12. Summary: `06-04-SUMMARY.md`.

## Progress

| Phase | Plans Complete | Status | Completed |
|-------|----------------|--------|-----------|
| 1. Аудит документации | 3/3 | ✅ Complete | 2026-05-15 |
| 2. Ревью структуры | 3/3 | ✅ Complete | 2026-05-15 |
| 3. Code Quality Review | 5/5 | ✅ Complete | 2026-05-15 |
| 03.1. OpenAI vs Anthropic SDK Research | 1/1 | ✅ Complete | 2026-06-12 |
| 03.1.1. Apply Research (P0 error handling) | 0/1 | Not started | - |
| 4. GitHub Issues Integration | 0/3 | Not started | - |
| 5. Миграция Humble → Lyrical | 2/2 | Planned | Wave 1 + Wave 2 plans ready |
| 6. Harness P0 Finalization | 4/4 | ◆ In progress | 2026-07-28 (all 4 plans closed-out; phase verification pending) |