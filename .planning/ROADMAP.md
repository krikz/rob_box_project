# Roadmap: Rob Box

## Overview

Трёхэтапный план подготовки кодовой базы к активной разработке: сначала документация приводится в соответствие с реальностью, затем проверяется структура Docker и пакетов, затем формируется полная картина технического долга. После завершения всех трёх фаз кодовая база готова к Milestone 2 — навигации и локализации.

## Phases

- [x] **Phase 1: Аудит документации** - Привести docs/ и README пакетов в соответствие с текущим состоянием проекта
- [x] **Phase 2: Ревью структуры** - Проверить Docker layout и пакеты на соответствие стандартам проекта
- [x] **Phase 3: Code Quality Review** - Разобрать tech debt, запустить статический анализ, задокументировать стратегию рефакторинга
- [ ] **Phase 4: GitHub Issues Integration** - Мигрировать tasks.json → GitHub Issues, настроить label taxonomy, обновить workflow агента

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

## Progress

| Phase | Plans Complete | Status | Completed |
|-------|----------------|--------|-----------|
| 1. Аудит документации | 3/3 | ✅ Complete | 2026-05-15 |
| 2. Ревью структуры | 3/3 | ✅ Complete | 2026-05-15 |
| 3. Code Quality Review | 5/5 | ✅ Complete | 2026-05-15 |
| 03.1. OpenAI vs Anthropic SDK Research | 1/1 | ✅ Complete | 2026-06-12 |
| 4. GitHub Issues Integration | 0/3 | Not started | - |
