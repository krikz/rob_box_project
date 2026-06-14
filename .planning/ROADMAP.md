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

### Phase 03.2: Music Quality Testing & Evaluation (INSERTED)

**Goal:** Fix prompt issues found in MiMo DJ log analysis, deploy to robot, test music generation quality (DJ set + gangster rap + non-DJ music), evaluate and iterate
**Requirements**: MUS-01, MUS-02, MUS-03, MUS-04, MUS-05
**Depends on:** Phase 03.1.1
**Issue:** #860
**Branch:** `feature/phase-3.2-music-testing`
**Success Criteria** (what must be TRUE):
  1. `search_samples` called at least once per DJ set when spack=1 used
  2. MC phrases ≤ 2 per track transition
  3. `set_dj_mode(enabled=True)` called after first `execute_music_code` in DJ mode
  4. spack=1 letters always verified via search_samples (no guessing)
  5. RENARDO_REFERENCE.md has sample packs section with spack=0/spack=1 letters
  6. 3 live tests completed on robot (DJ set, rap, freestyle)
  7. User Acceptance Testing scored ≥ 70% weighted average
  8. Testing report written with before/after scores and lessons learned
  9. ✅ "done" не произносится голосом после DJ set (только как plain text terminator)
  10. ✅ Ганкстер рэп не читается дважды (музыка + речь → стоп, без повтора)

**Plans:** 7 plans in 4 waves

Plans:
- [x] 03.2-01: RENARDO_REFERENCE.md — add spack=1/sample packs section (ALREADY DONE)
- [ ] 03.2-01-PLAN.md — Remove Star Wars bias, restructure melody references + strengthen search_samples/set_dj_mode rules (Wave 1)
- [ ] 03.2-02-PLAN.md — Fix compositor_prompt.txt (MC limits, "done" fix, rap double-speak) + dialogue_node.py (done filter, retry, frequency_penalty) (Wave 1)
- [ ] 03.2-03-PLAN.md — Melody Library: 6+ presets in TrackLibrary, type column, sample packs docs (Wave 2)
- [ ] 03.2-04-PLAN.md — Deploy to Vision Pi + Live Test A (DJ set) + Test B (gangster rap) (Wave 3)
- [ ] 03.2-05-PLAN.md — Live Test C (freestyle) + UAT scoring table (Wave 3)
- [ ] 03.2-06-PLAN.md — Iterate: fix top 3 UAT issues, re-deploy, re-test (Wave 4, conditional)
- [ ] 03.2-07-PLAN.md — Final Report: before/after scores, lessons learned (Wave 4)

### Phase 03.3: MiMo-Code Patterns Adoption: LLM Resilience Layer

**Goal:** Port 4 battle-tested patterns from Xiaomi MiMo-Code repo into dialogue_node.py to make the voice assistant resilient to LLM failures: empty responses, context overflow, repeated steps, and aggressive history truncation
**Requirements**: LLM-01, LLM-02, LLM-03, LLM-04
**Depends on:** Phase 03.2
**Branch:** `feature/phase-3.3-llm-resilience`
**Success Criteria** (what must be TRUE):
  1. `_classify_response()` correctly identifies: final, continue, think-only, invalid, failed — tested with 5+ mock scenarios
  2. Context pressure level (0-3) calculated before each LLM call; level ≥ 2 triggers memory-flush system-reminder injection
  3. Repeated-step detection: when LLM calls same tool+args ≥ 3 times, a `<system-reminder>` nudge is injected BEFORE doom-loop hard block
  4. `_truncate_history_outputs()` uses head+tail preservation (1500+1500 chars) instead of flat 200-char truncation
  5. Empty response recovery: think-only/invalid responses get one retry with nudge before fallback speech
  6. All changes work for ANY LLM provider (not MiMo-specific) — no provider-specific branches
  7. No regression: existing doom-loop protection, speak_text dedup, DJ mode all still work
  8. Log output clearly shows pressure level and classification for debugging
**Plans:** 4 plans in 2 waves

Plans:
- [ ] 03.3-01-PLAN.md — Port `_classify_response()` from classify.ts + integrate into `_agent_run()` result handling (Wave 1)
- [ ] 03.3-02-PLAN.md — Add `pressureLevel()` from overflow.ts + system-reminder injection for memory flush and repeated steps (Wave 1)
- [ ] 03.3-03-PLAN.md — Upgrade `_truncate_history_outputs()` to head+tail preservation (1500+1500) from prune.ts pattern (Wave 2)
- [ ] 03.3-04-PLAN.md — Integration test: deploy to robot, verify all 4 patterns work in live DJ + voice session (Wave 2)

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
| 03.2. Music Quality Testing & Evaluation | 0/7 | 🔄 In Progress | - |
| 03.3. MiMo-Code LLM Resilience Layer | 0/4 | Not started | - |
| 4. GitHub Issues Integration | 0/3 | Not started | - |
