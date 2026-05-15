# ROADMAP — Rob Box

**Current Milestone:** Milestone 1 — Качество кодовой базы  
**Next Milestone:** Milestone 2 — Навигация и локализация

---

## Milestone 1: Качество кодовой базы
> Цель: документация, структура и код готовы к активной разработке навигации

### Phase 1 — Аудит и обновление документации
**Goal:** Все docs/ файлы отражают реальное состояние проекта  
**Requirements:** DOCS-01, DOCS-02, DOCS-03, DOCS-04, DOCS-05, DOCS-06

**Plans:**
1. `docs-audit` — Аудит всех файлов docs/: найти устаревшее, задокументировать расхождения
2. `docs-update-architecture` — Обновить docs/architecture/ под текущую Docker-топологию
3. `docs-update-packages` — Обновить README каждого пакета в src/ (ноды, топики, параметры)
4. `docs-update-roadmap` — Синхронизировать ROADMAP.md с реальным состоянием фич

**UAT:**
- [ ] Новый разработчик может запустить робота только по документации (без дополнительных вопросов)
- [ ] `docs/architecture/` содержит актуальные диаграммы сервисов

---

### Phase 2 — Ревью структуры проекта
**Goal:** Docker layout и пакеты следуют единому стандарту  
**Requirements:** STRUCT-01, STRUCT-02, STRUCT-03, STRUCT-04, STRUCT-05

**Plans:**
1. `struct-docker-audit` — Проверить все Dockerfile-ы: нет COPY config/, COPY scripts/; volumes везде
2. `struct-packages-audit` — Проверить setup.py, package.xml всех пакетов: нет TODO, зависимости актуальны
3. `struct-fixes` — Устранить найденные нарушения стандартов (мелкие правки)

**UAT:**
- [ ] `grep -r "COPY config" docker/` возвращает 0 результатов
- [ ] `grep -r "TODO" src/*/setup.py` возвращает 0 результатов
- [ ] Все сервисы имеют `network_mode: host` и `depends_on: zenoh-router`

---

### Phase 3 — Code Quality Review
**Goal:** Все critical issues задокументированы, tech debt приоритизирован  
**Requirements:** CQ-01, CQ-02, CQ-03, CQ-04, CQ-05, CQ-06

**Plans:**
1. `cq-bugs-triage` — Разобрать CONCERNS.md: severity, план (fix/defer/accept), добавить в tasks.json
2. `cq-static-analysis` — Запустить flake8/black/isort, задокументировать отчёт
3. `cq-coverage-report` — Актуализировать отчёт о покрытии тестами, выявить модули < 50%
4. `cq-refactor-strategy` — Задокументировать стратегию рефакторинга dialogue_node.py для Milestone 3
5. `cq-stub-tagging` — Пометить все stub-реализации `# STUB:`, добавить в трекер

**UAT:**
- [ ] Каждый пункт CONCERNS.md имеет severity (critical/high/medium/low) и disposition
- [ ] Все stubs помечены `# STUB:` с ссылкой на задачу
- [ ] Отчёт flake8 задокументирован в .planning/

---

## Milestone 2: Навигация и локализация (следующий)
> Цель: робот знает своё положение после рестарта, объезжает 3D препятствия

**Planned Phases:**
- Phase 4 — AprilTag initial pose provider (потолочная камера → /initialpose)
- Phase 5 — OAK-D depth → Nav2 3D voxel costmap (решение проблемы дивана)
- Phase 6 — Интеграция и полевые испытания (большие помещения)

---

## Milestone 3: Голосовой ассистент v2 (будущее)
> Цель: стабильный LLM agent, голосовые команды реально двигают робота

**Planned Phases:**
- Phase 7 — dialogue_node рефакторинг (< 300 строк оркестратор)
- Phase 8 — Реализация stub-команд (position, objects, follow)
- Phase 9 — Voice → Navigation integration

---
*Last updated: 2026-05-15*
