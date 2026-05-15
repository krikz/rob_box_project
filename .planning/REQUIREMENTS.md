# Requirements: Rob Box — Milestone 1 (Качество кодовой базы)

**Defined:** 2026-05-15  
**Core Value:** Документация, структура и код отражают реальное состояние проекта и готовы к навигационному Milestone 2.

---

## v1 Requirements

### Документация (DOCS)

- [ ] **DOCS-01**: Все файлы в `docs/` актуальны — устаревшие разделы удалены или помечены
- [ ] **DOCS-02**: `README.md` корректно описывает текущее состояние проекта (железо, стек, запуск)
- [ ] **DOCS-03**: `docs/architecture/` отражает реальную Docker-топологию (Main Pi / Vision Pi сервисы)
- [ ] **DOCS-04**: `ROADMAP.md` синхронизирован с фактическим состоянием реализованных фич
- [ ] **DOCS-05**: Каждый ROS 2 пакет в `src/` имеет актуальный README с описанием нод, топиков, параметров
- [ ] **DOCS-06**: `docs/guides/` содержит актуальные гайды по запуску, деплою и разработке (QUICK_START, TROUBLESHOOTING, VISION_PI_SETUP, MONITORING_QUICK_REF)

### Структура проекта (STRUCT)

- [ ] **STRUCT-01**: Все Docker-сервисы следуют единому layout (`config/<service>/`, `scripts/<service>/`, volumes)
- [ ] **STRUCT-02**: Нет `COPY config/` или `COPY scripts/` в Dockerfile-ах (проверка соответствия стандарту)
- [ ] **STRUCT-03**: `setup.py` / `package.xml` во всех пакетах заполнены корректно (нет `TODO:`)
- [ ] **STRUCT-04**: Все зависимости в `package.xml` актуальны и соответствуют используемым импортам
- [ ] **STRUCT-05**: Структура `src/` пакетов соответствует соглашениям проекта (именование, layout)

### Code Quality Review (CQ)

- [ ] **CQ-01**: Все известные критические баги (CONCERNS.md) задокументированы в трекере с приоритетами
- [ ] **CQ-02**: Tech debt из CONCERNS.md разобран: каждый пункт имеет severity и план (fix / defer / accept)
- [ ] **CQ-03**: Проведён статический анализ (flake8, black, isort) — отчёт с количеством нарушений
- [ ] **CQ-04**: Покрытие тестами задокументировано: какие модули < 50% и почему
- [ ] **CQ-05**: `dialogue_node.py` (2040 строк, 13.4% coverage) — задокументирована стратегия рефакторинга для Milestone 3
- [ ] **CQ-06**: Stub-реализации (command_node.py, MCP get_robot_status) помечены `# STUB:` и добавлены в трекер

---

## v2 Requirements (Milestone 2 — Навигация)

- AprilTag initial pose после перезапуска/перемещения
- OAK-D depth → Nav2 3D voxel costmap (решение проблемы дивана)
- Интеграция потолочной камеры как основного локализатора

## v3 Requirements (Позже)

- AI HAT+ (Hailo-8L) object detection для Nav2
- dialogue_node рефакторинг (извлечение agent loop, history, modes)
- Voice commands → реальные навигационные действия (не stubs)
- Face recognition, Web UI, бизнес-логика доставки

---

## Out of Scope (Milestone 1)

| Feature | Reason |
|---------|--------|
| Исправление багов кода | Milestone 1 = аудит и документирование, не фикс |
| AI HAT+ интеграция | Hardware не закуплено |
| Navigation improvements | Milestone 2 |
| dialogue_node рефакторинг | Milestone 3, не блокирует навигацию |

---

## Traceability

| Requirement | Phase | Status |
|-------------|-------|--------|
| DOCS-01..06 | Phase 1 (Документация) | Pending |
| STRUCT-01..05 | Phase 2 (Структура) | Pending |
| CQ-01..06 | Phase 3 (Code Review) | Pending |

**Coverage:**
- v1 requirements: 17 total
- Mapped to phases: 17
- Unmapped: 0

---
*Requirements defined: 2026-05-15*
