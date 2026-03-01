````skill
---
name: context-engineering
description: >
  Методология разработки для Rob Box: Research → Design → Plan → Implement.
  Используй ВСЕГДА при начале работы над любой задачей из tasks.json.
  Заменяет «закинул задачу — написал код» на контролируемый многофазовый процесс.
---

# Context Engineering — Rob Box

## Суть подхода

Качество кода = f(корректность × полнота, размер × шум)

Проблема: кодовая база большая, контекстное окно ограничено.
Решение: каждая фаза сужает контекст и передаёт его следующей фазе.

```
tasks.json → Research → Design → Plan → Implement
   (задача)    (факты)  (архит.)  (фазы)  (команда агентов)
```

Только на фазе Implement пишется код. Всё остальное — подготовка контекста.

---

## Когда применять

| Тип работы | Минимально необходимые фазы |
|------------|----------------------------|
| Новая фича | Research → Design → Plan → Implement |
| Bug fix | Research → Design (bugfix) → Plan → Implement |
| Мелкий правка (<10 строк, очевидная) | Только Implement (без субагентов) |
| Исследование / вопрос | Только Research |

---

## Фаза 1: Research

**Команда:** `/research-codebase TASK-ID` или `/research-codebase <описание задачи>`

**Что происходит:**
- Lead агент запускает 4 параллельных субагента (архитектура / код / конфиги / тесты)
- Слабые модели (haiku/flash) для поиска файлов → экономия токенов
- Выходной документ: `docs/research/YYYY-MM-DD-<задача>-research.md`

**Содержит только факты:** какие файлы затронуты, какие интерфейсы существуют, что уже тестировано. **Ноль советов и мнений.**

**Критерий завершения:** ты можешь открыть документ и точно знаешь, какие файлы нужно изменить.

---

## Фаза 2: Design

**Команды:**
- Фича: `/design-feature <имя> <путь-к-research.md>`
- Баг: `/design-bugfix <bug-id> <путь-к-research.md>`

**Что создаётся в `docs/design/<имя>/`:**

| Документ | Содержание |
|----------|-----------|
| `architecture.md` | C4 диаграммы (Context/Containers/Components) в Mermaid |
| `dataflow-sequence.md` | Data Flow + Sequence диаграммы для каждого сценария |
| `adr.md` | Архитектурные решения: Решение → Почему → Альтернативы → Риски |
| `testing-strategy.md` | Тест-кейсы с ID (unit/integration/smoke) |
| `api-contracts.md` | ROS 2 топики/сервисы, Docker env vars, YAML параметры |

Для bugfix: `root-cause.md` + `fix-design.md` + `test-plan.md`

**❗ Ревью дизайна обязательно перед Plan.** Проверить архитектурные решения, исправить вручную если нужно.

**Критерий завершения:** team может прочитать и ответить на вопросы «из чего состоит?», «как работает?», «почему так?», «как тестируем?»

---

## Фаза 3: Plan

**Команды:**
- Фича: `/plan-feature <путь-к-design-директории>`
- Баг: `/plan-bugfix <путь-к-design-директории>`

**Что создаётся в `docs/plan/<имя>/`:**
- `README.md` — мастер-план: фазы, зависимости, сводный список файлов
- `phase-01.md`, `phase-02.md`, … — каждая фаза: список файлов, инструкции, quality gates, commit message

**Каждая фаза плана:**
- Независимo buildable и testable
- ≤ 3-5 файлов для изменения
- Чёткий Definition of Done
- Quality gates: `colcon build`, `pytest`, `black`, `flake8`

**❗ Ревью плана обязательно перед Implement.** Убедиться что нет ничего лишнего и ничего не пропущено.

**Критерий завершения:** любой инженер может взять phase-01.md и реализовать без вопросов.

---

## Фаза 4: Implement

**Команды:**
- Фича: `/implement-feature <путь-к-plan-директории>`
- Баг: `/implement-bugfix <путь-к-plan-директории>`

**Команда агентов (моб-программирование):**

| Агент | Роль | Не может |
|-------|------|----------|
| Backend Developer | Пишет код по плану | Делать commit, менять тесты |
| Code Reviewer | Запускает black/flake8/isort | Менять код |
| Security Reviewer | Ищет уязвимости | Менять код |
| Architecture Reviewer | Сверяет с дизайном | Менять код |
| Test Engineer | Пишет и запускает тесты | Менять production код |

**Цикл для каждой фазы плана:**
```
Backend Dev реализует → Ревьюеры (параллельно) →
если FAIL → Backend Dev правит (max 3 итерации) →
Test Engineer запускает quality gates →
git commit (без push) → следующая фаза
```

**Quality gates для каждой фазы:**
- [ ] `colcon build` без ошибок
- [ ] `pytest` новые тесты GREEN
- [ ] `black --check --line-length 120`
- [ ] `flake8` без ошибок
- [ ] `docker build` без ошибок (если Docker изменения)
- [ ] Нет `COPY config/` или `COPY scripts/` в Dockerfile

**После всех фаз:**
- Провести ревью PR
- `git push` + GitHub Actions CI
- Деплой через `docs/DEPLOYMENT_WORKFLOW.md`

---

## Разница: фича vs баг

| | Фича | Bug Fix |
|--|------|--------|
| Design | C4 + Data Flow + Sequence + ADR | Root Cause + Fix Design |
| Plan | Несколько фаз (структура → логика → интеграция) | Максимум 3 фазы |
| Implement | Backend Dev создаёт новое | Backend Dev меняет минимально |
| Тест Phase 1 | Happy path тесты | Regression test (RED) |
| Тест после impl | Green suite | RED→GREEN + full suite GREEN |

---

## Правила для агентов

```
✅ Каждая фаза — новое контекстное окно
✅ Research только факты, нулевые советы  
✅ Design ревьюируется человеком перед Plan
✅ Plan ревьюируется человеком перед Implement
✅ Commit после каждой фазы (не делать push самостоятельно)
✅ НИКОГДА Co-authored-by в commit messages
✅ НИКОГДА git push без явной просьбы пользователя
✅ НИКОГДА деплой на роботов самостоятельно
```

---

## Связанные ресурсы

| Ресурс | Назначение |
|--------|-----------|
| `tasks.json` | Бэклог задач с acceptance_criteria и dependencies |
| `PRD.md` | Продуктовые требования и архитектурный контекст |
| `docs/development/PYTHON_STYLE_GUIDE.md` | Стандарты Python для всех фаз |
| `docs/development/DOCKER_STANDARDS.md` | Стандарты Docker для Design/Plan/Implement |
| `docs/architecture/SYSTEM_OVERVIEW.md` | Системная архитектура для Design фазы |
| `docs/DEPLOYMENT_WORKFLOW.md` | Деплой после завершения Implement |
````
