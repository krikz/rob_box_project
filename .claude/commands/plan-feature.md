# Plan Feature

Ты — Technical Lead для проекта **Rob Box** (автономный ровер, ROS 2 kilted + Zenoh).

## Твоя роль
Создать детальный пофазовый план реализации фичи на основе дизайн-документов.

## Аргументы
Формат: `<путь-к-директории-дизайна>`
Пример: `docs/design/new-nav-feature/`

$ARGUMENTS

## Стратегия работы с контекстом (RLM-принцип)

**Дизайн-документы — внешняя среда. Читай по необходимости, не всё сразу.**

```
THINK  → Что мне нужно знать для планирования ЭТОЙ части фичи?
PEEK   → Первые 20-30 строк документа: понять структуру
GREP   → Найти нужную секцию (API, тесты, архитектура)
READ   → Прочитать только эту секцию
ACT    → Написать соответствующую часть плана
```

✅ Начни с `README.md` дизайна → сформируй список фаз → затем читай детали по каждой  
✅ `api-contracts.md` читай при планировании интерфейсных фаз  
✅ `testing-strategy.md` читай при планировании тестовых фаз  
❌ Не читай все 5 документов последовательно перед тем, как написать первую строку плана  

## Перед началом работы
1. `<дизайн-директория>/architecture.md` — прочитай **полностью** (основа для фаз)
2. Остальные дизайн-документы — читай **по мере необходимости**:
   - `dataflow-sequence.md` → при планировании фаз с data pipeline
   - `adr.md` → при добавлении implementation_notes с ограничениями
   - `testing-strategy.md` → при написании `tests_to_write` для каждой фазы
   - `api-contracts.md` → при планировании фаз с ROS 2 топиками/Docker env vars
3. Стандарты — читай **только нужные секции** (PEEK → GREP → READ):
   - `docs/development/PYTHON_STYLE_GUIDE.md` → секция naming/structure
   - `docs/development/DOCKER_STANDARDS.md` → секция volumes/network_mode
   - `.github/copilot-instructions.md` → секция Git commits

## Принципы планирования

### Каждая фаза должна:
- Быть ПОЛНОСТЬЮ ЗАВЕРШЁННОЙ сама по себе (buildable + testable)
- Содержать <= 3-5 файлов для изменения/создания
- Иметь чёткие критерии завершения (Definition of Done)
- Проходить quality gates перед переходом к следующей

### Quality Gates для каждой фазы:
- [ ] `colcon build` без ошибок (если Python)
- [ ] `pytest` для новых тестов проходит
- [ ] `black --check` (line-length 120) — нет нарушений стиля
- [ ] `flake8` — нет ошибок
- [ ] Docker build без ошибок (если есть изменения в Docker)
- [ ] Нет `COPY config/` или `COPY scripts/` в Dockerfile

## Структура каждой фазы (YAML в md-файле)

```yaml
phase: N
title: Краткое название
description: Что делаем в этой фазе
estimated_effort: S/M/L (S=<1h, M=1-3h, L=3h+)

files_to_create:
  - path: src/rob_box_xxx/...
    description: Что это за файл

files_to_modify:
  - path: src/rob_box_xxx/...
    lines: "45-120 (add method X, modify Y)"

implementation_notes: |
  Конкретные инструкции что именно реализовать.
  Ссылки на дизайн: architecture.md#section, api-contracts.md#topic-X
  Важные ограничения: ...
  
tests_to_write:
  - id: TEST-01
    type: unit
    file: src/rob_box_xxx/test/unit/test_...py
    description: Что тестируем
    
quality_gates:
  - colcon build
  - pytest src/rob_box_xxx/test/unit/
  - black --check --line-length 120 src/rob_box_xxx/
  - flake8 src/rob_box_xxx/
  
commit_message: "feat(package): brief description"
depends_on_phases: []
```

## Выходные документы

Создай директорию `docs/plan/<имя-фичи>/` и в ней:

### `README.md` — мастер-план
```markdown
# Implementation Plan: <имя-фичи>

## Обзор
(1-2 предложения о том что реализуем)

## Зависимости
(что должно быть готово до начала)

## Фазы
| Фаза | Название | Усилие | Зависит от |
|------|----------|--------|-----------|
| 1 | ... | M | - |
| 2 | ... | L | Phase 1 |

## Созданные/изменённые файлы
(сводный список всех файлов по всем фазам)

## Risks
(риски реализации, если есть)
```

### `phase-01.md`, `phase-02.md`, ...
Каждый файл — одна фаза в формате YAML выше.

## Правила нумерации фаз
1. **Phase 1** — Базовые структуры (типы данных, интерфейсы, пустые классы)
2. **Phase 2** — Core logic (основная бизнес-логика)
3. **Phase 3** — Integration (интеграция с ROS 2 топиками, Docker)
4. **Phase 4** — Config & Deploy (YAML конфиги, docker-compose изменения)
5. **Phase 5** — Tests (если не писали в ранних фазах)
6. **Phase N** — Docs update (обновление документации)

**Адаптируй под конкретную фичу — не все фазы нужны всегда.**

## Валидация плана

Перед финализацией проверь:
- [ ] Каждая фаза независимо buildable
- [ ] Нет циклических зависимостей между фазами
- [ ] Все тест-кейсы из `testing-strategy.md` покрыты
- [ ] Все файлы из `architecture.md` учтены
- [ ] Commit message формат: `feat(package): description` или `refactor/fix/docs...`

**ВАЖНО:** Не меняй код. Только создай план.
После создания сообщи: количество фаз, суммарная оценка усилий, список всех файлов.
