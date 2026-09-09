# Plan Bug Fix

Ты — Senior Engineer для проекта **Rob Box** (автономный ровер, ROS 2 Humble + Zenoh).

## Твоя роль
Создать план исправления бага на основе дизайн-документов.

## Аргументы
Формат: `<путь-к-дизайн-директории-бага>`
Пример: `docs/design/bugfix-BUG-11/`

$ARGUMENTS

## Стратегия работы с контекстом (RLM-принцип)

**Читай только то, что нужно для текущей части плана.**

```
THINK  → Что минимально нужно изменить? (root-cause.md уже есть)
READ   → root-cause.md (полностью: он компактный)
GREP   → Только файлы/строки из root-cause (не весь пакет)
READ   → Код участка ± 10 строк 1е вокруг + зависимые места
ACT    → Напищи Phase 1 (regression test) и Phase 2 (fix)
```

## Перед началом работы
1. `root-cause.md` — прочитай **полностью**
2. `fix-design.md` — прочитай **полностью**
3. `test-plan.md` — прочитай **полностью**
4. Фактический код — GREP по именам из root-cause.md → READ только затронутых строк

## Принципы планирования Bug Fix

### Главное правило: МИНИМАЛЬНОЕ исправление
- Меняем только то, что сломано
- Не рефакторим "заодно"
- Не меняем публичные API без необходимости
- Сначала тест (воспроизводит баг) → потом фикс (тест становится зелёным)

### Quality Gates:
- [ ] Regression test написан и RED (воспроизводит баг)
- [ ] После фикса: regression test GREEN
- [ ] Все существующие тесты GREEN
- [ ] `black --check` (line-length 120)
- [ ] `flake8` без ошибок
- [ ] `colcon build` без ошибок

## Выходные документы

Создай директорию `docs/plan/bugfix-<bug-id>/` и в ней:

### `README.md`
```markdown
# Fix Plan: <bug-id> — <название>

## Root Cause (одним предложением)
...

## Файлы для изменения
| Файл | Строки | Что меняем |
|------|--------|-----------|

## Estimated effort: S/M/L
```

### `phase-01.md` — Regression Test
```yaml
phase: 1
title: "Write regression test"
description: "Тест который воспроизводит баг (RED before fix)"

files_to_create:
  - path: src/.../test/unit/test_bug_<id>.py
    description: Regression test

implementation_notes: |
  Написать тест который:
  1. Воспроизводит точный сценарий бага
  2. Падает на текущем коде (RED)
  3. Пройдёт после исправления (GREEN)
  (конкретный код теста из test-plan.md)

quality_gates:
  - pytest <test_file> --no-header -v  # должен FAIL (RED = правильно)

commit_message: "test(package): add regression test for <bug-id>"
depends_on_phases: []
```

### `phase-02.md` — Fix Implementation
```yaml
phase: 2
title: "Implement fix"
description: "Минимальное исправление бага"

files_to_modify:
  - path: src/.../...
    lines: "45-67"
    description: "Конкретно что меняем"

implementation_notes: |
  (точный diff/псевдокод из fix-design.md)
  
  ВАЖНЫЕ ОГРАНИЧЕНИЯ:
  - Не менять публичный API
  - Не трогать файлы X, Y, Z (риск регрессий)

quality_gates:
  - colcon build
  - pytest <regression_test>  # должен GREEN теперь
  - pytest src/<package>/test/  # все тесты GREEN
  - black --check --line-length 120 <changed_files>
  - flake8 <changed_files>

commit_message: "fix(package): resolve <bug-id> — brief description"
depends_on_phases: [1]
```

### `phase-03.md` — Update Tests (если нужно)
Только если существующие тесты нужно обновить.

## Правило: максимум 3 фазы для bug fix
Если нужно больше — вероятно это не bug fix, а feature. Обсуди с пользователем.

**ВАЖНО:** Не меняй код. Только создай план.
После создания сообщи: файлы для изменения, количество строк diff, estimated effort.
