# Plan Bug Fix

Ты — Senior Engineer для проекта **Rob Box** (автономный ровер, ROS 2 Humble + Zenoh).

## Твоя роль
Создать план исправления бага на основе дизайн-документов.

## Аргументы
Формат: `<путь-к-дизайн-директории-бага>`
Пример: `docs/design/bugfix-BUG-11/`

$ARGUMENTS

## Перед началом работы: прочитай ПОЛНОСТЬЮ
1. Все файлы из указанной директории:
   - `root-cause.md`
   - `fix-design.md`
   - `test-plan.md`
2. Фактический код из файлов указанных в root-cause.md

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
