# Phase 4: GitHub Issues Integration — Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2026-05-15
**Phase:** 4-github-integration
**Areas discussed:** Scope обновления скиллов, Глубина нового скилла, Автоматизация миграции, TECH_DEBT дедупликация

---

## Area 1: Scope обновления скиллов

| Вариант | Описание | Выбран |
|---------|----------|--------|
| Хирургические замены | Только заменить строки с tasks.json в 2 файлах | |
| Full grep + deep update | Полный скан всех скиллов, deep rewrite narrative | ✓ |
| Только context-engineering | Обновить только главный скилл | |

**Выбор пользователя:** Full grep scan всех `.agents/skills/` + deep update (переписать narrative) найденных файлов.  
**Примечания:** grep подтвердил только 2 файла с tasks.json — `context-engineering/SKILL.md` (3 места) и `copilot-instructions.md` (1 место).

---

## Area 2: Глубина нового скилла github-issues-workflow

| Вариант | Описание | Выбран |
|---------|----------|--------|
| Cheatsheet (~50 строк) | Только таблица команд gh | |
| Lifecycle skill (~150 строк) | Полный процесс + пример цикла | ✓ |
| Comprehensive guide (200+) | Всё: troubleshooting, edge cases | |

**Выбор пользователя:** Lifecycle skill (~150 строк) со всеми секциями цикла и законченным примером.  
**Примечания:** Включить инструкции для создания новых Issues в процессе работы (нашёл баг → `gh issue create`).

---

## Area 3: Автоматизация миграции

| Вариант | Описание | Выбран |
|---------|----------|--------|
| Ручная | Создавать Issues вручную по списку | |
| Python-скрипт с state file | Скрипт + TASK_MAPPING.json | |
| Python-скрипт с gh search | Idempotency через `gh issue list --search` | ✓ |

**Выбор пользователя:** Python-скрипт `scripts/migrate_to_github.py`, idempotency через `gh issue list --search "TASK-XXX in:title"`. Удалить после выполнения.  
**Примечания:** Мигрировать все 20 задач (включая done → закрытые Issues). ~24 уникальных пункта из TECH_DEBT.md.

---

## Area 4: TECH_DEBT дедупликация

| Вариант | Описание | Выбран |
|---------|----------|--------|
| Fuzzy matching по title | Сравнивать названия, пропускать похожие | |
| Только task_ref | Дедупликация только через явное поле task_ref | ✓ |
| Мигрировать всё | Создать Issues для всех 30 пунктов | |

**Выбор пользователя:** Критерий дедупликации = только поле `task_ref`. Пункты с `disposition=accept` (6 шт.) пропустить.  
**Примечания:** BG-5/BG-6 имеют task_ref (уже будут из tasks.json). Fuzzy matching отклонён как слишком рискованный.

---

## Деferred Ideas

- GitHub Projects board — явно отклонён
- Fuzzy matching по title — отклонён (риск потери данных)
- GitHub Actions для автосоздания Issues — другая фаза
- TASK_MAPPING.json как state-файл — отклонён, используем gh search
