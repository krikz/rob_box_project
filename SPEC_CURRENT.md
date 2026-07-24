# SPEC_CURRENT — Текущее состояние и ближайшие шаги

> **Версия**: 1.0  
> **Дата**: 2026-07-24  
> **Ветка**: `feature/harness-p0-foundation`  
> **PR**: https://github.com/krikz/rob_box_project/pull/907  

---

## 1. Где мы сейчас

### 1.1 Завершённые этапы

| Этап | Результат | Статус |
|---|---|---|
| **ADR-0001** — Архитектура харнесов | 523 строки: порты, адаптеры, state-store, lifecycle hooks для DialogueNode / PersistentNode / TelegramNode | ✅ Proposed |
| **ADR-0002–0008** — MiniMax TTS интеграция | Провайдер, архитектура, контракты, pydantic-конфиг, extension points | ✅ Landed |
| **PR #907** — Код ревью | Все частные ревью (backend, frontend, security, architect) проведены | ✅ Done |

### 1.2 Текущий затык

**PR #907 ждёт сводного ревью и публикации review state.** Частные ревью завершены, но:

- `t_7d5bafa7` (pr-reviewer) — **blocked** (`needs_input`): требуется публикация сводного комментария в PR
- `t_fae576f4` (pr-reviewer) — **todo**: выставить финальный review state (APPROVE / REQUEST_CHANGES)
- `t_c06ff15b` (default) — **todo**: родительская задача ревью

### 1.3 Что уже настроено

- **18 профилей** Hermes с `backend: local`, `git:` блоком, `GITHUB_TOKEN`
- **Kanban-доска** `robbox` привязана к `/home/builder/hermes-share/rob_box_project`
- **Worktrees** почищены: остались только `t_7d5bafa7` и `t_c06ff15b`
- **Ветка** `feature/harness-p0-foundation` синхронизирована с origin

---

## 2. Что нужно сделать (ROADMAP — ближайшие шаги)

### Этап A: Завершить PR #907 review

| # | Задача | Исполнитель | Критерий готовности |
|---|---|---|---|
| A1 | Unblock `t_7d5bafa7` | Человек | Задача в `ready` |
| A2 | Опубликовать сводный комментарий | `pr-reviewer` | Комментарий виден в PR #907 |
| A3 | Выставить review state (APPROVE / REQUEST_CHANGES) | `pr-reviewer` | Review state установлен |
| A4 | Проверить результат | Человек | Все комментарии на месте |

### Этап B: Смерджить в main

| # | Задача | Исполнитель | Критерий |
|---|---|---|---|
| B1 | `git checkout main && git merge feature/harness-p0-foundation` | Человек | Fast-forward или разрешить конфликты |
| B2 | `git push origin main` | Человек | main обновлён |

### Этап C: Дальнейшее развитие (после merge)

| # | Направление | Контекст |
|---|---|---|
| C1 | **Реализация ADR-0001** — рефакторинг нод | Создать задачи на implementation харнесов |
| C2 | **Покрытие тестами** — DialogueNode (9% → 80%+) | ADR-0001, раздел 5 |
| C3 | **Telegram ↔ Voice мост** | ADR-0001, проблема 3 |
| C4 | **MiniMax TTS в проде** | ADR-0007c, runtime operations |

---

## 3. Ограничения (что нельзя менять)

- **Не трогать `main`** до завершения review PR #907 и ручного подтверждения
- **Не менять ADR-0001** — он уже прошёл полный цикл ревью
- **Не дублировать задачи** — все новые задачи создавать через kanban, не вручную
- **Воркеры работают в своих worktree** — не коммитить напрямую в `feature/harness-p0-foundation`

---

## 4. Как работать с этим документом

Этот документ — **источник истины** для всех воркеров на ближайшие задачи. При создании новой kanban-задачи:

```bash
hermes kanban create "Заголовок задачи" \
  --assignee <профиль> \
  --workspace worktree \
  --body "Прочитай SPEC_CURRENT.md — это источник истины. Твоя задача: ..."
```

Воркер обязан:
1. Прочитать `SPEC_CURRENT.md` и `docs/adr/0001-harness-architecture.md`
2. Следовать ROADMAP (раздел 2)
3. Соблюдать ограничения (раздел 3)
4. При завершении вызвать `kanban_complete` с summary
