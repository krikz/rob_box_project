# docs/reports/kanban/

Директория отчётов воркеров agent-flow по kanban-карточкам.

## Зачем

`hermes_cli.kanban_tools.kanban_complete` сохраняет только `Result` (~300 символов), `Summary` (одно предложение) и `Artifacts` (список путей **без содержимого**). Через 30 дней worktree GC — и для ретро/аудита не остаётся ничего, кроме «done».

Эта директория — **git-tracked SOT** для полных отчётов воркеров. Каждый файл `t_<task_id>.md` создаётся воркером перед `kanban_complete` и переживает worktree GC.

## Контракт

См. ADR-0077 (`docs/adr/0077-kanban-worker-report-file.md`).

Минимальный контракт:

- Файл создаётся в worktree воркера **ДО** `kanban_complete`.
- Содержит: title, duration, изменённые файлы, git log, raw-evidence (pytest), CI run_id, PR link, замечания.
- Коммитится (`report(<task_id>): <title>`) и пушится в ту же ветку, что и PR.
- Скрипт-помощник: `scripts/agent_flow/kanban-report-write.sh` — вызывается воркером.

## Worker workflow (issue #2438, ADR-0077 §8)

Воркер делает rebase **дважды** за сессию, плюс отчёт:

```
[claim] → [pre_flight rebase] → [work] → [push] → [post_flight rebase] → [report] → [kanban_complete]
```

1. **После claim, в начале сессии:**
   ```bash
   bash scripts/agent_flow/worker_pre_flight.sh "$HERMES_KANBAN_TASK" \
       "$(git rev-parse --abbrev-ref HEAD)" "${ISSUE_NUM:-}"
   # exit 0 → OK; exit 1 → rebase conflict, ручной resolve
   ```

2. **Перед `kanban_complete`** (post_flight + отчёт одной командой):
   ```bash
   bash scripts/agent_flow/kanban-report-write.sh "$HERMES_KANBAN_TASK"
   # exit 1 → rebase conflict, разрешить и повторить
   ```

   `kanban-report-write.sh` внутри вызывает `worker_post_flight.sh` (auto-rebase на origin/develop + auto-push через `push-via-gh-api.sh`). Если post_flight возвращает exit 1 (rebase conflict) — отчёт НЕ пишется → воркер должен разрешить конфликт и повторить.

**Почему:** без pre/post-flight воркеры стартуют на устаревших worktree, PR diverged, merge-gate ловит add/add конфликты (ретро PR #2351 — 66 коммитов behind, PR #2363 — add/add конфликт).

**Opt-out** для retro-карточек: `SKIP_PRE_FLIGHT=true` / `SKIP_POST_FLIGHT=true`.

## Naming

`docs/reports/kanban/t_<task_id>.md`, где `<task_id>` — `t_<hex>6+` (например `t_84434d4c`).

Примеры из реальных карточек (появятся по мере dogfooding):
- `docs/reports/kanban/t_84434d4c.md` — отчёт карточки, которая ввела этот контракт.

## Связанные

- ADR-0077 — решение и trade-offs (включая §8 Pre/Post-flight rebase).
- `scripts/agent_flow/kanban-report-write.sh` — helper-скрипт (вызывает post_flight).
- `scripts/agent_flow/worker_pre_flight.sh` — pre-work auto-rebase.
- `scripts/agent_flow/worker_post_flight.sh` — post-work auto-rebase.
- `scripts/agent_flow/tests/test_kanban_report_write.sh` — регресс-тест.
- `scripts/agent_flow/tests/test_worker_pre_flight.sh` — регресс-тест.
- `scripts/agent_flow/tests/test_worker_post_flight.sh` — регресс-тест.
- Skill `bundled/worker-rebase-protocol.md` — упоминается в body новых карточек.