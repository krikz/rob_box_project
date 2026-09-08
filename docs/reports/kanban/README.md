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

## Naming

`docs/reports/kanban/t_<task_id>.md`, где `<task_id>` — `t_<hex>6+` (например `t_84434d4c`).

Примеры из реальных карточек (появятся по мере dogfooding):
- `docs/reports/kanban/t_84434d4c.md` — отчёт карточки, которая ввела этот контракт.

## Связанные

- ADR-0077 — решение и trade-offs.
- `scripts/agent_flow/kanban-report-write.sh` — helper-скрипт.
- `scripts/agent_flow/tests/test_kanban_report_write.sh` — регресс-тест.
- AGENTS.md секция «Контракт отчёта воркера».