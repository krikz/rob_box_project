# Issue tracker: GitHub

Issues and specs for this repo live as GitHub issues in `krikz/rob_box_project`.
Use the `gh` CLI for all operations.

## Conventions

- **Create**: `gh issue create --title "..." --body "..."` (heredoc for multi-line bodies).
- **Read**: `gh issue view <number> --comments`.
- **List**: `gh issue list --state open --json number,title,body,labels,comments --jq '[.[] | {number, title, body, labels: [.labels[].name]}]'` with `--label` / `--state` filters.
- **Comment**: `gh issue comment <number> --body "..."`.
- **Labels**: `gh issue edit <number> --add-label "..."` / `--remove-label "..."`.
- **Close**: `gh issue close <number> --comment "..."`.

`gh` infers the repo from `git remote -v` when run inside a clone.

## Rob Box specifics (важно)

- **Триажа-автомата нет.** Issue разбирает координатор (процесс —
  `docs/process/claude-coordinator-robot-flow.md`, ADR-0162).
- **Создание задачи:** `gh issue create --title "..." --body-file <файл> --repo krikz/rob_box_project`;
  тело — raw-факты (лог, run_id, файл:строка). Метки — по смыслу (`bug`, `type:*`, `priority:*`).
- Метки `source:gsd`, `ai-generated`, `hermes`, `agent:*`, `needs-e2e`, `e2e-done`
  новым issue **не ставить** — их некому читать (`AGENTS.md`).

## Pull requests as a triage surface

**PRs as a request surface: no.** PR воркеров принимает и мёржит координатор
после CI (`AGENTS.md`, «Процесс-команды»).

## When a skill says "publish to the issue tracker"

Create a GitHub issue с подходящими `type:*` метками.

## When a skill says "fetch the relevant ticket"

Run `gh issue view <number> --comments`.
