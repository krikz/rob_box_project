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

- **Авто-триаж, не руками.** Issue с меткой `hermes` подхватывает
  `scripts/agent_flow/agent-flow-triage.sh` (каждую минуту) и заводит
  kanban-карточку. Просто `gh issue create` без `hermes` + `agent:<role>`
  карточку НЕ создаст — issue будет висеть.
- **Создание задачи:** `gh issue create --label "type:functional,priority:high,ai-generated,source:gsd" --repo krikz/rob_box_project`.
- **Таксономия лейблов** (`type:*`, `priority:*`, `agent:<role>`, `hermes`,
  `needs-e2e`, `e2e-done`, `e2e:rejected`, `no-e2e-required`, `needs-review`,
  `source:gsd`) — в `CONTRIBUTING.md` и `.agents/skills/github-issues-workflow/SKILL.md`.

## Pull requests as a triage surface

**PRs as a request surface: no.** Внутренние PR агентов идут через
merge-gate (`agent-flow-merge-gate.sh`) → e2e-ротацию, а не через ручной
triage внешних PR.

## When a skill says "publish to the issue tracker"

Create a GitHub issue с подходящими `type:*` + `agent:<role>` метками.

## When a skill says "fetch the relevant ticket"

Run `gh issue view <number> --comments`.
