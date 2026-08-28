---
name: agent-flow
description: agent-flow cron scripts (merge-gate, triage, e2e-process, completion-check) — git worktree auto-merge loop with e2e-rotation. Used when touching scripts/agent_flow/* or orchestrating Kanban worker PRs.
---

# agent-flow — automated PR/issue pipeline (issue #1534, ADR-0014)

## Назначение

`scripts/agent_flow/` — набор bash-скриптов, крутящихся в cron
(профиль `agent-flow`: triage — every 1m, merge-gate — every 5m,
e2e-process — every 60m; см. ADR-0019). Скрипты автоматически:

- мердят PR через `gh pr merge --auto` (после Q22 / retro-path / clean-pr-sweep)
- ставят/снимают labels (`needs-e2e`, `e2e-done`, `e2e:rejected`, `needs-review`)
- закрывают issue после успешного merge (PASS-proven path, retro-path)
- триажут новые issue (назначают assignee из kanban-профиля)

## Self-id / whoami — почему и как (issue #1534)

**Проблема:** Все side-effect'ы на PR/issue делаются от GH-токена krikz.
В истории GitHub непонятно, кто конкретно закрыл PR #1507 — товарищ Шифу
руками, или `agent-flow-completion-check` автоматически в фоне.

**Решение:** перед каждым side-effect скрипт пишет комментарий:

```
🤖 [agent:devops] script=agent-flow-merge-gate action=closing
   reason: post-merge cleanup: PASS-proven via e2e-done, PR MERGED into develop
```

## Helper: scripts/agent_flow/hermes_github.sh

Все процессные скрипты (merge-gate / triage / e2e-process / completion-check)
source'ят этот helper и используют convenience wrappers:

| Wrapper                                  | Что делает                                    |
|------------------------------------------|-----------------------------------------------|
| `whoami_close_issue N "reason"`          | перед `gh issue close N`                      |
| `whoami_reopen_issue N "reason"`         | перед `gh issue reopen N`                     |
| `whoami_add_label N L "reason" [meta]`   | перед `gh issue edit --add-label L`           |
| `whoami_remove_label N L "reason" [meta]`| перед `gh issue edit --remove-label L`        |
| `whoami_set_assignee N U "reason"`       | перед `gh issue edit --add-assignee U`        |
| `whoami_close_pr N "reason" [meta]`      | перед `gh pr close N`                         |

Или низкоуровневый `post_whoami_comment kind number action reason [k=v...]`.

### Idempotency (acceptance #5)

Helper перед публикацией делает GET `/repos/{owner}/{repo}/issues/{N}/comments`
и проверяет, есть ли в последних `HERMES_WHOAMI_WINDOW_SECONDS` (default 7200s
= 2ч) комментарий с тем же marker'ом:

```
🤖 [agent:<role>] script=<script_name> action=<action>
```

Если есть — skip silently (защита от reconcile-loop дублей при cron-flap'е).

### Failure semantics (acceptance #4)

Если `gh` упал (rate-limit / network / permission) — **action всё равно
выполняется** (helper возвращает 0), в stderr пишется warning:

```
[hermes_github] WARN: whoami-comment failed for issue#N ... — caller action will proceed
```

Это сознательный trade-off: traceability хороша, но не должна блокировать
основной cron-loop.

### Configuration

| Env var                          | Default          | Назначение                                |
|----------------------------------|------------------|-------------------------------------------|
| `HERMES_AGENT_ROLE`              | `agent:devops`   | role в marker'е (можно override на profile)|
| `HERMES_WHOAMI_WINDOW_SECONDS`   | `7200`           | окно идемпотентности                       |
| `GH_REPO`                        | (нет)            | `owner/repo` для API-вызовов               |

## Source-линии в процессных скриптах

Каждый из 4 скриптов содержит рядом с source'ом lib_user_unlabel_check.sh:

```bash
_LIB_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=hermes_github.sh
. "$_LIB_DIR_HERE/hermes_github.sh"
```

Комментарий `# shellcheck source=hermes_github.sh` нужен чтобы shellcheck
не предупреждал о non-constant source path.

## Тесты

`scripts/agent_flow/tests/test_hermes_github.sh` — 12 unit/integration
тестов. Использует fake `gh` (PATH-prepend), проверяет:

- формат marker'а
- idempotency (двойной вызов = один whoami)
- failure semantics (gh fail → caller proceeds)
- интеграцию во все 4 скрипта (grep по `source=hermes_github.sh`)
- install.sh EXPECTED содержит `hermes_github.sh`

Прогон:

```bash
bash scripts/agent_flow/tests/test_hermes_github.sh
```

Ожидаемый вывод: `Total: 12 / Passed: 12 / Failed: 0 / ALL TESTS PASSED`.

## Когда использовать whoami_*

**Используй перед** каждой мутацией PR/issue из process-скрипта:

- `gh issue close N` / `gh issue reopen N`
- `gh issue edit --add-label L` / `--remove-label L`
- `gh issue edit --add-assignee U` / `--remove-assignee U`
- `gh pr close N` (обычно merge вместо close, но в gate-block'е возможен)

**НЕ используй** для:

- `gh issue comment N --body "..."` (это сам комментарий, не action)
- `gh pr comment N` (аналогично)
- read-only операций (`gh issue view`, `gh pr view`)

## Решение проблем

**t03_post_format падает (script=hermes_github, ожидается test_hermes_github):**
`_auto_script_name` нашёл не тот frame. Алгоритм в helper'е:
проходит BASH_SOURCE-стек и skip'ает frame'ы с basename `hermes_github.sh`.
Если правило забыли — caller подменяется. Проверь что helper загружен
через `. "$_LIB_DIR_HERE/hermes_github.sh"`, а не через прямой exec.

**При ручном запуске helper постит «лишние» комментарии:** проверь
`HERMES_WHOAMI_WINDOW_SECONDS` — по дефолту 2ч, за это окно тот же action
подавляется. Если надо пропустить — установи 0 (хотя helper рассчитан
на короткие окна, а не на «не idempotent»).

**PR #N закрыт с пометкой krikz:** helper упал (rate-limit). Проверь
`GH_REPO` и gh auth. Action всё равно выполнилась — это by design (см.
Failure semantics).

## Acceptance check (issue #1534)

- [x] Helper-функция `post_whoami_comment(pr|issue, action, reason)` в `scripts/agent_flow/hermes_github.sh`
      (раскладка install.sh)
- [x] Используется в agent-flow-merge-gate / triage / e2e-process / completion-check
- [x] Формат: `🤖 [agent:<role>] script=<script_name> action=<action>`
- [x] Если comment нельзя постить — action всё равно делается, warning в лог
- [x] Тесты на idempotency (двойной запуск = один whoami)
- [x] Документация в `.agents/skills/agent-flow/SKILL.md`

## Связанное

- Issue #1534 — self-id whoami для traceability
- Issue #1527 — dispatch inputs (кто запустил build)
- ADR-0014 — hermes-agent-flow (основной архитектурный ADR)
- ADR-0018 — честный FAIL > красивый PASS (raw-вывод обязателен)
- t_7eb1ac08 — карточка этой задачи