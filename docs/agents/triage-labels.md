# Triage Labels

> ⚠️ У Rob Box НЕТ канонических mattpocock-ролей (`needs-triage`, `ready-for-agent`
> и т.д.) и нет автомата-триажа (hermes выведен, ADR-0162). Issue разбирает
> координатор по `docs/process/claude-coordinator-robot-flow.md`.
> Ниже — маппинг mattpocock-ролей на реальный процесс Rob Box.

| Роль в mattpocock/skills | Аналог в Rob Box | Смысл |
| ------------------------ | ---------------- | ----- |
| `needs-triage` | открытая issue без исполнителя | ждёт координатора |
| `needs-info` | comment с вопросом автору (нет метки) | не хватает данных — спрашиваем в комментарии |
| `ready-for-agent` | бриф воркеру от координатора (нет метки) | воркер берёт задачу в worktree |
| `ready-for-human` | вопрос Шифу в issue / докладе | решает Шифу |
| `wontfix` | `gh issue close` | не делаем — просто закрываем |

Метки `hermes`, `agent:*`, `source:gsd`, `ai-generated`, `needs-e2e`, `e2e-done` новым issue
не ставить (`AGENTS.md`, раздел «Issue»). Команды — `.agents/skills/github-issues-workflow/SKILL.md`.
