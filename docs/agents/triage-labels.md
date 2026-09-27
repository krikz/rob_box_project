# Triage Labels

> ⚠️ У Rob Box НЕТ канонических mattpocock-ролей (`needs-triage`, `ready-for-agent`
> и т.д.). Триаж автоматизирован: `scripts/agent_flow/agent-flow-triage.sh`
> подхватывает issues с меткой `hermes` и создаёт kanban-карточки.
> Ниже — маппинг mattpocock-ролей на РЕАЛЬНЫЙ workflow Rob Box.

| Роль в mattpocock/skills | Аналог в Rob Box | Смысл |
| ------------------------ | ---------------- | ----- |
| `needs-triage` | `hermes` (без `agent:<role>`) | ждёт подхвата `agent-flow-triage.sh` |
| `needs-info` | comment с вопросом автору (нет метки) | не хватает данных — спрашиваем в комментарии |
| `ready-for-agent` | `hermes` + `agent:<role>` | триаж заводит kanban-карточку на нужный профиль |
| `ready-for-human` | `agent:architect` / ручная карточка | Шифу или architect решает руками |
| `wontfix` | `gh issue close` | не делаем — просто закрываем |

Полная таксономия лейблов — `CONTRIBUTING.md` (секция labels) и
`.agents/skills/github-issues-workflow/SKILL.md`.
