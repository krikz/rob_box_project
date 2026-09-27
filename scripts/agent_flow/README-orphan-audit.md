# agent-flow-orphan-audit — telemetry для тёзок-карточек

`scripts/agent_flow/agent-flow-orphan-audit.sh` — **telemetry-only** watchdog
для agent-flow pipeline (retrospective-карточка `t_3dbde205`,
retro-key `g10c-prereq-merged-card-cancel`).

## Что делает

- Каждые 15 мин (cron) сканирует активные карточки канбана (`todo`,
  `ready`, `running`, `blocked`, `triage`) и группирует их по
  `(repo, issue_number)` — ищет `#N` и `krikz/<repo>` в `body`.
- Если в группе **≥ 2 карточек** → emit alert + пишет событие
  `orphan_detected` в `task_events` (по одной на КАЖДОЙ карточке-тёзке).
- Сам **cancel не делает** — это задача merge-gate (G10c guard,
  отдельная карточка `t_e39afb1c`); audit только алертит.

## Алерт-каналы

| `ORPHAN_ALERT_CHANNEL` | Поведение                                                |
|------------------------|----------------------------------------------------------|
| `file` (default)       | Дописывает строку `ORPHAN_ALERT issue=#N count=…`        |
|                        | в `$ORPHAN_ALERT_LOG` (default `~/.local/state/...`)     |
| `slack`                | POST на `$ORPHAN_SLACK_WEBHOOK`                          |
| `gh_discussion`        | `gh issue comment` на issue (для GH discussion)          |

Формат строки алерта (жёстко зафиксирован):

```
ORPHAN_ALERT issue=#2406 count=4 cards=[t_3dbde205,t_e39afb1c,t_6535e27d,t_40a610d0] merged_prs=[2458] recommendation=cancel
```

## Гейт на `recommendation`

- `gh issue view N --json closedByPullRequestsReferences` — REST-API,
  primary path. Если находит ≥ 1 merged PR → `recommendation=cancel`.
- Fallback (если REST пуст) — `gh pr list --state merged --limit 200`
  grep по title (`#N`) — для случаев, когда PR закрывает issue
  неявно (`partially addresses`).
- Если ничего не найдено → `recommendation=warn`. Merge-gate всё
  равно сделает финальный verdict (полнота покрытия).

## Idempotency

- По `issue:repo:N` — cooldown `ORPHAN_COOLDOWN_SECS` (default 3600s).
  State — SQLite в `$ORPHAN_STATE_FILE`.
- По `(task_id, issue)` — cooldown на уровне `task_events` чтобы не
  дублировать событие при коротких повторных сканах.

## Метрика

Файл `$ORPHAN_METRICS_FILE` (Prometheus textfile-format):

```
agent_flow_orphan_cards_total <gauge>
agent_flow_orphan_groups_total <gauge>
```

Опционально `$ORPHAN_PUSHGATEWAY_URL` — пуш в Pushgateway
(curl POST .../metrics/job/agent_flow_orphan_audit).

## Запуск

```bash
# однократно
bash scripts/agent_flow/agent-flow-orphan-audit.sh --dry-run

# с alert-каналом slack
ORPHAN_ALERT_CHANNEL=slack ORPHAN_SLACK_WEBHOOK=https://hooks.slack.com/... \
    bash scripts/agent_flow/agent-flow-orphan-audit.sh

# cron (agent-flow профиль, every 15m):
# добавить через `hermes cron create --no-agent --script agent-flow-orphan-audit.sh
#   --workdir <repo> "every 15m"`
```

## Журнал событий

После прогона в `task_events` появляются строки:

```sql
SELECT task_id, payload FROM task_events
WHERE kind='orphan_detected' ORDER BY id DESC LIMIT 10;
```

Payload содержит: `kind, repo, issue, merged_prs, recommendation,
retro_key=g10c-prereq-merged-card-cancel, ts, task_id, sibling_cards`.

## Связанные компоненты

| Компонент                       | Карточка     | Назначение                                          |
|---------------------------------|--------------|-----------------------------------------------------|
| Этот audit                      | `t_3dbde205` | Telemetry: detect + alert + event                   |
| G10c merge-gate guard           | `t_e39afb1c` | Cancel самих карточек после merge (G10c расширение) |
| ADR-AF-0066 (worker pre/post)   | —            | Shared helpers                                      |
| ADR-AF-0067 (G9c dedup)         | —            | Branch-name race-window dedup (PR #2519, merged)    |