# Stale-candidate Triage Runbook

> Операционный runbook для обработки issues, попавших под `stale-candidate`
> в krikz/rob_box_project.
> Источник процесса: ADR-0022 § 4.2 «GATE-2 двушаговая автозакрывалка»
> (`docs/adr/0022-process-e2e-done-gates.md`).
> Контекст потерь: ретро `orphan-stale-no-agent-assign` (14.09.2026).

## Что такое `stale-candidate`

Метка, которую автоматически ставит `scripts/agent_flow/agent-flow-merge-gate.sh`
на issues со статусом `OPEN` без process-меток (`agent:*`/`needs-*`/`hermes`/
`e2e-done`/`e2e:rejected`/`needs-plan`) более 24 часов (ADR-0022 GATE-2,
STALE_HOURS_2h).

`stale-candidate` — это **сигнал-о-забвении**, не сигнал-о-бесполезности. Если
issue всё ещё актуальна, она должна быть либо подхвачена (`agent:*`), либо
получить человеческий комментарий, либо быть закрыта руками.

## Базовый сценарий: triage-cron подхватывает `stale-candidate`

Обычный случай — `agent-flow-triage.sh` видит `hermes` или process-метки и
создаёт kanban-карточку. Это работает, если issue уже была в очереди
триажа хотя бы раз.

**Проблема**: если issue создана **напрямую** (без `hermes`) и stale-tick
прошёл раньше, чем кто-то успел проставить `agent:*` метку, issue зависает
как `stale-candidate AND NOT has agent:*` на десятки часов. Это и есть
класс инцидентов из ретро `orphan-stale-no-agent-assign`.

## Escalation: high-priority voice/operator-bug без `agent:*`-метки

> ⚠️ **Эта секция — новая** (заведена в рамках карточки `t_cf60f006`,
> ретро-синхронизация с Шифу от 14.09.2026). Закрывает класс orphan-stale
> для P0 voice/operator-bugs.

### Триггер

Сработал `stale-candidate` (или agent-flow-triage.sh НЕ присвоил `agent:*` в
течение 30 минут после создания issue) на issue, у которой выполнены **все**
три условия:

1. `priority:high` ИЛИ `priority:critical`
2. `bug` label
3. любой из domain-меток: `voice`, `operator`, `quest`, `llm`, `tts`, `stt`

### Что должен сделать `triage-cron` (целевое поведение)

В этом сценарии `agent-flow-triage.sh` должен **отдельно** (не через общую
очередь) обработать issue:

1. Проставить `agent:backend` (или `agent:devops`, если domain = `deploy`/`ci`)
   и `needs-triage` (если плана ещё нет) **немедленно**, без ожидания batch.
2. Завести kanban-карточку с `priority=0` (раньше других ready-задач).
3. Оставить комментарий в issue со ссылкой на kanban-карточку и упоминанием
   owner'а (`@krikz`) и Шифу (`@GOODWORKRINKZ`), если Шифу фигурирует в
   P0-листе (`docs/p0-voice-bugs-decisions.md`).
4. Снять `stale-candidate` (она выполнила свою функцию).

### Как понять, что есть P0

- Запросить `docs/p0-voice-bugs-decisions.md` (таблица решений Шифу).
- Если issue в таблице — действовать по соответствующей строке (workaround /
  hotfix / принять риск).
- Если issue НЕ в таблице, но удовлетворяет триггеру выше — это **candidate
  P0** (вероятный P0), эскалировать как probable-P0.

### Кого пинговать

| Роль | Кого пинговать | Когда |
|---|---|---|
| Owner репо | `@krikz` (mention в issue-комментарии) | всегда при срабатывании триггера |
| Шифу | `@GOODWORKRINKZ` (mention в issue-комментарии) | если issue в `docs/p0-voice-bugs-decisions.md` или candidate-P0 |
| Devops-канал | `oncall-devops` (mention в комментарии + отдельный alert) | если `stale-candidate` висит > 1 часа И priority:high (см. follow-up issue) |
| Agent-flow | внутренний alert в `agent-flow-error` log | если принудительный triage не сработал в течение 30 мин |

### Как отметить в issue

После применения триаж-действий оставить комментарий в issue с шаблоном:

```text
🚨 stale-candidate на priority:high voice/operator-bug.

Причина: [короткая — например: «stale-tick прошёл до того, как
agent-flow-triage.sh успел подобрать issue без hermes-метки»].

Действие:
- проставлено `agent:backend`, `needs-triage`
- заведена kanban-карточка t_XXXX
- @krikz пинг

Ссылки:
- Ретро-таблица: docs/retros/orphan-stale-no-agent-assign-2026-09-14.md
- ADR-0022 GATE-2: docs/adr/0022-process-e2e-done-gates.md § 4.2
- Решения Шифу: docs/p0-voice-bugs-decisions.md (если применимо)
```

Затем:
- снять `stale-candidate`;
- проставить `agent:backend` (или нужный профиль);
- проставить `needs-triage` или `needs-plan` (в зависимости от наличия
  плана в issue body).

### Чего НЕ делать

- Не закрывать issue автоматически, даже если `stale-candidate` висит > 24 ч.
  Это контра ADR-0018 («честный FAIL лучше красивого PASS») и причина
  ретро-инцидента.
- Не убирать `priority:high` без явного решения Шифу в issue-комментарии.
- Не удалять `stale-candidate` до того, как issue получила `agent:*` —
  иначе потеряется сигнал.

## Связанные артефакты

- `docs/adr/0022-process-e2e-done-gates.md` — канон GATE-2.
- `docs/retros/orphan-stale-no-agent-assign-2026-09-14.md` — ретро-таблица
  (источник проблемы и data-driven рекомендаций).
- `docs/p0-voice-bugs-decisions.md` — решения Шифу по P0 voice/operator-bugs.
- Follow-up issue: см. раздел «TODO» в ретро § 7.