# Stale-candidate Triage Runbook

> Операционный runbook для обработки issues, попавших под `stale-candidate`
> в krikz/rob_box_project.
> Источник процесса: ADR-0022 § 4.2 «GATE-2 двушаговая автозакрывалка»
> (`docs/adr/0022-process-e2e-done-gates.md`),
> ADR-AF-0032-amendment-1 «ISO-date collision + Backfill procedure»
> (`docs/adr/AF-0032-amendment-1-iso-date-collision.md`).
> Контекст потерь: ретро `orphan-stale-no-agent-assign` (14.09.2026),
> ретро 2026-W40 (`t_bf8216cb` — issue #3354 пропущен G9a dedup 19+ ч).

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

## Backfill procedure для пропущенных issues (новое, 2026-10-04)

> ⚠️ **Эта секция — новая** (заведена в рамках kanban `t_8616bfbb`,
> ретро-синхронизация с Шифу от 04.10.2026, источник — issue #3354
> пропущенный G9a intra-tick dedup 19+ ч, см.
> `docs/diagnostics/2026-10-03-triage-skip-3354.md`).
> Закрывает класс багов «nightly review нашёл issue без kanban-карточки».

### Отличие от stale-candidate escalation (выше)

`stale-candidate` escalation (предыдущая секция) — это **forward**:
issue только что получила `stale-candidate`, и мы **предотвращаем** её
пропуск, ставя `agent:*` метки немедленно.

`backfill` procedure (эта секция) — это **backward**:
nightly review (например, `t_bf8216cb`) **уже нашёл** issue, которая
висит OPEN с `hermes` label, но **не имеет** kanban-карточки (т.е.
triage должен был её подхватить, но не сделал — из-за G9a skip'а,
gh-side-effect fail'а или другого бага). Цель — **ручное создание
карточки** по шаблону и cross-link в meta-issue.

### Триггер

Nightly review (или devops на SSH-сессии с `sqlite3 ~/.hermes/kanban.db`)
обнаружил:

```sql
-- 1. issue с label hermes без kanban-карточки
SELECT number, title, created_at
FROM gh_issues
WHERE state = 'open'
  AND 'hermes' = ANY(labels)
  AND number NOT IN (
    SELECT CAST(SUBSTR(issue_ref, 2) AS INTEGER)
    FROM kanban_tasks
    WHERE issue_ref IS NOT NULL
  )
ORDER BY created_at DESC;

-- 2. альтернативный alert query (secondary watchdog, ADR-AF-0032-amendment-1 §5)
SELECT id, title, body, created_at
FROM tasks
WHERE issue_ref IS NULL
  AND labels LIKE '%hermes%'
  AND created_at > datetime('now', '-24 hours')
ORDER BY created_at DESC;
```

Ожидаемое: 0 строк. Тревожное: 1+ строка.

### Кого звать

| Класс issue | Assignee | Почему |
|---|---|---|
| `deployment` label | `agent:devops` (primary) | Deploy-bot domain |
| `voice`/`operator`/`bug` + `priority:high` | `agent:backend` (primary, force-triage § выше) | Voice/operator-bug fix |
| Любой другой | meta-issue owner | По домену issue |

Для **deploy-issue'ов** (например, #3354) — fanout к `agent:devops`.
Это **первый** задокументированный случай (`t_29682500`).

### Что делает assignee-воркер (backfill)

1. **Проверить root cause** через `agent-flow-triage.log`:

   ```bash
   grep "skip=${issue_number}" /home/builder/.hermes/logs/agent_flow/triage-*.log
   # или
   grep "DEDUP_INTRA" /home/builder/.hermes/logs/agent_flow/triage-*.log | grep "#${issue_number}"
   ```

2. **Создать kanban-карточку** вручную по шаблону архивных соседних
   issue'ов. Для deploy-issues — это:
   - **Title:** «🚨 Deploy issues on develop (staging) — YYYY-MM-DD»
   - **Body:** deploy-signature `deploy-fail:develop:staging:YYYY-MM-DD`
     + Vision Pi/Main Pi/rtabmap evidence (если есть в логах) +
     cross-link на **PR с фиксом** (если уже есть, например PR #3355
     для #3354) + cross-link на **meta-issue** (например, #3374).
   - **Assignee:** devops-профиль.
   - **Priority:** 0 (high, по образцу архивных `t_76bdb45a`/`t_0d6f165b`).

3. **Cross-link** в meta-issue (например, #3374) — comment с маркером
   `agent-flow:backfill-applied`:

   ```text
   agent-flow:backfill-applied
   issue=#3354
   kanban_card=t_29682500
   meta_issue=#3374
   root_cause_class=triage-dedup
   fix_status=merged   # или in-progress / not-yet
   PR_with_fix=https://github.com/krikz/rob_box_project/pull/3355
   ```

4. **Post-mortem** в самом issue (#3354 или эквивалент) — comment
   с diagnostic-ссылкой + краткое summary root cause (1 абзац):

   ```text
   agent-flow:post-mortem-backfill
   Этот issue был пропущен G9a intra-tick dedup (AF-0032 §2.1) 03.10.2026
   05:57Z и вручную подобран devops-воркером 04.10.2026 03:00Z.
   Fix: PR #3355 (READY). После merge этот issue закроется автоматически
   на ближайшем deploy-gate re-run (FP-сигнатуры больше не откроют
   "Deployment Completed With Issues").
   Diagnostic: docs/diagnostics/2026-10-03-triage-skip-3354.md
   Backfill-procedure: docs/runbooks/stale-candidate-triage.md § Backfill
   ```

5. **Если backfill-карточка obsolete** (fix уже в merged PR) — закрыть
   с verdict `premise-obsolete` (AF-0066) и cross-link на fix-PR. **Пример:**
   `t_29682500` закрыта за 51 сек с verdict premise-obsolete, fix в
   PR #3355 MERGEABLE+CLEAN.

### Audit trail (обязательные поля)

Каждая backfill-карточка **обязана** в комментарии `agent-flow:backfill-applied`
содержать **все 5 полей**:

- `issue=<N>` — какой issue был пропущен
- `kanban_card=<t_XXXX>` — какая карточка создана вручную
- `meta_issue=<#M>` — в каком meta-issue cross-link сделан
- `root_cause_class=<triage-dedup|cron-silent|gh-error|other>` — класс бага
- `fix_status=<in-progress|merged|not-yet>` — статус фикса

Этот audit-trail потом используется в nightly review для подсчёта класса
багов и приоритизации fix'ов (см. ADR-AF-0032-amendment-1 §6).

### Чего НЕ делать в backfill

- **Не создавать карточку без cross-link в meta-issue** — иначе parent
  process не узнает, что backfill применён, и issue «потеряется» снова.
- **Не использовать template вне архивного контекста** — backfill-карточка
  должна выглядеть как обычная triage-карточка (тот же assignee, тот же
  формат body), иначе е2e-rotation её не подхватит.
- **Не исправлять root cause в backfill-карточке** — это **отдельная
  devops-карточка** (`t_235579f1` для #3354). Backfill только создаёт
  карточку для issue'а; фикс — в отдельной worktree.
- **Не оставлять issue без `agent-flow:backfill-applied` маркера** —
  audit-trail обязателен (ADR-0018).

## Связанные артефакты

- `docs/adr/0022-process-e2e-done-gates.md` — канон GATE-2.
- `docs/adr/AF-0032-triage-dedup-guard.md` — канон G9a intra-tick dedup.
- `docs/adr/AF-0032-amendment-1-iso-date-collision.md` — ISO-date collision
  failure mode + mitigations (PR #3385, PR #3381) + backfill procedure
  (эта секция) + alert query.
- `docs/diagnostics/2026-10-03-triage-skip-3354.md` — полный raw-evidence
  report для primary примера backfill (#3354 → `t_29682500`).
- `docs/retros/orphan-stale-no-agent-assign-2026-09-14.md` — ретро-таблица
  (источник проблемы и data-driven рекомендаций).
- `docs/p0-voice-bugs-decisions.md` — решения Шифу по P0 voice/operator-bugs.
- Follow-up issue: см. раздел «TODO» в ретро § 7.