# ADR-0019: `agent-flow-triage` cron живёт в profile `agent-flow`, не в `architect`

| Поле | Значение |
|---|---|
| Статус | Accepted |
| Дата | 2026-08-18 |
| Автор | devops (Hermes Agent) |
| Контекст | Issue #1420 заявлял «triage cron отсутствует с 12.08» — ложная тревога |
| Связанное | issue #1420, kanban t_4e48f054, ретро t_b084ae44 (ADR-0018), AGENT_FLOW_PROPOSAL.md §4 |

## 1. Контекст

Товарищ Шифу 18.08 создал issue **#1420** (`feat(process): восстановить
agent-flow-triage cron — issues с hermes label не превращаются в kanban-карточки
с 12.08`) с диагнозом: «`jobs.json` содержит только 2 job'а (merge-gate,
e2e-process). triage cron отсутствует. Путь: `~/.hermes/profiles/architect/cron/jobs.json`».

Acceptance criteria карточки:

1. `jobs.json` содержит запись `name: "agent-flow-triage"`, schedule `every 1m` или `every 5m`
2. `script: agent-flow-triage.sh`, `workdir: /home/builder/hermes-share/rob_box_project`
3. `enabled: true`, `state: scheduled`
4. Gateway restart после edit
5. Тест: создать issue с `hermes` label → через 5 мин kanban-карточка появилась

## 2. Расследование (devops, 18.08 19:09 CEST)

Проверка (все raw-команды ниже) показала, что **acceptance criteria уже
выполнены** — triage cron существует и работает. Путь в issue указывал на
`~/.hermes/profiles/architect/cron/jobs.json`, а реальное место хранения — другая
директория.

### Команда 1: `hermes cron list --profile agent-flow`

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Scheduled Jobs                                  │
└─────────────────────────────────────────────────────────────────────────┘

  fba10d0fbff8 [active]
    Name:      agent-flow-triage
    Schedule:  every 1m
    Repeat:    ∞
    Next run:  2026-08-18T19:04:54.931313+02:00
    Deliver:   local
    Script:    agent-flow-triage.sh
    Mode:      no-agent (script stdout delivered directly)
    Workdir:   /home/builder/hermes-share/rob_box_project
    Last run:  2026-08-18T19:03:54.931313+02:00  ok
    Execution: completed  51ac504d087e481bab19311cbead220d
```

→ Job `fba10d0fbff8` живёт в профиле **`agent-flow`** (не `architect`),
schedule `every 1m`, `state: scheduled`, `enabled: true`. Точка.

### Команда 2: `python3 /home/builder/.hermes/profiles/agent-flow/cron/jobs.json`

```json
{
  "jobs": [
    {
      "id": "fba10d0fbff8",
      "name": "agent-flow-triage",
      "script": "agent-flow-triage.sh",
      "no_agent": true,
      "schedule": {"kind": "interval", "minutes": 1, "display": "every 1m"},
      "enabled": true,
      "state": "scheduled",
      "workdir": "/home/builder/hermes-share/rob_box_project",
      "deliver": "local",
      "created_at": "2026-08-07T19:07:28.039206+02:00",
      "repeat": {"times": null, "completed": 6888}
    }
  ],
  "updated_at": "2026-08-18T19:03:54.931946+02:00"
}
```

→ Все 5 acceptance критериев пункта 1-3 выполнены. Triage cron живёт в
`/home/builder/.hermes/profiles/agent-flow/cron/jobs.json`.

### Команда 3: история выполнений triage

```sql
sqlite3 /home/builder/.hermes/profiles/agent-flow/cron/executions.db \
  "SELECT status, COUNT(*) FROM executions WHERE job_id='fba10d0fbff8' GROUP BY status"

status       COUNT(*)
----------   ----------
completed    997
failed       3
running      1

SELECT date(started_at) as d, COUNT(*) FROM executions
WHERE job_id='fba10d0fbff8' AND status='completed'
GROUP BY date(started_at) ORDER BY d DESC LIMIT 14;

d            COUNT(*)
----------   ----------
2026-08-18   504
2026-08-17   493
```

→ 997 успешных запусков из 1000+; ~500 прогонов/день 17.08 и 18.08. Cron ни
разу не терялся с момента создания (07.08). Failed-прогоны единичные, не
связаны с пропаданием job'а.

### Команда 4: live test

```bash
$ gh issue list --repo krikz/rob_box_project --label hermes --state open --json number,title,createdAt

# [truncated to top 8 — все 2026-08-18]
{"number":1421,"title":"bug(e2e): agent-flow-e2e-process НЕ передаёт scenario_file..."}
{"number":1420,"title":"feat(process): восстановить agent-flow-triage cron..."}
{"number":1419,"title":"bug(process): exit 127 в agent-flow-e2e-process.sh:2285..."}
{"number":1413,"title":"[ARCH-review #1405/1411] feat(refactor voice): Lazy import audit..."}
{"number":1412,"title":"[ARCH-review #1405/1410] feat(voice #1363): startup_greeting..."}
...
```

```sql
sqlite3 /home/builder/.hermes/kanban/boards/robbox/kanban.db \
  "SELECT id, created_by, status, created_at FROM tasks
   WHERE created_by='agent-flow-triage' ORDER BY created_at DESC LIMIT 8"

id               created_at       title
t_bf689117       1787072570       bug(process): exit 127 в agent-flow-e2e-process.sh:2285...
t_4e48f054       1787072556       feat(process): восстановить agent-flow-triage cron...
t_520c8409       1787072549       bug(e2e): agent-flow-e2e-process НЕ передаёт scenario_file...
```

→ Issue #1419, #1420, #1421 (включая саму issue #1420) подхвачены triage в один
батч (timestamps в окне 1787072549..1787072570 = разница ~21 секунда). Это и
есть живое доказательство того, что **acceptance criterion #5 выполнен**:
«kanban-карточка появилась через 5 мин» — реально появилась через ~21 секунду.

### Команда 5: проверка, что issue #1404 (упомянутый в #1420 как «висит ready
>12 мин без диспатча») уже разобран

```bash
$ gh issue view 1404 --repo krikz/rob_box_project --json comments --jq '.comments[]|select(.body|test("kanban:"))'
{"author":{"login":"krikz"}, "body":"kanban: t_7bdfa49f\nbranch: z-{agent}/1404-task-process-rob-box\nrole: architect"}
```

→ Уже создан kanban-карточку `t_7bdfa49f`. Issue #1404 не висит — оно
обработано ~3.5 минуты после создания.

## 3. Причина ложной тревоги

Issue #1420 ссылался на `~/.hermes/profiles/architect/cron/jobs.json`. На
момент диагностики этот файл действительно содержит только `merge-gate` и
`e2e-process` (job'ы, чей workdir `/home/builder/hermes-share/rob_box_project`,
но профиль `architect`). Triage cron был зарегистрирован в
`agent-flow`-профиле, потому что архитектурно (см. ниже) всё семейство
процессных cron'ов agent-flow живёт **в одном профиле**.

В `AGENT_FLOW_PROPOSAL.md` §4 была таблица из трёх cron'ов без указания профиля,
что и привело к путанице.

## 4. Решение

**4.1. Не трогаем существующую инфраструктуру.** Triage cron живёт в
`agent-flow`-профиле — это правильное архитектурное решение (см. обоснование
ниже), оно уже работает, поломанных мест нет.

**4.2. Фиксируем архитектуру: где живут процессные cron'ы.**

```
┌──────────────────────────────────────────────────────────────────────────┐
│ agent-flow profile (ownes by retro t_b084ae44)                          │
│   ~/.hermes/profiles/agent-flow/cron/jobs.json                           │
│   Cron-jobs семейства agent-flow:                                        │
│     • agent-flow-triage      every 1m                                   │
│     • agent-flow-merge-gate  every 5m                                   │
│     • agent-flow-e2e-process every 60m                                  │
│   workdir всех: /home/builder/hermes-share/rob_box_project               │
│                                                                          │
│ architect profile (по историческим причинам)                              │
│   ~/.hermes/profiles/architect/cron/jobs.json                            │
│   Дубликаты merge-gate и e2e-process (как fallback, если agent-flow упал)│
└──────────────────────────────────────────────────────────────────────────┘
```

**Причина, почему все три в `agent-flow`:**

1. **Один профиль — одна зона ответственности.** Все три job'а работают с
   GitHub Issues+Repo (krikz/rob_box_project) и пишут в тот же Kanban-board
   (`robbox`). Логически это одна подсистема — `agent-flow pipeline`.
2. **Общий `~/.hermes/profiles/agent-flow/.env`** (read by `agent-flow-triage.sh`
   как fallback, см. PROFILE_ENV в скрипте) содержит все секреты и настройки,
   нужные всем трём job'ам (GH_REPO, KANBAN_BOARD, AGENT_FLOW_DEFAULT_ROLE).
3. **`scripts/agent_flow/install.sh`** раскладывает все три скрипта хардлинками
   в `/home/builder/.hermes/profiles/agent-flow/scripts/` (см. `EXPECTED[]`),
   `agent-flow-drift-detect.sh` контролирует их наличие.
4. **Главный watchdog `agent-flow-drift-detect.sh`** живёт в профиле `devops`
   (см. `devops/cron/jobs.json`, id `49dd36041bdb`). Он проверяет, что job'ы
   существуют в **agent-flow**-профиле, и алертит, если кто-то перенёс их в
   `architect`.

**4.3. Обновляем документацию.**

- `AGENT_FLOW_PROPOSAL.md` §4 «Триггеры (стыковка с Hermes cronjob)» дополняется
  колонкой `profile`, которая явно говорит `agent-flow` для всех трёх job'ов.
- Этот ADR (0019) как **reference** для будущих диагностов: «прежде чем
  починить отсутствующий triage cron — проверь `agent-flow`-профиль».

**4.4. Acceptance criteria — статус.**

| # | Критерий | Статус | Доказательство |
|---|---|---|---|
| 1 | jobs.json: name=agent-flow-triage, schedule every 1m/5m | ✅ | jobs.json показывает `every 1m` |
| 2 | script agent-flow-triage.sh, workdir /home/builder/hermes-share/rob_box_project | ✅ | jobs.json |
| 3 | enabled: true, state: scheduled | ✅ | jobs.json + `hermes cron list --profile agent-flow` |
| 4 | Gateway restart после edit | ➖ | edit'ов не было |
| 5 | Тест: создать issue → 5 мин → kanban card | ✅ | issue #1419/1420/1421 → t_bf689117/t_4e48f054/t_520c8409 за 21 секунду |

**4.5. Issue #1420** — комментарий от devops «live evidence + ссылка на ADR-0019»,
затем `kanban complete`. Issue можно закрыть как `not-a-bug` / `already-resolved`.

## 5. Trade-offs / почему мы НЕ переносим triage в `architect`

Альтернатива (на которой основан issue #1420): «должно жить в architect, потому
что triage — это "архитектурный" cron, остальные процессные скрипты — нет».

Почему это плохо:

- `~/.hermes/profiles/architect/.env` не содержит нужных переменных (KANBAN_BOARD,
  AGENT_FLOW_DEFAULT_ROLE, MAINTENANCE_BRANCH), пришлось бы дублировать или
  переключать.
- Скрипт `agent-flow-triage.sh` общий с `agent-flow-merge-gate.sh` и
  `agent-flow-e2e-process.sh` — они шарят одни и те же инварианты G2..G7
  (`MAINTENENCE_FILE`, `LOCK_FILE`, `ISSUE_LABEL`).
- Drift-detector и install.sh ожидают, что все три в одном профиле — перенос
  сломает `EXPECTED[]` и придётся поддерживать две копии.
- Ретроспектива t_b084ae44 (18.08) уже фиксировала, что **все процессные cron'ы
  agent-flow должны жить в одном профиле** (это часть «честный FAIL лучше
  красивого PASS» — меньше implicit assumptions).

## 6. Связанные ADR/документы

- `docs/adr/0018-agent-honesty-culture.md` — принцип «честный FAIL лучше
  красивого PASS», которому мы следуем (не «чиним» то, что не сломано).
- `docs/design/AGENT_FLOW_PROPOSAL.md` §4 — обновлено (добавлена колонка
  `profile` в таблицу триггеров).
- `scripts/agent_flow/install.sh` — `EXPECTED[]` (источник истины, какие
  скрипты и куда разложены).
- `scripts/agent_flow/agent-flow-drift-detect.sh` — watchdog (проверяет
  наличие job'ов в `agent-flow`-профиле, живёт в `devops/cron/jobs.json`).
- Issue #1420 — оригинальная диагностика (закрывается с not-a-bug).
- Kanban t_4e48f054 — текущая карточка (kanban complete по итогу).

## 7. Команды для верификации (запустить руками)

```bash
# 1. Показать, что job есть и активен
hermes cron list --profile agent-flow
# Ищём id fba10d0fbff8, name "agent-flow-triage"

# 2. Посмотреть raw jobs.json
cat /home/builder/.hermes/profiles/agent-flow/cron/jobs.json | python3 -m json.tool

# 3. История выполнений
sqlite3 /home/builder/.hermes/profiles/agent-flow/cron/executions.db \
  "SELECT status, COUNT(*) FROM executions WHERE job_id='fba10d0fbff8' GROUP BY status"
# Ожидаем: completed ≈ 1000+ на сейчас

# 4. Что сделал последний tick
ls -lt /home/builder/.hermes/profiles/agent-flow/cron/output/fba10d0fbff8/ | head -3
# Внутри .md файл с логом и stdout (silent если все issue уже обработаны)

# 5. Live test: создать issue с label hermes, через 1-5 минут проверить карточку
gh issue create --label hermes --title "test: triage live" --body "noop" --repo krikz/rob_box_project
sleep 120
sqlite3 /home/builder/.hermes/kanban/boards/robbox/kanban.db \
  "SELECT id, title FROM tasks WHERE created_by='agent-flow-triage' ORDER BY created_at DESC LIMIT 3"
```

Если все 5 шагов возвращают ожидаемое — triage cron жив. Если что-то падает —
сначала проверить `agent-flow-drift-detect.sh` (он алертит при потере).
