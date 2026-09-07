# ADR-0052: decomposed-children wake-up watchdog — будить детей, которых dispatcher не видит

| Поле | Значение |
|---|---|
| Статус | Proposed (после merge → Accepted) |
| Дата | 2026-09-07 |
| Автор | architect по наказу nightly-review `t_bfd19ffb` (2026-09-06) |
| Контекст | Декомпозиция эпика через `kanban create --parents` иногда создаёт child-задачи со `started_at=NULL`, `status=todo` (или даже `status=triage`), и **диспетчер их не поднимает**, потому что у этих детей нет `parent_id` в `task_links` — dispatcher ждёт `parents`/`status=todo`/`priority`/`assignee`, а не «возраст parent-decomposed-события». Результат: эпик висит мёртвым грузом (232ч на AV-11, 100ч на AV-27), родительская карточка `block_recurrences` растёт, Шифу узнаёт об этом только из ретро. |
| Затрагивает | (a) новый `scripts/agent_flow/agent-flow-decomposed-watchdog.sh`; (b) `scripts/agent_flow/install.sh` — `EXPECTED[]`, `ensure_decomposed_watchdog_cron()`, `_WATCHDOG_LAUNCHER_FILES[]`; (c) новый `scripts/agent_flow/tests/test_decomposed_watchdog.sh`; (d) **НЕ** затрагивает: dispatcher, kanban.py, hermes-agent — только механический cron-надзор |
| Родители | ADR-0031 (gsd-orphan-triage), ADR-0036 §4.3 (cron-надзор как жанр), ADR-0046 / ADR-0047 (orphan-cleanup — примеры того, как один симптом всплывает по 8 раз прежде чем формализуют), ADR-0049 (nightly-review — кто-то должен мониторить суточные deadlock'и) |
| Связанные | `t_bfd19ffb` (nightly-review 2026-09-06 — этот карточка и есть реакция на его «3 merged PR, AV-11 висит 232ч»), `t_42d98188` / `t_5f63b973` (root-карточки, на которых пойман паттерн), `kanban-retro-create.sh` (обёртка для создания карточек) |

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдается

Nightly-review `t_bfd19ffb` (2026-09-07 03:13) зафиксировал аномалию, которая **уже была видна в коде, но не была отрефакторена**: decomposed-эпик оставляет за собой дочерние карточки в состоянии «никогда не подняты диспетчером». Сырые доказательства (выполнены на robbox-доске 2026-09-07, develop HEAD `07d33d20`):

```
SELECT id, status, started_at, assignee FROM tasks WHERE id IN (...);
| t_<id>     | title                        | status | started_at | assignee     |
| t_42d98188 | [AV-11] root                 | todo   | 1787906654 | agent-flow   |
| t_82f555cf | Design AV-11 scenario        | todo   | NULL       | tester       |  ← висит 232ч
| t_77a878a8 | Execute AV-11 scenario       | todo   | NULL       | tester       |  ← висит 232ч
| t_93effef9 | Review PR AV-11              | todo   | NULL       | pr-reviewer  |  ← висит 232ч
| t_08288c77 | Bring up mixed-mode stack    | triage | NULL       | agent-flow   |  ← висит 232ч (в triage, ещё хуже)
```

В `task_events` для root-карточки событие декомпозиции **есть**:
```
{'kind': 'decomposed', 'payload': '{"child_ids": ["t_08288c77",...], "root_assignee": "agent-flow"}',
 'created_at': 1787921622}
```

Но в `task_links` для неё — **пусто**:
```
SELECT * FROM task_links WHERE parent_id='t_42d98188';  → 0 rows
```

То есть «декомпозиция» в этой реализации — это **событие, а не parent-edge**. У ребёнка нет `parent_id` в dispatcher-видимом смысле, и диспетчер смотрит только на «обычные» критерии готовности (`status='todo'`/`'ready'` + `assignee` + `priority`). Если ребёнок в `status='todo'` с `assignee` — он поднимется. **Если ребёнок в `status='triage'` или был создан в обходе dispatcher-маршрутизации** — не поднимется, и никто его не разбудит.

### 1.2 Масштаб проблемы (доказательство системности, а не единичного случая)

Тот же запрос ко ВСЕМ 20 последним `decomposed`-событиям в robbox-доске:

| root | root_status | decomposed_at | age_h | children | in_task_links |
|---|---|---|---|---|---|
| `t_5f63b973` | todo | 100ч назад | 100 | 4 | **0** |
| `t_d614d376` | done | 100ч назад | 100 | 5 | **0** |
| `t_7d02a2d0` | done | 100ч назад | 100 | 5 | **0** |
| `t_42d98188` | todo | 228ч назад | 228 | 4 | **0** ← AV-11 |
| `t_5f63b973` | … | … | … | … | … |

Из 20 последних decomposed-рутов: **только 3 имеют записи в `task_links`**. 17 — без parent-edge. Паттерн системный: «decomposed-событие пишется, но task_links не заполняются» — это либо баг в коде декомпозиции (стоит отдельный тикет), либо by-design-ограничение. Для нашего watchdog'а это и то и другое OK — мы **не лечим баг, мы мониторим симптом**.

### 1.3 Почему существующие механизмы не покрывают

| Механизм | Что должен ловить | Что НЕ ловит |
|---|---|---|
| `agent-flow-blocked-watchdog-scope.sh` (ADR-0036 §4.3) | running-карточки с `age > 4ч` и `assignee ∉ {architect, devops}` | `status=todo` / `status=triage` — вне scope |
| `nightly-review.sh` (ADR-0049) | суточный дайджест, ретро-карточки по факту | не «будит» — только репортит |
| `agent-flow-rotation-watchdog.sh` | жива ли e2e-ротация | не kanban dispatch |
| `dispatcher` (kanban.py) | берёт ready-todo с assignee и priority | **не смотрит на «возраст parent decomposed_at»** |
| Шифу eyeballs | видит AV-11 232ч сам | «Не делай руками» (AGENTS.md, ADR-0018) |

Диспетчер не имеет триггера «родитель декомпозировал N дней назад → поднять детей». Этот watchdog — **компенсирующий контур надзора**, как `blocked-watchdog-scope` для mis-scope.

## 2. Решение

Новый механический (no-agent) cron-скрипт `agent-flow-decomposed-watchdog.sh`, расширение жанра по образцу `agent-flow-blocked-watchdog-scope.sh`:

### 2.1 Контракт per tick

1. `flock` lock — не два тика одновременно
2. iterate over all kanban boards (`~/.hermes/kanban/boards/*/kanban.db`, если `KANBAN_DB_PATH` не задан — single-DB mode)
3. SELECT all `task_events.kind='decomposed'` → parse payload → child_ids
4. Для каждого ребёнка проверить criteria (см. §2.2)
5. Если criteria выполнены И за последние сутки **нет** нашего marker-комментария:
   - emit comment через `hermes kanban --board <board> comment <child_id> <body>`
   - `UPDATE tasks SET priority = priority + 1 WHERE id = ?` (через sqlite, +1 за тик)
6. Log stats: scanned, matched, skipped_idempotent, emitted, errors

### 2.2 Criteria (выполняются ВСЕ)

| # | Условие | Почему |
|---|---|---|
| 1 | `parent_task.status IN ('todo','ready','blocked','triage')` | root ещё «живой» эпик — некому поднять детей |
| 2 | `child.started_at IS NULL` | ребёнок ни разу не был поднят диспетчером |
| 3 | `child.status='todo' OR child.status='triage'` | `done`/`archived`/`running` — вне scope (running — dispatcher сам разбирается; done — закрывать не нужно) |
| 4 | `task_events.created_at < now - 24h` (86400s) | свежее 24ч — нормальный pickup-лаг, не алерт |
| 5 | `task_events.created_at > now - 30d` (guard) | decomposed старше 30 дней — это «архивный долг», не алерт (есть отдельный orphan-cleanup ADR-0046/47) |

### 2.3 Что НЕ делаем (явно)

- **НЕ reassign** — Шифу / agent-flow решает. Auto-reassign = потеря контекста, если ребёнку уже был назначен профиль.
- **НЕ unblock** — root может быть в `blocked` по валидной причине (capability). Watchdog не знает контекста.
- **НЕ создаём новые карточки** — только comment + priority bump. Это «надзор», не «чинилка».
- **НЕ лечим баг декомпозиции** — это отдельная задача (`task_links` empty при decomposed-событии). Watchdog компенсирует симптом.

### 2.4 ENV и контракт скрипта

```
DECOMPOSED_WATCHDOG_DRY_RUN=true   # log only, no comment, no UPDATE
AGE_THRESHOLD_SECONDS=86400        # default 24h
MAX_AGE_SECONDS=2592000            # default 30d guard
MARKER_TAG="🤖 [agent:agent-flow] script=agent-flow-decomposed-watchdog"
PRIORITY_BUMP=1                   # default +1 за тик
LOCK_FILE=/tmp/agent-flow-decomposed-watchdog.lock
LOG_FILE=/tmp/agent-flow-decomposed-watchdog.log
HERMES_CLI=hermes
KANBAN_BOARDS_DIR=/home/builder/.hermes/kanban/boards
```

Exit codes (конвенция `blocked-watchdog-scope`):
- `0` — всё ok, ничего не будили
- `1` — критичный сбой (нет python3 / sqlite3 / lock fail)
- `2` — кого-то разбудили (alert для cron)

### 2.5 Регистрация cron-job

`install.sh::ensure_decomposed_watchdog_cron()` — идемпотентная функция, регистрирующая interval-job (every 4h — компромисс между «свежестью» и «нагрузкой») в devops-профиле, `no_agent=true` (скрипт = watchdog).

Почему **every 4h**, а не `every 1h`: decomposed-алерт — не hot-path. Часовой цикл у `blocked-watchdog-scope` — для mis-scope (это race с воркерами). Decomposed — это «эпик застрял», 4-часовой lag приемлем. Если эпик 30 дней никто не поднял — 4ч ничего не решают; если эпик 25ч — следующий 4-часовой тик его поймает.

### 2.6 Backlog sweep (разовый, ручной)

Применяется ОДНОВРЕМЕННО с merge этого PR — для 8 мёртвых детей из таблицы карточки (см. ниже в §6 Acceptance) — manual `kanban_comment` с marker'ом:

```
🤖 [agent:agent-flow] script=agent-flow-decomposed-watchdog
action=backlog-sweep reason=decomposed-Nd-no-pickup
watchdog начинает работу с этого PR. Дальнейшие алерты — автоматические (каждые 4ч).
```

Без unblock, без priority bump. Это «честная запись», что мы видели долг и начинаем за ним следить.

## 3. Структура файлов

```
scripts/agent_flow/
├── agent-flow-decomposed-watchdog.sh    ← новый (no-agent, every 4h, devops profile)
└── tests/
    └── test_decomposed_watchdog.sh      ← новый (5 unit-тестов: criteria match, idempotency,
                                           triage skip, NULL assignee skip, fresh-decomposed skip)

scripts/agent_flow/install.sh            ← +1 строка в EXPECTED, +ensure_decomposed_watchdog_cron,
                                           +1 в _WATCHDOG_LAUNCHER_FILES
```

Скрипт **структурно повторяет** `agent-flow-blocked-watchdog-scope.sh` — общая обёртка (flock, board scan, idempotency, __STATS__/__RECORD__ sentinel), отличается только SQL и marker'ом. Дублирование кода допустимо: эти watchdog'и — независимые контуры надзора, смешивать их = потеря читаемости (ADR-0049 §4 «Расширить существующий blocked-watchdog — Почему не он»).

## 4. Альтернативы

| Вариант | Почему не он |
|---|---|
| **Лечить баг в dispatcher / `kanban create` (заполнять `task_links`)** | Правильно, но это отдельная задача с другим профилем (backend). Watchdog компенсирует симптом СЕЙЧАС, фиксим баг — потом. ADR-0052 не блокирует этот фикс, оба живут параллельно. |
| **Расширить существующий `blocked-watchdog-scope`** | Другой горизонт (running-карточки vs todo/triage-карточки), другой marker, разные критерии. Смешивание сделало бы оба скрипта нечитаемыми (ретро-мотив из ADR-0049 §4). |
| **Передать responsibility падаван-вахте (`night_padawan_tick`)** | Падаван-вахта — LLM-агент, тикает каждый час, но: (a) «никогда не merge'ить руками» — LLM может нечаянно сделать reassign; (b) LLM стоит токенов, mechanical cron — нет; (c) «честный FAIL» — LLM может пропустить алерт, mechanical cron — гарантированно проходит по всему списку. |
| **Каждые 5 минут вместо 4ч** | Шум. 4ч — компромисс: даёт свежесть, не создаёт лог-спам. Если эпик провисит 25ч — это всё равно «надо будить», а 4-часовой тик даст 1 алерт (следующий поймает idempotency-guard). |
| **Внутри dispatcher сделать триггер** | Меняет dispatcher-логику (отдельный риск, отдельный ревью). Этот watchdog — внешний надзор, не меняет существующий код. |

## 5. Trade-offs

| Плюс | Минус |
|---|---|
| Закрывает системный пробел «decomposed-дети спят вечно» | +1 cron-job (every 4h) = +6 тиков/сутки = negligible cost (mechanical) |
| Механический, не зависит от LLM — стабильный 24/7 | Дублирование обёртки с `blocked-watchdog-scope.sh` (сознательное) |
| Backlog sweep сразу при merge — нет «честных 5 дней» ожидания первого тика | Не лечит root cause (пустые `task_links`); это симптом-watchdog |
| Idempotent через marker → можно тикать часто | Если marker в `task_comments` потерян (truncate) — будет spam. Mitigation: `today_start_utc` window 24ч, не абсолютный dedup. |
| Priority bump +1/тик → после 2-3 тиков ребёнок поднимается dispatcher'ом выше | priority-bump может «перебить» ручной priority — допустимо, ребёнок-то спит |

## 6. Acceptance

| # | Критерий | Кто | Как проверить |
|---|---|---|---|
| 1 | `bash scripts/agent_flow/tests/test_decomposed_watchdog.sh` — 5/5 pass | воркер | raw-вывод теста в PR |
| 2 | `bash scripts/agent_flow/install.sh --list-files` содержит `agent-flow-decomposed-watchdog.sh` | воркер | вывод команды |
| 3 | На хосте после `install.sh` есть cron-job «Agent Flow Decomposed Watchdog (ADR-0052)» в devops-профиле, every 4h, no_agent | devops | `cat /home/builder/.hermes/profiles/devops/cron/jobs.json` |
| 4 | Backlog sweep: для 8 мёртвых детей из таблицы в карточке `t_a054d54c` — manual `kanban_comment` с marker'ом `decomposed-Nd-no-pickup` | воркер (backlog sweep) | `SELECT task_id, body FROM task_comments WHERE body LIKE '%decomposed-%-no-pickup%'` — должно быть 8 строк |
| 5 | Первый боевой тик `agent-flow-decomposed-watchdog.sh` с live-БД не падает (exit 0 или 2) | devops | лог `/tmp/agent-flow-decomposed-watchdog.log` |
| 6 | ADR-0052 в `docs/adr/` | воркер | `ls docs/adr/0052-*` |

## 7. Что НЕ покрывает

- **PR с mergeCommit в develop, но root-карточка в `todo`** — это другой паттерн («done-не-archived»), под watch другого watchdog'а или ручного triage.
- **Reassign ребёнка на другой профиль** — не делаем (см. §2.3).
- **Дедуп по `(parent_id, child_id)`** — нет, marker пишем в `task_comments` ребёнка, а не root'а. Если у ребёнка несколько родителей (теоретически) — marker всё равно один в сутки, idempotency работает.

## 8. Verification log (для будущего надзора)

- 2026-09-07 — карточка создана по итогам nightly-review `t_bfd19ffb` (03:13Z), root cause разобран в `t_a054d54c` body §1.1-§1.3.
- Боевой прогон на хосте — **не выполнялся** на момент написания ADR. До первого боевого тика статус «работает» ставить нельзя (принцип честного FAIL, ADR-0018).
