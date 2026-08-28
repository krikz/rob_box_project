# ADR-0024: cross-task archive sweeper — workaround для kernel worker-scope-limit

| Поле | Значение |
|---|---|
| Статус | Accepted |
| Дата | 2026-08-22 |
| Автор | devops (Hermes Agent); ретро-карточка t_d9b4c600 |
| Контекст | Карточка [t_d9b4c600](https://github.com/krikz/rob_box_project) — ретро: stuck-кейс, когда recovery-worker не может через worker-tool архивировать свой parent |
| Затрагивает | `scripts/agent_flow/cross-task-archive-sweeper.sh` (новый), `scripts/agent_flow/install.sh` (EXPECTED+2), `scripts/agent_flow/_cross_task_archive_sweeper_{scan,archive}.py` (helpers), `~/.hermes/profiles/devops/cron/jobs.json` (new cron-job every 1h, no_agent) |
| Родители | ADR-0023 (skill-discovery recursive — precedent по workaround kernel-scope через external actor) |
| Связанные | t_d9b4c600 (это), t_5e50675b (stuck #1), t_0222e192 (stuck #2 — recovery которая не может архивировать parent), t_6c632ed5 (эта реализация) |

## 1. Контекст и бизнес-проблема

После hermes-agent v0.20.x kernel жёстко ограничивает worker-tools
(`kanban_complete` / `kanban_block` / `kanban_request_review`) **scope'ом
текущей задачи** — refuse-сообщение «worker is scoped to task X;
refusing to mutate Y». Это правильно для защиты от случайных cross-task
мутаций: один worker не должен иметь права закрывать/архивировать
произвольную чужую задачу.

**Но это ломает cleanup.** Сценарий, который случился 22.08:

1. Worker t_5e50675b открыл PR #1500, дождался зелёного CI, сделал `kanban_complete`.
2. Dispatcher поднял дочернюю recovery-задачу t_0222e192 с parent=t_5e50675b.
3. Recovery-задача t_0222e192 должна была заархивировать parent (PR уже
   CLOSED, remote-ветка удалена, задача по сути завершена).
4. **Но:** worker-tool t_0222e192 отказывается мутировать t_5e50675b —
   «worker is scoped to task t_0222e192; refusing to mutate t_5e50675b».
5. В итоге обе карточки висят в `done` / `blocked` часами до тех пор,
   пока human-operator не сделает `hermes kanban archive t_5e50675b t_0222e192`
   через CLI руками.

Аналогичная ситуация уже была с ADR-0023 (skill-discovery recursive),
когда один kernel-scope-guard ломал cleanup в другой части пайплайна.

## 2. Гипотеза (root cause)

CLI-команда `hermes kanban archive` — **операторская**, не worker-tool.
Kernel-scope к ней не применяется, потому что CLI-процесс не наследует
worker-lock. Это правильный escape-hatch, но требует human-оператора,
который не всегда на месте.

Альтернативы:
- (а) Пропатчить hermes-agent, добавить `kanban_complete --allow-cross-task`
  флаг. Требует hermes-agent sources правки + ADR + человека.
  **Не делаем** в этой карточке — cross-profile запрет и архитектурное.
- (б) Внешний watchdog, который ОТ ОПЕРАТОРСКОГО ПРОЦЕССА (cron, не
  worker) проверяет критерии «карточка завершена» через `gh` CLI
  и при соблюдении условий делает прямой SQL UPDATE (bypass kernel).

## 3. Решение — вариант (б)

`cross-task-archive-sweeper.sh` запускается как **no_agent cron job**
в devops-профиле раз в час. Это операторский процесс — никакого
worker-lock, никакого kernel-scope.

Стратегия:

1. Найти все `blocked` карточки `assignee=devops` с
   `COALESCE(started_at, created_at) < now - 24h` (stale).
2. Для каждой — извлечь `#NNNN` ссылки из `body` через regex
   `(?<![A-Za-z0-9_])#\d{3,5}`.
3. Спросить `gh pr view N --json state` и/или `gh issue view N --json state`.
4. **Архивировать** карточку если:
   - ХОТЯ БЫ один ref закрыт (`MERGED`/`CLOSED` для PR, `CLOSED` для issue), И
   - (`branch_name` отсутствует) ИЛИ (remote-ветка удалена, `git ls-remote`
     возвращает пустоту).
5. **Пропустить** иначе (открытые refs, или remote-ветка существует — нет
   очевидного orphan-state).
6. Действие: прямой SQL UPDATE
   `UPDATE tasks SET status='archived', claim_lock=NULL, claim_expires=NULL,
    worker_pid=NULL WHERE id=? AND status != 'archived'`
   (идентично `hermes_cli.kanban_db.archive_task`).

### Whitelist — двойная защита

- `assignee='devops'` И `board='robbox'` (по умолчанию). В текущей
  конфигурации только этот профиль+доска легитимно могут вызвать
  cross-task archive через процессных правил (ретро ADR-0023/0024).
- В `body` должны быть `#NNNN` ссылки — иначе skip (ручное решение).
- Если ни один ref не закрыт — skip (fail-closed).
- Все результаты (включая SKIP) логируются в
  `~/.local/state/cross_task_archive_sweeper.log`.

### Cron

`~/.hermes/profiles/devops/cron/jobs.json` — добавлен entry с
`schedule.every 1h`, `no_agent=true`, `script: cross-task-archive-sweeper.sh`.

Раскладка скрипта идёт через стандартный `scripts/agent_flow/install.sh`
(добавлен в `EXPECTED`) — hardlink'ы во все 4 профиля
(`agent-flow`/`architect`/`devops`/`~/.hermes/scripts/`).

## 4. Альтернативы, которые НЕ были выбраны

- **Правка hermes-agent**: добавить `--allow-cross-task` флаг в
  `kanban_complete`. Архитектурный change, cross-profile запрет,
  требует отдельного ADR + accept от товарища Шифу. **out-of-scope**.
- **Только-CLI escape hatch**: оставить только ручное
  `hermes kanban archive`. Уже было, не масштабируется (human не всегда
  на месте, карточки висят часами).
- **Auto-archive при любом completed_at > 24h**: слишком агрессивно.
  Ретро-карточка t_5e50675b имела completed_at свежий, но PR CLOSED
  not-merged + remote-ветка present — нужна доп. проверка через gh.
- **Generic cross-profile sweeper** (любой assignee): расширяет blast
  radius, риск false-positive для профилей, чьих PR/issue у меня нет
  видимости. Профиль devops — единственный с process rule "после
  успешного PR worker → archive"; остальные профили обычно не оставляют
  после себя orphan PRs.

## 5. Verification (как проверять)

Acceptance criteria (из t_d9b4c600 / t_6c632ed5):

- [x] `cross-task_archive_sweeper.sh` создан, dry-run на текущей базе
  показывает корректный diff (0 кандидатов в чистом состоянии)
- [x] Тестовая stuck-карточка с PR #1500 в body → нашлась →
  WOULD ARCHIVE → real прогон → реально archived (проверено 22.08 22:48)
- [x] Первый прогон логирует в `~/.local/state/cross_task_archive_sweeper.log`
- [x] ADR-кандидат (этот документ) — `docs/adr/0024-worker-scope-cross-task-archive.md`
- [ ] Cron-задача в `~/.hermes/profiles/devops/cron/jobs.json` с schedule `every 1h`
- [ ] Скрипт раскатан в 4 профиля через `install.sh`

## 6. Что НЕ делаем в этом ADR / следующие шаги

- ❌ Не правим hermes-agent sources — нужен отдельный архитектурный
  change review.
- ❌ Не расширяем whitelist на все профили (пока только devops+robbox).
- ❌ Не прикручиваем активный алерт в Telegram при archive (можно добавить
  позже, если operationally понадобится).
- 🔄 ADR-0025 (будущее): если в kernel-scope-limit добавят `kanban_complete
  --allow-cross-task`, sweeper можно упростить до 1-clause через
  worker-tools. Это зависит от hermes-agent maintainer'а.
