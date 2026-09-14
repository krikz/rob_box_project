# AF-0096 — worktree cleanup обязателен в crash-path и idempotency-key обязателен для create

**Дата:** 2026-09-14
**Автор:** architect worker (ретро-карточка t_01574514)
**Контекст:** `hermes-agent/hermes_cli/kanban_db_dispatch.py` (reclaim/crash-path) + `hermes_cli/kanban_db_workspace.py` (`_cleanup_worktree_workspace`) + `hermes_cli/kanban_db.py::create_task` (`idempotency_key`)
**Статус:** proposed
**Связанные тикеты:** t_01574514 (ретро), t_a5488e86, t_10b51b22 (overshoot 7200s — тот же цикл)
**Связанные ADR:** AF-0026 (recovery-card-contract), AF-0065 (merge-gate scan-all-prs race-guard), AF-0079 (nightly-review dedup), ADR-0090 §2.1 (запрет фиксов без evidence), ADR-0093 (ring buffer)
**Ретро-ключ:** `architect-crash-loop-worktree-already-checked-out`

## Контекст

В `kanban.db` на board `robbox` обнаружен кластер из **47 archived карточек**
с **идентичным** `branch_name = "z-{agent}/968-tool-intent-based-execution-fire-and-for"`
(все архивированы, ни одна не `active`, ни одна не привела к merge в upstream).

Из них **consecutive_failures=2 + `last_failure_error` = workspace: git worktree
add failed ... `'<branch>' is already checked out at '<old_path>'`** —
то есть архитектура упала **с той же ошибкой** во всех 47 случаях:

```
workspace: git worktree add failed for
/home/builder/rob_box_project/.worktrees/<new_tid>
on branch z-{agent}/968-tool-intent-based-execution-fire-and-for:
fatal: 'z-{agent}/968-tool-intent-based-execution-fire-and-for'
is already checked out at '/home/builder/rob_box_project/.worktrees/<old_tid>'
```

Хронология (UTC+02:00):
- первая карточка: t_47258be9, 2026-08-12 18:08:10 (success через 18 минут — единственная)
- первая failed: t_4584f317, 2026-08-13 11:19:35
- последняя failed: t_ff4462c4, 2026-08-13 13:01:22
- 80-минутное окно тика: 2026-08-13 11:19 → 13:01 (≈ 80 минут, **26 карточек** в окне, через одну — ~2-минутный ритм)

**Все 47 карточек имеют `idempotency_key=NULL`.** Создатель не пользовался
dedup-обёрткой `kanban-retro-create.sh` (которая содержит 4 слоя dedup).

## Root cause (двухкомпонентный)

### Корневая причина #1 — crash-path НЕ чистит worktree

В `hermes_cli/kanban_db_workspace.py:113` определена
`_cleanup_workspace(conn, task_id)`, которая в worktree-ветке (`_REMOVABLE_KINDS`)
вызывает `_cleanup_worktree_workspace(task_id, path, branch_name)` (строка 144).

Эта функция зовётся ТОЛЬКО из двух мест:
- `kanban_db.py:2619` — внутри `complete_task()` (success)
- `kanban_db.py:3584` — внутри `archive_task()` (manual archive)

**Crash-path в `kanban_db_dispatch.py` НЕ зовёт `_cleanup_workspace`**. После
`_reclaim_dead_workers` (строка 805) статус переводится в `ready`,
`claim_lock`/`claim_expires`/`worker_pid` обнуляются — и **worktree остаётся
висеть на диске с зарегистрированной веткой**.

Цепочка:

1. Worker-A на task_id=X захватывает карточку, делает `git worktree add
   .worktrees/X branch_name=B`, ветка B привязана к `<repo>/.worktrees/X`.
2. Worker-A падает (crash, SIGKILL, overshoot 7200s, OOM). PID исчезает.
3. Dispatcher при следующем тике вызывает `_reclaim_dead_workers`: X →
   `ready`, worktree **НЕ трогается**, ветка B остаётся зарегистрированной
   на `<repo>/.worktrees/X`.
4. **Следующий** worker (или новая карточка Y с тем же branch_name=B)
   пытается `git worktree add .worktrees/Y branch_name=B` → git возвращает
   `fatal: 'B' is already checked out at '.worktrees/X'` → `spawn_failed`.
5. Crash-loop повторяется, пока worktree X не будет снят вручную
   (`git worktree remove --force .worktrees/X` + `git branch -D B`) или
   пока не появится `git worktree prune` (который НЕ запланирован в нашем
   scheduler — см. ниже).

Гипотеза A из тикета (cleanup не запускается после crash) — **подтверждена**.

### Корневая причина #2 — explicit `branch_name` без `idempotency_key`

В `hermes_cli/kanban_db.py::create_task()` (строка 1221) `idempotency_key`
обрабатывается на строке 1291-1296 — если задан, существующий
`status != 'archived'` task возвращается вместо создания дубля.

`kanban-retro-create.sh` (см. `~/.hermes/scripts/kanban-retro-create.sh`,
SOT `scripts/agent_flow/kanban-retro-create.sh`) инжектирует ключ через
`--idempotency-key "retro:<KEY>"` и добавляет в body маркер
`ретро-key: <KEY>` для 4-слойного dedup.

**Создатель 47 карточек дёргал `hermes kanban create` напрямую с явным
`--branch-name`, БЕЗ `--idempotency-key`, БЕЗ `kanban-retro-create.sh`.**
Подтверждено:

```sql
sqlite> SELECT idempotency_key, COUNT(*) FROM tasks
        WHERE branch_name = 'z-{agent}/968-tool-intent-based-execution-fire-and-for'
        GROUP BY idempotency_key;
(None, 47)
```

Каждая карточка получила **новый task_id**, но **тот же branch_name** (явно
переданный в CLI). Если бы `branch_name` не передавался, dispatcher
сгенерировал бы его из `_project_branch_name(project_obj, task_id, title)`
(`projects_db.py:480`) — это `<slug>/<task_id>-<title-slug>`, то есть
**branch_name был бы уникален per-task_id**. Гипотеза B из тикета
(recovery наследует branch_name без пересчёта) — **подтверждена частично**:
не «наследует», а «создатель копипастит ту же строку».

### Корневая причина #3 — `_cleanup_worktree_workspace` пропускает dirty/unpushed

`kanban_db_workspace.py:173` — `_cleanup_worktree_workspace` имеет гард:

```python
if _worktree_is_dirty(str(wp)) or _worktree_has_unpushed_commits(str(wp)):
    _kb._log.info("Preserving worktree for task %s: dirty or unpushed work at %s", ...)
    return
```

Это правильно для **успешно завершённых** карточек (нужно сохранить черновик),
но в crash-path worker **никогда не push'ит коммиты** до завершения —
значит **любой crashed-worktree будет сохранён навсегда**, потому что
«unpushed commits». Это превращает **каждый** crash в **вечный зомби-worktree**.

Гипотеза C (worktree GC редкий) — **отвергнута** в её буквальной форме, но
выявлен **более сильный баг**: cleanup **никогда** не сработает для crashed
worker'а, потому что GC-гард рассчитан на success-case.

## Дополнительные находки

### `workspace_path = /home/builder/rob_box_project` (legacy)

В `task_events` для этих карточек `workspace_path = "/home/builder/rob_box_project"`
(главный репо, **не** `.worktrees/<task_id>`). Это legacy-формат, который
`_resolve_worktree_workspace` всё равно приводит к `.worktrees/<task_id>` через
`_anchored_worktree(repo_root, task_id, branch_name)` — но legacy-payload
засоряет event log и затрудняет диагностику.

### Защита от recreate

`hermes_cli/kanban_db.py:1054` (см. `_new_task_id`) явно говорит:
"Idempotency belongs to `idempotency_key`, not id uniqueness." — то есть
авторы **знают**, что создатели должны передавать ключ. Но enforcement
отсутствует.

### Текущее состояние (проверено 2026-09-14)

- `git worktree list` — **0 зомби-worktree** от этих 47 карточек
- Ветка `z-{agent}/968-tool-intent-based-execution-fire-and-for` —
  **существует**, имеет коммиты с merged PR #1207/#1208 в upstream
- `kanban.db` (`status != 'archived'` для branch_name=...) — **0 активных**
- Создатель карточек сейчас неактивен (cron-jobs.json не содержит скрипт-источник)

Цикл не активен сейчас, но **механизм не починен** — следующий crash любого
agent-flow worker'а с явным `branch_name` воспроизведёт паттерн.

## Решение

### Phase 1 (must, в этом PR или в одной из ближайших правок crash-path)

**P1.1** — `_reclaim_dead_workers` (`kanban_db_dispatch.py:805`) после
`UPDATE tasks SET status = 'ready'...` **обязан** запустить cleanup worktree
**до** следующего dispatch. Конкретно:

```python
# после строки 836 (после UPDATE на ready)
try:
    from hermes_cli.kanban_db_workspace import _cleanup_worktree_workspace
    if row["workspace_kind"] == "worktree" and row["workspace_path"]:
        _cleanup_worktree_workspace(
            row["id"], row["workspace_path"], row["branch_name"]
        )
except Exception as e:
    _kb._log.warning("crash-path worktree cleanup failed for %s: %s",
                     row["id"], e)
```

Но `kanban_db_workspace._cleanup_worktree_workspace` пропускает
dirty/unpushed — для crash-path нужен **отдельный** cleanup, который
**всегда** снимает worktree. Параметр `force=True` или новая функция
`_cleanup_crashed_worktree(path, branch_name)`.

**P1.2** — между crash-обнаружением и следующим dispatch добавить
`git worktree prune` для репозитория (`<repo>/.worktrees/`). Это безопасно
после явного `git worktree remove --force`, потому что prune только
очищает metadata для уже удалённых путей.

### Phase 2 (must, идемпотентность create)

**P2.1** — `kanban_db.py::create_task()` при `branch_name != None` и
`workspace_kind == "worktree"` **обязан** автогенерировать
`idempotency_key = f"branch:{project_id or 'global'}:{branch_name}"`, если
явный `idempotency_key` не передан. Это **дедуплицирует** любые попытки
создать две карточки с одним branch_name, не ломая существующих создателей.

**P2.2** — `hermes_cli/cli_kanban.py` (или эквивалентный CLI-parser) при
`kanban create --branch-name X` без `--idempotency-key` должен **warning в stderr**
и автоматически добавлять `--idempotency-key "branch:X"`. Это **non-breaking**
(ключ только помогает, не мешает), но создаёт видимый сигнал для будущих
ретро.

### Phase 3 (nice-to-have, process)

**P3.1** — `kanban-retro-create.sh` уже имеет 4 слоя dedup. Добавить
**слой 5: branch-name guard** — если у существующей не-archived карточки
тот же `branch_name`, что у новой, и `branch_name` не autogenerated (не
начинается с `wt/`), — SKIP с пометкой `branch-name-collision`.

**P3.2** — ADR-0014 (process rules) требует, чтобы **любая** kanban-card
от LLM-агента проходила через `kanban-retro-create.sh`. Добавить проверку
в `agent-flow-blocked-watchdog.sh` или новый `agent-flow-retro-card-hygiene.sh`
(каждые 30 минут): `SELECT * FROM tasks WHERE created_by LIKE 'cron-%'
AND idempotency_key IS NULL AND created_at > NOW - 1h` — если > 0,
alert в Telegram.

**P3.3** — для успешного complete убрать "unpushed-guard" в cleanup,
если карточка **не** запушила свои коммиты до crash. Альтернатива:
для worktree-карточек **требовать** push в `bin/open-pr` (или эквивалент)
**до** `kanban_complete`, тогда unpushed-guard становится irrelevant.

## Verification (как проверить, что фикс работает)

После применения P1.1+P1.2:

```bash
# 1. Создать test-task с явным branch_name, убить worker, дождаться reclaim
hermes kanban create --title "crash-loop-test" --assignee test \
  --branch-name "z-test/crash-loop-fixture" --workspace worktree
sleep 5; kill -9 <pid>

# 2. После reclaim: worktree должен быть снят
ls /home/builder/rob_box_project/.worktrees/t_<id> 2>&1  # NOT FOUND

# 3. Можно создать новую карточку с тем же branch_name
hermes kanban create --title "crash-loop-test-2" --assignee test \
  --branch-name "z-test/crash-loop-fixture" --workspace worktree
# → должно succeed (а не "already checked out")
```

После применения P2.1:

```bash
# Два create с одним branch_name, без idempotency-key:
hermes kanban create --title "dup1" --assignee test \
  --branch-name "z-test/dup-fixture" --workspace worktree
# → t_X1
hermes kanban create --title "dup2" --assignee test \
  --branch-name "z-test/dup-fixture" --workspace worktree
# → ДОЛЖЕН вернуть t_X1 (NOT создавать t_X2)
```

## Риски

- **P1.1** может стереть черновики, которые worker успел сделать перед
  crash'ем (если worker пишет в worktree без push). Это **намеренный**
  trade-off: лучше потерять черновик, чем воспроизвести crash-loop
  на 47 карточек. Если нужна гарантия черновика — P3.3 (push перед
  complete) обязателен.
- **P2.1** может сломать существующих создателей, которые передают
  branch_name для project-link (где ветка должна быть уникальна per-task-id).
  Mitigation: автогенерировать ключ **только** если `project_id is None`
  (legacy branch_name path). Project-link path уже создаёт уникальную
  ветку per-task-id через `_project_branch_name`.

## Что НЕ делаем (явно)

- **НЕ** удаляем существующие 47 архивных карточек — это данность истории,
  по ним видна динамика.
- **НЕ** блокируем `kanban create` для LLM-кронов — это легитимный use-case
  (разные тикеты, разные branch_names).
- **НЕ** авто-mergem PR — это решение Шифу.
- **НЕ** вводим hard-limit на retry без human unblock — текущий
  `consecutive_failures >= 2` уже срабатывает (`gave_up` event в логах),
  просто архивирует вместо блокировки.

## Связанные ретро

- t_01574514 — текущая (этот документ)
- t_a5488e86 — overshoot 7200s (worker-loop, та же корневая причина)
- t_10b51b22 — overshoot 7200s (тот же цикл)
- t_a8e82f2d → AF-0065 — merge-gate race-guard (похожий паттерн: scan
  создаёт дубль без dedup)
- t_55ab37d4, t_35ff29f1 → kanban-retro-create.sh (ADR-0079, ночной
  review dedup) — прецедент 4-слойного dedup, на который опирается P3.1