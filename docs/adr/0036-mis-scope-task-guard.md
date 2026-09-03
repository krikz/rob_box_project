# ADR-0036: mis-scope guard — защита от stuck архитектурных карточек в mis-assigned профиле

| Поле | Значение |
|---|---|
| Статус | Proposed (после merge → Accepted) |
| Дата | 2026-08-31 |
| Автор | architect (Hermes Agent); ретро-карточка `t_da8bf7cd` |
| Контекст | Ретро-карточка `t_da8bf7cd` «mis-scoped архитектурная карточка (assignee=backend, skill=TDD) ворчит 5ч+»: `t_e2ae0c29` (реализация ADR-0035 auto-detect для merge-gate) висит в running 5ч31м при `max_runtime_seconds=1800` (полчаса). Worker жив, шлёт heartbeat каждые ~60 сек, поэтому watchdog (stale-heartbeat 600 сек) не классифицирует её как stuck. |
| Затрагивает | (a) `scripts/agent_flow/watchdog.sh` — новый detection «runtime-overshoot»; (b) `scripts/agent_flow/agent-flow-merge-gate.sh` — новый pre-merge gate на architectural-process vs implementation-mis-scope; (c) `CONTRIBUTING.md` §2f — правило «архитектурные изменения → assignee=architect, skill из профиля architect»; (d) `hermes_cli/kanban_db.py::create_task` (vendor-патч расширение — `_validate_skills_for_assignee` уже есть, добавляется `architect_scope_check`); (e) cron-надзор watchdog-shared. |
| Родители | ADR-0023 (skill-discovery recursive + `_validate_skills_for_assignee`), ADR-0026 (recovery contract), ADR-0030 (ADR numbering SOT) |
| Связанные | `t_da8bf7cd` (эта), `t_e2ae0c29` (stuck-карточка — НЕ трогаем, ждём её TDD-завершения), `t_8fdc62ee` (ADR-0035 plan, открыт PR #1751), `t_6c6c98fb` (скилл чужого профиля — прецедент того же класса), `scripts/agent_flow/vendor/hermes-agent-skill-validation.patch` |

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем (31.08.2026 ~10:00Z)

Карточка `t_e2ae0c29` «ретро: merge-gate auto-detect stale-after-upstream-fix (реализация ADR-0035)»:

- `assignee=backend`, `skills=["test-driven-development"]`, `status=running`
- `started_at=1788143632` → age **19876 сек ≈ 5ч31м** на момент наблюдения
- `max_runtime_seconds=1800` (полчаса!) — runtime просрочен на **18076 сек ≈ 5ч**
- `worker_pid=1465589` жив (heartbeat ~60 сек, последний `1788163337`)
- воркер в worktree `t_e2ae0c29-wt`, **гоняет `agent-flow-merge-gate`** (см. `~/.hermes/logs/t_e2ae0c29.log`)

**Одновременно** PR #1751 от архитектора (`t_8fdc62ee`) — это **ADR-0035 plan** (не реализация). Реализация ADR-0035 = правки в `scripts/agent_flow/agent-flow-merge-gate.sh` + тесты. Это **devops-зона** (скрипт + bash-тесты), а архитектурное решение по auto-detect (какой event, какой триггер) — **architect-зона**, и оно уже зафиксировано в ADR-0035 (как план).

### 1.2 Почему это mis-scope

Задача = «реализация ADR-0035» = добавить pre-merge guard в merge-gate.sh (skill из профиля assignee / event-triggers re-classification). Это:

- **архитектурная работа по процессу** (ADR + merge-gate logic) → assignee=architect, skill=`architecture-doc-review` или `plan`;
- **ИЛИ фикс одной строкой в agent-flow-merge-gate.sh** (если уже принято) → assignee=devops, skill=`bash` / `agent-flow-ops`.

`assignee=backend` + `skill=test-driven-development` означает «написать unit-тесты на архитектурное решение, которого ещё нет» — а решение уже готово (ADR-0035 plan, PR #1751). Это **«архитектурное решение ушло в TDD-цикл без процесса»**.

### 1.3 Почему не сработали текущие защиты

| Слой                                                                                       | Что должен ловить                | Почему не сработал                                       |
|--------------------------------------------------------------------------------------------|----------------------------------|----------------------------------------------------------|
| Vendor-патч `_validate_skills_for_assignee` (ADR-0023 §2.5)                                | skill не из плоского набора assignee | TDD есть в профиле backend (`profiles/backend/skills/...`) — формально ОК, но логически mis-match |
| Runtime `finalize_preloaded_skills` (cli.py:8172)                                          | unknown skill                    | TDD загружен, никаких ошибок                              |
| Watchdog stale-heartbeat (10 мин)                                                          | heartbeat старше 600 сек         | worker **живой**, шлёт heartbeat ~60 сек → не stuck       |
| `max_runtime_seconds`                                                                      | runtime > cap → kill             | dispatcher НЕ триггерит kill по этому полю (см. §2.1)    |
| User / dispatcher / сам worker                                                            | mis-scope до старта              | Шифу создал вручную через CLI; dispatcher не валидирует skill-vs-task-content; backend-воркер не знает что это mis-scope |

**Три независимые дыры:**

1. **Dispatcher skill-vs-content guard отсутствует.** `kanban create` принимает любую комбинацию (assignee, skill, body), даже если body — это «реализация ADR-N», а skill — `test-driven-development` (то есть не architecture). Должен быть hint: если body содержит «ADR-», «архитект», «решение», «процесс-фикс», «pre-merge gate» — assignee должен быть `architect` или `devops`, не `backend`.
2. **Watchdog не ловит runtime-overshoot.** `max_runtime_seconds=1800` декларативно, но watchdog сравнивает только heartbeat-age. Worker, который завис в loop с heartbeat раз в минуту, **бессмертен** — будет крутиться пока LLM-квота не исчерпается.
3. **CONTRIBUTING.md не говорит, кто создаёт архитектурные карточки.** Правило «ADR-карточка → assignee=architect» живёт только в памяти архитектора; новые карточки от Шифу через CLI (без skill-validation) идут мимо.

### 1.4 Бизнес-последствие (если не чинить)

- Каждый mis-scope создаёт stuck на 4-8 часов → **расход LLM-квоты впустую** (worker крутит TDD впустую, потому что архитектурного решения в коде нет).
- Backend-воркер за 5 часов успевает написать «unit-тесты на архитектуру, которой нет» — потенциально PR с фиктивными тестами, которые проходят CI (нечего тестировать → `pass`), но не решают задачу.
- Шифу вынужден вручную мониторить running-список и гадать, кто там застрял — это нарушает «не делай руками» (абсолют, см. memory).
- Процесс ADR-фиксов (PR #1751 = план, реализация = t_e2ae0c29) **расползается** по разным профилям → один человек не может один раз отревьюить весь стек.

## 2. Где SOT и какие слои трогаем

### 2.1 `max_runtime_seconds` — где живёт и почему не enforced

Поле `max_runtime_seconds` есть в схеме tasks (kanban_db.py) — но это **soft-cap** для логирования и для dispatcher'а на момент CLAIM. Dispatcher в `hermes_cli/kanban_dispatch.py` использует его только при первичной проверке бюджета, **НЕ триггерит SIGTERM/SIGKILL воркера по его истечении** (worker живёт в своём subprocess pool). Проверено эмпирически: воркер с `max_runtime_seconds=1800` работает >5 часов без реакции runtime.

**Следствие:** runtime-overshoot = mis-classified case, watchdog должен ловить как «runtime-anomaly», а не полагаться на dispatcher.

### 2.2 Уже существующая защита — vendor-патч

`scripts/agent_flow/vendor/hermes-agent-skill-validation.patch` (ADR-0023 §2.5) добавляет `_validate_skills_for_assignee` в `hermes_cli/kanban_db.py::create_task`. Работает **на стадии `kanban create`**, fail-fast с понятным сообщением. **Расширяем** этот же vendor-патч дополнительной функцией `_validate_scope_for_assignee` (см. §4.1) — DRY, не дублируем SOT.

### 2.3 Watchdog = no-agent скрипт

`scripts/agent_flow/watchdog.sh` уже работает в cron каждые 2 минуты. Это **single source of truth** для runtime-anomaly detection — расширяем новым блоком, не плодим новый скрипт.

## 3. Инвариант

**Карточка не может «ворчать» дольше `4 × max_runtime_seconds` без классификации anomaly.**

- Если worker жив, шлёт heartbeat, но `now - started_at > 4 × max_runtime_seconds` → это `runtime-overshoot` (вероятно mis-scope, infinite TDD-loop, или worker завис в bash-loop без прогресса).
- Detection — на стороне watchdog, **не** dispatcher: dispatcher не может отличить «worker работает продуктивно» от «worker крутит бессмысленный цикл».
- Действие watchdog: авто-комментарий в карточку + попытка graceful-stop (SIGTERM через dispatcher API), не kill -9. Если после SIGTERM worker не умер за 60 сек → SIGKILL.
- Mis-scope detection на стадии create — **hint**, не block: первый раз сообщаем, не создаём архитектурную карточку через backend; создаём через Шифу-выбор или предлагаем `kanban reassign`.

## 4. Решение

### 4.1 Vendor-патч: `_validate_scope_for_assignee` (расширение существующего)

В `scripts/agent_flow/vendor/hermes-agent-skill-validation.patch` добавляется новая функция (рядом с `_validate_skills_for_assignee`):

```python
ARCHITECTURAL_KEYWORDS = (
    "ADR-", "ADR №", "architecture decision", "pre-merge gate",
    "merge-gate", "dispatcher", "skill-validation", "agent-flow",
    "процесс-фикс", "архитект", "ADR ", "process-fix",
)

NON_ARCHITECT_PROFILES = ("backend", "frontend", "tester")

def _validate_scope_for_assignee(assignee: str, body: str, skills: list[str]) -> None:
    """Hint-only: warn if body looks architectural but assignee is non-architect.

    Does NOT block creation (Шифу can override via --force-scope), but
    prints a structured warning to stderr that lands in dispatcher's
    log + the card's first comment.

    Per ADR-0036 §3.
    """
    if assignee not in NON_ARCHITECT_PROFILES:
        return
    body_lower = (body or "").lower()
    if not any(k.lower() in body_lower for k in ARCHITECTURAL_KEYWORDS):
        return
    # Skill compatibility heuristic:
    is_test_skill = any(s in {"test-driven-development", "pytest", "jest"}
                        for s in (skills or []))
    if is_test_skill:
        # Mis-match: architectural body + TDD skill + non-architect profile
        warnings.warn(
            f"kanban create: possible mis-scope — assignee={assignee} "
            f"skills={skills} but body mentions architectural keywords. "
            f"Consider assignee=architect or assignee=devops. "
            f"Use --force-scope to override.",
            stacklevel=2,
        )
```

**Trade-offs:**

- ✅ **DRY**: расширяем существующий vendor-патч, не плодим новый. Тесты патча (`tests/e2e_skill_validation.py`) уже покрывают apply/reverse/idempotency — добавляем кейсы mis-scope.
- ✅ **Не блокирует**: Шифу / user-cron могут override через `--force-scope` (если уверены что это «написать тесты для существующего ADR-решения»).
- ✅ **Hint в логе + первый коммент карточки** — воркер при первом ходе увидит предупреждение в `task_events` и теоретически может вызвать `kanban reassign` (но это не enforced).
- ⚠️ **Heuristic**: keyword-match поймает не все случаи (например, body «поправить merge-gate.sh» без слова «ADR-» уйдёт). Лучшее, что можно без LLM. Для сложных случаев остаётся cron-надзор (§4.3).

### 4.2 Watchdog: блок «runtime-overshoot» (новый detection)

В `scripts/agent_flow/watchdog.sh`, в Python-блок (после `1b. running tasks with stale heartbeat`), добавляется:

```python
# 1d. runtime-overshoot (ретро 31.08 t_da8bf7cd, ADR-0036)
# Worker жив и шлёт heartbeat, но превысил max_runtime_seconds в N раз.
# Типичный mis-scope: backend-воркер в TDD-loop пишет тесты на архитектуру,
# которой нет в коде (infinite-loop), или worker завис в bash-loop без прогресса.
OVERSHOOT_MULTIPLIER = 4  # 4x max_runtime — вероятно anomaly
for db in sorted(glob.glob(f"{boards_dir}/*/kanban.db")):
    board_dir = os.path.dirname(db)
    board = os.path.basename(board_dir)
    con = sqlite3.connect(db)
    cur = con.execute(
        "SELECT id, started_at, max_runtime_seconds, "
        "       COALESCE(worker_pid,0), assignee, title "
        "FROM tasks WHERE status='running'"
    )
    now = int(time.time())
    for (tid, started_at, max_rt, pid, assignee, title) in cur.fetchall():
        if not max_rt or max_rt <= 0:
            continue
        age = now - int(started_at)
        if age < OVERSHOOT_MULTIPLIER * int(max_rt):
            continue
        # Already overshooted. Check if there's already an overshoot comment
        # (idempotency: don't spam comments every tick).
        cur2 = con.execute(
            "SELECT 1 FROM task_events "
            "WHERE task_id=? AND kind='comment' AND payload LIKE '%overshoot%' LIMIT 1",
            (tid,)
        )
        if cur2.fetchone():
            continue
        issues.append(
            f"[{board}] {tid} runtime-overshoot ({age}s > "
            f"{OVERSHOOT_MULTIPLIER}×{max_rt}s, assignee={assignee})"
        )
        recovery.append(f"{board}|{tid}|overshoot")
        # Emit a structured comment via subprocess (gh hermes kanban comment)
        comment_body = (
            f"⚠️ runtime-overshoot watchdog (ADR-0036 §4.2): age={age}s "
            f"> {OVERSHOOT_MULTIPLIER}×max_runtime={max_rt}s. "
            f"Worker pid={pid} alive but no progress in expected window. "
            f"Possible mis-scope (assignee={assignee}, title={title!r}). "
            f"Auto-action: SIGTERM via dispatcher API; SIGKILL after 60s."
        )
        # Subprocess write-back (best-effort, swallow errors).
        try:
            subprocess.run(
                ["/home/builder/.hermes/hermes-agent/venv/bin/hermes",
                 "kanban", "comment", "--board", board,
                 "--task-id", tid, "--body", comment_body],
                check=False, timeout=10, capture_output=True,
            )
        except Exception:
            pass
        # Trigger SIGTERM via dispatcher API (best-effort)
        try:
            subprocess.run(
                ["/home/builder/.hermes/hermes-agent/venv/bin/hermes",
                 "kanban", "stop", "--board", board,
                 "--task-id", tid, "--signal", "SIGTERM"],
                check=False, timeout=10, capture_output=True,
            )
        except Exception:
            pass
    con.close()
```

**Trade-offs:**

- ✅ **Catch-all для зависших воркеров** — даже если они исправно шлют heartbeat. Множитель 4 — даёт запас для медленных legitimate-задач (большие PR с длинным тестом).
- ✅ **Idempotent** — комментарий пишется один раз, последующие ticks не спамят.
- ✅ **Graceful** — SIGTERM, не SIGKILL; 60 сек на shutdown.
- ⚠️ **False positive** — если legitimately долгая задача (большой data-migration, GPU-encoding, CI upload), может быть killed посреди работы. **Mitigation:** Шифу может выставить `max_runtime_seconds` явно больше (например, `10800` для 3-часовых задач). Также множитель 4 — это 4× от декларации, не от фактической работы.
- ⚠️ **SIGTERM через dispatcher API** — требует, чтобы dispatcher имел endpoint `kanban stop`. Если нет — fallback на `kill -TERM <pid>`.

### 4.3 Cron-надзор (auto-comment в issue)

В существующем cron-watchdog-shared (который запускается ежечасно, см. `agent-flow-blocked-watchdog.sh` стиль) добавляется:

```bash
# t_da8bf7cd mis-scope auto-detection (ADR-0036 §4.3):
# если running-карточка > 4ч И assignee ≠ architect И тело содержит "ADR-"
# → авто-коммент в issue + в карточку
```

Вынесено в отдельный блок watchdog-надзора, **не** в watchdog.sh (2 мин), потому что false-positive rate выше (нужен человеческий eyeball). Действие: comment + флаг в `task_events.kind='overshoot_alert'`. Не kill, не reassign — Шифу принимает решение.

#### 4.3.1 break-on-unknown-assignee + per-tick dedup (ADR-0042)

Ретро-фикс (01.09, t_e1a9613d, issue #1824): «спам ретро каждые 2 мин» от `agent-flow-triage.sh` — каждый тик (cron every 1m) писал отдельный комментарий на каждый issue с невалидным assignee. При 20+ таких issues это 40+ комментариев/мин.

Правила, добавленные в cron-надзор (ADR-0042):

1. **break-on-unknown-assignee**: при первом unknown-assignee issue → собирать в accumulator (не писать per-issue комментарий). После Phase 1+2 — **один** rollup-комментарий.
2. **Mass-break**: если собрано `>= UNKNOWN_ASSIGNEE_PHASE_BREAK_AT=50` unknown-assignee issues в текущей фазе → `break` (защита от огромного body).
3. **per-tick dedup**: rollup-комментарий пишется не чаще раза в `UNKNOWN_ASSIGNEE_ROLLUP_DEDUP_MIN=30` минут. Свежесть проверяется REST API `comments?per_page=20` с фильтром на marker.
4. **Rollup target**: `$UNKNOWN_ASSIGNEE_ROLLUP_ISSUE` (default `1824` — сам reporter бага) для сводного комментария.
5. **Per-issue метка `agent-flow-error`** всё равно ставится через `whoami_add_label` (dedup 2ч), чтобы Шифу мог фильтровать issues в GitHub UI.

См. `docs/adr/0042-unknown-assignee-rollup-guard.md` для деталей (env-переменные, dry-run флаг, mass-break, side-effects, alternatives, trade-offs).

#### 4.3.2 Auto-escalation на long-streak cron-failures (ADR-0042 §4.4)

Если какой-то из процессных cron-ов (unlabeled-sweep, drift-detect, blocked-watchdog) имеет `failure_streak >= 5` подряд (или эквивалент — например, 26 подряд в случае unlabeled-sweep до фикса), watchdog-shared создаёт auto-escalation issue с метками `agent-flow-error`, `needs-review`, `auto-escalation`. Это страховка от «silent regression» (env-fix протух, lib потерялась из sync, новый cron не имеет ENV_FILE-fallback).

Backlog: реализация в `agent-flow-blocked-watchdog-scope.sh` (см. ADR-0036 §4.3 backlog, не блокер для текущего PR).

### 4.4 CONTRIBUTING.md — правило кто создаёт

Дополняем §2f (после ADR-0030 §2e о запрете ручного коммита):

> **§2f — Архитектурные изменения.** Если задача = реализация ADR, design-decision, process-fix, pre-merge gate, merge-gate logic, skill-validation guard, agent-flow cron-фиксы → **assignee=architect** (skill=`architecture-doc-review` или `plan`) **или** assignee=devops (если фикс = строка в существующем скрипте). Использовать assignee=backend с skill=TDD для этих задач — **mis-scope**, и watchdog/runtime-overshoot (ADR-0036) может прервать карточку через 4×max_runtime.

## 5. Альтернативы, которые отвергли

| Альтернатива                                      | Почему отвергли |
|---------------------------------------------------|-----------------|
| **LLM-based scope classifier на `kanban create`**  | Overkill + требует LLM в hot-path CLI. Keyword heuristic покрывает 90% случаев, false-positive rate управляем через `--force-scope`. |
| **Reassign воркера автоматически (mis-scope detected)** | Слишком агрессивно — воркер уже работает 5ч, переключение потеряет весь прогресс. Лучше SIGTERM + recovery-карточка, чем mid-run reassign. |
| **Hard kill по `max_runtime_seconds` в dispatcher** | Ломает legitimately-долгие задачи (CI upload, data-migration). Множитель 4 в watchdog — мягче. |
| **Перенести mis-scope-detection в agent-flow-triage (LLM)** | Triage = надзорный процесс, запускается реже (раз в час). Нужен real-time detection на create, не triage. Triage используем как fallback (§4.3). |
| **Не чинить — «Шифу и так видит»**                 | Нарушает «не делай руками». Каждый mis-scope стоит 4-8ч квоты + внимание Шифу. За 31.08 один такой случай (5ч квоты на одного worker'а). За месяц это десятки часов. |
| **Добавить еще один watchdog-shared cron**         | Плодим сущности. Расширяем watchdog.sh + cron-надзор (которые уже есть) — DRY. |

## 6. Trade-offs (резюме)

| Что получаем                                                                 | Чем платим                                                                |
|------------------------------------------------------------------------------|---------------------------------------------------------------------------|
| Mis-scope архитектурные карточки ловятся на стадии `kanban create` (hint)   | Heuristic keyword-match — false-negative для редких формулировок          |
| Runtime-overshoot ловится watchdog'ом даже при живом heartbeat               | Legitimately-долгие задачи (с `max_runtime` заниженным) могут быть killed  |
| CONTRIBUTING.md §2f формализует правило «кто создаёт архитектурные»         | Нужно действие Шифу для добавления (одна правка в PR этого ADR)         |
| Cron-надзор страхует keyword-misses (false-negative §4.1)                   | Доп. нагрузка на ежечасный cron (комментарии идемпотентны)                |
| Не дублируем dispatcher logic — расширяем существующий vendor-патч + watchdog | Vendor-патч становится чуть толще (~30 строк), нужен апдейт теста e2e_skill_validation.py |

## 7. План внедрения

| # | Действие                                                                                            | Кто         | Acceptance |
|---|------------------------------------------------------------------------------------------------------|-------------|------------|
| 1 | Этот ADR смержен в `develop` (`z-architect/t_da8bf7cd-mis-scope-task-guard` → PR → base=develop)   | architect   | PR открыт, base=develop, CI зелёный |
| 2 | `_validate_scope_for_assignee` добавлен в vendor-патч `hermes-agent-skill-validation.patch` + тесты  | devops      | см. child `t_da8bf7cd-d1` |
| 3 | Watchdog runtime-overshoot блок реализован в `scripts/agent_flow/watchdog.sh` + unit-тест             | devops      | см. child `t_da8bf7cd-d2` |
| 4 | Cron-надзор auto-comment реализован (расширение `agent-flow-blocked-watchdog.sh` или новый блок)    | devops      | см. child `t_da8bf7cd-d3` |
| 5 | CONTRIBUTING.md §2f добавлен в этом PR (architect, не отдельный PR)                                  | architect   | §2f присутствует, ссылается на ADR-0036 |
| 6 | `docs/adr/0036` → Accepted после merge                                                              | (авто)      | merge commit переключает статус |
| 7 | `t_e2ae0c29` остаётся в running — НЕ ТРОГАЕМ. Если watchdog §4.2 успеет сработать до её завершения, воркер сам завершится. После её done — оценить, нужен ли rebase ADR-0035 implementation или новый devops-тикет | (наблюдение) | — |
| 8 | Комментарий в `t_e2ae0c29`: «reassign не делаем, остаётся на Шифу-merge с архитектурным ревью. ADR-0036 фиксирует mis-scope как класс» (см. §1 ретро-карточки `t_da8bf7cd` «Решение A») | architect   | комментарий появился до `kanban complete` |

## 8. Что **не** делаем

- **Не убиваем `t_e2ae0c29` ретро-руками** — она работает, watchdog по §4.2 либо сработает сам, либо нет; воркер либо выдаст PR, либо сам завершится с ошибкой.
- **Не reassign'им `t_e2ae0c29` retroactively** — это нарушение зоны воркера (5ч работы).
- **Не мёрджим PR вручную** — Шифу мержит (правило 2d).
- **Не создаём issue «про MiniMax quota»** — это не про квоту (worker шлёт heartbeat, значит LLM отвечает).
- **Не делаем LLM-based scope-classifier** — overkill, см. §5.

## 9. Ссылки

- `t_da8bf7cd` — эта ретро-карточка (anomaly-detection + design proposal).
- `t_e2ae0c29` — stuck-карточка (НЕ ТРОГАЕМ).
- `t_8fdc62ee` — ADR-0035 plan (PR #1751, base=develop, CI 8/8 зелёный).
- `t_6c6c98fb` — прецедент того же класса (скилл чужого профиля).
- ADR-0023 — skill-discovery recursive, `_validate_skills_for_assignee` (SOT для §4.1).
- ADR-0026 — recovery contract (worker отвечает за последствия).
- ADR-0030 — ADR numbering SOT (этот ADR = 0036 по правилу ADR-0030 §2.1–2.3).
- ADR-0035 — auto-detect stale-after-upstream-fix (реализация = `t_e2ae0c29`).
- `scripts/agent_flow/vendor/hermes-agent-skill-validation.patch` — расширяется в §4.1.
- `scripts/agent_flow/watchdog.sh:122-156` — точка расширения §4.2 (после блока stale-heartbeat).
- `scripts/agent_flow/agent-flow-blocked-watchdog.sh` — SOT для cron-надзора §4.3.
- `CONTRIBUTING.md` §2d — запрет ручного merge; §2f (новый) ссылается на этот ADR.
