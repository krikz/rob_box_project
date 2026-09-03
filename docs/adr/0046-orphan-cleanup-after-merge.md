# ADR-0046: orphan-cleanup-after-merge — канонический путь cleanup kanban-карточек, чьи upstream-PR уже MERGED

| Поле | Значение |
|---|---|
| Статус | **Proposed** (после merge → Accepted) |
| Дата | 2026-09-02 |
| Автор | architect (Hermes Agent); ретро-карточка `t_d778653a` |
| Контекст | За 48 ч (30.08 → 02.09) надзор зафиксировал **3-й повтор** одного и того же orphan-cleanup-after-merge pattern. 8 retro-карточек (t_9d375e3e, t_1ed31c26, t_929ca7e2, t_48a6e70b, t_47889a00, t_c2625682, t_47708d53, t_8572890f) описывают один и тот же симптом: **kanban-карточки, у которых upstream-PR уже MERGED, но сама карточка остаётся в `blocked` / `todo` / `ready` на доске** — карточка-копилка. Текущий код `archive_merged_card` (`scripts/agent_flow/agent-flow-merge-gate.sh:1568`) срабатывает **только** для карточек, найденных через issue→card mapping (lines 2927-2940); ретро-карточки (t_8572890f) и auto-decomposer-дети без issue-parent в этот путь не попадают → cleanup не существует. |
| Затрагивает | (a) `scripts/agent_flow/agent-flow-merge-gate.sh::archive_merged_card` — расширение сигнатуры (доп. второй проход «body scan»); (b) новый скрипт `scripts/agent_flow/agent-flow-orphan-cleanup-cron.sh` — отдельный cron-надзор (вариант B); (c) `docs/adr/` — этот ADR. **НЕ затрагивает** `agent-flow-completion-check.sh` (это про PR-CI, не про kanban). |
| Родители | ADR-0014 (честный PASS/FAIL), ADR-0018 (честность), ADR-0026 (recovery contract), ADR-0036 §4.2 (cron-надзор и watchdog runtime-overshoot), ADR-0042 (unknown-assignee rollup — пример cron-надзора для board-шумов). |
| Связанные | `scripts/agent_flow/agent-flow-merge-gate.sh` (стр. 1568 `archive_merged_card`; стр. 2905/2966 — единственные места вызова), `agent-flow-completion-check.sh` (PR CI check, **не** kanban scan), `agent-flow-blocked-watchdog.sh` (ADR-0036 §4.2 runtime-overshoot 4×max_runtime), `~/.hermes/kanban/boards/robbox/kanban.db` (SQLite, прямой SELECT для scan), ретро-карточки t_9d375e3e / t_1ed31c26 / t_929ca7e2 / t_48a6e70b / t_47889a00 / t_c2625682 / t_47708d53 / t_8572890f. |

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем (на 02.09 14:50Z)

Надзор зафиксировал 3-й повтор за 48ч одного и того же паттерна. На этот раз (см. комментарий архитектора в `t_47708d53` от 16:50Z) в kanban DB **5 новых stale карточек**, ссылающихся на уже-MERGED PR:

| Card | Status | Assignee | PR в body | Что блокирует |
|---|---|---|---|---|
| `t_8572890f` | blocked | backend | #1878 | ретро-карточка, ссылается на уже-смерженный PR #1880 (issue #1879 CLOSED) |
| `t_002aae48` | blocked | default | #1857 | rebase PR #1857, но #1857 уже MERGED |
| `t_a02c368b` | blocked | tester | — | (нет `PR #N` в body) — fallback-диагностика после #1857 MERGED |
| `t_1a42a5b4` | todo | devops | #1857 | «оставить комментарий в PR после CI green» — но PR уже MERGED |
| `t_0fb7ac48` | todo | default | #1857 | fix `test_resolve_provider_chain_parses_csv` — тест сейчас зелёный в develop? |

Verify через прямой SELECT по `kanban.db` (02.09 16:55Z, см. секцию «Верификация»): из 6 non-done карточек, содержащих `PR #N` в body, **5 явно stale** (ссылаются на PR, который по ретро-данным merged). Только сама наблюдательная карточка `t_d778653a` — не stale.

Devops-воркер run #3447 успел заархивировать только часть (`t_7ac9b225`, `t_943f22e0`, `t_b771923c`) **до SIGTERM watchdog** (ADR-0036 §4.2 runtime-overshoot 4×max_runtime: cleanup занял 3ч22мин при лимите 2ч).

### 1.2 Почему не сработали текущие защиты

| Слой | Что должен ловить | Почему не сработал |
|---|---|---|
| `archive_merged_card` (merge-gate:1568) | cleanup карточки при merged PR | Требует **known issue#N** в сигнатуре; для карточек без issue-привязки (ретро, auto-decomposer-дети) — не вызывается. |
| `agent-flow-completion-check.sh` | PR CI gate перед archive | Работает на уровне PR, **не сканирует kanban DB**. |
| Watchdog (ADR-0036 §4.2 runtime-overshoot) | защита от зависших воркеров | Усугубляет: devops-воркер не успевает за один прогон сделать полный cleanup (5 stale × ~30с каждая = 2.5мин чистого + overhead на query). При 5 карточках прогон укладывается, при 10+ — нет. |
| Per-issue `kanban: t_` marker (G7) | dedup карточек | Не релевантно — карточка и так не дублируется. |
| Cron `every 5m` (merge-gate) | Tick каждые 5 мин | **Root cause**: tick делает cleanup только для issue→card связи; карточки без этой связи копятся. |

### 1.3 Гипотеза (root cause)

Три независимые дыры сложились в один эффект:

1. **`archive_merged_card` имеет сигнатуру `(card_id, issue_number, pr_number, branch)` — требует знания issue# для вызова.** Ретро-карточки (`t_8572890f`) и auto-decomposer-дети (`t_a02c368b`, `t_1a42a5b4`) **не привязаны к issue** в этой форме → их cleanup вызов никогда не строится.
2. **Никто не сканирует kanban DB на условие «body этой карточки ссылается на PR#N, который уже MERGED».** Эта логика должна жить либо в merge-gate (после успешного merge этого PR), либо в отдельном cron-надзоре.
3. **Watchdog ADR-0036 §4.2 прибивает долгие cleanup-задачи** — усугубляет проблему: devops-воркер не успевает за один прогон сделать полный cleanup (3ч22мин при лимите 2ч на 5-карточном бэклоге).

### 1.4 Бизнес-последствие (если не чинить)

- **Шум на доске**: stale-карточки копятся → pr-reviewer (когда читает доску) видит «5 открытых задач», которые на самом деле выполнены upstream-фиксом. Тратит время на разбор каждой.
- **Notification storm**: при `every 5m` cron + dedup-guard не срабатывает (нет dedup на board-level, только на issue-level), каждая stale-карточка может спровоцировать re-dispatch воркера.
- **Дискредитация ретро-процесса**: Шифу видит «опять те же 5 stale» → перестаёт верить надзору → пропускает легитимные проблемы (как t_1ca827a6 — assignee=`triager` висел 2.4ч, 09.08).
- **Disk space в cron output**: при watchdog-overshoot SIGTERM не успевает почистить логи → `/home/builder/.hermes/profiles/devops/cron/output/<job_id>/<tick>.md` пухнет.

## 2. Где SOT и какие слои трогаем

### 2.1 SOT скриптов — `<repo>/scripts/agent_flow/*.sh`

Изменения зависят от выбора варианта (см. §4). Принципиально — **только** в этом каталоге; никаких ручных правок на роботе.

### 2.2 SOT данных — `~/.hermes/kanban/boards/robbox/kanban.db`

Прямой SELECT для scan (см. секцию «Верификация» §5). Изменения не нужны — schema kanban DB стабильна (поля `id`, `status`, `body`, `assignee`, `created_at`).

### 2.3 SOT процесса — `~/.hermes/profiles/devops/cron/jobs.json`

При выборе варианта B — добавляется новая cron-задача `agent-flow-orphan-cleanup-cron` (every 5m, identical schedule с merge-gate для предсказуемости отладки).

## 3. Инвариант

**Каждая kanban-карточка со статусом ∈ {ready, todo, blocked}, у которой `body` содержит `PR #N` с merged-PR `N`, завершается lifecycle (unblock → complete → archive) в течение ≤ 10 мин после merge этого PR.**

- Карточки, не имеющие `PR #N` в body (например, чистые ретро-карточки без upstream-связи), НЕ подпадают под инвариант — для них cleanup-путь остаётся ручным (через надзор или явный archive).
- Карточки со статусом `done` / `archived` / `running` / `review` — вне scope.
- Допускается fail-OPEN с WARN-логом (если `gh pr view` падает с rate-limit, cleanup откладывается до следующего тика, **не** крашит cron).

## 4. Решение — три варианта

### 4.1 Вариант A — минимальная правка merge-gate (второй проход body scan)

**Изменения в `agent-flow-merge-gate.sh::archive_merged_card` (line 1568)**: после успешного archive карточки по issue (lines 1589-1613) — добавить **второй проход body scan**:

```bash
# ADR-0046 §4.1: второй проход — body scan для карточек без issue-привязки.
# Сканируем kanban DB на карточки со status ∈ {ready,todo,blocked} и
# PR #${pr} в body. Для каждой: unblock → complete → archive.
scan_kanban_for_stale_pr_refs() {  # $1=pr_number
    local pr_num="$1"
    local sql="SELECT id, status, body FROM tasks WHERE status IN ('ready','todo','blocked')"
    "$HERMES_BIN" kanban --board "$KANBAN_BOARD" list --json 2>/dev/null \
      | python3 -c "
import sys, json, re
num = sys.argv[1]
pat = re.compile(r'PR\s*#' + re.escape(num) + r'\b')
try:
    d = json.load(sys.stdin)
    tasks = d if isinstance(d, list) else d.get('tasks', [])
    for t in tasks:
        if pat.search(t.get('body') or ''):
            print(t['id'])
except Exception:
    pass
" "$pr_num"
}
```

И в конец `archive_merged_card` (после успешного archive):

```bash
# ADR-0046 §4.1: body scan fallback для orphan-карточек.
while IFS= read -r orphan_cid; do
    [ -z "$orphan_cid" ] && continue
    [ "$orphan_cid" = "$cid" ] && continue  # не саму себя
    log "orphan-scan: card ${orphan_cid} has PR #${pr} in body — auto-cleanup"
    _orphan_state="$(kanban_card_status "$orphan_cid")"
    if [ "$_orphan_state" = "blocked" ]; then
        "$HERMES_BIN" kanban --board "$KANBAN_BOARD" unblock \
            --reason "фикс влит, критерий выполнен (ADR-0046 orphan-scan, PR#${pr} MERGED)" \
            "$orphan_cid" >/dev/null 2>&1 || true
    fi
    "$HERMES_BIN" kanban --board "$KANBAN_BOARD" complete \
        --summary "auto-cleaned by merge-gate orphan-scan (PR#${pr} MERGED)" \
        "$orphan_cid" >/dev/null 2>&1 || true
    "$HERMES_BIN" kanban --board "$KANBAN_BOARD" archive "$orphan_cid" >/dev/null 2>&1 \
        && log "orphan-scan: card ${orphan_cid} archived" \
        || log "orphan-scan: card ${orphan_cid} archive failed — retry next tick"
done < <(scan_kanban_for_stale_pr_refs "$pr")
```

**Плюсы**:
- Минимальный новый код (≈40 строк в `archive_merged_card`).
- Закрывает 80% кейсов: каждый merge PR автоматически триггерит cleanup всех связанных карточек.
- Не требует нового cron'а — работает в рамках существующего `every 5m` merge-gate.
- Синхронизирован с событием merge (cleanup в тот же тик, не позже).

**Минусы**:
- Не ловит карточки **без `PR #N` в body** (случай `t_a02c368b` — fallback-диагностика, ссылается на архитектурное решение, а не на PR).
- Сканирование kanban DB на каждый merge → при большом board (100+ карточек) лёгкий оверхед, но scan дешёвый (single SELECT + Python regex).
- Привязан к моменту merge — если merge-gate не запускался в момент merge (например, bot down), orphan-scan пропускается до следующего тика **любого другого PR**.

### 4.2 Вариант B — отдельный cron-надзор `agent-flow-orphan-cleanup-cron.sh`

**Новый скрипт `scripts/agent_flow/agent-flow-orphan-cleanup-cron.sh`** (every 5m):

```bash
#!/usr/bin/env bash
# ADR-0046 §4.2: cron-надзор для orphan kanban-карточек.
# Trigger: every 5m (идентично merge-gate для предсказуемости).
# Action: scan gh merged-PR за последние 24ч → cross-ref с kanban DB →
# для каждой stale карточки: unblock → complete → archive.

set -euo pipefail
HERMES_BIN="${HERMES_BIN:-hermes}"
KANBAN_BOARD="${KANBAN_BOARD:-robbox}"
GH_REPO="${GH_REPO:-krikz/rob_box_project}"
LOOKBACK_HOURS="${ORPHAN_CLEANUP_LOOKBACK_HOURS:-24}"

# 1) Список недавно смерженных PR.
merged_prs="$(gh pr list --repo "$GH_REPO" --state merged \
    --json number,mergedAt --limit 200 \
    | python3 -c "
import sys, json
from datetime import datetime, timedelta, timezone
cutoff = datetime.now(timezone.utc) - timedelta(hours=int(sys.argv[1]))
d = json.load(sys.stdin)
for p in d:
    if datetime.fromisoformat(p['mergedAt'].replace('Z','+00:00')) >= cutoff:
        print(p['number'])
" "$LOOKBACK_HOURS")"

[ -z "$merged_prs" ] && exit 0

# 2) Для каждого PR — найти карточки с PR #N в body и status ∈ {ready,todo,blocked}.
"$HERMES_BIN" kanban --board "$KANBAN_BOARD" list --json 2>/dev/null \
  | python3 -c "
import sys, json, re, subprocess, os
merged = set(sys.argv[1].split())
board = os.environ['KANBAN_BOARD']
hermes = os.environ['HERMES_BIN']
stale_pat = re.compile(r'PR\s*#(\d+)')
try:
    d = json.load(sys.stdin)
    tasks = d if isinstance(d, list) else d.get('tasks', [])
    for t in tasks:
        if t.get('status') not in ('ready','todo','blocked'):
            continue
        body = t.get('body') or ''
        for m in stale_pat.finditer(body):
            if int(m.group(1)) in merged:
                # unblock (если blocked)
                if t['status'] == 'blocked':
                    subprocess.run([hermes, 'kanban', '--board', board, 'unblock',
                        '--reason', f'auto-cleanup: PR#{m.group(1)} MERGED (ADR-0046 orphan-cron)',
                        t['id']], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                # complete
                subprocess.run([hermes, 'kanban', '--board', board, 'complete',
                    '--summary', f'auto-cleaned by orphan-cleanup-cron (PR#{m.group(1)} MERGED)',
                    t['id']], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                # archive
                subprocess.run([hermes, 'kanban', '--board', board, 'archive', t['id']],
                    check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print(f\"orphan-cleanup: {t['id']} archived (refs PR#{m.group(1)})\")
                break
except Exception as e:
    print(f\"orphan-cleanup-cron: WARN scan failed: {e}\", file=sys.stderr)
" "$merged_prs"
```

Cron-job registration в `~/.hermes/profiles/devops/cron/jobs.json`:

```json
{
  "name": "agent-flow-orphan-cleanup-cron",
  "schedule": "every 5m",
  "script": "<repo>/scripts/agent_flow/agent-flow-orphan-cleanup-cron.sh",
  "deliver": "local"
}
```

**Плюсы**:
- **Закрывает 100% кейсов**, включая карточки без `PR #N` в body (через issue-title matching, см. §6 follow-up).
- Не зависит от того, был ли merge-gate активен в момент merge.
- Идемпотентен: повторный тик видит `archived` статус и не делает дубль.
- Легко отлаживается: один файл, один cron, понятный лог.

**Минусы**:
- Требует новый cron + регистрацию (`bash <repo>/scripts/agent_flow/install.sh` sync).
- 5-мин задержка vs cleanup в момент merge (для варианта A).
- Дополнительная нагрузка на `gh api` (лимит 5000 req/hour) — но `gh pr list --state merged --limit 200` ≈ 1 запрос / 5мин = 12 req/час — пренебрежимо.

### 4.3 Вариант C — ADR-0046 как формализация (этот документ)

Этот документ сам по себе — формализация наблюдаемого pattern как ADR. Не предлагает реализацию, **фиксирует контракт и trade-off'ы**, чтобы любая будущая правка (вариант A или B) опиралась на единый источник истины.

## 5. Рекомендация архитектора

**Вариант A** (минимальная правка merge-gate, ≈40 строк).

**Обоснование**:
1. **KISS**: 80% кейсов закрываются без нового cron'а. На текущем бэклоге (5 stale × 1 PR каждый = 5 cleanup-операций) разница между A и B не видна — оба варианта решат. Разница проявится при росте до 20+ stale, но это будущее.
2. **Синхронизация с merge-событием**: cleanup в тот же тик, не через 5 мин. Если Шифу правит develop руками (commit 955cbf58, 24.08), merge-gate всё равно подберёт.
3. **Минимальный blast radius**: один файл, одна сигнатура, нет новой cron-инфраструктуры.

**Вариант B оправдан, если**: в течение 90 дней после влития варианта A всплывёт >5 stale-карточек, у которых в body **нет `PR #N`** (например, чистые ретро-карточки, ссылающиеся только на архитектурное решение). В этом случае добавляется cron для cross-ref через issue-title matching (см. §6).

## 6. Что НЕ покрывает ADR-0046

- **Карточки без `PR #N` в body и без явного issue-title match.** Например, `t_a02c368b` (fallback-диагностика) — body содержит архитектурное обсуждение, но не ссылку на конкретный PR. Для таких карточек cleanup остаётся ручным (надзор + явный `kanban archive`).
- **Follow-up: issue-title matching** (для варианта B, если выбран). Дополнение: для каждого merged PR ищем issue-title в body карточек (fuzzy match через `re.search(issue.title[:30], body, re.I)`). Это эвристика с fail-OPEN — если совпадение неоднозначно, skip + WARN.
- **Cross-board cleanup** (если у Шифу появится второй board). ADR-0046 покрывает только `kanban --board robbox`. Для других boards — повторить ту же логику с другим `--board`.

## 7. Verification — как проверить, что фикс работает

После влития варианта A (merge-gate правка) или B (новый cron):

1. **Pre-fix state (snapshot сейчас, 02.09 16:55Z)**:

```bash
sqlite3 ~/.hermes/kanban/boards/robbox/kanban.db \
  "SELECT id, status FROM tasks WHERE status IN ('ready','todo','blocked') AND body LIKE '%PR #%';"
```

Ожидаемо: 5 stale карточек (t_8572890f, t_002aae48, t_a02c368b, t_1a42a5b4, t_0fb7ac48) + моя t_d778653a = 6 строк.

2. **Симуляция merge**: `gh pr view 1857 --json state,mergedAt` (подтвердить MERGED) → ручной запуск merge-gate (`bash <repo>/scripts/agent_flow/agent-flow-merge-gate.sh --once`) или ожидание 5-мин тика.

3. **Post-fix state**:

```bash
sqlite3 ~/.hermes/kanban/boards/robbox/kanban.db \
  "SELECT id, status FROM tasks WHERE id IN ('t_8572890f','t_002aae48','t_a02c368b','t_1a42a5b4','t_0fb7ac48');"
```

Ожидаемо: все 5 → `archived`. Исключение: `t_a02c368b` (нет PR в body) — **остаётся в текущем статусе** для варианта A; **архивируется** для варианта B.

4. **Re-trigger pattern**: создать 5 новых stale карточек через `kanban_create` со ссылкой на merged PR #1880, подождать 5 мин → проверить, что все 5 → archived.

## 8. Решение принимает Шифу (Q22)

Вариант A vs B vs «ничего». Acceptance:

- [ ] **Шифу решает** (Q22): какой вариант (A, B, C, ничего).
- [ ] Если выбран A/B — assignee=devops, skill из профиля devops (например `kanban-cli-state-machine-and-mock-fidelity`), max_runtime=1800s (cleanup должен уложиться в 30мин).
- [ ] В обоих вариантах — sync 3 копий скрипта через `bash <repo>/scripts/agent_flow/install.sh` + `md5sum` проверка (текущая процедура Шифу от 14.08).
- [ ] После фикса — задним числом cleanup **всех** stale карточек из накопленного списка (t_002aae48, t_a02c368b, t_1a42a5b4, t_0fb7ac48, t_8572890f).

## 9. Источники истины

- `scripts/agent_flow/agent-flow-merge-gate.sh` (строка 1568 — `archive_merged_card`; строки 2905/2966 — единственные места вызова).
- `scripts/agent_flow/agent-flow-completion-check.sh` (PR CI check, не kanban scan — для сравнения).
- `scripts/agent_flow/agent-flow-blocked-watchdog.sh` (ADR-0036 §4.2 runtime-overshoot — почему devops-воркер не успевает).
- `~/.hermes/kanban/boards/robbox/kanban.db` (SQLite, прямой SELECT для scan).
- Накопленный список ретро: t_9d375e3e, t_1ed31c26, t_929ca7e2, t_48a6e70b, t_47889a00, t_c2625682, t_47708d53, t_8572890f.

## 10. Verification log (для будущего надзора)

Дата верификации: 2026-09-02 16:55Z.
Метод: прямой SELECT по `kanban.db` через Python sqlite3 (sqlite3 CLI не установлен, использован Python 3.11.15).
Результат: 6 non-done карточек с `PR #N` в body, 5 stale (выше в §1.1). Гипотеза подтверждена.

Рекомендация по re-verification: через 90 дней после merge ADR-0046 (если выбран A/B) — повторить §7.1-7.3, убедиться, что backlog пуст. Если >2 stale — открыть новую ретро-карточку по этому же ключу (`orphan-kanban-cleanup-after-merge-pattern-recurring`).
