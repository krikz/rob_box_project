# ADR-0045: worktree base ref — `origin/develop` + drift-guard, чтобы каждый воркер стартовал с актуального develop

| Поле | Значение |
|---|---|
| Статус | Proposed (после merge → Accepted) |
| Дата | 2026-09-02 |
| Автор | architect (Hermes Agent); ретро-карточка `t_5e3c3e09`, issue #1887 |
| Родители | RECON-1571 (разведка и черновик воркера devops от 23.08), ADR-0030 (ADR numbering SOT — этот ADR = `0045` по правилу §2.2), ADR-0013 (incremental delivery — этот фикс ≤ 30 коммитов чтобы PR был mergeable), ADR-0018 (честный FAIL — никаких голословных «CI зелёный» без raw-вывода) |
| Контекст | Issue #1887: «2/3 свежих PR воркеров backend ушли с конфликтами (28 commits behind develop)». PR #1885 (`max_tokens/temp`) mergeable (0 commits behind), PR #1884 (`budget ретраи`) и PR #1886 (`planning-narration`) — оба `CONFLICTING`, 28 commits behind. Симптом повторяется уже третий раз за два месяца: RECON-1571 (23.08, 48–50 коммитов), ADR-0045 повтор #2 (02.09, 28 коммитов). Root cause НЕ изменился: `_ensure_git_worktree` берёт base из локального `HEAD` главного worktree, который отстаёт от `origin/develop`. Vendor-патч `hermes-agent-z-spawn-base-origin-develop.patch` лежит в репо с 23.08, но `git apply --check` отказывает на актуальном upstream `hermes-agent` → регресс бесшумный, никем не ловится. |
| Затрагивает | (a) `scripts/agent_flow/vendor/hermes-agent-z-spawn-base-origin-develop.patch` — **regen** под текущий `kanban_db.py` (anchor `@@ -7781` сместился из-за других воркеров); (b) `scripts/agent_flow/agent-flow-triage.sh` — добавить `git fetch origin develop --prune` в начале main (как у соседей `agent-flow-e2e-process.sh`, `agent-flow-merge-gate.sh`, `agent-flow-drift-detect.sh`); (c) `scripts/agent_flow/validate_branch_freshness.sh` — новый pre-PR hook (block если `git rev-list --count HEAD..origin/develop > 30`); (d) `scripts/agent_flow/install.sh` — добавить вызов validate_branch_freshness.sh в pre-PR секцию; (e) `docs/process-fix-roadmap.md` — фиксируем этот класс багов в roadmap. |
| Связанные | `t_5e3c3e09` (эта ретро-карточка), issue #1887 (репортёр), issue #1571 (первый репорт 23.08), PR #1573 (первый фикс 23.08 — `556ecc8c`), `RECON-1571-worktree-stale-base.md` (разведка), `scripts/agent_flow/vendor/hermes-agent-z-spawn-base-origin-develop.patch` (готовый, но не apply'имый патч), `scripts/agent_flow/tests/test_spawn_worktree_origin_develop_base.sh` (готовый регресс-тест), `t_07799ca7` (первый фикс воркером devops), `t_ecd43187` (ретро аналогичной серии 01.09 — triple-block), `t_62447edc` (ретро #1872/#1873 02.09) |

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем (02.09.2026 ~12:30Z)

3 свежих PR от воркеров backend (кластер за 30 мин, ~12:27–12:45):

| PR | Mergeable | Behind develop | Ahead develop |
|---|---|---|---|
| #1885 (`max_tokens/temp`) | ✅ MERGEABLE | **0** | 1 |
| #1884 (`budget ретраи`) | ❌ CONFLICTING | **28** | 1 |
| #1886 (`planning-narration`) | ❌ CONFLICTING | **28** | 1 |

Симптом идентичен RECON-1571 (23.08): ветка уходит в remote **с base 28 коммитов позади `origin/develop`** → merge-gate видит дивергенцию → `CONFLICTING`. Воркер #1885 (тот, у которого «получилось») вероятно сделал ручной rebase перед push или получил свежий worktree (например, после `agents_sleep.sh`, который делает fetch в начале каждого цикла — см. ADR-0018 §3).

Юзер прямо спрашивает (issue #1887): «почему воркер при старте не берёт самый свежий дев, ну или раз взял хреновый не сделал мерж с дева себе чтоб мержа такого не было».

### 1.2 Почему это повторный баг (а не новая аномалия)

`git log origin/develop --grep="worktree" -i --oneline` (02.09.2026):

```
556ecc8c fix(process #1571): worktree base ref — origin/develop + drift warning (#1573)   ← 23.08
a14aa52e fix(agent-flow): disable broken hermes-agent-spawn-worktree-precheck.patch vendor patch (#1589)
```

Первый фикс был сделан 23.08 (`556ecc8c`), воркер devops (`t_07799ca7`) закоммитил vendor-патч `hermes-agent-z-spawn-base-origin-develop.patch` + тест `test_spawn_worktree_origin_develop_base.sh`. Но **патч не живёт в `hermes-agent`** — потому что `git apply --check` падает на актуальном upstream:

```
$ cd /home/builder/.hermes/hermes-agent
$ git apply --check scripts/agent_flow/vendor/hermes-agent-z-spawn-base-origin-develop.patch
error: patch failed: hermes_cli/kanban_db.py:7781
error: hermes_cli/kanban_db.py: patch does not apply
```

Upstream `kanban_db.py` ушёл вперёд: параллельные воркеры пилили этот же файл (там валяются `kanban_db.py.orig`, `kanban.py.orig`, патчинг через 6+ других vendor-патчей). Якорь `@@ -7781` сместился. Без `agent-flow-regen-vendor-patch.sh` патч мертв, а у нас нет **никакого мониторинга**, что vendor-патчи отстали от upstream.

### 1.3 Три независимые дыры

| Дыра | Что должна ловить | Почему не сработала |
|---|---|---|
| **Vendor-патч drift**: `install.sh` применяет все `vendor/hermes-agent-*.patch` через `git apply --check` | Патч не apply'ится к upstream | `git apply --check` возвращает `error: patch failed` — `install.sh` его НЕ проглатывает, а выводит `ERROR patch does not apply cleanly — upstream moved`. **Но никто это не мониторит** в автоматическом режиме: `agent-flow-drift-detect.sh` сверяет только раскладку `EXPECTED` файлов (46 шт), не состояние applied-патчей. |
| **`agent-flow-triage.sh` без fetch**: создаёт карточки, не подтянув `origin/develop` | Воркер стартует на свежем develop | В отличие от `agent-flow-e2e-process.sh:2466`, `agent-flow-merge-gate.sh:573`, `agent-flow-drift-detect.sh:300` — в `agent-flow-triage.sh` нет `git fetch origin develop` в начале. **Это единственный из 4 cron-скриптов без fetch.** |
| **Pre-PR hook отсутствует**: PR может уйти с 28 коммитов behind | block до rebase | Нет `validate_branch_freshness.sh`. merge-gate ловит только E2E gates, honesty-claim, ADR-namespace (ADR-0030/0043); **freshness ветки — не его зона** (комина отвечает за merge-decision, а не за pre-PR quality-of-life воркера). |

### 1.4 Бизнес-последствие (если не чинить)

- **Каждый новый PR backend-воркера — лотерея**: если воркер стартанул после `agents_sleep.sh` (который делает fetch в начале cron'а) → свежий develop → mergeable; если после `agent-flow-triage.sh` (без fetch) → stale HEAD → 20–50 коммитов behind → CONFLICTING. **2/3 в этой серии проиграли**.
- **Расход квоты впустую**: воркер пишет 1 коммит фикса + push + PR, который невозможно смерджить. Шифу или воркеру-merge-gate приходится либо rebase-ить чужую ветку (а это «не делай руками»), либо переоткрывать карточку с новым воркером, который всё переделывает.
- **Рост ретро-карточек**: каждый такой случай → retrot-карточка в стиле `t_5e3c3e09` (эта), `t_ecd43187` (01.09, triple-block), `t_07799ca7` (23.08, оригинал). **Три повтора за 10 дней** — это уже класс, не аномалия.
- **Доверие к процессу падает**: Шифу вынужден вручную мониторить свежесть веток перед merge или делать rebase руками. Нарушение принципа «не делай руками» (абсолют, см. memory).

## 2. Где SOT и какие слои трогаем

### 2.1 Vendor-патч (уже существует, но не применим)

`scripts/agent_flow/vendor/hermes-agent-z-spawn-base-origin-develop.patch` — готовый фикс из RECON-1571. Содержит:

- `_resolve_worktree_base_ref(repo_root, base_branch="develop")` — best-effort `git fetch origin develop` (timeout 30с), возвращает `origin/develop` или fallback `"HEAD"` с `WORKTREE_BASE_FETCH_FAILED:` warning в stderr.
- `_warn_worktree_base_drift(repo_root, base_ref, threshold=10)` — `git rev-list --count HEAD..origin/develop`, при drift > threshold → `WORKTREE_BASE_DRIFT: HEAD..origin/develop = N commits` в stderr.
- Встраивание в `_ensure_git_worktree` else-ветке (когда `branch_name` НЕ существует).

**Что нужно**: **regenerate патча** под актуальный `kanban_db.py`. Утилита уже есть: `scripts/agent_flow/agent-flow-regen-vendor-patch.sh` (см. ADR-0042 §1 ссылка). После regen — `install.sh` подхватит через `apply_hermes_agent_patch` (idempotent, см. install.sh:331-364).

### 2.2 `agent-flow-triage.sh` — добавить fetch

В `main()` (или сразу после init-блока с переменными), **как у соседей**:

```bash
# Issue #1887 (ADR-0045 §2.2): fetch origin/develop перед созданием карточек,
# чтобы worktree-base был актуальным.
log "fetching origin/develop for up-to-date base..."
git fetch origin develop --prune 2>&1 | tee -a "$LOG" | head -5
git fetch origin "refs/heads/z-{agent}/*:refs/remotes/origin/z-{agent}/*" --prune 2>&1 | head -3
log "develop tip: $(git log origin/develop --oneline -1)"
```

Это **не меняет** поведение triage в happy-path: fetch — cheap operation (≤5 секунд), идемпотентен. В худшем случае (offline) — fetch падает, карточка создаётся как раньше. Семантика triage не ломается.

### 2.3 Pre-PR hook `validate_branch_freshness.sh` (новый)

Новый скрипт `scripts/agent_flow/validate_branch_freshness.sh`:

```bash
#!/bin/bash
# ============================================================================
# validate_branch_freshness.sh — pre-PR hook: блокирует push если ветка
# старше 30 коммитов behind origin/develop.
#
# Issue #1887 (ADR-0045): 2/3 свежих PR backend-воркеров ушли с 28 коммитов
# behind → CONFLICTING в merge-gate. Worker должен rebase ИЛИ
# пересоздать ветку от свежего origin/develop ДО push.
#
# Вызов: bash scripts/agent_flow/validate_branch_freshness.sh [BASE_REF]
#   BASE_REF (default: origin/develop) — что считать эталоном.
#   Env: MAX_BRANCH_BEHIND (default: 30) — допустимый drift в коммитах.
#
# Exit 0 — OK, exit 1 — branch stale.
# ============================================================================
set -euo pipefail

BASE_REF="${1:-origin/develop}"
MAX_BEHIND="${MAX_BRANCH_BEHIND:-30}"
THRESHOLD_FILE="${HOME:-/home/builder}/.hermes/state/branch_freshness_drift_max"

# Skip на merge-commit (CI merge) — он не считается "веткой воркера"
if [ "${GITHUB_EVENT_NAME:-}" = "pull_request" ] && \
   [ "${MERGE_COMMIT_INFERRED:-}" = "true" ]; then
    echo "[validate_branch_freshness] skip: merge-commit (CI)"
    exit 0
fi

HEAD="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo HEAD)"
BEHIND="$(git rev-list --count "${HEAD}...${BASE_REF}" 2>/dev/null || echo "?")"

if [ "$BEHIND" != "?" ] && [ "$BEHIND" -gt "$MAX_BEHIND" ]; then
    echo "FAIL: branch $HEAD is $BEHIND commits behind $BASE_REF (max $MAX_BEHIND)" >&2
    echo "       Rebase required:" >&2
    echo "         git fetch origin develop" >&2
    echo "         git rebase $BASE_REF" >&2
    echo "         git push --force-with-lease" >&2
    # Persist max-drift stat для cron-мониторинга (вне per-hook пути).
    mkdir -p "$(dirname "$THRESHOLD_FILE")" 2>/dev/null || true
    echo "$(date +%s) $HEAD $BEHIND" >> "$THRESHOLD_FILE" 2>/dev/null || true
    exit 1
fi
echo "[validate_branch_freshness] OK: $HEAD is $BEHIND commits behind $BASE_REF (max $MAX_BEHIND)"
exit 0
```

### 2.4 `install.sh` — вызов pre-PR hook

В существующий блок pre-PR секции (если есть; если нет — добавить новую секцию перед `hermes-agent vendor patches`):

```bash
# Pre-PR freshness hook (ADR-0045 §2.3)
if [ "${SKIP_BRANCH_FRESHNESS:-}" != "true" ] && \
   [ "${GITHUB_EVENT_NAME:-}" = "pull_request" ] && \
   [ "${MERGE_COMMIT_INFERRED:-}" != "true" ]; then
    if bash "$SCRIPT_DIR/validate_branch_freshness.sh" "origin/develop" 2>&1 | tee -a "$LOG"; then
        echo "  OK branch freshness"
    else
        echo "  FAIL branch freshness — see validate_branch_freshness.sh output" >&2
        exit 1
    fi
fi
```

**Trade-off**: этот hook жёстко блокирует PR при drift > 30. Если legitimate-причина быть старше (большой long-running feature branch) — Шифу может `SKIP_BRANCH_FRESHNESS=true` env-переменной. Это **opt-out, не opt-in**, что соответствует «не делай руками» — дефолт безопасный.

### 2.5 Альтернативная защита: cron-мониторинг vendor-патчей (ADR-0045 §4 backlog)

Это **не** часть этого PR, но **должно** стать отдельной карточкой: `agent-flow-drift-detect.sh` должен проверять не только раскладку EXPECTED, но и applied-состояние каждого `vendor/hermes-agent-*.patch` через `git -C /home/builder/.hermes/hermes-agent apply --reverse --check $patch` (см. install.sh:343). Если патч не applied И не apply'им — emit alert.

Это даст сигнал **раньше**, чем воркер успеет сделать PR с stale-base.

## 3. Инвариант

**Каждый worktree-воркер стартует на ветке, которая ≤ 30 коммитов behind `origin/develop`.**

- ≤ 30 коммитов — порог, при котором rebase тривиален (≤ 5 минут работы, без конфликтов в норме).
- > 30 коммитов → PR блокируется pre-PR hook до rebase.
- Fetch в `agent-flow-triage.sh` — **дополнительный** слой: даёт свежий `origin/develop` ДО создания worktree, не полагаясь на работу `agents_sleep.sh`.
- Vendor-патч `_resolve_worktree_base_ref` — **последний рубеж**: если предыдущие два слоя дали сбой, воркер хотя бы стартует с `origin/develop`, а не с локального HEAD.

**Три слоя защиты** (defense in depth):

1. **Fetch в triage** (pre-create) → воркер чаще получает свежий base при создании ветки.
2. **Vendor-патч** (in-create) → даже если fetch протух, `_ensure_git_worktree` использует `origin/develop` (или `HEAD`+warning).
3. **Pre-PR hook** (post-create, pre-push) → даже если оба предыдущих дали сбой, push блокируется.

Если все три слоя сломаны — это уже mis-config уровня «всё лежит», и блокировка PR правильная (Шифу увидит alert, починит руками через `SKIP_BRANCH_FRESHNESS=true`).

## 4. Решение

### 4.1 Regen vendor-патча (devops, `t_5e3c3e09-d1`)

Воркер devops запускает:

```bash
bash scripts/agent_flow/agent-flow-regen-vendor-patch.sh \
    scripts/agent_flow/vendor/hermes-agent-z-spawn-base-origin-develop.patch
```

Затем **локально** проверяет apply на чистом upstream:

```bash
cd /tmp/ha-test && git clone --depth 1 /home/builder/.hermes/hermes-agent ha-clean
cd ha-clean && \
    git apply /path/to/regen.patch && \
    echo "OK regen apply" && \
    git apply --reverse --check /path/to/regen.patch && \
    echo "OK regen idempotent"
```

Если regen прошёл — коммит в develop, base=develop, CI зелёный.

### 4.2 `agent-flow-triage.sh` — fetch в начале main (devops, `t_5e3c3e09-d2`)

В начало `main()` (после `set -euo pipefail` и парсинга env) добавить:

```bash
# ADR-0045 §2.2: подтянуть origin/develop перед обработкой issues.
# Это устраняет класс багов #1571/#1887, где воркеры создавали ветки
# от stale HEAD главного worktree. Fetch — best-effort: при offline
# падаем на HEAD, как раньше; никаких regressions в happy-path.
if [ "${SKIP_TRIAGE_FETCH:-}" != "true" ]; then
    log "fetching origin/develop for up-to-date base..."
    if ! git fetch origin develop --prune 2>&1 | tee -a "$LOG" | head -5; then
        log "WARN: git fetch origin develop failed — triage продолжается с local HEAD"
    fi
    log "develop tip: $(git log origin/develop --oneline -1 2>/dev/null || echo '(no develop)')"
fi
```

Регрессионный тест: `tests/test_triage_fetches_origin_develop.sh` — мокаем git, проверяем что fetch вызван в первые 30 секунд lifetime скрипта.

### 4.3 Pre-PR hook `validate_branch_freshness.sh` (devops, `t_5e3c3e09-d3`)

Скрипт описан в §2.3. Регрессионный тест: `tests/test_validate_branch_freshness.sh`:

- Сценарий A: branch 0 commits behind → exit 0.
- Сценарий B: branch 25 commits behind, MAX_BRANCH_BEHIND=30 → exit 0.
- Сценарий C: branch 31 commits behind → exit 1 + правильный текст в stderr.
- Сценарий D: merge-commit (CI) → exit 0 (skip).

### 4.4 Подключение к CI (devops, `t_5e3c3e09-d4`)

В `.github/workflows/pr-checks.yml` (если существует) или новый workflow:

```yaml
- name: Branch freshness
  if: github.event_name == 'pull_request'
  env:
    SKIP_BRANCH_FRESHNESS: ${{ vars.SKIP_BRANCH_FRESHNESS }}
  run: bash scripts/agent_flow/validate_branch_freshness.sh origin/develop
```

**Backlog (не этот PR)**: сделать это частью `agent-flow-merge-gate.sh` как pre-merge check, а не отдельным CI step. Пока — отдельный step достаточно.

### 4.5 Cron-мониторинг vendor-патчей (ADR-0045 §4 backlog, отдельная карточка)

В `scripts/agent_flow/agent-flow-drift-detect.sh` добавить блок:

```bash
# ADR-0045 §4.5: monitor vendor-patches applied-status.
for _patch in "$SCRIPT_DIR"/vendor/hermes-agent-*.patch; do
    [ -f "$_patch" ] || continue
    if [ "${_patch##*.}" = "DISABLED" ]; then
        continue  # явно отключённые (ретро t_07799ca7)
    fi
    if ( cd "$HERMES_AGENT_DIR" && git apply --reverse --check "$_patch" >/dev/null 2>&1 ); then
        echo "  OK   applied: $_patch"
    elif ( cd "$HERMES_AGENT_DIR" && git apply --check "$_patch" >/dev/null 2>&1 ); then
        echo "  APPL  pending: $_patch (apply on next install.sh)"
    else
        echo "  STALE: $_patch (upstream moved, regen required)"
        # TODO: emit alert (post to GitHub issue or notify Шифу)
    fi
done
```

**Не блокер** для текущего PR — но должен стать отдельной карточкой (devops), потому что без него §4.1 фикс будет регрессировать снова.

## 5. Альтернативы, которые отвергли

| Альтернатива | Почему отвергли |
|---|---|
| **Полностью убрать `_ensure_git_worktree` из hermes-agent, делать worktree в triage напрямую через bash** | Тонкая граница: `hermes-agent` — общий для нескольких проектов. Лезть в его поведение bash-обвязкой = создаём неявный контракт, который сломается у соседей. Vendor-патч — стандартный механизм интеграции с этим проектом (ADR-0042 §2.2 пример того же подхода). |
| **Hard-rebase воркера в начале каждой карточки (`git rebase origin/develop` после `worktree add`)** | Re-base может дать конфликт, который воркер не способен разрешить → карточка зависнет. Worker должен **стартовать** на свежем base, а не разрешать конфликты auto. |
| **Использовать `git pull --rebase` в начале каждого cron-тика** | Делает main worktree dirty (uncommitted changes в cron-среде?). Поведение нелокальное (мутирует `refs/heads/develop`). fetch + ничего больше — дешевле и безопаснее. |
| **Один скрипт `validate_branch_freshness.sh` и для pre-PR и для cron-мониторинга** | Разные триггеры: pre-PR = блокирующий hook, cron = advisory. Скрипт должен иметь mode (block/advise), что усложняет тестирование. Лучше DRY через shared lib. |
| **Увеличить MAX_BRANCH_BEHIND до 100, чтобы не блокировать legitimate-long-running** | Поощряет stale worktree'ы. Цель — заставить воркера **стартовать** на свежем, а не **копить** drift. 30 — баланс: больше normal-feature, меньше big-bang. |
| **Не делать pre-PR hook, полагаться только на fetch в triage + vendor-патч** | Если fetch и vendor-патч оба дадут сбой (offline + regen протух), PR всё равно уйдёт. Pre-PR hook — третий слой защиты (defense in depth). |

## 6. Trade-offs (резюме)

| Что получаем | Чем платим |
|---|---|
| Каждый worktree-воркер стартует на актуальном develop (≤ 30 коммитов behind) | +5 секунд на fetch в `agent-flow-triage.sh` каждые 1 минуту (cron) |
| PR блокируется при drift > 30, а не молча уходит с CONFLICTING | Opt-out через `SKIP_BRANCH_FRESHNESS=true` для legitimate-long-running (документировано в §2.4) |
| Vendor-патч regen-утилита уже есть, не нужно новых инструментов | Regen требует ручного вмешательства devops-воркера (но это разовая задача) |
| Reuse: `validate_branch_freshness.sh` применим в любом CI (GitHub Actions, локальный pre-push hook) | +1 shell-скрипт в `scripts/agent_flow/`, +1 тест |
| Три независимых слоя защиты (fetch / vendor-патч / pre-PR hook) | Больше точек конфигурации (3 env-переменные: `SKIP_TRIAGE_FETCH`, `MAX_BRANCH_BEHIND`, `SKIP_BRANCH_FRESHNESS`) |
| Cron-мониторинг vendor-патчей (backlog) даст ранний сигнал регрессии | Отдельная карточка (не блокер для текущего PR) |

## 7. План внедрения

| # | Действие | Кто | Acceptance |
|---|---|---|---|
| 1 | Этот ADR смержен в `develop` (PR `z-architect/t_5e3c3e09` → base=develop) | architect | PR открыт, CI зелёный |
| 2 | Vendor-патч `hermes-agent-z-spawn-base-origin-develop.patch` regen под актуальный `kanban_db.py` | devops | `t_5e3c3e09-d1` PR смержен, install.sh прогоняется чисто |
| 3 | `agent-flow-triage.sh` — fetch в начале main + env `SKIP_TRIAGE_FETCH` | devops | `t_5e3c3e09-d2` PR смержен, регресс-тест `test_triage_fetches_origin_develop.sh` зелёный |
| 4 | `scripts/agent_flow/validate_branch_freshness.sh` + регресс-тест | devops | `t_5e3c3e09-d3` PR смержен, `test_validate_branch_freshness.sh` зелёный |
| 5 | Pre-PR hook в `.github/workflows/pr-checks.yml` | devops | `t_5e3c3e09-d4` PR смержен, CI step виден в PR-чеке |
| 6 | `docs/adr/0045` → Accepted после merge | (авто) | merge commit переключает статус |
| 7 | Backlog: cron-мониторинг vendor-патчей в `agent-flow-drift-detect.sh` (ADR-0045 §4.5) | devops | отдельная карточка после merge этого ADR |
| 8 | Регресс-тест на issue #1887: после фикса PR #1884 и PR #1886 должны быть rebase'нуты и mergeable | (наблюдение) | merge-gate шлёт ✅ verdict |

## 8. Что **не** делаем

- **Не мерджим PR вручную** (правило 2d) — Шифу мержит.
- **Не переписываем PR #1884 / #1886 руками** — это работа воркера (rebase — часть его контракта).
- **Не убираем fetch из `agent-flow-e2e-process.sh` / `agent-flow-merge-gate.sh` / `agent-flow-drift-detect.sh`** — они уже правильные.
- **Не делаем LLM-based scope classifier для «stale PR»** — overkill, `git rev-list --count` дешевле и точнее.
- **Не вводим auto-rebase воркера при drift > 30** — rebase может дать неразрешимый конфликт → зависание. Block-and-alert лучше, чем silent-fix.
- **Не делаем vendor-патч частью репо** (в смысле «git submodule в hermes-agent») — это не наша зона, vendor-патчи = standard practice (см. ADR-0042 §2.2).
- **Не фиксим карточки-призраки вручную** (rebase PR #1884/#1886) — это «не делай руками». Ждём, что после merge этого ADR воркеры стартуют правильно.

## 9. Ссылки

- `t_5e3c3e09` — эта ретро-карточка.
- Issue #1887 — репортёр бага (02.09.2026).
- Issue #1571 — первый репортёр (23.08.2026), оригинал.
- PR #1573 (`556ecc8c`) — первый фикс воркером devops (`t_07799ca7`), лежит в develop.
- `docs/adr/RECON-1571-worktree-stale-base.md` — разведка и план реализации (vendor-патч + drift warning).
- `scripts/agent_flow/vendor/hermes-agent-z-spawn-base-origin-develop.patch` — готовый патч (нужен regen).
- `scripts/agent_flow/tests/test_spawn_worktree_origin_develop_base.sh` — готовый регресс-тест (нужна адаптация под regen).
- `scripts/agent_flow/agent-flow-regen-vendor-patch.sh` — утилита для regen vendor-патчей (ADR-0042 §2.2).
- ADR-0030 — ADR numbering SOT (этот ADR = `0045` по правилу §2.2).
- ADR-0036 — mis-scope guard, §4.3 cron-надзор (расширяется для мониторинга vendor-патчей в backlog).
- ADR-0042 — пример того, как правильно фиксить regression в process-скриптах (DRY через shared lib + env-vars).
- ADR-0013 — incremental delivery (PR ≤ 30 коммитов — именно этот порог выбран для `MAX_BRANCH_BEHIND`).
- ADR-0018 — честный FAIL (никаких голословных «CI зелёный», только raw-вывод).
- `hermes_cli/kanban_db.py::_ensure_git_worktree` (≈line 8215 в исходном RECON-1571) — точка изменения vendor-патча.