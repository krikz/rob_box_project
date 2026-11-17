# ADR-0042: unknown-assignee rollup — единый сводный комментарий вместо спама per-issue

| Поле | Значение |
|---|---|
| Статус | Proposed (после merge → Accepted) |
| Дата | 2026-09-01 |
| Автор | agent-flow (Hermes Agent); ретро-карточка `t_e1a9613d`, issue #1824 |
| Родители | ADR-0036 (mis-scope guard §4.3 cron-надзор), ADR-0041 (silent-drop guard для `gh issue edit`) |
| Контекст | Issue #1824 «bug(process): agent-flow-triage infinite-loop on invalid assignee (спам ретро каждые 2 мин)»: triage каждый тик (every 1m) писал ОТДЕЛЬНЫЙ комментарий на каждый issue с невалидным assignee (2 комментария на issue: invalid-assignee + whoami-label), при 20+ unknown issues это 40+ комментариев каждые 1-2 мин. Silent-drop guard из ADR-0041 закрыл только часть потока (`gh issue edit` для метки `agent-flow-error`), но **комментарии продолжали спамиться** в каждый issue отдельно. |
| Затрагивает | (a) `scripts/agent_flow/agent-flow-triage.sh` — invalid-assignee guard + новая функция `_emit_unknown_assignee_rollup` + `process_issues_json` accumulator; (b) `scripts/agent_flow/agent-flow-unlabeled-sweep.sh` — env-fallback fix (DRY → lib `af_load_profile_env`); (c) ADR-0036 §4.3 (cron-надзор расширяется правилом break-on-unknown-assignee + per-tick dedup). |
| Связанные | `t_e1a9613d` (эта), issue #1824 (репортёр бага), ADR-0041 (silent-drop guard, решает смежную проблему для `gh issue edit`), ADR-0036 §4.3 (cron-надзор, родительский процесс), `agent-flow-blocked-watchdog.sh` (расширяется break-on-unknown-assignee), `lib_agent_flow_common.sh::af_load_profile_env` (DRY — используется для env fallback fix). |

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем (до фикса, 31.08.2026 ~22:00Z)

`agent-flow-triage.sh` (cron `agent-flow-triage` в профиле `agent-flow`, every 1m) при обработке issues с меткой `hermes` запускал `process_issues_json` для каждого issue. Если `role` (из label `agent:<role>` или `AGENT_FLOW_DEFAULT_ROLE`) **отсутствует** в `hermes profile list` — guard `is_valid_profile` писал:

1. `gh issue comment <n>` — длинный текст «invalid assignee: <role>, valid profiles: [...]» (≈400 байт).
2. `whoami_add_label` — внутренний self-id whoami комментарий (≈200 байт).
3. `gh issue edit <n> --add-label agent-flow-error` — метка.

При 20 таких issues × 1 тик/мин × 60 мин = **2400 комментариев в час** в issues, которые юзер уже «забыл» (или создал автоматически через GSD без проверки assignee). Issue #1824 зафиксировал этот баг как «спам ретро каждые 2 мин» (фактически каждые 1 мин, но комментарий визуально спаривается → 2-мин ощущение).

### 1.2 Почему не сработали текущие защиты

| Слой | Что должен ловить | Почему не сработал |
|---|---|---|
| ADR-0041 silent-drop guard (`gh issue edit`) | Drop метки если assignee не существует | Закрыл **только `gh issue edit`**, не комментарии. |
| `is_valid_profile` (t_dd7a5749) | Не создавать kanban-карточку для unknown assignee | Работал, но **сайд-эффект (комментарий)** оставался. |
| Triage rate-limit (G3) | rate-limit → exit 0 | Не релевантно — это не про rate-limit, а про per-issue spam. |
| Cron `every 1m` | Tick каждый 1 мин | **Root cause**: тиков много → много комментариев. Не лечится снижением частоты (orphan issues накапливаются). |
| Per-issue `kanban: t_` marker (G7) | Не создавать карточку дважды | Не релевантно — карточка и так не создавалась. |

### 1.3 Бизнес-последствие (если не чинить)

- **Notification storm**: каждый owner 20+ issues получает `gh issue comment` notifications → они либо игнорируют, либо отключают notifications навсегда (потеря сигнала для ЛЕГИТИМНЫХ комментариев вроде `agent-flow:dedup-skip` или `kanban: t_xxx`).
- **GitHub rate limit**: 2400 комментариев/час × ~600 байт = ~1.4 MB/час исходящего трафика на gh API. На 1 час это ~80% rate budget'а core API (5000 req/hour).
- **Дискредитация ретро-процесса**: Шифу видит «опять ретро-спам» → перестаёт читать уведомления от agent-flow → пропускает легитимные mis-config (как в t_1ca827a6, 09.08 — карточка с assignee=`triager` висела 2.4ч).
- **Disk space в cron output**: каждый output содержит 20+ строк `🚨 issue #N: assignee 'X' НЕВАЛИДЕН` → 100+ строк лога на тик → `/home/builder/.hermes/profiles/agent-flow/cron/output/<job_id>/<tick>.md` быстро пухнет.

## 2. Где SOT и какие слои трогаем

### 2.1 SOT скриптов — `<repo>/scripts/agent_flow/*.sh`

Изменения в `agent-flow-triage.sh`:
- Заменить invalid-assignee guard (lines 967-991 в develop-версии): убрать per-issue `gh issue comment`/`gh issue edit`, оставить только **сбор в массив** `_unknown_assignee_records` (format: `<number>\t<role>\t<title_prefix>`, separator = `$IFS`).
- Добавить новую функцию `_emit_unknown_assignee_rollup` (вызывается после Phase 1+2): единый rollup-комментарий в `$UNKNOWN_ASSIGNEE_ROLLUP_ISSUE` (default `1824`) с маркером `agent-flow-triage:unknown-assignee-rollup`.
- Добавить `break`-on-mass (если `>= UNKNOWN_ASSIGNEE_PHASE_BREAK_AT=50` unknown-assignee в текущей фазе → break, продолжим Phase 2, потом rollup).
- Per-issue метка `agent-flow-error` всё ещё ставится (через `whoami_add_label` dedup), но комментарий — единый.

Изменения в `agent-flow-unlabeled-sweep.sh`:
- Заменить локальный 3-кандидатный ENV_FILE fallback (lines 117-131) на вызов `af_load_profile_env` из `lib_agent_flow_common.sh` (DRY — у неё уже правильный набор 4 кандидатов с absolute-path ПЕРВЫМ, что было критично для cron из профиля agent-flow/devops/architect где `$HERMES_HOME` = per-profile).
- Auto-escalation: добавить watchdog-shared cron (наследует ADR-0036 §4.3), который при `failure_streak >= N` тиков подряд (default 5) для unlabeled-sweep **пишет auto-escalation в issue #1824** и помечает нужные PR как `needs-review`.

### 2.2 Конфигурация (env-переменные)

Все через env, defaults в скрипте:

| Переменная | Default | Назначение |
|---|---|---|
| `UNKNOWN_ASSIGNEE_ROLLUP_ISSUE` | `1824` | Issue для сводного комментария (issue #1824 — сам reporter бага). |
| `UNKNOWN_ASSIGNEE_ROLLUP_DEDUP_MIN` | `30` | Per-tick dedup окно в минутах: если свежий комментарий с маркером в этом окне, новый НЕ пишется. |
| `UNKNOWN_ASSIGNEE_ROLLUP_LABEL` | `agent-flow-error` | Метка для каждого unknown issue (через whoami dedup, идемпотентно в окне 2ч). |
| `UNKNOWN_ASSIGNEE_ROLLUP_MARKER` | `agent-flow-triage:unknown-assignee-rollup` | Уникальный маркер в начале rollup-комментария для dedup-парсинга. |
| `UNKNOWN_ASSIGNEE_PHASE_BREAK_AT` | `50` | Mass-break: если собрано столько unknown-assignee в одной фазе → `break` (защита от огромного body). |
| `UNKNOWN_ASSIGNEE_ROLLUP_DRY_RUN` | `false` | Для тестов: skip gh side-effects. |

## 3. Инвариант

**Тик triage НЕ пишет больше 1 комментария на каждые 30 мин, независимо от числа issues с unknown-assignee.**

- Rollup-комментарий = ровно 1 на окно `UNKNOWN_ASSIGNEE_ROLLUP_DEDUP_MIN` минут.
- Если fresh rollup-комментарий уже есть в `UNKNOWN_ASSIGNEE_ROLLUP_ISSUE` → skip, только метка на новые issues (через whoami dedup).
- Если `UNKNOWN_ASSIGNEE_ROLLUP_ISSUE` закрыт → `gh issue comment` упадёт fail-OPEN, метки всё равно проставляются.
- Side-effect на каждый issue — ТОЛЬКО `whoami_add_label` (который уже dedup'ит в окне 2ч через `hermes_github.sh`). Никаких per-issue `gh issue comment` для invalid-assignee.

## 4. Решение

### 4.1 `agent-flow-triage.sh` — accumulator + rollup

#### 4.1.1 Замена per-issue guard на accumulator

В `process_issues_json` (lines 967-991 в develop):

```bash
# BEFORE (ретро-bug):
load_valid_profiles
if ! is_valid_profile "$role"; then
    log "🚨 issue #${number}: assignee '${role}' НЕВАЛИДЕН (нет в profile list) — пропускаем (errored)"
    if [ "$DRY_RUN" != "true" ]; then
        _valid_csv="$(printf '%s' "$VALID_PROFILES" | tr '|' ',' | sed 's/^,//;s/,$//')"
        gh issue comment "$number" --repo "$GH_REPO" --body "🚨 **agent-flow-triage: invalid assignee** ..." >/dev/null 2>&1 || true
        whoami_add_label "$number" "agent-flow-error" "invalid assignee=${role}..."
        gh issue edit "$number" --repo "$GH_REPO" --add-label "agent-flow-error" >/dev/null 2>&1 || true
    fi
    errored=$((errored+1)); continue
fi

# AFTER (ADR-0042):
load_valid_profiles
if ! is_valid_profile "$role"; then
    log "🚨 issue #${number}: assignee '${role}' НЕВАЛИДЕН — добавляю в rollup"
    _tp="$(printf '%s' "$title" | tr '[:upper:]' '[:lower:]' | tr -cs '[:alnum:][:space:]' ' ' | awk '{for(i=1;i<=6 && i<=NF;i++) printf "%s%s", $i, (i==6 || i==NF)?"":" "; print ""}')"
    _unknown_assignee_records="${_unknown_assignee_records:-}${_unknown_assignee_records:+$IFS}$(printf '%s\t%s\t%s' "$number" "$role" "$_tp")"
    errored=$((errored+1))
    # Mass-break (защита от огромного body):
    if [ "${_unknown_assignee_records:-}" ] && [ "$(printf '%s\n' "${_unknown_assignee_records//$IFS/\\n}" | wc -l)" -ge "${UNKNOWN_ASSIGNEE_PHASE_BREAK_AT}" ]; then
        log "🚨 phase-break: ${UNKNOWN_ASSIGNEE_PHASE_BREAK_AT}+ unknown-assignee in phase=${phase_label}, breaking inner loop"
        break
    fi
    continue
fi
```

#### 4.1.2 Новая функция `_emit_unknown_assignee_rollup`

После Phase 1+2 (в main loop, после `process_issues_json "phase2" "$phase2_filtered"`):

```bash
_emit_unknown_assignee_rollup || true
```

Сама функция (≈80 строк, вставлена в main scope после `process_issues_json`):

```bash
_emit_unknown_assignee_rollup() {
    [ -n "${_unknown_assignee_records:-}" ] || return 0
    local recs=() _count=0 _now_epoch _cutoff_epoch _last_marker_epoch _valid_csv _body _marker
    while IFS= read -r rec; do
        [ -n "$rec" ] || continue
        recs+=("$rec")
        _count=$((_count+1))
    done < <(printf '%s\n' "${_unknown_assignee_records//$IFS/\\n}")
    log "_emit_unknown_assignee_rollup: ${_count} unknown-assignee records (will roll up to issue #${UNKNOWN_ASSIGNEE_ROLLUP_ISSUE})"
    if [ "${UNKNOWN_ASSIGNEE_ROLLUP_DRY_RUN:-false}" = "true" ]; then
        log "DRY-RUN: would emit rollup to #${UNKNOWN_ASSIGNEE_ROLLUP_ISSUE}"
        return 0
    fi
    _marker="${UNKNOWN_ASSIGNEE_ROLLUP_MARKER}"
    _now_epoch="$(date -u +%s)"
    _cutoff_epoch=$((_now_epoch - UNKNOWN_ASSIGNEE_ROLLUP_DEDUP_MIN * 60))
    _last_marker_epoch="0"
    if _last_iso="$(gh api "repos/${GH_REPO}/issues/${UNKNOWN_ASSIGNEE_ROLLUP_ISSUE}/comments?per_page=20" \
        --jq '([.[] | select((.body // "") | test("'"${_marker}"'"))] | last | .created_at) // empty' 2>/dev/null || true)" \
        && [ -n "$_last_iso" ]; then
        _last_marker_epoch="$(date -u -d "$_last_iso" +%s 2>/dev/null || echo 0)"
    fi
    # Per-issue label — ВСЕГДА (и для dedup-hit, и для fresh-write)
    while IFS= read -r rec; do
        [ -n "$rec" ] || continue
        local _n _role _tp
        _n="$(printf '%s' "$rec" | cut -f1)"
        _role="$(printf '%s' "$rec" | cut -f2)"
        _tp="$(printf '%s' "$rec" | cut -f3)"
        whoami_add_label "$_n" "$UNKNOWN_ASSIGNEE_ROLLUP_LABEL" \
            "unknown assignee=${_role} (retro t_e1a9613d, issue #1824)" >/dev/null 2>&1 || true
        gh issue edit "$_n" --repo "$GH_REPO" --add-label "$UNKNOWN_ASSIGNEE_ROLLUP_LABEL" >/dev/null 2>&1 || true
    done < <(printf '%s\n' "${_unknown_assignee_records//$IFS/\\n}")
    # Dedup-hit: skip new comment
    if [ "${_last_marker_epoch:-0}" -ge "${_cutoff_epoch}" ] 2>/dev/null; then
        log "_emit_unknown_assignee_rollup: dedup-hit (last rollup @ ${_last_iso})"
        unknown_assignee_rollup_dedup_hit=$((unknown_assignee_rollup_dedup_hit+1))
        return 0
    fi
    # Fresh write: формируем body
    _valid_csv="$(printf '%s' "${VALID_PROFILES:-}" | tr '|' ',' | sed 's/^,//;s/,$//')"
    {
        printf '%s (tick=%s)\n\n' "${UNKNOWN_ASSIGNEE_ROLLUP_MARKER}" "$(date -Iseconds)"
        printf '🚨 **agent-flow-triage: unknown-assignee rollup** (%d issue(s))\n\n' "${_count}"
        printf '| Issue | Bad role | Title prefix |\n|---|---|---|\n'
        # ... (table generation) ...
    } > /tmp/agent-flow-rollup-body.$$.md
    gh issue comment "${UNKNOWN_ASSIGNEE_ROLLUP_ISSUE}" --repo "${GH_REPO}" \
        --body "$(cat /tmp/agent-flow-rollup-body.$$.md)" >/dev/null 2>&1 || true
    rm -f /tmp/agent-flow-rollup-body.$$.md
    unknown_assignee_rollup_emitted=$((unknown_assignee_rollup_emitted+1))
}
```

### 4.2 `agent-flow-unlabeled-sweep.sh` — DRY env-fallback

Заменяем локальный 3-кандидатный fallback на вызов `af_load_profile_env` из `lib_agent_flow_common.sh` (у неё уже правильный набор 4 кандидатов с absolute-path ПЕРВЫМ):

```bash
# AFTER (ADR-0042):
_LIB_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
. "$_LIB_DIR_HERE/lib_agent_flow_common.sh"
af_load_profile_env "${HERMES_HOME}/profiles/agent-flow/.env" || true
# Defensive: если GH_REPO всё ещё пустой → legacy absolute-path fallback
if [ -z "${GH_REPO:-}" ]; then
    for _candidate in "/home/builder/.hermes/profiles/agent-flow/.env"; do
        if [ -f "$_candidate" ]; then
            # shellcheck disable=SC1090
            set -a; . "$_candidate"; set +a
            break
        fi
    done
fi
```

Это закрывает root-cause для devops/architect профилей: у них `HERMES_HOME=/home/builder/.hermes/profiles/<profile>` → локальный fallback строил путь `/home/builder/.hermes/profiles/<profile>/profiles/agent-flow/.env` (не существует) → exit 1 на 26 тиков подряд.

### 4.3 ADR-0036 §4.3 — расширение правилами break-on-unknown-assignee + per-tick dedup

В ADR-0036 §4.3 (cron-надзор) добавляется явное правило: «Если в `agent-flow-triage.sh` обнаружен unknown-assignee issue — НЕ спамить per-issue комментарии (закрыто в ADR-0042), а писать ОДИН сводный комментарий в rollup-issue с dedup-окном `UNKNOWN_ASSIGNEE_ROLLUP_DEDUP_MIN` минут».

См. файл `docs/adr/0036-mis-scope-task-guard.md` §4.3 после merge ADR-0042.

### 4.4 Auto-escalation в `agent-flow-blocked-watchdog.sh` (ADR-0036 §4.3 — watchdog-shared)

Расширение watchdog-shared cron (текущий `agent-flow-blocked-watchdog.sh` стиль):

```bash
# Auto-escalation: если unlabeled-sweep имеет failure_streak >= 5 подряд →
# создать needs-review issue, который подсветит баг Шифу.
if [ "$failure_streak" -ge 5 ]; then
    log "auto-escalation: unlabeled-sweep failed ${failure_streak}x → needs-review"
    gh issue create --repo "$GH_REPO" \
        --title "[AUTO-ESCALATION] agent-flow-unlabeled-sweep cron ${failure_streak}x fail" \
        --label "agent-flow-error,needs-review,auto-escalation" \
        --body "..." || true
fi
```

## 5. Альтернативы, которые отвергли

| Альтернатива | Почему отвергли |
|---|---|
| **Снизить cron frequency (1m → 5m)** | Orphan issues накапливаются за 5 мин + теряем «real-time» для легитимных issues. Не решает root cause. |
| **Один комментарий на issue, но с TTL** | Нет механизма TTL в gh API. Per-issue dedup = race-prone + не масштабируется. |
| **Закрывать issue сразу при unknown-assignee** | Слишком агрессивно — юзер может добавить правильную метку через 5 мин, и мы закроем его легитимную работу. |
| **Использовать GitHub Discussions вместо Issues** | Шифу привык к issue-based workflow. Discussions — отдельная сущность, требует переучивания. |
| **Reassign issue автоматически на `AGENT_FLOW_DEFAULT_ROLE`** | Это ЛОМАЕТ намерение юзера (он хотел `agent:triager`, может быть, чтобы создать профиль позже). Нужно информировать, а не молча чинить. |
| **Использовать существующий ADR-0041 silent-drop guard расширенно** | ADR-0041 закрывает `gh issue edit`, не комментарии. Расширение его на комментарии = смешение семантик. Лучше отдельный ADR. |

## 6. Trade-offs (резюме)

| Что получаем | Чем платим |
|---|---|
| Per-tick spam eliminated (rollup 1/30min vs 20+/min) | Доп. REST API call для dedup-check (один раз за tick). |
| Per-issue метка `agent-flow-error` остаётся (через whoami dedup 2ч) | Не отличает fresh vs dedup-hit — оба ставят метку. |
| Mass-break защищает от огромного body (>50 issues) | Не все unknown-assignee issues попадут в один rollup (размазывается по нескольким тикам). Это OK — цель именно per-tick spam reduction. |
| Auto-escalation в watchdog-shared ловит long-streak unlabeled-sweep failures | Шифу получает дополнительный «needs-review» issue при 5+ fail — нужен eyeball. |
| DRY: `af_load_profile_env` переиспользуется в 5 скриптах (triage, merge-gate, e2e-process, deploy-sweep, **теперь unlabeled-sweep**) | Vendor-патч lib становится чуть толще (~10 строк), нужен sync check в drift-detect (уже есть). |

## 7. Миграция и rollout

1. **Step 1** (этот PR): заменить invalid-assignee guard в `agent-flow-triage.sh` на accumulator + rollup.
2. **Step 2** (этот PR): заменить env-fallback в `agent-flow-unlabeled-sweep.sh` на `af_load_profile_env` lib.
3. **Step 3** (этот PR): обновить ADR-0036 §4.3 правилом break-on-unknown-assignee.
4. **Step 4** (этот PR): sync 3 копии через `install.sh` + md5 verify (drift-detect OK).
5. **Step 5** (после merge, agent-flow e2e-process): прогнать unlabeled-sweep 3 раза подряд, проверить `failure_streak=0` + `last_status=ok`.
6. **Step 6** (после merge): создать fake-issue с меткой `hermes` + невалидный `agent:triager`, прогнать triage, проверить что:
   - Ровно 1 комментарий в issue #1824 (rollup).
   - Ровно 0 комментариев в fake-issue (только метка).
   - Второй тик (через 1 мин) → 0 новых комментариев (dedup).
7. **Step 7**: закрыть issue #1824 с ссылкой на эту ADR + PR.

## 8. Back-compat

- Все переменные имеют defaults → если ни одна не выставлена, скрипт ведёт себя как раньше (per-issue комментарий), за исключением того, что **rollup-комментарий НЕ пишется** (нужна `UNKNOWN_ASSIGNEE_ROLLUP_ISSUE` для явной активации). Wait — default `UNKNOWN_ASSIGNEE_ROLLUP_ISSUE=1824`, значит rollup активен по умолчанию. Backward-compat **нарушен** для тех, кто полагался на per-issue комментарии (например, Шифу мониторил issues по этим комментариям).
- Mitigation: ADR явно говорит, что per-issue комментарии заменяются на rollup, и Шифу должен следить за issue #1824 (или за `agent-flow-error` меткой на issues).
- `UNKNOWN_ASSIGNEE_ROLLUP_DRY_RUN=true` → можно откатить поведение для тестов.
