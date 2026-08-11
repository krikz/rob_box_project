#!/bin/bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-e2e-process.sh
# Каноническая версия живёт в репо. На хост раскладывается через
# `bash <repo>/scripts/agent_flow/install.sh`, который создаёт
# символические ссылки в:
#   - ~/.hermes/profiles/agent-flow/scripts/agent-flow-e2e-process.sh
#   - ~/.hermes/profiles/architect/scripts/agent-flow-e2e-process.sh
#   - ~/.hermes/scripts/agent-flow-e2e-process.sh
# Правка: редактируем <repo>/scripts/agent_flow/agent-flow-e2e-process.sh, commit, merge.
# На хост: bash <repo>/scripts/agent_flow/install.sh (или вручную cp + ln -sf).
# Если ты правишь этот файл НА ХОСТЕ руками — синхронизируй обратно в репо.
# ============================================================================
# agent-flow-e2e-process.sh — Phase 3: needs-e2e → bring up e2e/test-round-N,
# merge agent PR, run e2e, attach verdict to issue.
#
# Pure bash. No LLM. Idempotent. Driven entirely by env (see
# ~/.hermes/profiles/agent-flow/.env).
#
# Pipeline per tick (per AGENT_FLOW_PROPOSAL §3.4, simplified MVP):
#   1. MAINTENANCE gate (remote + local) -> exit 0 if paused
#   2. gh auth check                     -> exit 1 if not authed
#   3. Ensure `z-{e2e}/test-round-N` exists. If absent, create from
#      origin/develop (fresh fetch — Q24).
#   4. List open issues with label `needs-e2e` (sorted by issue number).
#   5. For each issue (sequential — to avoid concurrent round churn):
#        a. Skip if already `e2e-done` or `e2e:rejected`.
#        b. Look up the PR `z-{agent}/<id>-<slug>` (must be MERGED into develop
#           OR — during MVP we're also tolerant of still-open-but-green).
#        c. Smart: if PR was merged into develop later by another path,
#           we just need the agent branch tip.
#        d. Merge agent branch DIRECTLY into `z-{e2e}/test-round-N` (--no-ff,
#           no wip layer). Resolve trivial conflicts with `-X theirs` is
#           intentionally NOT done — we let the merge fail and mark
#           `e2e:rejected` (manual).
#        e. Push test-round-N.
#        f. Trigger `L-E2E Voice Test.yml` workflow on test-round-N.
#        g. Wait verdict (timeout E2E_RUN_TIMEOUT).
#        h. Download run artifact (dialog_e2e.wav if present).
#        k. Comment issue with verdict + run link + artifact links + meta.
#        l. Label `e2e-done` (SUCCESS) or `e2e:rejected` (FAIL).
#   6. Manual merge into develop stays human (Q5/Q7).
#   7. flock lock prevents parallel ticks.
#
# NOTE: this script does NOT auto-merge agent/<id>-<slug> into develop — that
# is the merge-gate's contract (PR must be mergeable + CI green). We pick up
# where merge-gate left off.

set -euo pipefail

# --- defaults (overridden by env / .env) -------------------------------------
# NOTE: hardcode /home/builder/.hermes — cron from per-profile gateway sets
# HERMES_HOME to the profile dir; PROFILE_ENV would then point at a
# non-existent path and GH_REPO would never load (see agent-flow-triage.sh).
HERMES_HOME=/home/builder/.hermes
HERMES_BIN="${HERMES_BIN:-/home/builder/.hermes/hermes-agent/venv/bin/hermes}"
export HOME=/home/builder

ISSUE_LABEL="${ISSUE_LABEL:-hermes}"
NEEDS_E2E_LABEL="${NEEDS_E2E_LABEL:-needs-e2e}"
DONE_LABEL="${DONE_LABEL:-e2e-done}"
REJECTED_LABEL="${REJECTED_LABEL:-e2e:rejected}"
# ретро 10.08 (t_9caf5d52): отдельная метка для infra-FAIL (квота 429, робот
# недоступен, build fail) — issue НЕ выпадает из ротации, needs-e2e остаётся.
INFRA_FAIL_LABEL="${INFRA_FAIL_LABEL:-e2e:infra-fail}"
ERROR_LABEL="${ERROR_LABEL:-agent-flow-error}"
NEEDS_REVIEW_LABEL="${NEEDS_REVIEW_LABEL:-needs-review}"
NO_E2E_LABEL="${NO_E2E_LABEL:-no-e2e-required}"
DEVELOP_BRANCH="${DEVELOP_BRANCH:-develop}"
# FOUNDATION_BRANCH: база для e2e test-round. Актуальный develop (Q24).
FOUNDATION_BRANCH="${FOUNDATION_BRANCH:-develop}"
# NOTE: {e2e} нельзя писать внутри ${VAR:-...} — bash берёт первую } как
# закрывающую конструкцию ${} и ломает строку. Присваиваем отдельно.
TEST_ROUND_PREFIX="${TEST_ROUND_PREFIX:-}"
WORK_BRANCH_PREFIX="${WORK_BRANCH_PREFIX:-}"
if [ -z "$TEST_ROUND_PREFIX" ]; then TEST_ROUND_PREFIX='z-{e2e}/test-round-'; fi
if [ -z "$WORK_BRANCH_PREFIX" ]; then WORK_BRANCH_PREFIX='z-{e2e}/wip-'; fi
E2E_WORKFLOW="${E2E_WORKFLOW:-L-E2E Voice Test.yml}"
BUILD_WORKFLOW="${BUILD_WORKFLOW:-L-Build-All-Services.yml}"
DEPLOY_WORKFLOW="${DEPLOY_WORKFLOW:-L-Deploy and Verify.yml}"
E2E_DEPLOY_ENV="${E2E_DEPLOY_ENV:-test}"
E2E_DEPLOY_REGISTRY="${E2E_DEPLOY_REGISTRY:-local}"
E2E_VOLUME="${E2E_VOLUME:-125}"
E2E_BUILD_TIMEOUT="${E2E_BUILD_TIMEOUT:-3600}"
E2E_DEPLOY_TIMEOUT="${E2E_DEPLOY_TIMEOUT:-1800}"
MAINTENANCE_BRANCH="${MAINTENANCE_BRANCH:-develop}"
MAINTENANCE_FILE="${MAINTENANCE_FILE:-MAINTENANCE}"
E2E_CI_TIMEOUT="${E2E_CI_TIMEOUT:-900}"        # 15 min — CI on PR
E2E_RUN_TIMEOUT="${E2E_RUN_TIMEOUT:-1800}"      # 30 min — e2e workflow
E2E_POLL_INTERVAL="${E2E_POLL_INTERVAL:-15}"
DRY_RUN="${DRY_RUN:-false}"
ISSUE_LIMIT="${ISSUE_LIMIT:-20}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-e2e-process.lock}"
LOG_PREFIX="${LOG_PREFIX:-[agent-flow-e2e-process]}"

# --- source profile .env if present -------------------------------------------
PROFILE_ENV="${HERMES_HOME}/profiles/agent-flow/.env"
if [ -f "$PROFILE_ENV" ]; then
    while IFS='=' read -r key val; do
        case "$key" in ''|\#*) continue ;; esac
        val="${val%\"}"; val="${val#\"}"
        val="${val%\'}"; val="${val#\'}"
        if [ -z "${!key:-}" ]; then
            export "$key=$val"
        fi
    done < "$PROFILE_ENV"
fi

# Defensive defaults.
: "${KANBAN_BOARD:=robbox}"
: "${REPO_DIR:=}"
: "${DRY_RUN:=false}"
: "${ISSUE_LABEL:=hermes}"
: "${NEEDS_E2E_LABEL:=needs-e2e}"
: "${DONE_LABEL:=e2e-done}"
: "${REJECTED_LABEL:=e2e:rejected}"
: "${INFRA_FAIL_LABEL:=e2e:infra-fail}"
: "${ERROR_LABEL:=agent-flow-error}"
: "${NEEDS_REVIEW_LABEL:=needs-review}"
: "${NO_E2E_LABEL:=no-e2e-required}"
: "${DEVELOP_BRANCH:=develop}"
: "${FOUNDATION_BRANCH:=develop}"
: "${TEST_ROUND_PREFIX:=}"
: "${WORK_BRANCH_PREFIX:=}"
: "${E2E_WORKFLOW:=L-E2E Voice Test.yml}"
: "${BUILD_WORKFLOW:=L-Build-All-Services.yml}"
: "${DEPLOY_WORKFLOW:=L-Deploy and Verify.yml}"
: "${E2E_DEPLOY_ENV:=test}"
: "${E2E_DEPLOY_REGISTRY:=local}"
: "${E2E_VOLUME:=125}"
: "${E2E_BUILD_TIMEOUT:=3600}"
: "${E2E_DEPLOY_TIMEOUT:=1800}"
: "${MAINTENANCE_BRANCH:=develop}"
: "${MAINTENANCE_FILE:=MAINTENANCE}"
: "${E2E_CI_TIMEOUT:=900}"
: "${E2E_RUN_TIMEOUT:=1800}"
: "${E2E_POLL_INTERVAL:=15}"
: "${ISSUE_LIMIT:=20}"
# робот для pre-flight (ретро #R2: no-reaction rounds 29/31 — жечь e2e-раунд на мёртвом роботе бессмысленно)
: "${E2E_ROBOT_HOST:=10.1.1.21}"
: "${E2E_ROBOT_USER:=ros2}"
: "${E2E_ROBOT_PASS:=}"   # пароль из окружения, в скрипт не пишем; пусто → pre-flight SKIP с warn

# --- helpers -----------------------------------------------------------------
log() { printf '%s %s %s\n' "$LOG_PREFIX" "$(date -Iseconds)" "$*" >&2; }
run() { if [ "$DRY_RUN" = "true" ]; then printf '%s DRY-RUN %s\n' "$LOG_PREFIX" "$*" >&2; else eval "$@"; fi; }

# G6: flock sentinel.
exec 9>"$LOCK_FILE" || { log "cannot open lock $LOCK_FILE"; exit 1; }
if ! flock -n 9; then
    log "another instance holds $LOCK_FILE — skip"; exit 0
fi

# --- G1: MAINTENANCE gate (remote + local) -----------------------------------
if [ -n "${GH_REPO:-}" ]; then
    remote_ref="${MAINTENANCE_BRANCH}:${MAINTENANCE_FILE}"
    if git ls-remote "https://github.com/${GH_REPO}.git" "$remote_ref" 2>/dev/null | grep -q .; then
        log "🛑 MAINTENANCE flag set on remote ${remote_ref} — skip"; exit 0
    fi
fi
if [ -n "${REPO_DIR:-}" ] && [ -d "$REPO_DIR" ]; then
    if git -C "$REPO_DIR" show "${MAINTENANCE_BRANCH}:${MAINTENANCE_FILE}" >/dev/null 2>&1; then
        log "🛑 MAINTENANCE flag set locally in ${REPO_DIR} — skip"; exit 0
    fi
fi

# --- G2: gh auth check (retry — сетевой сбой ≠ нет авторизации) ------------
_gh_auth_ok=0
for _try in 1 2 3; do
    if gh auth status >/dev/null 2>&1; then _gh_auth_ok=1; break; fi
    sleep 5
done
if [ "$_gh_auth_ok" -ne 1 ]; then
    log "gh auth not configured (или сеть недоступна после 3 попыток) — exit 1"; exit 1
fi

# --- required env ------------------------------------------------------------
: "${GH_REPO:?GH_REPO must be set (owner/repo)}"
if [ -z "${REPO_DIR}" ] || [ ! -d "$REPO_DIR" ]; then
    log "REPO_DIR ('${REPO_DIR:-}') is required and must exist for orchestrator scripts"; exit 1
fi

# Worktree dir for orchestrator — fresh, isolated from main clone. This is
# IMPORTANT: the main clone may be dirty from other workers; we cannot
# operate on its working tree without risking untracked state loss.
WORKTREE_DIR="${WORKTREE_DIR:-/tmp/agent-flow-e2e-$$}"
cleanup() {
    if [ -d "$WORKTREE_DIR" ]; then
        # Remove git worktree bookkeeping first, then the dir.
        git -C "$REPO_DIR" worktree remove --force "$WORKTREE_DIR" 2>/dev/null || true
        [ -d "$WORKTREE_DIR" ] && rm -rf "$WORKTREE_DIR"
    fi
}
trap cleanup EXIT INT TERM

# --- ensure we have a fresh worktree -------------------------------------------------
ensure_worktree() {
    # Reuse a worktree from the previous tick if it exists and is clean.
    if [ -d "$WORKTREE_DIR" ] && [ -d "$WORKTREE_DIR/.git" ]; then
        if git -C "$WORKTREE_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
            log "reusing worktree $WORKTREE_DIR"
            git -C "$WORKTREE_DIR" fetch origin --prune --quiet 2>/dev/null || true
            return 0
        fi
    fi
    rm -rf "$WORKTREE_DIR"
    mkdir -p "$(dirname "$WORKTREE_DIR")"
    git -C "$REPO_DIR" worktree add --detach "$WORKTREE_DIR" origin/"$DEVELOP_BRANCH" >/dev/null 2>&1 \
        || git -C "$REPO_DIR" worktree add --detach "$WORKTREE_DIR" "$DEVELOP_BRANCH" >/dev/null 2>&1 \
        || { log "failed to create worktree $WORKTREE_DIR"; return 1; }
    git -C "$WORKTREE_DIR" remote set-url origin "https://github.com/${GH_REPO}.git" 2>/dev/null || true
    git -C "$WORKTREE_DIR" fetch origin --prune --quiet 2>/dev/null || true
    log "created worktree $WORKTREE_DIR on ${DEVELOP_BRANCH}"
}

# --- find or create e2e/test-round-N -----------------------------------------
# Returns 0 + sets ROUND_BRANCH on success. N = max($N) на remote + 1
# (1, 2, 3 ...) — простой инкремент, БЕЗ даты.
ROUND_BRANCH=""
round_ensure() {
    local list max_n n
    list="$(git -C "$REPO_DIR" ls-remote --heads origin "${TEST_ROUND_PREFIX}*" 2>/dev/null \
        | awk '{print $2}' | sed "s#refs/heads/${TEST_ROUND_PREFIX}##" || true)"
    if [ -z "$list" ]; then
        max_n=0
    else
        max_n="$(printf '%s\n' "$list" | sort -n | tail -n1)"
    fi
    n=$((max_n + 1))
    ROUND_BRANCH="${TEST_ROUND_PREFIX}${n}"
    log "round number: max=${max_n} -> next=${n}"

    # If branch doesn't exist on remote, create it from foundation (fresh origin).
    if ! git -C "$REPO_DIR" ls-remote --heads origin "$ROUND_BRANCH" 2>/dev/null | grep -q .; then
        log "creating ${ROUND_BRANCH} from ${FOUNDATION_BRANCH} (fresh fetch)"
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would: git push origin origin/${FOUNDATION_BRANCH}:refs/heads/${ROUND_BRANCH}"
        else
            # CRITICAL: пушим origin/${FOUNDATION_BRANCH}, НЕ локальную ветку —
            # локальный develop может отстать (чужие коммиты). Всегда свежий.
            if ! git -C "$REPO_DIR" fetch origin "$FOUNDATION_BRANCH" 2>&1 | sed 's/^/  /'; then
                log "failed to fetch origin/${FOUNDATION_BRANCH}"; return 1
            fi
            if ! git -C "$REPO_DIR" push origin "origin/${FOUNDATION_BRANCH}:refs/heads/${ROUND_BRANCH}" 2>&1 | sed 's/^/  /'; then
                log "failed to create ${ROUND_BRANCH}"; return 1
            fi
        fi
    else
        log "reusing ${ROUND_BRANCH} (max N=${max_n})"
    fi

    # Make sure worktree has it.
    git -C "$WORKTREE_DIR" fetch origin "$ROUND_BRANCH" --quiet 2>/dev/null || true
}

# --- get current open issue numbers with needs-e2e ---------------------------
issues_json="$(gh issue list \
    --repo "$GH_REPO" \
    --label "$NEEDS_E2E_LABEL" \
    --state open \
    --limit "$ISSUE_LIMIT" \
    --json number,title,labels,body 2>/dev/null || true)"

# G3: rate-limit check on empty output.
if [ -z "$issues_json" ] || [ "$issues_json" = "[]" ]; then
    rate="$(gh api rate_limit --jq '.resources.core.remaining' 2>/dev/null || echo 999)"
    if [ "${rate:-999}" = "0" ]; then
        log "GitHub rate-limit exhausted — skip tick"; exit 0
    fi
    log "no issues with label '${NEEDS_E2E_LABEL}' on ${GH_REPO}"; exit 0
fi

# --- ensure e2e:infra-fail label exists (ретро 10.08 t_9caf5d52) -------------
# gh issue edit --add-label тихо падает если метки нет в репо. Создаём один раз.
if ! gh label list --repo "$GH_REPO" --limit 200 2>/dev/null | grep -q "^${INFRA_FAIL_LABEL}[[:space:]]"; then
    gh label create "$INFRA_FAIL_LABEL" --repo "$GH_REPO" --color "fbca04" \
        --description "e2e FAIL по инфраструктуре (квота 429 / робот недоступен / build) — issue остаётся в ротации" \
        >/dev/null 2>&1 || log "WARNING: failed to create label ${INFRA_FAIL_LABEL}"
fi

# --- prepare worktree --------------------------------------------------------
ensure_worktree || { log "worktree setup failed"; exit 1; }
round_ensure || { log "round setup failed"; exit 1; }
log "round branch: ${ROUND_BRANCH}"

# --- shared helpers (kept compatible with merge-gate.sh) --------------------
slugify() {
    printf '%s' "$1" \
        | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40
}

has_label() {
    printf '%s' "$1" | tr ',' '\n' | grep -Fxq "$2"
}

# Detect PR kind: "lint" (no e2e needed) vs "functional" (e2e required).
# Signal sources (priority order):
#   1) PR label `${NO_E2E_LABEL}` → lint (explicit worker opt-out)
#   2) PR title prefix `[lint]` / `[refactor]` → lint (worker shorthand)
#   3) otherwise → functional (e2e mandatory)
# Inputs: $1=pr_labels_csv (lowercased), $2=pr_title
# Output: prints "lint" or "functional"; rc=0 always.
detect_pr_kind() {  # $1=labels_csv $2=title
    local labels_csv title_lc prefix
    labels_csv="$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')"
    title_lc="$(printf '%s' "$2" | tr '[:upper:]' '[:lower:]')"
    if has_label "$labels_csv" "$NO_E2E_LABEL"; then
        printf '%s' "lint"; return 0
    fi
    # Title prefix detection (case-insensitive): первый токен до первого пробела.
    prefix="${title_lc%% *}"
    case "$prefix" in
        '[lint]'|'[refactor]') printf '%s' "lint"; return 0 ;;
    esac
    printf '%s' "functional"; return 0
}

# Worker-evidence gate (ретро t_d0151eb3): воркер может сам опубликовать
# комментарий с маркером `worker-evidence:` (raw-логи фичи на роботе,
# ссылка на run, и т.п.). Если такой комментарий есть в PR за последний
# час — процесс НЕ дублирует полный доклад, а только добавляет
# post-link "e2e PASS, см. worker-evidence выше".
# Inputs: $1=pr_number $2=since_iso (e.g. "2026-08-10T07:00:00Z")
# Output: prints "yes" / "no"; rc=0 always.
worker_evidence_recent() {  # $1=pr_number $2=since_iso
    local pr_number="$1" since_iso="$2"
    [ -z "$pr_number" ] && { printf 'no'; return 0; }
    # gh api issues/comments работает и для PR (PR comments — это issue comments).
    gh api "repos/${GH_REPO}/issues/${pr_number}/comments?since=${since_iso}&per_page=100" \
        --jq '.[] | select(.body | startswith("worker-evidence:")) | .user.login' \
        2>/dev/null | head -n1 \
        | { read -r _u; [ -n "$_u" ] && printf 'yes' || printf 'no'; }
}

# --- infra-fail detection (ретро 10.08 t_9caf5d52) --------------------------
# Отличить infra-FAIL (квота MiniMax 429, робот недоступен, build fail) от
# feature-FAIL (баг в коде фичи). Лабелер НЕ должен вешать e2e:rejected на
# инфраструктурный сбой — иначе issue навсегда выпадает из ротации (#1077/#1089).
# Сигналы infra (в порядке убывания надёжности):
#   1. В логе e2e-рана / артефактах есть маркеры квоты/сети/робота:
#      - MiniMax 429 / Too Many Requests / RateLimitError / Token Plan / 2056
#      - TTS fallback: "Silero v5 fallback" / "MiniMax auth error" / "переключаюсь на Silero"
#      - робот недоступен: "Connection refused" / "timed out" / "no route to host" / "ssh: connect"
#      - E2E_NO_REACTION (retry-скрипт не увидел ответа робота) — сам по себе НЕ
#        доказывает infra (может быть и фича), но в сочетании с 429/fallback — да.
#   2. Workflow-ран не создан/завис (это обрабатывается ДО verdict — здесь не нужно).
# Источники: артефакты (voice_e2e_*.log) + console-логи рана (actions/runs/{id}/logs)
# — на #1077 артефакт-лог отсутствовал, но консоль содержала «TTS FALLBACK: MiniMax
# квота» и «E2E_NO_REACTION», так что одних артефактов недостаточно.
# Inputs: $1=artifact_dir (скачанные артефакты e2e-рана) $2=run_id
# Output: prints "infra" / "feature"; rc=0 always.
detect_fail_kind() {  # $1=artifact_dir $2=run_id
    local dir="$1" run_id="$2" tmpdir="" found=0
    tmpdir="$(mktemp -d 2>/dev/null || echo "${WORKTREE_DIR}/.e2e-infra-$$")"
    mkdir -p "$tmpdir" 2>/dev/null || true
    # 1) Артефакты рана (логи робота voice_e2e_*.log, acceptance, ...).
    if [ -d "$dir" ]; then
        if grep -rhiE \
            -e '429[[:space:]]+Too Many Requests' \
            -e 'RateLimitError' \
            -e 'Token Plan usage limit' \
            -e 'API error 2056' \
            -e 'Silero v5 fallback' \
            -e 'MiniMax auth error' \
            -e 'переключаюсь на Silero' \
            -e 'Connection refused' \
            -e 'Connection timed out' \
            -e 'no route to host' \
            -e 'ssh:[[:space:]]+connect to host' \
            -e 'E2E_NO_REACTION' \
            "$dir" 2>/dev/null | grep -q .; then
            found=1
        fi
    fi
    # 2) Console-логи рана (фолбэк если артефакт-лог не загрузился).
    if [ "$found" -eq 0 ] && [ -n "$run_id" ]; then
        if curl -sL --max-time 60 -H "Authorization: token $(gh auth token 2>/dev/null || true)" \
            "https://api.github.com/repos/${GH_REPO}/actions/runs/${run_id}/logs" -o "${tmpdir}/run_logs.zip" 2>/dev/null \
            && unzip -o -q "${tmpdir}/run_logs.zip" -d "${tmpdir}/logs" 2>/dev/null; then
            if grep -rhiE \
                -e '429[[:space:]]+Too Many Requests' \
                -e 'RateLimitError' \
                -e 'Token Plan usage limit' \
                -e 'API error 2056' \
                -e 'Silero v5 fallback' \
                -e 'MiniMax auth error' \
                -e 'переключаюсь на Silero' \
                -e 'Connection refused' \
                -e 'Connection timed out' \
                -e 'no route to host' \
                -e 'ssh:[[:space:]]+connect to host' \
                -e 'E2E_NO_REACTION' \
                "${tmpdir}/logs" 2>/dev/null | grep -q .; then
                found=1
            fi
        fi
    fi
    rm -rf "$tmpdir" 2>/dev/null || true
    if [ "$found" -eq 1 ]; then printf 'infra'; else printf 'feature'; fi
}

# --- retry-with-backoff for gh workflow run ----------------------------------
# ретро 10.08 #1: gh workflow run может вернуть non-zero exit сразу после
# свежего push ветки (GitHub API ещё не проиндексировал ref). При этом workflow
# реально стартует — e2e_workflow потом появляется в gh run list. Решение:
# ретрай с backoff 5/10/15s, до 3 попыток. Скрипт останавливается раньше,
# только если ВСЕ попытки провалились.
_trigger_workflow_with_retry() {
    local _wf_name="$1"; shift
    local _attempts=0 _max=3 _sleep
    while [ "$_attempts" -lt "$_max" ]; do
        if gh workflow run "$_wf_name" --repo "$GH_REPO" "$@" >/dev/null 2>&1; then
            return 0
        fi
        _attempts=$((_attempts + 1))
        if [ "$_attempts" -lt "$_max" ]; then
            _sleep=$((_attempts * 5))
            log "    trigger ${_wf_name}: retry ${_attempts}/${_max} in ${_sleep}s (race-condition workaround)"
            sleep "$_sleep"
        fi
    done
    return 1
}

# Процесс-фикс (09.08): освободить ветку карточки от worktree старых
# (done/archived) карточек — иначе респавн падает «git worktree add failed»
# и карточка навсегда виснет в blocked. Путь worktree берём из самой карточки
# (kanban workspace_path), список worktree — через git -C <wt> (родительский клон).
free_stale_worktrees_for() {  # $1=task_id (t_<hex>)
    local task_id="$1" my_wt my_branch line wt_path wt_branch owner
    my_wt="$("$HERMES_BIN" kanban --board "$KANBAN_BOARD" show "$task_id" --json 2>/dev/null \
        | python3 -c 'import sys,json
try:
    d=json.load(sys.stdin); print(d.get("task",{}).get("workspace_path") or "")
except Exception: print("")' 2>/dev/null || true)"
    if [ -z "$my_wt" ] || [ ! -d "$my_wt" ]; then
        return 0
    fi
    my_branch="$(git -C "$my_wt" branch --show-current 2>/dev/null || true)"
    [ -z "$my_branch" ] && return 0
    wt_path=""
    while IFS= read -r line; do
        case "$line" in
            worktree\ *) wt_path="${line#worktree }" ;;
            branch\ *)
                wt_branch="${line#branch refs/heads/}"
                if [ "$wt_branch" = "$my_branch" ] && [ "$wt_path" != "$my_wt" ]; then
                    owner="$(basename "$wt_path")"
                    if [ "$owner" != "$task_id" ]; then
                        git -C "$my_wt" worktree remove --force "$wt_path" 2>/dev/null \
                            && log "  freed stale worktree $wt_path (branch $my_branch, card $owner)"
                    fi
                fi
                ;;
        esac
    done < <(git -C "$my_wt" worktree list --porcelain 2>/dev/null)
    git -C "$my_wt" worktree prune 2>/dev/null || true
    return 0
}

# Read PR head branch from issue + title.
compute_agent_branch() {  # $1=issue_number $2=title
    local n="$1" t="$2"
    printf '%s' "z-{agent}/${n}-$(slugify "$t")"
}

# --- process each issue ------------------------------------------------------
processed=0
errored=0
skipped=0
# Each per-issue pipeline step is bounded by E2E_CI_TIMEOUT + E2E_RUN_TIMEOUT.
# We process all `needs-e2e` issues per tick; failures on one issue do not
# stop the others. The chron rhythm (every 1h) caps the effective capacity.

while IFS=$'\t' read -r number title labels body; do
    [ -z "$number" ] && continue

    labels_norm="$(printf '%s' "$labels" | tr '[:upper:]' '[:lower:]')"

    # Idempotency: skip if already past e2e.
    if has_label "$labels_norm" "$DONE_LABEL" || has_label "$labels_norm" "$REJECTED_LABEL"; then
        log "issue #${number} already has ${DONE_LABEL}/${REJECTED_LABEL} — skip"
        skipped=$((skipped+1)); continue
    fi

    branch="$(compute_agent_branch "$number" "$title")"

    # task_id из marker-коммента (нужен для fallback wt/ и unblock в конце).
    e2e_task_id="$(gh issue view "$number" --repo "$GH_REPO" --comments --json comments \
        --jq '.comments[].body' 2>/dev/null \
        | grep -Eo '^kanban: t_[a-f0-9]+' \
        | tail -n1 \
        | sed 's/^kanban: //' || true)"

    # --- e2e params from ISSUE BODY (воркер пишет блок в карточке — контракт) ---
    # Формат в body issue:
    #   ## e2e
    #   voice_text: "Робот, спой песенку про енотика"
    #   voice_file: .github/e2e/voice_commands/spoy_peasenku_pro_enotika.ogg
    #   volume: 100
    #   record_seconds: 90
    #   llm: minimax-m2
    #   tts: minimax-male-qn-qingse
    #   stt: yandex
    # Если блока нет — дефолты (env/скрипт).
    e2e_voice_text=""
    e2e_voice_file=""
    e2e_volume=""
    e2e_record_seconds=""
    e2e_llm=""
    e2e_tts=""
    e2e_stt=""
    e2e_acceptance_check=""
    # Python-парсер issues_json экранирует переносы в литеральные \n — вернём реальные.
    # (иначе grep '^voice_text' не находит поле в середине однострочного body,
    #  а grep exit 1 + pipefail + set -e убивают скрипт)
    body_real="$(printf '%s' "$body" | sed 's/\\n/\
/g')"
    body_lower="$(printf '%s' "$body_real" | tr '[:upper:]' '[:lower:]')"
    if printf '%s' "$body_lower" | grep -q '## e2e'; then
        # Извлекаем значения: строка "voice_text: ..." / "volume: 100" и т.п.
        e2e_voice_text="$(printf '%s' "$body_real" | grep -iE '^[[:space:]]*voice_text[[:space:]]*:' | head -1 | sed -E 's/^[[:space:]]*voice_text[[:space:]]*:[[:space:]]*//; s/^"//; s/"$//' || true)"
        e2e_voice_file="$(printf '%s' "$body_real" | grep -iE '^[[:space:]]*voice_file[[:space:]]*:' | head -1 | sed -E 's/^[[:space:]]*voice_file[[:space:]]*:[[:space:]]*//; s/^"//; s/"$//' || true)"
        e2e_volume="$(printf '%s' "$body_real" | grep -iE '^[[:space:]]*volume[[:space:]]*:' | head -1 | sed -E 's/^[[:space:]]*volume[[:space:]]*:[[:space:]]*//' || true)"
        e2e_record_seconds="$(printf '%s' "$body_real" | grep -iE '^[[:space:]]*record_seconds[[:space:]]*:' | head -1 | sed -E 's/^[[:space:]]*record_seconds[[:space:]]*:[[:space:]]*//' || true)"
        e2e_llm="$(printf '%s' "$body_real" | grep -iE '^[[:space:]]*llm[[:space:]]*:' | head -1 | sed -E 's/^[[:space:]]*llm[[:space:]]*:[[:space:]]*//; s/^"//; s/"$//' || true)"
        e2e_tts="$(printf '%s' "$body_real" | grep -iE '^[[:space:]]*tts[[:space:]]*:' | head -1 | sed -E 's/^[[:space:]]*tts[[:space:]]*:[[:space:]]*//; s/^"//; s/"$//' || true)"
        e2e_stt="$(printf '%s' "$body_real" | grep -iE '^[[:space:]]*stt[[:space:]]*:' | head -1 | sed -E 's/^[[:space:]]*stt[[:space:]]*:[[:space:]]*//; s/^"//; s/"$//' || true)"
        e2e_acceptance_check="$(printf '%s' "$body_real" | grep -iE '^[[:space:]]*acceptance_check[[:space:]]*:' | head -1 | sed -E 's/^[[:space:]]*acceptance_check[[:space:]]*:[[:space:]]*//; s/^"//; s/"$//' || true)"
        log "issue #${number}: e2e params from body: volume=${e2e_volume:-default} voice_text=${e2e_voice_text:-default} llm=${e2e_llm:-default} tts=${e2e_tts:-default} stt=${e2e_stt:-default} acceptance_check=${e2e_acceptance_check:-default}"
    else
        log "issue #${number}: no '## e2e' block in body — using defaults"
    fi
    # Параметры с приоритетом: body > env > скрипт-дефолт
    [ -n "$e2e_volume" ] && E2E_VOLUME="$e2e_volume"

    # Look up agent PR. Allow either OPEN (CI green per merge-gate) or MERGED.
    pr_state="$(gh pr list --repo "$GH_REPO" --state all --head "$branch" \
        --json number,state,headRefName --jq 'if length>0 then .[0].state else "NONE" end' 2>/dev/null || echo NONE)"
    # Процесс-фикс (09.08): fallback на ветку wt/<task_id> (ретро-карточки
    # без issue создают PR с веткой wt/, а не z-{agent}/<id>-<slug>).
    if [ "$pr_state" = "NONE" ] && [ -n "${e2e_task_id:-}" ]; then
        wt_branch="wt/${e2e_task_id}"
        pr_state="$(gh pr list --repo "$GH_REPO" --state all --head "$wt_branch" \
            --json number,state,headRefName --jq 'if length>0 then .[0].state else "NONE" end' 2>/dev/null || echo NONE)"
        if [ "$pr_state" != "NONE" ]; then
            branch="$wt_branch"
            log "issue #${number}: PR найден по fallback-ветке ${wt_branch}"
        fi
    fi
    # Follow-up PR поверх MERGED-фикса (ретро 10.08, архитектор): каноническая
    # ветка z-{agent}/<id>-<slug> уже MERGED (issue e2e-done от старого PR),
    # но воркер открыл НОВЫЙ OPEN PR с номером issue в title (#1099 поверх
    # #1082 на ветке z-backend/t_d431ca0b-*; #1098 поверх #1052). Раньше такой
    # follow-up навсегда выпадал из e2e-ротации. Теперь: если канонический PR
    # MERGED, ищем OPEN follow-up по "${number} in:title" (CLEAN/MERGEABLE)
    # и тестируем ЕГО ветку.
    if [ "$pr_state" = "MERGED" ]; then
        _followup="$(gh pr list --repo "$GH_REPO" --state open \
            --search "${number} in:title" \
            --json number,headRefName,mergeStateStatus \
            --jq '[.[] | select(.mergeStateStatus == "CLEAN" or .mergeStateStatus == "MERGEABLE")][0] | "\(.number)\t\(.headRefName)"' 2>/dev/null || echo "")"
        if [ -n "$_followup" ] && [ "$_followup" != "null" ]; then
            _followup_num="${_followup%%$'\t'*}"
            _followup_head="${_followup#*$'\t'}"
            if [ -n "$_followup_num" ] && [ "$_followup_num" != "null" ]; then
                log "issue #${number}: follow-up OPEN PR #${_followup_num} (${_followup_head}) — тестируем вместо MERGED ${branch}"
                branch="$_followup_head"
                pr_state="OPEN"
            fi
        fi
    fi
    if [ "$pr_state" = "NONE" ]; then
        log "issue #${number}: no PR for ${branch} — merge-gate likely stale — skip"
        skipped=$((skipped+1)); continue
    fi
    pr_number="$(gh pr list --repo "$GH_REPO" --state all --head "$branch" \
        --json number --jq 'if length>0 then .[0].number else "" end' 2>/dev/null || echo "")"

    # Получаем labels/title PR для detect_pr_kind (lint vs functional).
    pr_meta="$(gh pr view "$pr_number" --repo "$GH_REPO" --json title,labels \
        --jq '{title: .title, labels: ([.labels[].name] | join(","))}' 2>/dev/null || echo '{}')"
    pr_title="$(printf '%s' "$pr_meta" | python3 -c 'import json,sys
try:
    print(json.load(sys.stdin).get("title",""))
except Exception: pass' 2>/dev/null || true)"
    pr_labels_csv="$(printf '%s' "$pr_meta" | python3 -c 'import json,sys
try:
    print((json.load(sys.stdin).get("labels","") or "").lower())
except Exception: pass' 2>/dev/null || true)"
    pr_kind="$(detect_pr_kind "${pr_labels_csv:-}" "${pr_title:-}")"

    # --- lint-ветка (ретро t_d0151eb3): e2e не нужен -----------------
    # Линт/refactor-PR — только стиль/докстринг/перенос строк. CI green (Lint
    # Code + Run Tests) достаточно. e2e на роботе жечь не нужно.
    # Действия: снять needs-e2e с PR+issue, поставить needs-review на PR,
    # опубликовать короткий комментарий в PR, разблокировать карточку.
    if [ "$pr_kind" = "lint" ]; then
        log "issue #${number}: PR #${pr_number} is ${pr_kind} — skipping e2e, posting short notice to PR"
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would: lint-skip for PR #${pr_number} (comment + needs-review + unblock)"
            processed=$((processed+1)); continue
        fi
        # 1) PR: снять needs-e2e, поставить needs-review.
        gh pr edit "$pr_number" --repo "$GH_REPO" --remove-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
        gh pr edit "$pr_number" --repo "$GH_REPO" --add-label "$NEEDS_REVIEW_LABEL" >/dev/null 2>&1 || true
        # 2) PR: короткий комментарий «только форматирование, e2e не нужен».
        gh pr comment "$pr_number" --repo "$GH_REPO" --body \
            "$(cat <<EOF
agent-flow: ℹ️ lint/refactor PR — e2e на роботе не требуется (CI green достаточно).
Снято: \`${NEEDS_E2E_LABEL}\`. Поставлено: \`${NEEDS_REVIEW_LABEL}\`.
Карточка разблокирована с причиной \`review-required: lint-PR #${pr_number}, e2e не нужен\`.
EOF
)" >/dev/null 2>&1 || log "issue #${number}: WARNING short PR-comment for lint failed"
        # 3) Issue: снять needs-e2e, поставить no-e2e-required для истории.
        gh issue edit "$number" --repo "$GH_REPO" --remove-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
        gh issue edit "$number" --repo "$GH_REPO" --add-label "$NO_E2E_LABEL" >/dev/null 2>&1 || true
        gh issue comment "$number" --repo "$GH_REPO" --body \
            "agent-flow: ℹ️ PR #${pr_number} — lint/refactor (CI green достаточно). \`${NEEDS_E2E_LABEL}\` снят, \`${NO_E2E_LABEL}\` поставлен для истории. \`${NEEDS_REVIEW_LABEL}\` стоит на PR." >/dev/null 2>&1 || true
        # 4) Разблокировать карточку (если знаем task_id).
        if [ -n "${e2e_task_id:-}" ]; then
            free_stale_worktrees_for "$e2e_task_id" || true
            if "$HERMES_BIN" kanban --board "$KANBAN_BOARD" unblock "$e2e_task_id" >/dev/null 2>&1; then
                log "issue #${number}: card ${e2e_task_id} unblocked (lint-skip, PR #${pr_number})"
            else
                log "issue #${number}: WARNING unblock failed for ${e2e_task_id}"
            fi
        fi
        processed=$((processed+1)); continue
    fi

    # --- Процесс-фикс (09.08, ретро #9): fallback на PR body ---
    # Воркер писал блок ## e2e в PR body, а контракт читает body ISSUE (#1077).
    # Если в issue body блока нет — пробуем прочитать из PR body.
    if [ -z "$e2e_voice_text" ] && [ -z "$e2e_voice_file" ] && [ -n "$pr_number" ]; then
        pr_body="$(gh pr view "$pr_number" --repo "$GH_REPO" --json body --jq '.body' 2>/dev/null || echo "")"
        if [ -n "$pr_body" ]; then
            pr_body_real="$(printf '%s' "$pr_body" | sed 's/\\n/\n/g')"
            pr_body_lower="$(printf '%s' "$pr_body_real" | tr '[:upper:]' '[:lower:]')"
            if printf '%s' "$pr_body_lower" | grep -q '## e2e'; then
                e2e_voice_text="$(printf '%s' "$pr_body_real" | grep -iE '^[[:space:]]*voice_text[[:space:]]*:' | head -1 | sed -E 's/^[[:space:]]*voice_text[[:space:]]*:[[:space:]]*//; s/^"//; s/"$//' || true)"
                e2e_voice_file="$(printf '%s' "$pr_body_real" | grep -iE '^[[:space:]]*voice_file[[:space:]]*:' | head -1 | sed -E 's/^[[:space:]]*voice_file[[:space:]]*:[[:space:]]*//; s/^"//; s/"$//' || true)"
                e2e_volume="$(printf '%s' "$pr_body_real" | grep -iE '^[[:space:]]*volume[[:space:]]*:' | head -1 | sed -E 's/^[[:space:]]*volume[[:space:]]*:[[:space:]]*//' || true)"
                e2e_record_seconds="$(printf '%s' "$pr_body_real" | grep -iE '^[[:space:]]*record_seconds[[:space:]]*:' | head -1 | sed -E 's/^[[:space:]]*record_seconds[[:space:]]*:[[:space:]]*//' || true)"
                e2e_llm="$(printf '%s' "$pr_body_real" | grep -iE '^[[:space:]]*llm[[:space:]]*:' | head -1 | sed -E 's/^[[:space:]]*llm[[:space:]]*:[[:space:]]*//; s/^"//; s/"$//' || true)"
                e2e_tts="$(printf '%s' "$pr_body_real" | grep -iE '^[[:space:]]*tts[[:space:]]*:' | head -1 | sed -E 's/^[[:space:]]*tts[[:space:]]*:[[:space:]]*//; s/^"//; s/"$//' || true)"
                e2e_stt="$(printf '%s' "$pr_body_real" | grep -iE '^[[:space:]]*stt[[:space:]]*:' | head -1 | sed -E 's/^[[:space:]]*stt[[:space:]]*:[[:space:]]*//; s/^"//; s/"$//' || true)"
                e2e_acceptance_check="$(printf '%s' "$pr_body_real" | grep -iE '^[[:space:]]*acceptance_check[[:space:]]*:' | head -1 | sed -E 's/^[[:space:]]*acceptance_check[[:space:]]*:[[:space:]]*//; s/^"//; s/"$//' || true)"
                log "issue #${number}: e2e params from PR #${pr_number} body (issue body had no ## e2e)"
                [ -n "$e2e_volume" ] && E2E_VOLUME="$e2e_volume"
            fi
        fi
    fi

    # Make sure worktree knows about the agent branch.
    git -C "$WORKTREE_DIR" fetch origin "$branch" --quiet 2>/dev/null || true

    # --- merge agent branch DIRECTLY into test-round-N (no wip layer) ---
    # Q20-rework: промежуточная wip-ветка не нужна. z-{agent}/<id>-<slug> уже
    # прошёл CI (PR в develop, merge-gate поставил needs-e2e) → льём сразу в round.
    log "issue #${number}: merging ${branch} directly into ${ROUND_BRANCH}"
    if [ "$DRY_RUN" = "true" ]; then
        log "DRY-RUN would: merge ${branch} into ${ROUND_BRANCH} and push"
        processed=$((processed+1)); continue
    fi

    git -C "$WORKTREE_DIR" fetch origin "$branch" --quiet 2>/dev/null || true
    # Refresh the round ref RIGHT BEFORE checkout. The build workflow pushes
    # "[skip ci]" SHA-tag commits back to the round branch, so origin/<round>
    # can go stale between the previous push and this merge (observed 09.08:
    # 2nd issue in a tick failed with "push rejected (fetch first)" because
    # checkout -B based on a pre-CI origin ref). Explicit force refspec, no
    # silent failure — stale ref here costs a whole e2e round.
    if ! git -C "$WORKTREE_DIR" fetch origin "+refs/heads/${ROUND_BRANCH}:refs/remotes/origin/${ROUND_BRANCH}" --quiet 2>/dev/null; then
        log "issue #${number}: WARNING refresh fetch of ${ROUND_BRANCH} failed — checkout may be stale"
    fi
    if ! git -C "$WORKTREE_DIR" checkout -B "$ROUND_BRANCH" "origin/${ROUND_BRANCH}" 2>&1 | sed 's/^/  /'; then
        log "issue #${number}: cannot checkout ${ROUND_BRANCH}"; errored=$((errored+1)); continue
    fi

    if ! git -C "$WORKTREE_DIR" merge --no-ff -m "agent-flow: merge ${branch} for issue #${number}" \
        "origin/${branch}" 2>&1 | sed 's/^/  /'; then
        log "issue #${number}: merge conflict rolling ${branch} into ${ROUND_BRANCH} — manual resolution required"
        # Процесс-фикс (10.08, ретро #3, архитектор): конфликт с develop НЕ сама ошибка
        # воркера — это нормальная ситуация когда develop убежал. Воркер должен
        # разрешить конфликт в ТОЙ ЖЕ ветке (никаких новых веток — Шифу прямо),
        # запушить, дождаться следующего прогона. Создаём карточку воркеру с
        # assignee=профиль по метке issue (agent:backend → backend, etc).
        _conflict_assignee="default"
        for lbl in $(gh issue view "$number" --repo "$GH_REPO" --json labels --jq '[.labels[].name] | .[]' 2>/dev/null); do
            case "$lbl" in
                agent:backend)    _conflict_assignee="backend"; break ;;
                agent:developer)  _conflict_assignee="developer"; break ;;
                agent:devops)     _conflict_assignee="devops"; break ;;
                agent:architect)  _conflict_assignee="architect"; break ;;
            esac
        done
        _conflict_body="## 🔀 merge conflict: \`${branch}\` → \`${ROUND_BRANCH}\` (ретро 10.08)

**ПРИЧИНА:** develop убежал вперёд, твоя ветка \`${branch}\` (PR #${pr_number:-?}) не мерджится напрямую.

**ОБЯЗАН** (по процессу Шифу 10.08):
1. **В той же ветке** (\`${branch}\`) — НЕ создавай новую ветку и НЕ новый PR (Шифу прямо: «не плодить ветки»).
2. **rebase** на origin/develop: \`git fetch origin develop && git rebase origin/develop\` (или merge develop в свою ветку, как удобнее — но В ТОЙ ЖЕ ветке).
3. Разреши конфликты → \`git add ... && git rebase --continue\` (или commit для merge).
4. \`git push --force-with-lease origin ${branch}\` (force-with-lease безопасен, ты не перезапишешь чужую работу).
5. **Метки снимать НЕ надо** (PR уже needs-e2e). Следующий тик e2e-process снова попробует merge в round → если конфликт разрешён → прогон пойдёт.
6. Карточка закрывается когда увидишь прогон своего PR в новом round.

**Запрещено:** плодить ветки, создавать новый PR, мержить в develop, ставить needs-review.

**Команды шпаргалка:**
\`\`\`bash
git fetch origin develop
git checkout ${branch}
git rebase origin/develop
# ... resolve conflicts ...
git add -A
git rebase --continue
git push --force-with-lease origin ${branch}
\`\`\`"
        _conflict_title="🔀 merge conflict: \`${branch}\` vs develop (issue #${number})"
        # Создаём (или обновляем существующую) карточку воркеру
        _existing_conflict="$(hermes kanban --board "$KANBAN_BOARD" list --json 2>/dev/null | python3 -c "
import json,sys
data = json.loads(sys.stdin.read())
for t in data:
    if t.get('status') in ('ready','running','blocked','todo') and t.get('title','').startswith('🔀 merge conflict:') and '${branch}' in t.get('title',''):
        print(t['id'])
        break
" 2>/dev/null | head -1)"
        if [ -n "$_existing_conflict" ]; then
            hermes kanban --board "$KANBAN_BOARD" comment "$_existing_conflict" "Конфликт остался при повторной попытке. Воркер всё ещё не разрешил. Шифу напомнил — **та же ветка, тот же PR, никаких новых**." >/dev/null 2>&1 || true
            log "issue #${number}: conflict card ${_existing_conflict} already exists — appended reminder"
        else
            hermes kanban --board "$KANBAN_BOARD" create \
                --assignee "$_conflict_assignee" \
                --priority 90 \
                --max-runtime 1800 \
                --body "$_conflict_body" \
                "$_conflict_title" >/dev/null 2>&1 || log "issue #${number}: WARNING conflict card create failed (${_conflict_assignee})"
            log "issue #${number}: conflict card created for ${_conflict_assignee}"
        fi

        gh issue edit "$number" --repo "$GH_REPO" --add-label "$REJECTED_LABEL" >/dev/null 2>&1 \
            --remove-label "$NEEDS_E2E_LABEL" 2>/dev/null || true
        gh issue comment "$number" --repo "$GH_REPO" --body \
            "agent-flow: 🔀 merge conflict rolling \`${branch}\` into \`${ROUND_BRANCH}\`. Карточка воркеру создана — **тот же PR, та же ветка, разреши конфликт rebase на origin/develop**, push --force-with-lease. Следующий тик попробует снова." >/dev/null 2>&1 || true
        git -C "$WORKTREE_DIR" merge --abort 2>/dev/null || true
        errored=$((errored+1)); continue
    fi

    # --- push round (with merged agent branch) ---
    # --force-with-lease: if the build workflow pushed "[skip ci]" tag
    # commits between our refresh-fetch and this push, reject safely instead
    # of clobbering them (or failing a blind force). A rejected push just
    # means the issue stays needs-e2e and the next tick retries on a new round.
    if ! git -C "$WORKTREE_DIR" push --force-with-lease origin "$ROUND_BRANCH" 2>&1 | sed 's/^/  /'; then
        log "issue #${number}: push of ${ROUND_BRANCH} failed"; errored=$((errored+1)); continue
    fi
    log "issue #${number}: ${branch} merged & pushed into ${ROUND_BRANCH}"

    # Refresh round branch in worktree.
    git -C "$WORKTREE_DIR" fetch origin "$ROUND_BRANCH" --quiet 2>/dev/null || true

    # --- Процесс-фикс (09.08, ретро #8): пред-проверка e2e-контракта ДО сборки ---
    # voice_file=spoy_new.ogg не существовал → ensure_voice_file падал через
    # 20 мин сборки (scp fail). Проверяем файл в round-ветке СЕЙЧАС:
    #  - если voice_file задан и есть в репо → ок (ensure_voice_file скопирует)
    #  - если voice_file задан, но в репо НЕТ и voice_text пуст → fail-fast:
    #    коммент в issue + skip (без 20-мин build/deploy впустую)
    if [ -n "$e2e_voice_file" ]; then
        if ! git -C "$WORKTREE_DIR" ls-files --error-unmatch -- "$e2e_voice_file" >/dev/null 2>&1; then
            if [ -z "$e2e_voice_text" ]; then
                log "issue #${number}: voice_file='${e2e_voice_file}' отсутствует в репо и voice_text пуст — fail-fast"
                gh issue comment "$number" --repo "$GH_REPO" --body \
                    "agent-flow: ❌ e2e-контракт невалиден: voice_file \`${e2e_voice_file}\` не найден в round-ветке, voice_text не задан (ensure_voice_file не сможет сгенерить). Поправь блок \`## e2e\` в body ISSUE: voice_file должен существовать в репо (.github/e2e/voice_commands/...) ИЛИ задай voice_text для генерации." >/dev/null 2>&1 || true
                skipped=$((skipped+1)); continue
            else
                log "issue #${number}: voice_file='${e2e_voice_file}' не в репо, но voice_text задан — ensure_voice_file сгенерит на лету"
            fi
        fi
    fi

    # --- chain: BUILD → DEPLOY → E2E on round branch (Q20-rework) ---
    # e2e-скрипт НЕ запускается пока round не собран и не задеплоен.
    # 1) L: Build All Services на round  → ждём success
    # 2) L: Deploy and Verify на round   → ждём success
    # 3) L: E2E Voice Test на round      → ждём verdict
    wait_workflow() {  # $1=workflow_name $2=branch $3=timeout_s $4=label $5=min_created_epoch
        local wf="$1" br="$2" tmo="$3" lbl="$4" min_epoch="${5:-0}" rid="" st="" dl
        # Ждём ПОЯВЛЕНИЯ нового run (createdAt >= момента триггера)
        dl=$((SECONDS + 120))
        while [ "$SECONDS" -lt "$dl" ]; do
            rid="$(gh run list --repo "$GH_REPO" --workflow "$wf" --branch "$br" \
                --limit 3 --json databaseId,createdAt --jq "[.[] | select(.createdAt >= \"$min_epoch\")][0].databaseId" 2>/dev/null || echo "")"
            if [ -n "$rid" ] && [ "$rid" != "null" ] && [ "$rid" != "" ]; then
                break
            fi
            sleep 5
        done
        if [ -z "$rid" ] || [ "$rid" = "null" ]; then
            log "issue #${number}: ${lbl} run not created"; return 1
        fi
        log "issue #${number}: ${lbl} run ${rid} created — waiting (timeout ${tmo}s)"
        dl=$((SECONDS + tmo))
        while [ "$SECONDS" -lt "$dl" ]; do
            st="$(gh run view "$rid" --repo "$GH_REPO" --json status --jq '.status' 2>/dev/null || echo "")"
            if [ "$st" = "completed" ]; then
                local concl _c_try
                concl=""
                # Ретро-фикс (09.08 #4): conclusion иногда пустой сразу после
                # completed (gh run view гонка) → success считался FAILURE.
                # Перечитываем до 3 раз с паузой, только потом вердикт.
                for _c_try in 1 2 3; do
                    concl="$(gh run view "$rid" --repo "$GH_REPO" --json conclusion --jq '.conclusion' 2>/dev/null || echo "")"
                    if [ -n "$concl" ] && [ "$concl" != "null" ]; then break; fi
                    sleep 5
                done
                if [ "$concl" = "success" ]; then
                    log "issue #${number}: ${lbl} OK (run ${rid})"
                    return 0
                else
                    log "issue #${number}: ${lbl} FAILED (run ${rid}, ${concl:-unknown})"
                    return 1
                fi
            fi
            sleep "$E2E_POLL_INTERVAL"
        done
        log "issue #${number}: ${lbl} TIMEOUT (${tmo}s)"
        return 1
    }

    # 1) Build
    log "issue #${number}: triggering ${BUILD_WORKFLOW} on ${ROUND_BRANCH} (push_to_registry=true)"
    b_epoch="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    # ретро 10.08 #1: race condition gh workflow run после свежего push.
    # API может вернуть non-zero exit, но workflow стартует. До 3 ретраев с backoff 5/10/15s.
    if ! _trigger_workflow_with_retry "$BUILD_WORKFLOW" --ref "$ROUND_BRANCH" \
        -f push_to_registry=true; then
        log "issue #${number}: failed to trigger ${BUILD_WORKFLOW} after retries"; errored=$((errored+1)); continue
    fi
    if ! wait_workflow "$BUILD_WORKFLOW" "$ROUND_BRANCH" "$E2E_BUILD_TIMEOUT" "build" "$b_epoch"; then
        gh issue comment "$number" --repo "$GH_REPO" --body \
            "agent-flow: ❌ build failed on ${ROUND_BRANCH} — e2e skipped. See https://github.com/${GH_REPO}/actions" >/dev/null 2>&1 || true
        errored=$((errored+1)); continue
    fi

    # 2) Deploy
    log "issue #${number}: triggering ${DEPLOY_WORKFLOW} on ${ROUND_BRANCH} (env=${E2E_DEPLOY_ENV}, registry=${E2E_DEPLOY_REGISTRY})"
    d_epoch="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    if ! _trigger_workflow_with_retry "$DEPLOY_WORKFLOW" --ref "$ROUND_BRANCH" \
        -f environment="$E2E_DEPLOY_ENV" -f registry_source="$E2E_DEPLOY_REGISTRY"; then
        log "issue #${number}: failed to trigger ${DEPLOY_WORKFLOW} after retries"; errored=$((errored+1)); continue
    fi
    if ! wait_workflow "$DEPLOY_WORKFLOW" "$ROUND_BRANCH" "$E2E_DEPLOY_TIMEOUT" "deploy" "$d_epoch"; then
        gh issue comment "$number" --repo "$GH_REPO" --body \
            "agent-flow: ❌ deploy failed on ${ROUND_BRANCH} — e2e skipped. See https://github.com/${GH_REPO}/actions" >/dev/null 2>&1 || true
        errored=$((errored+1)); continue
    fi

    # 2b) Pre-flight: робот жив и healthy перед e2e (ретро #R2 — no-reaction rounds 29/31).
    # Если контейнер недоступен/не healthy — не жечь e2e-раунд, а сразу e2e:rejected с причиной.
    # Креды — из окружения (E2E_ROBOT_PASS), паролей в скрипте нет.
    if [ -n "$E2E_ROBOT_PASS" ]; then
        robot_state="$(sshpass -p "$E2E_ROBOT_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 \
            "$E2E_ROBOT_USER@$E2E_ROBOT_HOST" \
            "docker ps --filter name=voice-assistant --format '{{.Status}}' 2>/dev/null; docker logs voice-assistant --since 30m 2>&1 | grep -cE 'ERROR|died'" 2>/dev/null || true)"
        state_line="$(printf '%s\n' "$robot_state" | sed -n 1p)"
        err_count="$(printf '%s\n' "$robot_state" | sed -n 2p)"
        log "issue #${number}: robot pre-flight: state='${state_line:-NO-ANSWER}' errors30m=${err_count:-?}"
        if [ -z "$state_line" ] || ! printf '%s' "$state_line" | grep -q "healthy"; then
            gh issue comment "$number" --repo "$GH_REPO" --body \
                "agent-flow: ❌ e2e skipped — voice-assistant не healthy на роботе (pre-flight: '${state_line:-NO-ANSWER}', errors30m=${err_count:-?}). Починить робота → повторный прогон." >/dev/null 2>&1 || true
            log "issue #${number}: robot pre-flight FAILED (${state_line:-NO-ANSWER}) — e2e skipped"
            errored=$((errored+1)); continue
        fi
    else
        log "issue #${number}: robot pre-flight SKIPPED (E2E_ROBOT_PASS не задан)"
    fi

    # 3) E2E voice test
    log "issue #${number}: triggering ${E2E_WORKFLOW} on ${ROUND_BRANCH} (volume=${E2E_VOLUME}, voice_text=${e2e_voice_text:-default})"
    e_epoch="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    e2e_args=()
    [ -n "$E2E_VOLUME" ] && e2e_args+=(-f "volume=$E2E_VOLUME")
    [ -n "$e2e_voice_text" ] && e2e_args+=(-f "voice_text=$e2e_voice_text")
    [ -n "$e2e_voice_file" ] && e2e_args+=(-f "voice_file=$e2e_voice_file")
    [ -n "$e2e_record_seconds" ] && e2e_args+=(-f "record_seconds=$e2e_record_seconds")
    [ -n "$e2e_llm" ] && e2e_args+=(-f "llm=$e2e_llm")
    [ -n "$e2e_tts" ] && e2e_args+=(-f "tts=$e2e_tts")
    [ -n "$e2e_stt" ] && e2e_args+=(-f "stt=$e2e_stt")
    [ -n "$e2e_acceptance_check" ] && e2e_args+=(-f "acceptance_check=$e2e_acceptance_check")
    if ! _trigger_workflow_with_retry "$E2E_WORKFLOW" --ref "$ROUND_BRANCH" "${e2e_args[@]}"; then
        log "issue #${number}: failed to trigger ${E2E_WORKFLOW} after retries"; errored=$((errored+1)); continue
    fi

    # --- wait for verdict (только СВЕЖИЙ run, createdAt >= момента триггера) ---
    log "issue #${number}: waiting verdict (timeout ${E2E_RUN_TIMEOUT}s)"
    deadline=$((SECONDS + E2E_RUN_TIMEOUT))
    run_id=""
    verdict=""
    while [ "$SECONDS" -lt "$deadline" ]; do
        run_id="$(gh run list --repo "$GH_REPO" --workflow "$E2E_WORKFLOW" --branch "$ROUND_BRANCH" \
            --limit 3 --json databaseId,createdAt --jq "[.[] | select(.createdAt >= \"$e_epoch\")][0].databaseId" 2>/dev/null || echo "")"
        if [ -n "$run_id" ] && [ "$run_id" != "null" ] && [ "$run_id" != "" ]; then
            status="$(gh run view "$run_id" --repo "$GH_REPO" --json status --jq '.status' 2>/dev/null || echo "")"
            if [ "$status" = "completed" ]; then
                # Ретро-фикс (09.08 #4): conclusion иногда пустой сразу после
                # completed — перечитываем до 3 раз, иначе success → FAILURE.
                verdict=""
                for _v_try in 1 2 3; do
                    verdict="$(gh run view "$run_id" --repo "$GH_REPO" --json conclusion --jq '.conclusion' 2>/dev/null || echo "")"
                    if [ -n "$verdict" ] && [ "$verdict" != "null" ]; then break; fi
                    sleep 5
                done
                break
            fi
        fi
        sleep "$E2E_POLL_INTERVAL"
    done

    if [ -z "$verdict" ]; then
        log "issue #${number}: e2e verdict timeout — manual review needed"
        gh issue edit "$number" --repo "$GH_REPO" --remove-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
        gh issue comment "$number" --repo "$GH_REPO" --body \
            "agent-flow: ❌ e2e verdict timeout (run id ${run_id:-unknown}). Manual review: https://github.com/${GH_REPO}/actions/runs/${run_id:-}" >/dev/null 2>&1 || true
        errored=$((errored+1)); continue
    fi

    # --- download artifact (best-effort) ---
    artifact_dir="${WORKTREE_DIR}/.e2e-artifacts/${number}"
    mkdir -p "$artifact_dir"
    gh run download "$run_id" --repo "$GH_REPO" --dir "$artifact_dir" 2>/dev/null || true
    audio_line=""
    if [ -n "$(ls -A "$artifact_dir" 2>/dev/null)" ]; then
        audio_files="$(find "$artifact_dir" -maxdepth 3 -type f | sed "s|^${artifact_dir}/||" | head -n5 | sed 's/^/  - /')"
        audio_line="### Audio artifacts
${audio_files}"
    fi

    # --- acceptance evidence (issue #1077 + e2e-контракт) ---
    # Воркер указал acceptance_check в блоке ## e2e → в e2e-доклад обязаны
    # попасть ДОКАЗАТЕЛЬСТВА работы фичи (не только verdict). Ищем артефакт
    # e2e-acceptance-<run_id>/e2e_acceptance_<run_id>.txt, скачанный выше.
    acceptance_line=""
    acc_file="$(find "$artifact_dir" -maxdepth 3 -type f -name 'e2e_acceptance_*.txt' 2>/dev/null | head -n1)"
    if [ -n "$acc_file" ] && [ -s "$acc_file" ]; then
        acceptance_line="### Acceptance ($e2e_acceptance_check)
$(cat "$acc_file")"
    fi

    # --- comment to issue ---
    # ретро 10.08 (t_9caf5d52): FAIL НЕ всегда = e2e:rejected.
    #   1. PR фикса уже merged → НЕ ставим e2e:rejected (ставим e2e-done +
    #      коммент с причиной) — stale-метка на смерженном фиксе навсегда
    #      выкидывает issue из ротации (#1089, PR #1090).
    #   2. infra-FAIL (квота 429, робот недоступен, build fail) → коммент без
    #      e2e:rejected (метка e2e:infra-fail), needs-e2e НЕ снимаем → issue
    #      остаётся в ротации, следующий тик повторит прогон (#1077: acceptance
    #      прошёл, а FAIL был только из-за квоты MiniMax).
    #   3. feature-FAIL (баг в коде) → e2e:rejected как раньше.
    fail_kind="feature"
    if [ "$verdict" != "success" ]; then
        # PR уже смержен? → merged (stale-метку не вешаем).
        if [ -n "$pr_number" ]; then
            pr_state_now="$(gh pr view "$pr_number" --repo "$GH_REPO" --json state --jq '.state' 2>/dev/null || echo '')"
            if [ "$pr_state_now" = "MERGED" ]; then
                fail_kind="merged"
            fi
        fi
        if [ "$fail_kind" = "feature" ]; then
            fail_kind="$(detect_fail_kind "$artifact_dir" "$run_id")"
        fi
    fi

    if [ "$verdict" = "success" ]; then
        label_action="add ${DONE_LABEL}"
        remove_action="remove ${NEEDS_E2E_LABEL}"
        verdict_emoji="✅"
    elif [ "$fail_kind" = "merged" ]; then
        # Фикс смержен — e2e-done вместо e2e:rejected (stale-метка не нужна).
        label_action="add ${DONE_LABEL}"
        remove_action="remove ${NEEDS_E2E_LABEL}"
        verdict_emoji="ℹ️"
    elif [ "$fail_kind" = "infra" ]; then
        # Инфра-сбой — issue остаётся в ротации (needs-e2e НЕ снимаем).
        label_action="add ${INFRA_FAIL_LABEL}"
        remove_action=""   # не снимаем needs-e2e
        verdict_emoji="⚠️"
    else
        label_action="add ${REJECTED_LABEL}"
        remove_action="remove ${NEEDS_E2E_LABEL}"
        verdict_emoji="❌"
    fi

    # Причина для комментария (merged/infra — отдельная строка в докладе).
    fail_note=""
    if [ "$fail_kind" = "merged" ]; then
        fail_note="> ⚠️ PR #${pr_number} уже смержен в ${DEVELOP_BRANCH} — метка \`e2e:rejected\` НЕ ставилась. Поставлена \`${DONE_LABEL}\` (фикс доехал до develop). Причина FAIL прогона: ${verdict} (см. run)."
    elif [ "$fail_kind" = "infra" ]; then
        fail_note="> ⚠️ FAIL по ИНФРАСТРУКТУРЕ (квота MiniMax 429 / робот недоступен / build fail) — метка \`e2e:rejected\` НЕ ставилась, issue остаётся в ротации (\`${NEEDS_E2E_LABEL}\` сохранён), следующий тик повторит прогон. Поставлена \`${INFRA_FAIL_LABEL}\`."
    fi

    # Динамический шаг (ретро 10.08 t_9caf5d52): merged/infra ≠ «чини код».
    if [ "$verdict" = "success" ]; then
        manual_step="If PASS: \\`gh pr merge --squash ${branch} -> develop\\` (manual merge per Q5)."
    elif [ "$fail_kind" = "merged" ]; then
        manual_step="PR #${pr_number} уже смержен — фикс в develop, e2e-прогон не нужен. Issue можно закрыть (или ждёт другого релиза)."
    elif [ "$fail_kind" = "infra" ]; then
        manual_step="Infra-FAIL (квота 429 / робот недоступен / build): жди следующего тика e2e-process — issue остаётся в ротации, прогон повторится автоматически."
    else
        manual_step="If FAIL: fix on the SAME branch (no new branches/PRs — Шифу прямо: «не плодить ветки»), push the fix, re-trigger e2e. The worker PR stays the same; only code in it changes."
    fi

    comment_body="$(cat <<EOF
## 📊 e2e-доклад #${number} — ${verdict^^}

### Verdict
${verdict_emoji} ${verdict^^}

${fail_note}

### Run
[run #${run_id}](https://github.com/${GH_REPO}/actions/runs/${run_id}) on \`${ROUND_BRANCH}\`

### Sources
- Agent PR: \`${branch}\` (issue #${number})
- Round: \`${ROUND_BRANCH}\`

${audio_line}

${acceptance_line}

### Accept
- A11 wake+prefix: accepted (agent PR CI green)
- e2e verdict: ${verdict^^}

### Manual next step
${manual_step}


---
_Generated by agent-flow-e2e-process (Phase 3)._
EOF
)"


    # ретро 10.08 (t_9caf5d52): при продолжительном infra-сбое (квота на часы)
    # каждый часовой тик НЕ должен плодить одинаковый доклад — если за последние
    # 6 часов уже есть доклад с тем же маркером fail_kind, пропускаем комментарий
    # (метки всё равно обновляются ниже).
    _skip_comment=0
    if [ "$fail_kind" = "infra" ]; then
        _since6="$(date -u -d '6 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
        _infra_dup="$(gh api "repos/${GH_REPO}/issues/${number}/comments?since=${_since6}&per_page=100" \
            --jq '[.[] | select(.body | contains("FAIL по ИНФРАСТРУКТУРЕ"))] | length' 2>/dev/null || echo 0)"
        if [ "${_infra_dup:-0}" -gt 0 ] 2>/dev/null; then
            _skip_comment=1
            log "issue #${number}: infra-fail comment already posted (×${_infra_dup} in 6h) — skip comment, labels only"
        fi
    fi
    if [ "$_skip_comment" -eq 0 ]; then
        if ! gh issue comment "$number" --repo "$GH_REPO" --body "$comment_body" >/dev/null 2>&1; then
            log "issue #${number}: WARNING comment write failed"
        fi
    fi

    # Процесс-фикс (10.08, ретро t_d0151eb3): дублировать e2e-доклад В PR тоже,
    # иначе товарищ Шифу смотрит PR — комментов нет → "где e2e-доказательства?". Для
    # функциональных PR (default) публикуем полный доклад. Для lint/refactor-PR
    # (issue с меткой no-e2e-required или PR с префиксом [lint]/[refactor]) —
    # короткий комментарий "только формат, e2e не нужен".
    if [ -n "$pr_number" ]; then
        # Определяем kind PR по issue-метке no-e2e-required (воркер ставит сам)
        # или по префиксу PR title ([lint]/[refactor]).
        _pr_kind="feature"
        if [ -n "$number" ] && gh issue view "$number" --repo "$GH_REPO" --json labels --jq '[.labels[].name] | index("no-e2e-required")' 2>/dev/null | grep -q -v '^null$'; then
            _pr_kind="lint"
        fi
        _pr_title="$(gh pr view "$pr_number" --repo "$GH_REPO" --json title --jq '.title' 2>/dev/null || echo '')"
        case "$_pr_title" in
            \[lint\]*|\[refactor\]*) _pr_kind="lint" ;;
        esac

        if [ "$_pr_kind" = "lint" ]; then
            _pr_comment_body="## ℹ️ e2e не требуется (lint/refactor PR)

PR помечен как lint/refactor — логика не менялась, достаточно CI green.
- Agent PR: \`${branch}\` (issue #${number})
- Round: \`${ROUND_BRANCH}\`

### Verify
CI: https://github.com/${GH_REPO}/actions (последний run на ветке \`${branch}\`)

### Accept
- A11 wake+prefix: accepted (agent PR CI green)
- e2e verdict: N/A (lint PR)

---
_Generated by agent-flow-e2e-process._"
        else
            # Тот же comment_body что и в issue, но с явной ссылкой на run и
            # командой просмотра лога робота.
            _pr_comment_body="${comment_body}

### Manual verification
\`\`\`bash
sshpass -p open ssh ros2@10.1.1.21 'docker logs voice-assistant --since <ts> | grep -E \"ПРИНЯТО|LLM INPUT|TTS:|deepseek\"'
\`\`\`"
        fi

        if ! gh pr comment "$pr_number" --repo "$GH_REPO" --body "$_pr_comment_body" >/dev/null 2>&1; then
            log "issue #${number}: WARNING PR comment write failed (pr #${pr_number})"
        fi

        # Для lint-PR — ставим needs-review сразу (товарищ Шифу ревьюит CI green)
        # Для feature-PR — НЕ ставим автоматически (воркер должен сам добавить
        # needs-review ПОСЛЕ ручной верификации по логам робота).
        if [ "$_pr_kind" = "lint" ] && [ "$verdict" = "success" ]; then
            gh pr edit "$pr_number" --repo "$GH_REPO" --add-label "needs-review" >/dev/null 2>&1 || true
        fi
    fi

    gh issue edit "$number" --repo "$GH_REPO" --add-label "${label_action#add }" >/dev/null 2>&1 || true
    # ретро 10.08 (t_9caf5d52): при infra-FAIL needs-e2e НЕ снимаем — issue
    # остаётся в ротации, следующий тик повторит прогон.
    if [ -n "$remove_action" ]; then
        gh issue edit "$number" --repo "$GH_REPO" --remove-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
    fi

    # --- GATE для воркера (ретро t_d0151eb3 + правило Шифу 10.08) ------------
    # После КАЖДОГО прогона (хоть красный, хоть зелёный) воркер ОБЯЗАН посмотреть
    # логи и рапортовать. Процесс создаёт карточку в канбане assignee=профиль
    # воркера (по метке issue agent:backend → backend; agent:developer → developer)
    # с конкретным ТЗ. Без выполнения этой карточки next-round НЕ идёт.
    #
    # Правило (Шифу, 10.08):
    #  - Красный по вине → чинит в той же ветке, тот же PR, push, re-trigger.
    #  - Красный НЕ по вине → сидит, ждёт следующего прогона.
    #  - Зелёный НЕ значит ок → иди в run, проверь voice_e2e_*.log, ищи
    #    доказательства работы фичи. Нашёл → worker-evidence: в PR. Нет → чини.
    #  - Никаких новых веток/PRов — тот же PR перепрогоняется пока e2e не
    #    покажет реальные доказательства работы фичи.
    if [ -n "$e2e_task_id" ] && [ -n "$branch" ] && [ "$fail_kind" != "infra" ] && [ "$fail_kind" != "merged" ]; then
        # ретро 10.08 (t_9caf5d52): при infra-FAIL / merged-PR карточку воркеру
        # НЕ создаём — воркеру нечего чинить (квота/робот/build или фикс уже в develop).
        # Определяем профиль воркера по меткам issue (agent:<role>)
        _worker_assignee="default"
        for lbl in $(gh issue view "$number" --repo "$GH_REPO" --json labels --jq '[.labels[].name] | .[]' 2>/dev/null); do
            case "$lbl" in
                agent:backend)    _worker_assignee="backend"; break ;;
                agent:developer)  _worker_assignee="developer"; break ;;
                agent:devops)     _worker_assignee="devops"; break ;;
                agent:architect)  _worker_assignee="architect"; break ;;
            esac
        done

        if [ "$verdict" = "success" ]; then
            _gate_body="## 🟢 e2e ЗЕЛЁНЫЙ — но это не значит «ок» (ретро t_d0151eb3)

**ОБЯЗАН** (по процессу Шифу 10.08): сходи в run [#${run_id}](https://github.com/${GH_REPO}/actions/runs/${run_id}) → открой **download artifacts → voice_e2e_${run_id}.log** и проверь что твоя фича реально отработала:
1. Команда распознана (ПРИНЯТО в stt_node логах)?
2. LLM ответил с тем что делает фича?
3. TTS озвучил, робот не «Empty assistant response»?

**ЕСЛИ ДОКАЗАТЕЛЬСТВА ЕСТЬ** → рапортуй в PR #${pr_number:-?} первой строкой \`worker-evidence: <кратко>\`, тело: кусок лога 5-15 строк с таймстампами + ссылка на run. После этого процесс поставит needs-review и карточка закроется.

**ЕСЛИ ДОКАЗАТЕЛЬСТВ НЕТ** в логе прогона → иди на 10.1.1.21 (\`sshpass -p open ssh ros2@10.1.1.21 'docker logs voice-assistant --since <ts>'\`), проиграй команду сам, добывай raw-лог. Чини **в той же ветке** \`${branch}\` (тот же PR, никаких новых веток — Шифу прямо: «не плодить ветки»), push, жди следующего прогона. Карточка остаётся до прогона с доказательствами.

**ЗАПРЕЩЕНО:** ставить needs-review самостоятельно, мержить, плодить ветки/PRы."
            _gate_title="🟢 e2e-ran #${number}: проверь worker-evidence для PR \`${branch}\`"
            _gate_priority=70
        else
            _gate_body="## 🔴 e2e КРАСНЫЙ — посмотри, определи свою вину (ретро t_d0151eb3)

**ОБЯЗАН** (по процессу Шифу 10.08): сходи в run [#${run_id}](https://github.com/${GH_REPO}/actions/runs/${run_id}) → **download artifacts → voice_e2e_${run_id}.log** и определи причину FAIL.

**ЕСЛИ ПО ТВОЕЙ ВИНЕ** (баг в твоём коде, exception в логах, неправильная команда в ## e2e блоке) → чини **в той же ветке** \`${branch}\` (никаких новых веток/PRов — Шифу прямо), push, жди следующего прогона. Карточка остаётся.

**ЕСЛИ НЕ ПО ТВОЕЙ ВИНЕ** (квота MiniMax 2056, сеть, race, бот робота недоступен) → сиди, жди следующего прогона. Карточка остаётся.

**ЗАПРЕЩЕНО:** плодить ветки/PRы, ставить needs-review, мержить."
            _gate_title="🔴 e2e-fail #${number}: проверь лог и определи вину (PR \`${branch}\`)"
            _gate_priority=80
        fi

        # Создаём карточку assignee=воркер с этим ТЗ (если уже есть от прошлого
        # прогона — не плодим, обновляем существующую по title prefix).
        _existing_gate="$(hermes kanban --board "$KANBAN_BOARD" list --json 2>/dev/null | python3 -c "
import json,sys
data = json.loads(sys.stdin.read())
for t in data:
    if t.get('status') in ('ready','running','blocked','todo') and t.get('title','').startswith(('🟢 e2e-ran #${number}:','🔴 e2e-fail #${number}:')):
        print(t['id'])
        break
" 2>/dev/null | head -1)"

        if [ -n "$_existing_gate" ]; then
            hermes kanban --board "$KANBAN_BOARD" comment "$_existing_gate" "$(echo "$_gate_body" | sed 's/^/  /' | sed '0,/${run_id}/{s/${run_id}/'"$run_id"'/}')" >/dev/null 2>&1 || true
            log "issue #${number}: gate card ${_existing_gate} already exists — appended progress"
        else
            # Создаём карточку (goal_mode=0, без --skill, чтобы воркер сам закрыл)
            hermes kanban --board "$KANBAN_BOARD" create \
                --assignee "$_worker_assignee" \
                --priority "$_gate_priority" \
                --max-runtime 1800 \
                --body "$_gate_body" \
                "$_gate_title" >/dev/null 2>&1 || log "issue #${number}: WARNING gate card create failed (${_worker_assignee})"
            log "issue #${number}: gate card created for ${_worker_assignee} (${verdict^^})"
        fi
    fi

    # ретро 10.08 #4: PR с needs-e2e зависал после успешного E2E — снимаем метку с PR
    # (раньше снимали только с issue, но merge-gate ставит needs-e2e на PR, и
    # после E2E-PR оставался в висящем состоянии; cron-ночник видел "no issues
    # with needs-e2e" → ничего не делал → PR навсегда). Снимаем с PR если есть.
    #
    # ретро t_d0151eb3 (дополнение-2): e2e-доклад публикуем И в issue, И в PR
    # (в PR — обязательно, товарищ Шифу смотрит PR а не issue; в issue — для истории).
    # Плюс автоматически ставим needs-review на PR (раньше падаван ставил руками).
    if [ -n "$pr_number" ]; then
        # ретро 10.08 (t_9caf5d52): при infra-FAIL needs-e2e с PR тоже НЕ снимаем
        # (issue остаётся в ротации — следующий тик повторит прогон).
        if [ -n "$remove_action" ]; then
            gh pr edit "$pr_number" --repo "$GH_REPO" --remove-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
        fi
        if [ "$verdict" = "success" ]; then
            gh pr edit "$pr_number" --repo "$GH_REPO" --add-label "$DONE_LABEL" >/dev/null 2>&1 || true
            # needs-review — окно ревью товарища Шифу (auto, не ручной падаван).
            gh pr edit "$pr_number" --repo "$GH_REPO" --add-label "$NEEDS_REVIEW_LABEL" >/dev/null 2>&1 || true
        fi

        # Worker-evidence gate: если воркер сам опубликовал доказательства с
        # маркером `worker-evidence:` за последний час — публикуем только
        # короткий пост-линк. Иначе публикуем полный e2e-доклад в PR.
        since_iso="$(date -u -d '1 hour ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null \
            || date -u +%Y-%m-%dT%H:%M:%SZ)"
        worker_evidence="no"
        if [ "$verdict" = "success" ]; then
            worker_evidence="$(worker_evidence_recent "$pr_number" "$since_iso")"
        fi

        if [ "$worker_evidence" = "yes" ]; then
            pr_comment_body="$(cat <<EOF
agent-flow: ✅ e2e ${verdict^^} на [run #${run_id}](https://github.com/${GH_REPO}/actions/runs/${run_id}) (round \`${ROUND_BRANCH}\`).
Воркер уже опубликовал \`worker-evidence:\` — см. выше. Поставлено: \`${DONE_LABEL}\`, \`${NEEDS_REVIEW_LABEL}\`.
EOF
)"
            log "issue #${number}: worker-evidence recent — posting short PR-comment only"
        else
            # Тот же $comment_body, что в issue, для PR — чтобы товарищ Шифу видел всё.
            pr_comment_body="$comment_body"
            log "issue #${number}: no worker-evidence recent — posting full e2e-доклад в PR #${pr_number}"
        fi

        if ! gh pr comment "$pr_number" --repo "$GH_REPO" --body "$pr_comment_body" >/dev/null 2>&1; then
            log "issue #${number}: WARNING PR-comment write failed for PR #${pr_number}"
        fi

        # Команда просмотра логов робота (если задан E2E_ROBOT_HOST).
        # Юзер видит её прямо в PR-комменте — copy-paste.
        if [ -n "${E2E_ROBOT_HOST:-}" ] && [ "$verdict" = "success" ]; then
            log_cmd="sshpass -p open ssh ${E2E_ROBOT_USER:-ros2}@${E2E_ROBOT_HOST} \\\"docker logs voice-assistant --since 30m | grep -E 'ERROR|died' || true\\\""
            gh pr comment "$pr_number" --repo "$GH_REPO" --body \
                "agent-flow: ℹ️ проверить живой лог робота (после merge в develop):
\`\`\`bash
${log_cmd}
\`\`\`" >/dev/null 2>&1 || true
        fi
    fi

    # Q22: после e2e разблокируем карточку с результатами — карточка сама решает:
    # PASS → ждёт ручного merge юзера → done; FAIL → воркер берёт снова (итерация).
    if [ -n "$e2e_task_id" ]; then
        # Процесс-фикс (09.08): старые done/archived карточки держат ту же ветку
        # (worktree .worktrees/t_<old>) → респавн падает «git worktree add failed»
        # и карточка навсегда виснет в blocked. Освобождаем ветку ДО unblock.
        free_stale_worktrees_for "$e2e_task_id" || true
        if "$HERMES_BIN" kanban --board "$KANBAN_BOARD" unblock "$e2e_task_id" >/dev/null 2>&1; then
            log "issue #${number}: card ${e2e_task_id} unblocked with e2e results (${verdict^^}) — card decides next step"
        else
            log "issue #${number}: WARNING unblock failed for ${e2e_task_id}"
        fi
    fi

    log "issue #${number}: ${verdict_emoji} ${verdict^^} — labels updated"
    processed=$((processed+1))
done < <(printf '%s' "$issues_json" | python3 -c '
import json, sys
data = json.load(sys.stdin)
# sort by issue number for predictable order
for issue in sorted(data, key=lambda i: i["number"]):
    n = issue["number"]
    t = issue["title"]
    l = ",".join(sorted({lab["name"] for lab in issue.get("labels", [])}))
    b = issue.get("body") or ""
    def esc(s):
        return s.replace("\\", "\\\\").replace("\t", "\\t").replace("\n", "\\n")
    sys.stdout.write(f"{n}\t{esc(t)}\t{esc(l)}\t{esc(b)}\n")
')

# --- summary -----------------------------------------------------------------
log "tick done: processed=${processed} skipped=${skipped} errored=${errored} round=${ROUND_BRANCH}"

if [ "$errored" -gt 0 ]; then exit 1; fi
exit 0
