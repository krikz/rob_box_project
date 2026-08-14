#!/bin/bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-triage.sh
# Каноническая версия живёт в репо. На хост раскладывается через
# `bash <repo>/scripts/agent_flow/install.sh`, который создаёт
# символические ссылки в:
#   - ~/.hermes/profiles/agent-flow/scripts/agent-flow-triage.sh
#   - ~/.hermes/profiles/architect/scripts/agent-flow-triage.sh
#   - ~/.hermes/scripts/agent-flow-triage.sh
# Правка: редактируем <repo>/scripts/agent_flow/agent-flow-triage.sh, commit, merge.
# На хост: bash <repo>/scripts/agent_flow/install.sh (или вручную cp + ln -sf).
# Если ты правишь этот файл НА ХОСТЕ руками — синхронизируй обратно в репо.
# ============================================================================
# agent-flow-triage.sh — Phase 1 agent-flow: GitHub Issues -> Hermes Kanban cards.
#
# Pure bash. No LLM. Idempotent. Driven entirely by env (see ~/.hermes/profiles/agent-flow/.env).
#
# Pipeline per tick:
#   1. MAINTENANCE gate (remote + local)  -> exit 0 if paused
#   2. gh auth check                       -> exit 1 if not authed
#   3. List open issues with label $ISSUE_LABEL on $GH_REPO
#   4. For each issue:
#        a. Skip if it already has $DONE_LABEL (e2e-done — work complete)
#        b. Skip if comment marker `kanban: t_<id>` already present (idempotency)
#        c. Skip if a card with `issue: #N` in body already exists (idempotency v2)
#        d. Skip if the would-be branch already has a MERGED PR (work in develop)
#        e. Resolve role from `agent:<role>` label, else $AGENT_FLOW_DEFAULT_ROLE
#        f. Compute branch name (agent/<issue>-<slug> or ~<slug> for service/infra)
#        g. `hermes kanban create` with --workspace worktree --branch $branch
#        h. Comment the new t_<id> into the issue (3x retry, exp-backoff)
#   5. flock lock prevents parallel ticks.
#
# Gates G2..G7 follow the table in ~/.hermes/profiles/agent-flow/skills/.../SKILL.md.

set -euo pipefail

# --- defaults (overridden by env / .env) -------------------------------------
# NOTE: hardcode /home/builder/.hermes — this script is owned by the host
# hermes install, not by the calling profile's $HOME (cron may spawn us from
# the agent-flow profile where $HOME is profile-relative).
HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
HERMES_BIN="${HERMES_BIN:-/home/builder/.hermes/hermes-agent/venv/bin/hermes}"

# Force HOME=/home/builder so that gh CLI and hermes binaries (which look in
# $HOME/.config/gh and $HOME/.hermes respectively) resolve to the real user
# install, not the per-profile $HOME that cron sets via build_subprocess_env.
export HOME=/home/builder
ISSUE_LABEL="${ISSUE_LABEL:-hermes}"
DONE_LABEL="${DONE_LABEL:-e2e-done}"
KANBAN_BOARD="${KANBAN_BOARD:-robbox}"
MAINTENANCE_BRANCH="${MAINTENANCE_BRANCH:-develop}"
MAINTENANCE_FILE="${MAINTENANCE_FILE:-MAINTENANCE}"
AGENT_FLOW_DEFAULT_ROLE="${AGENT_FLOW_DEFAULT_ROLE:-architect}"
AGENT_FLOW_MAX_RUNTIME="${AGENT_FLOW_MAX_RUNTIME:-1800}"
# Ретро-фикс (09.08 #2): крупные задачи (priority:P0 или объёмный body) получают
# увеличенный --max-runtime — иначе воркер упирается в бюджет на полпути.
AGENT_FLOW_MAX_RUNTIME_LARGE="${AGENT_FLOW_MAX_RUNTIME_LARGE:-3600}"
# Порог "объёмного" body issue (символов) — грубый прокси размера задачи.
AGENT_FLOW_LARGE_BODY_CHARS="${AGENT_FLOW_LARGE_BODY_CHARS:-2000}"
AGENT_FLOW_MAX_RETRIES="${AGENT_FLOW_MAX_RETRIES:-2}"
# ADR-0013 (docs/adr/0013-incremental-delivery-over-big-bang.md): PR > 50
# коммитов ИЛИ > 3000 строк запрещён без метки `big-bang-override` на issue.
# Triage проверяет это ДО `hermes kanban create` — если к issue уже привязан
# огромный PR (например, воркер случайно запушил 100 коммитов до того как
# merge-gate успел среагировать), мы НЕ создаём новую карточку на тот же issue
# (worker всё равно упрётся в merge-gate → бессмысленная работа). Шифу
# ставит метку вручную, чтобы явно разрешить.
BIG_BANG_OVERRIDE_LABEL="${BIG_BANG_OVERRIDE_LABEL:-big-bang-override}"
BIG_BANG_MAX_COMMITS="${BIG_BANG_MAX_COMMITS:-50}"
BIG_BANG_MAX_LINES="${BIG_BANG_MAX_LINES:-3000}"
DRY_RUN="${DRY_RUN:-false}"
ISSUE_LIMIT="${ISSUE_LIMIT:-50}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-triage.lock}"
LOG_PREFIX="${LOG_PREFIX:-[agent-flow-triage]}"

# --- source profile .env if present -----------------------------------------
# Precedence: caller env > .env > defaults. We do NOT use `set -a` because
# that would clobber caller overrides (matters for tests / cron flags).
PROFILE_ENV="${HERMES_HOME}/profiles/agent-flow/.env"
if [ -f "$PROFILE_ENV" ]; then
    while IFS='=' read -r key val; do
        # skip comments / blanks
        case "$key" in ''|\#*) continue ;; esac
        # strip surrounding quotes from .env value
        val="${val%\"}"; val="${val#\"}"
        val="${val%\'}"; val="${val#\'}"
        # only set if not already in caller env (treat empty as unset)
        if [ -z "${!key:-}" ]; then
            export "$key=$val"
        fi
    done < "$PROFILE_ENV"
fi

# Re-apply defaults for any vars still empty (defensive — .env may be partial).
: "${KANBAN_BOARD:=robbox}"
: "${MAINTENANCE_BRANCH:=develop}"
: "${MAINTENANCE_FILE:=MAINTENANCE}"
: "${REPO_DIR:=}"
: "${AGENT_FLOW_DEFAULT_ROLE:=architect}"
: "${AGENT_FLOW_MAX_RUNTIME:=1800}"
: "${AGENT_FLOW_MAX_RUNTIME_LARGE:=3600}"
: "${AGENT_FLOW_LARGE_BODY_CHARS:=2000}"
: "${AGENT_FLOW_MAX_RETRIES:=2}"
: "${BIG_BANG_OVERRIDE_LABEL:=big-bang-override}"
: "${BIG_BANG_MAX_COMMITS:=50}"
: "${BIG_BANG_MAX_LINES:=3000}"
: "${DRY_RUN:=false}"
: "${ISSUE_LABEL:=hermes}"
: "${DONE_LABEL:=e2e-done}"
: "${ISSUE_LIMIT:=50}"

# --- helpers -----------------------------------------------------------------
log()  { printf '%s %s %s\n' "$LOG_PREFIX" "$(date -Iseconds)" "$*" >&2; }
run()  { if [ "$DRY_RUN" = "true" ]; then printf '%s DRY-RUN %s\n' "$LOG_PREFIX" "$*"; else eval "$@"; fi; }

# flock: skip tick if another instance holds the lock.
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

# --- G2: gh auth check -------------------------------------------------------
if ! gh auth status >/dev/null 2>&1; then
    log "gh auth not configured — exit 1"; exit 1
fi

# --- required env ------------------------------------------------------------
: "${GH_REPO:?GH_REPO must be set (owner/repo)}"

# --- pull open issues -------------------------------------------------------
issues_json="$(gh issue list \
    --repo "$GH_REPO" \
    --label "$ISSUE_LABEL" \
    --state open \
    --limit "$ISSUE_LIMIT" \
    --json number,title,labels,body 2>/dev/null || true)"

# G3: empty output could mean rate-limit. Detect via direct API status if possible.
if [ -z "$issues_json" ] || [ "$issues_json" = "[]" ]; then
    rate="$(gh api rate_limit --jq '.resources.core.remaining' 2>/dev/null || echo 999)"
    if [ "${rate:-999}" = "0" ]; then
        log "GitHub rate-limit exhausted — skip tick"; exit 0
    fi
    log "no issues with label '${ISSUE_LABEL}' on ${GH_REPO}"; exit 0
fi

# --- branch-naming helpers --------------------------------------------------
slugify() {
    # lowercase, replace non-alnum with -, collapse, trim, kebab-case, cap 40
    printf '%s' "$1" \
        | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+|-+$//g; s/-{2,}/-/g' \
        | cut -c1-40
}

branch_for() {  # $1=labels_json  $2=issue_number  $3=title
    labels_norm="$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')"
    if printf '%s' "$labels_norm" | grep -Eq 'service:|infra:|ops:'; then
        printf 'z-{%s}' "$(slugify "$3")"
    else
        printf 'z-{agent}/%s-%s' "$2" "$(slugify "$3")"
    fi
}

role_for() {  # $1=labels_json
    printf '%s' "$1" \
        | grep -oE 'agent:[a-z0-9_-]+' \
        | head -n1 \
        | sed 's/^agent://' \
        || printf '%s' "$AGENT_FLOW_DEFAULT_ROLE"
}

# Ретро-фикс (09.08 #2): крупные задачи получают увеличенный --max-runtime.
# Крупная = label `priority:P0` ИЛИ объёмный body (>= AGENT_FLOW_LARGE_BODY_CHARS).
runtime_for() {  # $1=labels_csv  $2=body
    local labels_lower body_len
    labels_lower="$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')"
    body_len="${#2}"
    if printf '%s' "$labels_lower" | grep -Eq '(^|,)priority:p0(,|$)'; then
        printf '%s' "$AGENT_FLOW_MAX_RUNTIME_LARGE"; return
    fi
    if [ "$body_len" -ge "$AGENT_FLOW_LARGE_BODY_CHARS" ]; then
        printf '%s' "$AGENT_FLOW_MAX_RUNTIME_LARGE"; return
    fi
    printf '%s' "$AGENT_FLOW_MAX_RUNTIME"
}

# Ретро-фикс (09.08 #2): контракт воркера в каждой карточке — воркер коммитит
# WIP каждые ~15-20 мин (или при половине max_runtime), чтобы работа не
# пропадала при исчерпании бюджета (см. t_9435a3c5, t_0c0a98ac).
worker_contract_block() {  # $1=max_runtime
    cat <<EOF
## Контракт воркера (agent-flow)
- **Коммить WIP каждые ~15-20 мин** (или при достижении половины max_runtime=$1) — \`wip(scope): ...\` коммиты в свою ветку, push сразу. Незакоммиченная работа пропадает при исчерпании бюджета.
- PR один на всю задачу (WIP-коммиты идут в ту же ветку; merge-gate/e2e работают по ветке/PR, не по числу коммитов).

EOF
}

# --- process each issue ------------------------------------------------------
# Use python3 to safely parse JSON (jq may not be installed everywhere).
if command -v jq >/dev/null 2>&1; then
    parse() { jq -r "$@"; }
else
    parse() { python3 -c "import json,sys; d=json.load(sys.stdin); print($1)" <<<"${issues_json}"; }
fi

# Ретро-фикс (09.08 #14): дубликаты карточек от triage при переключении меток.
# Маркер `kanban: t_` в комментариях мог потеряться (коммент не записался /
# удалён) → triage создавал вторую карточку на тот же issue. Дополнительная
# идемпотентность: собрать мапу issue -> task_id из СУЩЕСТВУЮЩИХ карточек
# (по "issue: #N" в body) и не создавать дубль, если карточка уже есть.
existing_by_issue="$(printf '%s' "$("$HERMES_BIN" kanban --board "$KANBAN_BOARD" list --json 2>/dev/null || echo '[]')" | python3 -c '
import json, sys, re
try:
    d = json.load(sys.stdin)
    tasks = d if isinstance(d, list) else d.get("tasks", [])
except Exception:
    tasks = []
for t in tasks:
    if t.get("status") == "archived":
        continue
    body = t.get("body") or ""
    m = re.search(r"issue:\s*#(\d+)", body)
    if m:
        print("%s\t%s" % (m.group(1), t.get("id", "")))
')"

# Ретро-фикс (09.08 #1): старые done/archived карточки держат ветку через
# worktree → при спавне новой карточки `git worktree add` падает
# («already checked out») и карточка навсегда виснет в blocked. Собираем
# кандидатов-клонов (из workspace_path всех карточек + REPO_DIR) — главный
# клон, где диспетчер создаёт .worktrees/<task-id>, может отличаться от
# REPO_DIR (cron-клон) и от самой старой карточки.
WORKTREE_CLONES="$(printf '%s' "$("$HERMES_BIN" kanban --board "$KANBAN_BOARD" list --json 2>/dev/null || echo '[]')" | python3 -c '
import json, sys, os
try:
    d = json.load(sys.stdin)
    tasks = d if isinstance(d, list) else d.get("tasks", [])
except Exception:
    tasks = []
seen = set()
for t in tasks:
    wp = t.get("workspace_path") or ""
    if "/.worktrees/" in wp:
        repo = os.path.dirname(os.path.dirname(wp))
        if repo not in seen:
            seen.add(repo)
            print(repo)
')"
if [ -n "${REPO_DIR:-}" ]; then
    WORKTREE_CLONES="$(printf '%s\n%s\n' "$WORKTREE_CLONES" "$REPO_DIR" | awk 'NF && !seen[$0]++')"
fi

# Освободить worktree'ы на ветке $1, принадлежащие done/archived карточкам.
free_stale_worktrees_for_branch() {  # $1=branch
    local branch="$1" clone line wt_path wt_branch owner status
    [ -z "$WORKTREE_CLONES" ] && { log "  no worktree-owner card found — cannot locate main clone; skip"; return 0; }
    while IFS= read -r clone; do
        [ -n "$clone" ] || continue
        [ -d "$clone/.git" ] || continue
        wt_path=""
        while IFS= read -r line; do
            case "$line" in
                worktree\ *) wt_path="${line#worktree }" ;;
                branch\ *)
                    wt_branch="${line#branch refs/heads/}"
                    if [ "$wt_branch" = "$branch" ] && [ -n "$wt_path" ]; then
                        owner="$(basename "$wt_path")"
                        case "$owner" in
                            t_[a-f0-9]*)
                                # Ретро 12.08 t_8af6bf29: `hermes kanban show` падает
                                # после v0.20.0 (sqlite3.ProgrammingError). Читаем статус
                                # из kanban.db, fallback на JSON-режим show (не падает).
                                _db="${KANBAN_DB_PATH:-${HERMES_HOME}/kanban/boards/${KANBAN_BOARD}/kanban.db}"
                                status="$(python3 - "$_db" "$owner" <<'PY' 2>/dev/null || true
import sqlite3, sys
db, tid = sys.argv[1], sys.argv[2]
try:
    conn = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    row = conn.execute("SELECT status FROM tasks WHERE id=?", (tid,)).fetchone()
    conn.close()
    print(row[0] if row else "")
except Exception:
    pass
PY
)"
                                if [ -z "$status" ] && [ ! -f "$_db" ]; then
                                    status="$("$HERMES_BIN" kanban --board "$KANBAN_BOARD" show "$owner" --json 2>/dev/null \
                                        | python3 -c 'import sys,json
try: print(json.load(sys.stdin).get("task",{}).get("status",""))
except Exception: print("")' 2>/dev/null || true)"
                                fi
                                if [ "$status" = "done" ] || [ "$status" = "archived" ]; then
                                    git -C "$clone" worktree remove --force "$wt_path" 2>/dev/null \
                                        && log "  freed stale worktree $wt_path (branch $branch, card $owner, status=$status)"
                                fi
                                ;;
                        esac
                    fi
                    ;;
            esac
        done < <(git -C "$clone" worktree list --porcelain 2>/dev/null)
        git -C "$clone" worktree prune 2>/dev/null || true
    done <<< "$WORKTREE_CLONES"
    return 0
}

created=0
skipped=0
errored=0

while IFS=$'\t' read -r number title labels body; do
    [ -z "$number" ] && continue

    # Ретро-фикс (13.08, #968): переоткрытые issue (closed → reopened) —
    # это доработка, карточку создавать ЗАНОВО. Для них оба idempotency-гарда
    # (мёртвый kanban-маркер от archived-карточки, merged PR guard) — ложные:
    # работа была сделана, но юзер вернул задачу на доработку.
    is_reopened=false
    if _reopen_ts="$(gh api "repos/${GH_REPO}/issues/${number}/events" \
        --jq '[.[] | select(.event=="reopened")] | last | .created_at' 2>/dev/null || true)" \
        && [ -n "$_reopen_ts" ]; then
        is_reopened=true
        log "issue #${number} was REOPENED at ${_reopen_ts} — доработка, создаю свежую карточку"
    fi

    # Ретро-фикс (11.08 t_ce3ca0d9): НЕ создаём карточку для issue, где работа
    # уже завершена — метка $DONE_LABEL (e2e-done) ставится merge-gate после
    # мержа PR + успешного e2e. Раньше триаж плодил дубликаты для таких issue
    # (пример #1104 → t_8ebc85d9), т.к. issue остаётся OPEN после мержа.
    if printf '%s' "$labels" | tr ',' '\n' | tr '[:upper:]' '[:lower:]' | grep -Fxq "$DONE_LABEL"; then
        log "issue #${number} has ${DONE_LABEL} label (work done) — skip"
        skipped=$((skipped+1)); continue
    fi

    # Idempotency: check if a `kanban: t_` marker already exists in comments.
    # Ретро-фикс (11.08 t_ce3ca0d9): раньше использовался `gh issue view
    # --comments` (GraphQL), который может не вернуть старые комментарии при
    # пагинации. Теперь — REST API с --paginate, гарантированно все комментарии.
    if [ "$is_reopened" = "false" ] && \
        gh api "repos/${GH_REPO}/issues/${number}/comments" --paginate \
        --jq '.[].body' 2>/dev/null \
        | grep -Eq '^kanban: t_[a-f0-9]+'; then
        log "issue #${number} already has kanban marker — skip"
        skipped=$((skipped+1)); continue
    fi

    # Idempotency v2 (ретро 09.08 #14): карточка для этого issue уже есть.
    # Ретро-фикс (13.08, #968): для REOPENED issue проверяем статус существующей
    # карточки: если она ЖИВАЯ (running/ready/todo/blocked) — воркер уже работает,
    # дубль НЕ создаём; если мертва (done/archived) — создаём свежую.
    existing_id="$(printf '%s\n' "$existing_by_issue" | awk -F'\t' -v n="$number" '$1==n {print $2; exit}')"
    if [ -n "$existing_id" ]; then
        if [ "$is_reopened" = "false" ]; then
            log "issue #${number} already has card ${existing_id} (issue: #${number} in body) — skip"
            skipped=$((skipped+1)); continue
        fi
        _existing_status="$("$HERMES_BIN" kanban --board "$KANBAN_BOARD" show "$existing_id" --json 2>/dev/null \
            | python3 -c 'import json,sys
try: print(json.load(sys.stdin).get("task",{}).get("status",""))
except Exception: print("")')"
        case "$_existing_status" in
            running|ready|todo|blocked)
                log "issue #${number}: карточка ${existing_id} ЖИВАЯ (status=${_existing_status}) — воркер уже работает, дубль не создаём"
                skipped=$((skipped+1)); continue
                ;;
            done|archived|"")
                log "issue #${number}: старая карточка ${existing_id} мертва (status=${_existing_status:-unknown}) — создаю свежую на доработку"
                ;;
        esac
    fi

    role="$(role_for "$labels")"
    branch="$(branch_for "$labels" "$number" "$title")"
    max_runtime="$(runtime_for "$labels" "$body")"

    # Ретро-фикс (11.08 t_ce3ca0d9): если на ветке, которую мы бы создали для
    # этого issue, уже есть MERGED PR — работа ушла в develop, карточка не нужна.
    # Это второй рубеж после DONE_LABEL (страховка, если e2e-done не успели
    # проставить, а PR уже смержен).
    # Ретро-фикс (13.08, #968): для REOPENED issue этот гард ложный — юзер
    # вернул задачу на доработку, карточку создаём заново.
    if [ "$is_reopened" = "false" ] && \
        merged_pr="$(gh pr list --repo "$GH_REPO" --head "$branch" --state merged \
        --json number --jq '.[0].number' 2>/dev/null || true)" \
        && [ -n "$merged_pr" ]; then
        log "issue #${number}: branch ${branch} already has MERGED PR #${merged_pr} — skip"
        skipped=$((skipped+1)); continue
    fi

    # Ретро-фикс (13.08, #968): REOPENED issue, но на ветке уже есть OPEN PR —
    # работа в PR (ждёт merge-gate/юзера), карточку НЕ создаём. Без этого гарда
    # триаж плодил бесконечный цикл карточек на один issue: прошлая карточка
    # done → «старая мертва → создаю свежую» → воркер делает ту же работу →
    # done → ... (5+ карточек architect за 20 мин, t_6a4d501b → t_e25720e3).
    if open_pr="$(gh pr list --repo "$GH_REPO" --head "$branch" --state open \
        --json number --jq '.[0].number' 2>/dev/null || true)" \
        && [ -n "$open_pr" ]; then
        log "issue #${number}: branch ${branch} already has OPEN PR #${open_pr} — работа в PR, карточку не создаём (reopened-loop guard)"
        skipped=$((skipped+1)); continue
    fi

    # Ретро-фикс (14.08 t_0a765152): REOPENED issue — карточку создаём ЗАНОВО,
    # но branch_name НЕ переиспользуем, если на нём уже влит PR. Триаж брал
    # branch_name из прошлой карточки того же issue (#1217: PR #1220 merged на
    # z-{agent}/1217-e2e-40-deepseek, триаж создал t_7cc96c7d с ТОЙ ЖЕ веткой
    # → merge-gate по exact-match ветки ложно заархивировал ЖИВУЮ карточку,
    # чья работа ещё в OPEN PR #1231). Если на кандидате уже есть MERGED PR —
    # добавляем суффикс -v2/-r2 (цикл: -v3, -v4... пока ветка не свободна).
    if [ "$is_reopened" = "true" ]; then
        _branch_base="$branch"
        _branch_v=2
        while _merged_on_branch="$(gh pr list --repo "$GH_REPO" --head "$branch" \
            --state merged --json number --jq '.[0].number' 2>/dev/null || true)" \
            && [ -n "$_merged_on_branch" ]; do
            branch="${_branch_base}-v${_branch_v}"
            _branch_v=$((_branch_v+1))
        done
        if [ "$branch" != "$_branch_base" ]; then
            log "issue #${number}: REOPENED — ветка ${_branch_base} уже влита через PR #${_merged_on_branch}, беру ${branch} (ретро t_0a765152)"
        fi
    fi

    # Ретро-фикс (13.08, #968 v3): THROTTLE — если по issue в БД уже есть
    # карточка (ЛЮБОЙ статус, включая archived) за последние 4 часа — не
    # создаём новую. Без этого тик каждые 2 мин плодил карточку (13:00,
    # 13:02, 13:05...): воркер падал на spawn (worktree занят живой веткой),
    # карточка уходила в archived, следующий тик видел «нет живой» и создавал
    # снова. OPEN-PR guard не ловит, т.к. PR #1197 уже CLOSED.
    # Ретро-фикс (13.08, #968 v3.1): list БЕЗ --archived НЕ возвращает archived-
    # карточки, поэтому throttle не видел предыдущие карточки цикла (все они
    # уходили в archived) и тик создавал новую каждые ~2-5 мин (13:13, 13:18 —
    # даже после деплоя v3 в 13:07). Добавляем --archived: throttle видит ВСЕ
    # карточки за 4ч, включая archived, и цикл останавливается.
    _recent_cards="$("$HERMES_BIN" kanban --board "$KANBAN_BOARD" list --json --archived 2>/dev/null | python3 -c '
import json, sys, re, time
try:
    d = json.load(sys.stdin)
    tasks = d if isinstance(d, list) else d.get("tasks", [])
except Exception:
    tasks = []
now = time.time()
cutoff = now - 4 * 3600
for t in tasks:
    if (t.get("created_at") or 0) < cutoff:
        continue
    body = t.get("body") or ""
    if re.search(r"issue:\s*#%s" % re.escape("'"$number"'"), body):
        print(t.get("id", ""))
        break
' 2>/dev/null || true)"
    if [ -n "$_recent_cards" ]; then
        log "issue #${number}: свежая карточка ${_recent_cards} за последние 4ч — throttle, не создаём (reopened-loop v3)"
        skipped=$((skipped+1)); continue
    fi

    # Ретро-фикс (09.08 #1): освободить stale worktree'ы на этой ветке от
    # done/archived карточек ДО create — иначе диспетчер при спавне упадёт
    # «git worktree add failed» (ветка уже занята) и карточка навсегда
    # зависнет в blocked.
    if [ "$DRY_RUN" = "true" ]; then
        log "DRY-RUN would free stale worktrees for branch ${branch}"
    else
        free_stale_worktrees_for_branch "$branch" || true
    fi

    # --- big-bang-override check (ADR-0013, ретро t_9726053d) ------------
    # Если к issue уже привязан PR (воркер мог запушить в обход triage —
    # например, нашёл issue в GitHub UI и сделал PR руками), и этот PR
    # огромный (> ${BIG_BANG_MAX_COMMITS} коммитов ИЛИ > ${BIG_BANG_MAX_LINES}
    # строк) — не создаём новую карточку. Причина:
    #   - merge-gate всё равно заблокирует needs-e2e (нет override)
    #   - воркер потратит max_runtime впустую, потом встанет в тупик
    # Ищем PR по вычисленной ветке (`z-{agent}/N-slug` или service-ветка).
    # Если override-метка уже на issue — пропускаем gate (явное разрешение).
    # Если PR нет (нормальный путь) — пропускаем check (нечего проверять).
    if ! printf '%s' "$labels" | tr ',' '\n' | tr '[:upper:]' '[:lower:]' | grep -Fxq "$BIG_BANG_OVERRIDE_LABEL"; then
        _bb_pr_json="$(gh pr list --repo "$GH_REPO" --state all --head "$branch" \
            --json number,additions,commits 2>/dev/null || echo '[]')"
        _bb_pr_count="$(printf '%s' "$_bb_pr_json" | python3 -c '
import json, sys
try:
    d = json.load(sys.stdin)
    print(len(d) if isinstance(d, list) else 0)
except Exception:
    print(0)
' 2>/dev/null || echo 0)"
        if [ "${_bb_pr_count:-0}" -gt 0 ] 2>/dev/null; then
            _bb_pr_info="$(printf '%s' "$_bb_pr_json" | python3 -c '
import json, sys
try:
    d = json.load(sys.stdin)
    if not isinstance(d, list) or not d:
        print("0	0"); sys.exit(0)
    p = d[0]
    ncommits = len(p.get("commits") or [])
    nadd = int(p.get("additions") or 0)
    print(f"{ncommits}	{nadd}")
except Exception:
    print("0	0")
' 2>/dev/null || echo "0	0")"
            _bb_commits="$(printf '%s' "$_bb_pr_info" | cut -f1)"
            _bb_lines="$(printf '%s' "$_bb_pr_info" | cut -f2)"
            _bb_pr_num="$(printf '%s' "$_bb_pr_json" | python3 -c '
import json, sys
try:
    d = json.load(sys.stdin)
    print(d[0].get("number", "") if d else "")
except Exception:
    print("")
' 2>/dev/null || true)"
            _bb_reasons=""
            if [ "${_bb_commits:-0}" -gt "${BIG_BANG_MAX_COMMITS}" ] 2>/dev/null; then
                _bb_reasons="${_bb_reasons}${_bb_commits} коммитов > ${BIG_BANG_MAX_COMMITS}; "
            fi
            if [ "${_bb_lines:-0}" -gt "${BIG_BANG_MAX_LINES}" ] 2>/dev/null; then
                _bb_reasons="${_bb_reasons}${_bb_lines} строк > ${BIG_BANG_MAX_LINES}; "
            fi
            if [ -n "$_bb_reasons" ]; then
                log "issue #${number}: PR #${_bb_pr_num} BIG-BANG (${_bb_reasons% ;}) — блокируем triage, требуется split или ${BIG_BANG_OVERRIDE_LABEL}"
                if [ "$DRY_RUN" != "true" ]; then
                    gh issue comment "$number" --repo "$GH_REPO" --body \
                        "🚨 **PR #${_bb_pr_num} BIG-BANG** — нарушение ADR-0013: ${_bb_reasons% ;}.

Triage **НЕ создал** kanban-карточку для этого issue, чтобы воркер не сжёг max_runtime впустую (merge-gate всё равно заблокирует needs-e2e без override).

Что делать:
1. **split** на инкрементальные PR (по 1 эпику), ИЛИ
2. **товарищ Шифу** ставит \`${BIG_BANG_OVERRIDE_LABEL}\` на этот issue (явное override).

После override повторный тик triage создаст карточку.

Ссылки: ADR-0013, CONTRIBUTING.md §69-71." >/dev/null 2>&1 || true
                fi
                skipped=$((skipped+1)); continue
            fi
        fi
    fi

    body_block="${body:-}"
    labels_block="$(printf '%s' "$labels" | tr ',' '\n' | sort -u | paste -sd, -)"
    # Context block: sources of truth + where the repo lives (generic, from env)
    sources_block=""
    if [ -n "${GH_REPO:-}" ]; then
        sources_block="Context
  repo: ${GH_REPO}
  local_clone: ${REPO_DIR:-<not set>}
  base_branch: ${MAINTENANCE_BRANCH:-develop}
  sources_of_truth: README.md, CONTRIBUTING.md, AGENTS.md, SPEC_CURRENT.md (если есть), docs/adr/
  access: env GH_REPO/REPO_DIR, gh auth (уже авторизован). Токены не хардкодить.

"
    fi
    contract_block="$(worker_contract_block "$max_runtime")"
    # NOTE: $(...) strips trailing newlines, so add explicit "\n\n" separator
    # between the contract block and the issue body.
    full_body=$(printf 'Source\n  repo: %s\n  issue: #%s\n  labels: %s\n\n%s%s\n\n%s' \
        "$GH_REPO" "$number" "$labels_block" "$sources_block" "$contract_block" "$body_block")

    log "creating card: issue=#${number} role=${role} branch=${branch} max_runtime=${max_runtime}"

    if [ "$DRY_RUN" = "true" ]; then
        log "DRY-RUN would run: ${HERMES_BIN} kanban --board ${KANBAN_BOARD} create --assignee ${role} --workspace worktree --branch ${branch} --max-runtime ${max_runtime} --max-retries ${AGENT_FLOW_MAX_RETRIES} --body <...> -- \"<title>\""
        created=$((created+1)); continue
    fi

    # G4: kanban create. Note: --board is a GLOBAL flag (must precede `create`).
    if ! create_out="$(
        "$HERMES_BIN" kanban --board "$KANBAN_BOARD" create \
            --assignee "$role" \
            --workspace worktree \
            --branch "$branch" \
            --max-runtime "$max_runtime" \
            --max-retries "$AGENT_FLOW_MAX_RETRIES" \
            --body "$full_body" \
            --created-by "agent-flow-triage" \
            -- "$title" 2>&1
    )"; then
        log "kanban create FAILED for issue #${number}: ${create_out}"
        gh issue comment "$number" --repo "$GH_REPO" --body "agent-flow-error: kanban create failed
\`\`\`
${create_out}
\`\`\`" >/dev/null 2>&1 || true
        gh issue edit "$number" --repo "$GH_REPO" --add-label "agent-flow-error" >/dev/null 2>&1 || true
        errored=$((errored+1)); continue
    fi

    # Extract t_<id> from `Created t_<id>` or JSON envelope.
    task_id="$(printf '%s' "$create_out" | grep -oE 't_[a-f0-9]+' | head -n1)"
    if [ -z "$task_id" ]; then
        log "could not parse task id from create output: ${create_out}"
        errored=$((errored+1)); continue
    fi

    # G5: comment write — 3x retry, exp-backoff (card already exists).
    comment_body="kanban: ${task_id}
branch: ${branch}
role: ${role}"
    attempt=1
    while [ "$attempt" -le 3 ]; do
        if gh issue comment "$number" --repo "$GH_REPO" --body "$comment_body" >/dev/null 2>&1; then
            break
        fi
        sleep=$((2 ** attempt))
        log "comment attempt ${attempt}/3 failed, retry in ${sleep}s"
        sleep "$sleep"
        attempt=$((attempt+1))
    done
    if [ "$attempt" -gt 3 ]; then
        log "comment write failed after 3 retries — card ${task_id} exists but not marked in issue #${number}"
        errored=$((errored+1)); continue
    fi

    log "ok: issue #${number} -> ${task_id} (branch=${branch}, role=${role})"
    created=$((created+1))
done < <(printf '%s' "$issues_json" | python3 -c '
import json, sys
data = json.load(sys.stdin)
for issue in data:
    n = issue["number"]
    t = issue["title"]
    l = ",".join(sorted({lab["name"] for lab in issue.get("labels", [])}))
    b = issue.get("body") or ""
    # tab-delimited, escape tabs/newlines in fields
    def esc(s):
        return s.replace("\\", "\\\\").replace("\t", "\\t").replace("\n", "\\n")
    sys.stdout.write(f"{n}\t{esc(t)}\t{esc(l)}\t{esc(b)}\n")
')

# --- summary -----------------------------------------------------------------
log "tick done: created=${created} skipped=${skipped} errored=${errored}"

# Exit non-zero only on hard errors (G4/G5) so cron can alert.
if [ "$errored" -gt 0 ]; then exit 1; fi
exit 0
