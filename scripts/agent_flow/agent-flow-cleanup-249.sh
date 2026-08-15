#!/bin/bash
# ============================================================================
# agent-flow-cleanup-249.sh — безопасный cleanup /tmp на build-хосте 10.1.1.249
# (ретро 11.08 t_26a6d362: cleanup убил артефакты активного e2e-прогона round-49)
#
# ПРОБЛЕМА (11.08): ручной cleanup devops (t_0a5d65af) удалял /tmp/e2e_v2_*
# ВО ВРЕМЯ активного e2e-прогона (round-49, run 31544057593) → paplay open():
# No such file → ложный FAIL. Cleanup не проверял, чьи это артефакты.
#
# ПРАВИЛА:
#   1. НЕ удалять файлы моложе CLEANUP_MIN_AGE_MIN (default 30 мин) — активный
#      e2e-прогон пишет свежие /tmp/e2e_v2_* и /tmp/e2e_atomic_out.log.
#   2. Если e2e-process активен (flock /tmp/agent-flow-e2e-process.lock на
#      хосте ротации) — cleanup пропускается целиком.
#   3. Удаляются только: yandex_key_*, build_*.log, dialog_e2e_*.wav,
#      e2e_v2_*, voice_e2e_*.log старше MIN_AGE (root-owned мусор прошлых
#      ранов, который копится на 249 и мешает scp/paplay).
#   4. e2e_voice_test.sh (актуальный харнесс) НЕ удаляется никогда.
#
# Usage:
#   agent-flow-cleanup-249.sh [--min-age 30] [--dry-run]
# Env: BUILD_HOST=10.1.1.249, BUILD_USER=ros2, SSHPASS (или -p open),
#      LOCK_FILE=/tmp/agent-flow-e2e-process.lock (локальный flock e2e-process),
#      GH_REPO (owner/repo; для round/PR-branch cleanup),
#      ROUND_STALE_HOURS (default 24), MERGED_STALE_HOURS (default 2),
#      CLOSED_STALE_HOURS (default 24)
# Cron: devops-профиль every 6h (ретро 13.08 t_04d73108 — регистрация в install.sh)
# ============================================================================
set -euo pipefail

# gh auth на этом хосте: HOME=/home/builder (иначе gh ищет конфиг в
# $HOME/.config/gh профильной оболочки и падает «not logged in»).
export HOME=/home/builder

# source profile .env (GH_REPO и пр.) — как round_ensure.sh
PROFILE_ENV="/home/builder/.hermes/profiles/agent-flow/.env"
if [ -f "$PROFILE_ENV" ]; then
    while IFS='=' read -r key val; do
        case "$key" in ''|'#'*) continue ;; esac
        val="${val%\"}"; val="${val#\"}"
        val="${val%\'}"; val="${val#\'}"
        if [ -z "${!key:-}" ]; then
            export "$key=$val"
        fi
    done < "$PROFILE_ENV"
fi

BUILD_HOST="${BUILD_HOST:-10.1.1.249}"
BUILD_USER="${BUILD_USER:-ros2}"
SSHPASS_VAL="${SSHPASS:-open}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-e2e-process.lock}"
MIN_AGE_MIN="${MIN_AGE_MIN:-30}"
# Ретро 13.08 t_04d73108: cleanup-249 зарегистрирован в cron devops every 6h.
# Порог снижен 48→24ч, чтобы наверстать накопленные round-ветки 61-76/100-103
# (e2e-process пушит коммит в round ПЕРЕД каждым прогоном — ветка без коммитов
# 24ч гарантированно не в активной ротации; плюс flock-guard секции 1).
ROUND_STALE_HOURS="${ROUND_STALE_HOURS:-24}"
# Ретро 14.08 t_3cfb3b5b: PR-branch sweep (секция 5). merged PR старше
# MERGED_STALE_HOURS → delete ветку; closed PR старше CLOSED_STALE_HOURS → delete.
MERGED_STALE_HOURS="${MERGED_STALE_HOURS:-2}"
CLOSED_STALE_HOURS="${CLOSED_STALE_HOURS:-24}"
DRY_RUN=0
LOG_PREFIX="[cleanup-249]"

log() { printf '%s %s %s\n' "$LOG_PREFIX" "$(date -Iseconds)" "$*" >&2; }

while [ $# -gt 0 ]; do
    case "$1" in
        --min-age) MIN_AGE_MIN="$2"; shift 2 ;;
        --dry-run) DRY_RUN=1; shift ;;
        *) log "unknown arg: $1"; exit 2 ;;
    esac
done

# --- 1. e2e-process активен? (локальный flock на хосте ротации) -------------
if [ -f "$LOCK_FILE" ]; then
    if exec 9>"$LOCK_FILE" && flock -n 9 2>/dev/null; then
        : # lock свободен — можно чистить
    else
        log "🛑 e2e-process активен (flock $LOCK_FILE занят) — cleanup SKIP (ретро 11.08: не трогаем артефакты активного round)"
        exit 0
    fi
else
    log "lock $LOCK_FILE не найден — считаем ротацию неактивной (cleanup можно)"
fi

# --- 2. Проверка: свежий e2e-прогон на 249? (mtime guard на самом хосте) ----
# Даже если локальный flock свободен, e2e-прогон МОЖЕТ идти с другого хоста
# (workflow L-E2E шлёт харнесс на 249 напрямую с GitHub runner). Двойная
# защита: не удалять ничего моложе MIN_AGE_MIN.
log "min-age guard: ${MIN_AGE_MIN} мин (файлы моложе не трогаем)"

# --- 3. Собираем список мусора на 249 ---------------------------------------
# Паттерны: старые ключи, build-логи, wav-записи прошлых ранов, e2e_v2_*.
# Найти файлы старше MIN_AGE_MIN и НЕ трогать e2e_voice_test.sh.
find_cmd="find /tmp -maxdepth 1 -type f \( \
    -name 'yandex_key_*' -o -name 'build_*.log' -o -name 'dialog_e2e_*.wav' \
    -o -name 'e2e_v2_*' -o -name 'voice_e2e_*.log' -o -name 'e2e_atomic_out.log' \
\) -mmin +${MIN_AGE_MIN} ! -name 'e2e_voice_test.sh' -print 2>/dev/null"

if [ "$DRY_RUN" = "1" ]; then
    log "DRY-RUN: список файлов для удаления на ${BUILD_HOST} (старше ${MIN_AGE_MIN} мин):"
    sshpass -p "$SSHPASS_VAL" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 \
        "${BUILD_USER}@${BUILD_HOST}" "$find_cmd" 2>/dev/null | sed 's/^/  /'
    log "DRY-RUN: ничего не удалено"
    exit 0
fi

# Удаляем по одному (rm -f не падает на отсутствующих; -- не даёт съесть флаги).
# Ретро 14.08 (t_cf325071): root-owned файлы (yandex_key_*/voice_e2e_*.log от
# прошлых ранов) НЕ удаляются пользователем ros2 → xargs rm возвращает 123 →
# set -euo pipefail убивал ВЕСЬ скрипт ДО секции 4 (round-cleanup на GitHub).
# Именно поэтому stale round-ветки 61-110 не сносились неделями. Фикс:
#  - rm с 2>/dev/null (root-owned мусор — норма, не спамим лог);
#  - || true на пайп (независимо от rc rm/xargs скрипт идёт дальше).
sshpass -p "$SSHPASS_VAL" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 \
    "${BUILD_USER}@${BUILD_HOST}" \
    "list=\$($find_cmd); if [ -n \"\$list\" ]; then printf '%s\n' \"\$list\" | xargs -r rm -f -- 2>/dev/null && echo \"CLEANUP_OK removed: \$(printf '%s\n' \"\$list\" | wc -l) files\" || echo \"CLEANUP_OK partial (root-owned files skipped): \$(printf '%s\n' \"\$list\" | wc -l) files\"; else echo 'CLEANUP_OK nothing to remove'; fi" 2>&1 | sed 's/^/  /' || true

# --- 4. stale round-branch cleanup на remote (ретро 12.08 t_d3aeaa9b) ------
# ПРАВИЛО: round-ветка без e2e-активности > ROUND_STALE_HOURS (48ч) → delete.
# e2e-активность = последний коммит в round-ветке: e2e-process мержит
# agent-ветку в round и пушит ПЕРЕД запуском L-E2E, поэтому свежий коммит =
# свежий прогон. Старые round-ветки (напр. round-59, создан из develop ДО
# фиксов валидатора #1143 и ротации #1141) несут устаревшую базу: при reuse
# round_ensure берёт max-N со старой базой → e2e гоняется на регрессе.
# Guard: flock e2e-process уже проверен в секции 1 (активный round не тронем).
if [ -n "${GH_REPO:-}" ] && command -v gh >/dev/null 2>&1 && gh auth status >/dev/null 2>&1; then
    log "round-cleanup: ищу stale round-ветки (>${ROUND_STALE_HOURS}ч без e2e-активности) на ${GH_REPO}"
    now_epoch="$(date +%s)"
    while read -r round_branch round_sha; do
        [ -z "$round_branch" ] && continue
        last_date="$(gh api "repos/${GH_REPO}/commits/${round_sha}" --jq '.commit.committer.date' 2>/dev/null || echo '')"
        if [ -z "$last_date" ]; then
            log "  WARN: не удалось получить дату для ${round_branch} (${round_sha}) — пропуск"
            continue
        fi
        last_epoch="$(date -d "$last_date" +%s 2>/dev/null || echo 0)"
        age_h=$(( (now_epoch - last_epoch) / 3600 ))
        if [ "$age_h" -gt "$ROUND_STALE_HOURS" ]; then
            if [ "$DRY_RUN" = "1" ]; then
                log "  DRY-RUN: удалил бы ${round_branch} (последний коммит ${age_h}ч назад)"
            else
                # URL-encode { } в имени ветки (GitHub API требует %7B/%7D)
                enc="${round_branch//\{/%7B}"; enc="${enc//\}/%7D}"
                if gh api -X DELETE "repos/${GH_REPO}/git/refs/heads/${enc}" >/dev/null 2>&1; then
                    log "  DELETED stale round ${round_branch} (последний коммит ${age_h}ч назад)"
                else
                    log "  WARN: не удалось удалить ${round_branch}"
                fi
            fi
        else
            log "  keep ${round_branch} (последний коммит ${age_h}ч назад)"
        fi
    done < <(gh api "repos/${GH_REPO}/branches?per_page=100" \
        --jq '.[] | select(.name | startswith("z-{e2e}/test-round-")) | "\(.name)\t\(.commit.sha)"' 2>/dev/null || true)
else
    log "round-cleanup: GH_REPO/gh недоступны — пропуск (только /tmp cleanup)"
fi

# --- 5. stale PR-branch cleanup на remote (ретро 14.08 t_3cfb3b5b) ----------
# ПРОБЛЕМА (14.08): секция 4 чистит только z-{e2e}/test-round-*; per-card и
# прочие ветки после merge/close их PR копятся на origin вечно (в репо
# auto-delete-head-branches выключен, merge-gate архивирует карточки, но ветки
# не удаляет) и замусоривают реконсилейшн PR-сканы.
# ПРАВИЛО: remote-ветка, чей PR MERGED > MERGED_STALE_HOURS (default 2ч) ИЛИ
# CLOSED без merge > CLOSED_STALE_HOURS (default 24ч) → delete.
# Guard'ы (безопасность):
#   a. OPEN PR — ветки открытых PR не трогаем никогда;
#   b. default-ветка и develop (интеграционная) — не трогаем;
#   c. round-ветки z-{e2e}/test-round-* — не трогаем (их чистит секция 4);
#   d. fork-PR — ветка живёт в fork'е, origin не трогаем;
#   e. переиспользование — если HEAD-коммит ветки НОВЕЕ момента merge/close
#      PR (в ветку пушили после закрытия PR — ветку переиспользуют), не удаляем.
# Функция выделена, чтобы тестироваться изолированно
# (tests/test_cleanup_pr_sweep.sh).
sweep_stale_pr_branches() {
    local default_branch develop_branch repo_owner now_epoch
    local merged_list closed_list open_list branches_list seen_list
    local -A open_branches branch_sha

    log "pr-cleanup: sweep stale PR-веток (merged>${MERGED_STALE_HOURS}ч / closed>${CLOSED_STALE_HOURS}ч)"

    merged_list="$(mktemp)"; closed_list="$(mktemp)"; open_list="$(mktemp)"
    branches_list="$(mktemp)"; seen_list="$(mktemp)"
    # EXIT-trap: пути подставляем СЕЙЧАС (locals умрут раньше script-exit)
    trap "rm -f '$merged_list' '$closed_list' '$open_list' '$branches_list' '$seen_list'" EXIT

    default_branch="$(gh repo view "${GH_REPO}" --json defaultBranchRef --jq '.defaultBranchRef.name' 2>/dev/null || echo main)"
    develop_branch="develop"
    repo_owner="${GH_REPO%/*}"
    now_epoch="$(date +%s)"
    log "pr-cleanup: repo=${GH_REPO} default=${default_branch} owner=${repo_owner}"

    # guard a: собрать ветки OPEN PR заранее
    gh pr list --state open --limit 200 --json headRefName,headRepositoryOwner \
        --jq '.[] | select(.headRefName != null) | "\(.headRefName)\t\(.headRepositoryOwner.login)"' \
        2>/dev/null > "$open_list" || true
    local b owner
    while IFS=$'\t' read -r b owner; do
        [ -n "$b" ] && open_branches["$b"]=1
    done < "$open_list"

    # merged + closed PR списки. В REST closed ВКЛЮЧАЕТ merged (mergedAt!=null):
    # merged-проход обрабатывает merged-правило, closed-проход — только
    # closed-без-merge (merged-записи пропускаем через seen_list, либо — если
    # ветка не встречалась в merged-лимите — применяем merged-правило).
    gh pr list --state merged --limit 1000 --json number,headRefName,headRepositoryOwner,mergedAt \
        --jq '.[] | select(.headRefName != null) | "\(.number)\t\(.headRefName)\t\(.headRepositoryOwner.login)\t\(.mergedAt)"' \
        2>/dev/null > "$merged_list" || true
    gh pr list --state closed --limit 1000 --json number,headRefName,headRepositoryOwner,closedAt,mergedAt \
        --jq '.[] | select(.headRefName != null) | "\(.number)\t\(.headRefName)\t\(.headRepositoryOwner.login)\t\(.closedAt)\t\(.mergedAt)"' \
        2>/dev/null > "$closed_list" || true

    # карта ветка→sha (для guard e; --paginate — веток может быть >100)
    gh api --paginate "repos/${GH_REPO}/branches?per_page=100" \
        --jq '.[] | "\(.name)\t\(.commit.sha)"' 2>/dev/null > "$branches_list" || true
    local br sha
    while IFS=$'\t' read -r br sha; do
        [ -n "$br" ] && branch_sha["$br"]="$sha"
    done < "$branches_list"

    # $1=pr_number $2=branch $3=owner $4=state_time $5=label $6=stale_hours
    try_delete_pr_branch() {
        local pr_num="$1" branch="$2" owner="$3" state_time="$4" label="$5" stale_hours="$6"
        [ -n "$branch" ] || return 0
        # guard b: защищённые ветки
        if [ "$branch" = "$default_branch" ] || [ "$branch" = "$develop_branch" ]; then
            log "  keep ${branch} (защищённая ветка; ${label} PR #${pr_num})"
            return 0
        fi
        # guard c: round-ветки — секция 4
        case "$branch" in
            "z-{e2e}/test-round-"*) log "  keep ${branch} (round-ветка — секция 4)"; return 0 ;;
        esac
        # guard d: fork-PR — ветка в fork'е, origin не трогаем
        if [ "$owner" != "$repo_owner" ]; then
            log "  keep ${branch} (fork PR owner=${owner})"
            return 0
        fi
        # guard a: OPEN PR
        if [ -n "${open_branches[${branch}]:-}" ]; then
            log "  keep ${branch} (есть OPEN PR)"
            return 0
        fi
        # возраст по времени PR-состояния
        local state_epoch=0 age_h=0
        state_epoch="$(date -d "$state_time" +%s 2>/dev/null || echo 0)"
        [ "$state_epoch" -gt 0 ] || { log "  WARN: не распознана дата '${state_time}' для ${branch} — пропуск"; return 0; }
        age_h=$(( (now_epoch - state_epoch) / 3600 ))
        if [ "$age_h" -le "$stale_hours" ]; then
            log "  keep ${branch} (PR #${pr_num} ${label} ${age_h}ч назад ≤ ${stale_hours}ч)"
            return 0
        fi
        # ветки, которой уже нет на remote, не трогаем (не дёргаем API и не
        # шумим 404-удалениями). Если branches-API упала (пустая карта) —
        # fail-open: ничего не удаляем.
        if [ -z "${branch_sha[${branch}]:-}" ]; then
            log "  skip ${branch} (ветка уже удалена на remote)"
            return 0
        fi
        # guard e: ветка переиспользована после merge/close (HEAD новее state_time)
        local sha="${branch_sha[${branch}]}"
        if [ -n "$sha" ]; then
            local head_date="" head_epoch=0
            head_date="$(gh api "repos/${GH_REPO}/commits/${sha}" --jq '.commit.committer.date' 2>/dev/null || echo '')"
            if [ -n "$head_date" ]; then
                head_epoch="$(date -d "$head_date" +%s 2>/dev/null || echo 0)"
                if [ "$head_epoch" -gt "$state_epoch" ]; then
                    log "  keep ${branch} (HEAD ${head_date} новее ${label} ${state_time} — ветка переиспользована)"
                    return 0
                fi
            else
                log "  WARN: не удалось получить дату HEAD для ${branch} — fail-open (keep)"
                return 0
            fi
        fi
        # DELETE
        if [ "$DRY_RUN" = "1" ]; then
            log "  DRY-RUN: удалил бы ${branch} (PR #${pr_num} ${label} ${age_h}ч назад)"
            return 0
        fi
        # URL-encode { } в имени ветки (GitHub API требует %7B/%7D)
        local enc="${branch//\{/%7B}"; enc="${enc//\}/%7D}"
        if gh api -X DELETE "repos/${GH_REPO}/git/refs/heads/${enc}" >/dev/null 2>&1; then
            log "  DELETED stale ${branch} (PR #${pr_num} ${label} ${age_h}ч назад)"
        else
            log "  WARN: не удалось удалить ${branch} (PR #${pr_num} ${label}) — ветка, возможно, уже удалена"
        fi
    }

    log "pr-cleanup: merged-проход (merged > ${MERGED_STALE_HOURS}ч → delete)"
    local pr_num merged_at closed_at
    while IFS=$'\t' read -r pr_num b owner merged_at; do
        [ -n "$b" ] || continue
        echo "$b" >> "$seen_list"
        try_delete_pr_branch "$pr_num" "$b" "$owner" "$merged_at" "merged" "$MERGED_STALE_HOURS"
    done < "$merged_list"

    log "pr-cleanup: closed-проход (closed > ${CLOSED_STALE_HOURS}ч → delete)"
    while IFS=$'\t' read -r pr_num b owner closed_at merged_at; do
        [ -n "$b" ] || continue
        if grep -qxF "$b" "$seen_list"; then
            continue  # уже обработана в merged-проходе
        fi
        echo "$b" >> "$seen_list"
        if [ -n "$merged_at" ] && [ "$merged_at" != "null" ]; then
            # merged PR вне merged-лимита — применяем merged-правило
            try_delete_pr_branch "$pr_num" "$b" "$owner" "$merged_at" "merged" "$MERGED_STALE_HOURS"
        else
            try_delete_pr_branch "$pr_num" "$b" "$owner" "$closed_at" "closed" "$CLOSED_STALE_HOURS"
        fi
    done < "$closed_list"
}

if [ -n "${GH_REPO:-}" ] && command -v gh >/dev/null 2>&1 && gh auth status >/dev/null 2>&1; then
    sweep_stale_pr_branches
else
    log "pr-cleanup: GH_REPO/gh недоступны — пропуск (только /tmp + round cleanup)"
fi

log "cleanup done (${BUILD_HOST}, min-age ${MIN_AGE_MIN} мин)"
