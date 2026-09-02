#!/bin/bash
# ============================================================================
# agent-flow-protocol-violation-watchdog.sh — auto-resolve agent-flow tasks
# that crashed via `protocol_violation` (worker exited rc=0 without calling
# kanban_complete/kanban_block) when the underlying work is already in develop.
#
# Контекст / ретро t_52a6b973 (2026-09-02):
#   Pattern "protocol_violation loop": agent-flow worker делает реальную работу
#   (коммитит в feature-ветку, открывает PR, merge-gate мёрджит в develop),
#   но в финале сессии НЕ вызывает `kanban complete` (например, из-за
#   timeout на goal-loop, или worker вышел за рамки max_runtime).
#   Dispatcher трактует rc=0 без terminal call как `protocol_violation`,
#   через 3 итерации (_PROTOCOL_VIOLATION_FAILURE_LIMIT в hermes) карточка
#   уходит в gave_up → blocked → ручной triage.
#
#   Реальный кейс (t_0ed5689a / issue #1780):
#     - Рут-карточка висит в todo 24+ часа.
#     - Дочерние t_a5eed3a7 / t_4e98182a / t_956e0eb4 / t_85b38d89 — archived
#       (PR #1790, #1793, #1820 MERGED в develop).
#     - Работа выполнена; paperwork — нет.
#
# Контракт (per tick):
#   1. List tasks в статусе `todo`/`ready` с protocol_violation streak > 0
#      и assignee=agent-flow.
#   2. Для каждой: найти issue number (через body card — Source.issue), найти
#      MERGED PR (gh pr list --state merged --search "#<N>"), проверить что
#      mergeCommit.sha присутствует хотя бы в одном из base branches
#      (develop / feature/avatar / feature/quest).
#   3. Если да — emit `hermes kanban complete <tid>` с verifier-style
#      summary (кто, какие PR, какой sha). Не вызывается при наличии recent
#      watchdog-marker в комментариях карточки (идемпотентность 24h).
#
#   4. Если MERGED PR найден но mergeCommit отсутствует в base → НЕ завершаем
#      (orphan PR; пусть merge-gate rebase-разруливает).
#
# ENV:
#   GH_REPO                       — owner/repo (default krikz/rob_box_project)
#   KANBAN_BOARD                  — board slug (default robbox)
#   REPO_DIR                      — local clone для git branch --contains
#   BASE_BRANCHES                 — colon-separated (default develop:feature/avatar:feature/quest)
#   PROTOCOL_VIOLATION_DRY_RUN    — true → only log, no side-effects
#   LOCK_FILE                     — flock guard (default /tmp/agent-flow-protocol-violation-watchdog.lock)
#   WINDOW_HOURS                  — idempotency window для marker-комментария (default 24)
#
# Выходы:
#   stderr: structured summary
#   exit 0 — всё ok (включая «nothing to do»)
#   exit 1 — критичный сбой (gh auth, repo недоступен, sqlite locked)
#   exit 2 — найдены recovery-кандидаты (alert для cron, опционально)
#
# Pitfalls (gotchas):
#   - Не закрывать карточку если PR найден, но не смержен (gh pr list --state
#     merged обязателен; иначе ловим OPEN PR → false-positive).
#   - base branch проверка ОБЯЗАТЕЛЬНА: orphan PR может указывать на
#     feature/avatar, а у нас base=develop → sha отсутствует → skip.
#   - issue number извлекаем из `Source.issue: #N` в body; fallback — grep
#     issue-reference regex по всему body (менее надёжно).
#   - Не вызывать complete если task в `running` (значит worker ещё работает;
#     watchdog должен только лечить `todo`/`ready` после crash).
#   - marker-комментарий должен быть коротким и помеченным script= для
#     фильтрации следующих тиков.
# ============================================================================
set -euo pipefail

GH_REPO="${GH_REPO:-krikz/rob_box_project}"
KANBAN_BOARD="${KANBAN_BOARD:-robbox}"
REPO_DIR="${REPO_DIR:-/home/builder/hermes-share/rob_box_project}"
DRY_RUN="${PROTOCOL_VIOLATION_DRY_RUN:-false}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-protocol-violation-watchdog.lock}"
BASE_BRANCHES="${BASE_BRANCHES:-develop:feature/avatar:feature/quest}"
WINDOW_HOURS="${WINDOW_HOURS:-24}"

# --- flock guard (avoid race with dispatcher / merge-gate) -----------------
exec 9>"$LOCK_FILE" || true
if ! flock -n 9; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] protocol-violation-watchdog: another instance running — skip"
    exit 0
fi

# --- pre-flight ------------------------------------------------------------
if ! gh auth status >/dev/null 2>&1; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] protocol-violation-watchdog: gh auth failed — exit 1" >&2
    exit 1
fi
[ -d "${REPO_DIR}/.git" ] || {
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] protocol-violation-watchdog: REPO_DIR not a git repo — exit 1" >&2
    exit 1
}

# --- helpers ---------------------------------------------------------------
_now_iso() { date -u +%Y-%m-%dT%H:%M:%SZ; }
_now_s()   { date -u +%s; }

_marker_tag="🤖 [agent:devops] script=agent-flow-protocol-violation-watchdog"

# list_protocol_violation_tasks — возвращает tab-separated: task_id<TAB>issue_number<TAB>summary
# Только assignee=agent-flow + status in (todo, ready) + имеющие protocol_violation streak > 0
# (т.е. в теле карточки или в events есть protocol_violation=true marker).
list_protocol_violation_tasks() {
    hermes kanban --board "${KANBAN_BOARD}" list \
        --assignee agent-flow \
        --status todo,ready \
        --json 2>/dev/null \
    | python3 -c "
import json, re, sys
try:
    rows = json.load(sys.stdin)
except Exception:
    sys.exit(0)
# rows может быть list[dict] или {tasks: [...]}; нормализуем
if isinstance(rows, dict):
    rows = rows.get('tasks') or rows.get('items') or []
for r in rows:
    if not isinstance(r, dict):
        continue
    body = r.get('body') or ''
    if 'protocol_violation' not in body and 'consecutive_crashes' not in body:
        continue
    m = re.search(r'(?:Source|issue|issues?)\s*[:#]?\s*#?(\d{2,5})', body)
    issue = m.group(1) if m else ''
    print(f'{r[\"id\"]}\t{issue}\t{(r.get(\"title\") or \"\")[:80]}')
" 2>/dev/null
}

# card_has_recent_marker <tid> — проверяет, был ли уже наш marker-коммент за
# последние WINDOW_HOURS часов. Возвращает 0 если найден (→ idempotent skip).
# Использует grep+head -1 для timestamp-фильтра (POSIX-safe, без awk ERE).
card_has_recent_marker() {
    local _tid="$1"
    local _since_iso
    _since_iso="$(date -u -d "@$(( $(date -u +%s) - WINDOW_HOURS * 3600 ))" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || true)"
    [ -n "$_since_iso" ] || return 1
    # Вытаскиваем все строки лога с timestamp >= since и ищем marker.
    hermes kanban --board "${KANBAN_BOARD}" log "${_tid}" --limit 30 2>/dev/null \
        | awk -v since="${_since_iso}" '
            {
                # первое поле — timestamp вида 2026-09-02T02:00:00Z
                ts = substr($0, 1, 19)
                if (ts >= since) print
            }' \
        | grep -qF "${_marker_tag}" 2>/dev/null
}

# find_merged_pr <issue_num> — ищет MERGED PR с title, содержащим exact #<N>.
# Возвращает "PR#:sha:title" (stdout) или пустую строку.
# Если issue_num пустой — сразу возвращает пустую (нет смысла искать).
find_merged_pr() {
    local _num="$1"
    [ -n "${_num}" ] || return 0
    gh pr list --repo "${GH_REPO}" --state merged --search "#${_num}" \
        --json number,mergeCommit,baseRefName,headRefName,title 2>/dev/null \
    | python3 -c "
import json, re, sys
try:
    data = json.load(sys.stdin)
except Exception:
    sys.exit(0)
issue = '${_num}'
for pr in data:
    mc = pr.get('mergeCommit') or {}
    oid = mc.get('oid') if isinstance(mc, dict) else None
    if not oid:
        continue
    title = pr.get('title') or ''
    if re.search(r'(^|[^0-9])#' + issue + r'([^0-9]|$)', title):
        print(f'{pr[\"number\"]}:{oid}:{title[:60]}')
        sys.exit(0)
" 2>/dev/null
}

# is_in_base <sha> <base_branch> — проверяет, что sha присутствует в base
# (через `git branch --contains`). Возвращает 0 если найден.
# Проверяется и локальный branch, и remotes/origin/<base> (т.к. локальный
# develop может отсутствовать если fetch давно не запускался).
is_in_base() {
    local _sha="$1" _base="$2"
    local _branches
    _branches="$(
        { git -C "${REPO_DIR}" branch -a --contains "${_sha}" 2>/dev/null; \
          git -C "${REPO_DIR}" branch    --contains "${_sha}" 2>/dev/null; } \
        | grep -vE '^\*|^\+' \
        | awk '{print $NF}' \
        | sort -u
    )"
    grep -qxF "${_base}"       <<< "${_branches}" && return 0
    grep -qxF "remotes/origin/${_base}" <<< "${_branches}" && return 0
    return 1
}

# --- main loop -------------------------------------------------------------
_checked=0
_completed=0
_skipped_idempotent=0
_skipped_no_pr=0
_skipped_orphan_pr=0
_errors=0
_recovery=()

# list_protocol_violation_tasks пишет в stdout tab-separated: tid<TAB>issue<TAB>title.
# Сохраняем в массив чтобы не зависеть от subshell+pipefail.
_task_list="$(list_protocol_violation_tasks || true)"
if [ -z "${_task_list}" ]; then
    echo "[$(_now_iso)] pv-watchdog: no candidate tasks (list empty or all filtered out)"
    echo "[$(_now_iso)] pv-watchdog: checked=0 completed=0 skipped_idempotent=0 skipped_no_pr=0 skipped_orphan_pr=0 errors=0"
    exit 0
fi

# Используем process-substitution чтобы избежать subshell+pipefail проблем
# и бесконечного цикла (<<< повторяет строку).
# Разбираем каждую строку отдельно через mapfile (надёжнее чем while read + IFS).
# ВАЖНО: bash `read` с IFS=$'\t' НЕ сохраняет пустые поля — схлопывает.
# Поэтому парсим через split-string python в самом list_protocol_violation_tasks
# (это уже сделано: вывод tab-separated, где пустой issue → \t\t).
# Здесь просто split по tabs без IFS.
mapfile -t _task_lines < <(printf '%s\n' "${_task_list}")
for _line in "${_task_lines[@]}"; do
    [ -n "${_line}" ] || continue
    # parse tab-separated WITHOUT IFS-collapse: первое поле = до первого \t,
    # последнее = после последнего \t, среднее = между ними.
    tid="${_line%%$'\t'*}";            # до первого \t
    _rest="${_line#*$'\t'}";           # после первого \t
    issue_num="${_rest%%$'\t'*}";      # до следующего \t (= 2-й field)
    title="${_rest#*$'\t'}";           # после (= 3-й field, может содержать \t — нам неважно)
    [ -n "${tid}" ] || continue
    _checked=$(( _checked + 1 ))

    # 1. idempotency: уже был наш marker-коммент за последние 24h
    if card_has_recent_marker "${tid}"; then
        _skipped_idempotent=$(( _skipped_idempotent + 1 ))
        echo "[$(_now_iso)] pv-watchdog: SKIP ${tid} (recent marker found)"
        continue
    fi

    # 2. нужен issue number — без него нечего искать
    if [ -z "${issue_num}" ]; then
        _skipped_no_pr=$(( _skipped_no_pr + 1 ))
        echo "[$(_now_iso)] pv-watchdog: SKIP ${tid} (no issue number in body)"
        continue
    fi

    # 3. ищем MERGED PR для issue
    _pr_info="$(find_merged_pr "${issue_num}" || true)"
    if [ -z "${_pr_info}" ]; then
        _skipped_no_pr=$(( _skipped_no_pr + 1 ))
        echo "[$(_now_iso)] pv-watchdog: SKIP ${tid} (no merged PR for #${issue_num})"
        continue
    fi
    _pr_num="${_pr_info%%:*}"
    _rest="${_pr_info#*:}"
    _merge_sha="${_rest%%:*}"
    _pr_title="${_rest#*:}"

    # 4. mergeCommit.sha должен быть в одном из base branches
    _in_base=0
    _in_base_branch=""
    IFS=':' read -r -a _bases <<< "${BASE_BRANCHES}"
    for b in "${_bases[@]}"; do
        if is_in_base "${_merge_sha}" "${b}"; then
            _in_base=1
            _in_base_branch="${b}"
            break
        fi
    done
    if [ "${_in_base}" -ne 1 ]; then
        _skipped_orphan_pr=$(( _skipped_orphan_pr + 1 ))
        echo "[$(_now_iso)] pv-watchdog: SKIP ${tid} (PR#${_pr_num} sha=${_merge_sha} not in any base branch)"
        continue
    fi

    # 5. side-effect: comment + complete
    _summary="protocol-violation recovery: PR #${_pr_num} \"${_pr_title}\" merged into ${_in_base_branch} at sha ${_merge_sha:0:12}. Work is in develop — closing card."
    _metadata="$(python3 -c "
import json
print(json.dumps({
    'verdict': 'recovered_via_protocol_violation_watchdog',
    'pr_number': ${_pr_num},
    'merge_sha': '${_merge_sha}',
    'merge_base': '${_in_base_branch}',
    'issue_number': ${issue_num},
    'recovery_reason': 'worker exited rc=0 without kanban_complete; PR already MERGED into base'
}))
")"

    echo "[$(_now_iso)] pv-watchdog: RECOVER ${tid} (issue #${issue_num} → PR #${_pr_num} in ${_in_base_branch})"

    if [ "${DRY_RUN}" = "true" ]; then
        echo "[$(_now_iso)] pv-watchdog: DRY-RUN — would comment + complete ${tid}"
        continue
    fi

    # 5a. marker-комментарий
    if ! hermes kanban --board "${KANBAN_BOARD}" comment "${tid}" \
            "${_marker_tag}
auto-recovery: работа issue #${issue_num} уже merged через PR #${_pr_num} в ${_in_base_branch} (sha ${_merge_sha:0:12}). worker exited rc=0 без kanban_complete → watchdog закрывает карточку." \
            >/dev/null 2>&1; then
        _errors=$(( _errors + 1 ))
        echo "[$(_now_iso)] pv-watchdog: ERROR comment failed for ${tid}" >&2
        continue
    fi

    # 5b. complete с verifier-summary
    if hermes kanban --board "${KANBAN_BOARD}" complete "${tid}" \
            --summary "${_summary}" \
            --metadata "${_metadata}" \
            >/dev/null 2>&1; then
        _completed=$(( _completed + 1 ))
        _recovery+=("${tid}:PR#${_pr_num}:${_in_base_branch}")
    else
        _errors=$(( _errors + 1 ))
        echo "[$(_now_iso)] pv-watchdog: ERROR complete failed for ${tid}" >&2
    fi
done

# --- summary ----------------------------------------------------------------
echo "[$(_now_iso)] pv-watchdog: checked=${_checked} completed=${_completed} skipped_idempotent=${_skipped_idempotent} skipped_no_pr=${_skipped_no_pr} skipped_orphan_pr=${_skipped_orphan_pr} errors=${_errors}"
if [ "${#_recovery[@]}" -gt 0 ]; then
    echo "[$(_now_iso)] pv-watchdog: recovered tasks:"
    for r in "${_recovery[@]}"; do
        echo "  - ${r}"
    done
    exit 2  # alert: found recovery candidates (для cron, опционально)
fi
exit 0
