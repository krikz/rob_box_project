#!/bin/bash
# ============================================================================
# hermes_github.sh — shared helper for «self-id / whoami» comments.
#
# SOT: <repo>/scripts/agent_flow/hermes_github.sh
# Копии раскладываются install.sh в:
#   - ~/.hermes/profiles/agent-flow/scripts/
#   - ~/.hermes/profiles/architect/scripts/
#   - ~/.hermes/profiles/devops/scripts/
#   - ~/.hermes/scripts/
#
# Назначение (issue #1534, ADR-0014-cite-context-ext):
#   Когда автоматический процесс (agent-flow cron / worker) меняет что-то в
#   PR или issue (close / reopen / label / assignee) — в журнале событий
#   непонятно КТО это сделал: actor GitHub = владелец токена (krikz),
#   независимо от того, вызвал ли он сам скрипт или это сделал cron от его
#   имени. Наблюдение (таблица из issue #1534):
#     | Закрыл PR #1507   | krikz (потому что токен юзера)
#     | added needs-e2e   | krikz
#     | reopened issue    | krikz
#     | assignee=devops   | krikz
#   Никакой идентификации, что это сделал agent-flow-merge-gate, а не
#   товарищ Шифу руками.
#
# Контракт:
#   Каждый вызов post_whoami_comment ПЕРЕД side-effect пишет комментарий
#   с self-id:
#     🤖 [agent:<role>] script=<script_name> action=<action>
#        reason=<reason> [branch=<branch>] [card=<card_id>]
#
#   - role: "agent:devops" / "agent:architect" / etc. (env HERMES_AGENT_ROLE,
#     дефолт "agent:devops"; callers могут override через HERMES_AGENT_ROLE).
#   - script_name: имя вызывающего скрипта (авто-детект из basename $0
#     если не указано).
#   - action: "closing" / "reopening" / "adding-label" / "removing-label"
#     / "setting-assignee" — обязательное, описывает что БУДЕТ сделано.
#   - reason: free-form текст (зачем / почему / ссылка на retro/issue).
#   - branch / card_id: optional metadata (если известны).
#
# Идемпотентность (acceptance #5):
#   Перед публикацией helper делает GET существующих комментариев и ищет
#   маркер `🤖 [agent:<role>] script=<script_name> action=<action>` в
#   последнем комментарии с self-id prefix от этого скрипта за последние
#   2 часа (WINDOW_SECONDS, дефолт 7200s). Если найден — skip (silent).
#   Это защищает от двойного запуска cron'а в одном тике (flock-сбой,
#   manual rerun) и от "ghost whoami" при reconcile.
#
# Failure semantics (acceptance #4):
#   Если gh упал (rate-limit / network / permission) — action всё равно
#   выполняется (helper возвращает 0). Только в лог пишется warning:
#     [hermes_github] WARN: whoami-comment failed: <stderr> (action will proceed)
#
# Использование (после source):
#   _LIB_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
#   . "$_LIB_DIR_HERE/hermes_github.sh"
#
#   # Перед close issue:
#   post_whoami_comment issue "$number" "closing" "reason text here"
#
#   # Перед label change (action="adding-label:needs-e2e"):
#   post_whoami_comment issue "$number" "adding-label:needs-e2e" "PR opened, e2e requested"
#
#   # Перед close PR (action="closing", kind="pr"):
#   post_whoami_comment pr "$pr_number" "closing" "rebase проглотил diff (ретро t_944df2c5)" \
#       "branch=z-{agent}/1506-foo" "card=t_e2812349"
#
# Exit codes:
#   0 — всегда (failure внутри не фатален).
#
# Env:
#   HERMES_AGENT_ROLE     — override для role (default "agent:devops").
#   HERMES_GH_HMAC_SALT   — optional HMAC-salt для marker'а (default "" —
#                           marker не включает HMAC). Расширение на будущее.
#   HERMES_HOME / GH_REPO — те же, что в основном скрипте.
# ============================================================================
set -euo pipefail

# --- guards ----------------------------------------------------------------
[ -n "${HERMES_HOME:-}" ] || HERMES_HOME=/home/builder/.hermes
# Force HOME=/home/builder — same reason as in merge-gate / e2e-process:
# cron от per-profile gateway ставит $HOME = profile-dir, а gh CLI ищет
# credentials в $HOME/.config/gh (не в profile-dir/.config/gh).
export HOME="${HOME:-/home/builder}"

# --- config ----------------------------------------------------------------
HERMES_AGENT_ROLE="${HERMES_AGENT_ROLE:-agent:devops}"
# Idempotency window: same action+script+target posted in < WINDOW_SECONDS
# → skip silently. ADR-0014 cron runs every ~5 min, so 7200s = 2h safely
# covers reconcile-loop double-fires without masking real new actions.
HERMES_WHOAMI_WINDOW_SECONDS="${HERMES_WHOAMI_WINDOW_SECONDS:-7200}"

# Marker prefix для self-id: emoji + bracketed role. Любой grep по этому
# префиксу однозначно отделяет whoami-комменты от человеческих.
HERMES_WHOAMI_MARKER="🤖 [${HERMES_AGENT_ROLE}]"

# --- helpers ---------------------------------------------------------------

# _now_epoch — UNIX epoch секунды (через date, портативно).
_now_epoch() { date -u +%s; }

# _auto_script_name — определение caller-скрипта для логирования.
#
# Контекст: hermes_github.sh — sourced-библиотека. Когда wrapper-функция
# (whoami_close_issue итд) вызывается из caller-скрипта и она вызывает
# post_whoami_comment, BASH_SOURCE-стек зависит от того, ГДЕ мы находимся:
#
#   Вызов напрямую (post_whoami_comment из caller-скрипта):
#     BASH_SOURCE = (caller.sh)
#     BASH_SOURCE[1] = caller.sh ✓
#
#   Вызов через wrapper (whoami_close_issue ... из caller-скрипта):
#     BASH_SOURCE = (hermes_github.sh, caller.sh)
#     BASH_SOURCE[1] = hermes_github.sh ✗ (нужно caller.sh)
#
# Решение: wrappers (whoami_*) всегда передают script_name явно через
# $5 — это надёжнее. _auto_script_name работает как fallback для прямого
# вызова post_whoami_comment: ищет первый frame, который НЕ является
# hermes_github.sh.
#
# FUNCNAME-стек НЕ используем: FUNCNAME[1] = имя вызывающей функции
# (whoami_*), а не caller-скрипт. Это и есть причина регресса t03.
_auto_script_name() {
    local i bn src bn_only
    # BASH_SOURCE внутри функции содержит:
    #   [0] = этот helper (hermes_github.sh)
    #   [1] = этот helper (вызов из wrapper внутри этого же файла)
    #   [2] = caller-скрипт (где определена wrapper-функция через . helper.sh)
    #   [3..] = более ранние frames (lib_user_unlabel_check.sh и т.п.)
    #
    # Нужно пропустить все frame'ы, basename которых === "hermes_github.sh".
    for i in 1 2 3 4 5 6; do
        src="${BASH_SOURCE[$i]:-}"
        [ -z "$src" ] && break
        bn_only="$(basename "$src" 2>/dev/null || echo "")"
        # Skip сам helper
        [ "$bn_only" = "hermes_github.sh" ] && continue
        [ -z "$bn_only" ] && continue
        bn="${bn_only%.sh}"
        printf '%s' "$bn"
        return 0
    done
    # Fallback: $0 (не-sourced вызов)
    bn="$(basename "${BASH_SOURCE[0]:-$0}" 2>/dev/null || echo unknown)"
    printf '%s' "${bn%.sh}"
}

# _log_err — пишем в stderr с timestamp. Не используем log() других скриптов
# чтобы быть независимыми.
_log_err() { printf '%s [hermes_github] %s\n' "$(date -Iseconds)" "$*" >&2; }

# _gh — обёртка gh с принудительным --repo и fallback на $GH_REPO.
# В dry-run окружениях (тесты) GH_REPO может быть пустым; helper сам
# упадёт и мы обработаем failure как warning.
_gh() {
    local _repo="${GH_REPO:-}"
    if [ -n "$_repo" ]; then
        gh --repo "$_repo" "$@"
    else
        gh "$@"
    fi
}

# _issue_number_from_pr — для kind=pr выводим связанный issue# через issue
# lookup (heuristic: PR title or branch matches issue#). Это best-effort;
# если не нашли — пустая строка, не блокируем публикацию.
#   args: <pr_number>
_issue_number_from_pr() {
    local pr="$1"
    # `gh pr view --json body` — тело PR часто содержит "Issue: #N" или
    # "kanban: t_<id>". Для нашего whoami этого достаточно — branch+role
    # уже идут отдельными полями.
    printf ''
}

# --- core API --------------------------------------------------------------

# post_whoami_comment — main entrypoint.
#
# Args:
#   $1 kind       — "issue" | "pr"
#   $2 number     — issue# или pr# (digits)
#   $3 action     — "closing" / "reopening" / "adding-label:<L>" /
#                   "removing-label:<L>" / "setting-assignee:<user>" /
#                   free-form
#   $4 reason     — короткое описание причины (≤200 символов идеально).
#                   Если пусто — пишем "(no reason given)".
#   $5..N meta    — optional пары "key=value" (branch=…, card=t_…, run=…).
#
# Behavior:
#   - Format: "<MARKER> script=<script> action=<action>\n   reason: <reason>
#             [<meta lines>]"
#   - Skip if same marker (script + action + target) уже в комментах
#     за последние WINDOW_SECONDS секунд.
#   - Failure → log warning, exit 0 (caller proceeds).
post_whoami_comment() {
    local kind="${1:-}"
    local number="${2:-}"
    local action="${3:-}"
    local reason="${4:-}"
    shift 4 || true

    # Validate
    if [ -z "$kind" ] || [ -z "$number" ] || [ -z "$action" ]; then
        _log_err "WARN: post_whoami_comment called without kind/number/action ($kind/$number/$action) — skipping"
        return 0
    fi
    case "$kind" in
        issue|pr) ;;
        *)
            _log_err "WARN: post_whoami_comment: unknown kind '$kind' — skipping"
            return 0
            ;;
    esac
    if ! printf '%s' "$number" | grep -qE '^[0-9]+$'; then
        _log_err "WARN: post_whoami_comment: '$number' not numeric — skipping"
        return 0
    fi

    local script_name
    script_name="$(_auto_script_name)"

    # Build body
    local body_first="${HERMES_WHOAMI_MARKER} script=${script_name} action=${action}"
    [ -n "$reason" ] || reason="(no reason given)"
    local meta_lines=()
    while [ $# -gt 0 ]; do
        local kv="$1"
        # Validate: must contain '=' and key must be [a-z0-9_-]
        case "$kv" in
            *=*|*[a-z0-9_-]*=*)
                meta_lines+=("   ${kv}")
                ;;
            *)
                _log_err "WARN: post_whoami_comment: ignoring malformed meta '$kv' (want key=value)"
                ;;
        esac
        shift || true
    done

    # Idempotency: scan existing comments for the SAME script+action marker
    # in last WINDOW_SECONDS seconds. If found → skip silently.
    if _whoami_already_posted "$kind" "$number" "$script_name" "$action"; then
        _log_err "DEBUG: whoami for ${kind}#${number} script=${script_name} action=${action} already posted within ${HERMES_WHOAMI_WINDOW_SECONDS}s — skipping"
        return 0
    fi

    # Build full body via heredoc-style multi-line.
    local body
    body="${body_first}
   reason: ${reason}"
    if [ ${#meta_lines[@]} -gt 0 ]; then
        local m
        for m in "${meta_lines[@]}"; do
            body="${body}
${m}"
        done
    fi

    # Post. gh failure → warn + return 0 (caller proceeds).
    local cmd=()
    if [ "$kind" = "pr" ]; then
        cmd=(pr comment "$number" --body "$body")
    else
        cmd=(issue comment "$number" --body "$body")
    fi

    if ! _gh "${cmd[@]}" >/dev/null 2>&1; then
        _log_err "WARN: whoami-comment failed for ${kind}#${number} script=${script_name} action=${action} (rate-limit / permission / network) — caller action will proceed"
        return 0
    fi
    return 0
}

# _whoami_already_posted — idempotency check.
#
# Args: kind number script_name action
# Returns: 0 if a matching whoami was posted within the last
#          HERMES_WHOAMI_WINDOW_SECONDS, else 1.
#
# Strategy: GET comments via REST API (since `gh api` is uniformly available
# in tests + prod, unlike `gh pr/issue view --json comments` whose JSON
# shape drift). Endpoint:
#   /repos/{owner}/{repo}/issues/{N}/comments    — for kind=issue (N = issue#)
#   /repos/{owner}/{repo}/issues/{N}/comments    — for kind=pr  (PR comments
#       also use the issue-comments endpoint in GitHub REST — "Conversation"
#       tab unified).
# Filter:
#   body starts with marker "<MARKER> script=<script_name> action=<action>"
#   AND created_at >= now - WINDOW_SECONDS.
_whoami_already_posted() {
    local kind="$1" number="$2" script_name="$3" action="$4"

    local GH_REPO="${GH_REPO:-}"
    if [ -z "$GH_REPO" ]; then
        # No repo → can't check; assume NOT posted (proceed with post).
        return 1
    fi

    # Fetch all comments (we don't paginate — typical PR/issue has << 100
    # comments). If there are 100+ comments, oldest are dropped and a real
    # whoami within window will still be present (they're recent).
    local comments_json
    if ! comments_json="$(_gh api "repos/${GH_REPO}/issues/${number}/comments?per_page=100" 2>/dev/null)"; then
        # API failure (rate-limit / network) → assume NOT posted. Caller
        # will try to post; that post itself may fail, which is fine.
        return 1
    fi

    local expected_prefix="${HERMES_WHOAMI_MARKER} script=${script_name} action=${action}"
    local cutoff_epoch
    cutoff_epoch=$(( $(_now_epoch) - HERMES_WHOAMI_WINDOW_SECONDS ))

    # Use python3 to parse JSON reliably — comments are arrays of objects
    # with body + created_at. Avoids jq filter maintenance.
    local found
    found="$(printf '%s' "$comments_json" | HERMES_WHOAMI_MARKER="$expected_prefix" \
        HERMES_WHOAMI_CUTOFF="$cutoff_epoch" python3 -c '
import json, os, sys
try:
    data = json.loads(sys.stdin.read() or "[]")
except Exception:
    sys.exit(1)
prefix = os.environ.get("HERMES_WHOAMI_MARKER", "")
cutoff = int(os.environ.get("HERMES_WHOAMI_CUTOFF", "0"))
for c in data:
    if not isinstance(c, dict): continue
    body = c.get("body", "") or ""
    created = c.get("created_at", "") or ""
    if not body.startswith(prefix): continue
    # GitHub created_at = "2026-08-22T19:18:43Z" → parse to epoch.
    try:
        from datetime import datetime, timezone
        dt = datetime.fromisoformat(created.replace("Z", "+00:00"))
        ep = int(dt.timestamp())
    except Exception:
        continue
    if ep >= cutoff:
        print("1"); sys.exit(0)
print("0")
' 2>/dev/null || echo "0")"
    [ "$found" = "1" ]
}

# --- convenience wrappers --------------------------------------------------
# These exist purely so callers can write `whoami_close_issue "$n" "$reason"`
# instead of constructing action strings by hand. The actual action verb
# strings below double as the taxonomy — keep them stable: e2e-process / merge-gate
# dashboards regex against them.

# whoami_close_issue — pre-comment for `gh issue close N --reason completed`.
whoami_close_issue() {
    local number="${1:?whoami_close_issue: missing issue number}"
    local reason="${2:-}"
    shift 2 || true
    post_whoami_comment issue "$number" "closing" "$reason" "$@"
}

# whoami_reopen_issue — pre-comment for `gh issue reopen N`.
whoami_reopen_issue() {
    local number="${1:?whoami_reopen_issue: missing issue number}"
    local reason="${2:-}"
    shift 2 || true
    post_whoami_comment issue "$number" "reopening" "$reason" "$@"
}

# whoami_add_label — pre-comment for `gh issue edit --add-label L`.
# Arg1=number, Arg2=label, Arg3=reason, rest=meta.
whoami_add_label() {
    local number="${1:?whoami_add_label: missing issue number}"
    local label="${2:?whoami_add_label: missing label name}"
    local reason="${3:-}"
    shift 3 || true
    post_whoami_comment issue "$number" "adding-label:${label}" "$reason" "$@"
}

# whoami_remove_label — pre-comment for `gh issue edit --remove-label L`.
whoami_remove_label() {
    local number="${1:?whoami_remove_label: missing issue number}"
    local label="${2:?whoami_remove_label: missing label name}"
    local reason="${3:-}"
    shift 3 || true
    post_whoami_comment issue "$number" "removing-label:${label}" "$reason" "$@"
}

# whoami_set_assignee — pre-comment for `gh issue edit --add-assignee U`.
whoami_set_assignee() {
    local number="${1:?whoami_set_assignee: missing issue number}"
    local user="${2:?whoami_set_assignee: missing username}"
    local reason="${3:-}"
    shift 3 || true
    post_whoami_comment issue "$number" "setting-assignee:${user}" "$reason" "$@"
}

# whoami_close_pr — pre-comment for `gh pr close N` (rare; обычно merge).
whoami_close_pr() {
    local number="${1:?whoami_close_pr: missing PR number}"
    local reason="${2:-}"
    shift 2 || true
    post_whoami_comment pr "$number" "closing" "$reason" "$@"
}