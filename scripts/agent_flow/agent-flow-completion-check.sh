#!/bin/bash
# ============================================================================
# agent-flow-completion-check.sh — GATE-3 (ADR-0022 §4.3): CI-blocking
# completion. Блокирует archive карточки при `gh pr checks` FAILURE, чтобы
# регресс R5 (типичный случай: PR #1418, merge-gate merge'нул c красным CI,
# карточка archive'нулась как done) больше не повторялся.
#
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-completion-check.sh
# На хост раскладывает `bash <repo>/scripts/agent_flow/install.sh` —
# hardlink-копиями (cp -al), НЕ симлинками (см. заголовок install.sh).
# Полный список путей — там же; сверку копий держит agent-flow-drift-detect.sh.
#
# Usage:
#   bash agent-flow-completion-check.sh <pr_number>
#
# Exit codes:
#   0 = OK (CI all green AND PR not CONFLICTING — safe to archive card)
#   1 = FAILURE — do NOT archive; emit a one-line warning to stderr/log.
#   2 = usage error (missing/non-numeric PR).
#
# Что проверяет:
#   1) `gh pr view <pr> --json statusCheckRollup` — считает красные чек'и
#      (FAILURE / CANCELLED / TIMED_OUT / STALE). Тот же паттерн, что и
#      merge-gate.sh:2755-2756 + retro-path §4.3.
#   2) `gh pr view <pr> --json state,mergeStateStatus` — MERGED + DIRTY/
#      CONFLICTING или OPEN + CONFLICTING → fail (R5-сценарий / rebase нужен).
#   3) CI-only exemption (ADR-0022 §7.1 #12): PR затрагивает ТОЛЬКО
#      `.github/`, `scripts/agent_flow/`, `docs/` → exempt (lint/ci-only
#      фикс может легитимно содержать «FAILURE» в workflow-паттернах).
#
# Интеграция: вызывается из agent-flow-merge-gate.sh::archive_merged_card
# ПЕРЕД kanban archive. Если exit 1 → карточка остаётся done, идемпотентный
# retry следующего тика; после re-run CI с зелёным — archive проходит.
#
# ADR-0018 (honest FAIL > fake PASS): этот скрипт — enforcement.
# ============================================================================
set +e
# shellcheck source=lib_cron_env.sh
. "$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)/lib_cron_env.sh" || {
    printf "[%s] %s: lib_cron_env preflight failed — exit 1
" \
        "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$(basename "${BASH_SOURCE[0]:-$0}")" >&2
    exit 1
}
set -euo pipefail

set -euo pipefail

# --- args -------------------------------------------------------------------
pr="${1:-}"
if [ -z "$pr" ] || ! printf '%s' "$pr" | grep -qE '^[0-9]+$'; then
    echo "usage: $0 <pr_number>" >&2
    exit 2
fi

HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
KANBAN_BOARD="${KANBAN_BOARD:-robbox}"
KANBAN_DB="${KANBAN_DB:-$HOME/.hermes/kanban/boards/$KANBAN_BOARD/kanban.db}"
# Ретро 12.08: merge-gate форсит HOME=/home/builder для gh credentials.
export HOME="${HOME:-/home/builder}"

# GH_REPO: либо из env, либо из kanban DB (board config), иначе fallback.
GH_REPO="${GH_REPO:-}"
if [ -z "$GH_REPO" ] && [ -f "$KANBAN_DB" ]; then
    GH_REPO="$(python3 - "$KANBAN_DB" <<'PYEOF' 2>/dev/null || true
import sqlite3, sys, os
db = sys.argv[1]
try:
    if not os.path.exists(db): sys.exit(0)
    conn = sqlite3.connect(db)
    row = conn.execute("SELECT value FROM board_config WHERE key='GH_REPO'").fetchone()
    conn.close()
    if row: print(row[0])
except Exception:
    pass
PYEOF
)"
fi
GH_REPO="${GH_REPO:-krikz/rob_box_project}"

# self-id / whoami helper (issue #1534): перед declaring PR red (gate-block)
# этот скрипт пишет «🤖 [agent:<role>] script=… action=closing reason=…» на
# PR, чтобы в истории GitHub было видно КТО его gate-заблокировал (и, по
# цепочке, потенциально force-close'нул). helper идемпотентный (2h окно).
_LIB_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=hermes_github.sh
. "$_LIB_DIR_HERE/hermes_github.sh"

# --- 1) CI-only exemption (ADR-0022 §7.1 #12, ADR-0014 lint-bypass) ---------
# PR, затрагивающий ТОЛЬКО эти пути, exempt даже при красных чек'ах:
#   - .github/ (CI workflow правки)
#   - scripts/agent_flow/ (process scripts правки — наш собственный gate)
#   - docs/ (документация)
# Используем тот же проверенный паттерн, что и merge-gate.sh:2731-2744:
# raw JSON → python3 проверка префиксов. Никаких кастомных jq-фильтров.
_exempt_ci_only=0
_files_json="$(gh pr view "$pr" --repo "$GH_REPO" --json files \
    --jq '[.files[].path]' 2>/dev/null || echo '[]')"
if [ "$_files_json" != "[]" ] && [ -n "$_files_json" ]; then
    _ci_only="$(printf '%s' "$_files_json" | python3 -c '
import json, sys
try:
    files = json.load(sys.stdin)
    ok = bool(files) and all(
        f.startswith(".github/") or f.startswith("scripts/agent_flow/") or f.startswith("docs/")
        for f in files
    )
    print("1" if ok else "0")
except Exception:
    print("0")
' 2>/dev/null || echo 0)"
    [ "${_ci_only:-0}" = "1" ] && _exempt_ci_only=1
fi

# --- 2) gh pr checks FAILURE scan ------------------------------------------
# Считаем FAILURE/CANCELLED/TIMED_OUT/STALE через проверенный jq-фильтр
# (тот же что merge-gate.sh:2755-2756). 0 = OK, >0 = fail.
# Если gh упал (rate-limit / network) → echo 1 = FAIL (fail-safe).
_rollup="$(gh pr view "$pr" --repo "$GH_REPO" --json statusCheckRollup \
    --jq '[.statusCheckRollup[] | select(.conclusion == "FAILURE" or .conclusion == "CANCELLED" or .conclusion == "TIMED_OUT" or .conclusion == "STALE")] | length' 2>/dev/null || echo 1)"

_red_count="$(printf '%s' "$_rollup" | tr -dc '0-9' || echo 0)"
_red_count="${_red_count:-0}"

if [ "$_red_count" -gt 0 ] 2>/dev/null; then
    if [ "$_exempt_ci_only" = "1" ]; then
        echo "[completion-check] PR #${pr} has ${_red_count} red check(s) but PR is CI-only (lint/docs) — exempt (ADR-0022 §7.1 #12)" >&2
    else
        # issue #1534: self-id whoami BEFORE reporting PR red — этот gate
        # блокирует archive; следующий tick merge-gate может force-close
        # этот PR. helper идемпотентный (2h окно).
        whoami_close_pr "$pr" "completion-check: PR has ${_red_count} red check(s), NOT archiving card" "card=t_?"
        # Ровно тот лог-формат, который просит acceptance:
        #   [completion-check] card t_xxx PR #N has FAILURE check <name> — не архивирую
        printf '[completion-check] card ? PR #%s has FAILURE check — не архивирую\n' \
            "$pr" >&2
        exit 1
    fi
fi

# --- 3) PR state/mergeable sanity ------------------------------------------
# MERGED + mergeStateStatus=DIRTY/CONFLICTING → R5-сценарий (force-merge c
# красным CI). OPEN + CONFLICTING → воркер должен rebase до archive.
# Парсим raw JSON через python3 — тот же подход, что и merge-gate для
# files-проверки (стр. 2731-2744). Надёжнее, чем кастомный jq-фильтр
# (мок gh в тестах не понимает строковую интерполяцию `"\(.state) ..."`).
_view_json="$(gh pr view "$pr" --repo "$GH_REPO" --json state,mergeStateStatus \
    2>/dev/null || echo '{}')"
_state_merge="$(printf '%s' "$_view_json" | python3 -c '
import json, sys
try:
    d = json.loads(sys.stdin.read() or "{}")
    st = d.get("state", "") or ""
    ms = d.get("mergeStateStatus", "") or ""
    print(f"{st}|{ms}")
except Exception:
    print("|")
' 2>/dev/null || echo '|')"

case "$_state_merge" in
    *CONFLICTING*|*DIRTY*)
        printf '[completion-check] PR #%s mergeStateStatus=%s — resolve before archive\n' \
            "$pr" "$_state_merge" >&2
        exit 1
        ;;
esac

exit 0
