#!/bin/bash
# ============================================================================
# test_triage_reopened_oneshot.sh — юнит-тест одноразового is_reopened
#                                    в agent-flow-triage.sh
#                                    (ретро-фикс 22.08 #1506 reopened-loop).
#
# Проверяет, что:
#   T1: в triage.sh есть jq-фильтр `_last_marker_ts`, берущий ПОСЛЕДНИЙ
#       комментарий-маркер `kanban: t_<id>` (// empty — нет маркера → пусто).
#   T2: есть условие one-shot: is_reopened=true только если маркера НЕТ или
#       маркер СТАРШЕ последнего reopen; иначе reopen «потреблён».
#   T3: есть else-лог «потреблён маркером» (обычная идемпотентность).
#   T4: симуляция решения (python) — no-marker → REOPENED, marker<reopen →
#       REOPENED, marker>reopen → CONSUMED, marker==reopen → CONSUMED.
#   T5: bash -n syntax check triage.sh.
#
# Использование:
#   bash test_triage_reopened_oneshot.sh
# Env:
#   VERBOSE=1 — печатать подробности
# ============================================================================
set -uo pipefail

TESTS_DIR="$(cd "$(dirname "$0")" && pwd)"
SCRIPT_UNDER_TEST="$TESTS_DIR/../agent-flow-triage.sh"

PASS=0
FAIL=0
FAILED_CASES=()

# --- harness -------------------------------------------------------------
log() { if [ "${VERBOSE:-0}" = "1" ]; then printf '  %s\n' "$*"; fi; }
pass() { PASS=$((PASS+1)); printf '  \033[32m✓\033[0m %s\n' "$1"; }
fail() {
    FAIL=$((FAIL+1)); FAILED_CASES+=("$1")
    printf '  \033[31m✗\033[0m %s\n' "$1"
    if [ -n "${2:-}" ]; then printf '      %s\n' "$2"; fi
}

# --- T1: jq-фильтр `_last_marker_ts` присутствует -----------------------
echo "=== T1: jq-фильтр _last_marker_ts ==="

if grep -F '_last_marker_ts="$(gh api "repos/${GH_REPO}/issues/${number}/comments" --paginate' \
    "$SCRIPT_UNDER_TEST" >/dev/null; then
    pass "T1a: fetch _last_marker_ts из comments (REST, --paginate)"
else
    fail "T1a: _last_marker_ts fetch не найден"
fi

if grep -F '([.[] | select((.body // "") | test("^kanban: t_[a-f0-9]+"))] | last | .created_at) // empty' \
    "$SCRIPT_UNDER_TEST" >/dev/null; then
    pass "T1b: jq-фильтр берёт ПОСЛЕДНИЙ маркер + // empty при отсутствии"
else
    fail "T1b: jq-фильтр last-marker не найден"
fi

# --- T2: условие one-shot -----------------------------------------------
echo ""
echo "=== T2: условие one-shot is_reopened ==="

if grep -F '[ -z "$_last_marker_ts" ] || [[ "$_last_marker_ts" < "$_reopen_ts" ]]' \
    "$SCRIPT_UNDER_TEST" >/dev/null; then
    pass "T2a: is_reopened=true только при отсутствии маркера или marker<reopen"
else
    fail "T2a: условие one-shot не найдено"
fi

# --- T3: else-лог «потреблён» -------------------------------------------
echo ""
echo "=== T3: else-лог ==="

if grep -F 'потреблён маркером' "$SCRIPT_UNDER_TEST" >/dev/null; then
    pass "T3a: else-лог 'потреблён маркером' присутствует"
else
    fail "T3a: else-лог не найден"
fi

# --- T4: симуляция решения (python) -------------------------------------
echo ""
echo "=== T4: симуляция one-shot решения ==="

# Мимикрирует jq-фильтр + bash-сравнение из triage.sh.
decision() {  # $1=comments_json  $2=reopen_ts
    python3 - "$1" "$2" <<'PY'
import json, re, sys
comments = json.loads(sys.argv[1])
reopen = sys.argv[2]
last = ""
for c in comments:
    body = c.get("body") or ""
    if re.match(r"^kanban: t_[a-f0-9]+", body):
        last = c.get("created_at") or ""
# bash: [ -z "$_last_marker_ts" ] || [[ "$_last_marker_ts" < "$_reopen_ts" ]]
if last == "" or last < reopen:
    print("REOPENED")
else:
    print("CONSUMED")
PY
}

# T4a: нет маркера → REOPENED
R=$(decision '[{"body":"hello","created_at":"2026-08-22T11:00:00Z"}]' "2026-08-22T10:38:13Z")
[ "$R" = "REOPENED" ] && pass "T4a: нет маркера → REOPENED" \
    || fail "T4a: нет маркера" "got=$R expected=REOPENED"

# T4b: маркер СТАРШЕ reopen → REOPENED (создаём свежую карточку)
R=$(decision '[{"body":"kanban: t_0a1b2c","created_at":"2026-08-22T10:00:00Z"}]' "2026-08-22T10:38:13Z")
[ "$R" = "REOPENED" ] && pass "T4b: маркер < reopen → REOPENED" \
    || fail "T4b: маркер < reopen" "got=$R expected=REOPENED"

# T4c: маркер НОВЕЕ reopen → CONSUMED (идемпотентность по маркеру)
R=$(decision '[{"body":"kanban: t_a1b2c3","created_at":"2026-08-22T11:20:00Z"}]' "2026-08-22T10:38:13Z")
[ "$R" = "CONSUMED" ] && pass "T4c: маркер > reopen → CONSUMED" \
    || fail "T4c: маркер > reopen" "got=$R expected=CONSUMED"

# T4d: несколько маркеров — решает ПОСЛЕДНИЙ
R=$(decision '[{"body":"kanban: t_0a1b2c","created_at":"2026-08-22T10:00:00Z"},{"body":"kanban: t_a1b2c3","created_at":"2026-08-22T11:20:00Z"}]' "2026-08-22T10:38:13Z")
[ "$R" = "CONSUMED" ] && pass "T4d: несколько маркеров → последний решает (CONSUMED)" \
    || fail "T4d: последний маркер" "got=$R expected=CONSUMED"

# T4e: маркер == reopen → CONSUMED (строгое <, равно не «старше»)
R=$(decision '[{"body":"kanban: t_e0e0e0","created_at":"2026-08-22T10:38:13Z"}]' "2026-08-22T10:38:13Z")
[ "$R" = "CONSUMED" ] && pass "T4e: маркер == reopen → CONSUMED" \
    || fail "T4e: маркер == reopen" "got=$R expected=CONSUMED"

# --- T5: bash syntax ------------------------------------------------------
echo ""
echo "=== T5: bash syntax ==="
if bash -n "$SCRIPT_UNDER_TEST" 2>/tmp/sc_err; then
    pass "T5a: bash -n syntax check passed"
else
    fail "T5a: bash -n syntax check failed" "$(cat /tmp/sc_err)"
fi

# --- summary --------------------------------------------------------------
echo ""
echo "======================================"
echo "PASS=$PASS FAIL=$FAIL"
if [ "$FAIL" -gt 0 ]; then
    printf 'FAILED: %s\n' "${FAILED_CASES[*]}"
    exit 1
fi
echo "ALL PASS"
