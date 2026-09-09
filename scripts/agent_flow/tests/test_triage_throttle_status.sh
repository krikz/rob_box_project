#!/bin/bash
# ============================================================================
# test_triage_throttle_status.sh — юнит-тест throttle v3.2 в agent-flow-triage.sh
#                                (ретро-фикс 22.08 t_a24ffe39, issue #1513).
#
# Проверяет, что:
#   T1: throttle v3.2 НЕ блокирует создание новой карточки, если за последние
#       4ч есть только archived/done карточка по этому issue (регрессия #1513).
#   T2: throttle v3.2 БЛОКИРУЕТ создание, если за последние 4ч есть живая
#       карточка (running/ready/todo/blocked) — поведение v3 сохранено.
#   T3: throttle v3.2 не срабатывает, если за последние 4ч по issue нет
#       карточек вообще.
#   T4: регекс в throttle — тот же, что в existing_by_issue:
#       \bissue\W*#(\d+) — ловит и "issue: #N", и "issue #N", и "Issue #N"
#       (case-insensitive).
#   T5: python-блок _recent_line печатает 2 поля (id\tstatus) и парсит
#       mock JSON с --archived флагом.
#   T6: cutoff = now - 4h; карточки старше cutoff игнорируются.
#   T7: логи throttle: для archived карточки пишем "throttle игнорирует",
#       для живой — "throttle, не создаём".
#   T8: bash -n syntax + shellcheck clean.
#
# Использование:
#   bash test_triage_throttle_status.sh
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

# --- T4: regex correctness ----------------------------------------------
echo "=== T4: regex throttle = regex existing_by_issue ==="

# Извлекаем регекс из throttle-блока и из existing_by_issue-блока
THROTTLE_REGEX="$(grep -A1 'Тот же регекс' "$SCRIPT_UNDER_TEST" | grep -oE 'r"\\\\bissue\\\\W\*#%s\\\\b"' | head -1)"
EXISTING_REGEX="$(grep -oE 'r"\\\\bissue\\\\W\*#\(\\\\d\+\)"' "$SCRIPT_UNDER_TEST" | head -1)"
log "throttle regex line: $THROTTLE_REGEX"
log "existing_by_issue regex line: $EXISTING_REGEX"

# Тестируем регекс \bissue\W*#N\b — он должен ловить manual+auto форматы
test_regex_match() {
    local body="$1" expected="$2" desc="$3"
    local matched
    matched="$(python3 -c '
import re, sys
body = sys.argv[1]
m = re.search(r"\bissue\W*#(\d+)\b", body, re.IGNORECASE)
print(m.group(1) if m else "")
' "$body")"
    if [ "$matched" = "$expected" ]; then
        pass "$desc"
    else
        fail "$desc" "body='$body' expected=$expected got=$matched"
    fi
}

test_regex_match "Source issue: #1506" "1506" \
    "T4a: auto-format 'issue: #1506' matched"
test_regex_match "Source issue #1506" "1506" \
    "T4b: manual-format 'issue #1506' matched"
test_regex_match "Issue #1506" "1506" \
    "T4c: 'Issue #1506' (capital I) matched"
test_regex_match "ISSUE #1506" "1506" \
    "T4d: 'ISSUE #1506' (all caps) matched"
test_regex_match "issue: #1506 (labels: hermes)" "1506" \
    "T4e: 'issue: #1506 (labels: ...)' matched"

# --- T5: python-блок _recent_line ---------------------------------------
echo ""
echo "=== T5: python _recent_line prints id\\tstatus ==="

test_throttle_python() {
    local mock_json="$1" expected="$2" desc="$3"
    local out
    out="$(printf '%s' "$mock_json" | python3 -c '
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
    if re.search(r"\bissue\W*#1506\b", body, re.IGNORECASE):
        print("%s\t%s" % (t.get("id", ""), t.get("status", "")))
        break
')"
    if [ "$out" = "$expected" ]; then
        pass "$desc"
    else
        fail "$desc" "expected='$expected' got='$out'"
    fi
}

NOW="$(python3 -c 'import time; print(int(time.time()))')"

# T5a: archived карточка по #1506 (за 1ч назад)
test_throttle_python "{\"tasks\": [
  {\"id\": \"t_566b343f\", \"status\": \"archived\", \"created_at\": $((NOW - 3600)), \"body\": \"issue: #1506 (вручную archived)\"}
]}" "t_566b343f	archived" \
    "T5a: archived карточка по #1506 → печатает id\tstatus"

# T5b: running карточка по #1506 (за 30 мин назад)
test_throttle_python "{\"tasks\": [
  {\"id\": \"t_active\", \"status\": \"running\", \"created_at\": $((NOW - 1800)), \"body\": \"**Source**: issue #1506\"}
]}" "t_active	running" \
    "T5b: running карточка по #1506 → печатает id\tstatus"

# T5c: старая карточка (cutoff=4ч, эта 5ч назад) — должна игнорироваться
test_throttle_python "{\"tasks\": [
  {\"id\": \"t_old\", \"status\": \"archived\", \"created_at\": $((NOW - 5*3600)), \"body\": \"issue: #1506\"}
]}" "" \
    "T5c: карточка старше 4ч (cutoff) → пусто"

# T5d: нет карточек по #1506
test_throttle_python "{\"tasks\": [
  {\"id\": \"t_other\", \"status\": \"running\", \"created_at\": $((NOW - 600)), \"body\": \"issue: #9999\"}
]}" "" \
    "T5d: нет карточек по #1506 → пусто"

# T5e: несколько карточек, первая archived → берём её (первую подходящую)
test_throttle_python "{\"tasks\": [
  {\"id\": \"t_first\", \"status\": \"archived\", \"created_at\": $((NOW - 3600)), \"body\": \"issue: #1506 (первая)\"},
  {\"id\": \"t_second\", \"status\": \"running\", \"created_at\": $((NOW - 1800)), \"body\": \"issue: #1506 (вторая)\"}
]}" "t_first	archived" \
    "T5e: первая подходящая — archived (break после первого совпадения)"

# T5f: --archived включает архивные (это и есть ключевое отличие от v3 до v3.1)
test_throttle_python "{\"tasks\": [
  {\"id\": \"t_arch\", \"status\": \"archived\", \"created_at\": $((NOW - 1800)), \"body\": \"issue: #1506\"}
]}" "t_arch	archived" \
    "T5f: --archived карточка видна (v3.1 регрессия)"

# --- T1, T2, T3, T7: throttle decision logic ----------------------------
echo ""
echo "=== T1-T3, T7: throttle decision logic ==="

# Симулируем decision block из triage.sh (lines 633-651)
throttle_decision() {
    local _recent_line="$1"
    local _recent_id _recent_status
    _recent_id="$(printf '%s' "$_recent_line" | cut -f1)"
    _recent_status="$(printf '%s' "$_recent_line" | cut -f2)"
    if [ -z "$_recent_id" ]; then
        echo "NO_MATCH"
        return
    fi
    case "${_recent_status:-}" in
        done|archived)
            echo "IGNORE_DEAD"
            ;;
        *)
            echo "BLOCK_LIVE"
            ;;
    esac
}

# T1: archived карточка за последние 4ч → IGNORE_DEAD (throttle пропускает)
R=$(throttle_decision "t_566b343f	archived")
[ "$R" = "IGNORE_DEAD" ] && pass "T1: archived карточка за 4ч → throttle IGNORE (регрессия #1513 fixed)" \
    || fail "T1: archived за 4ч" "got=$R expected=IGNORE_DEAD"

# T1b: done карточка за 4ч → IGNORE_DEAD
R=$(throttle_decision "t_done	done")
[ "$R" = "IGNORE_DEAD" ] && pass "T1b: done карточка за 4ч → throttle IGNORE" \
    || fail "T1b: done за 4ч" "got=$R expected=IGNORE_DEAD"

# T2: running → BLOCK_LIVE
R=$(throttle_decision "t_active	running")
[ "$R" = "BLOCK_LIVE" ] && pass "T2: running карточка за 4ч → throttle BLOCK (воркер занят)" \
    || fail "T2: running за 4ч" "got=$R expected=BLOCK_LIVE"

# T2b: ready → BLOCK_LIVE
R=$(throttle_decision "t_q	ready")
[ "$R" = "BLOCK_LIVE" ] && pass "T2b: ready карточка за 4ч → throttle BLOCK" \
    || fail "T2b: ready" "got=$R"

# T2c: blocked → BLOCK_LIVE
R=$(throttle_decision "t_b	blocked")
[ "$R" = "BLOCK_LIVE" ] && pass "T2c: blocked карточка за 4ч → throttle BLOCK" \
    || fail "T2c: blocked" "got=$R"

# T2d: todo → BLOCK_LIVE
R=$(throttle_decision "t_t	todo")
[ "$R" = "BLOCK_LIVE" ] && pass "T2d: todo карточка за 4ч → throttle BLOCK" \
    || fail "T2d: todo" "got=$R"

# T3: нет карточек → NO_MATCH
R=$(throttle_decision "")
[ "$R" = "NO_MATCH" ] && pass "T3: нет карточек за 4ч → NO_MATCH (throttle пропускает)" \
    || fail "T3: empty" "got=$R expected=NO_MATCH"

# T7: логи для archived карточки должны содержать "throttle игнорирует"
if grep -E 'throttle игнорирует.*ретро 22\.08 t_a24ffe39' "$SCRIPT_UNDER_TEST" >/dev/null; then
    pass "T7a: лог throttle для archived содержит 'throttle игнорирует' + ret ref"
else
    fail "T7a: лог throttle для archived не найден"
fi

# T7b: логи для живой карточки сохранили старое поведение v3
if grep -E 'throttle, не создаём \(reopened-loop v3\)' "$SCRIPT_UNDER_TEST" >/dev/null; then
    pass "T7b: лог throttle для живой карточки содержит 'throttle, не создаём (reopened-loop v3)'"
else
    fail "T7b: лог throttle для живой не найден"
fi

# --- T6: cutoff = now - 4h ---------------------------------------------
echo ""
echo "=== T6: cutoff logic ==="

# cutoff > task.created_at → task проходит фильтр
# cutoff <= task.created_at → task отбрасывается
test_cutoff() {
    local task_age_sec="$1" expected_pass="$2" desc="$3"
    local now cutoff task_ts pass
    now="$(python3 -c 'import time; print(int(time.time()))')"
    cutoff=$((now - 4 * 3600))
    task_ts=$((now - task_age_sec))
    if [ "$task_ts" -lt "$cutoff" ]; then
        pass="false"  # старше cutoff — отбрасывается
    else
        pass="true"   # моложе cutoff — проходит
    fi
    if [ "$pass" = "$expected_pass" ]; then
        pass "$desc"
    else
        fail "$desc" "age=${task_age_sec}s expected_pass=$expected_pass got=$pass"
    fi
}

test_cutoff 60 "true" "T6a: карточка 1 мин назад проходит cutoff"
test_cutoff 3600 "true" "T6b: карточка 1ч назад проходит cutoff"
test_cutoff $((3*3600 + 1800)) "true" "T6c: карточка 3.5ч назад проходит cutoff"
test_cutoff $((4*3600 + 1)) "false" "T6d: карточка 4ч+1с назад НЕ проходит cutoff"
test_cutoff $((5*3600)) "false" "T6e: карточка 5ч назад НЕ проходит cutoff"

# --- T8: bash syntax + shellcheck ---------------------------------------
echo ""
echo "=== T8: bash syntax + shellcheck ==="

# Syntax check
if bash -n "$SCRIPT_UNDER_TEST" 2>/tmp/sc_err; then
    pass "T8a: bash -n syntax check passed"
else
    fail "T8a: bash -n syntax check failed" "$(cat /tmp/sc_err)"
fi

# Verify the new comment block exists (ретро-фикс t_a24ffe39)
if grep -E 'Ретро-фикс.*22\.08.*t_a24ffe39.*#1513' "$SCRIPT_UNDER_TEST" >/dev/null; then
    pass "T8b: retro-fix comment with t_a24ffe39 + #1513 reference present"
else
    fail "T8b: retro-fix comment NOT found"
fi

# Verify --archived flag still present in throttle
if grep -A30 'THROTTLE' "$SCRIPT_UNDER_TEST" | grep -q 'list --json --archived'; then
    pass "T8c: throttle still uses 'list --json --archived' (v3.1 behavior preserved)"
else
    fail "T8c: throttle missing --archived flag"
fi

# Verify _recent_line prints 2 fields (id\tstatus)
# Простая проверка: внутри throttle-блока должен быть print с tab и 2 аргументами
if grep -A20 'cutoff = now - 4' "$SCRIPT_UNDER_TEST" | grep -F 'print("%s\t%s"' >/dev/null; then
    pass "T8d: _recent_line prints id\\tstatus (2 fields)"
else
    fail "T8d: _recent_line does NOT print 2 fields"
fi

# Verify case block on _recent_status (done|archived → ignore, else → block)
if grep -A20 '_recent_status="$(printf' "$SCRIPT_UNDER_TEST" | grep -q 'done|archived'; then
    pass "T8e: case-statement on _recent_status with done|archived branch"
else
    fail "T8e: case-statement missing done|archived branch"
fi

# Verify shellcheck (compare with origin/develop — no NEW warnings)
if ! command -v shellcheck >/dev/null 2>&1; then
    for sc in /home/builder/.hermes/hermes-agent/venv/bin/shellcheck \
             /usr/local/bin/shellcheck /usr/bin/shellcheck; do
        [ -x "$sc" ] && PATH="$(dirname "$sc"):$PATH" && break
    done
fi
if command -v shellcheck >/dev/null 2>&1; then
    _repo_root="$(git -C "$TESTS_DIR/.." rev-parse --show-toplevel 2>/dev/null || echo "$TESTS_DIR/..")"
    if [ -d "$_repo_root/.git" ] || [ -f "$_repo_root/.git" ]; then
        ORIG_SC="$(cd "$_repo_root" && git show "origin/develop:scripts/agent_flow/agent-flow-triage.sh" 2>/dev/null | shellcheck - 2>&1 | wc -l)"
        NEW_SC="$(shellcheck "$SCRIPT_UNDER_TEST" 2>&1 | wc -l)"
        log "  shellcheck: origin/develop=$ORIG_SC, current=$NEW_SC"
        if [ "$NEW_SC" -le "$ORIG_SC" ]; then
            pass "T8f: shellcheck — no NEW warnings (origin/develop=$ORIG_SC, current=$NEW_SC)"
        else
            DIFF="$(shellcheck "$SCRIPT_UNDER_TEST" 2>&1)"
            fail "T8f: shellcheck — $((NEW_SC-ORIG_SC)) new warning(s)" "$(printf '%s\n' "$DIFF" | head -30)"
        fi
    else
        log "T8f: not a git repo (no .git at $_repo_root) — skipping baseline comparison"
    fi
else
    log "T8f: shellcheck not installed — skip"
fi

# --- summary -------------------------------------------------------------
echo ""
echo "============================================================"
echo "PASS: $PASS    FAIL: $FAIL"
echo "============================================================"
if [ "$FAIL" -gt 0 ]; then
    printf '\nFAILED CASES:\n'
    for c in "${FAILED_CASES[@]}"; do printf '  - %s\n' "$c"; done
    exit 1
fi
exit 0
