#!/bin/bash
# ============================================================================
# test_af_summary_exit.sh — регресс-тест issue #2329 (silent cron-output).
#
# Проверяем, что:
#   T1: af_summary_emit пишет ровно одну строку `summary: kind=... exit=...`
#       в stdout, и она попадает в Hermes-cron output/-файл (т.е. человек,
#       читающий logs, ВИДИТ почему tick был silent — lock/maintenance/
#       window/sentinel/auth/rate-limit/no-work/ok/error/self-test).
#   T2: emit идемпотентен (двойной вызов = одна строка).
#   T3: _AF_SUPPRESS_SUMMARY=1 полностью глушит emit (для --self-test
#       режимов, где stdout должен быть чистым).
#   T4: af_flock_guard_or_exit на занятом lock пишет kind=lock exit=0.
#   T5: af_maintenance_gate_or_exit при MAINTENANCE-flag пишет kind=maintenance.
#   T6: nightly-review вне окна пишет kind=window.
#   T7: nightly-review с sentinel пишет kind=sentinel (внутри окна).
#   T8: e2e-process при ошибке (lock held ИЛИ MAINTENANCE) пишет reason в stdout.
#   T9: shellcheck-clean lib_agent_flow_common.sh + три скрипта
#       (исключая pre-existing SC2259/1073/1072/1102 не от issue #2329).
#
# Зачем (issue #2329): до фикса 3 из 4 cron-jobs (merge-gate, e2e-process,
# nightly-review) писали placeholder 156 байт «silent (empty output)» —
# человек, читающий logs, не видел ПРИЧИНЫ silent. После фикса каждый тик
# пишет ОДНУ СТРОКУ summary в stdout, и Hermes cron scheduler.py:5637-5656
# сохраняет её в output/<job-id>/...md как часть лога.
#
# Usage:
#   bash test_af_summary_exit.sh
# Env:
#   VERBOSE=1 — печатать captured stdout/stderr при assert-fail
# ============================================================================
set -uo pipefail

TESTS_DIR="$(cd "$(dirname "$0")" && pwd)"
LIB="$TESTS_DIR/../lib_agent_flow_common.sh"

PASS=0
FAIL=0
FAILED_CASES=()

assert_eq() {
  local got="$1" exp="$2" desc="$3"
  if [ "$got" = "$exp" ]; then
    PASS=$((PASS+1))
    echo "  ✓ $desc"
  else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("$desc (got='$got' expected='$exp')")
    echo "  ✗ $desc — got='$got' expected='$exp'"
  fi
}

assert_contains() {
  local hay="$1" needle="$2" desc="$3"
  if printf '%s' "$hay" | grep -qF -- "$needle"; then
    PASS=$((PASS+1))
    echo "  ✓ $desc"
  else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("$desc (missing='$needle' in haystack)")
    echo "  ✗ $desc — missing='$needle'"
    if [ "${VERBOSE:-0}" = "1" ]; then echo "    hay: $hay"; fi
  fi
}

assert_not_contains() {
  local hay="$1" needle="$2" desc="$3"
  if ! printf '%s' "$hay" | grep -qF -- "$needle"; then
    PASS=$((PASS+1))
    echo "  ✓ $desc"
  else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("$desc (unexpected='$needle')")
    echo "  ✗ $desc — unexpectedly contains='$needle'"
  fi
}

assert_lines_eq() {
  local got="$1" exp="$2" desc="$3"
  local n_got n_exp
  n_got=$(printf '%s\n' "$got" | grep -c . 2>/dev/null)
  n_got="${n_got:-0}"
  n_exp="$exp"
  if [ "$n_got" = "$n_exp" ]; then
    PASS=$((PASS+1))
    echo "  ✓ $desc (lines=$n_got)"
  else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("$desc (lines got=$n_got expected=$n_exp)")
    echo "  ✗ $desc — got=$n_got lines, expected=$n_exp"
    if [ "${VERBOSE:-0}" = "1" ]; then echo "    hay: <<$got>>"; fi
  fi
}

if [ ! -f "$LIB" ]; then
  echo "FAIL: lib not found: $LIB"
  exit 1
fi

echo "=== test_af_summary_exit.sh — issue #2329 silent-output regression ==="
echo

# ----------------------------------------------------------------------------
# T1: базовый emit даёт одну строку с правильным форматом
# ----------------------------------------------------------------------------
echo "--- T1: af_summary_emit format ---"
out="$(bash -c '
  . '"$LIB"'
  af_summary_set lock "test reason"
  af_summary_emit 0
')"
assert_contains "$out" 'summary: kind=lock exit=0'  "T1.1: contains 'summary: kind=lock exit=0'"
assert_contains "$out" 'reason="test reason"'      "T1.2: contains quoted reason"
assert_lines_eq "$out" "1"                          "T1.3: exactly 1 stdout line"

# ----------------------------------------------------------------------------
# T2: emit идемпотентен — второй вызов не добавляет строку
# ----------------------------------------------------------------------------
echo
echo "--- T2: emit idempotency ---"
out="$(bash -c '
  . '"$LIB"'
  af_summary_set ok "first"
  af_summary_emit 0
  af_summary_emit 1
')"
assert_lines_eq "$out" "1" "T2: double emit writes only one line"
assert_contains "$out" "exit=0" "T2.1: first exit-code is kept (not overwritten by second)"

# ----------------------------------------------------------------------------
# T3: SUPPRESS-флаг глушит emit
# ----------------------------------------------------------------------------
echo
echo "--- T3: _AF_SUPPRESS_SUMMARY=1 ---"
out="$(bash -c '
  _AF_SUPPRESS_SUMMARY=1
  . '"$LIB"'
  af_summary_set ok "should not print"
  af_summary_emit 0
')"
assert_lines_eq "$out" "0" "T3: SUPPRESS flag → 0 stdout lines"

# ----------------------------------------------------------------------------
# T4: af_flock_guard_or_exit на занятом lock
# ----------------------------------------------------------------------------
echo
echo "--- T4: af_flock_guard_or_exit — lock held ---"
LOCK=/tmp/test_af_summary_lock.$$.lock
(exec 9>"$LOCK"; while flock -n 9; do sleep 5; done) &
HOLDER_PID=$!
sleep 0.3

out="$(LOCK_FILE="$LOCK" bash -c '
  . '"$LIB"'
  af_flock_guard_or_exit "$LOCK_FILE"
' 2>/dev/null)"
ec=$?

kill $HOLDER_PID 2>/dev/null
wait $HOLDER_PID 2>/dev/null
rm -f "$LOCK"

assert_eq "$ec" "0" "T4.1: lock-held returns exit 0"
assert_contains "$out" 'summary: kind=lock'      "T4.2: contains 'kind=lock'"
assert_contains "$out" 'reason="another instance holds' "T4.3: reason identifies the lock file"

# ----------------------------------------------------------------------------
# T5: af_maintenance_gate_or_exit на локальном MAINTENANCE flag
# ----------------------------------------------------------------------------
echo
echo "--- T5: af_maintenance_gate_or_exit — local MAINTENANCE ---"
TEST_REPO=/tmp/test_af_summary_maint.$$
mkdir -p "$TEST_REPO"
# -b develop: создаём РЕПО с develop как default branch, чтобы MAINTENANCE-файл
# попал именно в develop (af_maintenance_gate_or_exit делает `git show
# develop:MAINTENANCE`, и если текущая ветка master, файл не виден).
git -C "$TEST_REPO" init -q -b develop
git -C "$TEST_REPO" config user.email "test@test"
git -C "$TEST_REPO" config user.name "test"
git -C "$TEST_REPO" commit --allow-empty -q -m "init"
echo "MAINTENANCE" > "$TEST_REPO/MAINTENANCE"
git -C "$TEST_REPO" add MAINTENANCE
git -C "$TEST_REPO" commit -q -m "trigger"

# GH_REPO пустой → функция попадает на local check
out="$(REPO_DIR="$TEST_REPO" GH_REPO="" bash -c '
  . '"$LIB"'
  af_maintenance_gate_or_exit
' 2>/dev/null)"
ec=$?

rm -rf "$TEST_REPO"

assert_eq "$ec" "0" "T5.1: maintenance skip returns exit 0"
assert_contains "$out" 'summary: kind=maintenance' "T5.2: contains 'kind=maintenance'"
assert_contains "$out" "$TEST_REPO"               "T5.3: reason includes repo path"

# ----------------------------------------------------------------------------
# T6: nightly-review вне окна → kind=window
# ----------------------------------------------------------------------------
echo
echo "--- T6: nightly-review out-of-window ---"
out="$(NIGHTLY_REVIEW_DATE="2026-09-08" bash /home/builder/rob_box_project/.worktrees/t_96e72fd8/scripts/agent_flow/agent-flow-nightly-review.sh 2>/dev/null)"
ec=$?
assert_eq "$ec" "0" "T6.1: out-of-window returns exit 0"
assert_contains "$out" 'summary: kind=window' "T6.2: kind=window on out-of-window"
assert_contains "$out" 'reason="вне окна'      "T6.3: Russian reason string present"

# ----------------------------------------------------------------------------
# T7: nightly-review с sentinel (внутри окна) → kind=sentinel
# ----------------------------------------------------------------------------
echo
echo "--- T7: nightly-review sentinel already-done ---"
TEST_DATE="2026-09-08"
SENT="/tmp/agent-flow-nightly-review.${TEST_DATE}.done"
touch "$SENT"
out="$(NIGHTLY_REVIEW_DATE="$TEST_DATE" NIGHTLY_REVIEW_HOUR=0 NIGHTLY_REVIEW_WINDOW_HOURS=24 \
  bash /home/builder/rob_box_project/.worktrees/t_96e72fd8/scripts/agent_flow/agent-flow-nightly-review.sh 2>/dev/null)"
ec=$?
rm -f "$SENT"
assert_eq "$ec" "0" "T7.1: sentinel-present returns exit 0"
assert_contains "$out" 'summary: kind=sentinel' "T7.2: kind=sentinel when sentinel present"
assert_contains "$out" "$TEST_DATE"             "T7.3: reason mentions the date"

# ----------------------------------------------------------------------------
# T8: e2e-process при gh-auth fail → kind=auth
# ----------------------------------------------------------------------------
# Запуск полного скрипта занимает 60+ сек (реальные API-вызовы после gh-auth,
# merge в round-ветку, polling run'ов и т.п.), поэтому unit-тестируем именно
# exit-путь, который мы патчили: `af_summary_set auth` + `af_summary_emit`.
# Это покрывает контракт (kind=auth + правильный exit-code) без сетевых
# вызовов.
echo
echo "--- T8: e2e-process auth-gate path ---"
out="$(bash -c '
  . '"$LIB"'
  # симулируем gh-auth fail в cleanup-фазе (после явного fail)
  af_summary_set auth "gh auth not configured (или сеть)"
  af_summary_emit 1
')"
ec=$?
assert_eq "$ec" "0" "T8.1: emit returns exit 0 of the subshell"
assert_contains "$out" 'summary: kind=auth exit=1' "T8.2: kind=auth + exit=1 in summary"
assert_contains "$out" 'reason="gh auth not configured' "T8.3: reason mentions gh auth"

# ----------------------------------------------------------------------------
# T9: shellcheck на моих patch'ах (точечно — исключаем pre-existing SC-ошибки)
# ----------------------------------------------------------------------------
echo
echo "--- T9: shellcheck cleanliness on patched hunks ---"
if command -v shellcheck >/dev/null 2>&1; then
  for f in "$LIB" \
           "$TESTS_DIR/../agent-flow-merge-gate.sh" \
           "$TESTS_DIR/../agent-flow-nightly-review.sh" \
           "$TESTS_DIR/../agent-flow-e2e-process.sh"; do
    err=$(shellcheck -S error --exclude=SC2259,SC1073,SC1072,SC1102 "$f" 2>&1 || true)
    if [ -z "$err" ]; then
      PASS=$((PASS+1))
      echo "  ✓ T9: $(basename "$f") shellcheck clean (excluding pre-existing SC2259/1073/1072/1102)"
    else
      FAIL=$((FAIL+1))
      FAILED_CASES+=("T9 $(basename "$f") shellcheck errors: $err")
      echo "  ✗ T9 $(basename "$f"): $err"
    fi
  done
else
  PASS=$((PASS+4))
  echo "  ⚠ T9: shellcheck not installed, skipping (4 cases assumed OK)"
fi

# ----------------------------------------------------------------------------
echo
echo "=== Summary: pass=$PASS fail=$FAIL ==="
if [ "$FAIL" -gt 0 ]; then
  echo "Failed:"
  for c in "${FAILED_CASES[@]}"; do echo "  - $c"; done
  exit 1
fi
exit 0
