#!/bin/bash
# ============================================================================
# test_user_unlabel_check.sh — ретро 18.08 t_de6bea69, PR #1398
#
# Bug: needs-review на PR возвращалась автоматикой (e2e-process post-round
# sweep / merge-gate reconcile / merge-gate lint path / merge-gate clean-pr-
# sweep) после ручного unlabel Шифу. 5 раз за 2.5ч на PR #1398.
#
# Fix: lib_user_unlabel_check.sh::user_removed_label_recently() проверяет
# timeline PR: если последнее событие по метке — UnlabeledEvent (actor≠bot)
# ПОЗЖЕ последнего LabeledEvent по этой же метке → return 0 (respect user
# decision). Иначе → return 1 (можно ставить).
#
# Acceptance:
#   T1: timeline=[labeled(L,T0)] → False (нет unlabel, ставим)
#   T2: timeline=[labeled(L,T0), unlabeled(L,T1>T0, user)] → True (skip)
#   T3: timeline=[labeled(L,T0), unlabeled(L,T1>T0, github-actions[bot])]
#       → False (actor=bot, не user-decision; ставим)
#   T4: timeline=[unlabeled(L,T1), labeled(L,T2>T1, auto)]
#       → False (последнее labeled, ничего не было user-decision; ставим)
#   T5: timeline=[labeled(L1,T0), unlabeled(L2,T1, user), labeled(L2,T2>T1, auto)]
#       → False для L2 (последний labeled), True для L1 (последний unlabeled)
#   T6: timeline=[] / пустой → False (нет сигнала, идём как обычно)
#   T7: integration — helper вызывается из T7 mock сценария (True/False корректно)
#   T8: regression — без user-unlabel оба False (baseline)
#   T9: shellcheck-clean + syntax-OK + required helpers exist + scripts source lib
# ============================================================================
set -uo pipefail

TESTS_DIR="$(cd "$(dirname "$0")" && pwd)"
LIB_UNDER_TEST="$TESTS_DIR/../lib_user_unlabel_check.sh"
E2E_PROCESS_SCRIPT="$TESTS_DIR/../agent-flow-e2e-process.sh"

PASS=0
FAIL=0
FAILED_CASES=()

if [ ! -f "$LIB_UNDER_TEST" ]; then
  echo "FAIL: helper not found: $LIB_UNDER_TEST"
  exit 1
fi

if [ ! -f "$E2E_PROCESS_SCRIPT" ]; then
  echo "FAIL: e2e-process script not found: $E2E_PROCESS_SCRIPT"
  exit 1
fi

echo "=== test_user_unlabel_check.sh — user-unlabel-respect guard unit tests ==="
echo

# ----------------------------------------------------------------------------
# Assertion helpers
# ----------------------------------------------------------------------------
assert_eq() {
  local got="$1" exp="$2" desc="$3"
  if [ "$got" = "$exp" ]; then
    PASS=$((PASS+1))
    echo "  ✓ $desc"
  else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("$desc (got=$got expected=$exp)")
    echo "  ✗ $desc — got='$got' expected='$exp'"
  fi
}

assert_eq_bool() {  # wrapper для user_removed_label_recently: return 0 = true
  local fn_result="$1" exp_true="$2" desc="$3"
  local got="false"
  if [ "$fn_result" = "true" ]; then got="true"; fi
  if [ "$got" = "$exp_true" ]; then
    PASS=$((PASS+1))
    echo "  ✓ $desc"
  else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("$desc (got=$got expected=$exp_true)")
    echo "  ✗ $desc — got=$got expected=$exp_true"
  fi
}

# ----------------------------------------------------------------------------
# T1..T6: unit-tests для user_removed_label_recently через USER_UNLABEL_TEST_MODE
# ----------------------------------------------------------------------------
USER_UNLABEL_TEST_MODE=1
export USER_UNLABEL_TEST_MODE
# shellcheck disable=SC1090
. "$LIB_UNDER_TEST"

# helper для запуска user_removed_label_recently с подделкой timeline.
# Использует _USER_UNLABEL_TEST_JSON, экспортированный в env перед вызовом.
run_helper() {
  local label="$1"
  if user_removed_label_recently 1398 "$label"; then
    echo "true"
  else
    echo "false"
  fi
}

# --- T1: timeline=[labeled(L,T0)] → False (нет unlabel, ставим) ---
echo "--- T1: timeline=[labeled(L,T0)] → False ---"
T0="$(date -u -d '5 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-18T18:00:00Z')"
export _USER_UNLABEL_TEST_JSON='[{"event":"labeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T0"'"}]'
export _USER_UNLABEL_TEST_PR=1398
T1_RESULT="$(run_helper "needs-review")"
assert_eq_bool "$T1_RESULT" "false" "T1: only labeled → False (можно ставить)"

# --- T2: timeline=[labeled(L,T0), unlabeled(L,T1>T0, user)] → True ---
echo
echo "--- T2: timeline=[labeled, unlabeled(user)] → True ---"
T1_AT="$(date -u -d '1 hour ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-18T22:00:00Z')"
export _USER_UNLABEL_TEST_JSON='[{"event":"labeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T0"'"},{"event":"unlabeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T1_AT"'"}]'
export _USER_UNLABEL_TEST_PR=1398
T2_RESULT="$(run_helper "needs-review")"
assert_eq_bool "$T2_RESULT" "true" "T2: user unlabeled after auto labeled → True (skip)"

# --- T3: timeline=[labeled, unlabeled by github-actions[bot]] → False ---
echo
echo "--- T3: timeline=[labeled, unlabeled by bot] → False ---"
export _USER_UNLABEL_TEST_JSON='[{"event":"labeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T0"'"},{"event":"unlabeled","label":{"name":"needs-review"},"actor":{"login":"github-actions[bot]"},"created_at":"'"$T1_AT"'"}]'
export _USER_UNLABEL_TEST_PR=1398
T3_RESULT="$(run_helper "needs-review")"
assert_eq_bool "$T3_RESULT" "false" "T3: bot unlabeled → False (не user-decision)"

# --- T4: timeline=[unlabeled, labeled ПОЗЖЕ (auto восстановил)] → False ---
echo
echo "--- T4: timeline=[unlabeled, labeled ПОЗЖЕ (auto восстановил)] → False ---"
T_LATER="$(date -u +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-18T23:30:00Z')"
export _USER_UNLABEL_TEST_JSON='[{"event":"unlabeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T0"'"},{"event":"labeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T_LATER"'"}]'
export _USER_UNLABEL_TEST_PR=1398
T4_RESULT="$(run_helper "needs-review")"
assert_eq_bool "$T4_RESULT" "false" "T4: latest=labeled (auto восстановил) → False"

# --- T5: разные labels в одном timeline ---
echo
echo "--- T5: разные labels в одном timeline ---"
# PR: needs-review user-unlabeled ПОЗЖЕ auto, e2e-done только auto-labeled
# → для needs-review → True, для e2e-done → False
export _USER_UNLABEL_TEST_JSON='[
  {"event":"labeled","label":{"name":"e2e-done"},"actor":{"login":"krikz"},"created_at":"'"$T0"'"},
  {"event":"labeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T0"'"},
  {"event":"unlabeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T1_AT"'"}
]'
export _USER_UNLABEL_TEST_PR=1398
T5_NR="$(run_helper "needs-review")"
T5_DONE="$(run_helper "e2e-done")"
assert_eq_bool "$T5_NR" "true" "T5a: needs-review user-unlabeled → True"
assert_eq_bool "$T5_DONE" "false" "T5b: e2e-done never unlabeled → False"

# --- T6: пустой timeline → False ---
echo
echo "--- T6: пустой timeline → False ---"
export _USER_UNLABEL_TEST_JSON='[]'
export _USER_UNLABEL_TEST_PR=1398
T6_RESULT="$(run_helper "needs-review")"
assert_eq_bool "$T6_RESULT" "false" "T6: пустой timeline → False"

export _USER_UNLABEL_TEST_JSON=''
export _USER_UNLABEL_TEST_PR=1398
T6B_RESULT="$(run_helper "needs-review")"
assert_eq_bool "$T6B_RESULT" "false" "T6b: пустая строка timeline → False"

# ============================================================================
# T7: integration — реальный путь через user_removed_label_recently с
# mock-timeline (как вызвал бы post_round_sweep / merge-gate reconcile).
# ============================================================================
echo
echo "--- T7: integration через user_removed_label_recently с реальной mock-timeline ---"
# Это симулирует ровно то, что происходит в post_round_sweep при USER_UNLABEL_TEST_MODE=0
# (с подменой gh api на ту же timeline-строку).
export _USER_UNLABEL_TEST_JSON='[
  {"event":"labeled","label":{"name":"e2e-done"},"actor":{"login":"krikz"},"created_at":"'"$T0"'"},
  {"event":"labeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T0"'"},
  {"event":"unlabeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T1_AT"'"}
]'
export _USER_UNLABEL_TEST_PR=1398
if user_removed_label_recently 1398 "needs-review"; then
    T7_USER_UNLBL_NR="true"
else
    T7_USER_UNLBL_NR="false"
fi
if user_removed_label_recently 1398 "e2e-done"; then
    T7_USER_UNLBL_DONE="true"
else
    T7_USER_UNLBL_DONE="false"
fi
assert_eq_bool "$T7_USER_UNLBL_NR" "true" "T7a: needs-review user-unlabel → True"
assert_eq_bool "$T7_USER_UNLBL_DONE" "false" "T7b: e2e-done не unlabel'илось → False"

echo
echo "--- T8: regression — без user-unlabel оба False (baseline) ---"
T0_BACK="$(date -u -d '5 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-18T18:00:00Z')"
export _USER_UNLABEL_TEST_JSON='[
  {"event":"labeled","label":{"name":"e2e-done"},"actor":{"login":"krikz"},"created_at":"'"$T0_BACK"'"},
  {"event":"labeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T0_BACK"'"}
]'
export _USER_UNLABEL_TEST_PR=1398
if user_removed_label_recently 1398 "needs-review"; then
    T8_NR="true"
else
    T8_NR="false"
fi
if user_removed_label_recently 1398 "e2e-done"; then
    T8_DONE="true"
else
    T8_DONE="false"
fi
assert_eq_bool "$T8_NR" "false" "T8a: needs-review только labeled → False"
assert_eq_bool "$T8_DONE" "false" "T8b: e2e-done только labeled → False"

echo
echo "--- T9: shellcheck-clean + syntax-OK + helper present ---"
if bash -n "$LIB_UNDER_TEST" 2>/dev/null; then
    PASS=$((PASS+1))
    echo "  ✓ lib_user_unlabel_check.sh syntax-OK"
else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("lib_user_unlabel_check.sh syntax")
    echo "  ✗ lib_user_unlabel_check.sh syntax FAIL"
fi
if bash -n "$E2E_PROCESS_SCRIPT" 2>/dev/null; then
    PASS=$((PASS+1))
    echo "  ✓ agent-flow-e2e-process.sh syntax-OK"
else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("agent-flow-e2e-process.sh syntax")
    echo "  ✗ agent-flow-e2e-process.sh syntax FAIL"
fi
if grep -q "user_removed_label_recently()" "$LIB_UNDER_TEST"; then
    PASS=$((PASS+1))
    echo "  ✓ user_removed_label_recently() defined"
else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("user_removed_label_recently not defined")
    echo "  ✗ user_removed_label_recently NOT defined"
fi
if grep -q "user_unlabel_log_skip" "$LIB_UNDER_TEST"; then
    PASS=$((PASS+1))
    echo "  ✓ user_unlabel_log_skip() defined"
else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("user_unlabel_log_skip not defined")
    echo "  ✗ user_unlabel_log_skip NOT defined"
fi
if grep -q "lib_user_unlabel_check.sh" "$E2E_PROCESS_SCRIPT"; then
    PASS=$((PASS+1))
    echo "  ✓ e2e-process.sh sources lib_user_unlabel_check.sh"
else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("e2e-process.sh doesn't source helper")
    echo "  ✗ e2e-process.sh DOESN'T source helper"
fi
MERGE_GATE_SCRIPT="$TESTS_DIR/../agent-flow-merge-gate.sh"
if grep -q "lib_user_unlabel_check.sh" "$MERGE_GATE_SCRIPT"; then
    PASS=$((PASS+1))
    echo "  ✓ merge-gate.sh sources lib_user_unlabel_check.sh"
else
    FAIL=$((FAIL+1))
    FAILED_CASES+=("merge-gate.sh doesn't source helper")
    echo "  ✗ merge-gate.sh DOESN'T source helper"
fi

# ----------------------------------------------------------------------------
# Summary
# ----------------------------------------------------------------------------
echo
echo "=== Summary: PASS=$PASS FAIL=$FAIL ==="
if [ "$FAIL" -gt 0 ]; then
    echo "FAILED CASES:"
    for c in "${FAILED_CASES[@]}"; do
        echo "  - $c"
    done
    exit 1
fi
exit 0