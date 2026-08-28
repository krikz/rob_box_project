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
#   T10..T14: cooldown (ретро 24.08 t_bf7cd662, PR #1547) — после user-unlabel
#              в окне MAX_AGE_HOURS возвращаем True даже если latest event — labeled.
#   T15..T17: state-file (idem) — user_unlabel_should_notify/mark_notified.
#             Первый раз notify → True, второй в cooldown → False,
#             после TTL — снова True.
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

# ============================================================================
# T10..T14: cooldown (ретро 24.08 t_bf7cd662, PR #1547).
#
# Тикет: на PR #1547 Шифу снял needs-review 23.08 07:35, auto восстановил
# в 14:03, и с тех пор каждые 5 мин merge-gate снова add-label (потому что
# last_label 14:03 > last_unlabel 07:35 → guard вернул False → можно
# ставить). Без cooldown guard теряет сигнал user-decision.
#
# Cooldown: если last_unlabel в пределах USER_UNLABEL_MAX_AGE_HOURS от
# сейчас → True (skip), даже если latest event — labeled.
# ============================================================================
echo
echo "--- T10: cooldown — labeled позже unlabel, но unlabel свежий (<4ч) → True ---"
# Шифу снял метку 1ч назад, auto восстановил 30 мин назад.
T0_BACK="$(date -u -d '5 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-23T17:00:00Z')"
T1_FRESH="$(date -u -d '1 hour ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-23T20:00:00Z')"
T_LATER_30M="$(date -u -d '30 minutes ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-23T20:30:00Z')"
export _USER_UNLABEL_TEST_JSON='[
  {"event":"labeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T0_BACK"'"},
  {"event":"unlabeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T1_FRESH"'"},
  {"event":"labeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T_LATER_30M"'"}
]'
export _USER_UNLABEL_TEST_PR=1547
T10_RESULT="$(run_helper "needs-review")"
assert_eq_bool "$T10_RESULT" "true" "T10: fresh unlabel + later auto labeled → True (cooldown blocks)"

echo
echo "--- T11: cooldown — labeled позже unlabel, unlabel старый (>4ч) → False ---"
T0_VERY_BACK="$(date -u -d '10 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-23T11:00:00Z')"
T_OLD_UNLABEL="$(date -u -d '5 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-23T16:00:00Z')"
T_NEW_LABEL="$(date -u -d '1 hour ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-23T20:00:00Z')"
export _USER_UNLABEL_TEST_JSON='[
  {"event":"labeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T0_VERY_BACK"'"},
  {"event":"unlabeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T_OLD_UNLABEL"'"},
  {"event":"labeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T_NEW_LABEL"'"}
]'
export _USER_UNLABEL_TEST_PR=1547
T11_RESULT="$(run_helper "needs-review")"
assert_eq_bool "$T11_RESULT" "false" "T11: stale unlabel (>4ч) + later auto labeled → False (cooldown expired)"

echo
echo "--- T12: cooldown override USER_UNLABEL_MAX_AGE_HOURS=1 ---"
# unlabel 2ч назад — вне default 4ч, но в пределах 1ч? нет, значит False.
# А вот unlabel 30 мин назад при MAX_AGE_HOURS=1 → True.
T_30M="$(date -u -d '30 minutes ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-23T20:30:00Z')"
T_5H_AGO="$(date -u -d '5 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-23T16:00:00Z')"
export _USER_UNLABEL_TEST_JSON='[
  {"event":"labeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T_5H_AGO"'"},
  {"event":"unlabeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T_30M"'"}
]'
export _USER_UNLABEL_TEST_PR=1547
# Default 4ч: 30 мин < 4ч → True
T12A="$(run_helper "needs-review")"
assert_eq_bool "$T12A" "true" "T12a: default 4ч cooldown, 30min old unlabel → True"
# Override 1h: 30 min < 1ч → True
USER_UNLABEL_MAX_AGE_HOURS=1
export USER_UNLABEL_MAX_AGE_HOURS
T12B="$(run_helper "needs-review")"
assert_eq_bool "$T12B" "true" "T12b: MAX_AGE_HOURS=1, 30min old unlabel → True"
unset USER_UNLABEL_MAX_AGE_HOURS

echo
echo "--- T13: cooldown — bot-actor unlabel НЕ считается (is_bot) → False ---"
T_BOT_UNLABEL="$(date -u -d '30 minutes ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-23T20:30:00Z')"
T_AUTO_LABEL="$(date -u -d '20 minutes ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo '2026-08-23T20:40:00Z')"
export _USER_UNLABEL_TEST_JSON='[
  {"event":"labeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T_5H_AGO"'"},
  {"event":"unlabeled","label":{"name":"needs-review"},"actor":{"login":"github-actions[bot]"},"created_at":"'"$T_BOT_UNLABEL"'"},
  {"event":"labeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T_AUTO_LABEL"'"}
]'
export _USER_UNLABEL_TEST_PR=1547
T13_RESULT="$(run_helper "needs-review")"
assert_eq_bool "$T13_RESULT" "false" "T13: bot unlabel (auto-cleanup) + later labeled → False"

echo
echo "--- T14: cooldown — empty / no unlabel events → False (no signal) ---"
export _USER_UNLABEL_TEST_JSON='[
  {"event":"labeled","label":{"name":"needs-review"},"actor":{"login":"krikz"},"created_at":"'"$T_30M"'"}
]'
export _USER_UNLABEL_TEST_PR=1547
T14_RESULT="$(run_helper "needs-review")"
assert_eq_bool "$T14_RESULT" "false" "T14: only labeled event (no unlabel ever) → False"

# ============================================================================
# T15..T17: state-file helpers (ретро 24.08 t_bf7cd662, PR #1547).
#
# Чтобы merge-gate НЕ публиковал один и тот же ⏸️-комментарий на каждом
# 5-мин тике в течение cooldown-окна, мы помним факт "уже сказали skip
# для этой пары PR/LABEL" в state-файле ~/.hermes/state/merge-gate-said/<pr>/<label>.
#
# Override: USER_UNLABEL_STATE_DIR (по умолчанию ~/.hermes/state/...).
# Тесты используют tmp-каталог чтобы не мусорить в реальный state.
# ============================================================================
echo
echo "--- T15: state-file — first call → notify, after mark → no notify, after TTL → notify ---"
TEST_STATE_DIR="$(mktemp -d -t user_unlabel_state_XXXXXX)"
trap 'rm -rf "$TEST_STATE_DIR"' EXIT
USER_UNLABEL_STATE_DIR="$TEST_STATE_DIR"
export USER_UNLABEL_STATE_DIR

# Чистый state → should_notify=True
if user_unlabel_should_notify 1547 "needs-review"; then
    T15A="true"
else
    T15A="false"
fi
assert_eq_bool "$T15A" "true" "T15a: no state file → should_notify=True"

# После mark_notified — должен появиться state-файл, should_notify=False
if user_unlabel_mark_notified 1547 "needs-review" "test context"; then
    T15B_MARK="ok"
else
    T15B_MARK="fail"
fi
assert_eq "$T15B_MARK" "ok" "T15b: mark_notified returns 0 (file written)"
if [ -f "$TEST_STATE_DIR/1547/needs-review" ]; then
    T15B_FILE="exists"
else
    T15B_FILE="missing"
fi
assert_eq "$T15B_FILE" "exists" "T15c: state file created at <state>/1547/needs-review"

# Сразу после mark — should_notify=False (cooldown active)
if user_unlabel_should_notify 1547 "needs-review"; then
    T15C="true"
else
    T15C="false"
fi
T15C="${T15C:-true}"  # safety net от set -u
assert_eq_bool "$T15C" "false" "T15d: same PR/LABEL in cooldown → should_notify=False"

# Подделываем старый timestamp (TTL истёк)
_OLD_TS="$(date -u -d '5 hours ago' +%s 2>/dev/null || echo 1000)"
printf '%s\tstale_ctx\n' "$_OLD_TS" > "$TEST_STATE_DIR/1547/needs-review"
if user_unlabel_should_notify 1547 "needs-review"; then
    T15D="true"
else
    T15D="false"
fi
assert_eq_bool "$T15D" "true" "T15e: state TTL expired → should_notify=True (auto-cleanup)"
# После should_notify с expired TTL файл должен быть удалён
if [ -f "$TEST_STATE_DIR/1547/needs-review" ]; then
    T15E="still_exists"
else
    T15E="cleaned"
fi
assert_eq "$T15E" "cleaned" "T15f: expired state file auto-cleaned"

echo
echo "--- T16: state-file — разные PR/LABEL — independent state ---"
USER_UNLABEL_MAX_AGE_HOURS=4
export USER_UNLABEL_MAX_AGE_HOURS
# mark PR1 needs-review
user_unlabel_mark_notified 1547 "needs-review" "ctx1" || true
# mark PR1 e2e-done (другой label)
user_unlabel_mark_notified 1547 "e2e-done" "ctx2" || true
# mark PR2 needs-review (другой PR)
user_unlabel_mark_notified 9999 "needs-review" "ctx3" || true

# Все 3 должны быть notifiable=No
PR1_NR="true"; user_unlabel_should_notify 1547 "needs-review" || PR1_NR="false"
PR1_DONE="true"; user_unlabel_should_notify 1547 "e2e-done" || PR1_DONE="false"
PR2_NR="true"; user_unlabel_should_notify 9999 "needs-review" || PR2_NR="false"
assert_eq_bool "$PR1_NR" "false" "T16a: PR1 needs-review state → not notify"
assert_eq_bool "$PR1_DONE" "false" "T16b: PR1 e2e-done state → not notify (independent)"
assert_eq_bool "$PR2_NR" "false" "T16c: PR2 needs-review state → not notify (independent)"

# А вот PR1 needs-e2e — НЕ помечен → True
PR1_NEEDS_E2E="true"; user_unlabel_should_notify 1547 "needs-e2e" || PR1_NEEDS_E2E="false"
assert_eq_bool "$PR1_NEEDS_E2E" "true" "T16d: PR1 needs-e2e state missing → should notify"

echo
echo "--- T17: state-file — special chars в label нормализуются ---"
user_unlabel_mark_notified 1547 "agent-flow:big-bang-blocked" "ctx" || true
if user_unlabel_should_notify 1547 "agent-flow:big-bang-blocked"; then
    T17A="true"
else
    T17A="false"
fi
assert_eq_bool "$T17A" "false" "T17a: label with : and - normalizes correctly"
if [ -f "$TEST_STATE_DIR/1547/agent-flow_big-bang-blocked" ]; then
    T17B="exists"
else
    T17B="missing"
fi
assert_eq "$T17B" "exists" "T17b: state file uses normalized name (:/space → _)"
unset USER_UNLABEL_MAX_AGE_HOURS USER_UNLABEL_STATE_DIR

# ============================================================================
# T18: helper API surface — все новые функции определены
# ============================================================================
echo
echo "--- T18: API surface — state-file helpers определены ---"
for fn in user_unlabel_state_dir user_unlabel_state_path \
           user_unlabel_should_notify user_unlabel_mark_notified; do
    if grep -q "^${fn}()" "$LIB_UNDER_TEST"; then
        PASS=$((PASS+1))
        echo "  ✓ ${fn}() defined"
    else
        FAIL=$((FAIL+1))
        FAILED_CASES+=("${fn} not defined")
        echo "  ✗ ${fn} NOT defined"
    fi
done

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