#!/bin/bash
# ============================================================================
# test_nightly_review_persistence.sh — тесты
# scripts/agent_flow/nightly-review-record.sh (ADR-0079, issue #2159).
#
# Ревью 08.09 (до мержа #2177): первая версия ADR-0079 персистила находки
# ИЗНУТРИ agent-flow-nightly-review.sh, читая NIGHTLY_REVIEW_OUTCOME в том
# же прогоне, который создаёт карточку — то есть ДО того, как кто-либо
# посмотрел на код. В проде эту переменную некому было выставить; тесты
# были зелёными только потому, что сами её и передавали. Persistence
# переехала в отдельный скрипт, который вызывает САМ ревьюер — этот файл
# тестирует его напрямую, без hermes/gh (скрипт их не вызывает).
#
# Проверяемые гарантии:
#   P1. Базовая запись: JSONL создаётся, обязательные поля на месте.
#   P2. no-real-defect: --finding не нужен, findings=[].
#   P3. open-issue-<N> без --finding → exit 2 (нечего трекать/дедупить).
#   P4. Fingerprint = sha1(type:file:line:symbol)[:12], детерминирован.
#   P5. files-changed парсится в JSON-массив.
#   P6. Append-only: два вызова → две валидные строки в одном файле.
#   P7. Дедуп: находка с тем же fingerprint, что уже трекается открытым
#       issue в существующем JSONL — WARNING в stderr, exit всё равно 0
#       (fail-open, решение за воркером).
#   P8. Невалидный --outcome → exit 2.
#   P9. Невалидный JSON в --finding → exit 2.
#
# Invocation:
#   bash scripts/agent_flow/tests/test_nightly_review_persistence.sh
# Возвращает 0 при всех pass, 1 при первом fail.
# ============================================================================
set -uo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/.." && pwd)"
RECORD="$REPO_ROOT/nightly-review-record.sh"

TEST_TMP="${TEST_TMP:-/tmp/nightly-review-record-tests.$$}"
rm -rf "$TEST_TMP"
mkdir -p "$TEST_TMP/reports"

STDOUT_FILE="$TEST_TMP/stdout"
STDERR_FILE="$TEST_TMP/stderr"

run_record() {  # $@ = аргументы скрипта
    bash "$RECORD" "$@" > "$STDOUT_FILE" 2> "$STDERR_FILE"
    echo $?
}

reset_state() {
    rm -rf "$TEST_TMP/reports"
    mkdir -p "$TEST_TMP/reports"
}

TESTS_TOTAL=0
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_NAMES=()

run_test() {
    local name="$1" fn="$2"
    TESTS_TOTAL=$((TESTS_TOTAL + 1))
    printf '[ RUN     ] %s\n' "$name"
    if "$fn"; then
        TESTS_PASSED=$((TESTS_PASSED + 1))
        printf '[   PASS  ] %s\n' "$name"
    else
        TESTS_FAILED=$((TESTS_FAILED + 1))
        FAILED_NAMES+=("$name")
        printf '[   FAIL  ] %s\n' "$name"
    fi
}

assert_eq() {
    if [ "$1" != "$2" ]; then
        printf '  assert fail: %s\n    expected: %q\n    actual:   %q\n' "$3" "$1" "$2" >&2
        return 1
    fi
}

assert_contains() {
    case "$2" in
        *"$1"*) return 0 ;;
        *) printf '  assert fail: %s\n    needle: %q\n' "$3" "$1" >&2; return 1 ;;
    esac
}

# ============================================================================
# P1. Базовая запись: JSONL создаётся, обязательные поля на месте.
# ============================================================================
test_P1_basic_record() {
    reset_state
    local rc
    rc="$(run_record --task-id t_p1 --component nightly --outcome no-real-defect \
        --review-date 2026-09-08 --reports-dir "$TEST_TMP/reports")"
    assert_eq "0" "$rc" "P1: exit 0" || return 1
    local jsonl="$TEST_TMP/reports/nightly-review/2026-09-08.jsonl"
    [ -f "$jsonl" ] || { printf '  assert fail: P1: JSONL не создан (%s)\n' "$jsonl" >&2; return 1; }
    python3 - "$jsonl" <<'PY'
import json, sys
path = sys.argv[1]
required = {"ts", "review_date", "iso_week", "task_id", "component", "files_changed", "findings", "outcome"}
with open(path) as f:
    line = f.readline()
    rec = json.loads(line)
    missing = required - set(rec.keys())
    if missing:
        print("P1: без полей %s" % missing); sys.exit(1)
    if rec["task_id"] != "t_p1" or rec["component"] != "nightly" or rec["outcome"] != "no-real-defect":
        print("P1: неверные значения полей: %s" % rec); sys.exit(1)
PY
}

# ============================================================================
# P2. no-real-defect: --finding не нужен, findings=[].
# ============================================================================
test_P2_no_real_defect_no_finding_required() {
    reset_state
    local rc
    rc="$(run_record --task-id t_p2 --component src-voice --outcome no-real-defect \
        --review-date 2026-09-08 --reports-dir "$TEST_TMP/reports")"
    assert_eq "0" "$rc" "P2: exit 0 без --finding" || return 1
    local jsonl="$TEST_TMP/reports/nightly-review/2026-09-08.jsonl"
    local line
    line="$(head -1 "$jsonl")"
    assert_contains '"findings": []' "$line" "P2: findings пуст" || return 1
}

# ============================================================================
# P3. open-issue-<N> без --finding → exit 2 (нечего трекать/дедупить).
# ============================================================================
test_P3_open_issue_requires_finding() {
    reset_state
    local rc
    rc="$(run_record --task-id t_p3 --component nightly --outcome open-issue-42 \
        --review-date 2026-09-08 --reports-dir "$TEST_TMP/reports")"
    assert_eq "2" "$rc" "P3: exit 2 без --finding" || return 1
    assert_contains "requires" "$(cat "$STDERR_FILE")" "P3: сообщение об ошибке" || return 1
}

# ============================================================================
# P4. Fingerprint = sha1(type:file:line:symbol)[:12], детерминирован.
# ============================================================================
test_P4_fingerprint_deterministic() {
    reset_state
    local rc1 rc2 fp1 fp2 jsonl
    jsonl="$TEST_TMP/reports/nightly-review/2026-09-08.jsonl"
    rc1="$(run_record --task-id t_p4a --component nightly --outcome open-issue-1 \
        --review-date 2026-09-08 --reports-dir "$TEST_TMP/reports" \
        --finding '{"type":"dead-code","file":"a.py","line":10,"symbol":"foo"}')"
    assert_eq "0" "$rc1" "P4: exit 0 (первый вызов)" || return 1
    fp1="$(python3 - "$jsonl" <<'PY'
import json, sys
print(json.loads(open(sys.argv[1]).readline())["findings"][0]["fingerprint"])
PY
)"

    reset_state
    rc2="$(run_record --task-id t_p4b --component nightly --outcome open-issue-2 \
        --review-date 2026-09-08 --reports-dir "$TEST_TMP/reports" \
        --finding '{"type":"dead-code","file":"a.py","line":10,"symbol":"foo"}')"
    assert_eq "0" "$rc2" "P4: exit 0 (второй вызов)" || return 1
    fp2="$(python3 - "$jsonl" <<'PY'
import json, sys
print(json.loads(open(sys.argv[1]).readline())["findings"][0]["fingerprint"])
PY
)"

    assert_eq "$fp1" "$fp2" "P4: одинаковый вход → одинаковый fingerprint" || return 1
    [ "${#fp1}" -eq 12 ] || { printf '  assert fail: P4: длина fingerprint != 12 (%s)\n' "$fp1" >&2; return 1; }
}

# ============================================================================
# P5. files-changed парсится в JSON-массив.
# ============================================================================
test_P5_files_changed_parsed() {
    reset_state
    local rc
    rc="$(run_record --task-id t_p5 --component src-voice --outcome no-real-defect \
        --review-date 2026-09-08 --reports-dir "$TEST_TMP/reports" \
        --files-changed a.py,b.py,c.py)"
    assert_eq "0" "$rc" "P5: exit 0" || return 1
    local line
    line="$(head -1 "$TEST_TMP/reports/nightly-review/2026-09-08.jsonl")"
    assert_contains '"files_changed": ["a.py", "b.py", "c.py"]' "$line" "P5: files_changed массив" || return 1
}

# ============================================================================
# P6. Append-only: два вызова → две валидные строки в одном файле.
# ============================================================================
test_P6_append_only() {
    reset_state
    run_record --task-id t_p6a --component nightly --outcome no-real-defect \
        --review-date 2026-09-08 --reports-dir "$TEST_TMP/reports" >/dev/null
    run_record --task-id t_p6b --component src-voice --outcome no-real-defect \
        --review-date 2026-09-08 --reports-dir "$TEST_TMP/reports" >/dev/null
    local jsonl="$TEST_TMP/reports/nightly-review/2026-09-08.jsonl"
    local n
    n="$(wc -l < "$jsonl" | tr -d ' ')"
    assert_eq "2" "$n" "P6: два вызова = две строки" || return 1
    python3 - "$jsonl" <<'PY'
import json, sys
with open(sys.argv[1]) as f:
    for i, line in enumerate(f, 1):
        json.loads(line)
PY
}

# ============================================================================
# P7. Дедуп: находка с тем же fingerprint, что уже трекается открытым
#     issue → WARNING в stderr на втором вызове, exit всё равно 0.
# ============================================================================
test_P7_duplicate_warning() {
    reset_state
    run_record --task-id t_p7a --component nightly --outcome open-issue-100 \
        --review-date 2026-09-08 --reports-dir "$TEST_TMP/reports" \
        --finding '{"type":"dead-code","file":"a.py","line":10,"symbol":"foo"}' >/dev/null

    local rc
    rc="$(run_record --task-id t_p7b --component nightly --outcome duplicate-suppressed:x \
        --review-date 2026-09-09 --reports-dir "$TEST_TMP/reports" \
        --finding '{"type":"dead-code","file":"a.py","line":10,"symbol":"foo"}')"
    assert_eq "0" "$rc" "P7: exit 0 даже при дубле (fail-open)" || return 1
    assert_contains "WARNING" "$(cat "$STDERR_FILE")" "P7: предупреждение о дубле в stderr" || return 1
    assert_contains "open-issue-100" "$(cat "$STDERR_FILE")" "P7: ссылается на существующий issue" || return 1
}

# ============================================================================
# P8. Невалидный --outcome → exit 2.
# ============================================================================
test_P8_bad_outcome() {
    reset_state
    local rc
    rc="$(run_record --task-id t_p8 --component nightly --outcome bogus-value \
        --review-date 2026-09-08 --reports-dir "$TEST_TMP/reports")"
    assert_eq "2" "$rc" "P8: exit 2 при невалидном outcome" || return 1
}

# ============================================================================
# P9. Невалидный JSON в --finding → exit != 0.
# ============================================================================
test_P9_bad_finding_json() {
    reset_state
    local rc
    rc="$(run_record --task-id t_p9 --component nightly --outcome open-issue-1 \
        --review-date 2026-09-08 --reports-dir "$TEST_TMP/reports" \
        --finding 'not-json')"
    [ "$rc" != "0" ] || { printf '  assert fail: P9: ожидался exit != 0 для невалидного JSON\n' >&2; return 1; }
}

run_test "P1: базовая запись, обязательные поля"           test_P1_basic_record
run_test "P2: no-real-defect не требует --finding"         test_P2_no_real_defect_no_finding_required
run_test "P3: open-issue-* требует --finding"              test_P3_open_issue_requires_finding
run_test "P4: fingerprint детерминирован, длина 12"        test_P4_fingerprint_deterministic
run_test "P5: files-changed → JSON-массив"                 test_P5_files_changed_parsed
run_test "P6: append-only, обе строки валидны"              test_P6_append_only
run_test "P7: дедуп-warning на повторный fingerprint"       test_P7_duplicate_warning
run_test "P8: невалидный outcome → exit 2"                  test_P8_bad_outcome
run_test "P9: невалидный JSON в --finding → exit != 0"      test_P9_bad_finding_json

printf '\n[==========] %d tests, %d passed, %d failed\n' \
    "$TESTS_TOTAL" "$TESTS_PASSED" "$TESTS_FAILED"
if [ "$TESTS_FAILED" -ne 0 ]; then
    printf '[  FAILED  ] %s\n' "${FAILED_NAMES[@]}"
    printf 'artifacts: %s\n' "$TEST_TMP"
    exit 1
fi
rm -rf "$TEST_TMP"
exit 0
