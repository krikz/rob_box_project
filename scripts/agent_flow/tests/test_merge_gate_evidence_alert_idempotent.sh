#!/bin/bash
# ============================================================================
# test_merge_gate_evidence_alert_idempotent.sh — регресс-гард для замены
# inline grep на has_label() в needs_review_evidence_alert_pass_all
# (issue #2491).
#
# Поведение: если у PR уже есть метка EVIDENCE_MISSING_LABEL (=evidence-missing
# по умолчанию), watchdog НЕ должен её добавлять повторно (idempotent).
# До рефакторинга это делалось inline-grep'ом, после — вызовом has_label.
#
# Тест проверяет контракт «не добавлять, если уже есть» через сам helper
# has_label — это та же логика, что использует merge-gate после рефакторинга.
#
# Кейсы:
#   E1. evidence-missing уже есть → skip add
#   E2. evidence-missing отсутствует → нужно add
#   E3. case-insensitive: метка написана в любом регистре (мы lowercasing'им
#       перед has_label) — behavior preserved
#   E4. CSV с кучей меток, evidence-missing в середине → skip
#
# Run:
#   bash scripts/agent_flow/tests/test_merge_gate_evidence_alert_idempotent.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB_SH="${LIB_SH:-$TEST_DIR/../lib_agent_flow_common.sh}"

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

fail() { echo "FAIL: $*"; exit 1; }
pass() { echo "ok: $*"; }

[ -f "$LIB_SH" ] || fail "lib_agent_flow_common.sh not found: $LIB_SH"

HAS_LABEL_BODY="$(awk '
    $0 ~ "^has_label\\(\\)" { f=1 }
    f { print }
    f && /^\}/ { exit }
' "$LIB_SH")"
[ -n "$HAS_LABEL_BODY" ] || fail "could not extract has_label from $LIB_SH"
eval "$HAS_LABEL_BODY"

# Эмулируем логику merge-gate: lowercased CSV → has_label → if absent, add label.
# Возвращает 0 если метку нужно добавить, 1 если уже есть (skip).
should_add_evidence_missing() {
    local labels_csv="$1" target="${EVIDENCE_MISSING_LABEL:-evidence-missing}"
    local lower
    lower="$(printf '%s' "$labels_csv" | tr '[:upper:]' '[:lower:]')"
    if has_label "$lower" "$target"; then
        return 1  # skip
    fi
    return 0  # add
}

# --- E1-E4 --------------------------------------------------------------
check_should_add() {
    local name="$1" want_add="$2" csv="$3"
    local got_add
    if should_add_evidence_missing "$csv"; then got_add=0; else got_add=1; fi
    if [ "$got_add" != "$want_add" ]; then
        fail "$name: csv='$csv' → got_add='$got_add', want_add='$want_add'"
    fi
    pass "$name: csv='$csv' → add=$got_add"
}

check_should_add "E1_already_present_skip" 1 "needs-review,evidence-missing,agent:architect"
check_should_add "E2_absent_add"            0 "needs-review,agent:architect"
check_should_add "E3_mixed_case_skip"       1 "needs-review,Evidence-Missing"
check_should_add "E4_many_labels_skip"      1 "needs-review,agent:architect,evidence-missing,priority:p0"
check_should_add "E5_empty_csv_add"         0 ""
# E6: evidence-missing — единственная метка → она уже есть → skip.
# Идемпотентность требует skip в этом случае (НЕ add, иначе дубликат).
check_should_add "E6_only_evidence_skip"    1 "evidence-missing"

echo
echo "All tests passed (E1-E6)."
