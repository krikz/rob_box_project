#!/bin/bash
# ============================================================================
# test_has_label.sh — регресс-гард для helper'а has_label() в
# lib_agent_flow_common.sh (issue #2491).
#
# Контракт has_label (lib_agent_flow_common.sh:218-220):
#   has_label <labels_csv> <label_name> — есть ли метка в CSV-списке.
#   Контракт: $1 — «a,b,c» (обычно уже в lowercase). Через case, без
#   подпроцессов: на 5-минутном тике merge-gate это зовётся сотни раз.
#
# После рефакторинга issue #2491 helper стал load-bearing — его зовут
# runtime_for / max_retries_for / needs_review_evidence_alert_pass_all.
# Любая регрессия формата (CSV, case-folding, substring-vs-whole-word)
# разъедет 3 cron-скрипта одновременно.
#
# Кейсы (acceptance criteria):
#   L1. exact match: has_label "a,b,c" "b" → 0
#   L2. head match: has_label "b,a,c" "b" → 0 (label at head of CSV)
#   L3. tail match: has_label "a,c,b" "b" → 0 (label at tail of CSV)
#   L4. only one:   has_label "b"       "b" → 0
#   L5. absent:     has_label "a,b,c" "d" → 1
#   L6. substring NOT match: has_label "agent:architect" "agent" → 1
#       (label = whole CSV cell, не substring; иначе "agent" матчит "agent:architect")
#   L7. empty CSV:  has_label "" "b" → 1
#   L8. exact-name with colon: has_label "agent:architect,priority:p0" "agent:architect" → 0
#   L9. case-sensitivity (helper expects lowercase — uppercase is NOT match):
#       has_label "Priority:P0" "priority:p0" → 1
#   L10. priority:p0 regression (бывший inline grep):
#       has_label "priority:p0,agent:architect" "priority:p0" → 0
#   L11. evidence-missing regression (бывший inline grep в merge-gate):
#       has_label "needs-review,evidence-missing" "evidence-missing" → 0
#
# Run:
#   bash scripts/agent_flow/tests/test_has_label.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB_SH="${LIB_SH:-$TEST_DIR/../lib_agent_flow_common.sh}"

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

fail() { echo "FAIL: $*"; exit 1; }
pass() { echo "ok: $*"; }

[ -f "$LIB_SH" ] || fail "lib_agent_flow_common.sh not found: $LIB_SH"

# Извлекаем has_label() — функция однострочная (через case), кончается на
# standalone '}'. Тот же подход, что в test_runtime_for_retro.sh.
HAS_LABEL_BODY="$(awk '
    $0 ~ "^has_label\\(\\)" { f=1 }
    f { print }
    f && /^\}/ { exit }
' "$LIB_SH")"
[ -n "$HAS_LABEL_BODY" ] || fail "could not extract has_label from $LIB_SH"

eval "$HAS_LABEL_BODY"
type has_label >/dev/null 2>&1 || fail "has_label not loaded"

# --- L1-L11 --------------------------------------------------------------
check() {
    local name="$1" want="$2" csv="$3" needle="$4"
    has_label "$csv" "$needle"
    local got=$?
    if [ "$got" != "$want" ]; then
        fail "$name: has_label '$csv' '$needle' → got '$got', want '$want'"
    fi
    pass "$name: has_label '$csv' '$needle' → $got"
}

# 0 = match (true), 1 = no match (false)
check "L1_exact_middle"   0 "a,b,c"          "b"
check "L2_head_match"     0 "b,a,c"          "b"
check "L3_tail_match"     0 "a,c,b"          "b"
check "L4_only_one"       0 "b"              "b"
check "L5_absent"         1 "a,b,c"          "d"
check "L6_substring_no"   1 "agent:architect" "agent"
check "L7_empty_csv"      1 ""               "b"
check "L8_colon_name"     0 "agent:architect,priority:p0" "agent:architect"
check "L9_case_sensitive" 1 "Priority:P0"    "priority:p0"
check "L10_p0_regression" 0 "priority:p0,agent:architect" "priority:p0"
check "L11_evidence_regression" 0 "needs-review,evidence-missing" "evidence-missing"

echo
echo "All tests passed (L1-L11)."
