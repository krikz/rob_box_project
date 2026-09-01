#!/bin/bash
# ============================================================================
# test_e2e_process_consecutive_fails.sh — ADR-0040 §7.2 / §2.2.2:
# consecutive_fails state-машина. 3 fails подряд → e2e:infra-fail (terminal).
#
# Тестируем чистую логику state-файла через изолированные helpers
# (e2e_run_state_bump_fail / get / reset / set_infra_fail), без gh API.
#
# Scenarios:
#   F1. first fail: counter 0 → 1
#   F2. second fail: counter 1 → 2
#   F3. third fail: counter 2 → 3 (на этом этапе caller выставит e2e:infra-fail)
#   F4. reset: counter → 0, infra_fail=false
#   F5. multiple issues: каждый изолирован
#   F6. threshold semantics: 3 fails одного issue = terminal, не 3 разных
#
# Run:
#   bash scripts/agent_flow/tests/test_e2e_process_consecutive_fails.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_SH="${SCRIPT_SH:-$TEST_DIR/../agent-flow-e2e-process.sh}"

[ -f "$SCRIPT_SH" ] || { echo "FAIL: $SCRIPT_SH not found"; exit 1; }
command -v bash >/dev/null || { echo "FAIL: bash required"; exit 1; }
command -v python3 >/dev/null || { echo "FAIL: python3 required"; exit 1; }

PASS=0
FAIL=0
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

# Extract helpers from main script
extract_func() {
    local fname="$1" outfile="$2"
    awk -v fn="$fname" '
        $0 ~ "^"fn"\\(\\)" {flag=1; depth=0}
        flag {
            print
            for(i=1;i<=length($0);i++) {
                c=substr($0,i,1)
                if(c=="{") depth++
                if(c=="}") { depth--; if(depth==0) { flag=0; print ""; break } }
            }
        }
    ' "$SCRIPT_SH" > "$outfile"
}

for fn in e2e_run_state_load e2e_run_state_save e2e_run_state_get e2e_run_state_bump_fail e2e_run_state_reset e2e_run_state_set_infra_fail; do
    extract_func "$fn" "$WORK/${fn}.sh"
done

# Source all helpers
for f in "$WORK"/e2e_run_state_*.sh; do
    # shellcheck disable=SC1090
    source "$f"
done

# Setup isolated state file
RUN_STATE_FILE="$WORK/run_state.json"
export RUN_STATE_FILE

# Init empty
echo '{"schema_version":1,"issues":{}}' > "$RUN_STATE_FILE"

# ============================================================================
# F1. first fail: counter 0 → 1
# ============================================================================
echo "=== F1: first fail (counter 0 → 1) ==="
n="$(e2e_run_state_bump_fail 1831 "")"
n="$(printf '%s' "$n" | tr -d '[:space:]')"
[ "$n" = "1" ] && { echo "PASS F1: counter=1"; PASS=$((PASS+1)); } \
    || { echo "FAIL F1: got '$n'"; FAIL=$((FAIL+1)); }

# ============================================================================
# F2. second fail: counter 1 → 2
# ============================================================================
echo "=== F2: second fail (counter 1 → 2) ==="
n="$(e2e_run_state_bump_fail 1831 "")"
n="$(printf '%s' "$n" | tr -d '[:space:]')"
[ "$n" = "2" ] && { echo "PASS F2: counter=2"; PASS=$((PASS+1)); } \
    || { echo "FAIL F2: got '$n'"; FAIL=$((FAIL+1)); }

# ============================================================================
# F3. third fail: counter 2 → 3 (call site: выставить e2e:infra-fail)
# ============================================================================
echo "=== F3: third fail (counter 2 → 3) ==="
n="$(e2e_run_state_bump_fail 1831 "")"
n="$(printf '%s' "$n" | tr -d '[:space:]')"
[ "$n" = "3" ] && { echo "PASS F3: counter=3"; PASS=$((PASS+1)); } \
    || { echo "FAIL F3: got '$n'"; FAIL=$((FAIL+1)); }

# Call site: на counter=3 → set_infra_fail + label e2e:infra-fail
e2e_run_state_set_infra_fail 1831
val="$(e2e_run_state_get 1831 infra_fail)"
[ "$val" = "true" ] && { echo "PASS F3a: infra_fail=true после set_infra_fail"; PASS=$((PASS+1)); } \
    || { echo "FAIL F3a: infra_fail='$val'"; FAIL=$((FAIL+1)); }

# ============================================================================
# F4. reset (success verdict или ручной override): counter → 0, infra_fail → false
# ============================================================================
echo "=== F4: reset ==="
e2e_run_state_reset 1831
val_c="$(e2e_run_state_get 1831 consecutive_fails)"
val_i="$(e2e_run_state_get 1831 infra_fail)"
[ "$val_c" = "0" ] && { echo "PASS F4a: counter=0"; PASS=$((PASS+1)); } \
    || { echo "FAIL F4a: counter='$val_c'"; FAIL=$((FAIL+1)); }
[ "$val_i" = "false" ] && { echo "PASS F4b: infra_fail=false"; PASS=$((PASS+1)); } \
    || { echo "FAIL F4b: infra_fail='$val_i'"; FAIL=$((FAIL+1)); }

# ============================================================================
# F5. multiple issues: per-issue isolation
# ============================================================================
echo "=== F5: per-issue isolation ==="
e2e_run_state_bump_fail 1831 "" >/dev/null
e2e_run_state_bump_fail 1832 "" >/dev/null
e2e_run_state_bump_fail 1832 "" >/dev/null
e2e_run_state_bump_fail 1833 "" >/dev/null
e2e_run_state_bump_fail 1833 "" >/dev/null
e2e_run_state_bump_fail 1833 "" >/dev/null
v1="$(e2e_run_state_get 1831 consecutive_fails)"
v2="$(e2e_run_state_get 1832 consecutive_fails)"
v3="$(e2e_run_state_get 1833 consecutive_fails)"
[ "$v1" = "1" ] && [ "$v2" = "2" ] && [ "$v3" = "3" ] \
    && { echo "PASS F5: 1831=1, 1832=2, 1833=3"; PASS=$((PASS+1)); } \
    || { echo "FAIL F5: 1831='$v1', 1832='$v2', 1833='$v3'"; FAIL=$((FAIL+1)); }

# ============================================================================
# F6. threshold semantics: «3 fails подряд» = 1 issue, 3 fails — НЕ 3 разных
# Проверяем: если 3 разных issue получили по 1 fail → НИ ОДИН не достиг
# threshold (consecutive_fails каждого = 1, не 3). Это ADR-0040 Q2.
# ============================================================================
echo "=== F6: threshold = 3 fails ОДНОГО issue, не 3 разных ==="
# Сбросить всё
e2e_run_state_reset 1831 >/dev/null
e2e_run_state_reset 1832 >/dev/null
e2e_run_state_reset 1833 >/dev/null
e2e_run_state_bump_fail 1831 "" >/dev/null  # 1831 counter = 1
e2e_run_state_bump_fail 1832 "" >/dev/null  # 1832 counter = 1
e2e_run_state_bump_fail 1833 "" >/dev/null  # 1833 counter = 1
v1="$(e2e_run_state_get 1831 consecutive_fails)"
v2="$(e2e_run_state_get 1832 consecutive_fails)"
v3="$(e2e_run_state_get 1833 consecutive_fails)"
# НИ ОДИН не достиг 3 → caller НЕ выставляет e2e:infra-fail
if [ "$v1" = "1" ] && [ "$v2" = "1" ] && [ "$v3" = "1" ]; then
    echo "PASS F6: 3 разных issue по 1 fail → все < threshold (counter=1 каждый), e2e:infra-fail НЕ ставится ни одному"
    PASS=$((PASS+1))
else
    echo "FAIL F6: 1831='$v1', 1832='$v2', 1833='$v3' (expected all = 1)"
    FAIL=$((FAIL+1))
fi

# ============================================================================
# F7. state переживает процесс (file persists). Q3 MAINTENANCE compat.
# ============================================================================
echo "=== F7: state persists across process boundaries ==="
# Просто проверим что файл на диске содержит схему
[ -f "$RUN_STATE_FILE" ] && grep -q '"schema_version"' "$RUN_STATE_FILE" \
    && { echo "PASS F7: state file persists with schema"; PASS=$((PASS+1)); } \
    || { echo "FAIL F7: state file missing or malformed"; FAIL=$((FAIL+1)); }

# ============================================================================
echo "=================================================="
echo "PASS: $PASS"
echo "FAIL: $FAIL"
echo "=================================================="
[ "$FAIL" -eq 0 ] || exit 1
exit 0
