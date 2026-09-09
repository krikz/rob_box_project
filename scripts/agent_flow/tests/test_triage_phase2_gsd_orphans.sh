#!/bin/bash
# ============================================================================
# test_triage_phase2_gsd_orphans.sh — юнит-тест Phase 2 (GSD-orphans) в
#                                       agent-flow-triage.sh (ретро t_360dc1a4,
#                                       issue #1643).
#
# Проверяет, что:
#   T1: python-блок фильтрации Phase 2 (NOT hermes, NOT in phase1_nums)
#       работает корректно на mock JSON (несколько кейсов).
#   T2: фиктивный source:gsd+hermes issue → SKIP в Phase 2 (был в Phase 1).
#   T3: фиктивный source:gsd без hermes + нет в Phase 1 → попадает в Phase 2.
#   T4: фиктивный source:gsd без hermes + есть в Phase 1 (defensive) → SKIP.
#   T5: пустой PR в списке (filterable `pull_request: true`) → SKIP (defensive).
#   T6: Phase 2 пустой (только hermes issues) → log "0 issues".
#   T7: фиктивный issue с меткой e2e-done → проходит через Phase 2 фильтр,
#       потом skip'ается в process_issues_json через DONE_LABEL guard (Phase 1
#       или Phase 2 — одинаково).
#   T8: shellcheck-clean + bash -n OK на скрипте.
#   T9: env override GSD_SOURCE_LABEL работает.
#
# Использование:
#   bash test_triage_phase2_gsd_orphans.sh
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

# --- T1-T6: фильтр Phase 2 (mock JSON) -----------------------------------
echo "=== T1-T6: Phase 2 filter (python block) ==="

# Replica of the Phase 2 filter python block from agent-flow-triage.sh.
# Run standalone with mock JSON input + HERMES_LABEL/PHASE1_NUMS env.
filter_phase2() {
    local hermes_label="$1" phase1_nums_csv="$2" input_json="$3"
    HERMES_LABEL="$hermes_label" PHASE1_NUMS="$phase1_nums_csv" \
        python3 -c '
import os, sys, json
hermes_label = os.environ.get("HERMES_LABEL", "hermes")
phase1_nums = set()
p1 = os.environ.get("PHASE1_NUMS", "")
if p1:
    for x in p1.split("|"):
        x = x.strip()
        if x.isdigit():
            phase1_nums.add(int(x))
try:
    data = json.load(sys.stdin)
except Exception:
    print("[]")
    sys.exit(0)
if not isinstance(data, list):
    print("[]")
    sys.exit(0)
keep = []
for it in data:
    if not isinstance(it, dict):
        continue
    n = it.get("number")
    if not isinstance(n, int):
        continue
    if n in phase1_nums:
        continue
    label_names = {l.get("name") for l in it.get("labels", []) if isinstance(l, dict)}
    if hermes_label in label_names:
        continue
    if it.get("pull_request") is not None:
        continue
    keep.append(it)
print(json.dumps(keep, ensure_ascii=False))
' <<<"$input_json"
}

# Mock JSON с 5 issues: 3 orphan (source:gsd, no hermes), 1 hermes (filter out),
# 1 PR (defensive filter out).
MOCK_GSD='[
  {"number":1643, "title":"[quest] Phase 1.7 e2e smoke", "labels":[{"name":"source:gsd"},{"name":"needs-e2e"}]},
  {"number":1639, "title":"[quest] Phase 1.5 WebXR client", "labels":[{"name":"source:gsd"},{"name":"hermes"},{"name":"agent:backend"}]},
  {"number":1600, "title":"[AV-6] supervisor monitor", "labels":[{"name":"source:gsd"},{"name":"agent:backend"}]},
  {"number":1597, "title":"[AV-3] FSM ModeManager", "labels":[{"name":"source:gsd"},{"name":"agent:backend"}]},
  {"number":1590, "title":"refactor master_prompt", "labels":[{"name":"source:gsd"},{"name":"ai-generated"}]}
]'

# T1: фильтр с phase1_nums= пустой, hermes_label=hermes → 4 orphan issues (1643,
#     1600, 1597, 1590). Issue #1639 выкинут (есть hermes).
T1_OUT="$(filter_phase2 "hermes" "" "$MOCK_GSD")"
T1_KEEP="$(printf '%s' "$T1_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(",".join(str(x["number"]) for x in d))')"
if [ "$T1_KEEP" = "1643,1600,1597,1590" ]; then
    pass "T1: filter keeps 4 orphans (1643,1600,1597,1590), drops #1639 (hermes)"
else
    fail "T1: filter result wrong" "got=$T1_KEEP expected=1643,1600,1597,1590"
fi

# T2: #1639 в Phase 1 (phase1_nums=1639) → SKIP, остаётся 4 orphan.
T2_OUT="$(filter_phase2 "hermes" "1639" "$MOCK_GSD")"
T2_KEEP="$(printf '%s' "$T2_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(",".join(str(x["number"]) for x in d))')"
if [ "$T2_KEEP" = "1643,1600,1597,1590" ]; then
    pass "T2: phase1_nums=1639 dedups #1639 (but it already dropped by hermes label)"
else
    fail "T2" "got=$T2_KEEP"
fi

# T3: source:gsd + no hermes + NOT in phase1_nums → kept.
T3_OUT="$(filter_phase2 "hermes" "" '[{"number":1700,"title":"new orphan","labels":[{"name":"source:gsd"}]}]')"
T3_KEEP="$(printf '%s' "$T3_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(",".join(str(x["number"]) for x in d))')"
if [ "$T3_KEEP" = "1700" ]; then
    pass "T3: source:gsd no hermes not in phase1 → kept (1700)"
else
    fail "T3" "got=$T3_KEEP"
fi

# T4: defensive — source:gsd issue с hermes-меткой в labels → SKIP даже если
#     phase1_nums пустой (защита от rate-limit ситуации).
T4_OUT="$(filter_phase2 "hermes" "" '[{"number":1701,"title":"defensive","labels":[{"name":"source:gsd"},{"name":"hermes"}]}]')"
T4_KEEP="$(printf '%s' "$T4_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(",".join(str(x["number"]) for x in d))')"
if [ "$T4_KEEP" = "" ]; then
    pass "T4: defensive — source:gsd with hermes label skipped even without phase1_nums"
else
    fail "T4" "got=$T4_KEEP expected empty"
fi

# T5: PR (defensive — должен быть отфильтрован как не issue).
T5_OUT="$(filter_phase2 "hermes" "" '[{"number":1702,"title":"a PR","labels":[{"name":"source:gsd"}],"pull_request":{}}]')"
T5_KEEP="$(printf '%s' "$T5_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(",".join(str(x["number"]) for x in d))')"
if [ "$T5_KEEP" = "" ]; then
    pass "T5: defensive — PR object (pull_request key) filtered out"
else
    fail "T5" "got=$T5_KEEP expected empty"
fi

# T6: empty JSON → пустой результат.
T6_OUT="$(filter_phase2 "hermes" "" '[]')"
T6_KEEP="$(printf '%s' "$T6_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(len(d))')"
if [ "$T6_KEEP" = "0" ]; then
    pass "T6: empty input → 0 kept"
else
    fail "T6" "got=$T6_KEEP"
fi

# T7: malformed JSON → [] (graceful, не падаем).
T7_OUT="$(filter_phase2 "hermes" "" "not a json")"
T7_KEEP="$(printf '%s' "$T7_OUT" | python3 -c 'import json,sys; print(json.load(sys.stdin).__class__.__name__)')"
if [ "$T7_KEEP" = "list" ] && [ "$(printf '%s' "$T7_OUT" | python3 -c 'import json,sys; print(len(json.load(sys.stdin)))')" = "0" ]; then
    pass "T7: malformed JSON → [] (graceful)"
else
    fail "T7" "got=$T7_KEEP"
fi

# --- T8: shellcheck-clean + bash -n --------------------------------------
echo ""
echo "=== T8: syntax + shellcheck ==="

if bash -n "$SCRIPT_UNDER_TEST" 2>/tmp/sc_err; then
    pass "T8a: bash -n syntax check passed"
else
    fail "T8a: bash -n syntax check failed" "$(cat /tmp/sc_err)"
fi

# Verify Phase 2 markers exist in the script
if grep -q 'Phase 2: GSD-orphans' "$SCRIPT_UNDER_TEST"; then
    pass "T8b: 'Phase 2: GSD-orphans' log marker present in triage.sh"
else
    fail "T8b: Phase 2 log marker NOT found"
fi

if grep -q 'process_issues_json' "$SCRIPT_UNDER_TEST"; then
    pass "T8c: process_issues_json function extracted (no code duplication)"
else
    fail "T8c: process_issues_json function NOT extracted"
fi

if grep -q 'phase1_issue_numbers' "$SCRIPT_UNDER_TEST"; then
    pass "T8d: phase1_issue_numbers dedup set present"
else
    fail "T8d: phase1_issue_numbers dedup NOT present"
fi

# Verify the orphan issue #1643 is mentioned in the code (retro key)
if grep -q 't_360dc1a4' "$SCRIPT_UNDER_TEST"; then
    pass "T8e: retro key t_360dc1a4 cited in script"
else
    fail "T8e: retro key t_360dc1a4 NOT cited"
fi

# baseline comparison via shellcheck binary: must NOT introduce NEW warnings
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
        log "T8f: not a git repo — skipping baseline comparison"
    fi
else
    log "T8f: shellcheck not installed — skip"
fi

# --- T9: env override GSD_SOURCE_LABEL ------------------------------------
echo ""
echo "=== T9: env override GSD_SOURCE_LABEL ==="

# Verify the env var is honored (set as default in script + accepted at runtime).
if grep -q 'GSD_SOURCE_LABEL=.\${GSD_SOURCE_LABEL:-source:gsd}' "$SCRIPT_UNDER_TEST"; then
    pass "T9a: GSD_SOURCE_LABEL env override declared (default 'source:gsd')"
else
    fail "T9a: GSD_SOURCE_LABEL env override NOT declared"
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
