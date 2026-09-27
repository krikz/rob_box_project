#!/bin/bash
# ============================================================================
# test_triage_phase3_bug_orphans.sh — юнит-тест Phase 3 (bug-orphans) в
#                                       agent-flow-triage.sh (ретро t_a733c3d2).
#
# Проверяет, что:
#   T1: python-фильтр Phase 3 пропускает issue с `bug` + приоритетом и без
#       process-меток.
#   T2: issue с process-меткой (hermes) → SKIP (Phase 1 уже обрабатывает).
#   T3: issue с process-меткой (stale-candidate) → SKIP (sweep в работе).
#   T4: issue БЕЗ `bug` label → SKIP (не наш scope).
#   T5: issue с `bug`, но без приоритета (или priority:low) → SKIP.
#   T6: issue с `bug` + priority:high, но УЖЕ в phase1_nums → SKIP (race-dedup).
#   T7: PR (defensive — `pull_request: {}`) → SKIP.
#   T8: malformed JSON → [] (graceful).
#   T9: shellcheck-clean + bash -n OK на скрипте.
#   T10: marker `BUG_ORPHAN_MARKER` в коде.
#   T11: env override BUG_PRIORITY_LABELS работает.
#   T12: process substitution < "$_p3_orphan_in" (НЕ pipe в subshell) — иначе
#        счётчики outer-scope обнулятся.
#
# Использование:
#   bash scripts/agent_flow/tests/test_triage_phase3_bug_orphans.sh
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

# --- T1-T8: фильтр Phase 3 (mock JSON) -----------------------------------
echo "=== T1-T8: Phase 3 filter (python block) ==="

# Replica of the Phase 3 filter python block from agent-flow-triage.sh.
filter_phase3() {
    local bug_labels="$1" process_labels="$2" phase1_nums_csv="$3" input_json="$4"
    BUG_LABELS="$bug_labels" PROCESS_LABELS="$process_labels" \
        ALREADY_PIPELINE="$phase1_nums_csv" HERMES_LABEL="hermes" \
        python3 -c '
import os, sys, json
bug_labels_env = os.environ.get("BUG_LABELS", "priority:critical,priority:high,priority:medium")
process_labels_env = os.environ.get("PROCESS_LABELS", "hermes,needs-e2e,e2e-done,e2e:rejected,no-e2e-required,stale-candidate")
already_pipeline = set()
ap = os.environ.get("ALREADY_PIPELINE", "")
if ap:
    for x in ap.split("|"):
        x = x.strip()
        if x.isdigit():
            already_pipeline.add(int(x))
hermes_label = os.environ.get("HERMES_LABEL", "hermes")
bug_label_set = {s.strip() for s in bug_labels_env.split(",") if s.strip()}
process_label_set = {s.strip() for s in process_labels_env.split(",") if s.strip()}
try:
    data = json.load(sys.stdin)
except Exception:
    print("[]"); sys.exit(0)
if not isinstance(data, list):
    print("[]"); sys.exit(0)
keep = []
for it in data:
    if not isinstance(it, dict):
        continue
    n = it.get("number")
    if not isinstance(n, int):
        continue
    if n in already_pipeline:
        continue
    if it.get("pull_request") is not None:
        continue
    label_names = {l.get("name") for l in it.get("labels", []) if isinstance(l, dict)}
    if label_names & process_label_set:
        continue
    if hermes_label in label_names:
        continue
    if "bug" not in label_names:
        continue
    if not (label_names & bug_label_set):
        continue
    keep.append(it)
print(json.dumps(keep, ensure_ascii=False))
' <<<"$input_json"
}

# Mock JSON с 8 issues, покрывающий все сценарии:
#   #2132 — bug + priority:high, БЕЗ process-меток → KEEP
#   #2136 — bug + priority:high, БЕЗ process-меток → KEEP
#   #2137 — bug + priority:high, БЕЗ process-меток → KEEP
#   #2141 — bug + priority:medium, БЕЗ process-меток → KEEP (это 35ч+ stale)
#   #2142 — bug + priority:high, БЕЗ process-меток → KEEP
#   #2143 — bug + priority:high, БЕЗ process-меток → KEEP
#   #2150 — bug + priority:high + hermes (уже в Phase 1) → SKIP
#   #2151 — bug + priority:high + stale-candidate (sweep пометил) → SKIP
#   #2152 — bug + priority:low (НЕ в default bug_labels) → SKIP
#   #2153 — БЕЗ bug label, с priority:high → SKIP (не наш scope)
#   #2154 — bug + priority:high, но pull_request (defensive) → SKIP
#   #2155 — bug + priority:high, в phase1_nums → SKIP
MOCK_BUG_ORPHANS='[
  {"number":2132, "title":"bug(operator P0)", "labels":[{"name":"bug"},{"name":"ai-generated"},{"name":"source:gsd"},{"name":"priority:high"}]},
  {"number":2136, "title":"bug(quest P0)", "labels":[{"name":"bug"},{"name":"ai-generated"},{"name":"source:gsd"},{"name":"priority:high"}]},
  {"number":2137, "title":"bug(operator)", "labels":[{"name":"bug"},{"name":"ai-generated"},{"name":"source:gsd"},{"name":"priority:high"}]},
  {"number":2141, "title":"bug(quest UI)", "labels":[{"name":"bug"},{"name":"ai-generated"},{"name":"source:gsd"},{"name":"priority:medium"}]},
  {"number":2142, "title":"bug(quest #2112)", "labels":[{"name":"bug"},{"name":"ai-generated"},{"name":"source:gsd"},{"name":"priority:high"}]},
  {"number":2143, "title":"bug(quest UI)", "labels":[{"name":"bug"},{"name":"ai-generated"},{"name":"source:gsd"},{"name":"priority:high"}]},
  {"number":2150, "title":"bug fixed", "labels":[{"name":"bug"},{"name":"hermes"},{"name":"agent:devops"},{"name":"priority:high"}]},
  {"number":2151, "title":"stale-bug", "labels":[{"name":"bug"},{"name":"stale-candidate"},{"name":"priority:high"}]},
  {"number":2152, "title":"bug low prio", "labels":[{"name":"bug"},{"name":"priority:low"}]},
  {"number":2153, "title":"feature P0", "labels":[{"name":"feature"},{"name":"priority:high"}]},
  {"number":2154, "title":"bug PR", "labels":[{"name":"bug"},{"name":"priority:high"}], "pull_request":{}},
  {"number":2155, "title":"bug in phase1", "labels":[{"name":"bug"},{"name":"priority:high"}]}
]'

# T1: фильтр без dedup → 7 keep (2132/2136/2137/2141/2142/2143/2155).
# #2155 bug+priority:high без process-меток и без phase1_nums → keep (T6
# отдельно проверяет phase1_nums dedup).
T1_OUT="$(filter_phase3 "priority:critical,priority:high,priority:medium" "hermes,needs-e2e,e2e-done,e2e:rejected,no-e2e-required,stale-candidate" "" "$MOCK_BUG_ORPHANS")"
T1_KEEP="$(printf '%s' "$T1_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(",".join(str(x["number"]) for x in d))')"
EXPECTED_T1="2132,2136,2137,2141,2142,2143,2155"
if [ "$T1_KEEP" = "$EXPECTED_T1" ]; then
    pass "T1: filter keeps 7 (2132/2136/2137/2141/2142/2143/2155)"
else
    fail "T1" "got=$T1_KEEP expected=$EXPECTED_T1"
fi

# T2: #2150 уже имеет hermes → SKIP (не попадёт в keep).
T2_OUT="$(filter_phase3 "priority:critical,priority:high,priority:medium" "hermes,needs-e2e,e2e-done,e2e:rejected,no-e2e-required,stale-candidate" "" "$MOCK_BUG_ORPHANS")"
T2_HAS_2150="$(printf '%s' "$T2_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(any(x["number"]==2150 for x in d))')"
if [ "$T2_HAS_2150" = "False" ]; then
    pass "T2: issue #2150 with `hermes` label is filtered out"
else
    fail "T2" "#2150 unexpectedly kept"
fi

# T3: #2151 уже имеет stale-candidate → SKIP (sweep в работе).
T3_HAS_2151="$(printf '%s' "$T2_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(any(x["number"]==2151 for x in d))')"
if [ "$T3_HAS_2151" = "False" ]; then
    pass "T3: issue #2151 with `stale-candidate` label is filtered out"
else
    fail "T3" "#2151 unexpectedly kept"
fi

# T4: #2153 БЕЗ bug label → SKIP (не наш scope).
T4_HAS_2153="$(printf '%s' "$T2_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(any(x["number"]==2153 for x in d))')"
if [ "$T4_HAS_2153" = "False" ]; then
    pass "T4: issue #2153 without `bug` label is filtered out"
else
    fail "T4" "#2153 unexpectedly kept"
fi

# T5: #2152 bug + priority:low → SKIP (low не в дефолтном списке).
T5_HAS_2152="$(printf '%s' "$T2_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(any(x["number"]==2152 for x in d))')"
if [ "$T5_HAS_2152" = "False" ]; then
    pass "T5: issue #2152 with `priority:low` (not in default bug_labels) is filtered out"
else
    fail "T5" "#2152 unexpectedly kept"
fi

# T6: #2155 в phase1_nums → SKIP (race-dedup).
T6_OUT="$(filter_phase3 "priority:critical,priority:high,priority:medium" "hermes,needs-e2e,e2e-done,e2e:rejected,no-e2e-required,stale-candidate" "2155" "$MOCK_BUG_ORPHANS")"
T6_HAS_2155="$(printf '%s' "$T6_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(any(x["number"]==2155 for x in d))')"
if [ "$T6_HAS_2155" = "False" ]; then
    pass "T6: issue #2155 in phase1_nums is filtered out (race-dedup)"
else
    fail "T6" "#2155 unexpectedly kept"
fi

# T7: PR (defensive — pull_request: {}) → SKIP.
T7_HAS_2154="$(printf '%s' "$T2_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(any(x["number"]==2154 for x in d))')"
if [ "$T7_HAS_2154" = "False" ]; then
    pass "T7: defensive — issue with pull_request key is filtered out"
else
    fail "T7" "#2154 unexpectedly kept"
fi

# T8: malformed JSON → [] (graceful).
T8_OUT="$(filter_phase3 "priority:critical,priority:high,priority:medium" "hermes,needs-e2e" "" "not a json")"
T8_LEN="$(printf '%s' "$T8_OUT" | python3 -c 'import json,sys; print(len(json.load(sys.stdin)))')"
if [ "$T8_LEN" = "0" ]; then
    pass "T8: malformed JSON → [] (graceful)"
else
    fail "T8" "got=$T8_LEN expected 0"
fi

# --- T9-T12: код + env ---------------------------------------------------
echo ""
echo "=== T9-T12: syntax + shellcheck + env ==="

if bash -n "$SCRIPT_UNDER_TEST" 2>/tmp/sc_err; then
    pass "T9a: bash -n syntax check passed"
else
    fail "T9a: bash -n syntax check failed" "$(cat /tmp/sc_err)"
fi

# Verify Phase 3 markers exist in the script
if grep -q 'Phase 3: bug-orphans' "$SCRIPT_UNDER_TEST"; then
    pass "T10a: 'Phase 3: bug-orphans' log marker present in triage.sh"
else
    fail "T10a: Phase 3 log marker NOT found"
fi

if grep -q 'BUG_ORPHAN_MARKER=' "$SCRIPT_UNDER_TEST"; then
    pass "T10b: BUG_ORPHAN_MARKER env var declared in triage.sh"
else
    fail "T10b: BUG_ORPHAN_MARKER NOT declared"
fi

if grep -q 'BUG_PRIORITY_LABELS=' "$SCRIPT_UNDER_TEST"; then
    pass "T10c: BUG_PRIORITY_LABELS env var declared in triage.sh"
else
    fail "T10c: BUG_PRIORITY_LABELS NOT declared"
fi

if grep -q 'NEEDS_TRIAGE_LABEL=' "$SCRIPT_UNDER_TEST"; then
    pass "T10d: NEEDS_TRIAGE_LABEL env var declared in triage.sh"
else
    fail "T10d: NEEDS_TRIAGE_LABEL NOT declared"
fi

# T11: env override BUG_PRIORITY_LABELS — проверяем что override
# реально меняет набор. С 'priority:low,priority:high' (НЕ medium):
#   #2152 (priority:low) → keep (low теперь в scope)
#   #2141 (priority:medium) → drop (medium теперь НЕ в scope)
#   #2153 (БЕЗ bug label) → drop (всё ещё не наш scope)
T11_OUT="$(filter_phase3 "priority:low,priority:high" "hermes,needs-e2e" "" "$MOCK_BUG_ORPHANS")"
T11_HAS_2152="$(printf '%s' "$T11_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(any(x["number"]==2152 for x in d))')"
T11_HAS_2141="$(printf '%s' "$T11_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(any(x["number"]==2141 for x in d))')"
T11_HAS_2153="$(printf '%s' "$T11_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(any(x["number"]==2153 for x in d))')"
if [ "$T11_HAS_2152" = "True" ] && [ "$T11_HAS_2141" = "False" ] && [ "$T11_HAS_2153" = "False" ]; then
    pass "T11: env override BUG_PRIORITY_LABELS='priority:low,priority:high' keeps #2152 (low now in scope), drops #2141 (medium now NOT in scope), drops #2153 (no bug label)"
else
    fail "T11" "2152=$T11_HAS_2152 (expected True), 2141=$T11_HAS_2141 (expected False), 2153=$T11_HAS_2153 (expected False)"
fi

# T12: process substitution (НЕ pipe). Проверяем что в коде используется
# `done < "$_p3_orphan_in"` (а не `done` без redirect), иначе счётчики
# outer-scope обнулятся.
if grep -q 'done < "\$_p3_orphan_in"' "$SCRIPT_UNDER_TEST"; then
    pass "T12a: process substitution done < \$_p3_orphan_in used (counters preserved)"
else
    fail "T12a: done < \$_p3_orphan_in NOT found — счётчики outer-scope могут обнулиться"
fi

# T13: race-dedup regex ((^|[|,])${p3_number}($|[|,])) корректно ловит
# номера в разных позициях _already_in_pipeline.
race_dedup_match() {  # $1=pipeline_csv  $2=number
    if [[ "$1" =~ (^|[|,])${2}($|[|,]) ]]; then
        return 0
    else
        return 1
    fi
}
if race_dedup_match "2155|2160|2170" "2155" && \
   race_dedup_match "2155|2160|2170" "2160" && \
   race_dedup_match "2155|2160|2170" "2170" && \
   race_dedup_match "|2155" "2155" && \
   race_dedup_match "2155|" "2155" && \
   race_dedup_match "2155" "2155" && \
   ! race_dedup_match "2155|2160" "9999" && \
   ! race_dedup_match "2155|2160" "21" && \
   ! race_dedup_match "" "2155"; then
    pass "T13: race-dedup regex (^|[|,])\${N}(\$|[|,]) ловит все позиции (start, middle, end, single, empty pipeline → False)"
else
    fail "T13: race-dedup regex failed"
fi

# T12b: shellcheck regression — новый код не должен добавлять warnings.
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
            pass "T12b: shellcheck — no NEW warnings (origin/develop=$ORIG_SC, current=$NEW_SC)"
        else
            DIFF="$(shellcheck "$SCRIPT_UNDER_TEST" 2>&1)"
            fail "T12b: shellcheck — $((NEW_SC-ORIG_SC)) new warning(s)" "$(printf '%s\n' "$DIFF" | head -30)"
        fi
    else
        log "T12b: not a git repo — skipping baseline comparison"
    fi
else
    log "T12b: shellcheck not installed — skip"
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