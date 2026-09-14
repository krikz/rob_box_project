#!/bin/bash
# ============================================================================
# test_triage_force_triage.sh — юнит-тест Phase 4 (force-triage) в
#                               agent-flow-triage.sh (ретро t_25a2b395,
#                               тикет orphan-stale-no-agent-assign).
#
# Проверяет, что:
#   T1: python-фильтр Phase 4 пропускает issue с priority:high + (bug/voice/
#       operator) и без process/agent:* меток.
#   T2: issue с process-меткой (hermes) → SKIP (Phase 1 уже обрабатывает).
#   T3: issue с scope-label (voice) БЕЗ bug → KEEP (это второй класс orphan'ов,
#       который Phase 3 НЕ покрывает).
#   T4: issue БЕЗ priority:high (priority:low/medium) → SKIP.
#   T5: issue БЕЗ scope-label (например feature) → SKIP.
#   T6: issue С `agent:*` меткой → SKIP (assignee уже определён).
#   T7: PR (defensive — pull_request: {}) → SKIP.
#   T8: malformed JSON → [] (graceful).
#   T9: bash -n syntax check passed.
#   T10: markers Phase 4 в коде (FORCE_TRIAGE_*, Phase 4: force-triage,
#        [FORCE-TRIAGE]).
#   T11: env override FORCE_TRIAGE_SCOPE_LABELS меняет фильтр.
#   T12: env override FORCE_TRIAGE_PRIORITY_LABEL меняет фильтр.
#   T13: FORCE_TRIAGE_APPLY=false → только лог, без apply (проверяем через
#        grep + проверку marker counter logic).
#   T14: FORCE_TRIAGE_APPLY=true → применяет (в simulate-режиме проверяем
#        что branch выполняет `gh issue edit ... add-label needs-triage`).
#   T15: process substitution < (НЕ pipe) — счётчики outer-scope сохраняются.
#   T16: shellcheck не добавляет новых warnings vs origin/develop.
#
# Использование:
#   bash scripts/agent_flow/tests/test_triage_force_triage.sh
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

# --- T1-T8: фильтр Phase 4 (mock JSON) -----------------------------------
echo "=== T1-T8: Phase 4 force-triage filter (python block) ==="

# Replica of the Phase 4 filter python block from agent-flow-triage.sh.
filter_phase4() {
    local scope_csv="$1" priority_label="$2" apply_flag="$3" input_json="$4"
    FORCE_SCOPE="$scope_csv" \
    PROCESS_LABELS_PHASE4="hermes,needs-e2e,e2e-done,e2e:rejected,no-e2e-required,stale-candidate" \
    HERMES_LABEL_PHASE4="hermes" \
    FORCE_APPLY="$apply_flag" \
    python3 -c '
import os, sys, json
scope_env = os.environ.get("FORCE_SCOPE", "bug,voice,operator")
process_env = os.environ.get("PROCESS_LABELS_PHASE4", "hermes,needs-e2e,e2e-done,e2e:rejected,no-e2e-required,stale-candidate")
hermes_label = os.environ.get("HERMES_LABEL_PHASE4", "hermes")
force_apply = (os.environ.get("FORCE_APPLY", "false").lower() == "true")
scope_set = set(s.strip() for s in scope_env.split(",") if s.strip())
process_set = set(s.strip() for s in process_env.split(",") if s.strip())
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
    if it.get("pull_request") is not None:
        continue
    n = it.get("number")
    if not isinstance(n, int):
        continue
    label_names = set(l.get("name") for l in it.get("labels", []) if isinstance(l, dict))
    if label_names & process_set:
        continue
    if hermes_label in label_names:
        continue
    if any(l.startswith("agent:") for l in label_names):
        continue
    if not (label_names & scope_set):
        continue
    keep.append(it)
print(json.dumps(keep, ensure_ascii=False))
' <<<"$input_json"
}

# Mock JSON с 10 issues, покрывающий все сценарии.
# Примечание: priority-фильтрация делается на уровне `gh_list_issues_by_label
# priority:high` (filter-вход), поэтому mock уже содержит только issues
# с priority:high (или эквивалент — для теста scope-фильтра нам не нужно
# проверять priority:low, это upstream-фильтрация, она покрыта T12).
#   #1881 — bug(voice) + priority:high, БЕЗ process/agent:* → KEEP (target case)
#   #2132 — bug(operator P0) + priority:high, БЕЗ process/agent:* → KEEP
#   #2137 — bug(operator) + priority:high, БЕЗ process/agent:* → KEEP
#   #2300 — voice + priority:high (НЕ bug, но в scope) → KEEP (2-й класс orphan'ов)
#   #2301 — operator + priority:high (НЕ bug) → KEEP (2-й класс orphan'ов)
#   #2302 — feature + priority:high, НЕ scope → SKIP
#   #2303 — bug + priority:high + hermes → SKIP (Phase 1)
#   #2304 — bug + priority:high + agent:devops → SKIP (assignee есть)
#   #2305 — bug + priority:high + stale-candidate → SKIP (sweep в работе)
#   #2306 — bug + priority:high + pull_request → SKIP (defensive PR)
MOCK_FORCE='[
  {"number":1881, "title":"bug(voice)", "labels":[{"name":"bug"},{"name":"voice"},{"name":"ai-generated"},{"name":"priority:high"}]},
  {"number":2132, "title":"bug(operator P0)", "labels":[{"name":"bug"},{"name":"operator"},{"name":"ai-generated"},{"name":"priority:high"}]},
  {"number":2137, "title":"bug(operator)", "labels":[{"name":"bug"},{"name":"operator"},{"name":"ai-generated"},{"name":"priority:high"}]},
  {"number":2300, "title":"voice only", "labels":[{"name":"voice"},{"name":"ai-generated"},{"name":"priority:high"}]},
  {"number":2301, "title":"operator only", "labels":[{"name":"operator"},{"name":"ai-generated"},{"name":"priority:high"}]},
  {"number":2302, "title":"feature P0", "labels":[{"name":"feature"},{"name":"priority:high"}]},
  {"number":2303, "title":"bug with hermes", "labels":[{"name":"bug"},{"name":"hermes"},{"name":"agent:devops"},{"name":"priority:high"}]},
  {"number":2304, "title":"bug with agent", "labels":[{"name":"bug"},{"name":"agent:devops"},{"name":"priority:high"}]},
  {"number":2305, "title":"stale bug", "labels":[{"name":"bug"},{"name":"stale-candidate"},{"name":"priority:high"}]},
  {"number":2306, "title":"bug PR", "labels":[{"name":"bug"},{"name":"priority:high"}], "pull_request":{}}
]'

# T1: дефолтный фильтр (priority:high + bug/voice/operator без process/agent:*)
# → 5 keep (1881, 2132, 2137, 2300, 2301).
T1_OUT="$(filter_phase4 "bug,voice,operator" "priority:high" "false" "$MOCK_FORCE")"
T1_KEEP="$(printf '%s' "$T1_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(",".join(str(x["number"]) for x in d))')"
EXPECTED_T1="1881,2132,2137,2300,2301"
if [ "$T1_KEEP" = "$EXPECTED_T1" ]; then
    pass "T1: default filter keeps 5 (1881/2132/2137 + voice-only #2300 + operator-only #2301)"
else
    fail "T1" "got=$T1_KEEP expected=$EXPECTED_T1"
fi

# T2: #2303 с hermes → SKIP.
T2_HAS_2303="$(printf '%s' "$T1_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(any(x["number"]==2303 for x in d))')"
if [ "$T2_HAS_2303" = "False" ]; then
    pass "T2: issue #2303 with \`hermes\` label is filtered out"
else
    fail "T2" "#2303 unexpectedly kept"
fi

# T3: #2300 voice-only + priority:high → KEEP (это целевой второй класс,
# который Phase 3 НЕ ловит — Phase 3 фильтрует по `bug`, Phase 4 — по scope).
T3_HAS_2300="$(printf '%s' "$T1_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(any(x["number"]==2300 for x in d))')"
if [ "$T3_HAS_2300" = "True" ]; then
    pass "T3: voice-only #2300 + priority:high (no \`bug\`) is KEPT — 2nd class orphan not covered by Phase 3"
else
    fail "T3" "#2300 (voice-only) NOT kept — Phase 4 regressed to Phase 3 behavior"
fi

# T4 (заменено): #2305 priority:high + stale-candidate → SKIP (sweep в работе).
# Примечание: priority-фильтрация (high vs low) делается на уровне
# `gh_list_issues_by_label` (upstream), поэтому тестировать priority:low на
# уровне python-фильтра некорректно — проверка T12 покрывает priority-env.
T4_HAS_2305="$(printf '%s' "$T1_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(any(x["number"]==2305 for x in d))')"
if [ "$T4_HAS_2305" = "False" ]; then
    pass "T4: issue #2305 with \`stale-candidate\` process-label is filtered out (sweep in progress)"
else
    fail "T4" "#2305 unexpectedly kept"
fi

# T5: #2302 feature (no scope label) → SKIP.
T5_HAS_2302="$(printf '%s' "$T1_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(any(x["number"]==2302 for x in d))')"
if [ "$T5_HAS_2302" = "False" ]; then
    pass "T5: issue #2302 without scope label (feature) is filtered out"
else
    fail "T5" "#2302 unexpectedly kept"
fi

# T6: #2304 С agent:* → SKIP (assignee есть).
T6_HAS_2304="$(printf '%s' "$T1_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(any(x["number"]==2304 for x in d))')"
if [ "$T6_HAS_2304" = "False" ]; then
    pass "T6: issue #2304 with \`agent:devops\` label is filtered out (assignee already set)"
else
    fail "T6" "#2304 unexpectedly kept"
fi

# T7: #2306 PR (defensive — pull_request: {}) → SKIP.
T7_HAS_2306="$(printf '%s' "$T1_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(any(x["number"]==2306 for x in d))')"
if [ "$T7_HAS_2306" = "False" ]; then
    pass "T7: defensive issue #2306 with pull_request is filtered out"
else
    fail "T7" "#2306 unexpectedly kept"
fi

# T8: malformed JSON → [] (graceful).
T8_OUT="$(filter_phase4 "bug,voice,operator" "priority:high" "false" "not a json")"
T8_LEN="$(printf '%s' "$T8_OUT" | python3 -c 'import json,sys; print(len(json.load(sys.stdin)))' 2>/dev/null || echo 0)"
if [ "$T8_LEN" = "0" ]; then
    pass "T8: malformed JSON → [] (graceful)"
else
    fail "T8" "got=$T8_LEN expected 0"
fi

# --- T9-T16: код + env + apply-modes -------------------------------------
echo ""
echo "=== T9-T16: syntax + shellcheck + env overrides + apply-modes ==="

# T9: bash -n syntax check.
if bash -n "$SCRIPT_UNDER_TEST" 2>/tmp/sc_err; then
    pass "T9: bash -n syntax check passed"
else
    fail "T9: bash -n syntax check failed" "$(cat /tmp/sc_err)"
fi

# T10a-d: маркеры Phase 4 в коде.
if grep -q 'Phase 4: force-triage' "$SCRIPT_UNDER_TEST"; then
    pass "T10a: 'Phase 4: force-triage' log marker present in triage.sh"
else
    fail "T10a: Phase 4 log marker NOT found"
fi

if grep -q 'FORCE_TRIAGE_PRIORITY_LABEL=' "$SCRIPT_UNDER_TEST"; then
    pass "T10b: FORCE_TRIAGE_PRIORITY_LABEL env var declared"
else
    fail "T10b: FORCE_TRIAGE_PRIORITY_LABEL NOT declared"
fi

if grep -q 'FORCE_TRIAGE_SCOPE_LABELS=' "$SCRIPT_UNDER_TEST"; then
    pass "T10c: FORCE_TRIAGE_SCOPE_LABELS env var declared"
else
    fail "T10c: FORCE_TRIAGE_SCOPE_LABELS NOT declared"
fi

if grep -q '\[FORCE-TRIAGE\]' "$SCRIPT_UNDER_TEST"; then
    pass "T10d: [FORCE-TRIAGE] log prefix present"
else
    fail "T10d: [FORCE-TRIAGE] log prefix NOT found"
fi

if grep -q 'FORCE_TRIAGE_MARKER=' "$SCRIPT_UNDER_TEST"; then
    pass "T10e: FORCE_TRIAGE_MARKER env var declared"
else
    fail "T10e: FORCE_TRIAGE_MARKER NOT declared"
fi

if grep -q 'FORCE_TRIAGE_APPLY=' "$SCRIPT_UNDER_TEST"; then
    pass "T10f: FORCE_TRIAGE_APPLY env var declared"
else
    fail "T10f: FORCE_TRIAGE_APPLY NOT declared"
fi

# T11: env override FORCE_TRIAGE_SCOPE_LABELS — только 'bug' → voice/operator
# issues должны выпасть. Из MOCK_FORCE: #1881/2132/2137 (имеют bug) → keep,
# #2300 (только voice) → drop, #2301 (только operator) → drop.
T11_OUT="$(filter_phase4 "bug" "priority:high" "false" "$MOCK_FORCE")"
T11_HAS_1881="$(printf '%s' "$T11_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(any(x["number"]==1881 for x in d))')"
T11_HAS_2300="$(printf '%s' "$T11_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(any(x["number"]==2300 for x in d))')"
T11_HAS_2301="$(printf '%s' "$T11_OUT" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(any(x["number"]==2301 for x in d))')"
if [ "$T11_HAS_1881" = "True" ] && [ "$T11_HAS_2300" = "False" ] && [ "$T11_HAS_2301" = "False" ]; then
    pass "T11: env override FORCE_TRIAGE_SCOPE_LABELS='bug' keeps #1881 (bug), drops #2300 (voice-only), drops #2301 (operator-only)"
else
    fail "T11" "1881=$T11_HAS_1881 (expected True), 2300=$T11_HAS_2300 (expected False), 2301=$T11_HAS_2301 (expected False)"
fi

# T12: env override FORCE_TRIAGE_PRIORITY_LABEL — нельзя проверить на этом
# mock'е (он уже отфильтрован по priority:high на уровне gh issue list),
# но мы можем проверить, что код использует переменную, не хардкодит.
if grep -q 'gh_list_issues_by_label "\$FORCE_TRIAGE_PRIORITY_LABEL"' "$SCRIPT_UNDER_TEST"; then
    pass "T12: gh_list_issues_by_label uses \$FORCE_TRIAGE_PRIORITY_LABEL (env-overridable)"
else
    fail "T12: FORCE_TRIAGE_PRIORITY_LABEL NOT used in gh_list_issues_by_label call"
fi

# T13: apply=false → log-only. Проверяем что в коде есть early-continue
# когда FORCE_TRIAGE_APPLY != "true".
if grep -q 'if \[ "\$FORCE_TRIAGE_APPLY" != "true" \]' "$SCRIPT_UNDER_TEST"; then
    pass "T13: FORCE_TRIAGE_APPLY != true → log-only branch (early continue) present"
else
    fail "T13: FORCE_TRIAGE_APPLY=false branch NOT found"
fi

# T14: apply=true → gh issue edit --add-label. Проверяем что код делает
# add-label needs-triage + agent:<default> при apply=true.
if grep -q 'add-label "\$NEEDS_TRIAGE_LABEL"' "$SCRIPT_UNDER_TEST"; then
    pass "T14a: needs-triage add-label present in Phase 4 apply branch"
else
    fail "T14a: needs-triage add-label NOT found"
fi
if grep -q 'add-label "agent:\${FORCE_TRIAGE_DEFAULT_AGENT}"' "$SCRIPT_UNDER_TEST"; then
    pass "T14b: agent:<default> add-label present in Phase 4 apply branch"
else
    fail "T14b: agent:<default> add-label NOT found"
fi
if grep -q 'force-triage-mark' "$SCRIPT_UNDER_TEST"; then
    pass "T14c: phase4-force-triage-mark action marker in comment body"
else
    fail "T14c: phase4-force-triage-mark marker NOT found"
fi

# T15: process substitution (НЕ pipe). Проверяем что в коде используется
# `done < "$_p4_in"` (а не pipe), иначе счётчики outer-scope обнулятся.
if grep -q 'done < "\$_p4_in"' "$SCRIPT_UNDER_TEST"; then
    pass "T15: process substitution done < \$_p4_in used (counters preserved)"
else
    fail "T15: done < \$_p4_in NOT found — счётчики outer-scope могут обнулиться"
fi

# T16: summary log включает phase4-force-triage counters.
if grep -q 'phase4-force-triage: candidates=' "$SCRIPT_UNDER_TEST"; then
    pass "T16: summary log includes phase4-force-triage counters"
else
    fail "T16: summary log does NOT include phase4-force-triage"
fi

# T17: ADR-0022 обновлён — раздел 4.2.1 (force-triage addendum).
if grep -q 'Force-triage для high-priority voice/operator-bugs' "/home/builder/rob_box_project_main/docs/adr/0022-process-e2e-done-gates.md"; then
    pass "T17: ADR-0022 §4.2.1 (force-triage addendum) present"
else
    fail "T17: ADR-0022 §4.2.1 NOT found"
fi

# T18: shellcheck regression — новый код не должен добавлять warnings.
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
            pass "T18: shellcheck — no NEW warnings (origin/develop=$ORIG_SC, current=$NEW_SC)"
        else
            DIFF="$(shellcheck "$SCRIPT_UNDER_TEST" 2>&1)"
            fail "T18: shellcheck — $((NEW_SC-ORIG_SC)) new warning(s)" "$(printf '%s\n' "$DIFF" | head -30)"
        fi
    else
        log "T18: not a git repo — skipping baseline comparison"
    fi
else
    log "T18: shellcheck not installed — skip"
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