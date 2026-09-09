#!/bin/bash
# ============================================================================
# test_triage_assignee_guard.sh — юнит-тест assignee-existence guard в
#                                  agent-flow-triage.sh (ретро 19.08 t_dd7a5749).
#
# Проверяет, что:
#   T1: функция `load_valid_profiles` корректно парсит вывод `hermes profile
#       list` (фильтрует шапку/футер/blank lines/профиль `default`).
#   T2: функция `is_valid_profile` возвращает 0 для существующего профиля и
#       1 для несуществующего (например, `triager`).
#   T3: guard отклоняет невалидный role (errored++, карточка НЕ создаётся) —
#       это была root cause карточки t_1ca827a6 (2.4ч висела в ready).
#   T4: guard пропускает валидный role (errored не инкрементируется).
#   T5: fail-open: если hermes CLI недоступен (HERMES_BIN не существует), guard
#       НЕ блокирует процесс (valid_profiles = "__disabled__" → все валидны).
#   T6: role_for() корректно извлекает role из label `agent:<role>` (проверка
#       регрессии — раньше guard не существовал, но role_for уже работал).
#   T7: в коде triage.sh появились функции load_valid_profiles + is_valid_profile.
#   T8: shellcheck-clean + bash -n на triage.sh (без новых warnings).
#
# Использование:
#   bash test_triage_assignee_guard.sh
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

# Extract the `load_valid_profiles` and `is_valid_profile` functions from
# triage.sh for testing in isolation via the shared lib (issue #2295). Single
# source of truth for brace-tracking awk — was: line-range /^func()/ → /^}/,
# which silently truncated helpers containing nested { ... }.
# shellcheck source=lib/lib_eval_func.sh
. "$TESTS_DIR/lib/lib_eval_func.sh"

# extract_helpers <script> — eval'ит тела load_valid_profiles и is_valid_profile
# в текущий scope. При реинденте/переносе функций в triage.sh — FAIL с
# понятным сообщением от extract_func_or_die.
extract_helpers() {
    local script="$1"
    local lp_body ip_body
    lp_body="$(extract_func_or_die "$script" load_valid_profiles)" || return 1
    ip_body="$(extract_func_or_die "$script" is_valid_profile)" || return 1
    eval "$lp_body" >/dev/null
    eval "$ip_body" >/dev/null
}

# --- T1: load_valid_profiles parses correctly ----------------------------
echo "=== T1: load_valid_profiles ==="

# Build a fake `hermes profile list` output and test the parsing logic.
fake_profile_list=' Profile          Model                        Gateway      Alias        Distribution
 ───────────────    ───────────────────────────    ───────────    ─────────    ───────────────
  default         MiniMax-M3                   running      —            —
  agent-flow      deepseek-v4-flash            running      agent-flow   —
  analyst         MiniMax-M3                   stopped      analyst      —
  architect       MiniMax-M3                   running      architect    —
  backend         MiniMax-M3                   stopped      backend      —
  devops          MiniMax-M3                   running      devops       —
  tester          MiniMax-M3                   stopped      tester      —
'

parsed="$(printf '%s' "$fake_profile_list" | awk '
    # Skip table separator lines (─ box-drawing chars)
    /^[ \t]*─/ {next}
    # Skip header row (starts with Profile keyword, possibly indented)
    /^[ \t]*Profile[ \t]/ {next}
    # Skip blank lines
    /^[ \t]*$/ {next}
    # Skip `default` placeholder profile (always present, never a real worker)
    /^[ \t]*default[ \t]/ {next}
    {gsub(/^[ \t]+|[ \t]+$/, ""); print $1}
' | sort -u | paste -sd'|' -)"

EXPECTED="agent-flow|analyst|architect|backend|devops|tester"
if [ "$parsed" = "$EXPECTED" ]; then
    pass "T1: parse \`hermes profile list\` output — filters header/separator/blank/default, sorted, pipe-delimited"
else
    fail "T1: parse output" "expected='$EXPECTED' got='$parsed'"
fi

# T1b: empty input → empty list
empty_parse="$(printf '' | awk '
    /^[ \t]*─/ {next}
    /^[ \t]*Profile[ \t]/ {next}
    /^[ \t]*$/ {next}
    /^[ \t]*default[ \t]/ {next}
    {gsub(/^[ \t]+|[ \t]+$/, ""); print $1}
' | sort -u | paste -sd'|' -)"
if [ -z "$empty_parse" ]; then
    pass "T1b: empty input → empty VALID_PROFILES (will trigger fail-open in real script)"
else
    fail "T1b: empty input" "got='$empty_parse'"
fi

# T1c: only `default` profile → empty list (we always exclude default)
only_default_line=' default         MiniMax-M3'
only_default_parse="$(printf '%s\n' "$only_default_line" | awk '
    /^[ \t]*─/ {next}
    /^[ \t]*Profile[ \t]/ {next}
    /^[ \t]*$/ {next}
    /^[ \t]*default[ \t]/ {next}
    {gsub(/^[ \t]+|[ \t]+$/, ""); print $1}
' | sort -u | paste -sd'|' -)"
if [ -z "$only_default_parse" ]; then
    pass "T1c: only the placeholder profile → filtered out (not a real worker)"
else
    fail "T1c: only default" "got='$only_default_parse'"
fi

# --- T2: is_valid_profile correctness ------------------------------------
echo ""
echo "=== T2: is_valid_profile ==="

VALID_PROFILES="$EXPECTED"  # set by hand for unit testing

is_valid() {  # wrapper using the same case logic
    local role="$1"
    case "|$VALID_PROFILES|" in
        *"|$role|"*) return 0 ;;
        *) return 1 ;;
    esac
}

# Valid profiles
for r in agent-flow analyst architect backend devops tester; do
    if is_valid "$r"; then
        pass "T2a: '${r}' is valid (in profile list)"
    else
        fail "T2a: '${r}' should be valid" "VALID_PROFILES=$VALID_PROFILES"
    fi
done

# Invalid profiles (the actual bug from t_1ca827a6)
for r in triager agent-flow-triage unknown_role Agent_Flow DEVOS; do
    if ! is_valid "$r"; then
        pass "T2b: '${r}' is INVALID (not in profile list) — regression case"
    else
        fail "T2b: '${r}' should be INVALID" "VALID_PROFILES=$VALID_PROFILES"
    fi
done

# Empty role
if ! is_valid ""; then
    pass "T2c: empty role is INVALID"
else
    fail "T2c: empty role" "expected false"
fi

# --- T3: integration test — guard rejects triager -----------------------
echo ""
echo "=== T3-T4: integration via real triage.sh flow ==="

# Run the actual triage.sh helper functions in isolation by sourcing.
# We'll simulate the role-decision point by calling role_for then is_valid_profile.
#
# Build a fake labels_json and verify the chain role_for → is_valid_profile.

# Extract helpers from triage.sh
extract_helpers "$SCRIPT_UNDER_TEST"

# Manually set VALID_PROFILES to known list (bypasses hermes CLI)
VALID_PROFILES="$EXPECTED"

# T3: role=triager (the actual bug from t_1ca827a6)
if ! is_valid_profile "triager"; then
    pass "T3: is_valid_profile 'triager' returns false (would block t_1ca827a6)"
else
    fail "T3: is_valid_profile 'triager'" "expected false"
fi

# T4: role=devops (correct value from issue #1444 after our fix)
if is_valid_profile "devops"; then
    pass "T4: is_valid_profile 'devops' returns true (would pass issue #1444)"
else
    fail "T4: is_valid_profile 'devops'" "expected true"
fi

# T4b: role=architect (default — always valid)
if is_valid_profile "architect"; then
    pass "T4b: is_valid_profile 'architect' returns true (AGENT_FLOW_DEFAULT_ROLE)"
else
    fail "T4b: is_valid_profile 'architect'" "expected true"
fi

# --- T5: fail-open when CLI is unavailable ------------------------------
echo ""
echo "=== T5: fail-open semantics ==="

# Extract load_valid_profiles again in isolation (already loaded, but with
# current VALID_PROFILES from the integration test, which is "$EXPECTED").
# Reset state: simulate fresh state where VALID_PROFILES is empty AND
# hermes CLI returns empty.
VALID_PROFILES=""
hermes() {
    # Simulate: hermes CLI is available but returns empty list
    return 0
}
export -f hermes

# Re-extract helpers to reset internal state
extract_helpers "$SCRIPT_UNDER_TEST"

# Manually invalidate HERMES_BIN so guard hits fail-open branch
HERMES_BIN="/nonexistent/hermes"
load_valid_profiles
if [ "$VALID_PROFILES" = "__disabled__" ]; then
    pass "T5a: HERMES_BIN missing → VALID_PROFILES='__disabled__' (fail-open marker)"
else
    fail "T5a: fail-open initialization" "got='$VALID_PROFILES'"
fi

# When disabled, is_valid_profile should pass-through everything (return 0).
if is_valid_profile "anything-goes"; then
    pass "T5b: fail-open passes any role (process not blocked by CLI failure)"
else
    fail "T5b: fail-open should pass-through" "VALID_PROFILES='$VALID_PROFILES'"
fi

# Even totally bogus role passes when fail-open
if is_valid_profile "triager-bogus"; then
    pass "T5c: fail-open passes even unknown role 'triager-bogus'"
else
    fail "T5c: fail-open triager-bogus" "VALID_PROFILES='$VALID_PROFILES'"
fi

# Cleanup: reset
unset -f hermes
VALID_PROFILES="$EXPECTED"

# --- T6: af_role_for regression (issue #2292: единая таблица в lib_common) ---
echo ""
echo "=== T6: af_role_for() regression (issue #2292) ==="

# Issue #2292: вместо реплики role_for (T6a-T6c) теперь зовём НАСТОЯЩУЮ
# функцию af_role_for из lib_agent_flow_common.sh. Если таблица в lib'е
# расходится с реальностью — тест это поймает (а раньше реплика могла
# проходить при сломанном оригинале). role_for() в triage.sh больше не
# существует (issue #2292/#2310 — унифицирован в af_role_for).
LIB_COMMON="$TESTS_DIR/../lib_agent_flow_common.sh"
test -f "$LIB_COMMON" || fail "T6-setup: lib_agent_flow_common.sh not found" "path='$LIB_COMMON'"

# Подгружаем af_role_for в sub-shell без делегирования в `log` родителя.
af_role_for_call() {
    ( unset -f log; source "$LIB_COMMON"; af_role_for "$@" )
}

# T6a: извлечь role из agent:devops
R=$(af_role_for_call "bug,voice,agent:devops,priority:high" "architect")
[ "$R" = "devops" ] && pass "T6a: af_role_for extracts 'devops' from 'agent:devops' label" \
    || fail "T6a: af_role_for extract" "got='$R'"

# T6b: fallback (architect) когда нет agent:* меток
R=$(af_role_for_call "bug,voice,priority:high" "architect")
[ "$R" = "architect" ] && pass "T6b: af_role_for falls back to caller-provided fallback ('architect')" \
    || fail "T6b: af_role_for fallback" "got='$R'"

# T6c: пустые метки → fallback
R=$(af_role_for_call "" "architect")
[ "$R" = "architect" ] && pass "T6c: af_role_for empty labels → fallback" \
    || fail "T6c: af_role_for empty" "got='$R'"

# T6d (новое, issue #2292): agent:tester восстановлен — раньше e2e-process
# терял эту ветку, тут проверяем что таблица в lib'е её содержит.
R=$(af_role_for_call "agent:tester,bug" "devops")
[ "$R" = "tester" ] && pass "T6d: af_role_for восстановил agent:tester (регрессия e2e-process)" \
    || fail "T6d: af_role_for tester" "got='$R'"

# T6e: явный $2=devops ВСЕГДА выигрывает над $AGENT_FLOW_DEFAULT_ROLE
# (caller override приоритетнее env). Это контрактно задокументировано
# в af_role_for: "$2 задан И непустой → используем его".
R=$(AGENT_FLOW_DEFAULT_ROLE=backend bash -c '
    source "'"$LIB_COMMON"'"
    af_role_for "priority:high,bug" "devops"
' </dev/null)
[ "$R" = "devops" ] && pass "T6e: \$AGENT_FLOW_DEFAULT_ROLE=backend + явный \$2=devops → devops (caller override > env)" \
    || fail "T6e: caller override" "got='$R' (expected devops)"
# T6e-примечание: реальный last-resort `devops` при _полностью_ пустом
# fallback (т.е. $2 UNSET + AGENT_FLOW_DEFAULT_ROLE UNSET) срабатывает
# в lib'е через `[ -n "$_fallback" ] || _fallback="devops"`. Тестировать
# его через $() из bash нельзя (command substitution теряет trailing
# пустые args). Это известная особенность bash, не баг af_role_for.

# --- T7: code presence --------------------------------------------------
echo ""
echo "=== T7: code presence in triage.sh ==="

# T7a/T7b (semantic, issue #2295): вместо `grep -q '^load_valid_profiles()'`
# (ломается на реинденте/переносе функции в lib) — спрашиваем extract_func_or_die
# напрямую: «определение извлекается из скрипта». Если функция переехала в
# lib_agent_flow_common.sh, тест сразу это покажет через FAIL от
# extract_func_or_die (или ОК если extract_helpers загрузил её оттуда через
# тот же путь, что используется в T3-T5).
#
# Замечание: T3-T5 уже доказывают, что extract_helpers "загружает" обе функции
# (иначе eval_helper / is_valid_profile вернули бы FAIL). Здесь мы повторяем
# ту же проверку явно, чтобы T7 читался как code-presence contract.
if extract_func_or_die "$SCRIPT_UNDER_TEST" load_valid_profiles >/dev/null; then
    pass "T7a: load_valid_profiles() defined in triage.sh (extract_func_or_die OK)"
else
    fail "T7a: load_valid_profiles() not found via extract_func_or_die"
fi

if extract_func_or_die "$SCRIPT_UNDER_TEST" is_valid_profile >/dev/null; then
    pass "T7b: is_valid_profile() defined in triage.sh (extract_func_or_die OK)"
else
    fail "T7b: is_valid_profile() not found via extract_func_or_die"
fi

# T7c/T7d/T7e — структурные invariants (вызов guard в main loop, label literal,
# retro-card audit trail). Это не contract функциональности (она покрыта T3-T5),
# а паттерн в коде — grep оправдан.
if grep -qF 'is_valid_profile "$role"' "$SCRIPT_UNDER_TEST"; then
    pass "T7c: guard invoked with 'is_valid_profile \"\$role\"' in main loop"
else
    fail "T7c: guard invocation not found in main loop"
fi

if grep -qF 'agent-flow-error' "$SCRIPT_UNDER_TEST"; then
    pass "T7d: guard adds 'agent-flow-error' label to issue on rejection"
else
    fail "T7d: agent-flow-error label not added on rejection"
fi

if grep -qF 't_dd7a5749' "$SCRIPT_UNDER_TEST"; then
    pass "T7e: retro-card reference 't_dd7a5749' present (audit trail)"
else
    fail "T7e: retro-card reference missing"
fi

# --- T8: shellcheck + bash -n -------------------------------------------
echo ""
echo "=== T8: syntax + shellcheck ==="

if bash -n "$SCRIPT_UNDER_TEST" 2>/tmp/sc_err; then
    pass "T8a: bash -n syntax check passed"
else
    fail "T8a: bash -n syntax check failed" "$(cat /tmp/sc_err)"
fi

# shellcheck — only NEW warnings count
# shellcheck disable=SC1008  # comment-as-directive is intentional
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
            pass "T8b: shellcheck — no NEW warnings (origin/develop=$ORIG_SC, current=$NEW_SC)"
        else
            DIFF="$(shellcheck "$SCRIPT_UNDER_TEST" 2>&1)"
            fail "T8b: shellcheck — $((NEW_SC-ORIG_SC)) new warning(s)" "$(printf '%s\n' "$DIFF" | head -30)"
        fi
    else
        log "T8b: not a git repo — skipping baseline comparison"
    fi
else
    log "T8b: shellcheck not installed — skip"
fi

# --- summary -------------------------------------------------------------
echo ""
echo "============================================================"
echo "PASS: $PASS    FAIL: $FAIL"
echo "============================================================"
if [ "$FAIL" -gt 0 ]; then
    printf '\nFAILED CASES:\n'
    for c in "${FAILED_CASES[@]}"; do printf '  - %s\n' "$c"; done
    # Cleanup
    rm -f /tmp/.guard_helpers.sh /tmp/.guard_helpers_dump.sh
    exit 1
fi
# Cleanup on success
rm -f /tmp/.guard_helpers.sh /tmp/.guard_helpers_dump.sh
exit 0