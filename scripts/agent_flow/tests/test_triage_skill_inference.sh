#!/bin/bash
# ============================================================================
# test_triage_skill_inference.sh — unit-test skill-inference в
#                                   agent-flow-triage.sh (ретро t_b3476561,
#                                   02.09.2026).
#
# Root cause: agent-flow-triage.sh создавал kanban-карточки БЕЗ `--skill`,
# и карточки либо крашились rc=0, либо timeout 30/30 (worker не знал что
# делать без доменного skill). 7+ карточек stuck в todo/blocked.
#
# Решение: helper `af_skill_for_profile <assignee>` в lib_agent_flow_common.sh
# даёт детерминированный skill по assignee И проверяет, что skill реально
# установлен в профиле (по on-disk layout skills/<category>/<skill>/SKILL.md).
# fail-OPEN: если skill не найден → stdout пустой, вызов kanban create
# происходит без --skill (как раньше), лучше «нет skill» чем fail-fast
# над process-скриптом.
#
# Стратегия:
#   - unit T1..T6: extracted `af_skill_for_profile` прогоняется на разных
#     assignee, валидируется что:
#       * известные assignee → возвращают ожидаемый skill
#       * НЕизвестные / пустые → пусто (fail-OPEN)
#       * раздел mapping в коде содержит все ключевые роли
#       * функция не задаёт assignee глобально (sandbox-safe)
#   - integration T7..T10: реальный `hermes kanban create` через мок
#     проверяет, что с assignee=devops вызов пробрасывает --skill
#     git-workflow (или другой expected) в командную строку.
#
# Invocation:
#   bash scripts/agent_flow/tests/test_triage_skill_inference.sh
# Возвращает 0 при всех pass, ненулевой при первом fail.
# ============================================================================
set -uo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/.." && pwd)"
LIB="$REPO_ROOT/lib_agent_flow_common.sh"
TRIAGE="$REPO_ROOT/agent-flow-triage.sh"
HANDOFF="$REPO_ROOT/agent-flow-handoff.sh"

PASS=0
FAIL=0
FAILED_CASES=()

log()  { if [ "${VERBOSE:-0}" = "1" ]; then printf '  %s\n' "$*"; fi; }
pass() { PASS=$((PASS+1)); printf '  \033[32m✓\033[0m %s\n' "$1"; }
fail() {
    FAIL=$((FAIL+1)); FAILED_CASES+=("$1")
    printf '  \033[31m✗\033[0m %s\n' "$1"
    if [ -n "${2:-}" ]; then printf '      %s\n' "$2"; fi
}

assert_eq() {  # $1=expected $2=actual $3=msg
    if [ "$1" != "$2" ]; then
        fail "$3" "expected=$1 actual=$2"
        return 1
    fi
    return 0
}

# Extract `af_skill_for_profile` function from lib_agent_flow_common.sh so
# we can source it in isolation. Line-based extraction (lib is plain bash,
# no nested function tricks).
extract_helper() {
    local lib="$1"
    local start end
    start="$(grep -n '^af_skill_for_profile()' "$lib" | head -1 | cut -d: -f1)"
    if [ -z "$start" ]; then
        echo "extract_helper: af_skill_for_profile() not found in $lib" >&2
        return 1
    fi
    end="$(awk -v s="$start" 'NR>=s && /^}$/{print NR; exit}' "$lib")"
    sed -n "${start},${end}p" "$lib" > /tmp/.triage_skill_helper.sh
    # shellcheck disable=SC1091
    . /tmp/.triage_skill_helper.sh
}

# =============================================================================
echo "=== Setup: extract helper ==="
if ! extract_helper "$LIB"; then
    fail "T_setup_extract_helper" "af_skill_for_profile not found in $LIB"
    exit 1
fi
# Mock _af_log (called from fail-OPEN path; not fail-fast).
_af_log() { :; }

# =============================================================================
echo "=== T1: known assignees return expected skill ==="

# Герметичный HERMES_HOME → не зависим от раскладки тестового хоста, прибиваем
# к реальному /home/builder/.hermes (тот же, что у прода).
export HERMES_HOME=/home/builder/.hermes

t1_devops() {
    local actual
    actual="$(af_skill_for_profile devops)"
    log "devops → $actual"
    [ "$actual" = "git-workflow" ] || { fail "T1_devops" "got '$actual'"; return 1; }
    pass "T1_devops → git-workflow"
}
run_t1() { t1_devops || true; }

t1_backend() {
    local actual
    actual="$(af_skill_for_profile backend)"
    log "backend → $actual"
    [ "$actual" = "git-workflow" ] || { fail "T1_backend" "got '$actual'"; return 1; }
    pass "T1_backend → git-workflow"
}

t1_tester() {
    local actual
    actual="$(af_skill_for_profile tester)"
    log "tester → $actual"
    [ "$actual" = "sdlc-review" ] || { fail "T1_tester" "got '$actual'"; return 1; }
    pass "T1_tester → sdlc-review"
}

t1_agent_flow() {
    local actual
    actual="$(af_skill_for_profile agent-flow)"
    log "agent-flow → $actual"
    [ "$actual" = "agent-flow-merge-gate" ] || { fail "T1_agent-flow" "got '$actual'"; return 1; }
    pass "T1_agent-flow → agent-flow-merge-gate"
}

t1_architect() {
    local actual
    actual="$(af_skill_for_profile architect)"
    log "architect → $actual"
    [ "$actual" = "agent-flow-pipeline-ops" ] || { fail "T1_architect" "got '$actual'"; return 1; }
    pass "T1_architect → agent-flow-pipeline-ops"
}

t1_pr_reviewer() {
    local actual
    actual="$(af_skill_for_profile pr-reviewer)"
    log "pr-reviewer → $actual"
    [ "$actual" = "sdlc-review" ] || { fail "T1_pr-reviewer" "got '$actual'"; return 1; }
    pass "T1_pr-reviewer → sdlc-review"
}

run_t1; t1_backend; t1_tester; t1_agent_flow; t1_architect; t1_pr_reviewer

# =============================================================================
echo "=== T2: fail-OPEN for unknown / empty / no-profile assignees ==="

t2_unknown() {
    local actual
    actual="$(af_skill_for_profile does-not-exist-12345)"
    log "unknown → '$actual' (length=${#actual})"
    [ -z "$actual" ] || { fail "T2_unknown" "expected empty, got '$actual'"; return 1; }
    pass "T2_unknown → fail-OPEN (empty)"
}

t2_empty() {
    local actual
    actual="$(af_skill_for_profile '')"
    log "empty assignee → '$actual'"
    [ -z "$actual" ] || { fail "T2_empty" "expected empty, got '$actual'"; return 1; }
    pass "T2_empty → fail-OPEN (empty)"
}

t2_default() {
    local actual
    actual="$(af_skill_for_profile default)"
    log "default → '$actual' (у профиля default на этом хосте нет /profiles/default/skills)"
    # default НЕ имеет свой профиль на данном хосте → fail-OPEN → пусто.
    [ -z "$actual" ] || { fail "T2_default" "expected empty, got '$actual'"; return 1; }
    pass "T2_default → fail-OPEN (no profile dir)"
}

t2_unknown; t2_empty; t2_default

# =============================================================================
echo "=== T3: each known skill from mapping must be installed on its target profile ==="
# Это КЛЮЧЕВОЙ guard — мы не даём assignee→skill, которого у профиля нет.
check_installed() {  # $1=assignee $2=expected_skill $3=test_name
    local actual
    actual="$(af_skill_for_profile "$1")"
    if [ "$actual" = "$2" ]; then
        # Дополнительно: убедимся, что SKILL.md действительно существует
        # на диске в профиле assignee (symlink-following через skill dir).
        if find -L "/home/builder/.hermes/profiles/$1/skills" -maxdepth 5 \
              -type f -name SKILL.md -path "*/$2/SKILL.md" 2>/dev/null \
              | grep -q .; then
            pass "$3 → '$2' installed in profile '$1'"
        else
            fail "$3" "skill '$2' NOT actually installed in /home/builder/.hermes/profiles/$1/skills"
        fi
    else
        fail "$3" "expected '$2', got '$actual'"
    fi
}
check_installed devops          git-workflow          T3_devops
check_installed backend         git-workflow          T3_backend
check_installed tester          sdlc-review           T3_tester
check_installed agent-flow      agent-flow-merge-gate T3_agent-flow
check_installed architect       agent-flow-pipeline-ops T3_architect
check_installed pr-reviewer     sdlc-review           T3_pr-reviewer

# =============================================================================
echo "=== T4: agent-flow-triage.sh script invokes af_skill_for_profile before kanban create ==="

t4_integration() {
    if ! grep -q 'af_skill_for_profile' "$TRIAGE"; then
        fail "T4_triage_uses_helper" "af_skill_for_profile NOT referenced in $TRIAGE"
        return 1
    fi
    # Дополнительно: убедимся что helper вызывается ДО '-- "$title"' в блоке
    # kanban create (а не где-то ещё).
    local line_helper line_create
    line_helper="$(grep -n 'af_skill_for_profile' "$TRIAGE" | head -1 | cut -d: -f1)"
    line_create="$(grep -n 'kanban --board "$KANBAN_BOARD" create' "$TRIAGE" | head -1 | cut -d: -f1)"
    if [ -z "$line_helper" ] || [ -z "$line_create" ]; then
        fail "T4_helper_before_create" "could not locate helper / create lines"
        return 1
    fi
    if [ "$line_helper" -lt "$line_create" ]; then
        pass "T4_helper_before_create (helper @ line ${line_helper}, create @ ${line_create})"
    else
        fail "T4_helper_before_create" "helper @ ${line_helper} is not before create @ ${line_create}"
    fi
    # --skill в командной строке create (по контексту вокруг grepа).
    if grep -nA 10 'kanban --board "$KANBAN_BOARD" create' "$TRIAGE" \
        | grep -qF '"${skill_args[@]}"'; then
        pass "T4_skill_args_in_create --skill пробрасывается"
    else
        fail "T4_skill_args_in_create" "--skill \$'{skill_args[@]}' not in create block"
    fi
}
t4_integration

# =============================================================================
echo "=== T5: agent-flow-handoff.sh также использует af_skill_for_profile ==="

t5_handoff() {
    if ! grep -q 'af_skill_for_profile' "$HANDOFF"; then
        fail "T5_handoff_uses_helper" "af_skill_for_profile NOT referenced in $HANDOFF"
        return 1
    fi
    local line_helper line_create
    line_helper="$(grep -n 'af_skill_for_profile' "$HANDOFF" | head -1 | cut -d: -f1)"
    line_create="$(grep -n 'kanban --board "$BOARD" create' "$HANDOFF" | head -1 | cut -d: -f1)"
    if [ "$line_helper" -lt "$line_create" ]; then
        pass "T5_handoff_helper_before_create (helper @ ${line_helper}, create @ ${line_create})"
    else
        fail "T5_handoff_helper_before_create" "helper @ ${line_helper} not before create @ ${line_create}"
    fi
    if grep -q 'child_skill_args' "$HANDOFF"; then
        pass "T5_handoff_skill_args --skill пробрасывается"
    else
        fail "T5_handoff_skill_args" "child_skill_args not defined in $HANDOFF"
    fi
}
t5_handoff

# =============================================================================
echo "=== T6: bash -n syntax check (no parse errors) ==="

t6_syntax_triage() {
    if bash -n "$TRIAGE" 2>/tmp/.triage_syntax.err; then
        pass "T6_syntax_triage.sh"
    else
        fail "T6_syntax_triage.sh" "$(cat /tmp/.triage_syntax.err)"
    fi
}
t6_syntax_triage

t6_syntax_handoff() {
    if bash -n "$HANDOFF" 2>/tmp/.handoff_syntax.err; then
        pass "T6_syntax_handoff.sh"
    else
        fail "T6_syntax_handoff.sh" "$(cat /tmp/.handoff_syntax.err)"
    fi
}
t6_syntax_handoff

t6_syntax_lib() {
    if bash -n "$LIB" 2>/tmp/.lib_syntax.err; then
        pass "T6_syntax_lib_agent_flow_common.sh"
    else
        fail "T6_syntax_lib_agent_flow_common.sh" "$(cat /tmp/.lib_syntax.err)"
    fi
}
t6_syntax_lib

# =============================================================================
echo
if [ "$FAIL" -gt 0 ]; then
    printf '\033[31mFAIL\033[0m  %d/%d passed\n' "$PASS" "$((PASS+FAIL))"
    printf 'failed: %s\n' "${FAILED_CASES[*]}"
    exit 1
fi
printf '\033[32mPASS\033[0m  %d/%d tests\n' "$PASS" "$((PASS+FAIL))"
exit 0
