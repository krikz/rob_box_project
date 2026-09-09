#!/bin/bash
# ============================================================================
# test_af_skill_task_type.sh — hermetic test для task-type dimension в
#                               af_skill_for_profile (ретро 05.09).
#
# af_skill_for_profile <assignee> [labels_csv] теперь маппит тип задачи
# (label issue) на repo-скилл, переопределяя роль:
#   bug / type:bug                     -> systematic-debugging
#   type:functional / type:feature     -> test-driven-development
#   type:refactor / type:tech-debt     -> codebase-design
#   type:process                       -> agent-flow
#   нет type-label / task-скилл не уст. -> роль (git-workflow и т.д.)
#   неизвестный assignee / пусто       -> пусто (fail-OPEN)
#
# Invocation:
#   bash scripts/agent_flow/tests/test_af_skill_task_type.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB="$TEST_DIR/../lib_agent_flow_common.sh"

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

PASS=0; FAIL=0
pass() { PASS=$((PASS+1)); echo "  ✓ $1"; }
fail() { FAIL=$((FAIL+1)); echo "  ✗ $1 (${2:-})"; }

# Extract af_skill_for_profile (тот же приём, что в test_triage_skill_inference.sh)
start="$(grep -n '^af_skill_for_profile()' "$LIB" | head -1 | cut -d: -f1)"
[ -n "$start" ] || { echo "FAIL: af_skill_for_profile not found in $LIB"; exit 1; }
end="$(awk -v s="$start" 'NR>=s && /^}$/{print NR; exit}' "$LIB")"
sed -n "${start},${end}p" "$LIB" > "$WORK/helper.sh"
# shellcheck disable=SC1091
. "$WORK/helper.sh"
_af_log() { :; }

# --- герметичный HERMES_HOME: backend с repo/ + роль-скиллом ----------------
export HERMES_HOME="$WORK/hermes"
mkdir -p "$HERMES_HOME/profiles/backend/skills/repo"
for s in systematic-debugging test-driven-development codebase-design \
         verification-before-completion agent-flow; do
    mkdir -p "$HERMES_HOME/profiles/backend/skills/repo/$s"
    echo "# $s" > "$HERMES_HOME/profiles/backend/skills/repo/$s/SKILL.md"
done
mkdir -p "$HERMES_HOME/profiles/backend/skills/bundled/git-workflow"
echo "# git-workflow" > "$HERMES_HOME/profiles/backend/skills/bundled/git-workflow/SKILL.md"

assert_eq() {  # $1=expected $2=actual $3=msg
    if [ "$1" = "$2" ]; then pass "$3"; else fail "$3" "expected='$1' actual='$2'"; fi
}

# --- T1..T7: тип задачи переопределяет роль --------------------------------
assert_eq "systematic-debugging" "$(af_skill_for_profile backend 'type:bug,priority:high')" "type:bug -> systematic-debugging"
assert_eq "systematic-debugging" "$(af_skill_for_profile backend 'bug')" "bare bug -> systematic-debugging"
assert_eq "test-driven-development" "$(af_skill_for_profile backend 'type:functional')" "type:functional -> tdd"
assert_eq "test-driven-development" "$(af_skill_for_profile backend 'type:feature')" "type:feature -> tdd"
assert_eq "codebase-design" "$(af_skill_for_profile backend 'type:refactor')" "type:refactor -> codebase-design"
assert_eq "codebase-design" "$(af_skill_for_profile backend 'type:tech-debt')" "type:tech-debt -> codebase-design"
assert_eq "agent-flow" "$(af_skill_for_profile backend 'type:process')" "type:process -> agent-flow"

# --- T8..T9: роль fallback -------------------------------------------------
assert_eq "git-workflow" "$(af_skill_for_profile backend '')" "no labels -> git-workflow (role)"
assert_eq "git-workflow" "$(af_skill_for_profile backend)" "one arg -> git-workflow (role)"
assert_eq "git-workflow" "$(af_skill_for_profile backend 'type:security,priority:low')" "type:security -> role fallback"

# --- T10: task-скилл не установлен → fallback на роль ----------------------
# Отдельный HERMES_HOME2: backend без repo/ скиллов, только role-скилл.
HERMES2="$WORK/hermes2"
mkdir -p "$HERMES2/profiles/backend/skills/bundled/git-workflow"
echo "# git-workflow" > "$HERMES2/profiles/backend/skills/bundled/git-workflow/SKILL.md"
t10_actual="$(HERMES_HOME="$HERMES2" af_skill_for_profile backend 'type:bug')"
assert_eq "git-workflow" "$t10_actual" "task skill missing -> role fallback"

# --- T11: неизвестный assignee → пусто (fail-OPEN) -------------------------
assert_eq "" "$(af_skill_for_profile does-not-exist 'type:bug')" "unknown assignee -> empty"

# --- T12: регистр меток не важен -------------------------------------------
assert_eq "systematic-debugging" "$(af_skill_for_profile backend 'TYPE:BUG')" "upper-case label -> systematic-debugging"

echo
if [ "$FAIL" -gt 0 ]; then
    echo "FAIL  $PASS/$((PASS+FAIL)) passed"
    exit 1
fi
echo "PASS  $PASS/$((PASS+FAIL)) tests"
exit 0
