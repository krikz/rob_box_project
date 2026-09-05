#!/bin/bash
# ============================================================================
# test_sync_skills.sh — hermetic test для sync-skills.sh (ретро 05.09).
#
# sync-skills.sh доставляет allowlist repo-скиллов (.agents/skills/<skill>)
# в профили воркеров hardlink-ами (skills/repo/<skill>/SKILL.md). Без него
# af_skill_for_profile() маппит типы задач на скиллы, которых у профилей нет.
#
# Проверяем (всё в temp-директориях, без трогания реального /home/builder):
#   1. --list-skills печатает allowlist;
#   2. --dry-run не создаёт файлов;
#   3. реальный запуск доставляет SKILL.md во все профили (content match);
#   4. повторный запуск идемпотентен (content matches → skip);
#   5. отсутствующий source-скилл → exit 2.
#
# Invocation:
#   bash scripts/agent_flow/tests/test_sync_skills.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYNC="$TEST_DIR/../sync-skills.sh"

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

fail() { echo "FAIL: $*"; exit 1; }
pass() { echo "  PASS: $*"; }

# --- фейковый repo с source-скиллами ---------------------------------------
REPO="$WORK/repo"
mkdir -p "$REPO/.agents/skills"
for s in systematic-debugging test-driven-development codebase-design \
         verification-before-completion agent-flow \
         code-review to-tickets resolving-merge-conflicts ponytail; do
    mkdir -p "$REPO/.agents/skills/$s"
    printf -- '---\nname: %s\n---\n# %s body\n' "$s" "$s" > "$REPO/.agents/skills/$s/SKILL.md"
done

# --- фейковый HERMES_HOME с двумя профилями --------------------------------
HERMES="$WORK/hermes"
mkdir -p "$HERMES/profiles/backend/skills"
mkdir -p "$HERMES/profiles/devops/skills"

run_sync() {  # $@ = extra args
    REPO_DIR="$REPO" HERMES_HOME="$HERMES" SKILL_SYNC_PROFILES="backend devops" \
        bash "$SYNC" "$@" 2>&1
}

# TEST 1: --list-skills печатает allowlist ----------------------------------
OUT1=$(REPO_DIR="$REPO" bash "$SYNC" --list-skills 2>&1)
echo "TEST1 (--list-skills):"; echo "$OUT1"
echo "$OUT1" | grep -q "systematic-debugging" || fail "allowlist missing systematic-debugging"
echo "$OUT1" | grep -q "test-driven-development" || fail "allowlist missing test-driven-development"
echo "$OUT1" | grep -q "codebase-design" || fail "allowlist missing codebase-design"
echo "$OUT1" | grep -q "code-review" || fail "allowlist missing code-review"
echo "$OUT1" | grep -q "ponytail" || fail "allowlist missing ponytail"
pass "list-skills"

# TEST 2: dry-run не создаёт файлов -----------------------------------------
run_sync --dry-run >/dev/null 2>&1
[ ! -e "$HERMES/profiles/backend/skills/repo/systematic-debugging/SKILL.md" ] \
    || fail "dry-run created files in backend profile"
pass "dry-run is side-effect free"

# TEST 3: реальный запуск доставляет скиллы ---------------------------------
OUT3=$(run_sync)
echo "TEST3 (real run):"; echo "$OUT3"
[ -f "$HERMES/profiles/backend/skills/repo/systematic-debugging/SKILL.md" ] \
    || fail "backend: systematic-debugging not delivered"
[ -f "$HERMES/profiles/backend/skills/repo/agent-flow/SKILL.md" ] \
    || fail "backend: agent-flow not delivered"
[ -f "$HERMES/profiles/devops/skills/repo/codebase-design/SKILL.md" ] \
    || fail "devops: codebase-design not delivered"
[ -f "$HERMES/profiles/backend/skills/repo/code-review/SKILL.md" ] \
    || fail "backend: code-review not delivered"
[ -f "$HERMES/profiles/devops/skills/repo/ponytail/SKILL.md" ] \
    || fail "devops: ponytail not delivered"
cmp -s "$HERMES/profiles/backend/skills/repo/systematic-debugging/SKILL.md" \
       "$REPO/.agents/skills/systematic-debugging/SKILL.md" \
    || fail "delivered SKILL.md differs from source"
pass "skills delivered to backend + devops (content matches source)"

# TEST 4: повторный запуск идемпотентен -------------------------------------
OUT4=$(run_sync)
echo "TEST4 (idempotent):"; echo "$OUT4"
echo "$OUT4" | grep -q "content matches" || fail "second run not idempotent (expected 'content matches')"
pass "idempotent second run"

# TEST 5: отсутствующий source-скилл → exit 2 -------------------------------
BAD="$WORK/badrepo"
mkdir -p "$BAD/.agents/skills"
REPO_DIR="$BAD" HERMES_HOME="$HERMES" SKILL_SYNC_PROFILES="backend" \
    bash "$SYNC" >/dev/null 2>&1
rc=$?
[ "$rc" -eq 2 ] || fail "expected exit 2 for missing source, got $rc"
pass "missing source skill -> exit 2"

echo
echo "ALL TESTS PASSED"
exit 0
