#!/bin/bash
# ============================================================================
# test_scope_hint_force.sh — CLI smoke-test for the --force-scope escape
# hatch (ADR-0036 §4.1, sibling of t_aa585aa7 / t_369e7fca).
#
# ADR-0036 §4.1 wires a heuristic into `hermes kanban create`: when the
# assignee is an implementation profile (backend / frontend / tester) AND
# the body carries an architectural keyword (ADR-, merge-gate, dispatcher,
# process-fix, ...) AND a TDD-shaped skill is pinned (test-driven-development,
# pytest, jest), the CLI prints a structured stderr hint AND records a
# `scope_hint` task_event row on the card. The hint is advisory only;
# Шифу can pass `--force-scope` to suppress BOTH side-effects.
#
# This script is the e2e CLI counterpart to
# `scripts/agent_flow/tests/e2e_skill_validation.py` Case C
# (test_scope_force_override). The Python e2e exercises the heuristic
# directly through hermes_cli.kanban_db._validate_scope_for_assignee;
# this bash script exercises the FULL CLI path (argparse → create_task
# → scope_hint side-effects) against a real hermes binary.
#
# Acceptance criteria (card t_275dfb82, body: "should be silent"):
#
#   F1. Without --force-scope, the CLI prints the stderr hint AND records
#       a scope_hint task_event row. (Regression guard: heuristic still
#       wired and firing on the unforced path.)
#   F2. With --force-scope, the CLI is SILENT on stderr (no scope-hint
#       warning) AND records NO scope_hint task_event row. (The headline
#       acceptance for t_275dfb82.)
#   F3. With --force-scope, the task is STILL created (--force-scope
#       suppresses the hint, not the task). (Escape hatch is not a
#       create-block; matching ADR-0036 §5 explicit tradeoff.)
#   F4. For an architectural owner (assignee=architect) the CLI is silent
#       regardless of --force-scope, and no scope_hint event is recorded.
#       (Regression guard: heuristic only fires on implementation profiles.)
#
# Run:
#   bash scripts/agent_flow/tests/test_scope_hint_force.sh
#
# Required env:
#   HERMES_BIN    path to the hermes binary (default: $HOME/.hermes/hermes-agent/venv/bin/hermes)
#
# The script isolates state via HERMES_KANBAN_DB (per-call DB override,
# honoured by hermes_cli.kanban_db.kanban_db_path) so it never touches
# the operator's real kanban.db.
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

HERMES_BIN="${HERMES_BIN:-/home/builder/.hermes/hermes-agent/venv/bin/hermes}"

[ -x "$HERMES_BIN" ] || { echo "FAIL: hermes binary not executable at $HERMES_BIN"; exit 1; }
command -v bash >/dev/null || { echo "FAIL: bash required"; exit 1; }
command -v python3 >/dev/null || { echo "FAIL: python3 required (sqlite3 + JSON)"; exit 1; }

# ---------------------------------------------------------------------------
# TINY-SQL fixture — see test_blocked_watchdog_scope.sh for the same pattern.
# We do NOT need to recreate the full kanban schema here: hermes CLI calls
# `kanban init` (which creates the schema idempotently) and `kanban boards
# create` before we run the create command. The schema is created by the
# real hermes binary, then we read it back with python3.
# ---------------------------------------------------------------------------
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

DB="$WORK/kanban.db"
BOARD="scope-force-smoke-$$"

export HERMES_KANBAN_DB="$DB"

# Create the board via the real CLI (idempotent init + boards create).
"$HERMES_BIN" kanban --board "$BOARD" init >/dev/null 2>&1 || true
"$HERMES_BIN" kanban --board "$BOARD" boards create "$BOARD" >/dev/null 2>&1 || true

# A body that lights up EVERY keyword in ADR-0036 §4.1's heuristic:
# assignee=backend ∈ NON_ARCHITECT_PROFILES, body contains "ADR-",
# skill = test-driven-development ∈ _TEST_SKILL_MARKERS. Triple-positive.
# Any of those single fails would make the heuristic skip; we need all three
# so the test is a clean positive AND so removing --force-scope flips it
# back to a hard RED.
TRIPLE_POSITIVE_BODY="Проектирую ADR-9999 — pre-merge gate через dispatcher с process-fix архитектурой; merge-gate shell, тест не нужен"

run_create() {
    # $1 = whether to pass --force-scope (true|false)
    # $2 = assignee (backend|architect|...)
    # $3 = title suffix (so tasks differ when we count events)
    # $4 = body
    # $5 = skill (repeatable, just one for clarity)
    local force="$1" assignee="$2" suffix="$3" body="$4" skill="$5"
    local title="smoke-scope-hint-forced-${suffix}"
    local stderr_file="$WORK/stderr_${suffix}.txt"
    local stdout_file="$WORK/stdout_${suffix}.txt"
    local force_flag=""
    [ "$force" = "true" ] && force_flag="--force-scope"
    "$HERMES_BIN" kanban --board "$BOARD" create "$title" \
        --body "$body" --assignee "$assignee" --skill "$skill" $force_flag \
        >"$stdout_file" 2>"$stderr_file"
    echo "$?"
}

count_scope_hint_events() {
    # Print the number of `scope_hint` rows in task_events. Used to
    # distinguish F1 (≥1) vs F2/F3 (==0). Read the schema produced by the
    # real hermes binary; do NOT recreate the schema by hand here.
    python3 - "$DB" <<'PYEOF'
import sqlite3, sys
con = sqlite3.connect(sys.argv[1])
row = con.execute(
    "SELECT COUNT(*) FROM task_events WHERE kind='scope_hint'"
).fetchone()
print(row[0])
con.close()
PYEOF
}

assert_silent() {
    # $1 = suffix (which stderr to grep)
    # $2 = test name (for the failure message)
    local suffix="$1" name="$2"
    if grep -q "scope-hint" "$WORK/stderr_${suffix}.txt"; then
        echo "  FAIL: $name — stderr contains 'scope-hint' warning:"
        sed 's/^/    /' "$WORK/stderr_${suffix}.txt"
        return 1
    fi
    return 0
}

assert_hint_fired() {
    local suffix="$1" name="$2"
    if ! grep -q "scope-hint" "$WORK/stderr_${suffix}.txt"; then
        echo "  FAIL: $name — expected 'scope-hint' on stderr but it was silent:"
        sed 's/^/    /' "$WORK/stderr_${suffix}.txt"
        return 1
    fi
    return 0
}

# ---------------------------------------------------------------------------
# F1 — Without --force-scope the heuristic fires (regression baseline).
# ---------------------------------------------------------------------------
echo "F1: without --force-scope, heuristic fires (stderr + task_event)"
rc=$(run_create "false" "backend" "f1" "$TRIPLE_POSITIVE_BODY" "test-driven-development")
[ "$rc" -eq 0 ] || { echo "  FAIL: hermes exited $rc (expected 0)"; exit 1; }
assert_hint_fired "f1" "F1.stderr" || exit 1
n=$(count_scope_hint_events)
[ "$n" -ge 1 ] || { echo "  FAIL: F1.task_event — expected ≥1 scope_hint row, got $n"; exit 1; }
echo "  ok: F1 — stderr has hint + DB has $n scope_hint row(s)"

# ---------------------------------------------------------------------------
# F2 — With --force-scope the CLI is SILENT (headline acceptance).
# ---------------------------------------------------------------------------
echo "F2: with --force-scope, CLI is silent (no stderr warning)"
n_before_f2=$(count_scope_hint_events)
rc=$(run_create "true" "backend" "f2" "$TRIPLE_POSITIVE_BODY" "test-driven-development")
[ "$rc" -eq 0 ] || { echo "  FAIL: hermes exited $rc (expected 0)"; exit 1; }
assert_silent "f2" "F2.stderr" || exit 1
n_after_f2=$(count_scope_hint_events)
delta_f2=$(( n_after_f2 - n_before_f2 ))
[ "$delta_f2" -eq 0 ] || { echo "  FAIL: F2.task_event — expected 0 NEW scope_hint rows from F2, got +$delta_f2 (total $n_after_f2)"; exit 1; }
echo "  ok: F2 — stderr silent + 0 new scope_hint rows (total $n_after_f2)"

# ---------------------------------------------------------------------------
# F3 — With --force-scope the TASK is still created (escape hatch is not
# a blocker — ADR-0036 §5).
# ---------------------------------------------------------------------------
echo "F3: with --force-scope, task is STILL created (escape hatch, not block)"
n_tasks=$(python3 - "$DB" <<'PYEOF'
import sqlite3, sys
con = sqlite3.connect(sys.argv[1])
row = con.execute("SELECT COUNT(*) FROM tasks").fetchone()
print(row[0])
con.close()
PYEOF
)
[ "$n_tasks" -ge 2 ] || { echo "  FAIL: F3.tasks — expected ≥2 tasks in DB, got $n_tasks"; exit 1; }
echo "  ok: F3 — $n_tasks task(s) in DB after F1+F2"

# ---------------------------------------------------------------------------
# F4 — For an architectural owner, CLI is silent regardless of --force-scope
# (heuristic only fires on implementation profiles).
# ---------------------------------------------------------------------------
echo "F4: assignee=architect — silent regardless of --force-scope"
n_before_f4=$(count_scope_hint_events)
rc=$(run_create "false" "architect" "f4" "$TRIPLE_POSITIVE_BODY" "architecture-doc-review")
[ "$rc" -eq 0 ] || { echo "  FAIL: F4 — hermes exited $rc (expected 0)"; exit 1; }
assert_silent "f4" "F4.stderr" || exit 1
n_after_f4=$(count_scope_hint_events)
delta_f4=$(( n_after_f4 - n_before_f4 ))
[ "$delta_f4" -eq 0 ] || { echo "  FAIL: F4.task_event — expected 0 NEW scope_hint rows from F4, got +$delta_f4 (total $n_after_f4)"; exit 1; }
echo "  ok: F4 — architect silent + 0 new scope_hint rows (total $n_after_f4)"

echo
echo "ok: scope-hint force-smoke — все 4 кейса прошли"
exit 0