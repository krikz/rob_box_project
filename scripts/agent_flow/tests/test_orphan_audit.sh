#!/bin/bash
# ============================================================================
# Test: agent-flow-orphan-audit.sh — основные сценарии
#
# Покрывает:
#   1. dry-run: exit 0, не пишет state, не пишет alerts.
#   2. Реальный прогон на тестовой in-memory БД: 2+ тёзок → 1 alert +
#      N events; 1 тёзка → skip.
#   3. Idempotency: повтор → "skipped_cooldown".
#   4. Cooldown override: ORPHAN_COOLDOWN_SECS=0 → повтор alertится.
#   5. Метрика: textfile содержит expected строки.
#
# Не делает e2e на реальном GH API — только локальные БД.
# ============================================================================
set -euo pipefail

TEST_TMP="$(mktemp -d -t orphan_audit_test.XXXXXX)"
trap 'rm -rf "$TEST_TMP" 2>/dev/null || true' EXIT

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)"
AUDIT_SH="$SCRIPT_DIR/agent-flow-orphan-audit.sh"

if [ ! -x "$AUDIT_SH" ]; then
    echo "FAIL: $AUDIT_SH not executable"
    exit 1
fi

# Build fixture kanban.db with a synthetic orphan-group
FIXTURE_DB="$TEST_TMP/test_kanban.db"
python3 - "$FIXTURE_DB" <<'PYEOF'
import sqlite3, sys, time
db = sys.argv[1]
conn = sqlite3.connect(db)
c = conn.cursor()
c.executescript("""
CREATE TABLE tasks (
    id TEXT PRIMARY KEY, title TEXT NOT NULL, status TEXT NOT NULL,
    body TEXT, assignee TEXT, branch_name TEXT,
    workspace_kind TEXT NOT NULL DEFAULT 'scratch',
    workspace_path TEXT, created_at INTEGER NOT NULL, started_at INTEGER,
    completed_at INTEGER, current_run_id INTEGER, project_id TEXT,
    consecutive_failures INTEGER NOT NULL DEFAULT 0
);
""")
now = int(time.time())

# Группа 1: 3 карточки на issue #2406 (orphans)
for i, t in enumerate(["t_test_aaa", "t_test_bbb", "t_test_ccc"]):
    c.execute(
        "INSERT INTO tasks(id, title, status, body, created_at, started_at) VALUES (?, ?, 'ready', ?, ?, ?)",
        (t, f"test {t}", "fix(prompt): orphan test #2406 in krikz/rob_box_project", now, now - i),
    )

# Группа 2: 2 карточки на issue #9999 (тоже orphans)
for t in ["t_test_ddd", "t_test_eee"]:
    c.execute(
        "INSERT INTO tasks(id, title, status, body, created_at, started_at) VALUES (?, ?, 'blocked', ?, ?, ?)",
        (t, f"test {t}", "another fix for #9999 (krikz/rob_box_project)", now, now),
    )

# Single (должна быть пропущена)
c.execute(
    "INSERT INTO tasks(id, title, status, body, created_at, started_at) VALUES (?, ?, 'ready', ?, ?, ?)",
    ("t_test_single", "test single", "alone for #1234", now, now),
)

# Done-карточка (должна быть пропущена)
c.execute(
    "INSERT INTO tasks(id, title, status, body, created_at, started_at, completed_at) VALUES (?, ?, 'done', ?, ?, ?, ?)",
    ("t_test_done", "test done", "done for #2406 (krikz/rob_box_project)", now, now, now),
)

# task_events table (for write_card_events)
c.execute(
    "CREATE TABLE IF NOT EXISTS task_events ("
    "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
    "  task_id TEXT NOT NULL, run_id INTEGER, kind TEXT NOT NULL,"
    "  payload TEXT, created_at INTEGER NOT NULL"
    ")"
)

conn.commit()
conn.close()
print(f"fixture built: {db}")
PYEOF

export GH_CONFIG_DIR=/home/builder/.config/gh

# Helper для запуска audit
audit_run() {
    local cooldown="${1:-3600}"
    KANBAN_DB="$FIXTURE_DB" \
    ORPHAN_STATE_FILE="$TEST_TMP/state.sqlite" \
    ORPHAN_ALERT_LOG="$TEST_TMP/alerts.log" \
    ORPHAN_METRICS_FILE="$TEST_TMP/metrics.prom" \
    ORPHAN_COOLDOWN_SECS="$cooldown" \
    ORPHAN_GH_REPO_FALLBACK="" \
    GH_REPO="" \
    bash "$AUDIT_SH" 2>&1 | grep -E '^__STATS__:'
}

# === Test 1: first run ===
STATS1="$(audit_run 3600)"
echo "[test1] first run: $STATS1"
if [ "$STATS1" != "__STATS__:6:2:2:0:0:5" ]; then
    echo "FAIL: expected __STATS__:6:2:2:0:0:5, got $STATS1"
    cat "$TEST_TMP/alerts.log" 2>&1
    exit 1
fi
echo "[test1] OK"

# Verify alerts file has exactly 2 lines
ALERT_COUNT="$(wc -l < "$TEST_TMP/alerts.log")"
if [ "$ALERT_COUNT" != "2" ]; then
    echo "FAIL: expected 2 alert lines, got $ALERT_COUNT"
    cat "$TEST_TMP/alerts.log"
    exit 1
fi
echo "[test1] alert count OK: $ALERT_COUNT"

# Verify alert line format
grep -qE '^ORPHAN_ALERT issue=#2406 count=3 cards=\[t_test_[abc]{3},t_test_[abc]{3},t_test_[abc]{3}\] merged_prs=\[\] recommendation=warn$' "$TEST_TMP/alerts.log" \
    || { echo "FAIL: alert line format wrong for #2406"; cat "$TEST_TMP/alerts.log"; exit 1; }
echo "[test1] alert line format OK"

# Verify task_events inserted (5 events: 3 for #2406 + 2 for #9999)
EVENT_COUNT="$(python3 -c "import sqlite3; c=sqlite3.connect('$FIXTURE_DB').cursor(); c.execute(\"SELECT COUNT(*) FROM task_events WHERE kind='orphan_detected'\"); print(c.fetchone()[0])")"
if [ "$EVENT_COUNT" != "5" ]; then
    echo "FAIL: expected 5 orphan_detected events, got $EVENT_COUNT"
    exit 1
fi
echo "[test1] event count OK: $EVENT_COUNT"

# Verify metrics file
grep -qE '^agent_flow_orphan_cards_total 5$' "$TEST_TMP/metrics.prom" \
    || { echo "FAIL: metric orphan_cards_total wrong"; cat "$TEST_TMP/metrics.prom"; exit 1; }
grep -qE '^agent_flow_orphan_groups_total 2$' "$TEST_TMP/metrics.prom" \
    || { echo "FAIL: metric orphan_groups_total wrong"; cat "$TEST_TMP/metrics.prom"; exit 1; }
echo "[test1] metrics file OK"

# === Test 2: idempotency — second run within cooldown ===
STATS2="$(audit_run 3600)"
echo "[test2] second run: $STATS2"
if [ "$STATS2" != "__STATS__:6:2:0:2:0:5" ]; then
    echo "FAIL: expected __STATS__:6:2:0:2:0:5 (skipped_cooldown=2), got $STATS2"
    exit 1
fi
ALERT_COUNT2="$(wc -l < "$TEST_TMP/alerts.log")"
if [ "$ALERT_COUNT2" != "2" ]; then
    echo "FAIL: expected 2 alerts (no dup), got $ALERT_COUNT2"
    exit 1
fi
EVENT_COUNT2="$(python3 -c "import sqlite3; c=sqlite3.connect('$FIXTURE_DB').cursor(); c.execute(\"SELECT COUNT(*) FROM task_events WHERE kind='orphan_detected'\"); print(c.fetchone()[0])")"
if [ "$EVENT_COUNT2" != "5" ]; then
    echo "FAIL: expected still 5 events (no dup), got $EVENT_COUNT2"
    exit 1
fi
echo "[test2] idempotency OK"

# === Test 3: cooldown=0 → повторный alert allowed ===
STATS3="$(audit_run 0)"
echo "[test3] third run with cooldown=0: $STATS3"
if [ "$STATS3" != "__STATS__:6:2:2:0:0:5" ]; then
    echo "FAIL: expected __STATS__:6:2:2:0:0:5, got $STATS3"
    exit 1
fi
ALERT_COUNT3="$(wc -l < "$TEST_TMP/alerts.log")"
if [ "$ALERT_COUNT3" != "4" ]; then
    echo "FAIL: expected 4 alerts (2 groups × 2 runs), got $ALERT_COUNT3"
    exit 1
fi
echo "[test3] cooldown=0 OK"

# === Test 4: dry-run mode ===
DRYOUT="$(KANBAN_DB="$FIXTURE_DB" \
    ORPHAN_STATE_FILE="$TEST_TMP/state_dry.sqlite" \
    ORPHAN_ALERT_LOG="$TEST_TMP/alerts_dry.log" \
    ORPHAN_METRICS_FILE="$TEST_TMP/metrics_dry.prom" \
    ORPHAN_COOLDOWN_SECS=3600 \
    bash "$AUDIT_SH" --dry-run 2>&1 | tail -1)"
echo "[test4] dry-run tail: $DRYOUT"
if [ -f "$TEST_TMP/alerts_dry.log" ]; then
    echo "FAIL: dry-run should not write alerts.log"
    cat "$TEST_TMP/alerts_dry.log"
    exit 1
fi
if [ -f "$TEST_TMP/metrics_dry.prom" ]; then
    echo "FAIL: dry-run should not write metrics"
    cat "$TEST_TMP/metrics_dry.prom"
    exit 1
fi
echo "[test4] dry-run OK"

# === Test 5: пустая БД (никаких активных карточек) ===
EMPTY_DB="$TEST_TMP/empty.db"
python3 - "$EMPTY_DB" <<'PYEOF'
import sqlite3, sys
db = sys.argv[1]
conn = sqlite3.connect(db)
c = conn.cursor()
c.executescript("""
CREATE TABLE tasks (
    id TEXT PRIMARY KEY, title TEXT NOT NULL, status TEXT NOT NULL,
    body TEXT, assignee TEXT, branch_name TEXT,
    workspace_kind TEXT NOT NULL DEFAULT 'scratch',
    workspace_path TEXT, created_at INTEGER NOT NULL, started_at INTEGER,
    completed_at INTEGER, current_run_id INTEGER, project_id TEXT,
    consecutive_failures INTEGER NOT NULL DEFAULT 0
);
CREATE TABLE task_events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id TEXT NOT NULL, run_id INTEGER, kind TEXT NOT NULL,
    payload TEXT, created_at INTEGER NOT NULL
);
""")
conn.commit()
conn.close()
PYEOF
STATS5="$(KANBAN_DB="$EMPTY_DB" \
    ORPHAN_STATE_FILE="$TEST_TMP/state_empty.sqlite" \
    ORPHAN_ALERT_LOG="$TEST_TMP/alerts_empty.log" \
    ORPHAN_METRICS_FILE="$TEST_TMP/metrics_empty.prom" \
    ORPHAN_COOLDOWN_SECS=3600 \
    bash "$AUDIT_SH" 2>&1 | tail -1)"
echo "[test5] empty db: $STATS5"
if [ "$STATS5" != "__STATS__:0:0:0:0:0:0" ]; then
    echo "FAIL: expected __STATS__:0:0:0:0:0:0, got $STATS5"
    exit 1
fi
echo "[test5] empty db OK"

echo "ALL TESTS PASSED"
exit 0