#!/bin/bash
# ============================================================================
# test_decomposed_watchdog.sh — регресс-гард для
# agent-flow-decomposed-watchdog.sh (ADR-0052, nightly-review t_bfd19ffb).
#
# Тестируем чистую логику (без hermes CLI side-effects) через PATH-hijack:
# подставляем mock-hermes, который пишет в JOURNAL и обновляет task_comments
# через sqlite. SQLite DB — настоящая (через python sqlite3 модуль), чтобы
# тест работал с реальным контрактом схемы (task_events + tasks + task_comments).
#
# Scenarios (acceptance criteria ADR-0052 §6 #1):
#   T1 criteria_match:       decomposed 30ч назад, child started_at=NULL,
#                            status=todo, root live → comment + priority++
#   T2 idempotency:          2 тика подряд → один comment в task_comments,
#                            priority bump только ОДИН раз
#   T3 triage_child:         decomposed 30ч назад, child status=triage
#                            (патология AV-11 t_08288c77) → match
#   T4 null_started_recent:  decomposed 1ч назад (<24h threshold) →
#                            НЕТ comment (нормальный pickup-лаг)
#   T5 root_done:            decomposed 100ч назад, но root status='done'
#                            → НЕТ comment (dead root, нет смысла будить)
#
# Run:
#   bash scripts/agent_flow/tests/test_decomposed_watchdog.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WATCHDOG_SH="${WATCHDOG_SH:-$TEST_DIR/../agent-flow-decomposed-watchdog.sh}"

[ -f "$WATCHDOG_SH" ] || { echo "FAIL: $WATCHDOG_SH not found"; exit 1; }
command -v bash >/dev/null || { echo "FAIL: bash required"; exit 1; }
command -v python3 >/dev/null || { echo "FAIL: python3 required (sqlite3 + JSON)"; exit 1; }

# Каждый тест — fresh WORK с fresh mock-DB. Не модифицируем реальный kanban.db.
seed_db() {
    local db_path="$1"
    local root_id="$2"
    local child_id="$3"
    local root_status="$4"
    local child_status="$5"
    local decomposed_hours_ago="$6"
    local priority="$7"
    local now
    now="$(date -u +%s)"
    local decomposed_at=$(( now - decomposed_hours_ago * 3600 ))

    python3 - "$db_path" "$root_id" "$child_id" "$root_status" "$child_status" "$decomposed_at" "$priority" "$now" <<'PYEOF'
import sqlite3, json, sys
db, root_id, child_id, root_status, child_status, decomposed_at, priority, now = sys.argv[1:9]
priority = int(priority); decomposed_at = int(decomposed_at); now = int(now)

con = sqlite3.connect(db)
con.executescript('''
CREATE TABLE tasks (
    id TEXT PRIMARY KEY,
    title TEXT,
    body TEXT,
    assignee TEXT,
    status TEXT NOT NULL,
    priority INTEGER DEFAULT 0,
    created_at INTEGER NOT NULL,
    started_at INTEGER,
    completed_at INTEGER,
    workspace_kind TEXT NOT NULL DEFAULT 'scratch'
);
CREATE TABLE task_events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id TEXT NOT NULL,
    run_id INTEGER,
    kind TEXT NOT NULL,
    payload TEXT,
    created_at INTEGER NOT NULL
);
CREATE TABLE task_comments (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id TEXT NOT NULL,
    author TEXT NOT NULL,
    body TEXT NOT NULL,
    created_at INTEGER NOT NULL
);
''')
con.execute("INSERT INTO tasks (id, title, assignee, status, priority, created_at, started_at) "
            "VALUES (?, 'root epic', 'agent-flow', ?, ?, ?, NULL)",
            (root_id, root_status, priority, decomposed_at - 86400))
con.execute("INSERT INTO tasks (id, title, assignee, status, priority, created_at, started_at) "
            "VALUES (?, 'child', 'tester', ?, ?, ?, NULL)",
            (child_id, child_status, priority, decomposed_at))
con.execute("INSERT INTO task_events (task_id, kind, payload, created_at) VALUES (?, 'decomposed', ?, ?)",
            (root_id, json.dumps({"child_ids": [child_id], "root_assignee": "agent-flow"}), decomposed_at))
con.commit()
con.close()
PYEOF
}

make_mock_hermes() {
    local work="$1"
    cat > "$work/bin/hermes" <<'EOF'
#!/bin/bash
# mock hermes — эмулирует РЕАЛЬНОЕ поведение `hermes kanban --board X
# comment TID BODY`: пишет строку в task_comments (через sqlite) и в
# JOURNAL (для визуального assert). Это даёт idempotency-проверке в
# watchdog реальные данные, на которых она работает.
if [ "${1:-}" = "kanban" ]; then
    shift
    _board=""; _cmd=""; _tid=""; _body=""
    while [ $# -gt 0 ]; do
        case "$1" in
            --board) _board="$2"; shift 2 ;;
            comment) _cmd="comment"; shift ;;
            *)
                if [ -z "$_tid" ]; then _tid="$1"
                elif [ -z "$_body" ]; then _body="$1"
                else _body="$_body $1"; fi
                shift ;;
        esac
    done
    if [ "$_cmd" = "comment" ] && [ -n "$_tid" ]; then
        echo "MOCKED: kanban comment board=${_board} task=${_tid}" >> "$HERMES_JOURNAL"
        if [ -n "${KANBAN_DB_PATH:-}" ]; then
            python3 - "$KANBAN_DB_PATH" "$_tid" "$_body" >>"$HERMES_JOURNAL" 2>&1 <<'PYEOF'
import sqlite3, sys, time
db, tid, body = sys.argv[1], sys.argv[2], sys.argv[3]
con = sqlite3.connect(db)
con.execute(
    "INSERT INTO task_comments (task_id, author, body, created_at) VALUES (?, ?, ?, ?)",
    (tid, "agent-flow-bot", body, int(time.time())),
)
con.commit()
con.close()
PYEOF
        fi
        exit 0
    fi
fi
exit 0
EOF
    chmod +x "$work/bin/hermes"
}

count_comments() {
    local db="$1" tid="$2"
    python3 -c "
import sqlite3
con = sqlite3.connect('$db')
print(con.execute('SELECT COUNT(*) FROM task_comments WHERE task_id=\"$tid\"').fetchone()[0])
con.close()
"
}

get_priority() {
    local db="$1" tid="$2"
    python3 -c "
import sqlite3
con = sqlite3.connect('$db')
print(con.execute('SELECT priority FROM tasks WHERE id=\"$tid\"').fetchone()[0])
con.close()
"
}

run_tick() {
    local work="$1" db="$2"
    (
        export PATH="$work/bin:$PATH"
        export KANBAN_DB_PATH="$db"
        export KANBAN_BOARD="test"
        bash "$WATCHDOG_SH" >/dev/null 2>&1
    )
}

# ============================================================================
# T1: criteria match — comment + priority++
# ============================================================================
test_criteria_match() {
    local WORK; WORK="$(mktemp -d)"
    mkdir -p "$WORK/bin"
    local db="$WORK/kanban/test.db"
    mkdir -p "$(dirname "$db")"
    seed_db "$db" "t_root_a" "t_child_a" "todo" "todo" 30 5
    make_mock_hermes "$WORK"
    export HERMES_JOURNAL="$WORK/journal.txt"; : > "$HERMES_JOURNAL"

    run_tick "$WORK" "$db"

    local comments; comments="$(count_comments "$db" "t_child_a")"
    local prio;     prio="$(get_priority "$db" "t_child_a")"

    # cleanup
    rm -rf "$WORK"

    if [ "$comments" -eq 1 ] && [ "$prio" -eq 6 ]; then
        echo "  ok: criteria_match (comments=1 priority=5→6)"
        return 0
    fi
    echo "  FAIL: criteria_match (comments=$comments expected 1, priority=$prio expected 6)"
    return 1
}

# ============================================================================
# T2: idempotency — 2 тика подряд → один comment + priority bump только ОДИН раз
# ============================================================================
test_idempotency() {
    local WORK; WORK="$(mktemp -d)"
    mkdir -p "$WORK/bin"
    local db="$WORK/kanban/test.db"
    mkdir -p "$(dirname "$db")"
    seed_db "$db" "t_root_b" "t_child_b" "todo" "todo" 30 0
    make_mock_hermes "$WORK"
    export HERMES_JOURNAL="$WORK/journal.txt"; : > "$HERMES_JOURNAL"

    run_tick "$WORK" "$db"
    run_tick "$WORK" "$db"

    local comments; comments="$(count_comments "$db" "t_child_b")"
    local prio;     prio="$(get_priority "$db" "t_child_b")"

    rm -rf "$WORK"

    if [ "$comments" -eq 1 ] && [ "$prio" -eq 1 ]; then
        echo "  ok: idempotency (2 ticks → 1 comment, priority 0→1)"
        return 0
    fi
    echo "  FAIL: idempotency (comments=$comments expected 1, priority=$prio expected 1)"
    return 1
}

# ============================================================================
# T3: child status=triage (патология AV-11) → match
# ============================================================================
test_triage_child() {
    local WORK; WORK="$(mktemp -d)"
    mkdir -p "$WORK/bin"
    local db="$WORK/kanban/test.db"
    mkdir -p "$(dirname "$db")"
    seed_db "$db" "t_root_c" "t_child_c" "todo" "triage" 30 0
    make_mock_hermes "$WORK"
    export HERMES_JOURNAL="$WORK/journal.txt"; : > "$HERMES_JOURNAL"

    run_tick "$WORK" "$db"

    local comments; comments="$(count_comments "$db" "t_child_c")"

    rm -rf "$WORK"

    if [ "$comments" -eq 1 ]; then
        echo "  ok: triage_child (triage-статус ребёнка — патология AV-11, comment есть)"
        return 0
    fi
    echo "  FAIL: triage_child (comments=$comments expected 1)"
    return 1
}

# ============================================================================
# T4: свежий decomposed (1ч назад, <24h threshold) → НЕТ comment
# ============================================================================
test_recent_no_comment() {
    local WORK; WORK="$(mktemp -d)"
    mkdir -p "$WORK/bin"
    local db="$WORK/kanban/test.db"
    mkdir -p "$(dirname "$db")"
    seed_db "$db" "t_root_d" "t_child_d" "todo" "todo" 1 0
    make_mock_hermes "$WORK"
    export HERMES_JOURNAL="$WORK/journal.txt"; : > "$HERMES_JOURNAL"

    run_tick "$WORK" "$db"

    local comments; comments="$(count_comments "$db" "t_child_d")"
    local prio;     prio="$(get_priority "$db" "t_child_d")"

    rm -rf "$WORK"

    if [ "$comments" -eq 0 ] && [ "$prio" -eq 0 ]; then
        echo "  ok: recent_no_comment (1ч < 24h threshold, нет side-effects)"
        return 0
    fi
    echo "  FAIL: recent_no_comment (comments=$comments expected 0, priority=$prio expected 0)"
    return 1
}

# ============================================================================
# T5: root status=done → НЕТ comment (dead root)
# ============================================================================
test_root_done_no_comment() {
    local WORK; WORK="$(mktemp -d)"
    mkdir -p "$WORK/bin"
    local db="$WORK/kanban/test.db"
    mkdir -p "$(dirname "$db")"
    seed_db "$db" "t_root_e" "t_child_e" "done" "todo" 100 0
    make_mock_hermes "$WORK"
    export HERMES_JOURNAL="$WORK/journal.txt"; : > "$HERMES_JOURNAL"

    run_tick "$WORK" "$db"

    local comments; comments="$(count_comments "$db" "t_child_e")"
    local prio;     prio="$(get_priority "$db" "t_child_e")"

    rm -rf "$WORK"

    if [ "$comments" -eq 0 ] && [ "$prio" -eq 0 ]; then
        echo "  ok: root_done_no_comment (dead root, нет смысла будить)"
        return 0
    fi
    echo "  FAIL: root_done_no_comment (comments=$comments expected 0, priority=$prio expected 0)"
    return 1
}

# --- registry -------------------------------------------------------------
FAILED=0
for tn in test_criteria_match test_idempotency test_triage_child test_recent_no_comment test_root_done_no_comment; do
    if "$tn"; then
        : # already echoed ok
    else
        FAILED=$(( FAILED + 1 ))
    fi
done
unset HERMES_JOURNAL KANBAN_DB_PATH KANBAN_BOARD

if [ "$FAILED" -gt 0 ]; then
    echo "FAIL: $FAILED тестов упало"
    exit 1
fi
echo "ok: decomposed-watchdog: все 5 кейсов прошли"
exit 0
