#!/bin/bash
# ============================================================================
# test_stale_blocked_watchdog.sh — регресс-гард для
# agent-flow-stale-blocked-watchdog.sh (ретро 14.09 t_55c6c882).
#
# Тестируем чистую логику (без реальных hermes/gh side-effects) через
# PATH-hijack: подставляем mock-hermes (пишет в JOURNAL) и mock-gh
# (программируемые ответы per-PR через MOCK_GH_PRS_JSON).
#
# Scenarios:
#   S1. unmerged_pr_blocks_alert: PR #2385 в body, mock-gh возвращает
#       merged=false → SKIP, NO comment.
#   S2. merged_prs_all_parents_done_emits_alert: PR #2385+#2392 в body,
#       оба merged=true, parents все done → 1 comment emitted (marker
#       `⚠️ stale-blocked: prerequisites merged` в JOURNAL).
#   S3. parent_not_done_blocks_alert: PRs merged, но 1 parent in_progress →
#       SKIP, NO comment.
#   S4. not_a_pr_does_not_block_alert: PR #2385 merged, #9999 — issue →
#       mock-gh вернёт 404 для #9999. SKIP-logy "no_real_prs" (только issue
#       refs) → NO comment. (NB: это edge case — если ALL refs это issues,
#       без единого PR — не alert'им).
#   S5. not_a_pr_alongside_real_pr_still_alerts: PR #2385 merged, #9999
#       issue (404) → match (есть реальный merged PR), 1 comment.
#   S6. idempotent: 2 ticks подряд → 1 comment в JOURNAL.
#   S7. no_pr_refs_in_body_or_block_reason: blocked карточка без PR# →
#       SKIP "no_pr_ref".
#   S8. dry_run_no_comment: DRY_RUN=true → 1 record in __RECORD__, 0 в
#       JOURNAL (no actual side-effect).
#
# Run:
#   bash scripts/agent_flow/tests/test_stale_blocked_watchdog.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WATCHDOG_SH="${WATCHDOG_SH:-$TEST_DIR/../agent-flow-stale-blocked-watchdog.sh}"

[ -f "$WATCHDOG_SH" ] || { echo "FAIL: $WATCHDOG_SH not found (RED — скрипт не написан)"; exit 1; }
command -v bash >/dev/null || { echo "FAIL: bash required"; exit 1; }
command -v python3 >/dev/null || { echo "FAIL: python3 required (sqlite3 + JSON)"; exit 1; }

_pass=0
_fail=0

pass() { echo "  ok: $1"; _pass=$((_pass+1)); }
fail() { echo "  FAIL: $1"; _fail=$((_fail+1)); }

run_test() {
    local name="$1"
    WORK="$(mktemp -d)"
    export WORK

    mkdir -p "$WORK/bin" "$WORK/kanban"

    # Минимальная схема kanban, используемая watchdog'ом.
    python3 - <<PYEOF
import sqlite3
con = sqlite3.connect('$WORK/kanban/test.db')
con.executescript('''
CREATE TABLE tasks (
    id TEXT PRIMARY KEY,
    title TEXT,
    body TEXT,
    assignee TEXT,
    status TEXT NOT NULL,
    started_at INTEGER,
    max_runtime_seconds INTEGER,
    created_at INTEGER NOT NULL,
    workspace_kind TEXT NOT NULL DEFAULT 'scratch',
    session_id TEXT
);
CREATE TABLE task_comments (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id TEXT NOT NULL,
    author TEXT NOT NULL,
    body TEXT NOT NULL,
    created_at INTEGER NOT NULL
);
CREATE TABLE task_events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id TEXT NOT NULL,
    run_id INTEGER,
    kind TEXT NOT NULL,
    payload TEXT,
    created_at INTEGER NOT NULL,
    cost_in_cents REAL
);
CREATE TABLE task_links (
    parent_id TEXT NOT NULL,
    child_id TEXT NOT NULL,
    PRIMARY KEY (parent_id, child_id)
);
''')
con.commit()
con.close()
PYEOF

    export HERMES_JOURNAL="$WORK/journal.txt"
    : > "$HERMES_JOURNAL"

    # Mock-hermes: kanban comment → JOURNAL + запись в task_comments DB
    # (для проверки idempotency — S6 ожидает что 2-й tick не emit'ит второй
    # comment, потому что в DB уже есть marker за today).
    cat > "$WORK/bin/hermes" <<'EOF'
#!/bin/bash
# Mock-hermes (top-level script, no `local` keyword — используем простые var).
# Маршрутизация: фиксируем в журнал все вызовы kanban comment.
# Также пишем в task_comments DB для idempotency check.
case "${1:-}${2:-}${3:-}${4:-}" in
    *kanban*comment*)
        # argv: hermes kanban --board <board> comment <task_id> <body...>
        _arg_kanban="$1"
        _arg_board_flag="$2"  # --board
        _arg_board="$3"
        _arg_comment="$4"
        _arg_tid="$5"
        shift 5
        _body="$*"
        echo "MOCKED: kanban comment board=${_arg_board} task=${_arg_tid} body=${_body}" >> "$HERMES_JOURNAL"
        # DB write для idempotency: INSERT в task_comments с marker
        if [ -n "$KANBAN_DB_PATH" ] && [ -f "$KANBAN_DB_PATH" ]; then
            _now=$(date -u +%s)
            python3 - "$KANBAN_DB_PATH" "${_arg_tid}" "${_body}" "$_now" <<'PYEOF'
import sqlite3, sys
db_path, task_id, body, now = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
try:
    con = sqlite3.connect(db_path)
    con.execute("INSERT INTO task_comments (task_id, author, body, created_at) VALUES (?, 'mock-hermes', ?, ?)", (task_id, body, int(now)))
    con.commit()
    con.close()
except Exception as e:
    sys.stderr.write(f"mock-hermes DB write err: {e}\n")
PYEOF
        fi
        exit 0
        ;;
    *)
        # Неиспользуемые команды — пустая success.
        exit 0
        ;;
esac
EOF
    chmod +x "$WORK/bin/hermes"

    # Mock-gh: возвращаем JSON для каждого PR согласно MOCK_GH_PRS_JSON
    # (файл в формате {<pr_num>: {state, merged, merged_at}}).
    # Используем single-quoted heredoc + читаем путь из env-переменной
    # MOCK_GH_PRS_JSON (которая пересоздаётся в каждом test_* функции через
    # `export MOCK_GH_PRS_JSON=$WORK/gh_prs.json`). Так mock-gh работает с
    # актуальным путём даже если $WORK пересоздаётся между тестами.
    cat > "$WORK/bin/gh" <<'EOF'
#!/bin/bash
# gh api repos/.../pulls/<N> --jq <jq-filter>
# NOTE: top-level script (not function), no `local` keyword.
case "$1" in
    api)
        # Извлекаем номер PR из path (после pulls/).
        shift
        _path=""
        while [ $# -gt 0 ]; do
            case "$1" in
                --jq) shift; shift ;;  # skip jq expr
                --*) shift ;;
                repos/*) _path="$1"; shift ;;
                *) shift ;;
            esac
        done
        pr_num="${_path##*/pulls/}"
        # MOCK_GH_PRS_JSON env-переменная устанавливается test_* функцией
        # перед каждым запуском watchdog'а. Если не задана — fallback not_a_pr.
        mock_json="${MOCK_GH_PRS_JSON:-}"
        if [ -n "$mock_json" ] && [ -f "$mock_json" ]; then
            python3 - "$pr_num" "$mock_json" <<'PYEOF'
import json, sys
pr_num = sys.argv[1]
mock_json = sys.argv[2]
try:
    with open(mock_json) as f:
        d = json.load(f)
except Exception as e:
    print(json.dumps({"state": "error", "merged": False, "error": str(e)}))
    sys.exit(0)
if str(pr_num) in d:
    print(json.dumps(d[str(pr_num)]))
elif 'default_404' in d:
    print(json.dumps(d['default_404']))
else:
    print(json.dumps({'state': 'not_a_pr', 'merged': False}))
PYEOF
            exit 0
        fi
        # Fallback: 404 (not_a_pr)
        echo '{"state": "not_a_pr", "merged": false}'
        exit 0
        ;;
    auth)
        echo "✓ logged in"
        exit 0
        ;;
    *)
        exit 0
        ;;
esac
EOF
    chmod +x "$WORK/bin/gh"

    # Pre-populate DB сценария (вызывается ДО запуска watchdog'а в тестах).
    # Каждый тест вызывает этот хелпер из своего тела.

    (
        export PATH="$WORK/bin:$PATH"
        export KANBAN_DB_PATH="$WORK/kanban/test.db"
        export KANBAN_BOARD="test"
        bash "$WATCHDOG_SH" 2>/dev/null
    )
    local rc=$?
    echo "$rc" > "$WORK/rc.txt"
}

# ============================================================================
# S1: unmerged PR blocks alert
# ============================================================================
test_S1_unmerged_pr_blocks_alert() {
    run_test "S1_unmerged_pr_blocks_alert"

    python3 - <<PYEOF
import sqlite3, os, json, time
con = sqlite3.connect('$WORK/kanban/test.db')
now = int(time.time())
five_h_ago = now - 5 * 3600
con.execute("""INSERT INTO tasks (id, title, body, assignee, status, started_at, max_runtime_seconds, created_at)
VALUES ('t_stale_1', 'test', 'Body references PR #2385 only', 'default', 'blocked', ?, 1800, ?)""",
            (five_h_ago, five_h_ago))
con.execute("INSERT INTO task_links (parent_id, child_id) VALUES ('t_parent_done', 't_stale_1')")
con.execute("""INSERT INTO tasks (id, title, body, assignee, status, started_at, max_runtime_seconds, created_at)
VALUES ('t_parent_done', 'parent', 'done', 'default', 'done', ?, 1800, ?)""",
            (five_h_ago, five_h_ago))
con.commit()
con.close()
# Mock-gh: #2385 ещё open (merged=false)
with open('$WORK/gh_prs.json', 'w') as f:
    json.dump({'2385': {'state': 'open', 'merged': False, 'merged_at': None}}, f)
PYEOF

    export PATH="$WORK/bin:$PATH"
    export KANBAN_DB_PATH="$WORK/kanban/test.db"
    export KANBAN_BOARD="test"
    export MOCK_GH_PRS_JSON="$WORK/gh_prs.json"
    bash "$WATCHDOG_SH" 2>/dev/null

    local cnt
    cnt=$(grep -c "MOCKED:" "$HERMES_JOURNAL" 2>/dev/null || true)
    cnt="${cnt:-0}"
    # strip newlines (grep -c may produce "N\n" if file has multiple matches)
    cnt=$(echo "$cnt" | tr -d '\n')
    if [ "$cnt" -eq 0 ]; then
        pass "S1: unmerged PR blocks alert (no comment emitted)"
    else
        fail "S1: should NOT emit comment for unmerged PR, got $cnt"
        echo "  --- journal: $(cat $HERMES_JOURNAL)"
    fi
}

# ============================================================================
# S2: merged PRs + parents done → emit alert
# ============================================================================
test_S2_merged_prs_parents_done_emits_alert() {
    run_test "S2_merged_prs_parents_done_emits_alert"

    python3 - <<PYEOF
import sqlite3, time, json
con = sqlite3.connect('$WORK/kanban/test.db')
now = int(time.time())
five_h_ago = now - 5 * 3600
con.execute("""INSERT INTO tasks (id, title, body, assignee, status, started_at, max_runtime_seconds, created_at)
VALUES ('t_stale_2', 'test', 'Body references PR #2385 and PR #2392', 'default', 'blocked', ?, 1800, ?)""",
            (five_h_ago, five_h_ago))
con.execute("INSERT INTO task_links (parent_id, child_id) VALUES ('t_p2_done', 't_stale_2')")
con.execute("""INSERT INTO tasks (id, title, body, assignee, status, started_at, max_runtime_seconds, created_at)
VALUES ('t_p2_done', 'parent', 'done', 'default', 'done', ?, 1800, ?)""",
            (five_h_ago, five_h_ago))
con.commit()
con.close()
with open('$WORK/gh_prs.json', 'w') as f:
    json.dump({
        '2385': {'state': 'closed', 'merged': True, 'merged_at': '2026-09-14T11:24:31Z'},
        '2392': {'state': 'closed', 'merged': True, 'merged_at': '2026-09-14T11:16:10Z'},
    }, f)
PYEOF

    export PATH="$WORK/bin:$PATH"
    export KANBAN_DB_PATH="$WORK/kanban/test.db"
    export KANBAN_BOARD="test"
    export MOCK_GH_PRS_JSON="$WORK/gh_prs.json"
    bash "$WATCHDOG_SH" 2>/dev/null

    local cnt
    cnt=$(awk '/MOCKED:/ {c++} END {print c+0}' "$HERMES_JOURNAL" 2>/dev/null)
    if [ "$cnt" -eq 1 ]; then
        if grep -q "stale-blocked" "$HERMES_JOURNAL"; then
            pass "S2: merged PRs + parents done → emit alert (1 comment with marker)"
        else
            fail "S2: comment emitted but missing marker"
        fi
    else
        fail "S2: expected 1 comment, got $cnt"
        echo "  --- journal: $(cat $HERMES_JOURNAL)"
    fi
}

# ============================================================================
# S3: parent not done blocks alert
# ============================================================================
test_S3_parent_not_done_blocks_alert() {
    run_test "S3_parent_not_done_blocks_alert"

    python3 - <<PYEOF
import sqlite3, time, json
con = sqlite3.connect('$WORK/kanban/test.db')
now = int(time.time())
five_h_ago = now - 5 * 3600
con.execute("""INSERT INTO tasks (id, title, body, assignee, status, started_at, max_runtime_seconds, created_at)
VALUES ('t_stale_3', 'test', 'Body references PR #2385', 'default', 'blocked', ?, 1800, ?)""",
            (five_h_ago, five_h_ago))
con.execute("INSERT INTO task_links (parent_id, child_id) VALUES ('t_p3_inprog', 't_stale_3')")
con.execute("""INSERT INTO tasks (id, title, body, assignee, status, started_at, max_runtime_seconds, created_at)
VALUES ('t_p3_inprog', 'parent', 'in progress', 'default', 'in_progress', ?, 1800, ?)""",
            (five_h_ago, five_h_ago))
con.commit()
con.close()
with open('$WORK/gh_prs.json', 'w') as f:
    json.dump({'2385': {'state': 'closed', 'merged': True, 'merged_at': '2026-09-14T11:24:31Z'}}, f)
PYEOF

    export PATH="$WORK/bin:$PATH"
    export KANBAN_DB_PATH="$WORK/kanban/test.db"
    export KANBAN_BOARD="test"
    export MOCK_GH_PRS_JSON="$WORK/gh_prs.json"
    bash "$WATCHDOG_SH" 2>/dev/null

    local cnt
    cnt=$(awk '/MOCKED:/ {c++} END {print c+0}' "$HERMES_JOURNAL" 2>/dev/null)
    if [ "$cnt" -eq 0 ]; then
        pass "S3: parent in_progress blocks alert (no comment emitted)"
    else
        fail "S3: should NOT emit with unfinished parent, got $cnt"
        echo "  --- journal: $(cat $HERMES_JOURNAL)"
    fi
}

# ============================================================================
# S4: only issue refs (not_a_pr) → SKIP "no_real_prs"
# ============================================================================
test_S4_only_issue_refs_no_alert() {
    run_test "S4_only_issue_refs_no_alert"

    python3 - <<PYEOF
import sqlite3, time, json
con = sqlite3.connect('$WORK/kanban/test.db')
now = int(time.time())
five_h_ago = now - 5 * 3600
# #9999 — это issue (mock-gh вернёт 404 → not_a_pr)
con.execute("""INSERT INTO tasks (id, title, body, assignee, status, started_at, max_runtime_seconds, created_at)
VALUES ('t_stale_4', 'test', 'Body references #9999 only (issue, not PR)', 'default', 'blocked', ?, 1800, ?)""",
            (five_h_ago, five_h_ago))
con.execute("INSERT INTO task_links (parent_id, child_id) VALUES ('t_p4_done', 't_stale_4')")
con.execute("""INSERT INTO tasks (id, title, body, assignee, status, started_at, max_runtime_seconds, created_at)
VALUES ('t_p4_done', 'parent', 'done', 'default', 'done', ?, 1800, ?)""",
            (five_h_ago, five_h_ago))
con.commit()
con.close()
with open('$WORK/gh_prs.json', 'w') as f:
    # #9999 → 404 (default_404 не настроен → fallback not_a_pr)
    json.dump({}, f)
PYEOF

    export PATH="$WORK/bin:$PATH"
    export KANBAN_DB_PATH="$WORK/kanban/test.db"
    export KANBAN_BOARD="test"
    export MOCK_GH_PRS_JSON="$WORK/gh_prs.json"
    bash "$WATCHDOG_SH" 2>/dev/null

    local cnt
    cnt=$(awk '/MOCKED:/ {c++} END {print c+0}' "$HERMES_JOURNAL" 2>/dev/null)
    if [ "$cnt" -eq 0 ]; then
        pass "S4: only issue refs (not_a_pr) → no alert (no real PRs)"
    else
        fail "S4: should NOT emit with only issue refs, got $cnt"
        echo "  --- journal: $(cat $HERMES_JOURNAL)"
    fi
}

# ============================================================================
# S5: not_a_pr alongside real merged PR → emit alert
# ============================================================================
test_S5_not_a_pr_alongside_real_pr_emits() {
    run_test "S5_not_a_pr_alongside_real_pr_emits"

    python3 - <<PYEOF
import sqlite3, time, json
con = sqlite3.connect('$WORK/kanban/test.db')
now = int(time.time())
five_h_ago = now - 5 * 3600
con.execute("""INSERT INTO tasks (id, title, body, assignee, status, started_at, max_runtime_seconds, created_at)
VALUES ('t_stale_5', 'test', 'Body refs PR #2385 and #9999 (issue)', 'default', 'blocked', ?, 1800, ?)""",
            (five_h_ago, five_h_ago))
con.execute("INSERT INTO task_links (parent_id, child_id) VALUES ('t_p5_done', 't_stale_5')")
con.execute("""INSERT INTO tasks (id, title, body, assignee, status, started_at, max_runtime_seconds, created_at)
VALUES ('t_p5_done', 'parent', 'done', 'default', 'done', ?, 1800, ?)""",
            (five_h_ago, five_h_ago))
con.commit()
con.close()
with open('$WORK/gh_prs.json', 'w') as f:
    json.dump({'2385': {'state': 'closed', 'merged': True, 'merged_at': '2026-09-14T11:24:31Z'}}, f)
PYEOF

    export PATH="$WORK/bin:$PATH"
    export KANBAN_DB_PATH="$WORK/kanban/test.db"
    export KANBAN_BOARD="test"
    export MOCK_GH_PRS_JSON="$WORK/gh_prs.json"
    bash "$WATCHDOG_SH" 2>/dev/null

    local cnt
    cnt=$(awk '/MOCKED:/ {c++} END {print c+0}' "$HERMES_JOURNAL" 2>/dev/null)
    if [ "$cnt" -eq 1 ]; then
        pass "S5: real merged PR + issue ref → emit alert (1 comment)"
    else
        fail "S5: expected 1 comment, got $cnt"
        echo "  --- journal: $(cat $HERMES_JOURNAL)"
    fi
}

# ============================================================================
# S6: idempotency — 2 ticks подряд → 1 comment в JOURNAL
# ============================================================================
test_S6_idempotent() {
    run_test "S6_idempotent"

    python3 - <<PYEOF
import sqlite3, time, json
con = sqlite3.connect('$WORK/kanban/test.db')
now = int(time.time())
five_h_ago = now - 5 * 3600
con.execute("""INSERT INTO tasks (id, title, body, assignee, status, started_at, max_runtime_seconds, created_at)
VALUES ('t_stale_6', 'test', 'Body refs PR #2385', 'default', 'blocked', ?, 1800, ?)""",
            (five_h_ago, five_h_ago))
con.execute("INSERT INTO task_links (parent_id, child_id) VALUES ('t_p6_done', 't_stale_6')")
con.execute("""INSERT INTO tasks (id, title, body, assignee, status, started_at, max_runtime_seconds, created_at)
VALUES ('t_p6_done', 'parent', 'done', 'default', 'done', ?, 1800, ?)""",
            (five_h_ago, five_h_ago))
con.commit()
con.close()
with open('$WORK/gh_prs.json', 'w') as f:
    json.dump({'2385': {'state': 'closed', 'merged': True, 'merged_at': '2026-09-14T11:24:31Z'}}, f)
PYEOF

    export PATH="$WORK/bin:$PATH"
    export KANBAN_DB_PATH="$WORK/kanban/test.db"
    export KANBAN_BOARD="test"
    export MOCK_GH_PRS_JSON="$WORK/gh_prs.json"
    bash "$WATCHDOG_SH" 2>/dev/null
    bash "$WATCHDOG_SH" 2>/dev/null  # 2nd tick — idempotent SKIP

    local cnt
    cnt=$(awk '/MOCKED:/ {c++} END {print c+0}' "$HERMES_JOURNAL" 2>/dev/null)
    if [ "$cnt" -eq 1 ]; then
        pass "S6: 2 ticks → 1 comment (idempotent)"
    else
        fail "S6: expected 1 comment (idempotent), got $cnt"
        echo "  --- journal: $(cat $HERMES_JOURNAL)"
    fi
}

# ============================================================================
# S7: no PR refs in body or block-reason → SKIP "no_pr_ref"
# ============================================================================
test_S7_no_pr_refs_no_alert() {
    run_test "S7_no_pr_refs_no_alert"

    python3 - <<PYEOF
import sqlite3, time, json
con = sqlite3.connect('$WORK/kanban/test.db')
now = int(time.time())
five_h_ago = now - 5 * 3600
con.execute("""INSERT INTO tasks (id, title, body, assignee, status, started_at, max_runtime_seconds, created_at)
VALUES ('t_stale_7', 'test', 'No PR refs here at all, just waiting', 'default', 'blocked', ?, 1800, ?)""",
            (five_h_ago, five_h_ago))
con.execute("INSERT INTO task_links (parent_id, child_id) VALUES ('t_p7_done', 't_stale_7')")
con.execute("""INSERT INTO tasks (id, title, body, assignee, status, started_at, max_runtime_seconds, created_at)
VALUES ('t_p7_done', 'parent', 'done', 'default', 'done', ?, 1800, ?)""",
            (five_h_ago, five_h_ago))
con.commit()
con.close()
PYEOF

    export PATH="$WORK/bin:$PATH"
    export KANBAN_DB_PATH="$WORK/kanban/test.db"
    export KANBAN_BOARD="test"
    bash "$WATCHDOG_SH" 2>/dev/null

    local cnt
    cnt=$(awk '/MOCKED:/ {c++} END {print c+0}' "$HERMES_JOURNAL" 2>/dev/null)
    if [ "$cnt" -eq 0 ]; then
        pass "S7: no PR refs in body → no alert"
    else
        fail "S7: should NOT emit without PR refs, got $cnt"
        echo "  --- journal: $(cat $HERMES_JOURNAL)"
    fi
}

# ============================================================================
# S8: DRY_RUN=true → 0 actual side-effect (но __RECORD__ sentinels в stderr)
# ============================================================================
test_S8_dry_run_no_side_effect() {
    run_test "S8_dry_run_no_side_effect"

    python3 - <<PYEOF
import sqlite3, time, json
con = sqlite3.connect('$WORK/kanban/test.db')
now = int(time.time())
five_h_ago = now - 5 * 3600
con.execute("""INSERT INTO tasks (id, title, body, assignee, status, started_at, max_runtime_seconds, created_at)
VALUES ('t_stale_8', 'test', 'Body refs PR #2385', 'default', 'blocked', ?, 1800, ?)""",
            (five_h_ago, five_h_ago))
con.execute("INSERT INTO task_links (parent_id, child_id) VALUES ('t_p8_done', 't_stale_8')")
con.execute("""INSERT INTO tasks (id, title, body, assignee, status, started_at, max_runtime_seconds, created_at)
VALUES ('t_p8_done', 'parent', 'done', 'default', 'done', ?, 1800, ?)""",
            (five_h_ago, five_h_ago))
con.commit()
con.close()
with open('$WORK/gh_prs.json', 'w') as f:
    json.dump({'2385': {'state': 'closed', 'merged': True, 'merged_at': '2026-09-14T11:24:31Z'}}, f)
PYEOF

    export PATH="$WORK/bin:$PATH"
    export KANBAN_DB_PATH="$WORK/kanban/test.db"
    export KANBAN_BOARD="test"
    export MOCK_GH_PRS_JSON="$WORK/gh_prs.json"
    export DRY_RUN=true
    bash "$WATCHDOG_SH" 2>"$WORK/stderr.txt"

    local cnt
    cnt=$(awk '/MOCKED:/ {c++} END {print c+0}' "$HERMES_JOURNAL" 2>/dev/null)
    if [ "$cnt" -eq 0 ] && grep -q "DRY-RUN" "$WORK/stderr.txt"; then
        pass "S8: DRY_RUN=true → no side-effect, DRY-RUN in stderr"
    else
        fail "S8: DRY_RUN failed (cnt=$cnt, dry-run-stderr=$(grep DRY-RUN $WORK/stderr.txt | head -1))"
    fi
    unset DRY_RUN
}

# --- main -------------------------------------------------------------------
# Cleanup all WORK dirs created during this run (NOT trap-based — we want
# them to survive across test_* function calls since run_test is called
# inside test_* and we need to read journal/db after run_test returns).
_ALL_WORKS=()
cleanup_all_works() {
    for w in "${_ALL_WORKS[@]}"; do
        rm -rf "$w" 2>/dev/null || true
    done
}
trap cleanup_all_works EXIT

echo "=== test_stale_blocked_watchdog ==="
test_S1_unmerged_pr_blocks_alert; _ALL_WORKS+=("$WORK")
test_S2_merged_prs_parents_done_emits_alert; _ALL_WORKS+=("$WORK")
test_S3_parent_not_done_blocks_alert; _ALL_WORKS+=("$WORK")
test_S4_only_issue_refs_no_alert; _ALL_WORKS+=("$WORK")
test_S5_not_a_pr_alongside_real_pr_emits; _ALL_WORKS+=("$WORK")
test_S6_idempotent; _ALL_WORKS+=("$WORK")
test_S7_no_pr_refs_no_alert; _ALL_WORKS+=("$WORK")
test_S8_dry_run_no_side_effect; _ALL_WORKS+=("$WORK")

echo
echo "=== summary: $_pass passed, $_fail failed ==="
[ "$_fail" -eq 0 ]