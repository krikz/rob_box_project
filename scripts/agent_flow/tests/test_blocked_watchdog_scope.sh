#!/bin/bash
# ============================================================================
# test_blocked_watchdog_scope.sh — cron-надзор mis-scope карточек (ADR-0036 §4.3, ретро t_aa585aa7)
#
# Регресс-гард для agent-flow-blocked-watchdog-scope.sh. Тестируем чистую
# логику (без hermes CLI side-effects) через PATH-hijack: подставляем
# mock-hermes, который пишет в JOURNAL вместо реального вызова. SQLite DB
# — настоящая (через python sqlite3 модуль), чтобы тест работал с реальным
# контрактом схемы task_comments.
#
# Scenarios (acceptance criteria t_aa585aa7):
#   test_overshoot_comment:    criteria выполняются (age>4ч, non-architect/devops, ADR-)
#                              → comment в карточку (один)
#   test_overshoot_idempotent: 2 тика подряд → один comment в task_comments
#   test_overshoot_no_comment_architect: assignee=architect → НЕТ comment
#                                        (architect — allowed profile, не mis-scope)
#   test_overshoot_no_comment_no_adr:    body без "ADR-" → НЕТ comment
#                                        (heuristic: keyword heuristic)
#
# Run:
#   bash scripts/agent_flow/tests/test_blocked_watchdog_scope.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WATCHDOG_SH="${WATCHDOG_SH:-$TEST_DIR/../agent-flow-blocked-watchdog-scope.sh}"

[ -f "$WATCHDOG_SH" ] || { echo "FAIL: $WATCHDOG_SH not found (test RED — скрипт ещё не написан)"; exit 1; }
command -v bash >/dev/null || { echo "FAIL: bash required"; exit 1; }
command -v python3 >/dev/null || { echo "FAIL: python3 required (sqlite3 + JSON)"; exit 1; }

# Каждый тест — fresh WORK и fresh mock-DB. Не модифицируем реальный kanban.db.
run_test() {
    local name="$1"; shift
    local fn="$1"; shift
    WORK="$(mktemp -d)"
    trap 'rm -rf "$WORK"' RETURN

    mkdir -p "$WORK/bin" "$WORK/kanban"

    # Настоящая sqlite3 БД с минимальной схемой под наш контракт. Это
    # упрощает тест — нам не нужно создавать полную схему kanban (24 таблицы
    # + индексы), только tasks + task_comments.
    python3 - <<PYEOF
import sqlite3, os
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
''')
con.commit()
con.close()
PYEOF

    # mock-hermes: вместо реального `hermes kanban --board X comment TID BODY`
    # пишем в JOURNAL — там тест найдёт side-effect.
    cat > "$WORK/bin/hermes" <<'EOF'
#!/bin/bash
# Маршрутизация: фиксируем в журнал все вызовы kanban comment.
case "${1:-}${2:-}${3:-}${4:-}" in
    *kanban*comment*)
        # hermes kanban --board <board> comment <task_id> <body...>
        shift  # kanban
        shift  # --board
        local _board="$1"; shift
        shift  # comment
        local _tid="$1"; shift
        # Остальное — body (может быть multi-word, мы не парсим — пишем as-is).
        echo "MOCKED: kanban comment board=${_board} task=${_tid} body=$*" >> "$HERMES_JOURNAL"
        exit 0
        ;;
    *)
        # Неиспользуемые команды — пустая success.
        exit 0
        ;;
esac
EOF
    chmod +x "$WORK/bin/hermes"

    export HERMES_JOURNAL="$WORK/journal.txt"
    : > "$HERMES_JOURNAL"

    # Запускаем subshell с PATH, указывающим на наш mock-hermes первым.
    (
        export PATH="$WORK/bin:$PATH"
        export KANBAN_DB_PATH="$WORK/kanban/test.db"
        export KANBAN_BOARD="test"
        "$fn" 2>/dev/null
    )
    local rc=$?
    if [ "$rc" -eq 0 ]; then
        echo "  ok: $name"
    else
        echo "  FAIL: $name (rc=$rc)"
        echo "  --- journal:"
        cat "$HERMES_JOURNAL" 2>/dev/null || true
        echo "  --- DB:"
        python3 -c "
import sqlite3
con = sqlite3.connect('$WORK/kanban/test.db')
for r in con.execute('SELECT id, body, started_at, status FROM tasks').fetchall():
    print('  task:', r)
for r in con.execute('SELECT task_id, body FROM task_comments').fetchall():
    print('  comment:', r)
con.close()
" 2>/dev/null || true
        exit 1
    fi
    return 0
}

# ============================================================================
# T1: criteria выполняются → comment появляется
# ============================================================================
test_overshoot_comment() {
    local now
    now="$(date -u +%s)"
    local five_hours_ago=$(( now - 5 * 3600 ))  # 5 часов назад → age > 4ч

    python3 - <<PYEOF
import sqlite3
con = sqlite3.connect('$WORK/kanban/test.db')
con.execute("""
INSERT INTO tasks (id, title, body, assignee, status, started_at, max_runtime_seconds, created_at)
VALUES ('t_scope_a', 'mis-scope test', 'ADR-0035 реализация', 'backend', 'running', $five_hours_ago, 1800, $five_hours_ago)
""")
con.commit()
con.close()
PYEOF

    # Run скрипт. Он должен найти criteria и emit 1 comment.
    # NB: exit code 2 — alert mode (что-то закомментили), exit 0 — no-op.
    # Оба означают что скрипт отработал успешно.
    bash "$WATCHDOG_SH" 2>/dev/null
    local rc=$?
    case "$rc" in
        0|2)
            : # ok (0 = no-op, 2 = alert mode — закомментили что-то)
            ;;
        *)
            return $rc
            ;;
    esac

    local count
    count="$(python3 -c "
import sqlite3
con = sqlite3.connect('$WORK/kanban/test.db')
print(con.execute('SELECT COUNT(*) FROM task_comments WHERE task_id=\"t_scope_a\"').fetchone()[0])
con.close()
")"
    if [ "$count" -eq 1 ]; then
        return 0
    else
        echo "  expected 1 comment, got $count"
        return 1
    fi
}

# ============================================================================
# T2: idempotency — 2 тика подряд → один comment в сутки
# ============================================================================
test_overshoot_idempotent() {
    local now
    now="$(date -u +%s)"
    local five_hours_ago=$(( now - 5 * 3600 ))

    python3 - <<PYEOF
import sqlite3
con = sqlite3.connect('$WORK/kanban/test.db')
con.execute("""
INSERT INTO tasks (id, title, body, assignee, status, started_at, max_runtime_seconds, created_at)
VALUES ('t_scope_b', 'mis-scope test', 'ADR-0036 §4.3', 'backend', 'running', $five_hours_ago, 1800, $five_hours_ago)
""")
con.commit()
con.close()
PYEOF

    # Первый тик
    bash "$WATCHDOG_SH" >/dev/null 2>&1
    # Второй тик сразу за ним (свежий comment уже есть → должно skip)
    bash "$WATCHDOG_SH" >/dev/null 2>&1

    local count
    count="$(python3 -c "
import sqlite3
con = sqlite3.connect('$WORK/kanban/test.db')
print(con.execute('SELECT COUNT(*) FROM task_comments WHERE task_id=\"t_scope_b\"').fetchone()[0])
con.close()
")"
    if [ "$count" -eq 1 ]; then
        return 0
    else
        echo "  expected 1 comment после двух тиков, got $count"
        return 1
    fi
}

# ============================================================================
# T3: assignee=architect → НЕТ comment (architect allowed)
# ============================================================================
test_overshoot_no_comment_architect() {
    local now
    now="$(date -u +%s)"
    local five_hours_ago=$(( now - 5 * 3600 ))

    python3 - <<PYEOF
import sqlite3
con = sqlite3.connect('$WORK/kanban/test.db')
con.execute("""
INSERT INTO tasks (id, title, body, assignee, status, started_at, max_runtime_seconds, created_at)
VALUES ('t_scope_c', 'mis-scope test', 'ADR-0035 plan', 'architect', 'running', $five_hours_ago, 1800, $five_hours_ago)
""")
con.commit()
con.close()
PYEOF

    bash "$WATCHDOG_SH" >/dev/null 2>&1

    local count
    count="$(python3 -c "
import sqlite3
con = sqlite3.connect('$WORK/kanban/test.db')
print(con.execute('SELECT COUNT(*) FROM task_comments WHERE task_id=\"t_scope_c\"').fetchone()[0])
con.close()
")"
    if [ "$count" -eq 0 ]; then
        return 0
    else
        echo "  expected 0 comment для architect, got $count"
        return 1
    fi
}

# ============================================================================
# T4: body без "ADR-" → НЕТ comment (heuristic miss guard)
# ============================================================================
test_overshoot_no_comment_no_adr() {
    local now
    now="$(date -u +%s)"
    local five_hours_ago=$(( now - 5 * 3600 ))

    python3 - <<PYEOF
import sqlite3
con = sqlite3.connect('$WORK/kanban/test.db')
con.execute("""
INSERT INTO tasks (id, title, body, assignee, status, started_at, max_runtime_seconds, created_at)
VALUES ('t_scope_d', 'long backend task', 'Просто долгая задача без архитектурного смысла', 'backend', 'running', $five_hours_ago, 1800, $five_hours_ago)
""")
con.commit()
con.close()
PYEOF

    bash "$WATCHDOG_SH" >/dev/null 2>&1

    local count
    count="$(python3 -c "
import sqlite3
con = sqlite3.connect('$WORK/kanban/test.db')
print(con.execute('SELECT COUNT(*) FROM task_comments WHERE task_id=\"t_scope_d\"').fetchone()[0])
con.close()
")"
    if [ "$count" -eq 0 ]; then
        return 0
    else
        echo "  expected 0 comment без ADR-, got $count"
        return 1
    fi
}

# --- registry -------------------------------------------------------------
fail() { echo "FAIL: $*"; exit 1; }
pass() { echo "ok: $blocked-watchdog-scope: все кейсы прошли"; exit 0; }

# Запускаем каждый тест в изоляции (отдельный WORK).
FAILED=0
for tn in test_overshoot_comment test_overshoot_idempotent test_overshoot_no_comment_architect test_overshoot_no_comment_no_adr; do
    WORK="$(mktemp -d)"
    _test_rc=0
    _test_failed=0
    # NB: не используем subshell `(...)` — trap 'rm -rf' EXIT внутри может
    # исказить rc и сбить инкремент. Делаем cleanup вручную + return-code
    # capture через файл.
    mkdir -p "$WORK/bin" "$WORK/kanban"
    python3 - <<PYEOF
import sqlite3
con = sqlite3.connect('$WORK/kanban/test.db')
con.executescript('''
CREATE TABLE tasks (
    id TEXT PRIMARY KEY, title TEXT, body TEXT, assignee TEXT,
    status TEXT NOT NULL, started_at INTEGER, max_runtime_seconds INTEGER,
    created_at INTEGER NOT NULL, workspace_kind TEXT NOT NULL DEFAULT 'scratch', session_id TEXT
);
CREATE TABLE task_comments (
    id INTEGER PRIMARY KEY AUTOINCREMENT, task_id TEXT NOT NULL,
    author TEXT NOT NULL, body TEXT NOT NULL, created_at INTEGER NOT NULL
);
''')
con.commit()
con.close()
PYEOF
    cat > "$WORK/bin/hermes" <<'EOF'
#!/bin/bash
# mock hermes — эмулирует РЕАЛЬНОЕ поведение `hermes kanban --board X
# comment TID BODY`: пишет строку в task_comments (через sqlite3) и в
# JOURNAL (для визуального assert). Это даёт idempotency-проверке в
# watchdog-scope реальные данные, на которых она работает.
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
    (tid, "devops-bot", body, int(time.time())),
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
    chmod +x "$WORK/bin/hermes"
    export HERMES_JOURNAL="$WORK/journal.txt"
    : > "$HERMES_JOURNAL"
    export PATH="$WORK/bin:$PATH"
    export KANBAN_DB_PATH="$WORK/kanban/test.db"
    export KANBAN_BOARD="test"

    # Запускаем $tn. Capture rc в файл (subshell-альтернатива для надёжного
    # проброса rc).
    "$tn"
    _test_rc=$?
    if [ "$_test_rc" -ne 0 ]; then
        _test_failed=1
    fi

    # Cleanup. NB: НЕ unset PATH — это глобальная переменная shell.
    python3 -c "import shutil; shutil.rmtree('$WORK')" 2>/dev/null
    unset HERMES_JOURNAL KANBAN_DB_PATH KANBAN_BOARD

    if [ "$_test_failed" -ne 0 ]; then
        echo "  FAIL: $tn (rc=$_test_rc)"
        FAILED=$(( FAILED + 1 ))
    else
        echo "  ok: $tn"
    fi
done

if [ "$FAILED" -gt 0 ]; then
    echo "FAIL: $FAILED тестов упало"
    exit 1
fi
echo "ok: blocked-watchdog-scope: все 4 кейса прошли"
exit 0
