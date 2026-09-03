#!/bin/bash
# ============================================================================
# test_watchdog_overshoot.sh — ADR-0036 §4.2 / kanban t_8234ed76
#
# Регресс-гард для блока runtime-overshoot в watchdog.sh: карточки running
# с age-since-started > 4*max_runtime_seconds (но heartbeat может быть
# свежим — это catch-all для TDD/bash-loop без прогресса) → issues[] +
# idempotent comment в task_comments + sigterm action.
#
# Кейсы (acceptance criteria, ADR-0036 §4.2):
#   T1. test_overshoot_no_overshoot:
#       task с max_rt=1800, started_at=now-100s → НЕ триггерит (4×1800=7200s)
#   T2. test_overshoot_triggers:
#       task с max_rt=1800, started_at=now-8000s (>4×1800) → action file
#       содержит sigterm для этой карточки; ровно 1 коммент в task_comments
#   T3. test_overshoot_idempotent:
#       2 прохода watchdog на той же задаче → комментарий появляется 1 раз;
#       overshoot-action может появиться 1 или 2 раза (оба легитимны —
#       между проходами SIGTERM ещё не отправлен); но task_comments.count = 1
#   T4. test_overshoot_no_max_rt:
#       task с max_runtime_seconds=0 или NULL → skip (нет action, нет comment)
#
# Не проверяем (это bash-часть, не Python-блок):
#   - сам SIGTERM/SIGKILL (это часть bash после PYEOF; там тривиальный kill,
#     дополнительный гард не нужен)
#   - 60s grace-window SIGTERM→SIGKILL (требует мока времени — оверкилл для
#     юнит-теста; проверяется вручную на живом watchdog)
#
# Run:
#   bash scripts/agent_flow/tests/test_watchdog_overshoot.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WATCHDOG_SH="${WATCHDOG_SH:-$TEST_DIR/../watchdog.sh}"

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

fail() { echo "FAIL: $*"; exit 1; }
pass() { echo "ok: $*"; }

# --- извлекаем блок 1d runtime-overshoot из watchdog.sh ----------------------
# Блок начинается с комментария «# 1d. runtime-overshoot (ADR-0036 §4.2» и
# заканчивается перед «# 1c. provider-exhaustion». Используем ту же awk-стратегию,
# что в test_watchdog_provider_exhaustion.sh / test_watchdog_recovery_capability.sh.
awk '/# 1d\. runtime-overshoot/{f=1} f{print} f && /^# 1c\. provider-exhaustion/{exit}' \
    "$WATCHDOG_SH" > "$WORK/overshoot.py"
[ -s "$WORK/overshoot.py" ] || fail "overshoot block not found in $WATCHDOG_SH"

# --- фикстура: kanban.db с tasks + task_events + task_comments ---------------
# Полная схема — реальные колонки из kanban_db.py. Минимально нужные:
#   tasks: id, status, worker_pid, started_at, max_runtime_seconds,
#          last_heartbeat_at
#   task_events: id, task_id, kind, payload, created_at
#   task_comments: id, task_id, author, body, created_at
init_fixture() {
    local board_dir="$1"
    mkdir -p "$board_dir"
    rm -f "$board_dir/kanban.db"
    python3 - "$board_dir" <<'PYFIX'
import sqlite3, sys, os
board_dir = sys.argv[1]
db = os.path.join(board_dir, "kanban.db")
con = sqlite3.connect(db)
con.executescript("""
CREATE TABLE tasks (
    id TEXT PRIMARY KEY,
    title TEXT DEFAULT '',
    status TEXT DEFAULT 'todo',
    worker_pid INTEGER,
    started_at INTEGER,
    last_heartbeat_at INTEGER,
    max_runtime_seconds INTEGER,
    assignee TEXT DEFAULT ''
);
CREATE TABLE task_events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id TEXT,
    run_id INTEGER,
    kind TEXT,
    payload TEXT,
    created_at INTEGER
);
CREATE TABLE task_comments (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id TEXT,
    author TEXT,
    body TEXT,
    created_at INTEGER
);
""")
con.commit()
con.close()
PYFIX
}
# Один общий $BOARD_DIR на все тесты (у каждого test-task свой уникальный id,
# поэтому пересечение имён невозможно; поведение «задача из T2 не подхватывается
# в T3» нас не интересует, т.к. тесты смотрят на свои конкретные tid).
BOARD_DIR="$WORK/boards/robbox"
init_fixture "$BOARD_DIR"
[ -f "$BOARD_DIR/kanban.db" ] || fail "fixture db not created"

# --- раннер: вставка задач + запуск python-блока ----------------------------
# Принимает: фикстура-функция (записывает задачи и возвращает их ids),
# имя прогона; печатает stdout python-блока + actions_file + asserts.
run_overshoot() {
    local name="$1"
    local actions_file="$WORK/actions_${name}.txt"
    : > "$actions_file"
    cat > "$WORK/run.py" <<PYRUN
import sys, os, time, sqlite3, glob, subprocess, json
sys.argv = ["watchdog", "$WORK", "$WORK/boards", "600", "15", "", "$actions_file"]
# Окружение для python-блока: извлечённый код ссылается на issues/recovery/
# now/boards_dir/provider_actions_file/overshoot_actions_file. Контекст
# выставляем до exec() — блок ожидает их уже определёнными.
issues = []
recovery = []
workers_by_unit = set()
overshoot_actions = []
now = int(time.time())
boards_dir = "$WORK/boards"
provider_actions_file = ""
overshoot_actions_file = "$actions_file"
exec(open("$WORK/overshoot.py").read())
print("ISSUES:", issues)
print("ACTIONS:", overshoot_actions)
PYRUN
    python3 "$WORK/run.py" 2>&1
}

# --- helpers: вставка задач, проверка task_comments/task_events ------------
insert_task() {
    # insert_task <id> <status> <started_at> <max_rt> [pid]
    # Default pid = текущий shell-процесс (он точно жив в момент прогона —
    # иначе `os.kill(pid, 0)` в watchdog-блоке даст ESRCH и overshoot-логика
    # решит «уже мёртв, не flag'аем» — баг, который надо уметь тестировать
    # отдельно, но НЕ в основных positive-кейсах).
    local tid="$1" status="$2" started="$3" max_rt="$4"
    local pid="${5:-$$}"
    python3 - "$BOARD_DIR/kanban.db" "$tid" "$status" "$started" "$max_rt" "$pid" <<'PYI'
import sqlite3, os, sys, time
db, tid, status, started, max_rt, pid = sys.argv[1:]
con = sqlite3.connect(db)
con.execute(
    "INSERT OR REPLACE INTO tasks "
    "(id, status, worker_pid, started_at, max_runtime_seconds, last_heartbeat_at, title, assignee) "
    "VALUES (?, ?, ?, ?, ?, ?, '', 'devops')",
    (tid, status, int(pid), int(started), int(max_rt), int(time.time())),
)
con.commit()
con.close()
PYI
}

count_comments() {
    # count_comments <tid> <marker>
    local tid="$1" marker="$2"
    python3 - "$BOARD_DIR/kanban.db" "$tid" "$marker" <<'PYC'
import sqlite3, sys
db, tid, marker = sys.argv[1:]
con = sqlite3.connect(db)
n = con.execute(
    "SELECT COUNT(*) FROM task_comments WHERE task_id=? AND body LIKE ?",
    (tid, f"%{marker}%"),
).fetchone()[0]
con.close()
print(n)
PYC
}

count_overshoot_events() {
    # count_overshoot_events <tid>
    local tid="$1"
    python3 - "$BOARD_DIR/kanban.db" "$tid" <<'PYE'
import sqlite3, sys
db, tid = sys.argv[1:]
con = sqlite3.connect(db)
n = con.execute(
    "SELECT COUNT(*) FROM task_events WHERE task_id=? AND kind='overshoot_kill'",
    (tid,),
).fetchone()[0]
con.close()
print(n)
PYE
}

# --- T1. no_overshoot: max_rt=1800, started=now-100s → НЕ триггерит -----------
echo "=== T1: no_overshoot ==="
insert_task "t_no_overshoot" "running" "$(($(date +%s) - 100))" 1800
OUT1="$(run_overshoot t1 2>&1)"
echo "$OUT1"
grep -q "t_no_overshoot" "$WORK/actions_t1.txt" \
    && fail "T1: action для t_no_overshoot НЕ должно быть (age=100s, budget=7200s)"
[ "$(count_comments t_no_overshoot runtime-overshoot)" = "0" ] \
    || fail "T1: комментариев в task_comments быть не должно"
pass "T1: no_overshoot — пропуск по бюджету"

# --- T2. triggers: max_rt=1800, started=now-8000s → должен триггернуть ------
echo "=== T2: triggers ==="
TID2="t_triggers_$(date +%s)"
insert_task "$TID2" "running" "$(($(date +%s) - 8000))" 1800
OUT2="$(run_overshoot t2 2>&1)"
echo "$OUT2"
grep -q "^robbox|${TID2}|${$}|sigterm$" "$WORK/actions_t2.txt" \
    || fail "T2: action ${TID2}|${$}|sigterm не найден в actions_t2.txt"
NC2="$(count_comments "$TID2" runtime-overshoot)"
[ "$NC2" = "1" ] \
    || fail "T2: ожидали 1 комментарий с runtime-overshoot для $TID2, получили $NC2"
pass "T2: triggers — action + 1 comment"

# --- T3. idempotent: 2 прохода на той же задаче → comments.count = 1 --------
echo "=== T3: idempotent ==="
TID3="t_idempotent_$(date +%s)"
insert_task "$TID3" "running" "$(($(date +%s) - 8000))" 1800
# Первый прогон — comment ещё нет → должен записать
run_overshoot t3a >/dev/null 2>&1
NC3A="$(count_comments "$TID3" runtime-overshoot)"
[ "$NC3A" = "1" ] || fail "T3a: после 1-го прогона ожидали 1 comment, получили $NC3A"
# Второй прогон — comment уже есть → НЕ должен записать второй раз
run_overshoot t3b >/dev/null 2>&1
NC3B="$(count_comments "$TID3" runtime-overshoot)"
[ "$NC3B" = "1" ] || fail "T3b: после 2-го прогона ожидали всё ещё 1 comment, получили $NC3B"
pass "T3: idempotent — 2 прогона дали 1 comment ($NC3B)"

# --- T4. no_max_rt: max_runtime_seconds=0 или NULL → skip -------------------
echo "=== T4: no_max_rt ==="
TID4="t_no_max_rt_$(date +%s)"
# max_runtime_seconds = 0 — guard должен skip'нуть
insert_task "$TID4" "running" "$(($(date +%s) - 8000))" 0
OUT4="$(run_overshoot t4 2>&1)"
echo "$OUT4"
grep -q "${TID4}" "$WORK/actions_t4.txt" \
    && fail "T4: action для $TID4 (max_rt=0) НЕ должно быть"
[ "$(count_comments "$TID4" runtime-overshoot)" = "0" ] \
    || fail "T4: комментариев для $TID4 быть не должно"
pass "T4: no_max_rt=0 — skip"

# T4b. max_runtime_seconds=NULL (отсутствует в INSERT) — тоже skip
TID4B="t_no_max_rt_null_$(date +%s)"
python3 - "$BOARD_DIR/kanban.db" "$TID4B" "$(($(date +%s) - 8000))" "$$" <<'PYNULL'
import sqlite3, sys, time
db, tid, started, pid = sys.argv[1:]
con = sqlite3.connect(db)
con.execute(
    "INSERT INTO tasks (id, status, worker_pid, started_at, last_heartbeat_at, title, assignee) "
    "VALUES (?, 'running', ?, ?, ?, '', 'devops')",
    (tid, int(pid), int(started), int(time.time())),
)
con.commit()
con.close()
PYNULL
run_overshoot t4b >/dev/null 2>&1
grep -q "${TID4B}" "$WORK/actions_t4b.txt" \
    && fail "T4b: action для $TID4B (max_rt NULL) НЕ должно быть"
[ "$(count_comments "$TID4B" runtime-overshoot)" = "0" ] \
    || fail "T4b: комментариев для $TID4B быть не должно"
pass "T4b: no_max_rt NULL — skip"

# --- Защита от регрессии существующих тестов --------------------------------
# Проверяем что блок 1d НЕ сломал 1b/1c (structural smoke): awk-извлечение
# целого верхнеуровневого python-блока из watchdog.sh всё ещё даёт валидный
# Python. В watchdog.sh есть и вложенные (indented) PYEOF-блоки внутри
# sweep_stale_project_worktrees — их вытаскивать НЕ надо.
awk '/^python3 - "/,/^PYEOF$/' "$WATCHDOG_SH" | sed -e '/^PYEOF$/d' > "$WORK/full.py" || true
python3 -c "import py_compile, sys; py_compile.compile(sys.argv[1], doraise=True); print('full-heredoc OK')" \
    "$WORK/full.py" || fail "полный верхнеуровневый python-heredoc watchdog.sh перестал компилироваться"
pass "structural: верхнеуровневый python-heredoc watchdog.sh по-прежнему валиден"

# Дополнительно: bash -n для самого watchdog.sh (синтаксис всей обёртки).
bash -n "$WATCHDOG_SH" || fail "bash -n: синтаксис watchdog.sh сломался"
pass "structural: bash -n watchdog.sh OK"

echo
echo "ALL TESTS PASSED"
