#!/bin/bash
# ============================================================================
# test_watchdog_provider_quick_cooldown.sh — ретро 15.09 t_197de62a
#
# Регресс-гард для COOLDOWN-логики watchdog-provider-quick.sh:
# после block / block_loop_detected / unblock событий RECOVER_COOLDOWN_SEC
# (1800 сек = 30 мин) watchdog-provider-quick.sh НЕ ДЕЛАЕТ unblock даже при
# providers_alive=True. Это закрывает false-positive recovery: живой worker
# на ОБЫЧНОЙ задаче (не у провайдера) не должен «спасать» карточку из
# provider-exhaust блока (ретро t_197de62a).
#
# Тестируем именно watchdog-provider-quick.sh (1-мин fast-tick), не
# watchdog.sh (2-мин full tick) — для последнего см.
# test_watchdog_provider_exhaustion.sh.
#
# Кейсы:
#   Q1. blocked + gave_up НЕДАВНО (5 мин назад) + providers alive → НЕТ unblock
#       (cooldown ещё активен)
#   Q2. blocked + gave_up ДАВНО (>30 мин назад) + providers alive → unblock
#       (cooldown истёк)
#   Q3. blocked + ручной blocked (kind=capability) НЕД + providers alive →
#       НЕТ unblock (existing behaviour: guard на ручной блок)
#   Q4. blocked + gave_up 31 мин назад + последний unblock НЕДАВНО (5 мин)
#       → НЕТ unblock (cooldown от последнего unblock, не от gave_up)
#   Q5. running карточка с мёртвым pid + log marker → block (cooldown не
#       применяется к running-карточкам, только к recovery-волне)
#
# Run:
#   bash scripts/agent_flow/tests/test_watchdog_provider_quick_cooldown.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WATCHDOG_SH="${WATCHDOG_SH:-$TEST_DIR/../watchdog-provider-quick.sh}"

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

fail() { echo "FAIL: $*"; exit 1; }
pass() { echo "ok: $*"; }

[ -x "$WATCHDOG_SH" ] || fail "watchdog-provider-quick.sh not found: $WATCHDOG_SH"

# --- извлекаем python-блок provider-exhaustion из watchdog-provider-quick.sh --
# Ищем «python3 - ... <<'PYEOF' ... PYEOF» heredoc — это собственно python-код.
# Используем awk pattern: от строки с PYEOF (открытие heredoc) до строки
# с PYEOF (закрытие heredoc) — это надёжный маркер границ python-блока.
python3 - "$WATCHDOG_SH" "$WORK" <<'PYEOF'
import sys
path, work = sys.argv[1], sys.argv[2]
with open(path) as f:
    src = f.read()
# Ищем первое вхождение <<'PYEOF' ... PYEOF
start = src.find("<<'PYEOF'")
if start < 0:
    print("PYEOF_open not found")
    sys.exit(2)
# найдём конец heredoc: PYEOF на отдельной строке
end_marker = "\nPYEOF\n"
end = src.find(end_marker, start)
if end < 0:
    print("PYEOF_close not found")
    sys.exit(2)
py_src = src[start + len("<<'PYEOF'"):end]
# Проверяем, что в нём есть ключевые маркеры watchdog-provider-quick
with open(f"{work}/provider_scan.py", "w") as f:
    f.write(py_src)
print("extracted bytes:", len(py_src))
PYEOF
[ -s "$WORK/provider_scan.py" ] || fail "provider-exhaustion block not found in $WATCHDOG_SH"
grep -q "RECOVER_COOLDOWN_SEC" "$WORK/provider_scan.py" || fail "RECOVER_COOLDOWN_SEC missing — cooldown не внедрён (ретро t_197de62a)"

# --- фикстура: kanban.db + logs/ ---------------------------------------------
BOARD_DIR="$WORK/boards/robbox"
mkdir -p "$BOARD_DIR/logs"
python3 - "$BOARD_DIR" <<'PYEOF'
import sqlite3, sys, os, time
board_dir = sys.argv[1]
db = os.path.join(board_dir, "kanban.db")
con = sqlite3.connect(db)
con.executescript("""
CREATE TABLE tasks (
    id TEXT PRIMARY KEY,
    title TEXT DEFAULT '',
    status TEXT DEFAULT 'todo',
    assignee TEXT DEFAULT '',
    worker_pid INTEGER,
    last_heartbeat_at INTEGER,
    last_failure_error TEXT DEFAULT '',
    consecutive_failures INTEGER DEFAULT 0,
    block_kind TEXT,
    block_recurrences INTEGER DEFAULT 0,
    created_at INTEGER
);
CREATE TABLE task_events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id TEXT,
    kind TEXT,
    payload TEXT,
    created_at INTEGER
);
""")
now = int(time.time())

# Q1: blocked + gave_up НЕДАВНО (5 мин назад)
con.execute("INSERT INTO tasks VALUES ('t_cooldown_active','t','blocked','devops',NULL,?,'',0,NULL,0,?)", (now, now))
recent = now - 300
con.execute("INSERT INTO task_events (task_id,kind,payload,created_at) VALUES ('t_cooldown_active','blocked','{}',?)", (recent,))
con.execute("INSERT INTO task_events (task_id,kind,payload,created_at) VALUES ('t_cooldown_active','gave_up','{}',?)", (recent,))
open(os.path.join(board_dir, "logs", "t_cooldown_active.log"), "w").write(
    "Billing or credits exhausted: HTTP 402: Insufficient Balance\n")

# Q2: blocked + gave_up ДАВНО (>30 мин назад) → cooldown expired
con.execute("INSERT INTO tasks VALUES ('t_cooldown_expired','t','blocked','devops',NULL,?,'',0,NULL,0,?)", (now, now))
old = now - 1900
con.execute("INSERT INTO task_events (task_id,kind,payload,created_at) VALUES ('t_cooldown_expired','blocked','{}',?)", (old,))
con.execute("INSERT INTO task_events (task_id,kind,payload,created_at) VALUES ('t_cooldown_expired','gave_up','{}',?)", (old,))
open(os.path.join(board_dir, "logs", "t_cooldown_expired.log"), "w").write(
    "HTTP 402: Insufficient Balance\n")

# Q3: blocked + 'blocked' event НЕДАВНО (ручной) — guard на ручной блок
con.execute("INSERT INTO tasks VALUES ('t_manual_recent','t','blocked','devops',NULL,?,'',0,'capability',1,?)", (now, now))
con.execute("INSERT INTO task_events (task_id,kind,payload,created_at) VALUES ('t_manual_recent','blocked','{\"kind\":\"capability\"}',?)", (recent,))
open(os.path.join(board_dir, "logs", "t_manual_recent.log"), "w").write(
    "HTTP 402: Insufficient Balance\n")

# Q4: gave_up ДАВНО + unblock НЕДАВНО → cooldown от unblock (а не от gave_up)
con.execute("INSERT INTO tasks VALUES ('t_unblock_recent','t','blocked','devops',NULL,?,'',0,NULL,0,?)", (now, now))
con.execute("INSERT INTO task_events (task_id,kind,payload,created_at) VALUES ('t_unblock_recent','gave_up','{}',?)", (old,))   # old
con.execute("INSERT INTO task_events (task_id,kind,payload,created_at) VALUES ('t_unblock_recent','blocked','{}',?)", (recent,)) # recent
con.execute("INSERT INTO task_events (task_id,kind,payload,created_at) VALUES ('t_unblock_recent','unblocked','{}',?)", (recent,)) # recent unblock
open(os.path.join(board_dir, "logs", "t_unblock_recent.log"), "w").write(
    "HTTP 402: Insufficient Balance\n")

# Q5: running + мёртвый pid + marker → block (cooldown НЕ применяется)
con.execute("INSERT INTO tasks VALUES ('t_running_dead','t','running','devops',999999,?,'',0,NULL,0,?)", (now, now))
open(os.path.join(board_dir, "logs", "t_running_dead.log"), "w").write(
    "HTTP 429 Token Plan rate limit reached\n")

# Чистый свежий probe — чтобы providers_alive=True
con.execute("INSERT INTO tasks VALUES ('t_probe_clean','t','ready','devops',NULL,?,'',0,NULL,0,?)", (now, now))
open(os.path.join(board_dir, "logs", "t_probe_clean.log"), "w").write(
    "normal worker log, no provider errors\n")

con.commit()
con.close()
PYEOF
[ -f "$BOARD_DIR/kanban.db" ] || fail "fixture db not created"

# Все маркерные логи — свежие (1 мин назад), чистый probe — тоже свежий
touch -d '1 minute ago' "$BOARD_DIR/logs/t_cooldown_active.log"
touch -d '1 minute ago' "$BOARD_DIR/logs/t_cooldown_expired.log"
touch -d '1 minute ago' "$BOARD_DIR/logs/t_manual_recent.log"
touch -d '1 minute ago' "$BOARD_DIR/logs/t_unblock_recent.log"
touch -d '1 minute ago' "$BOARD_DIR/logs/t_running_dead.log"
touch -d '1 minute ago' "$BOARD_DIR/logs/t_probe_clean.log"

# --- запускаем python-блок с тестовым окружением -----------------------------
run_provider_scan() {
    local actions_file="$1"
    cat > "$WORK/run.py" <<RUN
import sys, os, glob, time, sqlite3, json
# Извлечённый python-блок читает sys.argv[1..3] = hermes_home, boards_dir,
# provider_actions_file (см. python3 - "\$HERMES_HOME" "\$KANBAN_BOARDS_DIR"
# "\$PROVIDER_ACTIONS_FILE" в watchdog-provider-quick.sh).
sys.argv = ["watchdog-provider-quick", "$WORK", "$WORK/boards", "$actions_file"]
now = int(time.time())
# Заглушки, чтобы извлечённый блок работал:
provider_actions = []
boards_dir = "$WORK/boards"
hermes_home = "$WORK"
provider_actions_file = "$actions_file"
exec(open("$WORK/provider_scan.py").read())
print("ACTIONS:", provider_actions)
print("PROVIDERS_ALIVE:", providers_alive)
RUN
    python3 "$WORK/run.py"
}

OUT="$(run_provider_scan "$WORK/actions.txt" 2>&1)" || { echo "$OUT"; fail "python block crashed"; }
echo "=== RUN ==="
echo "$OUT"
[ -f "$WORK/actions.txt" ] || fail "actions.txt not written"

# --- assertions --------------------------------------------------------------
echo "=== TEST ASSERTIONS ==="

# Q1: cooldown active → no unblock
grep -q "t_cooldown_active" "$WORK/actions.txt" && fail "Q1: t_cooldown_active should NOT be in actions (5 мин < 30 мин cooldown)"
echo "  Q1 ok — t_cooldown_active NOT unblocked (cooldown active)"

# Q2: cooldown expired → unblock
grep -q "^unblock|robbox|t_cooldown_expired$" "$WORK/actions.txt" || fail "Q2: t_cooldown_expired should be unblocked (gave_up 1900s ago > 1800s)"
echo "  Q2 ok — t_cooldown_expired unblocked (cooldown expired)"

# Q3: manual blocked → no unblock (existing guard)
grep -q "t_manual_recent" "$WORK/actions.txt" && fail "Q3: t_manual_recent should NOT be in actions (manual block guard)"
echo "  Q3 ok — t_manual_recent NOT unblocked (manual block guard)"

# Q4: unblock недавно → cooldown считается от unblock
grep -q "t_unblock_recent" "$WORK/actions.txt" && fail "Q4: t_unblock_recent should NOT be in actions (unblock недавно, cooldown активен)"
echo "  Q4 ok — t_unblock_recent NOT unblocked (cooldown от unblock, не от gave_up)"

# Q5: running + dead pid → block (cooldown не применяется)
grep -q "^block|robbox|t_running_dead$" "$WORK/actions.txt" || fail "Q5: t_running_dead should be blocked"
echo "  Q5 ok — t_running_dead blocked (cooldown не применяется к running)"

pass "watchdog-provider-quick cooldown: все кейсы прошли (Q1-Q5)"