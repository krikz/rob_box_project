#!/bin/bash
# ============================================================================
# test_watchdog_recovery_capability.sh — ретро t_e4bef56b
#
# Edge case для recovery-волны watchdog.sh (2-мин тик):
# если карточка заблокирована самим watchdog-provider-quick.sh
# (kind=capability), а не dispatcher'ом (gave_up/protocol_violation/crashed/
# rate_limited), то существующая recovery-волна её НЕ покрывает → карточка
# висит в blocked до ручного unblock.
#
# Фикс: добавили ветку elif last_ev[0] == "blocked" + payload.kind ==
# "capability" → unblock (только при providers_alive=True). Ручной блок
# юзера (needs_input/dependency/transient) не трогаем — гард по payload.kind.
#
# Кейсы:
#   C1. blocked + last_event.kind=blocked + payload.kind=capability +
#       providers_alive=True → action unblock
#   C2. blocked + last_event.kind=blocked + payload.kind=capability +
#       providers_alive=False → НЕТ action
#   C3. blocked + last_event.kind=blocked + payload.kind=needs_input
#       (ручной блок) + providers_alive=True → НЕТ action (гард)
#   C4. blocked + last_event.kind=blocked + payload.kind=dependency
#       (ручной блок) + providers_alive=True → НЕТ action (гард)
#   C5. blocked + last_event.kind=gave_up + payload={} + providers_alive=True
#       → unblock (старая ветка, регресс)
#   C6. blocked + last_event.kind=blocked + payload (невалидный JSON) +
#       providers_alive=True → НЕТ action (парсер не падает)
#
# Run:
#   bash scripts/agent_flow/tests/test_watchdog_recovery_capability.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WATCHDOG_SH="${WATCHDOG_SH:-$TEST_DIR/../watchdog.sh}"

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

fail() { echo "FAIL: $*"; exit 1; }
pass() { echo "ok: $*"; }

# --- извлекаем python-блок provider-exhaustion из watchdog.sh ----------------
# Блок начинается с комментария «# 1c. provider-exhaustion (ретро 15.08» и
# заканчивается перед «# 2. dispatcher status». Та же стратегия, что в
# test_watchdog_provider_exhaustion.sh. Дополнительно прeпендим `import json`
# (используется в t_e4bef56b recovery-ветке для capability-блока): при
# exec()-стратегии импорт в начале watchdog.sh PYEOF heredoc'а не виден,
# поэтому inject'им явно.
awk '/# 1c\. provider-exhaustion/{f=1} f{print} f && /^# 2\. dispatcher status/{exit}' \
    "$WATCHDOG_SH" > "$WORK/provider_scan.py"
printf 'import json\n' | cat - "$WORK/provider_scan.py" > "$WORK/provider_scan.tmp"
mv "$WORK/provider_scan.tmp" "$WORK/provider_scan.py"
[ -s "$WORK/provider_scan.py" ] || fail "provider-exhaustion block not found in $WATCHDOG_SH"

# --- фикстура: kanban.db + logs/ ---------------------------------------------
BOARD_DIR="$WORK/boards/robbox"
mkdir -p "$BOARD_DIR/logs"
python3 - "$BOARD_DIR" <<'PYEOF'
import sqlite3, sys, os, time, json
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
# C1. blocked + payload.kind=capability (auto-block от watchdog-provider-quick) +
#     providers alive → unblock
con.execute(
    "INSERT INTO tasks VALUES ('t_cap_unblock','t','blocked','backend',NULL,0,?)",
    (now,),
)
con.execute(
    "INSERT INTO task_events (task_id,kind,payload,created_at) "
    "VALUES ('t_cap_unblock','blocked',?,?)",
    (json.dumps({"reason": "provider-exhaustion", "kind": "capability",
                 "recurrences": 0}), now),
)
open(os.path.join(board_dir, "logs", "t_cap_unblock.log"), "w").write(
    "HTTP 429: rate limit (старый лог, провайдер уже не в этом состоянии)\n"
)
# C3. blocked + payload.kind=needs_input (ручной блок Шифу) + providers alive
#     → НЕ трогать (гард: ручной блок)
con.execute(
    "INSERT INTO tasks VALUES ('t_needs_input','t','blocked','backend',NULL,0,?)",
    (now,),
)
con.execute(
    "INSERT INTO task_events (task_id,kind,payload,created_at) "
    "VALUES ('t_needs_input','blocked',?,?)",
    (json.dumps({"reason": "ждать ответа юзера", "kind": "needs_input",
                 "recurrences": 0}), now),
)
open(os.path.join(board_dir, "logs", "t_needs_input.log"), "w").write(
    "HTTP 402: Insufficient Balance\n"
)
# C4. blocked + payload.kind=dependency (ручной блок) + providers alive
#     → НЕ трогать
con.execute(
    "INSERT INTO tasks VALUES ('t_dep_block','t','blocked','backend',NULL,0,?)",
    (now,),
)
con.execute(
    "INSERT INTO task_events (task_id,kind,payload,created_at) "
    "VALUES ('t_dep_block','blocked',?,?)",
    (json.dumps({"reason": "wait on parent", "kind": "dependency",
                 "recurrences": 0}), now),
)
open(os.path.join(board_dir, "logs", "t_dep_block.log"), "w").write(
    "HTTP 402: Insufficient Balance\n"
)
# C5. blocked + gave_up + payload={} + providers alive
#     → unblock (старая ветка, регресс-тест)
con.execute(
    "INSERT INTO tasks VALUES ('t_gave_up','t','blocked','backend',NULL,0,?)",
    (now,),
)
con.execute(
    "INSERT INTO task_events (task_id,kind,payload,created_at) "
    "VALUES ('t_gave_up','gave_up','{}',?)",
    (now,),
)
open(os.path.join(board_dir, "logs", "t_gave_up.log"), "w").write(
    "HTTP 402: Insufficient Balance\n"
)
# C6. blocked + payload мусор + providers alive → парсер НЕ падает, action НЕТ
con.execute(
    "INSERT INTO tasks VALUES ('t_bad_json','t','blocked','backend',NULL,0,?)",
    (now,),
)
con.execute(
    "INSERT INTO task_events (task_id,kind,payload,created_at) "
    "VALUES ('t_bad_json','blocked','{not valid json',?)",
    (now,),
)
open(os.path.join(board_dir, "logs", "t_bad_json.log"), "w").write(
    "HTTP 402: Insufficient Balance\n"
)
# probe_clean — чистый свежий лог для signal "providers_alive"
con.execute(
    "INSERT INTO tasks VALUES ('t_probe_clean','t','ready','backend',NULL,0,?)",
    (now,),
)
open(os.path.join(board_dir, "logs", "t_probe_clean.log"), "w").write(
    "normal worker log, no provider errors\n"
)
con.commit()
con.close()
PYEOF
[ -f "$BOARD_DIR/kanban.db" ] || fail "fixture db not created"

# --- таймстампы логов -------------------------------------------------------
# blocked-логи (с маркерами) старые → не считаются "живыми" провайдерами.
# t_probe_clean.log свежий → providers_alive=True (run A).
# Run B: удаляем probe_clean → providers_alive=False.
touch -d '1 minute ago' "$BOARD_DIR/logs/t_cap_unblock.log"
touch -d '1 minute ago' "$BOARD_DIR/logs/t_needs_input.log"
touch -d '1 minute ago' "$BOARD_DIR/logs/t_dep_block.log"
touch -d '1 minute ago' "$BOARD_DIR/logs/t_gave_up.log"
touch -d '1 minute ago' "$BOARD_DIR/logs/t_bad_json.log"
touch -d '1 minute ago' "$BOARD_DIR/logs/t_probe_clean.log"

# --- запускаем python-блок с тестовым окружением ------------------------------
run_provider_scan() {
    local actions_file="$1"
    cat > "$WORK/run.py" <<RUN
import sys, os, glob, time, sqlite3
sys.argv = ["watchdog", "$WORK", "$WORK/boards", "600", "15", "$actions_file"]
issues = []
recovery = []
now = int(time.time())
boards_dir = "$WORK/boards"
provider_actions_file = "$actions_file"
exec(open("$WORK/provider_scan.py").read())
print("ACTIONS:", provider_actions)
print("PROVIDERS_ALIVE:", providers_alive)
RUN
    python3 "$WORK/run.py"
}

# Прогон A — providers_alive=True (probe_clean свежий)
OUT_A="$(run_provider_scan "$WORK/actions_a.txt" 2>&1)" || { echo "$OUT_A"; fail "python block (run A) crashed"; }
echo "=== RUN A ==="; echo "$OUT_A"
[ -f "$WORK/actions_a.txt" ] || fail "actions_a.txt not written"
grep -q "^unblock|robbox|t_cap_unblock$" "$WORK/actions_a.txt" || fail "C1: unblock для t_cap_unblock (blocked+payload.kind=capability+alive) не найден"
grep -q "^unblock|robbox|t_gave_up$" "$WORK/actions_a.txt" || fail "C5: регресс — unblock для t_gave_up (старая ветка) не найден"
grep -q "t_needs_input" "$WORK/actions_a.txt" && fail "C3: НЕ должно быть action для t_needs_input (ручной блок, guard)"
grep -q "t_dep_block" "$WORK/actions_a.txt" && fail "C4: НЕ должно быть action для t_dep_block (ручной блок, guard)"
grep -q "t_bad_json" "$WORK/actions_a.txt" && fail "C6: НЕ должно быть action для t_bad_json (парсер молча скипнул)"

# Прогон B — providers_alive=False (probe_clean удалён)
# При providers_alive=False provider_actions остаётся пустым и файл не пишется
# (см. блок «if provider_actions and provider_actions_file» в watchdog.sh) — это
# ожидаемое поведение, а не баг.
rm -f "$BOARD_DIR/logs/t_probe_clean.log"
OUT_B="$(run_provider_scan "$WORK/actions_b.txt" 2>&1)" || { echo "$OUT_B"; fail "python block (run B) crashed"; }
echo "=== RUN B ==="; echo "$OUT_B"
if [ -f "$WORK/actions_b.txt" ]; then
    grep -q "t_cap_unblock" "$WORK/actions_b.txt" && fail "C2: НЕ должно быть unblock для t_cap_unblock при providers NOT alive"
    grep -q "t_gave_up" "$WORK/actions_b.txt" && fail "C2-regress: НЕ должно быть unblock для t_gave_up при providers NOT alive"
else
    echo "ok: B — actions_b.txt не создан (provider_actions пуст) — это ОК"
fi

pass "capability-recovery: все кейсы прошли (C1,C2,C3,C4,C5,C6)"
