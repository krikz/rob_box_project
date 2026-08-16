#!/bin/bash
# ============================================================================
# test_watchdog_provider_exhaustion.sh — ретро 15.08 t_3b9fadc5
#
# Регресс-гард для provider-exhaustion секции watchdog.sh: при 402/429
# (MiniMax 429/2056, DeepSeek 402) воркер печатает «Out of credits» и выходит
# rc=0 → dispatcher считает это protocol violation x2 → gave_up → карточка
# blocked навсегда. Фикс: watchdog детектит маркеры 402/429 в логах воркеров
# и (а) блокирует упавшие/готовые карточки с причиной «провайдер исчерпан,
# ждать» вместо protocol-violation цикла; (б) разблокирует карточки, уже
# заблокированные dispatcher'ом (gave_up), когда провайдеры снова отвечают.
#
# Проверяем на фикстуре изолированной kanban.db + logs/:
#   1. blocked карточка + 402 в логе + свежий лог без маркеров (providers
#      alive) → action unblock
#   2. running карточка + мёртвый pid + 429 в логе → action block
#   3. ready карточка + маркеры в логе + провайдеры НЕ живы → action block
#   4. ready карточка + маркеры + провайдеры живы → НЕТ action (dispatcher
#      сам респавнит)
#   5. blocked карточка без маркеров в логе → НЕТ action
#   6. blocked карточка с маркерами, но последнее событие kind='blocked'
#      (ручной block Шифу) → НЕТ action (guard от ручных блоков)
#
# Run:
#   bash scripts/agent_flow/tests/test_watchdog_provider_exhaustion.sh
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
# заканчивается перед «# 2. dispatcher status».
awk '/# 1c\. provider-exhaustion/{f=1} f{print} f && /^# 2\. dispatcher status/{exit}' \
    "$WATCHDOG_SH" > "$WORK/provider_scan.py"
[ -s "$WORK/provider_scan.py" ] || fail "provider-exhaustion block not found in $WATCHDOG_SH"

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
# 1. blocked + 402 в логе (dispatcher gave_up) — провайдеры живы → unblock
con.execute("INSERT INTO tasks VALUES ('t_unblock_me','t','blocked','backend',NULL,0,?)", (now,))
con.execute("INSERT INTO task_events (task_id,kind,payload,created_at) VALUES ('t_unblock_me','gave_up','{}',?)", (now,))
open(os.path.join(board_dir, "logs", "t_unblock_me.log"), "w").write(
    "Billing or credits exhausted: HTTP 402: Insufficient Balance\nOut of credits\n")
# 2. running + мёртвый pid + 429 в логе → block
con.execute("INSERT INTO tasks VALUES ('t_running_dead','t','running','devops',999999,0,?)", (now,))
open(os.path.join(board_dir, "logs", "t_running_dead.log"), "w").write(
    "anthropic.RateLimitError 429 Token Plan usage limit (2056)\n")
# 3. ready + маркеры + провайдеры НЕ живы (свежего лога без маркеров нет) → block
con.execute("INSERT INTO tasks VALUES ('t_ready_noprov','t','ready','devops',NULL,0,?)", (now,))
open(os.path.join(board_dir, "logs", "t_ready_noprov.log"), "w").write("HTTP 429: rate limit\n")
# 4. ready + маркеры + провайдеры живы (свежий чистый лог есть) → НЕТ action
con.execute("INSERT INTO tasks VALUES ('t_ready_alive','t','ready','devops',NULL,0,?)", (now,))
open(os.path.join(board_dir, "logs", "t_ready_alive.log"), "w").write("HTTP 429: rate limit\n")
#   4a. чистый свежий лог-проба (без маркеров) — сигнал «провайдеры живы».
#   В прогоне B удаляем его → providers_alive=False.
con.execute("INSERT INTO tasks VALUES ('t_probe_clean','t','ready','devops',NULL,0,?)", (now,))
open(os.path.join(board_dir, "logs", "t_probe_clean.log"), "w").write("normal worker log, no provider errors\n")
# 5. blocked без маркеров → НЕТ action
con.execute("INSERT INTO tasks VALUES ('t_blocked_clean','t','blocked','devops',NULL,0,?)", (now,))
con.execute("INSERT INTO task_events (task_id,kind,payload,created_at) VALUES ('t_blocked_clean','gave_up','{}',?)", (now,))
open(os.path.join(board_dir, "logs", "t_blocked_clean.log"), "w").write("some normal log\n")
# 6. blocked + маркеры, но ручной block (kind='blocked') → НЕТ action (guard)
con.execute("INSERT INTO tasks VALUES ('t_manual_block','t','blocked','devops',NULL,0,?)", (now,))
con.execute("INSERT INTO task_events (task_id,kind,payload,created_at) VALUES ('t_manual_block','blocked','{}',?)", (now,))
open(os.path.join(board_dir, "logs", "t_manual_block.log"), "w").write("HTTP 402: Insufficient Balance\n")
con.commit()
con.close()
PYEOF
[ -f "$BOARD_DIR/kanban.db" ] || fail "fixture db not created"

# --- провайдеры живы: свежий чистый лог без маркеров (для теста 4) -----------
touch -d '1 minute ago' "$BOARD_DIR/logs/t_ready_alive.log"
touch -d '1 minute ago' "$BOARD_DIR/logs/t_probe_clean.log"
# --- старим лог с маркерами ready-карточки, чтобы providers_alive НЕ
# --- считал его «живым» (для теста 3: маркерные логи не считаются живыми) ---
touch -d '1 minute ago' "$BOARD_DIR/logs/t_ready_noprov.log"
# blocked-лог тоже свежий, но с маркерами → не считается «живым»
touch -d '1 minute ago' "$BOARD_DIR/logs/t_unblock_me.log"
touch -d '1 minute ago' "$BOARD_DIR/logs/t_running_dead.log"
# t_blocked_clean без маркеров — состариваем, чтобы он НЕ делал providers_alive
# в прогоне B (единственный «живой» чистый лог — t_probe_clean.log, его мы
# удаляем между прогонами)
touch -d '3 days ago' "$BOARD_DIR/logs/t_blocked_clean.log"
touch -d '1 minute ago' "$BOARD_DIR/logs/t_manual_block.log"

# --- запускаем python-блок с тестовым окружением ------------------------------
# providers_alive вычисляется глобально по доске (есть свежий лог без маркеров
# 402/429). Кейсы 3 и 4 взаимоисключающие → два прогона:
#   Прогон A: t_ready_alive.log свежий и чистый → providers_alive=True
#             (проверяем unblock, running-dead block, ready-alive NO action,
#              clean-blocked NO action, manual-block NO action)
#   Прогон B: t_ready_alive.log убран (старый) → providers_alive=False
#             (проверяем ready-noprov block, unblock НЕ происходит)
run_provider_scan() {
    local actions_file="$1"
    cat > "$WORK/run.py" <<RUN
import sys, os, glob, time, sqlite3
sys.argv = ["watchdog", "$WORK", "$WORK/boards", "600", "15", "$actions_file"]
# Заглушки, чтобы извлечённый блок (который ссылается на внешние переменные
# issues/recovery/now/boards_dir) работал: переопределяем контекст.
issues = []
recovery = []
now = int(time.time())
boards_dir = "$WORK/boards"
provider_actions_file = "$actions_file"
exec(open("$WORK/provider_scan.py").read())
# Блок сам пишет actions.txt; печатаем, что насобирал
print("ACTIONS:", provider_actions)
print("PROVIDERS_ALIVE:", providers_alive)
RUN
    python3 "$WORK/run.py"
}

# Прогон A — providers_alive=True
OUT_A="$(run_provider_scan "$WORK/actions_a.txt" 2>&1)" || { echo "$OUT_A"; fail "python block (run A) crashed"; }
echo "=== RUN A ==="; echo "$OUT_A"
[ -f "$WORK/actions_a.txt" ] || fail "actions_a.txt not written"
grep -q "^unblock|robbox|t_unblock_me$" "$WORK/actions_a.txt" || fail "A1: unblock для t_unblock_me (blocked+402+alive) не найден"
grep -q "^block|robbox|t_running_dead$" "$WORK/actions_a.txt" || fail "A2: block для t_running_dead (running+мёртвый pid+429) не найден"
grep -q "t_ready_alive" "$WORK/actions_a.txt" && fail "A4: НЕ должно быть action для t_ready_alive (ready+маркеры+providers alive)"
grep -q "t_blocked_clean" "$WORK/actions_a.txt" && fail "A5: НЕ должно быть action для t_blocked_clean (blocked без маркеров)"
grep -q "t_manual_block" "$WORK/actions_a.txt" && fail "A6: НЕ должно быть action для t_manual_block (ручной block, guard)"

# Прогон B — providers_alive=False (убираем единственный чистый свежий лог)
rm -f "$BOARD_DIR/logs/t_probe_clean.log"
OUT_B="$(run_provider_scan "$WORK/actions_b.txt" 2>&1)" || { echo "$OUT_B"; fail "python block (run B) crashed"; }
echo "=== RUN B ==="; echo "$OUT_B"
[ -f "$WORK/actions_b.txt" ] || fail "actions_b.txt not written"
grep -q "^block|robbox|t_ready_noprov$" "$WORK/actions_b.txt" || fail "B3: block для t_ready_noprov (ready+маркеры+providers NOT alive) не найден"
grep -q "t_unblock_me" "$WORK/actions_b.txt" && fail "B7: НЕ должно быть unblock для t_unblock_me при providers NOT alive"

pass "provider-exhaustion: все кейсы прошли (A1,A2,A4,A5,A6,B3,B7)"
