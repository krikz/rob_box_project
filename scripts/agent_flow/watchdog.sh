#!/bin/bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/watchdog.sh
# Каноническая версия живёт в репо. На хост раскладывается через
# `bash <repo>/scripts/agent_flow/install.sh`, который создаёт
# символические ссылки в:
#   - ~/.hermes/profiles/agent-flow/scripts/watchdog.sh
#   - ~/.hermes/profiles/architect/scripts/watchdog.sh
#   - ~/.hermes/scripts/watchdog.sh
# Правка: редактируем <repo>/scripts/agent_flow/watchdog.sh, commit, merge.
# На хост: bash <repo>/scripts/agent_flow/install.sh (или вручную cp + ln -sf).
# Если ты правишь этот файл НА ХОСТЕ руками — синхронизируй обратно в репо.
# ============================================================================
# Agent Cockpeat / Flow Watch Tock — Agent heartbeat watchdog.
# Runs every 2 minutes via `hermes cron` (no LLM, this script IS the job).
#
# Per the Hermes Multi-Agent-2 video (Namreg, 35:30):
#   - Verifies kanban DB integrity
#   - Detects stuck tasks (heartbeat older than 10 minutes for a running task)
#   - Restarts the dispatcher if there's no running task but ready tasks exist
#   - Creates a recovery card if a worker died or hit timeout
#   - Prunes dead worker PIDs
#
# Output: Markdown status, printed only when something needs attention.
# Empty stdout = silent tick (no tokens consumed).

set -euo pipefail

HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
KANBAN_BOARDS_DIR="$HERMES_HOME/kanban/boards"
HEARTBEAT_STALE_SECONDS=600   # 10 min
RESTART_COOLDOWN_FILE="$HERMES_HOME/state/watchdog.last_restart"

mkdir -p "$(dirname "$RESTART_COOLDOWN_FILE")"

log() { printf '[watchdog] %s\n' "$*" >&2; }

# Delegate all detection to a Python helper so we can use the bundled
# Python 3 (with sqlite3) without depending on the sqlite3 CLI.
python3 - "$HERMES_HOME" "$KANBAN_BOARDS_DIR" "$HEARTBEAT_STALE_SECONDS" \
    <<'PYEOF'
import os, sys, glob, subprocess, time

hermes_home = sys.argv[1]
boards_dir = sys.argv[2]
stale_sec = int(sys.argv[3])

issues = []
recovery = []

for db in sorted(glob.glob(f"{boards_dir}/*/kanban.db")):
    if not os.path.isfile(db):
        continue
    board = os.path.basename(os.path.dirname(db))
    import sqlite3
    try:
        con = sqlite3.connect(db)
        # 1a. integrity check
        cur = con.execute("PRAGMA integrity_check(1)")
        if cur.fetchone()[0] != "ok":
            issues.append(f"[{board}] kanban.db integrity FAILED")
            con.close()
            continue
        # 1b. running tasks with stale heartbeat
        now = int(time.time())
        cur = con.execute(
            "SELECT id, COALESCE(worker_pid,0), COALESCE(last_heartbeat_at,0) "
            "FROM tasks WHERE status='running'"
        )
        for task_id, pid, heartbeat in cur.fetchall():
            age = now - int(heartbeat)
            if age >= stale_sec:
                alive = False
                if pid and pid > 0:
                    try:
                        os.kill(int(pid), 0)
                        alive = True
                    except (OSError, ProcessLookupError):
                        alive = False
                kind = "stuck" if alive else "dead"
                issues.append(f"[{board}] {task_id} {kind} (pid={pid} {'alive' if alive else 'gone'}, heartbeat={age}s old)")
                recovery.append(f"{board}|{task_id}|{kind}")
        con.close()
    except Exception as exc:
        issues.append(f"[{board}] db error: {exc}")

# 2. dispatcher status
dispatcher_alive = False
try:
    # Use ps + grep instead of pgrep -f to avoid self-match on the
    # watchdog subprocess's own command line. Filter out the watchdog
    # itself by excluding this script's basename.
    out = subprocess.run(
        ["ps", "-eo", "pid,comm,args"],
        capture_output=True, text=True
    )
    for line in out.stdout.splitlines()[1:]:
        # Skip ps, grep, and the watchdog subprocess itself
        if "ps -eo" in line:
            continue
        if "watchdog.sh" in line:
            continue
        if "hermes" in line and "kanban" in line and "dispatch" in line:
            dispatcher_alive = True
            break
except Exception:
    dispatcher_alive = False

if not dispatcher_alive:
    issues.append("[dispatcher] no dispatcher process running")

# 3. restart dispatcher if dead (with cooldown)
restarted = False
cooldown_file = f"{hermes_home}/state/watchdog.last_restart"
if not dispatcher_alive:
    cooldown_ok = True
    if os.path.exists(cooldown_file):
        try:
            last = int(open(cooldown_file).read().strip())
            if int(time.time()) - last < 300:
                cooldown_ok = False
        except Exception:
            pass
    if cooldown_ok:
        boards = sorted(glob.glob(f"{boards_dir}/*/kanban.db"))
        if boards:
            first_board = os.path.basename(os.path.dirname(boards[0]))
            with open(cooldown_file, "w") as f:
                f.write(str(int(time.time())))
            log_path = f"{hermes_home}/logs/dispatcher.log"
            os.makedirs(os.path.dirname(log_path), exist_ok=True)
            uvx = f"{hermes_home}/bin/uvx"
            subprocess.Popen(
                [uvx, "--from", "hermes-agent", "hermes", "kanban",
                 "--board", first_board, "dispatch"],
                stdout=open(log_path, "ab"), stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            restarted = True

# 4. report
if not issues:
    print(f"[watchdog] tick OK (dispatcher={'alive' if dispatcher_alive else 'dead'}, restart={restarted})", file=sys.stderr)
    sys.exit(0)

print(f"## Watchdog Tick — {time.strftime('%Y-%m-%dT%H:%M:%S%z')}")
print()
print(f"**Issues:** {len(issues)}")
print()
for issue in issues:
    print(f"- ⚠️ {issue}")
if recovery:
    print()
    print(f"**Recovery candidates:** {len(recovery)}")
    for rc in recovery:
        print(f"- 🔄 {rc}")
print()
print(f"Dispatcher: {'alive' if dispatcher_alive else 'dead'} (restarted: {restarted})")
PYEOF
