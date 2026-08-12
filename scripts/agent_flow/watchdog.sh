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
TELEGRAM_STUCK_MINUTES=15     # reconnect-loop detection window (retro 12.08 t_5af222ea)
RESTART_COOLDOWN_FILE="$HERMES_HOME/state/watchdog.last_restart"

mkdir -p "$(dirname "$RESTART_COOLDOWN_FILE")"

log() { printf '[watchdog] %s\n' "$*" >&2; }

# Delegate all detection to a Python helper so we can use the bundled
# Python 3 (with sqlite3) without depending on the sqlite3 CLI.
python3 - "$HERMES_HOME" "$KANBAN_BOARDS_DIR" "$HEARTBEAT_STALE_SECONDS" "$TELEGRAM_STUCK_MINUTES" \
    <<'PYEOF'
import os, sys, glob, subprocess, time
from datetime import datetime

hermes_home = sys.argv[1]
boards_dir = sys.argv[2]
stale_sec = int(sys.argv[3])
telegram_stuck_min = int(sys.argv[4]) if len(sys.argv) > 4 else 15

issues = []
recovery = []
workers_by_unit = set()   # systemd units hosting live kanban worker pids

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
            # which unit hosts this live worker (used by telegram self-heal
            # to avoid restarting a gateway that hosts in-flight workers)
            if pid and pid > 0:
                try:
                    with open(f"/proc/{int(pid)}/cgroup", encoding="utf-8") as cf:
                        cg = cf.read()
                    for tok in cg.split():
                        if "app.slice/" in tok:
                            unit = tok.split("app.slice/", 1)[1].split("/", 1)[0]
                            if unit:
                                workers_by_unit.add(unit)
                except Exception:
                    pass
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

# 3.5 telegram: duplicate token holders + reconnect loops (retro 12.08 t_5af222ea)
# Root cause of the 22h reconnect loop: several profiles shared ONE
# TELEGRAM_BOT_TOKEN (.env copied by the update), only one gateway can hold
# it, the rest retried every 300s forever and the watchdog was silent.
telegram_issues = []

def _profile_name(home_path: str) -> str:
    base = os.path.basename(home_path.rstrip("/"))
    return "default" if base == ".hermes" else base

# 3.5a root cause: profiles with an ACTIVE TELEGRAM_BOT_TOKEN in .env
token_profiles = []
for home_path in [hermes_home] + sorted(glob.glob(f"{hermes_home}/profiles/*")):
    env_path = os.path.join(home_path, ".env")
    if not os.path.isfile(env_path):
        continue
    try:
        for ln in open(env_path, encoding="utf-8", errors="replace"):
            if ln.startswith("TELEGRAM_BOT_TOKEN="):
                val = ln.split("=", 1)[1].strip().strip('"').strip("'")
                if val:
                    token_profiles.append(_profile_name(home_path))
                break
    except Exception:
        pass
if len(token_profiles) > 1:
    telegram_issues.append(
        f"[telegram] duplicate TELEGRAM_BOT_TOKEN holders: {', '.join(token_profiles)} — "
        "only ONE profile should hold the bot token; comment it out in the others' .env"
    )

# 3.5b reconnect loop per running gateway (log-based — gateway_state.json is
# unreliable: it keeps a stale per-platform entry from the previous process)
def _parse_log_ts(line: str) -> float:
    try:
        return datetime.strptime(line[:19], "%Y-%m-%d %H:%M:%S").timestamp()
    except Exception:
        return 0.0

def _proc_start_epoch(pid: str) -> float:
    """Epoch of a process start (from /proc/<pid>/stat field 22)."""
    try:
        with open(f"/proc/{pid}/stat", encoding="utf-8") as f:
            data = f.read()
        rpar = data.rfind(")")
        fields = data[rpar + 2:].split()
        start_ticks = int(fields[19])  # field 22 (1-based) after comm
        with open("/proc/uptime", encoding="utf-8") as f:
            uptime = float(f.read().split()[0])
        clk = os.sysconf("SC_CLK_TCK")
        return time.time() - uptime + start_ticks / clk
    except Exception:
        return 0.0

def _gateway_home(profile: str) -> str:
    return hermes_home if profile == "default" else f"{hermes_home}/profiles/{profile}"

def _gateway_unit(profile: str) -> str:
    return "hermes-gateway.service" if profile == "default" else f"hermes-gateway-{profile}.service"

def _systemctl_restart(unit: str) -> bool:
    env = dict(os.environ)
    uid = os.getuid()
    env.setdefault("XDG_RUNTIME_DIR", f"/run/user/{uid}")
    env.setdefault("DBUS_SESSION_BUS_ADDRESS", f"unix:path=/run/user/{uid}/bus")
    try:
        subprocess.run(
            ["systemctl", "--user", "restart", unit],
            capture_output=True, text=True, timeout=60, env=env,
        )
        return True
    except Exception:
        return False

now = int(time.time())
stuck_cutoff = now - telegram_stuck_min * 60
stuck_gateways = []   # (profile, unit, pid)
for line in out.stdout.splitlines()[1:]:
    if "hermes_cli.main" not in line or "gateway run" not in line:
        continue
    parts = line.split(None, 1)
    if len(parts) != 2:
        continue
    gw_pid = parts[0]
    gw_args = parts[1]
    profile = "default"
    if "--profile" in gw_args:
        try:
            profile = gw_args.split("--profile", 1)[1].split()[0].strip()
        except Exception:
            pass
    log_path = os.path.join(_gateway_home(profile), "logs", "gateway.log")
    if not os.path.isfile(log_path):
        continue
    try:
        last_hit = None
        with open(log_path, encoding="utf-8", errors="replace") as f:
            for ln in f:
                if "already in use" in ln or "Reconnecting telegram" in ln:
                    last_hit = ln
        if last_hit:
            hit_ts = _parse_log_ts(last_hit)
            proc_start = _proc_start_epoch(gw_pid)
            # stuck only if the CURRENT process is the one retrying: the
            # last hit must be newer than both the cutoff and the process start
            if hit_ts >= stuck_cutoff and hit_ts >= proc_start - 5:
                stuck_gateways.append((profile, _gateway_unit(profile), gw_pid))
    except Exception:
        continue

for profile, unit, gw_pid in stuck_gateways:
    if unit in workers_by_unit:
        telegram_issues.append(
            f"[telegram] gateway {unit} (pid={gw_pid}) stuck in token-lock reconnect loop but "
            f"hosts in-flight kanban workers — restart deferred; will self-heal when workers finish"
        )
        continue
    if _systemctl_restart(unit):
        telegram_issues.append(
            f"[telegram] gateway {unit} (pid={gw_pid}) stuck in token-lock reconnect loop — "
            f"restarted (unit now reloads .env without the token)"
        )
        recovery.append(f"telegram|{unit}|restarted")
    else:
        telegram_issues.append(
            f"[telegram] gateway {unit} (pid={gw_pid}) stuck in token-lock reconnect loop — "
            f"restart FAILED, manual: systemctl --user restart {unit}"
        )

issues.extend(telegram_issues)

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
