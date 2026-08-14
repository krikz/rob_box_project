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

# --- RUN_NOW trigger (ретро 12.08): сигнальный файл в develop → немедленный  ---
# e2e-process тикает каждый час; RUN_NOW в origin/develop означает «прогон
# прямо сейчас». Watchdog (every 2m) проверяет наличие файла и дергает
# e2e-process немедленно (без ожидания hourly тика). Сам e2e-process потом
# удалит файл после прогона (cleanup в конце тика).
GH_REPO="${GH_REPO:-krikz/rob_box_project}"
RUN_NOW_FILE="${RUN_NOW_FILE:-RUN_NOW}"
RUN_NOW_LOCK="${RUN_NOW_LOCK:-/tmp/agent-flow-run-now.lock}"
E2E_PROCESS_SCRIPT="${E2E_PROCESS_SCRIPT:-/home/builder/.hermes/scripts/agent-flow-e2e-process.sh}"
REPO_DIR="${REPO_DIR:-/home/builder/hermes-share/rob_box_project}"
E2E_LOCK_FILE="${E2E_LOCK_FILE:-/tmp/agent-flow-e2e-process.lock}"

if gh api "repos/${GH_REPO}/contents/${RUN_NOW_FILE}?ref=develop" --jq '.name' >/dev/null 2>&1; then
    # Есть RUN_NOW → запускаем e2e-process (если он не запущен и не в процессе).
    # Проверяем по flock e2e-process (а не по lock-файлу watchdog) — flock
    # надёжнее: если e2e-process уже держит lock, второй инстанс не нужен.
    if [ -f "$E2E_LOCK_FILE" ] && exec 9>"$E2E_LOCK_FILE" && ! flock -n 9 2>/dev/null; then
        log "⏳ RUN_NOW detected but e2e-process already holds flock ($E2E_LOCK_FILE) — skip"
        exec 9>&-
    else
        exec 9>&-
        log "🚀 RUN_NOW detected — triggering e2e-process (${E2E_PROCESS_SCRIPT})"
        nohup bash "$E2E_PROCESS_SCRIPT" \
            >> "$HERMES_HOME/logs/agent-flow-e2e-process-run-now.log" 2>&1 &
        echo $! > "$RUN_NOW_LOCK"
        log "🚀 e2e-process triggered (pid=$!)"
    fi
fi

# --- e2e-process liveness (надзор 13.08): авто-рестарт при краше ------------
# Падаван-вахта (LLM-крон) перезапускает e2e-process вручную по правилу
# «не тикал >90 мин», но её тик может быть убит квотой LLM (429/2056) — как
# 12.08 23:16 UTC, когда e2e-process упал (BrokenPipeError в python-пайпе
# round-69) и очередь needs-e2e (19 PR) встала на ~1ч. Watchdog (no-agent,
# без LLM) должен быть самодостаточен: если e2e-process мёртв (flock
# свободен), а needs-e2e PR-ы есть — поднимаем заново. Второй инстанс
# невозможен: e2e-process держит flock весь тик (G6).
_e2e_needs_e2e="$(gh api "repos/${GH_REPO}/pulls?state=open&per_page=100" \
    --jq '[.[] | select(.labels[]?.name == "needs-e2e")] | length' 2>/dev/null || echo 0)"
if [ "${_e2e_needs_e2e:-0}" -gt 0 ] \
    && flock -n "$E2E_LOCK_FILE" -c true 2>/dev/null; then
    log "🚑 e2e-process не держит flock (краш/завис), needs-e2e PR-ов: ${_e2e_needs_e2e} — перезапуск"
    nohup bash "$E2E_PROCESS_SCRIPT" \
        >> "$HERMES_HOME/logs/agent-flow-e2e-process-run-now.log" 2>&1 &
    echo $! > "$RUN_NOW_LOCK"
    log "🚀 e2e-process restarted (pid=$!)"
fi

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
# Реальный диспетчер — внутренний loop gateway'а профиля agent-flow
# (hermes-gateway-agent-flow.service), а НЕ одноразовый процесс
# 'hermes kanban dispatch' (спавн → exit). Живость определяем по
# процессу 'gateway run --profile agent-flow' и свежести heartbeat.
def _dispatcher_heartbeat() -> str:
    """Путь к heartbeat gateway'а agent-flow (реальный диспетчер)."""
    base = os.path.basename(hermes_home.rstrip("/"))
    if base == "agent-flow":  # HERMES_HOME уже профильный
        return os.path.join(hermes_home, "state", "gateway.heartbeat")
    return os.path.join(hermes_home, "profiles", "agent-flow", "state", "gateway.heartbeat")

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
        if "hermes_cli.main" in line and "gateway run" in line and "--profile agent-flow" in line:
            dispatcher_alive = True
            break
except Exception:
    dispatcher_alive = False

# fallback: свежий heartbeat gateway'а = диспетчер жив (даже если ps-матч
# не сработал — например, изменилась cmdline). Heartbeat пишется gateway'ом
# каждые ~2 мин, поэтому свежесть < stale_sec означает живой внутренний loop.
if not dispatcher_alive:
    try:
        hb_path = _dispatcher_heartbeat()
        if os.path.isfile(hb_path) and time.time() - os.path.getmtime(hb_path) < stale_sec:
            dispatcher_alive = True
    except Exception:
        pass

if not dispatcher_alive:
    issues.append("[dispatcher] gateway agent-flow (реальный диспетчер) не запущен")

# 3. restart dispatcher if dead AND ready tasks exist (with cooldown)
# uvx-рестарт УБРАН (ретро 13.08 t_901c790b): 'uvx --from hermes-agent hermes
# kanban dispatch' тянул hermes-agent из PyPI (32× установки, 4438 прогонов,
# Spawned: 0 — gateway-диспетчер делает всё сам; 1× 'Read-only file system'
# в uv-кэше). Вместо этого: если gateway agent-flow мёртв и есть ready-задачи —
# рестарт systemd-юнита (локально, без PyPI). Комментарий в шапке: рестарт
# только при ready-задачах.
restarted = False
cooldown_file = f"{hermes_home}/state/watchdog.last_restart"
if not dispatcher_alive:
    ready_count = 0
    for db in sorted(glob.glob(f"{boards_dir}/*/kanban.db")):
        try:
            con = sqlite3.connect(db)
            ready_count += con.execute("SELECT COUNT(*) FROM tasks WHERE status='ready'").fetchone()[0]
            con.close()
        except Exception:
            pass
    unit = "hermes-gateway-agent-flow.service"
    if ready_count > 0 and unit not in workers_by_unit:
        cooldown_ok = True
        if os.path.exists(cooldown_file):
            try:
                last = int(open(cooldown_file).read().strip())
                if int(time.time()) - last < 300:
                    cooldown_ok = False
            except Exception:
                pass
        if cooldown_ok:
            env = dict(os.environ)
            uid = os.getuid()
            env.setdefault("XDG_RUNTIME_DIR", f"/run/user/{uid}")
            env.setdefault("DBUS_SESSION_BUS_ADDRESS", f"unix:path=/run/user/{uid}/bus")
            # Страховка: рестартим только если юнит НЕ active (реально мёртв).
            # Активный юнит при устаревшем heartbeat = завис, но systemd
            # Restart=always сам поднимет при краше; ручной рестарт активного
            # юнита рискован при ложном детекте.
            try:
                ia = subprocess.run(
                    ["systemctl", "--user", "is-active", unit],
                    capture_output=True, text=True, timeout=30, env=env,
                )
                unit_active = ia.stdout.strip() == "active"
            except Exception:
                unit_active = False
            if unit_active:
                issues.append(f"[dispatcher] gateway {unit} active, но диспетчер не найден (heartbeat устарел?) — ручная проверка")
            else:
                with open(cooldown_file, "w") as f:
                    f.write(str(int(time.time())))
                log_path = f"{hermes_home}/logs/dispatcher.log"
                os.makedirs(os.path.dirname(log_path), exist_ok=True)
                try:
                    r = subprocess.run(
                        ["systemctl", "--user", "restart", unit],
                        capture_output=True, text=True, timeout=60, env=env,
                    )
                    restarted = r.returncode == 0
                    msg = f"[watchdog] dispatcher dead (ready={ready_count}) — restart {unit}: " + \
                          ("OK" if restarted else (r.stderr or r.stdout).strip()[:300])
                    with open(log_path, "ab") as lf:
                        lf.write((msg + "\n").encode())
                except Exception as exc:
                    restarted = False
                    with open(log_path, "ab") as lf:
                        lf.write(f"[watchdog] dispatcher dead (ready={ready_count}) — restart {unit} EXC: {exc}\n".encode())
    elif ready_count > 0:
        issues.append(f"[dispatcher] gateway {unit} hosts in-flight workers — restart deferred")

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

# ============================================================================
# Local worktree sweep (ретро 14.08 t_ee70ffc2)
# Проблема: /tmp/wt-* и /home/builder/wt-* worktree от завершённых карточек
# не чистятся: cleanup-249 чистит /tmp только на build-хосте 10.1.1.249,
# merge-gate free_stale_worktrees_for чистит только worktree своей карточки.
# Каждая завершённая карточка оставляет worktree-мусор (~2-3 ГБ).
# Решение: раз в WT_SWEEP_COOLDOWN_SEC (default 3600с) перебираем каталоги
# /tmp/wt-* и /home/builder/wt-*; если каталог старше WT_SWEEP_AGE_HOURS
# (default 48ч) и его ветки НЕТ в origin (git ls-remote) → git worktree
# remove --force + prune. Осиротевшие (gitdir отсутствует) → rm -rf.
# Guard: ветка есть в origin → не трогаем; develop/main/master → не трогаем;
# flock e2e/merge-gate/triage занят → тик пропускаем целиком.
# ============================================================================
WT_SWEEP_AGE_HOURS="${WT_SWEEP_AGE_HOURS:-48}"
WT_SWEEP_COOLDOWN_SEC="${WT_SWEEP_COOLDOWN_SEC:-3600}"
WT_SWEEP_STATE="${WT_SWEEP_STATE:-$HERMES_HOME/state/agent-flow-wt-sweep.last}"
WT_SWEEP_LOG="${WT_SWEEP_LOG:-$HERMES_HOME/logs/agent-flow-wt-sweep.log}"
WT_SWEEP_PATTERNS="${WT_SWEEP_PATTERNS:-/tmp/wt-* /home/builder/wt-*}"
WT_SWEEP_LOCKS="${WT_SWEEP_LOCKS:-/tmp/agent-flow-e2e-process.lock /tmp/agent-flow-merge-gate.lock /tmp/agent-flow-triage.lock}"
# Проектные worktree (.worktrees/<task_id>) от archived/done карточек (ретро 14.08 t_d007c365).
# kanban archive НЕ удаляет workspace/worktree, а `kanban gc` чистит только
# scratch-workspaces (workspace_kind != 'scratch' → skip). Поэтому .worktrees/t_*
# копятся (14.08: 42 шт, ~6.3 GB). Чистим по статусу карточки в kanban DB:
# archived/done → git worktree remove --force; running/ready/прочие и карточки,
# которых нет в DB → keep. Свой state-файл, чтобы оба sweep могли работать в
# одном тике (у локального sweep свой cooldown).
WT_PROJECT_WT_DIR="${WT_PROJECT_WT_DIR:-/home/builder/rob_box_project/.worktrees}"
WT_PROJECT_SWEEP_KANBAN_DB="${WT_PROJECT_SWEEP_KANBAN_DB:-$KANBAN_BOARDS_DIR/robbox/kanban.db}"
WT_PROJECT_SWEEP_STATE="${WT_PROJECT_SWEEP_STATE:-$HERMES_HOME/state/agent-flow-project-wt-sweep.last}"
WT_PROJECT_SWEEP_LOG="${WT_PROJECT_SWEEP_LOG:-$HERMES_HOME/logs/agent-flow-project-wt-sweep.log}"
WT_PROJECT_SWEEP_REMOVE_STATUSES="${WT_PROJECT_SWEEP_REMOVE_STATUSES:-archived done}"

sweep_stale_local_worktrees() {
    local now last_run age_cutoff pat d gitdir_file gitdir branch repo_root removed=0
    local lock
    # 1. cooldown: не чаще раза в WT_SWEEP_COOLDOWN_SEC
    if [ -f "$WT_SWEEP_STATE" ]; then
        last_run="$(cat "$WT_SWEEP_STATE" 2>/dev/null || echo 0)"
        now="$(date +%s)"
        [ $(( now - last_run )) -lt "$WT_SWEEP_COOLDOWN_SEC" ] && return 0
    fi
    # 2. flock-guard: активные процессы agent-flow → пропуск тика
    for lock in $WT_SWEEP_LOCKS; do
        if [ -f "$lock" ] && ! flock -n "$lock" -c true 2>/dev/null; then
            log "🛑 wt-sweep: $lock занят активным процессом — тик пропущен"
            return 0
        fi
    done
    # 3. кандидаты: каталоги по паттернам, старше AGE_HOURS (локально, без сети)
    age_cutoff=$(( $(date +%s) - WT_SWEEP_AGE_HOURS * 3600 ))
    local -a candidates=()
    local mtime
    for pat in $WT_SWEEP_PATTERNS; do
        for d in $pat; do
            [ -d "$d" ] || continue
            mtime="$(stat -c %Y "$d" 2>/dev/null || echo "")"
            [ -n "$mtime" ] || continue
            [ "$mtime" -lt "$age_cutoff" ] || continue
            candidates+=("$d")
        done
    done
    [ "${#candidates[@]}" -eq 0 ] && return 0
    # 4. рефы origin (один ls-remote на тик; сеть недоступна → пропуск)
    local remote_refs
    remote_refs="$(git -C "$REPO_DIR" ls-remote origin 2>/dev/null | awk '{print $2}' || true)"
    [ -n "$remote_refs" ] || { log "🛑 wt-sweep: ls-remote origin пуст/ошибка — тик пропущен"; return 0; }
    # 5. разбор кандидатов
    for d in "${candidates[@]}"; do
        gitdir_file="$d/.git"
        # не наш worktree (нет .git-файла) → не трогаем
        [ -f "$gitdir_file" ] || { log "wt-sweep: skip $d (нет .git-файла)"; continue; }
        gitdir="$(sed -n 's/^gitdir: //p' "$gitdir_file" | head -1)"
        # осиротевший каталог (.git-файл есть, gitdir отсутствует) → rm -rf
        if [ -z "$gitdir" ] || [ ! -d "$gitdir" ]; then
            rm -rf -- "$d" && {
                log "wt-sweep: removed orphan $d"
                echo "$(date -Is) REMOVED_ORPHAN $d" >> "$WT_SWEEP_LOG"
                removed=$((removed+1))
            }
            continue
        fi
        branch="$(git -C "$d" branch --show-current 2>/dev/null || true)"
        case "$branch" in
            develop|main|master) continue ;;
        esac
        # ветка в origin → активная, не трогаем
        if [ -n "$branch" ] && printf '%s\n' "$remote_refs" | grep -Fxq "refs/heads/$branch"; then
            continue
        fi
        repo_root="${gitdir%/.git/worktrees/*}"
        if [ -n "$repo_root" ] && [ -d "$repo_root/.git" ] \
            && git -C "$repo_root" worktree remove --force "$d" 2>/dev/null; then
            git -C "$repo_root" worktree prune 2>/dev/null || true
            log "wt-sweep: removed $d (branch '${branch:-detached}' не в origin)"
            echo "$(date -Is) REMOVED $d branch=${branch:-detached} repo=$repo_root" >> "$WT_SWEEP_LOG"
            removed=$((removed+1))
        else
            log "wt-sweep: FAILED remove $d (branch '${branch:-detached}')"
        fi
    done
    mkdir -p "$(dirname "$WT_SWEEP_STATE")"
    echo "$(date +%s)" > "$WT_SWEEP_STATE"
    [ "$removed" -gt 0 ] && log "wt-sweep: removed $removed stale worktree(s)"
    return 0
}

# ============================================================================
# sweep_stale_project_worktrees — проектные worktree .worktrees/<task_id>
# (ретро 14.08 t_d007c365: 41 archived/done карточка, ~6.3 GB не чистится —
# kanban archive не удаляет workspace, `kanban gc` чистит только scratch).
#
# Отличие от sweep_stale_local_worktrees: критерий удаления НЕ «ветки нет в
# origin» (у done-карточек t_9af45692/t_cc9cc56d/t_e0b221ff ветки ещё живут в
# origin), а статус карточки в kanban DB: archived|done → remove; running,
# ready и прочие, а также карточки, отсутствующие в DB → keep.
#
# Безопасность:
#   - смотрим только $WT_PROJECT_WT_DIR/t_* (не трогаем главный worktree
#     репозитория и чужие каталоги в .worktrees/);
#   - статус читаем из kanban DB (python3+sqlite3, как основной watchdog);
#   - DB недоступна/пуста → тик пропускается целиком (fail-closed);
#   - каталог без валидного .git-файла → не наш worktree, не трогаем;
#   - gitdir отсутствует → осиротевший каталог, rm -rf;
#   - удаление через git worktree remove --force + prune (не rm -rf руками),
#     чтобы не оставлять мусор в .git/worktrees/.
# ============================================================================
sweep_stale_project_worktrees() {
    local now last_run d tid status gitdir_file gitdir repo_root removed=0
    local lock db
    # 1. cooldown: не чаще раза в WT_SWEEP_COOLDOWN_SEC (свой state-файл)
    if [ -f "$WT_PROJECT_SWEEP_STATE" ]; then
        last_run="$(cat "$WT_PROJECT_SWEEP_STATE" 2>/dev/null || echo 0)"
        now="$(date +%s)"
        [ $(( now - last_run )) -lt "$WT_SWEEP_COOLDOWN_SEC" ] && return 0
    fi
    # 2. flock-guard: активные процессы agent-flow → пропуск тика
    for lock in $WT_SWEEP_LOCKS; do
        if [ -f "$lock" ] && ! flock -n "$lock" -c true 2>/dev/null; then
            log "🛑 project-wt-sweep: $lock занят активным процессом — тик пропущен"
            return 0
        fi
    done
    # 3. каталог проектных worktree и kanban DB должны существовать
    [ -d "$WT_PROJECT_WT_DIR" ] || return 0
    db="$WT_PROJECT_SWEEP_KANBAN_DB"
    [ -f "$db" ] || { log "🛑 project-wt-sweep: kanban DB $db не найден — тик пропущен"; return 0; }
    # 4. кандидаты: только t_* (карточки kanban), без age-фильтра — статус
    #    в DB является авторитетным (карточка может быть archived вчера).
    local -a candidates=()
    for d in "$WT_PROJECT_WT_DIR"/t_*; do
        [ -d "$d" ] || continue
        candidates+=("$d")
    done
    [ "${#candidates[@]}" -eq 0 ] && return 0
    # 5. статусы всех кандидатов одним python-вызовом (fail-closed: ошибка → пусто)
    local status_map
    status_map="$(python3 - "$db" "${candidates[@]}" <<'PYEOF' 2>/dev/null || true
import sqlite3, sys
db = sys.argv[1]
paths = sys.argv[2:]
ids = [p.rsplit("/", 1)[-1] for p in paths]
try:
    con = sqlite3.connect(db)
    try:
        rows = con.execute(
            "SELECT id, status FROM tasks WHERE id IN (%s)"
            % ",".join("?" * len(ids)),
            ids,
        ).fetchall()
    finally:
        con.close()
except Exception:
    sys.exit(0)
for tid, status in rows:
    print(f"{tid}\t{status}")
PYEOF
)"
    [ -n "$status_map" ] || { log "🛑 project-wt-sweep: kanban DB недоступен/пуст — тик пропущен"; return 0; }
    # 6. разбор кандидатов
    for d in "${candidates[@]}"; do
        tid="${d##*/}"
        gitdir_file="$d/.git"
        # не наш worktree (нет .git-файла) → не трогаем
        [ -f "$gitdir_file" ] || { log "project-wt-sweep: skip $d (нет .git-файла)"; continue; }
        gitdir="$(sed -n 's/^gitdir: //p' "$gitdir_file" | head -1)"
        # осиротевший каталог (.git-файл есть, gitdir отсутствует) → rm -rf
        # (это мусор независимо от статуса карточки — worktree не зарегистрирован)
        if [ -z "$gitdir" ] || [ ! -d "$gitdir" ]; then
            rm -rf -- "$d" && {
                log "project-wt-sweep: removed orphan $d (card $tid)"
                echo "$(date -Is) REMOVED_ORPHAN $d card=$tid" >> "$WT_PROJECT_SWEEP_LOG"
                removed=$((removed+1))
            }
            continue
        fi
        # статус карточки из status_map (tab-разделитель)
        status="$(printf '%s\n' "$status_map" | awk -F'\t' -v t="$tid" '$1==t {print $2; exit}')"
        # карточки нет в DB или статус не в списке на удаление → keep
        case " $WT_PROJECT_SWEEP_REMOVE_STATUSES " in
            *" $status "*) ;;
            *)
                log "project-wt-sweep: keep $d (card $tid status='${status:-<not-in-db>}')"
                continue ;;
        esac
        repo_root="${gitdir%/.git/worktrees/*}"
        if [ -n "$repo_root" ] && [ -d "$repo_root/.git" ] \
            && git -C "$repo_root" worktree remove --force "$d" 2>/dev/null; then
            git -C "$repo_root" worktree prune 2>/dev/null || true
            log "project-wt-sweep: removed $d (card $tid $status)"
            echo "$(date -Is) REMOVED $d card=$tid status=$status repo=$repo_root" >> "$WT_PROJECT_SWEEP_LOG"
            removed=$((removed+1))
        else
            log "project-wt-sweep: FAILED remove $d (card $tid $status)"
        fi
    done
    mkdir -p "$(dirname "$WT_PROJECT_SWEEP_STATE")"
    echo "$(date +%s)" > "$WT_PROJECT_SWEEP_STATE"
    [ "$removed" -gt 0 ] && log "project-wt-sweep: removed $removed stale project worktree(s)"
    return 0
}
sweep_stale_local_worktrees || true
sweep_stale_project_worktrees || true

# ============================================================================
# SOT→host auto-sync (ретро 13.08 t_767ab9b8)
# Проблема: фиксы agent-flow-*.sh в develop неактивны на хосте, пока кто-то
# вручную не запустит install.sh (лаг 13.07→13.14 UTC 13.08; при 429-квоте
# LLM-тики падаван-вахты не помогут — лаг неограничен).
# Решение: watchdog (no-agent, без LLM) сам сверяет SHA локальных
# agent-flow-*.sh с origin/develop SOT и при расхождении запускает install.sh
# (он сам делает .bak-бэкап перед заменой). Убирает зависимость от LLM-тиков.
# ============================================================================
SOT_SYNC_STATE="${SOT_SYNC_STATE:-$HERMES_HOME/state/agent-flow-sot-sync.last_sha}"
SOT_SYNC_LOG="${SOT_SYNC_LOG:-$HERMES_HOME/logs/agent-flow-sot-sync.log}"
SOT_SYNC_REPO="${SOT_SYNC_REPO:-$REPO_DIR}"
SOT_SYNC_SCRIPTS=(
    agent-flow-triage.sh
    agent-flow-merge-gate.sh
    agent-flow-e2e-process.sh
    agent-flow-handoff.sh
    round_ensure.sh
    agent-flow-cleanup-249.sh
    agent-flow-deploy-sweep.sh
    agent-flow-unlabeled-sweep.sh
    cron-loop.sh
    watchdog.sh
    agent-flow-drift-detect.sh
    install.sh
)

_sot_sync() {
    [ -d "$SOT_SYNC_REPO/.git" ] || { log "SOT-sync: repo $SOT_SYNC_REPO missing — skip"; return 0; }

    local remote_sha last_sha f sot_md5 loc_md5 drift=0
    remote_sha="$(git -C "$SOT_SYNC_REPO" ls-remote origin refs/heads/develop 2>/dev/null | awk '{print $1}')"
    [ -n "$remote_sha" ] || { log "SOT-sync: ls-remote failed (network/auth?) — skip"; return 0; }
    last_sha="$(cat "$SOT_SYNC_STATE" 2>/dev/null || true)"
    [ "$remote_sha" = "$last_sha" ] && return 0   # develop не двигался — ничего не делаем

    # fetch, чтобы origin/develop был свежим
    git -C "$SOT_SYNC_REPO" fetch --quiet origin develop 2>/dev/null \
        || { log "SOT-sync: fetch failed — skip"; return 0; }

    for f in "${SOT_SYNC_SCRIPTS[@]}"; do
        sot_md5="$(git -C "$SOT_SYNC_REPO" show "origin/develop:scripts/agent_flow/$f" 2>/dev/null | md5sum | awk '{print $1}')"
        loc_md5="$(md5sum "$HERMES_HOME/scripts/$f" 2>/dev/null | awk '{print $1}')"
        [ -n "$sot_md5" ] || continue
        if [ "$sot_md5" != "$loc_md5" ]; then
            drift=1
            log "SOT-sync: drift $f (local=$loc_md5 sot=$sot_md5)"
        fi
    done

    if [ "$drift" = "1" ]; then
        log "SOT-sync: drift detected — running install.sh (bak-копии делает сам install.sh)"
        local tmpdir
        tmpdir="$(mktemp -d /tmp/agent-flow-sot.XXXXXX)" || { log "SOT-sync: mktemp failed"; return 0; }
        # Раскладываем из origin/develop (git archive), а НЕ из рабочего дерева
        # репо: основной клон может сидеть на чужой worker-ветке.
        if git -C "$SOT_SYNC_REPO" archive origin/develop scripts/agent_flow 2>/dev/null \
            | tar -x -C "$tmpdir" 2>/dev/null; then
            if REPO_DIR="$tmpdir" bash "$tmpdir/scripts/agent_flow/install.sh" >>"$SOT_SYNC_LOG" 2>&1; then
                log "SOT-sync: install.sh OK — scripts synced to origin/develop"
                echo "$remote_sha" > "$SOT_SYNC_STATE"
            else
                log "SOT-sync: install.sh FAILED (see $SOT_SYNC_LOG) — state NOT updated, retry next tick"
            fi
        else
            log "SOT-sync: git archive failed — skip install"
        fi
        rm -rf "$tmpdir"
    else
        log "SOT-sync: develop moved ($last_sha→$remote_sha) but no script drift"
        echo "$remote_sha" > "$SOT_SYNC_STATE"
    fi
}
_sot_sync
