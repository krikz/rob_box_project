#!/bin/bash
# hermes-kanban-orphan-skill-watchdog.sh
# ----------------------------------------------------------------
# Detects Kanban tasks that crash on startup with "Unknown skill(s)"
# and (in self-healing mode) auto-fixes the skill list to a valid
# skill from the assignee profile, so the task can be dispatched
# without human intervention.
#
# Modes:
#   --detective  (default for older behaviour) — block the card with
#                an architectural comment so it stops wasting
#                dispatcher cycles. A human must edit the body before
#                the dispatcher retries.
#   --self-heal  (default NOW) — replace orphan skills with the first
#                valid skill from the assignee profile. If the profile
#                has no skills, drop the per-task skills entirely so
#                the worker uses the profile defaults. Emit an
#                ``orphan_skill_auto_fixed`` task_event and a comment
#                for audit. The task stays in ``ready`` and is
#                dispatched on the next tick.
#
# Why this exists:
#   - Pre-create skill validation (ADR-0036 §4.1, vendor-patch
#     scripts/agent_flow/vendor/hermes-agent-skill-validation.patch)
#     IS now wired into local hermes-cli (hermes_cli/kanban_db.py
#     ``_validate_skills_for_assignee``), so NEW cards with orphan
#     skills are rejected at create-time.
#   - But CARDS ALREADY BLOCKED before the patch landed
#     (retrospective t_09a2152b / t_51394ac6: t_84cb9466,
#     t_8572890f) are stuck in a watchdog loop that ONLY blocks,
#     never fixes. Without self-healing, the user/architect has to
#     manually edit each card body.
#   - This watchdog short-circuits that loop: orphan-skill crashes
#     are auto-fixed in-place, the card is recorded for audit, and
#     the dispatcher re-attempts on the next tick. ``--detective``
#     is preserved for environments that prefer strict blocking.
# ----------------------------------------------------------------
set -u

BOARD="${HERMES_KANBAN_BOARD:-robbox}"
DB="${HERMES_KANBAN_BOARD_DB:-/home/builder/.hermes/kanban/boards/${BOARD}/kanban.db}"
LOOKBACK_MIN="${LOOKBACK_MIN:-30}"
LOG_DIR="/home/builder/.hermes/kanban/boards/${BOARD}/logs"
PY="/home/builder/.hermes/hermes-agent/venv/bin/python3"
HERMES_BIN="${HERMES_BIN:-/home/builder/.hermes/hermes-agent/venv/bin/hermes}"

# Mode: detective = block only; self-heal = auto-fix (default).
# Override with --detective on the cron line if you want strict mode.
MODE="self-heal"
for arg in "$@"; do
    case "$arg" in
        --detective) MODE="detective" ;;
        --self-heal) MODE="self-heal" ;;
    esac
done

[ -d "$LOG_DIR" ] || { echo "no log dir at $LOG_DIR" >&2; exit 0; }
[ -f "$DB" ] || { echo "no board db at $DB" >&2; exit 0; }

# Idempotency: read SIGTERM-style marker file so multiple ticks don't
# re-block/re-fix the same card. Each handled task id is recorded here
# for 24h. Marker key includes the mode so a switch from detective to
# self-heal (or vice versa) reprocesses cards.
MARKER=/tmp/orphan-skill-watchdog-${BOARD}-${MODE}-seen.txt
touch "$MARKER"

# Scan:
#  1. *.log for "Unknown skill(s)" lines in last LOOKBACK_MIN minutes
#  2. task_runs.error in DB for the same string
# Map log-file task ids to their card ids via DB lookup, then handle
# each (block in detective mode, auto-fix in self-heal mode).

"$PY" - "$DB" "$LOG_DIR" "$LOOKBACK_MIN" "$MARKER" "$MODE" "$HERMES_BIN" <<'PY'
import os, sqlite3, sys, time, subprocess, json, re

db_path, log_dir, lookback_min, marker, mode, hermes_bin = sys.argv[1:7]
since_unix = int(time.time()) - int(lookback_min)*60
problem_ids = set()

# --- (1) scan logs ---
patt = re.compile(r"Error: Unknown skill\(s\): (.+)")
for fname in os.listdir(log_dir):
    p = os.path.join(log_dir, fname)
    mtime = int(os.path.getmtime(p))
    if mtime < since_unix: continue
    if not fname.endswith(".log"): continue
    task_id = fname[:-4]
    try:
        with open(p, "r", errors="replace") as f:
            content = f.read()
    except Exception:
        continue
    for line in content.splitlines():
        mm = patt.search(line)
        if mm:
            problem_ids.add((task_id, mm.group(1).strip()))
            break

# --- (2) query DB for orphan-skill runs as a fallback ---
conn = sqlite3.connect(db_path)
cur = conn.cursor()
cur.execute("""
  SELECT r.task_id, r.error
    FROM task_runs r
   WHERE r.error LIKE '%Unknown skill%'
     AND r.started_at >= ?
   ORDER BY r.started_at DESC
   LIMIT 50
""", (since_unix,))
for task_id, err in cur.fetchall():
    mm = patt.search(err or "")
    if mm:
        problem_ids.add((task_id, mm.group(1).strip()))

if not problem_ids:
    print(f"[watchdog/{mode}] no orphan-skill crashes in last {lookback_min}m")
    sys.exit(0)

print(f"[watchdog/{mode}] detected {len(problem_ids)} orphan-skill event(s)")

def already_handled(tid):
    try:
        with open(marker) as f:
            return any(line.startswith(tid+"|") and int(line.strip().split("|")[1]) > int(time.time())-86400 for line in f)
    except Exception:
        return False

def mark_handled(tid):
    with open(marker, "a") as f:
        f.write(f"{tid}|{int(time.time())}\n")

def record_event(task_id, kind, payload):
    conn2 = sqlite3.connect(db_path)
    cur2 = conn2.cursor()
    cur2.execute(
        "INSERT INTO task_events (task_id, run_id, kind, payload, created_at, cost_in_cents) VALUES (?, NULL, ?, ?, strftime('%s','now'), 0)",
        (task_id, kind, json.dumps(payload))
    )
    conn2.commit()
    conn2.close()

def assignee_first_skill(assignee):
    """Return the first installed skill name for ``assignee`` (sorted),
    or None if the profile has no skills / is unknown / is unreadable.

    Mirrors hermes_cli/kanban_db.py::_profile_skill_names so the
    watchdog's choice of replacement skill matches the runtime
    validation. Keeps a tiny import surface — only stdlib + agent
    helpers, no hermes_cli dependency (the watchdog must run even when
    hermes-cli is half-broken).
    """
    if not assignee:
        return None
    try:
        # Defer to hermes-cli if importable (matches runtime validation).
        import hermes_cli.kanban_db as kb
        names = kb._profile_skill_names(assignee)
    except Exception:
        # Fallback: walk ~/.hermes/profiles/<assignee>/skills/SKILL.md
        # via the same iter_skill_index_files helper. If even that
        # fails (no Python path), try a flat glob as last resort.
        try:
            from agent.skill_utils import iter_skill_index_files
            from pathlib import Path
            prof_dir = Path(f"/home/builder/.hermes/profiles/{assignee}")
            skills_dir = prof_dir / "skills"
            if not skills_dir.is_dir():
                return None
            names = set()
            for md in iter_skill_index_files(skills_dir, "SKILL.md"):
                if md.parent.name:
                    names.add(md.parent.name)
            names = frozenset(names) if names else None
        except Exception:
            return None
    if not names:
        return None
    return sorted(names)[0]

def set_skills(task_id, new_skills):
    """Replace the ``skills`` column on ``tasks`` with ``new_skills``
    (JSON-encoded list, or NULL if empty)."""
    conn2 = sqlite3.connect(db_path)
    cur2 = conn2.cursor()
    payload = json.dumps(new_skills) if new_skills else None
    cur2.execute(
        "UPDATE tasks SET skills = ? WHERE id = ?",
        (payload, task_id)
    )
    conn2.commit()
    conn2.close()

def post_comment(tid, body):
    return subprocess.run(
        [hermes_bin, "kanban", "--board", "robbox", "comment", tid, body],
        capture_output=True, text=True
    )

def block_card(tid, reason):
    return subprocess.run(
        [hermes_bin, "kanban", "--board", "robbox", "block", tid,
         "--kind", "transient", reason],
        capture_output=True, text=True
    )

def unblock_card(tid):
    """Move a transient-blocked card back to ready so the dispatcher
    picks it up on the next tick. Best-effort; harmless if already
    unblocked."""
    return subprocess.run(
        [hermes_bin, "kanban", "--board", "robbox", "unblock", tid],
        capture_output=True, text=True
    )

for tid, orphan_skills in problem_ids:
    # Sanity check — is this card still orphan, or already replaced?
    cur.execute("SELECT status, skills, assignee FROM tasks WHERE id=?", (tid,))
    row = cur.fetchone()
    if not row:
        continue
    status, skills, assignee = row
    if mode == "detective" and status not in ('blocked', 'todo', 'ready', 'running'):
        print(f"[watchdog/{mode}] skip {tid}: status={status!r} (already terminal/finished)")
        continue

    # Confirm skills column still references the orphan name (not
    # already fixed by an earlier tick or by a human edit).
    arr = []
    if skills:
        try:
            arr = json.loads(skills)
        except Exception:
            arr = []
    # The orphan_skills string from the regex can be a comma-separated
    # list ("foo, bar"); normalise into a set for membership checks.
    orphan_set = {s.strip() for s in orphan_skills.split(",") if s.strip()}
    if not any(o in orphan_set for o in arr):
        # All orphans already gone from the skills list — nothing to do.
        print(f"[watchdog/{mode}] skip {tid}: no orphan in current skills={arr}")
        continue

    if already_handled(tid):
        print(f"[watchdog/{mode}] skip {tid}: already handled within 24h")
        continue

    if mode == "detective":
        comment = (
            "🛑 **orphan-skill crash detected** by watchdog\n\n"
            f"- task: `{tid}`\n"
            f"- assignee: `{assignee}`\n"
            f"- orphan skills: `{orphan_skills}`\n"
            f"- these are NOT installed in the assignee's profile — "
            f"see agent-flow SOT `scripts/agent_flow/vendor/"
            f"hermes-agent-skill-validation.patch`.\n\n"
            "**Action taken:** card blocked (transient) to stop the crash-loop. "
            "Author must edit the body to use a valid skill from the assignee "
            "profile, or replace it with an empty list, before reopening.\n\n"
            "_Auto-blocked by `hermes-kanban-orphan-skill-watchdog.sh`._"
        )
        record_event(tid, "orphan_skill_auto_blocked", {"orphan": orphan_skills, "assignee": assignee, "mode": mode})
        res = post_comment(tid, comment)
        res2 = block_card(tid, f"orphan-skill crash: {orphan_skills} not in profile {assignee}")
        mark_handled(tid)
        print(f"[watchdog/{mode}] blocked {tid} (orphan={orphan_skills!r}, assignee={assignee}): comment_rc={res.returncode} block_rc={res2.returncode}")
        continue

    # ── self-heal path ──
    # Decide replacement skills: drop orphans, keep survivors, fill
    # the empty list with the first installed skill from the assignee
    # profile (so the worker still has something to load — empty
    # skills=[] would force profile defaults, which may also be
    # wrong; picking a deterministic valid skill is safer).
    survivor = [s for s in arr if s not in orphan_set]
    if not survivor:
        # All skills were orphans. Look up the first valid skill in the
        # profile and use it as the replacement.
        replacement = assignee_first_skill(assignee)
        if replacement:
            new_skills = [replacement]
            action = "replaced-with-first-installed"
        else:
            # Profile has no installed skills — drop per-task skills
            # entirely so the worker uses its default skills (or runs
            # bare). This is the only safe fallback when we can't
            # determine a valid replacement.
            new_skills = []
            action = "dropped-all-skills-no-profile-skills"
    else:
        # Some skills were valid — keep those, drop only orphans.
        new_skills = survivor
        action = "dropped-orphans-kept-survivors"

    set_skills(tid, new_skills)
    record_event(tid, "orphan_skill_auto_fixed", {
        "orphan": orphan_skills,
        "assignee": assignee,
        "old_skills": arr,
        "new_skills": new_skills,
        "action": action,
        "mode": mode,
    })
    comment = (
        "🩹 **orphan-skill auto-fix** by watchdog (self-heal mode)\n\n"
        f"- task: `{tid}`\n"
        f"- assignee: `{assignee}`\n"
        f"- orphan skills removed: `{orphan_skills}`\n"
        f"- replacement: **{action}** → `{new_skills or '(empty)'}`\n\n"
        "Pre-create skill validation (ADR-0036 §4.1, wired in "
        "hermes_cli/kanban_db.py) now rejects new cards with orphan "
        "skills. This watchdog fixes cards created BEFORE the patch "
        "landed so the dispatcher can pick them up on the next tick.\n\n"
        "_Auto-fixed by `hermes-kanban-orphan-skill-watchdog.sh --self-heal`._"
    )
    res_c = post_comment(tid, comment)
    # If the card was previously blocked by the same watchdog, unblock
    # it so the dispatcher re-attempts. Idempotent.
    res_u = unblock_card(tid) if status == "blocked" else None
    mark_handled(tid)
    print(f"[watchdog/{mode}] fixed {tid}: orphan={orphan_skills!r} → new_skills={new_skills} (action={action}); comment_rc={res_c.returncode} unblock_rc={getattr(res_u, 'returncode', 'n/a')}")

conn.close()
PY
