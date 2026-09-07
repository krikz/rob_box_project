#!/bin/bash
# ============================================================================
# agent-flow-decomposed-watchdog.sh — cron-надзор за детьми, которых dispatcher
# не поднимает после декомпозиции эпика (ADR-0052, nightly-review t_bfd19ffb).
#
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-decomposed-watchdog.sh
# Copies are laid down by install.sh into:
#   - ~/.hermes/profiles/agent-flow/scripts/
#   - ~/.hermes/profiles/architect/scripts/
#   - ~/.hermes/profiles/devops/scripts/
#   - ~/.hermes/profiles/backend/scripts/
#   - ~/.hermes/profiles/analyst/scripts/
#   - ~/.hermes/scripts/
#
# Контекст (ADR-0052 §1, ретро t_bfd19ffb):
#   Декомпозиция эпика через kanban create иногда создаёт child-задачи со
#   started_at=NULL, status=todo (или status=triage), и dispatcher их не
#   поднимает: у этих детей нет parent_id в task_links (системный баг — 17
#   из 20 последних decomposed-рутов не имеют task_links), dispatcher смотрит
#   только на status+assignee+priority. Результат: эпик висит мёртвым
#   грузом (232ч на AV-11, 100ч на AV-27), block_recurrences растёт, Шифу
#   узнаёт только из ретро.
#
#   Этот cron — компенсирующий надзор: каждые 4ч сканирует task_events
#   kind='decomposed', для каждого ребёнка проверяет criteria (см. ADR-0052
#   §2.2) и для match'нутых пишет ОДИН marker-комментарий в task_comments
#   ребёнка (idempotent в сутки) + делает priority += 1.
#
# ЧТО НЕ ДЕЛАЕМ (явно):
#   - НЕ reassign — Шифу / agent-flow решает. Auto-reassign = потеря контекста.
#   - НЕ unblock — root может быть в blocked по валидной причине (capability).
#   - НЕ создаём новые карточки — только comment + priority bump.
#   - НЕ лечим баг пустых task_links — это отдельная задача. Watchdog
#     компенсирует симптом СЕЙЧАС.
#
# Контракт (per tick):
#   1. flock lock (не два тика одновременно)
#   2. iterate over all kanban boards under ~/.hermes/kanban/boards/*/
#      (если KANBAN_DB_PATH задан — обрабатываем только эту БД; для тестов)
#   3. SELECT task_events.kind='decomposed' AND created_at > now-MAX_AGE
#      AND created_at < now-AGE_THRESHOLD → parse payload.child_ids
#   4. SELECT root task — проверяем status IN ('todo','ready','blocked','triage')
#   5. for each child: SELECT tasks WHERE id=child_id — проверяем started_at IS
#      NULL AND status IN ('todo','triage')
#   6. SELECT 1 FROM task_comments WHERE body LIKE '%<MARKER>%' AND
#      created_at > today_start_utc AND task_id=child_id → если 0 rows →
#      emit comment через `hermes kanban --board X comment` + UPDATE
#      tasks SET priority = priority + 1
#   7. Log stats: scanned, matched, skipped_idempotent, emitted, errors
#
# ENV:
#   KANBAN_DB_PATH              — single DB override (test mode). Если НЕ задан,
#                                 сканируем все ~/.hermes/kanban/boards/*/kanban.db
#   KANBAN_BOARD                — имя board для `hermes kanban comment` (single-DB mode)
#   DECOMPOSED_WATCHDOG_DRY_RUN=true — log only, no comment, no UPDATE
#   AGE_THRESHOLD_SECONDS       — default 86400 (24h) — минимальный возраст
#   MAX_AGE_SECONDS             — default 2592000 (30d) — guard против
#                                 архивного долга
#   PRIORITY_BUMP               — default 1 (на сколько повышать priority)
#   MARKER_TAG                  — default "🤖 [agent:agent-flow] script=agent-flow-decomposed-watchdog"
#   LOCK_FILE                   — flock guard (default /tmp/agent-flow-decomposed-watchdog.lock)
#   LOG_FILE                    — stats log (default /tmp/agent-flow-decomposed-watchdog.log)
#   HERMES_CLI                  — hermes binary (default: hermes)
#   KANBAN_BOARDS_DIR           — base для multi-board scan (default
#                                 /home/builder/.hermes/kanban/boards)
#
# Выходы:
#   - Stderr: structured summary (for cron delivery).
#   - Exit 0 — всё ok (даже если ничего не закомментировали).
#   - Exit 1 — критичный сбой (нет python3 / sqlite3 module / lock fail).
#   - Exit 2 — кого-то разбудили (alert для cron, опционально).
#
# Pitfalls (gotchas):
#   - task_events.payload — JSON в виде TEXT. json.loads может упасть на
#     не-валидном payload (был случай в t_8b193e5e 2026-09-04, payload="null").
#     Guard: try/except + skip.
#   - task_events.created_at — INTEGER (epoch seconds). Сравниваем с now_s
#     в epoch, не ISO-строками.
#   - task_comments.created_at — INTEGER (epoch seconds), сравниваем с
#     today_start_utc (00:00:00Z в epoch). Не с локальным TZ.
#   - WAL-mode kanban.db позволяет SELECT параллельно с INSERT dispatcher'а.
#     UPDATE priority делаем прямо через sqlite (отдельный connection);
#     если конкуренция с dispatcher'ом — last-write-wins, обе записи
#     монотонны (priority += 1), поэтому idempotency не ломается.
#   - priority-bump +1 может «перебить» ручной priority — допустимо, ребёнок-то
#     спит. ADR-0052 §5.
# ============================================================================
set -euo pipefail

DRY_RUN="${DECOMPOSED_WATCHDOG_DRY_RUN:-false}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-decomposed-watchdog.lock}"
LOG_FILE="${LOG_FILE:-/tmp/agent-flow-decomposed-watchdog.log}"
AGE_THRESHOLD_SECONDS="${AGE_THRESHOLD_SECONDS:-86400}"   # 24h
MAX_AGE_SECONDS="${MAX_AGE_SECONDS:-2592000}"             # 30d guard
PRIORITY_BUMP="${PRIORITY_BUMP:-1}"
MARKER_TAG="${MARKER_TAG:-🤖 [agent:agent-flow] script=agent-flow-decomposed-watchdog}"
KANBAN_BOARDS_DIR="${KANBAN_BOARDS_DIR:-/home/builder/.hermes/kanban/boards}"
HERMES_CLI="${HERMES_CLI:-hermes}"

# --- flock guard ------------------------------------------------------------
exec 9>"$LOCK_FILE" || true
if ! flock -n 9; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] decomposed-watchdog: another instance running — skip" >&2
    exit 0
fi

# --- pre-flight: python3 + sqlite3 module ----------------------------------
if ! command -v python3 >/dev/null 2>&1; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] decomposed-watchdog: python3 not on PATH — exit 1" >&2
    exit 1
fi
python3 -c "import sqlite3, json, sys" 2>/dev/null || {
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] decomposed-watchdog: python3 modules (sqlite3/json) missing — exit 1" >&2
    exit 1
}

# --- helpers ---------------------------------------------------------------
_now_iso() { date -u +%Y-%m-%dT%H:%M:%SZ; }
_now_s()   { date -u +%s; }

# compute_today_start_utc — epoch seconds для 00:00:00Z сегодняшнего дня.
_today_start_utc() {
    python3 - <<'PYEOF'
import datetime, calendar
dt = datetime.datetime.utcnow().replace(hour=0, minute=0, second=0, microsecond=0)
print(int(calendar.timegm(dt.timetuple())))
PYEOF
}

# --- discover DB list ------------------------------------------------------
if [ -n "${KANBAN_DB_PATH:-}" ]; then
    # Single-DB mode (test, CI).
    _db_list="$KANBAN_DB_PATH"
    _default_board="${KANBAN_BOARD:-default}"
else
    # Production: scan all boards.
    _db_list="$(ls "${KANBAN_BOARDS_DIR}"/*/kanban.db 2>/dev/null || true)"
    _default_board=""
fi

if [ -z "$_db_list" ]; then
    echo "[$(_now_iso)] decomposed-watchdog: no kanban.db found under ${KANBAN_BOARDS_DIR} — exit 0" >&2
    exit 0
fi

_now="$(_now_s)"
_today_start="$(_today_start_utc)"

# Aggregate counters (bash-side; python emits per-DB partials via sentinel
# __STATS__:scanned:matched:emitted:skipped_idempotent:errors).
_scanned_total=0
_matched_total=0
_emitted_total=0
_skipped_idempotent_total=0
_errors_total=0

# --- main loop -------------------------------------------------------------
while IFS= read -r db_path; do
    [ -n "$db_path" ] || continue
    [ -f "$db_path" ] || continue

    # Извлекаем board из пути (/.../boards/<board>/kanban.db).
    if [ -n "${KANBAN_DB_PATH:-}" ]; then
        _board="${_default_board}"
    else
        _board="$(basename "$(dirname "$db_path")")"
    fi

    # Один python3 heredoc: scan + criteria + idempotency + emit + UPDATE + stats.
    # Sentinel lines:
    #   __STATS__:scanned:matched:emitted:skipped_idempotent:errors
    #   __RECORD__:task_id|board|parent_id|age_h|priority_before|priority_after
    while IFS= read -r line; do
        case "$line" in
            __STATS__:*)
                local_part="${line#__STATS__:}"
                IFS=':' read -r s m e si er <<< "$local_part"
                _scanned_total=$(( _scanned_total + s ))
                _matched_total=$(( _matched_total + m ))
                _emitted_total=$(( _emitted_total + e ))
                _skipped_idempotent_total=$(( _skipped_idempotent_total + si ))
                _errors_total=$(( _errors_total + er ))
                ;;
            __RECORD__:*)
                # Запись для stats log (формат: t_<id>|<board>|<parent_id>|<age_h>|<prio_before>|<prio_after>)
                printf '  %s\n' "${line#__RECORD__:}"
                ;;
            *)
                # Прочий stdout (debug) — pass-through в stderr.
                [ -n "$line" ] && echo "$line" >&2
                ;;
        esac
    done < <(
        python3 - "$db_path" "$_board" "$_now" "$_today_start" \
                "$AGE_THRESHOLD_SECONDS" "$MAX_AGE_SECONDS" "$PRIORITY_BUMP" \
                "$MARKER_TAG" "$DRY_RUN" "$HERMES_CLI" \
                <<'PYEOF'
import sqlite3
import subprocess
import sys
import time

db_path        = sys.argv[1]
board          = sys.argv[2]
now_s          = int(sys.argv[3])
today_start    = int(sys.argv[4])
age_threshold  = int(sys.argv[5])
max_age        = int(sys.argv[6])
priority_bump  = int(sys.argv[7])
marker_tag     = sys.argv[8]
dry_run        = (sys.argv[9].lower() == "true")
hermes_cli     = sys.argv[10]

# Statuses ребёнка, которые считаем «спящими» (dispatcher не поднял).
SLEEPING_CHILD_STATUSES = {"todo", "triage"}
# Statuses root-эпика, которые считаем «живыми» (есть кого будить).
LIVE_ROOT_STATUSES = {"todo", "ready", "blocked", "triage"}

def log(msg):
    print(f"[{time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}] "
          f"decomposed-watchdog: {msg}", file=sys.stderr)

try:
    con = sqlite3.connect(db_path, timeout=10)
    con.row_factory = sqlite3.Row
except sqlite3.OperationalError as e:
    log(f"SKIP board={board} db={db_path} err=operational:{e}")
    print(f"__STATS__:0:0:0:0:0")
    sys.exit(0)

stats = {"scanned": 0, "matched": 0, "emitted": 0,
         "skipped_idempotent": 0, "errors": 0, "records": []}

try:
    # 1. SELECT decomposed-события в окне (24ч..30д назад).
    cur = con.execute(
        "SELECT id, task_id, payload, created_at FROM task_events "
        "WHERE kind='decomposed' "
        "  AND created_at >= ? AND created_at <= ? "
        "ORDER BY created_at DESC",
        (now_s - max_age, now_s - age_threshold),
    )
    decomposed_events = cur.fetchall()
except sqlite3.OperationalError as e:
    log(f"SKIP board={board} query_decomposed err={e}")
    con.close()
    print(f"__STATS__:0:0:0:0:0")
    sys.exit(0)

stats["scanned"] = len(decomposed_events)

# 2. Для каждого decomposed — проверить root и каждого ребёнка.
import json

for ev in decomposed_events:
    ev_id        = ev["id"]
    parent_id    = ev["task_id"]
    payload_raw  = ev["payload"] or "{}"
    decomposed_at = int(ev["created_at"])

    # 2a. Парсим payload — может быть битым JSON (см. ADR-0052 §8 pitfalls).
    try:
        payload = json.loads(payload_raw)
    except (json.JSONDecodeError, TypeError) as e:
        log(f"SKIP ev={ev_id} parent={parent_id} err=payload-not-json:{type(e).__name__}")
        continue
    if not isinstance(payload, dict):
        log(f"SKIP ev={ev_id} parent={parent_id} err=payload-not-dict")
        continue

    child_ids = payload.get("child_ids", [])
    if not isinstance(child_ids, list) or not child_ids:
        log(f"SKIP ev={ev_id} parent={parent_id} err=no-child-ids")
        continue

    # 2b. Проверяем root — должен быть в LIVE_ROOT_STATUSES.
    try:
        root_row = con.execute(
            "SELECT status FROM tasks WHERE id=?", (parent_id,)
        ).fetchone()
    except sqlite3.OperationalError as e:
        log(f"SKIP ev={ev_id} parent={parent_id} err=root-query:{e}")
        continue

    if root_row is None:
        log(f"SKIP ev={ev_id} parent={parent_id} err=root-deleted")
        continue
    root_status = root_row["status"] or ""
    if root_status not in LIVE_ROOT_STATUSES:
        log(f"SKIP ev={ev_id} parent={parent_id} root_status={root_status} (not live)")
        continue

    # 2c. Для каждого ребёнка — проверяем criteria.
    for child_id in child_ids:
        if not isinstance(child_id, str) or not child_id.startswith("t_"):
            log(f"SKIP ev={ev_id} parent={parent_id} err=invalid-child-id:{child_id!r}")
            continue

        try:
            child_row = con.execute(
                "SELECT status, started_at, priority FROM tasks WHERE id=?",
                (child_id,),
            ).fetchone()
        except sqlite3.OperationalError as e:
            stats["errors"] += 1
            log(f"ERROR ev={ev_id} child={child_id} err=child-query:{e}")
            continue

        if child_row is None:
            log(f"SKIP ev={ev_id} child={child_id} err=child-deleted")
            continue

        child_status   = child_row["status"] or ""
        child_started  = child_row["started_at"]
        child_priority = child_row["priority"] or 0

        # Criteria (ADR-0052 §2.2):
        # 1. started_at IS NULL — ребёнок ни разу не был поднят
        if child_started is not None:
            continue
        # 2. status ∈ {todo, triage} — dispatcher ещё не взял
        if child_status not in SLEEPING_CHILD_STATUSES:
            continue

        stats["matched"] += 1

        # 3. Idempotency — за сегодня уже был marker?
        try:
            idem_cur = con.execute(
                "SELECT 1 FROM task_comments "
                "WHERE task_id=? AND body LIKE ? AND created_at > ? LIMIT 1",
                (child_id, f"%{marker_tag}%", today_start),
            )
            if idem_cur.fetchone() is not None:
                stats["skipped_idempotent"] += 1
                log(f"SKIP {child_id} parent={parent_id} (idempotent — comment уже есть за today)")
                continue
        except sqlite3.OperationalError:
            # task_comments не существует — это OK, пустая idempotency.
            pass

        age_h = (now_s - decomposed_at) // 3600
        age_rem_min = ((now_s - decomposed_at) % 3600) // 60
        comment_body = (
            f"{marker_tag}\n"
            f"action=wake-up reason=decomposed-no-pickup "
            f"parent={parent_id} root_status={root_status}\n"
            f"decomposed {age_h}ч{age_rem_min}м назад (>{age_threshold//3600}ч threshold), "
            f"child status={child_status} started_at=NULL.\n"
            f"priority bump: {child_priority} → {child_priority + priority_bump}.\n"
            f"Watchdog компенсирует симптом (17/20 decomposed-рутов без task_links — "
            f"см. ADR-0052 §1.2). Шифу / agent-flow — оцени."
        )

        if dry_run:
            stats["emitted"] += 1
            log(f"[DRY-RUN] {child_id} parent={parent_id} would-comment "
                f"age={age_h}ч{age_rem_min}м priority {child_priority}→{child_priority+priority_bump}")
            stats["records"].append((child_id, board, parent_id, age_h,
                                     child_priority, child_priority + priority_bump))
            continue

        # 4a. Comment через hermes kanban --board comment.
        try:
            proc = subprocess.run(
                [hermes_cli, "kanban", "--board", board, "comment", child_id, comment_body],
                capture_output=True, text=True, timeout=15, check=False,
            )
            if proc.returncode != 0:
                stats["errors"] += 1
                log(f"ERROR comment {child_id} rc={proc.returncode} "
                    f"stderr={proc.stderr.strip()[:200]}")
                continue
        except subprocess.TimeoutExpired:
            stats["errors"] += 1
            log(f"ERROR timeout comment {child_id}")
            continue
        except Exception as e:
            stats["errors"] += 1
            log(f"ERROR exception comment {child_id} err={type(e).__name__}:{e}")
            continue

        # 4b. Priority bump — через отдельный connection (отдельный writer,
        # не блокируем WAL-mode SELECT выше). Используем тот же con для
        # простоты — sqlite WAL сериализует writer'ов.
        try:
            con.execute(
                "UPDATE tasks SET priority = priority + ? WHERE id=?",
                (priority_bump, child_id),
            )
            con.commit()
        except sqlite3.OperationalError as e:
            stats["errors"] += 1
            log(f"ERROR priority-bump {child_id} err={e}")
            continue

        stats["emitted"] += 1
        stats["records"].append((child_id, board, parent_id, age_h,
                                 child_priority, child_priority + priority_bump))
        log(f"WOKE-UP {child_id} parent={parent_id} board={board} "
            f"age={age_h}ч{age_rem_min}м priority {child_priority}→{child_priority+priority_bump}")

con.close()

print(f"__STATS__:{stats['scanned']}:{stats['matched']}:"
      f"{stats['emitted']}:{stats['skipped_idempotent']}:{stats['errors']}")
for r in stats["records"]:
    print(f"__RECORD__:{r[0]}|{r[1]}|{r[2]}|{r[3]}|{r[4]}|{r[5]}")
PYEOF
    )
done < <(printf '%s\n' "$_db_list")

# --- structured summary ----------------------------------------------------
echo "[$(_now_iso)] decomposed-watchdog: ✓ done scanned=${_scanned_total} matched=${_matched_total} emitted=${_emitted_total} skipped_idempotent=${_skipped_idempotent_total} errors=${_errors_total}" >&2

# --- write stats log -------------------------------------------------------
mkdir -p "$(dirname "$LOG_FILE")" 2>/dev/null || true
{
    printf '# decomposed-watchdog snapshot %s\n' "$(_now_iso)"
    printf 'scanned=%s matched=%s emitted=%s skipped_idempotent=%s errors=%s\n' \
        "$_scanned_total" "$_matched_total" "$_emitted_total" \
        "$_skipped_idempotent_total" "$_errors_total"
} >> "$LOG_FILE" 2>/dev/null || true

# --- exit code -------------------------------------------------------------
# exit 2 если emit'или хоть один (alert для cron), exit 0 если нечего.
# exit 1 уже обработан выше (нет python3).
if [ "$_emitted_total" -gt 0 ] && [ "$DRY_RUN" != "true" ]; then
    exit 2
fi
exit 0
