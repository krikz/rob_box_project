#!/bin/bash
# ============================================================================
# agent-flow-blocked-watchdog-scope.sh — cron-надзор mis-scope архитектурных
# карточек (ADR-0036 §4.3, ретро-карточка t_aa585aa7).
#
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-blocked-watchdog-scope.sh
# Copies are laid down by install.sh into:
#   - ~/.hermes/profiles/agent-flow/scripts/
#   - ~/.hermes/profiles/architect/scripts/
#   - ~/.hermes/profiles/devops/scripts/
#   - ~/.hermes/scripts/
#
# Контекст (ADR-0036 §4.3, ретро 31.08 t_da8bf7cd):
#   §4.2 runtime-overshoot watchdog ловит worker'ов с age > 4×max_runtime и
#   делает SIGTERM. Но это HARD detection. Если worker жив, шлёт heartbeat
#   и его max_runtime_seconds завышен (или watchdog не сработал по §4.1
#   keyword-miss) — архитектурная карточка в mis-assigned профиле может
#   висеть часами без реакции.
#
#   Этот cron — страховка для keyword-misses §4.1: ежечасно обходит все
#   kanban-доски и для running-карточек проверяет:
#     - age > 4 часа (started_at < now - 14400)
#     - assignee NOT IN ('architect', 'devops') — non-architect профили
#     - body LIKE '%ADR-%' — heuristic на «архитектурный» content
#   Если все три — emit ОДИН comment в карточку (idempotent через query
#   task_comments). Шифу eyeball'ит, решает что делать.
#
# ЧТО НЕ ДЕЛАЕМ (явно):
#   - НЕ SIGTERM / НЕ SIGKILL — это надзор, не kill. Только §4.2 watchdog
#     делает kill (auto-SIGTERM).
#   - НЕ reassign воркера — Шифу принимает решение. Auto-reassign = потеря
#     прогресса 5ч+ работы.
#   - НЕ LLM-based classifier — keyword heuristic достаточен (§4.1 ADR уже
#     доказал что LIKE '%ADR-%' покрывает 90% mis-scope).
#
# Контракт (per tick):
#   1. flock lock (не два тика одновременно)
#   2. iterate over all kanban boards under ~/.hermes/kanban/boards/*/
#      (если KANBAN_DB_PATH задан — обрабатываем только эту БД; это для
#      тестов и для CI-режима)
#   3. SELECT running-tasks со started_at < now-14400
#   4. for each candidate: python3-filter на assignee + body LIKE
#   5. for each match: SELECT 1 FROM task_comments WHERE body LIKE
#      '%<MARKER>%' AND created_at > today_start_utc AND task_id=?
#      → если 0 rows → emit comment через `hermes kanban --board X comment`
#   6. Log stats: scanned, matched, skipped_idempotent, emitted, errors
#
# ENV:
#   KANBAN_DB_PATH          — single DB override (test mode). Если НЕ задан,
#                             сканируем все ~/.hermes/kanban/boards/*/kanban.db
#   KANBAN_BOARD            — имя board для `hermes kanban comment` (single-DB mode)
#   BLOCKED_WATCHDOG_SCOPE_DRY_RUN=true — log only, no comment
#   AGE_THRESHOLD_SECONDS   — default 14400 (4ч)
#   MARKER_TAG              — default "⚠️ mis-scope надзор"
#   LOCK_FILE               — flock guard (default /tmp/agent-flow-blocked-watchdog-scope.lock)
#   LOG_FILE                — stats log (default /tmp/agent-flow-blocked-watchdog-scope.log)
#   HERMES_CLI              — hermes binary (default: hermes)
#
# Выходы:
#   - Stderr: structured summary (for cron delivery).
#   - Exit 0 — всё ok (даже если ничего не закомментили).
#   - Exit 1 — критичный сбой (нет python3 / sqlite3 module / lock fail).
#   - Exit 2 — найдены mis-scope карточки (alert для cron, опционально).
#
# Pitfalls (gotchas):
#   - "ADR-" — keyword может появляться в body не только для архитектурных
#     карточек (например, цитата ADR-0035 в тестах). Heuristic даёт
#     false-positive; Шифу eyeball'ит. Это ОК — over-alert лучше under-alert.
#   - assignee может быть NULL (для system-created задач). NULL ≠ 'architect' /
#     ≠ 'devops' → попадёт в criteria. Это намеренно — если Шифу создал
#     карточку без assignee и она висит, мы её пометим.
#   - DB может быть locked другим процессом (kanban dispatcher пишет).
#     WAL-mode решает конкуренцию для SELECT. INSERT (через hermes CLI,
#     не напрямую) идёт отдельным процессом.
#   - created_at в task_comments — INTEGER (epoch seconds), сравниваем с
#     today_start_utc (00:00:00Z в epoch). Не с локальным TZ.
# ============================================================================
set -euo pipefail

DRY_RUN="${BLOCKED_WATCHDOG_SCOPE_DRY_RUN:-false}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-blocked-watchdog-scope.lock}"
LOG_FILE="${LOG_FILE:-/tmp/agent-flow-blocked-watchdog-scope.log}"
AGE_THRESHOLD_SECONDS="${AGE_THRESHOLD_SECONDS:-14400}"  # 4h
MARKER_TAG="${MARKER_TAG:-⚠️ mis-scope надзор}"
KANBAN_BOARDS_DIR="${KANBAN_BOARDS_DIR:-/home/builder/.hermes/kanban/boards}"
HERMES_CLI="${HERMES_CLI:-hermes}"

# --- flock guard ------------------------------------------------------------
exec 9>"$LOCK_FILE" || true
if ! flock -n 9; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] blocked-watchdog-scope: another instance running — skip" >&2
    exit 0
fi

# --- pre-flight: python3 + sqlite3 module ----------------------------------
if ! command -v python3 >/dev/null 2>&1; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] blocked-watchdog-scope: python3 not on PATH — exit 1" >&2
    exit 1
fi
python3 -c "import sqlite3, json, sys" 2>/dev/null || {
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] blocked-watchdog-scope: python3 modules (sqlite3/json) missing — exit 1" >&2
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

# --- discover DB list -------------------------------------------------------
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
    echo "[$(_now_iso)] blocked-watchdog-scope: no kanban.db found under ${KANBAN_BOARDS_DIR} — exit 0" >&2
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

    # Один python3 heredoc: filter + idempotency + emit + stats.
    # ВАЖНО: shell-quoting внутри heredoc — body комментария собираем через
    # f-string и НЕ используем shell-переменных внутри docstring (используем
    # sys.argv). Все 8 параметров — positional args через env.
    #
    # Sentinel lines:
    #   __STATS__:scanned:matched:emitted:skipped_idempotent:errors
    #   __RECORD__:task_id|board|assignee|age_h|status
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
                # Запись для stats log (формат: t_<id>|<board>|<assignee>|<age_h>|<status>)
                printf '  %s\n' "${line#__RECORD__:}"
                ;;
            *)
                # Прочий stdout (debug) — pass-through в stderr.
                [ -n "$line" ] && echo "$line" >&2
                ;;
        esac
    done < <(
        python3 - "$db_path" "$_board" "$_now" "$_today_start" \
                "$AGE_THRESHOLD_SECONDS" "$MARKER_TAG" "$DRY_RUN" "$HERMES_CLI" \
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
marker_tag     = sys.argv[6]
dry_run        = (sys.argv[7].lower() == "true")
hermes_cli     = sys.argv[8]

ALLOWED_ASSIGNEES = {"architect", "devops"}

def log(msg):
    print(f"[{time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}] "
          f"blocked-watchdog-scope: {msg}", file=sys.stderr)

try:
    con = sqlite3.connect(db_path, timeout=10)
    con.row_factory = sqlite3.Row
    cur = con.execute(
        "SELECT id, body, assignee, started_at FROM tasks "
        "WHERE status='running' AND started_at IS NOT NULL"
    )
    candidates = cur.fetchall()
except sqlite3.OperationalError as e:
    log(f"SKIP board={board} db={db_path} err=operational:{e}")
    print(f"__STATS__:0:0:0:0:0")
    sys.exit(0)

stats = {"scanned": len(candidates), "matched": 0, "emitted": 0,
         "skipped_idempotent": 0, "errors": 0, "records": []}

for row in candidates:
    tid        = row["id"]
    body       = row["body"] or ""
    assignee   = row["assignee"] or ""  # NULL → ""
    started_at = int(row["started_at"])

    age = now_s - started_at
    if age < age_threshold:
        continue

    # Criteria (ADR-0036 §4.3):
    # 1. age > threshold (already checked above)
    # 2. assignee NOT IN allowed (NULL treated as non-allowed)
    if assignee in ALLOWED_ASSIGNEES:
        continue
    # 3. body LIKE '%ADR-%' (case-sensitive — ADR is uppercase convention)
    if "ADR-" not in body:
        continue

    stats["matched"] += 1

    # Idempotency check: уже комментили сегодня?
    try:
        idem_cur = con.execute(
            "SELECT 1 FROM task_comments "
            "WHERE task_id=? AND body LIKE ? AND created_at > ? LIMIT 1",
            (tid, f"%{marker_tag}%", today_start),
        )
        if idem_cur.fetchone() is not None:
            stats["skipped_idempotent"] += 1
            log(f"SKIP {tid} (idempotent — comment уже есть за today)")
            continue
    except sqlite3.OperationalError:
        # task_comments не существует — это OK, пустая idempotency.
        pass

    age_h = age // 3600
    age_rem_min = (age % 3600) // 60
    comment_body = (
        f"{marker_tag} (ADR-0036 §4.3, cron auto):\n"
        f"age={age_h}ч{age_rem_min}м > 4ч, assignee={assignee} "
        f"(не architect/devops), body содержит \"ADR-\".\n"
        f"Возможно mis-scope: архитектурная карточка в non-architect профиле.\n"
        f"Шифу — оцени. Vendor-патч hint уже есть (ADR-0036 §4.1)."
    )

    if dry_run:
        stats["emitted"] += 1
        log(f"[DRY-RUN] {tid} would-comment age={age_h}ч{age_rem_min}м assignee={assignee}")
        stats["records"].append((tid, board, assignee, age_h, "DRY-RUN"))
        continue

    # Side-effect: hermes kanban --board <board> comment <tid> <body>
    # NB: body многострочный — передаём как positional args (hermes comment
    # принимает task_id + body args, склеивая их в один body).
    try:
        proc = subprocess.run(
            [hermes_cli, "kanban", "--board", board, "comment", tid, comment_body],
            capture_output=True, text=True, timeout=15, check=False,
        )
        if proc.returncode == 0:
            stats["emitted"] += 1
            stats["records"].append((tid, board, assignee, age_h, "OK"))
            log(f"COMMENT {tid} board={board} age={age_h}ч{age_rem_min}м assignee={assignee}")
        else:
            stats["errors"] += 1
            log(f"ERROR comment {tid} rc={proc.returncode} stderr={proc.stderr.strip()[:200]}")
    except subprocess.TimeoutExpired:
        stats["errors"] += 1
        log(f"ERROR timeout comment {tid}")
    except Exception as e:
        stats["errors"] += 1
        log(f"ERROR exception comment {tid} err={type(e).__name__}:{e}")

con.close()

print(f"__STATS__:{stats['scanned']}:{stats['matched']}:"
      f"{stats['emitted']}:{stats['skipped_idempotent']}:{stats['errors']}")
for r in stats["records"]:
    print(f"__RECORD__:{r[0]}|{r[1]}|{r[2]}|{r[3]}|{r[4]}")
PYEOF
    )
done < <(printf '%s\n' "$_db_list")

# --- structured summary ----------------------------------------------------
echo "[$(_now_iso)] blocked-watchdog-scope: ✓ done scanned=${_scanned_total} matched=${_matched_total} emitted=${_emitted_total} skipped_idempotent=${_skipped_idempotent_total} errors=${_errors_total}" >&2

# --- write stats log -------------------------------------------------------
mkdir -p "$(dirname "$LOG_FILE")" 2>/dev/null || true
{
    printf '# blocked-watchdog-scope snapshot %s\n' "$(_now_iso)"
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
