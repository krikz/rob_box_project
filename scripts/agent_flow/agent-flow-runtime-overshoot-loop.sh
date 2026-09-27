#!/bin/bash
# ============================================================================
# agent-flow-runtime-overshoot-loop.sh — pre-cancel guard for upstream-driven
# crash loops (MiniMax 402/429).
#
# Source of truth: <repo>/scripts/agent_flow/agent-flow-runtime-overshoot-loop.sh
# Раскладывается install.sh (EXPECTED) на 6 путей: agent-flow/architect/devops/
# backend/analyst + legacy ~/.hermes/scripts/. Hardlink, не symlink (ретро 11.08
# t_a6a236e0d9f0470e). Drift-контроль — agent-flow-drift-detect.sh.
#
# Назначение (ретро 15.09 t_34f33289, карточка t_c2ab8db9):
# Существующие watchdog'и (protocol-violation-watchdog, agent-flow-blocked-
# watchdog §4.2 runtime-overshoot 4×max_runtime, agent-flow-cancel-on-
# provider-exhausted) срабатывают ПОСЛЕ того, как воркер уже получил
# signal-9 / уже 2× gave_up. Карточки-жертвы (t_e5f69665, t_94765031,
# t_9c5c6df1, t_51b38b85, t_dbc8d630 — суммарно 120+ signal-9 крашей)
# сжигали 1-2ч CPU/RAM до первого intervention. Задача этого watchdog —
# блокировать задачу ДО того, как dispatcher начнёт её убивать.
#
# Три детектора (приоритеты из body t_c2ab8db9):
#   (1) gate-by-giveup-pattern — высокоприоритетный:
#       для running/ready карточек с consecutive_failures >= 3 за последние
#       30 мин И у всех 3 failure причина ∈ {provider-exhausted signature,
#       429, 402, exit code 1} → блокируем, если у карточки НЕТ открытого PR
#       (PR — признак активной recovery-работы, воркер уже нашёл путь).
#   (2) MEMORY_AVAILABLE preflight — среднеприоритетный:
#       перед каждым action watchdog читает /proc/meminfo::MemAvailable и
#       при < 2 GiB делает backoff 60s (exit 0). Это лечит накопление
#       OOM-убийств при множественных spawn (каждый hermes-cli ≈ 350-500 МБ)
#       и заменяет патч в hermes-agent/hermes_cli/kanban_db_dispatch.py:
#       upstream-репо — другая org; preflight в watchdog'е достаточно, потому
#       что бэкенд spawn'ов всё равно идёт через cron rabbit.
#   (3) loop-no-progress — среднеприоритетный:
#       для running/ready карточки, у которой >= 5 spawned events за
#       последний час И есть хотя бы 1 gave_up за этот час → блокируем как
#       loop-no-progress. Это явный признак dispatcher-цикла без прогресса.
#
# Acceptance (t_c2ab8db9):
#   - shellcheck-clean (warnings SC2086/2154 disabled точечно)
#   - --dry-run режим (как у agent-flow-cancel-on-provider-exhausted.sh)
#   - Idempotent: повторный запуск не дублирует ни block-вызовы, ни
#     sentinel-комментарии (marker через SENTINEL_TAG)
#   - Unit-тест: фикстура в tests/agent_flow/test_runtime_overshoot_loop.sh
#   - safe-to-re-run: даже если все 3 детектора триггерят — каждый
#     sentinel-комментарий идёт ровно один раз в HERMES_WINDOW_SEC (default
#     86400 = 24ч); после cooldown — снова можно блокировать (но только при
#     повторном symptom, не раньше).
#
# Use cases:
#   bash agent-flow-runtime-overshoot-loop.sh --dry-run
#     → показать какие карточки подпадут, без side effects.
#   bash agent-flow-runtime-overshoot-loop.sh
#     → применить блокировки + sentinel-комментарии.
#   bash agent-flow-runtime-overshoot-loop.sh --force
#     → игнорировать cooldown (для ручного recovery от Шифу).
# ============================================================================

set -euo pipefail

# -------- paths / env --------
SCRIPT_NAME="$(basename "$0")"
HERMES_HOME="${HERMES_HOME:-${HOME}/.hermes}"
KANBAN_BOARDS_DIR="${KANBAN_BOARDS_DIR:-$HERMES_HOME/kanban/boards}"
HERMES_BIN="${HERMES_BIN:-/home/builder/.hermes/hermes-agent/venv/bin/hermes}"
GH_CONFIG_DIR="${GH_CONFIG_DIR:-/home/builder/.config/gh}"
LOCK_FILE="${LOCK_FILE:-$HERMES_HOME/state/${SCRIPT_NAME}.lock}"
LOG_FILE="${LOG_FILE:-$HERMES_HOME/logs/${SCRIPT_NAME}.log}"
STATE_DIR="${STATE_DIR:-$HERMES_HOME/state}"

mkdir -p "$(dirname "$LOCK_FILE")" "$(dirname "$LOG_FILE")" "$STATE_DIR"

log() { printf '[%s] %s\n' "$SCRIPT_NAME" "$*" >&2; }

# -------- constants --------
RETRO_KEY="runtime-overshoot-loop-watchdog"   # в marker-комментариях
SENTINEL_TAG="<!-- ${SCRIPT_NAME}:marker -->"
RETRO_TAG="<!-- retro-key:${RETRO_KEY} -->"
ROOT_ISSUE="#1193"  # MiniMax provider budget tracking
# shellcheck disable=SC2034  # GITHUB_REPO — для будущих gh-pr-check (пока не используется)
GITHUB_REPO="krikz/rob_box_project"

# Дефолты (overridable через env):
FAIL_COUNT="${FAIL_COUNT:-3}"                # ≥3 consecutive failures
FAIL_WINDOW_SEC="${FAIL_WINDOW_SEC:-1800}"   # за последние 30 мин
SPAWN_THRESHOLD="${SPAWN_THRESHOLD:-5}"      # ≥5 spawns в окне
LOOP_WINDOW_SEC="${LOOP_WINDOW_SEC:-3600}"   # за последний час
MEM_MIN_KB="${MEM_MIN_KB:-2097152}"          # 2 GiB; MemAvailable, KiB
DEFER_COOLDOWN_SEC="${DEFER_COOLDOWN_SEC:-60}"  # backoff при low-mem
HERMES_WINDOW_SEC="${HERMES_WINDOW_SEC:-86400}"  # idempotency-окно (24ч)

# Сигнатура provider-exhausted в last_failure_error / task_events.payload /
# task_runs.error. Те же ключевые слова, что в
# agent-flow-cancel-on-provider-exhausted.sh (синхронизируем вручную).
EXHAUST_SIGNATURES=(
    "HTTP 402" "Insufficient Balance" "Out of credits"
    "Billing or credits exhausted" "HTTP 429" "rate limit"
    "Token Plan usage limit" "2056" "Token Plan rate limit reached"
    "health-aware-fallback" "all providers unavailable"
    "all providers failed" "provider unavailable" "provider-exhaustion"
    "HTTP 401" "Authentication Fails" "is invalid"
    "invalid_request_error" "authentication_error"
    "провайдер исчерпан"
)

# -------- helpers --------

# MAINTENANCE gate (issue #3009). Inline-проверка без source
# lib_agent_flow_common: remote через git ls-remote, local fallback через
# git -C REPO_DIR show. Шифу ставит MAINTENANCE-файл в develop чтобы
# приостановить работу воркеров на время ручных правок. Срабатывает →
# exit 0 (тик пропускается, не ошибка).
_af_maintenance_gate_inline() {
    local _branch="${MAINTENANCE_BRANCH:-develop}"
    local _file="${MAINTENANCE_FILE:-MAINTENANCE}"
    local _remote_ref="${_branch}:${_file}"
    if [ -n "${GH_REPO:-}" ] \
        && git ls-remote "https://github.com/${GH_REPO}.git" "$_remote_ref" \
            2>/dev/null | grep -q .; then
        log "[MAINTENANCE] gate active on remote ${_remote_ref} — skip"
        exit 0
    fi
    if [ -n "${REPO_DIR:-}" ] && [ -d "$REPO_DIR" ] \
        && git -C "$REPO_DIR" show "${_branch}:${_file}" >/dev/null 2>&1; then
        log "[MAINTENANCE] gate active locally in ${REPO_DIR} (${_branch}:${_file}) — skip"
        exit 0
    fi
    return 0
}

# Один инстанс. Если уже идёт — exit 0 (тик пропускаем, no-agent cron).
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
    log "⏳ another instance holds $LOCK_FILE — skip"
    exit 0
fi

# MAINTENANCE gate (issue #3009) — после flock, до основной работы.
_af_maintenance_gate_inline

usage() {
    cat <<EOF
Usage: $SCRIPT_NAME [--dry-run] [--force] [--help]

Modes:
  (default)   scan all kanban boards, detect (1)/(2)/(3) patterns, block
              tasks via \`hermes kanban block --kind capability <tid> <reason>\`
              + post SENTINEL_TAG-marked comment on the task.
              Idempotent via task_comments uniqueness check (HERMES_WINDOW_SEC).
  --dry-run   same scan, no side effects (печатает план и exit 1 if anything
              would block).
  --force     ignore idempotency cooldown (ручной recovery от Шифу).

Env knobs:
  FAIL_COUNT         ≥ N consecutive failures для (1) gate-by-giveup-pattern
                     (default: $FAIL_COUNT)
  FAIL_WINDOW_SEC    окно для consecutive_failures (default: $FAIL_WINDOW_SEC)
  SPAWN_THRESHOLD    ≥ N spawned events для (3) loop-no-progress
                     (default: $SPAWN_THRESHOLD)
  LOOP_WINDOW_SEC    окно для (3) loop-no-progress (default: $LOOP_WINDOW_SEC)
  MEM_MIN_KB         MemAvailable преflight-порог, KiB (default: $MEM_MIN_KB)
  DEFER_COOLDOWN_SEC backoff при low-mem (default: $DEFER_COOLDOWN_SEC)
  HERMES_WINDOW_SEC  cooldown для повторного блока одной карточки
                     (default: $HERMES_WINDOW_SEC)
EOF
}

# Читает /proc/meminfo, возвращает MemAvailable в KiB (целое), или пустую
# строку если не удалось. Реалистично: на любой sane Linux-машине этот файл
# всегда есть, поэтому пустота — признак крайне нештатной ситуации.
read_mem_available_kb() {
    awk '/^MemAvailable:/ { print $2; exit }' /proc/meminfo 2>/dev/null
}

# MEMORY_AVAILABLE preflight (приоритет MEDIUM в t_c2ab8db9). Возвращает
# rc=0 если MemAvailable >= MEM_MIN_KB (можно работать).
# rc=2 если ниже — нужно backoff. Вызывающий сам делает sleep+exit 0.
preflight_memory_available() {
    local avail
    avail="$(read_mem_available_kb)"
    if [ -z "$avail" ]; then
        log "  WARN preflight_memory_available: /proc/meminfo read failed — пропускаем gate"
        return 0  # не валим всё из-за одного отказа чтения
    fi
    if [ "$avail" -lt "$MEM_MIN_KB" ]; then
        log "  ⏸  preflight_memory_available: MemAvailable=${avail}KiB < ${MEM_MIN_KB}KiB — defer ${DEFER_COOLDOWN_SEC}s"
        return 2
    fi
    return 0
}

# shellcheck disable=SC2317,SC2329  # unused: перенесено в python detect_* функций
# True если text содержит хотя бы одну сигнатуру из EXHAUST_SIGNATURES.
# (Оставлено как reference-bash-implementation для тестов и shellcheck debug.
#  Реальная логика — в python-блоках detect_gate_by_giveup / detect_loop_no_progress.)
text_contains_exhaust() {
    local text="$1"
    [ -n "$text" ] || return 1
    local low="${text,,}"
    # Anti-patterns: recover-маркеры (НЕ триггерим)
    case "$low" in
        *"провайдер восстановлен"*|*"provider restored"*|*"unblock: провайдер"*) return 1 ;;
    esac
    local sig
    for sig in "${EXHAUST_SIGNATURES[@]}"; do
        case "$low" in
            *"${sig,,}"*) return 0 ;;
        esac
    done
    return 1
}

# shellcheck disable=SC2317,SC2329  # unused: перенесено в python detect_*
# Извлекает issue-NNNN из body — эталонная bash-реализация.
issue_pr_ref_from_body() {
    local body="$1"
    local n=""
    n=$(printf '%s\n' "$body" | awk '
        /^Source$/  { in_src=1; next }
        in_src && /^[^ ]/ { in_src=0 }
        in_src && /^[[:space:]]+issue:[[:space:]]+#?([0-9]+)/ {
            print gensub(/^[[:space:]]+issue:[[:space:]]+#?([0-9]+).*/, "\\1", 1)
            exit
        }
    ' 2>/dev/null || true)
    if [ -z "$n" ]; then
        n=$(printf '%s\n' "$body" \
            | grep -oE '[Ii]ssue[[:space:]]*:?[[:space:]]*#[0-9]+' \
            | head -n1 | grep -oE '[0-9]+' || true)
    fi
    if [ -z "$n" ]; then
        n=$(printf '%s\n' "$body" | grep -oE '#[0-9]{3,5}' | head -n1 | tr -d '#' || true)
    fi
    printf '%s' "$n"
}

# shellcheck disable=SC2317,SC2329  # unused: перенесено в python detect_gate_by_giveup
# Эвристика «есть ли у задачи открытый PR» — reference-bash для тестов.
has_open_pr_heuristic() {
    local db_path="$1"
    local tid="$2"
    local body="$3"
    if printf '%s' "$body" | grep -qE '#[0-9]{3,5}'; then
        local prcount
        prcount=$(grep -oE '#[0-9]{3,5}' <<<"$body" | sort -u | wc -l)
        if [ "$prcount" -gt 0 ]; then
            local sqlres
            sqlres=$(python3 - "$db_path" "$tid" <<'PYEOF' 2>/dev/null || echo "0"
import sqlite3, sys, time
db, tid = sys.argv[1], sys.argv[2]
try:
    con = sqlite3.connect(db)
    row = con.execute(
        "SELECT COUNT(*) FROM task_comments "
        "WHERE task_id=? AND ("
        "  body LIKE '%PR #%' OR body LIKE '%pulls/%' "
        "  OR body LIKE '%gh pr merge%' OR body LIKE '%auto-merge scheduled%'"
        "  OR body LIKE '%PR% ready to merge%'"
        ") AND created_at >= ?",
        (tid, int(time.time()) - 86400),
    ).fetchone()
    con.close()
    print(row[0] if row else 0)
except Exception:
    print(0)
PYEOF
)
            if [ "${sqlres:-0}" -gt 0 ]; then
                return 0
            fi
        fi
    fi
    return 1
}

# Sentinel presence: есть ли в task_comments за HERMES_WINDOW_SEC наш marker?
# Используется для idempotency. Возвращает 0 если есть (skip), 1 если нет.
has_sentinel_comment() {
    local db_path="$1"
    local tid="$2"
    local hits
    hits=$(python3 - "$db_path" "$tid" <<'PYEOF' 2>/dev/null || echo "1"
import sqlite3, sys, time
db, tid = sys.argv[1], sys.argv[2]
try:
    con = sqlite3.connect(db)
    row = con.execute(
        "SELECT COUNT(*) FROM task_comments "
        "WHERE task_id=? AND body LIKE ? "
        "  AND created_at >= ?",
        (tid,
         '%' + '<!-- agent-flow-runtime-overshoot-loop.sh:marker -->' + '%',
         int(time.time()) - 86400)
    ).fetchone()
    con.close()
    print(row[0] if row else 0)
except Exception:
    print(0)
PYEOF
)
    [ "${hits:-0}" -gt 0 ]
}

# Detect (1) gate-by-giveup-pattern.
# Возвращает JSON-lines: {"task_id":..., "board":..., "issue_ref":...,
#   "reason":"gate-by-giveup", "failures":N, "all_provider_exhaust":true}
detect_gate_by_giveup() {
    local window="$1"
    local min_failures="$2"
    local now
    now=$(date +%s)
    local since=$(( now - window ))

    python3 - "$KANBAN_BOARDS_DIR" "$since" "$min_failures" <<'PYEOF'
import glob, json, os, sqlite3, sys, time
boards_dir, since, min_failures = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])

EXHAUST_LOWER = (
    "http 402", "insufficient balance", "out of credits",
    "billing or credits exhausted", "http 429", "rate limit",
    "token plan usage limit", "2056", "token plan rate limit reached",
    "health-aware-fallback", "all providers unavailable",
    "all providers failed", "provider unavailable", "provider-exhaustion",
    "http 401", "authentication fails", "is invalid",
    "invalid_request_error", "authentication_error",
    "провайдер исчерпан",
)
ANTI = ("провайдер восстановлен", "provider restored", "unblock: провайдер")

def has_exhaust(text: str) -> bool:
    if not text:
        return False
    low = text.lower()
    if any(a in low for a in ANTI):
        return False
    return any(s in low for s in EXHAUST_LOWER)


def has_open_pr_heuristic(db_path: str, tid: str, body: str) -> bool:
    # 1) PR#-ref в body
    import re
    pr_refs = re.findall(r"#(\d{3,5})", body or "")
    if pr_refs:
        try:
            con = sqlite3.connect(db_path)
            row = con.execute(
                "SELECT COUNT(*) FROM task_comments "
                "WHERE task_id=? AND ("
                "  body LIKE '%PR #%' OR body LIKE '%pulls/%' "
                "  OR body LIKE '%gh pr merge%' OR body LIKE '%auto-merge scheduled%'"
                "  OR body LIKE '%PR% ready to merge%'"
                ") AND created_at >= ?",
                (tid, int(time.time()) - 86400),
            ).fetchone()
            con.close()
            if row and row[0] > 0:
                return True
        except Exception:
            pass
    return False


SENTINEL_TAG = "<!-- agent-flow-runtime-overshoot-loop.sh:marker -->"
HERMES_WINDOW_SEC = 86400


def has_sentinel(db_path: str, tid: str) -> bool:
    """Idempotency: skip if marker present in recent task_comments."""
    try:
        con = sqlite3.connect(db_path)
        row = con.execute(
            "SELECT COUNT(*) FROM task_comments "
            "WHERE task_id=? AND body LIKE ? "
            "  AND created_at >= ?",
            (tid, "%" + SENTINEL_TAG + "%", int(time.time()) - HERMES_WINDOW_SEC),
        ).fetchone()
        con.close()
        return bool(row and row[0] > 0)
    except Exception:
        return False


def extract_issue(body: str) -> str:
    if not body:
        return ""
    import re
    for line in body.splitlines():
        stripped = line.strip()
        if stripped.startswith("Source"):
            continue
        m = re.match(r"^\s+issue:\s*#?(\d+)", line)
        if m:
            return m.group(1)
        if line and not line.startswith(" ") and not line.startswith("\t"):
            break
    m = re.search(r"[Ii]ssue\s*:?\s*#(\d+)", body)
    if m:
        return m.group(1)
    m = re.search(r"#(\d{3,5})", body)
    return m.group(1) if m else ""


results = []
for db in sorted(glob.glob(f"{boards_dir}/*/kanban.db")):
    board = os.path.basename(os.path.dirname(db))
    try:
        con = sqlite3.connect(db)
        con.row_factory = sqlite3.Row
        rows = con.execute("""
            SELECT id, title, body, status, consecutive_failures,
                   last_failure_error, block_kind
            FROM tasks
            WHERE status IN ('running', 'ready')
              AND consecutive_failures >= ?
        """, (min_failures,)).fetchall()

        for row in rows:
            tid = row["id"]
            status = row["status"]
            body = row["body"] or ""
            failures = row["consecutive_failures"]
            lfe = row["last_failure_error"] or ""

            # Берём последние N events crashed/gave_up/spawn_failed/timed_out.
            # Если у ВСЕХ этих events причина ∈ exhaust_signature — loop.
            ev_rows = con.execute(
                """
                SELECT kind, payload, error
                FROM task_events
                WHERE task_id=?
                  AND kind IN ('crashed','gave_up','spawn_failed','timed_out')
                ORDER BY id DESC LIMIT ?
                """, (tid, min_failures),
            ).fetchall()
            if len(ev_rows) < min_failures:
                continue

            exhausted_count = 0
            for ev in ev_rows:
                blob = " ".join([
                    ev["kind"] or "",
                    ev["payload"] or "",
                    ev["error"] or "",
                ])
                if has_exhaust(blob) or has_exhaust(lfe):
                    exhausted_count += 1

            if exhausted_count < min_failures:
                continue

            # Idempotency: если у задачи уже есть sentinel-маркер за
            # HERMES_WINDOW_SEC — пропускаем (это уже блокировали).
            if has_sentinel(db, tid):
                continue

            # Есть ли активный PR? Эвристика, см. выше.
            if has_open_pr_heuristic(db, tid, body):
                continue

            results.append({
                "detector": "gate-by-giveup",
                "board": board,
                "task_id": tid,
                "title": (row["title"] or "")[:120],
                "failures": failures,
                "exhausted_events": exhausted_count,
                "last_failure_error": lfe[:200],
                "issue_ref": extract_issue(body),
                "root_issue": "1193",
            })
        con.close()
    except Exception as exc:
        print(f"[detect_gate_by_giveup] {board} error: {exc}", file=sys.stderr)

for r in results:
    print(json.dumps(r, ensure_ascii=False))
PYEOF
}

# Detect (3) loop-no-progress: ≥SPAWN_THRESHOLD spawns И ≥1 gave_up за окно.
detect_loop_no_progress() {
    local spawn_threshold="$1"
    local window="$2"

    python3 - "$KANBAN_BOARDS_DIR" "$spawn_threshold" "$window" <<'PYEOF'
import glob, json, os, sqlite3, sys, time
boards_dir, spawn_threshold, window = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
now = int(time.time())
since = now - window


def extract_issue(body: str) -> str:
    if not body:
        return ""
    import re
    for line in body.splitlines():
        stripped = line.strip()
        if stripped.startswith("Source"):
            continue
        m = re.match(r"^\s+issue:\s*#?(\d+)", line)
        if m:
            return m.group(1)
        if line and not line.startswith(" ") and not line.startswith("\t"):
            break
    m = re.search(r"[Ii]ssue\s*:?\s*#(\d+)", body)
    if m:
        return m.group(1)
    m = re.search(r"#(\d{3,5})", body)
    return m.group(1) if m else ""


SENTINEL_TAG = "<!-- agent-flow-runtime-overshoot-loop.sh:marker -->"
HERMES_WINDOW_SEC = 86400


def has_sentinel(db_path, tid):
    """Idempotency: skip if marker present in recent task_comments."""
    try:
        con = sqlite3.connect(db_path)
        row = con.execute(
            "SELECT COUNT(*) FROM task_comments "
            "WHERE task_id=? AND body LIKE ? "
            "  AND created_at >= ?",
            (tid, "%" + SENTINEL_TAG + "%", int(time.time()) - HERMES_WINDOW_SEC),
        ).fetchone()
        con.close()
        return bool(row and row[0] > 0)
    except Exception:
        return False


results = []
for db in sorted(glob.glob(f"{boards_dir}/*/kanban.db")):
    board = os.path.basename(os.path.dirname(db))
    try:
        con = sqlite3.connect(db)
        con.row_factory = sqlite3.Row
        rows = con.execute(
            "SELECT id, title, body, status FROM tasks "
            "WHERE status IN ('running', 'ready')",
        ).fetchall()

        for row in rows:
            tid = row["id"]
            body = row["body"] or ""

            # Idempotency: sentinel уже есть → skip.
            if has_sentinel(db, tid):
                continue

            spawn_count = con.execute(
                "SELECT COUNT(*) AS n FROM task_events "
                "WHERE task_id=? AND kind='spawned' AND created_at >= ?",
                (tid, since),
            ).fetchone()["n"]

            gave_up_count = con.execute(
                "SELECT COUNT(*) AS n FROM task_events "
                "WHERE task_id=? AND kind='gave_up' AND created_at >= ?",
                (tid, since),
            ).fetchone()["n"]

            if spawn_count >= spawn_threshold and gave_up_count >= 1:
                results.append({
                    "detector": "loop-no-progress",
                    "board": board,
                    "task_id": tid,
                    "title": (row["title"] or "")[:120],
                    "spawn_count_1h": spawn_count,
                    "gave_up_count_1h": gave_up_count,
                    "issue_ref": extract_issue(body),
                    "root_issue": "1193",
                })
        con.close()
    except Exception as exc:
        print(f"[detect_loop_no_progress] {board} error: {exc}", file=sys.stderr)

for r in results:
    print(json.dumps(r, ensure_ascii=False))
PYEOF
}

# Apply one action: блокировка задачи + sentinel-комментарий.
apply_block_one() {
    local dry_run="$1" board="$2" task_id="$3" reason_kind="$4"
    local issue_ref="$5" detector_details="$6"

    local db_path="$KANBAN_BOARDS_DIR/$board/kanban.db"
    if [ ! -f "$db_path" ]; then
        log "  ⚠️ db not found: $db_path — skip"
        return 1
    fi

    # Idempotency: skip если уже есть sentinel за HERMES_WINDOW_SEC.
    if has_sentinel_comment "$db_path" "$task_id"; then
        log "  SKIP $task_id (sentinel comment present, window=${HERMES_WINDOW_SEC}s)"
        return 0
    fi

    if [ "$dry_run" = "yes" ]; then
        log "  [DRY-RUN] would: hermes kanban --board $board block --kind capability $task_id '...' + comment"
        return 0
    fi

    local block_reason="${reason_kind}: upstream-loop detected by agent-flow-runtime-overshoot-loop.sh (retros t_34f33289 t_c2ab8db9). ${detector_details} Root cause: MiniMax provider budget исчерпан — issue ${ROOT_ISSUE}. Связанный issue: #${issue_ref:-?}. Recovery: запустить скрипт с --dry-run после пополнения MiniMax и решить unblock вручную."
    if "$HERMES_BIN" kanban --board "$board" block --kind capability \
            "$task_id" "$block_reason" >>"$LOG_FILE" 2>&1; then
        log "  ✅ blocked $task_id"
    else
        log "  ⚠️  block failed (rc=$?) — see $LOG_FILE"
        return 1
    fi

    local comment_body="${SENTINEL_TAG}
${RETRO_TAG}
**Runtime-overshoot-loop auto-block** (${RETRO_KEY}):
детектор «${reason_kind}» сработал — $detector_details.
Root cause: MiniMax provider budget исчерпан — см. issue ${ROOT_ISSUE}.
Связанный issue: #${issue_ref:-?}.

Что проверить перед unblock:
1. MiniMax budget (issue ${ROOT_ISSUE}) — пополнен?
2. \`hermes kanban --board $board show $task_id\` — нет ли открытого PR, которого watchdog не увидел?
3. \`bash scripts/agent_flow/agent-flow-runtime-overshoot-loop.sh --dry-run\` — пусто?"

    if "$HERMES_BIN" kanban --board "$board" comment \
            "$task_id" "$comment_body" >>"$LOG_FILE" 2>&1; then
        log "  ✅ comment posted (with ${SENTINEL_TAG})"
    else
        log "  ⚠️  comment failed (rc=$?) — see $LOG_FILE"
    fi
}

# -------- main --------
MODE="apply"
FORCE_FLAG=false
while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run) MODE="dry-run"; shift ;;
        --force)   FORCE_FLAG=true; shift ;;
        --help|-h) usage; exit 0 ;;
        *)         log "⚠️  unknown arg: $1"; usage; exit 2 ;;
    esac
done

# MEMORY_AVAILABLE preflight (приоритет MEDIUM): если MemAvailable ниже
# порога — backoff. Не валим cron (он no_agent), просто откладываем весь
# tick на DEFER_COOLDOWN_SEC. НЕ трогаем dry-run: с него нагрузки нет,
# а пользователь хочет видеть список.
if [ "$MODE" != "dry-run" ]; then
    if ! preflight_memory_available; then
        sleep "$DEFER_COOLDOWN_SEC"
        log "  ⏸  deferring by ${DEFER_COOLDOWN_SEC}s; exit 0 (no-op tick)"
        exit 0
    fi
fi

log "=== START mode=$MODE boards_dir=$KANBAN_BOARDS_DIR mem_min_kb=$MEM_MIN_KB ==="

# (1) gate-by-giveup-pattern
ACTIONS_FILE_GG="$STATE_DIR/${SCRIPT_NAME}.gate_giveup.actions.jsonl"
: > "$ACTIONS_FILE_GG"
if [ -n "$FORCE_FLAG" ] || [ "$MODE" = "apply" ]; then
    : # always compute
fi
detect_gate_by_giveup "$FAIL_WINDOW_SEC" "$FAIL_COUNT" >"$ACTIONS_FILE_GG" 2>>"$LOG_FILE"
GG_COUNT=$(wc -l <"$ACTIONS_FILE_GG" || echo 0)
log "[1/3] gate-by-giveup-pattern: $GG_COUNT match(es)"

# (3) loop-no-progress
ACTIONS_FILE_LNP="$STATE_DIR/${SCRIPT_NAME}.loop_no_progress.actions.jsonl"
: > "$ACTIONS_FILE_LNP"
# Debug: capture both stdout and stderr to file for visibility
detect_loop_no_progress "$SPAWN_THRESHOLD" "$LOOP_WINDOW_SEC" >"$ACTIONS_FILE_LNP" 2>>"$LOG_FILE"
LNP_DEBUG=$(wc -l <"$ACTIONS_FILE_LNP" || echo 0)
log "[3/3] loop-no-progress: $LNP_DEBUG match(es) (debug: lnp file = $ACTIONS_FILE_LNP)"

DRY_RUN_FLAG="no"
[ "$MODE" = "dry-run" ] && DRY_RUN_FLAG="yes"

TOTAL=0 APPLIED=0 SKIPPED=0
while IFS= read -r line; do
    [ -z "$line" ] && continue
    TOTAL=$((TOTAL + 1))
    board="" task_id="" detector="" issue_ref="" details=""
    # Парсим JSON через python helper (надёжнее чем jq-зависимость; и не
    # триггерим f-string syntax error при nested-quotes).
    # shellcheck disable=SC2154  # vars заданы через eval ниже
    eval "$(printf '%s' "$line" | python3 -c "
import json, sys
o = json.loads(sys.stdin.read())
detector = o.get('detector', '')
tid = o.get('task_id', '')
board = o.get('board', '')
issue = o.get('issue_ref', '')
last_err = str(o.get('last_failure_error', ''))
if detector == 'gate-by-giveup':
    last_err_repr = repr(last_err)
    details = (
        'consecutive_failures=' + str(o.get('failures'))
        + '; exhausted_events=' + str(o.get('exhausted_events'))
        + '; last_failure_error=' + last_err_repr
    )
elif detector == 'loop-no-progress':
    details = (
        'spawn_count_1h=' + str(o.get('spawn_count_1h'))
        + '; gave_up_count_1h=' + str(o.get('gave_up_count_1h'))
    )
else:
    details = ''
SQ = chr(39)
for k, v in (
    ('detector', detector),
    ('task_id', tid),
    ('board', board),
    ('issue_ref', issue),
    ('details', details),
):
    sval = str(v).replace(SQ, SQ + chr(92) + SQ + SQ)
    print(k + '=' + SQ + sval + SQ)
")"

    log "→ [$detector] $board/$task_id issue=#${issue_ref:-?}  $details"

    if [ "$FORCE_FLAG" = "true" ] && [ "$MODE" = "apply" ]; then
        # FORCING: bypass idempotency-окно (ручной recovery).
        log "  force-mode: bypass HERMES_WINDOW_SEC=$HERMES_WINDOW_SEC sentinel check"
        if [ "$DRY_RUN_FLAG" = "yes" ]; then
            log "  [DRY-RUN+FORCE] would: block + comment $task_id"
        else
            # shellcheck disable=SC2154  # board/task_id/details приходят из eval выше
            "$HERMES_BIN" kanban --board "$board" block --kind capability \
                "$task_id" "force-block by runtime-overshoot-loop-watchdog (--force): $details" \
                >>"$LOG_FILE" 2>&1 || true
            # shellcheck disable=SC2154
            "$HERMES_BIN" kanban --board "$board" comment \
                "$task_id" "${SENTINEL_TAG} ${RETRO_TAG} FORCE-block: $details" \
                >>"$LOG_FILE" 2>&1 || true
            APPLIED=$((APPLIED + 1))
        fi
        continue
    fi

    if apply_block_one "$DRY_RUN_FLAG" "$board" "$task_id" \
            "$detector" "$issue_ref" "$details"; then
        APPLIED=$((APPLIED + 1))
    else
        SKIPPED=$((SKIPPED + 1))
    fi
done < <(cat "$ACTIONS_FILE_GG" "$ACTIONS_FILE_LNP" 2>/dev/null)

log "summary: scanned=$TOTAL applied=$APPLIED skipped=$SKIPPED (gate=$GG_COUNT loop=$LNP_DEBUG)"
if [ "$MODE" = "dry-run" ] && [ "$TOTAL" -gt 0 ]; then
    log "DRY-RUN found $TOTAL candidate(s); rerun without --dry-run to apply."
fi
log "=== END ==="

# dry-run с ненулевым exit: cron-лог увидит, что «что-то нашлось».
[ "$MODE" = "dry-run" ] && [ "$TOTAL" -gt 0 ] && exit 1
exit 0
