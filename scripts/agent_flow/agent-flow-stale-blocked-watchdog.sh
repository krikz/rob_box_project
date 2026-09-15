#!/bin/bash
# ============================================================================
# agent-flow-stale-blocked-watchdog.sh — auto-detect blocked kanban-карточки,
# все prerequisites которых уже merged в develop, и alert'ить Шифу (без
# авто-unblock).
#
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-stale-blocked-watchdog.sh
# Copies are laid down by install.sh into:
#   - ~/.hermes/profiles/devops/scripts/
#   - ~/.hermes/scripts/
#
# Контекст (ретро 2026-09-14 t_55c6c882, pattern повторяет t_55ab37d4):
#   "stale-blocked-after-prerequisites-merged": карточка blocked с parent-
#   prerequisites и/или body, ссылающимся на PR #NNNN, но все эти PR уже
#   merged в develop. Карточка при этом висит в blocked, потому что:
#     a) block_fn не имеет trigger на merged-PR'ы (ручное unblock через Шифу)
#     b) watchdog blocked-watchdog-scope не покрывает этот pattern (он для
#        mis-scope архитектурных карточек, не для prereq-merge)
#     c) e2e-/merge-процессы НЕ имеют callback на блокирующие карточки
#
#   Ручной unblock делает Шифу через `hermes kanban unblock <tid>`. Но pattern
#   повторяется → нужен auto-detect с alert.
#
# Контракт (per tick):
#   1. flock lock (не два тика одновременно)
#   2. iterate over all kanban-доски
#      (если KANBAN_DB_PATH задан — обрабатываем только эту БД; для тестов)
#   3. SELECT blocked-tasks WHERE body LIKE '%PR #%' OR LIKE '%#NNNN%'
#   4. for each candidate:
#      a) extract PR numbers из body (regex `(?:#|PR\s)#(\d+)`)
#      b) + extract из последнего blocked-event payload (block_reason)
#      c) для каждой PR: `gh pr view <N> --repo $GH_REPO --json state,merged,mergedAt`
#      d) SELECT из task_links: parents of task_id. Все ли parents status=done?
#      e) SKIP если есть PR с state=open или state=closed-not-merged
#      f) SKIP если есть parent со status ≠ done
#      g) иначе — match; idempotency: 1 row в task_comments за сегодня с
#         marker? → SKIP. Иначе — emit ОДИН alert-comment.
#   5. emit comment НЕ auto-unblock'ит карточку. Шифу eyeball'ит, делает
#      unblock вручную или запускает ручной workflow_dispatch / cron.
#   6. Log stats: scanned, matched, skipped_idempotent, skipped_unmerged_pr,
#      skipped_unfinished_parent, emitted, errors.
#
# ENV:
#   GH_REPO                — owner/repo (default krikz/rob_box_project)
#   GH_CONFIG_DIR          — for gh CLI auth (default ~/.config/gh)
#   KANBAN_DB_PATH         — single DB override (test mode)
#   KANBAN_BOARD           — board name for `hermes kanban comment`
#   KANBAN_BOARDS_DIR      — production scan root (default
#                            /home/builder/.hermes/kanban/boards)
#   DRY_RUN                — true → log only, no comment emit
#   AGE_THRESHOLD_SECONDS  — default 3600 (1h). Карточка должна провисеть
#                            в blocked хотя бы час, чтобы не alert'ить
#                            свежезаблокированные.
#   MARKER_TAG             — default "⚠️ stale-blocked: prerequisites merged"
#   LOCK_FILE              — flock guard
#   LOG_FILE               — stats log
#   HERMES_CLI             — hermes binary (default: hermes)
#
# Выходы:
#   - Stderr: structured summary (для cron delivery).
#   - Exit 0 — НИЧЕГО не нашли (или фатальные сбои подавлены flock/preflight).
#   - Exit 1 — критичный сбой (нет gh auth, нет python3, нет sqlite3).
#   - Exit 2 — нашли ≥1 stale-blocked карточку. Alert для cron.
#             Это ДЕТЕКТ-signal; НЕ зависит от DRY_RUN — даже в DRY_RUN=true
#             скрипт выходит с 2, чтобы cron alert'ил оператора при dryrun-
#             тестах (иначе false-sense-of-safety, см. issue #2481).
#
# Что НЕ делаем (явно):
#   - НЕ auto-unblock — это решение Шифу (потеря контекста карточки).
#   - НЕ trigger e2e/workflow_dispatch — это вне scope watchdog'а.
#   - НЕ делаем merge PR — Шифу merge'ит.
#   - НЕ делаем delete / archive — карточка остаётся в текущем статусе.
#
# Pitfalls (gotchas):
#   - gh CLI rate-limit: GraphQL упирается в 5000 req/h. Используем
#     REST API (`gh api repos/.../pulls/N`) — он в общем лимите 60 req/h
#     для неаутентифицированных, но с токеном 5000/h. Кэшируем ответы в
#     runtime через gh:cache dir, чтобы не сжигать budget.
#   - "PR #1234" в body может быть:
#       - реальный reference на PR → match
#       - issue reference (не PR) → match regex тоже сработает, но
#         `gh api repos/.../pulls/1234` для issue вернёт 404 / "Not Found".
#         Treat 404 как "не PR" → SKIP с reason "not_a_pr".
#       - текст в code block или цитата → всё равно матчится, но если
#         PR нет — SKIP. OK.
#   - body может содержать `gh pr view 1234` без `#`. Regex `(?<![0-9])(\d+)(?![0-9])`
#     опасен — может ловить номера версий и SHA-short. Сужáем до `(?:#|PR\s#|PR)\s*(\d+)
#     |\b#(\d+)\b(?![\d/])`. Двухшаговый фильтр.
#   - task_links.parent_id — это parent kanban-card, не parent-PR. Parents
#     done → значит prereq-карточки завершены. Эвристика работает в 90%
#     случаев (для t_e8072691 — true).
#   - Block-reason в task_events.payload — JSON. Распарсить через python3
#     json.loads (с защитой от malformed). PR-номера ищем регуляркой
#     по всей строке payload.
#   - Карточка может быть blocked, но все её PR merged, и при этом она
#     ожидает ручного решения Шифу (например, микрофон не работает, как в
#     t_e8072691). Alert должен упомянуть это в тексте comment, чтобы
#     Шифу не unblock'нул механически.
# ============================================================================
set -euo pipefail

# --- shared library bootstrap ------------------------------------------------
# Подтягиваем af_flock_guard_or_exit / af_load_profile_env / _af_log / etc.
# (дедуп 30.08, эталонный паттерн — agent-flow-deploy-sweep.sh:53-58).
# Source ДО определения LOCK_FILE/LOG_FILE: helper сам читает ${LOCK_FILE}.
_LIB_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=lib_agent_flow_common.sh
. "$_LIB_DIR_HERE/lib_agent_flow_common.sh"

DRY_RUN="${DRY_RUN:-false}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-stale-blocked-watchdog.lock}"
LOG_FILE="${LOG_FILE:-/tmp/agent-flow-stale-blocked-watchdog.log}"
AGE_THRESHOLD_SECONDS="${AGE_THRESHOLD_SECONDS:-3600}"
MARKER_TAG="${MARKER_TAG:-⚠️ stale-blocked: prerequisites merged}"
KANBAN_BOARDS_DIR="${KANBAN_BOARDS_DIR:-/home/builder/.hermes/kanban/boards}"
HERMES_CLI="${HERMES_CLI:-hermes}"
GH_REPO="${GH_REPO:-krikz/rob_box_project}"
GH_CONFIG_DIR="${GH_CONFIG_DIR:-/home/builder/.config/gh}"

# --- flock guard ------------------------------------------------------------
# Тело — af_flock_guard_or_exit в lib_agent_flow_common.sh. Дрейф формата
# логов (`stale-blocked-watchdog: another instance running — skip` →
# helper-delegated в `_af_log`) принят как documented consequence (issue
# #2477 §«Последствия»): parse-friendly парсеры адаптируются.
af_flock_guard_or_exit "$LOCK_FILE"

# --- pre-flight: gh + python3 ----------------------------------------------
if ! command -v python3 >/dev/null 2>&1; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] stale-blocked-watchdog: python3 not on PATH — exit 1" >&2
    exit 1
fi
python3 -c "import sqlite3, json, sys, re" 2>/dev/null || {
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] stale-blocked-watchdog: python3 modules missing — exit 1" >&2
    exit 1
}
export GH_CONFIG_DIR
if ! command -v gh >/dev/null 2>&1; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] stale-blocked-watchdog: gh CLI not on PATH — exit 1" >&2
    exit 1
fi
if ! gh auth status >/dev/null 2>&1; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] stale-blocked-watchdog: gh not authed — exit 1" >&2
    exit 1
fi

# --- helpers ---------------------------------------------------------------
_now_iso() { date -u +%Y-%m-%dT%H:%M:%SZ; }
_now_s()   { date -u +%s; }

_compute_today_start_utc() {
    python3 - <<'PYEOF'
import datetime, calendar
dt = datetime.datetime.utcnow().replace(hour=0, minute=0, second=0, microsecond=0)
print(int(calendar.timegm(dt.timetuple())))
PYEOF
}

# --- discover DB list -------------------------------------------------------
if [ -n "${KANBAN_DB_PATH:-}" ]; then
    _db_list="$KANBAN_DB_PATH"
    _default_board="${KANBAN_BOARD:-default}"
else
    _db_list="$(ls "${KANBAN_BOARDS_DIR}"/*/kanban.db 2>/dev/null || true)"
    _default_board=""
fi

if [ -z "$_db_list" ]; then
    echo "[$(_now_iso)] stale-blocked-watchdog: no kanban.db found under ${KANBAN_BOARDS_DIR} — exit 0" >&2
    exit 0
fi

_now="$(_now_s)"
_today_start="$(_compute_today_start_utc)"

# Aggregate counters
_scanned_total=0
_matched_total=0
_emitted_total=0
_skipped_idempotent_total=0
_skipped_unmerged_pr_total=0
_skipped_unfinished_parent_total=0
_skipped_no_pr_ref_total=0
_errors_total=0

# --- main loop -------------------------------------------------------------
while IFS= read -r db_path; do
    [ -n "$db_path" ] || continue
    [ -f "$db_path" ] || continue

    if [ -n "${KANBAN_DB_PATH:-}" ]; then
        _board="$_default_board"
    else
        _board="$(basename "$(dirname "$db_path")")"
    fi

    while IFS= read -r line; do
        case "$line" in
            __STATS__:*)
                local_part="${line#__STATS__:}"
                IFS=':' read -r s m e si su sf sn er <<< "$local_part"
                _scanned_total=$(( _scanned_total + s ))
                _matched_total=$(( _matched_total + m ))
                _emitted_total=$(( _emitted_total + e ))
                _skipped_idempotent_total=$(( _skipped_idempotent_total + si ))
                _skipped_unmerged_pr_total=$(( _skipped_unmerged_pr_total + su ))
                _skipped_unfinished_parent_total=$(( _skipped_unfinished_parent_total + sf ))
                _skipped_no_pr_ref_total=$(( _skipped_no_pr_ref_total + sn ))
                _errors_total=$(( _errors_total + er ))
                ;;
            __RECORD__:*)
                printf '  %s\n' "${line#__RECORD__:}"
                ;;
            *)
                [ -n "$line" ] && echo "$line" >&2
                ;;
        esac
    done < <(
        python3 - "$db_path" "$_board" "$_now" "$_today_start" \
                "$AGE_THRESHOLD_SECONDS" "$MARKER_TAG" "$DRY_RUN" "$HERMES_CLI" "$GH_REPO" \
                <<'PYEOF'
import sqlite3, subprocess, sys, time, re, os, json

db_path        = sys.argv[1]
board          = sys.argv[2]
now_s          = int(sys.argv[3])
today_start    = int(sys.argv[4])
age_threshold  = int(sys.argv[5])
marker_tag     = sys.argv[6]
dry_run        = (sys.argv[7].lower() == "true")
hermes_cli     = sys.argv[8]
gh_repo        = sys.argv[9]

PR_REGEX = re.compile(r'(?:#(\d+)\b|\bPR\s*#(\d+)\b|\bPR\s+(\d+)\b)')

def log(msg):
    print(f"[{time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}] "
          f"stale-blocked-watchdog: {msg}", file=sys.stderr)

def extract_pr_numbers(text):
    """Извлечь номера PR из текста. Возвращает set[int]."""
    if not text:
        return set()
    out = set()
    for m in PR_REGEX.finditer(text):
        for g in m.groups():
            if g:
                try:
                    out.add(int(g))
                except ValueError:
                    pass
    return out

# gh cache (per-process, not persisted between ticks — keeps simple)
_pr_cache = {}

def gh_pr_status(pr_num):
    """Возвращает dict {state, merged, mergedAt} или None для несуществующего PR.
    Использует REST API через `gh api`. Кэширует ответ в памяти процесса.
    """
    if pr_num in _pr_cache:
        return _pr_cache[pr_num]
    try:
        proc = subprocess.run(
            [os.environ.get('HERMES_GH', 'gh'), 'api',
             f'repos/{gh_repo}/pulls/{pr_num}',
             '--jq', '{state: .state, merged: .merged, merged_at: .merged_at, base_ref: .base.ref}'],
            capture_output=True, text=True, timeout=15, check=False,
        )
        if proc.returncode != 0:
            # 404 = не PR (может issue) → not_a_pr
            _pr_cache[pr_num] = {"state": "not_a_pr", "merged": False, "merged_at": None}
            return _pr_cache[pr_num]
        data = json.loads(proc.stdout)
        _pr_cache[pr_num] = data
        return data
    except (subprocess.TimeoutExpired, json.JSONDecodeError) as e:
        log(f"WARN gh_pr_status({pr_num}) exception: {type(e).__name__}:{e}")
        _pr_cache[pr_num] = {"state": "error", "merged": False, "merged_at": None, "error": str(e)}
        return _pr_cache[pr_num]

try:
    con = sqlite3.connect(db_path, timeout=10)
    con.row_factory = sqlite3.Row
    cur = con.execute(
        "SELECT id, body, status, created_at, started_at "
        "FROM tasks WHERE status='blocked'"
    )
    candidates = cur.fetchall()
except sqlite3.OperationalError as e:
    log(f"SKIP board={board} db={db_path} err=operational:{e}")
    print("__STATS__:0:0:0:0:0:0:0:0")
    sys.exit(0)

stats = {
    "scanned": len(candidates),
    "matched": 0,
    "emitted": 0,
    "skipped_idempotent": 0,
    "skipped_unmerged_pr": 0,
    "skipped_unfinished_parent": 0,
    "skipped_no_pr_ref": 0,
    "errors": 0,
    "records": [],
}

for row in candidates:
    tid  = row["id"]
    body = row["body"] or ""

    # 1. Extract PR numbers from body
    body_prs = extract_pr_numbers(body)
    # 2. + из последнего blocked-event payload
    last_block_payload = ""
    try:
        ev_cur = con.execute(
            "SELECT payload FROM task_events "
            "WHERE task_id=? AND kind='blocked' "
            "ORDER BY created_at DESC LIMIT 1",
            (tid,)
        )
        ev_row = ev_cur.fetchone()
        if ev_row and ev_row["payload"]:
            try:
                pld = json.loads(ev_row["payload"])
                if isinstance(pld, dict) and "reason" in pld:
                    last_block_payload = str(pld.get("reason", ""))
                else:
                    last_block_payload = ev_row["payload"]
            except (json.JSONDecodeError, TypeError):
                last_block_payload = ev_row["payload"]
    except sqlite3.OperationalError:
        pass
    block_prs = extract_pr_numbers(last_block_payload)

    all_prs = body_prs | block_prs
    if not all_prs:
        stats["skipped_no_pr_ref"] += 1
        continue

    # 3. Check parents: все done?
    parents_unfinished = []
    try:
        p_cur = con.execute(
            "SELECT t.id, t.status FROM task_links l "
            "JOIN tasks t ON t.id=l.parent_id "
            "WHERE l.child_id=?",
            (tid,)
        )
        for p in p_cur.fetchall():
            if p["status"] != "done":
                parents_unfinished.append((p["id"], p["status"]))
    except sqlite3.OperationalError:
        pass

    if parents_unfinished:
        stats["skipped_unfinished_parent"] += 1
        log(f"SKIP {tid} parents_unfinished={[p[0]+'='+p[1] for p in parents_unfinished]}")
        continue

    # 4. Check PR status: все merged?
    # Логика: пропускаем «not_a_pr» (это issue или другой entity — не блокер).
    # Реальный блокер: PR существует (state ∈ {open, closed}), и merged=false.
    unmerged_prs = []
    pr_statuses = {}  # для comment body
    not_pr_numbers = []
    real_pr_numbers = []
    for pr_num in sorted(all_prs):
        s = gh_pr_status(pr_num)
        pr_statuses[pr_num] = s
        state = s.get("state")
        if state == "not_a_pr":
            not_pr_numbers.append(pr_num)
            continue  # issue #NNNN или другой entity — не блокер
        real_pr_numbers.append(pr_num)
        if state != "closed" or not s.get("merged"):
            # PR существует и не merged в base → реальный блокер
            unmerged_prs.append((pr_num, state, s.get("merged")))

    # Если у нас вообще нет реальных PR-номеров (только issue/мусор) — SKIP.
    # Это false-positive heuristic'а; алерт не нужен.
    if not real_pr_numbers:
        stats["skipped_no_pr_ref"] += 1
        log(f"SKIP {tid} no_real_prs (only not_a_pr refs: {not_pr_numbers})")
        continue

    if unmerged_prs:
        stats["skipped_unmerged_pr"] += 1
        log(f"SKIP {tid} unmerged_prs={unmerged_prs}")
        continue

    # 5. Idempotency: уже alert'или сегодня?
    try:
        idem_cur = con.execute(
            "SELECT 1 FROM task_comments "
            "WHERE task_id=? AND body LIKE ? AND created_at > ? LIMIT 1",
            (tid, f"%{marker_tag}%", today_start),
        )
        if idem_cur.fetchone() is not None:
            stats["skipped_idempotent"] += 1
            log(f"SKIP {tid} (idempotent — alert уже есть за today)")
            continue
    except sqlite3.OperationalError:
        pass

    stats["matched"] += 1

    # 6. Build comment body
    pr_lines = []
    for pr_num in sorted(pr_statuses):
        s = pr_statuses[pr_num]
        merged_at = (s.get("merged_at") or "?")[:10]
        pr_lines.append(f"  - PR #{pr_num} → merged={s.get('merged')} ({merged_at})")
    pr_block = "\n".join(pr_lines)

    parents_block = "_(нет parent-карточек)_"
    if not parents_unfinished:
        # Все parents done (иначе бы SKIP выше)
        try:
            p_cur = con.execute(
                "SELECT t.id FROM task_links l JOIN tasks t ON t.id=l.parent_id "
                "WHERE l.child_id=?",
                (tid,)
            )
            parent_ids = [r["id"] for r in p_cur.fetchall()]
            if parent_ids:
                parents_block = ", ".join(f"`{p}`" for p in parent_ids)
        except sqlite3.OperationalError:
            pass

    comment_body = (
        f"{marker_tag} (ретро t_55c6c882, cron auto):\n\n"
        f"Карточка `{tid}` blocked, но **все prerequisites уже merged в `{gh_repo}` develop**:\n\n"
        f"**PRs:**\n{pr_block}\n\n"
        f"**Parents (kanban):** {parents_block}\n\n"
        f"**Что делать:**\n"
        f"1. Проверить, что `kind=blocked` event действительно ждёт этих PR\n"
        f"   (выше — `{last_block_payload[:200]}`).\n"
        f"2. Если всё OK — `hermes kanban unblock {tid}` (или assignee делает это сам).\n"
        f"3. Если карточка ждёт ещё чего-то (mic-infra, как в t_e8072691) —\n"
        f"   обновить block-reason и закрыть alert через unblock + re-block с новой\n"
        f"   причиной (это сбросит block_recurrences и даст свежий TTL).\n\n"
        f"_Этот alert auto-emit'ится раз в сутки (idempotent). Если карточка должна\n"
        f"оставаться blocked дольше — добавьте в block-reason явный «PR-not-yet-merged»_\n"
        f"_или используйте `hermes kanban block --kind=transient` (24h TTL)._"
    )

    if dry_run:
        stats["emitted"] += 1
        log(f"[DRY-RUN] {tid} would-alert PRs={sorted(pr_statuses)} parents={parents_block}")
        stats["records"].append((tid, board, "DRY-RUN", str(sorted(pr_statuses))))
        continue

    # 7. Side-effect: hermes kanban --board <board> comment <tid> <body>
    try:
        proc = subprocess.run(
            [hermes_cli, "kanban", "--board", board, "comment", tid, comment_body],
            capture_output=True, text=True, timeout=15, check=False,
        )
        if proc.returncode == 0:
            stats["emitted"] += 1
            stats["records"].append((tid, board, "OK", str(sorted(pr_statuses))))
            log(f"COMMENT {tid} board={board} PRs={sorted(pr_statuses)}")
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
      f"{stats['emitted']}:{stats['skipped_idempotent']}:"
      f"{stats['skipped_unmerged_pr']}:{stats['skipped_unfinished_parent']}:"
      f"{stats['skipped_no_pr_ref']}:{stats['errors']}")
for r in stats["records"]:
    print(f"__RECORD__:{r[0]}|{r[1]}|{r[2]}|{r[3]}")
PYEOF
    )
done < <(printf '%s\n' "$_db_list")

# --- structured summary ----------------------------------------------------
echo "[$(_now_iso)] stale-blocked-watchdog: ✓ done scanned=${_scanned_total} matched=${_matched_total} emitted=${_emitted_total} skipped_idempotent=${_skipped_idempotent_total} skipped_unmerged_pr=${_skipped_unmerged_pr_total} skipped_unfinished_parent=${_skipped_unfinished_parent_total} skipped_no_pr_ref=${_skipped_no_pr_ref_total} errors=${_errors_total}" >&2

# --- write stats log -------------------------------------------------------
mkdir -p "$(dirname "$LOG_FILE")" 2>/dev/null || true
{
    printf '# stale-blocked-watchdog snapshot %s\n' "$(_now_iso)"
    printf 'scanned=%s matched=%s emitted=%s skipped_idempotent=%s skipped_unmerged_pr=%s skipped_unfinished_parent=%s skipped_no_pr_ref=%s errors=%s\n' \
        "$_scanned_total" "$_matched_total" "$_emitted_total" \
        "$_skipped_idempotent_total" "$_skipped_unmerged_pr_total" \
        "$_skipped_unfinished_parent_total" "$_skipped_no_pr_ref_total" \
        "$_errors_total"
} >> "$LOG_FILE" 2>/dev/null || true

# --- exit code -------------------------------------------------------------
# Exit 2 = хотя бы одна stale-blocked карточка найдена и заalert'ена (или
# WOULD-была в DRY_RUN). Это ДЕТЕКТ-сignal для cron, не зависит от DRY_RUN
# (DRY_RUN влияет только на emit-side-effects: пишем ли реальный comment в
# kanban.DB или нет). Exit code == fact-of-detection; DRY_RUN == fact-of-mutation.
# Issue #2481: раньше DRY_RUN=true глушил exit 2 → cron не alert'ил оператора
# при dryrun-тестах (false-sense-of-safety).
if [ "$_emitted_total" -gt 0 ]; then
    exit 2
fi
exit 0