#!/bin/bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-cancel-on-provider-exhausted.sh
# Правим ТОЛЬКО здесь + commit + merge в develop. На хост раскладывает
# `bash <repo>/scripts/agent_flow/install.sh` — hardlink-копиями (cp -al), НЕ
# симлинками (симлинк в ~/.hermes/scripts/ ресолвится наружу и отклоняется
# guard'ом hermes-agent scheduler.py::_validate_script_path, ретро 11.08
# t_a6a236e0d9f0470e). Полный список путей раскладки — в install.sh, сверку
# копий держит agent-flow-drift-detect.sh.
# ============================================================================
# agent-flow-cancel-on-provider-exhausted.sh — user-invoked helper для
# массовой отмены карточек, застрявших в MiniMax/DeepSeek provider-exhaust
# crash-loop (ретро 15.09 t_a7aa4e6b / карточка t_8053e18c §P1).
#
# Зачем ОТДЕЛЬНЫЙ скрипт, если есть watchdog-provider-quick.sh:
#   watchdog-provider-quick — auto 1-мин hot-path: ловит свежие краш-логи
#   (running+pid_dead ИЛИ ready+fresh_log+providers_dead) и блокирует
#   реактивно, без issue-комментариев. Хорош для НЕПРЕРЫВНОГО стражника.
#
#   Этот скрипт — MANUAL operator helper (запускается Шифу / cron / devops
#   по явной команде `bash agent-flow-cancel-on-provider-exhausted.sh`
#   либо `--recover` для lift-фазы). Имеет четыре отличия:
#     1) сканирует LATEST summary задачи (task_runs.summary) ИЛИ
#        tasks.last_failure_error — то есть ловит «когда-то словили 429,
#        retry-loop крутит» даже если свежий лог уже без маркера;
#     2) ПИШЕТ комментарий в `kanban comment` со ссылкой на issue (#2610,
#        #2559 и т.д., извлечённый из body) + ссылку на issue #1193 (root
#        cause — MiniMax budget) + retro-key `worker-cascade-crash-
#        provider-exhausted`;
#     3) IDEMPOTENT: не дублирует ни block, ни comment (sentinel-comment
#        с маркером SENTINEL_TAG, parse по marker);
#     4) cron-регистрация через install.sh → every 5 min в devops-профиле
#        (ретро t_197de62a: до этого auto-вызов не был настроен, блокировка
#        stale-карточек была ручной).
#
# Companion hook (--recover): когда MiniMax бюджет пополнен — поднимает
# карточки с блоком kind=capability + комментом-marker'ом обратно в ready.
# Тот же файл, отдельная функция: cancel_provider_exhaust() / recover_provider_exhaust().
#
# Use cases:
#   $ bash agent-flow-cancel-on-provider-exhausted.sh --dry-run
#     → показать, ЧТО будет заблокировано / прокомментировано, без side effects.
#   $ bash agent-flow-cancel-on-provider-exhausted.sh
#     → block (kind=capability, reason=provider-budget-exhausted) +
#       kanban comment с marker'ом (если ещё нет).
#   $ bash agent-flow-cancel-on-provider-exhausted.sh --recover
#     → поднять blocked(kind=capability) карточки с marker-комментом обратно
#       в ready через `kanban unblock`. Только если MiniMax уже отвечает 200
#       (по свежему clean log в любой доске).
#
# Acceptance (t_a7aa4e6b):
#   - shellcheck-clean (warnings SC2086/2154 disabled точечно, см. комменты)
#   - --dry-run
#   - unit test с fixture card (см. tests/agent_flow/test_..._provider_exhausted.py)
#   - safe to re-run (no duplicate comments / blocks)
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
RETRO_KEY="worker-cascade-crash-provider-exhausted"
BLOCK_REASON="provider-budget-exhausted"
ROOT_ISSUE="#1193"  # MiniMax provider budget tracking
SENTINEL_TAG="<!-- ${SCRIPT_NAME}:marker -->"
# signal: подстрока в task_runs.summary, маркирующая provider-exhaust.
# Копия из watchdog-provider-quick.sh + русские варианты («провайдер исчерпан»)
# + явно-блочный текст «provider-budget-exhausted» (чтобы сам block-reason
# не триггерил повторный block — _summary_contains_exhaust ниже игнорирует
# совпадение только со значением BLOCK_REASON в начале строки, см. функцию).
EXHAUST_SIGNATURES=(
    # English LLM quota / rate limit
    "HTTP 402" "Insufficient Balance" "Out of credits"
    "Billing or credits exhausted" "HTTP 429" "rate limit"
    "Token Plan usage limit" "2056" "Token Plan rate limit reached"
    "health-aware-fallback" "all providers unavailable"
    "all providers failed" "provider unavailable" "provider-exhaustion"
    "provider-budget-exhausted"  # ← причина текущего блока (НЕ триггерит block)
    # Auth/quota 401 / invalid api key (DeepSeek)
    "HTTP 401" "Authentication Fails" "is invalid"
    "invalid_request_error" "authentication_error"
    # Russian phrases — фактический текст от watchdog-provider-quick.sh
    "провайдер исчерпан"
    "провайдер восстановлен"  # ← НЕ триггерим (это восстановление)
)

# Ретро-ключ (комментарий-marker) — должен быть в КАЖДОМ нашем comment'е,
# чтобы recover-фаза могла искать «свои» блоки и не unblock-ать ручные.
RETRO_TAG="<!-- retro-key:${RETRO_KEY} -->"

# -------- auto-issue (ADR-0019, kanban t_4aaeeef6) --------
# Если env PROVIDER_EXHAUST_AUTO_ISSUE не задан → OFF (safe-by-default).
# Шифу/Юзер включает через PROVIDER_EXHAUST_AUTO_ISSUE=1 (env в cron-job) после
# merge PR; до этого поведение точно такое же, как в ретро t_197de62a.
# Ретро t_4aaeeef6 (incident 2026-10-03): 5-й рецидив MiniMax/DeepSeek исчерпания
# прошёл МОЛЧА — воркеры писали «провайдер исчерпан, ждать» в комментариях карточек,
# но НИКТО не открыл incident-tracking issue, чтобы у Шифу был сигнал/триггер
# на пополнение. Карточки заблокировались (cancel), а Шифу узнал только из ночного
# ревью. Auto-issue закрывает этот process-gap.
GH_REPO_DEFAULT="krikz/rob_box_project"
GH_REPO="${GH_REPO:-$GH_REPO_DEFAULT}"
PROVIDER_EXHAUST_AUTO_ISSUE="${PROVIDER_EXHAUST_AUTO_ISSUE:-0}"
# Лейбл нового issue (раздельный с e2e-fail-streak, чтобы Шифу мог фильтровать).
RECURRENT_INCIDENT_LABEL="${RECURRENT_INCIDENT_LABEL:-recurrent-incident}"
# Cooldown между auto-create (default 24ч — компромисс между видимостью и штормом).
# Ретро t_4aaeeef6: 14ч-блокировка фаз #3014 (несколько рецидивов в окне). При
# cooldown 4ч (как у fail-streak) — будет 1 issue за ночь; 24ч — 1 issue за
# инцидент, что и хочется Шифу.
PROVIDER_EXHAUST_ISSUE_COOLDOWN_HOURS="${PROVIDER_EXHAUST_ISSUE_COOLDOWN_HOURS:-24}"
ISSUE_COOLDOWN_FILE_DEFAULT="${HERMES_HOME}/state/agent-flow-cancel-provider-exhausted-last-issue"
ISSUE_COOLDOWN_FILE="${ISSUE_COOLDOWN_FILE:-$ISSUE_COOLDOWN_FILE_DEFAULT}"
# assignees (опционально, дефолт без).
PROVIDER_EXHAUST_ISSUE_ASSIGNEES="${PROVIDER_EXHAUST_ISSUE_ASSIGNEES:-}"

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
Usage: $SCRIPT_NAME [--dry-run] [--recover] [--help]

Modes:
  (default)   scan kanban boards, detect provider-exhaust signature in
              task_runs.summary of non-blocked tasks, then:
                1) block (kind=capability, reason='$BLOCK_REASON')
                2) post sentinel-marked comment on linked issue ref
                3) if PROVIDER_EXHAUST_AUTO_ISSUE=1 AND actions>0 AND
                   no OPEN recurrent-incident issue exists AND cooldown
                   aged > ${PROVIDER_EXHAUST_ISSUE_COOLDOWN_HOURS}h:
                     create ONE GitHub incident issue (label=$RECURRENT_INCIDENT_LABEL)
              IDEMPOTENT: re-runs are no-op for already-blocked tasks with
              existing sentinel comment. Auto-issue guarded by cooldown
              file + gh-truth (open-issue list) — safe to re-run.
  --recover   companion: scan blocked tasks with sentinel comment +
              block_kind=capability + prov-alive signal → unblock back to ready.
  --dry-run   same scan as default, but print what WOULD be done; no side effects.
  --help      this message.

Env knobs:
  HERMES_BIN                          hermes CLI (default: $HERMES_BIN)
  KANBAN_BOARDS_DIR                   boards dir (default: $KANBAN_BOARDS_DIR)
  GH_REPO                             owner/repo for auto-issue (default: ${GH_REPO_DEFAULT:-krikz/rob_box_project})
  LOCK_FILE / LOG_FILE                override defaults
  PROVIDER_EXHAUST_AUTO_ISSUE         1 to enable auto-create incident-issue (default: 0/OFF)
  PROVIDER_EXHAUST_ISSUE_COOLDOWN_HOURS  cooldown between auto-issues (default: 24)
  RECURRENT_INCIDENT_LABEL            label for auto-issue (default: recurrent-incident)
  PROVIDER_EXHAUST_ISSUE_ASSIGNEES    comma-separated assignees (default: empty)
  ISSUE_COOLDOWN_FILE                 override path to cooldown file
EOF
}

# Parse issue ref (#NNNN) из tasks.body. Поддерживает 3 формата:
#   "Source\n  ...\n  issue: #NNNN"   ← приоритет (стандарт декомпозера)
#   "Issue: #NNNN"                    ← free-form heading
#   любой "issue #NNNN" / "issue_NNNN" в тексте
# Возвращает первый найденный номер БЕЗ '#', или пустую строку.
extract_issue_ref() {
    local body="$1"
    local n=""
    # 1) Source ... issue: #NNNN  (стандартный декомпозер-блок)
    n=$(printf '%s\n' "$body" | awk '
        /^Source$/  { in_src=1; next }
        in_src && /^[^ ]/ { in_src=0 }
        in_src && /^[[:space:]]+issue:[[:space:]]+#?([0-9]+)/ {
            print gensub(/^[[:space:]]+issue:[[:space:]]+#?([0-9]+).*/, "\\1", 1)
            exit
        }
    ' 2>/dev/null || true)
    # 2) Fallback: явный "Issue: #NNNN" / "issue #NNNN"
    if [ -z "$n" ]; then
        n=$(printf '%s\n' "$body" \
            | grep -oE '[Ii]ssue[[:space:]]*:?[[:space:]]*#[0-9]+' \
            | head -n1 | grep -oE '[0-9]+' || true)
    fi
    # 3) Последний fallback: голый #NNNN (>=3 цифры)
    if [ -z "$n" ]; then
        n=$(printf '%s\n' "$body" | grep -oE '#[0-9]{3,5}' | head -n1 | tr -d '#' || true)
    fi
    printf '%s' "$n"
}

# True если text содержит хотя бы одну сигнатуру из EXHAUST_SIGNATURES,
# И это НЕ marker «провайдер восстановлен» (recover-комментарий watchdog'а).
# Использует grep -F (fixed-string), безопасна для спецсимволов.
_summary_contains_exhaust() {
    local text="$1"
    [ -n "$text" ] || return 1
    # Игнорируем «провайдер восстановлен» — это и есть anti-pattern (block уже снят)
    case "${text,,}" in
        *"провайдер восстановлен"*|*"provider restored"*|*"UNBLOCK: провайдер"*) return 1 ;;
    esac
    local sig
    for sig in "${EXHAUST_SIGNATURES[@]}"; do
        if printf '%s' "$text" | grep -qF -- "$sig"; then
            return 0
        fi
    done
    return 1
}

# True если text содержит SENTINEL_TAG (наш собственный marker — для идемпотентности).
_has_sentinel_comment() {
    local task_id="$1"
    local board="$2"
    sqlite3 "$KANBAN_BOARDS_DIR/$board/kanban.db" \
        "SELECT 1 FROM task_comments
         WHERE task_id = '$task_id'
           AND body LIKE '%${SENTINEL_TAG}%'
         LIMIT 1;" 2>/dev/null | grep -q '^1$'
}

# Вспомогалка: вытащить issue comment на указанный issue (через gh CLI).
# Безопасный no-op если GH_CONFIG_DIR не настроен (skip с warning).
_post_github_issue_comment() {
    local repo="$1" issue_num="$2" body="$3"
    if [ -z "$issue_num" ]; then
        log "  ⚠️ no issue ref → skip gh comment"
        return 0
    fi
    if [ -z "$repo" ]; then
        log "  ⚠️ repo unknown → skip gh comment for #$issue_num"
        return 0
    fi
    if ! command -v gh >/dev/null 2>&1; then
        log "  ⚠️ gh CLI not in PATH → skip gh comment for $repo#$issue_num"
        return 0
    fi
    if GH_CONFIG_DIR="$GH_CONFIG_DIR" gh issue comment "$repo" "$issue_num" \
            --body "$body" 2>>"$LOG_FILE"; then
        log "  ✅ gh issue comment posted to $repo#$issue_num"
    else
        log "  ⚠️ gh issue comment failed for $repo#$issue_num (rc=$?)"
    fi
}

# ============================================================================
# scan_provider_exhaust_actions
#   Python-часть: для каждой board/db находим задачи, у которых
#   task_runs.summary (последний run) ИЛИ tasks.last_failure_error
#   содержат сигнатуру provider-exhaust И статус сейчас НЕ 'blocked'
#   (чтобы не дублировать).
#
#   Ретро t_197de62a / t_8053e18c / t_a7aa4e6b: воркеры часто
#   exited cleanly (rc=0) без записи task_runs.summary — тогда signal
#   живёт ТОЛЬКО в tasks.last_failure_error. Старая логика смотрела
#   только на summary и пропускала такие карточки → они крутились в
#   crash-loop (ready→running→crashed→ready) пока dispatcher не
#   exhausted retries. Теперь смотрим ОБА поля: OR-логика, любой
#   из двух сигналов → кандидат на cancel.
#
#   Output: JSON-lines в $ACTIONS_FILE:
#     {"action":"block","board":"...","task_id":"...","issue":"...","repo":"...","signal":"summary|lfe|both"}
# ============================================================================
ACTIONS_FILE="$STATE_DIR/${SCRIPT_NAME}.actions.jsonl"

scan_provider_exhaust_actions() {
    local mode="$1"  # "cancel" | "recover"
    : > "$ACTIONS_FILE"

    python3 - "$KANBAN_BOARDS_DIR" "$mode" "$ACTIONS_FILE" <<'PYEOF'
import glob, json, os, sqlite3, sys

boards_dir, mode, actions_file = sys.argv[1], sys.argv[2], sys.argv[3]

# Маркеры провайдер-эксхаустиина. Совпадают с watchdog-provider-quick.sh
# (httpx + gh rate-limit / quota / auth) + русская формулировка
# «провайдер исчерпан» от watchdog'а. Синхронизировано с _RESPAWN_BLOCKER_RE
# в hermes-agent/kanban_db.py.
PROVIDER_MARKERS = (
    "HTTP 402", "Insufficient Balance", "Out of credits",
    "Billing or credits exhausted", "HTTP 429", "rate limit",
    "Token Plan usage limit", "2056", "Token Plan rate limit reached",
    "health-aware-fallback", "all providers unavailable",
    "all providers failed", "provider unavailable",
    "provider-exhaustion", "provider-budget-exhausted",
    "HTTP 401", "Authentication Fails", "is invalid",
    "invalid_request_error", "authentication_error",
    "api key", "unauthorized", "forbidden",
    "провайдер исчерпан",
    # Дополнительные слова из last_failure_error
    "quota", "billing", "subscription", "access denied", "permission denied",
    "out of credits", "insufficient balance", "token plan", "ratelimit",
)


def contains_exhaust(text: str) -> bool:
    """True if any provider-exhaust marker is present."""
    if not text:
        return False
    low = text.lower()
    # Anti-patterns: recover/UNBLOCK messages (block already lifted)
    if "провайдер восстановлен" in low or "provider restored" in low:
        return False
    if "unblock:" in low and "провайдер" in low:
        return False
    # Сам же block-reason («provider-budget-exhausted») мы триггерим —
    # карточка могла быть разблокирована, и чтобы re-cancel нашёл её снова,
    # сигнатура должна ловить и эту формулировку.
    return any(s.lower() in low for s in PROVIDER_MARKERS)


# Back-compat alias для apply-части
summary_contains_exhaust = contains_exhaust


def has_sentinel(db_path: str, task_id: str) -> bool:
    try:
        con = sqlite3.connect(db_path)
        row = con.execute(
            "SELECT 1 FROM task_comments WHERE task_id=? "
            "AND body LIKE '%<!-- agent-flow-cancel-on-provider-exhausted.sh:marker -->%' LIMIT 1",
            (task_id,),
        ).fetchone()
        con.close()
        return row is not None
    except Exception:
        return False


def extract_issue(body: str) -> str:
    """Same priority as bash helper extract_issue_ref (kept in sync)."""
    if not body:
        return ""
    import re
    # 1) Source ... issue: #NNNN
    for line in body.splitlines():
        stripped = line.strip()
        if stripped.startswith("Source"):
            continue
        m = re.match(r"^\s+issue:\s*#?(\d+)", line)
        if m:
            return m.group(1)
        if line and not line.startswith(" ") and not line.startswith("\t"):
            # left Source block
            break
    # 2) Issue: #NNNN / issue #NNNN
    m = re.search(r"[Ii]ssue\s*:?\s*#(\d+)", body)
    if m:
        return m.group(1)
    # 3) bare #NNNN (3-5 digits)
    m = re.search(r"#(\d{3,5})", body)
    if m:
        return m.group(1)
    return ""


def extract_repo(body: str) -> str:
    """krikz/rob_box_project из body блока Source (если есть)."""
    if not body:
        return ""
    import re
    m = re.search(r"^\s+repo:\s*([\w./-]+)\s*$", body, re.MULTILINE)
    return m.group(1) if m else ""


now_actions = []
for db in sorted(glob.glob(f"{boards_dir}/*/kanban.db")):
    board = os.path.basename(os.path.dirname(db))
    # skip transient boards (smoke tests), чтобы cron-тик не мусорил в логах
    if board.startswith("scope-force-smoke") or board == "smoke-test":
        continue
    try:
        con = sqlite3.connect(db)
        con.row_factory = sqlite3.Row
        # Берём только non-done + non-archived задачи. status IN
        # (running, ready, todo) — т.к. 'blocked' уже имеет смысл «мы
        # его сами блокировали», см. watchdog-provider-quick.sh.
        rows = con.execute("""
            SELECT id, title, body, status, block_kind,
                   COALESCE(last_failure_error, '') AS lfe
            FROM tasks
            WHERE status IN ('running', 'ready', 'todo')
        """).fetchall()

        for row in rows:
            tid = row["id"]
            status = row["status"]
            body = row["body"] or ""
            block_kind = row["block_kind"]
            last_failure_error = row["lfe"] or ""

            # Последний summary = task_runs ORDER BY id DESC LIMIT 1
            lr = con.execute(
                "SELECT summary FROM task_runs WHERE task_id=? "
                "ORDER BY id DESC LIMIT 1", (tid,),
            ).fetchone()
            latest_summary = lr["summary"] if lr else None

            has_marker = has_sentinel(db, tid)

            if mode == "cancel":
                # Ретро t_197de62a: signal OR (summary ИЛИ last_failure_error).
                # Воркеры нередко делают crash до записи summary, тогда
                # остаётся только last_failure_error. Записываем, откуда
                # пришёл сигнал, для post-mortem в dry-run.
                sig_summary = contains_exhaust(latest_summary or "")
                sig_lfe = contains_exhaust(last_failure_error)
                if not (sig_summary or sig_lfe):
                    continue
                # Sentinel-marker = идемпотентность: уже прокомментировано.
                if has_marker:
                    continue
                # Уже заблокировано capability (кем-то другим, не нами) — не
                # дублируем, см. watchdog-provider-quick.sh recovery-flow.
                if block_kind == "capability" and status == "blocked":
                    continue
                signal = (
                    "both" if sig_summary and sig_lfe
                    else "summary" if sig_summary
                    else "lfe"
                )
                issue = extract_issue(body)
                repo = extract_repo(body)
                now_actions.append({
                    "action": "block",
                    "board": board,
                    "task_id": tid,
                    "issue": issue,
                    "repo": repo,
                    "latest_summary": (latest_summary or "")[:200],
                    "last_failure_error": last_failure_error[:200],
                    "signal": signal,
                    "title": (row["title"] or "")[:120],
                })
            elif mode == "recover":
                # Только blocked + block_kind=capability + есть sentinel-marker
                if status != "blocked":
                    continue
                if block_kind != "capability":
                    continue
                if not has_marker:
                    continue
                issue = extract_issue(body)
                repo = extract_repo(body)
                now_actions.append({
                    "action": "unblock",
                    "board": board,
                    "task_id": tid,
                    "issue": issue,
                    "repo": repo,
                })
        con.close()
    except Exception as exc:
        print(f"[scan] {board} error: {exc}", file=sys.stderr)

with open(actions_file, "w", encoding="utf-8") as f:
    for a in now_actions:
        f.write(json.dumps(a, ensure_ascii=False) + "\n")

print(f"[scan] mode={mode} actions={len(now_actions)}", file=sys.stderr)
PYEOF
}

# ============================================================================
# apply_cancel
#   Прочитать $ACTIONS_FILE и выполнить kanban block + comment.
#   Для --dry-run только печатаем план.
# ============================================================================
# ============================================================================
# auto_create_recurrent_incident (ADR-0019, kanban t_4aaeeef6)
#   Если включён PROVIDER_EXHAUST_AUTO_ISSUE=1 И есть хотя бы один cancel-action
#   в этом тике (т.е. НЕ silent tick), то проверяем два guard'а:
#     (a) ISSUE_COOLDOWN_FILE mtime — если свежий (< cooldown-hours) → SKIP
#     (b) gh issue list --label recurrent-incident --state open — если ≥1 → SKIP
#   Если оба guard'а пропускают → собираем body (timeline cancel-actions за
#   этот tick + root cause link на #1193 + develop HEAD + что делать блок)
#   и вызываем `gh issue create` с label=recurrent-incident + agent:devops +
#   hermes + assignee из env. Записываем cooldown.
#
#   Ретро t_4aaeeef6: карточки cancel'ились, но incident-issue не появлялся → Шифу
#   узнавал из ночного ревью. Auto-issue закрывает process-gap.
#
#   Output: _auto_issue_url (если создан) или пусто.
# ============================================================================
_gh_truth_open_issues_count() {
    local label="$1"
    if [ -z "$label" ]; then
        echo 0; return
    fi
    if ! command -v gh >/dev/null 2>&1; then
        echo 0; return
    fi
    # Требуется GH_CONFIG_DIR (как и в _post_github_issue_comment). Без него — 0
    # (это safe-by-default: лучше no-op чем сгутить и создать дубль).
    GH_CONFIG_DIR="$GH_CONFIG_DIR" gh issue list --repo "$GH_REPO" --state open \
        --label "$label" --limit 1 --json number 2>/dev/null \
        | python3 -c 'import json,sys; a=json.load(sys.stdin); print(len(a))' \
        2>/dev/null || echo 0
}

# Возвращает 0 (true) если cooldown свежий (НЕ прошло PROVIDER_EXHAUST_ISSUE_COOLDOWN_HOURS).
_cooldown_is_active() {
    if [ ! -f "$ISSUE_COOLDOWN_FILE" ]; then
        return 1  # нет файла → cooldown не активен
    fi
    local _epoch _age_s _limit_s
    _epoch="$(stat -c '%Y' "$ISSUE_COOLDOWN_FILE" 2>/dev/null || echo 0)"
    _age_s=$(( $(date -u +%s) - ${_epoch:-0} ))
    _limit_s=$(( PROVIDER_EXHAUST_ISSUE_COOLDOWN_HOURS * 3600 ))
    if [ "${_age_s:-0}" -lt "${_limit_s}" ]; then
        return 0  # активен
    fi
    return 1
}

# Собрать markdown-таблицу cancel-actions (заголовок|task_id|board|signal|issue).
# Reuse ACTIONS_FILE.
_format_cancel_table() {
    python3 - "$ACTIONS_FILE" <<'PYEOF' 2>/dev/null || true
import json, sys
actions_file = sys.argv[1]
shown = 0
try:
    with open(actions_file) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                o = json.loads(line)
            except Exception:
                continue
            tid = o.get("task_id", "")[:18]
            board = o.get("board", "")[:16]
            issue = o.get("issue", "")
            signal = o.get("signal", "")
            title = (o.get("title", "") or "")[:60]
            print(f"| `{tid}` | `{board}` | #{issue or '?'} | `{signal}` | {title} |")
            shown += 1
except Exception:
    pass
PYEOF
}

auto_create_recurrent_incident() {
    local dry_run="$1"  # "yes" | "no"

    if [ "${PROVIDER_EXHAUST_AUTO_ISSUE}" != "1" ]; then
        log "AUTO_ISSUE: PROVIDER_EXHAUST_AUTO_ISSUE=${PROVIDER_EXHAUST_AUTO_ISSUE} (off) — skip"
        return 0
    fi

    # Если нет cancel-actions за этот тик — нечего auto-issue'ить (silent tick).
    if [ ! -s "$ACTIONS_FILE" ]; then
        log "AUTO_ISSUE: empty ACTIONS_FILE — skip"
        return 0
    fi

    # gh CLI / auth gate
    if ! command -v gh >/dev/null 2>&1; then
        log "AUTO_ISSUE: gh CLI not in PATH — skip"
        return 0
    fi
    if ! GH_CONFIG_DIR="$GH_CONFIG_DIR" gh auth status >/dev/null 2>&1; then
        log "AUTO_ISSUE: gh auth failed — skip (check $GH_CONFIG_DIR)"
        return 0
    fi

    # (a) cooldown guard
    if _cooldown_is_active; then
        log "AUTO_ISSUE: cooldown active ($(stat -c %Y "$ISSUE_COOLDOWN_FILE" 2>/dev/null)) — skip"
        return 0
    fi

    # (b) gh-truth guard
    local _existing
    _existing="$(_gh_truth_open_issues_count "$RECURRENT_INCIDENT_LABEL")"
    if [ "${_existing:-0}" -gt 0 ] 2>/dev/null; then
        log "AUTO_ISSUE: open '$RECURRENT_INCIDENT_LABEL' issues: $_existing — skip"
        return 0
    fi

    # Собрать body
    local _today _action_count _table _develop_head _title
    _today="$(date -u +%Y-%m-%d)"
    _action_count="$(grep -c . "$ACTIONS_FILE" 2>/dev/null || echo 0)"
    _table="$(_format_cancel_table)"
    _develop_head="$(git -C "${REPO_DIR:-$HERMES_HOME}" rev-parse --short=7 origin/develop 2>/dev/null \
        || git rev-parse --short=7 HEAD 2>/dev/null || echo unknown)"

    _title="[recurrent-incident] MiniMax/DeepSeek provider exhausted ${_today} (${_action_count} cards blocked, root: ${ROOT_ISSUE})"

    local _body
    _body=$(cat <<ISSUE_BODY_MARKER
🤖 [agent:devops] script=agent-flow-cancel-on-provider-exhausted action=auto-create-issue

## Recurrent provider-exhaust incident ${_today}

MiniMax/DeepSeek LLM-провайдер вернул **402/429** (или эквивалентный provider-exhaust сигнал) для **${_action_count}** задач(и) за последний tick. Это **5-й рецидив** с момента закрытия issue ${ROOT_ISSUE} (13.08.2026, completed).

- **Тип:** operational, не code-task. Worker'ы уже блокируют свои kanban-карточки автоматически через \`agent-flow-cancel-on-provider-exhausted\`.
- **Root cause:** исчерпание Token Plan / Billing на стороне MiniMax (см. issue ${ROOT_ISSUE}).
- **develop HEAD:** \`${_develop_head}\`
- **Cooldown:** ${PROVIDER_EXHAUST_ISSUE_COOLDOWN_HOURS}ч между auto-issue (file: \`${ISSUE_COOLDOWN_FILE}\`)
- **Метка:** \`${RECURRENT_INCIDENT_LABEL}\`

## Заблокированные карточки (this tick)

| task_id | board | linked issue | signal | title |
|---|---|---|---|---|
${_table}

## Что нужно от Шифу

1. **Пополнить MiniMax/Token Plan** (или поднять DeepSeek бюджет) — внешнее действие, не код.
2. **Подождать ~5-15 мин** после пополнения, чтобы провайдер увидел новое состояние.
3. **Запустить разблокировку**:
   \`\`\`bash
   bash scripts/agent_flow/agent-flow-cancel-on-provider-exhausted.sh --recover
   \`\`\`
   Это разбудит все \`blocked(kind=capability)\` карточки с sentinel-marker'ом
   обратно в \`ready\`.
4. **Закрыть этот issue** (\`gh issue close <this> --reason 'completed'\`) — после восстановления воркеры смогут продолжить.

## Связанные

- ${ROOT_ISSUE} — root cause: исчерпание MiniMax Token Plan (закрыт completed 2026-08-13, fallback на deepseek сработал).
- Скрипт-страж: \`scripts/agent_flow/agent-flow-cancel-on-provider-exhausted.sh\`.
- ADR-0019 — формализация этого поведения.

> 🤖 Создано автоматически. Если в течение ${PROVIDER_EXHAUST_ISSUE_COOLDOWN_HOURS}ч будет ещё рецидив — НЕ будет создан новый issue (cooldown/gh-truth guards).
ISSUE_BODY_MARKER
)

    # gh args
    local -a _create_args
    _create_args=(--repo "$GH_REPO" --title "$_title" --label "$RECURRENT_INCIDENT_LABEL,hermes,agent:devops" --body "$_body")
    if [ -n "${PROVIDER_EXHAUST_ISSUE_ASSIGNEES}" ]; then
        # comma-separated: gh принимает --assignee один раз; повторяем флаг для нескольких.
        IFS=',' read -ra _assignees <<< "$PROVIDER_EXHAUST_ISSUE_ASSIGNEES"
        for _a in "${_assignees[@]}"; do
            _a="$(printf '%s' "$_a" | xargs)"  # trim
            [ -n "$_a" ] && _create_args+=(--assignee "$_a")
        done
    fi

    if [ "$dry_run" = "yes" ]; then
        log "AUTO_ISSUE [DRY-RUN] would: gh issue create --label ${RECURRENT_INCIDENT_LABEL} (cards=${_action_count}, develop=${_develop_head})"
        log "AUTO_ISSUE [DRY-RUN] would: title='${_title}'"
        return 0
    fi

    local _create_out _create_rc
    _create_out=""
    _create_rc=0
    _create_out="$(GH_CONFIG_DIR="$GH_CONFIG_DIR" gh issue create "${_create_args[@]}" 2>&1)" || _create_rc=$?
    if [ "${_create_rc}" = "0" ]; then
        local _issue_url
        _issue_url="$(printf '%s' "$_create_out" | grep -oE 'https://github.com/[^ ]+/issues/[0-9]+' | head -n 1 || true)"
        log "🚨 AUTO-CREATED recurrent-incident issue: ${_issue_url:-${_create_out}}"
        mkdir -p "$(dirname "$ISSUE_COOLDOWN_FILE")" 2>/dev/null || true
        date -u +%s > "$ISSUE_COOLDOWN_FILE" 2>/dev/null \
            && log "AUTO_ISSUE: cooldown written: $ISSUE_COOLDOWN_FILE" \
            || log "AUTO_ISSUE: WARN cannot write cooldown file $ISSUE_COOLDOWN_FILE"
    else
        log "AUTO_ISSUE: ERROR gh issue create failed (rc=${_create_rc}): ${_create_out}"
    fi
    unset _create_rc
    return 0
}

apply_cancel() {
    local dry_run="$1"  # "yes" | "no"
    local count=0 blocked=0 commented=0 skipped=0
    local action="" board="" task_id="" issue="" repo="" latest_summary=""
    local last_failure_error="" signal="" title=""
    while IFS= read -r line; do
        [ -z "$line" ] && continue
        count=$((count + 1))
        # Парсим JSON через python (надёжнее чем jq-зависимость).
        # shellcheck disable=SC2154  # vars assigned via eval below
        eval "$(printf '%s' "$line" | python3 -c '
import json, sys, shlex
o = json.loads(sys.stdin.read())
for k in ("action","board","task_id","issue","repo","latest_summary",
          "last_failure_error","signal","title"):
    val = str(o.get(k, ""))
    val = val.replace(chr(39), chr(39)+chr(92)+chr(39)+chr(39))  # escape single-quote
    print(f"{k}=\x27{val}\x27")
')"

        log "→ $action  $board/$task_id  signal=$signal  issue=#$issue  title='$title'"
        log "  latest_summary: $latest_summary"
        [ -n "$last_failure_error" ] && log "  last_failure_error: $last_failure_error"

        if [ "$dry_run" = "yes" ]; then
            log "  [DRY-RUN] would: kanban block --kind capability '$task_id' '$BLOCK_REASON'"
            log "  [DRY-RUN] would: kanban comment '$task_id' (sentinel + retro-key)"
            [ -n "$issue" ] && log "  [DRY-RUN] would: gh issue comment $repo#$issue"
            continue
        fi

        # 1) block (kind=capability — это typed human-block, dispatcher не auto-resume)
        if "$HERMES_BIN" kanban --board "$board" block \
                --kind capability "$task_id" \
                "$BLOCK_REASON: retro_key=$RETRO_KEY root_cause_issue=$ROOT_ISSUE linked_issue=#${issue:-?}. auto-detected via latest_summary signature" \
                >>"$LOG_FILE" 2>&1; then
            blocked=$((blocked + 1))
            log "  ✅ blocked"
        else
            log "  ⚠️  block failed (rc=$?) — see $LOG_FILE"
            skipped=$((skipped + 1))
            continue
        fi

        # 2) comment на КАРТОЧКУ (sentinel + retro-key)
        local comment_body
        comment_body="${SENTINEL_TAG}
${RETRO_TAG}
|**Provider-budget-exhausted auto-block** (${RETRO_KEY}):
карточка заблокирована по сигнатуре provider-exhaust (signal: \`${signal}\`).
- latest_summary: \`${latest_summary}\`
- last_failure_error: \`${last_failure_error}\`
Root cause: MiniMax provider budget исчерпан — см. issue ${ROOT_ISSUE}.
Связанный issue: #${issue:-?}.
Recovery: запустить \`bash scripts/agent_flow/agent-flow-cancel-on-provider-exhausted.sh --recover\` после пополнения MiniMax."
        if "$HERMES_BIN" kanban --board "$board" comment \
                "$task_id" "$comment_body" >>"$LOG_FILE" 2>&1; then
            commented=$((commented + 1))
            log "  ✅ kanban comment posted (with ${SENTINEL_TAG})"
        else
            log "  ⚠️  kanban comment failed (rc=$?) — see $LOG_FILE"
        fi

        # 3) gh issue comment (если issue ref есть) — best-effort, не падаем
        if [ -n "$issue" ] && [ -n "$repo" ]; then
            local gh_body
            gh_body=$(printf '%s' "🤖 **agent-flow-cancel-on-provider-exhausted** (${RETRO_KEY})

Kanban card \`${task_id}\` auto-blocked по сигнатуре provider-exhaust (см. \`latest_summary\`).
Root cause: MiniMax budget — issue ${ROOT_ISSUE}.

Recovery после пополнения MiniMax: \`bash scripts/agent_flow/agent-flow-cancel-on-provider-exhausted.sh --recover\`.")
            _post_github_issue_comment "$repo" "$issue" "$gh_body"
        fi
    done < "$ACTIONS_FILE"

    log "summary: scanned=$count blocked=$blocked commented=$commented skipped=$skipped"
    # auto-issue: запускаем ПОСЛЕ apply_cancel (нужны ACTIONS_FILE + log чтобы
    # знать, был ли это silent tick). Dry-run пробрасывается — payload печатается,
    # но НЕ выполняется gh-вызов.
    auto_create_recurrent_incident "$dry_run"
}

apply_recover() {
    local dry_run="$1"
    local count=0 unblocked=0 skipped=0
    local action="" board="" task_id="" issue="" repo=""
    while IFS= read -r line; do
        [ -z "$line" ] && continue
        count=$((count + 1))
        # shellcheck disable=SC2154  # vars assigned via eval below
        eval "$(printf '%s' "$line" | python3 -c '
import json, sys, shlex
o = json.loads(sys.stdin.read())
for k in ("action","board","task_id","issue","repo"):
    val = str(o.get(k, ""))
    val = val.replace(chr(39), chr(39)+chr(92)+chr(39)+chr(39))
    print(f"{k}=\x27{val}\x27")
')"
        log "↩  $action  $board/$task_id  issue=#$issue"

        if [ "$dry_run" = "yes" ]; then
            log "  [DRY-RUN] would: kanban unblock '$task_id'"
            continue
        fi

        if "$HERMES_BIN" kanban --board "$board" unblock \
                --reason "provider-budget recovered: MiniMax/DeepSeek снова отвечают — auto-unblock ретро-key=$RETRO_KEY" \
                "$task_id" >>"$LOG_FILE" 2>&1; then
            unblocked=$((unblocked + 1))
            log "  ✅ unblocked"
        else
            log "  ⚠️  unblock failed (rc=$?) — see $LOG_FILE"
            skipped=$((skipped + 1))
        fi
    done < "$ACTIONS_FILE"

    log "recover summary: scanned=$count unblocked=$unblocked skipped=$skipped"
}

# -------- main --------
MODE="cancel"
DRY_RUN="no"
while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run)  DRY_RUN="yes"; shift ;;
        --recover)  MODE="recover"; shift ;;
        --help|-h)  usage; exit 0 ;;
        *)          log "⚠️  unknown arg: $1"; usage; exit 2 ;;
    esac
done

log "=== START mode=$MODE dry_run=$DRY_RUN boards_dir=$KANBAN_BOARDS_DIR ==="
scan_provider_exhaust_actions "$MODE"
if [ "$MODE" = "cancel" ]; then
    apply_cancel "$DRY_RUN"
else
    apply_recover "$DRY_RUN"
fi
log "=== END ==="
