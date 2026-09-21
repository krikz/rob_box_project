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
HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
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

# -------- helpers --------

# Один инстанс. Если уже идёт — exit 0 (тик пропускаем, no-agent cron).
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
    log "⏳ another instance holds $LOCK_FILE — skip"
    exit 0
fi

usage() {
    cat <<EOF
Usage: $SCRIPT_NAME [--dry-run] [--recover] [--help]

Modes:
  (default)   scan kanban boards, detect provider-exhaust signature in
              task_runs.summary of non-blocked tasks, then:
                1) block (kind=capability, reason='$BLOCK_REASON')
                2) post sentinel-marked comment on linked issue ref
              IDEMPOTENT: re-runs are no-op for already-blocked tasks with
              existing sentinel comment.
  --recover   companion: scan blocked tasks with sentinel comment +
              block_kind=capability + prov-alive signal → unblock back to ready.
  --dry-run   same scan as default, but print what WOULD be done; no side effects.
  --help      this message.

Env knobs:
  HERMES_BIN           hermes CLI (default: $HERMES_BIN)
  KANBAN_BOARDS_DIR    boards dir (default: $KANBAN_BOARDS_DIR)
  LOCK_FILE / LOG_FILE override defaults
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
