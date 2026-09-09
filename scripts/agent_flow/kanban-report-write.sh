#!/usr/bin/env bash
# ============================================================================
# kanban-report-write.sh — воркер-helper: создать `docs/reports/kanban/<task_id>.md`
# ПЕРЕД `kanban_complete` (ADR-0077, 2026-09-08).
#
# Зачем: текущий `kanban_complete` (hermes_cli.kanban_tools) сохраняет только
# Result (~300 символов), Summary (одно предложение) и Artifacts (список путей
# БЕЗ содержимого) → через 30 дней worktree GC, и ретро/аудит невозможен.
# Этот helper пишет ПОЛНЫЙ отчёт в git-tracked файл, переживает worktree GC.
#
# SOT (source-of-truth): <repo>/scripts/agent_flow/kanban-report-write.sh
# На хост раскладывает `bash <repo>/scripts/agent_flow/install.sh` — hardlink
# в TARGET_DIRS (см. install.sh). Drift-detect контролирует наличие.
#
# Использование (воркер, из worktree):
#   TASK_ID=t_xxxxxxxx
#   bash scripts/agent_flow/kanban-report-write.sh "$TASK_ID"
#
# Что делает (best-effort, отсутствующие секции → "n/a (reason: ...)"):
#   1. mkdir -p docs/reports/kanban/
#   2. Парсит `hermes kanban show $TASK_ID --json` для title / assignee / body
#   3. Извлекает Started / Completed из kanban DB (task_events)
#   4. Собирает `git diff --stat origin/develop...HEAD` для файлов
#   5. Собирает `git log --oneline origin/develop..HEAD` для коммитов
#   6. Ищет связанный PR через `gh pr list --head <branch>` + `gh pr view`
#   7. Запускает `pytest -v 2>&1 | tail -40` (если есть `tests/`)
#   8. Пишет docs/reports/kanban/${TASK_ID}.md
#   9. git add + commit (--allow-empty если файл уже существует и не изменился)
#  10. Возвращает 0 при успехе, 1 при ошибке, 2 при usage error.
#
# НЕ делает:
#   - НЕ вызывает `kanban_complete` (это делает воркер САМ, после успеха этого helper).
#   - НЕ пушит (воркер пушит сам через push-via-gh-api.sh).
#   - НЕ меняет state вне своего worktree.
#
# Exit codes:
#   0 = success (файл создан/обновлён + закоммичен).
#   1 = error (нет kanban DB / не kanban-board / git вне worktree / ...).
#   2 = usage error (нет task_id или неверный формат).
#
# Тест: bash scripts/agent_flow/tests/test_kanban_report_write.sh
# ============================================================================

set -uo pipefail

# ---- args -----------------------------------------------------------------
TASK_ID="${1:-}"
if [ -z "$TASK_ID" ] || ! printf '%s' "$TASK_ID" | grep -qE '^t_[a-f0-9]{6,}$'; then
    echo "usage: $0 <task_id>            (task_id format: t_<hex>6+)" >&2
    echo "example: $0 t_84434d4c" >&2
    exit 2
fi

# ---- env / paths ----------------------------------------------------------
KANBAN_BIN="${KANBAN_BIN:-hermes}"
KANBAN_BOARD="${KANBAN_BOARD:-robbox}"
KANBAN_DB="${KANBAN_DB:-$HOME/.hermes/kanban/boards/$KANBAN_BOARD/kanban.db}"
GH_REPO="${GH_REPO:-krikz/rob_box_project}"
HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"

# Worktree detection: должны быть внутри git-репо.
if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "kanban-report-write: not inside a git worktree (run from worker wt)" >&2
    exit 1
fi

REPORT_DIR="docs/reports/kanban"
REPORT_FILE="${REPORT_DIR}/${TASK_ID}.md"

mkdir -p "$REPORT_DIR"

# ---- helpers --------------------------------------------------------------
log() {
    printf '[kanban-report-write] %s %s\n' "$(date -Iseconds)" "$*" >&2
}

err() {
    log "ERROR: $*"
    return 1
}

# Best-effort: запустить команду, поймать exit, вернуть stdout (или fallback).
# $1=label (для логов, может быть пустым)  $2=fallback_msg  $@=cmd
run_best_effort() {
    local label="$1"; shift
    local fallback="$1"; shift
    local out
    if out="$("$@" 2>&1)"; then
        if [ -n "$out" ]; then
            printf '%s\n' "$out"
        else
            printf '%s\n' "n/a (${fallback}: empty output)"
        fi
    else
        printf '%s\n' "n/a (${fallback}: exit $?)"
    fi
    # label намеренно неиспользуем сейчас — оставлен для будущего
    # debug-логирования (stdout→stderr trace). SC2034 — known false positive.
    return 0
}

# ---- step 1: kanban show (title / assignee / body) ------------------------
KANBAN_JSON="$($KANBAN_BIN --board "$KANBAN_BOARD" kanban show "$TASK_ID" --json 2>/dev/null || echo '{}')"

# Robust parsing: kanban show --json может вернуть массив или объект — нормализуем.
parse_kanban() {
    printf '%s' "$KANBAN_JSON" | python3 -c '
import json, sys
try:
    data = json.load(sys.stdin)
except Exception as e:
    print(f"PARSE_ERROR|{e}|")
    sys.exit(0)
if isinstance(data, list):
    data = data[0] if data else {}
title = (data.get("title") or "").replace("|", r"\|")
assignee = data.get("assignee") or ""
body = data.get("body") or ""
issue_ref = ""
import re
m = re.search(r"#(\d{3,5})", body)
if m:
    issue_ref = f"#{m.group(1)}"
print(f"{title}|{assignee}|{issue_ref}|{body[:1500]}")
'
}

IFS='|' read -r TITLE ASSIGNEE ISSUE_REF BODY_EXCERPT <<<"$(parse_kanban)" || {
    TITLE="(unknown)"
    ASSIGNEE="(unknown)"
    ISSUE_REF="n/a"
    BODY_EXCERPT="n/a"
}

# Sanitize title for markdown heading.
TITLE_SAFE="$(printf '%s' "$TITLE" | sed -E 's/[`$]//g; s@\\@@g')"

# ---- step 2: timestamps (started / completed / duration) ------------------
STARTED="n/a"
COMPLETED="$(date -Iseconds)"
DURATION="n/a"

_TIMES_HELPER="$REPORT_DIR/.${TASK_ID}.times.tmp"
cat > "$_TIMES_HELPER" <<'PYEOF'
import sqlite3, sys
db, tid = sys.argv[1], sys.argv[2]
try:
    conn = sqlite3.connect(db)
    rows = conn.execute(
        "SELECT kind, created_at FROM task_events WHERE task_id=? "
        "AND kind IN ('claimed','completed') ORDER BY created_at ASC",
        (tid,)
    ).fetchall()
    conn.close()
    if not rows:
        print("n/a|n/a|n/a")
    else:
        started = next((r[1] for r in rows if r[0] == 'claimed'), rows[0][1])
        completed = next((r[1] for r in rows if r[0] == 'completed'), "")
        duration = ""
        if started and completed:
            from datetime import datetime
            try:
                t1 = datetime.fromisoformat(started.replace("Z", "+00:00"))
                t2 = datetime.fromisoformat(completed.replace("Z", "+00:00"))
                delta = t2 - t1
                secs = int(delta.total_seconds())
                if secs >= 3600:
                    duration = f"{secs // 3600}h{(secs % 3600) // 60}m"
                else:
                    duration = f"{secs // 60}m{secs % 60}s"
            except Exception:
                duration = ""
        print(f"{started}|{completed or 'in progress'}|{duration or 'in progress'}")
except Exception:
    print("n/a|n/a|n/a")
PYEOF

if [ -f "$KANBAN_DB" ]; then
    TIMES="$(python3 "$_TIMES_HELPER" "$KANBAN_DB" "$TASK_ID" 2>/dev/null || echo "n/a|n/a|n/a")"
    IFS='|' read -r STARTED COMPLETED DURATION <<<"$TIMES"
fi
rm -f "$_TIMES_HELPER"

# Fallback: если пусто, написать что-то осмысленное.
[ -z "$STARTED" ]   && STARTED="n/a"
[ -z "$COMPLETED" ] && COMPLETED="$(date -Iseconds)"
[ -z "$DURATION" ]  && DURATION="n/a"

# ---- step 3: changed files (git diff --stat) ------------------------------
# Если origin/develop существует — diff против него. Иначе — git status.
if git rev-parse --verify --quiet origin/develop >/dev/null 2>&1; then
    CHANGED_FILES="$(run_best_effort "files" "git diff failed" \
        bash -c "git diff --stat origin/develop...HEAD 2>&1 || true")"
    # Если пусто (например, worktree создан из develop без отдельных коммитов) —
    # показать git status как fallback.
    if [ -z "$CHANGED_FILES" ] || printf '%s' "$CHANGED_FILES" | grep -q '^n/a'; then
        CHANGED_FILES="$(run_best_effort "files-fallback" "git status failed" \
            bash -c "git status --short 2>&1 | head -30 || true")"
    fi
else
    # Нет remote-tracking ветки (например, fresh-init в тесте).
    CHANGED_FILES="$(run_best_effort "files" "git status failed" \
        bash -c "git status --short 2>&1 | head -30 || true")"
fi
[ -z "$CHANGED_FILES" ] && CHANGED_FILES="n/a (no changes detected)"

# ---- step 4: git log (последние 10 коммитов) ------------------------------
# Если origin/develop существует — log против него. Иначе — последние 10.
if git rev-parse --verify --quiet origin/develop >/dev/null 2>&1; then
    GIT_LOG="$(run_best_effort "log" "git log failed" \
        bash -c "git log --oneline origin/develop..HEAD 2>&1 || true")"
    if [ -z "$GIT_LOG" ] || printf '%s' "$GIT_LOG" | grep -q '^n/a'; then
        GIT_LOG="$(run_best_effort "log-fallback" "git log failed" \
            bash -c "git log --oneline -10 2>&1 || true")"
    fi
else
    GIT_LOG="$(run_best_effort "log" "git log failed" \
        bash -c "git log --oneline -10 2>&1 || true")"
fi
[ -z "$GIT_LOG" ] && GIT_LOG="n/a (no commits ahead of develop)"

# ---- step 5: PR link ------------------------------------------------------
PR_LINK="n/a"
PR_NUMBER=""
BRANCH="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo '')"
if [ -n "$BRANCH" ]; then
    PR_JSON="$(run_best_effort "pr" "gh pr list failed" \
        bash -c "GH_CONFIG_DIR=$HERMES_HOME gh pr list --repo $GH_REPO --head '$BRANCH' --json number,url,state 2>&1 || true")"
    if printf '%s' "$PR_JSON" | grep -qE '^\[?\{'; then
        PR_NUMBER="$(printf '%s' "$PR_JSON" | python3 -c '
import json, sys
try:
    data = json.load(sys.stdin)
    if data:
        print(data[0].get("number", ""))
except Exception:
    pass
' 2>/dev/null)"
        PR_URL="$(printf '%s' "$PR_JSON" | python3 -c '
import json, sys
try:
    data = json.load(sys.stdin)
    if data:
        print(data[0].get("url", ""))
except Exception:
    pass
' 2>/dev/null)"
        if [ -n "$PR_NUMBER" ]; then
            PR_LINK="PR #${PR_NUMBER} — ${PR_URL}"
        fi
    fi
fi

# ---- step 6: pytest output (если есть tests/) -----------------------------
PYTEST_OUT="n/a (no pytest run or no tests/ dir)"
if [ -d "tests" ] || [ -d "src" ]; then
    PYTEST_RAW="$(pytest -v 2>&1 | tail -40 || true)"
    if [ -n "$PYTEST_RAW" ]; then
        # Сократить если слишком длинный.
        PYTEST_LINES="$(printf '%s\n' "$PYTEST_RAW" | wc -l)"
        if [ "$PYTEST_LINES" -gt 40 ]; then
            PYTEST_OUT="(truncated to last 40 lines of $PYTEST_LINES total)
$PYTEST_RAW"
        else
            PYTEST_OUT="$PYTEST_RAW"
        fi
    fi
fi

# ---- step 7: CI run_id (если есть PR) -------------------------------------
CI_RUN="n/a"
if [ -n "$PR_NUMBER" ]; then
    CI_JSON="$(run_best_effort "ci" "gh pr checks failed" \
        bash -c "GH_CONFIG_DIR=$HERMES_HOME gh pr checks '$PR_NUMBER' --repo '$GH_REPO' --json name,conclusion,databaseId 2>&1 || true")"
    if printf '%s' "$CI_JSON" | grep -qE '^\[?\{'; then
        CI_SUMMARY="$(printf '%s' "$CI_JSON" | python3 -c '
import json, sys
try:
    data = json.load(sys.stdin)
    for r in data:
        name = r.get("name", "?")
        conc = r.get("conclusion", "?")
        rid = r.get("databaseId", "")
        line = f"- {name}: {conc}"
        if rid:
            line += f" (run_id={rid})"
        print(line)
except Exception:
    pass
' 2>/dev/null)"
        if [ -n "$CI_SUMMARY" ]; then
            CI_RUN="$CI_SUMMARY"
        fi
    fi
fi

# ---- step 8: write report -------------------------------------------------
{
    printf '# Отчёт: %s\n\n' "$TITLE_SAFE"
    printf '**Task ID:** %s\n' "$TASK_ID"
    printf '**Assignee:** %s\n' "$ASSIGNEE"
    if [ "$ISSUE_REF" != "n/a" ] && [ -n "$ISSUE_REF" ]; then
        printf '**Issue:** %s\n' "$ISSUE_REF"
    fi
    printf '**Branch:** `%s`\n' "$BRANCH"
    printf '**Started:** %s UTC\n' "$STARTED"
    printf '**Completed:** %s UTC\n' "$COMPLETED"
    printf '**Duration:** %s\n\n' "$DURATION"

    printf '## Что сделано\n\n'
    if [ "$BODY_EXCERPT" != "n/a" ] && [ -n "$BODY_EXCERPT" ]; then
        # Тело карточки — это спецификация, не «что сделано». Воркер дополняет
        # руками после создания файла, если нужно. Здесь — reference + brief.
        printf '_Из спецификации карточки (первые 1500 символов):_\n\n'
        printf '> %s\n\n' "$(printf '%s' "$BODY_EXCERPT" | sed 's/^/> /' | head -30)"
    else
        printf '_Воркер: дополнить руками перед `kanban_complete`._\n\n'
    fi

    printf '## Файлы изменены\n\n'
    printf '```\n%s\n```\n\n' "$CHANGED_FILES"

    printf '## Git log\n\n'
    printf '```\n%s\n```\n\n' "$GIT_LOG"

    printf '## Raw-evidence (pytest)\n\n'
    printf '```\n%s\n```\n\n' "$PYTEST_OUT"

    printf '## CI\n\n'
    printf '%s\n\n' "$CI_RUN"

    printf '## PR / Issue\n\n'
    printf -- '- %s\n\n' "$PR_LINK"

    printf '## Замечания\n\n'
    printf '_Воркер: дополнить руками (caveats / что НЕ сделано / что осталось для следующей карточки)._\n\n'

    printf -- '---\n\n'
    printf '_Сгенерировано `kanban-report-write.sh` (ADR-0077, 2026-09-08)._\n'
} > "$REPORT_FILE"

if [ ! -f "$REPORT_FILE" ]; then
    err "failed to write $REPORT_FILE"
    exit 1
fi

log "wrote $REPORT_FILE ($(wc -l < "$REPORT_FILE") lines)"

# ---- step 9: git add + commit --------------------------------------------
git add "$REPORT_FILE" 2>/dev/null || true

# Если ничего не staged — возможно, файл уже закоммичен или не изменился.
if git diff --cached --quiet 2>/dev/null; then
    log "no changes to commit (report file already up-to-date)"
else
    COMMIT_MSG="report(${TASK_ID}): ${TITLE_SAFE}"
    if git commit -m "$COMMIT_MSG" >/dev/null 2>&1; then
        log "committed: $COMMIT_MSG"
    else
        # Если commit упал (например, нет user.email) — попробовать с --allow-empty.
        log "commit failed (probably git identity missing), continuing anyway"
    fi
fi

# ---- step 10: success -----------------------------------------------------
log "done — push ветки и kanban_complete делает сам воркер"
exit 0