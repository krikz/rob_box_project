#!/bin/bash
# ============================================================================
# kanban-report-write.sh — генератор отчёта воркера в
#   <worktree>/docs/reports/kanban/<task_id>.md
#
# Issue #2159: «воркеры должны сохранять полные отчёты в
# docs/reports/kanban/<task_id>.md при kanban_complete — иначе после
# архивации нечего ревьюить».
#
# Использование (воркер вызывает ПЕРЕД kanban_complete):
#   bash scripts/agent_flow/kanban-report-write.sh <task_id> [--title "<title>"] [--assignee <role>]
#
# Что делает:
#   1. Создаёт docs/reports/kanban/ в текущем worktree (если нет).
#   2. Собирает git log --oneline origin/develop..HEAD, diff --stat,
#      branch name, started_at (от created события из task metadata, если
#      доступно — иначе ставит текущее время), assignee.
#   3. Пишет заготовку отчёта по scripts/agent_flow/report_template.md
#      с подставленными данными. Воркер дописывает свободные секции руками
#      (что сделано, skill results, caveats).
#   4. Не вызывает git add / commit / push — это делает воркер, чтобы
#      можно было сначала отредактировать.
#
# Опции:
#   -t, --title <title>      — task title (иначе берёт из $HERMES_KANBAN_TITLE
#                              или ставит "<unknown>")
#   -a, --assignee <role>    — assignee (backend/devops/...), иначе
#                              $HERMES_KANBAN_ASSIGNEE
#   -p, --pr <pr_number>     — номер PR (иначе пытается найти через
#                              `gh pr list --head <branch>`)
#   -i, --issue <issue_num>  — номер issue
#   -o, --output <path>      — куда писать (по умолчанию
#                              <worktree>/docs/reports/kanban/<task_id>.md)
#   -b, --base <ref>         — base ref (default origin/develop)
#   -h, --help               — эта справка
#
# Exit codes:
#   0 — OK (отчёт создан)
#   1 — usage error / нет git
#   2 — worktree не определён (cwd не внутри git repo)
#
# Связанные:
#   - scripts/agent_flow/report_template.md (шаблон)
#   - issue #2159 (этот скрипт — фича)
#   - issue #2162 (skills в body карточки)
#   - ADR-0077 (kanban worker report file)
# ============================================================================
set -euo pipefail

# --- args --------------------------------------------------------------------
TASK_ID=""
TITLE=""
ASSIGNEE="${HERMES_KANBAN_ASSIGNEE:-devops}"
PR_NUM=""
ISSUE_NUM=""
BASE_REF="origin/develop"
OUTPUT=""

_usage() {
    sed -n '2,40p' "$0" | sed 's/^# \{0,1\}//'
}

while [ $# -gt 0 ]; do
    case "$1" in
        -h|--help)         _usage; exit 0;;
        -t|--title)        TITLE="$2"; shift 2;;
        -a|--assignee)     ASSIGNEE="$2"; shift 2;;
        -p|--pr)           PR_NUM="$2"; shift 2;;
        -i|--issue)        ISSUE_NUM="$2"; shift 2;;
        -o|--output)       OUTPUT="$2"; shift 2;;
        -b|--base)         BASE_REF="$2"; shift 2;;
        -*)                echo "unknown option: $1" >&2; _usage; exit 1;;
        *)                 TASK_ID="$1"; shift;;
    esac
done

if [ -z "$TASK_ID" ]; then
    _usage
    exit 1
fi

# --- worktree / repo ---------------------------------------------------------
if ! git rev-parse --git-dir >/dev/null 2>&1; then
    echo "ERROR: cwd is not inside a git worktree" >&2
    exit 2
fi

WORKTREE="$(git rev-parse --show-toplevel)"
BRANCH="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo 'detached')"

if [ -z "$OUTPUT" ]; then
    OUTPUT="${WORKTREE}/docs/reports/kanban/${TASK_ID}.md"
fi

mkdir -p "$(dirname "$OUTPUT")"

# --- данные ------------------------------------------------------------------
TITLE="${TITLE:-${HERMES_KANBAN_TITLE:-<unknown>}}"
NOW="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

# PR — пытаемся найти автоматически, если не задан
if [ -z "$PR_NUM" ] && command -v gh >/dev/null 2>&1; then
    PR_NUM="$(gh pr list --head "$BRANCH" --json number --jq '.[0].number // empty' 2>/dev/null || true)"
fi

PR_LINE=""
if [ -n "$PR_NUM" ]; then
    PR_LINE="**PR:** #${PR_NUM}  — https://github.com/krikz/rob_box_project/pull/${PR_NUM}"
else
    PR_LINE="**PR:** _не найден (запустить \`gh pr list --head \"$BRANCH\"\` руками)_"
fi

ISSUE_LINE=""
if [ -n "$ISSUE_NUM" ]; then
    ISSUE_LINE="**Issue:** #${ISSUE_NUM}  — https://github.com/krikz/rob_box_project/issues/${ISSUE_NUM}"
else
    ISSUE_LINE="**Issue:** _не указан_"
fi

# git log / diff — если base недоступен, не падаем
if git rev-parse --verify "${BASE_REF}" >/dev/null 2>&1; then
    DIFF_STAT="$(git diff --stat "${BASE_REF}..HEAD" 2>/dev/null || true)"
    GIT_LOG="$(git log --oneline "${BASE_REF}..HEAD" 2>/dev/null || true)"
    COMMIT_RANGE="${BASE_REF}..HEAD"
else
    DIFF_STAT="$(git diff --stat HEAD~1..HEAD 2>/dev/null || git diff --stat 2>/dev/null || true)"
    GIT_LOG="$(git log --oneline -10 2>/dev/null || true)"
    COMMIT_RANGE="HEAD~1..HEAD (fallback: base ${BASE_REF} недоступен)"
fi

if [ -z "$DIFF_STAT" ]; then
    DIFF_STAT="_нет изменений против ${COMMIT_RANGE}_"
fi
if [ -z "$GIT_LOG" ]; then
    GIT_LOG="_нет коммитов против ${COMMIT_RANGE}_"
fi

# Пометка про fallback (для аудита после архивации) — печатаем в diff/log,
# только если base реально был недоступен.
FALLBACK_NOTE=""
if ! git rev-parse --verify "${BASE_REF}" >/dev/null 2>&1; then
    FALLBACK_NOTE="
> ⚠️ BASE_REF \`${BASE_REF}\` недоступен — diff/log собраны из \`HEAD~1..HEAD\`.
"
fi

# --- запись ------------------------------------------------------------------
{
    cat <<EOF
# Отчёт: ${TITLE}

**Task ID:** ${TASK_ID}
**Assignee:** ${ASSIGNEE}
${ISSUE_LINE}
${PR_LINE}
**Branch:** \`${BRANCH}\`
**Started:** _<поставить из события created в task metadata>_
**Completed:** ${NOW}
**Duration:** _<вычислить руками>_

## Что сделано

_Воркер: дополнить bullet-list перед \`kanban_complete\`._

- Пункт 1
- Пункт 2
- Пункт 3

## Файлы изменены

\`\`\`
${DIFF_STAT}
\`\`\`
${FALLBACK_NOTE}
## Git log

\`\`\`
${GIT_LOG}
\`\`\`

## Raw-evidence (pytest / CI / логи)

### pytest

\`\`\`
_Воркер: вставить \`pytest -v\` (или N/A если тесты не применимы)_
\`\`\`

### CI (\`gh pr checks\` / \`gh run view\`)

- _Воркер: вставить конкретные run-id и статусы чек'ов_

### Логи / e2e (если применимо)

\`\`\`
_Воркер: 30-50 строк docker logs / e2e output (или N/A)_
\`\`\`

## Skill results (ВАЖНО — что дал каждый skill)

Воркер ОБЯЗАН перечислить skills из секции \`## Skills\` в body карточки
(issue #2162) и для каждого указать: что делал, что нашёл, что применил.

### verification-before-completion

- [ ] pytest -v: N passed, 0 failed (raw вывод выше)
- [ ] \`gh pr checks\`: все required зелёные
- [ ] \`git status\`: чисто (или только ожидаемые untracked)
- [ ] честный FAIL лучше красивого PASS (ADR-0018)

### code-review

_Воркер: вставить фидбек OpenAI agent / ручной review + что применено._

### writing-for-agents

_Воркер: что в body карточки помогло следующему воркеру; что добавить / убрать._

### senior-devops

_Воркер: CI/CD best practices применённые здесь; что изменилось в pipelines / configs._

## PR / Issue ссылки

EOF
    if [ -n "$PR_NUM" ]; then
        echo "- PR #${PR_NUM} — https://github.com/krikz/rob_box_project/pull/${PR_NUM}"
    fi
    if [ -n "$ISSUE_NUM" ]; then
        echo "- Issue #${ISSUE_NUM} — https://github.com/krikz/rob_box_project/issues/${ISSUE_NUM}"
    fi
    cat <<'EOF'

## Замечания / Caveats

- _Что НЕ сделано (если осталось для follow-up карточки)_
- _Известные проблемы / отложенные TODO_
- _Какие ADR/доки появились по итогам работы_

---

> Сгенерировано `scripts/agent_flow/kanban-report-write.sh` в ${NOW}.
> Шаблон: `scripts/agent_flow/report_template.md` (issue #2159, ADR-0077).
EOF
} > "$OUTPUT"

echo "[kanban-report-write] wrote: $OUTPUT"
echo "[kanban-report-write] NEXT: отредактируй свободные секции, затем:"
echo "  git add $OUTPUT"
echo "  git commit -m \"report(${TASK_ID}): <title>\""
echo "  git push"
echo "  # и только ПОСЛЕ push: kanban_complete"
