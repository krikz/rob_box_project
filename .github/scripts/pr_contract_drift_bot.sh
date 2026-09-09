#!/bin/bash
# ============================================================================
# pr_contract_drift_bot.sh — ADR-0044 (ретро t_527e1231 → t_58c69473)
#
# Классифицирует красные чеки OPEN PR и оставляет ОДИН информационный
# комментарий: «CI failing: rebase candidate (X jobs) vs contract-drift inside
# PR (Y jobs)» + разбивка по категориям.
#
# Контракт (инварианты ADR-0044 §3):
#   - НЕ ставит/снимает метки, НЕ закрывает issues/PR, НЕ создаёт карточек;
#   - НЕ комментирует CONFLICTING/DIRTY PR (там rebase корректен, merge-gate
#     уже пишет инструкцию);
#   - НЕ комментирует при классификации unknown (fail-open);
#   - НЕ комментирует, если все красные чеки — rebase_candidate (это ровно тот
#     случай, где rebase-совет merge-gate верен);
#   - дедуп по маркеру: существующий комментарий бота в окне DEDUP_HOURS
#     ОБНОВЛЯЕТСЯ (PATCH), а не дублируется.
#
# Env:
#   GH_REPO      (обяз.) owner/repo
#   PR_NUMBER    (обяз.) номер PR
#   DEDUP_HOURS  (опц., default 6) окно поиска своего комментария
#   DRY_RUN      (опц.) true → только печать, без вызовов записи
#
# Exit: 0 всегда при штатной работе (в т.ч. «нечего делать»); 1 — только
# ошибка конфигурации (нет GH_REPO/PR_NUMBER).
#
# Локальный прогон:
#   GH_REPO=krikz/rob_box_project PR_NUMBER=1857 DRY_RUN=true \
#     bash .github/scripts/pr_contract_drift_bot.sh
# ============================================================================
set -euo pipefail

MARKER='<!-- contract-drift-bot -->'
DEDUP_HOURS="${DEDUP_HOURS:-6}"
DRY_RUN="${DRY_RUN:-false}"

log() { printf '[contract-drift-bot] %s\n' "$*" >&2; }

if [ -z "${GH_REPO:-}" ] || [ -z "${PR_NUMBER:-}" ]; then
    log "FATAL: GH_REPO and PR_NUMBER are required"
    exit 1
fi

# --- 1. Состояние PR --------------------------------------------------------
_pr_json="$(gh pr view "$PR_NUMBER" --repo "$GH_REPO" \
    --json state,mergeStateStatus,headRefName,statusCheckRollup 2>/dev/null || echo '')"
if [ -z "$_pr_json" ]; then
    log "PR #${PR_NUMBER}: cannot read PR state — skip (fail-open)"
    exit 0
fi

_state="$(printf '%s' "$_pr_json" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("state") or "")')"
_merge_state="$(printf '%s' "$_pr_json" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("mergeStateStatus") or "")')"
_head="$(printf '%s' "$_pr_json" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("headRefName") or "")')"

if [ "$_state" != "OPEN" ]; then
    log "PR #${PR_NUMBER}: state=${_state} — skip (bot only comments OPEN PRs)"
    exit 0
fi
case "$_merge_state" in
    DIRTY|CONFLICTING)
        log "PR #${PR_NUMBER}: mergeStateStatus=${_merge_state} — real conflict, rebase is the right answer, skip (ADR-0044 §3)"
        exit 0
        ;;
esac

# --- 2. Классификация красных чеков ----------------------------------------
# Печатает: <class>\t<drift_count>\t<rebase_count>\n затем markdown-строки.
_classified="$(printf '%s' "$_pr_json" | python3 -c '
import json, re, sys

DRIFT = re.compile(r"(unit|lint|build|pytest|mypy|ruff|flake8|black|coverage|test summary|code quality|dockerfile|yaml)", re.I)
REBASE = re.compile(r"(integration|e2e|deploy|docker build|release|smoke)", re.I)

try:
    d = json.load(sys.stdin)
except Exception:
    print("unknown\t0\t0")
    raise SystemExit(0)

rollup = d.get("statusCheckRollup") or []
failed = [c for c in rollup
          if isinstance(c, dict)
          and str(c.get("conclusion") or "").upper() in ("FAILURE", "TIMED_OUT", "CANCELLED")]
if not failed:
    print("unknown\t0\t0")
    raise SystemExit(0)

drift, rebase, other = [], [], []
for c in failed:
    name = str(c.get("name") or "?")
    url = c.get("detailsUrl") or c.get("targetUrl") or c.get("html_url") or ""
    if REBASE.search(name):
        rebase.append((name, url))
    elif DRIFT.search(name):
        drift.append((name, url))
    else:
        other.append((name, url))

if drift:
    kind = "contract_drift"
elif rebase:
    kind = "rebase_candidate"
else:
    kind = "unknown"

print("{}\t{}\t{}".format(kind, len(drift), len(rebase) + len(other)))
print("__DRIFT__")
for n, u in drift:
    print("- **{}** — {}".format(n, u))
print("__REBASE__")
for n, u in rebase + other:
    print("- **{}** — {}".format(n, u))
' 2>/dev/null || printf 'unknown\t0\t0\n')"

_class="$(printf '%s\n' "$_classified" | head -n1 | cut -f1)"
_drift_n="$(printf '%s\n' "$_classified" | head -n1 | cut -f2)"
_rebase_n="$(printf '%s\n' "$_classified" | head -n1 | cut -f3)"
_drift_md="$(printf '%s\n' "$_classified" | sed -n '/^__DRIFT__$/,/^__REBASE__$/p' | sed '1d;$d')"
_rebase_md="$(printf '%s\n' "$_classified" | sed -n '/^__REBASE__$/,$p' | sed '1d')"

log "PR #${PR_NUMBER}: class=${_class} drift_jobs=${_drift_n} rebase_jobs=${_rebase_n} head=${_head}"

if [ "$_class" != "contract_drift" ]; then
    log "PR #${PR_NUMBER}: class=${_class} — no comment (fail-open / rebase advice already covered by merge-gate)"
    exit 0
fi

[ -n "$_drift_md" ] || _drift_md="- (не смог достать имена jobs; см. вкладку Checks)"
[ -n "$_rebase_md" ] || _rebase_md="- (нет)"

# --- 3. Тело комментария ---------------------------------------------------
_body="${MARKER}
## 🐛 CI failing: rebase candidate (${_rebase_n} jobs) vs contract-drift inside PR (${_drift_n} jobs)

PR #${PR_NUMBER} (\`${_head}\`): \`mergeStateStatus=${_merge_state}\`, красные чеки есть, и **как минимум ${_drift_n}** из них — unit/lint/build. Это признак **contract drift ВНУТРИ этого PR** (реализация и тест разошлись в одном диффе), а не отставания от develop. **Rebase такой PR НЕ починит.**

### Contract-drift (лечить код/тест в этой ветке)
${_drift_md}

### Rebase-candidate (может помочь develop-фикс)
${_rebase_md}

### Порядок работы
1. Открой contract-drift job по ссылке, прочитай assertion diff и имя упавшего теста.
2. Если реализация и тест расходятся внутри PR — правь их в \`${_head}\`, **без rebase**.
3. Rebase делай только если падение объясняется фиксом, который уже есть в develop.
4. Работай в той же ветке — новый PR не создавай.

_Автоматический комментарий (ADR-0044, ретро t_527e1231). Бот только информирует: метки не ставит, issues не закрывает, карточек не создаёт._"

# --- 4. Дедуп: свой комментарий в окне DEDUP_HOURS → PATCH, иначе POST -----
_since="$(date -u -d "${DEDUP_HOURS} hours ago" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
_existing_id="$(gh api "repos/${GH_REPO}/issues/${PR_NUMBER}/comments?since=${_since}&per_page=100" \
    --jq "[.[] | select(.body | contains(\"${MARKER}\"))] | last | .id // \"\"" 2>/dev/null || echo '')"

if [ "$DRY_RUN" = "true" ]; then
    log "DRY-RUN: would $( [ -n "$_existing_id" ] && echo "PATCH comment ${_existing_id}" || echo 'POST new comment' ) on PR #${PR_NUMBER}"
    printf '%s\n' "$_body"
    exit 0
fi

if [ -n "$_existing_id" ]; then
    if gh api -X PATCH "repos/${GH_REPO}/issues/comments/${_existing_id}" \
        -f body="$_body" >/dev/null 2>&1; then
        log "PR #${PR_NUMBER}: updated existing bot comment ${_existing_id} (dedup ${DEDUP_HOURS}h)"
    else
        log "PR #${PR_NUMBER}: WARNING PATCH comment ${_existing_id} failed"
    fi
    exit 0
fi

if gh pr comment "$PR_NUMBER" --repo "$GH_REPO" --body "$_body" >/dev/null 2>&1; then
    log "PR #${PR_NUMBER}: posted contract-drift comment (drift=${_drift_n}, rebase=${_rebase_n})"
else
    log "PR #${PR_NUMBER}: WARNING posting comment failed"
fi
