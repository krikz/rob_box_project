#!/bin/bash
# ============================================================================
# agent-flow-e2e-drift-watchdog.sh — observability for PR↔issue e2e-done drift
#
# Ретро 19.08 t_5cde0bc1 (PR #1398 / issue #1392): issue может вернуться в
# ротацию (needs-e2e), но канонический PR висит с e2e-done от предыдущего
# раунда → drift. Merge-gate теперь умеет reconcile (ретро 19.08), но пока
# он не сработал — нам нужна метрика «сколько PR в drift-е», чтобы:
#   1) Подсветить drift в дашборде / алерте (когда > 0, присылать watchdog-уведомление).
#   2) Собрать историческую статистику e2e_drift_minutes (мин с последнего
#      коммита PR до появления drift-а) для ретро-аналитики.
#
# Скрипт НЕ меняет labels (только наблюдение) — reconcile делает merge-gate
# в основном цикле. Watchdog безопасно запускать каждые 5–15 минут.
#
# Выходы:
#   - Лог: ${LOG_FILE:-/var/log/agent-flow-e2e-drift-watchdog.log} (TSV).
#   - Stderr: structured summary (для cron delivery).
#   - Exit 0 если drift не обнаружен, exit 1 если есть drift (для cron-алертов).
#
# ENV:
#   GH_REPO          — owner/repo (default krikz/rob_box_project)
#   HERMES_HOME      — база для лога (default /home/builder/.hermes)
#   LOG_FILE         — путь к логу (default ${HERMES_HOME}/logs/agent-flow-e2e-drift-watchdog.log)
#   DRIFT_THRESHOLD  — мин порог алерта (default 30m, для cron-нотификации)
#
# Использование:
#   bash scripts/agent_flow/agent-flow-e2e-drift-watchdog.sh
#   DRIFT_THRESHOLD=60 LOG_FILE=/tmp/drift.log bash scripts/agent_flow/agent-flow-e2e-drift-watchdog.sh
#
# Pitfalls:
#   - Не запускать одновременно с merge-gate: оба читают timeline issues,
#     гонка меток → ложный drift. CRON_LOCKFILE решает (flock).
#   - В dry-run (WATCHDOG_DRY_RUN=true) только логирует, не алертит.
# ============================================================================
set -euo pipefail

GH_REPO="${GH_REPO:-krikz/rob_box_project}"
# Ретро 30.08 (дедуп процессного слоя): дефолт был /var/log/... — каталог
# принадлежит root, `>> "$LOG_FILE"` под `set -euo pipefail` роняет тик с
# «Permission denied» ещё до вывода summary. Пишем туда же, куда остальные
# вотчдоги, — под HERMES_HOME.
HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
LOG_FILE="${LOG_FILE:-${HERMES_HOME}/logs/agent-flow-e2e-drift-watchdog.log}"
DRIFT_THRESHOLD="${DRIFT_THRESHOLD:-30}"
WATCHDOG_DRY_RUN="${WATCHDOG_DRY_RUN:-false}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-e2e-drift-watchdog.lock}"

# Sentinel: не запускаться параллельно с merge-gate (оба пишут в issues).
exec 9>"$LOCK_FILE" || true
if ! flock -n 9; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] watchdog: another instance running — skip" >&2
    exit 0
fi

mkdir -p "$(dirname "$LOG_FILE")" 2>/dev/null || true

# Проверка gh auth (G2).
if ! gh auth status >/dev/null 2>&1; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] watchdog: gh auth failed — exit 1" >&2
    exit 1
fi

# 1) Найти все OPEN PR с меткой e2e-done.
# gh pr list --state all --label e2e-done — ВАЖНО: state=all, чтобы не пропустить
# только что merged PR (drift может быть и там, если krikz вручную вернул issue).
# ВАЖНО: --json headRefName обязательно — иначе поле пустое, фильтр ниже
# оставит PR без анализа (drift пропущен).
_pr_done_json="$(gh pr list --repo "$GH_REPO" --state all \
    --label e2e-done \
    --json number,title,headRefName,state 2>/dev/null || echo '[]')"

if [ -z "$_pr_done_json" ] || [ "$_pr_done_json" = "[]" ]; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] watchdog: no PR with e2e-done label — exit 0" >&2
    exit 0
fi

# 2) Для каждого PR — найти issue (по номеру в headRefName или title) и
# проверить, что issue НЕ в needs-e2e. Если PR:e2e-done + issue:needs-e2e
# → drift (issue в ротации, но PR-side «готово» — Шифу не видит PR в
# ревью-очереди; ретро 19.08 t_5cde0bc1). Reconcile делает merge-gate в
# основном цикле, watchdog только наблюдает.
_drift_count=0
_drift_total_minutes=0
_drift_max_minutes=0
_drift_records=()

# Простой парсинг через python — стабильнее grep'а по JSON.
while IFS=$'\t' read -r pr_number title head_ref pr_state; do
    # Извлечь issue number из title (формат: "feat(voice #NNNN): ...").
    issue_num="$(printf '%s' "$title" | grep -oE '#[0-9]+' | head -n1 | tr -d '#' || true)"
    [ -n "$issue_num" ] || continue

    # Получить метки issue.
    issue_labels_csv="$(gh issue view "$issue_num" --repo "$GH_REPO" --json labels \
        --jq '[.labels[].name] | join(",")' 2>/dev/null || echo '')"
    issue_labels_norm="$(printf '%s' "$issue_labels_csv" | tr '[:upper:]' '[:lower:]')"

    # DRIFT: PR:e2e-done + issue:needs-e2e (issue в ротации, PR-side stale).
    # Согласованные состояния (skip):
    #   - issue:e2e-done (только что прогнали, оба свежие)
    #   - issue закрыт (state=CLOSED) — PR может ещё висеть с e2e-done до
    #     cleanup'а в merge-gate, но это нормальная post-merge фаза
    #   - PR не OPEN (merged) — reconcile уже неактуален
    case ",${issue_labels_norm}," in
        *,e2e-done,*) continue ;;  # согласованно
    esac
    case ",${issue_labels_norm}," in
        *,needs-e2e,*) ;;  # ← это и есть DRIFT
        *) continue ;;     # ни e2e-done, ни needs-e2e — нет ни rotation, ни свежего прогона
    esac

    # Дополнительная проверка: PR должен быть OPEN. Если MERGED — это
    # post-merge состояние, reconcile не нужен (merge-gate закроет issue).
    if [ "$pr_state" != "OPEN" ]; then continue; fi

    # Метрика: минут с последнего коммита PR.
    last_commit_iso="$(gh api "repos/${GH_REPO}/pulls/${pr_number}/commits?per_page=1" \
        --jq '.[0].commit.committer.date // empty' 2>/dev/null || echo '')"
    drift_minutes=0
    if [ -n "$last_commit_iso" ] && [ "$last_commit_iso" != "null" ]; then
        now_s="$(date -u +%s)"
        commit_s="$(date -u -d "$last_commit_iso" +%s 2>/dev/null || echo 0)"
        if [ "$commit_s" -gt 0 ] 2>/dev/null; then
            drift_minutes=$(( (now_s - commit_s) / 60 ))
        fi
    fi

    _drift_count=$(( _drift_count + 1 ))
    _drift_total_minutes=$(( _drift_total_minutes + drift_minutes ))
    if [ "$drift_minutes" -gt "$_drift_max_minutes" ] 2>/dev/null; then
        _drift_max_minutes="$drift_minutes"
    fi
    _drift_records+=("$(printf '%s\tPR #%s\tissue #%s\thead=%s\tstate=%s\tdrift=%sm' \
        "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$pr_number" "$issue_num" "$head_ref" "$pr_state" "$drift_minutes")")

done < <(printf '%s' "$_pr_done_json" | python3 -c '
import json, sys
try:
    data = json.load(sys.stdin)
except Exception:
    sys.exit(0)
for pr in data:
    print("\t".join([
        str(pr.get("number","")),
        pr.get("title","")[:80],
        pr.get("headRefName",""),
        pr.get("state",""),
    ]))
')

# 3) Записать в лог (TSV).
if [ "$_drift_count" -gt 0 ]; then
    {
        printf '# e2e-drift snapshot %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
        printf 'timestamp\tpr\tissue\thead\tstate\tdrift_minutes\n'
        for r in "${_drift_records[@]}"; do
            printf '%s\n' "$r"
        done
        printf '# total_drift=%s avg_drift=%sm max_drift=%sm threshold=%sm\n' \
            "$_drift_count" \
            "$(( _drift_count > 0 ? _drift_total_minutes / _drift_count : 0 ))" \
            "$_drift_max_minutes" \
            "$DRIFT_THRESHOLD"
    } >> "$LOG_FILE"
fi

# 4) Stderr-summary + exit-code для cron.
if [ "$_drift_count" -gt 0 ]; then
    echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] watchdog: 🚨 drift detected — count=${_drift_count} max_drift=${_drift_max_minutes}m threshold=${DRIFT_THRESHOLD}m log=${LOG_FILE}" >&2
    if [ "$WATCHDOG_DRY_RUN" != "true" ]; then
        # В dry-run (юзер тестирует) НЕ алертим (exit 0).
        if [ "$_drift_max_minutes" -gt "$DRIFT_THRESHOLD" ] 2>/dev/null; then
            exit 1  # cron-алерт
        fi
    fi
    exit 0  # drift есть, но ниже порога — info
fi

echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] watchdog: ✓ no drift (scanned $(printf '%s' "$_pr_done_json" | grep -oE '"number":' | wc -l) PRs with e2e-done)" >&2
exit 0
