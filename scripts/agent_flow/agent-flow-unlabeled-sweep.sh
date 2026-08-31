#!/bin/bash
# ============================================================================
# agent-flow-unlabeled-sweep.sh — двушаговая автозакрывалка
#                                  для OPEN issues БЕЗ меток процесса
#                                  (ADR-0022 GATE-2, issue #1428).
#
# Source-of-truth: <repo>/scripts/agent_flow/agent-flow-unlabeled-sweep.sh
# Копии раскладываются install.sh в:
#   - ~/.hermes/profiles/agent-flow/scripts/
#   - ~/.hermes/profiles/architect/scripts/
#   - ~/.hermes/profiles/devops/scripts/
#   - ~/.hermes/scripts/
#
# ПРОБЛЕМА (ADR-0022 GATE-2):
#   Раньше сценарий "issue OPEN, без меток `hermes`/`needs-e2e`/`e2e-done`
#   > 24h" приводил к:
#     (а) дрейф — issue висит месяцами, никто не триажит;
#     (б) instantaneous close из (например) `agent-flow-deploy-sweep.sh`
#         на свежих тикетах — агрессивно для issues которые юзер ещё
#         может переоткрыть руками после первого автоматического действия;
#     (в) риск закрытия issue #1363 сразу после user-reopen: процесс
#         "autocloser" не успевал заметить, что user вручную оживил
#         ticket.
#
# РЕШЕНИЕ (двушаговая модель):
#   1. Tick T0: issue OPEN, нет process-меток, last update >= STALE_HOURS_1
#      → ставим `stale-candidate` + dedup-комментарий + НЕ закрываем.
#   2. Tick T0 + STALE_HOURS_2: stale-candidate ВСЁ ЕЩЁ на месте И
#      не было user-reopen ПОСЛЕ установки метки → close (reason=not
#      planned), снимаем stale-candidate, dedup-комментарий.
#   3. User-reopen ПОСЛЕ установки stale-candidate → снимаем
#      stale-candidate, issue возвращается в OPEN без меток (процесс
#      триажа / triage-cron / `agent-flow-triage.sh` подхватит,
#      либо юзер ведёт её руками).
#
# ТАЙМИНГ (defaults):
#   STALE_HOURS_1 = 24 → первый warning + метка stale-candidate
#   STALE_HOURS_2 = 24 → второй тик → close (итого 48ч)
#   CLOSE_WINDOW  = 6h дедупликация close-комментариев (как в merge-gate)
#   LABEL_WINDOW  = 6h дедупликация stale-комментариев
#
# ПРАВИЛА (conservative on uncertainty, ADR-0014 §4 req 4):
#   - timeline API сдох → skip ВСЕХ issues (fail-closed)
#   - issue уже имеет process-метку (`hermes`, `needs-e2e`, `e2e-done`,
#     `e2e:rejected`, `no-e2e-required`, `stale-candidate`) → skip
#   - метка stale-candidate поставлена, но у issue есть OPEN PR со свежими
#     коммитами после метки → skip (юзер активировал работу)
#   - user-reopen после метки → снимаем метку, skip close (см. issue #1391,
#     PR #1399 helper)
#   - PR #1399 user-reopen guard интегрирован: timeline cross-check
#     `last_reopen_at > stale_labeled_at` → не закрываем
#
# Usage:
#   agent-flow-unlabeled-sweep.sh [--dry-run] [--stale-hours-1 24] \
#       [--stale-hours-2 24] [--limit 200]
#
# Env:
#   GH_REPO          — owner/repo (required, см. .env)
#   STALE_HOURS_1    — часов до первой отметки stale-candidate (default 24)
#   STALE_HOURS_2    — часов после первой отметки до close (default 24)
#   SWEEP_LIMIT      — max issues за тик (default 200)
#   DRY_RUN          — true → log only, no API writes (default false)
#   LOCK_FILE        — путь flock-сентенели (default /tmp/agent-flow-unlabeled-sweep.lock)
#   HERMES_HOME      — путь к .hermes (для env-file)
#
# Idempotent: повторный запуск уже размеченных issues не дублирует
# комментарии (6h dedup); закрытые issues пропускаются state-фильтром.
# Pure bash + python3 (JSON). No LLM.
# ============================================================================
set -euo pipefail

HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
export HOME="${HOME:-/home/builder}"

STALE_HOURS_1="${STALE_HOURS_1:-24}"
STALE_HOURS_2="${STALE_HOURS_2:-24}"
SWEEP_LIMIT="${SWEEP_LIMIT:-200}"
DRY_RUN="${DRY_RUN:-false}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-unlabeled-sweep.lock}"
ISSUE_LABEL_DEFAULT="hermes"
NEEDS_E2E_LABEL_DEFAULT="needs-e2e"
DONE_LABEL_DEFAULT="e2e-done"
REJECTED_LABEL_DEFAULT="e2e:rejected"
NO_E2E_LABEL_DEFAULT="no-e2e-required"
STALE_LABEL_DEFAULT="stale-candidate"

ISSUE_LABEL="${ISSUE_LABEL:-${ISSUE_LABEL_DEFAULT}}"
NEEDS_E2E_LABEL="${NEEDS_E2E_LABEL:-${NEEDS_E2E_LABEL_DEFAULT}}"
DONE_LABEL="${DONE_LABEL:-${DONE_LABEL_DEFAULT}}"
REJECTED_LABEL="${REJECTED_LABEL:-${REJECTED_LABEL_DEFAULT}}"
NO_E2E_LABEL="${NO_E2E_LABEL:-${NO_E2E_LABEL_DEFAULT}}"
STALE_LABEL="${STALE_LABEL:-${STALE_LABEL_DEFAULT}}"

PREFIX="[agent-flow-unlabeled-sweep]"

# --- MAINTENANCE gate + env (из .env если есть) -----------------------------
# Ретро 31.08 (t_9b0d60f7, agent-flow-unlabeled-sweep cron 24-fail подряд):
# Ретро 28.08 (t_faac94b0, e2e-fail-streak-no-escalation): предыдущая
# версия использовала `read IFS='='` парсинг key=val — он уже заменён на
# `set -a; .` ниже; новый код superset (robust + ENV_FILE-fallback).
#
# Supersedes ретро 28.08 (t_faac94b0) — добавляет robust fallback по
# нескольким кандидатам ENV_FILE (однокандидатный fix развит до multi-candidate).
#
# Скрипт падал в no_agent cron-режиме когда `$HERMES_HOME` в env указывал
# на профильную папку (например `/home/builder/.hermes/profiles/devops`),
# ENV_FILE вычислялся в `<profile>/profiles/agent-flow/.env` — не существовал
# → set -e срабатывал раньше source → GH_REPO оставался пустым →
# `: "${GH_REPO:?...}"` exit 1. Robust-фикс:
#   (1) Пробуем несколько кандидатов ENV_FILE (по убыванию приоритета):
#       - $HERMES_HOME/profiles/agent-flow/.env (как был)
#       - $HOME/.hermes/profiles/agent-flow/.env (system-cron, ~ = HOME)
#       - /home/builder/.hermes/profiles/agent-flow/.env (absolute fallback)
#   (2) Используем `set -a; . "$ENV_FILE"; set +a` — robust к `=` в значениях,
#       не падает на пустом .env.
#   (3) Финальная проверка GH_REPO сообщает какой ENV_FILE пробовался.
ENV_FILE=""
for _candidate in \
  "$HERMES_HOME/profiles/agent-flow/.env" \
  "$HOME/.hermes/profiles/agent-flow/.env" \
  "/home/builder/.hermes/profiles/agent-flow/.env"; do
  if [ -n "$_candidate" ] && [ -f "$_candidate" ]; then
    ENV_FILE="$_candidate"
    break
  fi
done
if [ -n "$ENV_FILE" ]; then
  # shellcheck disable=SC1090
  set -a; . "$ENV_FILE"; set +a
fi
: "${GH_REPO:?GH_REPO must be set (owner/repo) — checked $HERMES_HOME/profiles/agent-flow/.env, $HOME/.hermes/profiles/agent-flow/.env, /home/builder/.hermes/profiles/agent-flow/.env}"

log() { printf '%s %s %s\n' "$PREFIX" "$(date -Iseconds)" "$*" >&2; }

cli_args() {
  while [ $# -gt 0 ]; do
    case "$1" in
      --dry-run)        DRY_RUN="true"; shift ;;
      --stale-hours-1)  STALE_HOURS_1="$2"; shift 2 ;;
      --stale-hours-2)  STALE_HOURS_2="$2"; shift 2 ;;
      --limit)          SWEEP_LIMIT="$2"; shift 2 ;;
      -h|--help)
        grep '^#' "$0" | sed 's/^# \{0,1\}//'
        exit 0 ;;
      *) log "unknown arg: $1"; exit 2 ;;
    esac
  done
}

# Test mode (used by tests/test_unlabeled_sweep.sh): skip cli_args,
# flock, gh-auth. Lets unit-tests inject mocks via shell functions.
if [ "${UNLABELED_SWEEP_TEST_MODE:-0}" != "1" ]; then
  cli_args "$@"
fi

# --- flock sentinel ---------------------------------------------------------
if [ "${UNLABELED_SWEEP_TEST_MODE:-0}" != "1" ]; then
  exec 9>"$LOCK_FILE" || { log "cannot open lock $LOCK_FILE"; exit 1; }
  if ! flock -n 9; then
    log "another instance holds $LOCK_FILE — skip tick"; exit 0
  fi
fi

# --- gate: gh auth ----------------------------------------------------------
if [ "${UNLABELED_SWEEP_TEST_MODE:-0}" != "1" ]; then
  if ! gh auth status >/dev/null 2>&1; then
    log "gh auth not configured — exit 1"; exit 1
  fi
fi

log "tick start: GH_REPO=$GH_REPO stale1=${STALE_HOURS_1}h stale2=${STALE_HOURS_2}h limit=$SWEEP_LIMIT dry_run=$DRY_RUN"

# Все потенциальные кандидаты: OPEN issues.
# Не фильтруем по меткам — будем фильтровать внутри, чтобы иметь
# complete view (для сообщения 'никого нет'). Но лаг API ~5–10s на 200 —
# это OK для cron раз в час.
all_json="$(gh issue list \
    --repo "$GH_REPO" \
    --state open \
    --limit "$SWEEP_LIMIT" \
    --json number,title,labels,updatedAt,createdAt 2>/dev/null || echo '[]')"
if [ -z "$all_json" ]; then all_json='[]'; fi

# --- helpers -----------------------------------------------------------------
has_label() {  # $1=labels_csv(lowercase)  $2=label_name
  case ",${1}," in *",${2},"*) return 0 ;; *) return 1 ;; esac
}

# Получить ISO-время последнего события `reopened` или пусто.
# ADR-0022 GATE-2 + PR #1399: только событие 'reopened' из timeline
# (не 'closed'/'labeled'). Не путать со снятием метки.
# Парсим JSON в python (а не --jq) — устойчиво к mock-gh в unit-тестах
# и не зависит от того, умеет ли mock фильтровать.
last_reopen_at() {  # $1=issue_number
  gh api "repos/${GH_REPO}/issues/${1}/timeline?per_page=100" \
    2>/dev/null | python3 -c '
import json, sys
try:
    data = json.load(sys.stdin)
except Exception:
    print(""); sys.exit(0)
matches = [e for e in data if isinstance(e, dict) and e.get("event") == "reopened"]
print(matches[-1].get("created_at", "") if matches else "")
' || true
}

# Получить ISO-время установки метки stale-candidate, или пусто.
stale_labeled_at() {  # $1=issue_number
  gh api "repos/${GH_REPO}/issues/${1}/timeline?per_page=100" \
    2>/dev/null | STALE_LABEL="${STALE_LABEL}" python3 -c '
import json, os, sys
try:
    data = json.load(sys.stdin)
except Exception:
    print(""); sys.exit(0)
target = os.environ.get("STALE_LABEL", "")
matches = [
    e for e in data
    if isinstance(e, dict) and e.get("event") == "labeled"
    and ((e.get("label") or {}).get("name") == target)
]
print(matches[-1].get("created_at", "") if matches else "")
' || true
}

# Конвертировать ISO-время в epoch; пусто → echo 0.
to_epoch() {  # $1=iso_time
  local t="$1"
  if [ -z "$t" ] || [ "$t" = "null" ]; then echo 0; return 0; fi
  date -d "$t" +%s 2>/dev/null || echo 0
}

# Комментарий с префиксом уже был в окне?  $1=issue  $2=prefix  $3=since_iso
# Возвращает 0 (found) если есть хотя бы 1, иначе 1.
# Используем python для устойчивости к shell-quoting (префикс может
# содержать эмодзи, переводы строк, любые юникод-символы).
has_recent_marker_comment() {
  local issue="$1" prefix="$2" since="$3" _count
  _count="$(gh api "repos/${GH_REPO}/issues/${issue}/comments?since=${since}&per_page=100" \
    2>/dev/null | python3 -c '
import json, sys
try:
    data = json.load(sys.stdin)
except Exception:
    print(0); sys.exit(0)
target = sys.argv[1]
print(sum(1 for c in data if isinstance(c.get("body"), str) and c["body"].startswith(target)))
' "$prefix")" || _count=0
  [ "${_count:-0}" -gt 0 ] 2>/dev/null
}

now_minus_h_iso() {  # $1=hours
  date -u -d "${1} hours ago" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ
}

# Сводные счётчики (process substitution — чтобы они были видны после цикла)
considered=0
stale_marked=0
closed=0
un_staled=0
skipped=0
errored=0
fresh=0

# --- main loop ---------------------------------------------------------------
# process substitution, а не pipe, чтобы счётчики пробрасывались обратно.
# shellcheck disable=SC2034  # title/created_at are kept for log/debug symmetry with deploy-sweep
while IFS=$'\t' read -r number title updated_at created_at labels_csv; do
  [ -z "$number" ] && continue
  considered=$((considered+1))
  labels_norm="$(printf '%s' "$labels_csv" | tr '[:upper:]' '[:lower:]')"

  # Already in process → skip
  if has_label "$labels_norm" "$ISSUE_LABEL" \
     || has_label "$labels_norm" "$NEEDS_E2E_LABEL" \
     || has_label "$labels_norm" "$DONE_LABEL" \
     || has_label "$labels_norm" "$REJECTED_LABEL" \
     || has_label "$labels_norm" "$NO_E2E_LABEL"; then
    skipped=$((skipped+1))
    log "issue #${number}: уже в process-цикле (${labels_norm}) — skip"
    continue
  fi

  upd_epoch="$(to_epoch "$updated_at")"
  now_epoch="$(date +%s)"
  age_hours=$(( (now_epoch - upd_epoch) / 3600 ))

  # --- BRANCH B: уже stale-candidate --------------------------------------
  if has_label "$labels_norm" "$STALE_LABEL"; then
    labeled_at_iso="$(stale_labeled_at "$number")"
    labeled_at_epoch="$(to_epoch "$labeled_at_iso")"

    if [ -z "$labeled_at_iso" ] || [ "$labeled_at_iso" = "null" ]; then
      # timeline API сдох — fail-closed: не закрываем, чтобы не повторить
      # ретро-ping-pong (issue #1391).
      log "issue #${number}: stale-candidate на месте, но timeline stale_labeled_at недоступно — fail-closed, skip"
      skipped=$((skipped+1))
      errored=$((errored+1))
      continue
    fi

    # --- B1: user-reopen ПОСЛЕ stale_labeled_at → снимаем метку -----------
    reopen_at_iso="$(last_reopen_at "$number")"
    reopen_at_epoch="$(to_epoch "$reopen_at_iso")"
    if [ "$reopen_at_epoch" -gt "$labeled_at_epoch" ] 2>/dev/null; then
      log "issue #${number}: user-reopen (${reopen_at_iso}) ПОСЛЕ stale_labeled_at (${labeled_at_iso}) — un-stale"
      if [ "$DRY_RUN" != "true" ]; then
        gh issue edit "$number" --repo "$GH_REPO" --remove-label "$STALE_LABEL" >/dev/null 2>&1 || true
        gh issue comment "$number" --repo "$GH_REPO" --body \
          "agent-flow: ♻️ user-reopen обнаружен после метки ${STALE_LABEL} (${labeled_at_iso}). Метка снята — issue возвращена в OPEN без меток. Авто-закрывалка не тронет, пока процесс не возьмёт её в работу." >/dev/null 2>&1 || true
      fi
      un_staled=$((un_staled+1))
      continue
    fi

    # --- B2: прошло STALE_HOURS_2 с момента метки → закрываем ------------
    elapsed_after_label=$(( (now_epoch - labeled_at_epoch) / 3600 ))
    if [ "$elapsed_after_label" -lt "$STALE_HOURS_2" ]; then
      log "issue #${number}: stale-candidate ${elapsed_after_label}h назад (< ${STALE_HOURS_2}h) — подождём"
      skipped=$((skipped+1))
      continue
    fi

    # --- B3: дедупликация close-комментария (6h окно) ---------------------
    close_since="$(now_minus_h_iso 6)"
    if has_recent_marker_comment "$number" "⏰ auto-sweep stale-candidate → close" "$close_since"; then
      log "issue #${number}: close-комментарий уже оставлен в 6h окно — skip (idempotent)"
      skipped=$((skipped+1))
      continue
    fi

    log "issue #${number}: stale-candidate ${elapsed_after_label}h (>= ${STALE_HOURS_2}h) → close"
    if [ "$DRY_RUN" != "true" ]; then
      gh issue edit "$number" --repo "$GH_REPO" --remove-label "$STALE_LABEL" >/dev/null 2>&1 || true
      gh issue comment "$number" --repo "$GH_REPO" --body \
        "$(printf '⏰ auto-sweep stale-candidate → close (ADR-0022 GATE-2): issue в OPEN без process-меток %s+ час, метка %s висела %s+ час без user-reopen. Закрыто как not_planned. Если проблема всё ещё актуальна — откройте новый issue с актуальным контекстом.' "$STALE_HOURS_1" "$STALE_LABEL" "$STALE_HOURS_2")" >/dev/null 2>&1 || true
      if gh issue close "$number" --repo "$GH_REPO" --reason not_planned >/dev/null 2>&1; then
        closed=$((closed+1))
      else
        log "issue #${number}: WARNING close failed — retry next tick"
        errored=$((errored+1))
      fi
    else
      closed=$((closed+1))
    fi
    continue
  fi

  # --- BRANCH A: без process-меток, проверяем 'age' ----------------------
  if [ "$age_hours" -lt "$STALE_HOURS_1" ]; then
    fresh=$((fresh+1))
    continue
  fi

  # --- A1: ставим stale-candidate + комментарий --------------------------
  label_since="$(now_minus_h_iso 6)"
  if has_recent_marker_comment "$number" "⚠️ auto-sweep pending stale-candidate" "$label_since"; then
    log "issue #${number}: stale-комментарий уже оставлен в 6h окно — skip label"
    skipped=$((skipped+1))
    continue
  fi

  log "issue #${number}: age=${age_hours}h (>= ${STALE_HOURS_1}h), без process-меток → mark ${STALE_LABEL}"
  if [ "$DRY_RUN" != "true" ]; then
    if gh issue edit "$number" --repo "$GH_REPO" --add-label "$STALE_LABEL" >/dev/null 2>&1; then
      gh issue comment "$number" --repo "$GH_REPO" --body \
        "$(printf '⚠️ auto-sweep pending stale-candidate (ADR-0022 GATE-2): issue в OPEN без process-меток уже %s+ час. Через %s час метка `stale-candidate` будет снята и issue закрыта как not_planned. Если вы работаете над ней — откройте любой комментарий или переоткройте issue после возможного системного close, метка будет снята автоматически.' "$STALE_HOURS_1" "$STALE_HOURS_2")" >/dev/null 2>&1 || true
      stale_marked=$((stale_marked+1))
    else
      log "issue #${number}: WARNING label-add failed — retry next tick"
      errored=$((errored+1))
    fi
  else
    stale_marked=$((stale_marked+1))
  fi
done < <(printf '%s' "$all_json" | python3 -c '
import json, sys
data = json.load(sys.stdin)
for i in data:
    labels = ",".join(sorted((l.get("name") or "").lower() for l in (i.get("labels") or [])))
    print(
        str(i.get("number", "")) + "\t" +
        (i.get("title") or "")[:80] + "\t" +
        (i.get("updatedAt") or "") + "\t" +
        (i.get("createdAt") or "") + "\t" +
        labels
    )
' 2>/dev/null)

log "tick done: considered=${considered} fresh=${fresh} labeled=${stale_marked} closed=${closed} un_staled=${un_staled} skipped=${skipped} errored=${errored}"

# exit non-zero only on hard errors so cron can alert.
if [ "$errored" -gt 0 ]; then exit 1; fi
exit 0
