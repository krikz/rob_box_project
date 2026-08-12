#!/bin/bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-unlabeled-sweep.sh
# Каноническая версия живёт в репо. На хост раскладывается через
# `bash <repo>/scripts/agent_flow/install.sh`, который создаёт hardlink-копии в:
#   - ~/.hermes/profiles/agent-flow/scripts/
#   - ~/.hermes/profiles/architect/scripts/
#   - ~/.hermes/profiles/devops/scripts/
#   - ~/.hermes/scripts/
# Правка: редактируем <repo>/scripts/agent_flow/agent-flow-unlabeled-sweep.sh,
# commit, merge. На хост: bash <repo>/scripts/agent_flow/install.sh.
# ============================================================================
# agent-flow-unlabeled-sweep.sh — авто-sweep stale unlabeled issues (ретро 12.08 t_061d466e).
#
# ПРАВИЛО (ретро 12.08 t_061d466e):
#   open issue БЕЗ process-меток (hermes/agent:*/needs-e2e/e2e-done/
#   e2e:rejected/no-e2e-required) без апдейтов > SWEEP_DAYS (default 2д) →
#     возраст в окне [SWEEP_DAYS, MAX_AGE_DAYS] → эвристика по title/body:
#       voice/tts/music/audio/stt/vad/barge/speaker/speech/silero/yandex/vosk → agent:backend
#       ci/deploy/docker/build/workflow/action/runner/image/container/tag      → agent:devops
#       architect/architecture/adr/design/refactor/монолит                     → agent:architect
#     роль определена → авто-метки `agent:<role>` + `hermes` (triage создаст
#                       kanban-карточку) + коммент-напоминание
#     роль НЕ определена → коммент-напоминание (issue не размечена, нужен
#                          ручной триаж)
#   возраст > MAX_AGE_DAYS (default 21д) → ТОЛЬКО коммент-напоминание, НЕ
#     ставим hermes: слишком старые issues (например майские GSD #801-#849)
#     скорее всего уже неактуальны, авто-разметка запустит воркеров на
#     мёртвые задачи (ретро 12.08 t_061d466e).
#   build-failed issue (метка build-failure/ci/cd, title «Build Failed»)
#   старше BUILD_FAILED_CLOSE_DAYS (default 30д) → проверить, что билды
#   сейчас зелёные (L-Build Vision/Main Pi на develop) → resolved: close
#   с комментарием; CI не зелёный/не проверить → НЕ трогаем.
#
# ПОЧЕМУ: триаж (agent-flow-triage.sh) фильтрует только по метке `hermes`,
# а старые issues без неё (например #918 busy-loop от 29.07, #929 OOM,
# #931/#933 TTS, #1016 музыка) висят неделями неразмеченными. Этот скрипт —
# владелец stale unlabeled issues: размечивает по эвристике или напоминает.
#
# Idempotent: пропускает issues с process-метками, свежие (< SWEEP_DAYS) и
# уже размеченные. Комментарии дедуплицируются (24h окно). Pure bash + python3.
#
# Usage:
#   agent-flow-unlabeled-sweep.sh [--dry-run] [--limit 100]
# Env: GH_REPO, SWEEP_DAYS, BUILD_FAILED_CLOSE_DAYS, DRY_RUN, LOCK_FILE,
#      HERMES_HOME
# ============================================================================
set -euo pipefail

HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
export HOME="${HOME:-/home/builder}"

SWEEP_DAYS="${SWEEP_DAYS:-2}"
MAX_AGE_DAYS="${MAX_AGE_DAYS:-21}"
BUILD_FAILED_CLOSE_DAYS="${BUILD_FAILED_CLOSE_DAYS:-30}"
LIMIT="${SWEEP_LIMIT:-100}"
DRY_RUN="${DRY_RUN:-false}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-unlabeled-sweep.lock}"
PREFIX="[agent-flow-unlabeled-sweep]"

# --- MAINTENANCE gate + env -------------------------------------------------
ENV_FILE="$HERMES_HOME/profiles/agent-flow/.env"
if [ -f "$ENV_FILE" ]; then
  while IFS='=' read -r key val; do
    case "$key" in ''|\#*) continue ;; esac
    val="${val%\"}"; val="${val#\"}"; val="${val%\'}"; val="${val#\'}"
    if [ -z "${!key:-}" ]; then
      export "$key=$val"
    fi
  done < "$ENV_FILE"
fi
: "${GH_REPO:?GH_REPO must be set (owner/repo)}"

log() { printf '%s %s %s\n' "$PREFIX" "$(date -Iseconds)" "$*" >&2; }
run() {
  if [ "$DRY_RUN" = "true" ]; then
    printf '%s DRY-RUN %s\n' "$PREFIX" "$*" >&2
  else
    "$@"
  fi
}

# flock: skip tick if another instance holds the lock.
exec 9>"$LOCK_FILE" || { log "cannot open lock $LOCK_FILE"; exit 1; }
if ! flock -n 9; then
  log "another instance holds $LOCK_FILE — skip"; exit 0
fi

# gh auth check
if ! gh auth status >/dev/null 2>&1; then
  log "gh auth not configured — exit 1"; exit 1
fi

# --- list open issues WITHOUT process labels --------------------------------
# Тяжело отфильтровать «нет метки» через gh issue list, поэтому берём все
# open issues и фильтруем python'ом (как и положено — метки приходят в JSON).
issues_json="$(gh issue list \
    --repo "$GH_REPO" \
    --state open \
    --limit "$LIMIT" \
    --json number,title,labels,body,updatedAt 2>/dev/null || true)"

if [ -z "$issues_json" ] || [ "$issues_json" = "[]" ]; then
  log "no open issues — nothing to sweep"; exit 0
fi

# --- helpers -----------------------------------------------------------------
has_label_csv() {  # $1=labels_csv(lowercased) $2=label_name
  printf '%s' "$1" | tr ',' '\n' | grep -Fxq "$2"
}

PROCESS_LABELS="hermes needs-e2e e2e-done e2e:rejected no-e2e-required needs-discussion needs-review big-bang-override"
# Все метки agent:* считаются process-метками (уже размечены).
is_process_labeled() {  # $1=labels_csv(lowercased)
  local lbl
  for lbl in $PROCESS_LABELS; do
    if has_label_csv "$1" "$lbl"; then return 0; fi
  done
  if printf '%s' "$1" | tr ',' '\n' | grep -qE '^agent:'; then return 0; fi
  return 1
}

# Эвристика роли по меткам + title+body. Печатает role или пусто.
# Сначала смотрим существующие метки (они точнее текста): architecture →
# architect; ci/cd/deploy/docker → devops; voice/tts/music → backend.
heuristic_role() {  # $1=labels_csv(lowercased) $2=text
  local labels="$1" t="$2"
  if printf '%s' "$labels" | tr ',' '\n' | grep -qE '^architecture$|^architect$|^adr$'; then
    printf '%s' "architect"; return 0
  fi
  if printf '%s' "$labels" | tr ',' '\n' | grep -qE '^(ci|ci/cd|deployment|deploy|docker|devops|build-failure|infrastructure)$'; then
    printf '%s' "devops"; return 0
  fi
  if printf '%s' "$labels" | tr ',' '\n' | grep -qE '^(voice|tts|stt|music|audio|supercollider|vad|speech)$'; then
    printf '%s' "backend"; return 0
  fi
  if printf '%s' "$t" | grep -qiE 'voice|tts|stt|music|audio|vad|barge|speaker|speech|silero|yandex|vosk|mic|sound|rap|melod|song|музык|песн|голос|звук|трек|бит|речь|реп|диалог|dialog|say|speak|greet|приветств|фраза|phrase|greeting'; then
    printf '%s' "backend"; return 0
  fi
  if printf '%s' "$t" | grep -qiE 'ci|deploy|docker|build|workflow|action|runner|image|container|tag|pipeline|github|yaml'; then
    printf '%s' "devops"; return 0
  fi
  if printf '%s' "$t" | grep -qiE 'architect|architecture|adr|design|refactor|монолит|monolith|промпт|prompt|планировщик|scheduler'; then
    printf '%s' "architect"; return 0
  fi
  return 0
}

is_build_failed() {  # $1=labels_csv(lowercased) $2=title
  if has_label_csv "$1" "build-failure" || has_label_csv "$1" "build-failed"; then
    return 0
  fi
  printf '%s' "$2" | grep -qiE 'build failed|build failure'
}

# Проверяем, что билды сейчас зелёные: последний run L-Build Vision/Main Pi
# на develop = success. Возвращает 0 если зелёные (можно закрывать), 1 если
# нет / не проверить.
builds_green_now() {
  local wf ok=1
  for wf in "L-Build Vision Pi Services.yml" "L-Build Main Pi Services.yml"; do
    local concl
    concl="$(gh run list --repo "$GH_REPO" --workflow "$wf" --branch develop --limit 1 \
        --json conclusion --jq '.[0].conclusion // empty' 2>/dev/null || true)"
    if [ -z "$concl" ]; then
      log "  builds_green_now: workflow ${wf} не найден/нет runs — не можем подтвердить"
      return 1
    fi
    if [ "$concl" != "success" ]; then
      log "  builds_green_now: последний ${wf} на develop = ${concl} — НЕ закрываем"
      return 1
    fi
  done
  return 0
}

# Комментарий с дедупликацией (24h) — не спамим каждый тик.
comment_dedup() {  # $1=issue $2=prefix $3=body
  local issue="$1" prefix="$2" body="$3" since dup
  since="$(date -u -d '24 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
  dup="$(gh api "repos/${GH_REPO}/issues/${issue}/comments?since=${since}&per_page=100" \
      --jq '[.[] | select(.body | startswith("'$prefix'"))] | length' 2>/dev/null || echo 0)"
  if [ "${dup:-0}" -eq 0 ]; then
    run gh issue comment "$issue" --repo "$GH_REPO" --body "$body" >/dev/null
  else
    log "issue #${issue}: коммент ${prefix} уже есть (×${dup} за 24h) — dedup skip"
  fi
}

# --- process each issue ------------------------------------------------------
swept=0; labeled=0; reminded=0; closed=0; skipped=0

# NOTE: process substitution (не pipe) чтобы счётчики дошли до summary.
while IFS=$'\t' read -r number title_b64 updated_at labels_b64 body_b64; do
  [ -z "$number" ] && continue
  title="$(printf '%s' "$title_b64" | base64 -d 2>/dev/null || true)"
  labels_csv="$(printf '%s' "$labels_b64" | base64 -d 2>/dev/null || true)"
  body="$(printf '%s' "$body_b64" | base64 -d 2>/dev/null || true)"

  labels_norm="$(printf '%s' "$labels_csv" | tr '[:upper:]' '[:lower:]')"

  # Skip if already in triage / process cycle
  if is_process_labeled "$labels_norm"; then
    log "issue #${number}: уже в process-цикле (${labels_csv}) — skip"; skipped=$((skipped+1)); continue
  fi

  # Skip fresh issues (< SWEEP_DAYS since last update)
  upd_epoch="$(date -d "$updated_at" +%s 2>/dev/null || echo 0)"
  now_epoch="$(date +%s)"
  if [ $(( now_epoch - upd_epoch )) -lt $(( SWEEP_DAYS * 86400 )) ]; then
    log "issue #${number}: updated < ${SWEEP_DAYS}d ago — fresh, skip"; skipped=$((skipped+1)); continue
  fi

  log "issue #${number}: sweep «${title:0:60}»"

  # --- build-failed stale → проверка зелёного CI → close --------------------
  if is_build_failed "$labels_norm" "$title"; then
    age_days=$(( (now_epoch - upd_epoch) / 86400 ))
    if [ "$age_days" -lt "$BUILD_FAILED_CLOSE_DAYS" ]; then
      log "issue #${number}: build-failed, возраст ${age_days}д < ${BUILD_FAILED_CLOSE_DAYS}д — skip"
      skipped=$((skipped+1)); continue
    fi
    if builds_green_now; then
      log "issue #${number}: build-failed от ${age_days}д назад, билды сейчас зелёные — close"
      comment_dedup "$number" "✅ Авто-sweep" \
        "✅ **Авто-sweep** ($(date -u '+%Y-%m-%d %H:%M UTC')): build-failed issue без апдейтов > ${BUILD_FAILED_CLOSE_DAYS}д; последние L-Build Vision/Main Pi на develop зелёные — проблема неактуальна. Закрыто по правилу ретро 12.08 (stale build-failed)."
      run gh issue close "$number" --repo "$GH_REPO" --reason completed >/dev/null
      closed=$((closed+1))
    else
      log "issue #${number}: build-failed, но CI не зелёный/не проверить — НЕ трогаем"
      skipped=$((skipped+1))
    fi
    swept=$((swept+1)); continue
  fi

  # --- эвристика роли → agent:<role> + hermes --------------------------------
  role="$(heuristic_role "$labels_norm" "$title
$body")"
  # Возраст для решения: ставить hermes или только напомнить.
  age_days=$(( (now_epoch - upd_epoch) / 86400 ))
  if [ -n "$role" ] && [ "$age_days" -le "$MAX_AGE_DAYS" ]; then
    log "issue #${number}: эвристика → agent:${role} + hermes (возраст ${age_days}д ≤ ${MAX_AGE_DAYS}д)"
    comment_dedup "$number" "🔎 Авто-sweep" \
      "🔎 **Авто-sweep** ($(date -u '+%Y-%m-%d %H:%M UTC')): issue без process-меток > ${SWEEP_DAYS}д. По эвристике title/body определена роль **${role}** — проставлены \\\`agent:${role}\\\` и \\\`hermes\\\`; триаж создаст карточку (ретро 12.08 t_061d466e)."
    run gh issue edit "$number" --repo "$GH_REPO" --add-label "agent:${role}" >/dev/null
    run gh issue edit "$number" --repo "$GH_REPO" --add-label "hermes" >/dev/null
    labeled=$((labeled+1))
  else
    # Либо роль не определена, либо issue слишком старая (> MAX_AGE_DAYS) —
    # в обоих случаях только коммент-напоминание, без hermes (не запускаем
    # воркеров на потенциально неактуальные задачи).
    _remind_reason="эвристика не смогла определить роль"
    if [ -n "$role" ] && [ "$age_days" -gt "$MAX_AGE_DAYS" ]; then
      _remind_reason="возраст ${age_days}д > ${MAX_AGE_DAYS}д (слишком старая — ручной триаж)"
    fi
    log "issue #${number}: ${_remind_reason} — коммент-напоминание"
    comment_dedup "$number" "❗ Авто-sweep" \
      "❗ **Авто-sweep** ($(date -u '+%Y-%m-%d %H:%M UTC')): issue без process-меток > ${SWEEP_DAYS}д; ${_remind_reason}. Требуется ручная триаж-разметка (ретро 12.08 t_061d466e)."
    reminded=$((reminded+1))
  fi
  swept=$((swept+1))
done < <(printf '%s' "$issues_json" | python3 -c '
import json, sys, base64
d = json.load(sys.stdin)
for i in d:
    title_b64 = base64.b64encode(str(i["title"]).encode("utf-8")).decode("ascii")
    labels_b64 = base64.b64encode(str(",".join(sorted({lab["name"] for lab in i.get("labels", [])}))).encode("utf-8")).decode("ascii")
    body_b64 = base64.b64encode(str(i["body"]).encode("utf-8")).decode("ascii")
    print(str(i["number"]) + "\t" + title_b64 + "\t" + str(i["updatedAt"]) + "\t" + labels_b64 + "\t" + body_b64)
')

log "sweep done: swept=${swept} labeled=${labeled} reminded=${reminded} closed=${closed} skipped=${skipped}"
