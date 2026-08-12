#!/bin/bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-deploy-sweep.sh
# Каноническая версия живёт в репо. На хост раскладывается через
# `bash <repo>/scripts/agent_flow/install.sh`, который создаёт hardlink-копии в:
#   - ~/.hermes/profiles/agent-flow/scripts/
#   - ~/.hermes/profiles/architect/scripts/
#   - ~/.hermes/profiles/devops/scripts/
#   - ~/.hermes/scripts/
# Правка: редактируем <repo>/scripts/agent_flow/agent-flow-deploy-sweep.sh,
# commit, merge. На хост: bash <repo>/scripts/agent_flow/install.sh.
# ============================================================================
# agent-flow-deploy-sweep.sh — авто-sweep stale deployment issues (ретро 12.08).
#
# ПРАВИЛО (ретро/ADR t_d3e44336):
#   deployment-issue без апдейтов > STALE_HOURS (default 72ч) →
#     авто-проверка актуальности на живых Pi →
#       resolved  → close с комментарием (evidence)
#       актуально → авто-метка `hermes` (triage создаст kanban-карточку)
#       проверить нельзя (SSH fail / нет сигнатуры) → НЕ трогаем, лог
#
# ПОЧЕМУ: деплой-монитор (L-Deploy and Verify.yml) создавал issues
# автоматически, но никто не верифицировал/закрывал — висели неделями.
# Triage фильтрует только по метке `hermes`, поэтому deployment issues
# не попадали в конвейер issue→карточка by design. Этот скрипт —
# владелец stale deployment issues.
#
# Idempotent: пропускает issues с меткой hermes / e2e-done / e2e:rejected /
# свежие (< STALE_HOURS). Pure bash + python3 (JSON). No LLM.
#
# Usage:
#   agent-flow-deploy-sweep.sh [--stale-hours 72] [--dry-run] [--limit 100]
# Env: GH_REPO, STALE_HOURS, SSHPASS (или -p open), MAIN_PI_IP, VISION_PI_IP,
#      SSH_USER, SSH_OPTS, LOCK_FILE, DRY_RUN, HERMES_HOME
# ============================================================================
set -euo pipefail

HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
export HOME="${HOME:-/home/builder}"

STALE_HOURS="${STALE_HOURS:-72}"
LIMIT="${SWEEP_LIMIT:-100}"
DRY_RUN="${DRY_RUN:-false}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-deploy-sweep.lock}"
SSHPASS_VAL="${SSHPASS:-open}"
SSH_USER="${SSH_USER:-ros2}"
MAIN_PI_IP="${MAIN_PI_IP:-10.1.1.10}"
VISION_PI_IP="${VISION_PI_IP:-10.1.1.11}"
SSH_OPTS="${SSH_OPTS:--o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=8}"
PREFIX="[agent-flow-deploy-sweep]"

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

# --- list open deployment issues --------------------------------------------
issues_json="$(gh issue list \
    --repo "$GH_REPO" \
    --label deployment \
    --state open \
    --limit "$LIMIT" \
    --json number,title,labels,body,updatedAt 2>/dev/null || true)"

if [ -z "$issues_json" ] || [ "$issues_json" = "[]" ]; then
  log "no open issues with label 'deployment' — nothing to sweep"; exit 0
fi

# --- helpers -----------------------------------------------------------------
has_label() {  # $1=labels_json  $2=label_name
  printf '%s' "$1" | grep -q "\"name\":\"$2\""
}

parse_signature() {  # $1=body → echo "env|scope|container|kind" or empty
  local sig
  sig="$(printf '%s' "$1" | grep -oE 'deploy-signature: deploy-problem:[a-z0-9-]+:[a-z0-9-]+:[a-z0-9_-]+:[a-z0-9_]+:[a-f0-9]+' | head -n1 || true)"
  [ -z "$sig" ] && { echo ""; return 0; }
  # deploy-problem:ENV:SCOPE:CONTAINER:KIND:DIGEST
  printf '%s' "$sig" | sed -E 's/.*deploy-problem:([a-z0-9-]+):([a-z0-9-]+):([a-z0-9_-]+):([a-z0-9_]+):[a-f0-9]+/\1|\2|\3|\4/'
}

pi_ip_for_scope() {  # $1=scope
  if [ "$1" = "vision" ]; then echo "$VISION_PI_IP"; else echo "$MAIN_PI_IP"; fi
}

ssh_cmd() {  # $1=ip, $2=remote_cmd
  # -n / </dev/null: CRITICAL — ssh must NOT read the loop's stdin
  # (while read ... done < <(...)); otherwise it consumes the remaining
  # issue records and the sweep silently processes only the first few.
  sshpass -p "$SSHPASS_VAL" ssh -n $SSH_OPTS "${SSH_USER}@$1" "$2" < /dev/null 2>/dev/null || true
}

# --- verification per kind ---------------------------------------------------
# Returns: "resolved" | "actual" | "unknown"
verify_container_status() {  # $1=ip $2=container
  local out
  out="$(ssh_cmd "$1" "docker ps -a --format '{{.Names}}\t{{.Status}}' | grep -E '^$2[[:space:]]' || true")"
  [ -z "$out" ] && { log "  verify container_status: container '$2' not found on $1 — unknown"; echo "unknown"; return 0; }
  if printf '%s' "$out" | grep -qE 'Up [0-9]+'; then
    echo "resolved"
  else
    echo "actual"
  fi
}

verify_critical_log() {  # $1=ip $2=container $3=scope
  local logs
  logs="$(ssh_cmd "$1" "docker logs --tail 300 $2 2>&1 || true")"
  [ -z "$logs" ] && { log "  verify critical_log: no logs for '$2' on $1 — unknown"; echo "unknown"; return 0; }
  local crit
  crit="$(printf '%s\n' "$logs" | python3 "$DEDUP_HELPER" extract-log --scope "$3" --severity critical 2>/dev/null || true)"
  if [ -z "$crit" ]; then
    echo "resolved"
  else
    echo "actual"
  fi
}

verify_warning_log() {  # $1=ip $2=container $3=scope
  local logs
  logs="$(ssh_cmd "$1" "docker logs --tail 300 $2 2>&1 || true")"
  [ -z "$logs" ] && { log "  verify warning_log: no logs for '$2' on $1 — unknown"; echo "unknown"; return 0; }
  local warn
  warn="$(printf '%s\n' "$logs" | python3 "$DEDUP_HELPER" extract-log --scope "$3" --severity warning 2>/dev/null || true)"
  if [ -z "$warn" ]; then
    echo "resolved"
  else
    echo "actual"
  fi
}

verify_topic_check() {  # $1=ip $2=container
  local topics
  topics="$(ssh_cmd "$1" "docker exec $2 bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null' || true")"
  if [ -z "$topics" ]; then
    log "  verify topic_check: no topics from '$2' on $1 — actual"
    echo "actual"; return 0
  fi
  if printf '%s\n' "$topics" | grep -qE 'camera|rgb|depth|image'; then
    echo "resolved"
  else
    log "  verify topic_check: topics present but no camera/rgb/depth — actual"
    echo "actual"
  fi
}

close_with_evidence() {  # $1=issue $2=verdict_detail
  run gh issue close "$1" --repo "$GH_REPO" --comment \
    "✅ **Авто-sweep** ($(date -u '+%Y-%m-%d %H:%M UTC')): авто-проверка на живых Pi показала, что проблема ушла — $2. Закрыто по правилу ретро 12.08 (deployment issue без апдейтов > 72ч → авто-проверка + close)."
}

label_hermes() {  # $1=issue $2=verdict_detail
  run gh issue edit "$1" --repo "$GH_REPO" --add-label hermes >/dev/null
  run gh issue comment "$1" --repo "$GH_REPO" --body \
    "🔎 **Авто-sweep** ($(date -u '+%Y-%m-%d %H:%M UTC')): проблема **АКТУАЛЬНА** — $2. Проставлена метка \`hermes\` для триажа (ретро 12.08: stale deployment issue > 72ч → авто-метка hermes)."
}

# --- resolve dedup helper path (for critical_log verify) ---------------------
# Prefer REPO_DIR (set in agent-flow/.env); fallback to CWD.
DEDUP_HELPER="${DEDUP_HELPER:-}"
if [ -z "$DEDUP_HELPER" ]; then
  if [ -n "${REPO_DIR:-}" ] && [ -f "$REPO_DIR/.github/scripts/deployment_issue_dedup.py" ]; then
    DEDUP_HELPER="$REPO_DIR/.github/scripts/deployment_issue_dedup.py"
  elif [ -f ".github/scripts/deployment_issue_dedup.py" ]; then
    DEDUP_HELPER=".github/scripts/deployment_issue_dedup.py"
  fi
fi

# --- process each issue ------------------------------------------------------
swept=0; closed=0; labeled=0; skipped=0; unknown=0

# NOTE: use process substitution (not a pipe) so counter vars propagate
# back to the parent shell for the final summary.
while IFS=$'\t' read -r number title_b64 updated_at body_b64; do
  [ -z "$number" ] && continue
  title="$(printf '%s' "$title_b64" | base64 -d 2>/dev/null || true)"
  body="$(printf '%s' "$body_b64" | base64 -d 2>/dev/null || true)"

  labels_json="$(gh issue view "$number" --repo "$GH_REPO" --json labels --jq '.labels' < /dev/null 2>/dev/null || echo '[]')"

  # Skip if already in triage / done
  if has_label "$labels_json" "hermes"; then
    log "issue #${number}: already has hermes — skip"; skipped=$((skipped+1)); continue
  fi
  if has_label "$labels_json" "e2e-done" || has_label "$labels_json" "e2e:rejected"; then
    log "issue #${number}: work already done/rejected — skip"; skipped=$((skipped+1)); continue
  fi

  # Skip fresh issues (< STALE_HOURS since last update)
  upd_epoch="$(date -d "$updated_at" +%s 2>/dev/null || echo 0)"
  now_epoch="$(date +%s)"
  if [ $(( now_epoch - upd_epoch )) -lt $(( STALE_HOURS * 3600 )) ]; then
    log "issue #${number}: updated < ${STALE_HOURS}h ago — fresh, skip"; skipped=$((skipped+1)); continue
  fi

  # Parse deploy-signature
  sig="$(parse_signature "$body")"
  if [ -z "$sig" ]; then
    log "issue #${number}: no deploy-signature — cannot auto-verify, skip (manual triage)"; skipped=$((skipped+1)); continue
  fi
  env_name="$(printf '%s' "$sig" | cut -d'|' -f1)"
  scope="$(printf '%s' "$sig" | cut -d'|' -f2)"
  container="$(printf '%s' "$sig" | cut -d'|' -f3)"
  kind="$(printf '%s' "$sig" | cut -d'|' -f4)"
  ip="$(pi_ip_for_scope "$scope")"

  log "issue #${number}: sweep env=${env_name} scope=${scope} container=${container} kind=${kind} ip=${ip}"

  verdict="unknown"
  case "$kind" in
    container_status) verdict="$(verify_container_status "$ip" "$container")" ;;
    critical_log)     verdict="$(verify_critical_log "$ip" "$container" "$scope")" ;;
    warning_log)      verdict="$(verify_warning_log "$ip" "$container" "$scope")" ;;
    topic_check)      verdict="$(verify_topic_check "$ip" "$container")" ;;
    *) log "  unknown kind '${kind}' — skip"; skipped=$((skipped+1)); continue ;;
  esac

  case "$verdict" in
    resolved)
      log "issue #${number}: RESOLVED (${scope}/${container}/${kind}) — close"
      close_with_evidence "$number" "${scope}/${container}/${kind} больше не проявляется на ${ip}"
      closed=$((closed+1))
      ;;
    actual)
      log "issue #${number}: ACTUAL (${scope}/${container}/${kind}) — label hermes"
      label_hermes "$number" "${scope}/${container}/${kind} всё ещё в проблемном состоянии на ${ip}"
      labeled=$((labeled+1))
      ;;
    *)
      log "issue #${number}: verification unknown — left untouched"
      unknown=$((unknown+1))
      ;;
  esac
  swept=$((swept+1))
done < <(printf '%s' "$issues_json" | python3 -c '
import json, sys, base64
d = json.load(sys.stdin)
for i in d:
    body_b64 = base64.b64encode(str(i["body"]).encode("utf-8")).decode("ascii")
    title_b64 = base64.b64encode(str(i["title"]).encode("utf-8")).decode("ascii")
    print(str(i["number"]) + "\t" + title_b64 + "\t" + str(i["updatedAt"]) + "\t" + body_b64)
')

log "sweep done: swept=${swept} closed=${closed} labeled=${labeled} skipped=${skipped} unknown=${unknown}"
