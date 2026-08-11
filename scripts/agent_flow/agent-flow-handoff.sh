#!/bin/bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-handoff.sh
# Каноническая версия живёт в репо. На хост раскладывается через
# `bash <repo>/scripts/agent_flow/install.sh`, который создаёт
# символические ссылки в:
#   - ~/.hermes/profiles/agent-flow/scripts/agent-flow-handoff.sh
#   - ~/.hermes/profiles/architect/scripts/agent-flow-handoff.sh
#   - ~/.hermes/scripts/agent-flow-handoff.sh
# Правка: редактируем <repo>/scripts/agent_flow/agent-flow-handoff.sh, commit, merge.
# На хост: bash <repo>/scripts/agent_flow/install.sh (или вручную cp + ln -sf).
# Если ты правишь этот файл НА ХОСТЕ руками — синхронизируй обратно в репо.
# ============================================================================
# agent-flow-handoff.sh — Phase 2: done card -> approved, blocked child.
# Idempotent, no LLM. Configuration is read from agent-flow/.env.
set -euo pipefail
# NOTE: hardcode /home/builder/.hermes — cron from per-profile gateway sets
# HERMES_HOME to the profile dir; ENV_FILE would then point at a non-existent
# path and GH_REPO would never load.
HERMES_HOME=/home/builder/.hermes
HERMES_BIN="${HERMES_BIN:-/home/builder/.hermes/hermes-agent/venv/bin/hermes}"
export HOME="${HOME:-/home/builder}"
BOARD="${KANBAN_BOARD:-robbox}"
MAINTENANCE_BRANCH="${MAINTENANCE_BRANCH:-develop}"
MAINTENANCE_FILE="${MAINTENANCE_FILE:-MAINTENANCE}"
DRY_RUN="${DRY_RUN:-false}"
LOCK_FILE="${HANDOFF_LOCK_FILE:-/tmp/agent-flow-handoff.lock}"
PREFIX="[agent-flow-handoff]"
ENV_FILE="$HERMES_HOME/profiles/agent-flow/.env"
# Preserve explicit caller overrides while loading non-secret configuration.
if [[ -f "$ENV_FILE" ]]; then
  while IFS='=' read -r key val; do
    [[ -z "$key" || "$key" == \#* || -n "${!key:-}" ]] && continue
    val="${val%\"}"; val="${val#\"}"; val="${val%\'}"; val="${val#\'}"
    export "$key=$val"
  done < "$ENV_FILE"
fi
BOARD="${KANBAN_BOARD:-$BOARD}"
MAINTENANCE_BRANCH="${MAINTENANCE_BRANCH:-develop}"
MAINTENANCE_FILE="${MAINTENANCE_FILE:-MAINTENANCE}"
log() { printf '%s %s\n' "$PREFIX" "$*"; }
exec 9>"$LOCK_FILE"
flock -n 9 || { log "another tick is running; skip"; exit 0; }
# Q19: maintenance pauses the entire flow. Check configured local clone when present.
if [[ -n "${REPO_DIR:-}" && -d "$REPO_DIR" ]] && git -C "$REPO_DIR" show "$MAINTENANCE_BRANCH:$MAINTENANCE_FILE" >/dev/null 2>&1; then
  log "MAINTENANCE flag set locally; skip"; exit 0
fi
if [[ -n "${GH_REPO:-}" ]] && git ls-remote "https://github.com/${GH_REPO}.git" "$MAINTENANCE_BRANCH:$MAINTENANCE_FILE" 2>/dev/null | grep -q .; then
  log "MAINTENANCE flag set remotely; skip"; exit 0
fi
cards="$($HERMES_BIN kanban --board "$BOARD" list --json)"
# Emit one tab-delimited record per done card. Body is deliberately retained only in memory.
while IFS=$'\t' read -r id title_b64 body_b64; do
  [[ -z "$id" || -z "$body_b64" ]] && continue
  title="$(printf '%s' "$title_b64" | base64 -d)"
  body="$(printf '%s' "$body_b64" | base64 -d)"
  next="$(printf '%s\n' "$body" | grep -ioE '(^|[[:space:]])next:[[:space:]]*[a-zA-Z0-9_-]+' | head -n1 | sed -E 's/.*next:[[:space:]]*//I' || true)"
  [[ -z "$next" ]] && continue
  # Skip cards that are themselves handoff children (prevents recursion:
  # a child inherits `next:` and would spawn "Handoff: Handoff: ..." forever).
  if printf '%s' "$title" | grep -q '^Handoff:'; then continue; fi
  if printf '%s' "$body" | grep -q '^handoff-parent:'; then continue; fi
  # Marker prevents duplicate children across ticks.
  if printf '%s' "$cards" | grep -Fq "handoff-parent: $id"; then continue; fi
  child_title="Handoff: $title"
  child_body="Parent: $id
handoff-parent: $id
next: $next
awaiting-approval: approve execution of the handoff described below.

$body"
  if [[ "$DRY_RUN" == true ]]; then
    log "DRY-RUN: would create blocked child for $id -> $next and subscribe telegram:495039871"
    continue
  fi
  out="$($HERMES_BIN kanban --board "$BOARD" create --assignee "$next" --parent "$id" --workspace worktree --branch "z-{agent}/${id}-${next}" --initial-status blocked --body "$child_body" --created-by agent-flow-handoff "$child_title" 2>&1)" || { log "create failed for $id: $out"; exit 1; }
  child="$(printf '%s' "$out" | grep -oE 't_[a-f0-9]+' | head -n1 || true)"
  [[ -n "$child" ]] || { log "cannot parse child id: $out"; exit 1; }
  "$HERMES_BIN" kanban --board "$BOARD" notify-subscribe --platform telegram --chat-id 495039871 --notifier-profile pm "$child" >/dev/null
  log "created $child for $id; awaiting approval; next=$next"
done < <(printf '%s' "$cards" | python3 -c 'import base64,json,sys
for x in json.load(sys.stdin):
 if x.get("status")=="done":
  enc=lambda v: base64.b64encode((v or "").encode()).decode()
  print("%s\t%s\t%s" % (x.get("id", ""), enc(x.get("title")), enc(x.get("body"))))')
exit 0
