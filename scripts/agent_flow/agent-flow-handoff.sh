#!/bin/bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-handoff.sh
# Правим ТОЛЬКО здесь + commit + merge в develop. На хост раскладывает
# `bash <repo>/scripts/agent_flow/install.sh` — hardlink-копиями (cp -al), НЕ
# симлинками: симлинк в ~/.hermes/scripts/ ресолвится наружу и отклоняется
# guard'ом hermes-agent scheduler.py::_validate_script_path (ретро 11.08
# t_a6a236e0d9f0470e — 50 упавших тиков подряд, 1ч42м даунтайма).
# Полный список путей раскладки — в install.sh, сверку копий держит
# agent-flow-drift-detect.sh. Ручная правка копии на хосте затрётся.
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
# --- shared library bootstrap ------------------------------------------------
# Общие помощники (дедуп 30.08): af_load_profile_env, af_flock_guard_or_exit,
# af_maintenance_gate_or_exit. До этого здесь лежала пятая копия
# .env-преамбулы и вторая копия MAINTENANCE-гейта — со своими текстами логов.
_LIB_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=lib_agent_flow_common.sh
. "$_LIB_DIR_HERE/lib_agent_flow_common.sh"

# Preserve explicit caller overrides while loading non-secret configuration.
af_load_profile_env "$ENV_FILE"
BOARD="${KANBAN_BOARD:-$BOARD}"
MAINTENANCE_BRANCH="${MAINTENANCE_BRANCH:-develop}"
MAINTENANCE_FILE="${MAINTENANCE_FILE:-MAINTENANCE}"
log() { printf '%s %s\n' "$PREFIX" "$*"; }
af_flock_guard_or_exit "$LOCK_FILE"
# Q19: maintenance pauses the entire flow (remote-флаг + локальный клон).
af_maintenance_gate_or_exit
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
  # Ретро t_b3476561: handoff раньше не передавал --skill → карточки stuck.
  # af_skill_for_profile() даёт детерминированный skill по assignee + check
  # что он реально есть в профиле (fail-OPEN если нет).
  child_skill="$(af_skill_for_profile "$next")"
  child_skill_args=()
  if [ -n "$child_skill" ]; then
    child_skill_args=(--skill "$child_skill")
    log "  skill-inference (handoff): next=${next} -> skill=${child_skill}"
  fi
  if [[ "$DRY_RUN" == true ]]; then
    log "DRY-RUN: would create blocked child for $id -> $next (skill=${child_skill:-none}) and subscribe telegram:495039871"
    continue
  fi
  out="$($HERMES_BIN kanban --board "$BOARD" create --assignee "$next" --parent "$id" --workspace worktree --branch "z-{agent}/${id}-${next}" --initial-status blocked "${child_skill_args[@]}" --body "$child_body" --created-by agent-flow-handoff "$child_title" 2>&1)" || { log "create failed for $id: $out"; exit 1; }
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
