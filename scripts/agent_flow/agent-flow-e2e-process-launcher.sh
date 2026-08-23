#!/bin/bash
# ============================================================================
# agent-flow-e2e-process-launcher.sh — launcher для cron (no-agent job).
#
# Назначение: быть тонкой обёрткой, которая:
#   1. Подгружает env из профильного .env (GH_REPO, KANBAN_BOARD и т.д.)
#   2. Запускает канонический agent-flow-e2e-process.sh (он уже в scripts_dir
#      через hardlink от install.sh).
#
# Используется cron job'ом (no_agent=true) — БЕЗ LLM, просто bash.
# Скрипт сам НЕ является SOT — SOT живёт в
#   <repo>/scripts/agent_flow/agent-flow-e2e-process.sh
# и раскладывается через install.sh.
#
# Ретро 23.08 (t_98bb3a1d, e2e-rotation-idle-no-cron): до этого скрипта
# e2e-process запускался только вручную падаваном, и падаван упал с 401 →
# rotation простаивал 20ч+. Этот launcher восстанавливает автозапуск.
# ============================================================================

set -uo pipefail  # без -e — ошибки скрипта НЕ должны убивать cron-job

# 1. env из профильного .env (там GH_REPO, KANBAN_BOARD, MAINTENANCE_*).
ENV_FILE="/home/builder/.hermes/profiles/agent-flow/.env"
if [ ! -f "$ENV_FILE" ]; then
    echo "❌ launcher: env file not found: $ENV_FILE" >&2
    exit 0  # silent cron — не шуметь если профиля нет
fi
# shellcheck disable=SC1090
set -a; . "$ENV_FILE"; set +a

# 2. Убедимся что GH_TOKEN доступен (защита от cron-race, когда env не
#    прокидывается в подпроцесс). gh сам читает из keyring.
export GH_CONFIG_DIR="${GH_CONFIG_DIR:-/home/builder/.config/gh}"
if [ -z "${GH_TOKEN:-}" ] && command -v gh >/dev/null 2>&1; then
    GH_TOKEN="$(gh auth token 2>/dev/null || true)"
    [ -n "$GH_TOKEN" ] && export GH_TOKEN
fi

# 3. Запуск канонического скрипта. Не используем flock здесь — он уже
#    внутри e2e-process.sh (post-tick), и второй launcher просто пройдёт
#    мимо через skip-логику скрипта. Cron scheduler сам даёт нам запуск
#    каждые 20 мин, без перекрытия.
E2E_SCRIPT="/home/builder/.hermes/scripts/agent-flow-e2e-process.sh"
if [ ! -x "$E2E_SCRIPT" ]; then
    echo "❌ launcher: e2e script not executable: $E2E_SCRIPT" >&2
    exit 0
fi

# stdout — deliver, stderr — log (deliver=local → пишется в cron output).
bash "$E2E_SCRIPT" 2>&1