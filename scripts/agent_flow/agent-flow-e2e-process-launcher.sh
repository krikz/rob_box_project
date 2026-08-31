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
#
# Ретро 28.08 (t_faac94b0, e2e-fail-streak-no-escalation): после тика
# ротации вызываем fail-streak watchdog — если L: E2E Voice Test падает
# 5+ раз подряд, watchdog постит issue-comment; при 20+ создаёт pause-
# sentinel (заморозка ротации до ручного override). Без этого fail-streak
# 24 раунда подряд (~3 дня) прошёл без алерта (issue #1668 46h+ open
# без process-меток).
# ============================================================================

# ============================================================================
# Env preflight (ретро t_a2521b07, cron no-agent env fragile): FORCE'им
# HOME/HERMES_HOME/GH_CONFIG_DIR реального юзера, иначе `gh auth token`
# ниже пытается читать из sandbox-HOME → null → GH_TOKEN пустой →
# downstream agent-flow-e2e-process.sh падает на `gh auth status`.
set +e
# shellcheck source=lib_cron_env.sh
. "$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)/lib_cron_env.sh" || {
    printf "[%s] %s: lib_cron_env preflight failed — exit 1\n" \
        "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$(basename "${BASH_SOURCE[0]:-$0}")" >&2
    exit 1
}
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

# 2.5 Defaults for fail-streak escalation (ретро 28.08 t_4ead2dd4).
#   AUTO_NEEDS_REVIEW_ON_FAIL_STREAK — порог streak для auto-установки
#     needs-review на готовые PR (mergeStateStatus=CLEAN + Raw-evidence).
#     Параллельно E2E_FAIL_STREAK_WARN из PR #1721 watchdog для согласованных
#     алертов в одном тике (issue-comment + needs-review).
#   E2E_FAIL_STREAK_WARN / E2E_FAIL_STREAK_PAUSE — пороги watchdog'а из PR #1721
#     (если он разложен install.sh; в develop его ещё нет — задаются для
#     будущей совместимости, fallback'ом в e2e-process.sh).
: "${AUTO_NEEDS_REVIEW_ON_FAIL_STREAK:=5}"
: "${E2E_FAIL_STREAK_WARN:=5}"
: "${E2E_FAIL_STREAK_PAUSE:=20}"
export AUTO_NEEDS_REVIEW_ON_FAIL_STREAK E2E_FAIL_STREAK_WARN E2E_FAIL_STREAK_PAUSE

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
# set +e вокруг bash — ошибка ротации НЕ должна блокировать watchdog.
bash "$E2E_SCRIPT" 2>&1 || true

# 4. Fail-streak watchdog (ретро 28.08 t_faac94b0). Каждый тик (every 20m)
#    проверяем streak последних E2E runs. При streak ≥ WARN → issue comment
#    в процесс-релевантные issue; при streak ≥ PAUSE → создаём sentinel
#    (заморозка ротации, manual override). Идемпотентен: comment dedup 6h.
WATCHDOG_SCRIPT="/home/builder/.hermes/scripts/agent-flow-e2e-fail-streak-watchdog.sh"
if [ ! -x "$WATCHDOG_SCRIPT" ]; then
    # Fallback: возможно SOT ещё не разложен install.sh — берём прямо из репо.
    WATCHDOG_SCRIPT="${REPO_DIR:-/home/builder/hermes-share/rob_box_project}/scripts/agent_flow/agent-flow-e2e-fail-streak-watchdog.sh"
fi
if [ -x "$WATCHDOG_SCRIPT" ]; then
    bash "$WATCHDOG_SCRIPT" 2>&1 || \
        echo "⚠️ launcher: fail-streak watchdog failed (exit $?) — продолжим" >&2
else
    echo "⚠️ launcher: fail-streak watchdog not found: $WATCHDOG_SCRIPT" >&2
fi