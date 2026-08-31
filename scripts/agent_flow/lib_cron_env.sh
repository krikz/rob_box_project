#!/bin/bash
# ============================================================================
# lib_cron_env.sh — mandatory env preflight для cron-launched no-agent скриптов.
#
# SOT (source-of-truth): <repo>/scripts/agent_flow/lib_cron_env.sh
# На хост раскладывает `bash <repo>/scripts/agent_flow/install.sh` — hardlink
# (cp -al), НЕ симлинками: симлинк наружу из ~/.hermes/scripts/ отклоняется
# guard'ом hermes-agent scheduler.py::_validate_script_path (ретро 11.08
# t_a6a236e0d9f0470e). Список путей раскладки — в install.sh::EXPECTED.
#
# ============================================================================
# ЗАЧЕМ (ретро t_a2521b07, 31.08 — cron no-agent env fragile):
#
#   Cron-tick в devops-профиле запускает no-agent скрипт с workdir
#   (например /home/builder/hermes-share/rob_box_project), и:
#     1) передаёт $HOME из sandbox-оболочки профиля, например
#        /home/builder/.hermes/profiles/devops/home (НЕ реальный user-HOME).
#        `gh auth status` ищет конфиг в $HOME/.config/gh/hosts.yml → не
#        находит (там только mimeapps.list, pulse, renardo) → exit 1 →
#        "blocked-watchdog: gh auth failed — exit 1" (16-fail подряд,
#        карточка t_a2521b07).
#     2) передаёт $HERMES_HOME из sandbox-оболочки профиля, например
#        /home/builder/.hermes/profiles/devops — env-файл лежит в
#        /home/builder/.hermes/profiles/agent-flow/.env, и путь
#        "<HERMES_HOME>/profiles/agent-flow/.env" разваливается →
#        GH_REPO/KANBAN_BOARD пустые → guard `: ${GH_REPO:?...}` → exit 1 →
#        "agent-flow-unlabeled-sweep: GH_REPO must be set" (24-fail подряд,
#        карточка t_9b0d60f7).
#     3) $GH_REPO не задан (если .env не загрузился) — exit 1.
#     4) $GH_CONFIG_DIR не задан — gh может попытаться прочитать из
#        sandbox-HOME и упасть.
#
#   Параллельные наблюдения (одинаковый паттерн, разные скрипты):
#     - agent-flow-unlabeled-sweep (24 fail подряд, fix t_9b0d60f7) — ENV_FILE
#       fallback через HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}",
#       но это копипаста в каждом скрипте, а не общий механизм.
#     - agent-flow-blocked-watchdog (16 fail подряд, fix t_a2521b07) — даже
#       не пытался подгрузить .env, сразу `gh auth status` → exit 1.
#
# ============================================================================
# КОНТРАКТ:
#
#   # 1. Опционально: подгружаем env ДО set -e (безопасный путь).
#   #    Это позволяет catch'ить fail явным `|| { echo FATAL; exit 1; }`.
#   _cron_env_try_source() {
#       local lib_dir
#       lib_dir="$(cd "$(dirname "${BASH_SOURCE[1]:-$0}")" && pwd)"
#       # shellcheck source=lib_cron_env.sh
#       . "$lib_dir/lib_cron_env.sh"
#   }
#   set +e
#   _cron_env_try_source || { echo "[cron-env] FATAL: preflight failed" >&2; exit 1; }
#   set -e
#
#   После source доступны (все FORCE, кроме export-existing-wins из .env):
#     - HOME=/home/builder (real, не sandbox)
#     - GH_CONFIG_DIR=/home/builder/.config/gh (там hosts.yml с krikz PAT)
#     - HERMES_HOME=/home/builder/.hermes (не profile-sandbox)
#     - GH_REPO, KANBAN_BOARD, REPO_DIR, MAINTENANCE_* — из
#       /home/builder/.hermes/profiles/agent-flow/.env (export-existing-wins)
#     - _CRON_ENV_PREFLIGHT_OK=1 (marker для тестов)
#
#   Если /home/builder не существует → exit 1 (невозможно продолжать).
#   Если .env не существует → WARN в stderr, но не fail (caller сам решит).
#
# ============================================================================
# ЧЕГО ЭТОТ МОДУЛЬ НЕ ДЕЛАЕТ:
#
#   - Не запускает `gh auth status` (это делает вызывающий скрипт — guard
#     на свой exit code, чтобы можно было graceful-skip vs hard-fail).
#   - Не создаёт flock — это забота вызывающего скрипта.
#   - Не пишет "starting preflight" в лог — тихая работа; логи — у caller'а.
#
# ============================================================================
# ПОВЕДЕНИЕ:
#
#   - HERMES_HOME, HOME, GH_CONFIG_DIR — FORCE (export, перезаписываем).
#     Это by design: cron передаёт sandbox-HOME → нам НЕ нужно его уважать,
#     мы хотим credentials реального юзера.
#   - GH_REPO, KANBAN_BOARD, REPO_DIR, MAINTENANCE_* — из .env, но
#     export-existing-wins (если caller уже задал — оставляем).
#   - Defaults (safety-net) только если unset после source:
#       GH_REPO=krikz/rob_box_project
#       KANBAN_BOARD=robbox
#       REPO_DIR=/home/builder/hermes-share/rob_box_project
#   - Возвращаем 0 при успехе, 1 при критической ошибке (например,
#     /home/builder не существует или нет прав на чтение).
# ============================================================================

# Защита от двойного source: переменная _LIB_CRON_ENV_LOADED.
# Если уже загружен — ничего не делаем (идемпотентность).
if [ -n "${_LIB_CRON_ENV_LOADED:-}" ]; then
    return 0 2>/dev/null || true
fi
_LIB_CRON_ENV_LOADED=1

# ---- 1. Real-user paths (FORCE) -------------------------------------------
# Реальный HOME юзера (НЕ sandbox). Все credentials лежат тут.
REAL_HOME="/home/builder"
if [ ! -d "$REAL_HOME" ]; then
    printf '[lib_cron_env] FATAL: real HOME not found: %s\n' "$REAL_HOME" >&2
    return 1 2>/dev/null || exit 1
fi

# FORCE (НЕ `${VAR:-default}` — иначе остаётся sandbox-HERMES_HOME/HOME).
export HOME="$REAL_HOME"
export GH_CONFIG_DIR="$HOME/.config/gh"
export HERMES_HOME="/home/builder/.hermes"

# Sanity-check GH_CONFIG_DIR — если там нет hosts.yml, gh auth не пройдёт.
# Не валим preflight (caller сам решит, hard-fail или graceful-skip),
# но warn в stderr (это видно в cron delivery).
if [ ! -f "$GH_CONFIG_DIR/hosts.yml" ]; then
    printf '[lib_cron_env] WARN: GH_CONFIG_DIR=%s has no hosts.yml — gh auth will fail\n' \
        "$GH_CONFIG_DIR" >&2
fi

# ---- 2. Profile .env (export-existing-wins) -------------------------------
# Подгружаем /home/builder/.hermes/profiles/agent-flow/.env если есть.
# Только unset переменные (export-existing-wins): если caller уже задал
# (например через cron env) — не перезаписываем.
ENV_FILE="$HERMES_HOME/profiles/agent-flow/.env"
if [ -f "$ENV_FILE" ]; then
    _cron_env_load_env() {
        local _key _val
        while IFS='=' read -r _key _val; do
            # strip comments / blank lines
            case "$_key" in ''|'#'*) continue ;; esac
            # strip surrounding quotes (одинарные и двойные), один раз
            _val="${_val%\"}"; _val="${_val#\"}"
            _val="${_val%\'}"; _val="${_val#\'}"
            # export-existing-wins: skip если уже задано в env
            if [ -z "${!_key:-}" ]; then
                # shellcheck disable=SC2086
                export "$_key=$_val"
            fi
        done < "$ENV_FILE"
    }
    _cron_env_load_env
    unset -f _cron_env_load_env
else
    printf '[lib_cron_env] WARN: %s not present — defaults will be used\n' \
        "$ENV_FILE" >&2
fi

# ---- 3. Defaults (safety-net) --------------------------------------------
# Минимальный safety-net для guard'ов вида `: ${GH_REPO:?...}`.
# Реальный default — krikz/rob_box_project (основной репо процесса).
export GH_REPO="${GH_REPO:-krikz/rob_box_project}"
export KANBAN_BOARD="${KANBAN_BOARD:-robbox}"
export REPO_DIR="${REPO_DIR:-/home/builder/hermes-share/rob_box_project}"

# ---- 4. Marker для дедуп / тестов ----------------------------------------
# Помечаем факт успешного source'а — тесты могут проверять эту переменную.
export _CRON_ENV_PREFLIGHT_OK=1

return 0 2>/dev/null || true
