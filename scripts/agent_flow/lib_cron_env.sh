#!/bin/bash
# ============================================================================
# lib_cron_env.sh — shared environment preflight for cron-launched scripts.
#
# Ретро t_32997596 (2026-08-28, agent-flow-unlabeled-sweep 19 failures подряд
# + agent-flow-blocked-watchdog gh-auth failed):
#
# Проблема: cron-tick запускает скрипт из workdir (например
# /home/builder/hermes-share/rob_box_project) и:
#   1) передаёт $HERMES_HOME из профиля активной сессии, но он может
#      указывать на /home/builder/.hermes/profiles/<active> (sandbox),
#      а env-файл лежит в /home/builder/.hermes/profiles/agent-flow/.env —
#      путь "<HERMES_HOME>/profiles/agent-flow/.env" разваливается;
#   2) передаёт $HOME из sandbox-HOME (например
#      /home/builder/.hermes/profiles/devops/home), и `gh auth status`
#      ищет конфиг в sandbox, не находит /home/builder/.config/gh/hosts.yml,
#      падает с exit 1;
#   3) не передаёт $GH_REPO (если дефолт в скрипте не задан) — exit 1
#      на guard ": ${GH_REPO:?GH_REPO must be set}".
#
# Этот модуль — НЕ optional precheck, а mandatory preflight. Дёргается
# в самом начале скрипта ДО set -euo pipefail, чтобы любая ошибка env
# была обработана явно (не как exit 1 от guard).
#
# Контракт:
#   source /path/to/lib_cron_env.sh
#   _cron_env_preflight GH_REPO=krikz/rob_box_project
#
# Аргумент 1 — "VAR=default" пары (через пробел), которые должны быть в env
# после preflight. Если в .env значение уже задано — НЕ перезаписываем
# (export-existing-wins семантика).
#
# Поведение:
#   - Если HERMES_HOME указывает на sandbox-профиль (содержит /profiles/<name>),
#     принудительно ставим HERMES_HOME=/home/builder/.hermes
#   - Гарантируем HOME=/home/builder (real user-home, не sandbox)
#   - Гарантируем GH_CONFIG_DIR=/home/builder/.config/gh
#     (там лежит hosts.yml с krikz PAT)
#   - Source-им profiles/agent-flow/.env (если есть) с экспортом только
#     неустановленных переменных (export-existing-wins)
#   - Устанавливаем дефолты из аргументов
#   - Возвращаем exit 0 при успехе, exit 1 при критической ошибке
#
# ВАЖНО: после `set -euo pipefail` в вызывающем скрипте любой fail в этом
# модуле должен быть ОТЛОВЛЕН явно (set +e перед source, set -e после).
# Типичная идиома:
#   set -euo pipefail
#   ...
#   set +e
#   # shellcheck disable=SC1091
#   source "$(dirname "$0")/lib_cron_env.sh" || {
#       echo "[cron-env] FATAL: preflight failed — see log above" >&2
#       exit 1
#   }
#   set -e
#
# Или для optional (когда precheck должен exit-0 с warning):
#   set +e
#   source ... ; _cron_env_preflight_optional ...
#   set -e
#
# Exit codes:
#   0 — preflight ok (env гарантированно содержит нужные переменные)
#   1 — fatal (нет /home/builder, sandbox нечитаем, .env не парсится)
# ============================================================================

# Защита от двойного source: переменная _LIB_CRON_ENV_LOADED
[ -n "${_LIB_CRON_ENV_LOADED:-}" ] && return 0
_LIB_CRON_ENV_LOADED=1

# Stub для `local` keyword (вызывающий может быть в POSIX shell).
# В bash-скриптах работает и так, но оставим для совместимости.
if [ -n "${BASH_VERSION:-}" ]; then
    _lib_cron_env_log() { printf '[cron-env] %s %s\n' "$(date -Iseconds)" "$*" >&2; }
else
    _lib_cron_env_log() { printf '[cron-env] %s %s\n' "$(date -Iseconds)" "$*" >&2; }
fi

# ----------------------------------------------------------------------------
# _cron_env_resolve_home
# Возвращает real-user HOME (даже если вызывающий прислал sandbox-HOME).
# Печатает путь в stdout, exit 1 если /home/builder не существует.
# ----------------------------------------------------------------------------
_cron_env_resolve_home() {
    if [ -d "/home/builder" ]; then
        printf '%s' "/home/builder"
        return 0
    fi
    # fallback: env $HOME, если реальный
    if [ -n "${HOME:-}" ] && [ -d "$HOME" ] && [[ "$HOME" != *"/.hermes/profiles/"* ]]; then
        printf '%s' "$HOME"
        return 0
    fi
    _lib_cron_env_log "FATAL: cannot resolve real HOME (no /home/builder, HOME=$HOME)"
    return 1
}

# ----------------------------------------------------------------------------
# _cron_env_resolve_hermes_home
# Возвращает /home/builder/.hermes (даже если HERMES_HOME был передан как
# /home/builder/.hermes/profiles/<active>). Exit 1 если .hermes не существует.
# ----------------------------------------------------------------------------
_cron_env_resolve_hermes_home() {
    local resolved="/home/builder/.hermes"
    if [ ! -d "$resolved" ]; then
        _lib_cron_env_log "FATAL: $resolved not found — install Hermes first"
        return 1
    fi
    printf '%s' "$resolved"
}

# ----------------------------------------------------------------------------
# _cron_env_source_profile_env <profile>
# Source-ит <profile>/.env если файл существует. Не перезаписывает
# уже-установленные переменные (export-existing-wins). Поддерживает
# комментарии (#) и пустые строки, опциональные кавычки вокруг значения.
# ----------------------------------------------------------------------------
_cron_env_source_profile_env() {
    local profile="$1"
    local hermes_root="${HERMES_HOME:-/home/builder/.hermes}"
    local env_file="$hermes_root/profiles/$profile/.env"

    if [ ! -f "$env_file" ]; then
        _lib_cron_env_log "INFO: $env_file not present — skip source"
        return 0
    fi

    _lib_cron_env_log "INFO: sourcing $env_file (export-existing-wins)"
    # shellcheck disable=SC2163
    while IFS='=' read -r key val; do
        case "$key" in
            ''|\#*) continue ;;
        esac
        # strip surrounding quotes (single or double), один раз с каждой стороны
        val="${val%\"}"; val="${val#\"}"
        val="${val%\'}"; val="${val#\'}"
        # trim trailing whitespace/newlines
        val="${val%%[[:space:]]}"
        if [ -z "$val" ]; then continue; fi
        if [ -z "${!key:-}" ]; then
            export "$key=$val"
        fi
    done < "$env_file"
    return 0
}

# ----------------------------------------------------------------------------
# _cron_env_set_gh_config
# Гарантирует GH_CONFIG_DIR=/home/builder/.config/gh (там hosts.yml с PAT).
# Не перезаписывает уже заданный GH_CONFIG_DIR (если он непустой).
# ----------------------------------------------------------------------------
_cron_env_set_gh_config() {
    local gh_config="/home/builder/.config/gh"
    if [ ! -d "$gh_config" ]; then
        _lib_cron_env_log "WARN: $gh_config not found — gh auth may fail"
        return 0
    fi
    if [ -z "${GH_CONFIG_DIR:-}" ]; then
        export GH_CONFIG_DIR="$gh_config"
    fi
}

# ----------------------------------------------------------------------------
# _cron_env_apply_defaults "VAR=val VAR2=val2 ..."
# Применяет дефолты к переменным, которые после source остались unset.
# Парсит "KEY=VALUE" пары, разделенные пробелами. Поддерживает значение
# в кавычках.
# ----------------------------------------------------------------------------
_cron_env_apply_defaults() {
    for pair in "$@"; do
        case "$pair" in
            *=*)
                local key="${pair%%=*}"
                local val="${pair#*=}"
                val="${val%\"}"; val="${val#\"}"
                val="${val%\'}"; val="${val#\'}"
                if [ -z "${!key:-}" ]; then
                    export "$key=$val"
                    _lib_cron_env_log "INFO: applied default $key=$val"
                fi
                ;;
            *)
                _lib_cron_env_log "WARN: malformed default pair: $pair (expected KEY=VAL)"
                ;;
        esac
    done
}

# ----------------------------------------------------------------------------
# _cron_env_preflight <defaults...>
# Главная entry point. Аргументы — "KEY=VALUE" дефолты.
# Возвращает exit 0 при успехе, exit 1 при критической ошибке.
# ----------------------------------------------------------------------------
_cron_env_preflight() {
    # Step 1: HOME → /home/builder (real, not sandbox)
    local real_home
    real_home="$(_cron_env_resolve_home)" || return 1
    if [ "$HOME" != "$real_home" ] 2>/dev/null; then
        _lib_cron_env_log "INFO: HOME=$HOME -> $real_home"
        export HOME="$real_home"
    fi

    # Step 2: HERMES_HOME → /home/builder/.hermes (not profile-sandbox)
    local real_hermes
    real_hermes="$(_cron_env_resolve_hermes_home)" || return 1
    case "${HERMES_HOME:-}" in
        "$real_hermes") ;;
        "")
            export HERMES_HOME="$real_hermes"
            _lib_cron_env_log "INFO: HERMES_HOME was unset, set to $real_hermes"
            ;;
        *)
            # Если нам передали sandbox-HOME (/.../.hermes/profiles/<x>),
            # принудительно перезаписываем — иначе профильный env-file не найдётся.
            export HERMES_HOME="$real_hermes"
            _lib_cron_env_log "INFO: HERMES_HOME was '${HERMES_HOME:-}', overridden to $real_hermes"
            ;;
    esac

    # Step 3: GH_CONFIG_DIR → /home/builder/.config/gh (if not set)
    _cron_env_set_gh_config

    # Step 4: source profiles/agent-flow/.env (export-existing-wins)
    _cron_env_source_profile_env "agent-flow"

    # Step 5: применить defaults, переданные аргументами
    if [ "$#" -gt 0 ]; then
        _cron_env_apply_defaults "$@"
    fi

    return 0
}

# ----------------------------------------------------------------------------
# _cron_env_gh_auth_preflight [graceful]
# Проверяет `gh auth status`. Exit 0 если ok.
# Если передан аргумент "graceful" — при fail возвращает 0 и печатает WARN
# (для скриптов, которые должны skip-tick при отсутствии auth вместо exit-1).
# Иначе — exit 1 (для скриптов, которые хотят alert-ить cron).
# ----------------------------------------------------------------------------
_cron_env_gh_auth_preflight() {
    local mode="${1:-strict}"
    if gh auth status >/dev/null 2>&1; then
        return 0
    fi
    if [ "$mode" = "graceful" ]; then
        _lib_cron_env_log "WARN: gh auth failed — graceful skip (mode=graceful)"
        return 0
    fi
    _lib_cron_env_log "FATAL: gh auth failed (mode=strict)"
    return 1
}
