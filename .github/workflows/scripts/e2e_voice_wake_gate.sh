#!/usr/bin/env bash
# ============================================================================
# e2e_voice_wake_gate.sh — pure helpers для wake-gate pre-flight diagnostic
# (ADR-0027 §5.2, retro t_be491fba).
#
# Этот файл source'ится из e2e_voice_test.sh И из unit-тестов. НЕ выполняет
# main flow, НЕ читает ENV напрямую — все параметры через аргументы.
#
# Контекст: rounds 215-222 voice_core_suite_v1 показали fail-streak
# 3/3+ на cold-start wake-gate. TRANSCRIPT[dj02] = «стоп музыка» (без
# «Робот»). dj02_stop_music шаг показывает ✅ ПОЛНЫЙ ЦИКЛ + PATTERN_MISS,
# aggregate GATE-1 фейлит "expected tool calls not invoked: stop_music".
#
# Root cause = cold-start wake-gate no_wake_word (не LLM/music race, как
# задокументировано в t_9d229634 misdiagnosis). Pre-flight diagnostic
# разделяет:
#   GATE-0: wake-word detector passed (cold-start cleared)
#   GATE-1: expected tool-calls invoked (acceptance.json contract)
#
# Без wake-gate cleared (cold-start) GATE-1 НИКОГДА не пройдёт для шагов
# с wake-prefix — backlog-аккумулятор по design копит «обот»/«как дела»
# без «Робот» в начало (t_9d229634 retro). Шаг SKIP-ится, не FAIL — это
# корректное поведение, а не баг.
# ============================================================================

# --- wake_gate_cleared_since() ---------------------------------------------
# Возвращает 0 если wake-gate cleared в окне [$1=before_rfc3339, now]:
# в логах voice-assistant есть ЛЮБОЕ 'ПРИНЯТО' событие с wake-префиксом
# (Робот/Робокс) после $1. Это значит wake-word detector уже активирован
# в этой сессии.
#
# Используется в двух местах:
#  1) run_wake_gate_preflight() — перед сценарием, чтобы понять
#     cold-start cleared ли wake-gate. Если нет → skip-режим для шагов
#     с wake-префиксом.
#  2) per-step check внутри run_step() — чтобы каждый step мог проверить
#     «wake-gate всё ещё жив» (защита от деградации mid-scenario).
#
# Env: использует $ROBOT_SSH и $ROBOT_SSH_OVERRIDE из e2e_voice_test.sh.
# Аргументы:
#   $1 = before_rfc3339 — точка отсчёта (ISO8601)
# Returns:
#   0 — wake-gate cleared (хотя бы одно ПРИНЯТО с wake-prefix)
#   1 — wake-gate cold-start not cleared
wake_gate_cleared_since() {
    local before="$1"
    # ROBOT_SSH уже определён в caller (e2e_voice_test.sh) и сам
    # идемпотентен: если ROBOT_SSH_OVERRIDE задан — используется stub.
    # Без ROBOT_SSH (например, при ручном импорте из юнит-теста без
    # source основного скрипта) — fallback на прямой echo, чтобы тесты
    # могли подсунуть свой stub через переменную окружения LOGS_FILE.
    local logs
    if [ -n "${ROBOT_SSH:-}" ]; then
        logs="$(${ROBOT_SSH} "docker logs voice-assistant --since '${before}' 2>&1" 2>/dev/null || echo '')"
    elif [ -n "${LOGS_FILE:-}" ] && [ -f "${LOGS_FILE}" ]; then
        logs="$(cat "${LOGS_FILE}")"
    else
        echo "E2E_FATAL: wake_gate_cleared_since(): neither ROBOT_SSH nor LOGS_FILE set" >&2
        return 2
    fi
    # ПРИНЯТО с wake-префиксом. Wake-слова в rob_box: «Робот» (по дефолту)
    # и «Робокс» (альтернативный wake-word из issue #1252). Маркер
    # '✅ ПРИНЯТО' печатается stt_node.py при приёме wake-фразы.
    # Используем grep -F (fixed string) с двумя паттернами через -e,
    # чтобы избежать unicode-quirks BRE/ERE с кириллицей (Робок?с в -E
    # трактуется как «Робо + опц. к + с», что не совпадает с «Робот»).
    printf '%s' "$logs" | grep -qF "✅ ПРИНЯТО: Робот" \
        || printf '%s' "$logs" | grep -qF "✅ ПРИНЯТО: Робокс"
}

# --- detect_wake_word_in_text() --------------------------------------------
# Проверяет, начинается ли текст с wake-префикса. Используется в
# scenario-парсере чтобы выставить expect=wake-gated для шагов с wake.
#
# Wake-words: «Робот» (по дефолту) и «Робокс» (альтернативный, #1252).
# Это ДВА РАЗНЫХ слова, не «Робо + опциональный к + с». Используем bash
# case (без grep), чтобы не зависеть от unicode-quirks BRE/ERE — grep -E
# трактует «к?» как квантор «0 или 1 раз» и ломается на кириллице.
#
# Аргументы:
#   $1 = text — текст команды (например, "Робот, как дела")
# Returns:
#   0 — есть wake-prefix (текст начинается с «Робот» или «Робокс»)
#   1 — нет wake-prefix
detect_wake_word_in_text() {
    local text="${1:-}"
    # strip leading whitespace, then test fixed prefix
    local stripped="${text#"${text%%[![:space:]]*}"}"
    case "$stripped" in
        Робот*|Робокс*) return 0 ;;
        *) return 1 ;;
    esac
}

# --- classify_step_expect() ------------------------------------------------
# Маппинг step.expect → нормализованный kind. Используется в
# scenario-парсере и в run_step для switch-логики.
#
# Аргументы:
#   $1 = step_expect (raw value из scenario.json)
#   $2 = step_text   (для детекции wake-prefix если expect не задан)
# Returns (через stdout, одна строка):
#   cycle           — обычный полный цикл (ожидание LLM+TTS)
#   wake-gated      — нужен wake-gate preflight cleared (default для
#                     команд с wake-prefix)
#   backlog         — ожидание backlog-аккумулятора (фразы без wake)
#   unknown         — нераспознанный expect; fail-safe на cycle
classify_step_expect() {
    local expect_raw="${1:-}"
    local text="${2:-}"
    case "$expect_raw" in
        cycle|"")
            # Если expect пустой — auto-detect по wake-prefix (catches
            # стандартный кейс voice_core_suite_v1 без необходимости
            # явно ставить expect="wake-gated" в каждый шаг).
            # Если expect явно "cycle" — trust caller'а: НЕ auto-promote,
            # даже если текст начинается с wake. Это позволяет
            # тестировать чистый cycle flow на wake-тексте.
            if [ -z "$expect_raw" ] && detect_wake_word_in_text "$text"; then
                printf '%s' "wake-gated"
            else
                printf '%s' "cycle"
            fi
            ;;
        wake-gated|wake_gated)  printf '%s' "wake-gated" ;;
        backlog)                 printf '%s' "backlog" ;;
        *)                       printf '%s' "cycle" ;;
    esac
}

# --- run_wake_gate_preflight() ---------------------------------------------
# Перед прогоном сценария проверяет, что wake-gate cold-start cleared.
# Если ROBOT_SSH доступен — реальная проверка docker logs.
#
# ВАЖНО: НЕ играет никаких команд. Это read-only probe (проверяет
# существующие логи), не активный warm-up. Поведение «прогреть микрофон»
# делает сам робот (audio_node cold-start); мы только детектируем факт.
#
# Аргументы:
#   $1 = before_rfc3339 — точка отсчёта (ISO8601)
#   $2 = wake_gate_state_file — путь, куда писать JSON-вердикт
# Returns:
#   0 — cold-start cleared (wake-gate passed)
#   1 — cold-start not cleared (no ПРИНЯТО с wake-prefix в окне)
#   2 — probe error (нет доступа к docker logs, network и т.п.)
# Side effects:
#   Пишет JSON в $2:
#     {"cleared": true|false, "checked_at": "<iso>", "before": "<iso>",
#      "reason": "<text>", "error": "<text|null>"}
run_wake_gate_preflight() {
    local before="$1"
    local state_file="$2"
    local checked_at
    checked_at="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    local cleared=0
    local reason=""
    local error_msg="null"
    if wake_gate_cleared_since "$before"; then
        cleared=1
        reason="wake-gate already cleared (ПРИНЯТО with wake-prefix observed since ${before})"
    else
        cleared=0
        reason="wake-gate cold-start NOT cleared (no ПРИНЯТО with wake-prefix since ${before})"
    fi
    # Проверяем probe error: если ни cleared ни reason сменились — это
    # значит wake_gate_cleared_since вернула 2 (FATAL).
    # Перехватываем через отдельный probe-error check: если нет
    # доступа к docker logs — фиксируем как probe_error.
    if [ -z "${ROBOT_SSH:-}" ] && [ -z "${LOGS_FILE:-}" ]; then
        cleared=0
        error_msg="\"no ROBOT_SSH/LOGS_FILE — cannot probe\""
        reason="wake-gate preflight probe error"
    fi
    # Пишем JSON verdict (error_msg: либо "null", либо строка в кавычках;
    # reason в кавычках с экранированием внутренних " через sed).
    mkdir -p "$(dirname "$state_file")" 2>/dev/null || true
    # Используем printf для JSON-форматирования (безопаснее, чем heredoc
    # с интерполяцией — heredoc не экранирует спец-символы в reason/error).
    printf '{\n  "cleared": %s,\n  "checked_at": "%s",\n  "before": "%s",\n  "reason": "%s",\n  "error": %s\n}\n' \
        "$([ "$cleared" = "1" ] && echo "true" || echo "false")" \
        "$checked_at" \
        "$before" \
        "$(printf '%s' "$reason" | sed 's/"/\\"/g')" \
        "$error_msg" \
        > "$state_file"
    if [ "$error_msg" != "null" ]; then
        return 2
    fi
    [ "$cleared" = "1" ] && return 0 || return 1
}
