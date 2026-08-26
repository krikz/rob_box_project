#!/usr/bin/env bash
# ============================================================================
# e2e_voice_noisy_gate.sh — pure helpers для audio-noise pre-flight gate
# (Issue #1668, fix-t_67394082).
#
# Этот файл source'ится из e2e_voice_test.sh И из unit-тестов. НЕ выполняет
# main flow, НЕ читает ENV напрямую — все параметры через аргументы.
#
# Контекст: issue #1668 — STT-регрессия e2e (16 раундов подряд FAIL) вызвана
# фоновым голосом в комнате робота 10.1.1.21 (видео/радио). e2e harness жжёт
# ~5-7 минут на раунд и не даёт новой информации, если робот зашумлён.
# Без preflight проверки harness делает 11 шагов по ~30s = 5.5 минут,
# безнадёжно FAILит на wake-gate/no_accept, и e2e-process сжигает CI-минуты
# впустую (см. ADR-0032 §1.2 «Motivation»).
#
# Решение: перед прогоном сценария снимаем N сэмплов audio_rms_dbfs из
# docker logs voice-assistant и считаем скользящее среднее. Если за
# окно NOISY_WINDOW_S секунд среднее > NOISY_RMS_THRESHOLD_DBFS — fail-fast
# с exit 7 (отдельный код для detect_fail_kind noisy-preflight в e2e-process).
#
# Архитектура:
#   - read_audio_rms_logs() — pure read: docker logs → список RMS-сэмплов
#   - compute_avg_rms_dbfs() — pure compute: список → среднее dBFS
#   - noisy_preflight() — orchestrator: preflight() + запись verdict JSON
#   - mark_fail_kind() — sub-hook: вызывается из main flow для тега
#     E2E_NOISY_PREFLIGHT в aggregate fail_kind (→ detect_fail_kind узнаёт
#     о «robot busy/noisy» и классифицирует как infra → e2e:infra-fail,
#     не e2e:rejected).
#
# Env: использует $ROBOT_SSH из e2e_voice_test.sh (или LOGS_FILE для
# unit-тестов). Параметры через CLI-аргументы, не ENV — позволяет
# subshell-стиль тестирования через ROBOT_SSH_OVERRIDE/LOG_FILE.
# ============================================================================

# --- read_audio_rms_logs() --------------------------------------------------
# Читает из docker logs voice-assistant --since <window>s все строки вида
# "📊 [issue 1477] audio_rms_dbfs=<float> ..." и печатает в stdout по
# одной строке на сэмпл: "<epoch>|<rms_dbfs>".
#
# Аргументы:
#   $1 = window_seconds — окно (int seconds, default 30)
# Returns (через stdout):
#   "<ts>|<rms_dbfs>\n<ts>|<rms_dbfs>\n..."  — по одной строке на сэмпл
# Side effects: пишет прогресс в stderr (через printf >&2), чтобы main flow
#   мог его перенаправить в лог. В чистом режиме (для unit-тестов) stderr
#   подавляется через ENV QUIET=1.
read_audio_rms_logs() {
    local window_s="${1:-30}"
    local logs
    if [ -n "${ROBOT_SSH:-}" ]; then
        # Берём последние `window_s` секунд логов. Используем
        # --since <duration>s (формат docker logs). tail-100 строк обычно
        # достаточно для ~30s окна (при ~3 STT-активаций/мин в busy-комнате).
        # Если робот молчит — окно пустое (это OK — preflight PASS).
        logs="$(${ROBOT_SSH} "docker logs voice-assistant --since ${window_s}s 2>&1" 2>/dev/null || echo '')"
    elif [ -n "${LOGS_FILE:-}" ] && [ -f "${LOGS_FILE}" ]; then
        logs="$(cat "${LOGS_FILE}")"
    else
        [ "${QUIET:-0}" != "1" ] && echo "E2E_FATAL: read_audio_rms_logs(): neither ROBOT_SSH nor LOGS_FILE set" >&2
        return 2
    fi
    # Извлекаем пары <iso>|<rms_dbfs> из формата
    # "📊 [issue 1477] audio_rms_dbfs=-37.2 peak_dbfs=... ..."
    # docker logs префиксует каждую строку "<RFC3339> " — но но не всегда, поэтому
    # используем регулярку БЕЗ якоря на начало строки.
    # Force C locale для awk printf, иначе nl_NL.UTF-8 даст "-38,00" вместо
    # "-38.00", что ломает downstream compute_avg_rms_dbfs (regex /^-?\d+\.\d+$/).
    printf '%s\n' "$logs" \
        | grep -oE 'audio_rms_dbfs=[-]?[0-9]+\.[0-9]+' \
        | sed -E 's/audio_rms_dbfs=//' \
        | LC_ALL=C awk -v now="$(date -u +%s)" -v win="$window_s" 'BEGIN{i=0}
            {samples[i++]=$1}
            END {
                if (i==0) { exit 0 }
                step = win / i
                for (j=0; j<i; j++) {
                    printf "%d %.2f\n", (now - win + j*step), samples[j]
                }
            }'
}

# --- compute_avg_rms_dbfs() -------------------------------------------------
# Читает stdin (формат: "<ts> <rms_dbfs>\n...") и печатает среднее арифметическое
# rms_dbfs в stdout (одна строка). Пустой stdin → "-inf" (sentinel: «нет данных»).
#
# Используется в двух местах:
#   1) noisy_preflight() — основной путь
#   2) unit-тесты — для детерминированного теста с подсунутым stdin
compute_avg_rms_dbfs() {
    # Force C locale для printf — иначе nl_NL.UTF-8 даст "0,50" вместо "0.50",
    # что сломает downstream awk-сравнения и JSON парсинг.
    LC_ALL=C awk 'BEGIN{n=0; sum=0.0}
         NF==2 && $2 ~ /^[-+]?[0-9]+\.[0-9]+$/ {
             sum += $2; n++
         }
         END {
             if (n==0) { printf "-inf\n"; exit 0 }
             printf "%.2f\n", sum / n
         }'
}

# --- busy_recent() ----------------------------------------------------------
# Sub-check: есть ли в логах voice-assistant свежие «признаки занятости»:
# активный TTS (Синтез через/TTS finished), проигрывание музыки или просто
# активные STT-активации (audio_rms_dbfs события). Если >BUSY_THRESHOLD
# событий за окно — робот активен (не просто фоновый шум, а обрабатывает
# входы).
#
# Это ВТОРАЯ ОСЬ preflight: даже если RMS ниже threshold, постоянные
# STT-активации (≥16/мин по наблюдению t_6e587508) забивают backlog
# аккумулятор no_wake_word фразами, и e2e wake-gate step получает
# «обот» вместо «Робот» (issue #1668, см. ADR-0027 §5.2).
#
# Аргументы:
#   $1 = window_seconds — окно (default 30)
# Returns:
#   0 — робот активен (busy)
#   1 — робот тихо простаивает
#   2 — probe error
busy_recent() {
    local window_s="${1:-30}"
    local logs
    if [ -n "${ROBOT_SSH:-}" ]; then
        logs="$(${ROBOT_SSH} "docker logs voice-assistant --since ${window_s}s 2>&1" 2>/dev/null || echo '')"
    elif [ -n "${LOGS_FILE:-}" ] && [ -f "${LOGS_FILE}" ]; then
        logs="$(cat "${LOGS_FILE}")"
    else
        [ "${QUIET:-0}" != "1" ] && echo "E2E_FATAL: busy_recent(): neither ROBOT_SSH nor LOGS_FILE set" >&2
        return 2
    fi
    # Считаем маркеры активности. Категории:
    #   - audio_rms_dbfs: STT получил новый фрагмент (любой — wake/no_wake).
    #     BUSY_RMS_RATE — сколько таких событий за окно = «робот постоянно
    #     слышит микрофон → backlog забит».
    #   - Синтез через / TTS finished: робот сейчас говорит.
    #   - Воспроизведение завершено: конец воспроизведения.
    local rms_count tts_count
    rms_count="$(printf '%s\n' "$logs" | grep -c 'audio_rms_dbfs=' || true)"
    tts_count="$(printf '%s\n' "$logs" | grep -cE 'Синтез через|TTS finished|Воспроизведение' || true)"
    # BUSY_RMS_RATE_PER_MIN ≈ (rms_count / window_s) * 60.
    # Эмпирический порог из t_6e587508: ~16/мин background speech → backlog
    # не успевает очищаться. Используем более мягкий порог 10/мин
    # (≈5 событий за 30s окно), чтобы fail-fast срабатывал ДО того, как
    # backlog станет критическим.
    local rms_per_min=$(( (rms_count * 60) / (window_s > 0 ? window_s : 30) ))
    [ "${QUIET:-0}" != "1" ] && printf 'busy_recent: rms_count=%s tts_count=%s rms_per_min=%s\n' \
        "$rms_count" "$tts_count" "$rms_per_min" >&2
    # Busy если:
    #   - RMS-активность ≥10/мин (backlog pressure) ИЛИ
    #   - TTS активен прямо сейчас (робот говорит — не наше время).
    if [ "$rms_per_min" -ge 10 ] || [ "$tts_count" -gt 0 ]; then
        return 0
    fi
    return 1
}

# --- noisy_preflight() ------------------------------------------------------
# Перед прогоном сценария проверяет два условия:
#   (A) busy_recent() — робот активно STT-ит / TTS-ит
#   (B) RMS-dBFS avg > NOISY_RMS_THRESHOLD_DBFS — фон шумнее нормы
#
# Если ЛЮБОЕ срабатывает — fail-fast (exit код возврата != 0). Caller
# (e2e_voice_test.sh) интерпретирует коды:
#   0 — preflight cleared (тихо, можно играть)
#   1 — preflight NOT cleared (robot busy или noisy)
#   2 — probe error (нет доступа к docker logs)
#
# Аргументы:
#   $1 = window_seconds — окно наблюдения (default 30)
#   $2 = state_file — путь, куда писать JSON-verdict (идемпотентно: mkdir -p)
#   $3 = rms_threshold_dbfs — порог RMS в dBFS (default -45, env override
#       NOISY_RMS_THRESHOLD_DBFS). Эмпирический порог из t_6e587508:
#       тихий кабинет = -55..-65, busy-комната = -38..-42.
# Side effects:
#   - пишет JSON в $2
#   - печатает log в stderr (подавляется QUIET=1)
noisy_preflight() {
    local window_s="${1:-30}"
    local state_file="$2"
    local rms_threshold_dbfs="${3:-${NOISY_RMS_THRESHOLD_DBFS:--45}}"
    local checked_at
    checked_at="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    local cleared=0
    local reason=""
    local error_msg="null"
    # Шаг 1: busy_recent probe.
    local busy_rc=0
    busy_recent "$window_s" || busy_rc=$?
    # Шаг 2: RMS avg.
    local rms_avg
    rms_avg="$(read_audio_rms_logs "$window_s" | compute_avg_rms_dbfs)"
    local rms_rc=$?
    # Диагностический stderr-лог (подавляется QUIET=1 для чистого теста).
    if [ "${QUIET:-0}" != "1" ]; then
        printf 'noisy_preflight: window=%ss rms_avg=%s threshold=%s busy_rc=%s rms_rc=%s\n' \
            "$window_s" "$rms_avg" "$rms_threshold_dbfs" "$busy_rc" "$rms_rc" >&2
    fi
    # Вердикт: cleared если busy_rc==1 (тихо) И rms_avg <= threshold.
    if [ "$busy_rc" = "1" ] && awk -v a="$rms_avg" -v t="$rms_threshold_dbfs" \
        'BEGIN{ if (a=="-inf" || a<=t) exit 0; exit 1 }'; then
        cleared=1
        reason="robot quiet (rms_avg=${rms_avg} dBFS <= ${rms_threshold_dbfs}; busy=no)"
    elif [ "$busy_rc" = "0" ]; then
        cleared=0
        reason="robot busy (rms_per_min>=10 or TTS active in ${window_s}s window)"
    elif [ "$rms_avg" != "-inf" ] && awk -v a="$rms_avg" -v t="$rms_threshold_dbfs" \
        'BEGIN{ exit !(a > t) }'; then
        cleared=0
        reason="robot too noisy (rms_dbfs=${rms_avg} > ${rms_threshold_dbfs} threshold for ${window_s}s)"
    else
        # busy_rc==2 или rms_rc==2 → probe error.
        cleared=0
        error_msg="\"probe error: busy_rc=${busy_rc} rms_rc=${rms_rc}\""
        reason="noisy-preflight probe error (no docker logs access)"
    fi
    # Пишем JSON verdict (аналогично run_wake_gate_preflight).
    mkdir -p "$(dirname "$state_file")" 2>/dev/null || true
    printf '{\n  "cleared": %s,\n  "checked_at": "%s",\n  "window_s": %s,\n  "rms_avg_dbfs": "%s",\n  "rms_threshold_dbfs": %s,\n  "reason": "%s",\n  "error": %s,\n  "issue_ref": "#1668"\n}\n' \
        "$([ "$cleared" = "1" ] && echo "true" || echo "false")" \
        "$checked_at" \
        "$window_s" \
        "$rms_avg" \
        "$rms_threshold_dbfs" \
        "$(printf '%s' "$reason" | sed 's/"/\\"/g')" \
        "$error_msg" \
        > "$state_file"
    if [ "$error_msg" != "null" ]; then
        return 2
    fi
    [ "$cleared" = "1" ] && return 0 || return 1
}