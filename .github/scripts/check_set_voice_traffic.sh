#!/usr/bin/env bash
# ============================================================================
# check_set_voice_traffic.sh — регрессионный скрипт (issue #2138.C / ADR-0067)
#
# Источник истины: <repo>/.github/scripts/check_set_voice_traffic.sh
# Копия на runner (self-hosted) обновляется через checkout — НЕ через
# hardlink/symlink. Шаблон дистрибуции совпадает с check_container_status.sh
# (вызывается из .github/workflows/L-Deploy and Verify.yml).
#
# ЗАЧЕМ: ADR-0067 / issue #2138 §3.3 фиксирует 6 гипотез пропадающих смен
# голоса в picker'е. Чтобы сузить H1..H6 при инциденте, нужно логировать
# pub/sub-факт «picker выдал /avatar/set_voice» на стороне робота. Мост
# логирует «publishing…»/«published…» (issue #2138.C, см. C1), но это
# ВСЁ ЕЩЁ может быть погашено по дороге (DDS discovery, supervisor died).
#
# Этот скрипт — наблюдатель с другой стороны: «а есть ли вообще трафик
# на /avatar/set_voice за последнюю минуту». Если hz==0 при живом
# picker'е (ws_server принимает команды из Quest WebXR) — пишем в
# FINDINGS_JSONL alert с severity=critical.
#
# Входные env (как у check_container_status.sh):
#   PI_HOST, PI_USER, PI_PASSWORD, PI_COMPOSE_DIR, PI_SSH_OPTS — SSH-параметры.
#   HZ_WINDOW        — секунд замера (default=10, чтобы не блокировать
#                      workflow; ADR §3.3 упоминает 60s как «идеал», но
#                      runner это сразу превращает в >1m healthcheck —
#                      вместо этого hz-анализ запускается в фоне на Pi).
#   TOPIC            — ROS topic для мониторинга (default=/avatar/set_voice).
#   FINDINGS_FILE    — JSONL sink для findings (совместимо с остальным pipeline).
#   ENVIRONMENT, SCOPE — прокидываются в findings.
#
# Выход: JSON в stdout ``{healthy, hz, samples, reason}`` и HEALTHY_RESULT/
#        HZ_RESULT через export (workflow парсит через ``source``).
#
# Exit code 0 (по аналогии с check_container_status.sh: workflow
# проверяет HEALTHY_RESULT).
# ============================================================================
set -euo pipefail

HZ_WINDOW="${HZ_WINDOW:-10}"
TOPIC="${TOPIC:-/avatar/set_voice}"

# Required env
: "${PI_HOST:?PI_HOST must be set (e.g. 10.1.1.11)}"
: "${PI_USER:?PI_USER must be set (e.g. ros2)}"
: "${PI_PASSWORD:?PI_PASSWORD must be set (sshpass -p)}"
: "${PI_COMPOSE_DIR:?PI_COMPOSE_DIR must be set (e.g. ~/rob_box_project/docker/main)}"
: "${PI_SSH_OPTS:?PI_SSH_OPTS must be set (ssh options string)}"

# Optional env
SUMMARY_FILE="${SUMMARY_FILE:-/tmp/${PI_HOST//./_}_set_voice_summary.txt}"
FINDINGS_JSONL="${FINDINGS_FILE:-/dev/null}"

log() { printf '[check_set_voice_traffic] %s %s\n' "$(date -Iseconds)" "$*" >&2; }

# Безопасный порог: «есть хоть один message за окно». hz=0.001 в пересчёте
# на окно в 10s означает 0.01 сообщений — нереалистично мало, picker
# шлёт редкие команды (не поток). Реальные сценарии «живой picker,
# но traffic отсутствует» дают hz=0.0; «была одна команда 30s назад»
# на 60s-окне даёт hz ≈ 0.0167. Используем 0.001 как compromise — ловит
# «совсем пусто», не срабатывает на «была команда за время окна».
HZ_ALERT_THRESHOLD="${HZ_ALERT_THRESHOLD:-0.001}"

# --- Снять замер трафика на /avatar/set_voice -------------------------------
#
# ROS 2 CLI: `ros2 topic hz <TOPIC>` крутится бесконечно. Чтобы получить
# конечный результат, оборачиваем в `timeout` и читаем stdout.
#
# Формат вывода (Humble/Iron):
#   average rate: 0.0167
#         min: 30.000s max: 30.000s std dev: 0.000s window: 60
# (последняя строка — итоговая статистика, парсим через grep+sed)
#
# NB: на некоторых версиях ROS 2 последняя строка содержит "average rate:"
# ПЛЮС дополнительную инфу. Берём ПОСЛЕДНЮЮ строку с "average rate".
measure_hz() {
    local hz_output
    hz_output="$(sshpass -p "$PI_PASSWORD" ssh $PI_SSH_OPTS "${PI_USER}@${PI_HOST}" \
        "timeout ${HZ_WINDOW}s ros2 topic hz ${TOPIC} 2>/dev/null || true" \
        2>/dev/null || true)"
    printf '%s' "$hz_output"
}

log "🔍 Measuring /avatar/set_voice traffic on ${PI_HOST} (window=${HZ_WINDOW}s)"
HZ_OUTPUT="$(measure_hz)"

if [ -z "$HZ_OUTPUT" ]; then
    log "(empty hz output — node 'ros2 topic hz' не выполнился или тема не существует)"
    HZ=0.0
    SAMPLES=0
else
    # Парсим последнюю строку «average rate: X.XXXX». ROS пишет её при
    # graceful shutdown или по EOF; на некоторых версиях прибивается
    # внутри ``timeout`` SIGTERM'ом, и тогда строка может быть
    # неполной — тогда считаем трафик нулевым.
    HZ_LINE="$(printf '%s\n' "$HZ_OUTPUT" | grep -E 'average rate:' | tail -n 1 || true)"
    if [ -n "$HZ_LINE" ]; then
        HZ="$(printf '%s' "$HZ_LINE" | sed -E 's/.*average rate:[[:space:]]+([0-9.eE+-]+).*/\1/' || true)"
        SAMPLES="$(printf '%s\n' "$HZ_OUTPUT" | grep -c 'subscribed to' || true)"
    else
        # Если нет «average rate:» — был ли хоть один sample? ``subscribed to``
        # печатается один раз при старте. Если строка вообще не появилась,
        # значит ``ros2 topic hz`` завис / упал / тема не опубликована ни разу.
        if printf '%s' "$HZ_OUTPUT" | grep -q 'subscribed to'; then
            HZ=0.0
            SAMPLES=0
        else
            HZ=0.0
            SAMPLES=0
        fi
    fi
fi

# Sanitize — если парсинг не удался, HZ может оказаться пустым.
: "${HZ:=0.0}"

# Округляем до 4 знаков для стабильного вывода (через awk).
HZ_DISPLAY="$(awk -v h="$HZ" 'BEGIN { printf "%.4f", h+0 }' 2>/dev/null || echo '0.0000')"

# --- verdict ----------------------------------------------------------------
HEALTHY_BOOL="true"
REASON=""

# Используем awk для float comparison, т.к. bash не умеет.
IS_BELOW_THRESHOLD="$(awk -v h="$HZ_DISPLAY" -v t="$HZ_ALERT_THRESHOLD" \
    'BEGIN { print (h+0 < t+0) ? "1" : "0" }')"

if [ "$IS_BELOW_THRESHOLD" = "1" ]; then
    HEALTHY_BOOL="false"
    REASON="topic_hz_below_threshold: ${HZ_DISPLAY} Hz < ${HZ_ALERT_THRESHOLD} Hz (window=${HZ_WINDOW}s, topic=${TOPIC})"
fi

if [ "$HEALTHY_BOOL" = "false" ]; then
    log "❌ $REASON"
    {
        echo "=== $TOPIC on $PI_HOST ==="
        echo "Reason: $REASON"
        echo "Window: ${HZ_WINDOW}s"
        echo "Samples observed: ${SAMPLES}"
        echo "Latest hz output (truncated):"
        printf '%s\n' "$HZ_OUTPUT" | tail -n 5
        echo ""
    } >> "$SUMMARY_FILE"

    # Findings для остального pipeline (формат совпадает с тем, что workflow
    # пишет руками в *_findings.jsonl).
    if [ "$FINDINGS_JSONL" != "/dev/null" ]; then
        HZ_INFO="$(printf '%s' "$HZ_OUTPUT" | tail -n 3 | tr '\n' ';' | head -c 500 || true)"
        HZ_INFO="${HZ_INFO%;}"
        jq -nc \
            --arg container "ros-box-quest" \
            --arg kind "voice_picker_traffic" \
            --arg severity "critical" \
            --arg summary "Picker voice-change traffic отсутствует на ${TOPIC}" \
            --arg raw_text "$REASON. Window=${HZ_WINDOW}s, samples=${SAMPLES}. Last: $HZ_INFO" \
            '{environment:env.ENVIRONMENT, scope:env.SCOPE, container:$container, kind:$kind, severity:$severity, summary:$summary, raw_text:$raw_text}' \
            >> "$FINDINGS_JSONL" 2>/dev/null || true
    fi
else
    log "✅ traffic OK: hz=${HZ_DISPLAY} samples=${SAMPLES} window=${HZ_WINDOW}s"
fi

# Печатаем JSON для workflow (он парсит через jq).
jq -nc \
    --argjson healthy "$( [ "$HEALTHY_BOOL" = "true" ] && echo true || echo false )" \
    --arg hz "$HZ_DISPLAY" \
    --argjson samples "$SAMPLES" \
    --arg reason "${REASON:-}" \
    --arg window "$HZ_WINDOW" \
    --arg topic "$TOPIC" \
    '{healthy: $healthy, hz: ($hz | tonumber), samples: $samples, reason: $reason, window: $window, topic: $topic}'

# Через env, переданный через source, отдаём обратно в workflow.
export HEALTHY_RESULT="$HEALTHY_BOOL"
export HZ_RESULT="$HZ_DISPLAY"
export SAMPLES_RESULT="$SAMPLES"

# Дефолтный exit code = 0 (workflow проверяет HEALTHY_RESULT).
exit 0
