#!/bin/bash
# ============================================================================
# voice_e2e_dialog.sh — регрессионный e2e-тест для issue #933
#
#   «voice/tts: Silero fallback тоже падает на 1005 chars — нужен
#    per-provider max chunk + retry-halve»
#
# Сценарий (тот же, что в разделе «Воспроизведение» issue #933):
#   1. Прямая инъекция STT-фразы «робок расскажи длинный анекдот»
#      в /voice/stt/result — LLM (DeepSeek) отвечает ДЛИННЫМ текстом
#      (>1000 chars).
#   2. tts_node дробит ответ на per-provider chunks
#      (yandex=250 / silero=800 / minimax=5000, config voice_assistant.yaml)
#      и синтезирует КАЖДЫЙ chunk отдельно.
#   3. Проверяем ПОЛНЫЙ цикл в логах voice-assistant:
#        ✅ «Воспроизведение завершено» — робот доиграл ВЕСЬ ответ
#        ✅ нет «Too long text»   (Yandex gRPC INVALID_ARGUMENT)
#        ✅ нет «Synthesis error» (Silero v5 "Model couldn't generate...")
#        ✅ LLM OUTPUT присутствует (диалог реально прошёл)
#
# До bug-фикса (#931 + #933): Yandex падал на ~291 chars, Silero fallback
# падал на ~1005 chars → робот молчал (thinking.mp3 → тишина). Скрипт
# красный, если в логах снова появились эти ошибки или цикл не завершился.
#
# Usage:
#   ./voice_e2e_dialog.sh [ROBOT_HOST]
#
# Env:
#   ROBOT_HOST          IP робота (default 10.1.1.21)
#   ROBOT_USER          SSH user (default ros2)
#   SSHPASS             SSH password (default open)
#   PHASE_TEXT          STT-фраза (default 'робок расскажи длинный анекдот')
#   REACTION_TIMEOUT_S  Сколько ждать полный цикл (default 150)
#   LLM_MIN_CHARS       Ниже этой длины ответа тест считается inconclusive
#                       (default 500 — баг #933 проявлялся на >~500 chars)
#
# Exit codes:
#   0 — PASS (полный цикл, ошибок нет)
#   2 — FAIL (Too long text / Synthesis error / нет «Воспроизведение завершено»)
#   3 — SKIP (LLM ответил коротко — регрессия #933 не воспроизводится)
#   4 — ENV/SSH ошибка
# ============================================================================
set -u

ROBOT_HOST="${ROBOT_HOST:-10.1.1.21}"
ROBOT_USER="${ROBOT_USER:-ros2}"
SSHPASS="${SSHPASS:-open}"
PHASE_TEXT="${PHASE_TEXT:-робок расскажи длинный анекдот}"
REACTION_TIMEOUT_S="${REACTION_TIMEOUT_S:-150}"
LLM_MIN_CHARS="${LLM_MIN_CHARS:-500}"

# SSH-префикс. ВАЖНО (ретро t_0a5d65af): locale-префикс нельзя класть в
# переменную — bash выполняет его как команду. Только литералом.
ROBOT_SSH="sshpass -p ${SSHPASS} ssh -o StrictHostKeyChecking=no ${ROBOT_USER}@${ROBOT_HOST}"

log() { echo ">>> $*"; }
die() { echo "FATAL: $*" >&2; exit 4; }

log "=== voice_e2e_dialog.sh: регрессия issue #933 (per-provider chunking) ==="
log "ROBOT_HOST=${ROBOT_HOST}  PHASE_TEXT='${PHASE_TEXT}'"

# ── 1. Доступность робота и контейнера ─────────────────────────────────────
${ROBOT_SSH} "docker ps --format '{{.Names}}' 2>/dev/null | grep -q '^voice-assistant$'" \
    || die "voice-assistant контейнер не найден на ${ROBOT_HOST} (docker ps)"

log "✓ voice-assistant запущен на ${ROBOT_HOST}"

# ── 2. Отметка BEFORE и инъекция STT-фразы ─────────────────────────────────
BEFORE="$(${ROBOT_SSH} "date -u +%Y-%m-%dT%H:%M:%SZ" 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
log "BEFORE=${BEFORE}"
log "Инъекция STT: ${PHASE_TEXT}"

# Цитируем фразу так, чтобы она пережила: локальный bash → ssh → bash -c → ros2.
# Простая фраза без кавычек по умолчанию; при необходимости задавайте PHASE_TEXT
# одним словом-аргументом (см. --ros-args ниже).
INJECT_CMD="bash -lc 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash 2>/dev/null; ros2 topic pub --once /voice/stt/result std_msgs/msg/String \"data: ${PHASE_TEXT}\" 2>&1'"
if ! ${ROBOT_SSH} "docker exec voice-assistant ${INJECT_CMD}" >/tmp/voice_e2e_inject.log 2>&1; then
    log "⚠️  ros2 topic pub вернул ненулевой rc — смотрим, дошла ли фраза через логи"
    tail -5 /tmp/voice_e2e_inject.log >&2
fi

# ── 3. Ожидание полного цикла (акцепт → LLM → TTS → воспроизведение) ───────
log "Ожидание полного цикла (${REACTION_TIMEOUT_S}s)..."
START_EPOCH="$(date +%s)"
FOUND_PLAYBACK=0
EARLY_FAIL=0

while [ $(( $(date +%s) - START_EPOCH )) -lt "$REACTION_TIMEOUT_S" ]; do
    LOGS="$(${ROBOT_SSH} "docker logs voice-assistant --since '${BEFORE}' 2>&1" 2>/dev/null || echo '')"

    # Ранний fail: провайдер отверг длинный текст (баг #933 возвращается).
    if printf '%s' "$LOGS" | grep -q "Too long text"; then
        echo "FAIL_MARKER: Too long text (Yandex gRPC INVALID_ARGUMENT) в логах"
        EARLY_FAIL=1
        break
    fi
    if printf '%s' "$LOGS" | grep -q "Synthesis error"; then
        echo "FAIL_MARKER: Synthesis error (Silero v5) в логах"
        EARLY_FAIL=1
        break
    fi

    if printf '%s' "$LOGS" | grep -q "Воспроизведение завершено"; then
        FOUND_PLAYBACK=1
        break
    fi
    sleep 5
done

# ── 4. Анализ логов ────────────────────────────────────────────────────────
LOGS="$(${ROBOT_SSH} "docker logs voice-assistant --since '${BEFORE}' 2>&1" 2>/dev/null || echo '')"

echo ""
echo "──────────────────────────────────────────────────────────────"
echo "  Сводка логов (с ${BEFORE}):"
echo "──────────────────────────────────────────────────────────────"

LLM_OUT="$(printf '%s' "$LOGS" | grep -E "LLM OUTPUT|Turn done. Response" | tail -1)"
if [ -n "$LLM_OUT" ]; then
    echo "  LLM:  $LLM_OUT" | head -c 300
    echo ""
else
    echo "  LLM:  (LLM OUTPUT не найден)"
fi

printf '%s' "$LOGS" | grep -E "Синтез через|chunk\(s\)|Воспроизведение завершено" | tail -5 \
    | sed 's/^/  TTS:  /'

# Считаем длину ответа LLM: последний «LLM OUTPUT: '...'» или «Response: '...'».
LLM_LEN=0
if [ -n "$LLM_OUT" ]; then
    LLM_LEN="$(printf '%s' "$LLM_OUT" | sed -E "s/.*[‘'\"](.*)[’'\"]/\1/" | wc -m | tr -d ' ')"
fi

# ── 5. Вердикт ──────────────────────────────────────────────────────────────
FAIL_REASONS=""
[ "$EARLY_FAIL" = "1" ] && FAIL_REASONS="${FAIL_REASONS} провайдерский error в логах"
[ "$FOUND_PLAYBACK" = "0" ] && FAIL_REASONS="${FAIL_REASONS} нет «Воспроизведение завершено»"

if [ -n "$FAIL_REASONS" ]; then
    echo ""
    echo "❌ FAIL:${FAIL_REASONS}"
    printf '%s' "$LOGS" | grep -E "Too long text|Synthesis error|Синтез через|Воспроизведение|TTS finished" | tail -10 \
        | sed 's/^/     /'
    exit 2
fi

# Ответ мог быть коротким — тогда регрессия #933 не воспроизводилась
# (баг проявлялся на >~500 chars). Это SKIP, а не PASS.
if [ "$LLM_LEN" -lt "$LLM_MIN_CHARS" ]; then
    echo ""
    echo "⚠️  SKIP: LLM ответил коротко (${LLM_LEN} chars < ${LLM_MIN_CHARS}) — регрессия #933 не воспроизведена"
    echo "    Робот доиграл ответ без ошибок, но длинный текст не проверен."
    exit 3
fi

echo ""
echo "✅ PASS: полный цикл для длинного ответа (${LLM_LEN} chars) — без Too long text / Synthesis error"
exit 0
