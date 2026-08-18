#!/bin/bash
# ============================================================================
# e2e_voice_test.sh — атомарный голосовой e2e-тест rob_box_project (v2, 10.08)
#
# Отличия от e2e_remote.sh (шляпа):
#  1. Синтез команды Yandex TTS НА ЛЕТУ (голос + текст выбирает запускающий),
#     а не закоммиченный .ogg.
#  2. Атомарный цикл: play → ждём ПОЛНЫЙ цикл в логах робота
#     (STT ПРИНЯТО → LLM INPUT → LLM ответ → TTS finished → Воспроизведение
#     завершено). Шаг зелёный только когда цикл завершился целиком.
#  3. Таймаут акцепта (STT не принял команду) → повтор команды сам, N попыток.
#  4. Отказ LLM/TTS (429, Empty, error) → тест КРАСНЕЕТ сразу, не чинит.
#  5. Сценарии: JSON-файл со шагами, каждый шаг — свой голос + свой текст +
#     ожидаемые паттерны в логах (проверка фичи, напр. save_speaker_profile).
#
# Usage:
#   e2e_voice_test.sh --text "Робот, меня зовут Саша" [--voice anton]
#   e2e_voice_test.sh --scenario /tmp/scenario.json
#   e2e_voice_test.sh --text "..." --voice ermil --retries 3 --react-window 40
#
# Env (обязательные): YANDEX_API_KEY, ROBOT_HOST=10.1.1.21, SSHPASS
# Env (опциональные): E2E_MAX_ATTEMPTS=3, E2E_REACTION_WINDOW=40,
#                     E2E_RETRY_PAUSE=10, E2E_RECORD_EXTRA=20,
#                     GITHUB_RUN_ID (для OUT_DIR и симлинка
#                                    /tmp/dialog_e2e_<run_id>.wav —
#                                    workflow передаёт ${{ github.run_id }})
#
# Output (на запускающем хосте 249):
#   /tmp/e2e_v2_<run_id>/{model.json, scenario.json, step_N.log,
#                          recording.wav, verdict.txt}
#   Симлинк: /tmp/dialog_e2e_${RUN_ID}.wav -> recording.wav
#            (контракт для workflow L-E2E Voice Test.yml артефакта
#             e2e-voice-recording-<run_id>)
#   stdout: "E2E_STEP <N> OK|FAIL|SKIP" + "E2E_VERDICT PASS|FAIL"
# ============================================================================
set -u

# --- defaults ---------------------------------------------------------------
E2E_MAX_ATTEMPTS="${E2E_MAX_ATTEMPTS:-3}"
E2E_REACTION_WINDOW="${E2E_REACTION_WINDOW:-40}"   # сек ждём полный цикл после play
E2E_RETRY_PAUSE="${E2E_RETRY_PAUSE:-10}"
E2E_RECORD_EXTRA="${E2E_RECORD_EXTRA:-15}"          # хвост записи после реакции
E2E_RECORDING="${E2E_RECORDING:-1}"                 # 1 = писать микрофон (issue #1353)
ROBOT_HOST="${ROBOT_HOST:-10.1.1.21}"
ROBOT_USER="${ROBOT_USER:-ros2}"
# GITHUB_RUN_ID — передаётся из workflow L-E2E Voice Test.yml (${{ github.run_id }}),
# используется как RUN_ID (→ OUT_DIR и симлинк /tmp/dialog_e2e_<run_id>.wav).
# Если не задан (локальный запуск) — RUN_ID будет локальный timestamp. См. issue #1353.
GITHUB_RUN_ID="${GITHUB_RUN_ID:-}"
# NOTE (retro t_0a5d65af, round-50): НЕЛЬЗЯ ставить "LC_ALL=C " префиксом в
# ROBOT_SSH — при раскрытии ${ROBOT_SSH} bash выполняет "LC_ALL=C" как КОМАНДУ
# (rc=127 command not found), весь ROBOT_SSH возвращает пусто, check_cycle
# видит пустые логи и выдаёт no_accept при живом роботе. Locale префикс
# работает только как литерал перед командой, не через переменную.
ROBOT_SSH="sshpass -p ${SSHPASS:-open} ssh -n -o StrictHostKeyChecking=no ${ROBOT_USER}@${ROBOT_HOST}"
YANDEX_TTS_VOICE="${YANDEX_TTS_VOICE:-anton}"       # голос по умолчанию
YANDEX_SPEED="${YANDEX_SPEED:-1.0}"

# RUN_ID — уникальный идентификатор прогона. Используем github.run_id если
# передан (workflow L-E2E Voice Test.yml передаёт GITHUB_RUN_ID в env), иначе
# локальный timestamp. Это позволяет workflow'у надёжно находить OUT_DIR и
# забирать все артефакты через конкретный путь /tmp/e2e_v2_<run_id>/...
# (issue #1353).
RUN_ID="${GITHUB_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
OUT_DIR="/tmp/e2e_v2_${RUN_ID}"
RECORDING_RAW="/tmp/e2e_raw_${RUN_ID}.pcm"
RECORDING_WAV="${OUT_DIR}/recording.wav"
mkdir -p "$OUT_DIR"

# --- gating (issue #1385): lock-файл + ROS topic /voice/e2e/busy -----------
# Пока идёт e2e, dialogue_node слушает wake word через тот же ReSpeaker.
# Если параллельно идёт любой другой трафик (музыка, телеграм, муздрецы) —
# робот отвечает на чужой wake word и ответ попадает в recording.
# Решение: атомарный харнесс публикует /voice/e2e/busy=True в начале и
# =False в конце; dialogue_node подписан и в busy-режиме игнорирует wake word
# (issue #1385 acceptance: bus flag + ROS topic + trap cleanup + watchdog timeout).
E2E_LOCK_FILE="/tmp/e2e_running.lock"
E2E_BUSY_TIMEOUT_S="${E2E_BUSY_TIMEOUT_S:-300}"  # watchdog: e2e > 5 мин → автоснятие
acquire_e2e_busy() {
    # Идемпотентно: если lock уже есть (повторный запуск e2e_voice_test.sh
    # в одном тике) — оставляем, но НЕ публикуем занятно (предыдущий инстанс
    # ещё держит). Иначе counter в dialogue_node собьётся. Снимает lock тот
    # инстанс, который его создал (PID в lock-файле).
    local my_pid=$$
    if [ -f "$E2E_LOCK_FILE" ]; then
        local existing_pid
        existing_pid="$(cat "$E2E_LOCK_FILE" 2>/dev/null || echo 0)"
        if [ -n "$existing_pid" ] && kill -0 "$existing_pid" 2>/dev/null; then
            log "WARN: $E2E_LOCK_FILE уже занят PID=$existing_pid — не публикую busy повторно"
            E2E_LOCK_OWNER=0
            return 0
        fi
        # Stale lock (предыдущий процесс умер без cleanup) — снимаем и ставим свой.
        log "WARN: $E2E_LOCK_FILE stale (PID=$existing_pid мёртв) — перезаписываю"
        rm -f "$E2E_LOCK_FILE"
    fi
    printf '%s\n' "$my_pid" > "$E2E_LOCK_FILE"
    E2E_LOCK_OWNER=1
    # Publish ROS topic /voice/e2e/busy=True на роботе (запускающий хост 249
    # может и не быть роботом — публикуем через SSH в voice-assistant).
    # Используем тот же паттерн что voice_e2e_dialog.sh (source ros humble
    # + ros2 topic pub --once).
    ${ROBOT_SSH} \
        "docker exec voice-assistant bash -lc 'source /opt/ros/humble/setup.bash && \
         source /ws/install/setup.bash 2>/dev/null; \
         ros2 topic pub --once /voice/e2e/busy std_msgs/msg/Bool \"data: true\" 2>&1'" \
        > "$OUT_DIR/e2e_busy_publish.log" 2>&1 || \
        log "WARN: ros2 topic pub /voice/e2e/busy=true failed (см. $OUT_DIR/e2e_busy_publish.log)"
    log "✅ E2E BUSY: lock=$E2E_LOCK_FILE (PID=$my_pid), /voice/e2e/busy=True опубликован"
}
release_e2e_busy() {
    if [ "${E2E_LOCK_OWNER:-0}" != "1" ]; then
        # Не owner — не снимаем lock другого процесса
        return 0
    fi
    # Убиваем watchdog (он ещё может спать до 5 мин)
    if [ -n "${E2E_WATCHDOG_PID:-}" ]; then
        kill "$E2E_WATCHDOG_PID" 2>/dev/null || true
    fi
    rm -f "$E2E_LOCK_FILE"
    # Publish /voice/e2e/busy=False (best-effort: если SSH упал — watchdog
    # через E2E_BUSY_TIMEOUT_S сам подчистит lock-файл).
    ${ROBOT_SSH} \
        "docker exec voice-assistant bash -lc 'source /opt/ros/humble/setup.bash && \
         source /ws/install/setup.bash 2>/dev/null; \
         ros2 topic pub --once /voice/e2e/busy std_msgs/msg/Bool \"data: false\" 2>&1'" \
        > "$OUT_DIR/e2e_busy_release.log" 2>&1 || true
    log "✅ E2E BUSY: lock снят, /voice/e2e/busy=False опубликован"
}
# Ставим busy СРАЗУ (до parse args / синтеза) — даже TTS-синтез 30 сек не
# должен позволить wake-word из чужого теста перебить. trap'ом гарантируем
# снятие при любом exit (включая сигнал SIGTERM/SIGINT от watchdog).
E2E_LOCK_OWNER=0
acquire_e2e_busy
trap 'release_e2e_busy' EXIT INT TERM
# Watchdog: если e2e висит дольше E2E_BUSY_TIMEOUT_S — снимаем lock и busy
# (на случай если процесс был убит без trap'а — SIGKILL, OOM, segfault).
(
    sleep "$E2E_BUSY_TIMEOUT_S"
    if [ -f "$E2E_LOCK_FILE" ]; then
        log "⚠️ WATCHDOG: $E2E_LOCK_FILE > ${E2E_BUSY_TIMEOUT_S}s — принудительное снятие"
        rm -f "$E2E_LOCK_FILE"
        ${ROBOT_SSH} \
            "docker exec voice-assistant bash -lc 'source /opt/ros/humble/setup.bash && \
             source /ws/install/setup.bash 2>/dev/null; \
             ros2 topic pub --once /voice/e2e/busy std_msgs/msg/Bool \"data: false\" 2>&1'" \
            >/dev/null 2>&1 || true
    fi
) &
E2E_WATCHDOG_PID=$!

# --- самовосстановление артефакт-дира (ретро 11.08 t_26a6d362) -------------
# Параллельный infra-cleanup на 249 (t_0a5d65af) удалял /tmp/e2e_v2_* ВО ВРЕМЯ
# прогона → paplay open(): No such file → ложный FAIL (round-49, run 31544057593).
# Каждая запись в OUT_DIR идёт через ensure_outdir — дир пересоздаётся, если
# её удалили между шагами. Перед play файл пере-синтезируется, если пропал.
ensure_outdir() {
    if [ ! -d "$OUT_DIR" ]; then
        log "WARN: $OUT_DIR удалён (внешний cleanup?) — пересоздаю"
        mkdir -p "$OUT_DIR"
    fi
}

# --- запись микрофона (issue #1353) ------------------------------------------
# Старый e2e_remote.sh писал parec в /tmp/e2e_raw.pcm → /tmp/dialog_e2e_<id>.wav
# (контракт артефакта e2e-voice-recording-<run_id>). Atomic v2 (10.08) запись
# убрал → workflow L-E2E Voice Test.yml:381-387 пытается загрузить файл,
# которого нет, и молча пропускает артефакт (if-no-files-found: ignore).
# Фикс: пишем ВЕСЬ retry-цикл (все попытки всех шагов) в $RECORDING_RAW, в
# конце конвертируем в $RECORDING_WAV и делаем симлинк на путь, который
# ждёт workflow.
#
# Best-effort: если parec/ffmpeg отсутствуют — e2e НЕ падает (запись — это
# артефакт для пост-анализа, не критерий приёма). Если запись короткая
# (<1KB) — пишем warning в лог, но тест идёт дальше.
REC_PID=""
start_recording() {
    if [ "$E2E_RECORDING" != "1" ]; then
        log "RECORDING: выключен через E2E_RECORDING=0"
        return 0
    fi
    if ! command -v parec >/dev/null 2>&1; then
        log "WARN RECORDING: parec не найден (Katana без PulseAudio?) — пропускаю запись"
        return 0
    fi
    # Длительность: все попытки всех шагов + хвост. С запасом — scenario с 3
    # шагами по 3 попытки = 9 циклов; +E2E_RECORD_EXTRA секунд тишины после
    # последней реакции. timeout корректно убивает parec (SIGTERM → EOF),
    # дальше ffmpeg доводит raw PCM до валидного WAV.
    # Используем дефолтный source (PulseAudio @DEFAULT_SOURCE@) — он же
    # использовался в старом e2e_remote.sh.
    local max_steps="${E2E_MAX_STEPS:-5}"
    local total_secs=$(( E2E_MAX_ATTEMPTS * max_steps * (E2E_REACTION_WINDOW + E2E_RETRY_PAUSE) + E2E_RECORD_EXTRA + 30 ))
    log "RECORDING: старт parec → ${RECORDING_RAW} (timeout ${total_secs}s)"
    rm -f "$RECORDING_RAW"
    # shellcheck disable=SC2086
    timeout "$total_secs" parec --format=s16le --channels=1 --rate=16000 "$RECORDING_RAW" \
        >/tmp/e2e_rec_$$.log 2>&1 &
    REC_PID=$!
    # Дать parec ~1s подняться, иначе первые 0.5с могут пропасть
    sleep 1
    if ! kill -0 "$REC_PID" 2>/dev/null; then
        log "WARN RECORDING: parec не запустился ($(tail -1 /tmp/e2e_rec_$$.log))"
        REC_PID=""
        return 0
    fi
    log "RECORDING: pid=${REC_PID}"
}

# shellcheck disable=SC2329  # вызывается через trap 'stop_recording' EXIT
stop_recording() {
    if [ -z "$REC_PID" ]; then return 0; fi
    if ! kill -0 "$REC_PID" 2>/dev/null; then
        wait "$REC_PID" 2>/dev/null || true
        REC_PID=""
        return 0
    fi
    log "RECORDING: stop (SIGTERM pid=${REC_PID})"
    kill -TERM "$REC_PID" 2>/dev/null || true
    # Дать parec корректно закрыть pipe (graceful shutdown → корректный EOF)
    for _ in 1 2 3 4 5; do
        if ! kill -0 "$REC_PID" 2>/dev/null; then break; fi
        sleep 1
    done
    wait "$REC_PID" 2>/dev/null || true
    REC_PID=""

    ensure_outdir
    if [ ! -s "$RECORDING_RAW" ]; then
        log "WARN RECORDING: ${RECORDING_RAW} пустой (parec умер?) — запись не будет загружена"
        return 0
    fi
    # Конвертация raw PCM → WAV (как в e2e_remote.sh:55-89). Гарантирует
    # корректный RIFF header даже если SIGTERM пришёл раньше EOF.
    if ! command -v ffmpeg >/dev/null 2>&1; then
        log "WARN RECORDING: ffmpeg не найден — запись остаётся в raw PCM"
        return 0
    fi
    if ffmpeg -y -f s16le -ar 16000 -ac 1 -i "$RECORDING_RAW" "$RECORDING_WAV" 2>/dev/null; then
        local size
        size=$(stat -c %s "$RECORDING_WAV" 2>/dev/null || echo 0)
        log "RECORDING_DONE: ${RECORDING_WAV} (${size} bytes)"
        rm -f "$RECORDING_RAW"
        # Симлинк для совместимости со старым контрактом workflow.
        # /tmp/dialog_e2e_${RUN_ID}.wav — путь, который ждёт
        # actions/upload-artifact (L-E2E Voice Test.yml:385). RUN_ID берётся
        # из GITHUB_RUN_ID если передан, иначе timestamp (issue #1353).
        local compat_path="/tmp/dialog_e2e_${RUN_ID}.wav"
        ln -sfn "$RECORDING_WAV" "$compat_path"
        log "RECORDING_LINK: ${compat_path} -> ${RECORDING_WAV}"
    else
        log "WARN RECORDING: ffmpeg-конвертация не удалась — оставляю raw PCM"
    fi
}

# --- parse args -------------------------------------------------------------
TEXT=""
SCENARIO_FILE=""
VOICE=""
PATTERNS=""
CHECK_TG_ECHO=0
while [ $# -gt 0 ]; do
    case "$1" in
        --text)     TEXT="$2"; shift 2 ;;
        --voice)    VOICE="$2"; shift 2 ;;
        --scenario) SCENARIO_FILE="$2"; shift 2 ;;
        --patterns) PATTERNS="$2"; shift 2 ;;
        --retries)  E2E_MAX_ATTEMPTS="$2"; shift 2 ;;
        --react-window) E2E_REACTION_WINDOW="$2"; shift 2 ;;
        --check-tg-echo) CHECK_TG_ECHO=1; shift 1 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

if [ -z "${YANDEX_API_KEY:-}" ]; then
    echo "E2E_FATAL: YANDEX_API_KEY не задан" >&2; exit 2
fi
if [ -z "$TEXT" ] && [ -z "$SCENARIO_FILE" ]; then
    echo "E2E_FATAL: нужен --text или --scenario" >&2; exit 2
fi

# --- helpers ----------------------------------------------------------------
log() { echo ">>> $*"; }

# Синтез Yandex TTS gRPC v3: text + voice → /tmp/e2e_v2_<run>/cmd.wav
# Тот же контракт что tts_node._synthesize_yandex (tts.api.cloud.yandex.net:443)
synth_yandex() {  # $1=text $2=voice $3=out_wav
    local text="$1" voice="$2" out="$3"
    python3 - "$text" "$voice" "$out" <<'PY'
import sys, grpc, wave, io
from yandex.cloud.ai.tts.v3 import tts_pb2, tts_service_pb2_grpc
text, voice, out = sys.argv[1], sys.argv[2], sys.argv[3]
import os
key = os.environ["YANDEX_API_KEY"]
ch = grpc.secure_channel("tts.api.cloud.yandex.net:443", grpc.ssl_channel_credentials())
stub = tts_service_pb2_grpc.SynthesizerStub(ch)
req = tts_pb2.UtteranceSynthesisRequest(
    text=text,
    output_audio_spec=tts_pb2.AudioFormatOptions(
        container_audio=tts_pb2.ContainerAudio(container_audio_type=tts_pb2.ContainerAudio.WAV)
    ),
    hints=[tts_pb2.Hints(voice=voice), tts_pb2.Hints(speed=float(os.environ.get("YANDEX_SPEED","1.0")))],
    loudness_normalization_type=tts_pb2.UtteranceSynthesisRequest.LUFS,
)
resp = stub.UtteranceSynthesis(req, metadata=(("authorization", f"Api-Key {key}"),))
data = b""
for r in resp:
    data += r.audio_chunk.data
if not data:
    sys.exit("YANDEX_EMPTY")
with open(out, "wb") as f:
    f.write(data)
print(f"YANDEX_SYNTH_OK {len(data)} bytes voice={voice}")
PY
}

# Проверка полного цикла в логах робота с момента BEFORE.
# Возвращает: 0 = полный цикл (акцепт+LLM+TTS в ПРАВИЛЬНОМ ПОРЯДКЕ),
#             1 = нет акцепта, 2 = LLM/TTS error
check_cycle() {  # $1=before_rfc3339
    local before="$1"
    local logs
    logs="$(${ROBOT_SSH} "docker logs voice-assistant --since '${before}' 2>&1" 2>/dev/null || echo '')"
    # 1) акцепт STT
    if ! printf '%s' "$logs" | grep -q "ПРИНЯТО"; then
        return 1   # нет акцепта → retry команды
    fi
    # 2) LLM ошибки. "Empty assistant response|LLM.*(error|failed)" — красный,
    #    не чиним. 429/quota — НЕ красный: minimax квота (2056) исчерпана
    #    постоянно, но fallback-цепочка на deepseek (PR #1099) в develop
    #    работает — цикл завершается на следующем провайдере. Если TTS уже
    #    есть — fallback сработал, не ошибка; если TTS нет — retry (return 1).
    if printf '%s' "$logs" | grep -qE "Empty assistant response|LLM.*(error|failed)"; then
        printf '%s' "$logs" | grep -E "Empty assistant response|LLM.*(error|failed)" | tail -3 > "$OUT_DIR/llm_error.txt"
        return 2
    fi
    if printf '%s' "$logs" | grep -qE "429 Too Many|quota"; then
        if ! printf '%s' "$logs" | grep -q "TTS finished"; then
            printf '%s' "$logs" | grep -E "429|quota" | tail -3 > "$OUT_DIR/llm_quota.txt"
            log "⚠️ minimax 429/quota в логах, TTS не завершился — retry (fallback deepseek)"
            return 1
        fi
    fi
    # 3) ПОРЯДОК: LLM INPUT команды должен быть ПОСЛЕ ПРИНЯТО,
    #    а TTS finished — ПОСЛЕ LLM INPUT (иначе это приветствие/старый цикл)
    #    Issue #1127: берём ПОСЛЕДНИЙ TTS finished после accept_ts
    #    (раньше head -1 брал самый первый = от приветствия/старого цикла,
    #    что давало stale_cycle и ложный NO_ACCEPT).
    local accept_ts llm_ts tts_ts
    accept_ts="$(printf '%s' "$logs" | grep 'ПРИНЯТО' | tail -1 | grep -oE '\[[0-9]+\.[0-9]+\]' | tail -1 | tr -d '[]')"
    llm_ts="$(printf '%s' "$logs" | grep 'LLM INPUT' | tail -1 | grep -oE '\[[0-9]+\.[0-9]+\]' | tail -1 | tr -d '[]')"
    # TTS finished должен быть СТРОГО ПОСЛЕ accept_ts — иначе это приветствие
    if [ -n "$accept_ts" ]; then
        tts_ts="$(printf '%s' "$logs" | grep 'TTS finished' | awk -v acc="$accept_ts" '
            {
                match($0, /\[[0-9]+\.[0-9]+\]/);
                ts = substr($0, RSTART+1, RLENGTH-2);
                if (ts+0 > acc+0) { print ts; exit }
            }')"
    fi
    # fallback: если ROS timestamp не спарсился — берём wall-clock из docker logs
    if [ -z "$tts_ts" ]; then
        tts_ts="$(printf '%s' "$logs" | grep 'TTS finished' | head -1 | grep -oE '[0-9]{2}:[0-9]{2}:[0-9]{2}' | head -1)"
    fi
    if [ -z "$accept_ts" ] || [ -z "$tts_ts" ]; then
        return 1
    fi
    # LLM INPUT может отсутствовать (DJ/короткий ответ) — но если есть,
    # TTS должен быть ПОСЛЕ него. В любом случае TTS должен быть ПОСЛЕ акцепта.
    if [ -n "$llm_ts" ]; then
        if awk "BEGIN{exit !($tts_ts > $llm_ts)}"; then
            : # tts после llm — ок
        else
            printf '%s' "$logs" | grep -E "ПРИНЯТО|LLM INPUT|TTS finished" | tail -5 > "$OUT_DIR/stale_cycle.txt"
            return 1   # TTS ДО LLM INPUT = приветствие/старый цикл, не реакция
        fi
    fi
    if awk "BEGIN{exit !($tts_ts > $accept_ts)}"; then
        printf '%s' "$logs" | grep -E "ПРИНЯТО|LLM INPUT|TTS finished|Воспроизведение завершено" | tail -8 > "$OUT_DIR/cycle_log.txt"
        return 0
    fi
    printf '%s' "$logs" | grep -E "ПРИНЯТО|LLM INPUT|TTS finished" | tail -5 > "$OUT_DIR/stale_cycle.txt"
    return 1   # TTS ДО акцепта = приветствие
}

# Проверка ожидаемых паттернов (для фич, напр. speaker_analysis)
# $1=before, $2..=паттерны (grep -E). Печатает найденные, rc=0 если все найдены.
check_patterns() {
    local before="$1"; shift
    local logs pat rc=0
    logs="$(${ROBOT_SSH} "docker logs voice-assistant --since '${before}' 2>&1" 2>/dev/null || echo '')"
    for pat in "$@"; do
        if printf '%s' "$logs" | grep -qE "$pat"; then
            echo "  PATTERN_OK: $pat"
        else
            echo "  PATTERN_MISS: $pat"
            rc=1
        fi
    done
    # Issue #1134: return bash-convention (0=success, 1=fail). Внутренняя
    # ``rc=0`` означала success — возвращаем как есть, callers через
    # ``if check_patterns; then`` получат SUCCESS при rc=0 и FAIL при rc=1.
    return $rc
}

# Проверка эха диалога в Telegram (issue #1196, L2).
#
# После голосового e2e проверяем, что telegram_node получил ответ
# dialogue_node. Без внешнего Telegram — дёшево, ловит разрыв
# диалог↔бот. Признак: счётчик ``telegram_message_total{out,...}`` на
# :9101/metrics вырос (бот отправил сообщение) ИЛИ в логах telegram-bot
# есть вызов send_message. Если метрики недоступны (prometheus_client
# не установлен / старый образ) — fallback на логи, SKIP если нет
# вообще никаких признаков эха (это ок: голосовой e2e не обязан
# отправлять сообщения в чат; проверка — полуавтомат для RUN_NOW).
#
# Usage: check_telegram_echo <before_rfc3339>
# Возвращает: 0 = эхо подтверждено (метрика out>0 или send_message в логах),
#             1 = эха нет (не баг — просто не было отправки)
#             2 = telegram_node недоступен (SSH/метрики) — SKIP
check_telegram_echo() {
    local before="$1"
    local tg_logs tg_metrics out_count

    # 1. Метрики telegram-bot (:9101) — самый надёжный признак.
    tg_metrics="$(${ROBOT_SSH} "docker exec telegram-bot python3 -c \"
import urllib.request
try:
    data = urllib.request.urlopen('http://localhost:9101/metrics', timeout=5).read().decode()
except Exception as e:
    print('METRICS_UNAVAILABLE', e)
    raise SystemExit(0)
for line in data.splitlines():
    if line.startswith('telegram_message_total') and 'direction=\"out\"' in line:
        print(line)
\" 2>/dev/null" 2>/dev/null || echo '')"
    if [ -z "$tg_metrics" ]; then
        log "TG_ECHO: метрики недоступны (нет prometheus_client / старый образ) — fallback на логи"
    else
        out_count="$(printf '%s\n' "$tg_metrics" | grep 'direction="out"' | grep -oE '[0-9]+$' | head -1)"
        if [ -n "$out_count" ] && [ "${out_count:-0}" -gt 0 ]; then
            log "TG_ECHO: ✅ telegram_message_total{out,...} = ${out_count} (бот отправлял сообщения)"
            return 0
        fi
        log "TG_ECHO: метрика out=0 (голосовой e2e обычно не шлёт сообщения в чат) — смотрим логи"
    fi

    # 2. Логи telegram-bot: наличие send_message / dialogue response echo.
    #    Для голосового e2e без активного чата эхо не уходит, но
    #    telegram_node ПРИНИМАЕТ /voice/dialogue/response и логирует
    #    "Dropping dialogue echo" — это и есть доказательство связи
    #    диалог↔бот (L2 ловит именно разрыв на этом участке).
    tg_logs="$(${ROBOT_SSH} "docker logs telegram-bot --since '${before}' 2>&1" 2>/dev/null || echo '')"
    if printf '%s' "$tg_logs" | grep -qiE "send_message|response_echo|dialogue.*echo|Echo.*chat|Dropping dialogue echo|Failed to echo"; then
        log "TG_ECHO: ✅ telegram_node получил ответ dialogue_node (send_message / echo evidence)"
        printf '%s\n' "$tg_logs" | grep -iE "send_message|response_echo|dialogue.*echo|Echo.*chat|Dropping dialogue echo|Failed to echo" | tail -3 > "$OUT_DIR/tg_echo_evidence.txt" 2>/dev/null || true
        return 0
    fi

    # 3. Ни метрик, ни логов — сервис может быть не готов/не в этом контейнере.
    if ! ${ROBOT_SSH} "docker ps --filter name=telegram-bot --format '{{.Status}}'" 2>/dev/null | grep -q Up; then
        log "TG_ECHO: ⚠️ контейнер telegram-bot не запущен — SKIP"
        return 2
    fi
    log "TG_ECHO: эха нет (метрика out=0, send_message в логах нет) — голосовой e2e это допускает"
    return 1
}

# --- один атомарный шаг -----------------------------------------------------
run_step() {  # $1=text $2=voice $3=step_label
    local text="$1" voice="$2" label="$3"
    log "=== STEP ${label}: voice=${voice} text=\"${text}\" ==="

    # 1. Синтез команды
    ensure_outdir
    if [ ! -f "$OUT_DIR/cmd_${label}.wav" ]; then
        # Файл мог пропасть вместе с OUT_DIR (внешний cleanup на 249) —
        # пере-синтезируем, а не падаем с paplay open(): No such file.
        log "STEP ${label}: cmd_${label}.wav отсутствует — повторный синтез (cleanup-resilience)"
    fi
    if ! synth_yandex "$text" "$voice" "$OUT_DIR/cmd_${label}.wav" > "$OUT_DIR/synth_${label}.log" 2>&1; then
        log "STEP ${label}: FAIL — синтез Yandex упал ($(tail -1 "$OUT_DIR/synth_${label}.log"))"
        echo "E2E_STEP ${label} FAIL synth"
        return 1
    fi
    # EQ: highpass 200 + volume 1.2 + alimiter (клиппинг-фикс 514e7e87)
    ensure_outdir
    ffmpeg -y -i "$OUT_DIR/cmd_${label}.wav" -af "highpass=f=100,volume=3.0,alimiter=limit=0.98,adelay=1500|all=1" -ac 1 -ar 16000 "$OUT_DIR/cmd_${label}_eq.wav" 2>/dev/null

    # 2. Ждём тишины: робот не должен говорить перед командой (greeting/
    #    приветствие идёт через 12s после старта и может перебить команду).
    #    Ждём пока в логах нет свежих TTS-событий последние E2E_SILENCE_WAIT сек.
    E2E_SILENCE_WAIT="${E2E_SILENCE_WAIT:-15}"
    local quiet_start quiet_end
    quiet_start="$(date -u +%s)"
    while true; do
        quiet_end="$(date -u +%s)"
        if [ $((quiet_end - quiet_start)) -ge "$E2E_SILENCE_WAIT" ]; then
            break
        fi
        local tts_recent
        tts_recent="$(${ROBOT_SSH} "docker logs voice-assistant --since 20s 2>&1 | grep -cE 'Синтез через|TTS finished|Воспроизведение' 2>/dev/null | tail -1" 2>/dev/null || echo 0)"
        tts_recent="$(printf '%s' "$tts_recent" | grep -oE '[0-9]+' | tail -1)"
        if [ "$tts_recent" = "0" ] || [ -z "$tts_recent" ]; then
            : # тихо — копим окно тишины
        else
            quiet_start="$(date -u +%s)"  # робот говорит — сбрасываем окно
        fi
        sleep 5
    done
    log "STEP ${label}: робот молчит ${E2E_SILENCE_WAIT}s — команду можно играть"

    # 3. Retry-цикл акцепта
    local attempt reaction rc
    reaction=0
    for attempt in $(seq 1 "$E2E_MAX_ATTEMPTS"); do
        BEFORE="$(${ROBOT_SSH} "date -u +%Y-%m-%dT%H:%M:%SZ" 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
        log "STEP ${label}: PLAY attempt ${attempt}/${E2E_MAX_ATTEMPTS}"
        # Katana: громкость динамика 150% (по VOICE_COMMANDS_RESEARCH.md —
        # 100% даёт -42dB на микрофоне, wake word теряется)
        pactl set-sink-volume @DEFAULT_SINK@ 150% 2>/dev/null || true
        # cleanup-resilience (ретро 11.08 t_26a6d362): если eq-файл пропал
        # (OUT_DIR удалён внешним cleanup на 249) — пере-синтезируем и EQ,
        # а не получаем ложный FAIL от paplay open(): No such file.
        if [ ! -f "$OUT_DIR/cmd_${label}_eq.wav" ]; then
            log "STEP ${label}: cmd_${label}_eq.wav отсутствует перед play — пере-синтез (cleanup-resilience)"
            ensure_outdir
            synth_yandex "$text" "$voice" "$OUT_DIR/cmd_${label}.wav" > "$OUT_DIR/synth_${label}.log" 2>&1 \
                || { log "STEP ${label}: FAIL — повторный синтез Yandex упал ($(tail -1 "$OUT_DIR/synth_${label}.log"))"; echo "E2E_STEP ${label} FAIL synth"; return 1; }
            ensure_outdir
            ffmpeg -y -i "$OUT_DIR/cmd_${label}.wav" -af "highpass=f=100,volume=3.0,alimiter=limit=0.98,adelay=1500|all=1" -ac 1 -ar 16000 "$OUT_DIR/cmd_${label}_eq.wav" 2>/dev/null
        fi
        paplay "$OUT_DIR/cmd_${label}_eq.wav" && log "  PLAY_DONE" || log "  PLAY_FAIL"
        sleep "$E2E_REACTION_WINDOW"

        check_cycle "$BEFORE"
        rc=$?
        if [ "$rc" = "0" ]; then
            log "STEP ${label}: ✅ ПОЛНЫЙ ЦИКЛ (акцепт + LLM + TTS)"
            reaction=1
            break
        elif [ "$rc" = "2" ]; then
            log "STEP ${label}: ❌ LLM/TTS ERROR — тест красный, не чиним"
            echo "E2E_STEP ${label} FAIL llm_error (см. $OUT_DIR/llm_error.txt)"
            return 2
        fi
        log "STEP ${label}: нет акцепта (attempt ${attempt}) — повтор"
        sleep "$E2E_RETRY_PAUSE"
    done

    if [ "$reaction" != "1" ]; then
        log "STEP ${label}: ❌ NO_ACCEPT после ${E2E_MAX_ATTEMPTS} попыток"
        echo "E2E_STEP ${label} FAIL no_accept"
        return 1
    fi

    # 3. Проверка паттернов (если заданы для шага) — считываются из scenario
    return 0
}

# --- сценарий или одиночная команда ----------------------------------------
# Issue #1353: запись микрофона охватывает ВЕСЬ retry-цикл (все шаги, все
# попытки). Стартуем до if/else, останавливаем в trap EXIT (см. ниже).
start_recording

# Гарантированная остановка записи при любом завершении (PASS/FAIL/ошибка).
# stop_recording сам идемпотентен: повторный вызов с пустым REC_PID — noop.
trap 'stop_recording' EXIT

PASS=1
if [ -n "$SCENARIO_FILE" ]; then
    # scenario.json: {"steps":[{"text":"...","voice":"anton","label":"s1","patterns":["save_speaker_profile","speaker_id"]}]}
    cp "$SCENARIO_FILE" "$OUT_DIR/scenario.json"
    python3 - "$SCENARIO_FILE" <<'PY' > "$OUT_DIR/scenario_parsed.txt"
import json, sys
sc = json.load(open(sys.argv[1]))
for i, s in enumerate(sc.get("steps", [])):
    print(f"{i}\t{s.get('label', f's{i+1}')}\t{s.get('text','')}\t{s.get('voice','anton')}\t{json.dumps(s.get('patterns',[]))}")
PY
    while IFS=$'\t' read -r idx label text voice patterns_json; do
        [ -z "$idx" ] && continue
        run_step "$text" "$voice" "$label"
        rc=$?
        if [ "$rc" != "0" ]; then
            PASS=0
            # Проверяем паттерны даже при FAIL цикла? Нет — красный есть красный.
            continue
        fi
        # Паттерны шага
        if [ -n "$patterns_json" ] && [ "$patterns_json" != "[]" ]; then
            pats="$(printf '%s' "$patterns_json" | python3 -c 'import json,sys; print(" ".join(json.load(sys.stdin)))')"
            log "STEP ${label}: проверка паттернов: $pats"
            # окно — весь шаг (последние 6 минут покрывают синтез+play+цикл)
            check_patterns "$(date -u -d '-6 minutes' +%Y-%m-%dT%H:%M:%SZ)" $pats
            if [ $? != 0 ]; then
                PASS=0; log "STEP ${label}: ❌ паттерны не найдены"; echo "E2E_STEP ${label} FAIL patterns"
            else
                log "STEP ${label}: ✅ паттерны найдены"; echo "E2E_STEP ${label} OK"
            fi
        else
            echo "E2E_STEP ${label} OK"
        fi
    done < "$OUT_DIR/scenario_parsed.txt"
else
    run_step "$TEXT" "${VOICE:-$YANDEX_TTS_VOICE}" "single"
    rc=$?
    if [ "$rc" != "0" ]; then PASS=0; echo "E2E_STEP single FAIL"; else echo "E2E_STEP single OK"; fi
    # Дополнительные паттерны (--patterns "a,b,c") — проверяем после цикла
    if [ "$rc" = "0" ] && [ -n "$PATTERNS" ]; then
        log "single: проверка паттернов: $PATTERNS"
        IFS=',' read -r -a pat_arr <<< "$PATTERNS"
        check_patterns "$(date -u -d '-6 minutes' +%Y-%m-%dT%H:%M:%SZ)" "${pat_arr[@]}"
        if [ $? != 0 ]; then
            PASS=0; echo "E2E_STEP single FAIL patterns"
        else
            echo "E2E_STEP single OK patterns"
        fi
    fi
fi

if [ "$PASS" = "1" ]; then
    echo "E2E_VERDICT PASS"
    echo "E2E_REACTION_OK"    # маркер для пост-валидатора (в stdout → e2e_atomic_out.log)
    ensure_outdir
    echo "PASS" > "$OUT_DIR/verdict.txt"
    # Issue #1196 L2: после голосового e2e проверяем эхо в Telegram
    # (telegram_node получил ответ dialogue_node). Полуавтомат: включается
    # флагом --check-tg-echo (RUN_NOW / ручной прогон). Не фейлит вердикт —
    # голосовой e2e не обязан слать сообщения в чат; эхо-проверка ловит
    # разрыв диалог↔бот как диагностика.
    if [ "$CHECK_TG_ECHO" = "1" ]; then
        TG_ECHO_BEFORE="$(${ROBOT_SSH} "date -u +%Y-%m-%dT%H:%M:%SZ" 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
        check_telegram_echo "$TG_ECHO_BEFORE"
        TG_ECHO_RC=$?
        if [ "$TG_ECHO_RC" = "0" ]; then
            echo "E2E_TG_ECHO OK"
        elif [ "$TG_ECHO_RC" = "2" ]; then
            echo "E2E_TG_ECHO SKIP"
        else
            echo "E2E_TG_ECHO NO_ECHO"
        fi
    fi
    # Issue #1135/#1138: маркер для пост-валидатора в L-E2E Voice Test.yml.
    # ВАЖНО (ретро 12.08 t_4e592534): ``docker exec bash -c 'echo ...'`` БЕЗ
    # перенаправления НЕ попадает в ``docker logs`` (json-file драйвер пишет
    # только stdout PID 1 контейнера; вывод docker exec идёт в exec-сессию,
    # не в лог). Эмпирически: 0 вхождений. Рабочий способ — писать в
    # ``/proc/1/fd/1`` (stdout PID 1): ``docker logs`` его видит (проверено).
    # Сам маркер дублируется в stdout харнесса (E2E_REACTION_OK выше) — это
    # основной канал для валидатора (e2e_atomic_out.log).
    ${ROBOT_SSH} \
        "docker exec voice-assistant bash -c 'echo E2E_REACTION_OK > /proc/1/fd/1'" \
        >/dev/null 2>&1 || true
else
    echo "E2E_VERDICT FAIL"
    echo "E2E_NO_REACTION"    # маркер для пост-валидатора (в stdout → e2e_atomic_out.log)
    ensure_outdir
    echo "FAIL" > "$OUT_DIR/verdict.txt"
    ${ROBOT_SSH} \
        "docker exec voice-assistant bash -c 'echo E2E_NO_REACTION > /proc/1/fd/1'" \
        >/dev/null 2>&1 || true
fi
echo "E2E_ARTIFACTS $OUT_DIR"
exit $([ "$PASS" = "1" ] && echo 0 || echo 1)
