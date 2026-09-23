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
#   e2e_voice_test.sh --scenario /tmp/scenario.json [--acceptance /tmp/acceptance.json]
#   e2e_voice_test.sh --text "..." --voice ermil --retries 3 --react-window 40
#
# ADR-0022 GATE-1 (acceptance.json gate, issue #1428 / t_ba114e5c):
#   Если --scenario задан, по умолчанию требуется acceptance.json (next to
#   scenario.json либо через --acceptance <path>). Без acceptance.json →
#   FAIL c понятным сообщением. Отключить: --acceptance-skip (НЕ
#   рекомендуется — smoke-false-PASS, ADR-0022 §4.1 R1).
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
# Override для локального тестирования (юнит-тесты): если задан ROBOT_SSH_OVERRIDE,
# используем его вместо ssh-команды. Позволяет bash-юнит-тестам подсунуть
# stub без реального sshpass+robot.
if [ -n "${ROBOT_SSH_OVERRIDE:-}" ]; then
    ROBOT_SSH="$ROBOT_SSH_OVERRIDE"
fi
YANDEX_TTS_VOICE="${YANDEX_TTS_VOICE:-anton}"       # голос по умолчанию
YANDEX_SPEED="${YANDEX_SPEED:-1.0}"

# --- Провайдер синтеза КОМАНД харнесса -------------------------------------
# ВАЖНО: это провайдер, которым БИЛД-МАШИНА озвучивает команду в колонку, а
# НЕ провайдер, которым отвечает робот (тот живёт в tts_node на роботе).
# Исторически харнесс умел только Yandex, и когда доступ к папке Yandex Cloud
# отвалился (PERMISSION_DENIED на folder), КАЖДЫЙ шаг КАЖДОГО прогона падал
# "FAIL synth" — робота при этом никто даже не спрашивал (run 35533542706).
#
#   yandex  — gRPC SpeechKit v3, нужен YANDEX_API_KEY (историческое поведение);
#   minimax — HTTP T2A v2, нужен MINIMAX_API_KEY;
#   silero  — ЛОКАЛЬНЫЙ torch-синтез, ключ не нужен вообще.
#
# ГДЕ ИСПОЛНЯЕТСЯ ЭТОТ ФАЙЛ (22.09.2026): не в раннер-контейнере, а на ХОСТЕ
# katana — ros2@10.1.1.249. Job `runs-on: e2e` только scp'ит харнесс туда и
# дёргает его по ssh (L-E2E Voice Test.yml, шаг «Push atomic harness»). Поэтому
# зависимости silero живут на ХОСТЕ, а не внутри build-github-runner-*: во всех
# девяти раннер-контейнерах (образ myoung34/github-runner) стоит python 3.8 БЕЗ
# torch и numpy — и это нормально, synth_silero там не выполняется никогда.
# `docker exec build-github-runner-e2e python3 -c "import torch"` проверяет НЕ ТУ
# машину; правильная проверка —
#   ssh ros2@10.1.1.249 'python3 -c "import torch, numpy; print(torch.__version__)"'
#
# Состояние 249 на 22.09.2026: python 3.10.12, torch 2.8.0+cu128, numpy 2.2.6
# (в ~ros2/.local), модель — ~/.cache/rob_box_voice/tts_models/v4_ru.pt (40 МБ,
# ПЯТЫЙ кандидат списка в synth_silero; v5_ru на katana нет, он только на
# роботе). Голоса v4_ru — aidar/baya/kseniya/xenia/eugene, ровно тот набор, в
# который переводит map_tts_voice silero, так что подмена на дефолт не нужна.
#
# auto (дефолт) = пройтись по E2E_TTS_PROVIDER_ORDER и взять первого, кто
# реально синтезирует пробную фразу. Проба делается ОДИН раз за прогон, до
# первого шага: иначе 40-шаговый сценарий 40 раз ждал бы таймаут мёртвого
# провайдера. silero стоит последним и не требует ключа — это гарантированный
# донор, поэтому «облака легли» больше не равно «e2e красный».
#
# Проверено вживую 22.09.2026 при мёртвом Yandex (PERMISSION_DENIED на folder)
# и без MINIMAX_API_KEY: probe yandex FAIL → minimax «пропуск — нет ключа» →
# probe silero OK за 3.9 s → «E2E_TTS_PROVIDER silero auto». Фолбек рабочий.
#
# Чем гарантия держится: torch и модель на 249 поставлены РУКАМИ и в репозитории
# ничем не воспроизводятся. Если их смоют (переустановка хоста, чистка ~/.local),
# synth_silero упадёт в fail("PythonError") → probe FAIL → «E2E_TTS_PROVIDER
# none», и шаг разбора вердикта напечатает «это инфра, не робот»
# (L-E2E Voice Test.yml:363) — тихо зелёным прогон при этом не станет.
E2E_TTS_PROVIDER="${E2E_TTS_PROVIDER:-auto}"
E2E_TTS_PROVIDER_ORDER="${E2E_TTS_PROVIDER_ORDER:-yandex,minimax,silero}"
# Результат резолва (заполняется resolve_tts_provider, кэш на весь прогон).
E2E_TTS_PROVIDER_RESOLVED=""
# MiniMax T2A v2 — те же дефолты, что у tts_node (config/tts_node.yaml).
MINIMAX_TTS_MODEL="${MINIMAX_TTS_MODEL:-speech-02-hd}"
MINIMAX_TTS_BASE_URL="${MINIMAX_TTS_BASE_URL:-https://api.minimax.io}"
# Silero v5/v4 ru: torch.package-файл. Пути — как в tts_node (/models/silero),
# плюс кеши билд-машины; последний шанс — скачать через torch.hub.
E2E_SILERO_MODEL="${E2E_SILERO_MODEL:-}"
E2E_SILERO_SAMPLE_RATE="${E2E_SILERO_SAMPLE_RATE:-48000}"
# Причина FAIL (для пост-валидатора и e2e-process::detect_fail_kind).
# feature > llm_error > synth > no_reaction. feature всегда побеждает:
# если фича-ассерт (patterns/acceptance/GATE-1) зафейлился — это баг кода,
# а НЕ «робот не ответил» (иначе e2e-process классифицирует как infra).
E2E_FAIL_KIND=""

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

# Агрегатный GATE-1 (check_gate1_aggregate) сканирует docker logs с
# ``--since <before>``. Раньше E2E_RUN_BEFORE нигде не задавался → fallback
# ``date -u +%Y-%m-%dT%H:%M:%SZ`` вычислялся В КОНЦЕ прогона (== "now") →
# docker logs возвращал пустоту → GATE-1 всегда FAIL («expected tool calls
# not invoked»), хотя tool calls в логах были. Фикс: фиксируем момент старта
# прогона ПО ЧАСАМ РОБОТА (docker logs --since сравнивает с timestamp
# контейнера робота) до первого шага.
E2E_RUN_BEFORE="$(${ROBOT_SSH} "date -u +%Y-%m-%dT%H:%M:%SZ" 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"

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
    # e2e run 32595628905: хардкод max_steps=5 обрезал запись — voice_core_suite
    # содержит 11 шагов, и parec умирал до конца теста (в артефакт попадал
    # только кусок прогона). Считаем число шагов из scenario.json, если задан.
    if [ -n "$SCENARIO_FILE" ] && [ -f "$SCENARIO_FILE" ]; then
        local _n
        _n="$(python3 -c 'import json,sys; print(len(json.load(open(sys.argv[1], encoding="utf-8")).get("steps", [])))' "$SCENARIO_FILE" 2>/dev/null || echo 0)"
        case "$_n" in
            ''|*[!0-9]*) : ;;
            *) [ "$_n" -gt "$max_steps" ] && max_steps="$_n" ;;
        esac
    fi
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
    # 🔴 e2e run 32595628905: старый `if ! kill -0; then ... return 0; fi`
    # делал РАННИЙ ВЫХОД, когда parec уже умер от timeout (всегда так для
    # длинного scenario — max_steps был хардкодом 5 < 11 шагов). В итоге
    # сырой /tmp/e2e_raw_<RID>.pcm (25MB) НИКОГДА не конвертировался в
    # recording.wav, симлинк не создавался, а workflow collect падал на
    # stale-аудио другого прогона. Теперь: если parec ещё жив — гасим его,
    # если уже умер — просто конвертируем записанный RAW (он валиден).
    if kill -0 "$REC_PID" 2>/dev/null; then
        log "RECORDING: stop (SIGTERM pid=${REC_PID})"
        kill -TERM "$REC_PID" 2>/dev/null || true
        # Дать parec корректно закрыть pipe (graceful shutdown → корректный EOF)
        for _ in 1 2 3 4 5; do
            if ! kill -0 "$REC_PID" 2>/dev/null; then break; fi
            sleep 1
        done
    else
        log "RECORDING: parec уже завершился (timeout) — конвертирую записанный RAW"
    fi
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
    if ffmpeg -nostdin -y -f s16le -ar 16000 -ac 1 -i "$RECORDING_RAW" "$RECORDING_WAV" 2>/dev/null; then
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
ACCEPTANCE_FILE=""
ACCEPTANCE_SKIP=0
VOICE=""
PATTERNS=""
CHECK_TG_ECHO=0
while [ $# -gt 0 ]; do
    case "$1" in
        --text)     TEXT="$2"; shift 2 ;;
        --voice)    VOICE="$2"; shift 2 ;;
        --scenario) SCENARIO_FILE="$2"; shift 2 ;;
        --acceptance)        ACCEPTANCE_FILE="$2"; shift 2 ;;
        --acceptance-skip)   ACCEPTANCE_SKIP=1; shift 1 ;;
        --patterns) PATTERNS="$2"; shift 2 ;;
        --retries)  E2E_MAX_ATTEMPTS="$2"; shift 2 ;;
        --react-window) E2E_REACTION_WINDOW="$2"; shift 2 ;;
        --tts-provider) E2E_TTS_PROVIDER="$2"; shift 2 ;;
        --check-tg-echo) CHECK_TG_ECHO=1; shift 1 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

case "$E2E_TTS_PROVIDER" in
    auto|yandex|minimax|silero) ;;
    *) echo "E2E_FATAL: --tts-provider='$E2E_TTS_PROVIDER' — ожидается auto|yandex|minimax|silero" >&2; exit 2 ;;
esac
# Ключ требуем ТОЛЬКО у явно выбранного облачного провайдера. В auto отсутствие
# ключа — это не фатал, а «этот кандидат пропускается» (см. resolve_tts_provider):
# silero в конце очереди не требует ключей вообще, и прогон всё равно состоится.
if [ "$E2E_TTS_PROVIDER" = "yandex" ] && [ -z "${YANDEX_API_KEY:-}" ]; then
    echo "E2E_FATAL: YANDEX_API_KEY не задан (--tts-provider=yandex)" >&2; exit 2
fi
if [ "$E2E_TTS_PROVIDER" = "minimax" ] && [ -z "${MINIMAX_API_KEY:-}" ]; then
    echo "E2E_FATAL: MINIMAX_API_KEY не задан (--tts-provider=minimax)" >&2; exit 2
fi
if [ "$E2E_TTS_PROVIDER" = "auto" ] && [ -z "${YANDEX_API_KEY:-}" ] && [ -z "${MINIMAX_API_KEY:-}" ]; then
    echo "E2E_WARN: ни YANDEX_API_KEY, ни MINIMAX_API_KEY не заданы — auto пойдёт сразу в silero" >&2
fi
if [ -z "$TEXT" ] && [ -z "$SCENARIO_FILE" ]; then
    echo "E2E_FATAL: нужен --text или --scenario" >&2; exit 2
fi

# --- issue #2750: изоляция БД дикторов для акта «Знакомство» ---------------
# Акт 2 ночного марафона (night_marathon_act2_acquaintance_*) по сценарию
# регистрирует НАСТОЯЩИЕ голосовые профили ("Саша"/"Борис") в speakers.db.
# Раньше чистую базу под это получали ssh-командой СНАРУЖИ кода перед каждым
# прогоном (подтверждено владельцем в issue #2750): бэкап боевой
# /data/speakers.db в .bak-<UTC>Z + DELETE FROM embeddings/speakers — и один
# раз это стёрло профиль живого человека через 19 минут после регистрации.
#
# Замена — параметр узла speaker_id_node (e2e_mode, bool), НЕ топик: первая
# версия этой правки заводила /voice/speaker/e2e_mode, но
# scripts/lint/seam_without_consumer.py (ADR-0021) справедливо пометил его
# новым швом без потребителя — паблишер живёт здесь, в bash, а Python-сканер
# топиков видит только src/. Включение ГАРАНТИРУЕТ пустую e2e-базу (узел сам
# стирает e2e_db_path на переходе false→true — см.
# speaker_id_node.py:_apply_e2e_mode), боевая db_path не открывается на
# запись, пока режим включён. deactivate — из trap EXIT: что бы ни случилось
# со сценарием (PASS/FAIL/обрыв), робот обязан вернуться на боевую БД для
# мастерской.
#
# ⚠️ ГДЕ ЖИВЁТ ros2 (issue #2763, замер на Vision Pi 22.09.2026)
# ------------------------------------------------------------------
# Первая редакция звала ``${ROBOT_SSH} "ros2 param set ..."`` — то есть на
# ХОСТ Vision Pi. ROS на хосте НЕТ вообще:
#
#   ls /opt/ros            → No such file or directory
#   bash -lc 'ros2 --help' → FAIL
#
# Команда возвращала rc=127, активация уходила в ветку предупреждения, и
# акт 2 спокойно ехал по боевой /data/speakers.db — ровно то, что issue
# #2750 закрывал. Защита не срабатывала НИ РАЗУ с момента мержа #2759:
# 22.09 в боевой базе мастерской нашлись синтетические «Саша» и «Борис» от
# утренних прогонов рядом с профилем живого человека.
#
# ROS живёт только внутри контейнера, в /ws/install (НЕ /ros2_ws/install).
# Все прочие два десятка обращений в этом файле уже обёрнуты в
# ``docker exec voice-assistant`` — эти два были единственным исключением.
# ``--no-daemon`` — потому что демон ros2cli на роботе периодически умирает
# и роняет CLI в ``Fault 1: !rclpy.ok()`` (видели в vision-hailo, #2703).
robot_ros() {
    ${ROBOT_SSH} "docker exec voice-assistant bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; $*'"
}
# ⚠️ ЯКОРЬ — СОДЕРЖИМОЕ СЦЕНАРИЯ, А НЕ ЕГО ИМЯ (issue #2763, инцидент
# 22.09.2026, run 35729958215)
# ------------------------------------------------------------------
# Первая редакция матчилась по имени файла:
#
#     case "$1" in *night_marathon_act2_acquaintance*) return 0 ;;
#
# Но воркфлоу копирует сценарий на билд-машину под ФИКСИРОВАННЫМ именем
# (шаг «Push atomic harness + scenario to build machine»):
#
#     CMD="/tmp/e2e_voice_test.sh --scenario /tmp/e2e_scenario.json"
#
# то есть $SCENARIO_FILE здесь ВСЕГДА /tmp/e2e_scenario.json. Шаблон не
# совпадал никогда, activate_e2e_speaker_db не вызывался ни разу — молча,
# без строки в логе (отсутствие И «🧪 e2e_mode=true», И E2E_FATAL — это и
# есть симптом). Акт «Знакомство» прогона 35729958215 записал
# синтетических «Сашу» и «Бориса» в боевую /data/speakers.db рядом с
# профилями живых людей мастерской — ровно то, что закрывал issue #2750.
#
# Имя файла тут вообще не якорь: его назначает воркфлоу, а не автор
# сценария. Признак берём семантический — сценарий РЕГИСТРИРУЕТ дикторов.
# Изоляция нужна любому такому сценарию, а не конкретному акту по
# названию. Ошибиться в плюс безопасно (лишняя изоляция ничего не портит),
# в минус — нет: это запись в боевую БД мастерской.
#
# На 22.09.2026 register_speaker встречается ровно в одном сценарии
# марафона (act2_acquaintance, 8 упоминаний) — поведение то же, что
# задумывал #2759, но теперь оно действительно срабатывает.
scenario_registers_speakers() {
    [ -f "$1" ] || return 1
    grep -q 'register_speaker' "$1"
}
E2E_SPEAKER_DB_ACTIVATED=0

# Режим ЧИТАЕТСЯ обратно, а не берётся из exit-кода: ros2cli печатает
# «Set parameter failed» при отказе parameters_callback и всё равно может
# выйти с 0, а цена ошибки здесь — боевая БД мастерской с профилями живых
# людей. Единственное честное подтверждение — значение параметра на узле.
# Проверять надо ИМЕННО e2e_mode: ``db_path`` не меняется, он остаётся
# путём боевой базы (узел переключает активное соединение, а не параметр).
_read_e2e_mode() {
    robot_ros "ros2 param get /speaker_id_node e2e_mode --no-daemon" 2>/dev/null \
        | grep -aoE 'Boolean value is: (True|False)' | tail -1
}
activate_e2e_speaker_db() {
    robot_ros "ros2 param set /speaker_id_node e2e_mode true --no-daemon" >/dev/null 2>&1
    case "$(_read_e2e_mode)" in
        *True*)
            E2E_SPEAKER_DB_ACTIVATED=1
            log "🧪 speaker_id_node: e2e_mode=true — боевая /data/speakers.db не тронута"
            ;;
        *)
            # НЕ предупреждение, а фатал. Акт «Знакомство» регистрирует
            # настоящие голосовые профили; без изоляции он пишет их в базу,
            # которой пользуются живые люди в мастерской. Лучше не прогнать
            # акт совсем, чем прогнать его по боевой базе (issue #2750).
            echo "E2E_FATAL: не удалось включить e2e_mode у speaker_id_node — акт «Знакомство» писал бы в боевую /data/speakers.db" >&2
            echo "           проверь вручную: ssh <robot> \"docker exec voice-assistant bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; ros2 param get /speaker_id_node e2e_mode --no-daemon'\"" >&2
            exit 2
            ;;
    esac
}
deactivate_e2e_speaker_db() {
    [ "$E2E_SPEAKER_DB_ACTIVATED" = "1" ] || return 0
    robot_ros "ros2 param set /speaker_id_node e2e_mode false --no-daemon" >/dev/null 2>&1
    case "$(_read_e2e_mode)" in
        *False*)
            log "🧪 speaker_id_node: e2e_mode=false — вернулись на боевую /data/speakers.db"
            ;;
        *)
            # Робот остался на e2e-базе: в мастерской он перестанет узнавать
            # живых людей и будет писать их эмбеддинги в тестовую БД. Это
            # обязано быть видно в отчёте прогона, а не только в логах.
            log "❌ ВНИМАНИЕ: speaker_id_node НЕ вернулся на боевую speakers.db — робот сейчас на e2e-базе и не узнаёт живых людей!"
            log "   почини вручную: ssh <robot> \"docker exec voice-assistant bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; ros2 param set /speaker_id_node e2e_mode false --no-daemon'\""
            ;;
    esac
}

# --- issue #2781: изоляция БД долгосрочной памяти (voice_facts) ------------
# Изоляция дикторов выше (#2750/#2763) накрывает ТОЛЬКО speakers.db. Акт
# «Знакомство» ночного марафона и регистрирует голоса (register_speaker,
# изолировано), И называет LLM факты через memory_save (НЕ изолировано было
# до этой правки) — замер на Vision Pi 22.09.2026 нашёл 59 из 116 фактов
# боевой /data/voice_memory.db, упоминающих синтезированный каст марафона
# ("Саша не ест лук", "Борис болеет за Спартак") вперемешку с фактами живых
# людей мастерской (часть — с speaker_id=NULL, неотличимы по одному полю).
#
# Тот же паттерн, что у спикеров: якорь — СОДЕРЖИМОЕ сценария (memory_save
# упоминается в тексте сценария), не имя файла (issue #2763 — воркфлоу
# копирует сценарий под фиксированным /tmp/e2e_scenario.json, имя не несёт
# информации об авторе). Владелец параметра — mcp_server (нода, которая
# реально исполняет memory_save, см. tools/memory.py:MemorySaveTool),
# отдельный от speaker_id_node bool-параметр e2e_mode (issue #2781,
# mcp_server.py:_apply_e2e_mode) — тот же ros2 param set, не топик.
scenario_writes_memory() {
    [ -f "$1" ] || return 1
    grep -q 'memory_save' "$1"
}
E2E_MEMORY_DB_ACTIVATED=0

# Как и у _read_e2e_mode выше — читаем значение параметра ОБРАТНО, не
# верим exit-коду ros2cli (может быть 0 даже когда parameters_callback
# отклонил значение). Цена ошибки — боевая долгосрочная память мастерской.
_read_mcp_e2e_mode() {
    robot_ros "ros2 param get /mcp_server e2e_mode --no-daemon" 2>/dev/null \
        | grep -aoE 'Boolean value is: (True|False)' | tail -1
}
activate_e2e_memory_db() {
    robot_ros "ros2 param set /mcp_server e2e_mode true --no-daemon" >/dev/null 2>&1
    case "$(_read_mcp_e2e_mode)" in
        *True*)
            E2E_MEMORY_DB_ACTIVATED=1
            log "🧪 mcp_server: e2e_mode=true — боевая /data/voice_memory.db не тронута"
            ;;
        *)
            # ФАТАЛ, не предупреждение — как и у изоляции дикторов: лучше не
            # прогнать акт, чем засорить боевую долгосрочную память
            # мастерской синтезированными фактами каста марафона.
            echo "E2E_FATAL: не удалось включить e2e_mode у mcp_server — сценарий писал бы факты memory_save в боевую /data/voice_memory.db" >&2
            echo "           проверь вручную: ssh <robot> \"docker exec voice-assistant bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; ros2 param get /mcp_server e2e_mode --no-daemon'\"" >&2
            exit 2
            ;;
    esac
}
deactivate_e2e_memory_db() {
    [ "$E2E_MEMORY_DB_ACTIVATED" = "1" ] || return 0
    robot_ros "ros2 param set /mcp_server e2e_mode false --no-daemon" >/dev/null 2>&1
    case "$(_read_mcp_e2e_mode)" in
        *False*)
            log "🧪 mcp_server: e2e_mode=false — вернулись на боевую /data/voice_memory.db"
            ;;
        *)
            log "❌ ВНИМАНИЕ: mcp_server НЕ вернулся на боевую voice_memory.db — робот сейчас пишет долгосрочную память в e2e-базу!"
            log "   почини вручную: ssh <robot> \"docker exec voice-assistant bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; ros2 param set /mcp_server e2e_mode false --no-daemon'\""
            ;;
    esac
}

# --- issue #2809 — node_params: сценарий форсирует зону сомнения переспроса --
# Акт «переспрос личности» (night_marathon_act2b_identity_question) должен
# ГАРАНТИРОВАННО попасть в зону tentative (single/contested), а не confident —
# синтетические голоса MiniMax опознаются 0.87-0.96, выше band_high=0.80 по
# умолчанию, и переспрос почти никогда не случается на синтетике. Top-level
# поле сценария ``node_params`` форсирует конкретный параметр конкретного
# узла на время акта:
#
#   "node_params": {"/speaker_id_node": {"name_confidence_band_high": 0.99}}
#
# Реализация (apply_node_params/restore_node_params/_ros2_param_*) вынесена в
# e2e_voice_lib.sh — она чистые функции (никакого ENV на этапе source),
# юнит-тестируется отдельно от главного скрипта (scripts/testing/
# test_e2e_node_params.sh, robot_ros() подменяется стабом через
# ROBOT_SSH_OVERRIDE, как и весь остальной харнесс).
#
# Контракт (тот же принцип честности, что e2e_mode выше): значение не
# ВЕРИТСЯ на слово exit-коду ``ros2 param set`` — его читают ОБРАТНО и
# сравнивают с запрошенным, иначе колбэк параметров мог отклонить значение
# (или узел вообще не перехватывать этот параметр — см. issue #2809 fix в
# speaker_id_node.parameters_callback) и харнесс решил бы, что переопределение
# сработало, хотя оно тихо провалилось. Восстановление — К ИСХОДНОМУ
# значению, ПРОЧИТАННОМУ ДО изменения (не к хардкоду в этом файле), и делается
# ИЗ trap EXIT — что бы ни случилось со сценарием (PASS/FAIL/обрыв связи),
# узел обязан вернуться на калиброванное 0.80/0.15, иначе следующий прогон на
# этом же роботе (или живая мастерская, если робот не был перезапущен)
# получит зону сомнения, растянутую до 0.99, и все живые люди станут
# "tentative".
E2E_NODE_PARAM_ORIGINALS_FILE="${OUT_DIR}/.node_param_originals.tsv"
: > "$E2E_NODE_PARAM_ORIGINALS_FILE" 2>/dev/null || true

# --- helpers ----------------------------------------------------------------
log() { echo ">>> $*"; }

# mark_fail_kind() — запоминает самую информативную причину FAIL.
# Приоритет: feature > infra > llm_error > synth > no_reaction
# (feature не понижается).
# emit_step() — единственная точка публикации результата шага.
# Кроме stdout-маркера (контракт пост-валидатора, ADR-0015) дописывает строку
# в steps.jsonl. Раньше счёта шагов не существовало вообще: любой FAIL ронял
# весь прогон через `PASS=0; break`, а сколько шагов реально прошло — нигде не
# сохранялось. Человек видел только PASS/FAIL и не мог отличить «робот умер на
# первом шаге» от «10 из 11 прошло, споткнулись на последнем».
#
# Дубли по label — норма: run_step печатает свой FAIL, а scenario-цикл потом
# печатает итог шага. Сводка берёт ПОСЛЕДНЮЮ запись на label (см. write_summary).
emit_step() {  # $1="<label> <STATUS> [detail]"
    echo "E2E_STEP $1"
    ensure_outdir
    python3 - "$1" "$OUT_DIR/steps.jsonl" <<'PY'
import sys, json, time
parts = sys.argv[1].split(None, 2)
record = {
    "label": parts[0] if parts else "",
    "status": parts[1] if len(parts) > 1 else "",
    "detail": parts[2] if len(parts) > 2 else "",
    "at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
}
with open(sys.argv[2], "a", encoding="utf-8") as fh:
    fh.write(json.dumps(record, ensure_ascii=False) + "\n")
PY
}

mark_fail_kind() {  # $1=kind
    local kind="$1"
    case "$kind" in
        feature)     E2E_FAIL_KIND="feature" ;;
        # infra — звук не доехал до STT (audio_node отбросил фразу по длине /
        # захват мёртв / нет ReSpeaker). Ставится выше llm_error/synth/
        # no_reaction, потому что при таком отказе робот-логика вообще не
        # исполнялась и обвинять её нельзя (run 35658231116: `FAIL backlog_miss`
        # при `Речь отклонена: 17.37с`). Ниже feature: если в том же прогоне
        # есть настоящий acceptance-фейл, он актуальнее для разбора.
        infra)       [ "$E2E_FAIL_KIND" = "feature" ] || E2E_FAIL_KIND="infra" ;;
        llm_error)   { [ "$E2E_FAIL_KIND" = "feature" ] || [ "$E2E_FAIL_KIND" = "infra" ]; } || E2E_FAIL_KIND="llm_error" ;;
        synth)       { [ "$E2E_FAIL_KIND" = "feature" ] || [ "$E2E_FAIL_KIND" = "infra" ] || [ "$E2E_FAIL_KIND" = "llm_error" ]; } || E2E_FAIL_KIND="synth" ;;
        no_reaction) [ -z "$E2E_FAIL_KIND" ] && E2E_FAIL_KIND="no_reaction" ;;
    esac
}

# BUG-B (t_f0612a43): имя wav файла строится из ${label} и идёт в heredoc-Python
# synth_yandex, а также в paplay/ffmpeg. Раньше, если label содержал кириллицу
# и/или спец-символы (,?!«»), имя файла получалось невалидным для оболочки
# (pathname expansion ломал '?', запятая в Python heredoc путала аргументы),
# и synth_yandex возвращал YANDEX_EMPTY / permission denied с текстом вместо
# voice — шаг помечался FAIL synth без реального запуска команды.
#
# Фикс: транслитерация + ASCII slug ДО подстановки в out_wav. Исходный label
# сохраняется для логов и transcript.json (человекочитаемость).
#
# Реализация: safe_label определена в отдельном файле e2e_voice_lib.sh,
# чтобы можно было source'ить её из unit-тестов без побочных эффектов
# (source основного файла выполняет main flow и валится на проверке ENV).
SCRIPT_DIR_E2E="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR_E2E/e2e_voice_lib.sh"
# ADR-0029 §2.3 wake-gate pre-flight helpers (retro t_be491fba). Source'ится
# ПОСЛЕ e2e_voice_lib.sh, но ДО любых операций с docker logs. Pure functions
# only — никакого main flow, никакого чтения ENV.
# shellcheck disable=SC1091
source "$SCRIPT_DIR_E2E/e2e_voice_wake_gate.sh"

# bug(run 35658231116, 22.09.2026): этот блок стоял на строке ~124 — ДО
# парсинга аргументов (--scenario разбирается ниже, ~300) и ДО source
# e2e_voice_wake_gate.sh (~397). Под set -u обращение ${SCENARIO_FILE:-}
# не падало, а молча резолвилось в пустую строку, поэтому условие
# `[ -n "${SCENARIO_FILE:-}" ]` было ВСЕГДА ложным: любой scenario-прогон
# уходил в else-ветку, форсил WAKE_GATE_CLEARED=1 и писал в артефакт
# "single-text mode — preflight N/A". Проверено на живом прогоне
# 35658231116 (запуск был --scenario, а wake_gate_preflight.json содержал
# именно этот single-text reason). Следствие: SKIP-гард для
# expect="wake-gated" (ADR-0029 §2.3) не срабатывал никогда, и cold-start
# wake-gate флак краснел как acceptance/feature-fail — ровно тот
# misdiagnosis, против которого фича и делалась (ретро t_be491fba).
# Второй слой той же ошибки: run_wake_gate_preflight определяется в либе
# строкой выше — из старого места она была ещё и не видна.
# --- ADR-0029 §2.3: wake-gate pre-flight probe ----------------------------
# Retro t_be491fba: rounds 215-222 voice_core_suite_v1 показали fail-streak
# 3/3+ на cold-start wake-gate. dj02_stop_music шаг имеет ✅ ПОЛНЫЙ ЦИКЛ
# (акцепт + LLM + TTS) + PATTERN_MISS stop_music → aggregate GATE-1 фейлит
# "expected tool calls not invoked: stop_music". Root cause = cold-start
# wake-gate (TRANSCRIPT[dj02] = «стоп музыка» без «Робот»), а не LLM race
# как было misdiagnosed в t_9d229634.
#
# Probe: проверяем docker logs с момента E2E_RUN_BEFORE — есть ли ЛЮБОЕ
# ПРИНЯТО с wake-prefix (Робот/Робокс). Если нет → cold-start не пройден →
# step.expect="wake-gated" помечаются SKIP, не FAIL (backlog-аккумулятор
# копит «обот»/«как дела» без wake — это by design, не bug).
#
# Артефакт: $OUT_DIR/wake_gate_preflight.json — verdict, checked_at, before,
# reason, error. CI / ревью читают его для доказательства «это cold-start
# flake, а не acceptance fail».
WAKE_GATE_PREFLIGHT_FILE="${OUT_DIR}/wake_gate_preflight.json"
WAKE_GATE_CLEARED=0   # 1 = cold-start cleared, 0 = not cleared, 2 = probe error
# 1 = первый wake-gated шаг уже отыгран как cold-start проба. См. каскад-гард
# в run_step: без него непройденный гейт уводил в SKIP ВСЕ wake-gated шаги,
# потому что акцепта, который его откроет, взяться было неоткуда.
WAKE_GATE_PROBE_SPENT=0
# Сколько шагов РЕАЛЬНО пропущено из-за непройденного wake-gate. Именно это, а
# не сам факт «гейт не прогрелся», оправдывает пропуск агрегатного GATE-1:
# иначе оправдание применяется там, где оправдывать нечего (см. блок GATE-1
# SKIP-логики ниже и разбор прогона 35665111906).
WAKE_GATE_SKIPPED_STEPS=0
WAKE_GATE_PREFLIGHT_REASON=""
# Под set -u SCENARIO_FILE может быть не задан (single-text mode). Используем
# ${SCENARIO_FILE:-} для безопасного обращения.
if [ -n "${SCENARIO_FILE:-}" ]; then
    log "WAKE-GATE-PREFLIGHT: probe before=${E2E_RUN_BEFORE}"
    run_wake_gate_preflight "$E2E_RUN_BEFORE" "$WAKE_GATE_PREFLIGHT_FILE"
    case $? in
        0)  WAKE_GATE_CLEARED=1
            WAKE_GATE_PREFLIGHT_REASON="cold-start cleared"
            log "WAKE-GATE-PREFLIGHT: ✅ cold-start cleared" ;;
        1)  WAKE_GATE_CLEARED=0
            WAKE_GATE_PREFLIGHT_REASON="cold-start NOT cleared"
            log "WAKE-GATE-PREFLIGHT: ⚠️ cold-start NOT cleared — wake-gated steps will SKIP" ;;
        2)  WAKE_GATE_CLEARED=2
            WAKE_GATE_PREFLIGHT_REASON="probe error"
            log "WAKE-GATE-PREFLIGHT: ❌ probe error (no ROBOT_SSH / docker logs) — treating as not-cleared" ;;
    esac
else
    # single-text mode: preflight не применим (одиночный wake-step не
    # требует cold-start gate — он и ЕСТЬ cold-start probe). Пишем
    # минимальный JSON, чтобы артефакт всегда был.
    WAKE_GATE_CLEARED=1
    mkdir -p "$OUT_DIR"
    printf '{\n  "cleared": true,\n  "checked_at": "%s",\n  "before": "%s",\n  "reason": "single-text mode — preflight N/A (per-step wake-gated handled by run_step retry loop)",\n  "error": null\n}\n' \
        "$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
        "$E2E_RUN_BEFORE" \
        > "$WAKE_GATE_PREFLIGHT_FILE"
fi


# --- ADR-0022 GATE-1 acceptance.json (gating only) -------------------------
# Issue #2300 (09.09.2026): auto-discovery списка кандидатов
# (acceptance.json / <base>_acceptance.json / <prefix>_acceptance_v<N>.json)
# теперь живёт ТОЛЬКО в agent-flow-e2e-process.sh:resolve_acceptance_candidate().
# Это единственный резолвер в системе — раньше та же логика дублировалась
# здесь (issue #1452 / #1456 / #1551 исторические false-FAIL из-за рассинхрона
# harness ↔ deploy-side). Контракт: e2e-process резолвит один раз и
# передаёт путь явно через `-f acceptance_file=<path>` в workflow input →
# env ACCEPTANCE_FILE → сюда. Харнесс читает как есть, без fallback-поиска.
#
# Gating оставлен как guard для ручного запуска workflow (без e2e-process):
# scenario.json задан + acceptance.json отсутствует + не --acceptance-skip
# → FAIL (ADR-0022 §4.1 R1: smoke-false-PASS).
# Single-shot --text без scenario не требует acceptance (legitimate smoke).
if [ -n "$SCENARIO_FILE" ] && [ -z "$ACCEPTANCE_FILE" ] && [ "$ACCEPTANCE_SKIP" != "1" ]; then
    log "❌ GATE-1 FAIL: --scenario задан, но ACCEPTANCE_FILE не передан"
    log "   Контракт ADR-0022 §4.1 / issue #2300: путь к acceptance.json"
    log "   должен резолвиться на deploy-стороне (agent-flow-e2e-process.sh:"
    log "   resolve_acceptance_candidate), а не здесь. Этот запуск либо"
    log "   ручной (workflow_dispatch без e2e-process) — укажи --acceptance <path>"
    log "   явно, либо e2e-process не отрезолвил путь (см. его логи)."
    log "   Ожидаемые кандидаты в <dir(scenario)>:"
    log "     1) acceptance.json"
    log "     2) <scenario_basename>_acceptance.json"
    log "     3) <scenario_prefix>_acceptance.json  (prefix = strip _v<N>/_suite)"
    log "     4) <scenario_prefix>_acceptance_v<N>.json"
    log "   Обход (НЕ рекомендуется): --acceptance-skip"
    log "   Подробнее: docs/adr/0022-process-e2e-done-gates.md §4.1"
    mkdir -p "$OUT_DIR"
    cat > "$OUT_DIR/acceptance.json" <<EOF
{
  "gate": "GATE-1",
  "pass": false,
  "reason": "scenario.json provided but ACCEPTANCE_FILE env not set (issue #2300: auto-discovery is e2e-process-only)",
  "scenario_file": "$SCENARIO_FILE",
  "hint": "agent-flow-e2e-process.sh:resolve_acceptance_candidate resolves the acceptance path; harness reads it via env ACCEPTANCE_FILE. Manual runs must pass --acceptance <path> explicitly."
}
EOF
    echo "E2E_GATE1_MISSING_ACCEPTANCE"
    exit 1
fi

# --- ADR-0022 GATE-1 aggregate (top-level) ----------------------------------
# Возвращает 0 если PASS, 1 если FAIL.
# В отличие от per-step check_acceptance(), эта функция анализирует ВСЕ
# логи voice-assistant за весь прогон (агрегированно) и сверяет с
# top-level expected_tool_calls + must_not_call.
#
# Контракт acceptance.json:
#   expected_tool_calls: list[str]  — каждый должен встретиться хотя бы 1 раз
#   must_not_call:        list[str]  — НИ ОДНОГО не должно встретиться
#
# Пишет $OUT_DIR/acceptance.json (перезаписывает per-step формат, если
# был). Формат совместим с ADR-0022 §4.1.
check_gate1_aggregate() {  # $1=acceptance_file_path $2=before_rfc3339
    local acc_file="$1" before="${2:-$(date -u +%Y-%m-%dT%H:%M:%SZ)}"
    if [ ! -f "$acc_file" ]; then
        log "GATE-1: ❌ acceptance.json не найден: $acc_file"
        return 1
    fi

    local logs logs_file
    logs="$(${ROBOT_SSH} "docker logs voice-assistant --since '${before}' 2>&1" 2>/dev/null || echo '')"
    # Логи всего прогона могут превысить ARG_MAX (~2MB): передача через
    # env-переменную `LOGS="$logs" python3` падала с «Argument list too long»
    # (e2e run 32595628905, line 371 → пустой acceptance.json → GATE-1 ❌).
    # Пишем логи в файл и читаем их в Python из файла.
    logs_file="$OUT_DIR/.gate1_logs.txt"
    printf '%s' "$logs" > "$logs_file"

    # Парсим + валидируем acceptance.json в Python → пишем acceptance.json
    # с verdict в OUT_DIR.
    ACCEPTANCE_FILE="$acc_file" LOGS_FILE="$logs_file" \
        PYTHONPATH="$SCRIPT_DIR_E2E${PYTHONPATH:+:$PYTHONPATH}" \
        python3 - <<'PY' > "$OUT_DIR/acceptance.json"
import json, os, re, sys
# «тул вызван» != «тул есть в списке доступных»: dialogue_node печатает
# tools(56) и системный промпт на каждом ходе (см. e2e_tool_match.py).
from e2e_tool_match import tool_invoked
# Issue #2406: для discovery-тул ассерта нужен порядок «tool ДО voice».
from e2e_tool_match import (
    first_invocation_position,
    first_voice_cycle_position,
    TOOL_NAME_RE,
)

acc_path = os.environ["ACCEPTANCE_FILE"]
with open(os.environ["LOGS_FILE"], encoding="utf-8", errors="replace") as _f:
    logs = _f.read()

try:
    acc = json.load(open(acc_path))
except Exception as e:
    sys.stdout.write(json.dumps({
        "gate": "GATE-1",
        "pass": False,
        "reason": f"acceptance.json parse error: {e}",
        "acceptance_file": acc_path,
    }, ensure_ascii=False, indent=2))
    sys.exit(0)

# Schema validation: обязательные поля
expected_call = acc.get("expected_tool_calls", []) or []
must_not = acc.get("must_not_call", []) or []
if not isinstance(expected_call, list) or not isinstance(must_not, list):
    sys.stdout.write(json.dumps({
        "gate": "GATE-1",
        "pass": False,
        "reason": "expected_tool_calls and must_not_call must be list[str]",
        "acceptance_file": acc_path,
    }, ensure_ascii=False, indent=2))
    sys.exit(0)

# Substring search по всем логам прогона (case-insensitive).
# Tool names в логах: dialogue_node печатает "Calling MCP tool: <name>" и
# "MCP tool result: <name>"; некоторая tool_call нода — JSON-RPC формат.
# Ищем substring — robust к формату, ловит оба.
def has(frag):
    return tool_invoked(logs, frag)

actual_calls = []
for c in (expected_call + must_not):
    if has(c) and c not in actual_calls:
        actual_calls.append(c)

found_expected = [c for c in expected_call if has(c)]
missing_expected = [c for c in expected_call if not has(c)]
forbidden_called = [c for c in must_not if has(c)]

# Issue #2406: discovery-tools order check (top-level aggregate).
# Семантика та же, что в per-step check_acceptance, но работает по
# агрегированным логам всего прогона. Применяется только если
# acceptance.json верхнего уровня объявляет discovery_tools.
discovery_tools_raw = acc.get("discovery_tools", []) or []
discovery_tools = []
discovery_tool_errors = []
for _dt in discovery_tools_raw:
    if not isinstance(_dt, str) or not TOOL_NAME_RE.match(_dt.lower()):
        discovery_tool_errors.append(
            f"discovery_tools entry {repr(_dt)} is not a tool name "
            f"(must match ^[a-z][a-z0-9_]*$)"
        )
        continue
    discovery_tools.append(_dt)
discovery_failures = []
discovery_records = []
voice_pos = first_voice_cycle_position(logs) if discovery_tools else None
for _dt in discovery_tools:
    tool_pos = first_invocation_position(logs, _dt)
    rec = {
        "tool": _dt,
        "first_invocation_pos": tool_pos,
        "first_voice_pos": voice_pos,
    }
    if tool_pos is None:
        discovery_failures.append(
            f"discovery tool {_dt!r} was NOT invoked during run "
            f"(issue #2406: LLM bypassed the required tool call)"
        )
    elif voice_pos is not None and tool_pos > voice_pos:
        discovery_failures.append(
            f"discovery tool {_dt!r} invoked at pos {tool_pos}, "
            f"AFTER verbal answer at pos {voice_pos} "
            f"(issue #2406: verbal-only LLM answer before required tool)"
        )
    discovery_records.append(rec)

failures = []
if missing_expected:
    failures.append(
        "expected tool calls not invoked during run: "
        + ", ".join(missing_expected)
    )
if forbidden_called:
    failures.append(
        "forbidden tool calls invoked during run: "
        + ", ".join(forbidden_called)
    )
# Issue #2406: discovery_tools — то же что в per-step check_acceptance, но
# для top-level aggregate. Порядок «тул ДО voice» ассертится через
# first_invocation_position vs first_voice_cycle_position.
if discovery_tool_errors:
    failures.extend(discovery_tool_errors)
if discovery_failures:
    failures.extend(discovery_failures)

# Ретро 22.08 t_c7761956 (A3): voice-cycle-but-no-tool-call hint.
# Если TTS finished есть (LLM ответил голосом), но expected_tool_calls
# missing — LLM сделал verbal-only answer, проигнорировав side-effect.
# Это типовой root cause для dj02_stop_music (RULE #MUSIC не enforced):
# юзер сказал «стоп музыку», робот ответил голосом, но stop_music не вызван.
# Маркируем как soft-fail с явной подсказкой в reason — ретро / worker
# сразу видят где искать.
soft_hints = []
tts_finished_count = logs.lower().count("tts finished")
speak_text_count = logs.lower().count("speak_text")
if missing_expected and tts_finished_count >= 1:
    # Какие tools ожидались, но не были вызваны при наличии voice cycle
    likely_voice_skipped = [c for c in missing_expected if c.lower() not in ("set_voice",)]
    if likely_voice_skipped:
        soft_hints.append(
            f"voice cycle completed (TTS finished x{tts_finished_count}, "
            f"speak_text x{speak_text_count}) but expected tool call(s) "
            f"skipped: {', '.join(likely_voice_skipped)}. "
            f"LLM сделал verbal-only answer (RULE #MUSIC для stop_music / "
            f"RULE #VOICE-MULTI для multi-voice могут не enforce). "
            f"Проверь master_prompt_compact.txt и/или добавь explicit "
            f"tool-call enforcement в LLM-system reminder."
        )
        # Не добавляем к failures (это hint, не блокер GATE-1), но логируем
        # через stderr чтобы в verdict было видно.
        sys.stderr.write(
            f"[hint] GATE-1 soft-fail: voice-cycle OK but tool skipped — "
            f"{', '.join(likely_voice_skipped)}\n"
        )

result = {
    "gate": "GATE-1",
    "name": acc.get("name", ""),
    "acceptance_file": acc_path,
    "expected_tool_calls": expected_call,
    "must_not_call": must_not,
    "actual_tool_calls": actual_calls,
    "found_expected_calls": found_expected,
    "missing_expected_calls": missing_expected,
    "forbidden_calls": forbidden_called,
    "voice_cycle_count": tts_finished_count,
    "speak_text_count": speak_text_count,
    # Issue #2406: discovery-step enforcement (top-level aggregate).
    "discovery_tools": discovery_tools,
    "discovery_records": discovery_records,
    "discovery_first_voice_pos": voice_pos,
    "soft_hints": soft_hints,
    "pass": not failures,
    "reason": "; ".join(failures + soft_hints) if (failures or soft_hints) else "all checks passed",
}
sys.stdout.write(json.dumps(result, ensure_ascii=False, indent=2))
PY

    if grep -q '"pass": *true' "$OUT_DIR/acceptance.json"; then
        log "GATE-1: ✅ $(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["reason"])' "$OUT_DIR/acceptance.json")"
        return 0
    else
        log "GATE-1: ❌ $(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["reason"])' "$OUT_DIR/acceptance.json")"
        return 1
    fi
}
# Синтез Yandex TTS gRPC v3: text + voice → /tmp/e2e_v2_<run>/cmd.wav
# Тот же контракт что tts_node._synthesize_yandex (tts.api.cloud.yandex.net:443)
# Ретро 22.08 t_c7761956 (A2): synth failure теперь печатает JSON-строку с
# diagnostic (provider, error_class, http_code, latency_ms). Это позволяет
# сразу отличать network/quota/auth ошибки без grep stacktrace.
synth_yandex() {  # $1=text $2=voice $3=out_wav
    local text="$1" voice="$2" out="$3"
    local _start_ms _end_ms _latency_ms
    _start_ms="$(date +%s%3N 2>/dev/null || date +%s)000"
    python3 - "$text" "$voice" "$out" <<'PY'
import sys, grpc, wave, io, json, time, traceback
from yandex.cloud.ai.tts.v3 import tts_pb2, tts_service_pb2_grpc
text, voice, out = sys.argv[1], sys.argv[2], sys.argv[3]
import os
key = os.environ["YANDEX_API_KEY"]
start = time.monotonic()
err_class = "Unknown"
err_detail = ""
try:
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
        err_class = "EmptyResponse"
        err_detail = "Yandex TTS returned empty audio stream"
        latency_ms = int((time.monotonic() - start) * 1000)
        sys.stderr.write(json.dumps({
            "provider": "yandex",
            "voice": voice,
            "result": "fail",
            "error_class": err_class,
            "detail": err_detail,
            "latency_ms": latency_ms,
        }) + "\n")
        sys.exit("YANDEX_EMPTY")
    with open(out, "wb") as f:
        f.write(data)
    latency_ms = int((time.monotonic() - start) * 1000)
    print(f"YANDEX_SYNTH_OK {len(data)} bytes voice={voice}")
    sys.stderr.write(json.dumps({
        "provider": "yandex",
        "voice": voice,
        "result": "ok",
        "bytes": len(data),
        "latency_ms": latency_ms,
    }) + "\n")
except grpc.RpcError as e:
    latency_ms = int((time.monotonic() - start) * 1000)
    code = e.code() if hasattr(e, "code") else None
    code_name = code.name if code and hasattr(code, "name") else "Unknown"
    details = e.details() if hasattr(e, "details") else str(e)
    # Классификация: Unavailable/Timeout/Deadline → NetworkError,
    # Unauthenticated/PermissionDenied → AuthError, ResourceExhausted → QuotaError
    if code_name in ("UNAVAILABLE", "DEADLINE_EXCEEDED", "CANCELLED"):
        err_class = "NetworkError"
    elif code_name in ("UNAUTHENTICATED", "PERMISSION_DENIED"):
        err_class = "AuthError"
    elif code_name in ("RESOURCE_EXHAUSTED", "OUT_OF_RANGE"):
        err_class = "QuotaError"
    elif code_name in ("INVALID_ARGUMENT",):
        err_class = "BadRequest"
    else:
        err_class = f"GrpcError_{code_name}"
    sys.stderr.write(json.dumps({
        "provider": "yandex",
        "voice": voice,
        "result": "fail",
        "error_class": err_class,
        "http_code": code_name,
        "detail": details[:500],
        "latency_ms": latency_ms,
    }) + "\n")
    sys.exit(f"YANDEX_{err_class.upper()}")
except Exception as e:
    latency_ms = int((time.monotonic() - start) * 1000)
    err_class = "PythonError"
    detail = f"{type(e).__name__}: {e}"
    sys.stderr.write(json.dumps({
        "provider": "yandex",
        "voice": voice,
        "result": "fail",
        "error_class": err_class,
        "detail": detail[:500],
        "trace": traceback.format_exc()[:500],
        "latency_ms": latency_ms,
    }) + "\n")
    sys.exit(f"YANDEX_{err_class.upper()}")
PY
}

# Синтез MiniMax T2A v2 (HTTP): text + voice → out_wav.
# Тот же контракт, что tts_node._synthesize_minimax → rob_box_llm
# MiniMaxTTSProvider (POST /v1/t2a_v2, hex-кодированное audio в data.audio).
# Просим сразу wav: харнесс не ресемплит сам, ffmpeg-EQ ниже разберётся.
#
# Классификация ошибок зеркалит minimax_tts.py:933-954 и НЕ читается по
# HTTP-статусу: «план не поддерживает модель» приезжает 500-м, а исчерпанная
# квота — 200-м с base_resp.status_code=2056. Поэтому проверяем ОБА канала.
synth_minimax() {  # $1=text $2=voice $3=out_wav
    local text="$1" voice="$2" out="$3"
    python3 - "$text" "$voice" "$out" <<'PY'
import sys, os, json, time
text, voice, out = sys.argv[1], sys.argv[2], sys.argv[3]
key = os.environ.get("MINIMAX_API_KEY", "")
start = time.monotonic()

def fail(err_class, detail, http_code=None):
    payload = {
        "provider": "minimax", "voice": voice, "result": "fail",
        "error_class": err_class, "detail": str(detail)[:500],
        "latency_ms": int((time.monotonic() - start) * 1000),
    }
    if http_code is not None:
        payload["http_code"] = http_code
    sys.stderr.write(json.dumps(payload, ensure_ascii=False) + "\n")
    sys.exit("MINIMAX_" + err_class.upper())

if not key:
    fail("AuthError", "MINIMAX_API_KEY не задан")
try:
    import requests
except ImportError as exc:
    fail("PythonError", "requests не установлен: %s" % exc)

payload = {
    "model": os.environ.get("MINIMAX_TTS_MODEL", "speech-02-hd"),
    "text": text,
    "stream": False,
    "voice_setting": {
        "voice_id": voice,
        "speed": float(os.environ.get("YANDEX_SPEED", "1.0")),
    },
    "audio_setting": {
        "sample_rate": 32000, "bitrate": 128000, "format": "wav", "channel": 1,
    },
}
base = os.environ.get("MINIMAX_TTS_BASE_URL", "https://api.minimax.io").rstrip("/")
try:
    r = requests.post(
        base + "/v1/t2a_v2",
        headers={"Authorization": "Bearer " + key, "Content-Type": "application/json"},
        json=payload, timeout=60,
    )
except Exception as exc:
    fail("NetworkError", "%s: %s" % (type(exc).__name__, exc))

try:
    body = r.json()
except Exception:
    fail("BadResponse", "HTTP %s, не JSON: %s" % (r.status_code, r.text[:200]), r.status_code)

# Канал 1: HTTP >= 400 (тело — {"error": {"message": ...}}).
# Канал 2: HTTP 200 + base_resp.status_code != 0 (так приезжает квота).
base_resp = body.get("base_resp") or {}
status = int(base_resp.get("status_code", 0) or 0)
msg = str(base_resp.get("status_msg", "") or "")
if r.status_code >= 400 or status != 0:
    if not msg:
        msg = str((body.get("error") or {}).get("message", "") or r.text[:200])
    low = msg.lower()
    if "auth" in low or "key" in low or "token" in low or "plan" in low:
        err_class = "AuthError"
    elif "quota" in low or "rate" in low or "limit" in low or "balance" in low:
        err_class = "QuotaError"
    elif "invalid" in low or "param" in low or "voice" in low:
        err_class = "BadRequest"
    else:
        err_class = "ApiError"
    fail(err_class, "minimax API error %s: %s" % (status, msg), r.status_code)

audio_hex = (body.get("data") or {}).get("audio") or ""
if not audio_hex:
    fail("EmptyResponse", "minimax ответил без data.audio", r.status_code)
try:
    data = bytes.fromhex(audio_hex)
except ValueError as exc:
    fail("BadResponse", "data.audio не hex: %s" % exc, r.status_code)
with open(out, "wb") as f:
    f.write(data)
print("MINIMAX_SYNTH_OK %d bytes voice=%s" % (len(data), voice))
sys.stderr.write(json.dumps({
    "provider": "minimax", "voice": voice, "result": "ok",
    "bytes": len(data), "http_code": r.status_code,
    "latency_ms": int((time.monotonic() - start) * 1000),
}, ensure_ascii=False) + "\n")
PY
}

# Синтез Silero (локальный torch на ХОСТЕ katana 10.1.1.249, НЕ в раннер-
# контейнере — см. блок про E2E_TTS_PROVIDER_ORDER): text + voice → out_wav.
# Ключей не требует и в сеть не ходит — это тот самый «всегда живой» донор,
# ради которого затевался выбор провайдера.
#
# Модель — torch.package (НЕ torch.jit), как в tts_node:2007. Путь ищем в том
# же порядке, что нода, плюс кеши билд-машины; если нигде нет — тянем через
# torch.hub (один раз, дальше он кеширует в ~/.cache/torch/hub).
# Загрузка модели ~0.6s + синтез ~2s на фразу (замер на 10.1.1.249): дешевле
# сетевого round-trip, поэтому держать модель между шагами смысла нет.
synth_silero() {  # $1=text $2=voice $3=out_wav
    local text="$1" voice="$2" out="$3"
    python3 - "$text" "$voice" "$out" <<'PY'
import sys, os, json, time, wave, traceback
text, voice, out = sys.argv[1], sys.argv[2], sys.argv[3]
start = time.monotonic()

def fail(err_class, detail):
    sys.stderr.write(json.dumps({
        "provider": "silero", "voice": voice, "result": "fail",
        "error_class": err_class, "detail": str(detail)[:500],
        "latency_ms": int((time.monotonic() - start) * 1000),
    }, ensure_ascii=False) + "\n")
    sys.exit("SILERO_" + err_class.upper())

def warn(detail):
    sys.stderr.write(json.dumps({
        "provider": "silero", "result": "warn", "detail": str(detail)[:500],
    }, ensure_ascii=False) + "\n")

try:
    import torch
    import numpy as np
except ImportError as exc:
    # hostname у раннер-контейнеров совпадает с хостовым, поэтому диагностируем
    # по интерпретатору: на 249 это /usr/bin/python3 3.10 с ~ros2/.local.
    fail("PythonError",
         "torch/numpy недоступны (%s). python=%s %s. Харнесс должен исполняться "
         "на ХОСТЕ katana 10.1.1.249, где они стоят в ~ros2/.local; внутри "
         "build-github-runner-* их нет и быть не должно."
         % (exc, sys.executable, sys.version.split()[0]))

torch.set_grad_enabled(False)
torch.set_num_threads(int(os.environ.get("E2E_SILERO_THREADS", "4")))

home = os.path.expanduser("~")
candidates = [c for c in [
    os.environ.get("E2E_SILERO_MODEL") or "",
    "/models/silero/v5_ru.pt",
    "/cache/tts/silero_v5_ru.pt",
    home + "/.cache/rob_box_voice/tts_models/v5_ru.pt",
    home + "/.cache/rob_box_voice/tts_models/v4_ru.pt",
    home + "/.cache/torch/hub/snakers4_silero-models_master/src/silero/model/v4_ru.pt",
] if c]

model = None
used = ""
for path in candidates:
    if not os.path.exists(path):
        continue
    try:
        model = torch.package.PackageImporter(path).load_pickle("tts_models", "model")
        used = path
        break
    except Exception as exc:
        warn("%s: %s: %s" % (path, type(exc).__name__, exc))
if model is None:
    try:
        model, _ = torch.hub.load(
            repo_or_dir="snakers4/silero-models", model="silero_tts",
            language="ru", speaker="v5_ru",
        )
        used = "torch.hub:v5_ru"
    except Exception as exc:
        fail("ModelMissing", "нет локальной модели %s и torch.hub упал: %s" % (candidates, exc))
model.to(torch.device("cpu"))

# Голос, которого нет у модели, Silero встречает исключением — подставляем
# дефолт ноды (aidar), а не роняем шаг: сценарий всё равно будет озвучен.
speakers = list(getattr(model, "speakers", []) or [])
if speakers and voice not in speakers:
    warn("голос %r не в каталоге %s — беру aidar" % (voice, speakers))
    voice = "aidar" if "aidar" in speakers else speakers[0]

rate = int(os.environ.get("E2E_SILERO_SAMPLE_RATE", "48000"))
try:
    audio = model.apply_tts(
        ssml_text='<speak><prosody pitch="medium">' + text + "</prosody></speak>",
        speaker=voice, sample_rate=rate,
        put_accent=True, put_yo=True,
    )
except Exception as exc:
    fail("SynthError", "%s: %s | %s" % (type(exc).__name__, exc, traceback.format_exc()[:300]))

samples = audio.numpy()
if samples.size == 0:
    fail("EmptyResponse", "Silero вернул пустой тензор")
pcm = (np.clip(samples, -1.0, 1.0) * 32767).astype(np.int16)
with wave.open(out, "wb") as w:
    w.setnchannels(1)
    w.setsampwidth(2)
    w.setframerate(rate)
    w.writeframes(pcm.tobytes())
size = os.path.getsize(out)
print("SILERO_SYNTH_OK %d bytes voice=%s model=%s" % (size, voice, used))
sys.stderr.write(json.dumps({
    "provider": "silero", "voice": voice, "result": "ok",
    "bytes": size, "model": used,
    "latency_ms": int((time.monotonic() - start) * 1000),
}, ensure_ascii=False) + "\n")
PY
}

# tts_probe_provider() — «этот провайдер вообще живой?». Синтезирует короткую
# фразу во временный файл. 0 = живой, 1 = нет.
tts_probe_provider() {  # $1=provider
    local provider="$1" probe_wav probe_log rc=0
    probe_wav="$(mktemp -u /tmp/e2e_tts_probe_XXXXXX.wav)"
    probe_log="$(mktemp -u /tmp/e2e_tts_probe_XXXXXX.log)"
    case "$provider" in
        yandex)
            [ -n "${YANDEX_API_KEY:-}" ] || { log "TTS probe yandex: пропуск — нет YANDEX_API_KEY"; return 1; }
            synth_yandex "проверка связи" "anton" "$probe_wav" > "$probe_log" 2>&1 || rc=$?
            ;;
        minimax)
            [ -n "${MINIMAX_API_KEY:-}" ] || { log "TTS probe minimax: пропуск — нет MINIMAX_API_KEY"; return 1; }
            synth_minimax "проверка связи" "Russian_ReliableMan" "$probe_wav" > "$probe_log" 2>&1 || rc=$?
            ;;
        silero)
            synth_silero "проверка связи" "aidar" "$probe_wav" > "$probe_log" 2>&1 || rc=$?
            ;;
        *)
            log "TTS probe: неизвестный провайдер '$provider' — пропуск"
            return 1
            ;;
    esac
    if [ "$rc" != "0" ] || [ ! -s "$probe_wav" ]; then
        log "TTS probe ${provider}: FAIL $(tail -1 "$probe_log" 2>/dev/null)"
        rm -f "$probe_wav" "$probe_log"
        return 1
    fi
    log "TTS probe ${provider}: OK $(head -1 "$probe_log" 2>/dev/null)"
    rm -f "$probe_wav" "$probe_log"
    return 0
}

# resolve_tts_provider() — один раз за прогон выбирает провайдера синтеза и
# кладёт его в E2E_TTS_PROVIDER_RESOLVED. Явно заданный провайдер НЕ
# пробуется: если запросили yandex — падаем на yandex'е, а не уезжаем тихо на
# silero (иначе прогон «зелёный», но проверяли не то, что просили).
resolve_tts_provider() {
    [ -n "$E2E_TTS_PROVIDER_RESOLVED" ] && return 0
    if [ "$E2E_TTS_PROVIDER" != "auto" ]; then
        E2E_TTS_PROVIDER_RESOLVED="$E2E_TTS_PROVIDER"
        log "TTS provider: ${E2E_TTS_PROVIDER_RESOLVED} (задан явно, без пробы)"
        echo "E2E_TTS_PROVIDER ${E2E_TTS_PROVIDER_RESOLVED} explicit"
        return 0
    fi
    local candidate
    log "TTS provider: auto — пробую по очереди ${E2E_TTS_PROVIDER_ORDER}"
    IFS=',' read -r -a _tts_order <<< "$E2E_TTS_PROVIDER_ORDER"
    for candidate in "${_tts_order[@]}"; do
        candidate="$(printf '%s' "$candidate" | tr -d '[:space:]')"
        [ -n "$candidate" ] || continue
        if tts_probe_provider "$candidate"; then
            E2E_TTS_PROVIDER_RESOLVED="$candidate"
            log "TTS provider: выбран ${candidate} (auto)"
            echo "E2E_TTS_PROVIDER ${candidate} auto"
            return 0
        fi
    done
    log "TTS provider: ни один кандидат из ${E2E_TTS_PROVIDER_ORDER} не синтезирует"
    echo "E2E_TTS_PROVIDER none auto"
    return 1
}

# synth_command() — единственная точка синтеза команды для run_step.
# Резолвит провайдера (лениво, один раз), переводит голос сценария в каталог
# провайдера (map_tts_voice из e2e_voice_lib.sh) и диспатчит.
synth_command() {  # $1=text $2=voice $3=out_wav
    local text="$1" voice="$2" out="$3" provider native
    if ! resolve_tts_provider; then
        echo '{"provider": "none", "result": "fail", "error_class": "NoProvider", "detail": "ни один TTS-провайдер не доступен"}' >&2
        return 1
    fi
    provider="$E2E_TTS_PROVIDER_RESOLVED"
    native="$(map_tts_voice "$provider" "$voice")"
    case "$provider" in
        yandex)  synth_yandex  "$text" "$native" "$out" ;;
        minimax) synth_minimax "$text" "$native" "$out" ;;
        silero)  synth_silero  "$text" "$native" "$out" ;;
        *)       echo "{\"provider\": \"$provider\", \"result\": \"fail\", \"error_class\": \"UnknownProvider\"}" >&2; return 1 ;;
    esac
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

# Проверка накопления фоновой речи в SpeechAccumulator (backlog-аккумулятор,
# 2026-08-20-voice-backlog-accumulator-design.md). Фраза БЕЗ wake-слова НЕ
# дропается, а копится; маркер — 🗒️ [backlog] accumulated (no_wake_word).
# Используется для шагов с expect=backlog (ds01/ds02 voice_core_suite_v1.json),
# где полного TTS-цикла не будет (wake-gate молча аккумулирует).
# Возвращает 0 если маркер найден, 1 если нет.
check_backlog_accumulated() {  # $1=before_rfc3339
    local before="$1"
    local logs
    logs="$(${ROBOT_SSH} "docker logs voice-assistant --since '${before}' 2>&1" 2>/dev/null || echo '')"
    printf '%s' "$logs" | grep -qE '\[backlog\] accumulated \(no_wake_word\)'
}

# vad_reject_reason() — почему шаг НЕ доехал до STT (issue: ложный диагноз).
#
# bug(run 35658231116, 22.09.2026). Шаги n303_bg_boris / n304_bg_grisha_unknown
# отчитались `FAIL backlog_miss` — «backlog accumulation не подтверждён», то
# есть обвинили фичу бэклога. В docker logs робота за то же окно лежало:
#
#   ❌ Речь отклонена: 17.37с (min=0.3, max=15.0)
#
# audio_node выбросил фразу ПО ДЛИНЕ, ещё до STT: реплики этих шагов в
# синтезе minimax звучат 16-17.4с против speech_max_duration=15.0. Ни бэклог,
# ни диаризация, ни LLM в этом не участвовали вообще. Рядом n305 прошёл на
# 14.81с — то есть сценарий был лотереей с зазором 0.19с, а не проверкой.
#
# Диагноз «backlog_miss» отправлял ретро-инженера искать регресс в
# dialogue_node, которого там нет. По ADR-0018 честный FAIL обязан называть
# настоящую причину: возвращаем её отдельным kind'ом (vad_rejected), с
# измеренной длительностью и лимитом в тексте шага.
#
# Печатает в stdout человекочитаемую причину и возвращает 0, если фраза была
# отброшена до STT. Если отказа не было — печатает пусто и возвращает 1.
vad_reject_reason() {  # $1=before_rfc3339
    local before="$1" logs hit
    logs="$(${ROBOT_SSH} "docker logs voice-assistant --since '${before}' 2>&1" 2>/dev/null || echo '')"
    # ВАЖЕН ПОРЯДОК: 0.00с проверяем ДО общего «вне окна», иначе общая ветка
    # перехватывает пустой буфер и теряет подсказку про restart (поймано
    # тестом scripts/testing/test_e2e_vad_reject_diagnosis.sh, CASE 4).
    #
    # Захват аудио мёртв (ретро 17.09: «простоял ночь — не слышит голос»):
    # 0.00с означает, что буфер пуст, а не что фраза короткая.
    if printf '%s' "$logs" | grep -qE 'Речь отклонена: 0\.00с'; then
        printf 'vad_rejected — audio_node отдаёт пустой буфер (Речь отклонена: 0.00с): захват микрофона мёртв, лечится docker restart voice-assistant'
        return 0
    fi
    # Длина вне окна VAD (главный случай — слишком длинная реплика сценария).
    hit="$(printf '%s' "$logs" | grep -oE 'Речь отклонена: [0-9.]+с \(min=[0-9.]+, max=[0-9.]+\)' | tail -1)"
    if [ -n "$hit" ]; then
        printf 'vad_rejected — audio_node отбросил фразу ДО STT: %s' "$hit"
        return 0
    fi
    if printf '%s' "$logs" | grep -qE 'устройство не найдено'; then
        printf 'vad_rejected — audio_node не нашёл ReSpeaker (устройство не найдено): инфра, не робот-логика'
        return 0
    fi
    return 1
}

# emit_step_fail_or_vad() — единая точка вердикта для шагов, которые «не
# долетели». Если audio_node отбросил звук до STT, пишем настоящую причину;
# иначе — исходный kind, как было.
#   $1=label  $2=before_rfc3339  $3=fallback_marker  $4=fallback_kind
emit_step_fail_or_vad() {
    local label="$1" before="$2" fallback_marker="$3" fallback_kind="$4" reason
    if reason="$(vad_reject_reason "$before")"; then
        log "STEP ${label}: ❌ ${reason}"
        log "STEP ${label}: это НЕ регресс робота — фраза сценария не доехала до STT"
        emit_step "${label} FAIL vad_rejected"
        printf '%s\n' "$reason" >> "$OUT_DIR/vad_rejects.log" 2>/dev/null || true
        mark_fail_kind infra
        return 1
    fi
    emit_step "${label} ${fallback_marker}"
    mark_fail_kind "$fallback_kind"
    return 1
}

# --- один атомарный шаг -----------------------------------------------------
# Ожидаемое поведение определяется параметром $4 (expect_kind):
#   cycle       — полный цикл STT→LLM→TTS (по дефолту)
#   wake-gated  — то же, но ADR-0029 §2.3: если WAKE_GATE_CLEARED != 1
#                 (cold-start not cleared) → SKIP без fail (см. retro
#                 t_be491fba: cold-start wake-gate flake должен быть
#                 отделён от acceptance fail). Логирует `[skip:wake-gate-cold-start]`.
#   backlog     — фраза без wake-prefix копится в SpeechAccumulator
#
# run_step намеренно принимает уже-classified expect_kind (callers
# используют classify_step_expect для дефолта и override).
run_step() {  # $1=text $2=voice $3=step_label $4=expect_kind(cycle|wake-gated|backlog)
    local text="$1" voice="$2" label="$3" expect="${4:-cycle}"
    # BUG-B (t_f0612a43): если label содержит кириллицу/спецсимволы — slugify
    # для имени wav файла. Исходный label сохраняется для логов.
    local safe
    safe="$(safe_label "$label")"
    log "=== STEP ${label} (safe=${safe}): voice=${voice} text=\"${text}\" ==="

    # 0. ADR-0029 §2.3: wake-gated SKIP (retro t_be491fba). Если
    #    expect=wake-gated И preflight показал cold-start NOT cleared —
    #    шаг пропускается (SKIP), не FAIL. Это by design поведение
    #    backlog-аккумулятора: «Робот, ...» без wake → STT получает
    #    «...» (без «Робот»), dialogue_node копит в [backlog] вместо
    #    вызова LLM. Acceptance fail (missing stop_music) — это
    #    симптом cold-start flake, а не acceptance flake.
    if [ "$expect" = "wake-gated" ] && [ "${WAKE_GATE_CLEARED:-0}" != "1" ]; then
        # ПЕРЕПРОБА перед пропуском.
        #
        # bug(22.09.2026, поймано на живом прогоне 35664554084 до того, как
        # испортило марафон). Preflight зовёт wake_gate_cleared_since
        # "$E2E_RUN_BEFORE", а это `docker logs --since <старт прогона>`:
        # в момент старта окно ПУСТО по определению, поэтому ответ всегда
        # «cold-start NOT cleared». Пока preflight был мёртвым кодом, это
        # никого не трогало. Как только он заработал, гард начал резать всё:
        # в night-marathon 101 шаг из 125 не имеет явного expect и начинается
        # с «Робот», а classify_step_expect авто-повышает такие шаги до
        # wake-gated. То есть прогон отдал бы 101 SKIP и выглядел «не
        # красным», не проверив ничего. Это хуже любого FAIL.
        #
        # Семантика гарда — «робот ещё не проснулся», а не «прогон только
        # начался». К моменту, когда до wake-gated шага дошла очередь,
        # предыдущие шаги акта уже отдали свои «✅ ПРИНЯТО (respeaker): Робот,
        # ...», и окно с E2E_RUN_BEFORE больше не пусто. Поэтому спрашиваем
        # ЗАНОВО, здесь и сейчас, и пропускаем шаг только если гейт всё ещё
        # не пройден.
        if wake_gate_cleared_since "$E2E_RUN_BEFORE"; then
            WAKE_GATE_CLEARED=1
            WAKE_GATE_PREFLIGHT_REASON="cold-start cleared (перепроба на шаге ${label})"
            log "STEP ${label}: wake-gate прогрелся к этому моменту (перепроба) — шаг выполняется, не пропускается"
        elif [ "${WAKE_GATE_PROBE_SPENT:-0}" != "1" ]; then
            # Каскад-гард. Если первый же wake-gated шаг акта пропустить,
            # акцепта не появится никогда, следующая перепроба снова будет
            # пустой — и весь акт уйдёт в SKIP, ничего не проверив.
            # Разрываем это тем же доводом, что уже записан в single-text
            # ветке preflight'а: одиночный wake-шаг НЕ требует гейта, он и
            # ЕСТЬ cold-start проба. Поэтому ПЕРВЫЙ wake-gated шаг прогона
            # всегда играем, а гейт применяем к следующим — когда у нас уже
            # есть настоящий ответ, прогрелся робот или нет.
            WAKE_GATE_PROBE_SPENT=1
            log "STEP ${label}: wake-gate ещё не пройден, но это ПЕРВЫЙ wake-gated шаг прогона — играем его как cold-start пробу (пропускать нечего: без акцепта гейт не пройдёт никогда)"
        else
            log "STEP ${label}: SKIP [skip:wake-gate-cold-start] — ${WAKE_GATE_PREFLIGHT_REASON:-cold-start not cleared} (перепроба подтвердила, cold-start проба уже израсходована)"
            emit_step "${label} SKIP wake-gate-cold-start"
            # Не помечаем fail_kind — это не fail. Возвращаем специальный
            # код 3, который callers (scenario loop) интерпретируют как
            # «пропущен по systemic, не считать в aggregate FAIL».
            return 3
        fi
    fi

    # 1. Синтез команды
    ensure_outdir
    if [ ! -f "$OUT_DIR/cmd_${safe}.wav" ]; then
        # Файл мог пропасть вместе с OUT_DIR (внешний cleanup на 249) —
        # пере-синтезируем, а не падаем с paplay open(): No such file.
        log "STEP ${label}: cmd_${safe}.wav отсутствует — повторный синтез (cleanup-resilience)"
    fi
    if ! synth_command "$text" "$voice" "$OUT_DIR/cmd_${safe}.wav" > "$OUT_DIR/synth_${safe}.log" 2>&1; then
        log "STEP ${label}: FAIL — синтез ${E2E_TTS_PROVIDER_RESOLVED:-$E2E_TTS_PROVIDER} упал ($(tail -1 "$OUT_DIR/synth_${safe}.log"))"
        emit_step "${label} FAIL synth"
        mark_fail_kind synth
        return 1
    fi
    # EQ: highpass 200 + volume 1.2 + alimiter (клиппинг-фикс 514e7e87)
    ensure_outdir
    ffmpeg -nostdin -y -i "$OUT_DIR/cmd_${safe}.wav" -af "highpass=f=100,volume=3.0,alimiter=limit=0.98,adelay=1500|all=1" -ac 1 -ar 16000 "$OUT_DIR/cmd_${safe}_eq.wav" 2>/dev/null

    # 1b. Реплика влезает в окно VAD робота?
    #
    # bug(run 35658231116, 22.09.2026). Реплика n303_bg_boris в синтезе
    # minimax звучит 17.37с, окно audio_node — speech_max_duration=15.0.
    # audio_node выбрасывал фразу ДО STT, а харнесс тратил на неё две
    # попытки по react_window и отчитывался `FAIL backlog_miss`, то есть
    # обвинял dialogue_node. Соседний n305 прошёл на 14.81с — зазор 0.19с,
    # так что «зелёный» шаг был лотереей на скорости речи провайдера.
    #
    # Здесь wav уже синтезирован — мерим ФАКТ, а не оцениваем по символам.
    # Отдаём вердикт сразу: это дешевле (не жжём retry-окна) и честнее
    # (называем настоящую причину, а не симптом на конце цепочки).
    #
    # Мерим ИМЕННО _eq.wav — это файл, который реально уходит в paplay.
    # И сравниваем НЕ с speech_max_duration напрямую: audio_node мерит не
    # длину файла, а окно от первой речи до последней, с VAD-hangover'ом.
    # Замеры на прогоне 35658231116 (eq-длительность → что насчитал VAD):
    #     n301  6.23 → 9.95    n302  9.81 → 12.51   n303 14.84 → 17.37 ❌
    #     n304 13.39 → 16.09❌  n305 12.41 → 15.07❌ n306  9.13 → 13.28
    #     n307  6.27 →  8.67   n308  7.42 →  9.95
    # Накладка стабильна и лежит в 2.4-4.2s. Берём минимум наблюдённого
    # (2.5s) — это самый мягкий порог, который всё ещё правильно
    # классифицирует все восемь шагов выше (n303/n304/n305 ловятся,
    # n302/n306 не ловятся). Порог по файлу = speech_max_duration - 2.5.
    #
    # Дефолты синхронизированы с docker/vision/config/voice_assistant/
    # audio_node.yaml → audio_node → ros__parameters → speech_max_duration.
    # Рассинхрон дефолта и конфига ловит
    # scripts/testing/test_e2e_scenario_playable.sh.
    E2E_VAD_MAX_DURATION="${E2E_VAD_MAX_DURATION:-15.0}"
    E2E_VAD_OVERHEAD="${E2E_VAD_OVERHEAD:-2.5}"
    local cmd_dur vad_budget
    cmd_dur="$(ffprobe -v error -show_entries format=duration -of csv=p=0 \
        "$OUT_DIR/cmd_${safe}_eq.wav" 2>/dev/null | head -1)"
    vad_budget="$(awk -v m="$E2E_VAD_MAX_DURATION" -v o="$E2E_VAD_OVERHEAD" 'BEGIN{printf "%.2f", m-o}')"
    case "$cmd_dur" in
        ''|*[!0-9.]*)
            # ffprobe нет или ответил мусором — молча пропускаем проверку.
            # Это advisory-гейт: он не имеет права сам ронять прогон.
            log "STEP ${label}: длительность реплики не измерена (ffprobe недоступен) — проверка окна VAD пропущена" ;;
        *)
            if awk -v d="$cmd_dur" -v b="$vad_budget" 'BEGIN{exit !(d>b)}'; then
                log "STEP ${label}: ❌ реплика ${cmd_dur}s не влезает в окно VAD робота (бюджет ${vad_budget}s = speech_max_duration ${E2E_VAD_MAX_DURATION}s - VAD-накладка ${E2E_VAD_OVERHEAD}s)"
                log "STEP ${label}: audio_node отбросит её до STT — играть бессмысленно. Это баг СЦЕНАРИЯ (реплику надо разбить), не робота."
                emit_step "${label} FAIL scenario_too_long (${cmd_dur}s > ${vad_budget}s)"
                printf 'STEP %s: eq=%ss > budget %ss (speech_max_duration=%s, overhead=%s, provider=%s voice=%s)\n' \
                    "$label" "$cmd_dur" "$vad_budget" "$E2E_VAD_MAX_DURATION" "$E2E_VAD_OVERHEAD" \
                    "${E2E_TTS_PROVIDER_RESOLVED:-$E2E_TTS_PROVIDER}" "$voice" \
                    >> "$OUT_DIR/vad_rejects.log" 2>/dev/null || true
                mark_fail_kind infra
                return 1
            fi
            # Меньше секунды запаса — шаг лотерея: пройдёт или нет, зависит от
            # скорости речи провайдера. Молчать об этом нельзя: n305 набрал
            # 12.41s при бюджете 12.50s и в первой попытке всё равно отвалился.
            if awk -v d="$cmd_dur" -v b="$vad_budget" 'BEGIN{exit !(b-d<1.0)}'; then
                log "STEP ${label}: ⚠️ реплика ${cmd_dur}s при бюджете ${vad_budget}s — запас < 1s, шаг на грани отказа"
            fi ;;
    esac

    # 2. Ждём тишины: робот не должен говорить перед командой (greeting/
    #    приветствие идёт через 12s после старта и может перебить команду).
    #    Ждём пока в логах нет свежих TTS-событий последние E2E_SILENCE_WAIT сек.
    E2E_SILENCE_WAIT="${E2E_SILENCE_WAIT:-15}"
    # Абсолютный предел ожидания. Окно тишины СБРАСЫВАЕТСЯ каждый раз, когда
    # робот заговорил, поэтому без такого предела цикл не завершается никогда:
    # зависший greeting/announce-луп, либо `docker logs --since 20s`, который
    # из-за гранулярности и расхождения часов продолжает отдавать те же старые
    # строки, — и шаг висит до внешнего таймаута job'а (45 мин). На выходе
    # «cancelled» без единого маркера шага, что для разбора хуже любого FAIL.
    E2E_SILENCE_WAIT_MAX="${E2E_SILENCE_WAIT_MAX:-$((E2E_SILENCE_WAIT * 6))}"
    local quiet_start quiet_end loop_start
    quiet_start="$(date -u +%s)"
    loop_start="$quiet_start"
    while true; do
        quiet_end="$(date -u +%s)"
        if [ $((quiet_end - quiet_start)) -ge "$E2E_SILENCE_WAIT" ]; then
            break
        fi
        if [ $((quiet_end - loop_start)) -ge "$E2E_SILENCE_WAIT_MAX" ]; then
            log "STEP ${label}: ⚠️ робот не замолчал за ${E2E_SILENCE_WAIT_MAX}s — играю команду поверх. Если шаг упадёт, смотри сюда: возможен зависший TTS-луп на роботе, а не регресс в обработке команды."
            printf 'STEP %s: silence wait exceeded %ss\n' "$label" "$E2E_SILENCE_WAIT_MAX" \
                >> "$OUT_DIR/silence_wait_exceeded.log" 2>/dev/null || true
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

    # 3a. Backlog-аккумулятор (ds01/ds02 voice_core_suite_v1.json): фраза
    #     БЕЗ wake-слова копится в SpeechAccumulator, полного TTS-цикла НЕ
    #     будет (wake-gate молча аккумулирует). check_cycle тут неприменим —
    #     ждём маркер 🗒️ [backlog] accumulated (no_wake_word), а не реакцию.
    if [ "$expect" = "backlog" ]; then
        local battempt
        for battempt in $(seq 1 "$E2E_MAX_ATTEMPTS"); do
            BEFORE="$(${ROBOT_SSH} "date -u +%Y-%m-%dT%H:%M:%SZ" 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
            log "STEP ${label}: PLAY backlog attempt ${battempt}/${E2E_MAX_ATTEMPTS}"
            pactl set-sink-volume @DEFAULT_SINK@ 100% 2>/dev/null || true
            paplay "$OUT_DIR/cmd_${safe}_eq.wav" && log "  PLAY_DONE" || log "  PLAY_FAIL"
            sleep "$E2E_REACTION_WINDOW"
            if check_backlog_accumulated "$BEFORE"; then
                log "STEP ${label}: ✅ backlog accumulated (no_wake_word)"
                emit_step "${label} OK backlog"
                return 0
            fi
            log "STEP ${label}: backlog-маркер не найден (attempt ${battempt}) — повтор"
            sleep "$E2E_RETRY_PAUSE"
        done
        # Прежде чем обвинить бэклог — проверяем, доехал ли звук до STT вообще.
        # run 35658231116: n303/n304 отчитались backlog_miss, а в логах робота
        # лежало `Речь отклонена: 17.37с (max=15.0)` — audio_node выбросил
        # фразу по длине, dialogue_node её не видел. BEFORE здесь — окно
        # последней попытки, ровно то, что нас интересует.
        log "STEP ${label}: ❌ backlog accumulation не подтверждён после ${E2E_MAX_ATTEMPTS} попыток"
        emit_step_fail_or_vad "$label" "$BEFORE" "FAIL backlog_miss" feature
        return 1
    fi

    # 3. Retry-цикл акцепта
    local attempt reaction rc
    reaction=0
    for attempt in $(seq 1 "$E2E_MAX_ATTEMPTS"); do
        BEFORE="$(${ROBOT_SSH} "date -u +%Y-%m-%dT%H:%M:%SZ" 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
        log "STEP ${label}: PLAY attempt ${attempt}/${E2E_MAX_ATTEMPTS}"
        # Громкость динамика 100% (по умолчанию; явный >100% доступен через
        # inputs.volume в workflow, e.g. -f volume=120).
        pactl set-sink-volume @DEFAULT_SINK@ 100% 2>/dev/null || true
        # cleanup-resilience (ретро 11.08 t_26a6d362): если eq-файл пропал
        # (OUT_DIR удалён внешним cleanup на 249) — пере-синтезируем и EQ,
        # а не получаем ложный FAIL от paplay open(): No such file.
        if [ ! -f "$OUT_DIR/cmd_${safe}_eq.wav" ]; then
            log "STEP ${label}: cmd_${safe}_eq.wav отсутствует перед play — пере-синтез (cleanup-resilience)"
            ensure_outdir
            synth_command "$text" "$voice" "$OUT_DIR/cmd_${safe}.wav" > "$OUT_DIR/synth_${safe}.log" 2>&1 \
                || { log "STEP ${label}: FAIL — повторный синтез ${E2E_TTS_PROVIDER_RESOLVED:-$E2E_TTS_PROVIDER} упал ($(tail -1 "$OUT_DIR/synth_${safe}.log"))"; emit_step "${label} FAIL synth"; mark_fail_kind synth; return 1; }
            ensure_outdir
            ffmpeg -nostdin -y -i "$OUT_DIR/cmd_${safe}.wav" -af "highpass=f=100,volume=3.0,alimiter=limit=0.98,adelay=1500|all=1" -ac 1 -ar 16000 "$OUT_DIR/cmd_${safe}_eq.wav" 2>/dev/null
        fi
        paplay "$OUT_DIR/cmd_${safe}_eq.wav" && log "  PLAY_DONE" || log "  PLAY_FAIL"
        sleep "$E2E_REACTION_WINDOW"

        check_cycle "$BEFORE"
        rc=$?
        if [ "$rc" = "0" ]; then
            log "STEP ${label}: ✅ ПОЛНЫЙ ЦИКЛ (акцепт + LLM + TTS)"
            reaction=1
            break
        elif [ "$rc" = "2" ]; then
            log "STEP ${label}: ❌ LLM/TTS ERROR — тест красный, не чиним"
            emit_step "${label} FAIL llm_error (см. $OUT_DIR/llm_error.txt)"
            mark_fail_kind llm_error
            return 2
        fi
        log "STEP ${label}: нет акцепта (attempt ${attempt}) — повтор"
        sleep "$E2E_RETRY_PAUSE"
    done

    if [ "$reaction" != "1" ]; then
        # Та же развилка, что и в backlog-ветке: «робот не ответил» и «робот
        # не услышал, потому что audio_node отбросил звук до STT» — разные
        # диагнозы, и второй не имеет права выглядеть как первый.
        log "STEP ${label}: ❌ NO_ACCEPT после ${E2E_MAX_ATTEMPTS} попыток"
        emit_step_fail_or_vad "$label" "$BEFORE" "FAIL no_accept" no_reaction
        return 1
    fi

    # 3. Проверка паттернов (если заданы для шага) — считываются из scenario
    return 0
}

# --- артефакты e2e (issue #1396) -------------------------------------------
# В дополнение к verdict.txt харнесс теперь пишет:
#   transcript.json   — что РЕАЛЬНО распознал STT (text + duration_ms + lang)
#   audio_metrics.json — RMS/peak/silence_ratio по recording.wav
#   baseline_diff.json — diff с golden (если задан) или synthetic baseline
#   acceptance.json   — результат проверки acceptance-блока сценария
# Все файлы кладутся в OUT_DIR (рядом с verdict.txt); workflow upload'ит их
# отдельными actions/upload-artifact шагами.
parse_transcript() {  # $1=label $2=before_rfc3339
    local label="$1" before="$2"
    local logs
    logs="$(${ROBOT_SSH} "docker logs voice-assistant --since '${before}' 2>&1" 2>/dev/null || echo '')"
    local text duration_s expected
    expected="$3"
    # «✅ ПРИНЯТО (<source>): <текст>» — основной маркер распознанной фразы.
    # stt_node.py:923: self.get_logger().info(f"✅ ПРИНЯТО ({source}): {text}")
    #
    # bug(run 35658231116, 22.09.2026): паттерн был '✅ ПРИНЯТО:' — без тега
    # источника, который появился в 6e016325f (#2011). Совпадений ноль, поэтому
    # "recognized" в transcript.json всегда пустой, а в лог каждого шага шло
    # «TRANSCRIPT[...]: STT не вернул фразу (нет '✅ ПРИНЯТО')» — даже там, где
    # сам харнесс строкой выше отчитался «✅ ПОЛНЫЙ ЦИКЛ (акцепт + LLM + TTS)».
    # Два взаимно противоречащих утверждения в одном логе; check_cycle грепает
    # просто "ПРИНЯТО" (без двоеточия) и потому работал.
    # Тег делаем опциональным — старые логи роботов до #2011 тоже читаются.
    text="$(printf '%s' "$logs" | grep -oE '✅ ПРИНЯТО( \([^)]*\))?:[[:space:]]*[^[:space:]].*' | head -1 | sed -E 's/^✅ ПРИНЯТО( \([^)]*\))?:[[:space:]]*//' | tr -d '\r')"
    # Длительность STT-сегмента: «Получена фраза: X.XXс» (stt_node.py:456)
    local phrase_line
    phrase_line="$(printf '%s' "$logs" | grep 'Получена фраза' | tail -1 || true)"
    duration_s="$(printf '%s' "$phrase_line" | grep -oE '[0-9]+\.[0-9]+с' | head -1 | tr -d 'с' || true)"
    # Latency STT: «Получена фраза» (wake) → «✅ ПРИНЯТО» (accept)
    local stt_latency_ms
    if [ -n "$duration_s" ] && [ -n "$expected" ]; then
        stt_latency_ms="null"
    else
        stt_latency_ms="null"
    fi
    # lang: ищем строку с явным указанием языка; если не нашли — ru-RU default
    local lang
    lang="$(printf '%s' "$logs" | grep -oE 'language=ru-RU|язык: [a-zA-Z-]+|lang=ru-RU' | head -1 | tr -d '\n' || true)"
    if [ -z "$lang" ]; then lang="ru-RU"; fi
    # JSON собирает python, а не heredoc. В heredoc `"expected": ${expected}`
    # подставлялся БЕЗ кавычек, и живой артефакт выглядел так:
    #     "expected": Робот, стоп музыку,
    # то есть transcript.json был невалидным JSON на каждом прогоне с текстом.
    # Его читает e2e_baseline_diff.py:203 под `except: pass` — поэтому
    # keyword_match_pct молча не считался никогда. Распознанная фраза приходит
    # из логов робота и тоже может содержать кавычки, так что эскейпить надо и
    # её: единственный надёжный способ — отдать сериализацию json.dumps.
    python3 - "$OUT_DIR/transcript.json" "${label:-single}" "${expected:-}" \
             "${text:-}" "${lang:-ru-RU}" "${duration_s:-}" "$phrase_line" <<'PY'
import sys, json
out, label, expected, text, lang, duration_s, phrase_line = sys.argv[1:8]

def num_or_none(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None

payload = {
    "label": label,
    "expected": expected or None,
    "recognized": text,
    "lang": lang or "ru-RU",
    "duration_s": num_or_none(duration_s),
    "stt_latency_ms": None,
    "raw_phrase_line": phrase_line[:200],
}
with open(out, "w", encoding="utf-8") as fh:
    json.dump(payload, fh, ensure_ascii=False, indent=2)
    fh.write("\n")
PY
    if [ -n "$text" ]; then
        log "TRANSCRIPT[${label}]: ожидалось «${expected:-?}», распознано «${text}»"
    else
        log "TRANSCRIPT[${label}]: STT не вернул фразу (нет '✅ ПРИНЯТО')"
    fi
}

# Пишет audio_metrics.json (RMS/peak/silence) и baseline_diff.json.
# Использует stdlib python3 (soundfile на билд-машине 249 не обязателен) +
# ffmpeg ebur128 если ffmpeg есть (для LUFS-метрик).
write_artifacts_audio() {
    local voice_text="${1:-}"
    local wav="$OUT_DIR/recording.wav"
    if [ -f "$wav" ]; then
        # bug(run 35658231116): путь был repo-relative (.github/workflows/...),
        # а харнесс на 249 живёт одиночным файлом в /tmp с CWD=$HOME ros2 —
        # python3 не находил скрипт НИ РАЗУ, и audio_metrics.json всегда был
        # 36-байтной заглушкой {"error":"audio_metrics.py failed"}. Тот же
        # класс, что и потерянные e2e_voice_lib.sh / e2e_voice_wake_gate.sh /
        # e2e_tool_match.py (см. комментарий в L-E2E Voice Test.yml): зовём
        # через $SCRIPT_DIR_E2E, а workflow обязан скопировать файл рядом.
        if [ ! -f "$SCRIPT_DIR_E2E/e2e_audio_metrics.py" ]; then
            echo '{"error":"e2e_audio_metrics.py not deployed","expected_at":"'"$SCRIPT_DIR_E2E/e2e_audio_metrics.py"'"}' \
                > "$OUT_DIR/audio_metrics.json"
        elif python3 "$SCRIPT_DIR_E2E/e2e_audio_metrics.py" "$wav" \
                > "$OUT_DIR/audio_metrics.json" 2>"$OUT_DIR/audio_metrics.stderr"; then
            :
        else
            # Не прячем stderr в /dev/null: до фикса причина отказа была
            # невидима и «метрика не считалась» списывалось на recorder.
            printf '{"error":"audio_metrics.py failed","stderr":%s}\n' \
                "$(python3 -c 'import json,sys;print(json.dumps(open(sys.argv[1],encoding="utf-8",errors="replace").read()[-2000:],ensure_ascii=False))' "$OUT_DIR/audio_metrics.stderr" 2>/dev/null || echo '"<unreadable>"')" \
                > "$OUT_DIR/audio_metrics.json"
        fi
        log "ARTIFACTS: audio_metrics.json written ($(stat -c%s "$OUT_DIR/audio_metrics.json") bytes)"
    else
        log "WARN: $wav не найден — audio_metrics.json не пишется (recorder не запустился?)"
        echo '{"error":"recording.wav not found","wav_path":"'"$wav"'"}' > "$OUT_DIR/audio_metrics.json"
    fi

    # baseline_diff — нужен transcript (чтобы извлечь expected_keywords если есть)
    local td="$OUT_DIR/transcript.json"
    [ ! -f "$td" ] && echo '{}' > "$td"
    if [ -f "$wav" ]; then
        if [ ! -f "$SCRIPT_DIR_E2E/e2e_baseline_diff.py" ]; then
            echo '{"error":"e2e_baseline_diff.py not deployed","expected_at":"'"$SCRIPT_DIR_E2E/e2e_baseline_diff.py"'"}' \
                > "$OUT_DIR/baseline_diff.json"
        elif python3 "$SCRIPT_DIR_E2E/e2e_baseline_diff.py" "$wav" "" "$voice_text" "$td" \
                > "$OUT_DIR/baseline_diff.json" 2>"$OUT_DIR/baseline_diff.stderr"; then
            :
        else
            printf '{"error":"baseline_diff.py failed","stderr":%s}\n' \
                "$(python3 -c 'import json,sys;print(json.dumps(open(sys.argv[1],encoding="utf-8",errors="replace").read()[-2000:],ensure_ascii=False))' "$OUT_DIR/baseline_diff.stderr" 2>/dev/null || echo '"<unreadable>"')" \
                > "$OUT_DIR/baseline_diff.json"
        fi
        log "ARTIFACTS: baseline_diff.json written"
    else
        echo '{"error":"recording.wav not found, baseline diff skipped"}' > "$OUT_DIR/baseline_diff.json"
    fi
}

# write_summary() — сводит прогон в ОДИН машиночитаемый файл summary.json
# и печатает человекочитаемый блок в stdout.
#
# Зачем: метрики (timing/audio/baseline/acceptance) считались и раньше, но
# лежали по разным zip-артефактам, которые надо скачать и открыть руками.
# В GitHub UI и в комментарии к issue человек видел только PASS/FAIL, поэтому
# «расчёт качества» существовал на бумаге и не существовал на практике.
# summary.json — то, что рендерится в Step Summary и попадает людям на глаза.
#
# Вердикт остаётся БИНАРНЫМ (ADR-0015): счётчик шагов здесь — доказательство,
# а не новая шкала. «9/11 OK» не превращает FAIL в PASS.
write_summary() {
    ensure_outdir
    [ -f "$OUT_DIR/steps.jsonl" ] || : > "$OUT_DIR/steps.jsonl"
    python3 - "$OUT_DIR" "${PASS:-0}" "${E2E_FAIL_KIND:-}" \
             "${E2E_TTS_PROVIDER_RESOLVED:-${E2E_TTS_PROVIDER:-}}" "${RUN_ID:-}" <<'PY'
import json, os, sys, time

out_dir, passed, fail_kind, tts_provider, run_id = sys.argv[1:6]

def read_json(name):
    path = os.path.join(out_dir, name)
    try:
        with open(path, encoding="utf-8") as fh:
            return json.load(fh)
    except Exception:
        return None

# Итог шага = ПОСЛЕДНЯЯ запись на label: run_step печатает промежуточный FAIL,
# scenario-цикл потом печатает финальный вердикт того же шага.
steps = {}
order = []
try:
    with open(os.path.join(out_dir, "steps.jsonl"), encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except ValueError:
                continue
            label = rec.get("label", "")
            if label not in steps:
                order.append(label)
            steps[label] = rec
except OSError:
    pass

counts = {"OK": 0, "FAIL": 0, "SKIP": 0}
for label in order:
    status = (steps[label].get("status") or "").upper()
    if status in counts:
        counts[status] += 1
total = len(order)

acceptance = read_json("acceptance.json") or {}
audio = read_json("audio_metrics.json") or {}
baseline = read_json("baseline_diff.json") or {}

summary = {
    "run_id": run_id,
    "verdict": "PASS" if passed == "1" else "FAIL",
    "fail_kind": fail_kind or None,
    "tts_provider": tts_provider or None,
    "generated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    "steps": {
        "total": total,
        "ok": counts["OK"],
        "fail": counts["FAIL"],
        "skip": counts["SKIP"],
        "failed_labels": [
            l for l in order if (steps[l].get("status") or "").upper() == "FAIL"
        ],
    },
    "gate1": {
        "pass": acceptance.get("pass"),
        "reason": acceptance.get("reason"),
        "missing_expected_calls": acceptance.get("missing_expected_calls"),
        "forbidden_calls": acceptance.get("forbidden_calls"),
    },
    # Метрики качества: None означает «не посчиталось», и это видно, а не
    # тонет в тексте ошибки внутри отдельного артефакта.
    # Имена ключей — ровно как в e2e_audio_metrics.py (rms_dbfs/peak_dbfs,
    # НЕ rms_db). Разъехавшись, сводка показывает null при полностью живых
    # метриках; поймано прогоном 35658231116 на билд-машине.
    "audio": {
        "error": audio.get("error"),
        "rms_dbfs": audio.get("rms_dbfs"),
        "peak_dbfs": audio.get("peak_dbfs"),
        "silence_ratio": audio.get("silence_ratio"),
        "mic_working": audio.get("mic_working"),
    },
    "baseline": {
        "error": baseline.get("error"),
        "pass": baseline.get("pass"),
        "keyword_match_pct": baseline.get("keyword_match_pct"),
    },
}

with open(os.path.join(out_dir, "summary.json"), "w", encoding="utf-8") as fh:
    json.dump(summary, fh, ensure_ascii=False, indent=2)
    fh.write("\n")

st = summary["steps"]
print(">>> E2E_SUMMARY verdict=%s steps=%d/%d ok fail=%d skip=%d provider=%s"
      % (summary["verdict"], st["ok"], st["total"], st["fail"], st["skip"],
         summary["tts_provider"] or "?"))
if st["failed_labels"]:
    print(">>> E2E_SUMMARY failed: %s" % ", ".join(st["failed_labels"]))
if summary["audio"]["error"]:
    print(">>> E2E_SUMMARY audio metrics: %s" % summary["audio"]["error"])
else:
    print(">>> E2E_SUMMARY audio: rms=%s dBFS peak=%s dBFS silence=%s mic_working=%s"
          % (summary["audio"]["rms_dbfs"], summary["audio"]["peak_dbfs"],
             summary["audio"]["silence_ratio"], summary["audio"]["mic_working"]))
PY
    log "ARTIFACTS: summary.json written"
}

# Возвращает 0 если acceptance-чек PASS, 1 если FAIL.
# Acceptance-блок в scenario описывает ожидаемое поведение робота:
#   expected_tool_calls: list[str]  — должны быть вызваны (ищется в логах)
#   must_not_call:        list[str]  — НЕ должны быть вызваны. ⚠️ Свободный
#                                     (не-snake_case) фрагмент здесь ищется
#                                     подстрокой по ВСЕМУ логу шага
#                                     (tool_invoked() fallback) — включая
#                                     служебные строки биометрии/диагностики.
#                                     Для «робот не должен ПРОИЗНЕСТИ X» это
#                                     тавтологически красное поле (issue
#                                     #2779: «Борис» есть в
#                                     `identify candidates:` независимо от
#                                     того, что сказал робот) — используйте
#                                     must_not_say ниже.
#   must_not_say:         list[str]  — issue #2779: НЕ должно звучать в РЕЧИ
#                                     робота (та же область, что и
#                                     expected_keywords — robot_speech(),
#                                     БЕЗ строк биометрии/пользовательского
#                                     ввода). Тот же формат альтернации
#                                     "А|Б|В", что и expected_keywords.
#   expected_keywords:    list[str]  — должны быть в логах шага (признанная
#                                     фраза ИЛИ LLM OUTPUT / spoken=). Ключ
#                                     вида "Борис|Спартак|пицц" — это
#                                     АЛЬТЕРНАЦИЯ: достаточно любого варианта
#                                     (issue #2753). Регэксп НЕ поддержан:
#                                     сравнение идёт подстрокой по каждому
#                                     варианту, знаки ? . ( ) значат себя.
#   voice_changed:        bool       — set_voice сменил голос с дефолта
#   response_max_ms:      int        — T_total не должен превышать (если
#                                     найден e2e_timing.json)
#   discovery_tools:      list[str]  — issue #2406: каждый tool должен быть
#                                     вызван ДО первого голосового ответа
#                                     (verbal-only LLM answer regression guard)
# Пишет acceptance.json в OUT_DIR.
check_acceptance() {  # $1=label $2=acceptance_json_string $3=before_rfc3339
    local label="$1" acc_json="$2" before="$3"
    local rc=0
    local logs logs_file
    logs="$(${ROBOT_SSH} "docker logs voice-assistant --since '${before}' 2>&1" 2>/dev/null || echo '')"
    # Логи шага тоже пишем в файл (ARG_MAX защита, как в check_gate1_aggregate).
    logs_file="$OUT_DIR/.acceptance_${label}.txt"
    printf '%s' "$logs" > "$logs_file"

    # Прогон acceptance-чекера в Python (читает acc_json + logs → pass/fail + reason).
    ACC_JSON="$acc_json" LOGS_FILE="$logs_file" \
        PYTHONPATH="$SCRIPT_DIR_E2E${PYTHONPATH:+:$PYTHONPATH}" \
        python3 - <<'PY' > "$OUT_DIR/acceptance.json"
import json, os, re, sys
# «тул вызван» != «тул есть в списке доступных»: dialogue_node печатает
# tools(56) и системный промпт на каждом ходе (см. e2e_tool_match.py).
from e2e_tool_match import tool_invoked
# Issue #2406: для discovery-тул ассерта нужен порядок «tool ДО voice».
from e2e_tool_match import (
    first_invocation_position,
    first_voice_cycle_position,
    TOOL_NAME_RE,
)
# Issue #2764: expected_keywords матчатся по речи робота, не по всему логу.
from e2e_tool_match import keyword_hit, robot_speech
acc = json.loads(os.environ["ACC_JSON"])
with open(os.environ["LOGS_FILE"], encoding="utf-8", errors="replace") as _f:
    logs = _f.read()
def has(s, frag):
    return tool_invoked(logs, frag)
expected_call = acc.get("expected_tool_calls", []) or []
must_not = acc.get("must_not_call", []) or []
expected_kw = acc.get("expected_keywords", []) or []
# Issue #2779 — must_not_say: отдельное от must_not_call поле для «этого
# не должно ЗВУЧАТЬ». must_not_call свободным текстом матчит ВЕСЬ лог шага
# (tool_invoked() fallback), а в этот лог гарантированно попадают строки
# голосовой биометрии (`identify candidates: best='Борис'...`,
# `Speaker: 'Борис'`, `[Spkr:Борис]`) независимо от того, что робот
# ответил — тавтологическое КРАСНОЕ для проверки «незнакомцу не сказали
# чужое имя» (см. n210_grisha_no_name). must_not_say ищет ТОЛЬКО в
# robot_speech() — том же канале, что и expected_keywords (issue #2764).
must_not_say = acc.get("must_not_say", []) or []
# Issue #2406: discovery_tools — список тулов, которые ОБЯЗАНЫ быть вызваны
# ДО первого голосового ответа. Если в acceptance.json шага есть это поле —
# ассертим порядок, иначе — старый чек (только факт вызова).
# Каждый tool должен соответствовать TOOL_NAME_RE (snake_case имя тула),
# иначе — soft FAIL с подсказкой.
discovery_tools_raw = acc.get("discovery_tools", []) or []
discovery_tools = []
discovery_tool_errors = []
for _dt in discovery_tools_raw:
    if not isinstance(_dt, str) or not TOOL_NAME_RE.match(_dt.lower()):
        discovery_tool_errors.append(
            f"discovery_tools entry {repr(_dt)} is not a tool name "
            f"(must match ^[a-z][a-z0-9_]*$)"
        )
        continue
    discovery_tools.append(_dt)
voice_changed_req = bool(acc.get("voice_changed", False))
response_max_ms = acc.get("response_max_ms", 0) or 0

recognized = ""
# stt_node.py:923 печатает `✅ ПРИНЯТО ({source}): {text}` — тег источника
# добавлен в 6e016325f (#2011), а паттерн остался без него, поэтому recognized
# был пуст на каждом прогоне (35658231116). Тег опционален — старые логи тоже.
m = re.search(r"✅ ПРИНЯТО(?: \([^)]*\))?:\s*(.+)", logs)
if m:
    recognized = m.group(1).strip()
# Ключевые слова ищем в РЕЧИ РОБОТА (TTS text + аргумент speak_text +
# spoken=), а не по всему логу шага — см. robot_speech/keyword_hit в
# e2e_tool_match.py и issue #2764.
#
# Историю важно не откатить: когда-то ключи искались только в recognised
# (распознанной фразе), и «Красная» vs STT «красную» давал false-negative
# при корректно рассказанной сказке (mv03, run 32595628905). Починили это
# расширением на ВЕСЬ лог — но вместе с ответом робота в поиск попали
# реплика говорящего и подпись диктора '[Spkr:Саша]', и keyword-проверки
# стали проходить сами по себе. robot_speech даёт ровно ту область,
# которую хотел mv03 (LLM OUTPUT / spoken= / то, что ушло в синтез),
# без пользовательского ввода.
logs_low = logs.lower()

actual_calls = []
for c in (expected_call + must_not):
    if has(logs, c) and c not in actual_calls:
        actual_calls.append(c)

found_expected = [c for c in expected_call if has(logs, c)]
missing_expected = [c for c in expected_call if not has(logs, c)]
forbidden_called = [c for c in must_not if has(logs, c)]
# Issue #2753 — ключ вида "Борис|Спартак|пицц" сценарии пишут как
# АЛЬТЕРНАЦИЮ, по аналогии с соседним полем patterns (оно идёт через grep -E).
# Сравнение целой строкой искало её вместе с палками и не находило никогда:
# n209_recall_boris в run 35699257202 покраснел при идеальном ответе робота
# («…приходишь примерно раз в неделю с пиццей. Болеешь за Спартак…»), и так же
# вечно краснели n1002/n1009 в финальном акте. Разбиваем по "|" и ищем any-of,
# а НЕ включаем полноценный регэксп: иначе существующие ключи со знаком ?,
# точкой или скобками молча поменяли бы смысл.
def _keyword_hit(kw):
    return keyword_hit(logs, kw)

found_keywords = [k for k in expected_kw if _keyword_hit(k)]
missing_keywords = [k for k in expected_kw if not _keyword_hit(k)]
# Issue #2779 — зеркало found/missing_keywords, но для «не должно звучать».
# _keyword_hit() тот же самый (robot_speech()-scoped), значит фраза
# засчитывается только если её реально ПРОИЗНЁС робот — служебные строки
# биометрии (`identify candidates`, `Speaker: 'Борис'`, `[Spkr:Борис]`)
# в robot_speech() не попадают (см. e2e_tool_match.py:robot_speech).
forbidden_said = [k for k in must_not_say if _keyword_hit(k)]
# Диагностика в артефакт: без неё красный keyword-шаг неотличим от
# «робот вообще молчал» — а это разные починки.
robot_said = robot_speech(logs)

# Issue #2406: discovery-tools order check. Для каждого discovery-тула:
# - позиция первого execution-маркера в логе
# - позиция первого voice-cycle маркера (любого)
# Требование: tool_pos < voice_pos (тул вызван ДО голосового ответа).
# Если тул не вызван — отдельный FAIL (подобно missing_expected).
# Если голосовой цикл уже случился раньше вызова тула — отдельный FAIL
# с подсказкой про verbal-only LLM answer.
discovery_failures = []
discovery_records = []
voice_pos = first_voice_cycle_position(logs) if discovery_tools else None
for _dt in discovery_tools:
    tool_pos = first_invocation_position(logs, _dt)
    rec = {
        "tool": _dt,
        "first_invocation_pos": tool_pos,
        "first_voice_pos": voice_pos,
    }
    if tool_pos is None:
        discovery_failures.append(
            f"discovery tool {_dt!r} was NOT invoked at all "
            f"(issue #2406: LLM bypassed the required tool call)"
        )
    elif voice_pos is not None and tool_pos > voice_pos:
        discovery_failures.append(
            f"discovery tool {_dt!r} invoked at pos {tool_pos}, "
            f"AFTER verbal answer at pos {voice_pos} "
            f"(issue #2406: verbal-only LLM answer before required tool)"
        )
    discovery_records.append(rec)

# voice_changed: set_voice был вызван с голосом, отличным от дефолта.
# Лог: `[set_voice] voice='X' provider=... default=Y`. Если set_voice вернул
# voice_unavailable — лога не будет, и это честный FAIL (голос не сменился).
voice_change_ok = True
voice_change_detail = ""
if voice_changed_req:
    set_voice_matches = re.findall(
        r"\[set_voice\] voice='([^']*)'\s+provider=(\S+)\s+default=(\S+)", logs
    )
    if not set_voice_matches:
        voice_change_ok = False
        voice_change_detail = (
            "voice did not change: no successful [set_voice] log "
            "(set_voice not called or returned voice_unavailable)"
        )
    elif not any(v.strip().lower() != d.strip().lower() for v, _p, d in set_voice_matches):
        voice_change_ok = False
        voice_change_detail = (
            "voice did not change from default: "
            + ", ".join(f"{v}->{d}" for v, _p, d in set_voice_matches)
        )

# response_max_ms — берём из ранее записанного timing.json если есть
timing_path = os.environ.get("OUT_DIR", "") + "/timing.json"
measured_ms = None
if os.path.exists(timing_path):
    try:
        t = json.load(open(timing_path))
        measured_ms = t.get("metrics_ms", {}).get("total_latency_ms")
    except Exception:
        pass

failures = []
if missing_expected:
    failures.append(f"expected tool calls not invoked: {missing_expected}")
if forbidden_called:
    failures.append(f"forbidden tool calls invoked: {forbidden_called}")
if expected_kw and missing_keywords:
    failures.append(f"expected keywords missing in logs: {missing_keywords}")
if forbidden_said:
    failures.append(f"forbidden phrases spoken by robot: {forbidden_said}")
if discovery_tool_errors:
    failures.extend(discovery_tool_errors)
if discovery_failures:
    failures.extend(discovery_failures)
if not voice_change_ok:
    failures.append(voice_change_detail)
if response_max_ms and measured_ms and measured_ms > response_max_ms:
    failures.append(f"response time {measured_ms}ms > max {response_max_ms}ms")
result = {
    "label": os.environ.get("STEP_LABEL", ""),
    "expected_tool_calls": expected_call,
    "actual_tool_calls": actual_calls,
    "missing_expected_calls": missing_expected,
    "forbidden_calls": forbidden_called,
    "expected_keywords": expected_kw,
    "robot_speech": robot_said,
    "recognized": recognized,
    "found_keywords": found_keywords,
    "missing_keywords": missing_keywords,
    # Issue #2779 — must_not_say verdict (robot_speech-scoped, see above).
    "must_not_say": must_not_say,
    "forbidden_said": forbidden_said,
    # Issue #2406: discovery-step enforcement (per-step).
    # discovery_tools содержит имена тулов, которые ОБЯЗАНЫ быть вызваны
    # ДО первого голосового ответа. discovery_records — массив позиций
    # для трассировки (tool_pos vs voice_pos), см. test_issue_2406_*_step_enforcement.py.
    "discovery_tools": discovery_tools,
    "discovery_records": discovery_records,
    "discovery_first_voice_pos": voice_pos,
    "voice_changed": voice_changed_req,
    "voice_change_ok": voice_change_ok,
    "voice_change_detail": voice_change_detail,
    "response_max_ms": response_max_ms,
    "measured_total_ms": measured_ms,
    "pass": not failures,
    "reason": "; ".join(failures) if failures else "all checks passed",
}
print(json.dumps(result, ensure_ascii=False, indent=2))
PY
    # Итоговый rc — из acceptance.json (pass → 0)
    if grep -q '"pass": *true' "$OUT_DIR/acceptance.json"; then
        rc=0
        log "ACCEPTANCE[${label}]: ✅ $(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["reason"])' "$OUT_DIR/acceptance.json")"
    else
        rc=1
        log "ACCEPTANCE[${label}]: ❌ $(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["reason"])' "$OUT_DIR/acceptance.json")"
    fi
    return $rc
}

# --- выбор TTS-провайдера для синтеза команд -----------------------------
# Резолвим ДО записи и ДО первого шага по двум причинам:
#   1) выбор виден в самом начале лога, а не посреди сценария;
#   2) «ни один провайдер не отвечает» — это один понятный отказ
#      на весь прогон, а не 40 одинаковых "FAIL synth" подряд (run
#      35533542706 читался именно так — 11 шагов, одна причина).
# Не фатал: падаем штатным путём через mark_fail_kind synth ниже,
# чтобы артефакты и verdict.txt всё равно были собраны.
ensure_outdir
resolve_tts_provider || mark_fail_kind synth
cat > "$OUT_DIR/tts_provider.json" <<TTSJSON
{
  "requested": "${E2E_TTS_PROVIDER}",
  "order": "${E2E_TTS_PROVIDER_ORDER}",
  "resolved": "${E2E_TTS_PROVIDER_RESOLVED:-none}",
  "resolved_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
TTSJSON

# --- сценарий или одиночная команда ----------------------------------------
# Issue #1353: запись микрофона охватывает ВЕСЬ retry-цикл (все шаги, все
# попытки). Стартуем до if/else, останавливаем в trap EXIT (см. ниже).
start_recording

# Advisory health probe is always collected, but never changes PASS/FAIL.
# Keep it in the run artifact so e2e reports expose infrastructure health.
observe_step "${SCENARIO_FILE:+scenario}${SCENARIO_FILE:-single}" > "$OUT_DIR/health_snapshot.json" || true

# Issue #2750 — акт «Знакомство» получает изолированную БД дикторов ДО
# первого шага. Якорь — СОДЕРЖИМОЕ сценария (issue #2763), не имя файла.
if [ -n "$SCENARIO_FILE" ] && scenario_registers_speakers "$SCENARIO_FILE"; then
    activate_e2e_speaker_db
fi
# Issue #2781 — тот же сценарий (или другой) может писать долгосрочную
# память через memory_save — отдельная изоляция, отдельный узел (mcp_server).
if [ -n "$SCENARIO_FILE" ] && scenario_writes_memory "$SCENARIO_FILE"; then
    activate_e2e_memory_db
fi
# Issue #2809 — node_params override (см. apply_node_params выше). Применяем
# ДО первого шага, как и изоляцию БД: акт «переспрос личности» форсирует
# name_confidence_band_high на время всего прогона.
if [ -n "$SCENARIO_FILE" ]; then
    apply_node_params "$SCENARIO_FILE"
fi

# Гарантированная остановка записи, возврат speaker_id_node/mcp_server на
# боевые БД и восстановление node_params при любом завершении
# (PASS/FAIL/ошибка). Все хелперы идемпотентны: повторный вызов — noop
# (пустой REC_PID / E2E_*_DB_ACTIVATED=0 / пустой node_params_originals).
trap 'restore_node_params; deactivate_e2e_speaker_db; deactivate_e2e_memory_db; stop_recording' EXIT

PASS=1
if [ -n "$SCENARIO_FILE" ]; then
    # scenario.json: {"steps":[{"text":"...","voice":"anton","label":"s1",
    #           "patterns":["save_speaker_profile"],
    #           "acceptance":{"expected_tool_calls":["generate_music"],
    #                         "must_not_call":["execute_music_code"],
    #                         "expected_keywords":["песня"],
    #                         # issue #2406: тул ДО голосового ответа
    #                         "discovery_tools":["register_speaker"],
    #                         "response_max_ms":60000},
    #           # issue #2809 — условный ответ на переспрос личности:
    #           "when_robot_asked":"Саш.*это ты|это ты.*Саш",
    #           "required_question":true,
    #           "sleep_before_sec":35}]}
    #
    # when_robot_asked (issue #2809) — grep -E, регистронезависимо, ищется
    # ТОЛЬКО в robot_speech() ПРЕДЫДУЩЕГО шага (тот же канал, что и
    # expected_keywords/must_not_say — issue #2764/#2779, НЕ весь лог шага).
    # Совпало → шаг играется как обычный. Не совпало → шаг SKIP с причиной
    # robot_did_not_ask (отдельная строка в E2E_SUMMARY, не OK и не FAIL) —
    # ЕСЛИ ТОЛЬКО не задан required_question:true, тогда отсутствие вопроса
    # само по себе FAIL (нужно для детерминированного акта «переспрос
    # личности», где вопрос ОБЯЗАН прозвучать).
    # sleep_before_sec (issue #2809) — пауза ПЕРЕД шагом (имитация конца
    # сессии дольше identity_question_session_gap_sec).
    cp "$SCENARIO_FILE" "$OUT_DIR/scenario.json"
    # issue #2809 — условный ответ на переспрос ("when_robot_asked" — grep-подобный
    # паттерн против речи ПРЕДЫДУЩЕГО шага, "required_question" — FAIL, если
    # робот не спросил, "sleep_before_sec" — пауза перед шагом).
    #
    # issue #2824 (регресс, живой прогон 35851587044, develop b2f5560e5):
    # парсинг и разбор строки TSV вынесены в parse_scenario_to_tsv() и
    # E2E_SCENARIO_ROW_READ (e2e_voice_lib.sh) — ЕДИНАЯ точка истины,
    # используемая и здесь, и в scripts/testing/test_e2e_scenario_tsv_row.sh.
    # Раньше \t-разделитель ломался в самом bash `read` (см. докстринг
    # parse_scenario_to_tsv для разбора причины), а регресс-тест #2823
    # проверял python-парсер и bash-цикл ПО ОТДЕЛЬНОСТИ, из-за чего не
    # поймал взаимодействие между ними — теперь оба места читают ровно
    # тот же код, что и main flow.
    parse_scenario_to_tsv "$SCENARIO_FILE" "$OUT_DIR/scenario_parsed.txt"
    # issue #2809 — «речь предыдущего шага» для when_robot_asked. Пустой файл
    # на старте акта: у первого шага сценария просто нет предыдущего шага,
    # значит when_robot_asked на первом шаге не может совпасть НИКОГДА (это
    # осознанное поведение — акт не должен ставить when_robot_asked на шаг 0).
    LAST_STEP_SPEECH_FILE="$OUT_DIR/.last_step_speech.txt"
    : > "$LAST_STEP_SPEECH_FILE"
    while IFS=$'\x1f' eval "$E2E_SCENARIO_ROW_READ"; do
        [ -z "$idx" ] && continue
        case "$retry_acceptance" in
            ''|*[!0-9]*) retry_acceptance=0 ;;
        esac
        # issue #2809 — sleep_before_sec: пауза ПЕРЕД шагом (имитация конца
        # сессии диалога дольше identity_question_session_gap_sec). Идёт ДО
        # STEP_BEFORE (окно логов шага не должно включать саму паузу).
        case "$sleep_before_sec" in
            ''|*[!0-9.]*) sleep_before_sec=0 ;;
        esac
        if awk "BEGIN{exit !($sleep_before_sec > 0)}" 2>/dev/null; then
            log "STEP ${label}: sleep_before_sec=${sleep_before_sec}s — пауза перед шагом (issue #2809)"
            sleep "$sleep_before_sec"
        fi
        # issue #2809 — when_robot_asked: условный шаг-ответ на переспрос.
        # Проверяется ПРОТИВ РЕЧИ ПРЕДЫДУЩЕГО шага (тот же LAST_STEP_SPEECH_FILE,
        # который заполняется в конце обработки каждого шага ниже — см. issue
        # 2809 после закрытия retry-цикла). grep -qiE — регистронезависимо,
        # синтаксис ERE (то же "А|Б" что и expected_keywords/must_not_say).
        step_skip_no_question=0
        if [ -n "$when_robot_asked" ]; then
            _prev_speech="$(cat "$LAST_STEP_SPEECH_FILE" 2>/dev/null || echo '')"
            if when_robot_asked_matches "$_prev_speech" "$when_robot_asked"; then
                log "STEP ${label}: when_robot_asked='${when_robot_asked}' — совпало с речью предыдущего шага, играем как обычно"
            else
                if [ "$required_question" = "1" ]; then
                    PASS=0
                    mark_fail_kind feature
                    log "STEP ${label}: ❌ required_question — робот НЕ задал ожидаемый вопрос ('${when_robot_asked}' не найден в речи предыдущего шага: '${_prev_speech}')"
                    emit_step "${label} FAIL robot_did_not_ask"
                else
                    log "STEP ${label}: ⏭ SKIP — робот не задал вопрос ('${when_robot_asked}' не найден в речи предыдущего шага), пропускаем без FAIL"
                    emit_step "${label} SKIP robot_did_not_ask"
                fi
                step_skip_no_question=1
            fi
        fi
        if [ "$step_skip_no_question" = "1" ]; then
            # Шаг не разыгрывался — реальной речи не было, LAST_STEP_SPEECH_FILE
            # намеренно НЕ трогаем (следующий шаг увидит ту же "тишину").
            continue
        fi
        # ADR-0029 §2.3: classify step.expect (auto-detect wake-prefix →
        # wake-gated). Классифицируем в bash, чтобы Python-парсер
        # оставался pure-data.
        expect="$(classify_step_expect "$expect_raw" "$text")"
        # retry_acceptance = доп. попытки при FAIL patterns/acceptance.
        # Нужно для dj01: LLM недетерминирован — если renardo не запустился
        # (execute_music_code не вызван), повторяем команду (e2e run 32595628905).
        attempt_n=0
        step_ok=0
        cycle_failed=0
        step_skipped=0
        # issue #2809 — начало окна логов ЭТОГО шага (для LAST_STEP_SPEECH_FILE
        # ниже, после закрытия retry-цикла). Фиксируется на ПЕРВОЙ попытке —
        # ретраи не должны сдвигать окно вперёд, иначе речь из первой
        # (неудачной) попытки потеряется для when_robot_asked следующего шага.
        step_window_start=""
        # Кто именно провалился на ПОСЛЕДНЕЙ попытке и кто проходил хоть раз.
        # bug(run 35658231116, 22.09.2026): step_ok пересчитывается с нуля на
        # каждой попытке, поэтому паттерны и acceptance обязаны сойтись в ОДНОЙ.
        # У n306_who_was_talking они сошлись в РАЗНЫХ:
        #   попытка 1: PATTERN_OK + ACCEPTANCE ❌ (нет ключевого слова «Борис»)
        #   попытка 2: PATTERN_MISS + ACCEPTANCE ✅ all checks passed
        # Итог — `FAIL` с текстом «см. acceptance.json», а в acceptance.json
        # лежит «✅ all checks passed»: артефакт прямо противоречит вердикту.
        # Причина в том, что паттерн шага ловит ОДНОРАЗОВЫЙ переход состояния
        # («[backlog] flushed to LLM backlog_handled=true»): на первой попытке
        # бэклог уже слит, на повторе сливать нечего, и паттерн не может
        # совпасть больше никогда. Такой шаг в принципе неретраебельный, и
        # молчать об этом нельзя — иначе разбор уходит в dialogue_node.
        last_fail_what=""
        pat_ok_any=0
        acc_ok_any=0
        pat_checked=0
        acc_checked=0
        while :; do
            last_fail_what=""
            STEP_BEFORE="$(${ROBOT_SSH} "date -u +%Y-%m-%dT%H:%M:%SZ" 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
            [ -z "$step_window_start" ] && step_window_start="$STEP_BEFORE"
            run_step "$text" "$voice" "$label" "$expect"
            rc=$?
            # Пишем transcript (даже при FAIL — для ретро-анализа, что STT услышал)
            parse_transcript "$label" "$STEP_BEFORE" "$text"
            # ADR-0029 §2.3: rc=3 = SKIP wake-gate-cold-start (не fail, не
            # pass). Шаг пропущен по systemic — backlog-аккумулятор скопит
            # фразу без wake, LLM не получит команду, и aggregate GATE-1
            # не должен фейлить на этом шаге.
            if [ "$rc" = "3" ]; then
                step_skipped=1
                # Счётчик РЕАЛЬНЫХ пропусков — только он оправдывает пропуск
                # агрегатного GATE-1 ниже (разбор прогона 35665111906: гейт
                # пропускался при НУЛЕ пропущенных шагов).
                WAKE_GATE_SKIPPED_STEPS=$((WAKE_GATE_SKIPPED_STEPS + 1))
                log "STEP ${label}: wake-gated SKIP (cold-start not cleared) — см. $WAKE_GATE_PREFLIGHT_FILE"
                break
            fi
            if [ "$rc" != "0" ]; then
                PASS=0
                cycle_failed=1
                # Проверяем паттерны даже при FAIL цикла? Нет — красный есть красный.
                break
            fi
            step_ok=1
            # Паттерны шага
            if [ -n "$patterns_json" ] && [ "$patterns_json" != "[]" ]; then
                # bug(run 34414065635, 09.09.2026): раньше здесь было
                # `pats="$(... " ".join(...))"` + `check_patterns ... $pats`
                # без кавычек — bash разбивал паттерн ПО ПРОБЕЛАМ, и
                # многословный regex превращался в несколько independent
                # паттернов. Наблюдалось живьём:
                #   pattern: \[backlog\] accumulated \(no_wake_word\).*speaker='Саш
                #   →  PATTERN_OK: \[backlog\]
                #      PATTERN_OK: accumulated
                #      PATTERN_MISS: \(no_wake_word\).*speaker='Саш
                # А в 1280_barge_in_abort_old_topic.json «Cancel: new STT input»
                # проверялся как четыре паттерна, из которых «new» и «input»
                # матчат почти любой лог — сьюта зеленела на мусоре.
                # Читаем паттерны построчно в массив: JSON-строка с переводом
                # строки внутри паттерна не поддерживается (и не нужна).
                # `tr -d '\015'` обязателен: mapfile -t срезает только \n, а
                # CR остаётся ВНУТРИ значения — паттерн «set_voice\r» не
                # матчит ничего и шаг краснеет без объяснимой причины.
                # Тот же класс, что inputs.scenario_file с CRLF
                # (test_e2e_voice_workflow_crlf_inputs.sh).
                mapfile -t _pats_arr < <(printf '%s' "$patterns_json" \
                    | python3 -c 'import json,sys
for p in json.load(sys.stdin):
    print(p.replace("\n", " "))' | tr -d '\015')
                log "STEP ${label}: проверка паттернов (${#_pats_arr[@]}): ${_pats_arr[*]}"
                pat_checked=1
                check_patterns "$STEP_BEFORE" "${_pats_arr[@]}"
                if [ $? != 0 ]; then
                    step_ok=0
                    last_fail_what="patterns"
                else
                    pat_ok_any=1
                    log "STEP ${label}: ✅ паттерны найдены"
                fi
            fi
            # Acceptance-чек (issue #1396): если в шаге задан блок acceptance,
            # пишем acceptance.json и (если ERROR) — FAIL (хотя цикл прошёл).
            if [ -n "$acceptance_json" ] && [ "$acceptance_json" != "{}" ]; then
                acc_checked=1
                OUT_DIR="$OUT_DIR" STEP_LABEL="$label" \
                    check_acceptance "$label" "$acceptance_json" "$STEP_BEFORE"
                if [ $? != 0 ]; then
                    step_ok=0
                    last_fail_what="${last_fail_what:+$last_fail_what+}acceptance"
                else
                    # Регистрация диктора, которая СКЛЕИЛАСЬ с чужим профилем,
                    # не является регистрацией.
                    #
                    # bug(живой прогон 35667281570, акт 2, 22.09.2026). Шаг
                    # n204_boris_intro_long получил OK, потому что acceptance
                    # проверяет только факт вызова register_speaker. А в логах
                    # робота за то же окно:
                    #   user_input='[Spkr:Саша] ... Меня зовут Борис ...'
                    #   🔗 Speaker 'Борис' merged into existing profile
                    #      (id=dc417cef) — voice matched an already-known speaker
                    #   ✅ [issue 1077] Speaker registered: 'Борис' id=dc417cef
                    # То есть голос Бориса опознан как Саша, Борис склеен в
                    # профиль Саши, а профиль ПЕРЕИМЕНОВАН — Саша исчез.
                    # В /data/speakers.db после акта остался ОДИН диктор
                    # «Борис» с двумя эмбеддингами вместо двух дикторов.
                    # Текст шага при этом прямо просит «Запомни мой голос
                    # ОТДЕЛЬНО от Сашиного... я не хочу, чтобы ты нас путал».
                    # Зелёный шаг поверх потерянной личности — ровно тот
                    # красивый PASS, против которого ADR-0018.
                    case "$acceptance_json" in
                        *register_speaker*)
                            _merge_log="$(${ROBOT_SSH} "docker logs voice-assistant --since '${STEP_BEFORE}' 2>&1" 2>/dev/null \
                                | grep -oE "Speaker '[^']*' merged into existing profile \(id=[0-9a-f]*\)" | tail -1)"
                            if [ -n "$_merge_log" ]; then
                                step_ok=0
                                last_fail_what="${last_fail_what:+$last_fail_what+}speaker_merged"
                                log "STEP ${label}: ❌ регистрация СКЛЕИЛАСЬ с уже известным диктором: ${_merge_log}"
                                log "STEP ${label}: это НЕ новый профиль — существующий переименован, прежняя личность потеряна. Шаг просит запомнить голос ОТДЕЛЬНО, значит проверка не пройдена."
                                log "STEP ${label}: смотреть register_match_threshold в speaker_id_node и различимость голосов TTS-провайдера ${E2E_TTS_PROVIDER_RESOLVED:-$E2E_TTS_PROVIDER} (у minimax Russian_ReliableMan и Russian_HandsomeChildhoodFriend неразличимы для resemblyzer)."
                                printf '%s\n' "STEP ${label}: ${_merge_log}" >> "$OUT_DIR/speaker_merges.log" 2>/dev/null || true
                            else
                                acc_ok_any=1
                                log "STEP ${label}: ✅ acceptance PASS"
                            fi
                            ;;
                        *)
                            acc_ok_any=1
                            log "STEP ${label}: ✅ acceptance PASS"
                            ;;
                    esac
                fi
            fi
            # Retry при FAIL patterns/acceptance (если разрешён сценарием)
            if [ "$step_ok" = "1" ]; then
                break
            fi
            if [ "$attempt_n" -ge "$retry_acceptance" ]; then
                break
            fi
            attempt_n=$((attempt_n + 1))
            log "STEP ${label}: ❌ проверка не прошла — retry ${attempt_n}/${retry_acceptance}"
            sleep "$E2E_RETRY_PAUSE"
        done
        # issue #2809 — запоминаем РЕЧЬ РОБОТА за это окно шага для
        # when_robot_asked СЛЕДУЮЩЕГО шага. Делается ВСЕГДА (даже при
        # cycle_failed/step_skipped — тогда файл честно останется пустым,
        # это валидный сигнал "робот ничего не сказал"), поэтому стоит ДО
        # ветвления по cycle_failed/step_skipped/step_ok ниже.
        if [ -n "${LAST_STEP_SPEECH_FILE:-}" ]; then
            _step_logs_for_speech="$(${ROBOT_SSH} "docker logs voice-assistant --since '${step_window_start:-$STEP_BEFORE}' 2>&1" 2>/dev/null || echo '')"
            if PYTHONPATH="$SCRIPT_DIR_E2E${PYTHONPATH:+:$PYTHONPATH}" python3 -c '
import sys
from e2e_tool_match import robot_speech
sys.stdout.write(robot_speech(sys.stdin.read()))
' <<< "$_step_logs_for_speech" > "$LAST_STEP_SPEECH_FILE" 2>/dev/null; then
                :
            else
                : > "$LAST_STEP_SPEECH_FILE"
            fi
        fi
        if [ "$cycle_failed" = "1" ]; then
            continue
        fi
        if [ "$step_skipped" = "1" ]; then
            # ADR-0029 §2.3: SKIP — не pass, не fail. Не помечаем fail_kind
            # и не влияем на PASS aggregate. echo уже сделал run_step
            # ('E2E_STEP <label> SKIP wake-gate-cold-start').
            :
        elif [ "$step_ok" = "1" ]; then
            emit_step "${label} OK"
        else
            PASS=0
            mark_fail_kind feature
            # Указываем на ТО, что реально провалилось на последней попытке.
            # Раньше текст всегда отправлял в acceptance.json — даже когда
            # провалились паттерны, а acceptance в том же прогоне прошёл, и
            # файл содержал «✅ all checks passed» (n306, run 35658231116).
            case "$last_fail_what" in
                patterns)   _where="паттерны шага (acceptance тут ни при чём)" ;;
                acceptance) _where="acceptance (см. $OUT_DIR/acceptance.json)" ;;
                *acceptance) _where="паттерны И acceptance (см. $OUT_DIR/acceptance.json)" ;;
                *)          _where="проверка шага (см. $OUT_DIR/acceptance.json)" ;;
            esac
            # Улики разъехались по попыткам: каждая подпроверка проходила хотя
            # бы раз, но ни разу вместе. Как правило это значит, что паттерн
            # шага ловит ОДНОРАЗОВЫЙ переход состояния (например
            # «[backlog] flushed to LLM») — на повторе его уже не будет,
            # и шаг не может пройти ни при каком числе ретраев.
            if [ "$pat_checked" = "1" ] && [ "$acc_checked" = "1" ] \
               && [ "$pat_ok_any" = "1" ] && [ "$acc_ok_any" = "1" ]; then
                log "STEP ${label}: ❌ улики разъехались по попыткам: паттерны проходили в одной попытке, acceptance — в другой, вместе ни разу."
                log "STEP ${label}: почти наверняка паттерн шага ловит ОДНОРАЗОВОЕ событие (напр. «[backlog] flushed to LLM»), которое на повторе не повторяется. Шаг как написан НЕ проверяем ретраем — это дефект СЦЕНАРИЯ, а не робота: либо убери retry_acceptance, либо перенеси одноразовый паттерн в отдельный шаг без ретрая."
                log "STEP ${label}: ❌ провалилось на последней попытке: ${_where}"
                emit_step "${label} FAIL retry_split_evidence"
            else
                log "STEP ${label}: ❌ проверка не прошла после retry — ${_where}"
                emit_step "${label} FAIL"
            fi
        fi
    done < "$OUT_DIR/scenario_parsed.txt"

    # --- ADR-0029 §2.3: GATE-1 SKIP-логика ------------------------------------
    # Если все wake-gated steps были SKIP (cold-start не cleared), aggregate
    # GATE-1 не должен фейлить — это by-design поведение backlog-аккумулятора
    # (см. retro t_be491fba). Фиксируем это в $OUT_DIR/gate1_skip_reason.json
    # и выводим явный маркер E2E_GATE1_SKIP_WAKE_GATE для пост-валидатора.
    # bug(живой прогон 35665111906, 22.09.2026 — регресс, внесённый этой же
    # серией правок). Условие было `WAKE_GATE_CLEARED != 1`. Пока preflight
    # был мёртвым кодом, он ВСЕГДА уходил в else-ветку и форсил
    # WAKE_GATE_CLEARED=1, поэтому агрегатный GATE-1 выполнялся всегда. Как
    # только preflight заработал, scenario-ветка честно сообщила «cold-start
    # NOT cleared» — и агрегатный GATE-1 (ADR-0022, главный гард против
    # smoke-false-PASS) стал ПРОПУСКАТЬСЯ на каждом scenario-прогоне.
    # В логе акта 1 это видно как `E2E_GATE1_SKIP_WAKE_GATE` при 9/10 OK и
    # нулевом числе SKIP-шагов: оправдание применялось там, где оправдывать
    # было нечего.
    #
    # Честное правило: пропускать GATE-1 можно только если шаги РЕАЛЬНО были
    # пропущены из-за wake-gate. Ни одного такого шага — гейт обязан считаться.
    # Сам факт «гейт не прогрелся» ничего не оправдывает, если он никому не
    # помешал отыграться.
    if [ "${WAKE_GATE_SKIPPED_STEPS:-0}" -gt 0 ] && [ -n "${SCENARIO_FILE:-}" ]; then
        printf '{\n  "skip_reason": "wake-gate cold-start not cleared",\n  "skipped_steps": %s,\n  "preflight_artifact": "wake_gate_preflight.json",\n  "scenarios_steps_classified": "wake-gated steps SKIP by design (backlog-accumulator design)",\n  "retro": "t_be491fba (cold-start wake-gate misdiagnosis)"\n}\n' \
            "${WAKE_GATE_SKIPPED_STEPS:-0}" > "$OUT_DIR/gate1_skip_reason.json"
        log "GATE-1: ⏭ SKIP — ${WAKE_GATE_SKIPPED_STEPS} шаг(ов) пропущено по wake-gate cold-start (см. wake_gate_preflight.json)"
        echo "E2E_GATE1_SKIP_WAKE_GATE"
    else
        # --- ADR-0022 GATE-1: top-level aggregate acceptance check -----------
        # Если --scenario задан и acceptance.json найден, прогоняем aggregate
        # проверку: каждый expected_tool_calls должен быть вызван хотя бы раз
        # за весь прогон (в логах docker voice-assistant); ни один из
        # must_not_call не должен быть вызван. Пишет $OUT_DIR/acceptance.json
        # с verdict. Если verdict == FAIL → PASS=0.
        if [ -n "$ACCEPTANCE_FILE" ] && [ -f "$ACCEPTANCE_FILE" ]; then
            check_gate1_aggregate "$ACCEPTANCE_FILE" "${E2E_RUN_BEFORE:-$(date -u +%Y-%m-%dT%H:%M:%SZ)}"
            if [ $? != 0 ]; then
                PASS=0
                mark_fail_kind feature
                log "GATE-1: ❌ aggregate acceptance FAIL (см. $OUT_DIR/acceptance.json)"
                echo "E2E_GATE1_FAIL"
            else
                log "GATE-1: ✅ aggregate acceptance PASS"
                echo "E2E_GATE1_OK"
            fi
        fi
    fi
else
    STEP_BEFORE="$(${ROBOT_SSH} "date -u +%Y-%m-%dT%H:%M:%SZ" 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
    # single-text mode: classify expect (auto-detect wake-prefix → wake-gated)
    single_expect="$(classify_step_expect "" "$TEXT")"
    run_step "$TEXT" "${VOICE:-$YANDEX_TTS_VOICE}" "single" "$single_expect"
    rc=$?
    # Пишем transcript для single-режима
    parse_transcript "single" "$STEP_BEFORE" "$TEXT"
    # ADR-0029 §2.3: rc=3 = SKIP wake-gate-cold-start. Single mode
    # bypass'ит preflight (см. выше — WAKE_GATE_CLEARED=1 для single),
    # но классификация expect может сработать (если юзер дал wake-текст
    # через --text). Защита: rc=3 в single mode = wake-gate flake, FAIL.
    if [ "$rc" = "3" ]; then
        PASS=0
        log "single: wake-gated SKIP — это flake для single-text, помечаем как no_reaction"
        mark_fail_kind no_reaction
        emit_step "single FAIL no_reaction (wake-gated SKIP)"
    elif [ "$rc" != "0" ]; then PASS=0; emit_step "single FAIL"; else emit_step "single OK"; fi
    # Дополнительные паттерны (--patterns "a,b,c") — проверяем после цикла
    if [ "$rc" = "0" ] && [ -n "$PATTERNS" ]; then
        log "single: проверка паттернов: $PATTERNS"
        IFS=',' read -r -a pat_arr <<< "$PATTERNS"
        check_patterns "$STEP_BEFORE" "${pat_arr[@]}"
        if [ $? != 0 ]; then
            PASS=0; mark_fail_kind feature; emit_step "single FAIL patterns"
        else
            emit_step "single OK patterns"
        fi
    fi
fi

# --- финальные артефакты (audio/baseline, запускаются после ВСЕХ шагов) ---
# Берём voice_text последнего шага (или single) для baseline.
FINAL_VOICE_TEXT=""
if [ -n "$SCENARIO_FILE" ]; then
    FINAL_VOICE_TEXT="$(python3 -c 'import json; print(json.load(open(sys.argv[1]))["steps"][-1].get("text",""))' "$SCENARIO_FILE" 2>/dev/null || echo '')"
else
    FINAL_VOICE_TEXT="$TEXT"
fi
# Запись ОБЯЗАНА быть сконвертирована до замеров: audio_metrics/baseline_diff
# читают $OUT_DIR/recording.wav, а создаёт его stop_recording, который висит на
# `trap ... EXIT` — то есть отрабатывал ПОЗЖЕ. В итоге в каждом прогоне (в том
# числе зелёном 34928781542) оба артефакта содержали ровно это:
#     {"error":"recording.wav not found"}
# при том что сам recording.wav лежал рядом в архиве. Единственная в проекте
# метрика качества звука не посчиталась ни разу. stop_recording идемпотентен
# (ранний выход при пустом REC_PID), так что trap ниже остаётся как страховка.
stop_recording
write_artifacts_audio "$FINAL_VOICE_TEXT"
write_summary

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
    # Маркер ПРИЧИНЫ отказа (для пост-валидатора и detect_fail_kind):
    #   E2E_FEATURE_FAIL — робот ответил, но фича-ассерт (patterns/
    #                      acceptance/GATE-1) не выполнен → баг кода
    #   E2E_LLM_ERROR    — LLM/TTS вернул ошибку (не квота/fallback)
    #   E2E_INFRA_FAIL   — синтез/воспроизведение упали на билд-машине
    #   E2E_NO_REACTION  — робот не ответил (дефолт, самый слабый сигнал)
    fail_marker=""
    case "${E2E_FAIL_KIND:-no_reaction}" in
        feature)    fail_marker="E2E_FEATURE_FAIL" ;;
        llm_error)  fail_marker="E2E_LLM_ERROR" ;;
        synth)      fail_marker="E2E_INFRA_FAIL" ;;
        *)          fail_marker="E2E_NO_REACTION" ;;
    esac
    echo "$fail_marker"
    ensure_outdir
    echo "FAIL" > "$OUT_DIR/verdict.txt"
    ${ROBOT_SSH} \
        "docker exec voice-assistant bash -c 'echo ${fail_marker} > /proc/1/fd/1'" \
        >/dev/null 2>&1 || true
fi
echo "E2E_ARTIFACTS $OUT_DIR"
exit $([ "$PASS" = "1" ] && echo 0 || echo 1)
