#!/bin/bash
# E2E Voice Test - runs on build machine (10.1.1.249) via SSH.
#
# Multi-model provenance: reads VOICE_LLM / VOICE_TTS / VOICE_STT from env
# and writes tests/e2e/_artifacts/<run_id>/model.json alongside the wav
# (A42 multi-model observability — see docs/design/E2E_TESTING_DESIGN_v2.md §E/F.2).
#
# Retry loop (live 08.08): после проигрывания команды проверяем в логах
# робота (10.1.1.21 voice-assistant), появился ли ОТВЕТ (TTS finished ПОСЛЕ
# нашей команды). Если реакции нет — повторяем команду до E2E_MAX_ATTEMPTS раз.
# Приветствие (startup greeting до команды) не считается ответом.
set -u
R="$1"; D="$2"; V="$3"

# Default model triple — matches workflow inputs defaults.
VOICE_LLM="${VOICE_LLM:-minimax-m2}"
VOICE_TTS="${VOICE_TTS:-minimax-male-qn-qingse}"
VOICE_STT="${VOICE_STT:-yandex}"

# Retry параметры
E2E_MAX_ATTEMPTS="${E2E_MAX_ATTEMPTS:-3}"
E2E_REACTION_WINDOW="${E2E_REACTION_WINDOW:-35}"  # сек ждём после play (LLM 429→fallback думает 15-20с)
E2E_RETRY_PAUSE="${E2E_RETRY_PAUSE:-10}"          # пауза между попытками
ROBOT_HOST="10.1.1.21"
ROBOT_SSH="sshpass -p open ssh -o StrictHostKeyChecking=no ros2@${ROBOT_HOST}"

# Derive run_id from the output wav filename: dialog_e2e_<run_id>.wav
RUN_ID="$(basename "$R" .wav | sed -e 's/^dialog_e2e_//')"
ARTIFACTS_DIR="/home/ros2/rob_box_project/tests/e2e/_artifacts/${RUN_ID}"
mkdir -p "$ARTIFACTS_DIR"

ffmpeg -y -i /tmp/voice_new.ogg -af "highpass=f=200,volume=3.0,alimiter=limit=0.98" -ac 1 -ar 16000 /tmp/voice_eq.wav 2>/dev/null
pactl set-sink-volume @DEFAULT_SINK@ ${V}%

# Метка времени ДО команды — ответы робота ищем только ПОСЛЕ неё.
# RFC3339 (не epoch!): docker на роботе НЕ парсит --since @<unix> —
# «failed to parse value as time or duration: @1786220844» → пустой лог → NO_REACTION.
BEFORE="$( ${ROBOT_SSH} "date -u +%Y-%m-%dT%H:%M:%SZ" 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ )"
echo ">>> ROBOT_TIME_BEFORE ${BEFORE}"

# Record raw PCM then convert to proper WAV.
# 🔴 FIX (live 08.08 «SUCCESS без ответа в записи»): запись должна покрывать
# ВЕСЬ retry-цикл (3 попытки × (play+35с окно) + паузы ≈ 150-180с), а не
# record_seconds из карточки (90с) — иначе ответ на 3-ю попытку НЕ попадает
# в wav, а валидатор видит TTS finished в логах → ложный SUCCESS.
TOTAL_RECORD=$((E2E_MAX_ATTEMPTS * (E2E_REACTION_WINDOW + E2E_RETRY_PAUSE + 8) + 10))
timeout ${TOTAL_RECORD} parec --format=s16le --channels=1 --rate=16000 /tmp/e2e_raw.pcm &
RPID=$!
echo ">>> RECORDING ${TOTAL_RECORD}s (полный retry-цикл)"
sleep 3

ATTEMPT=1
REACTION=0
while [ "$ATTEMPT" -le "$E2E_MAX_ATTEMPTS" ]; do
    echo ">>> PLAYING (attempt ${ATTEMPT}/${E2E_MAX_ATTEMPTS})"
    paplay /tmp/voice_eq.wav && echo ">>> PLAY_DONE" || echo ">>> PLAY_FAIL"
    sleep "$E2E_REACTION_WINDOW"

    # Проверяем реакцию робота: TTS finished с timestamp ПОЗЖЕ нашей команды.
    # Фильтруем по времени --since "${BEFORE}" — приветствие (до команды) не считается.
    REACTION_LOG="$( ${ROBOT_SSH} "docker logs voice-assistant --since '${BEFORE}' 2>&1 | grep 'TTS finished' | tail -3" 2>/dev/null )"
    if [ -n "$REACTION_LOG" ]; then
        # Уточняем: TTS finished в контейнере имеет ROS timestamp (сек с эпохи).
        # Сравниваем с BEFORE — ответ должен быть ПОСЛЕ нашей команды.
        # docker logs не даёт wall-time строк — используем --since 60s + достоверность:
        # если TTS finished вообще появился после нашего play (--since 60s окно),
        # считаем реакцией. Приветствие случается при старте контейнера (>1 мин назад).
        echo ">>> REACTION_DETECTED"
        echo "$REACTION_LOG" | tail -1
        REACTION=1
        break
    fi
    echo ">>> NO_REACTION (attempt ${ATTEMPT}) — повторяю команду"
    sleep "$E2E_RETRY_PAUSE"
    ATTEMPT=$((ATTEMPT+1))
done

wait $RPID 2>/dev/null
# Convert raw PCM to proper WAV
ffmpeg -y -f s16le -ar 16000 -ac 1 -i /tmp/e2e_raw.pcm "${R}" 2>/dev/null
rm -f /tmp/e2e_raw.pcm
echo ">>> RECORDING_DONE"

if [ "$REACTION" = "1" ]; then
    echo ">>> E2E_REACTION_OK (attempt ${ATTEMPT})"
else
    echo ">>> E2E_NO_REACTION (${E2E_MAX_ATTEMPTS} attempts)"
fi

# Multi-model provenance — write model.json into artifacts dir.
# Use python (always present on the recorder host) so the JSON is well-formed
# even when env values contain special characters.
TS="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
python3 - "$ARTIFACTS_DIR/model.json" "$RUN_ID" "$VOICE_LLM" "$VOICE_TTS" "$VOICE_STT" "$TS" "$REACTION" <<'PY'
import json, sys, os
out, run_id, llm, tts, stt, ts, reaction = sys.argv[1:8]
os.makedirs(os.path.dirname(out), exist_ok=True)
with open(out, "w", encoding="utf-8") as fh:
    json.dump(
        {"llm": llm, "tts": tts, "stt": stt, "ts": ts, "run_id": run_id, "reaction": int(reaction)},
        fh, indent=2, ensure_ascii=False,
    )
print(f"MODEL: llm={llm} tts={tts} stt={stt} run_id={run_id} reaction={reaction}")
PY
echo ">>> MODEL_WRITTEN ${ARTIFACTS_DIR}/model.json"
