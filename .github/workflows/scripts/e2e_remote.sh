#!/bin/bash
# E2E Voice Test - runs on build machine (10.1.1.249) via SSH.
#
# Multi-model provenance: reads VOICE_LLM / VOICE_TTS / VOICE_STT from env
# and writes tests/e2e/_artifacts/<run_id>/model.json alongside the wav
# (A42 multi-model observability — see docs/design/E2E_TESTING_DESIGN_v2.md §E/F.2).
set -u
R="$1"; D="$2"; V="$3"

# Default model triple — matches workflow inputs defaults.
VOICE_LLM="${VOICE_LLM:-minimax-m2}"
VOICE_TTS="${VOICE_TTS:-minimax-male-qn-qingse}"
VOICE_STT="${VOICE_STT:-yandex}"

# Derive run_id from the output wav filename: dialog_e2e_<run_id>.wav
RUN_ID="$(basename "$R" .wav | sed -e 's/^dialog_e2e_//')"
ARTIFACTS_DIR="/home/ros2/rob_box_project/tests/e2e/_artifacts/${RUN_ID}"
mkdir -p "$ARTIFACTS_DIR"

ffmpeg -y -i /tmp/voice_new.ogg -af "highpass=f=200,volume=3.0,alimiter=limit=0.98" -ac 1 -ar 16000 /tmp/voice_eq.wav 2>/dev/null
pactl set-sink-volume @DEFAULT_SINK@ ${V}%
# Record raw PCM then convert to proper WAV
timeout ${D} parec --format=s16le --channels=1 --rate=16000 /tmp/e2e_raw.pcm &
RPID=$!
sleep 3
echo ">>> PLAYING"
paplay /tmp/voice_eq.wav && echo ">>> PLAY_DONE" || echo ">>> PLAY_FAIL"
wait $RPID 2>/dev/null
# Convert raw PCM to proper WAV
ffmpeg -y -f s16le -ar 16000 -ac 1 -i /tmp/e2e_raw.pcm "${R}" 2>/dev/null
rm -f /tmp/e2e_raw.pcm
echo ">>> RECORDING_DONE"

# Multi-model provenance — write model.json into artifacts dir.
# Use python (always present on the recorder host) so the JSON is well-formed
# even when env values contain special characters.
TS="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
python3 - "$ARTIFACTS_DIR/model.json" "$RUN_ID" "$VOICE_LLM" "$VOICE_TTS" "$VOICE_STT" "$TS" <<'PY'
import json, sys, os
out, run_id, llm, tts, stt, ts = sys.argv[1:7]
os.makedirs(os.path.dirname(out), exist_ok=True)
with open(out, "w", encoding="utf-8") as fh:
    json.dump(
        {"llm": llm, "tts": tts, "stt": stt, "ts": ts, "run_id": run_id},
        fh, indent=2, ensure_ascii=False,
    )
print(f"MODEL: llm={llm} tts={tts} stt={stt} run_id={run_id}")
PY
echo ">>> MODEL_WRITTEN ${ARTIFACTS_DIR}/model.json"