#!/usr/bin/env bash
# tools/ffmpeg_decode/scenario_wav.sh — Scenario 2/4.
#
# Decode a WAV source file through ffmpeg into the format expected by
# tts_node (int16 LE mono @ 16 kHz) and feed it through the production
# ``to_pcm_int16`` decode path.
#
# Usage:
#     bash tools/ffmpeg_decode/scenario_wav.sh
#     bash tools/ffmpeg_decode/scenario_wav.sh <path/to/file.wav>
#
# Default source: tts_audio_bench/fixtures/wav/sine_440hz_1.00s_16000.wav
#                 (RIFF/WAVE, int16 mono, 16000 Hz, 16000 frames = 1.0 s).
#
# Exit codes: see tools/ffmpeg_decode/README.md.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=_common.sh
source "${HERE}/_common.sh"

require_tool ffmpeg
require_tool ffprobe
require_tool python3

SCENARIO="wav"
DEFAULT_FIXTURE="${FIXTURES_ROOT}/wav/sine_440hz_1.00s_16000.wav"
EXPECTED_DURATION="${EXPECTED_DURATION:-1.0}"
TOLERANCE="${TOLERANCE:-0.05}"

FIXTURE="${1:-${DEFAULT_FIXTURE}}"
assert_fixture_exists "${FIXTURE}"

# The WAV fixtures carry their own sample-rate header so we follow
# whatever the source advertises. Set TO_PCM_OUTPUT_RATE to 16000 by
# default since that's the tts_node AudioData contract.
export TO_PCM_OUTPUT_RATE="${TO_PCM_OUTPUT_RATE:-16000}"
# Read the source rate so the input side doesn't fight us.
SRC_RATE="$(ffprobe -v error -select_streams a:0 -show_entries stream=sample_rate \
    -of default=noprint_wrappers=1:nokey=1 "${FIXTURE}")"
echo "[${SCENARIO}] source=${FIXTURE} src_sr=${SRC_RATE}Hz target_sr=${TO_PCM_OUTPUT_RATE}Hz"
echo "[${SCENARIO}] expected_duration=${EXPECTED_DURATION}s tolerance=±${TOLERANCE}s"

# 1) ffmpeg decode → raw PCM s16le mono @ 16 kHz (the tts_node contract).
DECODED_RAW="$(run_ffmpeg_decode "${SCENARIO}" "${FIXTURE}" "")"
DECODED_BYTES="$(wc -c <"${DECODED_RAW}")"
echo "[${SCENARIO}] ffmpeg decoded ${DECODED_BYTES} bytes → ${DECODED_RAW}"

# 2) Wrap into a WAV so ffprobe can read duration.
DECODED_WAV="${ARTIFACTS_BASE}/${SCENARIO}/decoded.wav"
wav_wrap "${DECODED_RAW}" "${DECODED_WAV}"
MEASURED_DURATION="$(run_ffprobe_duration "${DECODED_WAV}")"
echo "[${SCENARIO}] ffprobe duration=${MEASURED_DURATION}s"

# 3) Auto-verify duration (acceptance gate).
assert_duration "${SCENARIO}" "${MEASURED_DURATION}" "${EXPECTED_DURATION}" "${TOLERANCE}"

# 4) Production-path round-trip through to_pcm_int16.
decode_via_to_pcm_int16 "${SCENARIO}" "wav" "${DECODED_RAW}" "${EXPECTED_DURATION}"

echo "[${SCENARIO}] OK"
exit 0
