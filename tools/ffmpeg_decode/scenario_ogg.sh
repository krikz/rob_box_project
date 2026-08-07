#!/usr/bin/env bash
# tools/ffmpeg_decode/scenario_ogg.sh — Scenario 4/4.
#
# Decode an OGG/Vorbis source file through ffmpeg into the format
# expected by tts_node (int16 LE mono @ 16 kHz) and feed it through the
# production ``to_pcm_int16`` decode path.
#
# Usage:
#     bash tools/ffmpeg_decode/scenario_ogg.sh
#     bash tools/ffmpeg_decode/scenario_ogg.sh <path/to/file.ogg>
#
# Default source: tts_audio_bench/fixtures/ogg/sine_440hz_1.00s_16000.ogg
#                 (Ogg/Vorbis, 64 kbps, exactly 1.0 s).
#
# Exit codes: see tools/ffmpeg_decode/README.md.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=_common.sh
source "${HERE}/_common.sh"

require_tool ffmpeg
require_tool ffprobe
require_tool python3

SCENARIO="ogg"
DEFAULT_FIXTURE="${FIXTURES_ROOT}/ogg/sine_440hz_1.00s_16000.ogg"
EXPECTED_DURATION="${EXPECTED_DURATION:-1.0}"
TOLERANCE="${TOLERANCE:-0.05}"

FIXTURE="${1:-${DEFAULT_FIXTURE}}"
assert_fixture_exists "${FIXTURE}"

export TO_PCM_OUTPUT_RATE="${TO_PCM_OUTPUT_RATE:-16000}"
CONTAINER_DUR="$(ffprobe -v error -show_entries format=duration \
    -of default=noprint_wrappers=1:nokey=1 "${FIXTURE}")"
echo "[${SCENARIO}] source=${FIXTURE} container_dur=${CONTAINER_DUR}s target_sr=${TO_PCM_OUTPUT_RATE}Hz"
echo "[${SCENARIO}] expected_decoded_duration=${EXPECTED_DURATION}s tolerance=±${TOLERANCE}s"

# 1) ffmpeg decode OGG → raw PCM s16le mono @ 16 kHz (tts_node contract).
DECODED_RAW="$(run_ffmpeg_decode "${SCENARIO}" "${FIXTURE}" "")"
DECODED_BYTES="$(wc -c <"${DECODED_RAW}")"
echo "[${SCENARIO}] ffmpeg decoded ${DECODED_BYTES} bytes → ${DECODED_RAW}"

# 2) Wrap into a WAV so ffprobe can read duration of the *decoded* audio.
DECODED_WAV="${ARTIFACTS_BASE}/${SCENARIO}/decoded.wav"
wav_wrap "${DECODED_RAW}" "${DECODED_WAV}"
MEASURED_DURATION="$(run_ffprobe_duration "${DECODED_WAV}")"
echo "[${SCENARIO}] decoded_duration=${MEASURED_DURATION}s"

# 3) Auto-verify decoded duration (acceptance gate).
assert_duration "${SCENARIO}" "${MEASURED_DURATION}" "${EXPECTED_DURATION}" "${TOLERANCE}"

# 4) Production-path round-trip — re-encode decoded PCM to OGG/Vorbis,
#    then feed it through to_pcm_int16 (the production OGG decode path).
decode_via_to_pcm_int16 "${SCENARIO}" "ogg" "${DECODED_RAW}" "${EXPECTED_DURATION}"

echo "[${SCENARIO}] OK"
exit 0
