#!/usr/bin/env bash
# tools/ffmpeg_decode/scenario_mp3.sh — Scenario 3/4.
#
# Decode an MP3 source file through ffmpeg into the format expected by
# tts_node (int16 LE mono @ 16 kHz) and feed it through the production
# ``to_pcm_int16`` decode path.
#
# Usage:
#     bash tools/ffmpeg_decode/scenario_mp3.sh
#     bash tools/ffmpeg_decode/scenario_mp3.sh <path/to/file.mp3>
#
# Default source: tts_audio_bench/fixtures/mp3/sine_440hz_1.00s_16000.mp3
#                 (MPEG-1 Layer III, ~64 kbps, container reports 1.08 s
#                  due to LAME encoder padding — *decoded* PCM is 1.0 s).
#
# Note: the decoded duration gate (1.0 s ± 0.05 s) is intentionally
# applied to the **decoded** audio, not the container-reported length.
# That mirrors what tts_node measures downstream once the bytes are on
# the AudioData topic.
#
# Exit codes: see tools/ffmpeg_decode/README.md.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=_common.sh
source "${HERE}/_common.sh"

require_tool ffmpeg
require_tool ffprobe
require_tool python3

SCENARIO="mp3"
DEFAULT_FIXTURE="${FIXTURES_ROOT}/mp3/sine_440hz_1.00s_16000.mp3"
EXPECTED_DURATION="${EXPECTED_DURATION:-1.0}"
TOLERANCE="${TOLERANCE:-0.05}"

FIXTURE="${1:-${DEFAULT_FIXTURE}}"
assert_fixture_exists "${FIXTURE}"

export TO_PCM_OUTPUT_RATE="${TO_PCM_OUTPUT_RATE:-16000}"
CONTAINER_DUR="$(ffprobe -v error -show_entries format=duration \
    -of default=noprint_wrappers=1:nokey=1 "${FIXTURE}")"
echo "[${SCENARIO}] source=${FIXTURE} container_dur=${CONTAINER_DUR}s target_sr=${TO_PCM_OUTPUT_RATE}Hz"
echo "[${SCENARIO}] expected_decoded_duration=${EXPECTED_DURATION}s tolerance=±${TOLERANCE}s"

# 1) ffmpeg decode MP3 → raw PCM s16le mono @ 16 kHz (tts_node contract).
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

# 4) Production-path round-trip — re-encode the decoded PCM to MP3, then
#    feed it through to_pcm_int16 (the production MP3 decode path).
decode_via_to_pcm_int16 "${SCENARIO}" "mp3" "${DECODED_RAW}" "${EXPECTED_DURATION}"

echo "[${SCENARIO}] OK"
exit 0
