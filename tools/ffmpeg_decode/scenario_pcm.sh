#!/usr/bin/env bash
# tools/ffmpeg_decode/scenario_pcm.sh — Scenario 1/4.
#
# Decode a raw PCM (header-less int16 LE mono) source file through
# ffmpeg into the format expected by tts_node (int16 LE mono @ 16 kHz)
# and feed it through the production ``to_pcm_int16`` decode path.
#
# Usage:
#     bash tools/ffmpeg_decode/scenario_pcm.sh
#     bash tools/ffmpeg_decode/scenario_pcm.sh <path/to/file.pcm>
#
# Default source: tts_audio_bench/fixtures/pcm/sine_440hz_1.00s_16000.pcm
#                 (32000 bytes, 16000 samples × int16, exactly 1.0 s).
#
# Exit codes (stable — see tools/ffmpeg_decode/README.md):
#     0 — pass (decoded duration matches, production round-trip matches)
#     1 — required tool missing
#     2 — source fixture missing
#     3 — ffmpeg decode failed
#     4 — decoded duration outside tolerance
#     5 — to_pcm_int16 round-trip failed
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=_common.sh
source "${HERE}/_common.sh"

require_tool ffmpeg
require_tool ffprobe
require_tool python3

SCENARIO="pcm"
DEFAULT_FIXTURE="${FIXTURES_ROOT}/pcm/sine_440hz_1.00s_16000.pcm"
EXPECTED_DURATION="${EXPECTED_DURATION:-1.0}"
TOLERANCE="${TOLERANCE:-0.05}"

FIXTURE="${1:-${DEFAULT_FIXTURE}}"
assert_fixture_exists "${FIXTURE}"

echo "[${SCENARIO}] source=${FIXTURE}"
echo "[${SCENARIO}] expected_duration=${EXPECTED_DURATION}s tolerance=±${TOLERANCE}s"

# 1) ffmpeg decode → raw PCM s16le mono @ 16 kHz (what tts_node expects).
DECODED_RAW="$(run_ffmpeg_decode "${SCENARIO}" "${FIXTURE}" "s16le")"
DECODED_BYTES="$(wc -c <"${DECODED_RAW}")"
echo "[${SCENARIO}] ffmpeg decoded ${DECODED_BYTES} bytes → ${DECODED_RAW}"

# 2) Wrap into a WAV so ffprobe can introspect duration cleanly.
DECODED_WAV="${ARTIFACTS_BASE}/${SCENARIO}/decoded.wav"
wav_wrap "${DECODED_RAW}" "${DECODED_WAV}"
MEASURED_DURATION="$(run_ffprobe_duration "${DECODED_WAV}")"
echo "[${SCENARIO}] ffprobe duration=${MEASURED_DURATION}s"

# 3) Duration must match expected (within tolerance) — this is the
#    acceptance gate the task brief requires ("длительность decoded-аудио
#    проверена автоматически").
assert_duration "${SCENARIO}" "${MEASURED_DURATION}" "${EXPECTED_DURATION}" "${TOLERANCE}"

# 4) Round-trip through the production ``to_pcm_int16`` so we exercise
#    the exact decode function tts_node uses for raw-PCM containers.
decode_via_to_pcm_int16 "${SCENARIO}" "pcm" "${DECODED_RAW}" "${EXPECTED_DURATION}"

echo "[${SCENARIO}] OK"
exit 0
