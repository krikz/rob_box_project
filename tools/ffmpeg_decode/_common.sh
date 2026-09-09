#!/usr/bin/env bash
# tools/ffmpeg_decode/_common.sh — shared helpers for ffmpeg decode scenarios.
#
# Source this from a scenario script. Provides:
#   - locate_repo_root:     absolute path to project root.
#   - require_tool:         exit 1 with a clear message if a tool is missing.
#   - run_ffmpeg_decode:    ffmpeg source-format → raw PCM s16le mono 16 kHz.
#   - run_ffprobe_duration: ffprobe-reported duration of a WAV/PCM/any file.
#   - assert_duration:      compares a measured duration against an expected
#                           value with a configurable tolerance; returns
#                           non-zero on mismatch (used as the scenario's
#                           known exit code).
#   - wav_wrap:             wrap a raw PCM stream into a valid WAV file
#                           so ffprobe can introspect it.
#   - decode_via_to_pcm_int16: round-trip the production ``to_pcm_int16``
#                           helper from rob_box_voice so the scenario
#                           exercises the exact decode path the production
#                           ``tts_node`` would use for that container.
#
# Conventions
# -----------
# - Scenarios run from any CWD; all paths are resolved via locate_repo_root.
# - Output WAV / raw PCM files land under
#       tts_audio_bench/artifacts/ffmpeg_decode/<scenario>/
#   so a single ``find tts_audio_bench/artifacts/ffmpeg_decode`` shows every
#   decoded artifact produced by the suite.
# - Exit codes are documented in tools/ffmpeg_decode/README.md and stable:
#       0 — pass
#       1 — required tool missing (ffmpeg / ffprobe / python3)
#       2 — source fixture missing
#       3 — ffmpeg decode failed
#       4 — duration outside tolerance
#       5 — production ``to_pcm_int16`` round-trip failed
#       6 — duration sanity check failed before even running ffmpeg
set -euo pipefail

# ----------------------------------------------------------------------
# Repo location
# ----------------------------------------------------------------------
locate_repo_root() {
    # Walk up from the script's directory until we find a Makefile.
    local dir="${BASH_SOURCE[0]}"
    while true; do
        dir="$(cd "$(dirname "$dir")" && pwd)"
        if [[ -f "${dir}/Makefile" ]]; then
            echo "${dir}"
            return 0
        fi
        if [[ "${dir}" == "/" ]]; then
            echo "ERROR: could not locate project root (no Makefile found)" >&2
            return 1
        fi
    done
}

REPO_ROOT="$(locate_repo_root)"
ARTIFACTS_BASE="${REPO_ROOT}/tts_audio_bench/artifacts/ffmpeg_decode"
FIXTURES_ROOT="${REPO_ROOT}/tts_audio_bench/fixtures"

# ----------------------------------------------------------------------
# Tool presence
# ----------------------------------------------------------------------
require_tool() {
    local tool="$1"
    if ! command -v "${tool}" >/dev/null 2>&1; then
        echo "FAIL: required tool '${tool}' not found on PATH" >&2
        exit 1
    fi
}

# ----------------------------------------------------------------------
# ffmpeg: source-format → raw PCM s16le mono @ 16 kHz.
# Args:
#   $1 — scenario name (used for output directory)
#   $2 — source file path (PCM raw, WAV, MP3, or OGG)
#   $3 — input container hint for ffmpeg (e.g. "s16le" for raw PCM)
#        — pass empty string for containerised formats (WAV/MP3/OGG).
# Output:
#   absolute path to the .raw file written under artifacts/.
# ----------------------------------------------------------------------
run_ffmpeg_decode() {
    local scenario="$1"
    local source="$2"
    local in_fmt="$3"

    local out_dir="${ARTIFACTS_BASE}/${scenario}"
    mkdir -p "${out_dir}"
    local out_raw="${out_dir}/decoded.raw"

    local -a input_args
    if [[ -n "${in_fmt}" ]]; then
        # Raw PCM needs explicit format / sample-rate / channel hints so
        # ffmpeg can parse the bytes. The fixtures are 16 kHz mono int16
        # by convention; a scenario that needs a different rate should
        # override via TO_PCM_INPUT_RATE below.
        input_args=(-f "${in_fmt}" -ar "${TO_PCM_INPUT_RATE:-16000}" -ac 1 -i "${source}")
    else
        input_args=(-i "${source}")
    fi

    # -y: overwrite. -v error: quiet. Output is little-endian 16-bit mono.
    if ! ffmpeg -y -hide_banner -loglevel error \
            "${input_args[@]}" \
            -f s16le -acodec pcm_s16le -ac 1 -ar "${TO_PCM_OUTPUT_RATE:-16000}" \
            "${out_raw}"; then
        echo "FAIL: ffmpeg decode failed for ${scenario} (source=${source})" >&2
        exit 3
    fi
    echo "${out_raw}"
}

# ----------------------------------------------------------------------
# Wrap a raw int16-LE-mono stream into a WAV so ffprobe can introspect.
# Args:
#   $1 — raw PCM file (must be int16 LE mono @ TO_PCM_OUTPUT_RATE)
#   $2 — output WAV path
# ----------------------------------------------------------------------
wav_wrap() {
    local raw="$1"
    local wav="$2"
    local rate="${TO_PCM_OUTPUT_RATE:-16000}"
    python3 - "${raw}" "${wav}" "${rate}" <<'PYEOF'
import sys, struct, wave
raw_path, wav_path, rate = sys.argv[1], sys.argv[2], int(sys.argv[3])
with open(raw_path, "rb") as fh:
    pcm = fh.read()
with wave.open(wav_path, "wb") as w:
    w.setnchannels(1)
    w.setsampwidth(2)
    w.setframerate(rate)
    w.writeframes(pcm)
PYEOF
}

# ----------------------------------------------------------------------
# ffprobe-reported duration in seconds (with stderr suppression).
# Args:
#   $1 — file (WAV, raw PCM with -f s16le, or containerised audio)
#        — for raw PCM, callers should pass the WAV-wrapped version.
# Echoes a float duration.
# ----------------------------------------------------------------------
run_ffprobe_duration() {
    ffprobe -v error -show_entries format=duration \
        -of default=noprint_wrappers=1:nokey=1 "$1"
}

# ----------------------------------------------------------------------
# Duration assertion. Compares two floats with a tolerance and emits a
# clear PASS / FAIL line on stdout/stderr. Returns 0 on match, 4 on
# mismatch (used as the scenario's exit code).
# Args:
#   $1 — scenario name
#   $2 — measured duration (float, seconds)
#   $3 — expected duration (float, seconds)
#   $4 — absolute tolerance (float, seconds, default 0.05)
# ----------------------------------------------------------------------
assert_duration() {
    local scenario="$1"
    local measured="$2"
    local expected="$3"
    local tol="${4:-0.05}"
    python3 - "${scenario}" "${measured}" "${expected}" "${tol}" <<'PYEOF'
import sys
scenario, measured, expected, tol = sys.argv[1:5]
m = float(measured); e = float(expected); t = float(tol)
diff = abs(m - e)
status = "PASS" if diff <= t else "FAIL"
print(f"[{scenario}] duration: measured={m:.3f}s expected={e:.3f}s "
      f"tol={t:.3f}s diff={diff:.3f}s → {status}")
sys.exit(0 if status == "PASS" else 4)
PYEOF
}

# ----------------------------------------------------------------------
# Production-path round-trip via ``to_pcm_int16``.
#
# This calls into rob_box_voice.utils.audio_transcode.to_pcm_int16 so the
# scenario exercises the *exact* decode function tts_node calls for the
# given container. We feed the decoded raw PCM back through the helper
# (wrapped as the right container) and assert the helper's reported
# sample-rate matches the scenario's expected output rate and the
# decoded duration is within the same ±0.05 s tolerance as the
# ffprobe-based gate above.
#
# Note on byte count: OGG/Vorbis (and to a lesser extent MP3) re-encoding
# adds/removes a handful of priming samples per cycle, so the byte count
# after a second encode-decode round-trip is *not* guaranteed to match
# the source byte-for-byte. Duration is the durable acceptance signal
# (it survives encoder padding); the byte_diff_pct is logged for
# visibility but does not affect PASS/FAIL.
#
# Args:
#   $1 — scenario name
#   $2 — container name (lowercase: pcm / wav / mp3 / ogg)
#   $3 — raw decoded PCM file (int16 LE mono @ TO_PCM_OUTPUT_RATE)
#   $4 — expected duration (float, seconds)
# Returns 0 on match, 5 on round-trip mismatch.
# ----------------------------------------------------------------------
decode_via_to_pcm_int16() {
    local scenario="$1"
    local container="$2"
    local decoded_raw="$3"
    local expected_dur="$4"
    local rate="${TO_PCM_OUTPUT_RATE:-16000}"
    python3 - "${scenario}" "${container}" "${decoded_raw}" "${expected_dur}" "${rate}" "${REPO_ROOT}" <<'PYEOF'
import sys, os, struct, wave, io
scenario, container, raw_path, expected_dur, rate, repo_root = sys.argv[1:7]
expected_dur = float(expected_dur); rate = int(rate)

# Make rob_box_voice + rob_box_llm importable from the source tree.
sys.path.insert(0, os.path.join(repo_root, "src", "rob_box_voice"))
sys.path.insert(0, os.path.join(repo_root, "src", "rob_box_llm"))
# rob_box_voice.utils expects audio_transcode to be importable; on the
# source tree it's rob_box_voice.utils.audio_transcode which imports
# from rob_box_llm.tts, so the order above is sufficient.

# Stub the heavy audio deps that rob_box_voice imports at module-load
# time on hosts that don't have pyaudio/USB libs installed (mirrors
# what tts_audio_bench/scripts/run_bench.py does).
import types

def _stub(name, attrs=None):
    mod = types.ModuleType(name)
    for k, v in (attrs or {}).items():
        setattr(mod, k, v)
    sys.modules[name] = mod
    return mod

class _PyAudio:
    def __init__(self, *a, **kw): return None
    def get_device_count(self): return 0

_stub("pyaudio", {"PyAudio": _PyAudio})
_stub("pixel_ring")
_stub("spidev")
usb_pkg = _stub("usb")
usb_core = _stub("usb.core", {"Device": type("_UsbDevice", (), {})})
usb_pkg.core = usb_core  # type: ignore[attr-defined]
_stub("usb.util")

from rob_box_voice.utils.audio_transcode import to_pcm_int16
from rob_box_llm.tts import TTSFormat

fmt_map = {
    "pcm": TTSFormat.PCM,
    "wav": TTSFormat.WAV,
    "mp3": TTSFormat.MP3,
    "ogg": TTSFormat.OGG,
}
fmt = fmt_map[container.lower()]

# Build the right container around the raw decoded PCM so we feed the
# production helper a realistic input (PCM provider response / WAV blob
# / MP3 bytes / OGG bytes).
with open(raw_path, "rb") as fh:
    raw = fh.read()

if fmt == TTSFormat.PCM:
    blob = raw
elif fmt == TTSFormat.WAV:
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(rate)
        w.writeframes(raw)
    blob = buf.getvalue()
else:
    # Round-trip via ffmpeg (encode raw PCM → MP3/OGG) so the input to
    # to_pcm_int16 is a real compressed blob.
    import subprocess, tempfile
    with tempfile.NamedTemporaryFile(suffix="." + container, delete=False) as tf:
        out_path = tf.name
    try:
        subprocess.run(
            [
                "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
                "-f", "s16le", "-ar", str(rate), "-ac", "1", "-i", raw_path,
                "-codec:a", "libmp3lame" if fmt == TTSFormat.MP3 else "libvorbis",
                "-b:a", "64k", out_path,
            ],
            check=True,
        )
        with open(out_path, "rb") as fh:
            blob = fh.read()
    finally:
        try: os.unlink(out_path)
        except FileNotFoundError: pass

decoded = to_pcm_int16(blob, fmt, default_sample_rate=rate)
got_dur = len(decoded.pcm) / 2 / float(decoded.sample_rate)
expected_bytes = int(round(expected_dur * rate * 2))
got_bytes = len(decoded.pcm)
byte_diff_pct = abs(got_bytes - expected_bytes) / expected_bytes * 100.0
dur_diff = abs(got_dur - expected_dur)

ok = decoded.channels == 1 and decoded.sample_rate == rate \
     and dur_diff <= 0.05
status = "PASS" if ok else "FAIL"
print(f"[{scenario}/to_pcm_int16] fmt={container} sr={decoded.sample_rate} "
      f"ch={decoded.channels} bytes={got_bytes} (expected~{expected_bytes}, "
      f"diff={byte_diff_pct:.2f}%) dur={got_dur:.3f}s (expected={expected_dur:.3f}s, "
      f"diff={dur_diff:.3f}s) → {status}")
sys.exit(0 if ok else 5)
PYEOF
}

# ----------------------------------------------------------------------
# Sanity-check that the source fixture exists before ffmpeg runs.
# ----------------------------------------------------------------------
assert_fixture_exists() {
    local fixture="$1"
    if [[ ! -f "${fixture}" ]]; then
        echo "FAIL: source fixture not found: ${fixture}" >&2
        exit 2
    fi
}
