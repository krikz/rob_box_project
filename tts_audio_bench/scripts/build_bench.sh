#!/usr/bin/bash
# Build rob_box_llm + rob_box_voice for the bench.
#
# Inside the project's `ghcr.io/krikz/rob_box_base:rtabmap` Docker
# image (Ubuntu 22.04 jammy + ROS2 Humble — see
# `docker/main/ros2_control/Dockerfile` for an example of extending
# that base) this is a straight `colcon build`. On hosts without ROS2
# (e.g. Debian 13 trixie used by the bench container), we fall back to
# a Python-only sanity check: import every module the bench exercises
# and confirm the production code parses + initialises correctly.
#
# The script returns 0 if either path succeeds — the bench is designed
# to be runnable on minimal hosts.

set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="${SCRIPT_DIR}/../.."

# ---------------------------------------------------------------------------
# 1) Detect environment.
# ---------------------------------------------------------------------------

HAS_COLCON=0
HAS_ROS2=0
if command -v colcon >/dev/null 2>&1; then
    HAS_COLCON=1
fi
if [ -f /opt/ros/humble/setup.bash ]; then
    HAS_ROS2=1
fi

if [ "$HAS_COLCON" = "1" ] && [ "$HAS_ROS2" = "1" ]; then
    echo "[build_bench] Detected ROS2 Humble + colcon — running real colcon build"
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    cd "${PROJECT_ROOT}"
    colcon build \
        --packages-select rob_box_llm rob_box_voice \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers console_direct+
    echo "[build_bench] colcon build OK"
    exit 0
fi

# ---------------------------------------------------------------------------
# 2) Fallback: minimal Python-only build.
# ---------------------------------------------------------------------------
echo "[build_bench] ROS2 Humble not detected — running fallback (Python import check)"
cd "${PROJECT_ROOT}"
PYTHONPATH=src/rob_box_llm:src/rob_box_voice python3 - <<'PY'
"""Fallback build: validate that the bench can import the production code.

Runs the same import surface as the bench but stops short of spinning up
a real ROS node (which requires rclpy). This catches:
* setup.py missing data_files
* missing __init__.py
* syntax errors in any module
* import-time failures (e.g. circular import, missing dep)
"""
import sys
import importlib

required = [
    "rob_box_llm",
    "rob_box_llm.tts",
    "rob_box_llm.errors",
    "rob_box_llm.providers.minimax_tts",
    "rob_box_llm.providers.deepseek",
    "rob_box_llm.providers.fake",
]

failed = []
for name in required:
    try:
        importlib.import_module(name)
        print(f"  [OK] import {name}")
    except Exception as exc:
        failed.append((name, exc))
        print(f"  [FAIL] import {name}: {exc!r}")

# The bench reaches into rob_box_voice.utils.audio_transcode via a sys.path
# hack (because utils/__init__.py imports pyaudio, which the bench host
# doesn't have). Verify audio_transcode imports cleanly under that path.
sys.path.insert(0, "src/rob_box_voice/rob_box_voice/utils")
try:
    import audio_transcode  # noqa: F401
    print("  [OK] import audio_transcode (via utils path)")
except Exception as exc:
    failed.append(("audio_transcode", exc))
    print(f"  [FAIL] import audio_transcode: {exc!r}")

# TTSNode itself drags in torch/sounddevice/yandex-cloud-ml-sdk — these
# are not needed by the bench but are needed in production. Verify the
# module PARSES without import-time failures (compile-only).
import py_compile
try:
    py_compile.compile(
        "src/rob_box_voice/rob_box_voice/tts_node.py",
        doraise=True,
    )
    print("  [OK] py_compile src/rob_box_voice/rob_box_voice/tts_node.py")
except py_compile.PyCompileError as exc:
    failed.append(("tts_node.py", exc))
    print(f"  [FAIL] py_compile tts_node.py: {exc}")

if failed:
    print(f"\n[build_bench] {len(failed)} import/compile failure(s):")
    for name, exc in failed:
        print(f"  - {name}: {exc!r}")
    sys.exit(1)

print("\n[build_bench] Python fallback build OK")
PY