#!/usr/bin/env bash
# Ставит тег ядра из Dockerfile voice-assistant в изолированный venv.
# Префикс не системный и не .venv чекаута: CI и локальный прогон не делят пакеты.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
DOCKERFILE="$ROOT/docker/sound/voice_assistant/Dockerfile"
PIN="$(grep -oE "rob_box_dialog_core@[A-Za-z0-9._-]+" "$DOCKERFILE" | head -1 | cut -d@ -f2)"
if [ -z "$PIN" ]; then
  echo "пин rob_box_dialog_core@ не найден в $DOCKERFILE" >&2
  exit 1
fi

if [ -n "${ROB_BOX_DIALOG_CORE_PREFIX:-}" ]; then
  PREFIX="$ROB_BOX_DIALOG_CORE_PREFIX"
elif [ -n "${RUNNER_TEMP:-}" ]; then
  PREFIX="$RUNNER_TEMP/dialog-core-pin"
else
  PREFIX="/tmp/dialog-core-pin-$PIN"
fi

# setup-python в CI даёт venv сам. Без python3-venv (локально) — uv.
if ! python3 -m venv "$PREFIX" 2>/dev/null; then
  command -v uv >/dev/null 2>&1 || { echo "нет python3-venv и нет uv" >&2; exit 1; }
  rm -rf "$PREFIX"
  uv venv --python python3 "$PREFIX"
fi
if ! "$PREFIX/bin/python" -m pip --version >/dev/null 2>&1; then
  uv pip install --python "$PREFIX/bin/python" pip
fi
"$PREFIX/bin/python" -m pip install --upgrade pip
"$PREFIX/bin/python" -m pip install "rob-box-dialog-core @ git+https://github.com/krikz/rob_box_dialog_core@${PIN}"
"$PREFIX/bin/python" -I -c "import rob_box_dialog; print(rob_box_dialog.__file__)"
echo "PIN=$PIN"
echo "PREFIX=$PREFIX"
if [ -n "${GITHUB_ENV:-}" ]; then
  echo "ROB_BOX_DIALOG_CORE_PREFIX=$PREFIX" >> "$GITHUB_ENV"
fi
