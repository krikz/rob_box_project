#!/usr/bin/env bash
# Гоняет гард на установленном пине. Итог pytest — 1 passed, не skip.
# Отрицательный контроль печатает «подмена» и обязан поймать src/ в начале sys.path.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$ROOT"
if [ -z "${ROB_BOX_DIALOG_CORE_PREFIX:-}" ]; then
  echo "нет ROB_BOX_DIALOG_CORE_PREFIX: install_pin.sh не записал префикс" >&2
  exit 1
fi

echo "=== negative control: src planted on sys.path ==="
python3 tests/ci/dialog_core_pin/guard.py --substitution

python3 -m pip install --quiet pytest
LOG="$(mktemp)"
set -o pipefail
python3 -m pytest -v -o addopts= -p no:cacheprovider \
  tests/ci/dialog_core_pin/test_dialog_core_installed_pin.py::test_installed_pin_import_is_not_shadowed_by_src \
  | tee "$LOG"
grep -E "1 passed" "$LOG" >/dev/null
