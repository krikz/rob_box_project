#!/usr/bin/env bash
# Гоняет гард на установленном пине. Итог pytest — passed, не skip.
# Отрицательный контроль печатает «подмена» и обязан поймать src/ в начале sys.path.
# Второй контроль подсовывает ключ REASON_WORDS, которого в пине нет, и обязан покраснеть (#5210).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$ROOT"
if [ -z "${ROB_BOX_DIALOG_CORE_PREFIX:-}" ]; then
  echo "нет ROB_BOX_DIALOG_CORE_PREFIX: install_pin.sh не записал префикс" >&2
  exit 1
fi

echo "=== negative control: src planted on sys.path ==="
python3 tests/ci/dialog_core_pin/guard.py --substitution

echo "=== negative control: synthetic REASON_WORDS key absent from the pin ==="
python3 tests/ci/dialog_core_pin/guard.py --missing-key

python3 -m pip install --quiet pytest
LOG="$(mktemp)"
set -o pipefail
python3 -m pytest -v -o addopts= -p no:cacheprovider \
  tests/ci/dialog_core_pin/test_dialog_core_installed_pin.py \
  | tee "$LOG"
grep -E "[0-9]+ passed" "$LOG" >/dev/null
if grep -E "^FAILED |^ERROR |[0-9]+ skipped|[0-9]+ failed|[0-9]+ error" "$LOG" >/dev/null; then
  echo "гард пропущен или красный — это не зелёный прогон" >&2
  exit 1
fi
