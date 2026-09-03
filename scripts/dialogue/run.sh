#!/usr/bin/env bash
# Локальный текстовый чат с диалоговой системой РОББОКСа (без ROS2).
#
#   ./scripts/dialogue/run.sh
#   ./scripts/dialogue/run.sh --providers deepseek --debug
#   ./scripts/dialogue/run.sh --once "роббокс, привет" --wake-word
#
# Выбирает интерпретатор: .venv в корне репозитория → python3.12+ →
# python3. Ниже 3.12 не поедет: rob_box_core объявляет dataclass-поле со
# значением mappingproxy, что разрешено только с 3.12.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

export PYTHONUTF8=1
export PYTHONIOENCODING=utf-8

pick_python() {
    if [[ -x "${REPO_ROOT}/.venv/bin/python" ]]; then
        echo "${REPO_ROOT}/.venv/bin/python"; return
    fi
    if [[ -x "${REPO_ROOT}/.venv/Scripts/python.exe" ]]; then
        echo "${REPO_ROOT}/.venv/Scripts/python.exe"; return
    fi
    for candidate in python3.13 python3.12 python3 python; do
        if command -v "${candidate}" >/dev/null 2>&1; then
            echo "${candidate}"; return
        fi
    done
    return 1
}

PYTHON="$(pick_python)" || {
    echo "Python не найден. Нужен Python 3.12+." >&2
    exit 1
}

exec "${PYTHON}" "${SCRIPT_DIR}/chat.py" "$@"
