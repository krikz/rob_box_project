#!/usr/bin/env bash
# Консоль диалога РОББОКСа на ходе v2 (сухой стенд, без ROS2 и без робота).
#
#   ./scripts/dialogue/run.sh
#   ./scripts/dialogue/run.sh --providers deepseek --debug
#   ./scripts/dialogue/run.sh --once "Сколько у тебя заряда?" --once "Робот, стоп"
#   ./scripts/dialogue/run.sh --db ~/.rob_box/console_memory.e2e.db --variant B
#
# Выбирает интерпретатор: .venv в корне репозитория → python3.12+ →
# python3. Нужны пакеты `openai`, `httpx`, `pyyaml` (зависимости rob_box_llm).
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
