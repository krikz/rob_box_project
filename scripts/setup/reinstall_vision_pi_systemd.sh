#!/usr/bin/env bash
# ============================================================================
# reinstall_vision_pi_systemd.sh — переустановка ТОЛЬКО systemd unit +
# health-timer на Vision Pi. Без apt install / clone repo / zram-swap.
#
# Используется из CI (L-Deploy and Verify, шаг "[Vision Pi] Reinstall Systemd
# Units + Health Timer", issue #3018), а также руками:
#
#   bash scripts/setup/reinstall_vision_pi_systemd.sh
#
# Контракт:
#   - Идемпотентен: запуск поверх существующих unit/timer безопасен.
#   - Запускать от пользователя, который будет владеть unit (по умолчанию
#     ros2). $USER должен быть ros2 (или тем, кто запускает docker compose).
#   - Требует sudo (запись в /etc/systemd/system, install в /usr/local/bin).
#
# Зачем отдельный скрипт:
#   - setup_vision_pi.sh — это полная переустановка Vision Pi (apt, clone,
#     zram-swap, …). На этапе деплоя CI это лишнее и рискованное.
#   - Нужен именно "refresh unit + timer" из CI.
# ============================================================================

set -euo pipefail

# USER должен быть ros2 (или другим владельцем unit). Если не задан —
# используем текущего, но тогда setup_autostart может создать unit с другим
# User= и сломать текущий стек.
if [[ -z "${USER:-}" ]]; then
    USER="$(id -un)"
fi

if [[ "$USER" != "ros2" ]]; then
    echo "⚠️  Внимание: USER=$USER (ожидается ros2). Unit будет создан для этого пользователя."
    echo "    Если контейнеры запускаются от ros2, то docker compose up -d от '$USER' может упасть."
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Source setup_vision_pi.sh — он определяет все helper-функции
# (setup_autostart, setup_health_monitor, log_*, и т.д.). main() НЕ вызываем.
# awk вырезает всё до первой строки "^main() {" (включительно).
source <(awk '/^main\(\) \{/{exit} {print}' "$SCRIPT_DIR/setup_vision_pi.sh")

echo "🔧 Reinstalling systemd units + health-timer (user=$USER, home=$HOME)"

setup_autostart
setup_health_monitor

echo "✅ Done"
