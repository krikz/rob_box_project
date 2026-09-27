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
# ============================================================================

set -euo pipefail

if [[ -z "${USER:-}" ]]; then
    USER="$(id -un)"
fi

if [[ "$USER" != "ros2" ]]; then
    echo "⚠️  Внимание: USER=$USER (ожидается ros2). Unit будет создан для этого пользователя."
    echo "    Если контейнеры запускаются от ros2, то docker compose up -d от '$USER' может упасть."
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SETUP_SCRIPT="$SCRIPT_DIR/setup_vision_pi.sh"

# setup_health_monitor() использует BASH_SOURCE[0] для поиска
# ../monitoring/robbox_vision_health_check.sh. Поэтому нельзя source'ить
# через process substitution <(awk ...): там BASH_SOURCE укажет на /dev/fd/*.
# Временный файл рядом с setup_vision_pi.sh сохраняет корректный dirname.
FILTERED_SETUP="$(mktemp "$SCRIPT_DIR/.setup_vision_pi.filtered.XXXXXX.sh")"
trap 'rm -f "$FILTERED_SETUP"' EXIT
awk '/^main\(\) \{/{exit} {print}' "$SETUP_SCRIPT" > "$FILTERED_SETUP"
source "$FILTERED_SETUP"

echo "🔧 Reinstalling systemd units + health-timer (user=$USER, home=$HOME)"

# CI-режим (L-Deploy and Verify): ssh без TTY не даёт sudo спросить пароль,
# а setup_autostart/setup_health_monitor используют `sudo tee ... << EOF` —
# stdin занят heredoc'ом, поэтому `sudo -S` там не применить. Кэшируем
# credentials заранее через `sudo -v` (пароль из SUDO_PASSWORD): дальнейшие
# sudo-вызовы в этом же процессе не требуют TTY. Вручную (с TTY)
# SUDO_PASSWORD не задан — sudo спросит пароль сам, поведение не меняется.
if [[ -n "${SUDO_PASSWORD:-}" ]]; then
    echo "$SUDO_PASSWORD" | command sudo -S -p '' -v
fi

setup_autostart
setup_health_monitor

echo "✅ Done"
