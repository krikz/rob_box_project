#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════
# 🔊 РОББОКС Sound Pi Setup (ADR-0159)
# ═══════════════════════════════════════════════════════════════════════════
# Быстрая настройка третьего узла — Sound Pi: ReSpeaker, голосовой стек,
# SuperCollider, perception_bridge для ESP32 (serial через TTL-USB, profile sensors). Обёртка над setup_node.sh
# с ролью `sound`, по образцу setup_main_pi.sh.
#
# Использование (на самом Sound Pi, пользователь ros2):
#   curl -fsSL https://raw.githubusercontent.com/krikz/rob_box_project/develop/scripts/setup/setup_sound_pi.sh | bash
# или из клона:
#   bash ~/rob_box_project/scripts/setup/setup_sound_pi.sh
#
# Что настраивает роль `sound` (см. setup_node.sh → setup_sound_node):
#   - Docker + compose, клон репо, MOTD, aliases, systemd robbox-sound.service
#     (WorkingDirectory docker/sound)
#   - netplan: eth0 metric 100 + wlan0 metric 600 (как на Main/Vision Pi)
#   - config.txt: usb_max_current_enable=1, enable_uart=1 (нужна перезагрузка)
#   - udev: /dev/sensor_hub для TTL-USB конвертера ESP32
#   - alsa-utils, каталоги /opt/rob_box/{models,samples,scores}, zram-своп
#   - /etc/docker/daemon.json: insecure-registries 10.1.1.249:5000 (деплой с registry_source=local)
#   - chrony --role sound (клиент времени Main Pi, ADR-0137)
# ═══════════════════════════════════════════════════════════════════════════

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NEW_SCRIPT="$SCRIPT_DIR/setup_node.sh"

if [ ! -f "$NEW_SCRIPT" ]; then
    curl -fsSL https://raw.githubusercontent.com/krikz/rob_box_project/develop/scripts/setup/setup_node.sh | bash -s sound
else
    bash "$NEW_SCRIPT" sound
fi
