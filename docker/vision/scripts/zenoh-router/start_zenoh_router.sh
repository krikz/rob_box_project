#!/bin/sh
set -e

echo "=========================================="
echo "  Vision Zenoh Router"
echo "=========================================="
echo "Config: /config/zenoh_router_config.json5"

# Копируем конфиг в /tmp и подставляем параметризованные IP из env
# (defaults сохраняют текущую конфигурацию 10.1.1.x)
CONFIG_FILE="/tmp/zenoh_router_config.json5"
cp /config/zenoh_router_config.json5 "$CONFIG_FILE"
sed -i "s|\${ZENOH_MAIN_PI_IP}|${ZENOH_MAIN_PI_IP:-10.1.1.10}|g; s|\${ZENOH_VISION_PI_IP}|${ZENOH_VISION_PI_IP:-10.1.1.11}|g" "$CONFIG_FILE"

echo "   ZENOH_MAIN_PI_IP=${ZENOH_MAIN_PI_IP:-10.1.1.10}"
echo "   ZENOH_VISION_PI_IP=${ZENOH_VISION_PI_IP:-10.1.1.11}"

exec /zenohd -c "$CONFIG_FILE"
