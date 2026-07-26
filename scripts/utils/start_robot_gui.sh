#!/bin/bash
# Запуск Robot Control GUI

# Цвета для вывода
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🤖 Запуск Robot Control GUI...${NC}"

# Настройка Zenoh для подключения к роботу
export ROBOT_ID=RBXU100001
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# Создаём Zenoh конфиг с namespace (как в RViz)
ZENOH_CONFIG="/tmp/zenoh_gui_config_${ROBOT_ID}.json5"
cp "$(dirname "$0")/../local_test/zenoh_client_config.json5" "$ZENOH_CONFIG"
# Добавляем namespace в Zenoh конфиг
sed -i 's|"mode": "peer",|"mode": "peer",\n  "namespace": "robots/'$ROBOT_ID'",|' "$ZENOH_CONFIG"
export ZENOH_SESSION_CONFIG_URI="$ZENOH_CONFIG"

echo -e "${GREEN}✅ Zenoh настроен для робота ${ROBOT_ID}${NC}"

# Переход в директорию проекта
cd "$(dirname "$0")/.." || exit

# Загрузка ROS2 окружения
source /opt/ros/kilted/setup.bash
source install/setup.bash 2>/dev/null || true

# Запуск GUI
echo -e "${BLUE}🚀 Запускаю GUI...${NC}"
python3 tools/robot_control_gui_simple.py

echo -e "${GREEN}👋 GUI закрыт${NC}"
