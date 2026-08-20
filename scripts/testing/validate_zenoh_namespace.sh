#!/bin/bash
# Проверка namespace Zenoh (Lyrical: ZENOH_CONFIG_OVERRIDE в контейнерах).
# Запускать на Main/Vision Pi. Раньше namespace генерился в /tmp/zenoh_session_config.json5
# через sed в ros_with_namespace.sh — теперь это одна env-переменная в compose.
set -e

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'

# Определяем хост
HOST_TYPE=""
if [ -f "/etc/hostname" ]; then
    HOSTNAME=$(cat /etc/hostname)
    [[ "$HOSTNAME" == *"vision"* ]] && HOST_TYPE="vision"
    [[ "$HOSTNAME" == *"main"* ]] && HOST_TYPE="main"
fi
if [ -z "$HOST_TYPE" ]; then
    echo -e "${RED}✗ Не могу определить Pi (vision/main)${NC}"
    echo "Запускайте на Main или Vision Pi"
    exit 1
fi
echo -e "${GREEN}✅ Detected: ${HOST_TYPE^^} Pi${NC}"
echo ""

# ROBOT_ID из .env
ENV_FILE="$HOME/rob_box_project/docker/${HOST_TYPE}/.env"
if [ ! -f "$ENV_FILE" ]; then
    echo -e "${RED}✗ Нет $ENV_FILE${NC}"
    exit 1
fi
ROBOT_ID=$(grep "^ROBOT_ID=" "$ENV_FILE" | cut -d'=' -f2)
if [ -z "$ROBOT_ID" ]; then
    echo -e "${RED}✗ ROBOT_ID не задан в $ENV_FILE${NC}"
    exit 1
fi
echo -e "${GREEN}✓ ROBOT_ID=$ROBOT_ID → namespace: robots/$ROBOT_ID${NC}"
echo ""

# Проверка роутера
ROUTER_CONTAINER="zenoh-router"
[ "$HOST_TYPE" = "vision" ] && ROUTER_CONTAINER="zenoh-router-vision"
if docker ps --format '{{.Names}}' | grep -q "^${ROUTER_CONTAINER}$"; then
    echo -e "${GREEN}✓ ${ROUTER_CONTAINER} запущен${NC}"
    if docker exec "$ROUTER_CONTAINER" pgrep -f rmw_zenohd >/dev/null 2>&1; then
        echo -e "${GREEN}✓ rmw_zenohd процесс жив${NC}"
    else
        echo -e "${RED}✗ rmw_zenohd не запущен в ${ROUTER_CONTAINER}${NC}"
    fi
else
    echo -e "${RED}✗ ${ROUTER_CONTAINER} не запущен${NC}"
fi
echo ""

# Проверка ZENOH_CONFIG_OVERRIDE в ROS-контейнерах
if [ "$HOST_TYPE" = "vision" ]; then
    NODES=("oak-d" "led-matrix" "ceiling-camera" "voice-assistant" "telegram-bot")
else
    NODES=("twist-mux" "robot-state-publisher" "rtabmap" "ros2-control" "lslidar" "perception" "nav2" "teleop")
fi

for c in "${NODES[@]}"; do
    if docker ps --format '{{.Names}}' | grep -q "^${c}$"; then
        OV=$(docker exec "$c" env 2>/dev/null | grep '^ZENOH_CONFIG_OVERRIDE=' | cut -d'=' -f2- || echo "")
        if [[ "$OV" == *"namespace=\"robots/$ROBOT_ID\""* ]]; then
            echo -e "${GREEN}✓ $c: namespace ok${NC}"
        else
            echo -e "${RED}✗ $c: нет namespace в ZENOH_CONFIG_OVERRIDE (got: $OV)${NC}"
        fi
    else
        echo -e "${YELLOW}⚠️  $c не запущен${NC}"
    fi
done

echo ""
echo -e "${BLUE}Ожидаемые топики в облаке: robots/$ROBOT_ID/...${NC}"
echo -e "${BLUE}Проверка: ros2 topic list | grep robots/$ROBOT_ID${NC}"
