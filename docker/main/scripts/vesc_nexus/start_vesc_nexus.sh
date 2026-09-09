#!/bin/bash
# start_vesc_nexus.sh - Запуск VESC Nexus драйвера
#
# Проверяет наличие CAN интерфейса и запускает ROS2 ноду
# для управления VESC моторами через CAN

set -e

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}         VESC Nexus Driver Startup Script              ${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"

# Параметры CAN интерфейса
CAN_INTERFACE="${CAN_INTERFACE:-can0}"
CONFIG_FILE="${CONFIG_FILE:-/config/shared/vesc_nexus/vesc_config.yaml}"

echo -e "${GREEN}📋 Configuration:${NC}"
echo -e "   CAN Interface: ${CAN_INTERFACE}"
echo -e "   Config File:   ${CONFIG_FILE}"
echo ""

# ═══════════════════════════════════════════════════════════════
# 1. Проверка наличия CAN интерфейса
# ═══════════════════════════════════════════════════════════════
echo -e "${YELLOW}🔍 Checking CAN interface...${NC}"

if ! ip link show ${CAN_INTERFACE} > /dev/null 2>&1; then
    echo -e "${RED}❌ ERROR: CAN interface '${CAN_INTERFACE}' not found!${NC}"
    echo ""
    echo -e "${YELLOW}Available network interfaces:${NC}"
    ip link show
    echo ""
    echo -e "${YELLOW}Troubleshooting:${NC}"
    echo "  1. Check if CAN Shield is connected to Raspberry Pi"
    echo "  2. Verify Device Tree configuration in /boot/config.txt:"
    echo "     dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25"
    echo "  3. Run setup script on host:"
    echo "     sudo /path/to/setup_can0.sh"
    echo "  4. Restart Raspberry Pi after configuration changes"
    exit 1
fi

# Проверка состояния интерфейса
CAN_STATE=$(ip -details link show ${CAN_INTERFACE} | grep -oP 'state \K\w+' || echo "UNKNOWN")
echo -e "${GREEN}✅ CAN interface found: ${CAN_INTERFACE} (state: ${CAN_STATE})${NC}"

if [ "$CAN_STATE" != "UP" ]; then
    echo -e "${YELLOW}⚠️  WARNING: CAN interface is not UP!${NC}"
    echo "   Run on host: sudo ip link set ${CAN_INTERFACE} up type can bitrate 500000"
fi

# ═══════════════════════════════════════════════════════════════
# 2. Проверка конфигурационного файла
# ═══════════════════════════════════════════════════════════════
echo ""
echo -e "${YELLOW}📄 Checking configuration file...${NC}"

if [ ! -f "${CONFIG_FILE}" ]; then
    echo -e "${RED}❌ ERROR: Configuration file not found: ${CONFIG_FILE}${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Configuration file found${NC}"

# ═══════════════════════════════════════════════════════════════
# 3. Source ROS 2 environment
# ═══════════════════════════════════════════════════════════════
echo ""
echo -e "${YELLOW}🔧 Sourcing ROS 2 environment...${NC}"

if [ -f "/ws/install/setup.bash" ]; then
    source /ws/install/setup.bash
    echo -e "${GREEN}✅ Workspace sourced${NC}"
else
    echo -e "${RED}❌ ERROR: Workspace setup.bash not found!${NC}"
    exit 1
fi

# ═══════════════════════════════════════════════════════════════
# 4. Вывод информации о VESC configuration
# ═══════════════════════════════════════════════════════════════
echo ""
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}              VESC Configuration Summary               ${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"

# Извлекаем параметры из YAML (простой grep, для сложной логики нужен yq)
VESC_IDS=$(grep "vesc_ids:" ${CONFIG_FILE} | sed 's/.*\[\(.*\)\].*/\1/' || echo "N/A")
WHEEL_LABELS=$(grep "wheel_labels:" ${CONFIG_FILE} -A 4 | tail -n 4 | sed 's/.*"\(.*\)".*/\1/' | tr '\n' ', ' || echo "N/A")

echo -e "CAN Interface:  ${CAN_INTERFACE}"
echo -e "VESC IDs:       ${VESC_IDS}"
echo -e "Wheel Labels:   ${WHEEL_LABELS}"
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo ""

# ═══════════════════════════════════════════════════════════════
# 5. Запуск VESC Nexus node
# ═══════════════════════════════════════════════════════════════
echo -e "${GREEN}🚀 Starting VESC Nexus driver...${NC}"
echo ""

# Передаем путь к конфигу через переменную окружения или параметр
export VESC_CONFIG_FILE="${CONFIG_FILE}"

# Запуск launch файла
exec ros2 launch vesc_nexus vesc_nexus_node.launch.py

# Если exec не сработал (не должно произойти), выводим ошибку
echo -e "${RED}❌ ERROR: Failed to start VESC Nexus driver${NC}"
exit 1
