#!/bin/bash
# start_ros2_control.sh - Запуск ROS2 Control Manager с VESC Hardware Interface
#
# Этот скрипт запускает:
# 1. controller_manager (ros2_control_node) с URDF
# 2. Загружает VescSystemHardwareInterface плагин (из URDF)
# 3. Spawns контроллеры (joint_state_broadcaster, diff_drive_controller)

set -e

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}      ROS2 Control Manager + VESC Startup Script       ${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"

# ═══════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════
# CAN_INTERFACE можно переопределить через environment variable
# Поддерживаемые значения: can0, can1
CAN_INTERFACE="${CAN_INTERFACE:-can0}"
URDF_PATH="${URDF_PATH:-/ws/src/rob_box_description/urdf/rob_box.xacro}"
CONTROLLER_CONFIG="${CONTROLLER_CONFIG:-/config/shared/controllers/controller_manager.yaml}"

# Валидация CAN интерфейса
if [[ ! "$CAN_INTERFACE" =~ ^can[01]$ ]]; then
    echo -e "${RED}❌ ERROR: Invalid CAN interface '${CAN_INTERFACE}'${NC}"
    echo "Supported: can0, can1"
    exit 1
fi

echo -e "${GREEN}📋 Configuration:${NC}"
echo -e "   CAN Interface:       ${CAN_INTERFACE}"
echo -e "   URDF Path:           ${URDF_PATH}"
echo -e "   Controller Config:   ${CONTROLLER_CONFIG}"
echo ""

# ═══════════════════════════════════════════════════════════════
# 1. Проверка CAN интерфейса
# ═══════════════════════════════════════════════════════════════
echo -e "${YELLOW}🔍 Checking CAN interface...${NC}"

if ! ip link show ${CAN_INTERFACE} > /dev/null 2>&1; then
    echo -e "${RED}❌ ERROR: CAN interface '${CAN_INTERFACE}' not found!${NC}"
    echo ""
    echo -e "${YELLOW}Available interfaces:${NC}"
    ip link show
    echo ""
    echo -e "${YELLOW}Please run on host:${NC}"
    echo "  cd /path/to/rob_box_project/host/main"
    echo "  sudo ./install_host_scripts.sh"
    echo ""
    echo "Or manually:"
    echo "  sudo /opt/rob_box/setup_can.sh ${CAN_INTERFACE}"
    exit 1
fi

# Проверяем link state (UP/DOWN), игнорируя CAN controller state (ERROR-ACTIVE/ERROR-PASSIVE/BUS-OFF)
LINK_STATE=$(ip link show ${CAN_INTERFACE} | grep -oP '<.*>' | grep -q 'UP' && echo "UP" || echo "DOWN")
CAN_CTRL_STATE=$(ip -details link show ${CAN_INTERFACE} | grep -oP 'can state \K[A-Z-]+' || echo "UNKNOWN")

echo -e "${GREEN}✅ CAN interface: ${CAN_INTERFACE}${NC}"
echo -e "   Link state: ${LINK_STATE}"
echo -e "   CAN controller state: ${CAN_CTRL_STATE}"

if [ "$LINK_STATE" != "UP" ]; then
    echo -e "${RED}❌ ERROR: CAN interface link is DOWN!${NC}"
    echo "Run on host: sudo /path/to/setup_can.sh ${CAN_INTERFACE}"
    exit 1
fi

# ERROR-ACTIVE - нормальное состояние (может передавать и принимать)
# ERROR-PASSIVE - ограниченная передача (много ошибок, но работает)
# BUS-OFF - критическое состояние (полностью отключен от шины)
if [ "$CAN_CTRL_STATE" = "BUS-OFF" ]; then
    echo -e "${RED}❌ ERROR: CAN controller in BUS-OFF state!${NC}"
    echo "Too many errors detected. Check wiring and termination resistors."
    echo "Restart interface: sudo ip link set ${CAN_INTERFACE} down && sudo ip link set ${CAN_INTERFACE} up"
    exit 1
fi

if [ "$CAN_CTRL_STATE" = "ERROR-PASSIVE" ]; then
    echo -e "${YELLOW}⚠️  WARNING: CAN controller in ERROR-PASSIVE state${NC}"
    echo "High error rate detected. Check bus quality."
fi

# ═══════════════════════════════════════════════════════════════
# 2. Source ROS2 workspace
# ═══════════════════════════════════════════════════════════════
echo ""
echo -e "${YELLOW}🔧 Sourcing ROS2 environment...${NC}"

if [ ! -f "/ws/install/setup.bash" ]; then
    echo -e "${RED}❌ ERROR: Workspace not built!${NC}"
    exit 1
fi

source /ws/install/setup.bash
echo -e "${GREEN}✅ Workspace sourced${NC}"

# ═══════════════════════════════════════════════════════════════
# 3. Проверка URDF
# ═══════════════════════════════════════════════════════════════
echo ""
echo -e "${YELLOW}📄 Checking URDF...${NC}"

if [ ! -f "${URDF_PATH}" ]; then
    echo -e "${RED}❌ ERROR: URDF not found: ${URDF_PATH}${NC}"
    exit 1
fi

# Проверяем что URDF валиден через xacro
echo -e "${CYAN}   Validating URDF with xacro...${NC}"
if ! xacro ${URDF_PATH} > /dev/null 2>&1; then
    echo -e "${RED}❌ ERROR: URDF validation failed!${NC}"
    echo "Check xacro syntax in ${URDF_PATH}"
    exit 1
fi

echo -e "${GREEN}✅ URDF valid${NC}"

# ═══════════════════════════════════════════════════════════════
# 4. Проверка controller config
# ═══════════════════════════════════════════════════════════════
echo ""
echo -e "${YELLOW}⚙️  Checking controller configuration...${NC}"

if [ ! -f "${CONTROLLER_CONFIG}" ]; then
    echo -e "${RED}❌ ERROR: Controller config not found: ${CONTROLLER_CONFIG}${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Controller config found${NC}"

# ═══════════════════════════════════════════════════════════════
# 5. Запуск Controller Manager
# ═══════════════════════════════════════════════════════════════
echo ""
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}              Starting Controller Manager               ${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo ""

# Генерируем URDF из xacro
echo -e "${CYAN}📝 Generating URDF from xacro...${NC}"
ROBOT_DESCRIPTION=$(xacro ${URDF_PATH})

echo -e "${GREEN}🚀 Starting ros2_control_node (controller_manager)...${NC}"
echo ""
echo -e "${CYAN}This will:${NC}"
echo -e "${CYAN}  1. Load VescSystemHardwareInterface from URDF${NC}"
echo -e "${CYAN}  2. Initialize CAN connection to VESC motors${NC}"
echo -e "${CYAN}  3. Configure controller_manager${NC}"
echo -e "${CYAN}  4. Wait for controllers to be spawned via CLI${NC}"
echo ""

# Запускаем controller_manager напрямую (ros2_control_node)
# robot_description передается как параметр через --ros-args
# В Humble нет ros2_control_node.launch.py, запускаем ноду напрямую
exec ros2 run controller_manager ros2_control_node \
    --ros-args \
    -p robot_description:="${ROBOT_DESCRIPTION}" \
    --params-file ${CONTROLLER_CONFIG}

# Если exec не сработал (не должно произойти)
echo -e "${RED}❌ ERROR: Failed to start controller manager${NC}"
exit 1
