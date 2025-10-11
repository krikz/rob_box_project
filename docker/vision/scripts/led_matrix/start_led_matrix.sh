#!/bin/bash
# start_led_matrix.sh - Запуск LED Matrix системы на Vision Pi
#
# Запускает:
#   1. led_matrix_driver - низкоуровневый SPI драйвер
#   2. led_matrix_compositor - композитор логических групп
#
# Использование:
#   ./start_led_matrix.sh

set -e

# Цвета для вывода
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo "============================================"
echo "   LED Matrix System Startup"
echo "============================================"
echo ""

# Проверка SPI устройства
SPI_DEVICE="/dev/spidev0.0"
if [ ! -e "$SPI_DEVICE" ]; then
    echo -e "${RED}❌ ERROR: SPI device $SPI_DEVICE not found!${NC}"
    echo ""
    echo "SPI должен быть включен в Raspberry Pi Configuration:"
    echo "  sudo raspi-config"
    echo "    → Interface Options"
    echo "    → SPI"
    echo "    → Yes"
    echo ""
    echo "Или добавьте в /boot/config.txt:"
    echo "  dtparam=spi=on"
    echo ""
    echo "После включения перезагрузите Pi:"
    echo "  sudo reboot"
    echo ""
    exit 1
fi

echo -e "${GREEN}✅ SPI device found: $SPI_DEVICE${NC}"

# Проверка прав доступа к SPI
if [ ! -r "$SPI_DEVICE" ] || [ ! -w "$SPI_DEVICE" ]; then
    echo -e "${YELLOW}⚠️  WARNING: No read/write permissions for $SPI_DEVICE${NC}"
    echo "Attempting to fix permissions..."
    sudo chmod 666 "$SPI_DEVICE" || {
        echo -e "${RED}❌ Failed to change permissions${NC}"
        echo "Run manually:"
        echo "  sudo chmod 666 $SPI_DEVICE"
        exit 1
    }
    echo -e "${GREEN}✅ Permissions fixed${NC}"
fi

# Проверка установки pi5neo
echo ""
echo "Checking pi5neo library..."
python3 -c "import pi5neo" 2>/dev/null || {
    echo -e "${RED}❌ ERROR: pi5neo library not installed!${NC}"
    echo ""
    echo "Install with:"
    echo "  pip3 install --break-system-packages pi5neo"
    echo ""
    exit 1
}
echo -e "${GREEN}✅ pi5neo library installed${NC}"

# Проверка конфигурационных файлов
DRIVER_CONFIG="/config/led_matrix/led_matrix_driver.yaml"
COMPOSITOR_CONFIG="/config/led_matrix/led_matrix_compositor.yaml"

if [ ! -f "$DRIVER_CONFIG" ]; then
    echo -e "${YELLOW}⚠️  WARNING: Driver config not found: $DRIVER_CONFIG${NC}"
    echo "Using embedded defaults..."
fi

if [ ! -f "$COMPOSITOR_CONFIG" ]; then
    echo -e "${YELLOW}⚠️  WARNING: Compositor config not found: $COMPOSITOR_CONFIG${NC}"
    echo "Using embedded defaults..."
fi

# Source ROS workspace
echo ""
echo "Sourcing ROS workspace..."
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ws/install/setup.bash

echo -e "${GREEN}✅ ROS workspace sourced${NC}"

# Информация о конфигурации
echo ""
echo "============================================"
echo "   LED Configuration"
echo "============================================"
echo "Total LEDs: 381"
echo "  - 4× 8×8 panels (headlights): 256 LEDs"
echo "  - 5× 5×5 panels (main display): 125 LEDs"
echo ""
echo "Logical Groups:"
echo "  - headlight_front_left (8×8)"
echo "  - headlight_front_right (8×8)"
echo "  - headlight_rear_left (8×8)"
echo "  - headlight_rear_right (8×8)"
echo "  - main_display (5×25)"
echo "  - headlights_front (16×8)"
echo "  - headlights_rear (16×8)"
echo "  - headlights_all (16×16)"
echo ""
echo "Topics:"
echo "  IN:  /panel_image (sensor_msgs/Image)"
echo "  OUT: /led_matrix/data (std_msgs/Int8MultiArray)"
echo "============================================"
echo ""

# Запуск launch файла (запускает driver и compositor)
echo -e "${GREEN}🚀 Starting LED Matrix System...${NC}"
echo ""

if [ -f "$COMPOSITOR_CONFIG" ]; then
    exec ros2 launch led_matrix_compositor led_matrix_compositor_launch.py
else
    # Если конфига нет, запускаем с дефолтными параметрами
    exec ros2 run led_matrix_compositor led_matrix_compositor
fi
