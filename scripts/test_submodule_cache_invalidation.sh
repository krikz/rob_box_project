#!/bin/bash
# Тест проверки инвалидации кэша при изменении субмодулей
# Этот скрипт проверяет что Docker пересобирает образ при изменении SHA субмодуля

set -e

echo "=== Тест инвалидации кэша Docker при изменении субмодулей ==="
echo

# Цвета для вывода
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Проверяем что мы в корне проекта
if [ ! -f ".gitmodules" ]; then
    echo -e "${RED}Ошибка: Запустите скрипт из корня проекта${NC}"
    exit 1
fi

echo "1. Проверка текущего SHA субмодулей..."
ROS2LEDS_SHA=$(git submodule status src/ros2leds | awk '{print $1}' | sed 's/^[+-]//')
VESC_NEXUS_SHA=$(git submodule status src/vesc_nexus | awk '{print $1}' | sed 's/^[+-]//')

echo "   ros2leds SHA: ${ROS2LEDS_SHA}"
echo "   vesc_nexus SHA: ${VESC_NEXUS_SHA}"
echo

echo "2. Проверка что Dockerfile содержат ARG для субмодулей..."

# Проверяем led_matrix Dockerfile
if grep -q "ARG ROS2LEDS_SHA" docker/vision/led_matrix/Dockerfile; then
    echo -e "   ${GREEN}✓${NC} led_matrix Dockerfile содержит ARG ROS2LEDS_SHA"
else
    echo -e "   ${RED}✗${NC} led_matrix Dockerfile НЕ содержит ARG ROS2LEDS_SHA"
    exit 1
fi

# Проверяем vesc_nexus Dockerfile
if grep -q "ARG VESC_NEXUS_SHA" docker/main/vesc_nexus/Dockerfile; then
    echo -e "   ${GREEN}✓${NC} vesc_nexus Dockerfile содержит ARG VESC_NEXUS_SHA"
else
    echo -e "   ${RED}✗${NC} vesc_nexus Dockerfile НЕ содержит ARG VESC_NEXUS_SHA"
    exit 1
fi

# Проверяем ros2_control Dockerfile
if grep -q "ARG VESC_NEXUS_SHA" docker/main/ros2_control/Dockerfile; then
    echo -e "   ${GREEN}✓${NC} ros2_control Dockerfile содержит ARG VESC_NEXUS_SHA"
else
    echo -e "   ${RED}✗${NC} ros2_control Dockerfile НЕ содержит ARG VESC_NEXUS_SHA"
    exit 1
fi

echo

echo "3. Проверка что Dockerfile используют ARG для инвалидации кэша..."

# Проверяем что есть RUN echo с SHA
if grep -q 'RUN echo "Building with ros2leds SHA: \${ROS2LEDS_SHA}"' docker/vision/led_matrix/Dockerfile; then
    echo -e "   ${GREEN}✓${NC} led_matrix Dockerfile использует ROS2LEDS_SHA в RUN echo"
else
    echo -e "   ${RED}✗${NC} led_matrix Dockerfile НЕ использует ROS2LEDS_SHA в RUN echo"
    exit 1
fi

if grep -q 'RUN echo "Building with vesc_nexus SHA: \${VESC_NEXUS_SHA}"' docker/main/vesc_nexus/Dockerfile; then
    echo -e "   ${GREEN}✓${NC} vesc_nexus Dockerfile использует VESC_NEXUS_SHA в RUN echo"
else
    echo -e "   ${RED}✗${NC} vesc_nexus Dockerfile НЕ использует VESC_NEXUS_SHA в RUN echo"
    exit 1
fi

if grep -q 'RUN echo "Building with vesc_nexus SHA: \${VESC_NEXUS_SHA}"' docker/main/ros2_control/Dockerfile; then
    echo -e "   ${GREEN}✓${NC} ros2_control Dockerfile использует VESC_NEXUS_SHA в RUN echo"
else
    echo -e "   ${RED}✗${NC} ros2_control Dockerfile НЕ использует VESC_NEXUS_SHA в RUN echo"
    exit 1
fi

echo

echo "4. Проверка workflow файлов..."

# Проверяем L-Build Vision Pi Services
if grep -q 'ROS2LEDS_SHA=$(git submodule status src/ros2leds' .github/workflows/L-Build\ Vision\ Pi\ Services.yml; then
    echo -e "   ${GREEN}✓${NC} L-Build Vision Pi Services передаёт ROS2LEDS_SHA"
else
    echo -e "   ${RED}✗${NC} L-Build Vision Pi Services НЕ передаёт ROS2LEDS_SHA"
    exit 1
fi

# Проверяем L-Build Main Pi Services
if grep -q 'VESC_NEXUS_SHA=$(git submodule status src/vesc_nexus' .github/workflows/L-Build\ Main\ Pi\ Services.yml; then
    echo -e "   ${GREEN}✓${NC} L-Build Main Pi Services передаёт VESC_NEXUS_SHA"
else
    echo -e "   ${RED}✗${NC} L-Build Main Pi Services НЕ передаёт VESC_NEXUS_SHA"
    exit 1
fi

# Проверяем G-Build Vision Pi Services  
if grep -q "ROS2LEDS_SHA=.*hashFiles('src/ros2leds/" .github/workflows/G-Build\ Vision\ Pi\ Services.yml; then
    echo -e "   ${GREEN}✓${NC} G-Build Vision Pi Services передаёт ROS2LEDS_SHA через hashFiles"
else
    echo -e "   ${RED}✗${NC} G-Build Vision Pi Services НЕ передаёт ROS2LEDS_SHA"
    exit 1
fi

# Проверяем G-Build Main Pi Services
if grep -q "VESC_NEXUS_SHA=.*hashFiles('src/vesc_nexus/" .github/workflows/G-Build\ Main\ Pi\ Services.yml; then
    echo -e "   ${GREEN}✓${NC} G-Build Main Pi Services передаёт VESC_NEXUS_SHA через hashFiles"
else
    echo -e "   ${RED}✗${NC} G-Build Main Pi Services НЕ передаёт VESC_NEXUS_SHA"
    exit 1
fi

echo
echo -e "${GREEN}=== Все проверки пройдены успешно! ===${NC}"
echo
echo "Механизм инвалидации кэша настроен правильно."
echo "При изменении субмодулей Docker будет пересобирать образы."
