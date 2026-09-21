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

echo "4. Проверка механизма передачи SHA в сборку..."

# С Phase 2 механизм разнесён: ДАННЫЕ (какой сервис от какого субмодуля
# зависит) живут в docker/build-manifest.yaml, ЛОГИКА (посчитать SHA и
# подмешать build-arg) — в композите .github/actions/l-build-service.
# Пер-Pi workflow'ы только прокидывают поле матрицы в композит.
# Исключение — L-Build Single Service.yml: он композит НЕ использует и
# считает SHA сам.

# 4.1 Манифест объявляет submodule_sha для обоих сервисов
if grep -q "submodule_sha: src/ros2leds" docker/build-manifest.yaml; then
    echo -e "   ${GREEN}✓${NC} build-manifest объявляет submodule_sha для led-matrix"
else
    echo -e "   ${RED}✗${NC} build-manifest НЕ объявляет submodule_sha: src/ros2leds"
    exit 1
fi

if grep -q "submodule_sha: src/vesc_nexus" docker/build-manifest.yaml; then
    echo -e "   ${GREEN}✓${NC} build-manifest объявляет submodule_sha для ros2-control"
else
    echo -e "   ${RED}✗${NC} build-manifest НЕ объявляет submodule_sha: src/vesc_nexus"
    exit 1
fi

# 4.2 Генератор матрицы прокидывает поле в job matrix
if grep -q '"submodule_sha"' scripts/ci/gen_build_matrix.py; then
    echo -e "   ${GREEN}✓${NC} gen_build_matrix.py прокидывает submodule_sha в матрицу"
else
    echo -e "   ${RED}✗${NC} gen_build_matrix.py НЕ прокидывает submodule_sha"
    exit 1
fi

# 4.3 Композит считает SHA и подмешивает его как <BASENAME>_SHA build-arg
if grep -qF 'git submodule status "$SUBMODULE_PATH"' .github/actions/l-build-service/action.yml; then
    echo -e "   ${GREEN}✓${NC} композит l-build-service считает SHA субмодуля"
else
    echo -e "   ${RED}✗${NC} композит l-build-service НЕ считает SHA субмодуля"
    exit 1
fi

# 4.4 Пер-Pi workflow'ы передают поле матрицы в композит
for wf in "L-Build Vision Pi Services" "L-Build Main Pi Services"; do
    if grep -qF 'compute-submodule-sha: ${{ matrix.submodule_sha }}' ".github/workflows/${wf}.yml"; then
        echo -e "   ${GREEN}✓${NC} ${wf} передаёт submodule_sha в композит"
    else
        echo -e "   ${RED}✗${NC} ${wf} НЕ передаёт submodule_sha в композит"
        exit 1
    fi
done

# 4.5 Single Service композит не использует — считает SHA инлайном
if grep -qF 'ROS2LEDS_SHA=$(git submodule status src/ros2leds' ".github/workflows/L-Build Single Service.yml"; then
    echo -e "   ${GREEN}✓${NC} L-Build Single Service считает ROS2LEDS_SHA инлайном"
else
    echo -e "   ${RED}✗${NC} L-Build Single Service НЕ считает ROS2LEDS_SHA"
    exit 1
fi

if grep -qF 'VESC_NEXUS_SHA=$(git submodule status src/vesc_nexus' ".github/workflows/L-Build Single Service.yml"; then
    echo -e "   ${GREEN}✓${NC} L-Build Single Service считает VESC_NEXUS_SHA инлайном"
else
    echo -e "   ${RED}✗${NC} L-Build Single Service НЕ считает VESC_NEXUS_SHA"
    exit 1
fi

echo
echo -e "${GREEN}=== Все проверки пройдены успешно! ===${NC}"
echo
echo "Механизм инвалидации кэша настроен правильно."
echo "При изменении субмодулей Docker будет пересобирать образы."
