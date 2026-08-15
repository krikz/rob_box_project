#!/bin/bash
# Скрипт проверки конфигурации Zenoh роутеров
# Использование: ./scripts/validate_zenoh_config.sh

set -e

echo "=========================================="
echo "  Проверка конфигурации Zenoh роутеров"
echo "=========================================="
echo ""

VISION_CONFIG="docker/vision/config/zenoh_router_config.json5"
MAIN_CONFIG="docker/main/config/zenoh_router_config.json5"

# FA-4: конфиги параметризованы через ${ZENOH_*}. Для проверки подставляем
# значения по умолчанию (как это делают start_zenoh_router.sh на рантайме),
# чтобы проверки ниже оставались осмысленными.
VISION_CONFIG_TMP=$(mktemp)
MAIN_CONFIG_TMP=$(mktemp)
trap 'rm -f "$VISION_CONFIG_TMP" "$MAIN_CONFIG_TMP"' EXIT
sed -e 's|\${ZENOH_MAIN_PI_IP}|10.1.1.10|g' \
    -e 's|\${ZENOH_VISION_PI_IP}|10.1.1.11|g' \
    "$VISION_CONFIG" > "$VISION_CONFIG_TMP"
sed -e 's|\${ZENOH_MAIN_PI_IP}|10.1.1.10|g' \
    -e 's|\${ZENOH_CLOUD_ROUTER}|tcp/zenoh.robbox.online:7447|g' \
    "$MAIN_CONFIG" > "$MAIN_CONFIG_TMP"
VISION_CONFIG="$VISION_CONFIG_TMP"
MAIN_CONFIG="$MAIN_CONFIG_TMP"

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Счётчики
ERRORS=0
WARNINGS=0
SUCCESS=0

echo "Проверка Vision Pi конфигурации..."
echo "-----------------------------------"

# Проверка 1: Vision Pi НЕ должен слушать на eth0:7447
if grep -q "tcp/\[::\]:7447#iface=eth0" "$VISION_CONFIG" 2>/dev/null; then
    echo -e "${RED}❌ ОШИБКА: Vision Pi роутер слушает на tcp/[::]:7447#iface=eth0${NC}"
    echo "   Это конфликтует с Main Pi роутером!"
    echo "   Удалите эту строку из listen.endpoints в $VISION_CONFIG"
    ERRORS=$((ERRORS + 1))
else
    echo -e "${GREEN}✅ OK: Vision Pi роутер НЕ слушает на eth0:7447${NC}"
    SUCCESS=$((SUCCESS + 1))
fi

# Проверка 2: Vision Pi должен слушать на своём IP eth0
if grep -q '"tcp/10.1.1.11:7447"' "$VISION_CONFIG"; then
    echo -e "${GREEN}✅ OK: Vision Pi роутер слушает на 10.1.1.11:7447${NC}"
    SUCCESS=$((SUCCESS + 1))
else
    echo -e "${RED}❌ ОШИБКА: Vision Pi роутер не слушает на 10.1.1.11:7447${NC}"
    echo "   Добавьте 'tcp/10.1.1.11:7447' в listen.endpoints"
    ERRORS=$((ERRORS + 1))
fi

# Проверка 3: Vision Pi должен подключаться к Main Pi через eth0
if grep -q '"tcp/10.1.1.10:7447#iface=eth0"' "$VISION_CONFIG"; then
    echo -e "${GREEN}✅ OK: Vision Pi подключается к Main Pi через eth0${NC}"
    SUCCESS=$((SUCCESS + 1))
else
    echo -e "${YELLOW}⚠️  ПРЕДУПРЕЖДЕНИЕ: Vision Pi может не подключаться к Main Pi через eth0${NC}"
    echo "   Проверьте connect.endpoints в $VISION_CONFIG"
    WARNINGS=$((WARNINGS + 1))
fi

echo ""
echo "Проверка Main Pi конфигурации..."
echo "-----------------------------------"

# Проверка 4: Main Pi должен слушать на конкретном IP 10.1.1.10:7447
if grep -q '"tcp/10.1.1.10:7447"' "$MAIN_CONFIG"; then
    echo -e "${GREEN}✅ OK: Main Pi роутер слушает на 10.1.1.10:7447${NC}"
    SUCCESS=$((SUCCESS + 1))
else
    echo -e "${RED}❌ ОШИБКА: Main Pi роутер не слушает на 10.1.1.10:7447${NC}"
    echo "   Vision Pi не сможет подключиться!"
    echo "   Добавьте 'tcp/10.1.1.10:7447' в listen.endpoints"
    ERRORS=$((ERRORS + 1))
fi

# Проверка 5: Main Pi НЕ должен использовать localhost (весь трафик через eth0)
if grep -q '"tcp/127.0.0.1:7447"' "$MAIN_CONFIG" || grep -q '"tcp/localhost:7447"' "$MAIN_CONFIG"; then
    echo -e "${YELLOW}⚠️  ПРЕДУПРЕЖДЕНИЕ: Main Pi использует localhost${NC}"
    echo "   Рекомендуется использовать только IP eth0 для всего трафика"
    WARNINGS=$((WARNINGS + 1))
else
    echo -e "${GREEN}✅ OK: Main Pi использует только IP eth0 (весь трафик через Gigabit Ethernet)${NC}"
    SUCCESS=$((SUCCESS + 1))
fi

# Проверка 6: Main Pi НЕ должен использовать [::]:7447 в listen endpoints
MAIN_LISTEN_ENDPOINTS=$(awk '/listen:/{flag=1} flag && /endpoints: \[/{getline; while(getline && !/\]/){if(!/^[[:space:]]*\/\//){print}}}' "$MAIN_CONFIG")
if echo "$MAIN_LISTEN_ENDPOINTS" | grep -q '"tcp/\[::\]:7447'; then
    echo -e "${RED}❌ ОШИБКА: Main Pi использует [::]:7447 в listen endpoints!${NC}"
    echo "   Замените на конкретный IP: tcp/10.1.1.10:7447"
    ERRORS=$((ERRORS + 1))
else
    echo -e "${GREEN}✅ OK: Main Pi не использует проблемный [::]:7447${NC}"
    SUCCESS=$((SUCCESS + 1))
fi

echo ""
echo "=========================================="
echo "  Результаты проверки"
echo "=========================================="
echo -e "${GREEN}✅ Успешно: $SUCCESS${NC}"
echo -e "${YELLOW}⚠️  Предупреждения: $WARNINGS${NC}"
echo -e "${RED}❌ Ошибки: $ERRORS${NC}"
echo ""

if [ $ERRORS -gt 0 ]; then
    echo -e "${RED}ПРОВЕРКА НЕ ПРОЙДЕНА!${NC}"
    echo "Исправьте ошибки перед развертыванием."
    exit 1
elif [ $WARNINGS -gt 0 ]; then
    echo -e "${YELLOW}ПРОВЕРКА ПРОЙДЕНА С ПРЕДУПРЕЖДЕНИЯМИ${NC}"
    echo "Рекомендуется устранить предупреждения."
    exit 0
else
    echo -e "${GREEN}ВСЕ ПРОВЕРКИ ПРОЙДЕНЫ!${NC}"
    echo "Конфигурация корректна."
    exit 0
fi
