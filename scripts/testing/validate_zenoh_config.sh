#!/bin/bash
# Проверка конфигурации Zenoh (Lyrical: rmw_zenohd + ZENOH_CONFIG_OVERRIDE).
# Конфиг задаётся прямо в docker-compose.yaml через ZENOH_CONFIG_OVERRIDE,
# отдельных zenoh_*_config.json5 больше нет.
# Использование: ./scripts/testing/validate_zenoh_config.sh

set -e

MAIN_COMPOSE="docker/main/docker-compose.yaml"
VISION_COMPOSE="docker/vision/docker-compose.yaml"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
ERRORS=0; WARNINGS=0; SUCCESS=0

check() { # $1=label $2=literal-pattern $3=file $4=mode(required|warn)
    if grep -qF "$2" "$3" 2>/dev/null; then
        echo -e "${GREEN}✅ OK: $1${NC}"; SUCCESS=$((SUCCESS+1))
    elif [ "$4" = "warn" ]; then
        echo -e "${YELLOW}⚠️  ПРЕДУПРЕЖДЕНИЕ: $1${NC}"; WARNINGS=$((WARNINGS+1))
    else
        echo -e "${RED}❌ ОШИБКА: $1${NC}"; ERRORS=$((ERRORS+1))
    fi
}

echo "=========================================="
echo "  Проверка Zenoh (rmw_zenohd + ZENOH_CONFIG_OVERRIDE)"
echo "=========================================="
echo ""

check "Vision Pi подключается к Main Pi через eth0" 'tcp/10.1.1.10:7447#iface=eth0' "$VISION_COMPOSE" required
check "Vision Pi слушает на 10.1.1.11:7447" 'tcp/10.1.1.11:7447' "$VISION_COMPOSE" required
check "Main Pi слушает на 10.1.1.10:7447" 'tcp/10.1.1.10:7447' "$MAIN_COMPOSE" required
check "Main Pi подключается к облаку" 'zenoh.robbox.online:7447' "$MAIN_COMPOSE" warn
check "Роутер запускается через rmw_zenohd (healthcheck)" 'pgrep -f rmw_zenohd' "$MAIN_COMPOSE" required
check "Namespace override присутствует (Main)" 'ZENOH_CONFIG_OVERRIDE=namespace="robots/${ROBOT_ID}"' "$MAIN_COMPOSE" required
check "Namespace override присутствует (Vision)" 'ZENOH_CONFIG_OVERRIDE=namespace="robots/${ROBOT_ID}"' "$VISION_COMPOSE" required
check "QoS drop для rtabmap (защита от transport-closure)" 'rt/rtabmap/**' "$MAIN_COMPOSE" required

# Не должно быть IPv6-only listen (known issue: падение роутера на IPv4-only системах)
if grep -qF '"tcp/[::]' "$MAIN_COMPOSE" "$VISION_COMPOSE"; then
    echo -e "${RED}❌ ОШИБКА: найден IPv6-only listen tcp/[::]${NC}"; ERRORS=$((ERRORS+1))
else
    echo -e "${GREEN}✅ OK: нет IPv6-only listen${NC}"; SUCCESS=$((SUCCESS+1))
fi

# DDS-наследие должно быть вычищено
if grep -qE 'ROS_DOMAIN_ID|ROS_AUTOMATIC_DISCOVERY_RANGE|ZENOH_SESSION_CONFIG_URI|eclipse/zenoh' "$MAIN_COMPOSE" "$VISION_COMPOSE"; then
    echo -e "${YELLOW}⚠️  ПРЕДУПРЕЖДЕНИЕ: найдено DDS/старое наследие в compose${NC}"; WARNINGS=$((WARNINGS+1))
else
    echo -e "${GREEN}✅ OK: DDS-наследие вычищено${NC}"; SUCCESS=$((SUCCESS+1))
fi

echo ""
echo -e "${GREEN}✅ Успешно: $SUCCESS${NC}  ${YELLOW}⚠️ Предупреждения: $WARNINGS${NC}  ${RED}❌ Ошибки: $ERRORS${NC}"
[ $ERRORS -gt 0 ] && exit 1 || exit 0
