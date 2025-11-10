#!/bin/bash
# Диагностика проблемы "Address in use" для Zenoh роутера
# Использование: ./scripts/diagnose_zenoh_port_conflict.sh

set -e

echo "=========================================="
echo "  Диагностика конфликта портов Zenoh"
echo "=========================================="
echo ""

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}1. Проверка запущенных Zenoh роутеров...${NC}"
echo "-----------------------------------"

ZENOH_CONTAINERS=$(docker ps -a --filter "name=zenoh-router" --format "{{.Names}}\t{{.Status}}\t{{.ID}}")

if [ -z "$ZENOH_CONTAINERS" ]; then
    echo -e "${GREEN}✅ Контейнеры zenoh-router не найдены${NC}"
else
    echo "$ZENOH_CONTAINERS"
    echo ""
    
    # Подсчет запущенных контейнеров
    RUNNING_COUNT=$(docker ps --filter "name=zenoh-router" --format "{{.Names}}" | wc -l)
    
    if [ "$RUNNING_COUNT" -gt 1 ]; then
        echo -e "${RED}❌ ПРОБЛЕМА: Найдено $RUNNING_COUNT запущенных zenoh-router контейнеров!${NC}"
        echo "   Должен быть только ОДИН активный контейнер."
        echo ""
    elif [ "$RUNNING_COUNT" -eq 1 ]; then
        echo -e "${GREEN}✅ Найден 1 запущенный zenoh-router контейнер${NC}"
    fi
fi

echo ""
echo -e "${BLUE}2. Проверка процессов на порту 7447...${NC}"
echo "-----------------------------------"

# Проверка кто слушает порт 7447
if command -v netstat &> /dev/null; then
    PORT_CHECK=$(netstat -tlnp 2>/dev/null | grep :7447 || echo "")
    if [ -z "$PORT_CHECK" ]; then
        echo -e "${GREEN}✅ Порт 7447 свободен${NC}"
    else
        echo -e "${YELLOW}⚠️  Порт 7447 занят:${NC}"
        echo "$PORT_CHECK"
    fi
elif command -v ss &> /dev/null; then
    PORT_CHECK=$(ss -tlnp 2>/dev/null | grep :7447 || echo "")
    if [ -z "$PORT_CHECK" ]; then
        echo -e "${GREEN}✅ Порт 7447 свободен${NC}"
    else
        echo -e "${YELLOW}⚠️  Порт 7447 занят:${NC}"
        echo "$PORT_CHECK"
    fi
else
    echo -e "${YELLOW}⚠️  netstat/ss не доступны, проверка пропущена${NC}"
fi

echo ""
echo -e "${BLUE}3. Проверка логов zenoh-router...${NC}"
echo "-----------------------------------"

RUNNING_ROUTERS=$(docker ps --filter "name=zenoh-router" --format "{{.Names}}")

if [ -n "$RUNNING_ROUTERS" ]; then
    for container in $RUNNING_ROUTERS; do
        echo ""
        echo "Логи контейнера: $container"
        echo "---"
        
        # Проверка на ошибку "Address in use"
        if docker logs "$container" 2>&1 | grep -q "Address in use"; then
            echo -e "${RED}❌ НАЙДЕНА ОШИБКА: Address in use${NC}"
            docker logs "$container" 2>&1 | grep -A 2 "Address in use" | tail -5
        else
            echo -e "${GREEN}✅ Ошибок 'Address in use' не найдено${NC}"
            docker logs "$container" 2>&1 | tail -3
        fi
    done
else
    echo -e "${YELLOW}⚠️  Нет запущенных zenoh-router контейнеров${NC}"
fi

echo ""
echo "=========================================="
echo -e "${BLUE}  РЕКОМЕНДАЦИИ ПО УСТРАНЕНИЮ${NC}"
echo "=========================================="
echo ""

if [ "$RUNNING_COUNT" -gt 1 ]; then
    echo -e "${RED}ПРОБЛЕМА: Несколько экземпляров zenoh-router${NC}"
    echo ""
    echo "Решение:"
    echo "  1. Остановить ВСЕ контейнеры zenoh-router:"
    echo "     docker stop \$(docker ps -aq --filter 'name=zenoh-router')"
    echo ""
    echo "  2. Удалить старые контейнеры:"
    echo "     docker rm \$(docker ps -aq --filter 'name=zenoh-router')"
    echo ""
    echo "  3. Перезапустить через docker-compose:"
    echo "     cd ~/rob_box_project/docker/main"
    echo "     docker-compose up -d zenoh-router"
    echo ""
elif docker logs zenoh-router 2>&1 | grep -q "Address in use" 2>/dev/null; then
    echo -e "${RED}ПРОБЛЕМА: Порт 7447 уже занят${NC}"
    echo ""
    echo "Возможные причины:"
    echo "  1. Старый процесс zenoh все еще работает"
    echo "  2. Другой сервис использует порт 7447"
    echo "  3. Предыдущий контейнер не освободил порт"
    echo ""
    echo "Решение:"
    echo "  1. Остановить контейнер:"
    echo "     docker-compose -f ~/rob_box_project/docker/main/docker-compose.yaml down zenoh-router"
    echo ""
    echo "  2. Подождать 5 секунд для освобождения порта"
    echo ""
    echo "  3. Запустить заново:"
    echo "     docker-compose -f ~/rob_box_project/docker/main/docker-compose.yaml up -d zenoh-router"
    echo ""
    echo "  4. Если проблема сохраняется, проверить кто использует порт:"
    echo "     sudo netstat -tlnp | grep 7447"
    echo "     sudo lsof -i :7447"
    echo ""
else
    echo -e "${GREEN}✅ Явных проблем не обнаружено${NC}"
    echo ""
    echo "Если контейнер не запускается, попробуйте:"
    echo "  1. Пересоздать контейнер:"
    echo "     cd ~/rob_box_project/docker/main"
    echo "     docker-compose down zenoh-router"
    echo "     docker-compose up -d zenoh-router"
    echo ""
    echo "  2. Проверить логи:"
    echo "     docker logs zenoh-router"
    echo ""
fi

echo "=========================================="
echo ""
