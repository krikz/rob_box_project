#!/bin/bash
# Скрипт диагностики подключения к облачному Zenoh роутеру
# Проверяет конфигурацию, подключение и способность получать команды

set -e

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "=========================================="
echo "  🔍 Диагностика Zenoh Cloud подключения"
echo "=========================================="
echo ""

# Проверка 1: Переменная ROBOT_ID
echo -e "${BLUE}📋 Проверка 1: ROBOT_ID${NC}"
if [ -f ".env" ]; then
    ROBOT_ID=$(grep "^ROBOT_ID=" .env | cut -d= -f2)
    if [ -n "$ROBOT_ID" ]; then
        echo -e "${GREEN}✓${NC} ROBOT_ID найден: $ROBOT_ID"
    else
        echo -e "${RED}✗${NC} ROBOT_ID не найден в .env"
        exit 1
    fi
else
    echo -e "${RED}✗${NC} Файл .env не найден"
    exit 1
fi
echo ""

# Проверка 2: Zenoh роутер запущен
echo -e "${BLUE}📋 Проверка 2: Zenoh Router${NC}"
if docker ps | grep -q "zenoh-router"; then
    echo -e "${GREEN}✓${NC} Zenoh Router запущен"
    
    # Проверить логи на подключение к облаку
    if docker logs zenoh-router 2>&1 | grep -q "zenoh.robbox.online"; then
        echo -e "${GREEN}✓${NC} Конфигурация указывает на zenoh.robbox.online"
    else
        echo -e "${YELLOW}⚠${NC} Не найдено упоминание zenoh.robbox.online в логах"
    fi
else
    echo -e "${RED}✗${NC} Zenoh Router не запущен"
    echo "   Запустите: docker-compose up -d zenoh-router"
    exit 1
fi
echo ""

# Проверка 3: twist-mux запущен и подписан
echo -e "${BLUE}📋 Проверка 3: twist-mux${NC}"
if docker ps | grep -q "twist-mux"; then
    echo -e "${GREEN}✓${NC} twist-mux запущен"
    
    # Проверить подписки
    if docker logs twist-mux 2>&1 | grep -q "cmd_vel_voice"; then
        echo -e "${GREEN}✓${NC} twist-mux подписан на cmd_vel_voice"
    else
        echo -e "${RED}✗${NC} twist-mux НЕ подписан на cmd_vel_voice"
    fi
else
    echo -e "${RED}✗${NC} twist-mux не запущен"
fi
echo ""

# Проверка 4: Сетевое подключение к облаку
echo -e "${BLUE}📋 Проверка 4: Сетевое подключение${NC}"
if timeout 5 bash -c "echo >/dev/tcp/zenoh.robbox.online/7447" 2>/dev/null; then
    echo -e "${GREEN}✓${NC} Порт 7447 доступен на zenoh.robbox.online"
else
    echo -e "${RED}✗${NC} Не удалось подключиться к zenoh.robbox.online:7447"
    echo "   Проверьте интернет подключение и firewall"
fi
echo ""

# Проверка 5: REST API облачного роутера
echo -e "${BLUE}📋 Проверка 5: Cloud Router REST API${NC}"
if curl -s --max-time 5 http://zenoh.robbox.online:8000/@/router/status >/dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} REST API облачного роутера доступен"
    
    # Получить режим работы
    MODE=$(curl -s http://zenoh.robbox.online:8000/@/router/config 2>/dev/null | grep -o '"mode":"[^"]*"' | cut -d: -f2 | tr -d '"')
    if [ "$MODE" = "router" ]; then
        echo -e "${GREEN}✓${NC} Облачный роутер в режиме: router"
    else
        echo -e "${RED}✗${NC} Облачный роутер в режиме: ${MODE:-unknown}"
        echo -e "${YELLOW}   ПРОБЛЕМА: Облачный роутер должен быть в режиме 'router'!${NC}"
        echo "   См. docs/cloud/README.md для исправления"
    fi
else
    echo -e "${YELLOW}⚠${NC} REST API облачного роутера недоступен"
    echo "   Проверьте что zenoh.robbox.online:8000 доступен"
fi
echo ""

# Проверка 6: Namespace конфигурация
echo -e "${BLUE}📋 Проверка 6: Zenoh Namespace${NC}"
TWIST_MUX_CONTAINER=$(docker ps -q -f name=twist-mux)
if [ -n "$TWIST_MUX_CONTAINER" ]; then
    # Проверить сгенерированную конфигурацию
    if docker exec "$TWIST_MUX_CONTAINER" test -f /tmp/zenoh_session_config.json5 2>/dev/null; then
        NAMESPACE=$(docker exec "$TWIST_MUX_CONTAINER" grep -o 'namespace: "robots/[^"]*"' /tmp/zenoh_session_config.json5 2>/dev/null | cut -d/ -f2 | tr -d '"')
        if [ "$NAMESPACE" = "$ROBOT_ID" ]; then
            echo -e "${GREEN}✓${NC} Namespace настроен правильно: robots/$ROBOT_ID"
        else
            echo -e "${RED}✗${NC} Namespace неправильный: robots/${NAMESPACE:-not set}"
            echo "   Ожидается: robots/$ROBOT_ID"
        fi
    else
        echo -e "${YELLOW}⚠${NC} Не найден сгенерированный конфиг namespace"
    fi
fi
echo ""

# Проверка 7: Тест публикации (если установлен zenoh CLI)
echo -e "${BLUE}📋 Проверка 7: Тест публикации${NC}"
if command -v z_pub &> /dev/null; then
    echo "Отправка тестового сообщения через zenoh..."
    echo "test" | z_pub -e "tcp/zenoh.robbox.online:7447" -k "robots/$ROBOT_ID/0/test_topic" 2>&1 | head -3
    echo -e "${GREEN}✓${NC} Тест публикации выполнен (проверьте вывод выше)"
else
    echo -e "${YELLOW}⚠${NC} zenoh CLI не установлен (опционально)"
    echo "   Установка: cargo install zenoh --features=unstable"
fi
echo ""

# Итоговая информация
echo "=========================================="
echo "  📊 Итоговая информация"
echo "=========================================="
echo "Robot ID: $ROBOT_ID"
echo "Expected namespace: robots/$ROBOT_ID"
echo "Expected topic key: robots/$ROBOT_ID/0/cmd_vel_voice"
echo ""
echo "Для отправки команды через REST API:"
echo "  curl -X PUT https://zenoh.robbox.online/robots/$ROBOT_ID/0/cmd_vel_voice \\"
echo "    -H \"Content-Type: application/octet-stream\" \\"
echo "    --data-binary @twist_message.bin"
echo ""
echo "Для мониторинга логов twist_mux:"
echo "  docker logs -f twist-mux"
echo ""

# Итоговый статус
echo "=========================================="
if docker ps | grep -q "twist-mux" && docker ps | grep -q "zenoh-router"; then
    echo -e "${GREEN}✓ Базовая конфигурация в порядке${NC}"
    echo ""
    echo "Если команды всё ещё не доходят, проверьте:"
    echo "1. Формат сообщения (должен быть CDR binary, не JSON)"
    echo "2. Режим облачного роутера (должен быть 'router')"
    echo "3. Логи на облачном сервере"
else
    echo -e "${RED}✗ Обнаружены проблемы${NC}"
    echo ""
    echo "Запустите все сервисы:"
    echo "  docker-compose up -d"
fi
echo "=========================================="
