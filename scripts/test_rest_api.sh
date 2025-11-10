#!/bin/bash
# Скрипт для тестирования REST API → ROS коммуникации
# Включает детальное логирование Zenoh и ROS

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo "=================================================="
echo "  🧪 Тестирование REST API → ROS"
echo "=================================================="
echo ""

# Проверка 1: Получить список топиков
echo -e "${BLUE}📋 Шаг 1: Получение списка топиков через REST API${NC}"
echo "URL: http://zenoh.robbox.online:8000/robots/RBXU100001/**"
echo ""

TOPICS=$(curl -s "http://zenoh.robbox.online:8000/robots/RBXU100001/**" 2>/dev/null | head -20)
if [ -n "$TOPICS" ]; then
    echo -e "${GREEN}✓${NC} Топики получены:"
    echo "$TOPICS" | grep "cmd_vel"
    echo ""
else
    echo -e "${RED}✗${NC} Не удалось получить топики"
    exit 1
fi

# Проверка 2: Найти полный ключ cmd_vel_voice
echo -e "${BLUE}📋 Шаг 2: Поиск полного ключа cmd_vel_voice${NC}"
FULL_KEY=$(curl -s "http://zenoh.robbox.online:8000/robots/RBXU100001/**" 2>/dev/null | grep "cmd_vel_voice" | head -1)
if [ -n "$FULL_KEY" ]; then
    echo -e "${GREEN}✓${NC} Найден ключ:"
    echo "$FULL_KEY"
    echo ""
else
    echo -e "${YELLOW}⚠${NC} Ключ cmd_vel_voice не найден"
    echo "Проверяем все ключи с 'cmd_vel':"
    curl -s "http://zenoh.robbox.online:8000/robots/RBXU100001/**" 2>/dev/null | grep "cmd_vel" | head -5
    echo ""
fi

# Проверка 3: Создать тестовый CDR файл
echo -e "${BLUE}📋 Шаг 3: Создание тестового CDR файла${NC}"
cat > /tmp/test_twist_cdr.py << 'PYTHON_EOF'
#!/usr/bin/env python3
"""Создание тестового Twist CDR файла"""
import struct

# Twist: 6 x float64 (little endian)
# linear.x = 0.1, остальные = 0.0
twist_data = struct.pack('<dddddd', 
    0.1,  # linear.x
    0.0,  # linear.y
    0.0,  # linear.z
    0.0,  # angular.x
    0.0,  # angular.y
    0.0   # angular.z
)

with open('/tmp/twist.cdr', 'wb') as f:
    f.write(twist_data)

print(f"CDR файл создан: {len(twist_data)} байт")
print(f"Содержимое (hex): {twist_data.hex()}")
PYTHON_EOF

python3 /tmp/test_twist_cdr.py
echo ""

# Проверка 4: Отправить на упрощённый топик
echo -e "${BLUE}📋 Шаг 4: Тест отправки на упрощённый топик${NC}"
echo "Попытка 1: robots/RBXU100001/cmd_vel_voice"
RESPONSE=$(curl -s -w "\nHTTP_CODE:%{http_code}" -X PUT \
  "http://zenoh.robbox.online/robots/RBXU100001/cmd_vel_voice" \
  -H "Content-Type: application/octet-stream" \
  --data-binary @/tmp/twist.cdr 2>&1)

HTTP_CODE=$(echo "$RESPONSE" | grep "HTTP_CODE:" | cut -d: -f2)
if [ "$HTTP_CODE" = "200" ]; then
    echo -e "${GREEN}✓${NC} HTTP 200 OK - сообщение принято"
else
    echo -e "${YELLOW}⚠${NC} HTTP $HTTP_CODE"
fi
echo ""

# Проверка 5: Отправить на топик с domain
echo -e "${BLUE}📋 Шаг 5: Тест отправки с domain${NC}"
echo "Попытка 2: robots/RBXU100001/0/cmd_vel_voice"
RESPONSE=$(curl -s -w "\nHTTP_CODE:%{http_code}" -X PUT \
  "http://zenoh.robbox.online/robots/RBXU100001/0/cmd_vel_voice" \
  -H "Content-Type: application/octet-stream" \
  --data-binary @/tmp/twist.cdr 2>&1)

HTTP_CODE=$(echo "$RESPONSE" | grep "HTTP_CODE:" | cut -d: -f2)
if [ "$HTTP_CODE" = "200" ]; then
    echo -e "${GREEN}✓${NC} HTTP 200 OK - сообщение принято"
else
    echo -e "${YELLOW}⚠${NC} HTTP $HTTP_CODE"
fi
echo ""

# Проверка 6: Если есть полный ключ, попробовать отправить на него
if [ -n "$FULL_KEY" ]; then
    echo -e "${BLUE}📋 Шаг 6: Тест отправки на полный ключ${NC}"
    # Извлечь ключ из вывода
    CLEAN_KEY=$(echo "$FULL_KEY" | sed 's/^[[:space:]]*//' | cut -d' ' -f1)
    echo "Попытка 3: $CLEAN_KEY"
    
    RESPONSE=$(curl -s -w "\nHTTP_CODE:%{http_code}" -X PUT \
      "http://zenoh.robbox.online/$CLEAN_KEY" \
      -H "Content-Type: application/octet-stream" \
      --data-binary @/tmp/twist.cdr 2>&1)
    
    HTTP_CODE=$(echo "$RESPONSE" | grep "HTTP_CODE:" | cut -d: -f2)
    if [ "$HTTP_CODE" = "200" ]; then
        echo -e "${GREEN}✓${NC} HTTP 200 OK - сообщение принято"
    else
        echo -e "${YELLOW}⚠${NC} HTTP $HTTP_CODE"
    fi
    echo ""
fi

# Проверка 7: Инструкции для проверки на роботе
echo "=================================================="
echo -e "${BLUE}📋 Проверка на роботе${NC}"
echo "=================================================="
echo ""
echo "Выполните на роботе (ros2@10.1.1.20):"
echo ""
echo "1. Включить debug логи Zenoh:"
echo "   docker exec twist-mux bash -c 'export RUST_LOG=zenoh=debug && ros2 topic echo /cmd_vel_voice'"
echo ""
echo "2. Мониторить логи twist_mux:"
echo "   docker logs -f twist-mux"
echo ""
echo "3. Проверить получение на ROS топике:"
echo "   docker exec twist-mux ros2 topic echo /cmd_vel_voice"
echo ""
echo "4. Включить Zenoh bridge debug (если установлен):"
echo "   docker exec zenoh-router bash -c 'export RUST_LOG=zenoh=debug,zenoh_bridge=debug'"
echo ""
echo "5. Проверить подписки Zenoh:"
echo "   curl http://10.1.1.10:8000/@/local/subscriber | jq"
echo ""

echo "=================================================="
echo "  📊 Итоги тестирования"
echo "=================================================="
echo ""
echo "Проверьте логи на роботе, чтобы понять:"
echo "- Приходят ли сообщения в Zenoh роутер на роботе"
echo "- Транслируются ли они в ROS топик"
echo "- На какой именно ключ нужно публиковать"
echo ""
echo "Сохраните вывод и поделитесь для анализа!"
echo "=================================================="
