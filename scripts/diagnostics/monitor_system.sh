#!/bin/bash
# Скрипт мониторинга системы робота
# Использование: ./monitor_system.sh

echo "=================================="
echo "  МОНИТОРИНГ СИСТЕМЫ РОБОТА"
echo "=================================="
echo ""

# Определяем какая это Pi по наличию контейнеров
if docker ps | grep -q "oak-d"; then
    PI_TYPE="Pi #1 (Камера OAK-D)"
    MAIN_CONTAINER="oak-d"
elif docker ps | grep -q "rtabmap"; then
    PI_TYPE="Pi #2 (RTAB-Map)"
    MAIN_CONTAINER="rtabmap"
else
    PI_TYPE="Неизвестно"
    MAIN_CONTAINER=""
fi

echo "Система: $PI_TYPE"
echo ""

# CPU
echo "--- CPU ИСПОЛЬЗОВАНИЕ ---"
CPU_USAGE=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)
echo "Общая нагрузка CPU: ${CPU_USAGE}%"

if [ ! -z "$MAIN_CONTAINER" ]; then
    CONTAINER_CPU=$(docker stats --no-stream --format "{{.CPUPerc}}" $MAIN_CONTAINER)
    echo "Контейнер $MAIN_CONTAINER: $CONTAINER_CPU"
fi

# Память
echo ""
echo "--- ПАМЯТЬ ---"
free -h | grep "Mem:" | awk '{print "Использовано: " $3 " / " $2 " (" int($3/$2*100) "%)"}'

if [ ! -z "$MAIN_CONTAINER" ]; then
    CONTAINER_MEM=$(docker stats --no-stream --format "{{.MemUsage}}" $MAIN_CONTAINER)
    echo "Контейнер $MAIN_CONTAINER: $CONTAINER_MEM"
fi

# Температура
echo ""
echo "--- ТЕМПЕРАТУРА ---"
TEMP=$(vcgencmd measure_temp | cut -d'=' -f2)
echo "CPU температура: $TEMP"

# Проверка перегрева
TEMP_NUM=$(echo $TEMP | cut -d"'" -f1)
if (( $(echo "$TEMP_NUM > 75" | bc -l) )); then
    echo "⚠️  ВНИМАНИЕ: Высокая температура!"
elif (( $(echo "$TEMP_NUM > 70" | bc -l) )); then
    echo "⚠️  Температура повышена"
fi

# Место на диске
echo ""
echo "--- ДИСК ---"
df -h / | tail -1 | awk '{print "Использовано: " $3 " / " $2 " (" $5 ")"}'

# ROS топики (если ROS доступен)
echo ""
echo "--- ROS 2 ТОПИКИ ---"

if command -v ros2 &> /dev/null; then
    # Проверяем наличие топиков OAK-D
    if ros2 topic list 2>/dev/null | grep -q "oak"; then
        echo "✓ Топики OAK-D найдены"
        
        # FPS RGB
        RGB_HZ=$(timeout 3 ros2 topic hz /oak/rgb/image_raw/compressed 2>/dev/null | grep "average rate" | awk '{print $3}')
        if [ ! -z "$RGB_HZ" ]; then
            echo "  RGB FPS: $RGB_HZ Hz"
        fi
        
        # FPS Depth
        DEPTH_HZ=$(timeout 3 ros2 topic hz /oak/stereo/image_raw/compressedDepth 2>/dev/null | grep "average rate" | awk '{print $3}')
        if [ ! -z "$DEPTH_HZ" ]; then
            echo "  Depth FPS: $DEPTH_HZ Hz"
        fi
        
        # Bandwidth
        RGB_BW=$(timeout 3 ros2 topic bw /oak/rgb/image_raw/compressed 2>/dev/null | grep "average" | awk '{print $2, $3}')
        if [ ! -z "$RGB_BW" ]; then
            echo "  RGB Bandwidth: $RGB_BW"
        fi
    else
        echo "✗ Топики OAK-D не найдены"
    fi
    
    # Проверяем RTAB-Map
    if ros2 topic list 2>/dev/null | grep -q "rtabmap"; then
        echo "✓ Топики RTAB-Map найдены"
    fi
else
    echo "ROS 2 недоступен в текущем окружении"
    echo "Запустите: source /opt/ros/humble/setup.bash"
fi

# Docker контейнеры
echo ""
echo "--- DOCKER КОНТЕЙНЕРЫ ---"
docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" | grep -E "oak-d|rtabmap|apriltag"

# Сетевая статистика (если iftop установлен)
echo ""
echo "--- СЕТЬ ---"
if command -v iftop &> /dev/null; then
    echo "Для детального мониторинга сети запустите: sudo iftop -i eth0"
else
    # Показываем текущую статистику интерфейса
    RX_BYTES=$(cat /sys/class/net/eth0/statistics/rx_bytes 2>/dev/null || echo "0")
    TX_BYTES=$(cat /sys/class/net/eth0/statistics/tx_bytes 2>/dev/null || echo "0")
    echo "Статистика eth0: RX=$(numfmt --to=iec $RX_BYTES), TX=$(numfmt --to=iec $TX_BYTES)"
fi

echo ""
echo "=================================="
echo "Обновлено: $(date '+%Y-%m-%d %H:%M:%S')"
echo "=================================="
