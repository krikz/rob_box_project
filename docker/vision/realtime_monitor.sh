#!/bin/bash
# Real-time camera monitoring - показывает CPU, сеть, publishers каждые 2 секунды
# Использование: ./realtime_monitor.sh [duration_in_seconds]

DURATION=${1:-30}  # По умолчанию 30 секунд
INTERVAL=2

echo "========================================="
echo "  Real-time Camera Monitor"
echo "  Duration: ${DURATION}s, Interval: ${INTERVAL}s"
echo "========================================="
echo ""

# Перезапускаем камеру
echo "[$(date +%H:%M:%S)] Restarting camera..."
docker restart oak-d > /dev/null 2>&1
echo "[$(date +%H:%M:%S)] Waiting 20s for camera initialization..."
sleep 20

echo ""
echo "Time      | CPU (usr/sys/idle) | eth0 RX    | eth0 TX    | RGB Pub | Depth Pub | Temp  | Throttle"
echo "----------|--------------------|-----------|-----------|---------|-----------| ------|---------"

ITERATIONS=$((DURATION / INTERVAL))

for i in $(seq 1 $ITERATIONS); do
    TIMESTAMP=$(date +%H:%M:%S)
    
    # CPU
    CPU_LINE=$(top -bn1 | grep '%Cpu')
    CPU_USR=$(echo $CPU_LINE | awk '{print $2}')
    CPU_SYS=$(echo $CPU_LINE | awk '{print $4}')
    CPU_IDLE=$(echo $CPU_LINE | awk '{print $8}')
    
    # Сеть
    RX1=$(cat /sys/class/net/eth0/statistics/rx_bytes)
    TX1=$(cat /sys/class/net/eth0/statistics/tx_bytes)
    sleep $INTERVAL
    RX2=$(cat /sys/class/net/eth0/statistics/rx_bytes)
    TX2=$(cat /sys/class/net/eth0/statistics/tx_bytes)
    RX_RATE=$(( (RX2 - RX1) / INTERVAL / 1024 ))
    TX_RATE=$(( (TX2 - TX1) / INTERVAL / 1024 ))
    
    # Publishers
    RGB_PUB=$(docker exec oak-d /ros_entrypoint.sh ros2 topic info /camera/camera/color/image_raw/compressed 2>/dev/null | grep "Publisher count" | awk '{print $3}')
    DEPTH_PUB=$(docker exec oak-d /ros_entrypoint.sh ros2 topic info /camera/camera/depth/image_raw/compressed 2>/dev/null | grep "Publisher count" | awk '{print $3}')
    
    # Температура и throttling
    TEMP=$(vcgencmd measure_temp | cut -d= -f2)
    THROTTLE=$(vcgencmd get_throttled | cut -d= -f2)
    
    # Форматируем вывод
    printf "%s | %4s/%4s/%5s | %7s KB/s | %7s KB/s | %7s | %9s | %5s | %s\n" \
        "$TIMESTAMP" "$CPU_USR" "$CPU_SYS" "$CPU_IDLE" "$RX_RATE" "$TX_RATE" \
        "${RGB_PUB:-?}" "${DEPTH_PUB:-?}" "$TEMP" "$THROTTLE"
done

echo ""
echo "========================================="
echo "  Monitoring Complete"
echo "========================================="
echo ""

# Финальная проверка на Main Pi
echo "Checking RTAB-Map on Main Pi (10.1.1.20)..."
MAIN_STATUS=$(ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 ros2@10.1.1.20 "docker logs rtabmap --tail 5 2>&1 | grep -c 'Did not receive data'" 2>/dev/null || echo "N/A")

if [ "$MAIN_STATUS" = "0" ]; then
    echo "✅ RTAB-Map is RECEIVING data!"
elif [ "$MAIN_STATUS" = "N/A" ]; then
    echo "⚠️  Cannot connect to Main Pi"
else
    echo "❌ RTAB-Map is NOT receiving data (warnings count: $MAIN_STATUS)"
fi

echo ""
