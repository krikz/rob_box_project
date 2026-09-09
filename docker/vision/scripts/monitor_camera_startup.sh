#!/bin/bash
# Комплексный мониторинг запуска камеры OAK-D
# Запускает камеру и собирает все метрики за 60 секунд

echo "=== Camera Startup Monitor ==="
echo "Started at: $(date)"
echo ""

# Создаем директорию для логов
LOGDIR="/tmp/camera_monitor_$(date +%Y%m%d_%H%M%S)"
mkdir -p $LOGDIR
echo "Logs will be saved to: $LOGDIR"

# Перезапускаем камеру
echo "Restarting camera..."
docker restart oak-d
sleep 5

echo ""
echo "=== Starting 60-second monitoring session ==="
echo ""

# Фоновый мониторинг CPU
(
  for i in {1..30}; do
    echo "$(date +%H:%M:%S) $(top -bn1 | grep '%Cpu' | awk '{print $2,$4,$8}')" >> $LOGDIR/cpu.log
    sleep 2
  done
) &

# Фоновый мониторинг питания/throttling
(
  for i in {1..30}; do
    echo "$(date +%H:%M:%S) throttled=$(vcgencmd get_throttled | cut -d= -f2) temp=$(vcgencmd measure_temp | cut -d= -f2) volts=$(vcgencmd measure_volts | cut -d= -f2)" >> $LOGDIR/power.log
    sleep 2
  done
) &

# Фоновый мониторинг сети eth0
(
  PREV_RX=$(cat /sys/class/net/eth0/statistics/rx_bytes)
  PREV_TX=$(cat /sys/class/net/eth0/statistics/tx_bytes)
  sleep 2
  for i in {1..29}; do
    CURR_RX=$(cat /sys/class/net/eth0/statistics/rx_bytes)
    CURR_TX=$(cat /sys/class/net/eth0/statistics/tx_bytes)
    RX_RATE=$(( (CURR_RX - PREV_RX) / 2 / 1024 ))  # KB/s
    TX_RATE=$(( (CURR_TX - PREV_TX) / 2 / 1024 ))  # KB/s
    echo "$(date +%H:%M:%S) eth0_rx=${RX_RATE}KB/s eth0_tx=${TX_RATE}KB/s" >> $LOGDIR/network_eth0.log
    PREV_RX=$CURR_RX
    PREV_TX=$CURR_TX
    sleep 2
  done
) &

# Фоновый мониторинг сети wlan0
(
  PREV_RX=$(cat /sys/class/net/wlan0/statistics/rx_bytes 2>/dev/null || echo 0)
  PREV_TX=$(cat /sys/class/net/wlan0/statistics/tx_bytes 2>/dev/null || echo 0)
  sleep 2
  for i in {1..29}; do
    CURR_RX=$(cat /sys/class/net/wlan0/statistics/rx_bytes 2>/dev/null || echo 0)
    CURR_TX=$(cat /sys/class/net/wlan0/statistics/tx_bytes 2>/dev/null || echo 0)
    RX_RATE=$(( (CURR_RX - PREV_RX) / 2 / 1024 ))  # KB/s
    TX_RATE=$(( (CURR_TX - PREV_TX) / 2 / 1024 ))  # KB/s
    echo "$(date +%H:%M:%S) wlan0_rx=${RX_RATE}KB/s wlan0_tx=${TX_RATE}KB/s" >> $LOGDIR/network_wlan0.log
    PREV_RX=$CURR_RX
    PREV_TX=$CURR_TX
    sleep 2
  done
) &

# Мониторинг Docker логов камеры
(
  docker logs -f oak-d 2>&1 | while IFS= read -r line; do
    echo "$(date +%H:%M:%S) $line" >> $LOGDIR/camera.log
  done
) &
DOCKER_PID=$!

# Мониторинг ROS топиков каждые 10 секунд
(
  sleep 20  # Даем камере время запуститься
  for i in {1..4}; do
    echo "=== $(date +%H:%M:%S) ===" >> $LOGDIR/topics.log
    docker exec oak-d /ros_entrypoint.sh ros2 topic info /camera/camera/color/image_raw/compressed >> $LOGDIR/topics.log 2>&1
    docker exec oak-d /ros_entrypoint.sh ros2 topic info /camera/camera/depth/image_rect_raw/compressedDepth >> $LOGDIR/topics.log 2>&1
    sleep 10
  done
) &

# Ждем завершения мониторинга
echo "Monitoring for 60 seconds..."
sleep 60

# Останавливаем фоновые процессы
kill $DOCKER_PID 2>/dev/null
killall -q tail

echo ""
echo "=== Monitoring Complete ==="
echo ""

# Выводим сводку
echo "=== CPU Summary ==="
cat $LOGDIR/cpu.log | tail -10
echo ""

echo "=== Power Summary ==="
cat $LOGDIR/power.log | tail -10
echo ""

echo "=== Network eth0 Summary ==="
cat $LOGDIR/network_eth0.log | tail -10
echo ""

echo "=== Network wlan0 Summary ==="
cat $LOGDIR/network_wlan0.log | tail -10
echo ""

echo "=== Camera Status ==="
docker logs oak-d 2>&1 | grep -E "Camera ready|ERROR|Exception" | tail -5
echo ""

echo "=== Topic Info ==="
cat $LOGDIR/topics.log | tail -20
echo ""

echo "=== Full logs saved to: $LOGDIR ==="
echo "To view:"
echo "  cat $LOGDIR/cpu.log"
echo "  cat $LOGDIR/power.log"
echo "  cat $LOGDIR/network_eth0.log"
echo "  cat $LOGDIR/network_wlan0.log"
echo "  cat $LOGDIR/camera.log"
echo "  cat $LOGDIR/topics.log"
