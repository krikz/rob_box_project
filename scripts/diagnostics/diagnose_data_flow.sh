#!/bin/bash
# Финальная диагностика связи Vision Pi (10.1.1.21) <-> Main Pi (10.1.1.20)
# Использует sshpass для автоматической аутентификации

SSH_CMD="sshpass -p 'open' ssh -o StrictHostKeyChecking=no"

echo "========================================="
echo " Camera Data Flow Diagnostic"
echo " Vision Pi (10.1.1.21) -> Main Pi (10.1.1.20)"
echo "========================================="
echo ""

# Полный перезапуск камеры через docker-compose
echo "[1/5] Full restart of camera on Vision Pi (docker-compose)..."
$SSH_CMD ros2@10.1.1.21 'cd ~/rob_box_project/docker/vision && docker-compose down --remove-orphans && docker-compose up -d' > /dev/null 2>&1
echo "  Waiting 30s for initialization..."
sleep 30

echo ""
echo "[2/5] Checking publishers on Vision Pi..."
RGB_PUB=$($SSH_CMD ros2@10.1.1.21 'docker exec oak-d /ros_entrypoint.sh ros2 topic info /camera/camera/color/image_raw/compressed 2>/dev/null | grep "Publisher count" | awk "{print \$3}"')
DEPTH_PUB=$($SSH_CMD ros2@10.1.1.21 'docker exec oak-d /ros_entrypoint.sh ros2 topic info /camera/camera/depth/image_raw/compressed 2>/dev/null | grep "Publisher count" | awk "{print \$3}"')

echo "  RGB publisher count: $RGB_PUB"
echo "  Depth publisher count: $DEPTH_PUB"

echo ""
echo "[3/5] Checking network traffic on Vision Pi..."
RX1=$($SSH_CMD ros2@10.1.1.21 'cat /sys/class/net/eth0/statistics/rx_bytes')
TX1=$($SSH_CMD ros2@10.1.1.21 'cat /sys/class/net/eth0/statistics/tx_bytes')
sleep 3
RX2=$($SSH_CMD ros2@10.1.1.21 'cat /sys/class/net/eth0/statistics/rx_bytes')
TX2=$($SSH_CMD ros2@10.1.1.21 'cat /sys/class/net/eth0/statistics/tx_bytes')
RX_RATE=$(( (RX2 - RX1) / 3 / 1024 ))
TX_RATE=$(( (TX2 - TX1) / 3 / 1024 ))

echo "  eth0 RX: $RX_RATE KB/s"
echo "  eth0 TX: $TX_RATE KB/s"

echo ""
echo "[4/5] Checking topic visibility on Main Pi..."
MAIN_RGB=$($SSH_CMD ros2@10.1.1.20 'docker exec rtabmap bash -c "/opt/ros/humble/bin/ros2 topic list 2>/dev/null | grep -c color/image_raw"' 2>/dev/null)
MAIN_DEPTH=$($SSH_CMD ros2@10.1.1.20 'docker exec rtabmap bash -c "/opt/ros/humble/bin/ros2 topic list 2>/dev/null | grep -c depth/image_raw"' 2>/dev/null)

echo "  Main Pi sees RGB topics: $MAIN_RGB"
echo "  Main Pi sees Depth topics: $MAIN_DEPTH"

echo ""
echo "[5/5] Checking RTAB-Map data reception..."
RTAB_WARNINGS=$($SSH_CMD ros2@10.1.1.20 'docker logs rtabmap --tail 10 2>&1 | grep -c "Did not receive data"')

if [ "$RTAB_WARNINGS" = "0" ]; then
    echo "  ✅ RTAB-Map IS receiving data!"
else
    echo "  ❌ RTAB-Map NOT receiving data (warnings: $RTAB_WARNINGS)"
fi

echo ""
echo "========================================="
echo " Summary"
echo "========================================="

if [ "$RGB_PUB" = "1" ] && [ "$DEPTH_PUB" = "1" ]; then
    echo "✅ Camera publishers: ACTIVE"
else
    echo "❌ Camera publishers: INACTIVE (RGB=$RGB_PUB, Depth=$DEPTH_PUB)"
fi

if [ "$TX_RATE" -gt "100" ]; then
    echo "✅ Network traffic: HIGH ($TX_RATE KB/s - uncompressed data)"
elif [ "$TX_RATE" -gt "20" ]; then
    echo "✅ Network traffic: MODERATE ($TX_RATE KB/s - compressed data)"
else
    echo "⚠️  Network traffic: LOW ($TX_RATE KB/s - may indicate no data flow)"
fi

if [ "$MAIN_RGB" -gt "0" ] && [ "$MAIN_DEPTH" -gt "0" ]; then
    echo "✅ DDS discovery: WORKING (Main Pi sees camera topics)"
else
    echo "❌ DDS discovery: BROKEN (Main Pi doesn't see camera topics)"
fi

if [ "$RTAB_WARNINGS" = "0" ]; then
    echo "✅ Data reception: SUCCESS"
else
    echo "❌ Data reception: FAILED"
fi

echo ""
