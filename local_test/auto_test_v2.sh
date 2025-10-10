#!/bin/bash
# ПРАВИЛЬНАЯ конфигурация для 2D LaserScan
# Источник: http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot

TEST_NUM=${1:-1}
TEST_DURATION=40
LOG_DIR="/tmp/rtabmap_test_logs"
mkdir -p "$LOG_DIR"

echo "Удаляем старую БД..."
rm -f /tmp/rtabmap_test.db

echo "Запускаем fake_lidar_publisher..."
python3 local_test/fake_lidar_publisher.py > "$LOG_DIR/test${TEST_NUM}_lidar.log" 2>&1 &
LIDAR_PID=$!
sleep 2

echo "===================================================================================="
echo "TEST $TEST_NUM: subscribe_depth:=false + subscribe_rgbd:=false + subscribe_scan:=true"
echo "===================================================================================="

ros2 launch rtabmap_launch rtabmap.launch.py \
    args:="--delete_db_on_start --udebug" \
    use_sim_time:=false \
    frame_id:=base_link \
    odom_frame_id:=odom \
    subscribe_depth:=false \
    subscribe_rgbd:=false \
    subscribe_scan:=true \
    scan_topic:=/scan \
    qos:=1 \
    approx_sync:=false \
    rtabmap_viz:=false \
    rviz:=false \
    visual_odometry:=false \
    icp_odometry:=true \
    odom_topic:=odom \
    database_path:=/tmp/rtabmap_test.db \
    wait_for_transform:=0.5 \
    > "$LOG_DIR/test${TEST_NUM}_rtabmap.log" 2>&1 &

RTABMAP_PID=$!
echo "Работа в течение $TEST_DURATION секунд..."
sleep "$TEST_DURATION"

echo "Останавливаем процессы..."
kill $RTABMAP_PID $LIDAR_PID 2>/dev/null
sleep 2
killall -9 rtabmap icp_odometry fake_lidar_publisher 2>/dev/null

echo ""
echo "==================== АНАЛИЗ ЛОГОВ ===================="
RTABMAP_LOG="$LOG_DIR/test${TEST_NUM}_rtabmap.log"

echo "--- 1. Параметры subscribe_* ---"
grep -E "subscribe_scan|subscribe_rgb|subscribe_depth|subscribe_rgbd" "$RTABMAP_LOG" | head -15

echo ""
echo "--- 2. Подписки rtabmap ---"
grep "rtabmap subscribed to" "$RTABMAP_LOG" -A 10 | head -20

echo ""
echo "--- 3. icp_odometry ---"
grep "icp_odometry subscribed to" "$RTABMAP_LOG" | head -5

echo ""
echo "Логи: $RTABMAP_LOG"
