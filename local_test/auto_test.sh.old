#!/bin/bash
# Автоматический тест rtabmap - запуск на 40 секунд с логами

TEST_NUM=${1:-1}
LOG_DIR="/tmp/rtabmap_test_logs"
mkdir -p "$LOG_DIR"

echo "========================================="
echo " RTAB-Map Auto Test #$TEST_NUM"
echo "========================================="
echo ""
echo "Логи сохраняются в: $LOG_DIR"
echo ""

# Настройка ROS 2
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG="$(pwd)/local_test/zenoh_local_config.json5"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

echo "✅ Zenoh configured"

# TF: base_link -> laser
ros2 run tf2_ros static_transform_publisher 0 0 0.2 0 0 0 base_link laser > "$LOG_DIR/test${TEST_NUM}_tf.log" 2>&1 &
TF_PID=$!
sleep 1

# Fake lidar
python3 local_test/fake_lidar_publisher.py > "$LOG_DIR/test${TEST_NUM}_lidar.log" 2>&1 &
LIDAR_PID=$!
sleep 2

echo "🚀 Запуск rtabmap.launch.py..."
echo ""

# Запуск с разными параметрами в зависимости от TEST_NUM
case $TEST_NUM in
    1)
        echo "TEST 1: Базовая конфигурация (как на Pi)"
        ros2 launch rtabmap_launch rtabmap.launch.py \
            args:="--delete_db_on_start --udebug" \
            use_sim_time:=false \
            frame_id:=base_link \
            odom_frame_id:=odom \
            rgb_topic:=/null/rgb \
            depth_topic:=/null/depth \
            camera_info_topic:=/null/camera_info \
            scan_topic:=/scan \
            qos:=0 \
            approx_sync:=false \
            rtabmap_viz:=false \
            rviz:=false \
            visual_odometry:=false \
            icp_odometry:=true \
            odom_topic:=odom \
            database_path:=/tmp/rtabmap_test.db \
            wait_for_transform:=0.5 \
            > "$LOG_DIR/test${TEST_NUM}_rtabmap.log" 2>&1 &
        ;;
    2)
        echo "TEST 2: С subscribe_scan явно"
        ros2 launch rtabmap_launch rtabmap.launch.py \
            args:="--delete_db_on_start --udebug" \
            use_sim_time:=false \
            frame_id:=base_link \
            odom_frame_id:=odom \
            subscribe_scan:=true \
            subscribe_rgb:=false \
            subscribe_depth:=false \
            rgb_topic:=/null/rgb \
            depth_topic:=/null/depth \
            camera_info_topic:=/null/camera_info \
            scan_topic:=/scan \
            qos:=0 \
            approx_sync:=false \
            rtabmap_viz:=false \
            rviz:=false \
            visual_odometry:=false \
            icp_odometry:=true \
            odom_topic:=odom \
            database_path:=/tmp/rtabmap_test.db \
            wait_for_transform:=0.5 \
            > "$LOG_DIR/test${TEST_NUM}_rtabmap.log" 2>&1 &
        ;;
    3)
        echo "TEST 3: Без камерных топиков вообще"
        ros2 launch rtabmap_launch rtabmap.launch.py \
            args:="--delete_db_on_start --udebug" \
            use_sim_time:=false \
            frame_id:=base_link \
            odom_frame_id:=odom \
            subscribe_scan:=true \
            subscribe_rgbd:=false \
            scan_topic:=/scan \
            qos:=0 \
            approx_sync:=false \
            rtabmap_viz:=false \
            rviz:=false \
            visual_odometry:=false \
            icp_odometry:=true \
            odom_topic:=odom \
            database_path:=/tmp/rtabmap_test.db \
            wait_for_transform:=0.5 \
            > "$LOG_DIR/test${TEST_NUM}_rtabmap.log" 2>&1 &
        ;;
    4)
        echo "TEST 4: С launch_prefix для stereo=false"
        ros2 launch rtabmap_launch rtabmap.launch.py \
            args:="--delete_db_on_start --udebug" \
            use_sim_time:=false \
            frame_id:=base_link \
            odom_frame_id:=odom \
            stereo:=false \
            subscribe_scan:=true \
            scan_topic:=/scan \
            qos:=0 \
            approx_sync:=false \
            rtabmap_viz:=false \
            rviz:=false \
            visual_odometry:=false \
            icp_odometry:=true \
            odom_topic:=odom \
            database_path:=/tmp/rtabmap_test.db \
            wait_for_transform:=0.5 \
            > "$LOG_DIR/test${TEST_NUM}_rtabmap.log" 2>&1 &
        ;;
    *)
        echo "TEST $TEST_NUM: Custom (добавьте свои параметры)"
        ros2 launch rtabmap_launch rtabmap.launch.py \
            args:="--delete_db_on_start --udebug" \
            use_sim_time:=false \
            frame_id:=base_link \
            scan_topic:=/scan \
            icp_odometry:=true \
            rtabmap_viz:=false \
            > "$LOG_DIR/test${TEST_NUM}_rtabmap.log" 2>&1 &
        ;;
esac

RTABMAP_PID=$!

echo "PIDs: TF=$TF_PID, Lidar=$LIDAR_PID, RTABMAP=$RTABMAP_PID"
echo ""
echo "⏳ Ждём 40 секунд..."

# Ждём 40 секунд
for i in {1..40}; do
    sleep 1
    if [ $((i % 10)) -eq 0 ]; then
        echo "   $i сек..."
    fi
done

echo ""
echo "🛑 Останавливаем процессы..."

kill $RTABMAP_PID 2>/dev/null
sleep 2
kill $LIDAR_PID 2>/dev/null
kill $TF_PID 2>/dev/null

# Ждём завершения
sleep 3

echo ""
echo "📊 Анализ логов..."
echo "========================================="

# Анализируем логи
echo ""
echo "=== SUBSCRIBE ПАРАМЕТРЫ ==="
grep "subscribe_" "$LOG_DIR/test${TEST_NUM}_rtabmap.log" | grep -E "(scan|rgb|depth)" | head -10

echo ""
echo "=== ПОДПИСКИ (subscribed to) ==="
grep -A 10 "subscribed to" "$LOG_DIR/test${TEST_NUM}_rtabmap.log" | head -20

echo ""
echo "=== ОШИБКИ ==="
grep -i "error\|failed\|cannot" "$LOG_DIR/test${TEST_NUM}_rtabmap.log" | head -10

echo ""
echo "=== WARNINGS ==="
grep "Did not receive data" "$LOG_DIR/test${TEST_NUM}_rtabmap.log" | tail -5

echo ""
echo "========================================="
echo "✅ Тест завершён!"
echo ""
echo "Полные логи:"
echo "  - $LOG_DIR/test${TEST_NUM}_rtabmap.log"
echo "  - $LOG_DIR/test${TEST_NUM}_lidar.log"
echo "  - $LOG_DIR/test${TEST_NUM}_tf.log"
echo ""
echo "Для просмотра полных логов:"
echo "  cat $LOG_DIR/test${TEST_NUM}_rtabmap.log | less"
echo ""
