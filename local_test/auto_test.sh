#!/bin/bash
# Автоматический тест rtabmap с разными конфигурациями параметров
# Запуск: ./auto_test.sh <номер_теста>
# Пример: ./auto_test.sh 1
#
# ПРАВИЛЬНАЯ конфигурация из официальной документации:
# http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot
# Раздел "2D laser + Wheel Odometry as guess"
#
# КЛЮЧЕВОЕ РЕШЕНИЕ:
# subscribe_depth:=false - НЕ подписываемся на depth
# subscribe_rgbd:=false - НЕ подписываемся на rgbd  
# subscribe_scan:=true - ПОДПИСЫВАЕМСЯ на LaserScan

# Время работы теста (секунды)
TEST_DURATION=40

# Директория для логов
LOG_DIR="/tmp/rtabmap_test_logs"
mkdir -p "$LOG_DIR"

# Проверяем аргумент
if [ $# -eq 0 ]; then
    echo "Использование: $0 <номер_теста>"
    echo "Доступные тесты:"
    echo "  1 - ✅ ПРАВИЛЬНАЯ конфигурация: subscribe_depth:=false + subscribe_rgbd:=false + subscribe_scan:=true"
    echo "  2 - ❌ СТАРАЯ (неправильная): /null/* топики + scan_topic (для сравнения)"
    echo "  3 - Тест с Reg/Force3DoF:=true (2D SLAM режим)"
    echo "  4 - Тест с дополнительными ICP параметрами"
    exit 1
fi

TEST_NUM=$1

echo "Удаляем старую БД..."
rm -f /tmp/rtabmap_test.db

echo "Запускаем fake_lidar_publisher..."
python3 local_test/fake_lidar_publisher.py > "$LOG_DIR/test${TEST_NUM}_lidar.log" 2>&1 &
LIDAR_PID=$!
sleep 2

echo "Запускаем rtabmap с конфигурацией теста $TEST_NUM..."

case $TEST_NUM in
    1)
        echo "===================================================================================="
        echo "TEST 1: ✅ ПРАВИЛЬНАЯ конфигурация (из официальной документации)"
        echo "subscribe_depth:=false + subscribe_rgbd:=false + subscribe_scan:=true"
        echo "Источник: http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot"
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
        ;;
    2)
        echo "===================================================================================="
        echo "TEST 2: ❌ СТАРАЯ (неправильная) конфигурация с /null/* топиками"
        echo "Для сравнения - покажет что subscribe_scan=false"
        echo "===================================================================================="
        ros2 launch rtabmap_launch rtabmap.launch.py \
            args:="--delete_db_on_start --udebug" \
            use_sim_time:=false \
            frame_id:=base_link \
            odom_frame_id:=odom \
            rgb_topic:=/null/rgb \
            depth_topic:=/null/depth \
            camera_info_topic:=/null/camera_info \
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
        ;;
    3)
        echo "===================================================================================="
        echo "TEST 3: Правильная конфигурация + Reg/Force3DoF:=true (2D SLAM)"
        echo "===================================================================================="
        ros2 launch rtabmap_launch rtabmap.launch.py \
            args:="--delete_db_on_start --udebug -p Reg/Force3DoF:true -p Reg/Strategy:1" \
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
        ;;
    4)
        echo "===================================================================================="
        echo "TEST 4: Правильная конфигурация + дополнительные ICP параметры"
        echo "===================================================================================="
        ros2 launch rtabmap_launch rtabmap.launch.py \
            args:="--delete_db_on_start --udebug -p Reg/Force3DoF:true -p Reg/Strategy:1 -p Icp/VoxelSize:0.05 -p Icp/MaxCorrespondenceDistance:0.1" \
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
        ;;
    *)
        echo "Неизвестный тест: $TEST_NUM"
        kill $LIDAR_PID 2>/dev/null
        exit 1
        ;;
esac

RTABMAP_PID=$!

echo ""
echo "Работа в течение $TEST_DURATION секунд..."
echo "LIDAR PID: $LIDAR_PID"
echo "RTABMAP PID: $RTABMAP_PID"
echo ""

sleep "$TEST_DURATION"

echo ""
echo "Останавливаем процессы..."
kill $RTABMAP_PID 2>/dev/null
kill $LIDAR_PID 2>/dev/null
sleep 2

# Убиваем все оставшиеся процессы
killall -9 rtabmap icp_odometry fake_lidar_publisher 2>/dev/null

echo ""
echo "==================== АНАЛИЗ ЛОГОВ ===================="
echo "Логи сохранены в: $LOG_DIR/"
echo ""

RTABMAP_LOG="$LOG_DIR/test${TEST_NUM}_rtabmap.log"

echo "--- 1. Параметры subscribe_* (КЛЮЧЕВЫЕ!) ---"
grep -E "subscribe_scan|subscribe_rgb|subscribe_depth|subscribe_rgbd" "$RTABMAP_LOG" | head -15

echo ""
echo "--- 2. Подписки rtabmap (куда подписался rtabmap?) ---"
grep "rtabmap subscribed to" "$RTABMAP_LOG" -A 10 | head -20

echo ""
echo "--- 3. Подписки icp_odometry (должен подписаться на /scan) ---"
grep "icp_odometry subscribed to" "$RTABMAP_LOG" | head -5

echo ""
echo "--- 4. Ошибки и предупреждения (кроме Zenoh) ---"
grep -i "error\|warning\|failed" "$RTABMAP_LOG" | grep -v "Unable to connect to a Zenoh router" | head -10

echo ""
echo "--- 5. Статистика обработки данных ---"
grep "Processed" "$RTABMAP_LOG" | tail -5

echo ""
echo "======================================================"
echo "Полный лог rtabmap: $RTABMAP_LOG"
echo "Полный лог lidar:   $LOG_DIR/test${TEST_NUM}_lidar.log"
echo "======================================================"
echo ""
echo "Для просмотра полного лога:"
echo "  less $RTABMAP_LOG"
echo ""
