#!/bin/bash
# Startup script для micro-ROS Agent
# Обеспечивает связь между ROS2 и micro-ROS устройствами (ESP32)

set -e

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  🔌 Starting micro-ROS Agent for ESP32 Sensor Hub"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Проверка ROS_DISTRO
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ERROR: ROS_DISTRO not set"
    exit 1
fi

echo "✅ ROS_DISTRO: $ROS_DISTRO"
echo "✅ RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# Настройки serial порта
SERIAL_PORT="${MICROROS_SERIAL_PORT:-/dev/ttyUSB0}"
BAUDRATE="${MICROROS_BAUDRATE:-115200}"

echo ""
echo "📡 Serial Configuration:"
echo "   Port: $SERIAL_PORT"
echo "   Baudrate: $BAUDRATE"

# Проверка наличия serial устройства
if [ ! -e "$SERIAL_PORT" ]; then
    echo "⚠️  WARNING: Serial port $SERIAL_PORT not found!"
    echo "   Available ports:"
    ls -la /dev/tty* 2>/dev/null | grep -E "ttyUSB|ttyAMA|ttyACM" || echo "   No serial devices found"
    echo ""
    echo "   Waiting for device to appear..."
    
    # Ждем до 30 секунд появления устройства
    for i in {1..30}; do
        if [ -e "$SERIAL_PORT" ]; then
            echo "✅ Device appeared: $SERIAL_PORT"
            break
        fi
        sleep 1
    done
    
    if [ ! -e "$SERIAL_PORT" ]; then
        echo "❌ ERROR: Serial port $SERIAL_PORT still not available after 30s"
        echo "   Check ESP32 connection and docker device mapping"
        exit 1
    fi
fi

# Проверка прав доступа к устройству
if [ ! -r "$SERIAL_PORT" ] || [ ! -w "$SERIAL_PORT" ]; then
    echo "⚠️  WARNING: No read/write permissions for $SERIAL_PORT"
    echo "   Current permissions:"
    ls -l "$SERIAL_PORT"
    echo ""
    echo "   Attempting to fix with chmod (requires privileged container or dialout group)"
    chmod 666 "$SERIAL_PORT" 2>/dev/null || echo "   Failed to change permissions"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  🚀 Launching micro_ros_agent"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Transport: Serial"
echo "Command: ros2 run micro_ros_agent micro_ros_agent serial --dev $SERIAL_PORT -b $BAUDRATE"
echo ""
echo "💡 Tip: ESP32 должен быть запрограммирован с micro-ROS firmware"
echo "         и должен передавать на том же baudrate: $BAUDRATE"
echo ""

# Запуск agent с автоперезапуском при ошибках
while true; do
    echo "🔄 Starting agent at $(date)"
    
    # Запуск micro-ROS agent
    # --dev - serial device
    # -b - baudrate
    # -v6 - verbose level (опционально для debugging)
    ros2 run micro_ros_agent micro_ros_agent serial \
        --dev "$SERIAL_PORT" \
        -b "$BAUDRATE" \
        || {
            echo "❌ Agent crashed with exit code $?"
            echo "   Restarting in 5 seconds..."
            sleep 5
            continue
        }
    
    echo "⚠️  Agent exited normally (unexpected)"
    sleep 5
done
