#!/bin/bash

# Скрипт для принудительного запуска публикации камеры
# После запуска контейнера выполните этот скрипт

echo "Настройка OAK-D для принудительной публикации..."

# Отключаем lazy publisher для цветного изображения
docker exec oak-d /ros_entrypoint.sh ros2 param set /camera/camera color.i_enable_lazy_publisher false
echo "✅ Отключен lazy publisher для RGB"

# Отключаем lazy publisher для глубины
docker exec oak-d /ros_entrypoint.sh ros2 param set /camera/camera depth.i_enable_lazy_publisher false
echo "✅ Отключен lazy publisher для Depth"

sleep 2

# Проверяем публикацию
echo ""
echo "Проверка топиков..."
docker exec oak-d /ros_entrypoint.sh ros2 topic info /oak/rgb/image_raw/compressed | grep "Publisher count"
docker exec oak-d /ros_entrypoint.sh ros2 topic info /oak/stereo/image_raw/compressedDepth | grep "Publisher count"

echo ""
echo "✅ Готово! Камера должна публиковать данные."
echo "Проверьте: ros2 topic hz /oak/rgb/image_raw/compressed"
