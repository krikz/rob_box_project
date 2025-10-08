#!/bin/bash

# Скрипт диагностики OAK-D камеры
# Запускать на Vision Pi: ./diagnose.sh

echo "=========================================="
echo "OAK-D Camera Diagnostics"
echo "=========================================="
echo ""

echo "1. Проверка статуса контейнера:"
docker ps | grep oak-d
echo ""

echo "2. Последние логи камеры (поиск ошибок):"
docker logs oak-d 2>&1 | grep -E '(ERROR|WARN|Camera ready|Finished)' | tail -10
echo ""

echo "3. Список ROS2 топиков от камеры:"
docker exec oak-d bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list | grep oak'
echo ""

echo "4. Информация о сжатом RGB топике:"
docker exec oak-d bash -c 'source /opt/ros/humble/setup.bash && ros2 topic info /oak/rgb/image_raw/compressed'
echo ""

echo "5. Информация о базовом RGB топике (если есть):"
docker exec oak-d bash -c 'source /opt/ros/humble/setup.bash && ros2 topic info /oak/rgb/image_raw' 2>&1 || echo "Базовый топик отсутствует"
echo ""

echo "6. Проверка публикации (5 секунд):"
echo "   Проверяем /oak/rgb/image_raw/compressed..."
timeout 5 docker exec oak-d bash -c 'source /opt/ros/humble/setup.bash && ros2 topic hz /oak/rgb/image_raw/compressed' 2>&1 || echo "   НЕТ ДАННЫХ или TIMEOUT"
echo ""

echo "7. Нагрузка системы:"
echo "   CPU и память:"
free -h
uptime
echo ""

echo "8. Проверка USB устройств:"
lsusb | grep -i 'luxonis\|oak\|movidius' || echo "   OAK-D не найден в USB!"
echo ""

echo "9. Проверка конфигурации камеры:"
echo "   Параметры из oak_d_config.yaml:"
grep -E '(image_transport|i_fps|i_rgb_resolution|i_enabled)' ~/rob_box_project/docker/vision/config/oak_d_config.yaml | head -10
echo ""

echo "=========================================="
echo "Диагностика завершена"
echo "=========================================="
