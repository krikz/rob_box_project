#!/bin/bash
# RViz2 — вид от первого лица камеры OAK-D
# Fixed Frame: camera_color_optical_frame
# View: FPS (следует за камерой робота)

echo "🎥 === RViz2 FPV — вид из камеры робота ==="
echo ""

source /opt/ros/lyrical/setup.bash

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI="$(dirname "$0")/zenoh_local_session.json5"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# Фикс конфликта snap/GTK
unset GTK_PATH GTK_EXE_PREFIX GIO_MODULE_DIR

SCRIPT_DIR="$(dirname "$0")"

echo "✓ Fixed Frame: camera_color_optical_frame"
echo "✓ View: FPS (First Person Shooter)"
echo ""
echo "Отображается:"
echo "  • /rtabmap/odom_last_frame  — облако точек (текущий кадр, что видит камера)"
echo "  • /rtabmap/odom_local_map   — накопленная локальная карта"
echo "  • /camera/camera/color/image_raw/compressed — цветное изображение"
echo "  • /scan                     — лидар"
echo ""
echo "Управление видом в RViz:"
echo "  Mouse drag  — поворот камеры"
echo "  W/A/S/D     — движение (если зажат левый клик)"
echo ""
echo "Подключение к роботу через Zenoh..."
echo ""

rviz2 -d "$SCRIPT_DIR/rob_box_fpv.rviz"
