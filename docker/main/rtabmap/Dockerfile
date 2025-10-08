FROM introlab3it/rtabmap_ros:humble-latest

# Установка плагинов для сжатия изображений
RUN apt-get update && \
    apt-get install -y \
        ros-humble-image-transport-plugins \
        ros-humble-compressed-image-transport \
        ros-humble-theora-image-transport \
    && rm -rf /var/lib/apt/lists/*

# Устанавливаем rmw_zenoh для лучшей производительности и надёжности
RUN apt-get update && \
    apt-get install -y \
    ros-humble-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

# Добавляем путь к библиотекам Zenoh в LD_LIBRARY_PATH
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib:$LD_LIBRARY_PATH
