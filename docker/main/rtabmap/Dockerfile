FROM introlab3it/rtabmap_ros:humble-latest

# Установка плагинов для сжатия изображений
RUN apt-get update && \
    apt-get install -y \
        ros-humble-image-transport-plugins \
        ros-humble-compressed-image-transport \
        ros-humble-theora-image-transport \
    && rm -rf /var/lib/apt/lists/*

# Устанавливаем rmw_cyclonedds
RUN apt-get update && \
    apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
