FROM introlab3it/rtabmap_ros:jazzy-latest

# Установка плагинов для сжатия изображений
RUN apt-get update && \
    apt-get install -y \
        ros-jazzy-image-transport-plugins \
        ros-jazzy-compressed-image-transport \
        ros-jazzy-theora-image-transport \
    && rm -rf /var/lib/apt/lists/*
