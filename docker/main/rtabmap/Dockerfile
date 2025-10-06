FROM introlab3it/rtabmap_ros:kilted-latest

# Установка плагинов для сжатия изображений
RUN apt-get update && \
    apt-get install -y \
        ros-kilted-image-transport-plugins \
        ros-kilted-compressed-image-transport \
        ros-kilted-theora-image-transport \
    && rm -rf /var/lib/apt/lists/*