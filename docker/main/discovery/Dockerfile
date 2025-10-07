FROM ros:jazzy-ros-base

# Установи Fast DDS
RUN apt-get update && apt-get install -y \
    ros-jazzy-fastrtps \
    && rm -rf /var/lib/apt/lists/*

# fast-discovery-server поставляется с fastrtps
CMD ["fast-discovery-server", "-i", "0", "-p", "11811"]