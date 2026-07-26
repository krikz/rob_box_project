FROM luxonis/depthai-ros:kilted-latest

# Установка rmw_zenoh_cpp
RUN apt-get update && \
    apt-get install -y ros-kilted-rmw-zenoh-cpp && \
    rm -rf /var/lib/apt/lists/*

# Запуск rmw_zenohd
CMD ["/bin/bash", "-c", "source /opt/ros/kilted/setup.bash && ros2 run rmw_zenoh_cpp rmw_zenohd"]
