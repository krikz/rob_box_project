FROM ros:lyrical-ros-base

# Установка rmw_zenoh_cpp
RUN apt-get update && \
    apt-get install -y ros-lyrical-rmw-zenoh-cpp && \
    rm -rf /var/lib/apt/lists/*

# Запуск rmw_zenohd
CMD ["/bin/bash", "-c", "source /opt/ros/lyrical/setup.bash && ros2 run rmw_zenoh_cpp rmw_zenohd"]
