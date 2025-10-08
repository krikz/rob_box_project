FROM introlab3it/rtabmap_ros:humble-latest

# Обновление GPG ключа ROS 2 и установка rmw_zenoh_cpp
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    gpg --batch --yes --no-tty --dearmor -o /usr/share/keyrings/ros2-latest-archive-keyring.gpg && \
    apt-get update && \
    apt-get install -y ros-humble-rmw-zenoh-cpp && \
    rm -rf /var/lib/apt/lists/*

# Запуск rmw_zenohd
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && ros2 run rmw_zenoh_cpp rmw_zenohd"]
