FROM introlab3it/rtabmap_ros:humble-latest

# Установка rmw_zenoh_cpp  
RUN apt-get update && \
    apt-get install -y ros-humble-rmw-zenoh-cpp && \
    rm -rf /var/lib/apt/lists/*

# Запуск rmw_zenohd
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && ros2 run rmw_zenoh_cpp rmw_zenohd"]
