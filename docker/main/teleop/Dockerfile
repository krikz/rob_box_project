# syntax=docker/dockerfile:1
FROM ghcr.io/krikz/rob_box_base:ros2-zenoh-humble-latest

# Install joy node from ROS 2 packages and Bluetooth dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-joy-linux \
    joystick \
    jstest-gtk \
    bluez \
    python3-pip \
    python3-dbus \
    && rm -rf /var/lib/apt/lists/*

# Install Python BLE libraries via pip (not available in Ubuntu 22.04 apt)
RUN pip3 install --no-cache-dir bleak dbus-fast

# Copy workspace source (пакет rob_box_teleop)
COPY --chown=ros2:ros2 src/rob_box_teleop /workspace/src/rob_box_teleop

# Build workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd /workspace && \
    colcon build \
        --packages-select rob_box_teleop \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --symlink-install

WORKDIR /workspace

# Default command will be overridden by docker-compose
CMD ["bash"]
