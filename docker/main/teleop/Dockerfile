# syntax=docker/dockerfile:1
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh-kilted-latest
FROM ${BASE_IMAGE}

# Install joy node from ROS 2 packages and Serial dependencies
RUN rm -f /etc/apt/apt.conf.d/02proxy && apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-joy-linux \
    joystick \
    jstest-gtk \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

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
