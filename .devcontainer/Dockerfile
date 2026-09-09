# DevContainer для rob_box_project ROS2 Humble Development

FROM osrf/ros:humble-desktop-full

# Аргументы для неинтерактивной установки
ARG DEBIAN_FRONTEND=noninteractive

# Установка дополнительных инструментов разработки
RUN apt-get update && apt-get install -y \
    # Build tools
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # Development tools
    git \
    vim \
    nano \
    wget \
    curl \
    # ROS2 packages
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-gazebo-ros-pkgs \
    # micro-ROS
    ros-humble-micro-ros-agent \
    # Debugging tools
    gdb \
    valgrind \
    # Python packages
    python3-pytest \
    python3-pytest-cov \
    # Network tools for testing
    net-tools \
    iputils-ping \
    # Clean up
    && rm -rf /var/lib/apt/lists/*

# Установка Python зависимостей
RUN pip3 install --no-cache-dir \
    pre-commit \
    black \
    flake8 \
    pytest-timeout

# Инициализация rosdep
RUN rosdep update

# Создание рабочего пространства
WORKDIR /workspace

# Автоматический source setup.bash при входе в терминал
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /root/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> /root/.bashrc && \
    echo "alias cbp='colcon build --symlink-install --packages-select'" >> /root/.bashrc && \
    echo "alias ct='colcon test'" >> /root/.bashrc && \
    echo "alias ctr='colcon test-result --verbose'" >> /root/.bashrc

# Установка цветов в терминале
RUN echo 'export PS1="\[\e[36m\][rob_box]\[\e[m\] \[\e[32m\]\u\[\e[m\]:\[\e[33m\]\w\[\e[m\]\\$ "' >> /root/.bashrc

# Команда по умолчанию
CMD ["/bin/bash"]
