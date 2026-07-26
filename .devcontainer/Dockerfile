# DevContainer для rob_box_project ROS2 kilted Development

FROM osrf/ros:kilted-desktop-full

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
    ros-kilted-xacro \
    ros-kilted-joint-state-publisher-gui \
    ros-kilted-robot-state-publisher \
    ros-kilted-rviz2 \
    ros-kilted-gazebo-ros-pkgs \
    # micro-ROS
    ros-kilted-micro-ros-agent \
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
RUN echo "source /opt/ros/kilted/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /root/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> /root/.bashrc && \
    echo "alias cbp='colcon build --symlink-install --packages-select'" >> /root/.bashrc && \
    echo "alias ct='colcon test'" >> /root/.bashrc && \
    echo "alias ctr='colcon test-result --verbose'" >> /root/.bashrc

# Установка цветов в терминале
RUN echo 'export PS1="\[\e[36m\][rob_box]\[\e[m\] \[\e[32m\]\u\[\e[m\]:\[\e[33m\]\w\[\e[m\]\\$ "' >> /root/.bashrc

# Команда по умолчанию
CMD ["/bin/bash"]
