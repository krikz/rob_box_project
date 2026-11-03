# Dockerfile симуляции РОББОКСА: Gazebo Fortress + ros_gz + gz_ros2_control (amd64).
#
# Шаг 1 плана docs/research/2026-11-04-gazebo-sim-navigation.md §6: минимальный
# образ для проверки «робот в Gazebo едет от /cmd_vel». Профиль compose,
# Nav2/rtabmap/twist_mux и переключатель use_sim_time для боевых скриптов —
# шаг 2 (docker/sim/docker-compose.yaml).
#
# Сборка из корня репо:
#   docker build -f docker/sim/Dockerfile -t rob_box_sim:step1 .
#
# Контроллеры берутся из ТОГО ЖЕ файла, что на роботе
# (docker/main/config/controllers/controller_manager.yaml), путь внутри
# образа тот же, что у контейнера ros2-control: /config/controllers/.
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-gz \
        ros-humble-gz-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-ros2controlcli \
        ros-humble-controller-manager \
        ros-humble-robot-state-publisher \
        ros-humble-xacro \
        ros-humble-topic-tools \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src/rob_box_description /ws/src/rob_box_description
COPY src/rob_box_sim /ws/src/rob_box_sim
COPY docker/main/config/controllers/controller_manager.yaml /config/controllers/controller_manager.yaml

RUN source /opt/ros/humble/setup.bash \
    && colcon build --packages-select rob_box_description rob_box_sim \
    && rm -rf /ws/build /ws/log

COPY docker/sim/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "rob_box_sim", "sim.launch.py"]
