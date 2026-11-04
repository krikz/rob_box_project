# Dockerfile симуляции РОББОКСА: Gazebo Fortress + ros_gz + gz_ros2_control
# и боевой стек навигации Main Pi (amd64).
#
# План: docs/research/2026-11-04-gazebo-sim-navigation.md §6.
#   Шаг 1 — робот в Gazebo едет от /cmd_vel.
#   Шаг 2 — один образ на все сервисы docker/sim/compose.yaml:
#           robot_state_publisher, twist_mux, rtabmap, Nav2 запускаются
#           БОЕВЫМИ скриптами docker/main/scripts/** с БОЕВЫМИ конфигами
#           docker/main/config/** (оба монтируются compose, в образ не копируются).
#
# Сборка из корня репо:
#   docker build -f docker/sim/Dockerfile -t rob_box_sim:local .
#
# Отличия от образов робота (arm64, ghcr.io/krikz/rob_box_base:*):
#   * rtabmap — бинарь ros-humble-rtabmap-ros из репозитория ROS, а не
#     introlab3it/rtabmap_ros:humble-latest (docker/base/Dockerfile.rtabmap);
#   * rmw_zenoh_cpp тот же 0.1.8 (zenoh 1.6.2, как роутер eclipse/zenoh:1.6.2),
#     .deb amd64 с того же snapshot, что docker/base/Dockerfile.ros2-zenoh
#     (в docker/base/vendor/ лежат только arm64).
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Gazebo Fortress + ros2_control (шаг 1). robot_state_publisher и xacro нужны
# боевому скрипту start_robot_state_publisher.sh (сервис robot-state-publisher).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-gz \
        ros-humble-gz-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-ros2controlcli \
        ros-humble-controller-manager \
        ros-humble-robot-state-publisher \
        ros-humble-xacro \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Стек навигации Main Pi: тот же набор, что docker/main/nav2/Dockerfile,
# плюс rtabmap и twist_mux. procps — для healthcheck (pgrep) из боевого compose.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-nav2-smac-planner \
        ros-humble-nav2-regulated-pure-pursuit-controller \
        ros-humble-nav2-simple-commander \
        ros-humble-rtabmap-ros \
        ros-humble-twist-mux \
        procps \
        curl \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# rmw_zenoh_cpp 0.1.8 (пин и причина — docker/base/Dockerfile.ros2-zenoh).
RUN set -eux; \
    snap=http://snapshots.ros.org/humble/2026-03-29/ubuntu/pool/main/r; \
    curl -fsSL -o /tmp/rmw-zenoh.deb \
        "$snap/ros-humble-rmw-zenoh-cpp/ros-humble-rmw-zenoh-cpp_0.1.8-1jammy.20260304.205755_amd64.deb"; \
    curl -fsSL -o /tmp/zenoh-vendor.deb \
        "$snap/ros-humble-zenoh-cpp-vendor/ros-humble-zenoh-cpp-vendor_0.1.8-1jammy.20260304.075447_amd64.deb"; \
    apt-get update; \
    apt-get install -y --no-install-recommends /tmp/rmw-zenoh.deb /tmp/zenoh-vendor.deb; \
    rm -f /tmp/rmw-zenoh.deb /tmp/zenoh-vendor.deb; \
    rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src/rob_box_description /ws/src/rob_box_description
COPY src/rob_box_sim /ws/src/rob_box_sim

RUN source /opt/ros/humble/setup.bash \
    && colcon build --packages-select rob_box_description rob_box_sim \
    && rm -rf /ws/build /ws/log

ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

COPY docker/sim/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "rob_box_sim", "sim.launch.py"]
