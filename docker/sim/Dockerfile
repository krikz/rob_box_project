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

# ffmpeg — склейка кадров камер-сенсоров в mp4/gif (docker/sim/record_video.sh).
RUN apt-get update && apt-get install -y --no-install-recommends ffmpeg     && rm -rf /var/lib/apt/lists/*

# apriltag_ros — тот же apriltag_node, что в контейнере oak-d робота (стенд локализации #4346:
# тег в мире, детекции в /detections → rtabmap). Отдельный слой: кэш слоёв выше не сбрасывается.
RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-apriltag-ros \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src/rob_box_description /ws/src/rob_box_description
COPY src/rob_box_sim /ws/src/rob_box_sim

RUN source /opt/ros/humble/setup.bash \
    && colcon build --packages-select rob_box_description rob_box_sim \
    && rm -rf /ws/build /ws/log

ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

# #4475: WouldSpinOnStall — тот же .so, что docker/main/nav2/Dockerfile.
# bt_navigator грузит rob_box_would_spin_on_stall_bt_node из plugin_lib_names
# (боевой nav2_params.yaml). Без .so configure падает: cannot open shared object.
# Угол считает rob_box_teleop.turn_in_place (том compose, PYTHONPATH в start_nav2_direct.sh).
# doTransform<PoseStamped> зовёт KDL — без -lorocos-kdl dlopen падает на Quaternion.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        python3-dev \
    && rm -rf /var/lib/apt/lists/*
COPY docker/main/nav2/would_spin_on_stall.cpp docker/main/nav2/turn_in_place_py.hpp /tmp/turn-bt/
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /tmp/turn-bt && \
    g++ -std=c++17 -fPIC -shared -DBT_PLUGIN_EXPORT \
      -I/opt/ros/humble/include \
      \$(for d in /opt/ros/humble/include/*/; do printf ' -I%s' \"\$d\"; done) \
      -I/tmp/turn-bt -I/usr/include/python3.10 \
      -o /opt/ros/humble/lib/librob_box_would_spin_on_stall_bt_node.so \
      would_spin_on_stall.cpp \
      -L/opt/ros/humble/lib -Wl,-rpath,/opt/ros/humble/lib \
      -Wl,--no-as-needed \
      -lbehaviortree_cpp_v3 -lrclcpp -ltf2 -ltf2_ros -lorocos-kdl -lpython3.10 \
      -lnav_msgs__rosidl_typesupport_cpp -lsensor_msgs__rosidl_typesupport_cpp \
      -lgeometry_msgs__rosidl_typesupport_cpp -ltf2_msgs__rosidl_typesupport_cpp" \
    && rm -rf /tmp/turn-bt

COPY docker/sim/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "rob_box_sim", "sim.launch.py"]
