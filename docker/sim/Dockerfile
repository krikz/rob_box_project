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
#   * rtabmap — бинарь ros-lyrical-rtabmap-ros из репозитория ROS, а не
#     introlab3it/rtabmap_ros:lyrical-latest (docker/base/Dockerfile.rtabmap);
#   * rmw_zenoh_cpp — ros-lyrical-rmw-zenoh-cpp из apt (0.10.6, zenoh-c 1.8.0).
#     Humble-пин 0.1.8 (jammy .deb, zenoh 1.6.2) на resolute не ставится и
#     не совпадает с клиентом lyrical. Роутер сима — eclipse/zenoh:1.8.0
#     (docker/sim/compose.yaml), не 1.6.2 боевого compose.
FROM ros:lyrical-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# packages.ros.org на этой машине отдал deb того же размера с чужим SHA256
# (gz-msgs-vendor, gz-ogre-next-vendor, gz-gui-vendor, ros-gz-sim-demos,
# сборка 17.11, exit 100). Прямой curl с хоста тот же URL отдал ожидаемый
# SHA256. Без ретрая apt падает на первом Hash Sum mismatch.
# Pipeline-Depth 0 — тот же приём, что в docker/base/Dockerfile.ros2-zenoh.
RUN printf '%s\n' \
        'Acquire::Retries "5";' \
        'Acquire::http::Pipeline-Depth "0";' \
        > /etc/apt/apt.conf.d/99retries

# Gazebo Fortress + ros2_control (шаг 1). robot_state_publisher и xacro нужны
# боевому скрипту start_robot_state_publisher.sh (сервис robot-state-publisher).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-lyrical-ros-gz \
        ros-lyrical-gz-ros2-control \
        ros-lyrical-ros2-controllers \
        ros-lyrical-ros2controlcli \
        ros-lyrical-controller-manager \
        ros-lyrical-robot-state-publisher \
        ros-lyrical-xacro \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Стек навигации Main Pi: тот же набор, что docker/main/nav2/Dockerfile,
# плюс rtabmap и twist_mux. procps — для healthcheck (pgrep) из боевого compose.
# rmw_zenoh_cpp — из apt дистрибутива, как docker/base/Dockerfile.ros2-zenoh.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-lyrical-navigation2 \
        ros-lyrical-nav2-bringup \
        ros-lyrical-nav2-smac-planner \
        ros-lyrical-nav2-regulated-pure-pursuit-controller \
        ros-lyrical-nav2-simple-commander \
        ros-lyrical-rtabmap-ros \
        ros-lyrical-twist-mux \
        ros-lyrical-rmw-zenoh-cpp \
        procps \
        curl \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# ffmpeg — склейка кадров камер-сенсоров в mp4/gif (docker/sim/record_video.sh).
RUN apt-get update && apt-get install -y --no-install-recommends ffmpeg     && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src/rob_box_description /ws/src/rob_box_description
COPY src/rob_box_sim /ws/src/rob_box_sim

RUN source /opt/ros/lyrical/setup.bash \
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
# ros:lyrical-ros-base — Ubuntu 26.04, Python 3.12, не 3.10 humble.
# behaviortree_cpp_v3 в lyrical переименован в behaviortree_cpp (см. docker/main/nav2/Dockerfile).
RUN /bin/bash -c "source /opt/ros/lyrical/setup.bash && cd /tmp/turn-bt && \
    g++ -std=c++17 -fPIC -shared -DBT_PLUGIN_EXPORT \
      -I/opt/ros/lyrical/include \
      \$(for d in /opt/ros/lyrical/include/*/; do printf ' -I%s' \"\$d\"; done) \
      -I/tmp/turn-bt -I/usr/include/python3.12 \
      -o /opt/ros/lyrical/lib/librob_box_would_spin_on_stall_bt_node.so \
      would_spin_on_stall.cpp \
      -L/opt/ros/lyrical/lib -Wl,-rpath,/opt/ros/lyrical/lib \
      -Wl,--no-as-needed \
      -lbehaviortree_cpp -lrclcpp -ltf2 -ltf2_ros -lorocos-kdl -lpython3.12 \
      -lnav_msgs__rosidl_typesupport_cpp -lsensor_msgs__rosidl_typesupport_cpp \
      -lgeometry_msgs__rosidl_typesupport_cpp -ltf2_msgs__rosidl_typesupport_cpp" \
    && rm -rf /tmp/turn-bt

COPY docker/sim/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "rob_box_sim", "sim.launch.py"]
