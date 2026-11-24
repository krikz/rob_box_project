# Nav2 Navigation Stack для Main Pi
# Автономная навигация с планированием пути и избежанием препятствий
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh-humble-latest
FROM ${BASE_IMAGE}

LABEL org.opencontainers.image.title="Nav2 Navigation Stack"
LABEL org.opencontainers.image.description="ROS2 Nav2 для автономной навигации rob_box"
LABEL org.opencontainers.image.authors="krikz"

# APT через apt-cacher-ng на katana (APT_PROXY передаёт gen_build_matrix.py).
# Базовый образ удаляет свой 02proxy, поэтому без этого блока apt ходил
# напрямую, а прямой путь с katana ненадёжен: GPG error (run 34387765562),
# Hash Sum mismatch у .deb того же размера (run 36421581380, nav2/voice-base).
# Шаблон и причина Pipeline-Depth 0 — docker/base/Dockerfile.ros2-zenoh.
ARG APT_PROXY=""
RUN if [ -n "$APT_PROXY" ]; then \
        PROXY_HOST=$(echo "$APT_PROXY" | sed 's|.*://\([^:/]*\).*|\1|'); \
        PROXY_PORT=$(echo "$APT_PROXY" | sed 's|.*://[^:]*:\([0-9]*\).*|\1|'); \
        if timeout 5 bash -c "cat < /dev/null > /dev/tcp/$PROXY_HOST/$PROXY_PORT" 2>/dev/null; then \
            printf '%s\n' "Acquire::http::Proxy \"$APT_PROXY\";" 'Acquire::http::Timeout "10";' \
                'Acquire::Retries "3";' 'Acquire::http::Pipeline-Depth "0";' > /etc/apt/apt.conf.d/02proxy; \
            echo "✅ Using APT proxy: $APT_PROXY"; \
        else \
            echo "⚠️  APT proxy not accessible, will use direct connection"; \
        fi \
    fi

# Установка Nav2 пакетов
RUN apt-get update && apt-get install -y \
    # Nav2 Core
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-bt-navigator \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    # Costmaps (локальная и глобальная)
    ros-${ROS_DISTRO}-nav2-costmap-2d \
    # Планировщик пути
    ros-${ROS_DISTRO}-nav2-planner \
    ros-${ROS_DISTRO}-nav2-navfn-planner \
    ros-${ROS_DISTRO}-nav2-smac-planner \
    # Контроллер (следование по траектории)
    ros-${ROS_DISTRO}-nav2-controller \
    ros-${ROS_DISTRO}-nav2-dwb-controller \
    ros-${ROS_DISTRO}-nav2-regulated-pure-pursuit-controller \
    # Поведения (recovery behaviors - переименован из recoveries в behaviors)
    ros-${ROS_DISTRO}-nav2-behaviors \
    # Локализация
    ros-${ROS_DISTRO}-nav2-amcl \
    # Map Server
    ros-${ROS_DISTRO}-nav2-map-server \
    # Waypoint следование
    ros-${ROS_DISTRO}-nav2-waypoint-follower \
    # Утилиты
    ros-${ROS_DISTRO}-nav2-util \
    ros-${ROS_DISTRO}-nav2-msgs \
    # Behavior Tree плагины
    ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
    python3-dev \
    python3-yaml \
    # Дополнительные зависимости
    ros-${ROS_DISTRO}-navigation2 \
    && rm -rf /var/lib/apt/lists/*

# Pin Nav2 1.1.20:
# - controller_server fails closed when goal pose TF fails (upstream ignored
#   transformPose bool → default (0,0,0) → false "Reached the goal");
# - on progress/controller abort, terminate pending FollowPath too (#4694).
#   Humble SimpleActionServer otherwise restarts the pending goal on the same
#   thread and the BT never sees FAILURE;
# - NavFn refuses a path whose tail crosses lethal or unknown beyond a
#   lidar-seen wall (#4694). allow_unknown stays true;
# - NavFn gradient origin normalization: negative dx/dy in the four-cell
#   gradient loop underflowed the cell index, smearing the potential into
#   the previous row/column (#4682);
# - progress also fails when the base is stuck against an obstacle (#5003):
#   a held command, no motion of required_movement_radius, blocking cell on
#   the command course. The decision is C++ in controller_server (ADR-0148).
#   The process does not embed CPython (#5021).
# - recovery BackUp skips a collision tick when the costmap_raw copy is older
#   than costmap_freshness_tolerance (#5003): the copy is published at 2 Hz,
#   so a tick within tens of milliseconds of a clear still shows the wall.
#   Past the skip limit the check fails closed. The decision lives in
#   behavior_server; BackUp is its own plugin library, so that .so ships too
#   (ADR-0148).
COPY controller_goal_tf_fail.patch /tmp/controller_goal_tf_fail.patch
COPY controller_progress_abort.patch /tmp/controller_progress_abort.patch
COPY controller_stuck_against.patch /tmp/controller_stuck_against.patch
COPY navfn_refuse_unknown_tail.patch /tmp/navfn_refuse_unknown_tail.patch
COPY navfn_gradient_origin.patch /tmp/navfn_gradient_origin.patch
COPY costmap_publish_on_clear.patch /tmp/costmap_publish_on_clear.patch
COPY behavior_costmap_freshness.patch /tmp/behavior_costmap_freshness.patch
RUN apt-get update && apt-get install -y \
    build-essential \
    curl \
    patch \
    python3-dev \
    python3-colcon-common-extensions \
    && mkdir -p /tmp/nav2-fix/src \
    && curl -fsSL -o /tmp/nav2-fix/navigation2.tar.gz \
        "https://github.com/ros-navigation/navigation2/archive/a097086719c88f781aa59788eca29ac6ca5e56db.tar.gz" \
    && tar -xzf /tmp/nav2-fix/navigation2.tar.gz -C /tmp/nav2-fix/src \
    && mv /tmp/nav2-fix/src/navigation2-a097086719c88f781aa59788eca29ac6ca5e56db \
        /tmp/nav2-fix/src/navigation2 \
    && patch -d /tmp/nav2-fix/src/navigation2 -p1 </tmp/controller_goal_tf_fail.patch \
    && patch -d /tmp/nav2-fix/src/navigation2 -p1 </tmp/controller_progress_abort.patch \
    && patch -d /tmp/nav2-fix/src/navigation2 -p1 </tmp/controller_stuck_against.patch \
    && patch -d /tmp/nav2-fix/src/navigation2 -p1 </tmp/navfn_refuse_unknown_tail.patch \
    && patch -d /tmp/nav2-fix/src/navigation2 -p1 </tmp/navfn_gradient_origin.patch \
    && patch -d /tmp/nav2-fix/src/navigation2 -p1 </tmp/costmap_publish_on_clear.patch \
    && patch -d /tmp/nav2-fix/src/navigation2 -p1 </tmp/behavior_costmap_freshness.patch \
    && rm -rf /var/lib/apt/lists/*
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && cd /tmp/nav2-fix && \
    colcon build --symlink-install --base-paths /tmp/nav2-fix/src/navigation2 \
        --packages-select nav2_controller nav2_navfn_planner nav2_costmap_2d nav2_behaviors --cmake-args -DCMAKE_BUILD_TYPE=Release" \
    && cp /tmp/nav2-fix/install/nav2_controller/lib/libcontroller_server_core.so \
        /opt/ros/${ROS_DISTRO}/lib/libcontroller_server_core.so \
    && cp /tmp/nav2-fix/install/nav2_navfn_planner/lib/libnav2_navfn_planner.so \
        /opt/ros/${ROS_DISTRO}/lib/libnav2_navfn_planner.so \
    && cp /tmp/nav2-fix/install/nav2_costmap_2d/lib/libnav2_costmap_2d_core.so \
        /opt/ros/${ROS_DISTRO}/lib/libnav2_costmap_2d_core.so \
    && cp /tmp/nav2-fix/install/nav2_behaviors/lib/libbehavior_server_core.so \
        /opt/ros/${ROS_DISTRO}/lib/libbehavior_server_core.so \
    && cp /tmp/nav2-fix/install/nav2_behaviors/lib/libnav2_drive_on_heading_behavior.so \
        /opt/ros/${ROS_DISTRO}/lib/libnav2_drive_on_heading_behavior.so \
    && cp /tmp/nav2-fix/install/nav2_behaviors/lib/libnav2_back_up_behavior.so \
        /opt/ros/${ROS_DISTRO}/lib/libnav2_back_up_behavior.so \
    && echo "#5003: the shared-library linker allows undefined symbols, so the broken image built silently. A symbol undefined in a plugin .so is fine when behavior_server defines it; fail only when no .so in the image defines it." \
    && nm -D --defined-only /opt/ros/${ROS_DISTRO}/lib/*.so > /tmp/nav2-defined-symbols \
    && for so in libnav2_back_up_behavior.so libnav2_drive_on_heading_behavior.so libbehavior_server_core.so; do \
         for sym in $(nm -D --undefined-only /opt/ros/${ROS_DISTRO}/lib/$so | awk '/nav2_behaviors|behavior_server/ {print $2}'); do \
           if ! grep -ql " ${sym}$" /tmp/nav2-defined-symbols; then \
             echo "FAIL: ${sym} is undefined in $so and defined by no .so in the image (#5003)" >&2; \
             exit 1; \
           fi; \
         done; \
       done \
    && rm -rf /tmp/nav2-fix /tmp/controller_goal_tf_fail.patch \
        /tmp/controller_progress_abort.patch /tmp/controller_stuck_against.patch \
        /tmp/navfn_refuse_unknown_tail.patch /tmp/navfn_gradient_origin.patch \
        /tmp/costmap_publish_on_clear.patch /tmp/behavior_costmap_freshness.patch
# #4475: узел дерева WouldSpinOnStall. Угол считает rob_box_teleop.turn_in_place
# (том compose, PYTHONPATH в start_nav2_direct.sh) — тот файл, что проверяет pytest.
# doTransform<PoseStamped> в tf2_geometry_msgs.hpp зовёт KDL. Без -lorocos-kdl
# dlopen падает на KDL::Rotation::Quaternion, хотя getTimestamp уже свой.
COPY would_spin_on_stall.cpp turn_in_place_py.hpp /tmp/turn-bt/
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && cd /tmp/turn-bt && \
    g++ -std=c++17 -fPIC -shared -DBT_PLUGIN_EXPORT \
      -I/opt/ros/${ROS_DISTRO}/include \
      \$(for d in /opt/ros/${ROS_DISTRO}/include/*/; do printf ' -I%s' \"\$d\"; done) \
      -I/tmp/turn-bt -I/usr/include/python3.10 \
      -o /opt/ros/${ROS_DISTRO}/lib/librob_box_would_spin_on_stall_bt_node.so \
      would_spin_on_stall.cpp \
      -L/opt/ros/${ROS_DISTRO}/lib -Wl,-rpath,/opt/ros/${ROS_DISTRO}/lib \
      -Wl,--no-as-needed \
      -lbehaviortree_cpp_v3 -lrclcpp -ltf2 -ltf2_ros -lorocos-kdl -lpython3.10 \
      -lnav_msgs__rosidl_typesupport_cpp -lsensor_msgs__rosidl_typesupport_cpp \
      -lgeometry_msgs__rosidl_typesupport_cpp -ltf2_msgs__rosidl_typesupport_cpp" \
    && rm -rf /tmp/turn-bt
RUN rm -f /etc/apt/apt.conf.d/02proxy

# Создание рабочих директорий
RUN mkdir -p /maps /config/nav2

WORKDIR /workspace

# Health check. На роботе этот блок перекрыт healthcheck в docker-compose.yaml
# (том ./scripts/nav2 → /scripts), поэтому доезжает только со сборкой образа.
# Числа — healthcheck_budget.py (#5150): Timeout 10s не поднимаем (решение
# Шифу 23.11), Interval 15s, StartPeriod 90s. Старый node-list зеленел при
# мёртвом controller_server (#5043).
HEALTHCHECK --interval=15s --timeout=10s --start-period=90s --retries=3 \
    CMD bash /scripts/healthcheck.sh

# Точка входа - ROS2 setup
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && exec \"$@\"", "--"]

CMD ["bash"]
