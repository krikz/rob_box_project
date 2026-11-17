# Ceiling Camera - использует базовый образ ros2-zenoh
# Локальная сборка: docker build --build-arg BASE_IMAGE=rob_box_base:ros2-zenoh
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh
FROM ${BASE_IMAGE}

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

# USB Camera — контракт develop: ros2 run usb_cam usb_cam_node_exe
# (start_ceiling_camera.sh, camera_params.yaml). Бинарного ros-lyrical-usb-cam
# в packages.ros.org нет (resolute amd64/arm64, 2026-11-17: в pool только
# orig.tar.gz + .dsc + debian.tar.xz, в Packages.gz пакета нет). v4l2_camera
# есть, но это другой узел и другие параметры — контракт не подменяем.
# Собираем выпущенный usb_cam 0.8.1 из того же source-пакета.
# apriltag-ros — детектор тегов потолка (ceiling_apriltag.launch.py), запускается
# только при CEILING_APRILTAG_ENABLED=true; python3-numpy — кормушка ceiling_tag_feed.py.
ARG USB_CAM_VERSION=0.8.1
RUN apt-get update && apt-get install -y \
    ros-lyrical-image-transport-plugins \
    ros-lyrical-cv-bridge \
    ros-lyrical-apriltag-ros \
    ros-lyrical-camera-info-manager \
    ros-lyrical-image-transport \
    ros-lyrical-ament-cmake-auto \
    ros-lyrical-rosidl-default-generators \
    python3-numpy \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    curl \
    ca-certificates \
    ffmpeg \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*
RUN set -eux; \
    pool="http://packages.ros.org/ros2/ubuntu/pool/main/r/ros-lyrical-usb-cam"; \
    mkdir -p /tmp/usb_cam_src; \
    curl -fsSL -o /tmp/usb_cam_src/usb_cam.orig.tar.gz \
        "${pool}/ros-lyrical-usb-cam_${USB_CAM_VERSION}.orig.tar.gz"; \
    tar -xzf /tmp/usb_cam_src/usb_cam.orig.tar.gz -C /tmp/usb_cam_src; \
    src="$(echo /tmp/usb_cam_src/ros-lyrical-usb-cam-*)"; \
    test -f "${src}/package.xml"; \
    mkdir -p /ws/src; \
    mv "${src}" /ws/src/usb_cam; \
    rm -rf /tmp/usb_cam_src
RUN bash -c "set -eux; source /opt/ros/lyrical/setup.bash; \
    cd /ws; \
    colcon build --merge-install --packages-select usb_cam \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF; \
    test -x /ws/install/lib/usb_cam/usb_cam_node_exe"
RUN printf '%s\n' "${USB_CAM_VERSION}" > /version.txt
RUN rm -f /etc/apt/apt.conf.d/02proxy

# Создаем директорию для camera_info файлов (требуется для usb_cam)
# Фактический файл калибровки будет монтирован через volume
RUN mkdir -p /root/.ros/camera_info

# ВАЖНО: Скрипты и конфиги НЕ копируются в образ!
# Они монтируются через volumes в docker-compose.yaml:
# - ./config:/config:ro (общие конфиги: zenoh, cyclonedds)
# - ./config/ceiling-camera:/config/ceiling-camera:ro (конфиг камеры)
# - ./config/ceiling-camera/ceiling_camera.yaml:/root/.ros/camera_info/ceiling_camera.yaml:ro (калибровка)
# - ./scripts/ceiling-camera:/scripts:ro (скрипт запуска)

# CMD монтируется из volume: /scripts/start_ceiling_camera.sh
CMD ["/scripts/start_ceiling_camera.sh"]
