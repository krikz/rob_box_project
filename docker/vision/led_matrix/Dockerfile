# Dockerfile для LED Matrix Driver и Compositor
# Управление NeoPixel LED матрицами через SPI на Vision Pi (Raspberry Pi 5)
#
# Архитектура:
#   - led_matrix_driver: низкоуровневый драйвер (SPI → LED через pi5neo)
#   - led_matrix_compositor: композитор панелей (sensor_msgs/Image → driver)
#
# Физическая конфигурация:
#   - 4× панели 8×8 (фары передние/задние)
#   - 5× панели 5×5 (main_display 5×25)
#   - Общая цепочка: 381 LED (256 + 125)
#
# Сборочный шов (docs/plans/2026-09-15-builder-runtime-seam.md, пилот, §5):
# builder-стадия ставит build-essential/python3-dev/libffi-dev — нужны ТОЛЬКО
# для компиляции C-расширения pip-пакета spidev (под arm64 на PyPI нет
# prebuilt wheel, собирается из исходников) — и гоняет colcon build.
# Runtime-стадия компилятор и dev-заголовки не содержит: только то, что нужно
# ноде для запуска — apt runtime-пакеты ROS, готовые pip site-packages
# (скопированы как собранные .so/.py; ABI гарантированно совпадает, т.к. обе
# стадии используют один и тот же BASE_IMAGE → одна версия Python/glibc) и
# сам colcon workspace /ws/install.
#
# ВНИМАНИЕ (план §2.1): BASE_IMAGE сам наследует build-essential/colcon/rosdep
# от ros:humble-ros-base (апстрим OSRF) — эта стадия НЕ убирает унаследованный
# тулчейн, только то, что этот файл добавляет поверх базы (build-essential,
# python3-dev, libffi-dev, pip build-артефакты). Полное устранение тулчейна
# требует отдельной облегчённой runtime-базы — вне скоупа этого пилота.
#
# ОТКЛОНЕНИЕ ОТ ЧЕРНОВИКА ПЛАНА (§5.2): план предлагал
# `COPY --from=builder /pyinstall /usr/local`. Проверено локально
# (generic Ubuntu 22.04 — та же ОС, что и BASE_IMAGE, python3-pip 22.0.2):
# `pip3 install --prefix=/pyinstall ...` на Debian/Ubuntu не кладёт файлы в
# /pyinstall/lib/..., а добавляет ЕЩЁ ОДИН уровень — /pyinstall/local/lib/...
# (патч Debian в sysconfig, добавляющий "local" в install-scheme независимо от
# заданного --prefix). При копировании /pyinstall → /usr/local пакеты попадали
# бы в несуществующий для интерпретатора путь /usr/local/local/lib/... и
# `import spidev`/`import pi5neo` падали бы в рантайме на роботе, а не в CI —
# ровно тот класс регрессии, который план требует ловить (§10). Исправлено:
# копируем /pyinstall/local → /usr/local (см. Stage 2).

ARG ROS_DISTRO=humble
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh
ARG ROS2LEDS_SHA=unknown

# ============================================================
# Stage 1: builder — тулчейн + компиляция colcon workspace
# ============================================================
FROM ${BASE_IMAGE} AS builder

ARG ROS_DISTRO
ARG ROS2LEDS_SHA
ARG APT_PROXY=""

# apt-cacher-ng на build-хосте. Раннер уже передаёт APT_PROXY этому сервису
# (build-args в "L-Build Vision Pi Services.yml" / "L-Build Single Service.yml"),
# но Dockerfile его не объявлял — build-arg молча игнорировался, и обе стадии
# ходили напрямую в packages.ros.org. Это уронило сборку run 35027911983
# ("Hash Sum mismatch" на Packages.gz): multi-stage гоняет apt-get update в двух
# стадиях параллельно, и шанс поймать рассинхронизированный edge CDN удваивается.
# Паттерн 1:1 из docker/base/Dockerfile.ros2-zenoh:17-29 — с проверкой
# доступности прокси и тихим fallback на прямое соединение.
RUN if [ -n "$APT_PROXY" ]; then \
        PROXY_HOST=$(echo "$APT_PROXY" | sed 's|.*://\([^:/]*\).*|\1|'); \
        PROXY_PORT=$(echo "$APT_PROXY" | sed 's|.*://[^:]*:\([0-9]*\).*|\1|'); \
        if timeout 5 bash -c "cat < /dev/null > /dev/tcp/$PROXY_HOST/$PROXY_PORT" 2>/dev/null; then \
            echo "Acquire::http::Proxy \"$APT_PROXY\";" > /etc/apt/apt.conf.d/02proxy; \
            echo "Acquire::http::Timeout \"10\";" >> /etc/apt/apt.conf.d/02proxy; \
            echo "Acquire::Retries \"3\";" >> /etc/apt/apt.conf.d/02proxy; \
            echo "OK: APT proxy $APT_PROXY"; \
        else \
            echo "WARN: APT proxy недоступен, идём напрямую"; \
        fi \
    fi

# Тулчейн для сборки C-расширения spidev (нет prebuilt wheel под arm64 на
# PyPI, собирается из исходников) и для colcon build. build-essential/
# python3-dev/libffi-dev ставятся ТОЛЬКО в этой стадии — в рантайм не попадут.
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    build-essential \
    libffi-dev \
    && rm -rf /var/lib/apt/lists/*

# Python-библиотеки для LED. pi5neo - библиотека для управления WS2812B через
# SPI на Raspberry Pi 5. Ставим с --prefix=/pyinstall, чтобы потом скопировать
# только готовый результат в runtime-стадию (без компилятора, без pip-кеша,
# без промежуточных .c/.o файлов сборки spidev). См. предупреждение о layout
# в шапке файла — pip кладёт файлы в /pyinstall/local/..., не /pyinstall/....
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir --prefix=/pyinstall \
    pi5neo \
    spidev \
    numpy

# Рабочая директория
WORKDIR /ws
RUN mkdir -p src

# ============================================================
# ОПТИМИЗАЦИЯ КЭШИРОВАНИЯ: Копируем файлы от наименее к наиболее изменяемым
# ============================================================

# Инвалидация кэша при изменении субмодуля ros2leds
# КРИТИЧНО: Записываем SHA в файл для создания зависимости слоя!
# Docker инвалидирует кэш когда ROS2LEDS_SHA изменится
RUN echo "Building with ros2leds SHA: ${ROS2LEDS_SHA}" && \
    echo "${ROS2LEDS_SHA}" > /tmp/ros2leds_build_sha && \
    echo "Cache breaker created: $(cat /tmp/ros2leds_build_sha)"

# 1. Копируем package.xml (метаданные зависимостей) - изменяются редко
COPY src/ros2leds/led_matrix_driver/package.xml /ws/src/led_matrix_driver/
COPY src/ros2leds/led_matrix_compositor/package.xml /ws/src/led_matrix_compositor/

# 2. Установка ROS зависимостей через rosdep (кэшируется если package.xml не изменился)
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y || true

# 3. Копируем setup.py - изменяются реже чем код
COPY src/ros2leds/led_matrix_driver/setup.py /ws/src/led_matrix_driver/
COPY src/ros2leds/led_matrix_compositor/setup.py /ws/src/led_matrix_compositor/

# 4. Только ТЕПЕРЬ копируем весь исходный код (изменяется часто)
COPY src/ros2leds/led_matrix_driver /ws/src/led_matrix_driver
COPY src/ros2leds/led_matrix_compositor /ws/src/led_matrix_compositor

# 5. Сборка пакетов
#
# ВТОРОЕ ОТКЛОНЕНИЕ ОТ ЧЕРНОВИКА ПЛАНА (§5.2 предлагал сохранить
# --symlink-install "как есть"): убран флаг --symlink-install. Проверено
# локальной сборкой (amd64, тот же BASE_IMAGE): с --symlink-install colcon
# для ament_python создаёт в /ws/install НЕ копии файлов, а symlink'и/egg-link
# обратно в /ws/build и /ws/src (например,
# install/led_matrix_driver/lib/python3.10/site-packages/led-matrix-driver.egg-link
# указывает на /ws/src/led_matrix_driver; launch/config/package.xml в
# install/.../share — тоже symlink'и на /ws/build/...). План §2.2 утверждает,
# что интерфейс шва — это только /ws/install; при --symlink-install это
# НЕВЕРНО для ament_python: /ws/install без /ws/src и /ws/build оказывается
# битым (`import led_matrix_driver` падает ModuleNotFoundError, launch-файл
# отсутствует). В текущем одностадийном образе это не проявляется, потому
# что /ws/src и /ws/build остаются в том же слое. Без --symlink-install
# colcon кладёт в /ws/install настоящие копии — подтверждено локальной
# сборкой: source setup.bash без ошибок, `import led_matrix_driver`/
# `led_matrix_compositor` работает, launch-файл на месте, /ws/src в рантайме
# не нужен.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
    --packages-select led_matrix_driver led_matrix_compositor \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# ============================================================
# Stage 2: runtime — та же база, БЕЗ build-essential/python3-dev/libffi-dev
# ============================================================
FROM ${BASE_IMAGE}

ARG ROS_DISTRO
ARG APT_PROXY=""

# apt-cacher-ng на build-хосте. Раннер уже передаёт APT_PROXY этому сервису
# (build-args в "L-Build Vision Pi Services.yml" / "L-Build Single Service.yml"),
# но Dockerfile его не объявлял — build-arg молча игнорировался, и обе стадии
# ходили напрямую в packages.ros.org. Это уронило сборку run 35027911983
# ("Hash Sum mismatch" на Packages.gz): multi-stage гоняет apt-get update в двух
# стадиях параллельно, и шанс поймать рассинхронизированный edge CDN удваивается.
# Паттерн 1:1 из docker/base/Dockerfile.ros2-zenoh:17-29 — с проверкой
# доступности прокси и тихим fallback на прямое соединение.
RUN if [ -n "$APT_PROXY" ]; then \
        PROXY_HOST=$(echo "$APT_PROXY" | sed 's|.*://\([^:/]*\).*|\1|'); \
        PROXY_PORT=$(echo "$APT_PROXY" | sed 's|.*://[^:]*:\([0-9]*\).*|\1|'); \
        if timeout 5 bash -c "cat < /dev/null > /dev/tcp/$PROXY_HOST/$PROXY_PORT" 2>/dev/null; then \
            echo "Acquire::http::Proxy \"$APT_PROXY\";" > /etc/apt/apt.conf.d/02proxy; \
            echo "Acquire::http::Timeout \"10\";" >> /etc/apt/apt.conf.d/02proxy; \
            echo "Acquire::Retries \"3\";" >> /etc/apt/apt.conf.d/02proxy; \
            echo "OK: APT proxy $APT_PROXY"; \
        else \
            echo "WARN: APT proxy недоступен, идём напрямую"; \
        fi \
    fi

# Метаданные
LABEL org.opencontainers.image.title="LED Matrix System for Vision Pi"
LABEL org.opencontainers.image.description="ROS2 LED matrix driver and compositor with pi5neo"
LABEL org.opencontainers.image.source="https://github.com/krikz/ros2leds"

# Установка runtime-зависимостей: SPI/GPIO инструменты и ROS2-пакеты для
# Image messages. БЕЗ python3-dev/build-essential/libffi-dev — компилятор
# в рантайме не нужен, C-расширение spidev уже собрано в builder-стадии.
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-spidev \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# pip site-packages из builder-стадии — уже собранные .so/.py (pi5neo, spidev,
# numpy), ABI совпадает (обе стадии — один и тот же BASE_IMAGE, одна версия
# Python/glibc). Копируем /pyinstall/local, а не /pyinstall — см. предупреждение
# в шапке файла про Debian install-scheme, добавляющий "local" в путь.
COPY --from=builder /pyinstall/local /usr/local

# colcon workspace из builder-стадии — единственный продукт сборки, который
# обязан пересечь шов (план §2.2).
COPY --from=builder /ws/install /ws/install

# Рабочая директория
WORKDIR /ws
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

# ROS2 и Zenoh environment variables
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV RUST_LOG=zenoh=info
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ENV ZENOH_CONFIG=/config/shared/zenoh_session_config.json5

# Проверка доступности SPI устройства
HEALTHCHECK --interval=15s --timeout=5s --start-period=10s --retries=3 \
    CMD ros2 node list | grep -q led_matrix || exit 1

# Точка входа - source ROS и запуск через startup script
ENTRYPOINT ["/bin/bash", "-c", "source /ws/install/setup.bash && exec \"$@\"", "--"]

# По умолчанию запускаем оба компонента через launch file
CMD ["bash", "-c", "source /ws/install/setup.bash && ros2 launch led_matrix_compositor led_matrix_compositor_launch.py"]
