# Чистая ОС, похожая на Pi робота, для прогона scripts/setup/setup_node.sh (#4375).
# Только то, что уже есть в образе Ubuntu Server для Raspberry Pi до установки: systemd, sudo, python3, curl.
# Пользователь ros2 с sudo без пароля (на живой Pi sudo спрашивает пароль — здесь некому вводить).
ARG BASE=ubuntu:25.04
FROM ${BASE}
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update -qq \
 && apt-get install -y -qq --no-install-recommends systemd systemd-sysv sudo python3 curl ca-certificates \
      iproute2 kmod udev dbus \
 && rm -rf /var/lib/apt/lists/* \
 && useradd -m -s /bin/bash ros2 \
 && echo 'ros2 ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ros2 \
 && chmod 0440 /etc/sudoers.d/ros2
STOPSIGNAL SIGRTMIN+3
CMD ["/sbin/init"]
