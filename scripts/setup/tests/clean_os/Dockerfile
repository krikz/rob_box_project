# Чистая ОС, похожая на Pi робота, для прогона scripts/setup/setup_node.sh (#4375).
# Только то, что уже есть в образе Ubuntu Server для Raspberry Pi до установки: systemd, sudo, python3, curl.
# Пользователь ros2 с sudo без пароля (на живой Pi sudo спрашивает пароль — здесь некому вводить).
#
# APT_HTTPS=1 (по умолчанию) переводит источники Ubuntu на https: с рабочей машины 09.11 http://*.ubuntu.com
# по порту 80 не отвечал (curl: таймаут), а https отвечал. ca-certificates берётся без проверки TLS, но apt
# всё равно сверяет GPG-подписи репозитория, так что подменить пакет нельзя.
ARG BASE=ubuntu:25.04
FROM ${BASE}
ARG APT_HTTPS=1
ENV DEBIAN_FRONTEND=noninteractive
RUN if [ "$APT_HTTPS" = 1 ]; then \
      for f in /etc/apt/sources.list.d/ubuntu.sources /etc/apt/sources.list; do \
        [ -f "$f" ] && sed -i -E 's#http://(ports|archive|security)\.ubuntu\.com#https://\1.ubuntu.com#g' "$f"; \
      done; \
    fi \
 && apt-get -o Acquire::https::Verify-Peer=false update -qq \
 && apt-get -o Acquire::https::Verify-Peer=false install -y -qq --no-install-recommends ca-certificates \
 && apt-get update -qq \
 && apt-get install -y -qq --no-install-recommends systemd systemd-sysv sudo python3 curl \
      iproute2 kmod udev dbus \
 && rm -rf /var/lib/apt/lists/* \
 && useradd -m -s /bin/bash ros2 \
 && echo 'ros2 ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ros2 \
 && chmod 0440 /etc/sudoers.d/ros2
STOPSIGNAL SIGRTMIN+3
CMD ["/sbin/init"]
