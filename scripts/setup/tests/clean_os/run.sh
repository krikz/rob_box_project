#!/usr/bin/env bash
# Прогон установки узла в чистой ОС в контейнере с systemd (#4375). Только локальный docker, ни робота, ни katana.
#
#   bash scripts/setup/tests/clean_os/run.sh <main|vision|sound> [ubuntu:25.04] [linux/arm64]
#
# Что делает: собирает образ (Dockerfile рядом), поднимает контейнер с systemd, кладёт в него закоммиченный
# HEAD репо (git archive) в /home/ros2/rob_box_project, запускает setup_node.sh <роль> с
# ROBBOX_SETUP_SKIP_HW=1 (без netplan/config.txt/udev/Hailo) и ROBBOX_SETUP_NO_GIT=1 (без клона из GitHub),
# повторяет установку (идемпотентность), проверяет агента host/chuangong/check_install.sh.
# Логи — в $OUT (по умолчанию ./clean_os_<роль>_<arch>). Контейнер удаляется, если KEEP=1 не задан.
set -uo pipefail

ROLE="${1:?роль: main|vision|sound}"
BASE="${2:-ubuntu:25.04}"
PLATFORM="${3:-linux/arm64}"
# pwd -W — путь вида D:/… в Git Bash на Windows (docker и git там — Windows-программы); на Linux — pwd.
HERE="$(cd "$(dirname "$0")" && (pwd -W 2>/dev/null || pwd))"
export MSYS_NO_PATHCONV=1
REPO="$(git -C "$HERE" rev-parse --show-toplevel)"
ARCH="${PLATFORM#linux/}"
OUT="${OUT:-$PWD/clean_os_${ROLE}_${ARCH}}"
NAME="robbox-clean-os-${ROLE}-${ARCH}-$$"
IMG="robbox-clean-os:${BASE//[:\/]/-}-${ARCH}"
mkdir -p "$OUT"

docker build -q --platform "$PLATFORM" --build-arg "BASE=$BASE" -t "$IMG" "$HERE" >"$OUT/build.log" 2>&1 \
    || { echo "сборка образа упала, см. $OUT/build.log"; exit 2; }
docker run -d --name "$NAME" --platform "$PLATFORM" --privileged --cgroupns=host \
    -v /sys/fs/cgroup:/sys/fs/cgroup:rw --tmpfs /run --tmpfs /run/lock "$IMG" >/dev/null || exit 2
[ "${KEEP:-0}" = 1 ] || trap 'docker rm -f "$NAME" >/dev/null 2>&1' EXIT

for _ in $(seq 1 60); do
    st="$(docker exec "$NAME" systemctl is-system-running 2>/dev/null || true)"
    case "$st" in running|degraded) break ;; esac
    sleep 2
done
echo "systemd: ${st:-нет ответа}" | tee "$OUT/systemd.txt"

git -C "$REPO" archive --format=tar --prefix=rob_box_project/ HEAD \
    | docker exec -i "$NAME" tar -x -C /home/ros2
docker exec "$NAME" chown -R ros2:ros2 /home/ros2/rob_box_project
docker exec "$NAME" sh -c '. /etc/os-release; echo "$PRETTY_NAME $(uname -m)"; python3 --version' | tee "$OUT/os.txt"

run_setup() {
    docker exec -u ros2 -w /home/ros2 -e HOME=/home/ros2 -e USER=ros2 \
        -e ROBBOX_SETUP_SKIP_HW=1 -e ROBBOX_SETUP_NO_GIT=1 "$NAME" \
        bash /home/ros2/rob_box_project/scripts/setup/setup_node.sh "$ROLE" </dev/null
}
run_setup >"$OUT/setup_1.log" 2>&1
echo "setup_node.sh $ROLE (1-й прогон): rc=$?" | tee "$OUT/result.txt"
run_setup >"$OUT/setup_2.log" 2>&1
echo "setup_node.sh $ROLE (2-й прогон): rc=$?" | tee -a "$OUT/result.txt"

docker exec -u ros2 -e HOME=/home/ros2 "$NAME" bash /home/ros2/rob_box_project/host/chuangong/check_install.sh \
    >"$OUT/check_install.log" 2>&1
echo "check_install.sh: rc=$?" | tee -a "$OUT/result.txt"
docker exec "$NAME" sh -c 'systemctl list-unit-files --no-pager | grep -E "chuangong|robbox|chrony|zram"; \
    echo ---; cat /etc/docker/daemon.json 2>&1; echo ---; systemctl --failed --no-pager' >"$OUT/units.txt" 2>&1
cat "$OUT/result.txt"
