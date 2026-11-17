#!/bin/bash
# Restart all 4 билд-раннера + поднять 5-8 (root-owned workspace сначала чистится через docker)
set -e
cd /home/ros2/rob_box_project/docker/build

echo "=== Restart existing 1-4 ==="
docker stop build-github-runner-1 build-github-runner-2 build-github-runner-3 build-github-runner-4 2>&1 | tail -4

# Чистим root-owned workspace docker-контейнером
for i in 1 2 3 4; do
  echo "--- clean runner $i ---"
  docker run --rm --entrypoint sh -v $(pwd)/data/runner$i:/ws localhost:5000/krikz/rob_box:voice-assistant-humble-test -c 'rm -rf /ws/_work && echo CLEANED' 2>&1 | tail -1
done

# Запускаем все 8
echo "=== Up 1-8 ==="
docker compose up -d github-runner-1 github-runner-2 github-runner-3 github-runner-4 github-runner-5 github-runner-6 github-runner-7 github-runner-8 2>&1 | tail -3

sleep 25
echo "=== status ==="
docker ps --format '{{.Names}} {{.Status}}' | grep github-runner | sort
