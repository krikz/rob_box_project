# Интеграция rob_box_rtabmap_manager в Docker

## 📋 Задача
Добавить новый пакет `rob_box_rtabmap_manager` в контейнер rtabmap на Main Pi

## 🔧 Изменения

### 1. Dockerfile rtabmap

**Файл:** `docker/base/rtabmap/Dockerfile`

Добавить сборку пакета rob_box_rtabmap_manager:

```dockerfile
# После сборки других пакетов rob_box
COPY src/rob_box_rtabmap_manager ./src/rob_box_rtabmap_manager

RUN . /opt/ros/humble/setup.sh && \
    colcon build \
        --packages-select rob_box_rtabmap_manager \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --symlink-install

# Source workspace
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc
```

**Расположение:** Добавить ПОСЛЕ сборки rob_box_description и ПЕРЕД финальных инструкций

---

### 2. Launch файл rtabmap

**Вариант А: Отдельный launch (рекомендуется)**

Создать файл: `docker/main/scripts/rtabmap/start_rtabmap_with_manager.sh`

```bash
#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# Запуск rtabmap и manager в фоне
ros2 launch rob_box_rtabmap_manager rtabmap_manager.launch.py &
MANAGER_PID=$!

# Запуск основного rtabmap
exec ros2 launch rtabmap_ros rtabmap.launch.py \
    args:="--delete_db_on_start" \
    ...остальные параметры...

# При завершении убить manager
trap "kill $MANAGER_PID" EXIT
```

**Вариант Б: Включить в docker-compose**

В `docker/main/docker-compose.yaml`:

```yaml
services:
  rtabmap-manager:
    image: ${SERVICE_IMAGE_PREFIX:-ghcr.io/krikz/rob_box}:rtabmap-${ROS_DISTRO:-humble}-${IMAGE_TAG:-latest}
    container_name: rtabmap-manager
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_zenoh_cpp
      - LD_LIBRARY_PATH=/opt/ros/humble/opt/zenoh_cpp_vendor/lib:/opt/ros/humble/lib
    volumes:
      - ./maps:/maps
      - ./config:/config:ro
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /workspace/install/setup.bash &&
        ros2 run rob_box_rtabmap_manager rtabmap_manager_node
      "
    restart: unless-stopped
    depends_on:
      - zenoh-router
```

---

### 3. Volume для backup

Убедиться что директория для backup создаётся:

**В docker-compose.yaml:**
```yaml
volumes:
  - ./maps:/maps  # ✅ уже есть
```

**На хосте (Main Pi):**
```bash
mkdir -p ~/rob_box_project/docker/main/maps/deleted_backups
```

---

## 🚀 Деплой

### Локальная сборка (для теста):

```bash
# Main Pi
cd ~/rob_box_project/docker/base/rtabmap
docker build -t rob_box:rtabmap-humble-test .

# Запустить с новым образом
cd ~/rob_box_project/docker/main
docker-compose up -d rtabmap-manager
```

### CI/CD сборка:

GitHub Actions автоматически соберёт при merge в develop/main

**Workflow:** `.github/workflows/build-main-services.yml`

Проверить что rtabmap образ включает rob_box_rtabmap_manager:
```yaml
# Уже настроено копирование всех src/*
COPY src/ ./src/
```

---

## ✅ Проверка после деплоя

### 1. Проверить что нода запущена:

```bash
# SSH на Main Pi
ssh ros2@10.1.1.20

# Проверить логи
docker logs rtabmap-manager -f

# Ожидаемый вывод:
# [INFO] [rtabmap_manager]: ✅ RTABMap Manager Node запущен
# [INFO] [rtabmap_manager]:   БД: /maps/rtabmap.db
# [INFO] [rtabmap_manager]:   Backup: /maps/deleted_backups
# [INFO] [rtabmap_manager]:   Auto backup: True
```

### 2. Проверить ROS сервис:

```bash
# В контейнере rtabmap
docker exec rtabmap bash -c "
    source /opt/ros/humble/setup.bash && 
    source /workspace/install/setup.bash && 
    ros2 service list | grep rtabmap_manager
"

# Ожидаемый вывод:
# /rtabmap_manager/delete_all_data
```

### 3. Тест вызова сервиса:

```bash
# ВНИМАНИЕ: Это удалит БД!
docker exec rtabmap bash -c "
    source /opt/ros/humble/setup.bash && 
    source /workspace/install/setup.bash && 
    ros2 service call /rtabmap_manager/delete_all_data std_srvs/srv/Trigger
"

# Ожидаемый ответ:
# success: True
# message: "БД удалена (XX МБ). RTABMap создаст новую при запуске."
```

### 4. Проверить голосовую команду:

На Vision Pi через микрофон:
```
"Роббокс, удали все карты"
→ Ожидание подтверждения
"Да"
→ Робот: "Все карты удалены. БД удалена..."
```

Проверить логи dialogue_node:
```bash
docker logs voice-assistant -f | grep delete_all
```

---

## 🐛 Troubleshooting

### Проблема: Сервис недоступен

**Симптом:**
```
[ERROR] [dialogue_node]: ❌ Сервис rtabmap_manager недоступен
```

**Решение:**
1. Проверить что rtabmap-manager запущен: `docker ps | grep rtabmap`
2. Проверить логи: `docker logs rtabmap-manager`
3. Проверить Zenoh связь между Vision Pi и Main Pi
4. Перезапустить: `docker-compose restart rtabmap-manager`

### Проблема: Ошибка прав доступа

**Симптом:**
```
[ERROR] [rtabmap_manager]: ❌ Ошибка прав доступа: Permission denied
```

**Решение:**
```bash
# На хосте Main Pi
sudo chown -R ros2:ros2 ~/rob_box_project/docker/main/maps/
chmod -R 755 ~/rob_box_project/docker/main/maps/
```

### Проблема: Backup не создаётся

**Симптом:**
```
[WARN] [rtabmap_manager]: ⚠️  Backup не создан!
```

**Решение:**
```bash
# Создать директорию вручную
mkdir -p ~/rob_box_project/docker/main/maps/deleted_backups
chmod 755 ~/rob_box_project/docker/main/maps/deleted_backups
```

---

## 📝 Checklist для деплоя

- [ ] Обновлён Dockerfile rtabmap (добавлена сборка rob_box_rtabmap_manager)
- [ ] Создана директория `/maps/deleted_backups` на хосте
- [ ] Добавлен launch/docker-compose для rtabmap_manager_node
- [ ] Собран новый Docker образ
- [ ] Запущен rtabmap-manager контейнер
- [ ] Проверен ROS сервис `/rtabmap_manager/delete_all_data`
- [ ] Протестирована голосовая команда "удали все карты"
- [ ] Проверен автоматический backup
- [ ] Обновлена документация (если нужно)

---

**Автор:** GitHub Copilot  
**Дата:** 2025-11-17  
**Для:** Rob Box Project - Main Pi RTABMap Manager
