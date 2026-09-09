# Краткий справочник РОББОКС

<div align="center">
  <strong>Быстрый доступ ко всем командам и настройкам</strong>
  <br>
  <sub>Версия: 1.0.0 | Дата: 2025-10-12</sub>
</div>

---

## 🚀 Быстрый старт

### Main Pi

```bash
# Переход в папку проекта
cd ~/rob_box_project/docker/main

# Запуск всех сервисов
docker compose up -d

# Проверка статуса
docker compose ps

# Просмотр логов
docker compose logs -f rtabmap
```

### Vision Pi

```bash
# Переход в папку проекта
cd ~/rob_box_project/docker/vision

# Запуск всех сервисов
docker compose up -d

# Проверка статуса
docker compose ps

# Просмотр логов
docker compose logs -f oak-d
```

---

## 📡 Сетевая конфигурация

| Устройство | Ethernet (данные) | WiFi (управление) |
|------------|-------------------|-------------------|
| **Main Pi** | 10.1.1.10 | 10.1.1.20 |
| **Vision Pi** | 10.1.1.11 | 10.1.1.21 |
| **Host PC** | — | 10.1.1.5 |

**SSH подключение**:
```bash
ssh ros2@10.1.1.20  # Main Pi
ssh ros2@10.1.1.21  # Vision Pi
```

---

## 🐳 Docker команды

```bash
# Остановка всех контейнеров
docker compose down

# Перезапуск конкретного сервиса
docker compose restart rtabmap

# Обновление образов
docker compose pull

# Просмотр логов
docker compose logs -f [service_name]

# Вход в контейнер
docker exec -it [container_name] bash

# Очистка старых образов
docker system prune -a
```

---

## 🤖 ROS 2 команды

### Просмотр топиков/нод

```bash
# Список нод
docker exec -it zenoh-router ros2 node list

# Список топиков
docker exec -it zenoh-router ros2 topic list

# Echo топика
docker exec -it zenoh-router ros2 topic echo /scan

# Частота топика
docker exec -it zenoh-router ros2 topic hz /scan

# Информация о топике
docker exec -it zenoh-router ros2 topic info /scan
```

### Управление роботом

```bash
# Управление с клавиатуры (teleop)
docker exec -it twist-mux ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Отправка одной команды
docker exec -it twist-mux ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

### LED анимации

```bash
# Список анимаций
docker exec -it led-matrix ros2 service call /animation_player/list_animations std_srvs/srv/Trigger

# Загрузить анимацию
docker exec -it led-matrix ros2 service call /animation_player/load_animation std_msgs/srv/String \
  "data: 'police_lights'"

# Запуск
docker exec -it led-matrix ros2 service call /animation_player/play std_srvs/srv/Trigger

# Стоп
docker exec -it led-matrix ros2 service call /animation_player/stop std_srvs/srv/Trigger
```

---

## 🗺️ SLAM и навигация

### RTAB-Map

```bash
# Просмотр карты
docker exec -it rtabmap ros2 topic echo /map --once

# Сброс базы данных (новая карта)
docker exec -it rtabmap rm -rf /root/.ros/rtabmap/*

# Просмотр статистики
docker exec -it rtabmap ros2 topic echo /rtabmap/info --once
```

### LiDAR

```bash
# Проверка LiDAR
docker exec -it lslidar ros2 topic hz /scan

# Просмотр сканов
docker exec -it lslidar ros2 topic echo /scan --once
```

---

## 🔧 Диагностика

### Проверка Zenoh

```bash
# Проверка роутера
curl http://10.1.1.10:8000/@/local/router

# Просмотр сессий
curl http://10.1.1.10:8000/@/local/sessions
```

### Проверка CAN шины (VESC)

```bash
# Поднять CAN интерфейс
sudo ip link set can0 up type can bitrate 500000

# Проверка CAN трафика
candump can0

# Отправка тестового сообщения
cansend can0 123#DEADBEEF
```

### Проверка сенсоров ESP32

```bash
# Просмотр данных от всех сенсоров
docker exec -it micro-ros-agent ros2 topic echo /device/snapshot

# Управление вентилятором
docker exec -it micro-ros-agent ros2 topic pub --once /device/command \
  robot_sensor_hub_msg/msg/DeviceCommand \
  "{device_type: 2, device_id: 0, command_code: 0, param_1: 0.75, param_2: 0.0}"
```

---

## 🎨 Визуализация

### RViz2

```bash
# Запуск RViz на хост-системе
ros2 launch rob_box_bringup display.launch.py

# Добавить топики:
# - /scan (LaserScan)
# - /map (Map)
# - /camera/color/image_raw (Image)
# - /rtabmap/cloud_map (PointCloud2)
```

### Foxglove Studio

1. Открыть Foxglove Studio
2. Подключиться к `ws://10.1.1.10:8765`
3. Добавить панели: Image, Map, 3D, Plot

---

## 📊 Мониторинг

### CPU/RAM

```bash
# Нагрузка на Pi
htop

# Docker статистика
docker stats

# Температура CPU
vcgencmd measure_temp
```

### Сеть

```bash
# Проверка Ethernet
ip addr show eth0

# Скорость интерфейса
ethtool eth0 | grep Speed

# Сетевой трафик
iftop -i eth0
```

---

## 🔄 Обновление системы

### Обновление Docker образов

```bash
# Main Pi
cd ~/rob_box_project/docker/main
docker compose pull
docker compose up -d

# Vision Pi
cd ~/rob_box_project/docker/vision
docker compose pull
docker compose up -d
```

### Обновление конфигураций

```bash
# После изменения config/*.yaml просто перезапустить контейнер
docker compose restart [service_name]

# Пересборка образа НЕ требуется!
```

---

## 🛠️ Troubleshooting

### Контейнер не запускается

```bash
# Проверить логи
docker compose logs [service_name]

# Проверить зависимости
docker compose config

# Удалить и пересоздать
docker compose down
docker compose up -d
```

### Zenoh не подключается

```bash
# Проверить firewall
sudo ufw status

# Проверить роутер
curl http://10.1.1.10:8000/@/local/router

# Перезапустить роутер
docker compose restart zenoh-router
```

### LiDAR не работает

```bash
# Проверить IP адрес LiDAR
ping 192.168.1.200

# Проверить UDP порт
sudo tcpdump -i eth0 port 2368

# Перезапустить драйвер
docker compose restart lslidar
```

---

## 📚 Документация

- **[README.md](../README.md)** — Главная страница проекта
- **[Архитектура системы](../architecture/SYSTEM_OVERVIEW.md)** — Системная архитектура
- **[Аппаратное обеспечение](../architecture/HARDWARE.md)** — Аппаратное обеспечение
- **[Программное обеспечение](../architecture/SOFTWARE.md)** — Программное обеспечение
- **[Развёртывание](../deployment/README.md)** — Развёртывание системы
- **[Участие в проекте](../../CONTRIBUTING.md)** — Как внести вклад

---

## 🔗 Полезные ссылки

- GitHub: https://github.com/krikz/rob_box_project
- Docker Registry: https://ghcr.io/krikz/rob_box
- ROS 2 Humble Docs: https://docs.ros.org/en/humble/
- Zenoh: https://zenoh.io/
- RTAB-Map: http://introlab.github.io/rtabmap/

---

<div align="center">
  <sub>Последнее обновление: 2025-10-12 | Версия: 1.0.0</sub>
</div>
