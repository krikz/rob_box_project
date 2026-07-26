# Rob Box Teleoperation - Quick Start

## 🎯 Что сделано

Создан пакет **rob_box_teleop** для прямого управления роботом через Bluetooth джойстик.

### ✅ Основные компоненты:

1. **ROS 2 пакет** `rob_box_teleop`
   - Нода `joystick_control_node` - обработка джойстика
   - Автоматическое обнаружение подключения/отключения
   - ARM/DISARM кнопки с голосовым подтверждением
   - Публикация в `/cmd_vel_joy` (приоритет 100)

2. **Docker контейнер** `teleop`
   - Dockerfile с joy_node и нашей нодой
   - Privileged режим для доступа к `/dev/input`
   - Healthcheck на процесс ноды

3. **Конфигурация**
   - Маппинг кнопок/осей для ExpressLRS джойстика
   - Ограничения скорости (0.5 м/с linear, 1.0 рад/с angular)
   - Deadzone 0.1 против дрейфа

## 🚀 Как использовать

### 1. Подключить джойстик на Main Pi

```bash
# SSH на Main Pi
sshpass -p 'open' ssh ros2@10.1.1.20

# Включить джойстик в режим pairing

# Подключить через bluetoothctl
bluetoothctl
> pair 8C:4F:00:C2:04:96
> connect 8C:4F:00:C2:04:96
> trust 8C:4F:00:C2:04:96
> exit

# Проверить устройство
ls -la /dev/input/js0
jstest /dev/input/js0
```

### 2. Локальный тест (dev машина)

```bash
# Собрать пакет
cd ~/rob_box_project
colcon build --packages-select rob_box_teleop --symlink-install
source install/setup.bash

# Запустить (нужен подключенный джойстик!)
ros2 run rob_box_teleop joystick_control_node --ros-args \
  --params-file src/rob_box_teleop/config/joystick_params.yaml
```

### 3. Deploy на робота

```bash
# Коммит изменений
git add src/rob_box_teleop docker/main
git commit -m "feat(teleop): add Bluetooth joystick control with voice feedback"
git push origin develop

# GitHub Actions автоматически:
# 1. Соберёт образ ghcr.io/krikz/rob_box:teleop-kilted-dev
# 2. Задеплоит на роботов
```

### 4. Запуск на роботе

```bash
# SSH на Main Pi
sshpass -p 'open' ssh ros2@10.1.1.20

cd ~/rob_box_project/docker/main

# Запустить teleop контейнер
docker-compose up -d teleop

# Проверить логи
docker logs -f teleop
```

## 🎮 Управление

1. **Включить джойстик** → Робот: "Джойстик подключен"
2. **Нажать ARM (кнопка 4)** → Робот: "Моторы активированы, готов к езде"
3. **Управлять стиками:**
   - Axis 1 (Pitch) - вперёд/назад
   - Axis 3 (Yaw) - повороты
4. **Нажать DISARM (кнопка 5)** → Робот: "Моторы отключены"

## 📊 Приоритеты команд

```
twist_mux принимает команды с приоритетами:
┌─────────────────────────────────────┐
│ Emergency Stop (255) - аварийная    │
├─────────────────────────────────────┤
│ Joystick (100) ← наш пакет          │
├─────────────────────────────────────┤
│ Web UI (50)                         │
├─────────────────────────────────────┤
│ Voice (25)                          │
├─────────────────────────────────────┤
│ Navigation (10)                     │
└─────────────────────────────────────┘
```

Джойстик имеет **второй по важности** приоритет после аварийной остановки!

## 🔧 Настройка маппинга

Если кнопки не совпадают, отредактируйте:

```yaml
# docker/main/config/teleop/joystick_params.yaml
joystick_control_node:
  ros__parameters:
    # Узнать номера кнопок: jstest /dev/input/js0
    button_arm: 4
    button_disarm: 5
    axis_linear_x: 1
    axis_angular_z: 3
```

## 🐛 Troubleshooting

**Джойстик не подключается:**
```bash
# Проверить Bluetooth
bluetoothctl show
bluetoothctl devices

# Перезапустить bluetooth
sudo systemctl restart bluetooth
```

**Ноды не стартуют:**
```bash
# Проверить что twist-mux запущен
docker ps | grep twist-mux

# Проверить Zenoh router
docker ps | grep zenoh-router

# Логи
docker logs teleop -f
```

**Неправильный маппинг:**
```bash
# Определить номера кнопок/осей
jstest /dev/input/js0

# Обновить конфиг и перезапустить
docker-compose restart teleop
```

## 📁 Файлы пакета

```
src/rob_box_teleop/                 - ROS 2 пакет
├── rob_box_teleop/
│   └── joystick_control_node.py    - Главная нода
├── config/
│   └── joystick_params.yaml        - Параметры
├── package.xml                     - Манифест пакета
├── setup.py                        - Python setup
└── CMakeLists.txt                  - CMake config

docker/main/
├── teleop/
│   └── Dockerfile                  - Образ контейнера
├── config/teleop/
│   └── joystick_params.yaml        - Конфиг для Docker
├── scripts/teleop/
│   └── start_teleop.sh             - Startup скрипт
└── docker-compose.yaml             - Сервис teleop добавлен
```

## ✨ Возможности

- ✅ Автообнаружение джойстика
- ✅ Голосовые уведомления (через TTS)
- ✅ ARM/DISARM для безопасности
- ✅ Интеграция с twist_mux
- ✅ Deadzone против дрейфа
- ✅ Ограничение скорости
- ✅ Docker deployment
- ✅ Healthcheck

## 🎉 Готово к использованию!

Пакет полностью готов. Осталось только:
1. Закоммитить изменения
2. Запушить в develop
3. GitHub Actions задеплоит на роботов автоматически
4. Подключить джойстик и тестировать!
