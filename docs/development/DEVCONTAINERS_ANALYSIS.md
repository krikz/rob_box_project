# DevContainers для rob_box_project - Идеи и Анализ

## Обзор концепции (из видео Articulated Robotics)

**DevContainers** - это VS Code расширение, которое позволяет разрабатывать код внутри Docker контейнера, при этом:
- Код остаётся на хосте (автоматически монтируется)
- IDE видит все библиотеки и зависимости внутри контейнера
- Не нужны виртуальные машины
- Конфигурация окружения можно шарить с командой

## Ключевые файлы

### `.devcontainer/Dockerfile`
Определяет Docker образ для разработки. Можно:
- Использовать готовые образы (например `osrf/ros:humble-desktop`)
- Добавлять свои инструменты (gdb, valgrind, python packages)
- Настраивать aliases и bashrc

### `.devcontainer/devcontainer.json`
Конфигурация VS Code:
```json
{
  "name": "ROS2 Humble Dev",
  "build": {
    "dockerfile": "Dockerfile"
  },
  "mounts": [
    "source=/path/to/host,target=/container/path,type=bind"
  ],
  "customizations": {
    "vscode": {
      "extensions": [
        "ms-vscode.cpptools",
        "ms-python.python",
        "ms-iot.vscode-ros"
      ]
    }
  },
  "forwardPorts": [8080, 6080],
  "postCreateCommand": "colcon build --symlink-install",
  "remoteUser": "root"
}
```

## Преимущества для rob_box_project

### 1. Унифицированное окружение разработки
**Проблема сейчас:**
- Разные разработчики могут иметь разные версии ROS2
- Ubuntu 22.04 vs 24.04 различия
- Различные установленные пакеты

**Решение DevContainer:**
- Все работают в одинаковом контейнере с ROS2 Humble
- Одинаковые версии библиотек
- Новый разработчик: git clone → "Reopen in Container" → готов к работе

### 2. Параллельная разработка для разных платформ
**Сценарий:**
- Main Pi (ARM64, Ubuntu 22.04, ROS2 Humble)
- Vision Pi (ARM64, Ubuntu 22.04, специфичные ML библиотеки)
- Desktop (x86_64, Ubuntu 24.04, ROS2 Jazzy для тестирования)

**С DevContainers:**
- Открыть проект для Main Pi → контейнер с Humble ARM64 simulation
- Открыть проект для Vision Pi → контейнер с OpenCV, TensorFlow
- Открыть проект для Desktop → контейнер с Jazzy

Всё на одном компьютере, без VM!

### 3. Интеграция с существующими Docker образами
rob_box уже использует Docker:
- `ghcr.io/krikz/rob_box_base:ros2-zenoh-humble`
- `ghcr.io/krikz/rob_box:micro-ros-agent-humble-latest`
- `ghcr.io/krikz/rob_box:nav2-humble-latest`

**Можно создать DevContainer на базе этих образов!**

### 4. GUI приложения (RViz, Gazebo)
Два подхода из видео:

**Подход 1: X11 forwarding**
```json
{
  "runArgs": ["--network=host", "--env=DISPLAY"],
  "mounts": ["source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind"]
}
```
Требует `xhost +local:` на хосте.

**Подход 2: VNC Remote Desktop**
```json
{
  "features": {
    "ghcr.io/devcontainers/features/desktop-lite:1": {}
  },
  "forwardPorts": [6080, 5901]
}
```
Открываешь `localhost:6080` в браузере → видишь рабочий стол с RViz.

### 5. Рекомендуемые расширения VS Code
DevContainer может автоматически устанавливать расширения:
```json
{
  "customizations": {
    "vscode": {
      "extensions": [
        "ms-vscode.cpptools",           // C/C++ IntelliSense
        "ms-python.python",             // Python support
        "ms-python.vscode-pylance",     // Python type checking
        "ms-iot.vscode-ros",            // ROS tools
        "ms-azuretools.vscode-docker",  // Docker management
        "twxs.cmake",                   // CMake syntax
        "streetsidesoftware.code-spell-checker",
        "eamodio.gitlens"               // Git integration
      ]
    }
  }
}
```

### 6. Автоматизация сборки
```json
{
  "postCreateCommand": "rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install",
  "postStartCommand": "source install/setup.bash"
}
```

## Конкретные сценарии для rob_box

### Сценарий 1: Разработка rob_box_description (URDF/Xacro)
**DevContainer конфигурация:**
```json
{
  "name": "rob_box URDF Development",
  "build": { "dockerfile": "Dockerfile" },
  "customizations": {
    "vscode": {
      "extensions": [
        "ms-iot.vscode-ros",
        "redhat.vscode-xml"
      ]
    }
  },
  "postCreateCommand": "colcon build --packages-select rob_box_description",
  "forwardPorts": [6080],
  "features": {
    "ghcr.io/devcontainers/features/desktop-lite:1": {}
  }
}
```

**Workflow:**
1. Открыть VS Code в DevContainer
2. Редактировать `rob_box_complete.xacro`
3. Сохранить → автоматический пересборка (можно настроить file watcher)
4. Открыть `localhost:6080` в браузере
5. Запустить `ros2 launch rob_box_bringup display.launch.py` в VNC терминале
6. Видеть изменения в RViz немедленно

### Сценарий 2: Разработка навигации (Nav2)
**DevContainer на базе nav2 образа:**
```dockerfile
FROM ghcr.io/krikz/rob_box:nav2-humble-latest

# Добавить dev tools
RUN apt-get update && apt-get install -y gdb python3-pytest
```

**Особенности:**
- Все Nav2 зависимости уже установлены
- Можно тестировать конфигурацию параметров
- Запускать симуляцию в Gazebo

### Сценарий 3: Разработка micro-ROS интеграции
**DevContainer для ESP32 + micro-ROS:**
```json
{
  "name": "rob_box micro-ROS Dev",
  "build": { "dockerfile": ".devcontainer/Dockerfile.microros" },
  "mounts": [
    "source=/dev,target=/dev,type=bind"  // Для доступа к /dev/ttyUSB0
  ],
  "runArgs": ["--privileged"],  // Для serial port
  "postCreateCommand": "colcon build --packages-select robot_sensor_hub_msg"
}
```

**Workflow:**
1. Редактировать `robot_sensor_hub_msg`
2. Автоматическая пересборка
3. Запустить micro-ROS agent внутри контейнера
4. Запустить `scripts/test_sensor_hub.py`
5. Тестировать с реальным ESP32

### Сценарий 4: Multi-root workspace (Main + Vision)
VS Code поддерживает multi-root workspaces:
```json
{
  "folders": [
    { "path": ".", "name": "Main Pi" },
    { "path": "host/vision", "name": "Vision Pi" }
  ],
  "settings": {}
}
```

**Каждая папка может иметь свой DevContainer!**
- `./devcontainer/` → Main Pi контейнер
- `host/vision/.devcontainer/` → Vision Pi контейнер с ML библиотеками

## Сравнение с текущим подходом

### Текущий подход (Docker Compose на Pi)
**Плюсы:**
- ✅ Production-ready окружение
- ✅ Реальное железо (CAN, UART, GPIO)
- ✅ CI/CD build and push

**Минусы:**
- ❌ Разработка на хосте, запуск в контейнере (разрыв)
- ❌ IDE не видит зависимости внутри контейнера
- ❌ Нужно пересобирать образ для изменений
- ❌ Нет IntelliSense для ROS2 пакетов

### С DevContainers
**Плюсы:**
- ✅ IDE полностью видит окружение
- ✅ IntelliSense работает с ROS2
- ✅ Быстрая итерация (symlink install)
- ✅ Команда работает в одинаковом окружении
- ✅ Можно разрабатывать на любой ОС

**Минусы:**
- ❌ Не для production (только development)
- ❌ Нет реального железа внутри контейнера (нужен Docker on Pi)
- ❌ Performance overhead на симуляцию

## Гибридный подход (рекомендация)

### Development (DevContainer)
На компьютере разработчика:
```
rob_box_project/
├── .devcontainer/
│   ├── Dockerfile           # Dev образ с tools
│   └── devcontainer.json
├── src/
│   ├── rob_box_description/
│   ├── rob_box_bringup/
│   └── robot_sensor_hub_msg/
└── ...
```

**Workflow:**
1. `git clone` проекта
2. VS Code → "Reopen in Container"
3. Разработка с полным IntelliSense
4. Тестирование в симуляции (Gazebo)
5. Commit & push

### Production (Docker Compose на Pi)
На реальном роботе:
```
docker/
├── main/
│   ├── docker-compose.yaml
│   ├── nav2/Dockerfile
│   ├── micro_ros_agent/Dockerfile
│   └── ...
└── vision/
    └── ...
```

**Workflow:**
1. Git pull на Pi
2. `docker compose up -d`
3. Реальное железо (CAN, sensors)
4. Production testing

## Примеры aliases для DevContainer

```bash
# В .devcontainer/Dockerfile
RUN echo 'alias cb="colcon build --symlink-install"' >> /root/.bashrc && \
    echo 'alias cbp="colcon build --symlink-install --packages-select"' >> /root/.bashrc && \
    echo 'alias ct="colcon test"' >> /root/.bashrc && \
    echo 'alias ctr="colcon test-result --verbose"' >> /root/.bashrc && \
    echo 'alias launch="ros2 launch rob_box_bringup"' >> /root/.bashrc && \
    echo 'alias rviz="ros2 launch rob_box_bringup display.launch.py"' >> /root/.bashrc && \
    echo 'alias topics="ros2 topic list"' >> /root/.bashrc && \
    echo 'alias nodes="ros2 node list"' >> /root/.bashrc
```

## Потенциальные проблемы и решения

### Проблема 1: Размер образа
**Решение:** Multi-stage builds
```dockerfile
# Stage 1: Build
FROM osrf/ros:humble-desktop AS builder
COPY src/ /ws/src/
RUN colcon build --install-base /opt/rob_box

# Stage 2: Dev
FROM osrf/ros:humble-desktop
COPY --from=builder /opt/rob_box /opt/rob_box
RUN apt-get install -y dev-tools
```

### Проблема 2: X11 forwarding на разных ОС
**Windows:** Нужен X server (VcXsrv, Xming)
**macOS:** Нужен XQuartz
**Linux:** Работает из коробки

**Решение:** Использовать VNC (desktop-lite feature) - работает везде через браузер.

### Проблема 3: USB devices (для micro-ROS)
**Решение:**
```json
{
  "runArgs": [
    "--privileged",
    "--device=/dev/ttyUSB0:/dev/ttyUSB0"
  ],
  "mounts": [
    "source=/dev,target=/dev,type=bind"
  ]
}
```

### Проблема 4: Performance (Gazebo симуляция)
**Решение:** GPU forwarding
```json
{
  "runArgs": [
    "--gpus=all",
    "--env=NVIDIA_VISIBLE_DEVICES=all",
    "--env=NVIDIA_DRIVER_CAPABILITIES=all"
  ]
}
```

## Roadmap для внедрения

### Фаза 1: Простой DevContainer (1-2 часа)
- [ ] Создать `.devcontainer/Dockerfile` на базе `osrf/ros:humble-desktop`
- [ ] Создать базовый `.devcontainer/devcontainer.json`
- [ ] Добавить ROS2 расширения VS Code
- [ ] Протестировать с `rob_box_description`

### Фаза 2: GUI Support (2-3 часа)
- [ ] Настроить X11 forwarding для Linux
- [ ] Добавить VNC desktop-lite feature
- [ ] Протестировать RViz через браузер
- [ ] Задокументировать setup для Windows/macOS

### Фаза 3: Multi-container setup (3-4 часа)
- [ ] Создать отдельные DevContainers для:
  - Main Pi development
  - Vision Pi development
  - micro-ROS development
- [ ] Настроить multi-root workspace

### Фаза 4: CI/CD интеграция (2-3 часа)
- [ ] GitHub Codespaces конфигурация (похоже на DevContainer)
- [ ] Automated testing в контейнере
- [ ] Pre-commit hooks

## Ресурсы

### Официальная документация
- https://code.visualstudio.com/docs/devcontainers/containers
- https://containers.dev/ (спецификация)
- https://github.com/devcontainers/features

### ROS-специфичные примеры
- https://github.com/athackst/vscode_ros2_workspace (отличный template)
- https://github.com/devcontainers/templates/tree/main/src/ros2

### Видео источник
- Articulated Robotics: "Docker is EASY with VS Code"
- Концепция: разработка внутри контейнера без потери IDE функций
- Ключевая идея: "Editor window is a door into its own little world"

## Выводы

DevContainers - это **идеальное дополнение** к существующей Docker инфраструктуре rob_box:

**Когда использовать:**
- ✅ Development на компьютере разработчика
- ✅ Тестирование новых фич в симуляции
- ✅ Onboarding новых разработчиков
- ✅ CI/CD testing

**Когда НЕ использовать:**
- ❌ Production deployment на роботе
- ❌ Hardware-in-the-loop тесты (если нет Docker на Pi)

**Рекомендация:** Внедрить постепенно, начиная с простого DevContainer для URDF разработки, затем расширять функционал.

**Приоритет:** СРЕДНИЙ
- Не блокирует текущую разработку
- Улучшит developer experience
- Упростит онбординг новых контрибьютеров
- Синергия с существующей Docker инфраструктурой

**Следующий шаг:** Создать простой proof-of-concept DevContainer для `rob_box_description` и протестировать workflow.
