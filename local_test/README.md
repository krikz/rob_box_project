# 🧪 Local Test Scripts - Инструменты для удалённой диагностики

Коллекция утилит для тестирования и мониторинга робота через Zenoh middleware с локальной машины (без необходимости SSH на Raspberry Pi).

## � Структура

- **voice/** - Тесты голосового ассистента (STT, LLM, TTS)
- **hardware/** - Тесты железа (LED, ReSpeaker, USB устройства)
- Корень: скрипты визуализации и мониторинга через Zenoh

## �📋 Обзор

Все скрипты используют **Zenoh middleware** для подключения к роботу через сеть. Это позволяет:
- ✅ Просматривать топики, TF трансформации, изображения с камеры
- ✅ Визуализировать данные в RViz2
- ✅ Мониторить AprilTag детекции
- ✅ Работать без SSH подключения к Pi

## 🚀 Quick Start

### Предварительные требования

```bash
# ROS 2 Humble с Zenoh middleware
sudo apt install ros-humble-rmw-zenoh-cpp

# Дополнительные пакеты
sudo apt install ros-humble-rqt-image-view python3-opencv python3-cv-bridge
```

### Конфигурация подключения

Файл `zenoh_client_config.json5` настроен для подключения к:
- **Main Pi**: `10.1.1.10:7447` (навигация, SLAM, управление)
- **Vision Pi**: `10.1.1.21:7447` (камера, AprilTag, сенсоры)

```json5
{
  "mode": "client",
  "connect": {
    "endpoints": [
      "tcp/10.1.1.21:7447",  // Vision Pi
      "tcp/10.1.1.10:7447"   // Main Pi
    ]
  },
  "scouting": {
    "multicast": {"enabled": false}
  }
}
```

---

## 🎨 Инструменты визуализации

### 1. `launch_rviz.sh` - RViz2 с Zenoh

**Назначение:** Запуск RViz2 для визуализации 3D модели робота, карты, траекторий

**Использование:**
```bash
cd local_test
./launch_rviz.sh
```

**Что делает:**
- Подключается к роботу через Zenoh
- Источник ROS 2 Humble
- Запускает RViz2 с доступом ко всем топикам робота

**Доступные данные:**
- `/map` - RTAB-Map карта
- `/scan` - LiDAR сканы
- `/camera/*` - Изображения с OAK-D камеры
- `/tf`, `/tf_static` - TF трансформации
- `/odom` - Одометрия
- `/detections` - AprilTag детекции

**Пример конфигурации RViz2:**
- Add → RobotModel (для 3D модели)
- Add → TF (для фреймов)
- Add → LaserScan (topic: `/scan`)
- Add → Map (topic: `/map`)
- Add → Camera (topic: `/camera/rgb/image_raw`)

---

### 2. `view_camera.sh` - Просмотр видео с камеры

**Назначение:** Запуск rqt_image_view для просмотра изображений с OAK-D камеры

**Использование:**
```bash
cd local_test
./view_camera.sh
```

**Что делает:**
1. Подключается к Vision Pi через Zenoh
2. Показывает список доступных топиков с изображениями
3. Запускает rqt_image_view

**Доступные топики:**
- `/camera/rgb/image_raw/compressed` - RGB изображение (сжатое JPEG)
- `/camera/depth/image_rect_raw/compressedDepth` - Карта глубины (PNG)
- `/camera/color/image_raw` - Несжатое RGB (если republish включён)

**Выбор топика в rqt_image_view:**
- Dropdown menu сверху → выбрать топик
- Рекомендуется: `/camera/rgb/image_raw/compressed` (меньше трафика)

---

### 3. `visualize_apriltag.py` - Визуализация AprilTag с наложением

**Назначение:** Python нода для визуализации AprilTag детекций с рамками и данными

**Использование:**
```bash
cd local_test
./visualize_apriltag.sh
```

**Или напрямую:**
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI="$(pwd)/zenoh_client_config.json5"
python3 visualize_apriltag.py
```

**Что показывает:**
- 🟢 **Зелёные рамки** вокруг обнаруженных маркеров
- 🔴 **Красные точки** на углах маркера
- 🔵 **Синий круг** в центре маркера
- **Текст с данными:**
  - TAG ID (номер маркера)
  - Quality (decision_margin - чем выше, тем надёжнее)
  - Hamming distance (ошибки: 0 = Perfect)
- **Счётчик детекций** в верхнем левом углу

**Подписки:**
- `/camera/rgb/image_raw/compressed` - Изображение
- `/detections` - AprilTag детекции (apriltag_msgs/AprilTagDetectionArray)

**Горячие клавиши:**
- `q` или `ESC` - выход
- `s` - сохранить текущий кадр как screenshot

**Зависимости:**
```bash
pip install opencv-python numpy
sudo apt install python3-cv-bridge
```

---

### 4. `visualize_apriltag_v2.py` - Улучшенная визуализация

**Назначение:** Расширенная версия с дополнительными данными (pose, расстояние, углы)

**Отличия от v1:**
- Показывает расстояние до маркера (если доступно pose)
- Показывает углы поворота (roll, pitch, yaw)
- Цветовое кодирование по качеству детекции
- История детекций (FPS counter)

**Использование:** Аналогично `visualize_apriltag.py`

---

## 📊 Инструменты мониторинга

### 5. `monitor_apriltag.sh` - Мониторинг AprilTag в терминале

**Назначение:** Мониторинг AprilTag детекций в текстовом режиме (без GUI)

**Использование:**
```bash
cd local_test
./monitor_apriltag.sh
```

**Что показывает:**
```
🎯 TAG 5 | Quality: 142.3 | Dist: 1.23m (0.45, -0.12, 1.18)
🎯 TAG 12 | Quality: 98.7 | Dist: 2.04m (1.20, 0.35, 1.65)
```

**Данные:**
- **TAG ID** - номер обнаруженного маркера
- **Quality** - decision_margin (надёжность детекции)
- **Dist** - расстояние до маркера в метрах
- **(x, y, z)** - координаты в системе камеры

**Применение:**
- Быстрая проверка работы AprilTag детектора
- Мониторинг без GUI (через SSH)
- Проверка дальности и качества детекции

---

## 🧪 Тестовые файлы (для разработки)

### `auto_test.sh`
Автоматический тест основных компонентов системы:
- Проверка доступности топиков
- Проверка TF трансформаций
- Проверка публикации данных

### `quick_test.sh`
Быстрая проверка состояния робота (ping, ROS topics, services)

### `fake_lidar_publisher.py`
Фейковый LiDAR для тестирования RTAB-Map без реального оборудования

### `test_rtabmap_2d_lidar.launch.py`
Launch файл для тестирования RTAB-Map с 2D LiDAR

---

## 🔧 Troubleshooting

> **Полное руководство:** `.agents/skills/zenoh-dev-setup/SKILL.md`

### ⚠️ КРИТИЧНО: 5 правил Zenoh подключения

1. **Переменная `ZENOH_SESSION_CONFIG_URI`** — НЕ `ZENOH_CONFIG` (rmw_zenoh_cpp игнорирует `ZENOH_CONFIG`)
2. **Mode: "peer"** — НЕ "client" (client не видит топики через локальный роутер)
3. **Namespace `robots/RBXU100001`** — должен совпадать с роботом (без namespace видны только `/parameter_events`, `/rosout`)
4. **`#iface=wlp1s0`** — указать WiFi интерфейс в конфиге роутера (проверить: `ip link show`)
5. **`ros2 daemon stop`** — сбросить кэш daemon при смене конфига или перезапуске роутера

### Проблема: "No topics found"

**Причина:** Zenoh не может подключиться к роботу

**Решение:**
```bash
# Проверьте что роботы доступны в сети
ping 10.1.1.10  # Main Pi
ping 10.1.1.21  # Vision Pi

# Проверьте что Zenoh router запущен на Pi
ssh ros2@10.1.1.21 "docker ps | grep zenoh"
```

### Проблема: "RMW_IMPLEMENTATION not found"

**Причина:** Zenoh middleware не установлен

**Решение:**
```bash
sudo apt install ros-humble-rmw-zenoh-cpp
```

### Проблема: Изображение не отображается

**Причина:** Камера не публикует или топик неправильный

**Решение:**
```bash
# Проверьте список топиков
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI="$(pwd)/zenoh_client_config.json5"
ros2 topic list | grep image

# Проверьте публикацию
ros2 topic hz /camera/rgb/image_raw/compressed
```

### Проблема: AprilTag не обнаруживаются

**Причина:** AprilTag нода не запущена на Vision Pi

**Решение:**
```bash
ssh ros2@10.1.1.21 "docker ps | grep apriltag"
# Если нет - запустить:
ssh ros2@10.1.1.21 "cd ~/rob_box_project/docker/vision && docker-compose up -d apriltag"
```

---

## 📚 Дополнительная информация

### Архитектура Zenoh

```
Local Machine (client)
    ↓ tcp/10.1.1.21:7447
Vision Pi (zenoh-router)
    ↓ peers
Main Pi (zenoh-router)
```

### Zenoh режимы
- **client** - Local machine (подключается к роутерам)
- **peer** - Main Pi (может работать автономно)
- **peer** - Vision Pi (может работать автономно)

### Полезные команды

```bash
# Список топиков
ros2 topic list

# Частота публикации
ros2 topic hz /camera/rgb/image_raw/compressed

# Эхо топика
ros2 topic echo /detections

# Информация о топике
ros2 topic info /scan

# Список TF фреймов
ros2 run tf2_tools view_frames
```

---

## 🔗 Связанные документы

- [Zenoh Configuration](../docs/architecture/ZENOH_SETUP.md)
- [AprilTag Setup](../docs/guides/APRILTAG_SETUP.md)
- [Camera Configuration](../docs/guides/OAK_D_SETUP.md)
- [RViz2 Visualization](../docs/guides/RVIZ_SETUP.md)

---

**Создано:** October 24, 2025  
**Автор:** Rob Box Project Team  
**Версия:** 1.0.0
