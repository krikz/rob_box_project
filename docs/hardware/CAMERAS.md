# Camera System Configuration

## Overview

Rob Box использует **двухкамерную систему** для различных задач восприятия и навигации.

## Camera 1: OAK-D Stereo Camera

### Описание
- **Модель:** OAK-D (OpenCV AI Kit with Depth)
- **Производитель:** Luxonis
- **Тип:** RGB-D стерео камера с AI ускорителем
- **Расположение:** Передняя часть робота
- **Назначение:** Основная камера для навигации, SLAM, детекции препятствий

### Характеристики

**Аппаратные:**
- RGB сенсор: 4K (используется 720p для экономии ресурсов)
- Стерео глубина: до 10м
- Встроенный IMU (акселерометр + гироскоп)
- AI ускоритель: Intel Myriad X VPU
- Интерфейс: USB 3.0

**Текущая конфигурация (оптимизирована для Raspberry Pi):**
- RGB разрешение: 720p (1280x720)
- Стерео разрешение: 640x400
- FPS: 5 (снижено с 10 для экономии CPU/bandwidth)
- Neural Network: отключена (экономия RAM)
- IMU: включен (необходим для ICP odometry в RTAB-Map)

### ROS 2 Topics

**Namespace:** `/camera`

#### Основные топики:
```
/camera/rgb/image_raw              # RGB изображение (sensor_msgs/Image)
/camera/rgb/image_raw/compressed   # Сжатое RGB (sensor_msgs/CompressedImage)
/camera/rgb/camera_info            # Калибровка RGB камеры

/camera/stereo/image_raw           # Стерео depth (sensor_msgs/Image)
/camera/stereo/image_raw/compressed # Сжатая глубина
/camera/stereo/camera_info         # Калибровка стерео

/camera/imu/data                   # IMU данные (sensor_msgs/Imu)
```

#### Дополнительные топики (если NN включена):
```
/camera/nn/spatial_detections      # 3D детекции объектов
```

### Конфигурационные файлы

**Основная конфигурация:**
```
docker/vision/config/oak-d/oak_d_config.yaml
```

**Launch файлы:**
```
docker/vision/oak-d/launch/oakd_with_apriltag.launch.py  # OAK-D + AprilTag
docker/vision/oak-d/launch/oakd_apriltag_only.launch.py  # Только AprilTag
```

**Docker образ:**
```
ghcr.io/krikz/rob_box:oak-d-humble-latest
```

### Использование в системе

1. **RTAB-Map SLAM:** 
   - Использует `/camera/stereo/image_raw` для построения карты
   - Использует `/camera/imu/data` для ICP odometry

2. **AprilTag Detection:**
   - Использует `/camera/rgb/image_raw` для детекции маркеров
   - Публикует `/tf` трансформации для локализации

3. **Визуализация:**
   - `/camera/rgb/image_raw/compressed` для удаленного просмотра через web

### Настройка производительности

**Текущие ограничения (для Raspberry Pi 4):**
```yaml
i_rgb_resolution: "720p"    # Снижено с 1080p
i_stereo_width: 640         # Снижено с 1280
i_stereo_height: 400        # Снижено с 720
i_fps: 5.0                  # Снижено с 10 FPS
i_max_q_size: 1             # Минимальная очередь (экономия RAM)
```

**Для более мощного железа можно увеличить:**
```yaml
i_rgb_resolution: "1080p"
i_stereo_width: 1280
i_stereo_height: 720
i_fps: 10.0
```

---

## Camera 2: Ceiling USB Camera

### Описание
- **Тип:** Generic USB webcam
- **Расположение:** Верхняя часть робота (направлена вниз)
- **Назначение:** Мониторинг груза, обзор сверху

### Характеристики

**Аппаратные:**
- Разрешение: до 1280x720
- FPS: 10
- Интерфейс: USB 2.0/3.0
- Устройство: `/dev/video0`

**Текущая конфигурация:**
- Разрешение: 1280x720
- FPS: 10
- Pixel format: YUYV
- Auto exposure: включен
- Auto focus: выключен

### ROS 2 Topics

**Namespace:** `/ceiling_camera`

#### Топики:
```
/ceiling_camera/image_raw              # USB камера изображение (sensor_msgs/Image)
/ceiling_camera/image_raw/compressed   # Сжатое изображение
/ceiling_camera/camera_info            # Калибровка (default/uncalibrated)
```

### Конфигурационные файлы

**Основная конфигурация:**
```
docker/vision/config/ceiling-camera/camera_params.yaml
```

**Startup скрипт:**
```
docker/vision/scripts/ceiling-camera/start_ceiling_camera.sh
```

**Docker образ:**
```
ghcr.io/krikz/rob_box:ceiling-camera-humble-latest
```

### Использование в системе

1. **Мониторинг груза:**
   - Визуальная проверка наличия и состояния груза
   - Детекция смещения груза во время движения

2. **Визуализация:**
   - `/ceiling_camera/image_raw/compressed` для удаленного просмотра

3. **Опциональные задачи:**
   - Детекция QR-кодов на грузе
   - Инспекция верхней части окружения

### Примечания

**Опциональность:**
- Камера является опциональной
- Система работает нормально без нее
- Если `/dev/video0` не найдено, контейнер завершится с ошибкой (ожидаемое поведение)

**Калибровка:**
- По умолчанию используется uncalibrated camera info
- Для точных измерений необходима калибрация через `camera_calibration` package

---

## Общая архитектура

### Топики в системе

```
Vision Pi:
  ├── OAK-D Container
  │   ├── /camera/rgb/image_raw
  │   ├── /camera/stereo/image_raw
  │   └── /camera/imu/data
  │
  └── Ceiling Camera Container
      └── /ceiling_camera/image_raw

Main Pi (через Zenoh):
  └── RTAB-Map
      ├── Подписан на /camera/stereo/image_raw
      └── Публикует /rtabmap/odom_rgbd_image
```

### Network Bandwidth Considerations

**При 5 FPS:**
- OAK-D RGB (720p compressed): ~100-150 KB/frame = 0.5-0.75 MB/s
- OAK-D Stereo (640x400): ~80-120 KB/frame = 0.4-0.6 MB/s
- Ceiling Camera (720p compressed): ~80-100 KB/frame = 0.8-1.0 MB/s
- **Total:** ~1.7-2.35 MB/s

**Ethernet (1 Gbps):** Комфортный запас для передачи между Pi

### TF Frames

```
base_link
  └── camera_link (OAK-D физическая позиция)
       ├── camera_rgb_optical_frame (RGB камера)
       └── camera_stereo_optical_frame (Stereo камера)

  └── ceiling_camera_link (USB камера физическая позиция)
       └── ceiling_camera_optical_frame (USB камера)
```

**Определены в:** `src/rob_box_description/urdf/rob_box.xacro`

---

## Troubleshooting

### OAK-D не запускается

**Симптомы:**
- Контейнер падает при старте
- Ошибка `X_LINK_ERROR`

**Решения:**
1. Проверить USB подключение: `lsusb | grep 03e7`
2. Проверить USB 3.0 режим: `i_usb_speed: "SUPER"`
3. Отключить диагностику: `i_enable_diagnostics: false`
4. Перезагрузить Vision Pi

### Ceiling Camera не найдена

**Симптомы:**
- Контейнер завершается с ошибкой
- `ERROR: /dev/video0 not found`

**Решения:**
1. Проверить подключение USB камеры
2. Проверить устройство: `ls -l /dev/video*`
3. Если камера не нужна - нормально, просто игнорируйте

### Низкий FPS

**Симптомы:**
- Реальный FPS меньше заданного
- Задержки в изображении

**Решения:**
1. Снизить разрешение
2. Снизить FPS в конфигурации
3. Проверить загрузку CPU: `htop`
4. Проверить bandwidth: `iftop -i eth0`

### Темное изображение

**Для OAK-D:**
- Проверить освещение помещения
- OAK-D имеет встроенный auto exposure

**Для Ceiling Camera:**
```yaml
autoexposure: true  # Убедитесь что включен
```

---

## Дополнительная информация

**Документация OAK-D:**
- https://docs.luxonis.com/
- https://github.com/luxonis/depthai-ros

**Калибрация камер:**
- http://wiki.ros.org/camera_calibration
- Для OAK-D - уже откалибрована на заводе

**Image Transport:**
- http://wiki.ros.org/image_transport
- Автоматическое сжатие для экономии bandwidth

---

*Last Updated: October 29, 2025*
*Maintainer: Rob Box Project Team*
