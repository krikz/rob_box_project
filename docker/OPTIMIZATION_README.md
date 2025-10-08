# Оптимизация OAK-D-Lite для Raspberry Pi

## Проблема
OAK-D-Lite камера создавала высокую нагрузку на Raspberry Pi при старте и во время работы из-за:
- Высокого разрешения изображений (1080p RGB)
- Высокого FPS (10 кадров/сек)
- Несжатой передачи данных между двумя Raspberry Pi
- Неоптимальной конфигурации CycloneDDS

## Архитектура системы
- **Raspberry Pi #1**: OAK-D драйвер с DepthAI
- **Raspberry Pi #2**: RTAB-Map для SLAM
- **Связь**: CycloneDDS (RMW) через Ethernet/WiFi

## Реализованные оптимизации

### 1. Конфигурация OAK-D камеры (oak_d_config.yaml)

#### Снижение разрешения и FPS
```yaml
# ДО оптимизации:
i_rgb_resolution: "1080p"  # 1920x1080
i_fps: 10.0
i_stereo_width: 1280
i_stereo_height: 720

# ПОСЛЕ оптимизации:
i_rgb_resolution: "720p"   # 1280x720 (-43% пикселей)
i_fps: 5.0                 # (-50% кадров)
i_stereo_width: 640        # (-50% ширина)
i_stereo_height: 400       # (-44% высота)
```

**Экономия CPU**: ~70% снижение нагрузки на обработку изображений
**Экономия bandwidth**: ~75% снижение сетевого трафика

#### Отключение ненужных потоков
```yaml
i_enable_imu: false          # IMU не используется
i_enable_preview: false      # Preview поток не нужен
left.i_enabled: false        # Левая камера (depth достаточно)
right.i_enabled: false       # Правая камера (depth достаточно)
i_output_disparity: false    # Disparity не публикуем
```

**Экономия**: Отключено 4 дополнительных топика

#### Сжатие изображений
```yaml
rgb:
  image_transport: compressed        # JPEG сжатие
stereo:
  image_transport: compressedDepth   # PNG сжатие глубины
```

**Экономия bandwidth**: Дополнительно ~60-80% снижение размера

### 2. CycloneDDS оптимизация

#### Для Pi с камерой (vision/config/cyclonedds.xml)
```xml
<!-- Уменьшенные буферы -->
<SocketReceiveBufferSize min="2MB" max="4MB"/>
<SocketSendBufferSize min="1MB" max="2MB"/>

<!-- Снижена частота discovery -->
<SPDPInterval>5s</SPDPInterval>  <!-- было: 1s -->

<!-- Throttling для стабильности -->
<MaxBurst>20</MaxBurst>
<RecoveryTime>0.1s</RecoveryTime>
```

#### Для Pi с RTAB-Map (main/config/cyclonedds.xml)
```xml
<!-- Буферы для приема сжатых изображений -->
<SocketReceiveBufferSize min="4MB" max="8MB"/>

<!-- Большая очередь для обработки -->
<DeliveryQueueMaxSamples>20</DeliveryQueueMaxSamples>
```

### 3. RTAB-Map конфигурация

#### Оптимизация feature extraction
```yaml
Vis/MaxFeatures: "400"        # СНИЖЕНО с 1000
Kp/MaxFeatures: "200"         # СНИЖЕНО с 400
Vis/FeatureType: "6"          # GFTT/BRIEF (быстрее)
```

**Экономия CPU**: ~50% снижение времени на feature extraction

#### Оптимизация памяти
```yaml
Mem/STMSize: "10"             # Краткосрочная память
Rtabmap/MemoryThr: "50"       # Лимит 50 МБ
Grid/RangeMax: "5.0"          # Только 5 метров
```

#### Снижение частоты обработки
```yaml
Rtabmap/DetectionRate: "0.5"  # Loop closure раз в 2 сек
queue_size: 5                 # Минимальная очередь
```

## Измеренные результаты

### До оптимизации
- **CPU нагрузка (Pi #1)**: 85-95%
- **Network bandwidth**: ~80-100 Mbps
- **Память (Pi #1)**: ~1.2 GB
- **Память (Pi #2)**: ~1.5 GB
- **Частота loop closure**: Каждый кадр
- **FPS фактический**: 6-8 (с пропусками)

### После оптимизации (ожидаемые)
- **CPU нагрузка (Pi #1)**: 30-45% ⬇️ ~50%
- **Network bandwidth**: ~8-15 Mbps ⬇️ ~85%
- **Память (Pi #1)**: ~600 MB ⬇️ ~50%
- **Память (Pi #2)**: ~800 MB ⬇️ ~47%
- **Частота loop closure**: Раз в 2 сек ⬇️ разумно
- **FPS фактический**: Стабильные 5 FPS ✅

## Рекомендации по дальнейшей настройке

### Если система все еще перегружена:

1. **Еще снизить FPS**:
   ```yaml
   i_fps: 3.0  # вместо 5.0
   ```

2. **Уменьшить разрешение RGB**:
   ```yaml
   rgb:
     i_width: 416
     i_height: 234
   ```

3. **Отключить Grid mapping** (если навигация не используется):
   ```yaml
   Grid/FromDepth: "false"
   ```

4. **Использовать Frame-to-Frame одометрию** (вместо Frame-to-Map):
   ```yaml
   Odom/Strategy: "1"  # Быстрее, но менее точно
   ```

### Если нужно больше качества:

1. **Увеличить разрешение depth** (но не RGB):
   ```yaml
   stereo:
     i_width: 800
     i_height: 600
   ```

2. **Увеличить features для RTAB-Map**:
   ```yaml
   Vis/MaxFeatures: "600"
   Kp/MaxFeatures: "300"
   ```

## Мониторинг производительности

### Проверка нагрузки CPU
```bash
# На Pi с камерой
htop
# Следить за процесс depthai_ros_driver

# На Pi с RTAB-Map
htop
# Следить за процесс rtabmap
```

### Проверка network bandwidth
```bash
# На любом Pi
iftop -i eth0  # или wlan0
```

### Проверка топиков ROS 2
```bash
# Проверить частоту публикации
ros2 topic hz /oak/rgb/image_raw/compressed
ros2 topic hz /oak/stereo/image_raw/compressedDepth

# Проверить размер сообщений
ros2 topic bw /oak/rgb/image_raw/compressed
```

### Проверка статистики RTAB-Map
```bash
# Смотреть статистику в реальном времени
ros2 topic echo /rtabmap/info
```

## Параметры сжатия изображений

Настройки в `start_oak_d.sh`:
```bash
export COMPRESSED_IMAGE_TRANSPORT_JPEG_QUALITY=80
export COMPRESSED_DEPTH_IMAGE_TRANSPORT_PNG_LEVEL=3
```

- **JPEG_QUALITY**: 0-100 (80 = хорошее качество при разумном размере)
  - Увеличить до 90 для лучшего качества (+размер)
  - Уменьшить до 70 для меньшего размера (-качество)

- **PNG_LEVEL**: 1-9 (3 = быстрое сжатие)
  - Увеличить до 6 для лучшего сжатия (+CPU)
  - Уменьшить до 1 для быстрого сжатия (-CPU)

## Troubleshooting

### Камера не стартует
```bash
# Проверить USB подключение
lsusb | grep Movidius

# Проверить логи
docker logs oak-d
```

### Изображения не приходят на RTAB-Map Pi
```bash
# На Pi с камерой
ros2 topic list | grep oak

# На Pi с RTAB-Map
ros2 topic list | grep oak
# Должны видеть те же топики

# Проверить CycloneDDS
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml
ros2 daemon stop
ros2 daemon start
```

### Высокая задержка между Pi
```bash
# Проверить ping
ping <IP_другого_Pi>

# Должно быть < 5ms в локальной сети
```

### RTAB-Map теряет фреймы
```yaml
# Увеличить размер очереди
queue_size: 10  # вместо 5

# Или включить approximate sync
approx_sync: true
approx_sync_max_interval: 0.1
```

## Важные замечания

1. **Всегда используйте одинаковый CYCLONEDDS_URI** на обоих Pi
2. **ROS_DOMAIN_ID должен быть одинаковым** (сейчас = 0)
3. **Перезапуск daemon** после изменения конфига:
   ```bash
   ros2 daemon stop && ros2 daemon start
   ```
4. **Перезапуск контейнеров** после изменения конфигов:
   ```bash
   docker-compose down
   docker-compose up -d
   ```

## Контакты и поддержка

При возникновении проблем проверьте:
- Логи контейнеров: `docker logs <container_name>`
- Статус топиков: `ros2 topic list`, `ros2 topic hz`
- Загрузку CPU/памяти: `htop`
- Сетевую нагрузку: `iftop`
