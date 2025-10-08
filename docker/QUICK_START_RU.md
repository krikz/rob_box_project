# Быстрый старт - Оптимизация OAK-D для Raspberry Pi

## Что было сделано

### ✅ Снижена нагрузка на CPU
- RGB разрешение: 1080p → 720p (-43%)
- FPS: 10 → 5 (-50%)
- Stereo разрешение: 1280x720 → 640x400 (-60%)
- Отключены: IMU, preview, left/right камеры
- Features в RTAB-Map: 1000 → 400 (-60%)

### ✅ Оптимизирован сетевой трафик
- Включено JPEG сжатие для RGB (качество 80)
- Включено PNG сжатие для depth (уровень 3)
- CycloneDDS буферы уменьшены с 10MB до 2-4MB
- Ожидаемое снижение bandwidth: ~85%

### ✅ Оптимизирована память
- RTAB-Map STM: 10 локаций (было неограничено)
- Memory threshold: 50 MB
- Grid range: 5 метров (было неограничено)
- Queue size: 5 (минимум)

## Основные изменения в файлах

### 1. `docker/vision/config/oak_d_config.yaml`
- Разрешение и FPS снижены
- Сжатие включено
- Ненужные потоки отключены

### 2. `docker/vision/config/cyclonedds.xml`
- Буферы оптимизированы для Pi
- Discovery interval увеличен до 5s
- Throttling настроен

### 3. `docker/main/config/cyclonedds.xml`
- Буферы приема увеличены для RTAB-Map
- Синхронизация с vision Pi

### 4. `docker/main/config/rtabmap_config.yaml` (НОВЫЙ)
- Полная конфигурация RTAB-Map
- Оптимизация под Raspberry Pi
- Настроены топики от OAK-D

### 5. `docker/vision/config/start_oak_d.sh`
- Добавлены переменные сжатия изображений

## Как применить изменения

### На Raspberry Pi #1 (с камерой):
```bash
cd docker/vision
docker-compose down
docker-compose build
docker-compose up -d

# Проверить логи
docker logs -f oak-d
```

### На Raspberry Pi #2 (с RTAB-Map):
```bash
cd docker/main
docker-compose down

# Запустить с новой конфигурацией
docker-compose up -d

# Проверить логи
docker logs -f rtabmap
```

## Быстрая проверка

### Проверить топики камеры:
```bash
# На любом Pi
ros2 topic list | grep oak

# Должны увидеть:
# /oak/rgb/image_raw/compressed
# /oak/rgb/camera_info
# /oak/stereo/image_raw/compressedDepth
# /oak/stereo/camera_info
```

### Проверить FPS:
```bash
ros2 topic hz /oak/rgb/image_raw/compressed
# Должно быть ~5 Hz
```

### Проверить bandwidth:
```bash
ros2 topic bw /oak/rgb/image_raw/compressed
# Должно быть ~1-2 MB/s (вместо 10-15 MB/s)
```

### Проверить CPU:
```bash
htop
# depthai процесс должен быть 30-45% (было 85-95%)
# rtabmap процесс должен быть 40-60% (было 80-100%)
```

## Если нужна дополнительная оптимизация

### Вариант 1: Еще снизить FPS
В `oak_d_config.yaml`:
```yaml
i_fps: 3.0  # вместо 5.0
```

### Вариант 2: Уменьшить качество JPEG
В `start_oak_d.sh`:
```bash
export COMPRESSED_IMAGE_TRANSPORT_JPEG_QUALITY=60  # вместо 80
```

### Вариант 3: Отключить grid mapping
В `rtabmap_config.yaml`:
```yaml
Grid/FromDepth: "false"
```

## Если качество недостаточное

### Вариант 1: Увеличить features
В `rtabmap_config.yaml`:
```yaml
Vis/MaxFeatures: "600"  # вместо 400
Kp/MaxFeatures: "300"   # вместо 200
```

### Вариант 2: Повысить качество JPEG
В `start_oak_d.sh`:
```bash
export COMPRESSED_IMAGE_TRANSPORT_JPEG_QUALITY=90  # вместо 80
```

## Важно!

1. **Перезапускайте контейнеры** после изменения конфигов
2. **Проверяйте логи** на ошибки
3. **Мониторьте CPU** и bandwidth во время работы
4. **ROS_DOMAIN_ID=0** должен быть одинаковым на обоих Pi
5. **CycloneDDS конфиги** должны быть синхронизированы

## Полная документация

См. `OPTIMIZATION_README.md` для детальной информации.
