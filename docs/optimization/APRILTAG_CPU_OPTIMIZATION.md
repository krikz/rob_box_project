# AprilTag CPU Optimization Guide

## Проблема

AprilTag детекция потребляет ~169% CPU на Raspberry Pi Vision (1.7 ядра ARM).

## Решение: Оптимизация параметров apriltag_ros

### Ключевые изменения в `docker/vision/config/apriltag/apriltag_config.yaml`

| Параметр | Было | Стало | Эффект |
|----------|------|-------|--------|
| `threads` | 2 | 1 | **~50% снижение CPU** - один поток достаточен для одного потока изображений |
| `decimate` | 1.0 | 2.0 | **~40-50% снижение CPU** - обработка на половинном разрешении |

### Ожидаемый результат

- **CPU usage**: ~169% → **~50-70%** (экономия ~60-70%)
- **Detection quality**: Небольшое снижение точности на дальних дистанциях
- **Detection rate**: Сохраняется на близких дистанциях (до 2-3 метров)

### Компромиссы

#### ✅ Преимущества
- Значительное снижение нагрузки на CPU Raspberry Pi
- Освобождение ресурсов для других задач (SLAM, навигация)
- Снижение тепловыделения и энергопотребления
- Работает "из коробки" без переписывания кода

#### ⚠️ Недостатки
- Снижение точности детекции на дальних расстояниях (>3м)
- Возможно пропускание мелких тегов на расстоянии
- Меньшая точность определения позы на краях изображения

### Дальнейшая оптимизация (опционально)

Если требуется еще большее снижение CPU:

```yaml
detector:
  threads: 1
  decimate: 3.0    # Еще меньше разрешение (но больше потерь точности)
  refine: false    # Отключить уточнение границ (~10% экономии CPU)
```

**Внимание**: `decimate: 3.0` и `refine: false` сильно снизят качество детекции!

### Альтернативные подходы

#### 1. Снижение FPS камеры
Текущий FPS: 5 Hz (уже оптимизирован)

Можно попробовать снизить до 3-4 Hz в `oak_d_config.yaml`:
```yaml
i_fps: 3.0  # Еще меньше нагрузка
```

#### 2. Использование сжатых изображений
```yaml
image_transport: compressed
```
Может помочь если узким местом является передача данных.

#### 3. Миграция на `dai.node.AprilTag` (долгосрочно)
Переход на встроенный AprilTag ноду DepthAI:
- Требует написание custom ROS 2 ноды
- Переносит обработку на CPU камеры
- Полностью освобождает CPU Raspberry Pi

См. исследование в PR: "Document AprilTag hardware acceleration limitations"

## Тестирование

### До оптимизации
```bash
# На Vision Pi
top -p $(pgrep apriltag_node)
# Ожидается: 160-180% CPU
```

### После оптимизации
```bash
# Перезапуск контейнера
cd ~/rob_box_project/docker/vision
./update_and_restart.sh

# Проверка CPU
top -p $(pgrep apriltag_node)
# Ожидается: 50-80% CPU
```

### Проверка качества детекции
```bash
# Проверяем публикацию топика
ros2 topic hz /apriltag/detections

# Проверяем детекции
ros2 topic echo /apriltag/detections --once
```

**Тест в реальных условиях:**
1. Разместить AprilTag на расстоянии 1-2 метра
2. Убедиться что детекция работает стабильно
3. Проверить дальность детекции (должна быть 2-3 метра минимум)

## Мониторинг

Используйте Grafana для отслеживания CPU usage:
```
http://10.1.1.10:3000
Dashboard: Vision Pi Containers
Panel: CPU Usage by Container
```

## Откат изменений

Если оптимизация привела к проблемам:

```bash
cd ~/rob_box_project/docker/vision/config/apriltag
# Восстановить старые значения
sed -i 's/threads: 1/threads: 2/' apriltag_config.yaml
sed -i 's/decimate: 2.0/decimate: 1.0/' apriltag_config.yaml

# Перезапустить
cd ~/rob_box_project/docker/vision
./update_and_restart.sh
```

## Ссылки

- [AprilTag ROS2 Performance Comparison](https://github.com/terranrobotics/apriltag_ros2/blob/master/docs/performance_comparison.md)
- [AprilTag Library Documentation](https://github.com/AprilRobotics/apriltag)
- Исходный анализ: Issue "Используется CPU-based AprilTag вместо NPU на камере"

---

**Дата**: 2025-10-29  
**Автор**: GitHub Copilot  
**Статус**: Implemented - Ready for Testing
