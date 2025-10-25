# AprilTag Integration Migration Guide

## Обзор изменений

**Дата:** 2025-10-25
**Автор:** GitHub Copilot Agent
**Версия:** 1.0.0

### Что изменилось?

Детекция AprilTag теперь интегрирована в контейнер OAK-D вместо отдельного контейнера `apriltag`. Это снижает потребление ресурсов Vision Pi и упрощает архитектуру системы.

### Причина изменений

1. **Проблема**: Отдельный контейнер `apriltag` значительно потреблял ресурсы Raspberry Pi Vision
2. **Исследование**: OAK-D Lite использует Intel Movidius Myriad X (RVC2), который выполняет AprilTag detection на HOST CPU
3. **Решение**: Объединить оба процесса (camera + apriltag) в одном контейнере для оптимизации

### Технические детали

#### Архитектура "ДО"

```
Vision Pi
├── zenoh-router
├── oak-d (camera only)
└── apriltag (separate container)
    └── Subscribes to /camera/rgb/image_raw
    └── Publishes to /apriltag/detections
```

**Ресурсы:**
- 2 Docker контейнера
- 2 отдельных ROS 2 процесса
- Дополнительные накладные расходы на IPC

#### Архитектура "ПОСЛЕ"

```
Vision Pi
├── zenoh-router
└── oak-d (camera + apriltag integrated)
    ├── camera_node → /camera/rgb/image_raw
    └── apriltag_node → /apriltag/detections
```

**Ресурсы:**
- 1 Docker контейнер
- 2 ROS 2 ноды в одном процессе
- Снижение накладных расходов

---

## Миграция

### Шаг 1: Обновление кода

```bash
cd ~/rob_box_project
git checkout develop
git pull origin develop
```

### Шаг 2: Остановка старых контейнеров

```bash
cd ~/rob_box_project/docker/vision
docker-compose down
```

### Шаг 3: Удаление старого образа apriltag (опционально)

```bash
docker rmi ghcr.io/krikz/rob_box:apriltag-humble-latest
docker rmi ghcr.io/krikz/rob_box:apriltag-humble-dev
docker rmi ghcr.io/krikz/rob_box:apriltag-humble-test
```

### Шаг 4: Pull нового образа OAK-D

```bash
docker-compose pull oak-d
```

### Шаг 5: Запуск обновленной системы

```bash
docker-compose up -d
```

### Шаг 6: Проверка работы

```bash
# Проверяем что OAK-D контейнер запустился
docker ps | grep oak-d

# Проверяем логи
docker logs oak-d --tail 50

# Проверяем что оба узла работают
docker exec oak-d ros2 node list

# Проверяем топики (должны быть камера И apriltag)
docker exec oak-d ros2 topic list | grep -E "camera|apriltag"

# Ожидаемый вывод:
# /camera/rgb/image_raw
# /camera/rgb/camera_info
# /apriltag/detections
# /apriltag/detections/image
```

---

## Откат (Rollback)

Если что-то пошло не так, можно откатиться к предыдущей версии:

```bash
cd ~/rob_box_project
git checkout <previous-commit>
cd docker/vision
docker-compose down
docker-compose up -d
```

---

## Файлы которые изменились

### Удалены/Устарели

- ❌ `docker/vision/apriltag/` - весь каталог больше не нужен
- ❌ `docker/vision/scripts/apriltag/start_apriltag.sh` - больше не используется
- ❌ Секция `apriltag:` в `docker/vision/docker-compose.yaml`

### Изменены

- ✏️ `docker/vision/oak-d/Dockerfile` - добавлены пакеты apriltag
- ✏️ `docker/vision/scripts/oak-d/start_oak_d.sh` - новый launch файл
- ✏️ `docker/vision/docker-compose.yaml` - удален сервис apriltag
- ✏️ `docker/vision/README.md` - обновлена документация

### Новые файлы

- ✅ `docker/vision/oak-d/launch/oakd_with_apriltag.launch.py` - комбинированный launch

---

## Мониторинг и проверка

### Проверка потребления ресурсов

**ДО (2 контейнера):**
```bash
docker stats oak-d apriltag --no-stream
```

**ПОСЛЕ (1 контейнер):**
```bash
docker stats oak-d --no-stream
```

Ожидаемое снижение:
- RAM: ~200-300 MB экономии (удаление накладных расходов контейнера)
- CPU: Меньшая фрагментация процессорного времени

### Проверка производительности AprilTag

```bash
# Частота публикации топика
ros2 topic hz /apriltag/detections

# Эхо для проверки детекций
ros2 topic echo /apriltag/detections --once
```

### Проверка логов на ошибки

```bash
# Проверяем что нет ошибок связанных с apriltag
docker logs oak-d 2>&1 | grep -i "error\|fail\|apriltag"
```

---

## FAQ

### Q: Изменится ли формат топика `/apriltag/detections`?

**A:** Нет, формат остается тот же самый - `apriltag_msgs/AprilTagDetectionArray`.

### Q: Будут ли работать старые конфигурации?

**A:** Да, конфигурация AprilTag (`/config/apriltag/apriltag_config.yaml`) остается без изменений.

### Q: Нужно ли изменять конфигурацию на Main Pi?

**A:** Нет, Main Pi не затронут этими изменениями. Он продолжит получать `/apriltag/detections` через Zenoh.

### Q: Можно ли вернуть старую архитектуру?

**A:** Да, можно откатиться на предыдущий коммит. Однако рекомендуется использовать новую архитектуру для экономии ресурсов.

### Q: Работает ли это на OAK-D Lite (RVC2)?

**A:** Да, работает. AprilTag detection на RVC2 выполняется на HOST CPU, что и происходит сейчас в контейнере OAK-D.

### Q: А если у меня OAK-D Pro (RVC4)?

**A:** RVC4 устройства могут запускать AprilTag на встроенном octacore CPU устройства, что еще более эффективно. В будущем можно будет настроить on-device detection для RVC4.

---

## Дополнительные ресурсы

- [DepthAI ROS Driver Documentation](https://docs.luxonis.com/software/ros/depthai-ros/driver/)
- [AprilTag ROS 2 Documentation](https://docs.ros.org/en/ros2_packages/humble/api/apriltag_ros/)
- [OAK-D Lite Hardware Specs](../docs/architecture/HARDWARE.md)

---

## Контакты и поддержка

Если возникли проблемы с миграцией:

1. Проверьте логи: `docker logs oak-d`
2. Создайте Issue в GitHub с описанием проблемы и логами
3. Используйте rollback процедуру для временного возврата к старой версии

---

**Важно:** Перед применением на production роботе, протестируйте изменения на development/staging системе!
