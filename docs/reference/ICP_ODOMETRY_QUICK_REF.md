# RTAB-Map ICP Одометрия - Краткий Справочник

## Быстрая диагностика

### Проверка логов

```bash
# Следить за логами ICP одометрии в реальном времени
ssh ros2@10.1.1.20
docker logs -f rtabmap 2>&1 | grep icp_odometry

# Подсчёт ошибок за последнюю минуту
docker logs --since 1m rtabmap 2>&1 | grep "Registration failed" | wc -l
```

### Типичные ошибки и их значение

| Ошибка | Причина | Решение |
|--------|---------|---------|
| `limit out of bounds: rot: X/0.78` | Поворот превышает лимит | ✅ Увеличен до 1.0 рад |
| `limit out of bounds: tr: X/0.2` | Смещение превышает лимит | ✅ Увеличено до 0.5м |
| `corrRatio=0.01/0.10` | Мало совпадающих точек | ✅ Снижен порог до 0.2 |
| `ICP correction too large` | Комбинированное превышение | ✅ Оба лимита увеличены |

## Ключевые параметры

| Параметр | Старое значение | Новое значение | Эффект |
|----------|----------------|----------------|--------|
| `Icp/MaxTranslation` | 0.2м | **0.5м** | Позволяет быстрое движение |
| `Icp/MaxRotation` | 0.78 рад (~45°) | **1.0 рад (~57°)** | Позволяет резкие повороты |
| `Icp/CorrespondenceRatio` | 0.4 (40%) | **0.2 (20%)** | Менее строгий порог |

## Мониторинг метрик

### Хорошие показатели
```
[INFO] Odom: ratio=0.234567, std dev=0.001234m|0.002345rad, update time=0.005s
```
- **ratio** > 0.2 ✅
- **std dev** < 0.01 ✅
- **update time** < 0.01s ✅

### Плохие показатели
```
[WARN] Registration failed: "Cannot compute transform (corrRatio=0.015957/0.100000)"
[INFO] Odom: ratio=0.015957, std dev=0.000000m|0.000000rad
```
- **ratio** < 0.1 ❌
- **std dev** = 0 (регистрация не удалась) ❌

## Файлы конфигурации

### docker/main/config/rtabmap/rtabmap.yaml
```yaml
icp_odometry:
  ros__parameters:
    Icp/MaxTranslation: "0.5"
    Icp/MaxRotation: "1.0"
    Icp/CorrespondenceRatio: "0.2"
```

### docker/main/docker-compose.yaml
```yaml
args:=--delete_db_on_start
  -p Icp/MaxTranslation:0.5
  -p Icp/MaxRotation:1.0
  -p Icp/CorrespondenceRatio:0.2
```

## Применение изменений

```bash
# Обновить и перезапустить на Main Pi
sshpass -p 'open' ssh ros2@10.1.1.20 \
  'cd ~/rob_box_project/docker/main && ./update_and_restart.sh'

# Проверить, что контейнер запустился
ssh ros2@10.1.1.20 'docker ps | grep rtabmap'
```

## Дополнительная настройка

Если проблемы продолжаются:

```yaml
# Увеличить количество итераций ICP
Icp/Iterations: "50"  # было 30

# Более строгая сходимость
Icp/Epsilon: "0.0001"  # было 0.001

# Более мягкое сопоставление точек
Icp/MaxCorrespondenceDistance: "0.15"  # было 0.1
```

## Полная документация

См. [docs/reference/ICP_ODOMETRY_TUNING.md](./ICP_ODOMETRY_TUNING.md)
