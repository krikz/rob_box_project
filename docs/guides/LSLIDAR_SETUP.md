# Руководство по настройке LSLIDAR N10

## Подключение оборудования

### Технические характеристики LSLIDAR N10
- **Модель**: N10 Navigation & Obstacle Avoidance LiDAR
- **Тип**: 2D 360° однолучевой лидар
- **Дальность**: 0.02м - 12м @ 70% отражение
- **Точность**: ±3см (0-6м), ±4.5см (6-12м)
- **Угол обзора**: 360°
- **Разрешение**: 0.48° - 0.96°
- **Частота данных**: 4500 точек/сек
- **Размеры**: φ52 × 36.1мм
- **Вес**: 60г
- **Защита**: IPX-4

### Физическое подключение

**Подключение к Vision Pi:**
1. **USB/Serial кабель**: Подключите LSLIDAR к Vision Pi через USB
2. **Питание**: 5В питание для лидара
3. **Путь устройства**: Обычно определяется как `/dev/ttyUSB0`

**Проверка подключения:**
```bash
# На Vision Pi проверьте USB устройства
ls -la /dev/ttyUSB*

# Должно показать:
# /dev/ttyUSB0  (устройство лидара)

# Проверьте права доступа
sudo chmod 666 /dev/ttyUSB0
```

### Физический монтаж

Установите LSLIDAR в центре верхней части робота:
- **Позиция**: Спереди робота, 0.25м от base_link
- **Высота**: 0.2м над base_link
- **Ориентация**: Выровнен с направлением движения робота (ось X)

Иерархия фреймов URDF:
```
base_link
  └─ lslidar_n10 (позиция: x=0.25м, y=0.0м, z=0.2м)
```

## Настройка ПО

### Docker контейнер

Контейнер уже настроен в `docker/vision/docker-compose.yaml`:

```yaml
lslidar:
  build:
    context: .
    dockerfile: lslidar/Dockerfile
  container_name: lslidar
  network_mode: host  # Требуется для UDP multicast
  privileged: true
```

### Сборка и запуск

**На Vision Pi:**
```bash
cd ~/rob_box_project/docker/vision

# Соберите контейнер лидара
docker-compose build lslidar

# Запустите лидар (остановит если уже работает)
docker-compose up -d lslidar

# Проверьте логи
docker logs lslidar -f
```

Ожидаемый вывод:
```
Zenoh router is ready!
Starting LSLIDAR N10 driver...
[INFO] LSLIDAR N10 connected
[INFO] Publishing scan on /scan topic
[INFO] Scan rate: ~10 Hz
```

### Проверка потока данных

**Проверьте ROS 2 топики:**
```bash
# Список топиков (должен быть /scan)
ros2 topic list | grep scan

# Мониторинг данных сканирования
ros2 topic hz /scan

# Просмотр сообщений
ros2 topic echo /scan --once
```

Ожидаемый вывод:
```
average rate: 10.000
  min: 0.100s max: 0.100s std dev: 0.00100s window: 10

ranges: [12.0, 11.95, ..., 0.35, 0.02]  # 4500 точек
angle_min: -3.14159
angle_max: 3.14159
```

## Интеграция с RTAB-Map

RTAB-Map автоматически получает топик `/scan` для:
- **2D карта сетки**: Обнаружение препятствий для навигации
- **Замыкание петель**: Сопоставление сканов для коррекции дрейфа
- **Локализация**: ICP выравнивание с визуальной одометрией

Конфигурация в `docker/main/config/rtabmap/rtabmap_config.yaml`:
```yaml
subscribe_scan: true
scan_topic: /scan

# Параметры сканирования
Mem/LaserScanNormalK: "0"  # Отключено (2D лидар)
Grid/FromDepth: "true"     # Комбинировать с камерой глубины
Grid/CellSize: "0.05"      # Разрешение сетки 5см
Grid/RangeMax: "5.0"       # Использовать лидар до 5м
```

## Устранение неисправностей

### Лидар не обнаружен

**Проверьте USB подключение:**
```bash
# Список USB устройств
ls -la /dev/ttyUSB*

# Должно показать /dev/ttyUSB0 или /dev/ttyUSB1
# Если не найдено, проверьте dmesg на ошибки USB
dmesg | grep ttyUSB
```

**Проверьте права доступа:**
```bash
# Дайте доступ к USB устройству
sudo chmod 666 /dev/ttyUSB0

# Добавьте пользователя в группу dialout (постоянное решение)
sudo usermod -a -G dialout $USER
# Затем выйдите и войдите снова
```

### Нет топика /scan

**Проверьте контейнер:**
```bash
# Статус контейнера
docker ps -a | grep lslidar

# Полные логи
docker logs lslidar

# Перезапустите контейнер
docker restart lslidar
```

**Проверьте соединение Zenoh:**
```bash
# Убедитесь что Zenoh router работает
docker logs zenoh-router-vision | grep "listening"

# Проверьте обнаружение ROS 2
ros2 node list | grep lslidar
```

### Низкая частота сканирования

**Проверьте нагрузку системы:**
```bash
# Использование CPU
htop

# Использование памяти (должно быть <80%)
free -h
```

**Уменьшите другие процессы:**
```bash
# Временно остановите камеру
docker stop oak-d

# Проверьте улучшение частоты сканирования
ros2 topic hz /scan
```

### Проблемы с качеством данных сканирования

**Проверьте расположение лидара:**
- Убедитесь, что нет препятствий, блокирующих обзор 360°
- Высота установки должна быть >20см от земли
- Держите подальше от отражающих поверхностей (стекло, зеркала)

**Настройте фильтрацию в `lidar_config.yaml`:**
```yaml
min_range: 0.05  # Увеличьте если много шума рядом с лидаром
max_range: 8.0   # Уменьшите если дальние данные ненадежны
```

## Производительность

**Использование ресурсов:**
- CPU: ~5-10% (lslidar_driver)
- RAM: ~200МБ
- Сеть: ~1 Мбит/с UDP

**Общая нагрузка системы (Vision Pi):**
- Камера OAK-D: 4-5ГБ RAM
- LSLIDAR: 200МБ RAM
- Всего: ~5.2ГБ / 7.8ГБ (67% - хорошо!)

**Интеграция сканирования с RTAB-Map:**
- Визуальная одометрия: RGB-D камера (основная)
- Сопоставление сканов: Лидар (вторичная, замыкание петель)
- Карта сетки: Объединенная глубина + лазер для навигации
- Слияние одометрии: Визуальная + ICP для точности

## Связанные файлы

- **URDF**: `src/rob_box_description/urdf/rob_box.xacro` (строка 268)
- **Docker**: `docker/vision/lslidar/Dockerfile`
- **Конфиг**: `docker/vision/config/lslidar/lslidar_config.yaml`
- **RTAB-Map**: `docker/main/config/rtabmap/rtabmap_config.yaml`

