# Исправление проблемы отключения OAK-D камеры

## Проблема

OAK-D Lite камера отваливается через некоторое время (обычно 8+ часов) с ошибками:
```
[ERROR] [camera]: No data on logger queue!
[ERROR] [camera]: Camera diagnostics error: Communication exception - possible device error/misconfiguration. 
Original message 'Couldn't read data from stream: 'sys_logger_queue' (X_LINK_ERROR)'
```

## Причины

1. **USB autosuspend** - Linux автоматически переводит USB устройства в режим энергосбережения
2. **USB buffer overflow** - буферы USB переполняются при длительной работе
3. **Thermal throttling** - перегрев Raspberry Pi влияет на стабильность USB
4. **Device watchdog timeout** - внутренний watchdog камеры срабатывает при потере связи

## Решение

### 1. USB Power Management

Создан скрипт `setup_usb_power.sh`, который:
- Отключает USB autosuspend для устройств Movidius (03e7)
- Устанавливает power/control в режим "on" (всегда включено)
- Предотвращает автоматическое отключение USB устройств

### 2. Camera Health Watchdog

Создан скрипт `watchdog.sh`, который:
- Мониторит логи контейнера oak-d каждые 30 секунд
- Обнаруживает ошибки X_LINK_ERROR и "No data on logger queue"
- Автоматически перезапускает контейнер после 5 ошибок подряд
- Логирует все действия в `/tmp/oak-d-watchdog.log`

### 3. Оптимизация USB параметров

Добавлены параметры в `oak_d_config.yaml`:
- `i_usb_chunk_kb: 64` - уменьшенный размер USB chunk (по умолчанию 256)
- `i_pipeline_dump: ""` - отключен pipeline dump
- `i_calibration_dump: false` - отключен calibration dump

## Установка

### На Vision Pi (ros2@10.1.1.21)

```bash
# 1. Перейти в директорию проекта
cd ~/rob_box_project/docker/vision

# 2. Обновить код из репозитория
git pull origin main

# 3. Установить watchdog service
cd scripts/oak-d
./install_watchdog.sh

# 4. Перезапустить контейнер oak-d для применения изменений
cd ~/rob_box_project/docker/vision
./update_and_restart.sh
```

### Проверка работы

```bash
# Статус watchdog сервиса
sudo systemctl status oak-d-watchdog

# Просмотр логов watchdog
sudo journalctl -u oak-d-watchdog -f

# Или просмотр файла логов
tail -f /tmp/oak-d-watchdog.log

# Логи контейнера камеры
docker logs oak-d -f
```

## Управление Watchdog

```bash
# Запустить
sudo systemctl start oak-d-watchdog

# Остановить
sudo systemctl stop oak-d-watchdog

# Перезапустить
sudo systemctl restart oak-d-watchdog

# Включить автозапуск
sudo systemctl enable oak-d-watchdog

# Отключить автозапуск
sudo systemctl disable oak-d-watchdog
```

## Удаление (если нужно откатить изменения)

```bash
# Остановить и отключить watchdog
sudo systemctl stop oak-d-watchdog
sudo systemctl disable oak-d-watchdog

# Удалить service файл
sudo rm /etc/systemd/system/oak-d-watchdog.service

# Перезагрузить systemd
sudo systemctl daemon-reload
```

## Ожидаемый результат

- ✅ USB устройство не уходит в autosuspend
- ✅ При возникновении X_LINK_ERROR контейнер автоматически перезапускается
- ✅ Камера работает стабильно 24/7 без ручного вмешательства
- ✅ Все события логируются для анализа

## Дополнительные меры (если проблема сохраняется)

### 1. Проверка USB кабеля
- Используйте качественный USB 3.0 кабель длиной не более 1 метра
- Проверьте надёжность соединения

### 2. Охлаждение Raspberry Pi
```bash
# Проверить температуру
vcgencmd measure_temp

# Должно быть < 70°C. Если выше - добавить радиатор или вентилятор
```

### 3. Питание USB
```bash
# Проверить напряжение USB
vcgencmd get_throttled

# Если 0x0 - всё OK, иначе проблемы с питанием
```

### 4. Отключение USB autosuspend на уровне системы (хост)
```bash
# На Vision Pi добавить в /etc/rc.local перед exit 0:
echo -1 > /sys/module/usbcore/parameters/autosuspend
```

## Логи и отладка

### Типичные записи в логе watchdog при нормальной работе:
```
[2025-11-11 10:00:00] 🐕 OAK-D Watchdog запущен
[2025-11-11 10:00:30] ✅ Контейнер работает нормально
```

### При обнаружении проблемы:
```
[2025-11-11 15:30:00] ❌ Обнаружена ошибка X_LINK_ERROR (1/5)
[2025-11-11 15:30:30] ❌ Обнаружена ошибка X_LINK_ERROR (2/5)
...
[2025-11-11 15:32:00] 🔄 КРИТИЧНО: 5 ошибок подряд - перезапуск контейнера
[2025-11-11 15:32:05] ✅ Контейнер перезапущен
```

## Файлы изменённые в этом исправлении

1. `docker/vision/scripts/oak-d/setup_usb_power.sh` - новый файл
2. `docker/vision/scripts/oak-d/watchdog.sh` - новый файл
3. `docker/vision/scripts/oak-d/oak-d-watchdog.service` - новый файл
4. `docker/vision/scripts/oak-d/install_watchdog.sh` - новый файл
5. `docker/vision/scripts/oak-d/start_oak_d.sh` - обновлён (добавлен вызов setup_usb_power.sh)
6. `docker/vision/config/oak-d/oak_d_config.yaml` - обновлён (добавлены USB параметры)
