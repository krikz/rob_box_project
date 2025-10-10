# Скрипты мониторинга питания

## Обзор

В проекте доступны три скрипта для мониторинга питания и USB устройств Raspberry Pi:

### 1. `check_power_status.sh` - Полная проверка состояния

Выполняет единоразовую проверку всех параметров системы.

**Использование:**
```bash
./docker/scripts/check_power_status.sh
```

**Показывает:**
- ✅ Режим питания (USB limit, PSU max current)
- ✅ Троттлинг и undervoltage
- ✅ Напряжение, частота CPU, температура
- ✅ Список USB устройств
- ✅ Использование памяти и диска
- ✅ Сетевые интерфейсы
- ✅ Uptime и load average

**Рекомендуется для:**
- Первичной диагностики проблем с питанием
- Проверки после изменения конфигурации
- Документирования состояния системы

---

### 2. `check_usb_devices.sh` - Детальный анализ USB

Детальная информация о каждом подключенном USB устройстве.

**Использование:**
```bash
./docker/scripts/check_usb_devices.sh
```

**Показывает:**
- ✅ USB Max Current конфигурация
- ✅ USB топология (дерево устройств)
- ✅ Детали каждого устройства:
  - Название, ID, Bus/Device
  - Скорость (USB 2.0/3.0)
  - Максимальное энергопотребление
  - Производитель, серийный номер
- ✅ Проверка специфичных устройств:
  - OAK-D камера (Luxonis)
  - LSLIDAR (USB Serial)
  - USB хабы

**Рекомендуется для:**
- Диагностики проблем с конкретными USB устройствами
- Проверки энергопотребления USB
- Поиска конфликтов USB устройств

---

### 3. `monitor_power_live.sh` - Мониторинг в реальном времени

Непрерывный мониторинг ключевых метрик с автообновлением экрана.

**Использование:**
```bash
# Обновление каждые 2 секунды (по умолчанию)
./docker/scripts/monitor_power_live.sh

# Или задать свой интервал (в секундах)
./docker/scripts/monitor_power_live.sh 5
```

**Показывает:**
- ⚡ Питание (напряжение, USB limit, троттлинг)
- 💻 Система (CPU частота, температура, load average)
- 🧠 Память (использование RAM)
- 🔌 USB устройства (список подключенных)
- 🌐 Сеть (активные интерфейсы с IP)
- 💾 Хранилище (использование диска)

**Цветовые индикаторы:**
- 🟢 Зеленый - всё в норме
- 🟡 Желтый - предупреждение
- 🔴 Красный - проблема требует внимания

**Рекомендуется для:**
- Мониторинга во время стресс-тестов
- Отслеживания поведения под нагрузкой
- Диагностики периодических проблем
- Проверки стабильности питания

**Выход:** Нажмите `Ctrl+C`

---

## Быстрый старт

### На Main Pi (10.1.1.20)

```bash
# SSH подключение
ssh ros2@10.1.1.20

# Перейти в проект
cd /home/ros2/rob_box_project

# Быстрая проверка
./docker/scripts/check_power_status.sh

# Мониторинг в реальном времени
./docker/scripts/monitor_power_live.sh
```

### На Vision Pi (10.1.1.21)

```bash
# SSH подключение
ssh ros2@10.1.1.21

# Перейти в проект
cd /home/ros2/rob_box_project

# Проверка USB (особенно важно для OAK-D и LSLIDAR)
./docker/scripts/check_usb_devices.sh

# Мониторинг USB нагрузки
./docker/scripts/monitor_power_live.sh 1
```

---

## Типичные сценарии использования

### Сценарий 1: Диагностика медленной работы Pi

```bash
# Шаг 1: Проверить общее состояние
./docker/scripts/check_power_status.sh

# Если обнаружен троттлинг или undervoltage:
# → Проверить источник питания
# → Проверить конфигурацию (см. POWER_MANAGEMENT.md)

# Шаг 2: Мониторинг под нагрузкой
./docker/scripts/monitor_power_live.sh

# Запустить стресс-тест в другом терминале
# stress --cpu 4 --timeout 60
```

### Сценарий 2: USB устройство не работает

```bash
# Шаг 1: Проверить подключение и энергопотребление
./docker/scripts/check_usb_devices.sh

# Проверить, что устройство видно в lsusb
# Проверить, что USB max current достаточно

# Шаг 2: Если устройство потребляет много тока
# Включить полный USB режим (если питание позволяет)
sudo nano /boot/firmware/config.txt
# Добавить: usb_max_current_enable=1
sudo reboot
```

### Сценарий 3: Мониторинг стабильности робота

```bash
# Терминал 1: Запуск мониторинга
./docker/scripts/monitor_power_live.sh 2

# Терминал 2: Запуск основной программы
cd docker/main
docker-compose up

# Наблюдать за:
# - Напряжением (должно быть > 4.8V)
# - Температурой (должна быть < 70°C)
# - Троттлингом (должен быть 0x0)
# - Load average (не должен превышать число ядер)
```

### Сценарий 4: Проверка после изменения питания

```bash
# После подключения нового источника питания или HAT

# Шаг 1: Проверить, что Pi видит правильный ток
# Выключить Pi, вынуть SD-карту, включить
# → Появится diagnostic screen с информацией о питании

# Шаг 2: После загрузки ОС проверить конфигурацию
./docker/scripts/check_power_status.sh | grep -A 5 "РЕЖИМ ПИТАНИЯ"

# Шаг 3: Стресс-тест
./docker/scripts/monitor_power_live.sh 1 &
stress --cpu 4 --timeout 60
# Проверить, что нет undervoltage или троттлинга
```

---

## Интерпретация результатов

### Троттлинг (get_throttled)

Значение `0x0` - всё хорошо.

Если не ноль, расшифровка битов:
- `0x1` (bit 0) - **Undervoltage detected** - Напряжение < 4.63V СЕЙЧАС
- `0x2` (bit 1) - **ARM frequency capped** - Частота ограничена СЕЙЧАС
- `0x4` (bit 2) - **Currently throttled** - Троттлинг активен СЕЙЧАС
- `0x8` (bit 3) - **Soft temp limit** - Температурное ограничение СЕЙЧАС

- `0x10000` (bit 16) - Undervoltage **было** с момента загрузки
- `0x20000` (bit 17) - Frequency cap **было** с момента загрузки
- `0x40000` (bit 18) - Throttling **было** с момента загрузки
- `0x80000` (bit 19) - Temp limit **было** с момента загрузки

**Пример**: `0x50000` = `0x10000 | 0x40000` = undervoltage и throttling было в прошлом

### USB Max Current

- `0` (600mA) - Ограниченный режим
  - Суммарно все USB порты могут потреблять до 600mA
  - Pi думает, что питание < 5A
  - Решение: настроить PSU_MAX_CURRENT или использовать HAT

- `1` (1600mA) - Полный режим
  - Суммарно все USB порты могут потреблять до 1600mA
  - Pi уверен, что питание >= 5A
  - Каждый порт автоматически отключится при превышении

### Напряжение

- `> 5.0V` - Отлично (обычно 5.1V от официального адаптера)
- `4.8-5.0V` - Нормально
- `< 4.8V` - **Проблема**: Возможна нестабильная работа
- `< 4.63V` - **Критично**: Гарантированный undervoltage и троттлинг

### Температура

- `< 60°C` - Норма
- `60-70°C` - Тепло, но приемлемо
- `70-80°C` - Высокая, рекомендуется улучшить охлаждение
- `> 80°C` - **Проблема**: Начнется температурный троттлинг
- `> 85°C` - **Критично**: Агрессивный троттлинг

---

## Удаленный мониторинг

Для мониторинга с другой машины:

```bash
# Запуск скрипта через SSH
ssh ros2@10.1.1.20 '/home/ros2/rob_box_project/docker/scripts/check_power_status.sh'

# Непрерывный мониторинг через SSH
ssh ros2@10.1.1.20 '/home/ros2/rob_box_project/docker/scripts/monitor_power_live.sh 5'

# Или через watch
watch -n 5 "ssh ros2@10.1.1.20 '/home/ros2/rob_box_project/docker/scripts/check_power_status.sh'"
```

---

## Автоматический мониторинг (systemd service)

Для постоянного логирования состояния питания можно создать systemd service:

```bash
# Создать файл службы
sudo nano /etc/systemd/system/power-monitor.service
```

Содержимое:
```ini
[Unit]
Description=Power Monitoring Logger
After=multi-user.target

[Service]
Type=simple
User=ros2
WorkingDirectory=/home/ros2/rob_box_project
ExecStart=/bin/bash -c 'while true; do /home/ros2/rob_box_project/docker/scripts/check_power_status.sh >> /home/ros2/rob_box_project/log/power_monitor.log 2>&1; sleep 300; done'
Restart=always

[Install]
WantedBy=multi-user.target
```

Активация:
```bash
sudo systemctl daemon-reload
sudo systemctl enable power-monitor.service
sudo systemctl start power-monitor.service

# Проверка
sudo systemctl status power-monitor.service

# Просмотр логов
tail -f /home/ros2/rob_box_project/log/power_monitor.log
```

---

## Дополнительная документация

Для полной информации о питании Raspberry Pi 5 и решениях проблем см.:
- **[POWER_MANAGEMENT.md](POWER_MANAGEMENT.md)** - Подробное руководство по питанию

---

## Источники

- Видео-стенограмма: "Powering Raspberry Pi 5 for Robotics"
- Официальная документация Raspberry Pi
- Опыт разработки rob_box_project

---

**Создано**: 2025-10-10
**Автор**: КУКОРЕКЕН
**Проект**: rob_box_project
