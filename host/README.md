# Host Scripts - Rob Box Project

Скрипты для запуска **на самих Raspberry Pi** (не в контейнерах).

## Структура

```
host/
├── main/              # Скрипты для Main Pi
│   ├── setup_can.sh              # Настройка CAN интерфейсов
│   ├── can-setup.service         # Systemd service для автозапуска CAN
│   └── install_host_scripts.sh   # Установщик скриптов на Pi
│
└── vision/            # Скрипты для Vision Pi
    └── (пусто пока)
```

## Различие: host/ vs docker/

### `host/` - Скрипты на Raspberry Pi
- Запускаются **на самой Pi** (bare metal)
- Настройка hardware (CAN, GPIO, I2C, и т.д.)
- Systemd сервисы
- Требуют root привилегий
- **Пример:** `setup_can.sh` - настраивает CAN интерфейс на уровне OS

### `docker/main/scripts/` - Скрипты в контейнерах
- Запускаются **внутри Docker контейнеров**
- Используют уже настроенный hardware (через `/dev`)
- Запуск ROS2 нод и приложений
- **Пример:** `start_ros2_control.sh` - запускает controller_manager

## Main Pi Setup

### 1. Предварительные требования

**Hardware:**
- Raspberry Pi 4/5
- MCP2515 CAN Hat (1 или 2 шт)

**Software:**
```bash
sudo apt-get install -y can-utils iproute2
```

**Device Tree Configuration** (`/boot/config.txt`):
```ini
# CAN0 interface
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25

# CAN1 interface (если нужен второй)
dtoverlay=mcp2515-can1,oscillator=16000000,interrupt=24
```

После изменений:
```bash
sudo reboot
```

### 2. Установка host-скриптов

**Автоматическая установка:**
```bash
cd /path/to/rob_box_project/host/main
sudo ./install_host_scripts.sh
```

Скрипт выполнит:
- ✅ Копирование `setup_can.sh` в `/opt/rob_box/`
- ✅ Установку systemd service `can-setup.service`
- ✅ Включение автозапуска при загрузке Pi
- ✅ Запуск CAN интерфейсов (can0 и can1)

**Ручная установка:**
```bash
# 1. Копируем скрипт
sudo mkdir -p /opt/rob_box
sudo cp setup_can.sh /opt/rob_box/
sudo chmod +x /opt/rob_box/setup_can.sh

# 2. Копируем systemd service
sudo cp can-setup.service /etc/systemd/system/

# 3. Включаем и запускаем
sudo systemctl daemon-reload
sudo systemctl enable can-setup.service
sudo systemctl start can-setup.service
```

### 3. Проверка

**Статус сервиса:**
```bash
sudo systemctl status can-setup
```

**Проверка CAN интерфейсов:**
```bash
ip link show can0
ip link show can1
```

**Мониторинг CAN трафика:**
```bash
candump can0
```

### 4. Управление

**Перезапуск CAN:**
```bash
sudo systemctl restart can-setup
```

**Остановка CAN:**
```bash
sudo systemctl stop can-setup
```

**Логи:**
```bash
sudo journalctl -u can-setup -f
```

**Ручной запуск:**
```bash
# Один интерфейс
sudo /opt/rob_box/setup_can.sh can0

# Оба интерфейса
sudo /opt/rob_box/setup_can.sh can0
sudo /opt/rob_box/setup_can.sh can1
```

## Vision Pi Setup

(В разработке)

## Troubleshooting

### CAN интерфейс не найден

**Проблема:**
```
❌ CAN interface 'can0' not found!
```

**Решение:**
1. Проверьте `/boot/config.txt`:
   ```bash
   cat /boot/config.txt | grep mcp2515
   ```
2. Должны быть строки:
   ```
   dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
   ```
3. Перезагрузите Pi:
   ```bash
   sudo reboot
   ```

### Модули ядра не загружены

**Проблема:**
```
❌ can модуль не найден
```

**Проверка:**
```bash
lsmod | grep can
```

**Ручная загрузка:**
```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mcp251x
```

### Service не запускается

**Проверка логов:**
```bash
sudo journalctl -u can-setup -n 50
```

**Проверка файлов:**
```bash
ls -la /opt/rob_box/setup_can.sh
ls -la /etc/systemd/system/can-setup.service
```

## Development

### Тестирование изменений

После изменения скриптов:

```bash
# 1. Переустановить
cd /path/to/rob_box_project/host/main
sudo ./install_host_scripts.sh

# 2. Проверить
sudo systemctl status can-setup
ip link show can0
```

### Удаление

```bash
# Остановить и отключить service
sudo systemctl stop can-setup
sudo systemctl disable can-setup

# Удалить файлы
sudo rm /etc/systemd/system/can-setup.service
sudo rm -rf /opt/rob_box

# Reload systemd
sudo systemctl daemon-reload
```

## См. также

- [CAN Setup Guide](../../guides/CAN_SETUP.md) - Полная документация по CAN
- [VESC Integration](../../reference/VESC_INTEGRATION.md) - Интеграция VESC моторов
- [ROS2 Control Architecture](../../reference/ROS2_CONTROL_SYSTEM_OVERVIEW.md) - Архитектура управления
