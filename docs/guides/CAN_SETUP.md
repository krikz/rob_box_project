# CAN Setup для Main Pi

## Обзор

Main Pi имеет **2 CAN интерфейса** для подключения VESC регуляторов:
- **can0** - первый MCP2515 SPI CAN контроллер
- **can1** - второй MCP2515 SPI CAN контроллер

## Hardware Setup

### MCP2515 CAN Hat для Raspberry Pi

**Connections:**
```
CAN Hat 1 (can0):
  - SPI: CE0 (GPIO 8)
  - INT: GPIO 25
  - CLK: 16 MHz oscillator

CAN Hat 2 (can1):
  - SPI: CE1 (GPIO 7)  
  - INT: GPIO 24
  - CLK: 16 MHz oscillator
```

### Device Tree Configuration

Добавьте в `/boot/config.txt`:

```ini
# CAN0 interface (первый MCP2515)
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25

# CAN1 interface (второй MCP2515) 
dtoverlay=mcp2515-can1,oscillator=16000000,interrupt=24
```

**После изменений перезагрузите Raspberry Pi:**
```bash
sudo reboot
```

## Software Setup

### 1. Установка CAN утилит

```bash
sudo apt-get update
sudo apt-get install -y can-utils iproute2
```

### 2. Настройка CAN интерфейса

Используйте скрипт `setup_can.sh`:

```bash
# Setup can0
sudo ./host/main/setup_can.sh can0

# Setup can1
sudo ./host/main/setup_can.sh can1
```

**Автоматическая установка с автозапуском:**
```bash
cd host/main
sudo ./install_host_scripts.sh
```

Это установит:
- `/opt/rob_box/setup_can.sh` - скрипт настройки
- `/etc/systemd/system/can-setup.service` - systemd сервис для автозапуска

**Ручная настройка (если нужно):**
```bash
# can0
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# can1
sudo ip link set can1 type can bitrate 500000
sudo ip link set can1 up
```

### 3. Проверка интерфейсов

```bash
# Список интерфейсов
ip link show can0
ip link show can1

# Детальная информация
ip -details link show can0
ip -details link show can1

# Статистика
ifconfig can0
ifconfig can1
```

## VESC Configuration

### Определение CAN интерфейса

Выберите какой CAN интерфейс использовать для VESC:

**Вариант 1: Через environment variable в docker-compose**
```yaml
ros2-control:
  environment:
    - CAN_INTERFACE=can0  # или can1
```

**Вариант 2: Через .env файл**
```bash
echo "CAN_INTERFACE=can0" >> docker/main/.env
```

### Настройка VESC CAN IDs

В `docker/main/config/vesc_nexus/vesc_config.yaml`:

```yaml
can_interface: "can0"  # или can1
vesc_ids: [49, 124, 81, 94]  # [FL, FR, RL, RR]
```

**Установка CAN ID через VESC Tool:**
1. Подключитесь к каждому VESC через USB
2. App Configuration → General → CAN Status → CAN ID
3. Установите уникальные ID для каждого мотора
4. Write Configuration

## Мониторинг CAN

### candump - просмотр CAN трафика

```bash
# Все сообщения
candump can0

# С timestamp
candump -t a can0

# Фильтр по ID (например, VESC ID 49 = 0x31)
candump can0,031:7FF

# Оба интерфейса
candump can0 can1
```

### cansniffer - интерактивный мониторинг

```bash
# Интерактивный просмотр с обновлением
cansniffer can0

# С бинарным представлением
cansniffer -c can0
```

### cansend - отправка тестовых сообщений

```bash
# Формат: ID#DATA
cansend can0 123#DEADBEEF
```

## Troubleshooting

### CAN интерфейс не найден

**Проблема:**
```
❌ CAN interface 'can0' not found!
```

**Решение:**
1. Проверьте `/boot/config.txt` - есть ли `dtoverlay=mcp2515-can0`
2. Перезагрузите Pi: `sudo reboot`
3. Проверьте модули ядра: `lsmod | grep can`
4. Загрузите модули вручную: `sudo modprobe can && sudo modprobe can_raw && sudo modprobe mcp251x`

### CAN интерфейс в состоянии DOWN

**Проблема:**
```
CAN interface: can0 (state: DOWN)
```

**Решение:**
```bash
sudo systemctl restart can-setup
# или вручную:
sudo /opt/rob_box/setup_can.sh can0
```

### Нет ответа от VESC

**Проблема:**
```
No response from VESC on CAN
```

**Проверка:**
1. **Питание:** VESC включены и получают питание?
2. **CAN терминаторы:** 120Ω резисторы установлены на концах шины?
3. **CAN проводка:** CANH и CANL правильно подключены?
4. **CAN битрейт:** VESC настроены на 500 kbit/s?
5. **CAN IDs:** Правильные ли ID в конфигурации?

**Тест CAN трафика:**
```bash
# Мониторинг трафика
candump can0

# Если пусто - проверьте hardware
```

### Ошибки CAN

**Ошибка: Bus-off**
```
can0: Bus-off, restarting
```
**Причины:**
- Слишком много ошибок CAN
- Неправильный битрейт
- Проблемы с терминаторами или проводкой

**Решение:**
```bash
# Restart интерфейса
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000 restart-ms 100
```

### Проверка hardware

```bash
# Kernel logs для CAN
dmesg | grep -i mcp251x
dmesg | grep -i can

# SPI устройства
ls -la /dev/spi*

# Device tree
dtc -I fs /proc/device-tree | grep -A 10 mcp2515
```

## Автозапуск CAN

Создайте systemd service для автоматической настройки CAN при загрузке:

**`/etc/systemd/system/can-setup.service`:**
```ini
[Unit]
Description=Setup CAN interfaces for VESC
After=network.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/setup_can.sh can0
ExecStart=/usr/local/bin/setup_can.sh can1
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

**Установка:**
```bash
# Скрипты уже установлены через install_host_scripts.sh
# Если нет, то:
sudo cp host/main/setup_can.sh /opt/rob_box/
sudo chmod +x /opt/rob_box/setup_can.sh
sudo cp host/main/can-setup.service /etc/systemd/system/

# Enable service
sudo systemctl daemon-reload
sudo systemctl enable can-setup
sudo systemctl start can-setup

# Проверка
sudo systemctl status can-setup
```

## References

- [Linux SocketCAN](https://www.kernel.org/doc/html/latest/networking/can.html)
- [MCP2515 Driver](https://www.kernel.org/doc/html/latest/networking/device_drivers/can/can327.html)
- [VESC CAN Protocol](https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md)
- [VESC Nexus Documentation](https://github.com/krikz/vesc_nexus)
