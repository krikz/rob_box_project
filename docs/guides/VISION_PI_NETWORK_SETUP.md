# Vision Pi Network Setup

## Конфигурация сети Vision Pi для Raspberry Pi 5

### Приоритеты интерфейсов

Vision Pi использует **dual-interface** конфигурацию с автоматическим failover:

- **eth0** (Ethernet): Основной интерфейс, metric **100** (высокий приоритет)
- **wlan0** (WiFi): Резервный интерфейс, metric **600** (низкий приоритет)

При подключенном Ethernet-кабеле весь трафик идёт через eth0. При отключении кабеля система автоматически переключается на wlan0.

### Быстрая настройка через Raspberry Pi Imager

**Рекомендуемый способ:**

1. Установите **Raspberry Pi Imager**:
   ```bash
   sudo apt install rpi-imager
   ```

2. Запустите Imager и выберите:
   - **OS**: Other general-purpose OS → Ubuntu → **Ubuntu Server 24.04.3 LTS (64-bit)**
   - **Storage**: Ваша SD карта
   - **Settings** (⚙️):
     - **Hostname**: `VISION`
     - **Username**: `ros2`, **Password**: `open`
     - **WiFi SSID**: `ROS2`, **Password**: `1234567890`
     - **Wireless LAN country**: `RU`
     - **Enable SSH**: ✅ Use password authentication

3. Запишите образ на SD карту

4. Вставьте SD карту в Raspberry Pi 5 и загрузите

5. После загрузки (2-3 минуты) настройте приоритеты сети:
   ```bash
   cd /path/to/rob_box_project
   ./scripts/setup_vision_pi_network.sh VISION.local
   ```

6. Перезагрузите для применения USB power и SPI:
   ```bash
   ssh ros2@VISION.local 'sudo reboot'
   ```

### Ручная настройка

Если нужно настроить уже работающий Pi:

```bash
# 1. Скопируйте скрипт на Vision Pi
scp scripts/setup_vision_pi_network.sh ros2@VISION.local:~/

# 2. Запустите на Vision Pi
ssh ros2@VISION.local
./setup_vision_pi_network.sh localhost
```

### Netplan конфигурация

Файл `/etc/netplan/50-cloud-init.yaml`:

```yaml
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: true
      dhcp-identifier: mac
      optional: true
      dhcp4-overrides:
        route-metric: 100  # Высокий приоритет
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
  wifis:
    wlan0:
      dhcp4: true
      dhcp-identifier: mac
      optional: true
      access-points:
        "ROS2":
          password: "1234567890"
      dhcp4-overrides:
        route-metric: 600  # Низкий приоритет
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
```

### Применение изменений

```bash
sudo netplan apply
```

### Проверка состояния

```bash
# Список интерфейсов
ip addr show

# Маршруты (должны быть два default через eth0 и wlan0)
ip route show

# Проверка приоритетов (меньше metric = выше приоритет)
ip route show | grep default
# Ожидаемый вывод:
# default via 10.1.1.1 dev eth0 proto dhcp src 10.1.1.11 metric 100
# default via 10.1.1.1 dev wlan0 proto dhcp src 10.1.1.21 metric 600
```

### Аппаратные настройки

В `/boot/firmware/config.txt`:

```ini
# USB Power для 30W блока питания через GPIO
usb_max_current_enable=1

# SPI для LED Matrix (GPIO 10 MOSI)
dtparam=spi=on
```

**Требуется перезагрузка** после изменения config.txt!

### Типичные проблемы

#### eth0 не поднимается

```bash
# Проверьте кабель
ip link show eth0

# Вручную поднимите интерфейс
sudo ip link set eth0 up

# Перезапустите networkd
sudo systemctl restart systemd-networkd
```

#### WiFi не подключается

```bash
# Проверьте wpa_supplicant
sudo systemctl status wpa_supplicant@wlan0

# Проверьте доступные сети
sudo iwlist wlan0 scan | grep ESSID

# Перезапустите WiFi
sudo systemctl restart wpa_supplicant@wlan0
```

#### Оба интерфейса работают, но трафик идёт не через тот

```bash
# Проверьте метрики маршрутов
ip route show | grep default

# Если метрики неправильные - переприменить netplan
sudo netplan apply
```

### IP адреса

По умолчанию роутер выдаёт:
- **eth0**: `10.1.1.11/24`
- **wlan0**: `10.1.1.21/24`

Доступ по любому из адресов, но основной маршрут через eth0.

### SSH доступ

```bash
# Через mDNS (удобно)
ssh ros2@VISION.local

# Через IP (если mDNS не работает)
ssh ros2@10.1.1.11   # Ethernet (основной)
ssh ros2@10.1.1.21   # WiFi (резервный)

# Пароль: open
```

### Автоматизация

Скрипт `scripts/setup_vision_pi_network.sh` автоматизирует всю настройку:

```bash
# Использование
./scripts/setup_vision_pi_network.sh [hostname_or_ip]

# Примеры
./scripts/setup_vision_pi_network.sh VISION.local
./scripts/setup_vision_pi_network.sh 10.1.1.21
```

Скрипт требует установленный `sshpass`:
```bash
sudo apt install sshpass
```

---

**Автор**: rob_box_project  
**Дата**: 2025-10-15  
**Raspberry Pi**: 5 (8GB)  
**OS**: Ubuntu Server 24.04.3 LTS
