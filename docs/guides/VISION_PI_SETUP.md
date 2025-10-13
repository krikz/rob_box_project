# 🤖 Vision Pi - Автоматическая установка

Автоматический скрипт для настройки Vision Pi с нуля на свежей установке Raspberry Pi OS.

## 🚀 Быстрая установка (одна команда)

```bash
curl -fsSL https://raw.githubusercontent.com/krikz/rob_box_project/develop/scripts/setup_vision_pi.sh | bash
```

**Что делает скрипт:**
- ✅ Проверяет и устанавливает Docker + Docker Compose
- ✅ Устанавливает необходимые системные пакеты (git, curl, vim, htop, etc)
- ✅ Клонирует репозиторий rob_box_project в ~/rob_box_project
- ✅ Настраивает кастомный MOTD с логотипом РОББОКС
- ✅ Настраивает автозапуск Docker контейнеров через systemd
- ✅ Показывает системную информацию, температуру, напряжение, Docker статус

## 📋 Предварительные требования

1. **Raspberry Pi 4/5** с установленной **Raspberry Pi OS Lite (64-bit)**
2. **Интернет соединение** (WiFi или Ethernet)
3. **SSH доступ** к Raspberry Pi

## 🔧 Пошаговая инструкция

### Шаг 1: Подготовка флешки

**Вариант A: Raspberry Pi Imager (рекомендуется)**
1. Скачай [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
2. Выбери OS: **Raspberry Pi OS Lite (64-bit)**
3. Настрой:
   - Hostname: `vision-pi`
   - Username: `pi`
   - Password: `<твой пароль>`
   - WiFi: SSID и пароль
   - SSH: Enable
4. Запиши на флешку 128GB

**Вариант B: Вручную**
```bash
# Скачать образ
wget https://downloads.raspberrypi.org/raspios_lite_arm64/images/raspios_lite_arm64-2024-07-04/2024-07-04-raspios-bookworm-arm64-lite.img.xz

# Записать на флешку (замени sdX!)
xzcat 2024-07-04-raspios-bookworm-arm64-lite.img.xz | sudo dd of=/dev/sdX bs=4M status=progress conv=fsync
```

### Шаг 2: Первая загрузка

1. Вставь флешку в Vision Pi
2. Подключи:
   - Ethernet кабель к Main Pi (10.1.1.10)
   - Питание 5V/5A
3. Дождись загрузки (~30-60 сек)

### Шаг 3: Подключение по SSH

```bash
# Через WiFi (если настроил в Imager)
ssh pi@vision-pi.local
# или
ssh pi@<WiFi IP адрес>

# Через Ethernet (если Main Pi маршрутизирует)
ssh pi@10.1.1.11
```

### Шаг 4: Настройка сети (если не через Imager)

```bash
# Hostname
sudo hostnamectl set-hostname vision-pi

# Ethernet статический IP (для связи с Main Pi)
sudo nmcli con mod "Wired connection 1" ipv4.addresses 10.1.1.11/24 ipv4.method manual
sudo nmcli con mod "Wired connection 1" ipv4.gateway 10.1.1.1

# WiFi для SSH доступа
sudo nmcli dev wifi connect "YOUR_WIFI_SSID" password "YOUR_WIFI_PASSWORD"
sudo nmcli con mod "YOUR_WIFI_SSID" ipv4.addresses 10.1.1.21/24 ipv4.method manual

# Перезагрузка сети
sudo systemctl restart NetworkManager
```

### Шаг 5: Запуск автоматической установки

```bash
# Одна команда - всё установит автоматически!
curl -fsSL https://raw.githubusercontent.com/krikz/rob_box_project/develop/scripts/setup_vision_pi.sh | bash
```

**Что будет происходить:**
1. Скрипт покажет логотип РОББОКС 🎨
2. Установит Docker (~2-3 мин)
3. Установит системные пакеты (~1 мин)
4. Склонирует репозиторий rob_box_project (~30 сек)
5. Настроит кастомный MOTD
6. Настроит автозапуск контейнеров
7. Покажет итоговую информацию

**Время установки:** ~5-10 минут

### Шаг 6: Перелогиниться

```bash
# Выйти из SSH
exit

# Подключиться заново для применения группы docker
ssh pi@vision-pi.local
```

**Теперь при входе увидишь:**
```
    ╔═══════════════════════════════════════════════════════════════════╗
    ║                                                                   ║
    ║   ██████╗  ██████╗ ██████╗ ██████╗  ██████╗ ██╗  ██╗███████╗    ║
    ║   ██╔══██╗██╔═══██╗██╔══██╗██╔══██╗██╔═══██╗██║ ██╔╝██╔════╝    ║
    ║   ██████╔╝██║   ██║██████╔╝██████╔╝██║   ██║█████╔╝ ███████╗    ║
    ║   ██╔══██╗██║   ██║██╔══██╗██╔══██╗██║   ██║██╔═██╗ ╚════██║    ║
    ║   ██║  ██║╚██████╔╝██████╔╝██████╔╝╚██████╔╝██║  ██╗███████║    ║
    ║   ╚═╝  ╚═╝ ╚═════╝ ╚═════╝ ╚═════╝  ╚═════╝ ╚═╝  ╚═╝╚══════╝    ║
    ║                                                                   ║
    ║                    🎥 Vision Pi (10.1.1.11) 🎥                    ║
    ║                                                                   ║
    ╚═══════════════════════════════════════════════════════════════════╝

SYSTEM INFO
  Hostname:       vision-pi
  Uptime:         2 hours, 15 minutes
  Load Average:   0.15, 0.18, 0.12
  Memory:         1.2G/7.8G
  Disk Usage:     5.2G/117G (5%)

HARDWARE
  CPU Temp:       45.2°C
  Core Voltage:   0.8512V

NETWORK
  Ethernet (eth0):  10.1.1.11 [Data Link]
  WiFi (wlan0):     10.1.1.21 [SSH/Management]

DOCKER CONTAINERS
  Status:         5/5 running
  ✓ vision-zenoh-router    Up 2 hours
  ✓ vision-oak-d           Up 2 hours
  ✓ vision-lslidar         Up 2 hours
  ✓ vision-apriltag        Up 2 hours
  ✓ vision-voice-assistant Up 2 hours

═══════════════════════════════════════════════════════════════
Quick Commands:
  cd ~/rob_box_project/docker/vision  - Go to Vision Pi config
  docker compose ps                    - Check containers status
  docker compose logs -f               - View all container logs
  docker compose restart               - Restart all containers
═══════════════════════════════════════════════════════════════
```

### Шаг 7: Логин в GitHub Container Registry

```bash
# Создай Personal Access Token на GitHub с правами read:packages
# Settings → Developer settings → Personal access tokens → Tokens (classic)

# Логин
echo "YOUR_GITHUB_TOKEN" | docker login ghcr.io -u YOUR_USERNAME --password-stdin
```

### Шаг 8: Запуск Docker контейнеров

```bash
cd ~/rob_box_project/docker/vision

# Скачать все образы из registry
docker compose pull

# Запустить все контейнеры
docker compose up -d

# Проверить статус
docker compose ps

# Посмотреть логи
docker compose logs -f
```

**Контейнеры запустятся:**
- `vision-zenoh-router` - Zenoh маршрутизатор (центральный узел)
- `vision-oak-d` - OAK-D Lite камера
- `vision-lslidar` - LSLIDAR N10 2D лидар
- `vision-apriltag` - AprilTag детекция
- `vision-voice-assistant` - Voice Assistant + LED анимации

## 🎨 Кастомный MOTD (приветствие)

### Что показывает

- **Логотип РОББОКС** в ASCII art с фирменным цветом #7C3AED (фиолетовый)
- **Системная информация:** hostname, uptime, load average, память, диск
- **Температура и напряжение:** CPU temp, Core voltage
- **Сеть:** Ethernet (10.1.1.11) и WiFi (10.1.1.21) IP адреса
- **Docker контейнеры:** статус всех контейнеров
- **Быстрые команды:** подсказки для управления

### Ручное обновление MOTD

MOTD автоматически обновляется при каждом входе, но можно вызвать вручную:

```bash
/usr/local/bin/robbox-motd
```

### Отключение MOTD (если надоел)

```bash
# Закомментировать в ~/.bashrc
nano ~/.bashrc
# Найти строку:
# /usr/local/bin/robbox-motd
# И закомментировать её:
# # /usr/local/bin/robbox-motd
```

## 🔄 Автозапуск контейнеров

Скрипт настраивает systemd service `robbox-vision.service` для автоматического запуска контейнеров при загрузке.

**Управление service:**

```bash
# Проверить статус
sudo systemctl status robbox-vision

# Запустить вручную
sudo systemctl start robbox-vision

# Остановить
sudo systemctl stop robbox-vision

# Перезапустить
sudo systemctl restart robbox-vision

# Отключить автозапуск
sudo systemctl disable robbox-vision

# Включить автозапуск
sudo systemctl enable robbox-vision
```

## 📊 Проверка системы

```bash
# Температура CPU
vcgencmd measure_temp

# Напряжение
vcgencmd measure_volts core

# USB устройства (OAK-D, LIDAR, ReSpeaker)
lsusb

# Сетевые интерфейсы
ip addr

# Docker контейнеры
docker ps

# Логи контейнера
docker logs vision-oak-d -f

# Zenoh топики
docker exec vision-zenoh-router zenoh-bridge-dds -h
```

## 🐛 Troubleshooting

### Docker не найден после установки

```bash
# Проверь что ты в группе docker
groups

# Если нет docker в списке:
sudo usermod -aG docker $USER

# Перелогинься
exit
ssh pi@vision-pi
```

### Контейнеры не запускаются

```bash
# Проверь логи
docker compose -f ~/rob_box_project/docker/vision/docker-compose.yaml logs

# Проверь что образы скачаны
docker images | grep rob_box

# Если нет - логин в registry и pull
echo "TOKEN" | docker login ghcr.io -u USERNAME --password-stdin
docker compose pull
```

### OAK-D камера не определяется

```bash
# Проверь USB устройства
lsusb | grep "Movidius"

# Если не видно - проверь USB power
vcgencmd get_config int | grep usb_max_current_enable

# Должно быть usb_max_current_enable=1
# Если нет - увеличь USB ток (см. RASPBERRY_PI_USB_POWER_FIX.md)
```

### MOTD не показывается

```bash
# Проверь что скрипт есть
ls -la /usr/local/bin/robbox-motd

# Запусти вручную
/usr/local/bin/robbox-motd

# Проверь .bashrc
grep robbox-motd ~/.bashrc
```

## 📝 Дополнительная документация

- [QUICK_START.md](../docs/guides/QUICK_START.md) - Быстрый старт
- [VISION_PI_DEPLOYMENT.md](../docs/deployment/VISION_PI_DEPLOYMENT.md) - Развёртывание Vision Pi
- [RASPBERRY_PI_USB_POWER_FIX.md](../docs/guides/RASPBERRY_PI_USB_POWER_FIX.md) - Увеличение USB тока
- [TROUBLESHOOTING.md](../docs/guides/TROUBLESHOOTING.md) - Решение проблем

## 🎯 Следующие шаги

После успешной установки Vision Pi:

1. **Настрой Main Pi** - повтори процесс для Main Pi (будет отдельный скрипт)
2. **Проверь связь Zenoh** - убедись что Vision Pi передаёт данные на Main Pi
3. **Калибруй камеры** - настрой OAK-D и AprilTag детекцию
4. **Запусти RTAB-Map** - начни картографирование
5. **Протестируй Voice Assistant** - проверь голосовые команды

**Приятной работы с РОББОКС! 🤖**

---

**Автор:** AI Agent + krikz  
**Дата:** 2025-10-13  
**Версия:** 1.0
