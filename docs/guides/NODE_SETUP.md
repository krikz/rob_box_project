# 🤖 РОББОКС - Unified Node Setup

Универсальная система настройки узлов для роботизированной платформы РОББОКС.

## 📦 Доступные типы узлов

### 1. 🎥 Vision Pi
**IP:** `10.1.1.11`  
**Функции:**
- OAK-D Lite камера (стерео, RGB, детекция объектов)
- LSLidar N10 (360° 2D лидар)
- LED Matrix 38×10 NeoPixel WS2812B
- ReSpeaker Mic Array v2.0 (голосовой ассистент)
- AprilTag детекция

**Docker сервисы:**
- `oak-d` - обработка камеры
- `lslidar` - обработка лидара
- `apriltag` - детекция маркеров
- `led-matrix` - управление LED матрицей
- `voice-assistant` - голосовое управление

### 2. 🧠 Main Pi
**IP:** `10.1.1.10`  
**Функции:**
- Главный контроллер робота
- micro-ROS agent (связь с ESP32)
- Zenoh router (DDS мост)
- Navigation stack (SLAM, планирование пути)
- Координация между узлами

**Docker сервисы:**
- `micro-ros-agent` - связь с микроконтроллерами
- `zenoh-router` - сетевой мост

### 3. 📡 Sensor Hub
**IP:** `10.1.1.20`  
**Платформа:** ESP32 + micro-ROS  
**Функции:**
- IMU (MPU6050/MPU9250)
- GPS модуль
- Энкодеры колес
- Драйверы двигателей (VESC)
- Датчики расстояния

**Прошивка:** PlatformIO + micro-ROS

### 4. 🦾 Manipulator (в разработке)
**IP:** `10.1.1.30`  
**Функции:**
- Управление манипулятором/рукой
- Динамиксель сервоприводы
- Захват объектов

### 5. 🛠️ Custom Node
**IP:** `10.1.1.100`  
**Функции:** Пользовательская конфигурация

## 🚀 Быстрый старт

### Интерактивная установка

```bash
curl -fsSL https://raw.githubusercontent.com/krikz/rob_box_project/develop/scripts/setup_node.sh | bash
```

Скрипт спросит какой тип узла настроить и проведет через весь процесс.

### Установка с указанием типа узла

```bash
# Vision Pi
curl -fsSL https://raw.githubusercontent.com/krikz/rob_box_project/develop/scripts/setup_node.sh | bash -s vision

# Main Pi
curl -fsSL https://raw.githubusercontent.com/krikz/rob_box_project/develop/scripts/setup_node.sh | bash -s main

# Sensor Hub
curl -fsSL https://raw.githubusercontent.com/krikz/rob_box_project/develop/scripts/setup_node.sh | bash -s sensor-hub
```

### Специализированные скрипты

```bash
# Vision Pi (прямой вызов)
curl -fsSL https://raw.githubusercontent.com/krikz/rob_box_project/develop/scripts/setup_vision_pi.sh | bash

# Main Pi (прямой вызов)
curl -fsSL https://raw.githubusercontent.com/krikz/rob_box_project/develop/scripts/setup_main_pi.sh | bash
```

## 📋 Что делает скрипт

1. **Определяет тип узла** - интерактивно или через параметр
2. **Устанавливает Docker + Docker Compose** - если еще не установлены
3. **Клонирует репозиторий** - `~/rob_box_project` (ветка develop)
4. **Настраивает MOTD** - красивое приветствие с логотипом РОББОКС
5. **Настраивает автозапуск** - Docker контейнеры запускаются при загрузке
6. **Специфичная настройка** - проверка устройств, конфигурация

## 🔧 После установки

### 1. Перелогиниться (если нужно)

```bash
exit
ssh pi@<node_ip>
```

### 2. Логин в GitHub Container Registry

```bash
echo YOUR_TOKEN | docker login ghcr.io -u YOUR_USERNAME --password-stdin
```

### 3. Запуск Docker контейнеров

```bash
# Vision Pi
cd ~/rob_box_project/docker/vision
docker compose pull
docker compose up -d

# Main Pi
cd ~/rob_box_project/docker/main
docker compose pull
docker compose up -d
```

### 4. Проверка статуса

```bash
docker compose ps
docker compose logs -f
```

## 🌐 Сетевая конфигурация

Все узлы работают в единой сети `10.1.1.0/24`:

| Узел | IP | Ethernet | WiFi |
|------|-----|----------|------|
| Main Pi | 10.1.1.10 | eth0 (metric 100) | wlan0 (metric 600) |
| Vision Pi | 10.1.1.11 | eth0 (metric 100) | wlan0 (metric 600) |
| Sensor Hub | 10.1.1.20 | - | ESP32 WiFi |
| Manipulator | 10.1.1.30 | eth0 | wlan0 |

**WiFi настройки:**
- SSID: `ROS2`
- Password: `1234567890`

## 📊 MOTD (Приветствие)

После установки при входе вы увидите:

```
    ╔═══════════════════════════════════════════════════════════════════╗
    ║                                                                   ║
    ║  ██████╗   ██████╗  ╔╩═══╗██ ██████╗   ██████╗  ██╗  ██╗ ██████╗  ║
    ║  ██╔══██╗ ██╔═══██╗ ██╔═══╩╗ ██╔═══╩╗ ██╔═══██╗ ██║ ██╔╝██╔════╝  ║
    ║  ██████╔╝ ██║   ██║ ██████╔╝ ██████╔╝ ██║   ██║ █████╔╝ ██║       ║
    ║  ██╔═══╝  ██║   ██║ ██╔══██╗ ██╔══██╗ ██║   ██║ ██╔═██╗ ██║       ║
    ║  ██║      ╚██████╔╝ ██████╔╝ ██████╔╝ ╚██████╔╝ ██║  ██╗╚██████╗  ║
    ║  ╚═╝       ╚═════╝  ╚═════╝  ╚═════╝   ╚═════╝  ╚═╝  ╚═╝ ╚═════╝  ║
    ║                                                                   ║
    ║              🎥 Vision Pi (10.1.1.11) 🎥                          ║
    ║                                                                   ║
    ╚═══════════════════════════════════════════════════════════════════╝

SYSTEM INFO
  Hostname:       VISION
  Uptime:         2 hours, 35 minutes
  Load Average:   0.45, 0.52, 0.48
  Memory:         1.2G / 7.8G
  Disk Usage:     5.3G / 29G (19%)

HARDWARE
  CPU Temp:       42.5'C
  Core Voltage:   0.8500V

NETWORK
  Ethernet (eth0):  10.1.1.11
  WiFi (wlan0):     10.1.1.11

DOCKER CONTAINERS
  Status:         5/5 running

  ✓ oak-d         Up 2 hours
  ✓ lslidar       Up 2 hours
  ✓ apriltag      Up 2 hours
  ✓ led-matrix    Up 2 hours
  ✓ voice-asst    Up 2 hours
```

## 🔄 Обновление

Скрипт автоматически обновляет репозиторий если он уже существует:

```bash
cd ~/rob_box_project
git pull origin develop
```

Или запустите скрипт снова - он проверит и обновит.

## 🐛 Troubleshooting

### Docker группа не применяется

```bash
exit
ssh pi@<node_ip>
```

### Контейнеры не запускаются

```bash
# Проверка логов
docker compose logs

# Перезапуск
docker compose restart

# Полная пересборка
docker compose down
docker compose pull
docker compose up -d
```

### USB устройства не видны

```bash
# Список USB устройств
lsusb

# Перезагрузка
sudo reboot
```

## 📚 Дополнительная документация

- [CI/CD Pipeline](../CI_CD_PIPELINE.md)
- [Vision Pi Network Setup](VISION_PI_NETWORK_SETUP.md)
- [Docker Architecture](../architecture/SYSTEM_OVERVIEW.md)

## 🤝 Contributing

См. [CONTRIBUTING.md](../../CONTRIBUTING.md)

## 📄 License

MIT License - см. [LICENSE](../../../LICENSE)

---

**Автор:** AI Agent + krikz  
**Дата:** 2025-10-15  
**Версия:** 2.0
