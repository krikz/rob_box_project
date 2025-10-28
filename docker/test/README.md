# Test Environment for x86_64 Development

**Назначение:** Полноценная тестовая среда для запуска всех Rob Box сервисов на x86_64 машинах через QEMU emulation.

## 📋 Оглавление

- [Обзор](#обзор)
- [Установка](#установка)
- [Использование](#использование)
- [Архитектура](#архитектура)
- [Ожидаемое поведение](#ожидаемое-поведение)
- [Мониторинг](#мониторинг)
- [Troubleshooting](#troubleshooting)

---

## 🎯 Обзор

Этот каталог содержит docker-compose конфигурацию для запуска полной Rob Box системы на x86_64/AMD64 машинах.

**Use Cases:**
- ✅ Разработка без наличия Raspberry Pi
- ✅ Интеграционное тестирование всех сервисов
- ✅ Smoke testing ROS2 топиков и коммуникации
- ✅ CI/CD testing в GitHub Actions
- ✅ Обучение и демонстрации

**Ограничения:**
- ⚠️ Hardware-dependent ноды будут падать (ожидаемо)
- ⚠️ Производительность 5-10x медленнее чем на native ARM64
- ⚠️ Не для production использования

---

## 🚀 Установка

### Шаг 1: Установка QEMU

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y qemu-user-static binfmt-support

# Регистрация QEMU в kernel
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

# Проверка
docker run --rm --platform linux/arm64 arm64v8/ubuntu uname -m
# Вывод должен быть: aarch64
```

### Шаг 2: Установка Docker Buildx

```bash
# Создать multi-arch builder
docker buildx create --name multiarch --driver docker-container --use
docker buildx inspect --bootstrap

# Проверка поддерживаемых платформ
docker buildx inspect | grep Platforms
# Должно быть: linux/amd64, linux/arm64, ...
```

### Шаг 3: Подготовка переменных окружения

```bash
cd /home/runner/work/rob_box_project/rob_box_project/docker/test

# Копировать example файл
cp .env.example .env

# Редактировать если нужно
nano .env
```

---

## 📦 Использование

### Запуск всей системы

```bash
cd /home/runner/work/rob_box_project/rob_box_project/docker/test

# Запустить все сервисы
./scripts/start_test_env.sh

# Или напрямую через docker-compose
docker compose -f docker-compose-x86-test.yaml up -d

# Посмотреть логи
docker compose -f docker-compose-x86-test.yaml logs -f

# Остановить
docker compose -f docker-compose-x86-test.yaml down
```

### Запуск отдельных сервисов

```bash
# Только Zenoh router
docker compose -f docker-compose-x86-test.yaml up -d zenoh-router

# Main Pi сервисы
docker compose -f docker-compose-x86-test.yaml up -d \
  zenoh-router rtabmap nav2 robot-state-publisher perception

# Vision Pi сервисы (будут падать без hardware)
docker compose -f docker-compose-x86-test.yaml up -d \
  oak-d lslidar voice-assistant led-matrix
```

### Smoke Testing

```bash
# Запустить smoke test
./scripts/smoke_test.sh

# Мониторинг топиков
./scripts/monitor_topics.sh

# Проверка здоровья сервисов
./scripts/check_health.sh
```

---

## 🏗️ Архитектура

### Структура каталога

```
docker/test/
├── docker-compose-x86-test.yaml    # Полная композиция всех сервисов
├── .env.example                    # Пример переменных окружения
├── .env                            # Локальные переменные (не в git)
├── config/                         # Конфигурации для тестирования
│   ├── zenoh_test_config.json5     # Zenoh настройки для localhost
│   ├── zenoh_session_config.json5  # Zenoh session config
│   └── ros2_test_params.yaml       # ROS2 параметры
├── scripts/
│   ├── start_test_env.sh           # Запуск окружения
│   ├── stop_test_env.sh            # Остановка окружения
│   ├── smoke_test.sh               # Smoke testing всех топиков
│   ├── monitor_topics.sh           # Мониторинг ROS2 топиков
│   ├── check_health.sh             # Проверка здоровья контейнеров
│   └── generate_mock_data.sh       # Генерация mock данных для тестов
├── mock_data/                      # Mock данные для hardware-less тестов
│   ├── fake_scan.bag               # Fake LiDAR data
│   ├── fake_image.bag              # Fake camera data
│   └── fake_odom.bag               # Fake odometry
└── README.md                       # Этот файл
```

### Сервисы в docker-compose-x86-test.yaml

| Service | Platform | Status | Description |
|---------|----------|--------|-------------|
| `zenoh-router` | amd64 | ✅ OK | Центральный роутер (native) |
| `rtabmap` | arm64 | ⚠️ IDLE | SLAM (QEMU, без сенсоров ждёт топиков) |
| `nav2` | arm64 | ⚠️ IDLE | Навигация (QEMU, ждёт карту и /scan) |
| `robot-state-publisher` | arm64 | ✅ OK | URDF publisher (QEMU) |
| `perception` | arm64 | ✅ OK | Context aggregator (QEMU) |
| `twist-mux` | arm64 | ✅ OK | Мультиплексор команд (QEMU) |
| `oak-d` | arm64 | ❌ FAIL | Camera (QEMU, падает без /dev/bus/usb) |
| `lslidar` | arm64 | ❌ FAIL | LiDAR (QEMU, падает без device) |
| `voice-assistant` | arm64 | ❌ FAIL | Voice (QEMU, падает без микрофона) |
| `led-matrix` | arm64 | ❌ FAIL | LED (QEMU, падает без SPI) |
| `micro-ros-agent` | arm64 | ❌ FAIL | ESP32 link (QEMU, падает без UART) |
| `vesc-nexus` | arm64 | ❌ FAIL | Motor control (QEMU, падает без UART) |

---

## 📊 Ожидаемое поведение

### ✅ Успешно запускаются

**Сервисы без hardware dependencies:**

1. **zenoh-router** (native amd64)
   - Запускается нормально
   - REST API доступен на `:8000`
   - WebSocket на `:7447`

2. **robot-state-publisher** (QEMU arm64)
   - Публикует `/robot_description`
   - Публикует `/tf_static` с URDF frames
   - Работает нормально

3. **twist-mux** (QEMU arm64)
   - Мультиплексирует `/cmd_vel` топики
   - Приоритизирует источники команд
   - Работает нормально

4. **perception (context-aggregator)** (QEMU arm64)
   - Агрегирует данные с сенсоров
   - Публикует `/perception/robot_context`
   - Работает, но без реальных данных

5. **nav2** (QEMU arm64)
   - Запускается, все ноды инициализируются
   - Ждёт топики `/scan`, `/odom`, `/map`
   - Idle state до появления данных

### ⚠️ Запускаются, но не функциональны

**Сервисы ожидающие топики от hardware:**

6. **rtabmap** (QEMU arm64)
   - Запускается успешно
   - Ожидает топики:
     - `/camera/rgb/image_raw`
     - `/camera/depth/image_rect_raw`
     - `/scan` (от lslidar)
   - **Ошибка:** "Did not receive data since 5 seconds" (ожидаемо)

### ❌ Падают с ошибками

**Сервисы требующие физические устройства:**

7. **oak-d** (QEMU arm64)
   - Запускается
   - **Ошибка:** `[error] Failed to find device (ma2x8x), error message: USB DEVICE ERROR`
   - **Статус:** Падает через 5-10 секунд

8. **lslidar** (QEMU arm64)
   - Запускается
   - **Ошибка:** `Device opening fail, Could not open serial port /dev/ttyUSB_Lslidar`
   - **Статус:** Падает через 3-5 секунд

9. **voice-assistant** (QEMU arm64)
   - Запускается
   - **Ошибка:** `ALSA lib: cannot find card '2'` (ReSpeaker не найден)
   - **Статус:** Падает через 5-10 секунд

10. **led-matrix** (QEMU arm64)
    - Запускается
    - **Ошибка:** `Can't open /dev/spidev0.0`
    - **Статус:** Падает через 1-2 секунды

11. **micro-ros-agent** (QEMU arm64)
    - Запускается
    - **Ошибка:** `Cannot open serial port /dev/ttyUSB0`
    - **Статус:** Падает через 1-2 секунды

12. **vesc-nexus** (QEMU arm64)
    - Запускается
    - **Ошибка:** `VESC not found on any port`
    - **Статус:** Падает через 5 секунд

---

## 🔍 Мониторинг

### Проверка статуса контейнеров

```bash
# Все контейнеры
docker compose -f docker-compose-x86-test.yaml ps

# Только запущенные
docker compose -f docker-compose-x86-test.yaml ps --filter "status=running"

# Только упавшие
docker compose -f docker-compose-x86-test.yaml ps --filter "status=exited"
```

### Логи

```bash
# Все логи
docker compose -f docker-compose-x86-test.yaml logs -f

# Конкретный сервис
docker compose -f docker-compose-x86-test.yaml logs -f rtabmap

# Последние 100 строк
docker compose -f docker-compose-x86-test.yaml logs --tail=100 oak-d
```

### Мониторинг ROS2 топиков

```bash
# Запустить мониторинг скрипт
./scripts/monitor_topics.sh

# Или вручную
docker exec test-zenoh-router bash -c "
  source /opt/ros/humble/setup.bash
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  ros2 topic list
"
```

### Zenoh REST API

```bash
# Проверка роутера
curl http://localhost:8000/@/local/router | jq

# Список subscribers
curl http://localhost:8000/@/local/subscriber | jq

# Список publishers
curl http://localhost:8000/@/local/publisher | jq
```

---

## 🐛 Troubleshooting

### Проблема: QEMU не работает

**Симптомы:**
```
exec user process caused: exec format error
```

**Решение:**
```bash
# Переустановить QEMU
sudo apt-get install --reinstall qemu-user-static binfmt-support

# Пере-регистрировать
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

# Проверить
ls -la /proc/sys/fs/binfmt_misc/qemu-*
```

### Проблема: Контейнеры не видят друг друга

**Симптомы:**
```
Zenoh connection timeout
```

**Решение:**
```bash
# Проверить что используется network_mode: host
grep "network_mode" docker-compose-x86-test.yaml

# Проверить что zenoh-router запущен
docker ps | grep zenoh-router

# Проверить порты
ss -tulnp | grep 7447
ss -tulnp | grep 8000
```

### Проблема: Образы не найдены

**Симптомы:**
```
no matching manifest for linux/arm64
```

**Решение:**
```bash
# Проверить что образы собраны для arm64
docker manifest inspect ghcr.io/krikz/rob_box:rtabmap-humble-latest | grep architecture

# Если нет - собрать локально
cd ../..  # В корень проекта
./scripts/local-build.sh rtabmap linux/arm64
```

### Проблема: Медленная производительность

**Симптомы:**
Все тормозит, RTABMAP не работает

**Решение:**
Это нормально для QEMU emulation. Рекомендации:
- Использовать только для smoke testing
- Не запускать SLAM/vision ноды
- Увеличить CPU cores для Docker Desktop

```bash
# Проверить CPU usage
docker stats

# Ограничить количество сервисов
docker compose -f docker-compose-x86-test.yaml up -d \
  zenoh-router robot-state-publisher twist-mux perception
```

---

## 📚 Дополнительные ресурсы

### Связанные документы

- [ARM64_EMULATION_ON_X86.md](../../docs/development/ARM64_EMULATION_ON_X86.md) - Теория и возможности
- [DOCKER_STANDARDS.md](../../docs/development/DOCKER_STANDARDS.md) - Docker стандарты
- [TESTING_GUIDE.md](../../docs/development/TESTING_GUIDE.md) - Тестирование
- [CI_CD_PIPELINE.md](../../docs/CI_CD_PIPELINE.md) - GitHub Actions

### Полезные команды

```bash
# Проверить платформу контейнера
docker inspect <container> | grep Architecture

# Проверить QEMU внутри контейнера
docker exec <container> uname -m

# Benchmark производительности
docker run --rm --platform linux/arm64 arm64v8/ubuntu bash -c "
  apt-get update && apt-get install -y sysbench
  sysbench cpu --cpu-max-prime=20000 run
"
```

---

## 🎯 Следующие шаги

1. **Запустить smoke test:** `./scripts/smoke_test.sh`
2. **Проверить топики:** `./scripts/monitor_topics.sh`
3. **Добавить mock данные:** Для тестирования RTAB-Map без реальных сенсоров
4. **Интеграционные тесты:** Проверить взаимодействие между сервисами

---

**Создано:** October 28, 2025  
**Версия:** 1.0.0
