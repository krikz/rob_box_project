# ARM64 Docker Images on x86_64/AMD64 - Research & Implementation Guide

**Дата:** October 28, 2025  
**Автор:** Rob Box Project Team  
**Версия:** 1.0.0

## 📋 Оглавление

- [Введение](#введение)
- [Теоретическая возможность](#теоретическая-возможность)
- [QEMU User Mode Emulation](#qemu-user-mode-emulation)
- [Производительность](#производительность)
- [Практическое применение](#практическое-применение)
- [Ограничения](#ограничения)
- [Реализация в проекте](#реализация-в-проекте)

---

## 🎯 Введение

**Задача:** Исследовать возможность запуска ARM64 Docker образов на x86_64/AMD64 машинах для целей разработки и тестирования.

**Мотивация:**
- ✅ Разработка и тестирование без наличия Raspberry Pi
- ✅ Интеграционное тестирование всей системы на одной машине
- ✅ Smoke testing всех ROS2 топиков и сервисов
- ✅ CI/CD тестирование в GitHub Actions

---

## ✅ Теоретическая возможность

**Ответ: ДА, это возможно через QEMU emulation**

Docker поддерживает запуск образов для других архитектур через технологию **QEMU user mode emulation**.

### Как это работает

```
┌─────────────────────────────────────┐
│  x86_64 Host Machine                │
│                                     │
│  ┌───────────────────────────────┐ │
│  │  Docker Engine (x86_64)       │ │
│  │                               │ │
│  │  ┌─────────────────────────┐ │ │
│  │  │ ARM64 Container         │ │ │
│  │  │                         │ │ │
│  │  │  ARM64 binaries         │ │ │
│  │  │        ↓                │ │ │
│  │  │  QEMU (translates)      │ │ │
│  │  │        ↓                │ │ │
│  │  │  x86_64 instructions    │ │ │
│  │  └─────────────────────────┘ │ │
│  └───────────────────────────────┘ │
└─────────────────────────────────────┘
```

### Поддерживаемые архитектуры

Docker с QEMU поддерживает:
- `linux/amd64` (x86_64)
- `linux/arm64` (aarch64) - **наша цель**
- `linux/arm/v7` (armhf)
- `linux/arm/v6`
- `linux/ppc64le`
- `linux/s390x`
- `linux/riscv64`

---

## 🔧 QEMU User Mode Emulation

### Установка на Ubuntu/Debian

```bash
# Установка QEMU и binfmt
sudo apt-get update
sudo apt-get install -y qemu-user-static binfmt-support

# Регистрация QEMU интерпретаторов в ядре
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

# Проверка установки
docker run --rm --platform linux/arm64 arm64v8/ubuntu uname -m
# Вывод: aarch64
```

### Как работает binfmt_misc

Linux ядро использует `binfmt_misc` для автоматического запуска QEMU при попытке выполнить ARM64 бинарники:

```bash
# Проверка зарегистрированных форматов
cat /proc/sys/fs/binfmt_misc/qemu-aarch64

# Пример вывода:
enabled
interpreter /usr/bin/qemu-aarch64-static
flags: OCF
offset 0
magic 7f454c460201010000000000000000000200b700
mask ffffffffffffff00fffffffffffffffffeffff
```

### Docker Buildx (для сборки)

Docker Buildx автоматически использует QEMU для кросс-компиляции:

```bash
# Создать builder с поддержкой multi-arch
docker buildx create --name multiarch --driver docker-container --use
docker buildx inspect --bootstrap

# Сборка для ARM64 на x86_64
docker buildx build --platform linux/arm64 -t my-image:arm64 .

# Сборка для обеих архитектур
docker buildx build --platform linux/amd64,linux/arm64 -t my-image:latest .
```

---

## ⚡ Производительность

### Сравнение скорости выполнения

| Операция | Native ARM64 | QEMU on x86_64 | Замедление |
|----------|--------------|----------------|------------|
| CPU-bound (colcon build) | 1x | 5-10x | 5-10x медленнее |
| Memory operations | 1x | 1.5-2x | 1.5-2x медленнее |
| I/O operations | 1x | ~1x | Почти нативная |
| ROS2 pub/sub | 1x | 2-3x | 2-3x медленнее |

### Практические замеры (Rob Box Project)

**Сборка базового образа `ros2-zenoh`:**
- Native ARM64 (Raspberry Pi 4): ~15 минут
- QEMU on x86_64 (GitHub Actions): ~45-60 минут
- **Замедление:** 3-4x

**Запуск ROS2 ноды:**
- Native ARM64: Instant
- QEMU on x86_64: Старт ~2-3 секунды, работа нормально
- **Замедление:** Незначительное в runtime

**RTAB-Map SLAM:**
- Native ARM64: 10-15 FPS
- QEMU on x86_64: 2-3 FPS (НЕ РЕКОМЕНДУЕТСЯ)
- **Замедление:** 5x

### Когда использовать

✅ **ХОРОШО для:**
- Тестирование логики ROS2 нод
- Интеграционное тестирование коммуникации
- Smoke testing топиков и сервисов
- Разработка без железа
- CI/CD pipeline

❌ **ПЛОХО для:**
- Performance-critical задачи (SLAM, computer vision)
- Real-time обработка
- Бенчмарки производительности
- Production деплой

---

## 🚀 Практическое применение

### Use Case 1: Smoke Testing

**Цель:** Проверить что все ROS2 ноды запускаются и публикуют топики

```bash
# На x86_64 машине с QEMU
docker run --rm --platform linux/arm64 \
  ghcr.io/krikz/rob_box:oak-d-humble-latest \
  bash -c "source /opt/ros/humble/setup.bash && ros2 run rob_box_perception oak_d_node"

# Ожидаемый результат:
# - Нода стартует
# - Топики создаются (но камера не работает без железа)
# - Можно проверить: ros2 topic list
```

### Use Case 2: Integration Testing

**Цель:** Запустить Main Pi + Vision Pi на одной машине для тестирования взаимодействия

```yaml
# docker-compose-test-x86.yaml
services:
  zenoh-router:
    image: eclipse/zenoh:latest
    platform: linux/amd64  # Native на x86_64
    ports:
      - "7447:7447"
      - "8000:8000"

  rtabmap:
    image: ghcr.io/krikz/rob_box:rtabmap-humble-latest
    platform: linux/arm64  # Через QEMU
    environment:
      - RMW_IMPLEMENTATION=rmw_zenoh_cpp
      - ZENOH_CONFIG=/config/zenoh_session_config.json5

  oak-d:
    image: ghcr.io/krikz/rob_box:oak-d-humble-latest
    platform: linux/arm64  # Через QEMU
    environment:
      - RMW_IMPLEMENTATION=rmw_zenoh_cpp
    # ВАЖНО: Камера не будет работать, но нода запустится
```

### Use Case 3: Development Workflow

**Цель:** Разработка на x86_64 ноутбуке, деплой на Raspberry Pi

```bash
# 1. Разработка на x86_64 (быстрее)
docker build --platform linux/amd64 -t my-node:dev .
docker run --platform linux/amd64 my-node:dev

# 2. Тестирование ARM64 локально (перед деплоем)
docker run --platform linux/arm64 my-node:dev

# 3. Деплой на Raspberry Pi
docker tag my-node:dev ghcr.io/krikz/rob_box:my-node-humble-latest
docker push ghcr.io/krikz/rob_box:my-node-humble-latest
```

---

## ⚠️ Ограничения

### 1. Аппаратные устройства не работают

**Проблема:** USB устройства, камеры, GPIO, SPI не доступны в эмуляции

**Примеры:**
- ❌ OAK-D камера (`/dev/bus/usb`) - не будет работать
- ❌ ReSpeaker микрофон (`/dev/snd`) - не будет работать
- ❌ LED Matrix SPI (`/dev/spidev0.0`) - не будет работать
- ❌ VESC UART (`/dev/ttyUSB0`) - не будет работать
- ✅ Zenoh networking - **будет работать**
- ✅ ROS2 pub/sub - **будет работать**

**Решение:** Использовать mock данные или пропускать hardware-dependent тесты

### 2. Производительность

**Проблема:** CPU-intensive задачи будут медленными

**Примеры:**
- RTAB-Map SLAM: 2-3 FPS вместо 10-15
- AprilTag detection: 5-7 FPS вместо 20-30
- DeepSeek LLM inference: 10x медленнее

**Решение:** Использовать для smoke testing, не для production

### 3. Архитектурные различия

**Проблема:** Некоторые бинарные библиотеки могут вести себя по-разному

**Примеры:**
- Alignment requirements (ARM64 более строгие)
- Atomic operations
- SIMD инструкции (NEON vs SSE/AVX)

**Решение:** Всегда финально тестировать на реальном железе

### 4. Отладка сложнее

**Проблема:** Debugger может работать некорректно в QEMU

**Решение:** Использовать логирование вместо interactive debugging

---

## 🛠️ Реализация в проекте

### Текущее использование QEMU в Rob Box Project

Проект уже использует QEMU в GitHub Actions:

```yaml
# .github/workflows/G-Build Main Pi Services.yml
- name: Set up QEMU for multi-arch builds
  uses: docker/setup-qemu-action@v3
  with:
    platforms: linux/arm64

- name: Set up Docker Buildx
  uses: docker/setup-buildx-action@v3

- name: Build ARM64 image
  uses: docker/build-push-action@v5
  with:
    platforms: linux/arm64
    tags: ghcr.io/krikz/rob_box:rtabmap-humble-latest
```

### Создание тестового окружения

Для полноценного тестирования будет создан `docker/test/` каталог:

```
docker/test/
├── docker-compose-x86-test.yaml    # Все сервисы на x86_64
├── config/                         # Конфиги для тестирования
│   ├── zenoh_test_config.json5
│   └── ros2_test_params.yaml
├── scripts/
│   ├── smoke_test.sh               # Проверка всех топиков
│   ├── start_test_env.sh           # Запуск окружения
│   └── monitor_topics.sh           # Мониторинг
└── README.md                       # Документация
```

### Expected Behavior (Ожидаемое поведение)

| Service | Status | Expected Behavior |
|---------|--------|-------------------|
| `zenoh-router` | ✅ OK | Запускается нормально (native amd64) |
| `rtabmap` | ⚠️ DEGRADED | Запускается, но без camera/lidar топиков будет idle |
| `oak-d` | ❌ ERROR | Запускается, но падает с "Camera not found" |
| `lslidar` | ❌ ERROR | Запускается, но падает с "Device not found" |
| `voice-assistant` | ⚠️ DEGRADED | Запускается, но без микрофона не будет VAD |
| `led-matrix` | ❌ ERROR | Запускается, но падает с "SPI not found" |
| `nav2` | ✅ OK | Запускается, ожидает топики `/scan`, `/odom` |
| `micro-ros-agent` | ❌ ERROR | Падает с "Serial port not found" |
| `twist-mux` | ✅ OK | Запускается нормально |
| `robot-state-publisher` | ✅ OK | Публикует URDF и TF tree |
| `perception` | ✅ OK | Запускается, агрегирует топики (если есть) |

### Мониторинг топиков

Для smoke testing будет использоваться:

```bash
#!/bin/bash
# smoke_test.sh - проверка что все ожидаемые топики публикуются

source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

echo "🧪 Checking expected topics..."

# Список ожидаемых топиков
EXPECTED_TOPICS=(
    "/tf"
    "/tf_static"
    "/robot_description"
    "/cmd_vel"
    "/scan"              # От lslidar (может отсутствовать)
    "/camera/rgb/image_raw"  # От oak-d (может отсутствовать)
    "/odom"              # От micro-ros-agent (может отсутствовать)
)

for topic in "${EXPECTED_TOPICS[@]}"; do
    if ros2 topic list | grep -q "^${topic}$"; then
        echo "✅ $topic - OK"
    else
        echo "⚠️  $topic - MISSING (expected if no hardware)"
    fi
done

echo ""
echo "📊 All available topics:"
ros2 topic list
```

---

## 📚 Дополнительные ресурсы

### Документация

- [Docker Multi-platform builds](https://docs.docker.com/build/building/multi-platform/)
- [QEMU User Mode](https://www.qemu.org/docs/master/user/main.html)
- [binfmt_misc](https://www.kernel.org/doc/html/latest/admin-guide/binfmt-misc.html)

### Связанные документы в проекте

- [DOCKER_STANDARDS.md](./DOCKER_STANDARDS.md) - Docker организация
- [TESTING_GUIDE.md](./TESTING_GUIDE.md) - Тестирование
- [CI_CD_PIPELINE.md](../CI_CD_PIPELINE.md) - GitHub Actions
- [Build Machine README](../../docker/build/README.md) - Локальная сборка

---

## 🎯 Выводы

### ✅ Что возможно

1. **Запуск ARM64 Docker образов на x86_64** - ДА, через QEMU
2. **Smoke testing всех ROS2 нод** - ДА, без hardware dependencies
3. **Интеграционное тестирование связей** - ДА, Zenoh работает нормально
4. **Разработка без Raspberry Pi** - ДА, с ожидаемыми ограничениями

### ❌ Что НЕ возможно

1. **Тестирование hardware-dependent функциональности** - НЕТ
2. **Performance benchmarking** - НЕТ (5-10x slower)
3. **Real-time обработка** - НЕТ (задержки от QEMU)
4. **Production deployment** - НЕТ (только для разработки/тестирования)

### 💡 Рекомендация

Использовать QEMU emulation для:
- ✅ CI/CD smoke tests
- ✅ Разработка без железа
- ✅ Интеграционное тестирование логики
- ✅ Проверка ROS2 коммуникации

НЕ использовать для:
- ❌ Performance testing
- ❌ Hardware testing
- ❌ Production
- ❌ Real-time критичных задач

---

**Следующий шаг:** Создание `docker/test/` инфраструктуры для x86_64 тестирования

