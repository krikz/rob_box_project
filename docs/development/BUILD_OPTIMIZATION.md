# Оптимизация сборки Docker образов

## Проблема

Сборка некоторых образов (особенно lslidar и apriltag) занимает много времени. При этом часто изменения касаются только конфигов или скриптов, которые не требуют пересборки всего образа.

## Решение: Volume Mounting вместо COPY

### ❌ Неправильно (старый подход)

```dockerfile
# Dockerfile
COPY lslidar/config/lsx10_custom.yaml /config/lsx10_custom.yaml
COPY lslidar/scripts/start_lslidar.sh /start_lslidar.sh
RUN chmod +x /start_lslidar.sh
```

**Проблемы:**
- Любое изменение конфига или скрипта инвалидирует Docker cache
- Требуется пересборка образа даже для мелких правок
- Долгое время развертывания изменений

### ✅ Правильно (текущий подход)

```dockerfile
# Dockerfile - НЕ содержит COPY конфигов/скриптов
# Комментарий объясняет структуру volumes
# ВАЖНО: Конфиги и скрипты НЕ копируются в образ!
# Они монтируются через volumes в docker-compose.yaml
```

```yaml
# docker-compose.yaml
services:
  lslidar:
    image: ghcr.io/krikz/rob_box:lslidar-humble-latest
    volumes:
      - ./config:/config/shared:ro              # Общие конфиги (zenoh, cyclonedds)
      - ./config/lslidar:/config/lslidar:ro     # Конфиги lslidar
      - ./scripts/lslidar:/scripts:ro           # Скрипты запуска
    command: ["/scripts/start_lslidar.sh"]
```

**Преимущества:**
- ✅ Изменения конфигов применяются мгновенно (просто restart контейнера)
- ✅ Не требуется пересборка образа
- ✅ Образ собирается один раз и переиспользуется
- ✅ Быстрое развертывание обновлений

## Структура папок

Стандартная структура для каждого Pi:

```
docker/
├── main/                          # Для Main Pi (10.1.1.20)
│   ├── config/                    # Shared конфиги для всех контейнеров
│   │   ├── cyclonedds.xml         # → /config/shared/cyclonedds.xml
│   │   ├── zenoh_session_config.json5  # → /config/shared/zenoh_session_config.json5
│   │   └── rtabmap/               # Конфиги специфичные для rtabmap
│   │       └── rtabmap.yaml       # → /config/rtabmap/rtabmap.yaml
│   ├── scripts/
│   │   └── robot_state_publisher/ # Скрипты запуска
│   │       └── start_robot_state_publisher.sh  # → /scripts/start_robot_state_publisher.sh
│   ├── rtabmap/
│   │   └── Dockerfile
│   └── robot_state_publisher/
│       └── Dockerfile
│
└── vision/                        # Для Vision Pi (10.1.1.21)
    ├── config/                    # Shared конфиги
    │   ├── cyclonedds.xml
    │   ├── zenoh_session_config.json5
    │   ├── apriltag/              # → /config/apriltag/
    │   │   └── apriltag_config.yaml
    │   ├── lslidar/               # → /config/lslidar/
    │   │   ├── lslidar_headless_launch.py
    │   │   └── lsx10_custom.yaml
    │   └── oak-d/                 # → /config/oak-d/
    │       └── oak_d_config.yaml
    ├── scripts/
    │   ├── apriltag/              # → /scripts/
    │   │   └── start_apriltag.sh
    │   ├── lslidar/               # → /scripts/
    │   │   └── start_lslidar.sh
    │   ├── voice_assistant/       # → /scripts/
    │   │   └── start_voice_assistant.sh
    │   └── oak-d/                 # → /scripts/
    │       └── start_oak_d.sh
    ├── apriltag/
    │   └── Dockerfile
    ├── lslidar/
    │   └── Dockerfile
    └── oak-d/
        ├── Dockerfile
        └── launch/                # → /oak-d/launch/
            └── oakd_apriltag_only.launch.py
```

**Ключевые моменты:**
- `config/` - shared конфиги (zenoh, cyclonedds)
- `config/{service}/` - конфиги специфичные для сервиса
- `scripts/{service}/` - скрипты запуска сервиса
- `{service}/Dockerfile` - только Dockerfile и необходимые для сборки файлы

## Правила для Dockerfiles

### 1. Образы должны содержать только НЕИЗМЕНЯЕМУЮ инфраструктуру

В Dockerfile должно быть:
- ✅ Установка пакетов через `apt-get install`
- ✅ Клонирование git репозиториев драйверов
- ✅ Компиляция ROS пакетов через `colcon build`
- ✅ Установка Python зависимостей через `pip`
- ❌ НЕТ копирования конфигов (COPY config/)
- ❌ НЕТ копирования скриптов (COPY scripts/)
- ❌ НЕТ копирования launch файлов (COPY launch/)
- ❌ НЕТ копирования URDF (COPY src/)

### 2. Шаблон комментария в Dockerfile

Добавляйте в конец каждого Dockerfile:

```dockerfile
# ВАЖНО: Конфиги и скрипты НЕ копируются в образ!
# Они монтируются через volumes в docker-compose.yaml:
# - ./config:/config/shared:ro (общие конфиги: zenoh, cyclonedds)
# - ./{service}/config:/config/{service}:ro (конфиги сервиса)
# - ./{service}/scripts:/scripts:ro (скрипты запуска)
# - ./{service}/launch:/launch:ro (launch файлы, если нужно)

CMD ["/scripts/start_{service}.sh"]
```

### 3. Оптимизация порядка инструкций

Docker кэширует слои построчно. Располагайте инструкции от наименее изменяемых к наиболее изменяемым:

```dockerfile
# 1. Базовый образ (почти никогда не меняется)
FROM ghcr.io/krikz/rob_box_base:ros2-zenoh

# 2. Установка системных пакетов (редко меняется)
RUN apt-get update && apt-get install -y \
    ros-humble-apriltag-ros \
    && rm -rf /var/lib/apt/lists/*

# 3. Клонирование и сборка (редко меняется)
RUN git clone --depth 1 https://github.com/repo.git && \
    colcon build --symlink-install

# 4. Volumes с конфигами/скриптами (часто меняются, но НЕ в Dockerfile!)
# Монтируются в docker-compose.yaml
```

## Оптимизация размера контекста сборки

### Используйте .dockerignore

Создайте `.dockerignore` в папке с docker-compose.yaml:

```
# docker/vision/.dockerignore
*                          # Игнорируем всё по умолчанию

# Разрешаем только нужные папки
!apriltag/
!lslidar/
!oak-d/
!zenoh-router/

        └── Dockerfile
**/.git
**/.vscode
**/node_modules
```
- `config/audio/` - shared runtime-конфиги для нескольких сервисов одного стека

Это ускоряет передачу контекста в Docker daemon.

## Пример оптимизированного Dockerfile (lslidar)

```dockerfile
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:pcl
FROM ${BASE_IMAGE}

# Установка зависимостей для сборки
RUN apt-get update && apt-get install -y \
    ros-humble-rosidl-default-generators \
    ros-humble-builtin-interfaces \
    ros-humble-ament-cmake \
    && rm -rf /var/lib/apt/lists/*

# Клонирование и сборка драйвера
WORKDIR /ws/src
RUN git clone --depth 1 -b M10P/N10P https://github.com/Lslidar/lslidar_ros2_driver.git

WORKDIR /ws
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# Volumes монтируются в docker-compose.yaml
CMD ["/scripts/start_lslidar.sh"]
```

**Результат:**
- Образ собирается один раз
- При изменении конфига/скрипта: `docker-compose restart lslidar` (2-3 секунды)
- Без изменения: `docker-compose pull && docker-compose up -d lslidar` (30-60 секунд)
- С пересборкой образа: `docker-compose build lslidar` (5-10 минут для lslidar!)

## Быстрое применение изменений

### Изменили конфиг или скрипт:

```bash
# Просто перезапустить контейнер
docker-compose restart lslidar
# или
docker restart lslidar
```

Время: **2-5 секунд**

### Изменили код в Dockerfile:

```bash
# Пересобрать локально и запустить
docker-compose build lslidar
docker-compose up -d lslidar
```

Время: **5-10 минут**

### Получить обновленный образ из registry:

```bash
# Скачать новый образ и перезапустить
docker-compose pull lslidar
docker-compose up -d lslidar
```

Время: **30-60 секунд**

## CI/CD Pipeline

GitHub Actions автоматически собирает образы при изменениях в `docker/{main,vision}/**`:

```yaml
# .github/workflows/build-vision-services.yml
on:
  push:
    branches: [develop, main]
    paths:
      - 'docker/vision/**'  # Триггер на изменения в Dockerfiles
      - '!docker/vision/**/config/**'   # НЕ триггерить на конфиги
      - '!docker/vision/**/scripts/**'  # НЕ триггерить на скрипты
```

**Важно:** Изменения в `config/` и `scripts/` НЕ должны триггерить сборку, так как они монтируются через volumes.

## Проверка перед коммитом

Перед коммитом убедитесь:

```bash
# 1. В Dockerfile нет COPY конфигов/скриптов
grep -r "COPY.*config\|COPY.*scripts" docker/*/*/Dockerfile

# Если что-то найдено - это ошибка!

# 2. docker-compose.yaml монтирует все нужные volumes
grep -A 10 "volumes:" docker/*/docker-compose.yaml

# Должны быть:
# - ./config:/config/shared:ro
# - ./{service}/config:/config/{service}:ro
# - ./{service}/scripts:/scripts:ro
```

## Оптимизация загрузки моделей ML/AI

### Проблема

Модели машинного обучения (Vosk STT, Silero TTS) имеют большой размер (145 MB) и загружаются из внешних источников во время Docker build. Это замедляет сборку на 10+ минут при каждом изменении.

### Решение: BuildKit Cache Mounts

Используйте Docker BuildKit cache mounts для кеширования загруженных файлов между сборками:

```dockerfile
# ❌ ПЛОХО - скачивается каждый раз при пересборке
RUN wget -O /models/model.pt https://example.com/model.pt

# ✅ ХОРОШО - кешируется между сборками
RUN --mount=type=cache,target=/model_cache,sharing=locked \
    mkdir -p /models && \
    # Проверяем кеш перед загрузкой
    if [ -f /model_cache/model.pt ]; then \
        echo "✅ Using cached model"; \
        cp /model_cache/model.pt /models/model.pt; \
    else \
        echo "⬇️ Downloading model..."; \
        wget -O /models/model.pt https://example.com/model.pt && \
        cp /models/model.pt /model_cache/model.pt; \
    fi
```

**Преимущества:**
- ✅ Первая сборка: скачивает модели и сохраняет в кеш (10+ минут)
- ✅ Последующие сборки: копирует из кеша (секунды)
- ✅ Работает для локальных сборок и GitHub Actions
- ✅ Кеш сохраняется между разными Docker образами
- ✅ Не требует коммита больших файлов в Git

**Где используется:**
- `docker/vision/voice_base/Dockerfile` - тяжёлые apt/pip зависимости, Vosk + Silero модели, ReSpeaker drivers, `audio_common_msgs`
- `docker/vision/voice_resources/Dockerfile` - большие Renardo/FoxDot sample packs для shared volume
- `docker/vision/voice_assistant/Dockerfile` - только fast-changing app layer (`rob_box_voice`, `rob_box_animations`, `rob_box_mcp_tools`)

### Рекомендуемая схема слоёв для voice stack

Для сервисов voice на Vision Pi теперь используйте три уровня:

1. **`voice-base`** — всё медленно меняющееся и тяжёлое:
    - apt/pip зависимости;
    - offline STT/TTS модели;
    - ReSpeaker drivers;
    - ROS message dependencies.
2. **`voice-resources`** — большие immutable ресурсы, не нужные для пересборки app layer:
    - Renardo/FoxDot sample packs;
    - one-shot инициализация named volume.
3. **`voice-assistant`** — только код и package metadata:
    - Python/ROS пакеты проекта;
    - launch/config/prompts, которые реально нужны при сборке пакета;
    - без скачивания моделей и sample packs.

### Правило для mutable ресурсов

Если ресурс:
- часто меняется,
- текстовый,
- не нужен во время `colcon build`,

то его нужно монтировать через volume, а не запекать в образ. Примеры:
- `sound_pack/`
- runtime config из `docker/vision/config/`
- startup scripts из `docker/vision/scripts/`

### Правило для CI trigger paths

Workflow должен следить не только за `docker/vision/**`, но и за реальными входами сборки:
- `src/rob_box_voice/**`
- `src/rob_box_animations/**`
- `src/rob_box_perception_msgs/**`
- `src/rob_box_mcp_tools/**`
- `src/rob_box_telegram/**`
- `src/ros2leds/**`

**Важно:** 
- Используйте `sharing=locked` для безопасности при параллельных сборках
- BuildKit включен по умолчанию в GitHub Actions (`docker/setup-buildx-action@v3`)
- Локальные сборки требуют `DOCKER_BUILDKIT=1` (или Docker 23.0+)

## Документация

См. также:
- **[DOCKER_STANDARDS.md](DOCKER_STANDARDS.md)** - Стандарты Docker Compose
- **[Архитектура системы](../architecture/SYSTEM_OVERVIEW.md)** - Архитектура проекта
- **[AGENT_GUIDE.md](AGENT_GUIDE.md)** - Руководство для AI агентов (содержит эти правила)

---

**Создано**: 2025-10-10  
**Обновлено**: 2026-03-08 (добавлены `voice-base`/`voice-resources` layering rules)  
**Автор**: КУКОРЕКЕН  
**Проект**: rob_box_project
