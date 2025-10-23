# Скрипты для управления Docker образами

Этот каталог содержит утилиты для управления Docker образами проекта Rob Box.

## Обзор скриптов

### 🔧 set-docker-tags.sh

**Назначение:** Автоматическое определение и установка правильных Docker тегов на основе текущей ветки Git.

**Использование:**
```bash
# Автоматическое определение ветки и установка тегов
source scripts/set-docker-tags.sh

# После выполнения будут установлены переменные окружения:
# - IMAGE_TAG (latest, dev, test, rc-X.Y.Z)
# - ROS_DISTRO (humble)
# - SERVICE_IMAGE_PREFIX (ghcr.io/krikz/rob_box)
```

**Логика определения тегов:**

| Ветка | IMAGE_TAG | Описание |
|-------|-----------|----------|
| `main` | `latest` | Production образы |
| `develop` | `dev` | Development образы |
| `feature/*` | `test` | Feature testing образы |
| `release/1.0.0` | `rc-1.0.0` | Release candidate |
| `hotfix/1.0.1` | `hotfix-1.0.1` | Hotfix образы |
| Другие | `test` | По умолчанию testing |

**Что делает скрипт:**
1. Определяет текущую ветку Git
2. Выбирает соответствующий `.env` файл из `docker/` каталога
3. Копирует его в `docker/vision/.env` и `docker/main/.env`
4. Устанавливает переменные окружения для текущей shell сессии

**Пример вывода:**
```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
🔧 Настройка Docker тегов для РОББОКС
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Текущая ветка: develop
Конфигурация: Development (develop branch)
Env файл: .env.develop
Image tag: dev

✅ Скопирован docker/.env.develop → docker/vision/.env
✅ Скопирован docker/.env.develop → docker/main/.env

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✅ Docker теги настроены успешно!
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Будут использоваться образы с тегом: -humble-dev

Примеры:
  - ghcr.io/krikz/rob_box:voice-assistant-humble-dev
  - ghcr.io/krikz/rob_box:oak-d-humble-dev
  - ghcr.io/krikz/rob_box:rtabmap-humble-dev
```

---

### 🔨 local-build.sh

**Назначение:** Локальная сборка Docker образов для ускорения разработки.

**Использование:**
```bash
# Собрать один сервис
./scripts/local-build.sh voice-assistant

# Собрать все Vision Pi сервисы
./scripts/local-build.sh vision

# Собрать все Main Pi сервисы
./scripts/local-build.sh main

# Собрать все сервисы
./scripts/local-build.sh all

# Собрать для конкретной платформы (по умолчанию linux/arm64)
./scripts/local-build.sh voice-assistant linux/amd64

# Показать справку
./scripts/local-build.sh help
```

**Доступные сервисы:**

**Vision Pi:**
- `oak-d` - OAK-D camera driver
- `lslidar` - LSLIDAR N10 driver
- `apriltag` - AprilTag detector
- `led-matrix` - LED matrix driver
- `voice-assistant` - Voice assistant + animations
- `perception` - Perception & dialogue system

**Main Pi:**
- `robot-state-publisher` - Robot state publisher
- `rtabmap` - RTAB-Map SLAM
- `twist-mux` - Twist multiplexer
- `micro-ros-agent` - Micro-ROS agent
- `ros2-control` - ROS2 Control + VESC
- `nav2` - Nav2 navigation stack

**Переменные окружения:**
```bash
# Тег образа (по умолчанию: local)
export IMAGE_TAG=local

# ROS дистрибутив (по умолчанию: humble)
export ROS_DISTRO=humble

# Платформа (по умолчанию: linux/arm64)
export PLATFORM=linux/arm64
```

**Пример вывода:**
```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
🔨 Локальная сборка Docker образов РОББОКС
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Ветка: develop
Платформа: linux/arm64
Тег: local
ROS дистрибутив: humble

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📦 Сборка сервиса: voice-assistant
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Контекст: /path/to/rob_box_project
Dockerfile: /path/to/docker/vision/voice_assistant/Dockerfile
Образ: ghcr.io/krikz/rob_box:voice-assistant-humble-local

[build output...]

✅ Успешно собран: ghcr.io/krikz/rob_box:voice-assistant-humble-local
```

**Производительность:**

| Сценарий | Время сборки | Примечание |
|----------|--------------|------------|
| x86_64 → x86_64 | ~2-5 мин | Нативная сборка |
| ARM64 → ARM64 | ~5-10 мин | Нативная сборка на Raspberry Pi |
| x86_64 → ARM64 | ~20-60 мин | Через QEMU (медленно!) |

**Рекомендации:**
- ✅ Используйте для разработки на той же архитектуре
- ✅ Идеально для быстрой итерации кода
- ⚠️ Для cross-compilation лучше использовать GitHub Actions
- ⚠️ Локальные образы не публикуются в ghcr.io автоматически

---

## Типичные сценарии использования

### Сценарий 1: Разработка новой фичи

```bash
# 1. Создать feature ветку
git checkout develop
git pull
git checkout -b feature/my-awesome-feature

# 2. Настроить теги (автоматически установит IMAGE_TAG=test)
source scripts/set-docker-tags.sh

# 3. Внести изменения в код
# ... редактирование файлов ...

# 4. Собрать образ локально для быстрого тестирования
IMAGE_TAG=local ./scripts/local-build.sh voice-assistant

# 5. Протестировать локально
cd docker/vision
IMAGE_TAG=local docker-compose up voice-assistant

# 6. Если всё работает - запушить в GitHub
git add .
git commit -m "feat: implement awesome feature"
git push origin feature/my-awesome-feature

# 7. GitHub Actions автоматически:
#    - Соберёт образ с тегом -humble-test
#    - Запустит тесты
#    - Мерджнет в develop если успешно
```

### Сценарий 2: Деплой в production

```bash
# На Raspberry Pi Vision (10.1.1.21)
ssh ros2@10.1.1.21

# Переключиться на main ветку
cd ~/rob_box_project
git checkout main
git pull

# Настроить теги для production (IMAGE_TAG=latest)
source scripts/set-docker-tags.sh

# Скачать последние образы
cd docker/vision
docker-compose pull

# Запустить сервисы
docker-compose up -d

# Проверить статус
docker-compose ps
docker logs -f voice-assistant
```

### Сценарий 3: Тестирование develop образов

```bash
# На Raspberry Pi
ssh ros2@10.1.1.21
cd ~/rob_box_project

# Переключиться на develop
git checkout develop
git pull

# Настроить теги для development (IMAGE_TAG=dev)
source scripts/set-docker-tags.sh

# Скачать и запустить dev образы
cd docker/vision
docker-compose pull
docker-compose up -d

# Мониторинг
./realtime_monitor.sh
```

### Сценарий 4: Локальная разработка на x86_64

```bash
# На локальной машине (не Raspberry Pi)
cd /path/to/rob_box_project

# Настроить теги
source scripts/set-docker-tags.sh

# Собрать для x86_64 (быстро)
./scripts/local-build.sh voice-assistant linux/amd64

# Запустить локально
cd docker/vision
IMAGE_TAG=local docker-compose up voice-assistant

# Итеративная разработка
# 1. Редактировать код
# 2. Пересобрать: ./scripts/local-build.sh voice-assistant linux/amd64
# 3. Перезапустить: docker-compose up voice-assistant
# 4. Тестировать
# 5. Повторить
```

---

## Интеграция с существующими скриптами

### update_and_restart.sh

Скрипт деплоя на Raspberry Pi уже использует переменные окружения:

```bash
# В docker/vision/scripts/update_and_restart.sh
source ../../scripts/set-docker-tags.sh
docker-compose pull
docker-compose up -d
```

### diagnose.sh

Диагностический скрипт проверяет какие образы используются:

```bash
# Проверить текущие образы
docker-compose config | grep image:

# Должно показать правильные теги на основе .env файла
```

---

## Troubleshooting

### Проблема: Неправильный IMAGE_TAG

**Симптомы:**
```
docker-compose pull
ERROR: pull access denied for ghcr.io/krikz/rob_box:voice-assistant-humble-wrong
```

**Решение:**
```bash
# Проверить текущий IMAGE_TAG
echo $IMAGE_TAG

# Если пустой или неправильный - перезапустить скрипт
source scripts/set-docker-tags.sh

# Проверить что .env файлы обновились
cat docker/vision/.env | grep IMAGE_TAG
cat docker/main/.env | grep IMAGE_TAG
```

### Проблема: Локальные образы не используются

**Симптомы:**
```
docker-compose up
# Использует ghcr.io образы вместо локальных
```

**Решение:**
```bash
# Убедиться что IMAGE_TAG=local установлен
export IMAGE_TAG=local

# Проверить что образ существует локально
docker images | grep rob_box

# Запустить с явным IMAGE_TAG
IMAGE_TAG=local docker-compose up voice-assistant
```

### Проблема: Медленная сборка ARM64 на x86_64

**Симптомы:**
```
./scripts/local-build.sh voice-assistant linux/arm64
# Зависает на 20+ минут
```

**Решение:**
```bash
# Вариант 1: Собрать для текущей архитектуры
./scripts/local-build.sh voice-assistant linux/amd64

# Вариант 2: Использовать GitHub Actions для ARM64
git push  # GitHub Actions соберёт за ~10 минут

# Вариант 3: Собрать на Raspberry Pi нативно
sshpass -p 'open' ssh ros2@10.1.1.21
cd ~/rob_box_project
./scripts/local-build.sh voice-assistant  # Нативный ARM64
```

---

## Дополнительные ресурсы

- **CI/CD Pipeline:** `docs/CI_CD_PIPELINE.md`
- **Docker Standards:** `docs/development/DOCKER_STANDARDS.md`
- **Agent Guide:** `docs/development/AGENT_GUIDE.md`
- **GitHub Actions Workflows:** `.github/workflows/`

---

**Последнее обновление:** Октябрь 2025
**Автор:** Rob Box Project Team
