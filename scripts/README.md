# Скрипты для управления Docker образами

Этот каталог содержит утилиты для управления Docker образами проекта Rob Box.

## Обзор скриптов

### 🚀 create_release.sh

**Назначение:** Автоматическое создание релизов с семантическим версионированием. Скрипт автоматически определяет текущую версию из git тегов и вычисляет следующую версию на основе типа релиза.

**Использование:**

```bash
# Интерактивный режим с меню выбора
./scripts/create_release.sh

# Автоматический режим - major релиз (X.0.0)
./scripts/create_release.sh major

# Автоматический режим - minor релиз (x.Y.0)
./scripts/create_release.sh minor

# Автоматический режим - patch релиз (x.y.Z)
./scripts/create_release.sh patch
```

**Что делает скрипт:**
1. Находит последний тег версии в формате `v?.?.?` или `?.?.?`
2. Предлагает выбрать тип релиза:
   - **Major** - несовместимые изменения API (1.0.0 → 2.0.0)
   - **Minor** - новая функциональность, обратно совместимо (1.0.0 → 1.1.0)
   - **Patch** - исправления ошибок (1.0.0 → 1.0.1)
3. Автоматически вычисляет следующую версию
4. Создаёт аннотированный git тег с описанием
5. Опционально отправляет тег в удалённый репозиторий

**Пример интерактивного режима:**
```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
🚀 Создание релиза РОББОКС
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

ℹ  Поиск существующих тегов версий...
✓  Найдена текущая версия: 1.2.5

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Выбор типа релиза
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Текущая версия: 1.2.5

Выберите тип релиза:

  1) Major (несовместимые изменения API)
     1.2.5 → 2.0.0

  2) Minor (новая функциональность, обратно совместимо)
     1.2.5 → 1.3.0

  3) Patch (исправления ошибок)
     1.2.5 → 1.2.6

  0) Отмена

Ваш выбор: 2

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Подтверждение
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Текущая версия: 1.2.5
Новая версия:   1.3.0
Тип релиза:     minor

Отправить тег в удалённый репозиторий сразу? (y/n): y

Создать релиз с версией 1.3.0? (y/n): y

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Создание тега
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

ℹ  Текущая ветка: main
ℹ  Создание тега: v1.3.0

Введите описание релиза (или нажмите Enter для использования по умолчанию):
> Added voice assistant improvements and bug fixes

✓  Тег v1.3.0 успешно создан
ℹ  Отправка тега в удалённый репозиторий...
✓  Тег v1.3.0 успешно отправлен в origin

✓  Релиз 1.3.0 успешно создан!

ℹ  Следующие шаги:
  1. Проверьте тег: git show v1.3.0
  2. Создайте release на GitHub: https://github.com/krikz/rob_box_project/releases/new?tag=v1.3.0
```

**Семантическое версионирование:**

Проект следует [Semantic Versioning 2.0.0](https://semver.org/):

- **Major (X.0.0)** - несовместимые изменения API
  - Примеры: удаление ROS топиков, изменение форматов сообщений, изменение публичного API
- **Minor (x.Y.0)** - новая функциональность, обратно совместимо
  - Примеры: новые сервисы, новые топики, новые параметры с значениями по умолчанию
- **Patch (x.y.Z)** - исправления ошибок, обратно совместимо
  - Примеры: баг фиксы, улучшения производительности, обновления документации

**Проверки безопасности:**
- Проверяет что вы в git репозитории
- Предупреждает если вы не на ветке main/master
- Проверяет наличие незакоммиченных изменений
- Требует подтверждения перед созданием тега

---

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

### 🔍 validate_zenoh_namespace.sh

**Назначение:** Проверка корректности конфигурации Zenoh namespace для облачной связности.

**Использование:**
```bash
# На Vision Pi или Main Pi
./scripts/validate_zenoh_namespace.sh
```

**Что проверяет:**
1. ✅ Наличие ROBOT_ID в .env файле
2. ✅ Валидность формата ROBOT_ID (только буквы, цифры, подчеркивания)
3. ✅ Запущенные Docker контейнеры
4. ✅ Переменная окружения ROBOT_ID в контейнерах
5. ✅ Сгенерированный namespace в `/tmp/zenoh_session_config.json5`
6. ✅ Логи контейнеров на подтверждение namespace
7. ✅ Доступность Zenoh router REST API
8. ✅ Наличие топиков с префиксом `robots/{ROBOT_ID}/`
9. ✅ Подключение к облачному роутеру (только Main Pi)

**Пример вывода:**
```
╔══════════════════════════════════════════════════════════════╗
║  Валидатор конфигурации Zenoh Namespace                     ║
╔══════════════════════════════════════════════════════════════╗

✅ Обнаружено: VISION Pi

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1. Проверка предварительных требований
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✓ docker найден
✓ curl найден
✓ grep найден

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
2. Проверка конфигурации ROBOT_ID
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✓ ROBOT_ID найден в .env: RBXU100001
✓ Формат ROBOT_ID корректен
  Ожидаемый namespace: robots/RBXU100001

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
3. Проверка Docker контейнеров
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✓ Контейнер zenoh-router-vision запущен
✓ Контейнер oak-d запущен
✓ Контейнер lslidar запущен
✓ Контейнер voice-assistant запущен

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
4. Проверка Namespace в контейнерах
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Проверка oak-d...
  ✓ Переменная окружения ROBOT_ID: RBXU100001
  ✓ Namespace в конфигурации: robots/RBXU100001
  ✓ Namespace подтверждён в логах контейнера

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
5. Проверка подключения к Zenoh Router
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✓ REST API локального Zenoh роутера доступен

Проверка топиков с префиксом namespace...
✓ Найдены топики с namespace robots/RBXU100001:
  → robots/RBXU100001/camera/rgb/image_raw
  → robots/RBXU100001/camera/depth/image_rect_raw
  → robots/RBXU100001/scan
  → robots/RBXU100001/odom
  → robots/RBXU100001/cmd_vel

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
6. Итоги
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Конфигурация Namespace:
  Robot ID:  RBXU100001
  Namespace: robots/RBXU100001
  Хост:      VISION Pi

Ожидаемый формат топиков в облаке:
  robots/RBXU100001/camera/rgb/image_raw
  robots/RBXU100001/cmd_vel
  robots/RBXU100001/odom

Примеры подписки в облаке:
  Все роботы: robots/**
  Этот робот: robots/RBXU100001/**
  Конкретные: robots/RBXU100001/camera/**

✅ Валидация завершена
```

**Связанная документация:** `docs/architecture/ZENOH_CLOUD_NAMESPACES.md`

---

## Дополнительные ресурсы

- **CI/CD Pipeline:** `docs/CI_CD_PIPELINE.md`
- **Docker Standards:** `docs/development/DOCKER_STANDARDS.md`
- **Agent Guide:** `docs/development/AGENT_GUIDE.md`
- **Zenoh Cloud Namespaces:** `docs/architecture/ZENOH_CLOUD_NAMESPACES.md`
- **GitHub Actions Workflows:** `.github/workflows/`

---

**Последнее обновление:** Октябрь 2025
**Автор:** Rob Box Project Team
