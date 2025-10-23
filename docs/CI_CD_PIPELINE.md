# CI/CD Pipeline Documentation

Автоматизированная система сборки и деплоя для rob_box_project.

## Архитектура Pipeline

```
┌─────────────────┐
│ Feature Branch  │
│ (feature/*)     │
└────────┬────────┘
         │ push
         ▼
┌─────────────────────────────────────────────────┐
│ GitHub Actions:                                 │
│ auto-merge-feature-to-develop.yml               │
│                                                 │
│ 1. Detect changes (vision/main/docs)           │
│ 2. Build changed services                      │
│ 3. Auto-merge to develop (if success)          │
│ 4. Delete feature branch                       │
└────────┬────────────────────────────────────────┘
         │ auto-merge
         ▼
┌─────────────────┐
│ Develop Branch  │
│ (develop)       │
└────────┬────────┘
         │ push
         ▼
┌─────────────────────────────────────────────────┐
│ GitHub Actions:                                 │
│ auto-merge-to-main.yml                          │
│                                                 │
│ 1. Build ALL services (build-all.yml)          │
│ 2. Auto-merge to main (if all success)         │
│ 3. Create release tag                          │
└────────┬────────────────────────────────────────┘
         │ auto-merge
         ▼
┌─────────────────┐
│ Main Branch     │
│ (main)          │
└────────┬────────┘
         │
         ▼
┌─────────────────────────────────────────────────┐
│ Docker Images Published                         │
│ ghcr.io/krikz/rob_box:*-humble-latest           │
└─────────────────────────────────────────────────┘
```

## Workflows

### 1. Auto-merge Feature to Develop

**Файл:** `.github/workflows/auto-merge-feature-to-develop.yml`

**Триггер:**
```yaml
push:
  branches:
    - 'feature/**'
    - 'feat/**'
```

**Логика:**
1. **Detect Changes** - определяет какие сервисы изменились
   - Vision Pi (`docker/vision/**`)
   - Main Pi (`docker/main/**`)
   - Documentation (`docs/**`)

2. **Conditional Build** - собирает только изменённые сервисы
   - `build-vision` - если изменились Vision Pi сервисы
   - `build-main` - если изменились Main Pi сервисы
   - Пропускает сборку если изменилась только документация

3. **Auto-merge** - после успешной сборки
   ```bash
   git checkout develop
   git merge --no-ff feature/branch-name
   git push origin develop
   git push origin --delete feature/branch-name
   ```

4. **Delete Feature Branch** - удаляет feature ветку после мерджа

**Результат:**
- Feature ветка автоматически вливается в `develop`
- Docker images: `*-humble-dev`
- Feature ветка удаляется

### 2. Auto-merge Develop to Main

**Файл:** `.github/workflows/auto-merge-to-main.yml`

**Триггер:**
```yaml
push:
  branches:
    - develop
```

**Логика:**
1. **Build All Services** - собирает ВСЕ сервисы через `build-all.yml`
   - Base images (ros2-zenoh)
   - Vision Pi (oak-d, lslidar, apriltag, led-matrix, voice-assistant)
   - Main Pi (micro-ros-agent, zenoh-router)

2. **Auto-merge** - если ВСЕ сборки успешны
   ```bash
   git checkout main
   git merge --no-ff develop
   git push origin main
   ```

3. **Create Release Tag**
   ```bash
   git tag -a "release-YYYYMMDD-HHMMSS"
   git push origin "release-YYYYMMDD-HHMMSS"
   ```

**Результат:**
- `develop` автоматически вливается в `main`
- Docker images: `*-humble-latest`
- Создаётся release tag

### 3. Build Vision Services

**Файл:** `.github/workflows/build-vision-services.yml`

**Сервисы:**
- `oak-d` - OAK-D camera
- `lslidar` - LSLIDAR N10
- `apriltag` - AprilTag detector
- `led-matrix` - NeoPixel LED matrix driver
- `voice-assistant` - Voice assistant + animations

**Платформа:** `linux/arm64` (Raspberry Pi 5)

### 4. Build Main Services

**Файл:** `.github/workflows/build-main-services.yml`

**Сервисы:**
- `micro-ros-agent` - micro-ROS bridge
- `zenoh-router` - Zenoh DDS router

**Платформа:** `linux/arm64` (Raspberry Pi 5)

### 5. Build Base Images

**Файл:** `.github/workflows/build-base-images.yml`

**Образы:**
- `ros2-zenoh-humble` - ROS2 Humble + Zenoh

### 6. Build All

**Файл:** `.github/workflows/build-all.yml`

Вызывает все остальные workflows:
- `build-base-images.yml`
- `build-vision-services.yml`
- `build-main-services.yml`

## Docker Image Tags

### Tag Naming Convention

```
ghcr.io/krikz/rob_box:<service>-<distro>-<version>
```

**Примеры:**
- `ghcr.io/krikz/rob_box:voice-assistant-humble-latest`
- `ghcr.io/krikz/rob_box:voice-assistant-humble-dev`
- `ghcr.io/krikz/rob_box:voice-assistant-humble-test`
- `ghcr.io/krikz/rob_box:voice-assistant-humble-abc1234` (SHA)

### Tags по веткам

| Ветка | Tag | Описание | Env файл |
|-------|-----|----------|----------|
| `main` | `humble-latest` | Продакшн, стабильная версия | `.env.main` |
| `develop` | `humble-dev` | Development, тестирование | `.env.develop` |
| `feature/*` | `humble-test` | Feature testing | `.env.feature` |
| `release/X.X.X` | `humble-rc-X.X.X` | Release candidate | `.env.develop` + override |
| `hotfix/*` | `humble-hotfix-X.X.X` | Hotfix | `.env.main` + override |
| SHA commit | `humble-<sha>` | Специфичная версия | - |

### Автоматическое управление тегами

Docker-compose файлы теперь используют переменные окружения для определения тегов:

```yaml
services:
  voice-assistant:
    image: ${SERVICE_IMAGE_PREFIX}:voice-assistant-${ROS_DISTRO}-${IMAGE_TAG}
```

**Настройка тегов для текущей ветки:**

```bash
# Автоматически определить ветку и установить правильный IMAGE_TAG
source scripts/set-docker-tags.sh

# Или вручную установить нужный тег
export IMAGE_TAG=dev  # или latest, test, rc-1.0.0
```

**Готовые конфигурации в env файлах:**

- `docker/.env.main` → `IMAGE_TAG=latest` (production)
- `docker/.env.develop` → `IMAGE_TAG=dev` (development)
- `docker/.env.feature` → `IMAGE_TAG=test` (testing)

## Workflow для разработчика

### Создание новой фичи

```bash
# 1. Создать feature ветку от develop
git checkout develop
git pull origin develop
git checkout -b feature/my-awesome-feature

# 2. Разработка
# ... внести изменения ...

# 3. Коммит и push
git add .
git commit -m "feat: add awesome feature"
git push origin feature/my-awesome-feature

# 4. GitHub Actions автоматически:
#    - Соберёт изменённые сервисы
#    - Мерджнет в develop (если сборка успешна)
#    - Удалит feature ветку
```

**Результат:** Feature автоматически в `develop`, образы с тегом `-dev` доступны для тестирования.

### Релиз в production

```bash
# 1. Убедиться что develop стабилен
# - Протестировать на реальном железе
# - Проверить все сервисы

# 2. Push в develop (если ещё не запушено)
git push origin develop

# 3. GitHub Actions автоматически:
#    - Соберёт ВСЕ сервисы
#    - Мерджнет develop в main (если всё успешно)
#    - Создаст release tag
#    - Опубликует образы с тегом -latest
```

**Результат:** Production образы с тегом `-latest` готовы к деплою.

### Hotfix

```bash
# 1. Создать hotfix ветку от main
git checkout main
git pull origin main
git checkout -b hotfix/critical-fix

# 2. Исправление
# ... внести изменения ...

# 3. Коммит и push
git add .
git commit -m "fix: critical bug"
git push origin hotfix/critical-fix

# 4. Создать PR в main вручную
# 5. После мерджа в main - cherry-pick в develop
git checkout develop
git cherry-pick <commit-hash>
git push origin develop
```

## Ручное управление

### Отключить auto-merge

Если нужно отключить автомердж, закомментировать trigger в workflow:

```yaml
# Отключить auto-merge feature to develop
on:
  push:
    branches:
      - 'DISABLED-feature/**'  # Добавить DISABLED-
```

### Ручной trigger сборки

```bash
# Через GitHub CLI
gh workflow run build-vision-services.yml --ref develop

# Или через web interface
# GitHub → Actions → Build Vision Pi Services → Run workflow
```

### Откат изменений

```bash
# Откатить develop к предыдущему коммиту
git checkout develop
git reset --hard HEAD~1
git push origin develop --force

# Откатить main (ОСТОРОЖНО!)
git checkout main
git reset --hard <commit-hash>
git push origin main --force

# Лучше создать revert commit
git checkout main
git revert <bad-commit-hash>
git push origin main
```

## Мониторинг Pipeline

### GitHub Actions UI

1. Перейти на GitHub → Actions
2. Выбрать workflow
3. Просмотреть историю запусков
4. Проверить логи

### Уведомления

Настроить GitHub Notifications:
- Settings → Notifications
- Watch repository
- Custom: Actions

### Статусы

Проверить статус сборки:
```bash
# Через GitHub CLI
gh run list --workflow=build-vision-services.yml

# Последний статус
gh run view --log
```

## Troubleshooting

### Feature ветка не мерджится

**Причины:**
1. Сборка упала - проверить логи в Actions
2. Конфликт мерджа - нужен ручной мердж
3. Workflow disabled - проверить `.github/workflows/`

**Решение:**
```bash
# Ручной мердж
git checkout develop
git pull
git merge feature/my-feature
# Resolve conflicts
git push origin develop
```

### Develop не мерджится в main

**Причины:**
1. Хотя бы одна сборка упала - все сервисы должны собираться
2. Конфликт мерджа

**Решение:**
```bash
# Проверить что всё собирается
gh workflow run build-all.yml --ref develop

# Ручной мердж если нужно
git checkout main
git pull
git merge develop
# Resolve conflicts
git push origin main
```

### Docker образы не публикуются

**Причины:**
1. Нет прав на ghcr.io
2. Сборка упала
3. Wrong platform (должен быть linux/arm64)

**Решение:**
```bash
# Проверить секреты
gh secret list

# Проверить permissions в workflow
# permissions:
#   contents: read
#   packages: write
```

### Неправильные теги Docker образов

**Проблема:** docker-compose использует `-latest` образы вместо `-dev` или `-test`

**Решение:**
```bash
# Автоматическая настройка на основе текущей ветки
cd /path/to/rob_box_project
source scripts/set-docker-tags.sh

# Проверить что IMAGE_TAG установлен правильно
echo $IMAGE_TAG

# Или вручную установить нужный тег
export IMAGE_TAG=dev
```

## Локальная разработка и сборка

### Локальная сборка Docker образов

Для ускорения разработки можно собирать образы локально, не дожидаясь GitHub Actions:

```bash
# Собрать один сервис
./scripts/local-build.sh voice-assistant

# Собрать все Vision Pi сервисы
./scripts/local-build.sh vision

# Собрать все Main Pi сервисы
./scripts/local-build.sh main

# Собрать все сервисы
./scripts/local-build.sh all

# Собрать для конкретной платформы
./scripts/local-build.sh voice-assistant linux/amd64
```

**Примечание:** Локальная сборка создаёт образы с тегом `IMAGE_TAG=local`. Они будут использованы если запустить `docker-compose` с `IMAGE_TAG=local`.

### Запуск GitHub Actions локально с act

Установите [act](https://github.com/nektos/act) для запуска workflows локально:

```bash
# Установка
brew install act  # macOS
curl https://raw.githubusercontent.com/nektos/act/master/install.sh | sudo bash  # Linux

# Список всех workflows
act -l

# Запуск конкретного workflow (dry run)
act -W .github/workflows/build-vision-services.yml -n

# Запуск конкретного job
act -j build-oak-d

# Запуск с секретами
echo "GITHUB_TOKEN=ghp_xxx" > .secrets
act --secret-file .secrets
```

**Важно:** 
- Сборка ARM64 образов на x86_64 будет очень медленной через QEMU
- Для разработки рекомендуется использовать `./scripts/local-build.sh` вместо act
- act полезен для тестирования логики workflows, но не для реальной сборки образов

### Быстрая итерация при разработке

**Сценарий 1: Разработка на x86_64, деплой на Raspberry Pi**

```bash
# 1. Настроить теги для текущей ветки
source scripts/set-docker-tags.sh

# 2. Собрать образ для x86_64 (быстрая разработка)
IMAGE_TAG=local ./scripts/local-build.sh voice-assistant linux/amd64

# 3. Протестировать локально
cd docker/vision
IMAGE_TAG=local docker-compose up voice-assistant

# 4. Запушить изменения - GitHub Actions соберёт для ARM64
git add .
git commit -m "feat: update voice assistant"
git push

# 5. Через ~10 минут образ с правильным тегом будет на ghcr.io
# 6. На Raspberry Pi выполнить:
ssh ros2@10.1.1.21
cd ~/rob_box_project/docker/vision
source ../../scripts/set-docker-tags.sh
docker-compose pull voice-assistant
docker-compose up -d voice-assistant
```

**Сценарий 2: Разработка непосредственно на Raspberry Pi**

```bash
# 1. SSH на Raspberry Pi
ssh ros2@10.1.1.21

# 2. Перейти в проект
cd ~/rob_box_project

# 3. Настроить теги
source scripts/set-docker-tags.sh

# 4. Собрать образ локально (нативный ARM64)
IMAGE_TAG=local ./scripts/local-build.sh voice-assistant

# 5. Запустить
cd docker/vision
IMAGE_TAG=local docker-compose up -d voice-assistant

# 6. Проверить логи
docker logs -f voice-assistant
```

### Переключение между окружениями

```bash
# Production (main branch images)
source scripts/set-docker-tags.sh  # автоматически определит main
cd docker/vision && docker-compose pull && docker-compose up -d

# Development (develop branch images)
export IMAGE_TAG=dev
cd docker/vision && docker-compose pull && docker-compose up -d

# Local testing (locally built images)
export IMAGE_TAG=local
cd docker/vision && docker-compose up -d

# Specific version
export IMAGE_TAG=rc-1.0.0
cd docker/vision && docker-compose pull && docker-compose up -d
```

## Best Practices

1. **Feature ветки:**
   - Короткие имена: `feature/voice-assistant`, `feat/led-fix`
   - Одна фича - одна ветка
   - Регулярно rebase на develop

2. **Коммиты:**
   - Conventional commits: `feat:`, `fix:`, `docs:`, `refactor:`
   - Описательные сообщения
   - Один логический change на коммит

3. **Тестирование:**
   - Тестировать локально перед push
   - Использовать `-dev` образы для staging
   - Проверять на реальном железе перед main

4. **Деплой:**
   - Develop → staging/testing
   - Main → production
   - Rollback через release tags

## Дополнительная информация

- **Docker README:** `docker/vision/README.md`
- **Deployment Guide:** `docker/vision/DEPLOYMENT.md`
- **Architecture:** `docs/ARCHITECTURE.md`
