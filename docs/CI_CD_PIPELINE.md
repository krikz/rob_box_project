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
- `ghcr.io/krikz/rob_box:voice-assistant-humble-abc1234` (SHA)

### Tags по веткам

| Ветка | Tag | Описание |
|-------|-----|----------|
| `main` | `humble-latest` | Продакшн, стабильная версия |
| `develop` | `humble-dev` | Development, тестирование |
| `feature/*` | `humble-dev` | После мерджа в develop |
| `release/X.X.X` | `humble-rc-X.X.X` | Release candidate |
| SHA commit | `humble-<sha>` | Специфичная версия |

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
