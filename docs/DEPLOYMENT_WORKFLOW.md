# Automated Deployment and Health Check Workflow

## Обзор

Workflow `G: Deploy and Verify` автоматизирует процесс развертывания обновлений на роботе и проверяет работоспособность системы.

**Местоположение:** `.github/workflows/G-Deploy and Verify.yml`

## Возможности

- ✅ **Автоматизированный деплой** на Main Pi и Vision Pi через SSH
- ✅ **Выбор ветки** для деплоя (main, develop, feature/test)
- ✅ **Выбор окружения** (production, staging, test)
- ✅ **Проверка здоровья контейнеров** - статус, логи, ошибки
- ✅ **Проверка ROS2 топиков** - наличие и работоспособность
- ✅ **Автоматическое создание issue** при обнаружении проблем
- ✅ **Умное назначение ответственных**:
  - Критические ошибки → @copilot
  - Предупреждения → @krikz
- ✅ **Dry-run режим** для тестирования без реального деплоя

## Как использовать

### Запуск через GitHub UI

1. Перейти в GitHub → Actions
2. Выбрать workflow **"G: Deploy and Verify"**
3. Нажать **"Run workflow"**
4. Заполнить параметры:
   - **Branch:** Ветка для деплоя (main/develop/feature/test)
   - **Environment:** Окружение (production/staging/test)
   - **Skip pull images:** Пропустить загрузку образов (использовать локальные)
   - **Dry run:** Сухой прогон (не выполнять реальный деплой)
5. Нажать **"Run workflow"**

### Запуск через GitHub CLI

```bash
# Production деплой из main ветки
gh workflow run "G-Deploy and Verify.yml" \
  -f branch=main \
  -f environment=production

# Staging деплой из develop ветки
gh workflow run "G-Deploy and Verify.yml" \
  -f branch=develop \
  -f environment=staging

# Тестовый деплой с dry-run
gh workflow run "G-Deploy and Verify.yml" \
  -f branch=develop \
  -f environment=test \
  -f dry_run=true

# Быстрый деплой без pull образов (использовать локальные)
gh workflow run "G-Deploy and Verify.yml" \
  -f branch=main \
  -f environment=production \
  -f skip_pull_images=true
```

## Параметры workflow

### `branch` (required)

Ветка для деплоя.

**Опции:**
- `main` - Production ветка (стабильная версия)
- `develop` - Development ветка (тестирование)
- `feature/test` - Feature ветка (разработка)

### `environment` (required)

Целевое окружение.

**Опции:**
- `production` - Production окружение (IMAGE_TAG=latest)
- `staging` - Staging окружение (IMAGE_TAG=dev)
- `test` - Test окружение (IMAGE_TAG=test)

### `skip_pull_images` (optional)

Пропустить загрузку Docker образов из registry.

**Использование:**
- `false` (default) - Загрузить свежие образы из registry
- `true` - Использовать локальные образы (быстрее)

**Когда использовать:**
- Если образы уже на Pi и не изменились
- Для быстрого перезапуска без обновления образов

### `dry_run` (optional)

Сухой прогон без реального деплоя.

**Использование:**
- `false` (default) - Выполнить реальный деплой
- `true` - Только показать что будет сделано

**Когда использовать:**
- Тестирование workflow
- Проверка параметров перед реальным деплоем

## Процесс деплоя

### 1. Setup (подготовка)

- Установка SSH и sshpass
- Настройка переменных окружения
- Определение IP адресов Pi

### 2. Vision Pi Deployment

```
🛑 Stop Containers
   └─> docker compose down --remove-orphans

📥 Update Code
   └─> git checkout <branch>
   └─> git pull origin <branch>

📦 Pull Docker Images
   └─> docker compose pull

🚀 Start Containers
   └─> docker compose up -d
```

### 3. Main Pi Deployment

```
🛑 Stop Containers
   └─> docker compose down --remove-orphans

📥 Update Code
   └─> git checkout <branch>
   └─> git pull origin <branch>

📦 Pull Docker Images
   └─> docker compose pull

🚀 Start Containers
   └─> docker compose up -d
```

### 4. Initialization Wait

```
⏳ Wait 30 seconds for container initialization
```

### 5. Health Checks

#### Container Status Check

Проверяет статус всех контейнеров через `docker compose ps`:
- ✅ Running - контейнер работает
- ❌ Exited/Error - контейнер упал

#### Container Logs Check

Анализирует последние 100 строк логов каждого контейнера:

**Критические ошибки:**
- `CRITICAL`
- `FATAL`
- `ERROR.*failed`
- `segmentation fault`
- `core dumped`

**Предупреждения:**
- `WARN`
- `WARNING`

#### ROS2 Topics Check

Проверяет наличие ROS2 топиков в контейнерах:

**Vision Pi:**
- `/oak/rgb/image_raw/compressed` - RGB камера
- Другие топики от OAK-D

**Main Pi:**
- Проверяет общее количество топиков
- Должно быть > 5 топиков для нормальной работы

### 6. Issue Creation (при ошибках)

Если обнаружены проблемы, автоматически создается GitHub Issue:

#### Критические ошибки → @copilot

**Условия:**
- CRITICAL/FATAL ошибки в логах
- Контейнеры не запущены
- Отсутствуют обязательные ROS2 топики

**Labels:** `bug`, `critical`, `deployment`

**Issue содержит:**
- Полную информацию о деплое
- Статус контейнеров на обоих Pi
- Количество ошибок и предупреждений
- Ссылку на workflow run с логами
- Рекомендации по исправлению
- Quick commands для диагностики

#### Предупреждения → @krikz

**Условия:**
- Только WARNING в логах
- Все контейнеры запущены
- Частичные проблемы с топиками

**Labels:** `bug`, `deployment`

**Issue содержит:**
- Информацию о предупреждениях
- Статус системы
- Рекомендации по мониторингу

### 7. Summary

Финальная сводка деплоя:

```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📊 DEPLOYMENT SUMMARY
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Branch: main
Environment: production
Image Tag: latest
Timestamp: 2025-10-28 21:00:00 UTC

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Vision Pi (10.1.1.21)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Containers Healthy: true
Failed Containers: 0
Critical Errors: 0
Warnings: 2
Topics Status: true

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Main Pi (10.1.1.20)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Containers Healthy: true
Failed Containers: 0
Critical Errors: 0
Warnings: 1
Topics Status: true

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✅ DEPLOYMENT SUCCESSFUL
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

## Типичные сценарии использования

### Сценарий 1: Production деплой

После успешного тестирования в develop, разворачиваем в production:

```bash
# 1. Убедиться что main ветка содержит нужные изменения
git checkout main
git pull

# 2. Запустить деплой
gh workflow run "G-Deploy and Verify.yml" \
  -f branch=main \
  -f environment=production

# 3. Следить за прогрессом
gh run watch

# 4. Проверить результаты
# - Если успешно - всё готово
# - Если ошибки - автоматически создан issue
```

### Сценарий 2: Staging тестирование

Тестируем develop ветку перед мерджем в main:

```bash
# 1. Запустить деплой в staging
gh workflow run "G-Deploy and Verify.yml" \
  -f branch=develop \
  -f environment=staging

# 2. Проверить работоспособность
# - Логи в workflow
# - SSH на Pi для ручной проверки

# 3. Если всё ОК - мерджить в main
git checkout main
git merge develop
git push
```

### Сценарий 3: Быстрый рестарт

Перезапустить контейнеры без обновления образов:

```bash
gh workflow run "G-Deploy and Verify.yml" \
  -f branch=main \
  -f environment=production \
  -f skip_pull_images=true
```

### Сценарий 4: Тестирование workflow

Проверить работу workflow без реального деплоя:

```bash
gh workflow run "G-Deploy and Verify.yml" \
  -f branch=develop \
  -f environment=test \
  -f dry_run=true
```

### Сценарий 5: Feature тестирование

Развернуть feature ветку для тестирования:

```bash
# 1. Создать feature ветку и запушить
git checkout -b feature/my-feature
# ... внести изменения ...
git push

# 2. Собрать образы через CI/CD
# (автоматически при push в feature/* ветку)

# 3. Развернуть на test окружение
gh workflow run "G-Deploy and Verify.yml" \
  -f branch=feature/my-feature \
  -f environment=test

# 4. Проверить работу
# 5. Если OK - создать PR в develop
```

## Troubleshooting

### Проблема: SSH connection failed

**Симптомы:**
```
Permission denied (publickey,password)
```

**Решение:**
- Проверить что Pi доступен в сети: `ping 10.1.1.21`
- Проверить что sshpass работает: `sshpass -p 'open' ssh ros2@10.1.1.21 'echo OK'`
- Убедиться что пароль 'open' правильный

### Проблема: Containers failed to start

**Симптомы:**
```
❌ 2 container(s) not running on Vision Pi
```

**Решение:**
1. Проверить логи workflow для деталей
2. SSH на Pi и проверить:
   ```bash
   ssh ros2@10.1.1.21
   cd ~/rob_box_project/docker/vision
   docker compose ps
   docker logs <failed-container>
   ```
3. Проверить автоматически созданный issue для рекомендаций

### Проблема: Images not found

**Симптомы:**
```
⚠️  Warning: Some images may not be available
```

**Решение:**
1. Проверить что образы собраны в GitHub Actions
2. Проверить что правильный IMAGE_TAG:
   - main → latest
   - develop → dev
   - feature/* → test
3. Проверить registry: `docker images | grep rob_box`

### Проблема: No ROS2 topics

**Симптомы:**
```
❌ No topics found (ROS2 may not be initialized yet)
```

**Решение:**
1. Подождать дольше (30 секунд может быть недостаточно)
2. Проверить Zenoh router: `docker logs zenoh-router`
3. Проверить что контейнеры запустились: `docker ps`
4. Вручную проверить топики:
   ```bash
   docker exec oak-d bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list'
   ```

### Проблема: Workflow создает false positive issues

**Симптомы:**
- Issue создается но система работает
- Много WARNING но нет реальных проблем

**Решение:**
1. Проверить логи в workflow для деталей
2. Уточнить фильтры для критических ошибок
3. Добавить исключения для известных warning
4. Закрыть false positive issue с пометкой "wontfix"

## Best Practices

### 1. Всегда тестировать в staging

```bash
# ❌ ПЛОХО - сразу в production
gh workflow run ... -f branch=develop -f environment=production

# ✅ ХОРОШО - сначала staging
gh workflow run ... -f branch=develop -f environment=staging
# Проверить работу
gh workflow run ... -f branch=main -f environment=production
```

### 2. Использовать dry-run для новых веток

```bash
# ✅ ХОРОШО - сначала dry-run
gh workflow run ... -f branch=feature/new -f dry_run=true
# Проверить параметры
gh workflow run ... -f branch=feature/new
```

### 3. Мониторить workflow runs

```bash
# Следить за текущим run
gh run watch

# Проверить последние 5 runs
gh run list --workflow="G-Deploy and Verify.yml" --limit 5

# Посмотреть логи конкретного run
gh run view <run-id> --log
```

### 4. Не игнорировать warnings

- Даже если система работает, warnings могут привести к проблемам
- Проверять issues от @krikz и исправлять предупреждения
- Следить за трендами (увеличение warnings со временем)

### 5. Документировать изменения

После деплоя добавлять комментарий:

```bash
# В PR или issue
Deployed to production via workflow run #123
- All containers healthy
- ROS2 topics active
- No critical errors
- 2 warnings (known issue #456)
```

## Интеграция с CI/CD

Workflow `G: Deploy and Verify` дополняет существующий CI/CD:

```
Feature Branch → G: Auto-merge Feature to Develop
                       ↓ (build images)
                       ↓
                    Develop → G: Auto-merge to Main
                       ↓ (build images)
                       ↓
                     Main → G: Deploy and Verify ← Ручной запуск
                       ↓
                  Production Robot
```

**Преимущества:**
- CI/CD собирает образы автоматически
- Deploy workflow разворачивает вручную
- Полный контроль над production деплоем
- Автоматическая диагностика и issue creation

## Безопасность

### SSH Credentials

Workflow использует пароль `'open'` через sshpass.

**⚠️  ВАЖНО:**
- Пароль хардкоднут в workflow (не секрет)
- Pi находятся в локальной сети (10.1.1.x)
- Нет доступа из интернета

**Для production окружения рекомендуется:**
1. Использовать SSH ключи вместо паролей
2. Хранить ключи в GitHub Secrets
3. Настроить firewall на Pi

### Docker Registry Access

Workflow подтягивает образы из `ghcr.io/krikz/rob_box`.

**Требования:**
- Публичный registry (или настроенный доступ на Pi)
- Docker login на Pi (если private registry)

## Дополнительная информация

- **CI/CD Pipeline:** `docs/CI_CD_PIPELINE.md`
- **Docker Standards:** `docs/development/DOCKER_STANDARDS.md`
- **Deployment Guide:** `docker/vision/DEPLOYMENT.md`
- **Troubleshooting:** `docs/TROUBLESHOOTING.md`

---

**Документ создан:** 2025-10-28
**Версия workflow:** 1.0.0
**Автор:** GitHub Copilot
