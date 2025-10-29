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
   - **Registry source:** Источник Docker образов (skip/github/local)
   - **Dry run:** Сухой прогон (не выполнять реальный деплой)
5. Нажать **"Run workflow"**

### Запуск через GitHub CLI

```bash
# Production деплой из main ветки (GitHub registry)
gh workflow run "G-Deploy and Verify.yml" \
  -f branch=main \
  -f environment=production \
  -f registry_source=github

# Staging деплой из develop ветки (Local build machine registry)
gh workflow run "G-Deploy and Verify.yml" \
  -f branch=develop \
  -f environment=staging \
  -f registry_source=local

# Тестовый деплой с dry-run
gh workflow run "G-Deploy and Verify.yml" \
  -f branch=develop \
  -f environment=test \
  -f registry_source=github \
  -f dry_run=true

# Быстрый деплой без pull образов (использовать существующие локальные)
gh workflow run "G-Deploy and Verify.yml" \
  -f branch=main \
  -f environment=production \
  -f registry_source=skip
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

### `registry_source` (required)

Источник Docker образов.

**Опции:**
- `github` (default) - GitHub Container Registry (ghcr.io/krikz/rob_box)
  - Используется для production деплоя
  - Образы собраны через GitHub Actions (G-Build workflows)
- `local` - Local Build Machine Registry (localhost:5000/krikz/rob_box)
  - Используется для быстрого локального тестирования
  - Образы собраны на локальной build machine (L-Build workflows)
  - В 10-20x быстрее pull на Raspberry Pi (локальная сеть)
- `skip` - Не загружать образы, использовать существующие
  - Используется для быстрого перезапуска контейнеров
  - Не обновляет образы, только перезапускает

**Когда использовать:**
- `github` - Для production и staging окружений
- `local` - Для разработки и тестирования с локальной build machine
- `skip` - Для быстрого рестарта без обновления образов

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
- Настройка источника Docker образов (registry prefix)

### 2. Verification (проверка готовности) 🆕

**Добавлено в версии от 2025-10-29**

Проверяет наличие необходимых инструментов и доступность целевых хостов:

#### SSH Tools Check
```
✅ ssh: /usr/bin/ssh
   OpenSSH_9.x, OpenSSL x.x.x
✅ sshpass: /usr/bin/sshpass
```

Если инструменты отсутствуют:
```
❌ ERROR: 'sshpass' command not found in runner
❌ DEPLOYMENT FAILED: Missing required SSH tools
```

#### Network Connectivity Check

Проверяет доступность Raspberry Pi:

**Vision Pi (10.1.1.21):**
- ICMP ping test (1 пакет, таймаут 2 сек)
- SSH port (22) TCP connection test (таймаут 3 сек)

**Main Pi (10.1.1.20):**
- ICMP ping test (1 пакет, таймаут 2 сек)
- SSH port (22) TCP connection test (таймаут 3 сек)

Если хосты недоступны:
```
❌ Vision Pi SSH port (22) is NOT accessible
❌ DEPLOYMENT FAILED: Target hosts unreachable

Possible reasons:
  • GitHub Actions runners cannot reach private network IPs (10.1.1.x)
  • This workflow requires a self-hosted runner on the same network
  • Devices may be powered off or network is down
  • Firewall blocking connections

Solutions:
  1. Use a self-hosted GitHub Actions runner on the local network
  2. Set up a VPN connection from GitHub runners to your network
  3. Use SSH tunneling or a bastion host with public IP
```

**Важно:** Эта проверка пропускается в dry-run режиме.

### 3. Vision Pi Deployment

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

# 2. Запустить деплой из GitHub registry
gh workflow run "G-Deploy and Verify.yml" \
  -f branch=main \
  -f environment=production \
  -f registry_source=github

# 3. Следить за прогрессом
gh run watch

# 4. Проверить результаты
# - Если успешно - всё готово
# - Если ошибки - автоматически создан issue
```

### Сценарий 2: Staging тестирование

Тестируем develop ветку перед мерджем в main:

```bash
# 1. Запустить деплой в staging из GitHub registry
gh workflow run "G-Deploy and Verify.yml" \
  -f branch=develop \
  -f environment=staging \
  -f registry_source=github

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
  -f registry_source=skip
```

### Сценарий 4: Тестирование workflow

Проверить работу workflow без реального деплоя:

```bash
gh workflow run "G-Deploy and Verify.yml" \
  -f branch=develop \
  -f environment=test \
  -f registry_source=github \
  -f dry_run=true
```

### Сценарий 5: Feature тестирование (Local build machine)

Развернуть feature ветку для быстрого тестирования с локальной build machine:

```bash
# 1. Создать feature ветку и запушить
git checkout -b feature/my-feature
# ... внести изменения ...
git push

# 2. Собрать образы на локальной build machine через L-Build workflow
gh workflow run "L-Build All Services.yml"

# 3. Развернуть на test окружение из локального registry
gh workflow run "G-Deploy and Verify.yml" \
  -f branch=feature/my-feature \
  -f environment=test \
  -f registry_source=local

# 4. Проверить работу (pull будет в 10-20x быстрее)
# 5. Если OK - собрать через GitHub и создать PR в develop
```

## Troubleshooting

### Проблема: Missing SSH tools in runner

**Симптомы:**
```
❌ ERROR: 'ssh' command not found in runner
❌ ERROR: 'sshpass' command not found in runner
❌ DEPLOYMENT FAILED: Missing required SSH tools
```

**Решение:**
1. Проверить что установка sshpass в шаге "Setup SSH and Environment" прошла успешно
2. Проверить логи установки: `sudo apt-get install -y sshpass`
3. Убедиться что используется правильный runner (ubuntu-latest)
4. Если проблема повторяется, использовать self-hosted runner

**Примечание:** С версии от 2025-10-29 workflow автоматически проверяет наличие SSH инструментов перед началом деплоя.

### Проблема: Target hosts unreachable

**Симптомы:**
```
⚠️  Vision Pi is NOT reachable via ICMP (ping)
❌ Vision Pi SSH port (22) is NOT accessible
❌ DEPLOYMENT FAILED: Target hosts unreachable
```

**Причины:**
- **GitHub Actions hosted runners** не могут достучаться до частных IP адресов (10.1.1.x)
- Raspberry Pi устройства находятся в локальной сети
- Устройства выключены или сеть недоступна
- Firewall блокирует соединения

**Решение:**

**Вариант 1: Self-hosted runner (рекомендуется)**
```bash
# На машине в той же сети что и Raspberry Pi
# 1. Скачать GitHub Actions runner
curl -o actions-runner-linux-x64-2.319.1.tar.gz -L \
  https://github.com/actions/runner/releases/download/v2.319.1/actions-runner-linux-x64-2.319.1.tar.gz
tar xzf ./actions-runner-linux-x64-2.319.1.tar.gz

# 2. Настроить runner
./config.sh --url https://github.com/krikz/rob_box_project --token YOUR_TOKEN

# 3. Запустить как сервис
sudo ./svc.sh install
sudo ./svc.sh start

# 4. Обновить workflow для использования self-hosted runner
# В .github/workflows/G-Deploy and Verify.yml:
# runs-on: [self-hosted, linux]
```

**Вариант 2: VPN туннель**
- Настроить WireGuard/OpenVPN на GitHub runner
- Подключиться к локальной сети через VPN
- Требует дополнительной настройки безопасности

**Вариант 3: SSH bastion host**
- Настроить публично доступный bastion host
- Проброс SSH соединений к Raspberry Pi через bastion
- Требует дополнительной инфраструктуры

**Примечание:** С версии от 2025-10-29 workflow автоматически различает проблемы с SSH инструментами и проблемы с сетевой доступностью.

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
