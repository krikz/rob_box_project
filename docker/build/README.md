# Build Machine Infrastructure

**Локальная инфраструктура для сборки Docker образов Rob Box Project**

## 📋 Оглавление

- [Обзор](#обзор)
- [Проблема и решение](#проблема-и-решение)
- [Архитектура](#архитектура)
- [Установка](#установка)
- [Использование](#использование)
- [Интеграция с CI/CD](#интеграция-с-cicd)
- [Обслуживание](#обслуживание)
- [Troubleshooting](#troubleshooting)

---

## 🎯 Обзор

Build machine - это выделенная машина в локальной сети (10.1.1.x), которая:

1. **Запускает GitHub Actions workflows локально** - используя self-hosted runner
2. **Хранит Docker образы локально** - через Docker Registry
3. **Кэширует APT пакеты** - через apt-cacher-ng для ускорения сборки

**Результат:** Обновление Raspberry Pi происходит в **10-20 раз быстрее**, так как образы не скачиваются из интернета.

---

## 🔍 Проблема и решение

### Проблема

Текущая архитектура:
```
GitHub Actions (cloud) → builds images → pushes to ghcr.io
                                              ↓
                                    Raspberry Pi pulls from ghcr.io (slow!)
```

**Недостатки:**
- ❌ Долгая загрузка образов (5-15 минут на каждый Pi)
- ❌ Зависимость от скорости интернета
- ❌ Повторная загрузка одних и тех же пакетов при каждой сборке
- ❌ Трафик интернет-канала

### Решение

Новая архитектура:
```
GitHub Actions (self-hosted) → builds locally → local registry
                                                      ↓
                                            Raspberry Pi pulls locally (fast!)
```

**Преимущества:**
- ✅ Быстрая загрузка образов (30-60 секунд)
- ✅ Работает без интернета (после первоначальной настройки)
- ✅ APT пакеты кэшируются - ускорение сборки в 3-5 раз
- ✅ Экономия трафика

---

## 🏗️ Архитектура

### Компоненты

```
docker/build/
├── docker-compose.yaml          # Оркестрация сервисов
├── .env                         # Переменные окружения
├── .env.secrets                 # GitHub токены (не в git!)
├── config/                      # Конфигурации сервисов
│   ├── registry-config.yml      # Docker Registry настройки
│   └── acng.conf                # APT Cache настройки
├── scripts/                     # Утилиты
│   ├── setup.sh                 # Первоначальная настройка
│   ├── check_status.sh          # Проверка статуса
│   ├── restart.sh               # Перезапуск сервисов
│   ├── update_and_restart.sh    # Обновление и перезапуск
│   └── configure_raspberry_pi.sh # Настройка Raspberry Pi
└── data/                        # Persistent данные (не в git!)
    ├── registry/                # Docker образы
    ├── apt-cache/               # APT пакеты
    └── runner/                  # GitHub runner workspace
```

### Сервисы

#### 1. Docker Registry (порт 5000)
- Локальное хранилище Docker образов
- Автоматически используется Raspberry Pi для pull
- Веб-интерфейс доступен на порту 8080

#### 2. APT Cacher NG (порт 3142)
- Кэширует APT/dpkg пакеты
- Используется во время `apt-get install` в Dockerfile
- Отчеты доступны: http://build-machine:3142/acng-report.html

#### 3. GitHub Actions Runner
- Self-hosted runner для выполнения workflows
- Использует Docker-in-Docker для сборки образов
- Автоматически регистрируется в GitHub репозитории

#### 4. Registry UI (порт 8080)
- Веб-интерфейс для просмотра образов в registry
- Удаление образов через UI
- Просмотр тегов и размеров

---

## 🚀 Установка

### Требования

- **ОС:** Linux (x86_64 или ARM64)
- **RAM:** Минимум 4GB (рекомендуется 8GB)
- **Диск:** Минимум 50GB свободного места
- **Сеть:** Доступ к локальной сети 10.1.1.x
- **Docker:** Docker Engine 20.10+ и docker-compose v2
- **Для x86_64:** QEMU и buildx для кросс-компиляции ARM64 образов

### Шаг 1: Клонирование репозитория

```bash
git clone https://github.com/krikz/rob_box_project.git
cd rob_box_project/docker/build
```

### Шаг 2: Создание .env файлов

```bash
# Копируем примеры
cp .env.example .env
cp .env.secrets.example .env.secrets

# Редактируем .env
nano .env
# Обновите BUILD_MACHINE_IP на IP вашей build машины
```

### Шаг 3: Настройка GitHub токена

1. Перейдите на https://github.com/settings/tokens/new
2. Создайте Personal Access Token (classic) с правами:
   - ✅ `repo` (full control of private repositories)
   - ✅ `workflow` (update GitHub Action workflows)
3. Скопируйте токен и вставьте в `.env.secrets`:

```bash
nano .env.secrets
# GITHUB_TOKEN=ghp_your_token_here
# GITHUB_OWNER=krikz
# GITHUB_REPO=rob_box_project
```

### Шаг 3.1: Настройка QEMU (только для x86_64)

Если build machine на x86_64, необходимо настроить QEMU для кросс-компиляции ARM64 образов:

```bash
# Установка QEMU
sudo apt-get update
sudo apt-get install -y qemu-user-static binfmt-support

# Регистрация QEMU для multi-arch builds
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

# Проверка
docker buildx ls
# Должен быть builder с поддержкой linux/arm64

# Если нужно создать builder
docker buildx create --name multiarch --driver docker-container --use
docker buildx inspect --bootstrap
```

**Примечание:** Сборка ARM64 образов на x86_64 будет медленнее (эмуляция через QEMU), но всё равно быстрее чем использование GitHub Actions из-за локального APT cache и registry.

### Шаг 4: Запуск инфраструктуры

```bash
./scripts/setup.sh
```

Скрипт автоматически:
- Создаст необходимые директории
- Скачает Docker образы
- Запустит все сервисы
- Проверит статус

### Шаг 5: Проверка статуса

```bash
./scripts/check_status.sh
```

Вы должны увидеть:
```
✅ Registry is accessible
✅ APT Cache is accessible
✅ Runner container is running
```

---

## 📖 Использование

### Настройка Raspberry Pi

На **каждом Raspberry Pi** (Main и Vision) выполните:

```bash
# Скопируйте скрипт на Raspberry Pi
scp docker/build/scripts/configure_raspberry_pi.sh ros2@10.1.1.20:~
scp docker/build/scripts/configure_raspberry_pi.sh ros2@10.1.1.21:~

# На Main Pi (10.1.1.20)
ssh ros2@10.1.1.20
chmod +x configure_raspberry_pi.sh
BUILD_MACHINE_IP=10.1.1.5 ./configure_raspberry_pi.sh

# На Vision Pi (10.1.1.21)
ssh ros2@10.1.1.21
chmod +x configure_raspberry_pi.sh
BUILD_MACHINE_IP=10.1.1.5 ./configure_raspberry_pi.sh
```

Скрипт настроит:
- Docker для использования локального registry
- APT для использования локального кэша

### Использование локального registry в docker-compose

В `docker-compose.yaml` на Raspberry Pi можно указывать локальный registry:

```yaml
services:
  oak-d:
    # Вместо ghcr.io используем локальный registry
    image: 10.1.1.5:5000/krikz/rob_box:oak-d-humble-latest
    # Или автоматически пробовать оба варианта (см. ниже)
```

Docker автоматически попробует сначала локальный registry (если настроен как mirror).

### Использование APT cache в Dockerfile

APT автоматически использует кэш после настройки на Raspberry Pi.

Для использования в Dockerfile при сборке:

```dockerfile
# В начале Dockerfile добавьте (для сборки на build machine)
ARG APT_PROXY
RUN if [ -n "$APT_PROXY" ]; then \
        echo "Acquire::http::Proxy \"$APT_PROXY\";" > /etc/apt/apt.conf.d/02proxy; \
    fi

RUN apt-get update && apt-get install -y \
    ros-humble-nav2-msgs \
    && rm -rf /var/lib/apt/lists/*
```

---

## 🔄 Интеграция с CI/CD

### GitHub Actions workflows

Workflows автоматически запустятся на self-hosted runner, если добавить:

```yaml
jobs:
  build:
    runs-on: self-hosted  # Вместо ubuntu-latest
    # или более специфично:
    # runs-on: [self-hosted, Linux, X64, rob-box]
```

### Обновление workflows для локальной сборки

Пример изменения в `.github/workflows/build-main-services.yml`:

```yaml
jobs:
  build-rtabmap:
    runs-on: self-hosted  # ← Изменено с ubuntu-latest
    
    steps:
      # ... остальные шаги без изменений
      
      - name: Push to local registry
        run: |
          # Пушим в локальный registry
          docker tag ghcr.io/krikz/rob_box:rtabmap-humble-latest \
                     ${{ secrets.BUILD_MACHINE_IP }}:5000/krikz/rob_box:rtabmap-humble-latest
          docker push ${{ secrets.BUILD_MACHINE_IP }}:5000/krikz/rob_box:rtabmap-humble-latest
```

### Проверка работы runner

```bash
# На build machine
docker logs -f build-github-runner

# В GitHub
# Settings → Actions → Runners
# Вы должны увидеть "rob-box-build-machine" со статусом "Idle"
```

---

## 🔧 Обслуживание

### Проверка статуса

```bash
./scripts/check_status.sh
```

### Перезапуск сервисов

```bash
./scripts/restart.sh
```

### Обновление инфраструктуры

```bash
./scripts/update_and_restart.sh
```

### Очистка старых образов

```bash
# Просмотр образов в registry
curl http://localhost:5000/v2/_catalog | jq

# Удаление образа (через Registry UI)
# Откройте http://build-machine-ip:8080
# Найдите образ → Delete

# Или через API
curl -X DELETE http://localhost:5000/v2/<image-name>/manifests/<digest>
```

### Очистка APT cache

```bash
# Статистика кэша
curl http://localhost:3142/acng-report.html

# Очистка через веб-интерфейс
# Откройте http://build-machine-ip:3142/acng-report.html
# Start/Stop scanning → Delete marked packages

# Или вручную
docker exec -it build-apt-cache bash
# Внутри контейнера:
# rm -rf /var/cache/apt-cacher-ng/*
```

### Мониторинг использования диска

```bash
# Размер данных
du -sh data/*

# Результат:
# 2.5G    data/registry     ← Docker образы
# 1.8G    data/apt-cache    ← APT пакеты
# 500M    data/runner       ← Runner workspace
```

### Backup критичных данных

```bash
# Backup registry (образы)
tar -czf registry-backup-$(date +%Y%m%d).tar.gz data/registry/

# Backup не требуется для apt-cache и runner (воссоздаются автоматически)
```

---

## 🐛 Troubleshooting

### Runner не запускается

**Проблема:** `build-github-runner` контейнер постоянно перезапускается

**Решение:**
```bash
# Проверьте логи
docker logs build-github-runner

# Частые причины:
# 1. Неверный GITHUB_TOKEN
#    → Проверьте .env.secrets, убедитесь что токен валиден
# 2. Неверные GITHUB_OWNER/GITHUB_REPO
#    → Проверьте названия в .env.secrets
# 3. Runner уже зарегистрирован
#    → GitHub Settings → Actions → Runners → удалите старый runner
```

### Registry недоступен с Raspberry Pi

**Проблема:** `Error response from daemon: Get "http://10.1.1.5:5000/v2/": dial tcp 10.1.1.5:5000: connect: connection refused`

**Решение:**
```bash
# На build machine проверьте что registry запущен
docker ps | grep registry

# Проверьте firewall
sudo ufw status
sudo ufw allow 5000/tcp

# Проверьте что registry слушает на всех интерфейсах
netstat -tlnp | grep 5000

# На Raspberry Pi проверьте connectivity
ping 10.1.1.5
curl http://10.1.1.5:5000/v2/
```

### APT cache не работает

**Проблема:** Пакеты всё равно скачиваются напрямую, не через кэш

**Решение:**
```bash
# На Raspberry Pi проверьте конфигурацию
cat /etc/apt/apt.conf.d/02proxy
# Должно быть: Acquire::http::Proxy "http://10.1.1.5:3142";

# Проверьте доступность
curl http://10.1.1.5:3142/acng-report.html

# Проверьте что proxy используется
apt-get update
# В выводе должно быть: "Get:1 http://10.1.1.5:3142/..."
```

### Медленная сборка образов

**Проблема:** Сборка всё равно занимает много времени

**Решение:**
```bash
# 1. Проверьте что используется APT cache
docker exec build-github-runner env | grep APT

# 2. Проверьте статистику кэша
curl http://10.1.1.5:3142/acng-report.html

# 3. Убедитесь что используется GitHub Actions cache
# В workflow должно быть:
#   cache-from: type=gha
#   cache-to: type=gha,mode=max

# 4. Проверьте использование базовых образов
# Dockerfile должен начинаться с FROM rob_box_base:*
```

### Переполнение диска

**Проблема:** `no space left on device` на build machine

**Решение:**
```bash
# Проверьте использование
df -h
du -sh docker/build/data/*

# Очистите старые образы Docker
docker system prune -a

# Очистите registry (удалите неиспользуемые образы через UI)
# http://build-machine-ip:8080

# Очистите APT cache
docker exec -it build-apt-cache rm -rf /var/cache/apt-cacher-ng/uburep
# (пакеты будут закэшированы заново при следующей сборке)
```

### Runner не выполняет workflows

**Проблема:** Workflows запускаются, но runner не подхватывает их

**Решение:**
```bash
# Проверьте что runner онлайн в GitHub
# Settings → Actions → Runners
# rob-box-build-machine должен быть "Idle", не "Offline"

# Проверьте логи runner
docker logs -f build-github-runner

# Убедитесь что workflow указывает правильный runs-on
# .github/workflows/*.yml:
#   runs-on: self-hosted  ← должно быть именно так
#   # или:
#   runs-on: [self-hosted, Linux, X64, rob-box]

# Проверьте labels runner
docker exec build-github-runner cat /runner/.runner
```

---

## 📊 Статистика и метрики

### Сравнение времени развертывания

| Операция | До (GitHub) | После (Local) | Ускорение |
|----------|-------------|---------------|-----------|
| Pull образа (1GB) | 10-15 мин | 30-60 сек | **10-20x** |
| Сборка образа | 15-20 мин | 5-8 мин | **2-3x** |
| Полное обновление Pi | 25-35 мин | 3-5 мин | **7-10x** |

### Использование ресурсов build machine

| Сервис | CPU (idle) | CPU (active) | RAM | Disk |
|--------|-----------|--------------|-----|------|
| Registry | <1% | 5-10% | 50-100MB | 2-5GB |
| APT Cache | <1% | 10-20% | 100-200MB | 1-3GB |
| GitHub Runner | <1% | 80-100% | 500MB-2GB | 1-5GB |
| **Итого** | ~2% | ~100% | ~1-2GB | ~5-15GB |

---

## 📚 Дополнительные ресурсы

### Документация компонентов

- [Docker Registry](https://docs.docker.com/registry/)
- [apt-cacher-ng](https://wiki.debian.org/AptCacherNg)
- [GitHub Self-Hosted Runners](https://docs.github.com/en/actions/hosting-your-own-runners)
- [Docker Registry UI](https://github.com/Joxit/docker-registry-ui)

### Связанные документы

- [AGENT_GUIDE.md](../docs/development/AGENT_GUIDE.md) - Гайд для AI агентов
- [DOCKER_STANDARDS.md](../docs/development/DOCKER_STANDARDS.md) - Стандарты Docker
- [CI_CD_PIPELINE.md](../docs/CI_CD_PIPELINE.md) - Описание CI/CD

---

## 🤝 Contributing

При внесении изменений в build machine инфраструктуру:

1. Обновите этот README.md
2. Обновите скрипты в `scripts/`
3. Протестируйте на реальной build machine
4. Обновите `.env.example` если добавлены новые переменные

---

## 📝 Changelog

### 2025-10-26
- ✅ Первоначальная версия build machine infrastructure
- ✅ Docker Registry для локального хранения образов
- ✅ APT Cacher NG для кэширования пакетов
- ✅ GitHub Actions self-hosted runner
- ✅ Utility scripts для управления
- ✅ Документация

---

**Maintained by:** Rob Box Project Team  
**Last Updated:** 2025-10-26
