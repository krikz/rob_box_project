# Build Machine Quick Start Guide

**Быстрое руководство по развертыванию и использованию build machine**

## 🚀 Установка за 5 минут

### На Build Machine

```bash
# 1. Клонируйте репозиторий
git clone https://github.com/krikz/rob_box_project.git
cd rob_box_project/docker/build

# 2. Настройте GitHub токен
cp .env.secrets.example .env.secrets
nano .env.secrets
# Добавьте ваш GitHub Personal Access Token

# 3. Запустите инфраструктуру
./scripts/setup.sh

# 4. Проверьте статус
./scripts/check_status.sh
```

### На Raspberry Pi (Main и Vision)

```bash
# Скопируйте и запустите скрипт конфигурации
BUILD_MACHINE_IP=10.1.1.5 bash <(curl -s http://your-git-server/configure_raspberry_pi.sh)
```

---

## 📋 Основные команды

### Управление Build Machine

```bash
# Проверить статус всех сервисов
cd ~/rob_box_project/docker/build
./scripts/check_status.sh

# Перезапустить сервисы
./scripts/restart.sh

# Обновить и перезапустить
./scripts/update_and_restart.sh

# Посмотреть логи
docker compose logs -f

# Посмотреть логи конкретного сервиса
docker logs -f build-github-runner
docker logs -f build-registry
docker logs -f build-apt-cache
```

### Работа с Registry

```bash
# Просмотр образов в registry
curl http://localhost:5000/v2/_catalog | jq

# Просмотр тегов образа
curl http://localhost:5000/v2/krikz/rob_box/tags/list | jq

# Загрузка образа в registry
./scripts/push_to_local_registry.sh ghcr.io/krikz/rob_box:oak-d-humble-latest

# Очистка registry
./scripts/cleanup_registry.sh --dry-run  # Просмотр
./scripts/cleanup_registry.sh --all      # Удаление всех образов
```

### Работа с APT Cache

```bash
# Просмотр статистики кэша
curl http://localhost:3142/acng-report.html

# Просмотр кэшированных пакетов
docker exec build-apt-cache du -sh /var/cache/apt-cacher-ng/*

# Очистка кэша (освобождение места)
docker exec -it build-apt-cache bash
# Внутри контейнера:
rm -rf /var/cache/apt-cacher-ng/uburep/*
```

---

## 🔍 Проверка работоспособности

### Тест 1: Registry доступен

```bash
# На build machine
curl http://localhost:5000/v2/

# Ожидаемый результат: {}
```

```bash
# На Raspberry Pi
curl http://10.1.1.5:5000/v2/

# Ожидаемый результат: {}
```

### Тест 2: APT Cache работает

```bash
# На build machine
curl http://localhost:3142/acng-report.html | grep "APT-Cacher"

# Ожидаемый результат: содержит "APT-Cacher NG"
```

```bash
# На Raspberry Pi
cat /etc/apt/apt.conf.d/02proxy

# Ожидаемый результат: Acquire::http::Proxy "http://10.1.1.5:3142";

# Тест загрузки пакета через кэш
sudo apt-get update
# В выводе должно быть: "Get:1 http://10.1.1.5:3142/..."
```

### Тест 3: GitHub Runner активен

```bash
# На build machine
docker logs build-github-runner | grep "Listening for Jobs"

# Ожидаемый результат: "Listening for Jobs"
```

```bash
# На GitHub
# Settings → Actions → Runners
# Должен быть виден: rob-box-build-machine [Idle]
```

### Тест 4: Полный цикл сборки и развертывания

```bash
# 1. На GitHub: запустите workflow с self-hosted runner
# Actions → выберите workflow → Run workflow

# 2. Проверьте что сборка выполняется локально
docker logs -f build-github-runner

# 3. После сборки проверьте что образ в registry
curl http://localhost:5000/v2/_catalog | jq

# 4. На Raspberry Pi: обновите образ
docker pull 10.1.1.5:5000/krikz/rob_box:oak-d-humble-latest

# Должно быть быстро (30-60 секунд вместо 10-15 минут)
```

---

## 📊 Мониторинг

### Веб-интерфейсы

| Сервис | URL | Описание |
|--------|-----|----------|
| Registry UI | http://10.1.1.5:8080 | Просмотр и удаление образов |
| APT Cache Report | http://10.1.1.5:3142/acng-report.html | Статистика кэша |

### Метрики производительности

```bash
# Использование CPU и памяти
docker stats

# Размер данных
du -sh ~/rob_box_project/docker/build/data/*

# Количество образов в registry
curl -s http://localhost:5000/v2/_catalog | jq '.repositories | length'

# Размер APT cache
docker exec build-apt-cache du -sh /var/cache/apt-cacher-ng/
```

---

## 🔧 Типичные сценарии

### Сценарий 1: Первая сборка проекта

```bash
# 1. На build machine: запустить инфраструктуру
cd ~/rob_box_project/docker/build
./scripts/setup.sh

# 2. На GitHub: запустить build-all workflow
# Actions → Build All → Run workflow
# Убедитесь что в workflow.yml указано: runs-on: self-hosted

# 3. Дождаться завершения сборки (~20-30 минут первый раз)
# Последующие сборки будут быстрее благодаря кэшу

# 4. На Raspberry Pi: настроить использование локального registry
BUILD_MACHINE_IP=10.1.1.5 ./configure_raspberry_pi.sh

# 5. На Raspberry Pi: обновить сервисы
cd ~/rob_box_project/docker/main && ./update_and_restart.sh
cd ~/rob_box_project/docker/vision && ./update_and_restart.sh
```

### Сценарий 2: Разработка нового сервиса

```bash
# 1. Локальная разработка
git checkout -b feature/my-service
# ... сделать изменения ...

# 2. Коммит и пуш
git add .
git commit -m "feat: add my-service"
git push origin feature/my-service

# 3. GitHub Actions автоматически:
#    - Запустит сборку на self-hosted runner
#    - Соберет образ с использованием APT cache
#    - Загрузит в локальный registry
#    - Загрузит в ghcr.io

# 4. На Raspberry Pi: тестирование
docker pull 10.1.1.5:5000/krikz/rob_box:my-service-humble-test
# Быстрая загрузка из локального registry!
```

### Сценарий 3: Обновление production

```bash
# 1. Merge в main (автоматически запустится сборка)
git checkout main
git merge develop
git push origin main

# 2. Дождаться завершения сборки (проверить в Actions)

# 3. На Raspberry Pi: автоматическое обновление
# (если настроен auto-update в workflow)
# Или вручную:
ssh ros2@10.1.1.20 'cd ~/rob_box_project/docker/main && ./update_and_restart.sh'
ssh ros2@10.1.1.21 'cd ~/rob_box_project/docker/vision && ./update_and_restart.sh'

# 4. Проверить что всё работает
ssh ros2@10.1.1.20 'docker ps'
ssh ros2@10.1.1.21 'docker ps'
```

### Сценарий 4: Очистка места

```bash
# На build machine регулярно выполняйте очистку

# Очистка Docker
docker system prune -a --volumes
# Освободится: образы, контейнеры, volumes

# Очистка registry (старые образы)
./scripts/cleanup_registry.sh --all

# Очистка APT cache (если нужно много места)
docker exec build-apt-cache rm -rf /var/cache/apt-cacher-ng/*
# Пакеты будут закэшированы заново при следующей сборке
```

---

## ⚡ Советы по оптимизации

### 1. Используйте базовые образы

Базовые образы (`rob_box_base:ros2-zenoh`, `rob_box_base:rtabmap`, etc.) сильно ускоряют сборку:

```dockerfile
# ✅ ХОРОШО
FROM rob_box_base:ros2-zenoh
RUN apt-get update && apt-get install -y my-package

# ❌ ПЛОХО (долго)
FROM ubuntu:22.04
RUN apt-get update && apt-get install -y ros-humble-desktop
```

### 2. Используйте BuildKit cache

В GitHub Actions workflow:

```yaml
- name: Build Docker image
  uses: docker/build-push-action@v5
  with:
    cache-from: type=gha
    cache-to: type=gha,mode=max
```

### 3. Группируйте RUN команды

```dockerfile
# ✅ ХОРОШО (один слой, один запрос к APT cache)
RUN apt-get update && apt-get install -y \
    package1 \
    package2 \
    package3 \
    && rm -rf /var/lib/apt/lists/*

# ❌ ПЛОХО (три слоя, три запроса)
RUN apt-get install -y package1
RUN apt-get install -y package2
RUN apt-get install -y package3
```

### 4. Регулярно обновляйте base images

```bash
# Раз в неделю пересобирайте базовые образы
# Actions → Build Base Images → Run workflow

# Это подтянет обновления безопасности
```

---

## 🆘 Что делать если...

### Проблема: Build machine не запускается

```bash
# Проверьте логи
docker compose logs

# Проверьте свободное место
df -h

# Перезапустите
./scripts/restart.sh
```

### Проблема: Сборка всё равно долгая

```bash
# Проверьте что используется APT cache
docker logs build-github-runner | grep 3142

# Проверьте статистику кэша
curl http://localhost:3142/acng-report.html

# Проверьте что используется BuildKit cache
# В логах GitHub Actions должно быть: "importing cache manifest from gha"
```

### Проблема: Raspberry Pi не может pull образ

```bash
# На Raspberry Pi проверьте конфигурацию
cat /etc/docker/daemon.json

# Должно быть:
# {
#   "insecure-registries": ["10.1.1.5:5000"],
#   "registry-mirrors": ["http://10.1.1.5:5000"]
# }

# Перезапустите Docker
sudo systemctl restart docker
```

---

## 📚 Дополнительно

- 📖 Полная документация: [README.md](README.md)
- 🔧 Troubleshooting: [README.md#troubleshooting](README.md#troubleshooting)
- 🏗️ Архитектура: [README.md#архитектура](README.md#архитектура)
- 🤖 AI агенты: [../docs/development/AGENT_GUIDE.md](../docs/development/AGENT_GUIDE.md)

---

**Last Updated:** 2025-10-26
