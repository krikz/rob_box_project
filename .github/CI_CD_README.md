# GitHub Actions CI/CD для rob_box_project

## Автоматическая сборка базовых образов

Проект использует GitHub Actions для автоматической сборки и публикации базовых Docker образов в GitHub Container Registry (ghcr.io).

### Workflow: Build Base Docker Images

**Файл:** `.github/workflows/build-base-images.yml`

**Триггеры:**
- Push в `main` с изменениями в `docker/base/**`
- Ручной запуск (workflow_dispatch)
- Еженедельная пересборка (воскресенье 2:00 UTC) для обновления upstream образов

**Собираемые образы:**
1. `ghcr.io/krikz/rob_box_base:ros2-zenoh` - базовый ROS 2 + Zenoh
2. `ghcr.io/krikz/rob_box_base:rtabmap` - RTAB-Map SLAM
3. `ghcr.io/krikz/rob_box_base:depthai` - DepthAI для OAK-D камеры
4. `ghcr.io/krikz/rob_box_base:pcl` - Point Cloud Library для лидаров

**Архитектуры:** `linux/amd64`, `linux/arm64` (поддержка Raspberry Pi)

## Использование предсобранных образов

### На Raspberry Pi (автоматически):

Все Dockerfile уже настроены на использование образов из ghcr.io по умолчанию:

```bash
# Main Pi
cd ~/rob_box_project/docker/main
docker compose pull  # Скачиваем последние базовые образы
docker compose build && docker compose up -d

# Vision Pi
cd ~/rob_box_project/docker/vision
docker compose pull
docker compose build && docker compose up -d
```

### Локальная разработка:

Если вы хотите собрать базовые образы локально (без GitHub):

```bash
# Main Pi
cd ~/rob_box_project/docker/main
./scripts/build_base_images.sh

# Vision Pi
cd ~/rob_box_project/docker/vision
./scripts/build_base_images.sh
```

Затем собирайте сервисы с локальными образами:

```bash
# Переопределяем BASE_IMAGE для использования локальных образов
docker compose build --build-arg BASE_IMAGE=rob_box_base:ros2-zenoh robot-state-publisher
docker compose build --build-arg BASE_IMAGE=rob_box_base:rtabmap rtabmap
```

## Доступ к образам

Образы публикуются в публичном registry и доступны без авторизации:

```bash
# Просмотр доступных тегов
docker pull ghcr.io/krikz/rob_box_base:ros2-zenoh
docker pull ghcr.io/krikz/rob_box_base:rtabmap
docker pull ghcr.io/krikz/rob_box_base:depthai
docker pull ghcr.io/krikz/rob_box_base:pcl
```

## Кэширование

GitHub Actions использует кэширование слоёв Docker для ускорения сборки:
- Первая сборка: ~15-20 минут
- Последующие сборки: ~3-5 минут (благодаря кэшу)

## Мониторинг сборок

Статус сборок можно посмотреть в разделе Actions репозитория:
https://github.com/krikz/rob_box_project/actions

## Ручной запуск сборки

1. Перейти в Actions → Build Base Docker Images
2. Нажать "Run workflow"
3. Выбрать ветку (обычно `main`)
4. Нажать "Run workflow"

Образы будут пересобраны и опубликованы через 20-30 минут.

## Troubleshooting

### Ошибка "image not found"

Если Docker не может найти образ из ghcr.io:

```bash
# Проверьте доступность образа
docker pull ghcr.io/krikz/rob_box_base:ros2-zenoh

# Если не работает, соберите локально
cd ~/rob_box_project/docker/<main|vision>
./scripts/build_base_images.sh
```

### Использование старой версии образа

Очистите локальный кэш и скачайте свежие образы:

```bash
docker compose pull
docker compose down
docker compose up -d --force-recreate
```

### Сборка для другой архитектуры

GitHub Actions собирает для `amd64` и `arm64`. Если нужна другая платформа:

```bash
# Локальная сборка для конкретной платформы
docker buildx build --platform linux/arm64 \
  -f docker/base/Dockerfile.ros2-zenoh \
  -t rob_box_base:ros2-zenoh docker/base
```
