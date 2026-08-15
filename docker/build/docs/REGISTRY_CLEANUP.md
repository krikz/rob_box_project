# Registry Cleanup Guide

## Автоочистка registry (keep N)

`cleanup_registry.sh --keep N` удаляет старые версии образов, оставляя
последние N для каждого сервис-варианта:

```bash
cd ~/rob_box_project/docker/build
./scripts/cleanup_registry.sh --keep 5 --dry-run  # Просмотр (ничего не удаляет)
./scripts/cleanup_registry.sh --keep 5            # Удалить старые версии + GC
```

### Как это работает

1. Теги вида `voice-assistant-humble-dev-abc1234` группируются по
   сервис-варианту (отбрасывается sha-суффикс) → `voice-assistant-humble-dev`.
2. Внутри группы теги сортируются по времени создания образа (`created` из
   image config), удаляются все, кроме последних N.
3. Rolling-теги без sha-суффикса (`dev`, `latest`, `local`, `test`,
   `voice-assistant-humble-dev`) **никогда не удаляются** — они нужны для деплоя.
4. После удаления запускается `registry garbage-collect` для сборки blob'ов.

### Автоматическая очистка (cron)

Раз в сутки в 04:00 (`0 4 * * *`) cron запускает
`cleanup_registry.sh --keep 5`. Устанавливается идемпотентно в `setup.sh`.

Проверка:
```bash
crontab -l | grep cleanup_registry
tail -50 /tmp/cleanup_registry.log
du -sh data/registry
```

## Быстрое удаление образа из локального registry

### Использование скрипта

```bash
cd ~/rob_box_project/docker/build
./scripts/delete_image_from_registry.sh <имя-образа>
```

### Примеры

```bash
# Удалить все версии robot-state-publisher
./scripts/delete_image_from_registry.sh robot-state-publisher

# Удалить все версии voice-assistant
./scripts/delete_image_from_registry.sh voice-assistant

# Удалить все версии oak-d
./scripts/delete_image_from_registry.sh oak-d
```

### Что делает скрипт

1. ✅ Удаляет теги через Registry API
2. ✅ Останавливает registry контейнер
3. ✅ Удаляет директории тегов с файловой системы (через sudo)
4. ✅ Перезапускает registry
5. ✅ Запускает garbage collection
6. ✅ Удаляет локальные Docker образы
7. ✅ Чистит buildx cache
8. ✅ Проверяет результат

### После удаления

Образ готов к пересборке через:

**Вариант 1: Локальная сборка**
```bash
cd ~/rob_box_project
./scripts/local-build.sh <имя-сервиса>
```

**Вариант 2: GitHub Actions**
- Открыть GitHub → Actions → "L: Build Single Service"
- Выбрать параметры:
  - branch: `develop`
  - pi_type: `main` или `vision`
  - service: `имя-сервиса`

**Вариант 3: Полная пересборка Main/Vision Pi**
```bash
# Main Pi services
cd ~/rob_box_project/docker/build
gh workflow run "L-Build Main Pi Services.yml" --ref develop

# Vision Pi services
gh workflow run "L-Build Vision Pi Services.yml" --ref develop
```

## Ручная очистка (если скрипт не работает)

### Шаг 1: Остановить registry
```bash
cd ~/rob_box_project/docker/build
docker compose down build-registry
```

### Шаг 2: Найти директории тегов
```bash
find ./data/registry/docker/registry/v2/repositories/krikz/rob_box/_manifests/tags/ \
  -type d -name "*<имя-образа>*"
```

### Шаг 3: Удалить директории
```bash
sudo rm -rf ./data/registry/docker/registry/v2/repositories/krikz/rob_box/_manifests/tags/*<имя-образа>*
```

### Шаг 4: Перезапустить registry
```bash
docker compose up -d build-registry
```

### Шаг 5: Удалить локальные образы
```bash
docker images | grep <имя-образа> | awk '{print $1":"$2}' | xargs docker rmi -f
```

### Шаг 6: Очистить buildx cache
```bash
docker builder prune -f
```

## Проверка результата

### Проверить registry
```bash
curl -s http://localhost:5000/v2/krikz/rob_box/tags/list | jq -r '.tags[]' | grep <имя-образа>
```

Должен вернуть **пустой результат** (no matches).

### Проверить локальные образы
```bash
docker images | grep <имя-образа>
```

Должен вернуть **пустой результат** (no matches).

## Troubleshooting

### "Permission denied" при удалении директорий
**Решение:** Используйте `sudo rm -rf`

### Теги остаются в registry после API deletion
**Решение:** Используйте filesystem deletion (sudo rm -rf) после остановки registry

### Garbage collection не удаляет blob'ы
**Причина:** Это нормально - blob'ы могут использоваться другими образами

**Решение:** Запустить `cleanup_registry.sh --all` для полной очистки (удалит ВСЕ неиспользуемые blob'ы)

## См. также

- [cleanup_registry.sh](../scripts/cleanup_registry.sh) - Полная очистка registry (все неиспользуемые blob'ы)
- [Docker Build README](../README.md) - Общая документация build machine
- [CI/CD Pipeline](../../docs/CI_CD_PIPELINE.md) - Автоматизированная сборка через GitHub Actions
