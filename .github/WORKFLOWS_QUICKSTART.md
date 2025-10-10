# 🚀 Быстрый старт GitHub Actions

## Запуск сборки вручную

1. **Открыть Actions:** https://github.com/krikz/rob_box_project/actions
2. **Выбрать workflow:**
   - 🔥 **Build All Docker Images** - собрать всё (рекомендуется)
   - 📦 **Build Base Docker Images** - только базовые образы
   - 🤖 **Build Main Pi Services** - только Main Pi (robot-state-publisher, rtabmap)
   - 📷 **Build Vision Pi Services** - только Vision Pi (oak-d, lslidar, apriltag)
3. **Запустить:** `Run workflow` → выбрать ветку → `Run workflow`

## Автоматические сборки

✅ **Ночная сборка:** каждый день в **3:00 UTC** (6:00 МСК)  
📦 Собирает: **все образы** (build-all.yml)  
🌿 Ветка: **main**

## Теги Docker образов

| Ветка | Tag | Пример полного имени образа |
|-------|-----|------------------------------|
| `main` | `latest` | `ghcr.io/krikz/rob_box:lslidar-humble-latest` |
| `develop` | `dev` | `ghcr.io/krikz/rob_box:lslidar-humble-dev` |
| `release/v1.0.0` | `rc-v1.0.0` | `ghcr.io/krikz/rob_box:lslidar-humble-rc-v1.0.0` |

## Время сборки (примерное)

- ⏱️ Полная сборка: **~1 час**
- ⏱️ Базовые образы: **~30-40 минут**
- ⏱️ Только Main Pi: **~15-20 минут**
- ⏱️ Только Vision Pi: **~20-25 минут**

## Смотреть собранные образы

📦 GitHub Packages: https://github.com/krikz?tab=packages

Две коллекции:
- `rob_box_base` - базовые образы (ros2-zenoh, rtabmap, depthai, pcl)
- `rob_box` - сервисы (robot-state-publisher, rtabmap, oak-d, lslidar, apriltag)

## Изменить время ночной сборки

Редактировать `.github/workflows/build-all.yml`:

```yaml
schedule:
  - cron: '0 3 * * *'  # Формат: Минуты Часы День Месяц День_недели (UTC)
```

**Примеры:**
- `'0 2 * * *'` - каждый день в 2:00 UTC (5:00 МСК)
- `'0 3 * * 1'` - каждый понедельник в 3:00 UTC  
- `'0 3 1 * *'` - 1-го числа каждого месяца в 3:00 UTC

## Полная документация

📖 См. [WORKFLOWS_GUIDE.md](./.WORKFLOWS_GUIDE.md) для детальной информации
