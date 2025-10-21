# GitHub Actions CI/CD

Полная документация по CI/CD конвейеру находится в основной документации проекта.

## 📚 Документация

- **[CI/CD Pipeline](../docs/CI_CD_PIPELINE.md)** - Полное описание CI/CD конвейера, workflows и процессов сборки

## 🚀 Быстрый старт

### Запуск сборки вручную

1. **Открыть Actions:** https://github.com/krikz/rob_box_project/actions
2. **Выбрать workflow:**
   - 🔥 **Build All Docker Images** - собрать всё (рекомендуется)
   - 📦 **Build Base Docker Images** - только базовые образы
   - 🤖 **Build Main Pi Services** - только Main Pi
   - 📷 **Build Vision Pi Services** - только Vision Pi
3. **Запустить:** `Run workflow` → выбрать ветку → `Run workflow`

### Автоматические сборки

✅ **При пуше в develop:** автоматическая сборка всех образов  
✅ **При пуше в main:** публикация образов с тегом `latest`  
✅ **Ночная сборка:** каждый день в 3:00 UTC

## 🐳 Docker образы

📦 **GitHub Packages:** https://github.com/krikz?tab=packages

Две коллекции:
- `rob_box_base` - базовые образы (ros2-zenoh, rtabmap, depthai, pcl)
- `rob_box` - сервисы (robot-state-publisher, rtabmap, oak-d, lslidar, apriltag)

## 📁 Workflows

Все workflow файлы находятся в `.github/workflows/`:

- `auto-merge-feature-to-develop.yml` - автомерж feature → develop
- `auto-merge-to-main.yml` - автомерж develop → main
- `build-all.yml` - полная сборка всех образов
- `build-base-images.yml` - базовые образы
- `build-main-services.yml` - сервисы Main Pi
- `build-vision-services.yml` - сервисы Vision Pi
- `validate-docker-compose.yml` - валидация docker-compose

Подробное описание каждого workflow см. в [CI/CD Pipeline](../docs/CI_CD_PIPELINE.md).
