# 📚 Документация Rob Box Project

Полная документация робототехнической платформы Rob Box на ROS 2 Humble.

## 📂 Структура документации

### 🏗️ [architecture/](architecture/) - Архитектура системы
Описание архитектуры робота, аппаратных и программных компонентов.

- [**SYSTEM_OVERVIEW.md**](architecture/SYSTEM_OVERVIEW.md) - Общая архитектура системы
- [**HARDWARE.md**](architecture/HARDWARE.md) - Аппаратные компоненты
- [**SOFTWARE.md**](architecture/SOFTWARE.md) - Программные компоненты

### 📦 [packages/](packages/) - Документация пакетов
Документация ROS 2 пакетов проекта (ссылки на src/).

- [**rob_box_voice**](../src/rob_box_voice/README.md) - Голосовой ассистент с AI
- [**rob_box_animations**](../src/rob_box_animations/README.md) - LED анимации
- [**rob_box_bringup**](../src/rob_box_bringup/) - Системный запуск
- [**rob_box_description**](../src/rob_box_description/) - URDF модель робота

### 🚀 [deployment/](deployment/) - Развёртывание
Инструкции по развёртыванию на роботе и в Docker.

- [**READY_FOR_DEPLOY.md**](deployment/READY_FOR_DEPLOY.md) - Чеклист готовности к деплою
- [**VOICE_ASSISTANT_DOCKER.md**](deployment/VOICE_ASSISTANT_DOCKER.md) - Voice Assistant в Docker
- [**VISION_PI_DEPLOYMENT.md**](deployment/VISION_PI_DEPLOYMENT.md) - Развёртывание Vision Pi

### 🛠️ [development/](development/) - Разработка
Руководства для разработчиков.

- [**AGENT_GUIDE.md**](development/AGENT_GUIDE.md) - Руководство для AI агентов
- [**BUILD_OPTIMIZATION.md**](development/BUILD_OPTIMIZATION.md) - Оптимизация сборки
- [**DOCKER_STANDARDS.md**](development/DOCKER_STANDARDS.md) - Стандарты Docker
- [**LOCAL_BUILD.md**](development/LOCAL_BUILD.md) - Локальная сборка Docker
- [**DOCKER_BUILD_FIXES.md**](development/DOCKER_BUILD_FIXES.md) - Исправления сборки
- [**DEVCONTAINERS_ANALYSIS.md**](development/DEVCONTAINERS_ANALYSIS.md) - Анализ devcontainers

### 📖 [guides/](guides/) - Руководства пользователя
Пошаговые инструкции по настройке и использованию.

- [**QUICK_START.md**](guides/QUICK_START.md) - Быстрый старт
- [**NAV2_SETUP.md**](guides/NAV2_SETUP.md) - Настройка навигации Nav2
- [**CAN_SETUP.md**](guides/CAN_SETUP.md) - Настройка CAN шины
- [**LSLIDAR_SETUP.md**](guides/LSLIDAR_SETUP.md) - Настройка LSLIDAR
- [**POWER_MANAGEMENT.md**](guides/POWER_MANAGEMENT.md) - Управление питанием
- [**RASPBERRY_PI_USB_POWER_FIX.md**](guides/RASPBERRY_PI_USB_POWER_FIX.md) - Исправление USB питания
- [**VISUALIZATION.md**](guides/VISUALIZATION.md) - Визуализация в RViz2
- [**TROUBLESHOOTING.md**](guides/TROUBLESHOOTING.md) - Решение проблем

### 📊 [reports/](reports/) - Отчёты и аудиты
Технические отчёты, аудиты, итоги сессий.

- [**VISION_PI_USB_POWER_AUDIT_2025-10-13.md**](reports/VISION_PI_USB_POWER_AUDIT_2025-10-13.md)
- [**VISION_PI_CONTAINERS_FIX_2025-10-13.md**](reports/VISION_PI_CONTAINERS_FIX_2025-10-13.md)

### ⚙️ Другие документы

- [**CI_CD_PIPELINE.md**](CI_CD_PIPELINE.md) - CI/CD конвейер

## 🔗 Связанные документы

- [**README.md**](../README.md) - Главная страница проекта
- [**CONTRIBUTING.md**](../../../CONTRIBUTING.md) - Правила участия в разработке
- [**.github/workflows/**](../.github/workflows/) - GitHub Actions workflows
- [**docker/**](../docker/) - Docker конфигурации

## 📝 Правила документации

### Где размещать документацию

1. **Общая документация проекта** → `/docs/`
   - Архитектура, развёртывание, руководства

2. **Документация конкретного пакета** → `/src/<пакет>/docs/`
   - Детали реализации, фазы разработки, тестирование

3. **Корень проекта** → ТОЛЬКО эти файлы:
   - `README.md` - Обзор проекта
   - `CONTRIBUTING.md` - Правила разработки
   - `CHANGELOG.md` - История изменений
   - `LICENSE` - Лицензия

### Язык документации

- **Русский язык** - основной язык документации
- **Английский язык** - README.md пакетов (для GitHub)

### Формат

- Markdown (.md)
- Диаграммы: PlantUML, Mermaid, ASCII art
- Скриншоты: PNG, WebP (оптимизированные)

## 🤝 Вклад в документацию

При добавлении новых функций или изменении существующих **обязательно** обновляйте документацию:

1. Обновите соответствующий файл в `/docs/` или `/src/<пакет>/docs/`
2. Обновите ссылки в README-индексах
3. Проверьте все перекрёстные ссылки
4. Добавьте запись в `CHANGELOG.md`

---

**Последнее обновление:** 2025-10-13  
**Версия документации:** 2.0  
**Статус:** ✅ Реорганизовано по стандартам ROS2
