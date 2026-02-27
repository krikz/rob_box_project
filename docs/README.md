# 📚 Документация Rob Box Project

Полная документация робототехнической платформы Rob Box на ROS 2 Humble.

## 🎯 Быстрый доступ

### 🗺️ Roadmap
- [**ROADMAP_SIMPLE.md**](../ROADMAP_SIMPLE.md) ⭐ - **Презентационный формат** - обзор проекта для широкой аудитории (исследователи, менеджеры, презентации)
- [**ROADMAP.md**](../ROADMAP.md) - Техническая дорожная карта проекта: реализованные и планируемые фичи, технологический стек, этапы развития

## 📂 Структура документации

**[📖 Полное описание структуры документации](DOCUMENTATION_STRUCTURE.md)**
**[🎨 Руководство по Mermaid диаграммам](MERMAID_DIAGRAMS.md)** ⭐ - Инструкции по работе с диаграммами

### 🏗️ [architecture/](architecture/) - Архитектура системы
Описание архитектуры робота, аппаратных и программных компонентов.

- [**README.md**](architecture/README.md) - Навигация по архитектурной документации
- [**SYSTEM_OVERVIEW.md**](architecture/SYSTEM_OVERVIEW.md) - Общая архитектура системы
- [**HARDWARE.md**](architecture/HARDWARE.md) - Аппаратные компоненты (Main Pi 16GB, Vision Pi 8GB)
- [**SOFTWARE.md**](architecture/SOFTWARE.md) - Программные компоненты
- [**NETWORK_TOPOLOGY.md**](architecture/NETWORK_TOPOLOGY.md) - 🌐 Сетевая топология: IP-адреса, Zenoh, SSH
- [**ICP_ODOMETRY.md**](architecture/ICP_ODOMETRY.md) - ICP Одометрия + Wheel Fusion для RTAB-Map
- [**INTERNAL_DIALOGUE_VOICE_ASSISTANT.md**](architecture/INTERNAL_DIALOGUE_VOICE_ASSISTANT.md) - Internal Dialogue + Voice Assistant
- [**ZENOH_CLOUD_NAMESPACES.md**](architecture/ZENOH_CLOUD_NAMESPACES.md) - Zenoh Cloud Namespaces

### 📦 [packages/](packages/) - Документация пакетов
Документация ROS 2 пакетов проекта (ссылки на src/).

- [**README.md**](packages/README.md) - Навигация по пакетам
- [**rob_box_voice**](../src/rob_box_voice/README.md) - Голосовой ассистент с AI
- [**rob_box_animations**](../src/rob_box_animations/README.md) - LED анимации
- [**rob_box_bringup**](../src/rob_box_bringup/) - Системный запуск
- [**rob_box_description**](../src/rob_box_description/) - URDF модель робота

### 🚀 [deployment/](deployment/) - Развёртывание
Инструкции по развёртыванию на роботе и в Docker.

- [**README.md**](deployment/README.md) - Навигация по развёртыванию
- [**DEPLOYMENT_WORKFLOW.md**](deployment/DEPLOYMENT_WORKFLOW.md) - Workflow деплоя через GitHub Actions
- [**READY_FOR_DEPLOY.md**](deployment/READY_FOR_DEPLOY.md) - Чеклист готовности к деплою
- [**VOICE_ASSISTANT_DOCKER.md**](deployment/VOICE_ASSISTANT_DOCKER.md) - Voice Assistant в Docker
- [**VISION_PI_DEPLOYMENT.md**](deployment/VISION_PI_DEPLOYMENT.md) - Развёртывание Vision Pi
- [**MONITORING_DEPLOYMENT.md**](deployment/MONITORING_DEPLOYMENT.md) - Развёртывание системы мониторинга

### 🛠️ [development/](development/) - Разработка
Руководства для разработчиков.

**AI & Code Quality**
- [**GitHub Copilot Instructions**](../.github/copilot-instructions.md) ⭐ - Инструкции для AI-ассистентов
- [**agents/**](development/agents/) - Промпты специализированных AI-агентов (docs, navigation, backend, voice, frontend, devops...)
- [**AGENT_GUIDE.md**](development/AGENT_GUIDE.md) ⭐ - Руководство для AI агентов
- [**VOICE_AGENT_BEST_PRACTICES.md**](development/VOICE_AGENT_BEST_PRACTICES.md) - Best practices для голосового агента
- [**LLM_REFACTORING_SUMMARY.md**](development/LLM_REFACTORING_SUMMARY.md) - Итоги рефакторинга LLM

**Development Guides**
- [**DOCKER_STANDARDS.md**](development/DOCKER_STANDARDS.md) - Стандарты Docker
- [**DOCKER_TAG_MANAGEMENT.md**](development/DOCKER_TAG_MANAGEMENT.md) - Управление тегами Docker
- [**BUILD_MACHINE**](../docker/build/README.md) ⭐ - Локальная инфраструктура сборки
- [**PYTHON_STYLE_GUIDE.md**](development/PYTHON_STYLE_GUIDE.md) - Python coding standards
- [**BUILD_OPTIMIZATION.md**](development/BUILD_OPTIMIZATION.md) - Оптимизация сборки
- [**LOCAL_BUILD.md**](development/LOCAL_BUILD.md) - Локальная сборка Docker
- [**TESTING_GUIDE.md**](development/TESTING_GUIDE.md) - Руководство по тестированию
- [**LINTING_GUIDE.md**](development/LINTING_GUIDE.md) - Руководство по линтингу

**Optimization**
- [**APRILTAG_CPU_OPTIMIZATION.md**](development/APRILTAG_CPU_OPTIMIZATION.md) - Оптимизация CPU для AprilTag
- [**MODEL_CACHE_OPTIMIZATION.md**](development/MODEL_CACHE_OPTIMIZATION.md) - Оптимизация кэша моделей

**Архив** — устаревшие файлы в [development/archive/](development/archive/) (15 файлов)

### 📖 [guides/](guides/) - Руководства пользователя
Пошаговые инструкции по настройке и использованию.

- [**README.md**](guides/README.md) - Навигация по руководствам
- [**QUICK_START.md**](guides/QUICK_START.md) - Быстрый старт
- [**NAV2_SETUP.md**](guides/NAV2_SETUP.md) - Настройка навигации Nav2
- [**CAN_SETUP.md**](guides/CAN_SETUP.md) - Настройка CAN шины
- [**LSLIDAR_SETUP.md**](guides/LSLIDAR_SETUP.md) - Настройка LSLIDAR
- [**POWER_MANAGEMENT.md**](guides/POWER_MANAGEMENT.md) - Управление питанием
- [**RASPBERRY_PI_USB_POWER_FIX.md**](guides/RASPBERRY_PI_USB_POWER_FIX.md) - Исправление USB питания
- [**VISUALIZATION.md**](guides/VISUALIZATION.md) - Визуализация в RViz2
- [**TROUBLESHOOTING.md**](guides/TROUBLESHOOTING.md) - Решение проблем
- [**ANIMATION_EDITOR.md**](guides/ANIMATION_EDITOR.md) - Редактор LED анимаций
- [**MONITORING_QUICK_REF.md**](guides/MONITORING_QUICK_REF.md) - Краткая справка по мониторингу
- [**MONITORING_SYSTEM.md**](guides/MONITORING_SYSTEM.md) - Система мониторинга с Grafana
- [**HEALTH_MONITORING.md**](guides/HEALTH_MONITORING.md) - 🏥 Мониторинг здоровья системы
- [**KNOWN_OPERATIONAL_WARNINGS.md**](guides/KNOWN_OPERATIONAL_WARNINGS.md) - Известные операционные предупреждения
- [**MAPPING_PRACTICES_RESEARCH.md**](guides/MAPPING_PRACTICES_RESEARCH.md) - Исследование практик маппинга

### 🔧 [fixes/](fixes/) - Исправления и фиксы
Документация по исправлениям проблем, багфиксам и решениям.

- [**README.md**](fixes/README.md) - Навигация по исправлениям
- **Zenoh исправления** - 9 документов по Zenoh и сети
- **RViz исправления** - 4 документа по визуализации
- **Docker исправления** - 3 документа по сборке
- **Robot/URDF исправления** - 3 документа по модели робота
- **ROS2 исправления** - 3 документа по нодам

### 📊 [reports/](reports/) - Отчёты и аудиты
Технические отчёты, аудиты, итоги сессий.

- [**README.md**](reports/README.md) - Навигация по отчётам
- [**AI_HAT_UPGRADE_ANALYSIS.md**](reports/AI_HAT_UPGRADE_ANALYSIS.md) - Анализ апгрейда AI HAT
- [**VOICE_ASSISTANT_BUILD_FIX_2025-10-24_FINAL.md**](reports/VOICE_ASSISTANT_BUILD_FIX_2025-10-24_FINAL.md) - Итоговый фикс сборки Voice Assistant
- [**ZENOH_NAMESPACE_ANALYSIS_2025-10-23.md**](reports/ZENOH_NAMESPACE_ANALYSIS_2025-10-23.md) - Анализ Zenoh namespace
- [**TF_TRANSFORMATION_FIX.md**](reports/TF_TRANSFORMATION_FIX.md) - Исправление TF трансформаций
- **Архив** — устаревшие версии отчётов в [reports/archive/](reports/archive/) (2 файла)

### 🔧 [prompts/](prompts/) - Промпты и вспомогательные материалы
- [**PRD_PROMT.md**](prompts/PRD_PROMT.md) - Промпт генерации PRD
- [**prompt_PRD__tasks.md**](prompts/prompt_PRD__tasks.md) - Промпт генерации задач

### ⚙️ Другие документы

- [**CI_CD_PIPELINE.md**](CI_CD_PIPELINE.md) - CI/CD конвейер и Build Machine
- [**MERMAID_DIAGRAMS.md**](MERMAID_DIAGRAMS.md) - Руководство по Mermaid диаграммам

## 🔗 Связанные документы

- [**README.md**](../README.md) - Главная страница проекта
- [**CONTRIBUTING.md**](../CONTRIBUTING.md) - Правила участия в разработке
- [**CI/CD Pipeline**](CI_CD_PIPELINE.md) - GitHub Actions workflows
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

**Последнее обновление:** 2026-02-27  
**Версия документации:** 3.0  
**Статус:** ✅ Актуализировано (реорганизация структуры, архивирование устаревших файлов, исправление ссылок)
