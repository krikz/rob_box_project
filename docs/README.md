# 📚 Документация Rob Box Project

Полная документация робототехнической платформы Rob Box на ROS 2 Humble.

## 📂 Структура документации

**[📖 Полное описание структуры документации](DOCUMENTATION_STRUCTURE.md)**
**[🎨 Руководство по Mermaid диаграммам](MERMAID_DIAGRAMS.md)** ⭐ - Инструкции по работе с диаграммами

### 🏗️ [architecture/](architecture/) - Архитектура системы
Описание архитектуры робота, аппаратных и программных компонентов.

- [**README.md**](architecture/README.md) - Навигация по архитектурной документации
- [**SYSTEM_OVERVIEW.md**](architecture/SYSTEM_OVERVIEW.md) - Общая архитектура системы
- [**HARDWARE.md**](architecture/HARDWARE.md) - Аппаратные компоненты
- [**SOFTWARE.md**](architecture/SOFTWARE.md) - Программные компоненты
- [**INTERNAL_DIALOGUE_VOICE_ASSISTANT.md**](architecture/INTERNAL_DIALOGUE_VOICE_ASSISTANT.md) - ⭐ **НОВОЕ** Internal Dialogue + Voice Assistant (полная документация)
- [**INTERNAL_DIALOGUE_V2.md**](architecture/INTERNAL_DIALOGUE_V2.md) - ⚠️ УСТАРЕЛО (см. выше)

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
- [**READY_FOR_DEPLOY.md**](deployment/READY_FOR_DEPLOY.md) - Чеклист готовности к деплою
- [**VOICE_ASSISTANT_DOCKER.md**](deployment/VOICE_ASSISTANT_DOCKER.md) - Voice Assistant в Docker
- [**VISION_PI_DEPLOYMENT.md**](deployment/VISION_PI_DEPLOYMENT.md) - Развёртывание Vision Pi
- [**MONITORING_DEPLOYMENT.md**](deployment/MONITORING_DEPLOYMENT.md) - 🆕 Развёртывание системы мониторинга (октябрь 2025)

### 🛠️ [development/](development/) - Разработка
Руководства для разработчиков.

**AI & Code Quality (NEW 🆕)**
- [**GitHub Copilot Instructions**](../.github/copilot-instructions.md) ⭐ - Инструкции для AI-ассистентов
- [**CODE_REVIEW_SUMMARY.md**](development/CODE_REVIEW_SUMMARY.md) 🆕 - Code review summary (Oct 2025)
- [**CODE_REVIEW_2025-10.md**](development/CODE_REVIEW_2025-10.md) 🆕 - Полный code review
- [**IMPROVEMENT_RECOMMENDATIONS.md**](development/IMPROVEMENT_RECOMMENDATIONS.md) 🆕 - Рекомендации по улучшению

**Development Guides**
- [**AGENT_GUIDE.md**](development/AGENT_GUIDE.md) ⭐ - Руководство для AI агентов
- [**DOCKER_STANDARDS.md**](development/DOCKER_STANDARDS.md) - Стандарты Docker
- [**BUILD_MACHINE**](../docker/build/README.md) 🆕 ⭐ - Локальная инфраструктура сборки (октябрь 2025)
- [**PYTHON_STYLE_GUIDE.md**](development/PYTHON_STYLE_GUIDE.md) - Python coding standards
- [**BUILD_OPTIMIZATION.md**](development/BUILD_OPTIMIZATION.md) - Оптимизация сборки
- [**LOCAL_BUILD.md**](development/LOCAL_BUILD.md) - Локальная сборка Docker
- [**DOCKER_BUILD_FIXES.md**](development/DOCKER_BUILD_FIXES.md) - Исправления сборки
- [**DEVCONTAINERS_ANALYSIS.md**](development/DEVCONTAINERS_ANALYSIS.md) - Анализ devcontainers

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
- [**HEALTH_MONITORING.md**](guides/HEALTH_MONITORING.md) - 🏥 Мониторинг здоровья системы
- [**MONITORING_SYSTEM.md**](guides/MONITORING_SYSTEM.md) - 🆕 Система мониторинга с Grafana (октябрь 2025)
- [**MAPPING_PRACTICES_RESEARCH.md**](guides/MAPPING_PRACTICES_RESEARCH.md) - 🆕 Исследование практик маппинга (октябрь 2025)
- [**INTERNAL_DIALOGUE_USAGE.md**](guides/INTERNAL_DIALOGUE_USAGE.md) - ⚠️ УСТАРЕЛО (см. architecture/INTERNAL_DIALOGUE_VOICE_ASSISTANT.md)

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
- [**VISION_PI_USB_POWER_AUDIT_2025-10-13.md**](reports/VISION_PI_USB_POWER_AUDIT_2025-10-13.md)
- [**VISION_PI_CONTAINERS_FIX_2025-10-13.md**](reports/VISION_PI_CONTAINERS_FIX_2025-10-13.md)
- [**DOCKER_BUILD_FIX_2025-10-18.md**](reports/DOCKER_BUILD_FIX_2025-10-18.md)
- [**ZENOH_NAMESPACE_ANALYSIS_2025-10-23.md**](reports/ZENOH_NAMESPACE_ANALYSIS_2025-10-23.md) - 🆕 Анализ Zenoh namespace (октябрь 2025)
- [**TF_TRANSFORMATION_FIX.md**](reports/TF_TRANSFORMATION_FIX.md) - 🆕 Исправление TF трансформаций (октябрь 2025)
- [**TF_FIX_QUICK_REFERENCE.md**](reports/TF_FIX_QUICK_REFERENCE.md) - 🆕 Краткая справка по TF fix (октябрь 2025)

### ⚙️ Другие документы

- [**CI_CD_PIPELINE.md**](CI_CD_PIPELINE.md) - CI/CD конвейер и Build Machine
- [**MONITORING_QUICK_REF.md**](MONITORING_QUICK_REF.md) - 🆕 Краткая справка по мониторингу (октябрь 2025)
- [**WORKFLOW_CHANGES.md**](WORKFLOW_CHANGES.md) - 🆕 Изменения в workflow (октябрь 2025)
- [**DOCKER_TAG_MANAGEMENT.md**](DOCKER_TAG_MANAGEMENT.md) - Управление тегами Docker

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

**Последнее обновление:** 2025-10-24  
**Версия документации:** 2.1  
**Статус:** ✅ Актуализировано (мониторинг, TF fix, перемещение контейнеров, Zenoh, voice assistant)
