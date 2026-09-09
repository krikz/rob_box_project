# 🛠️ Разработка Rob Box

Документация для разработчиков проекта.

## 📄 Документы

### [agents/](agents/) 🆕
**Промпты специализированных AI-агентов**

- `docs-agent.md` — документация (этот агент)
- `navigation-agent.md` — Nav2, SLAM, одометрия
- `backend-agent.md` — REST API, FastAPI, база данных
- `voice-agent.md` — speech-to-text, TTS, LLM
- `frontend-agent.md` — React веб-интерфейс
- `devops-agent.md` — Docker, CI/CD, деплой
- `security-agent.md` — безопасность, секреты, TLS
- `git-agent.md` — коммиты, PR, ветки
- `scenarios-agent.md` — сценарии доставки, логика
- `structure-agent.md` — архитектура кода, рефакторинг

### [AGENT_GUIDE.md](AGENT_GUIDE.md) ⭐
**Руководство для AI агентов**

- Обзор системы (Main Pi, Vision Pi)
- Структура Docker проекта
- Доступ к Raspberry Pi
- Инструментарий мониторинга и диагностики
- Анализ последних изменений
- Текущая архитектура Zenoh
- Скрипты обновления и управления

### [GitHub Copilot Instructions](../../.github/copilot-instructions.md) ⭐ NEW
**Инструкции для GitHub Copilot и AI-ассистентов**

- Полный обзор проекта и технологий
- Критические файлы и правила
- Docker development rules
- Python coding standards с примерами
- ROS 2 patterns и best practices
- Security и управление секретами
- Development workflow и troubleshooting
- Quick reference для частых задач

### [CODE_REVIEW_SUMMARY.md](CODE_REVIEW_SUMMARY.md) 🆕
**Краткое резюме code review (Oct 2025)**

- Статус проекта: Production-Ready (79%)
- Ключевые находки и сильные стороны
- Оценки по компонентам
- Приоритетные действия
- Влияние на AI-ассистентов

### [CODE_REVIEW_2025-10.md](CODE_REVIEW_2025-10.md) 🆕
**Полный code review проекта (Oct 2025)**

- Executive summary
- Детальный анализ 10 компонентов
- Code quality metrics
- Примеры кода с улучшениями
- Next steps и рекомендации

### [IMPROVEMENT_RECOMMENDATIONS.md](IMPROVEMENT_RECOMMENDATIONS.md) 🆕
**Практические рекомендации по улучшению**

- High/Medium/Low priority рекомендации
- Конкретные примеры реализации
- Plan внедрения по неделям
- Ожидаемые метрики улучшения

### [DOCKER_STANDARDS.md](DOCKER_STANDARDS.md)
**Стандарты Docker проекта**

- Структура Docker файлов
- Volumes и конфигурация
- Environment переменные
- Network настройка
- Dependencies и зависимости
- Best practices

### [BUILD_OPTIMIZATION.md](BUILD_OPTIMIZATION.md)
**Оптимизация времени выполнения**

- Оптимизация OAK-D камеры
- Оптимизация RTAB-Map SLAM
- Настройка DDS (CycloneDDS/Zenoh)
- Оптимизация сети между Pi
- Снижение CPU/RAM нагрузки

### [DOCKER_BUILD_OPTIMIZATION.md](DOCKER_BUILD_OPTIMIZATION.md)
**Оптимизация процесса сборки Docker**

- Оптимизация Dockerfile структуры
- Кэширование слоев
- Multi-stage builds
- GitHub Actions cache
- Ускорение CI/CD сборок

### [LOCAL_BUILD.md](LOCAL_BUILD.md)
**Локальная сборка Docker образов**

- Сборка на dev машине (x86_64)
- Эмуляция ARM64 через QEMU
- Тестирование образов локально
- Отладка Dockerfile
- Push в registry

### [DOCKER_BUILD_FIXES.md](DOCKER_BUILD_FIXES.md)
**Исправления ошибок сборки**

- Типичные проблемы Docker builds
- Решения для apriltag/lslidar
- Memory/CPU лимиты
- Networking issues
- Package conflicts

### [VOICE_ASSISTANT_BUILD_FIX.md](VOICE_ASSISTANT_BUILD_FIX.md)
**Исправления Voice Assistant**

- Ошибки компиляции Python packages
- ONNX Runtime для ARM64
- DeepSeek API интеграция
- TTS/STT модели deployment
- Audio драйверы (ReSpeaker)

### [DEVCONTAINERS_ANALYSIS.md](DEVCONTAINERS_ANALYSIS.md)
**Анализ devcontainers**

- VS Code devcontainer конфигурация
- ROS 2 development environment
- Extensions и инструменты
- Debugging setup
- Remote development

### [LINTING_GUIDE.md](LINTING_GUIDE.md)
**Руководство по линтингу**

- Python linters (flake8, black, pylint)
- C++ linters (clang-format, cpplint)
- YAML validators
- Dockerfile linting (hadolint)
- Pre-commit hooks

### [PYTHON_STYLE_GUIDE.md](PYTHON_STYLE_GUIDE.md)
**Руководство по стилю Python**

- PEP 8 стандарты
- ROS 2 Python conventions
- Docstring format
- Type hints
- Testing conventions

### [TESTING_GUIDE.md](TESTING_GUIDE.md)
**Руководство по тестированию**

- Unit тесты (pytest, unittest)
- Integration тесты
- Launch тесты ROS 2
- Mock объекты и fixtures
- CI/CD тестирование

### AI Development Reports
Отчеты о разработке с использованием AI:
- [**AI_CONTEXT_MAP.md**](AI_CONTEXT_MAP.md) - Карта контекста проекта для AI
- [**AI_DEVELOPMENT_COMPLETE.md**](AI_DEVELOPMENT_COMPLETE.md) - Завершенные AI сессии
- [**AI_DEVELOPMENT_REVIEW.md**](AI_DEVELOPMENT_REVIEW.md) - Обзор AI разработки
- [**AI_TROUBLESHOOTING_CHECKLIST.md**](AI_TROUBLESHOOTING_CHECKLIST.md) - Чеклист диагностики для AI

## 🔧 Инструменты разработки

### Скрипты в `scripts/`
- `test_docker_builds.sh` - Тестирование всех Docker образов
- `test_docker_local_arm64.sh` - Локальное тестирование ARM64
- `validate_dockerfiles.sh` - Валидация Dockerfile синтаксиса
- `audit_documentation.sh` - Проверка документации
- `check_broken_links.sh` - Поиск битых ссылок

### Скрипты в `docker/scripts/`
- `diagnose_data_flow.sh` - Диагностика потоков данных
- `monitor_system.sh` - Мониторинг системы
- `tune.sh` - Тюнинг производительности

## 🧪 Тестирование

### Локальное тестирование в `local_test/`
- `quick_test.sh` - Быстрая проверка
- `auto_test.sh` - Автоматические тесты
- `fake_lidar_publisher.py` - Имитация LIDAR
- `test_rtabmap_2d_lidar.launch.py` - Тест RTAB-Map

### Unit тесты
- Тесты пакетов в `src/*/test/`
- ROS 2 launch тесты
- Python pytest

## 🔗 Связанные документы

- [Архитектура системы](../architecture/SYSTEM_OVERVIEW.md)
- [CI/CD Pipeline](../CI_CD_PIPELINE.md)
- [Руководства](../guides/)
- [Deployment](../deployment/)

## 📝 Процесс разработки

1. **Локальная разработка**
   - Создание feature branch
   - Разработка в devcontainer или нативно
   - Unit тесты

2. **Тестирование**
   - Локальная сборка Docker образов
   - Запуск auto_test.sh
   - Проверка документации

3. **Code Review**
   - Pull Request на GitHub
   - CI/CD проверки
   - Review от мантейнеров

4. **Деплой**
   - Merge в develop
   - Автоматическая сборка в CI
   - Deploy на тестовый робот
   - После проверки: merge в main

## 🤝 Вклад

См. [CONTRIBUTING.md](../../CONTRIBUTING.md) для деталей процесса разработки.

---

**Навигация:** [← Назад в docs/](../README.md)
