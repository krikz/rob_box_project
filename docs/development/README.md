# 🛠️ Разработка Rob Box

Документация для разработчиков проекта.

## 📄 Документы

### [agents/](agents/) 🆕
**Промпты специализированных AI-агентов**

- `docs-agent.md` — документация (этот агент)
- `product-manager-agent.md` — приоритизация бэклога
- `structure-agent.md` — архитектура кода, рефакторинг

Шаблонные «доменные» промты (backend, voice, navigation и др.) удалены (ADR-0163): процесс — `docs/process/claude-coordinator-robot-flow.md`.

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

### [DOCKER_STANDARDS.md](DOCKER_STANDARDS.md)
**Стандарты Docker проекта**

- Структура Docker файлов
- Volumes и конфигурация
- Environment переменные
- Network настройка
- Dependencies и зависимости
- Best practices

### [LOCAL_BUILD.md](LOCAL_BUILD.md)
**Локальная сборка Docker образов**

- Сборка на dev машине (x86_64)
- Эмуляция ARM64 через QEMU
- Тестирование образов локально
- Отладка Dockerfile
- Push в registry

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
**Тесты** — устаревшая заглушка; актуально — `AGENTS.md` (гарды до пуша) и `TEST_COVERAGE_STRATEGY.md`.

## 🔧 Инструменты разработки

### Скрипты в `scripts/`
- `test_docker_builds.sh` - Тестирование всех Docker образов
- `test_docker_local_arm64.sh` - Локальное тестирование ARM64
- `validate_dockerfiles.sh` - Валидация Dockerfile синтаксиса
- `audit_documentation.sh` - Проверка документации
- `diagnostics/check_links.py` - Поиск битых ссылок в `*.md` (единственная проверялка, ADR-0166)

### Скрипты в `docker/scripts/`
- `diagnose_data_flow.sh` - Диагностика потоков данных
- `monitor_system.sh` - Мониторинг системы
- `tune.sh` - Тюнинг производительности

## 🧪 Тестирование

### Локальное тестирование в `tools/dev/` (бывший `local_test/`)
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
