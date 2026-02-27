# Структура документации Rob Box Project

Полная документация проекта РОББОКС организована в структурированную иерархию.

## 📂 Главные разделы

### [README.md](../README.md)
Главная страница проекта - точка входа для всех пользователей.

### [docs/README.md](README.md)
Центральный хаб документации со ссылками на все разделы.

## 🗂️ Категории документации

### 🏗️ [architecture/](architecture/)
Архитектура робототехнической системы
- Общая архитектура (SYSTEM_OVERVIEW.md)
- Аппаратные компоненты (HARDWARE.md)
- Программные компоненты (SOFTWARE.md)
- Сетевая топология (NETWORK_TOPOLOGY.md)
- Internal Dialogue + Voice Assistant
- ICP Одометрия, Zenoh Cloud Namespaces
- **archive/** — устаревшие документы (1 файл)

### 📦 [packages/](packages/)
Документация ROS 2 пакетов
- Основная документация пакетов → `src/*/README.md`
- Mapping commands (команды маппинга)
- Sound effects integration (интеграция звуков)

### 🚀 [deployment/](deployment/)
Развёртывание на роботе
- Deployment Workflow (GitHub Actions)
- Чеклисты готовности
- Vision Pi deployment
- Voice Assistant в Docker
- Мониторинг деплоймент

### 🛠️ [development/](development/)
Для разработчиков
- Agent Guide, AI агенты (agents/)
- Docker стандарты, теги, оптимизация сборки
- Python стиль, линтинг, тестирование
- Voice Agent, LLM рефакторинг
- AprilTag/Model Cache оптимизация
- **archive/** — устаревшие/дублирующие документы (15 файлов)

### 📖 [guides/](guides/)
Руководства пользователя
- Quick Start, Troubleshooting
- Настройка компонентов (Nav2, CAN, LSLIDAR)
- Управление питанием, USB Power Fix
- Visualization, Animation Editor
- Мониторинг (quick ref, system, health)
- Операционные предупреждения
- **archive/** — устаревшие документы (1 файл)

### 🔧 [fixes/](fixes/)
Исправления и багфиксы
- Zenoh (9 документов)
- RViz, Docker, Robot/URDF, ROS2
- Анимации, TTS, звуки, токены

### 📊 [reports/](reports/)
Технические отчёты и аудиты
- USB Power Audit, Container fixes
- Zenoh analysis, TF transformation
- Silero V5, NAV2 analysis
- Voice Assistant builds
- **archive/** — устаревшие версии отчётов (2 файла)

### 📝 [prompts/](prompts/)
Промпты и вспомогательные материалы
- PRD промпты, транскрипты

## 📝 Правила документации

### Где размещать документы

1. **Общая документация** → `docs/`
2. **Документация пакета** → `src/<пакет>/docs/`
3. **Корень проекта** → только README.md, CONTRIBUTING.md, CHANGELOG.md

### Формат документов

- Markdown (.md)
- Русский язык для основной документации
- Английский для README пакетов
- Диаграммы: PlantUML, Mermaid, ASCII art

### Связность документации

Каждый раздел имеет:
- **README.md** - навигация по разделу
- Ссылки на родительский раздел (← Назад)
- Ссылки на связанные документы
- Полный список файлов раздела

## 🔗 Навигация

### От главной страницы
```
README.md
  └── docs/README.md
       ├── architecture/     (8 файлов + archive/)
       ├── packages/         (6 файлов)
       ├── deployment/       (6 файлов)
       ├── development/      (~30 файлов + agents/ + archive/)
       ├── guides/           (~27 файлов + archive/)
       ├── fixes/            (~40 файлов)
       ├── reports/          (~43 файлов + archive/)
       └── prompts/          (4 файла)
```

### Перекрёстные ссылки
- Документы свободно ссылаются друг на друга
- Относительные пути от текущего файла
- Валидные ссылки проверены автоматически

## ✅ Качество документации

### Проверки
- ✅ Все файлы связаны от main README
- ✅ Нет битых ссылок
- ✅ Нет дубликатов
- ✅ Категории READMEs полные
- ✅ Внешние ссылки на GitHub repos

### Метрики
- **Активных файлов:** ~185 markdown документов
- **Архивированных:** ~18 документов (в archive/ подпапках)
- **Категорий:** 8 основных разделов
- **Orphaned:** не проверялось после реорганизации
- **Broken links:** исправлены основные (copilot-instructions, CHANGELOG)

## 🤝 Обновление документации

При добавлении нового документа:
1. Поместите в правильную категорию
2. Обновите README.md категории
3. Добавьте ссылки в связанные документы
4. Проверьте все пути относительно файла

## 📚 Полезные ссылки

- [CONTRIBUTING.md](../CONTRIBUTING.md) - как внести вклад
- [CHANGELOG.md](../CHANGELOG.md) - история изменений
- [CI/CD Pipeline](CI_CD_PIPELINE.md) - автоматизация сборки
- [GitHub README](../.github/README.md) - GitHub Actions

---

**Последнее обновление:** 2026-02-27
**Статус:** ✅ Структура реорганизована и актуализирована
