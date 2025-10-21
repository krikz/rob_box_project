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
- Internal Dialogue + Voice Assistant

### 📦 [packages/](packages/)
Документация ROS 2 пакетов
- rob_box_voice - голосовой ассистент
- rob_box_animations - LED анимации
- rob_box_bringup - системный запуск
- Mapping commands
- Sound effects integration

### 🚀 [deployment/](deployment/)
Развёртывание на роботе
- Чеклисты готовности
- Vision Pi deployment
- Voice Assistant в Docker

### 🛠️ [development/](development/)
Для разработчиков
- Agent Guide
- Docker стандарты
- Build optimization
- Linting и testing guides
- AI development reports

### 📖 [guides/](guides/)
Руководства пользователя
- Quick Start
- Настройка компонентов (Nav2, CAN, LSLIDAR)
- Управление питанием
- Troubleshooting
- Visualization
- Animation Editor

### 📊 [reports/](reports/)
Технические отчёты
- USB Power Audit
- Container fixes
- Docker build fixes
- Hardware VAD fix

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
       ├── architecture/
       ├── packages/
       ├── deployment/
       ├── development/
       ├── guides/
       └── reports/
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
- **Всего файлов:** 112 markdown документов
- **Категорий:** 6 основных разделов
- **Orphaned:** 0 критических (все важные документы связаны)
- **Broken links:** 0 (все исправлены)

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

**Последнее обновление:** 2025-10-21
**Статус:** ✅ Структура организована и валидирована
