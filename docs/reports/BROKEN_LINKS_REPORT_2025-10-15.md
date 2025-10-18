# 🔗 Отчёт по битым ссылкам в документации

**Дата:** 2025-10-15  
**Коммит:** d00b02c

## 📊 Статистика

- **Всего проверено ссылок:** 275
- **Найдено битых ссылок:** 65
- **Файлов с битыми ссылками:** 21
- **Исправлено в этой сессии:** 18 ссылок

## ✅ Что исправлено

1. **Анимации (21 ссылка)**
   - Гифки переехали: `src/rob_box_animations/assets/` → `docs/assets/animations/`
   - Исправлены все ссылки в `src/rob_box_animations/docs/ANIMATIONS.md`

2. **Структура документации (12 ссылок)**
   - Удалены дублирующиеся `docs/docs/` пути
   - Исправлены пути в `README.md`, `host/README.md`
   - Обновлены ссылки на реорганизованную документацию

3. **Относительные пути (6 ссылок)**
   - Исправлены `../docs/` → `../`
   - Исправлены `../CONTRIBUTING.md` → `../../CONTRIBUTING.md`

## ❌ Что осталось исправить

### Критические (требуют создания файлов)

#### 1. `.github/WORKFLOWS_QUICKSTART.md`
- `WORKFLOWS_GUIDE.md` - нужно создать подробный гайд по workflows

#### 2. `docker/README_RU.md`
- `DOCKER_STANDARDS.md` - стандарты Docker контейнеров
- `AGENT_GUIDE.md` - гайд по агентам

#### 3. `docs/architecture/`
- `API_REFERENCE.md` - API документация
- `DEPLOYMENT.md` - гайд по деплою (или переименовать из `docs/deployment/`)

#### 4. `docs/guides/`
- `TROUBLESHOOTING.md` - общий troubleshooting
- `RASPBERRY_PI_USB_POWER_FIX.md` - уже есть, нужно поправить ссылки
- `API_REFERENCE.md` - дублирует architecture

### Некритические (можно удалить или заменить)

#### 1. Устаревшие ссылки
- `docs/development/VOICE_ASSISTANT_ARCHITECTURE.md` → заменить на `src/rob_box_voice/README.md`
- `docs/guides/CAN_SETUP.md` → есть в `host/README.md`
- `docs/reference/*` → все переехали в другие места

#### 2. Внешние зависимости
- `src/ros2leds/LICENSE` - отсутствует в submodule
- Нужно добавить LICENSE в ros2leds

## 📝 План действий

### Приоритет 1 (срочно)

1. Создать `docs/CI_CD_PIPELINE.md` - документация по CI/CD
2. Создать `docs/architecture/API_REFERENCE.md` - API спецификация
3. Переименовать/переместить файлы troubleshooting

### Приоритет 2 (важно)

4. Создать `.github/WORKFLOWS_GUIDE.md` - подробный гайд по GitHub Actions
5. Добавить `LICENSE` в `src/ros2leds/`
6. Создать `docker/DOCKER_STANDARDS.md` - стандарты Docker

### Приоритет 3 (можно позже)

7. Убрать все ссылки на `docs/reference/*` - эта структура упразднена
8. Консолидировать troubleshooting гайды
9. Обновить все ссылки на hardware/software документацию

## 🛠️ Инструменты

Созданы два скрипта для работы с документацией:

### `scripts/check_links.py`
Проверяет все markdown файлы на битые ссылки:
```bash
python3 scripts/check_links.py
```

### `scripts/fix_broken_links.py`
Автоматически исправляет известные битые ссылки:
```bash
python3 scripts/fix_broken_links.py
```

## 📦 Структура документации (актуальная)

```
docs/
├── README.md                           # Главная страница документации
├── CI_CD_PIPELINE.md                   # ❌ Нужно создать
├── architecture/                        # Архитектура системы
│   ├── HARDWARE.md                     # ✅ Описание железа
│   ├── SOFTWARE.md                     # ✅ Программная архитектура
│   ├── SYSTEM_OVERVIEW.md              # ✅ Общий обзор
│   └── API_REFERENCE.md                # ❌ Нужно создать
├── deployment/                          # Развёртывание
│   └── VISION_PI_DEPLOYMENT.md         # ✅ Развёртывание Vision Pi
├── development/                         # Разработка
│   ├── BUILD_OPTIMIZATION.md           # ✅ Оптимизация сборки
│   └── README.md                       # ✅ Гайд разработчика
├── guides/                              # Пользовательские гайды
│   ├── QUICK_START.md                  # ✅ Быстрый старт
│   ├── NODE_SETUP.md                   # ✅ Настройка узлов
│   ├── VISION_PI_SETUP.md              # ✅ Настройка Vision Pi
│   ├── VISION_PI_NETWORK_SETUP.md      # ✅ Сеть Vision Pi
│   ├── VISUALIZATION.md                # ✅ Визуализация
│   ├── LSLIDAR_SETUP.md                # ✅ Настройка лидара
│   ├── NAV2_SETUP.md                   # ✅ Навигация
│   ├── POWER_MANAGEMENT.md             # ✅ Управление питанием
│   └── RASPBERRY_PI_USB_POWER_FIX.md   # ✅ USB питание
├── packages/                            # Пакеты ROS2
│   └── README.md                       # ✅ Описание пакетов
├── reports/                             # Отчёты и аудиты
│   └── *.md                            # ✅ Различные отчёты
└── assets/                              # Медиа файлы
    ├── animations/                      # ✅ GIF анимации
    └── *.gif                           # ✅ Демо и скриншоты
```

## 🔄 Следующие шаги

1. Создать недостающие файлы из списка "Критические"
2. Прогнать `fix_broken_links.py` ещё раз с обновлённым маппингом
3. Проверить все ссылки: `check_links.py`
4. Обновить документацию в соответствии с новой структурой
5. Добавить правила ведения документации в `CONTRIBUTING.md`

## 📌 Правила ведения документации

### Структура ссылок

- ✅ Использовать относительные пути от текущего файла
- ✅ Проверять ссылки перед коммитом: `python3 scripts/check_links.py`
- ✅ Не создавать дублирующиеся пути (`docs/docs/`)
- ❌ Не использовать абсолютные пути от корня проекта

### Именование файлов

- ✅ `UPPER_SNAKE_CASE.md` для документации
- ✅ `README.md` для описания директории/пакета
- ✅ Понятные имена файлов, описывающие содержание

### Организация

- ✅ Группировать по категориям (architecture, guides, development)
- ✅ Один файл = одна тема
- ✅ Ссылаться на существующие файлы вместо дублирования
- ❌ Не создавать глубокую вложенность (>3 уровня)

---

**Автор:** AI Agent  
**Инструменты:** `check_links.py`, `fix_broken_links.py`
