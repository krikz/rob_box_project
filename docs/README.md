# 📚 Документация Rob Box Project

Полная документация проекта автономного робота на ROS 2.

## 🚀 Быстрый старт

Начните здесь, если вы впервые работаете с проектом:

- **[Быстрый старт (RU)](getting-started/QUICK_START_RU.md)** - Запуск системы за 10 минут
- **[Чеклист запуска](getting-started/CHECKLIST.md)** - Контрольный список для проверки системы

## 📖 Руководства пользователя

Пошаговые инструкции по работе с компонентами системы:

### Питание и мониторинг
- **[Управление питанием RPi5](guides/POWER_MANAGEMENT.md)** - Полное руководство по питанию Raspberry Pi 5
  - Режимы питания (USB-A, USB-C PD, GPIO, официальный адаптер)
  - Симптомы недостаточного питания и троттлинга
  - Программные и аппаратные решения
  - Рекомендации для rob_box_project

- **[Скрипты мониторинга питания](guides/POWER_MONITORING_SCRIPTS.md)** - Использование скриптов мониторинга
  - `check_power_status.sh` - полная проверка состояния
  - `check_usb_devices.sh` - анализ USB устройств
  - `monitor_power_live.sh` - мониторинг в реальном времени
  - Типичные сценарии и интерпретация результатов

### Настройка и конфигурация
- **[Настройка LSLIDAR](guides/LSLIDAR_SETUP.md)** - Подключение и настройка лидара LSLIDAR N10
- **[Bash алиасы](guides/BASH_ALIASES.md)** - Удобные алиасы для работы с проектом
- **[Решение проблем](guides/TROUBLESHOOTING.md)** - Диагностика и устранение неисправностей

## 📋 Справочная информация

Детальная техническая документация:

- **[Архитектура системы](reference/ARCHITECTURE.md)** - Полное описание архитектуры проекта
  - Топология сети (Main Pi + Vision Pi)
  - Zenoh middleware конфигурация
  - ROS 2 граф нод и топиков
  - Аппаратная конфигурация

- **[Оптимизация системы](reference/OPTIMIZATION_README.md)** - Детальное описание оптимизаций
  - Оптимизация OAK-D камеры
  - Настройка CycloneDDS
  - Конфигурация RTAB-Map
  - Результаты тестирования

- **[Краткое резюме оптимизации](reference/OPTIMIZATION_SUMMARY.md)** - Быстрый обзор проделанной работы

- **[Zenoh: видео-гайд](reference/ZENOH_VIDEO_GUIDE.md)** - Конспект видео про Zenoh middleware
- **[Zenoh: оптимизация энергопотребления](reference/ZENOH_POWER_OPTIMIZATION.md)** - Специфичные настройки Zenoh
- **[Системные настройки](reference/SYSTEM_TUNING.md)** - Тюнинг операционной системы для роботики

## 🔧 Документация для разработчиков

Руководства для участников проекта и AI агентов:

- **[Руководство для AI агентов](development/AGENT_GUIDE.md)** - Критически важно для AI!
  - Обзор системы и топология сети
  - Структура Docker проекта
  - Доступ к Raspberry Pi
  - **Критические правила для Dockerfiles**
  - Инструментарий мониторинга

- **[Стандарты Docker Compose](development/DOCKER_STANDARDS.md)** - Правила работы с Docker
  - Структура файлов и папок
  - Стандарты volumes и environment
  - Workflow добавления сервисов
  - Валидация конфигурации

- **[Оптимизация сборки Docker](development/BUILD_OPTIMIZATION.md)** - Ускорение разработки
  - Volume mounting вместо COPY
  - Структура config/ и scripts/
  - Правила для Dockerfiles
  - Быстрое применение изменений (2-5 сек вместо 5-10 мин)

- **[CONTRIBUTING.md](../CONTRIBUTING.md)** - Как участвовать в проекте

## 🗂️ Структура документации

```
docs/
├── getting-started/     # Быстрый старт и первые шаги
├── guides/              # Пошаговые руководства для пользователей
├── reference/           # Справочная и техническая информация
└── development/         # Документация для разработчиков
```

## 📊 Статус проекта

- **Версия**: 2025-10-10 (Active Development)
- **ROS 2**: Humble
- **Платформа**: Raspberry Pi 4/5
- **Middleware**: Zenoh (rmw_zenoh_cpp)

## 🔗 Полезные ссылки

- **GitHub**: [github.com/krikz/rob_box_project](https://github.com/krikz/rob_box_project)
- **Docker Registry**: [ghcr.io/krikz/rob_box](https://ghcr.io/krikz/rob_box)
- **Main README**: [../README.md](../README.md)

---

**Последнее обновление**: 2025-10-10  
**Автор**: КУКОРЕКЕН  
**Проект**: rob_box_project
