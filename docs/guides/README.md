# 📖 Руководства пользователя Rob Box

Пошаговые инструкции по настройке и использованию робота.

## 🚀 Быстрый старт

### [QUICK_START.md](QUICK_START.md)
**Краткий справочник РОББОКС**

- Первый запуск системы
- Базовые команды управления
- Проверка работоспособности
- Основные операции

### [VISION_PI_SETUP.md](VISION_PI_SETUP.md)
**Автоматическая установка Vision Pi** ⭐ NEW

- Установка с нуля одной командой
- Автоматическая настройка Docker
- Кастомный MOTD с логотипом РОББОКС
- Автозапуск контейнеров
- Полная документация процесса

## ⚙️ Настройка компонентов

### [NAV2_SETUP.md](NAV2_SETUP.md)
**Настройка Nav2 Navigation Stack**

- Установка Nav2 пакетов
- Конфигурация параметров
- Интеграция с RTAB-Map
- Настройка costmap
- Планирование пути
- Recovery поведения

### [CAN_SETUP.md](CAN_SETUP.md)
**CAN Setup для Main Pi**

- Подключение CAN адаптера
- Настройка CAN интерфейса
- Конфигурация VESC
- Тестирование связи
- Troubleshooting

### [LSLIDAR_SETUP.md](LSLIDAR_SETUP.md)
**Настройка LSLIDAR N10**

- Подключение LIDAR
- Установка драйверов
- Конфигурация параметров
- Калибровка
- Интеграция с Nav2

### [VISUALIZATION.md](VISUALIZATION.md)
**Визуализация робота в RViz2**

- Запуск RViz2
- Настройка displays
- Визуализация URDF модели
- Отображение сенсорных данных
- Планирование навигации
- Запись bag файлов

## 🔌 Управление питанием

### [POWER_MANAGEMENT.md](POWER_MANAGEMENT.md)
**Управление питанием Raspberry Pi 5**

- Схема питания системы
- Распределение нагрузки
- Мониторинг потребления
- Power delivery настройка
- Защита от перегрузок

### [RASPBERRY_PI_USB_POWER_FIX.md](RASPBERRY_PI_USB_POWER_FIX.md)
**Увеличение USB тока для мощного БП**

- Проблема USB power limit
- Редактирование EEPROM
- Установка 5A USB ограничения
- Проверка изменений
- Safety considerations

## 🔧 Диагностика

### [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
**Диагностика и решение проблем**

- Проблемы запуска
- Сетевые проблемы (Zenoh, Ethernet)
- Проблемы с сенсорами (OAK-D, LIDAR, ReSpeaker)
- Docker контейнеры
- Nav2 navigation issues
- Voice assistant проблемы
- LED анимации
- Performance troubleshooting

## 🏗️ Сборка и развертывание

### [SINGLE_SERVICE_BUILD.md](SINGLE_SERVICE_BUILD.md)
**Быстрая сборка одного Docker образа** 🆕 (октябрь 2025)

- Workflow для оперативной сборки одного сервиса
- Экономия времени: 1-5 минут вместо 30-60 минут
- Выбор ветки и конкретного сервиса
- Сборка на локальном runner
- Main Pi, Vision Pi, и базовые образы
- Пошаговые инструкции

### [HEALTH_MONITORING.md](HEALTH_MONITORING.md)
**Мониторинг здоровья системы** 🏥

- Мониторинг температуры компонентов
- Состояние вентиляторов
- Измерение веса робота
- Health status топики
- Автоматические уведомления
- Графики и метрики

### [MONITORING_SYSTEM.md](MONITORING_SYSTEM.md)
**Система мониторинга Grafana** 🆕 (октябрь 2025)

- Grafana + Prometheus + Loki
- Distributed monitoring architecture
- Красивые дашборды с 20 панелями
- Мониторинг контейнеров и логов
- Resource usage visualization
- Enable/disable скрипты

### [MAPPING_PRACTICES_RESEARCH.md](MAPPING_PRACTICES_RESEARCH.md)
**Исследование практик маппинга** 🆕 (октябрь 2025)

- Лучшие практики для RTAB-Map SLAM
- Рекомендации по настройке параметров
- Техники улучшения качества карты
- Процедуры картографии
- Tips and tricks

## 🎨 Дополнительные возможности

### [ANIMATION_EDITOR.md](ANIMATION_EDITOR.md)
**Редактор LED анимаций**

- Keyframe-based редактор
- Одновременное редактирование всех панелей
- Экспорт в YAML + PNG
- Загрузка существующих анимаций
- Автоконвертация из старого формата

### [VISION_PI_NETWORK_SETUP.md](VISION_PI_NETWORK_SETUP.md)
**Настройка сети Vision Pi**

- Конфигурация Ethernet
- Static IP настройка
- Zenoh router конфигурация
- Network диагностика

### [BASH_ALIASES.md](BASH_ALIASES.md)
**Удобные bash алиасы**

- Алиасы для управления Docker
- Команды мониторинга
- Shortcuts для частых операций

### [NODE_SETUP.md](NODE_SETUP.md)
**Настройка отдельных ROS 2 нод**

- Конфигурация нод
- Launch файлы
- Parameters и topics

### [MOTD_PREVIEW.md](MOTD_PREVIEW.md)
**Предпросмотр MOTD**

- Кастомное приветствие при входе
- Логотип РОББОКС в терминале
- Системная информация

### [INTERNAL_DIALOGUE_USAGE.md](INTERNAL_DIALOGUE_USAGE.md)
⚠️ **УСТАРЕЛО** - см. [architecture/INTERNAL_DIALOGUE_VOICE_ASSISTANT.md](../architecture/INTERNAL_DIALOGUE_VOICE_ASSISTANT.md)

## 🔗 Связанные документы

- [Архитектура системы](../architecture/SYSTEM_OVERVIEW.md)
- [Развёртывание](../deployment/)
- [Разработка](../development/)
- [Документация пакетов](../packages/)

## 💡 Советы по использованию

### Перед началом работы:
1. Проверьте питание всех компонентов
2. Убедитесь что Ethernet между Pi работает
3. Запустите базовую диагностику (`docker/scripts/diagnose_data_flow.sh`)
4. Проверьте что Zenoh router запущен на обоих Pi

### Первый запуск:
1. Начните с [QUICK_START.md](QUICK_START.md)
2. Настройте необходимые компоненты по соответствующим гайдам
3. При проблемах смотрите [TROUBLESHOOTING.md](TROUBLESHOOTING.md)

### Регулярное обслуживание:
- Проверяйте логи Docker контейнеров
- Мониторьте температуру Raspberry Pi
- Обновляйте Docker образы при изменениях
- Делайте backup карты RTAB-Map (`docker/main/maps/`)

---

**Навигация:** [← Назад в docs/](../README.md)
