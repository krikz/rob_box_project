# 📖 Руководства пользователя Rob Box

Пошаговые инструкции по настройке и использованию робота.

## 🚀 Быстрый старт

### [QUICK_START.md](QUICK_START.md)
**Краткий справочник РОББОКС**

- Первый запуск системы
- Базовые команды управления
- Проверка работоспособности
- Основные операции

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
