# 📊 Отчёты и аудиты Rob Box

Технические отчёты, аудиты, исправления проблем и итоги сессий разработки.

## 📄 Отчёты 2025-10-18

### [DOCKER_BUILD_FIX_2025-10-18.md](DOCKER_BUILD_FIX_2025-10-18.md)
**Docker Build Fix - Voice Assistant Container**

**Проблема:**
- Docker сборка падала с CMake ошибкой: `builtin_interfaces__rosidl_generator_c couldn't be found`
- Python пакет rob_box_animations использовал `find_package()` для Python зависимостей
- Отсутствовала директория `launch/` в Docker образе

**Решение:**
1. Переустановка ROS2 пакетов с `--reinstall` в том же RUN
2. Удалены ненужные `find_package(sensor_msgs/std_msgs/std_srvs)` из CMakeLists.txt
3. Добавлена `COPY src/rob_box_animations/launch` в Dockerfile

**Результат:**
- ✅ Docker build успешно завершается
- ✅ Правильный паттерн для Python-only ROS2 пакетов
- ✅ Все директории копируются в образ

**Коммиты:** `6c9f77f`, `9961f56`, `9007d44`

## 📄 Отчёты 2025-10-15

### [BROKEN_LINKS_REPORT_2025-10-15.md](BROKEN_LINKS_REPORT_2025-10-15.md)
**Отчёт о битых ссылках в документации**

- Аудит всех markdown файлов
- Список найденных битых ссылок
- Рекомендации по исправлению

### [2025-10-15-ros2-yaml-parser-issue.md](2025-10-15-ros2-yaml-parser-issue.md)
**ROS2 YAML Parser Issue**

- Проблема с парсингом конфигурационных файлов
- Workaround решения
- Обновление до исправленной версии

## 📄 Отчёты 2025-10-13

### [VISION_PI_CONTAINERS_FIX_2025-10-13.md](VISION_PI_CONTAINERS_FIX_2025-10-13.md)
**Vision Pi Containers Fix Report**

**Проблема:**
- Docker контейнеры на Vision Pi не запускались корректно
- Неправильная конфигурация volumes
- Отсутствие network_mode: host

**Исправления:**
- Добавлены все необходимые config volumes
- Настроен `network_mode: host` для всех сервисов
- Исправлены зависимости между контейнерами
- Обновлены Dockerfile для apriltag и lslidar

**Результат:**
- Все контейнеры Vision Pi запускаются стабильно
- OAK-D Lite, LSLIDAR, AprilTag работают корректно
- Zenoh router маршрутизирует данные на Main Pi

### [VISION_PI_USB_POWER_AUDIT_2025-10-13.md](VISION_PI_USB_POWER_AUDIT_2025-10-13.md)
**Vision Pi - USB Power Budget Audit**

**Проблема:**
- OAK-D Lite камера требует до 1.5A
- LSLIDAR N10 требует до 1.0A
- ReSpeaker Mic Array требует до 0.5A
- Общее потребление превышает стандартный USB лимит (1.6A)

**Анализ:**
- Raspberry Pi 5 USB порты по умолчанию ограничены 0.6A
- Необходимо увеличение до 5.0A для мощного БП

**Решение:**
- Редактирование EEPROM: `POWER_OFF_ON_HALT=0, USB_CURRENT_LIMIT=5A`
- Установка мощного БП 5V/5A
- Использование активных USB хабов для сенсоров

**Результат:**
- Все USB устройства работают стабильно
- Нет проблем с питанием при пиковых нагрузках

### [HARDWARE_VAD_FIX.md](HARDWARE_VAD_FIX.md)
**Hardware VAD Fix Report**

- Исправление проблем с ReSpeaker VAD
- Настройка аппаратного детектора голоса
- Оптимизация параметров

## 📝 Типы отчётов

### Технические аудиты
Детальный анализ компонентов системы:
- Электропитание и распределение
- Сетевая инфраструктура
- Производительность системы
- Использование ресурсов

### Исправление проблем (Fix Reports)
Документация процесса решения проблем:
- Описание проблемы
- Root cause analysis
- Шаги воспроизведения
- Примененные решения
- Результаты тестирования
- Lessons learned

### Итоги сессий (Session Summaries)
Краткие отчёты по итогам разработки:
- Выполненные задачи
- Принятые решения
- Обновленная архитектура
- Следующие шаги

## 🔗 Связанные документы

- [Troubleshooting](../guides/TROUBLESHOOTING.md)
- [Development](../development/)
- [Architecture](../architecture/)

## 📋 Правила оформления отчётов

### Naming Convention
```
<COMPONENT>_<TYPE>_<DATE>.md

Примеры:
- VISION_PI_USB_POWER_AUDIT_2025-10-13.md
- VOICE_ASSISTANT_DOCKER_FIX_2025-10-12.md
- NAV2_PERFORMANCE_AUDIT_2025-10-15.md
- WEEKLY_SESSION_SUMMARY_2025-10-01.md
```

### Структура отчёта

```markdown
# <Название отчёта>

**Дата:** YYYY-MM-DD
**Автор:** <Имя>
**Статус:** [В процессе | Завершено | Архив]

## Контекст
Описание ситуации, которая привела к созданию отчёта

## Проблема (для Fix Reports)
Детальное описание проблемы

## Анализ
Исследование причин

## Решение
Примененные исправления

## Результаты
Итоги и проверка

## Выводы
Lessons learned, рекомендации

## Связанные документы
Ссылки на код, Issues, PRs
```

---

**Навигация:** [← Назад в docs/](../README.md)
