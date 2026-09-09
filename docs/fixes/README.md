# 🔧 Исправления и фиксы Rob Box

Документация по исправлениям проблем, багфиксам и решениям технических задач.

## 🗣️ Исправления Voice Assistant (Январь 2026)

### [ANIMATION_LOOP_FIX.md](ANIMATION_LOOP_FIX.md) 🆕
**Исправление бесконечного цикла анимаций**

- Проблема: LED анимации зависали в бесконечном цикле
- Решение: Добавлен MAX_ITERATIONS и таймер для force stop
- Результат: Анимации корректно останавливаются через заданное время

### [ANIMATION_TTS_FIX.md](ANIMATION_TTS_FIX.md) 🆕
**Синхронизация TTS и анимаций**

- Проблема: Анимация "thinking" не останавливалась после TTS
- Решение: Обновлена логика state machine и sync механизма
- Результат: Плавный переход между состояниями голосового ассистента

### [SCARY_STORY_FIX.md](SCARY_STORY_FIX.md) 🆕
**Исправление scary story dialogue node**

- Проблема: Ошибки в скриптах историй, неправильное завершение
- Решение: Рефакторинг dialogue flow, добавлены fallback механизмы
- Результат: Истории проигрываются без ошибок, корректное завершение

### [PROMPT_REPETITION_FIX.md](PROMPT_REPETITION_FIX.md) 🆕
**Исправление повторяющихся промптов в LLM**

- Проблема: DeepSeek R1 получал дублирующиеся промпты
- Решение: Дедупликация context в dialogue manager
- Результат: Более релевантные ответы, меньше токенов

### [QOS_MISMATCH_FIX.md](QOS_MISMATCH_FIX.md) 🆕
**Исправление QoS mismatch между нодами**

- Проблема: Некоторые ноды не получали сообщения из-за QoS несоответствия
- Решение: Стандартизация QoS политик для `/voice/*` топиков
- Результат: Stable communication между voice nodes

## 🤖 Исправления LLM Backend (Январь 2026)

### [DEEPSEEK_CONNECTION_POOL_FIX.md](DEEPSEEK_CONNECTION_POOL_FIX.md) 🆕
**Исправление connection pool deadlock**

- Проблема: ThreadPoolExecutor зависал при множественных запросах к DeepSeek
- Решение: Отдельные пулы для R1 и V3, увеличены лимиты, таймауты
- Результат: Стабильная parallel обработка streaming запросов

### [DEEPSEEK_REASONER_FIX.md](DEEPSEEK_REASONER_FIX.md) 🆕
**Исправление DeepSeek R1 reasoning chunks**

- Проблема: Reasoning токены попадали в TTS, "chipmunk voice"
- Решение: Фильтрация reasoning before sending to TTS
- Результат: Только финальный текст озвучивается, корректная скорость

### [TOKEN_USAGE_LOGGING.md](TOKEN_USAGE_LOGGING.md) 🆕
**Система логирования токенов LLM**

- Реализация: PostgreSQL таблица для tracking token usage
- Метрики: input/output tokens, cost, latency, model
- Использование: Анализ стоимости, оптимизация промптов

### [TOKEN_USAGE_RU.md](TOKEN_USAGE_RU.md) 🆕
**Инструкция по настройке token usage tracking (RU)**

- Пошаговая настройка PostgreSQL и таблиц
- Интеграция с dialogue_node
- Dashboard и визуализация данных

### [FIX_SUMMARY.md](FIX_SUMMARY.md) 🆕
**Общий summary всех январских фиксов**

- Краткое описание всех 10 исправлений
- Связи между фиксами
- Результаты и метрики

---

## 🤖 Исправления роботной платформы

### [NAV2_NAVIGATION_TUNING_2025-12-16.md](NAV2_NAVIGATION_TUNING_2025-12-16.md) 🆕
**Настройка навигации Nav2 для плавного движения и стабильности карты**

- Исправление агрессивного поведения робота
- Решение проблемы отката назад (backup recovery)
- Устранение "взлета" на карте RTAB-Map
- Добавление transform_tolerance для ICP delay
- Увеличение RGBD/OptimizeMaxError для loop closures
- Снижение Oscillation critic для плавного движения
- Safety margin 200mm для избежания столкновений

### [ROBOT_ORIENTATION_FIX.md](ROBOT_ORIENTATION_FIX.md)
**Исправление ориентации робота в RViz**

- Проблема отображения в RViz
- Исправление URDF модели
- Проверка результатов

### [WHEEL_AXES_FIX_2025-11-20.md](WHEEL_AXES_FIX_2025-11-20.md)
**Исправление осей колес Rob Box (итоговое решение)**

- Анализ проблемы осей
- Корректировка URDF
- Валидация изменений

### [WHEEL_JOINTS_FIX_2025-11-20.md](WHEEL_JOINTS_FIX_2025-11-20.md)
**Исправление джойнтов колес Rob Box**

- Проблема с джойнтами колес
- Обновление конфигурации
- Тестирование

## 📡 Исправления Zenoh и сети

### [ZENOH_FIX_SUMMARY_2025-11-10.md](ZENOH_FIX_SUMMARY_2025-11-10.md)
**Резюме исправления конфликта портов Zenoh роутера**

- Описание проблемы
- Решение конфликта портов
- Развертывание исправления

### [ZENOH_FIX_2025-11-10_MAXIMUM.md](ZENOH_FIX_2025-11-10_MAXIMUM.md)
**Максимальное исправление Zenoh Transport ошибок**

- Усиленное исправление транспортных ошибок
- Конфигурация роутера и клиентов
- Полная документация изменений

### [ZENOH_FIX_2025-11-10_DEPLOYMENT.md](ZENOH_FIX_2025-11-10_DEPLOYMENT.md)
**Развёртывание усиленного исправления Zenoh Transport**

- Инструкции по развертыванию
- Проверка на Main Pi и Vision Pi
- Валидация результатов

### [ZENOH_TRANSPORT_FIX_QUICKREF.md](ZENOH_TRANSPORT_FIX_QUICKREF.md)
**Quick Reference: Zenoh Transport Error Fix**

- Быстрый справочник по исправлению
- Основные команды
- Типичные ошибки

### [ZENOH_ROUTER_PORT_CONFLICT_FIX_2025-11-10.md](ZENOH_ROUTER_PORT_CONFLICT_FIX_2025-11-10.md)
**Исправление конфликта портов Zenoh роутера на Vision Pi**

- Диагностика конфликта
- Изменение портов
- Проверка работоспособности

### [ZENOH_PORT_CONFLICT_SOLUTION.md](ZENOH_PORT_CONFLICT_SOLUTION.md)
**Решение проблемы "Address in use" на Main Pi и Vision Pi**

- Анализ проблемы с портами
- Решение для обоих Pi
- Конфигурация роутеров

### [ZENOH_ETHERNET_INTERFACE_FIX_2025-11-10.md](ZENOH_ETHERNET_INTERFACE_FIX_2025-11-10.md)
**Исправление маршрутизации трафика Zenoh через Ethernet**

- Проблема использования WiFi вместо Ethernet
- Привязка к eth0 интерфейсу
- Конфигурация listen endpoints

### [ZENOH_ETHERNET_QUICKFIX.md](ZENOH_ETHERNET_QUICKFIX.md)
**Быстрое исправление: Zenoh через Ethernet**

- Суть проблемы
- Быстрое решение
- Проверка результатов

## 🎥 Исправления RViz и визуализации

### [RVIZ_ZENOH_FIX_FINAL.md](RVIZ_ZENOH_FIX_FINAL.md)
**RViz Zenoh Router Endpoint Fix**

- Проблема подключения RViz к Zenoh
- Конфигурация endpoint
- Тестирование связи

### [RVIZ_ZENOH_CRITICAL_FIX_2025-11-19.md](RVIZ_ZENOH_CRITICAL_FIX_2025-11-19.md)
**RViz Zenoh Critical Configuration Fix**

- Критическое исправление конфигурации
- Обновление параметров
- Валидация работы

### [RVIZ_NAMESPACE_FIX.md](RVIZ_NAMESPACE_FIX.md)
**Fix: RViz не видит топики робота - решение проблемы namespace**

- Проблема с namespace
- Исправление конфигурации
- Проверка топиков

### [RVIZ_ZENOH_NAMESPACE_INVESTIGATION.md](RVIZ_ZENOH_NAMESPACE_INVESTIGATION.md)
**RViz Zenoh Namespace Investigation Report**

- Полное исследование проблемы
- Анализ топологии Zenoh
- Диагностика и решение

## 🔖 Исправления AprilTag

### [APRILTAG_TF_FIX_2025-11-10.md](APRILTAG_TF_FIX_2025-11-10.md)
**AprilTag TF Transform Fix**

- Проблема с TF трансформациями
- Исправление конфигурации
- Тестирование детекции

## 📡 Исправления LiDAR

### [LIDAR_180_ROTATION_FIX_2025-11-20.md](LIDAR_180_ROTATION_FIX_2025-11-20.md)
**LiDAR 180° Rotation Fix**

- Проблема переворота данных LiDAR
- Исправление ориентации
- Валидация сканирования

## 🐳 Исправления Docker и сборки

### [DOCKER_BUILD_CONTEXT_FIX.md](DOCKER_BUILD_CONTEXT_FIX.md)
**Docker Build Context Fix for Local Runner Workflows**

- Проблема с build context
- Исправление workflow
- Оптимизация сборки

### [DOCKER_FIXES_2025-10-25.md](DOCKER_FIXES_2025-10-25.md)
**Docker Container Fixes**

- Множественные исправления контейнеров
- Обновление конфигураций
- Проверка работоспособности

### [LOCAL_BUILDER_FIX_SUMMARY.md](LOCAL_BUILDER_FIX_SUMMARY.md)
**Local Builder Fix Summary**

- Исправления локального builder
- Оптимизация процесса сборки
- Улучшение производительности

## 🤖 Исправления ROS 2 нод

### [FIX_ROBOT_STATE_PUBLISHER_CACHE_INVALIDATION.md](FIX_ROBOT_STATE_PUBLISHER_CACHE_INVALIDATION.md)
**Исправление кэширования robot_state_publisher**

- Проблема с кэшем состояния робота
- Инвалидация кэша
- Проверка публикации TF

### [FIX_SOUND_REPETITION_ISSUE.md](FIX_SOUND_REPETITION_ISSUE.md)
**Исправление проблемы повторения звука**

- Анализ проблемы
- Исправление логики воспроизведения
- Тестирование

### [FIX_TTS_CHUNKS_ORDER.md](FIX_TTS_CHUNKS_ORDER.md)
**Исправление порядка чанков TTS**

- Проблема с порядком воспроизведения
- Синхронизация чанков
- Валидация результатов

## 🔗 Связанные документы

- [Руководства пользователя](../guides/TROUBLESHOOTING.md)
- [Отчеты о проблемах](../reports/)
- [Архитектура системы](../architecture/SYSTEM_OVERVIEW.md)
- [Руководство по разработке](../development/)

---

**Навигация:** [← Назад в docs/](../README.md)
