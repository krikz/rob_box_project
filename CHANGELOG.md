# Changelog

Все значимые изменения в проекте Rob Box документируются в этом файле.

Формат основан на [Keep a Changelog](https://keepachangelog.com/ru/1.0.0/),
и этот проект придерживается [Semantic Versioning](https://semver.org/lang/ru/).

## [Unreleased]

### Добавлено
- Голосовой ассистент rob_box_voice с DeepSeek, Vosk STT, Silero TTS
- LED анимации rob_box_animations для WS2812B матриц
- Интеграция Zenoh для распределённой связи между Vision Pi и Main Pi
- Docker контейнеры для всех сервисов
- RTAB-Map SLAM с OAK-D Lite камерой
- AprilTag детекция на Vision Pi
- Nav2 навигация с командным управлением
- Документация в docs/ по стандартам ROS 2
- Система мониторинга с Grafana, Prometheus, Loki (24 октября 2025)
  - Легковесный мониторинг на отдельной машине
  - cAdvisor и Promtail на обоих Raspberry Pi
  - Красивые Grafana дашборды с 20 панелями
  - Скрипты enable/disable для управления мониторингом
- Полная документация по Zenoh namespace и облачному подключению (23 октября 2025)
- Исследование практик маппинга для RTAB-Map (24 октября 2025)
- Time awareness в dialogue_node - робот теперь знает текущее время (24 октября 2025)
- dialogue_id для синхронизации TTS чанков между сеансами диалога (24 октября 2025)

### Изменено
- Миграция с ROS 2 topics на Zenoh pub/sub
- Переход на offline-first стратегию для STT/TTS
- Реорганизация Docker структуры по стандартам проекта
- Оптимизация сборки для Raspberry Pi 4
- Система накопления запросов в dialogue_node - все запросы отправляются одним пакетом в DeepSeek (4 ноября 2025)
  - Запросы накапливаются в очереди с таймаутом 2.5 секунды
  - Все накопленные запросы (в том числе пришедшие во время обработки LLM) отправляются одним батчем
  - Улучшена обработка и минимизировано количество запросов к API
- Перемещение perception и lslidar контейнеров с Vision Pi на Main Pi (24 октября 2025)
  - Освобождение ресурсов Vision Pi для камеры и микрофона
  - Централизация обработки данных на Main Pi
- Рефакторинг системы мониторинга - агенты на Pi, центральный стек на отдельной машине (24 октября 2025)
- Изменена стратегия CI/CD - создание PR вместо прямого auto-merge (23 октября 2025)
- Реорганизация скриптов и конфигов согласно DOCKER_STANDARDS.md (24 октября 2025)

### Исправлено
- USB питание на Vision Pi для OAK-D камеры
- Проблемы с контейнерами Vision Pi (config volumes, network_mode)
- Ошибки компиляции apriltag и lslidar драйверов в Docker
- TF трансформации - robot-state-publisher теперь использует Zenoh namespace wrapper (24 октября 2025)
- Порядок TTS чанков - предотвращение смешивания между сеансами диалога (24 октября 2025)
- Отсутствующие директории scripts/ в Docker volumes (24 октября 2025)
- Дублирование запусков тестов и линтинга в CI/CD (23 октября 2025)
- Предупреждение 'PerceptionEvent не найден' в voice-assistant (24 октября 2025)
- Orphaned workflow build-all-local.yml - добавлен placeholder с deprecation notice (28 октября 2025)

## [0.1.0] - 2025-10-04

### Добавлено
- Первый релиз базовой системы
- URDF модель робота rob_box_description
- Базовые launch файлы rob_box_bringup
- Интеграция VESC моторных контроллеров vesc_nexus
- ESP32 сенсорный хаб robot_sensor_hub_msg
- LED драйверы ros2leds и led_matrix_driver

---

**Навигация:** [← Назад в README](README.md) | [📚 Документация](docs/README.md)
