<div align="center">
  <img src="assets/logo.svg" alt="РОББОКС Logo" width="400"/>
  
  # РОББОКС - Автономный колесный ровер

  [![Build All Docker Images](https://github.com/krikz/rob_box_project/actions/workflows/build-all.yml/badge.svg)](https://github.com/krikz/rob_box_project/actions/workflows/build-all.yml)
  [![Build Base Images](https://github.com/krikz/rob_box_project/actions/workflows/build-base-images.yml/badge.svg)](https://github.com/krikz/rob_box_project/actions/workflows/build-base-images.yml)
</div>

## Описание проекта
РОББОКС - это образовательный проект автономного робота, созданный для изучения работы с ROS2. Проект охватывает основные аспекты разработки роботизированных систем, включая управление движением, обработку данных с датчиков и взаимодействие с пользователем.

## 📚 Документация

### 🗺️ Roadmap
- **[ROADMAP_SIMPLE.md](ROADMAP_SIMPLE.md)** ⭐ - **Презентационный формат** - обзор проекта для широкой аудитории без технического жаргона
- **[ROADMAP.md](ROADMAP.md)** - Техническая дорожная карта: реализованные и планируемые фичи, технологический стек, этапы развития

### 🚀 Быстрый старт
Новичкам начать здесь:
- **[Быстрый старт](docs/guides/QUICK_START.md)** - Запуск системы за 10 минут
- **[Чеклист](docs/guides/VISION_PI_SETUP.md)** - Контрольный список запуска

### 📖 Руководства пользователя
- **[Визуализация в RViz](docs/guides/VISUALIZATION.md)** - Проверка URDF модели
- **[Управление питанием RPi5](docs/guides/POWER_MANAGEMENT.md)** - Питание, троттлинг, HAT
- **[Система мониторинга](docs/guides/MONITORING_SYSTEM.md)** ⭐ - Grafana, Prometheus, Loki для мониторинга робота
- **[Скрипты мониторинга](docker/monitor_system.sh)** - Проверка состояния системы
- **[Настройка LSLIDAR](docs/guides/LSLIDAR_SETUP.md)** - Подключение лидара
- **[Решение проблем](docs/guides/TROUBLESHOOTING.md)** - Диагностика и устранение неисправностей
- **[Bash алиасы](docs/deployment/VISION_PI_DEPLOYMENT.md)** - Удобные команды

### 📖 Справочная информация
- **[Архитектура](docs/architecture/SYSTEM_OVERVIEW.md)** - Полная архитектура системы
- **[Конфигурация RTAB-Map + LiDAR](docs/guides/VISUALIZATION.md)** ⭐ - 2D SLAM с лазерным сканером
- **[Fusion 360 Measurements](docs/architecture/HARDWARE.md)** - Геометрия робота
- **[VESC Integration](https://github.com/krikz/vesc_nexus)** - Интеграция моторов
- **[Оптимизация](docs/development/BUILD_OPTIMIZATION.md)** - Детали оптимизации OAK-D + RTAB-Map
- **[Резюме оптимизации](docs/development/BUILD_OPTIMIZATION.md)** - Краткий обзор изменений

### 🔧 Для разработчиков
- **[Руководство для AI агентов](docs/development/AGENT_GUIDE.md)** ⭐ Критически важно!
- **[Стандарты Docker](docs/development/DOCKER_STANDARDS.md)** - Правила работы с Docker
- **[Оптимизация сборки](docs/development/BUILD_OPTIMIZATION.md)** - Ускорение разработки
- **[Build Machine](docker/build/README.md)** ⭐ - Локальная инфраструктура сборки (10-20x быстрее)
- **[CI/CD Pipeline](docs/CI_CD_PIPELINE.md)** - Автоматизация сборки и деплоя
- **[Contributing](CONTRIBUTING.md)** - Как участвовать в проекте

📂 **[Полная документация](docs/README.md)** - Структурированный каталог всей документации

## ⚡ Последние изменения

**19 февраля 2026** - PRD и система AI-агентов:
- 📋 **PRD.md** — Product Requirements Document: 34 задачи, milestones, acceptance criteria
- 🤖 **11 AI-агентов** — специализированные агенты: navigation, backend, voice, frontend, devops, docs, diagnostics и др.
- 🔍 **diagnostics-agent** — удалённая SSH-диагностика контейнеров, ROS 2 топиков и здоровья сервисов
- 📊 **tasks.json** — структурированный список задач с приоритетами и test_steps
- 📝 **progress.md** — лог выполнения задач агентами

**30 января 2026** - Стабилизация голосового ассистента:
- 🔧 **Бесконечные анимации** — race condition исправлен, добавлен лимит итераций MAX_ITERATIONS=10
- 🎵 **51 звуковой эффект** — sound_catalog.json, загрузка по контексту, GetSoundInfoTool
- ⏱️ **Stream timeouts** — таймауты и ThreadPoolExecutor fix для dialogue_node
- 📊 **Логирование токенов** — учёт usage токенов LLM API ([TOKEN_USAGE_LOGGING.md](docs/fixes/TOKEN_USAGE_LOGGING.md))
- 🔗 **DeepSeek connection pool** — отключён idle timeout, QoS mismatch исправлен

**20 декабря 2025** - ICP Одометрия и NAV2 тюнинг:
- 📐 **ICP Odometry** — fusion wheel odometry + ICP (Iterative Closest Point) для RTAB-Map
- 🎯 **NAV2 Tuning** — детальная настройка параметров навигации ([гайд](docs/fixes/NAV2_NAVIGATION_TUNING_2025-12-16.md))
- 📡 **ICP архитектура** — [документация TF-дерева и потоков данных](docs/architecture/ICP_ODOMETRY.md)

**4 ноября 2025** - LLM провайдеры и GUI:
- 🔄 **Fallback Qwen ↔ DeepSeek** — автоматическое переключение провайдеров LLM при ошибке
- 🖥️ **GUI управления роботом** — интерфейс мониторинга и управления (`tools/robot_control_gui_simple.py`)
- 🎤 **enable_search** — параметр для Qwen API web-поиска
- ⚡ **Batching запросов** — все запросы с таймаутом 2.5с отправляются одним пакетом в LLM

**26 октября 2025** - Build Machine Infrastructure:
- 🚀 **Build Machine** - Локальная инфраструктура для сборки Docker образов
- ⚡ **10-20x быстрее** - обновление Raspberry Pi с 25-35 мин до 3-5 мин
- 🗄️ **Локальный Registry** - Docker образы хранятся в локальной сети
- 📦 **APT Cache** - кэширование пакетов ускоряет сборку в 3-5 раз
- 🤖 **Self-hosted Runner** - GitHub Actions выполняются локально

**24 октября 2025** - Крупные улучшения системы:
- ✅ **Система мониторинга** - Grafana, Prometheus, Loki для наблюдения за роботом
- ✅ **Исправление TF трансформаций** - robot-state-publisher теперь корректно публикует через Zenoh
- ✅ **Улучшение голосового ассистента** - добавлена синхронизация TTS чанков и time awareness
- ✅ **Перемещение контейнеров** - perception и lslidar теперь на Main Pi для оптимизации
- ✅ **Документация Zenoh** - полное руководство по namespace и облачному подключению
- ✅ **Исследование маппинга** - лучшие практики для RTAB-Map SLAM

**23 октября 2025** - Улучшения CI/CD:
- ✅ Изменена стратегия auto-merge на PR-based workflow для лучшего контроля
- ✅ Исправлены дублирующиеся запуски тестов и линтинга

**10 октября 2025** - Реорганизация проекта:
- ✅ Исправлена структура Dockerfiles (убраны COPY конфигов/скриптов)
- ✅ Реорганизована файловая структура (`config/{service}/`, `scripts/{service}/`)
- ✅ Добавлена документация по питанию RPi5 и скрипты мониторинга
- ✅ Структурирована документация в `docs/` с категориями
- ✅ Изменения конфигов теперь применяются за 2-5 сек вместо 5-10 мин

**8 октября 2025** - Оптимизация для Raspberry Pi:
- 🚀 Снижение CPU: с 85% до 30-45% на Pi с камерой
- 🌐 Снижение network: с 80-100 Mbps до 8-15 Mbps
- 💾 Экономия памяти: ~50% на обоих Pi
- ✅ Стабильная работа: 5 FPS без пропусков

## 🎯 Цель проекта

**РОББОКС** - автономный робот-доставщик для использования внутри помещений.

### Первый этап: Автономная навигация
- ✅ Построение карты помещения (RTAB-Map SLAM)
- ✅ Локализация робота на карте
- ✅ Планирование траектории и навигация
- 🔄 Разметка карты (точки интереса, зоны доставки)

### Второй этап: Бизнес-процесс доставки
- 📱 Клиентское приложение для вызова робота
- 🚚 Доставка предметов из точки A в точку B
- 🏠 Док-станция для автоматической подзарядки
- 🤖 Голосовой ассистент для взаимодействия

## 🛠️ Технические характеристики

### Механика и привод
- **Колеса:** 4×10" (от электросамоката)
- **Регуляторы:** 4× VESC (CAN-шина)
- **Hardware Interface:** [vesc_nexus](https://github.com/krikz/vesc_nexus) - ROS2 драйвер для управления VESC через CAN
- **CAN Shield:** установлен на Main Pi для связи с VESC

### Бортовые компьютеры
- **Main Pi** (Raspberry Pi 5, 16GB RAM):
  - RTAB-Map SLAM (LiDAR-based 2D SLAM)
  - Robot State Publisher
  - CAN Shield для связи с VESC
  - Навигация и планирование (Nav2)
  - LSLIDAR N10 драйвер
  - Perception (health monitoring, context aggregator)
  - MCP Tools сервер (управление роботом через LLM)
  
- **Vision Pi** (Raspberry Pi 5, 8GB RAM):
  - OAK-D-Lite драйвер + AprilTag детектор
  - MJPEG потолочная камера (ceiling-based локализация)
  - ReSpeaker Mic Array v2.0 (голосовой ассистент)
  - LED Matrix Driver (381 NeoPixel)
  - Telegram-бот оператора (`rob_box_telegram`)

### Сенсоры
- **Камера:** OAK-D-Lite (RGB + Stereo Depth)
- **Raspberry Pi Camera:** для ориентации по потолку
- **Лидар:** LSLIDAR N10 (360° 2D)
- **Микрофон:** ReSpeaker Mic Array v2.0 (для голосового ассистента)
- **Сенсорная плата:** ESP32 с micro-ROS ([robot_sensor_hub](https://github.com/krikz/robot_sensor_hub))
  - 1-8× датчиков AHT30 (температура/влажность)
  - Тензодатчик HX711 (измерение веса)
  - 2× вентилятора с PWM и тахометром

### Индикация
- **LED матрицы:** [ros2leds](https://github.com/krikz/ros2leds) - управление через ROS2
  - 4× NeoPixel 8×8 (фары: передние и задние)
  - 5× NeoPixel 5×5 (основной дисплей 5×25)
  - Композитор для объединения панелей в логические группы

### Коммуникация
- **Middleware:** Zenoh DDS (rmw_zenoh_cpp)
- **Сеть:** WiFi роутер D-Link DIR-320
- **Связь между Pi:** Zenoh router для оптимизации трафика

## 🚀 Функциональные возможности

### Реализовано
- ✅ **SLAM и построение карты** (RTAB-Map с LSLIDAR N10 2D LiDAR)
- ✅ **Управление двигателями** через VESC Nexus (CAN-интерфейс)
- ✅ **LED индикация** с 381 NeoPixel (4 группы: фары + дисплей)
- ✅ **Мониторинг здоровья** робота (температура, вентиляторы, вес)
- ✅ **AprilTag детектор** для маркеров (Vision Pi)
- ✅ **Распределённая архитектура** на двух Raspberry Pi с Zenoh DDS
- ✅ **Система мониторинга** с Grafana, Prometheus, Loki (октябрь 2025)
- ✅ **Голосовой ассистент** с time awareness и синхронизацией TTS (октябрь 2025)
- ✅ **LLM Fallback** автоматический Qwen ↔ DeepSeek (ноябрь 2025)
- ✅ **ICP Одометрия** — fusion wheel + ICP для RTAB-Map (декабрь 2025)
- ✅ **GUI управления роботом** — мониторинг и телеуправление (ноябрь 2025)
- ✅ **Telegram-бот оператора** — управление роботом через Telegram + LLM (2026)
- ✅ **MCP Tools сервер** — инструменты управления роботом для LLM агентов (2026)

### В разработке
- 🔄 **Навигация** - автономное планирование и движение по карте (TASK-003/004)
- 🔄 **REST API** — бэкенд для управления роботом (TASK-011)
- 🔄 **Веб-интерфейс** — React-приложение для заказа доставки (TASK-017)
- 🔄 **Клиентское приложение** для заказа доставки (TASK-017)

### Планируется
- 📋 **Док-станция** для автоматической зарядки
- 📋 **Интеграция с LLM** для естественного общения
- 📋 **Система предотвращения столкновений**
- 📋 **Телеметрия и удалённый мониторинг**

## 📦 Связанные репозитории

- **[vesc_nexus](https://github.com/krikz/vesc_nexus)** - ROS2 драйвер для VESC регуляторов через CAN
  - Поддержка до 4+ VESC на CAN-шине
  - Публикация одометрии и состояния моторов
  - Управление через `/cmd_vel`
  
- **[ros2leds](https://github.com/krikz/ros2leds)** - Управление NeoPixel матрицами
  - Драйвер для цепочки светодиодов через SPI
  - Композитор для объединения панелей в логические группы
  - Поддержка произвольного расположения панелей
  
- **[robot_sensor_hub](https://github.com/krikz/robot_sensor_hub)** - Сенсорная плата на ESP32 с micro-ROS
  - Температура/влажность (AHT30)
  - Тензодатчик (HX711)
  - Управление вентиляторами с мониторингом RPM

## 💻 Программное обеспечение

- **ОС:** Ubuntu 24.04.2 LTS (на обоих Raspberry Pi)
- **Фреймворк:** ROS 2 Humble Hawksbill
- **Middleware:** Zenoh DDS (rmw_zenoh_cpp) для оптимизации сетевого трафика
- **SLAM:** RTAB-Map (RGB-D + 2D LiDAR)
- **Камера:** DepthAI для OAK-D-Lite
- **Контейнеризация:** Docker + Docker Compose

