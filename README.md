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
- **[ROADMAP.md](ROADMAP.md)** ⭐ - Полная дорожная карта проекта: реализованные и планируемые фичи, технологический стек, этапы развития

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
- **Main Pi** (Raspberry Pi, 15GB RAM):
  - RTAB-Map SLAM
  - Robot State Publisher
  - CAN Shield для связи с VESC
  - Навигация и планирование
  - LSLIDAR N10 драйвер (перемещён с Vision Pi 24.10.2025)
  - Perception (health monitoring, context aggregator)
  
- **Vision Pi** (Raspberry Pi, 8GB RAM):
  - OAK-D-Lite драйвер
  - AprilTag детектор
  - Raspberry Pi Camera (направлена вверх для ceiling-based локализации)
  - ReSpeaker Mic Array v2.0 (голосовой ассистент)

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
- ✅ **SLAM и построение карты** (RTAB-Map с OAK-D + LSLIDAR)
- ✅ **Управление двигателями** через VESC Nexus (CAN-интерфейс)
- ✅ **LED индикация** с композитором панелей
- ✅ **Мониторинг здоровья** робота (температура, вентиляторы, вес)
- ✅ **AprilTag детектор** для маркеров
- ✅ **Распределённая архитектура** на двух Raspberry Pi с Zenoh
- ✅ **Система мониторинга** с Grafana, Prometheus, Loki (октябрь 2025)
- ✅ **Голосовой ассистент** с time awareness и синхронизацией TTS (октябрь 2025)

### В разработке
- 🔄 **Навигация** - автономное планирование и движение по карте
- 🔄 **Разметка карты** - программный комплекс для создания точек доставки
- 🔄 **Улучшение голосового ассистента** - интеграция с LLM для естественного общения
- 🔄 **Клиентское приложение** для заказа доставки

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

