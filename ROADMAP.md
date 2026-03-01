# РОББОКС - Roadmap и реализованные фичи

<div align="center">
  <img src="assets/logo.svg" alt="РОББОКС Logo" width="300"/>
  
  **Дорожная карта развития автономного робота-доставщика**
  
  [![Status](https://img.shields.io/badge/Status-Active%20Development-green)]()
  [![ROS](https://img.shields.io/badge/ROS-Humble-blue)]()
  [![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-red)]()
</div>

---

## 📋 Содержание

- [Обзор проекта](#обзор-проекта)
- [Реализованные фичи](#реализованные-фичи)
  - [Инфраструктура](#инфраструктура)
  - [Аппаратное обеспечение](#аппаратное-обеспечение)
  - [Навигация и локализация](#навигация-и-локализация)
  - [Восприятие и сенсоры](#восприятие-и-сенсоры)
  - [Управление и привод](#управление-и-привод)
  - [Взаимодействие с пользователем](#взаимодействие-с-пользователем)
  - [Мониторинг и диагностика](#мониторинг-и-диагностика)
- [Планируемые фичи](#планируемые-фичи)
  - [Навигация следующего уровня](#навигация-следующего-уровня)
  - [Продвинутая локализация](#продвинутая-локализация)
  - [AI и автономность](#ai-и-автономность)
  - [Бизнес-логика доставки](#бизнес-логика-доставки)
  - [Расширение инфраструктуры](#расширение-инфраструктуры)
- [Технологический стек](#технологический-стек)
- [Этапы развития](#этапы-развития)
- [⭐ Быстрый обзор фич по тирам](#-быстрый-обзор-фич-по-тирам)

---

## ⭐ Быстрый обзор фич по тирам

Упрощенный список ключевых фич без дублей, организованных по актуальности и технологической продвинутости.

### 🔥 Tier S - Cutting Edge (На пике технологий, высокий спрос)

**Реализовано:**
- ✅ **Voice Assistant с LLM** - DeepSeek/Qwen + Vosk STT + Silero TTS (offline fallback)
- ✅ **Zenoh DDS** - современная альтернатива CycloneDDS с облачной интеграцией
- ✅ **Build Machine** - локальная CI/CD инфраструктура (10-20x ускорение)
- ✅ **Dual Raspberry Pi 5** - распределённая обработка (16GB + 8GB)

**Планируется:**
- 🔄 **LLM-powered Autonomous Agent** - полноценный AI агент с tool use (PR #362, в активной разработке)
- � **AI HAT+ 26 TOPS (Hailo-8L NPU)** - hardware-accelerated inference (YOLOv8, Whisper, face recognition), Vision Pi. Анализ: `docs/AI_HAT_UPGRADE_ANALYSIS.md`
- 🔮 **Stereo Visual Odometry** - visual SLAM от OAK-D камеры
- 🔮 **Sensor Fusion (EKF)** - объединение всех источников одометрии
- 🔮 **Self-Hosted LLM Infrastructure** - собственные сервера для LLM моделей

### 💎 Tier A - Advanced (Продвинутые, востребованные)

**Реализовано:**
- ✅ **RTAB-Map SLAM** - RGB-D + 2D LiDAR fusion с оптимизацией для Pi (🔧 активное тестирование совместимости)
- ✅ **Nav2 Navigation** - автономная навигация с obstacle avoidance (🔧 активное тестирование настройки и оптимизация)
- ✅ **Monitoring Stack** - Grafana + Prometheus + Loki (20+ dashboard panels)
- ✅ **381 LED Matrix** - композитор панелей с анимациями и эмоциями

**Планируется:**
- 🔮 **Semantic Mapping** - разметка карты с точками интереса (kitchen, bedroom, charging station)
- 🔮 **Auto-Docking Station** - визуальная локализация + зарядка (±1-2 см точность)
- 🔮 **Dynamic Obstacle Avoidance** - real-time детекция и prediction траекторий людей (зависит от AI HAT+ person detection)
- 🔮 **Ceiling AprilTag Localization** - абсолютное позиционирование без дрейфа
- 🔮 **Client App** - Web/Mobile для заказа доставки с tracking
- 🔮 **Face Recognition & User Database** - распознавание лиц и база пользователей (зависит от AI HAT+)
- 🔮 **QR Code Authorization** - авторизация по QR-коду для доступа к грузу

### 🛠️ Tier B - Solid (Надёжные, проверенные)

**Реализовано:**
- ✅ **VESC Motor Control** - CAN Bus управление с wheel odometry (50 Hz)
- ✅ **ros2_control** - стандартный hardware interface для моторов
- ✅ **OAK-D Lite** - RGB-D камера с оптимизацией трафика (~85% сжатие)
- ✅ **AprilTag Detection** - визуальные маркеры для локализации
- ✅ **ESP32 Sensor Hub** - температура, вес, вентиляторы (micro-ROS)
- ✅ **Docker Microservices** - контейнеризация с volume-монтированием конфигов

**Планируется:**
- 🔮 **Behavior Trees** - иерархическое планирование задач (Groot visualization)
- 🔮 **Backend API** - управление заказами, routing optimization
- 🔮 **Cargo Management** - погрузка/разгрузка с weight verification
- 🔮 **Simulator** - Gazebo/Isaac Sim для тестирования
- 🔮 **OTA Updates** - over-the-air обновления с rollback

### 🔬 Tier C - Experimental (Исследовательские, нишевые)

**Планируется:**
- 🔮 **UWB Localization** - ultra-wideband для больших помещений/улицы (10-30 см точность)
- 🔮 **RTK GPS** - для работы на открытых территориях (1-2 см точность)
- 🔮 **Multi-Floor Navigation** - elevator detection, 3D mapping
- 🔮 **Multi-Robot Fleet** - координация нескольких роботов, charging queue

---

## Обзор проекта

**РОББОКС** — это образовательный проект автономного робота-доставщика для работы внутри помещений. Проект демонстрирует современные подходы к созданию роботизированных систем с использованием ROS 2, распределённой архитектуры и облачных технологий.

### Ключевые цели:
- 🎯 **Автономная навигация** в закрытых помещениях
- 📦 **Доставка предметов** между точками интереса
- 🤖 **Естественное взаимодействие** через голосовой интерфейс
- 🔬 **Исследовательская платформа** для изучения ROS 2 и робототехники

---

## Реализованные фичи

### 🏗️ Инфраструктура

#### ✅ CI/CD Pipeline
- **GitHub Actions workflows** для автоматической сборки (текущее решение)
  - Отдельные workflows для Main Pi и Vision Pi
  - Автоматическая сборка базовых образов (ros2-zenoh, rtabmap, depthai, pcl)
  - Conditional build - сборка только изменённых сервисов
- **Автоматическое создание PR** из feature веток в develop
- **Автоматическое создание PR** из develop в main
- **Docker Registry** интеграция с GitHub Container Registry (ghcr.io)
- **Semantic versioning** для Docker образов (`-latest`, `-dev`, `-test`, `-rc-X.X.X`)
- **Скрипт управления тегами** (`scripts/set-docker-tags.sh`)
- 🔮 **В планах:** переход на собственную CI/CD инфраструктуру для ускорения

**Стек:** GitHub Actions, Docker, ghcr.io  
**Документация:** `docs/CI_CD_PIPELINE.md`

#### ✅ Build Machine Infrastructure (локальная ускоренная сборка)
- **Self-hosted GitHub Actions runner** для локальной сборки
- **Локальный Docker Registry** (`localhost:5000`) для быстрого pull на Pi
- **APT Cacher NG** для кэширования пакетов (3-5x ускорение)
- **Результат:** Обновление Pi сократилось с 25-35 минут до 3-5 минут (10-20x)

**Стек:** Docker Registry, APT Cacher NG, GitHub Actions  
**Документация:** `docker/build/README.md`

#### ✅ Система мониторинга
- **Grafana** дашборды с 20+ панелями
- **Prometheus** для сбора метрик
- **Loki** для централизованных логов
- **cAdvisor** на каждом Pi для метрик контейнеров
- **Promtail** на каждом Pi для отправки логов
- **Легковесная архитектура:** агенты на Pi, центральный стек на отдельной машине

**Стек:** Grafana, Prometheus, Loki, cAdvisor, Promtail  
**Документация:** `docs/guides/MONITORING_SYSTEM.md`

#### ✅ Docker микросервисная архитектура
- **Распределённая архитектура:** Main Pi (навигация) + Vision Pi (камера, голос)
- **Volume монтирование** конфигов - изменения за 5 сек вместо 5-10 мин пересборки
- **Zenoh Router** как базовая зависимость для всех сервисов
- **network_mode: host** для оптимизации производительности
- **Стандартизированная структура:** `docker/{main,vision}/{service}/`

**Стек:** Docker, Docker Compose  
**Документация:** `docs/development/DOCKER_STANDARDS.md`

#### ✅ Deployment автоматизация
- **Automated deployment workflow** (`L-Deploy and Verify.yml`)
- **Проверка здоровья контейнеров** после деплоя
- **Анализ логов на ошибки**
- **Проверка ROS 2 топиков**
- **Автоматическое создание GitHub Issues** при проблемах

**Стек:** GitHub Actions, SSH, Docker  
**Документация:** `docs/DEPLOYMENT_WORKFLOW.md`

#### ✅ Code Quality
- **Pre-commit hooks** для проверки перед коммитом
- **Hadolint** для линтинга Dockerfiles
- **YAML lint** для конфигурационных файлов
- **Python style guide** (black, isort, flake8)

**Стек:** pre-commit, hadolint, yamllint, black, isort, flake8  
**Документация:** `docs/development/PYTHON_STYLE_GUIDE.md`

---

### 🔩 Аппаратное обеспечение

#### ✅ Dual Raspberry Pi архитектура
- **Main Pi (16GB RAM):**
  - RTAB-Map SLAM
  - Nav2 Navigation
  - ros2_control для моторов
  - LSLIDAR N10 драйвер
  - Perception (health monitor, context aggregator)
  - CAN Shield для связи с VESC
  
- **Vision Pi (8GB RAM):**
  - OAK-D Lite камера
  - AprilTag детектор
  - Raspberry Pi Camera (потолочная локализация)
  - ReSpeaker Mic Array v2.0
  - LED Matrix драйвер (381 LEDs)
  - Voice Assistant
  - � **AI HAT+ 26 TOPS** (в планировании) - Hailo-8L NPU, person detection, Whisper STT, face recognition

**Стек:** Raspberry Pi 5 (2.4GHz ARM Cortex-A76)  
**Документация:** `docs/architecture/HARDWARE.md`

#### ✅ Сенсоры
- **LSLIDAR N10** - 2D лазерный сканер (360°, 10 Hz)
- **OAK-D Lite** - RGB-D камера (1280×720 @ 5Hz, depth 640×400)
- **Raspberry Pi Camera** - для ceiling-based локализации
- **ReSpeaker Mic Array v2.0** - 6-микрофонный массив с hardware AEC
- **ESP32 Sensor Hub:**
  - 1-8× AHT30 (температура/влажность)
  - HX711 (тензодатчик для измерения веса)
  - 2× вентилятора с PWM и тахометром

**Стек:** USB, SPI, I2C, micro-ROS  
**Документация:** `docs/architecture/HARDWARE.md`

#### ✅ Актуаторы
- **2× VESC** моторные контроллеры (CAN Bus)
- **4× колеса** 10" от электросамоката
- **381× NeoPixel LED** (4× 8×8 панели + 1× 25×5 дисплей)
- **Управление вентиляторами** через ESP32

**Стек:** VESC, CAN Bus, WS2812B LEDs  
**Интеграция:** [vesc_nexus](https://github.com/krikz/vesc_nexus), [ros2leds](https://github.com/krikz/ros2leds)

#### ✅ Сетевая топология
- **Dual Network:** Gigabit Ethernet (10.1.1.x) для ROS 2, WiFi (10.1.1.2x) для SSH
- **Static IP адресация:**
  - Main Pi: 10.1.1.10 (eth0), 10.1.1.20 (wlan0)
  - Vision Pi: 10.1.1.11 (eth0), 10.1.1.21 (wlan0)
- **Zenoh Router топология:** Main Pi (peer), Vision Pi (client)
- **Облачное подключение:** zenoh.robbox.online:7447 для удалённого мониторинга

**Стек:** Ethernet, WiFi, Zenoh  
**Документация:** `docs/architecture/SYSTEM_OVERVIEW.md`, `docs/architecture/ZENOH_CLOUD_NAMESPACES.md`

---

### 🗺️ Навигация и локализация

#### ✅ RTAB-Map SLAM (🔧 активное тестирование совместимости)
- **RGB-D + 2D LiDAR fusion** для построения карты
- **3D облако точек + 2D occupancy grid**
- **Loop closure detection** для коррекции дрейфа
- **Persistent maps** - сохранение и загрузка карт
- **Оптимизация для Raspberry Pi:**
  - Снижено разрешение изображений (×2 decimation)
  - Уменьшено количество feature points (400 вместо 1000)
  - Использован GFTT+BRIEF вместо SIFT/SURF
  - Результат: ~50% снижение CPU, стабильные 5 FPS

**Стек:** RTAB-Map, PCL, OpenCV  
**Документация:** `docs/guides/MAPPING_PRACTICES_RESEARCH.md`

#### ✅ Nav2 Navigation Stack (🔧 активное тестирование настройки и оптимизация)
- **Глобальное планирование:** NavFn Planner (A* на сетке)
- **Локальное следование:** DWB Controller (Dynamic Window Approach)
- **Избежание препятствий:** Local/Global Costmaps
- **Recovery behaviors:** Spin, BackUp, Clear Costmaps
- **Waypoint navigation:** следование через последовательность точек
- **Behavior Trees:** гибкая логика навигации

**Стек:** Nav2, BehaviorTree.CPP  
**Документация:** `docs/guides/NAV2_SETUP.md`

#### ✅ Одометрия
- **Wheel odometry** от VESC моторов (50 Hz)
- **Публикация в TF дерево** через vesc_nexus
- **Fusion с IMU** (планируется через ESP32)

**Стек:** VESC CAN, ros2_control  
**Интеграция:** [vesc_nexus](https://github.com/krikz/vesc_nexus)

#### ✅ AprilTag детекция
- **Визуальная локализация** по маркерам
- **Интегрирована в oak-d контейнер** для оптимизации
- **Публикация TF трансформаций** тегов
- **Поддержка семейств:** tag36h11, tagStandard41h12

**Стек:** apriltag_ros, depthai  
**Документация:** `docs/packages/`

---

### 👁️ Восприятие и сенсоры

#### ✅ OAK-D Lite интеграция
- **RGB камера:** 1280×720 @ 5 Hz с JPEG сжатием (quality 80)
- **Stereo depth:** 640×400 @ 5 Hz с PNG сжатием (level 3)
- **Hardware-accelerated** обработка на DepthAI чипе
- **Оптимизация трафика:** 80-100 Mbps → 8-15 Mbps (сжатие ~85%)

**Стек:** DepthAI, depthai-ros  
**Документация:** `docs/development/BUILD_OPTIMIZATION.md`

#### 🔄 Context Aggregator (MPC Lite)
- **Централизованное хранилище контекста** робота (⚠️ в процессе доработки)
- **События от всех систем:**
  - Vision события (object detection, person detection)
  - Sensor events (температура, вес, RPM вентиляторов)
  - System events (ошибки, предупреждения)
  - Navigation events (goal reached, obstacle detected)
- **Публикация агрегированных событий** для Internal Dialogue

**Стек:** ROS 2, Python  
**Пакет:** `rob_box_perception`  
**Документация:** `docs/architecture/INTERNAL_DIALOGUE_VOICE_ASSISTANT.md`

#### 🔄 Health Monitor
- **Мониторинг температуры** компонентов (8× AHT30 сенсоров) (⚠️ требует переработки)
- **Контроль оборотов вентиляторов** (RPM monitoring)
- **Измерение веса груза** (HX711 тензодатчик)
- **Публикация событий** при аномалиях (overheating, fan failure)

**Стек:** micro-ROS, ESP32  
**Пакет:** `rob_box_perception`  
**Интеграция:** [robot_sensor_hub](https://github.com/krikz/robot_sensor_hub)

---

### 🎮 Управление и привод

#### ✅ VESC Nexus драйвер
- **CAN Bus интерфейс** для управления до 4+ VESC контроллеров
- **Velocity control** через `/cmd_vel`
- **Wheel odometry** публикация (50 Hz)
- **Motor state monitoring** (ток, напряжение, температура)
- **Дифференциальный привод**

**Стек:** CAN Bus, vesc_nexus  
**Репозиторий:** [github.com/krikz/vesc_nexus](https://github.com/krikz/vesc_nexus)

#### ✅ ros2_control интеграция
- **Hardware Interface** для VESC моторов
- **Controller Manager** для управления контроллерами
- **joint_state_broadcaster** для публикации состояния суставов
- **diff_drive_controller** для дифференциального привода
- **TF публикация** трансформаций колёс

**Стек:** ros2_control, ros2_controllers  
**Документация:** `docs/architecture/SYSTEM_OVERVIEW.md` (раздел 9.4)

#### ✅ Twist Mux
- **Приоритизация команд скорости** из разных источников
- **Источники:**
  1. Navigation (`/cmd_vel_nav`) - низкий приоритет
  2. Teleop (`/cmd_vel_teleop`) - средний приоритет
  3. Safety (`/cmd_vel_safety`) - высокий приоритет
- **Автоматическое переключение** на основе активности источника

**Стек:** twist_mux  
**Документация:** `docs/architecture/SYSTEM_OVERVIEW.md`

---

### 🤖 Взаимодействие с пользователем

#### 🔄 Голосовой ассистент (Voice Assistant)
- **Wake word detection:** робок, робот, роббокс, робокос (6 вариантов)
- **Speech-to-Text (STT):** Vosk offline модель для русского языка
- **Text-to-Speech (TTS):** Silero V4 (качественный русский голос)
- **Dialogue с LLM:** 
  - Основные модели: DeepSeek, Qwen (через API)
  - Локальные модели используются как fallback при недоступности облачных
  - 🔮 **В планах:** собственные сервера для размещения LLM моделей
- **Command execution:** распознавание и выполнение команд навигации
- **Hardware AEC:** подавление эха на ReSpeaker Mic Array
- **Команда "помолчи":** переход в SILENCED режим
- **Time awareness:** робот знает текущее время и дату
- **Синхронизация TTS чанков:** dialogue_id для предотвращения смешивания между сеансами

**Стек:** Vosk, Silero, DeepSeek/Qwen, ReSpeaker  
**Пакет:** `rob_box_voice`  
**Документация:** `docs/architecture/INTERNAL_DIALOGUE_VOICE_ASSISTANT.md`

#### 🔄 Internal Dialogue (Внутренний диалог)
- **Постоянное размышление** без wake word (⚠️ в процессе доработки)
- **Context-aware:** получает события от Context Aggregator
- **Может вмешаться** когда релевантно (даже без wake word)
- **Продолжает работу** даже в SILENCED режиме
- **Reflection mechanism:** анализ событий и принятие решений

**Стек:** DeepSeek/Qwen, Python  
**Пакет:** `rob_box_perception`  
**Документация:** `docs/architecture/INTERNAL_DIALOGUE_VOICE_ASSISTANT.md`

#### ✅ LED Анимации
- **381 LED адресация:** 4× NeoPixel 8×8 (фары) + 1× NeoPixel 25×5 (дисплей)
- **Композитор панелей:** объединение в логические группы
- **Готовые анимации:** радуга, бегущий огонь, пульсация, эмоции
- **ROS 2 интеграция:** управление через топики
- **Эмоциональная индикация:** happy, sad, thinking, listening

**Стек:** WS2812B, SPI, ros2leds  
**Пакет:** `rob_box_animations`, `led_matrix_driver`  
**Репозиторий:** [github.com/krikz/ros2leds](https://github.com/krikz/ros2leds)

#### ✅ Звуковые эффекты
- **Библиотека звуков** для событий (boot, wake, sleep, error)
- **Синхронизация с TTS:** предотвращение наложения
- **Управление через ROS 2 топики**

**Стек:** PulseAudio, Python  
**Пакет:** `rob_box_voice`  
**Документация:** `docs/packages/SOUND_EFFECTS_QUICKSTART.md`

---

### 📊 Мониторинг и диагностика

#### ✅ Grafana дашборды
- **20+ панелей** для визуализации метрик
- **System metrics:** CPU, RAM, Network, Disk
- **Container metrics:** per-container CPU/RAM usage
- **ROS 2 metrics:** topic rates, message sizes
- **Custom metrics:** температура, вес, RPM вентиляторов

**Стек:** Grafana  
**Документация:** `docs/guides/MONITORING_SYSTEM.md`

#### ✅ Централизованные логи
- **Loki** для хранения и индексации логов
- **Promtail** на каждом Pi для отправки логов
- **Grafana Explore** для поиска и анализа
- **Docker container logs** от всех сервисов
- **Retention policy:** 7 дней

**Стек:** Loki, Promtail  
**Документация:** `docs/guides/MONITORING_SYSTEM.md`

#### ✅ Diagnostic scripts
- **monitor_system.sh** - комплексная проверка состояния
- **check_status.sh** - статус Docker контейнеров
- **health check endpoints** в контейнерах
- **ROS 2 topic monitoring** скрипты

**Стек:** Bash, Docker  
**Документация:** `docker/monitor_system.sh`

#### ✅ Zenoh инспекция
- **zenoh-bridge-dds** для подключения RViz через интернет
- **Namespace изоляция** для multi-robot поддержки
- **Cloud router:** zenoh.robbox.online для удалённого доступа
- **Logging и debugging** через Zenoh Explore

**Стек:** Zenoh  
**Документация:** `docs/architecture/ZENOH_CLOUD_NAMESPACES.md`

---

## Планируемые фичи

### 🚀 Навигация следующего уровня

#### 🔮 Semantic mapping
- **Разметка карты:** точки интереса, зоны доставки, no-go зоны
- **Программный комплекс** для визуального редактирования карт
- **Semantic labels:** kitchen, living room, bedroom, charging station
- **Integration с Nav2:** планирование с учётом семантики

**Технологии:** RViz plugins, SQLite/PostgreSQL для хранения меток  
**Зависимости:** RTAB-Map maps, Nav2  
**Приоритет:** 🔴 Высокий

#### 🔮 Dynamic obstacle avoidance
- **Real-time detection** движущихся объектов
- **Prediction** траекторий людей
- **Adaptive replanning** при появлении препятствий
- **Social navigation:** вежливое поведение рядом с людьми

**Технологии:** OAK-D depth, YOLOv8n на AI HAT+ (person detection)  
**Зависимости:** OAK-D, Nav2, AI HAT+ (Vision Pi)  
**Приоритет:** 🟡 Средний

#### 🔮 Multi-floor navigation
- **Elevator detection** и вызов
- **Floor transitions** в планировании
- **3D mapping** для многоуровневых помещений

**Технологии:** RTAB-Map 3D, Floor detection  
**Зависимости:** RTAB-Map  
**Приоритет:** ⚪ Низкий (требуется соответствующее помещение)

---

### 🎯 Продвинутая локализация

#### 🔮 Stereo-camera локализация
- **Visual odometry** от OAK-D Lite stereo камеры
- **Feature tracking** для улучшения одометрии
- **Fusion с wheel odometry** через robot_localization
- **Reduced drift** по сравнению с чисто wheel odometry

**Технологии:** OAK-D stereo, ORB-SLAM3, robot_localization  
**Зависимости:** OAK-D, RTAB-Map  
**Приоритет:** 🔴 Высокий

#### 🔮 Ceiling-based локализация (AprilTags)
- **Raspberry Pi Camera** направлена вверх
- **AprilTag маркеры** на потолке в ключевых точках
- **Absolute positioning** без накопления ошибок
- **Loop closure** для RTAB-Map

**Технологии:** Pi Camera, apriltag_ros  
**Зависимости:** Pi Camera уже установлена  
**Приоритет:** 🔴 Высокий

#### 🔮 UWB локализация (открытые пространства)
- **Ultra-Wideband** позиционирование для больших помещений/улицы
- **Anchors:** стационарные базовые станции (3-4 шт)
- **Tag:** на роботе
- **Точность:** 10-30 см
- **Use case:** склады, паркинги, улица

**Технологии:** Pozyx UWB, Qorvo DWM1001  
**Hardware:** UWB модуль + 3-4 anchors (~$300-500)  
**Приоритет:** 🟡 Средний

#### 🔮 RTK GPS локализация (улица)
- **Real-Time Kinematic GPS** для работы на улице
- **Base station:** стационарная для коррекции
- **Rover:** на роботе
- **Точность:** 1-2 см
- **Use case:** доставка на открытых территориях

**Технологии:** u-blox ZED-F9P, NTRIP  
**Hardware:** RTK модуль + антенна (~$300-400)  
**Приоритет:** ⚪ Низкий (требуется outdoor тестирование)

#### 🔮 Sensor fusion (robot_localization)
- **Fusion всех источников одометрии:**
  - Wheel odometry (VESC)
  - Visual odometry (OAK-D)
  - IMU (ESP32)
  - AprilTag corrections
  - UWB/RTK (если установлены)
- **Extended Kalman Filter (EKF)** для оптимальной оценки позы
- **Reduced drift** и улучшенная точность

**Технологии:** robot_localization (navsat_transform_node, ekf_node)  
**Зависимости:** Все источники одометрии  
**Приоритет:** 🔴 Высокий

---

### 🤖 AI и автономность

#### � AI HAT+ для Vision Pi (26 TOPS — в планировании)
- **Hardware-accelerated AI inference** на Raspberry Pi 5 (Vision Pi — HAT-слот свободен)
- **Чип:** Hailo-8L, 26 TOPS, PCIe 2.0 x1 через GPIO HAT-коннектор
- **Приоритизированные use cases (из анализа):**
  - **[P1]** Person detection (YOLOv8n) — safety stop + триггер диалога робота-гида
  - **[P1]** STT через Whisper HEF — замена Vosk, снижение CPU с ~50% до ~10%
  - **[P2]** Face recognition (retinaface + arcface) — персонализация + авторизация
  - **[P3]** Dynamic obstacle avoidance — публикация moving obstacles в Nav2 costmap
  - **[P4]** Semantic segmentation, gesture recognition — экспериментально
- **Совместимость:**
  - ✅ NeoPixel SPI (GPIO 10) продолжает работать — HAT использует PCIe, не SPI
  - ✅ USB устройства (OAK-D, ReSpeaker) не затронуты
  - ❌ Main Pi — невозможно (HAT-слот занят CAN HAT для VESC)
- **Что НЕ даст HAT+ (важно):** LLM inference — 26 TOPS недостаточно для трансформеров

**Технологии:** Raspberry Pi AI HAT+, Hailo-8L NPU (26 TOPS), HailoRT + TAPPAS  
**Hardware:** AI HAT+ (~$70-100)  
**Зависимости:** Vision Pi, OAK-D camera  
**Документация:** `docs/AI_HAT_UPGRADE_ANALYSIS.md`  
**Приоритет:** 🔴 Высокий

#### 🔄 LLM-powered агент (PR #362 - в активной разработке)
- **Полноценный AI агент** для автономного поведения
- **Perception → Reasoning → Action** цикл
- **Multi-modal reasoning:**
  - Визуальная информация от камеры
  - Пространственная информация от SLAM
  - Текстовые команды от пользователя
  - События от сенсоров
- **Tool use:** агент может вызывать ROS 2 actions/services
- **Memory system:** долговременная и кратковременная память
- **Example workflow:**
  1. Пользователь: "Принеси мне воду из кухни"
  2. LLM планирует: найти кухню → найти воду → принести пользователю
  3. Выполнение: навигация → детекция объекта → захват → возврат
  
**Технологии:** LangChain/LlamaIndex, GPT-4V/Claude/DeepSeek, VectorDB  
**Зависимости:** Voice Assistant, Context Aggregator, Nav2  
**Приоритет:** 🔴 Высокий

#### 🔮 Vision Language Models (VLM)
- **Описание сцены** для агента (что робот видит)
- **Object recognition** через multimodal LLM
- **Visual question answering:** "Что на столе?"
- **Grounding:** связывание текстовых описаний с визуальными объектами
- **Требования к ресурсам:**
  - Модели типа LLaVA-1.5 (7B): ~14GB VRAM
  - Qwen-VL: ~8-16GB VRAM в зависимости от версии
  - Для Raspberry Pi: inference через облачные API (GPT-4V, Claude 3)
  - Альтернатива: квантизованные модели на отдельном GPU-сервере

**Технологии:** GPT-4V/Claude 3 (облачные), LLaVA/Qwen-VL (локальные на GPU-сервере)  
**Зависимости:** OAK-D camera, GPU-сервер (опционально для локальных моделей)  
**Приоритет:** 🟡 Средний

#### 🔮 Behavior Tree планирование
- **Hierarchical planning** для сложных задач
- **Reactive behaviors:** быстрая реакция на события
- **Композируемые behaviors:** переиспользование подзадач
- **Groot visualization:** визуальное редактирование деревьев
- **Железо:** работает на Raspberry Pi (Nav2 уже использует BehaviorTree.CPP)

**Технологии:** BehaviorTree.CPP, Groot  
**Зависимости:** Nav2 (уже использует BT)  
**Приоритет:** 🟡 Средний

---

### 📦 Бизнес-логика доставки

#### 🔮 Клиентское приложение
- **Web/Mobile UI** для заказа доставки
- **QR-код сканирование** для идентификации пользователя
- **Выбор точки доставки** из списка
- **Tracking** положения робота в реальном времени
- **Уведомления** о статусе доставки

**Технологии:** React Native/Flutter, WebSocket для real-time  
**Зависимости:** Nav2, Backend API  
**Приоритет:** 🟡 Средний

#### 🔮 Backend для управления заказами
- **REST API** для клиентского приложения
- **Queue management:** очередь доставок
- **Routing optimization:** оптимальный маршрут для нескольких доставок
- **Database:** заказы, пользователи, история
- **Analytics:** статистика доставок, эффективность

**Технологии:** FastAPI/Django, PostgreSQL, Redis  
**Зависимости:** Nav2  
**Приоритет:** 🟡 Средний

#### 🔮 Док-станция и автоматический докинг (Charging station & Auto-docking)
- **Автоматический возврат** на док-станцию при низком заряде
  - Мониторинг уровня заряда батареи (BMS integration)
  - Автоматическая генерация цели навигации к док-станции
  - Прерывание текущей задачи при критическом заряде
- **Визуальная локализация** док-станции
  - AprilTag маркеры на док-станции для точного обнаружения
  - IR beacons для грубой локализации на большом расстоянии
  - Fusion визуальной и IR локализации для надёжности
- **Точное позиционирование** для подключения контактов
  - Fine-tuned visual servoing для выравнивания (±1-2 см точность)
  - Feedback от контактных сенсоров для подтверждения докинга
  - Retry mechanism при неудачном докинге (3-5 попыток)
- **Процесс зарядки:**
  - Автоматическое определение подключения к зарядке
  - Мониторинг тока и напряжения зарядки
  - Публикация статуса зарядки в ROS 2 топики
  - LED индикация процесса зарядки (анимация на 381 LED)
  - Уведомление через голосовой ассистент о начале/завершении зарядки
- **Undocking (отстыковка):**
  - Проверка полного заряда перед undocking
  - Плавный выезд с док-станции (backward motion)
  - Верификация успешной отстыковки
- **Multi-robot support:**
  - Очередь на зарядку при наличии нескольких роботов
  - Координация через центральный scheduler
  - Приоритизация роботов с низким зарядом

**Технологии:** AprilTag, IR beacons, visual servoing, BMS integration  
**Hardware:** 
- Док-станция с контактами зарядки (~$200-300)
- IR beacons (опционально, ~$30-50)
- Контактные сенсоры для feedback (~$10-20)
**Зависимости:** Nav2, Vision Pi (AprilTag), Battery monitoring (ESP32)  
**Приоритет:** 🟡 Средний (критично для автономной работы)

#### 🔮 Cargo management
- **Погрузка/разгрузка:** механизм открытия/закрытия грузового отсека
- **Weight verification:** проверка что груз взят/доставлен (HX711 тензодатчик уже есть)
- **Cargo LED indicator:** индикация статуса груза на LED дисплее
- **Security:** контроль доступа к грузу (замок, пин-код на LED дисплее)

**Технологии:** Servo/stepper для двери, HX711 (уже установлен)  
**Hardware:** Servo/stepper мотор (~$20-50)  
**Приоритет:** 🟡 Средний

---

### 🏗️ Расширение инфраструктуры

#### 🔮 Simulator интеграция
- **Gazebo Ignition/Isaac Sim** для тестирования
- **Digital twin:** точная модель робота в симуляции
- **Nav2 testing:** отладка навигации без реального робота
- **Scenario testing:** различные сценарии доставки

**Технологии:** Gazebo Ignition, Isaac Sim, URDF  
**Зависимости:** rob_box_description (URDF уже есть)  
**Приоритет:** 🟡 Средний

#### 🔮 Cloud телеметрия
- **Persistent storage** метрик в облаке
- **Historical analysis:** анализ долговременных трендов
- **Alerting:** уведомления о критических событиях
- **Dashboard:** веб-интерфейс для мониторинга флота роботов

**Технологии:** InfluxDB Cloud, Grafana Cloud, AWS/GCP  
**Зависимости:** Monitoring system (уже есть)  
**Приоритет:** ⚪ Низкий

#### ✅ OTA Updates (реализовано через Docker)
- **Over-The-Air обновление** Docker образов (уже работает)
  - Pull свежих образов из registry
  - Restart контейнеров с новыми версиями
  - Используется через `docker compose pull && docker compose up -d`
- 🔮 **Планируется улучшение:**
  - Автоматический rollback при сбое
  - Staged rollout для постепенного обновления флота
  - Version management и контроль версий на каждом роботе
  - Интеграция с Watchtower для автоматических обновлений

**Технологии:** Docker, Docker Compose, Watchtower (планируется)  
**Зависимости:** CI/CD (уже есть)  
**Приоритет:** 🟡 Средний

#### 🔮 Self-Hosted CI/CD Infrastructure
- **Собственные мощности** для CI/CD вместо GitHub Actions
- **Преимущества:**
  - Значительное ускорение сборки (нативный ARM64 без эмуляции)
  - Отсутствие лимитов на минуты сборки
  - Полный контроль над окружением
  - Интеграция с локальным Build Machine
- **Компоненты:**
  - Jenkins/GitLab CI/Drone на выделенном сервере
  - ARM64 build agents для нативной сборки
  - Интеграция с локальным Docker Registry
  - Webhook integration с GitHub
- **Результат:** 2-5x ускорение по сравнению с GitHub Actions на эмуляции

**Технологии:** Jenkins/GitLab CI/Drone, ARM64 hardware  
**Зависимости:** Build Machine (уже есть)  
**Приоритет:** 🟡 Средний

#### 🔮 Multi-robot coordination
- **Fleet management:** управление несколькими роботами
- **Collision avoidance:** координация траекторий
- **Task allocation:** распределение доставок между роботами
- **Charging queue:** очередь на док-станцию

**Технологии:** ROS 2 multi-robot, rmf_core, Zenoh multi-namespace  
**Зависимости:** Zenoh (уже есть), Nav2  
**Приоритет:** ⚪ Низкий

---

## Технологический стек

### Платформа и ОС
| Компонент | Технология | Версия |
|-----------|-----------|--------|
| **Операционная система** | Ubuntu Server | 24.04.2 LTS |
| **ROS Middleware** | ROS 2 | Humble Hawksbill |
| **DDS Middleware** | Zenoh DDS | rmw_zenoh_cpp |
| **Контейнеризация** | Docker + Docker Compose | 24.x |
| **Оркестрация** | Docker Compose | v2.x |

### Навигация и SLAM
| Компонент | Технология | Назначение |
|-----------|-----------|-----------|
| **SLAM** | RTAB-Map | RGB-D + 2D LiDAR картография |
| **Navigation** | Nav2 | Планирование и следование траектории |
| **Localization** | AMCL (в Nav2) | Monte Carlo локализация |
| **Планировщик** | NavFn, Smac Planner | Глобальное планирование пути |
| **Контроллер** | DWB, Regulated Pure Pursuit | Локальное следование |

### Восприятие
| Компонент | Технология | Назначение |
|-----------|-----------|-----------|
| **RGB-D Camera** | OAK-D Lite + DepthAI | Цветное изображение + глубина |
| **2D LiDAR** | LSLIDAR N10 | 360° лазерное сканирование |
| **AprilTag** | apriltag_ros | Визуальные маркеры |
| **Pi Camera** | Raspberry Pi Camera Module | Потолочная локализация |

### Управление и привод
| Компонент | Технология | Назначение |
|-----------|-----------|-----------|
| **Motor Controllers** | VESC + CAN Bus | Управление моторами |
| **Hardware Interface** | vesc_nexus | ROS 2 драйвер для VESC |
| **Control Framework** | ros2_control | Абстракция управления |
| **Velocity Mux** | twist_mux | Приоритизация команд |

### AI и NLP
| Компонент | Технология | Назначение |
|-----------|-----------|-----------|
| **LLM** | DeepSeek | Dialogue и reasoning |
| **STT** | Vosk | Распознавание речи (offline) |
| **TTS** | Silero V4 | Синтез речи (offline) |
| **Wake Word** | Porcupine/Snowboy | Активация по ключевому слову |
| **Context Store** | Custom (Python) | Хранение контекста разговора |

### Визуализация и мониторинг
| Компонент | Технология | Назначение |
|-----------|-----------|-----------|
| **Visualization** | RViz2 | 3D визуализация ROS 2 |
| **Monitoring** | Grafana + Prometheus | Метрики и дашборды |
| **Logging** | Loki + Promtail | Централизованные логи |
| **Container Metrics** | cAdvisor | Метрики Docker контейнеров |

### CI/CD и DevOps
| Компонент | Технология | Назначение |
|-----------|-----------|-----------|
| **CI/CD** | GitHub Actions | Автоматическая сборка и деплой |
| **Container Registry** | GitHub Container Registry | Хранение Docker образов |
| **Local Registry** | Docker Registry | Локальное хранилище образов |
| **APT Cache** | APT Cacher NG | Кэширование пакетов |
| **Code Quality** | pre-commit, hadolint, yamllint | Линтинг и проверка стиля |

### Коммуникация
| Компонент | Технология | Назначение |
|-----------|-----------|-----------|
| **Middleware** | Zenoh DDS | Распределённая связь |
| **Network** | Ethernet (1 Gbps) + WiFi | Dual network design |
| **Cloud Bridge** | Zenoh Router | Облачное подключение |
| **Namespace Isolation** | Zenoh Namespace | Multi-robot support |

---

## Этапы развития

### 🎯 Этап 1: Базовая автономия (В ПРОЦЕССЕ ДОРАБОТКИ 🔄)
**Цель:** Робот может самостоятельно перемещаться в помещении

- [x] Построение карты помещения (RTAB-Map SLAM)
- [x] Локализация на карте
- [x] Планирование траектории (Nav2)
- [x] Избежание статических препятствий
- [x] Управление моторами через VESC
- [x] Распределённая архитектура (2× Raspberry Pi)
- [x] CI/CD pipeline для автоматической сборки
- [x] Система мониторинга (Grafana, Prometheus, Loki)
- [ ] **Оптимизация скорости навигации** - замена колёс на более медленные для улучшения планирования траектории

**Статус:** ⚠️ В процессе оптимизации (Октябрь 2025)

---

### 🎯 Этап 2: Естественное взаимодействие (В РАЗРАБОТКЕ 🔄)
**Цель:** Робот понимает речь и может поддерживать диалог

- [x] Голосовой ассистент с wake word detection
- [x] STT + TTS (offline/cloud hybrid)
- [x] Dialogue с LLM (DeepSeek/Qwen)
- [x] LED индикация и анимации
- [ ] **Internal Dialogue (рефлексия)** - в процессе доработки
- [ ] **Context Aggregator** - в процессе доработки
- [ ] **Semantic mapping** - разметка карты с точками доставки
- [ ] **Голосовые команды навигации** - "Иди на кухню"
- [ ] **LLM-powered агент** для автономного поведения

**Ожидаемое завершение:** Q1 2026

---

### 🎯 Этап 3: Улучшенная локализация (ПЛАНИРУЕТСЯ 🔮)
**Цель:** Робот может надёжно локализоваться в разных условиях

- [ ] **Stereo-camera локализация** (visual odometry от OAK-D)
- [ ] **Ceiling AprilTags** локализация (Pi Camera)
- [ ] **Sensor fusion** (robot_localization EKF)
- [ ] **UWB локализация** для больших помещений (опционально)
- [ ] **RTK GPS** для работы на улице (опционально)

**Ожидаемое завершение:** Q2 2026

---

### 🎯 Этап 4: Бизнес-логика доставки (ПЛАНИРУЕТСЯ 🔮)
**Цель:** Робот может выполнять доставки по заказам

- [ ] **Клиентское приложение** для заказа доставки
- [ ] **Backend для управления заказами**
- [ ] **Док-станция и автоматический докинг:**
  - Визуальная локализация (AprilTag + IR beacons)
  - Точное позиционирование (±1-2 см)
  - Автоматическая зарядка с мониторингом
  - Undocking процесс
- [ ] **Cargo management** - механизм погрузки/разгрузки
- [ ] **Fleet management** - управление несколькими роботами (опционально)

**Ожидаемое завершение:** Q3-Q4 2026

---

### 🎯 Этап 5: AI агент и автономность (ИССЛЕДОВАТЕЛЬСКИЙ 🔬)
**Цель:** Робот самостоятельно планирует и выполняет задачи

- [ ] **VLM интеграция** для понимания сцены
- [ ] **LLM-powered планирование** задач
- [ ] **Memory system** долговременная и кратковременная
- [ ] **Behavior Trees** для иерархического планирования
- [ ] **RL для навигации** (исследовательская задача)

**Ожидаемое завершение:** 2027+

---

## 📚 Дополнительные ресурсы

### Документация
- [README.md](README.md) - Обзор проекта
- [CHANGELOG.md](CHANGELOG.md) - История изменений
- [CONTRIBUTING.md](CONTRIBUTING.md) - Как участвовать
- [docs/](docs/) - Полная техническая документация

### Связанные репозитории
- [vesc_nexus](https://github.com/krikz/vesc_nexus) - ROS 2 драйвер для VESC
- [ros2leds](https://github.com/krikz/ros2leds) - Управление NeoPixel LED
- [robot_sensor_hub](https://github.com/krikz/robot_sensor_hub) - ESP32 с micro-ROS

### Пример похожего проекта (AI агент)
- [PR #362](https://github.com/krikz/rob_box_project/pull/362) - Пример интеграции LLM агента

---

<div align="center">
  <sub>Версия 1.0.0 | Дата: 2026-01-13 | Автор: РОББОКС Team</sub>
  <br>
  <sub>⭐ Поставьте звезду если проект вам интересен!</sub>
</div>
