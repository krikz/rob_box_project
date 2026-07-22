# 🏗️ Архитектура Rob Box

Документация архитектуры робототехнической платформы.

## 📄 Документы

### [SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md)
**Общая архитектура системы**

- Концепция распределённой системы (Main Pi + Vision Pi)
- Архитектура связи (Zenoh Bridge)
- Компоненты системы
- Схемы взаимодействия
- Топология сети

### [HARDWARE.md](HARDWARE.md)
**Аппаратные компоненты**

- Main Pi (Raspberry Pi 5, 16GB)
- Vision Pi (Raspberry Pi 5, 8GB)
- Сенсоры (OAK-D Lite, LSLIDAR N10, ReSpeaker)
- Актуаторы (VESC, моторы)
- Периферия (LED матрицы, ESP32)
- Схемы подключения
- Питание и распределение энергии

### [SOFTWARE.md](SOFTWARE.md)
**Программные компоненты**

- ROS 2 Humble пакеты
- Навигация (Nav2, RTABMap)
- Восприятие (OAK-D pipeline, AprilTag)
- Управление (Twist Mux, VESC контроллер)
- AI системы (Voice Assistant, DeepSeek)
- Docker архитектура
- Zenoh middleware

### [ICP_ODOMETRY.md](ICP_ODOMETRY.md)
**ICP Одометрия и Wheel Odometry Fusion**

- Что такое ICP (Iterative Closest Point)
- Архитектура fusion: wheel odometry + ICP
- Роли узлов: ros2-control, icp_odometry, rtabmap
- TF дерево и потоки данных
- Параметры ICP алгоритма

### [NETWORK_TOPOLOGY.md](NETWORK_TOPOLOGY.md)
**Сетевая топология**

- IP-адресация: Main Pi (10.1.1.10/20), Vision Pi (10.1.1.11/21)
- Dual Network: Ethernet (данные) + WiFi (управление)
- Zenoh топология: peer/client/cloud modes
- Порты сервисов и SSH доступ

### [ZENOH_CLOUD_NAMESPACES.md](ZENOH_CLOUD_NAMESPACES.md)
**Zenoh: Облачное подключение и Namespaces**

- Что такое Zenoh namespace (отличия от ROS namespace)
- Текущая реализация (ROBOT_ID, wrapper script, Docker)
- Топология облачной сети
- Детали конфигурации
- Тестирование и валидация
- Устранение неполадок
- Соображения безопасности (TLS/mTLS)

### [minimax-provider.md](minimax-provider.md)
**MiniMax LLM-провайдер (верхний уровень)**

- Решение о подключении MiniMax как opt-in адаптера LLM (ADR-0002, Accepted)
- Архитектурный разбор PR #907, as-is → target
- Маппинг возможностей, registry/fallback/secrets
- Фазный rollout M0–M6

### [minimax-tts-architecture.md](minimax-tts-architecture.md)
**MiniMax TTS-провайдер: архитектура интеграции**

- TTS-контракт `BaseTTSProvider` / `MiniMaxTTSProvider`
- Конфигурация через ROS-параметры и ENV
- Цепочка: MiniMax API → `TTSAudio` → `tts_node` → `/voice/audio/speech`
- Таблица маппинга `TTSSettings` → T2A v2 body
- См. также [ros2-audio-contract-spec.md](ros2-audio-contract-spec.md) (dataflow, QoS, ответственность за конверсию/ресэмплинг)

### [minimax-tts-integration-design.md](minimax-tts-integration-design.md)
**MiniMax TTS: design-контракт интеграции (ADR-0004)**

- Port/Adapter (`BaseTTSProvider`), `TTSProviderRegistry` + `TTSProviderFactory`
- Retry-policy, circuit breaker, opt-in streaming
- AudioStamped / `speech_meta` для streaming с метаданными
- Trade-off матрица

### [ros2-audio-contract-spec.md](ros2-audio-contract-spec.md)
**ROS 2 Audio Contract — MiniMax TTS → Speaker** (спецификация, Proposed)

- Frozen PCM-контракт v1: `int16 LE` mono 16 кГц, `audio_common_msgs/AudioData`
- Карта топиков и QoS-профилей
- **Таблица статичных vs варьирующихся от голоса параметров** (явно)
- **Таблица ответственности за конверсию/ресэмплинг** (6 сценариев несовпадения формата)
- Mermaid dataflow-диаграмма потока от API MiniMax до динамика
- Sink-архитектура (direct sounddevice / sound_play / audio_play_node)
- Требования к латентности (TTFA, jitter, CPU)
- Сопутствующая Mermaid: [`../diagrams/minimax-tts-ros2-dataflow.mmd`](../diagrams/minimax-tts-ros2-dataflow.mmd)

## 🔗 Связанные документы

- [Документация пакетов](../packages/)
- [Руководства по настройке](../guides/)
- [CI/CD Pipeline](../CI_CD_PIPELINE.md)
- [Отчет об анализе Zenoh](../reports/ZENOH_NAMESPACE_ANALYSIS_2025-10-23.md)

---

**Навигация:** [← Назад в docs/](../README.md)
