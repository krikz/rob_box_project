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

- ROS 2 kilted пакеты
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

## 🔗 Связанные документы

- [Документация пакетов](../packages/)
- [Руководства по настройке](../guides/)
- [CI/CD Pipeline](../CI_CD_PIPELINE.md)
- [Отчет об анализе Zenoh](../reports/ZENOH_NAMESPACE_ANALYSIS_2025-10-23.md)

---

**Навигация:** [← Назад в docs/](../README.md)
