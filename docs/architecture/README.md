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

- Main Pi (Raspberry Pi 5, 8GB)
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
