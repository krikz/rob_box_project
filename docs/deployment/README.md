# 🚀 Развёртывание Rob Box

Инструкции по развёртыванию системы на роботе.

## 📄 Документы

### [VOICE_ASSISTANT_DOCKER.md](VOICE_ASSISTANT_DOCKER.md)
**Voice Assistant в Docker**

- Структура Docker образа
- Зависимости и модели
- Конфигурация контейнера
- Развёртывание на Vision Pi
- Troubleshooting

### [VISION_PI_DEPLOYMENT.md](VISION_PI_DEPLOYMENT.md)
**Развёртывание Vision Pi**

- Docker Compose конфигурация
- Сервисы Vision Pi
- Zenoh Router настройка
- Сенсоры (OAK-D, ReSpeaker)
- LED матрица и анимации

**Примечание:** LSLIDAR перемещён на Main Pi (24 октября 2025)

### [MONITORING_DEPLOYMENT.md](MONITORING_DEPLOYMENT.md)
**Система мониторинга** 🆕

- Развёртывание на отдельной машине
- Grafana + Prometheus + Loki
- Агенты на обоих Raspberry Pi
- Красивые дашборды
- Быстрый старт

## 🔗 Связанные документы

- [Docker стандарты](../development/DOCKER_STANDARDS.md)
- [Локальная сборка](../development/LOCAL_BUILD.md)
- [CI/CD Pipeline](../CI_CD_PIPELINE.md)
- [Troubleshooting](../guides/TROUBLESHOOTING.md)

---

**Навигация:** [← Назад в docs/](../README.md)
