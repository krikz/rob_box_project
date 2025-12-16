# GitHub Copilot - Rob Box Project Navigator

## 🎯 Проект
**Rob Box** — автономный ровер на ROS 2 Humble + Zenoh DDS  
Dual Raspberry Pi 4: Main (10.1.1.10) + Vision (10.1.1.11)

## 📚 Где искать информацию

### 🔴 КРИТИЧНО - Прочитать ПЕРЕД изменениями

| Тема | Файл | Что внутри |
|------|------|-----------|
| **AI Agents Guide** | `docs/development/AGENT_GUIDE.md` | Архитектура Docker, Zenoh, примеры workflow, deployment |
| **Docker Rules** | `docs/development/DOCKER_STANDARDS.md` | ❌ COPY config/scripts, ✅ volumes, network_mode: host |
| **Python Style** | `docs/development/PYTHON_STYLE_GUIDE.md` | black, isort, flake8, ROS 2 patterns, naming |

### 🏗️ Архитектура

| Компонент | Документация |
|-----------|--------------|
| Системная архитектура | `docs/architecture/SYSTEM_OVERVIEW.md` |
| Железо (Pi, сенсоры, моторы) | `docs/architecture/HARDWARE.md` |
| Софт (ROS 2, Docker, Zenoh) | `docs/architecture/SOFTWARE.md` |
| Сетевая топология | `docs/architecture/NETWORK_TOPOLOGY.md` |

### 🔧 Разработка

| Задача | Документация |
|--------|--------------|
| Docker сборка | `docs/development/BUILD_OPTIMIZATION.md` |
| CI/CD Pipeline | `docs/CI_CD_PIPELINE.md` |
| Тестирование | `docs/development/TESTING_GUIDE.md` |
| Линтинг | `docs/development/LINTING_GUIDE.md` |
| Деплой | `docs/DEPLOYMENT_WORKFLOW.md` |

### 🐛 Отладка

| Проблема | Решение |
|----------|---------|
| Общие проблемы | `docs/guides/TROUBLESHOOTING.md` |
| Камера не публикует данные | `docs/guides/CAMERA_TROUBLESHOOTING.md` |
| Zenoh connection issues | `docs/fixes/ZENOH_FIX_SUMMARY_2025-11-10.md` |
| Мониторинг системы | `docs/MONITORING_QUICK_REF.md` |

### 📦 Пакеты ROS 2

| Пакет | Назначение | Документация |
|-------|-----------|--------------|
| `rob_box_voice` | Voice assistant (STT, TTS, dialogue) | `docs/packages/rob_box_voice/` |
| `rob_box_perception` | Health monitor, context aggregator | `docs/packages/rob_box_perception/` |
| `rob_box_animations` | LED matrix animations (381 LEDs) | `docs/packages/rob_box_animations/` |
| `rob_box_description` | URDF robot model | `docs/packages/rob_box_description/` |

### 🌐 Сеть и middleware

**IP-адреса:**
- Main Pi: `10.1.1.10` (eth0), `10.1.1.20` (wlan0)
- Vision Pi: `10.1.1.11` (eth0), `10.1.1.21` (wlan0)

**Zenoh конфигурация:**
```yaml
RMW_IMPLEMENTATION=rmw_zenoh_cpp
ZENOH_CONFIG=/config/zenoh_session_config.json5
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

### 🐳 Docker структура

```
docker/
├── base/          # Базовые образы: ros2-zenoh, rtabmap, depthai, pcl
├── main/          # Main Pi сервисы (rtabmap, nav2, control, lslidar, perception)
│   ├── config/    # ✅ Конфиги (монтируются volumes, НЕ COPY!)
│   ├── scripts/   # ✅ Скрипты (монтируются volumes, НЕ COPY!)
│   └── <service>/ # Только Dockerfile
└── vision/        # Vision Pi сервисы (oak-d, voice, led-matrix, apriltag)
    ├── config/    # ✅ Конфиги (volumes)
    ├── scripts/   # ✅ Скрипты (volumes)
    └── <service>/ # Только Dockerfile
```

## ⚡ Быстрые команды

### SSH доступ
```bash
# Vision Pi
sshpass -p 'open' ssh ros2@10.1.1.21

# Main Pi  
sshpass -p 'open' ssh ros2@10.1.1.20
```

### Обновление на Pi
```bash
# Vision Pi
sshpass -p 'open' ssh ros2@10.1.1.21 \
  'cd ~/rob_box_project/docker/vision && ./scripts/update_and_restart.sh'
```

### Диагностика
```bash
docker ps                  # Статус контейнеров
docker logs <name> -f      # Логи в реальном времени
ros2 topic list            # ROS 2 топики
ros2 topic hz /scan        # Частота публикации
```

## 🚨 Критичные правила

### 🤖 Деплой на роботов
- ❌ **НИКОГДА** самостоятельно не копировать файлы на робота (scp, rsync)
- ❌ **НИКОГДА** не редактировать файлы напрямую на роботе
- ❌ **НИКОГДА** не делать `git pull` на роботе без запроса пользователя
- ✅ **ВСЕГДА** деплой через GitHub Actions workflow (`docs/DEPLOYMENT_WORKFLOW.md`)
- ✅ **ВСЕГДА** репозитории на роботах должны быть чистыми (`git status` = clean)
- ✅ **ТОЛЬКО** по явной просьбе пользователя выполнять команды на роботе
- ⚠️ Изменения делаем в dev-репозитории → commit → push → workflow деплоит на роботов

### Docker
- ❌ **НИКОГДА** `COPY config/` в Dockerfile
- ❌ **НИКОГДА** `COPY scripts/` в Dockerfile  
- ✅ **ВСЕГДА** `network_mode: host`
- ✅ **ВСЕГДА** `depends_on: zenoh-router`
- ✅ **ВСЕГДА** volumes: `./config:/config:ro`

### Python
- ✅ `black` (line-length 120)
- ✅ `isort` (profile black)
- ✅ `self.get_logger().info()` НЕ `print()`
- ✅ Type hints для public API
- ✅ Google-style docstrings

### Git commits
```
feat(voice): add command node
fix(docker): add missing dependency
docs(readme): update hardware specs
```

## 📖 Расширенная документация

Для подробностей используй `@docs/development/<файл>.md` в чате:
- `@docs/development/AGENT_GUIDE.md` - полный гайд
- `@docs/architecture/SYSTEM_OVERVIEW.md` - детальная архитектура
- `@docs/CI_CD_PIPELINE.md` - GitHub Actions workflows

---
**Обновлено:** 19 ноября 2025  
**Размер:** ~150 строк (было 823)  
**Подход:** Навигация → детали в отдельных файлах
