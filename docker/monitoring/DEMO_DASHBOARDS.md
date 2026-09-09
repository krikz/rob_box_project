# Демонстрационные дашборды для стенда

Четыре специализированных дашборда для демонстрационного стенда с четырьмя мониторами.

## 🎯 Обзор дашбордов

### Dashboard 1: Системный обзор и ошибки
**Назначение:** Главный монитор с общей информацией о системе  
**Файл:** `demo_1_system_overview.json`  
**UID:** `rob_box_demo_1`

**Содержимое:**
- ✅ Мониторы нагрузки CPU/Memory для обоих Pi (4 gauge)
- ✅ Графики нагрузки CPU по контейнерам (Main Pi и Vision Pi)
- ✅ Вывод всех ошибок из всех контейнеров в едином окне

**Рекомендации по отображению:**
- Основной/центральный монитор
- Разрешение: 1920x1080 или выше
- Refresh rate: 10 секунд

---

### Dashboard 2: Навигация и движение
**Назначение:** Мониторинг нод навигации и моторов  
**Файл:** `demo_2_navigation.json`  
**UID:** `rob_box_demo_2`

**Содержимое по нодам:**
- **RTAB-Map** — SLAM система, картографирование
- **Nav2 Navigation Stack** — планировщик траекторий, контроллер движения
  - bt_navigator, controller_server, planner_server, smoother_server
  - waypoint_follower, lifecycle_manager, velocity_smoother, behavior_server
- **ROS2 Control** — управление моторами через VESC
  - ros2_control_node, diff_drive_controller, joint_state_broadcaster
- **Twist Mux** — мультиплексирование команд скорости с приоритизацией
- **LSLIDAR** — 2D лидар для сканирования окружения
- **Robot State Publisher** — публикация TF трансформаций

**Контейнеры:** rtabmap, nav2, ros2-control, twist-mux, lslidar, robot-state-publisher

---

### Dashboard 3: Восприятие и сенсоры
**Назначение:** Мониторинг нод восприятия и датчиков  
**Файл:** `demo_3_perception.json`  
**UID:** `rob_box_demo_3`

**Содержимое по нодам:**
- **OAK-D Camera** — стерео RGB-D камера с интегрированным AprilTag детектором
  - oak, oakd, apriltag_node
- **Ceiling Camera** — USB камера для обзора сверху
  - usb_cam, ceiling_camera
- **micro-ROS Agent** — связь с ESP32 Sensor Hub (IMU, ToF, IR)
- **Perception System** — система внутреннего восприятия
  - health_monitor — мониторинг состояния системы
  - context_aggregator — агрегация контекста (MPC lite)
  - reflection_node — внутренний диалог робота
  - vision_stub — обработка визуальных данных
  - startup_greeting — приветствие при загрузке

**Контейнеры:** oak-d, ceiling-camera, perception

---

### Dashboard 4: Голос и интерфейс
**Назначение:** Мониторинг голосового ассистента и LED интерфейса  
**Файл:** `demo_4_voice.json`  
**UID:** `rob_box_demo_4`

**Содержимое по нодам:**
- **Voice Assistant** — голосовой ассистент
  - audio_node — обработка аудио с ReSpeaker
  - led_node — управление LED индикаторами на ReSpeaker
  - dialogue_node — диалоговая система (DeepSeek)
  - tts_node — синтез речи (Silero TTS v4)
  - stt_node — распознавание речи (Vosk)
  - sound_node — звуковые эффекты
  - command_node — распознавание команд для навигации
- **LED Matrix** — LED дисплей (NeoPixel)
  - led_matrix_compositor
- **Zenoh Router** — роутеры на обоих Pi для DDS коммуникации

**Контейнеры:** voice-assistant, led-matrix, zenoh-router, zenoh-router-vision

---

## 🚀 Быстрый старт

### 1. Включение мониторинга

На обоих Pi должен быть запущен профиль мониторинга:

```bash
# Main Pi
cd ~/rob_box_project/docker/main
./scripts/enable_monitoring.sh

# Vision Pi
cd ~/rob_box_project/docker/vision
./scripts/enable_monitoring.sh
```

### 2. Доступ к дашбордам

Откройте Grafana в браузере: `http://10.1.1.10:3000`
- **Логин:** admin
- **Пароль:** robbox

В меню "Dashboards" вы увидите 5 дашбордов:
- **Rob Box Dashboard** — оригинальный полный дашборд
- **1. Системный обзор и ошибки** — Dashboard 1
- **2. Навигация и движение** — Dashboard 2
- **3. Восприятие и сенсоры** — Dashboard 3
- **4. Голос и интерфейс** — Dashboard 4

### 3. Настройка мониторов

Рекомендуемое расположение для 4 мониторов:

```
┌─────────────────┬─────────────────┐
│   Dashboard 1   │   Dashboard 2   │
│     Обзор       │    Навигация    │
│  (Главный)      │                 │
├─────────────────┼─────────────────┤
│   Dashboard 3   │   Dashboard 4   │
│   Восприятие    │      Голос      │
│                 │                 │
└─────────────────┴─────────────────┘
```

**Опции отображения:**
- Используйте полноэкранный режим: `d` → `f` или кнопка "View" → "Fullscreen"
- Настройте циклическую смену дашбордов: "Dashboard settings" → "Make editable" → "Playlist"
- Используйте TV Mode для автоматического скрытия меню: добавьте `?kiosk` в URL

**Примеры URL для киоск-режима:**
```
http://10.1.1.10:3000/d/rob_box_demo_1?kiosk
http://10.1.1.10:3000/d/rob_box_demo_2?kiosk
http://10.1.1.10:3000/d/rob_box_demo_3?kiosk
http://10.1.1.10:3000/d/rob_box_demo_4?kiosk
```

---

## 📊 Описание панелей

### Gauge панели (Dashboard 1)
- **Зеленый:** < 60% загрузки (нормально)
- **Желтый:** 60-80% загрузки (предупреждение)
- **Красный:** > 80% загрузки (критично)

### Timeseries графики (Dashboard 1)
- Показывают CPU usage в реальном времени для каждого контейнера
- Легенда справа с метриками: mean, last, max
- Tooltip при наведении показывает детали

### Logs панели (Dashboards 2-4)
- Показывают логи с автообновлением каждые 10 секунд
- Временное окно: последние 15 минут
- Сортировка: от новых к старым
- Фильтруются по контейнерам через Loki queries

---

## 🔍 LogQL запросы

Дашборды используют Loki для фильтрации логов. Вот примеры запросов:

### Dashboard 1 (Все ошибки):
```logql
{job="docker"} |~ "(?i)(error|exception|fatal|critical|failed)"
```

### Dashboard 2 (Навигация):
```logql
{container="rtabmap"}
{container="nav2"}
{container="ros2-control"}
{container="twist-mux"}
{container="lslidar"}
{container="robot-state-publisher"}
```

### Dashboard 3 (Восприятие):
```logql
{container="oak-d"}
{container="ceiling-camera"}
{container="perception"}
```

### Dashboard 4 (Голос):
```logql
{container="voice-assistant"}
{container="led-matrix"}
{container="zenoh-router"}
{container="zenoh-router-vision"}
```

---

## 🛠️ Кастомизация

### Изменение refresh rate

По умолчанию все дашборды обновляются каждые 10 секунд. Для изменения:

1. Откройте дашборд
2. Нажмите на иконку ⚙️ (Dashboard settings)
3. General → Auto refresh → выберите нужный интервал (5s, 10s, 30s, 1m)

### Изменение временного окна

По умолчанию показываются последние 15 минут. Для изменения:

1. В правом верхнем углу выберите временной диапазон
2. Примеры: "Last 5 minutes", "Last 30 minutes", "Last 1 hour"

### Фильтрация логов

Для фильтрации логов по конкретной ноде добавьте фильтр в LogQL:

```logql
# Только логи определенной ноды
{container="voice-assistant"} |~ "audio_node"

# Исключить информационные сообщения
{container="rtabmap"} != "info"

# Только ошибки
{container="nav2"} |~ "(?i)error"
```

---

## 📝 Генерация дашбордов

Дашборды были сгенерированы автоматически скриптом `/tmp/generate_dashboards.py`.

Для регенерации или модификации:

1. Отредактируйте скрипт
2. Запустите: `python3 /tmp/generate_dashboards.py`
3. Перезапустите Grafana: `docker restart grafana`

---

## 🔗 Связанная документация

- [Система мониторинга](../../docs/guides/MONITORING_SYSTEM.md) — полная документация
- [Краткий справочник](../../docs/MONITORING_QUICK_REF.md) — команды и настройка
- [AGENT_GUIDE.md](../../docs/development/AGENT_GUIDE.md) — архитектура системы

---

## 🐛 Устранение неполадок

### Дашборды не появляются

```bash
# Проверить, что Grafana запущен
docker ps | grep grafana

# Проверить логи Grafana
docker logs grafana

# Перезапустить Grafana
cd ~/rob_box_project/docker/monitoring
docker-compose restart grafana
```

### Нет данных в панелях

```bash
# Проверить, что Prometheus собирает метрики
curl http://10.1.1.10:9090/api/v1/targets

# Проверить, что Loki получает логи
curl http://10.1.1.10:3100/ready

# Проверить, что Promtail на Vision Pi отправляет логи
docker exec promtail-vision wget -qO- http://10.1.1.10:3100/ready
```

### Ошибки в логах не отображаются

```bash
# Проверить, что контейнеры имеют метку logging: "promtail"
docker inspect <container> | grep -A 5 '"Labels"'

# Проверить Promtail конфигурацию
cat ~/rob_box_project/docker/main/config/monitoring/promtail-config.yaml
```

---

**Последнее обновление:** 2025-11-01  
**Версия:** 1.0  
**Автор:** Rob Box Team
