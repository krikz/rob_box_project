# Мониторинг робота - Краткая справка

## 🚀 Быстрый старт

### Включение мониторинга

**Main Pi:**
```bash
cd ~/rob_box_project/docker/main
./scripts/enable_monitoring.sh
```

**Vision Pi:**
```bash
cd ~/rob_box_project/docker/vision
./scripts/enable_monitoring.sh
```

### Доступ к Grafana

```
http://<main-pi-ip>:3000
Логин: admin
Пароль: robbox
```

### Выключение мониторинга

**Main Pi:**
```bash
cd ~/rob_box_project/docker/main
./scripts/disable_monitoring.sh
```

**Vision Pi:**
```bash
cd ~/rob_box_project/docker/vision
./scripts/disable_monitoring.sh
```

## 📊 Что мониторится

- **CPU, память, сеть** - метрики контейнеров через cAdvisor
- **Логи** - все контейнеры с меткой `logging: "promtail"`
- **Системные логи** - `/var/log/*.log` на обоих Pi

## 🎯 Демонстрационные дашборды

Доступны **4 специализированных дашборда** для демо-стенда с 4 мониторами:

1. **Системный обзор и ошибки** - мониторы CPU/Memory, графики нагрузки, все ошибки
2. **Навигация и движение** - RTAB-Map, Nav2, ROS2 Control, LiDAR (~18 нод)
3. **Восприятие и сенсоры** - OAK-D, камеры, micro-ROS, perception (~10 нод)
4. **Голос и интерфейс** - voice-assistant, LED-matrix, Zenoh (~10 нод)

**Киоск-режим URL:**
```
http://10.1.1.10:3000/d/rob_box_demo_1?kiosk
http://10.1.1.10:3000/d/rob_box_demo_2?kiosk
http://10.1.1.10:3000/d/rob_box_demo_3?kiosk
http://10.1.1.10:3000/d/rob_box_demo_4?kiosk
```

📖 **Документация:** [DEMO_DASHBOARDS.md](../docker/monitoring/DEMO_DASHBOARDS.md)

## 🔗 Доступ к сервисам

| Сервис     | URL                    | Описание              |
|------------|------------------------|-----------------------|
| Grafana    | http://10.1.1.10:3000  | Дашборды и визуализация |
| Prometheus | http://10.1.1.10:9090  | Метрики               |
| cAdvisor   | http://10.1.1.10:8080  | Main Pi метрики       |
| cAdvisor   | http://10.1.1.11:8080  | Vision Pi метрики     |

## 📝 Просмотр логов в Grafana

1. Откройте Grafana: `http://10.1.1.10:3000`
2. `Explore` → выберите `Loki`
3. Примеры запросов:

```logql
# Логи конкретного контейнера
{container="oak-d"}

# Логи с Vision Pi
{host="vision-pi"}

# Ошибки в RTAB-Map
{container="rtabmap"} |= "error"
```

## 💾 Потребление ресурсов

**Main Pi:** ~320MB RAM (idle), ~570MB RAM (active)
**Vision Pi:** ~70MB RAM (idle), ~120MB RAM (active)

## 🔧 Диагностика

### Проверка статуса контейнеров

```bash
# Main Pi
docker ps | grep -E "(cadvisor|prometheus|loki|promtail|grafana)"

# Vision Pi
docker ps | grep -E "(cadvisor|promtail)"
```

### Проверка подключения Vision Pi → Main Pi

```bash
# На Vision Pi
docker exec promtail-vision wget -qO- http://10.1.1.10:3100/ready
# Должно вернуть: ready
```

## 📖 Полная документация

См. [MONITORING_SYSTEM.md](guides/MONITORING_SYSTEM.md)
