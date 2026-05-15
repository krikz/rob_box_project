# Мониторинг робота - Краткая справка

> **Архитектура мониторинга**: Prometheus, Loki и Grafana развёртываются на **отдельной машине** (не на Main Pi и не на Vision Pi). Конфигурация: `docker/monitoring/docker-compose.yaml`. Агенты `cAdvisor` и `Promtail` запущены на каждом Pi и отправляют метрики на мониторинговую машину. Все команды `docker compose` ниже выполняются на **мониторинговой машине**, если не указано иное.

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
http://<monitoring-machine-ip>:3000
Логин: admin
Пароль: robbox
```

> **Примечание**: IP-адрес мониторинговой машины зависит от конфигурации вашей сети. По умолчанию — машина разработчика в том же VLAN что и Pi.

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
http://<monitoring-ip>:3000/d/rob_box_demo_1?kiosk
http://<monitoring-ip>:3000/d/rob_box_demo_2?kiosk
http://<monitoring-ip>:3000/d/rob_box_demo_3?kiosk
http://<monitoring-ip>:3000/d/rob_box_demo_4?kiosk
```

📖 **Документация:** [DEMO_DASHBOARDS.md](../docker/monitoring/DEMO_DASHBOARDS.md)

## 🔗 Доступ к сервисам

> **Выполняется на мониторинговой машине** (не на Pi). Замените `<monitoring-ip>` на IP мониторинговой машины.

| Сервис     | URL                    | Описание              |
|------------|------------------------|-----------------------|
| Grafana    | http://\<monitoring-ip\>:3000  | Дашборды и визуализация |
| Prometheus | http://\<monitoring-ip\>:9090  | Метрики               |
| cAdvisor   | http://10.1.1.10:8080  | Main Pi метрики (агент на Main Pi) |
| cAdvisor   | http://10.1.1.11:8080  | Vision Pi метрики (агент на Vision Pi) |

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
