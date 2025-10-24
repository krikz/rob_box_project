# Система мониторинга Rob Box

## Обзор

Легковесная система мониторинга для робота Rob Box, позволяющая отслеживать состояние контейнеров, логи и метрики производительности с обоих Raspberry Pi.

### Компоненты стека

**На Main Pi (10.1.1.10):**
- **cAdvisor** (порт 8080) - мониторинг контейнеров
- **Prometheus** (порт 9090) - сбор и хранение метрик
- **Loki** (порт 3100) - агрегация логов
- **Promtail** (порт 9080) - сбор локальных логов
- **Grafana** (порт 3000) - визуализация и дашборды

**На Vision Pi (10.1.1.11):**
- **cAdvisor** (порт 8080) - мониторинг контейнеров
- **Promtail** (порт 9080) - сбор логов и отправка на Main Pi

### Архитектура

```
┌──────────────────────────────────────────────────────────────┐
│ Vision Pi (10.1.1.11)                                         │
│                                                               │
│  ┌─────────────┐         ┌──────────────┐                    │
│  │  cAdvisor   │─────────│  Promtail    │                    │
│  │  :8080      │         │  :9080       │                    │
│  └─────────────┘         └──────┬───────┘                    │
│        │                         │                            │
└────────┼─────────────────────────┼────────────────────────────┘
         │                         │
         │ Метрики                 │ Логи
         │ (Prometheus             │ (Loki API)
         │  scrape)                │
         │                         │
┌────────┼─────────────────────────┼────────────────────────────┐
│        ▼                         ▼                            │
│  ┌─────────────┐         ┌──────────────┐                    │
│  │ Prometheus  │         │     Loki     │                    │
│  │    :9090    │         │    :3100     │                    │
│  └──────┬──────┘         └──────┬───────┘                    │
│         │                       │                             │
│         └───────────┬───────────┘                             │
│                     ▼                                         │
│              ┌──────────────┐                                 │
│              │   Grafana    │◄──── Веб-интерфейс             │
│              │    :3000     │                                 │
│              └──────────────┘                                 │
│                                                               │
│  Main Pi (10.1.1.10)                                          │
└───────────────────────────────────────────────────────────────┘
```

## Быстрый старт

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

Откройте в браузере: `http://<main-pi-ip>:3000`
- **Логин:** admin
- **Пароль:** robbox

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

## Что мониторится

### Метрики контейнеров (cAdvisor)

- **CPU:** Использование процессора по контейнерам
- **Память:** RAM, swap, cache
- **Сеть:** RX/TX throughput, пакеты, ошибки
- **Диск:** I/O операции, throughput
- **Процессы:** Количество процессов/треды

### Логи (Promtail + Loki)

Автоматический сбор логов из:
- Всех Docker контейнеров с меткой `logging: "promtail"`
- Системных логов `/var/log/*.log`

Собираемые контейнеры:
- zenoh-router, rtabmap, oak-d
- apriltag, lslidar, voice-assistant
- led-matrix, perception, nav2
- ros2-control, twist-mux, micro-ros-agent
- robot-state-publisher

## Использование Grafana

### Просмотр логов

1. Откройте Grafana: `http://10.1.1.10:3000`
2. Перейдите в `Explore` (иконка компаса)
3. Выберите источник данных `Loki`
4. Запросы для фильтрации:

```logql
# Все логи с Vision Pi
{host="vision-pi"}

# Логи конкретного контейнера
{container="oak-d"}

# Логи по сервису из docker-compose
{service="rtabmap"}

# Ошибки в логах
{container="rtabmap"} |= "error"

# Последние 100 строк
{container="voice-assistant"} | line_format "{{.line}}"
```

### Просмотр метрик

1. В Grafana перейдите в `Explore`
2. Выберите источник данных `Prometheus`
3. Примеры запросов:

```promql
# CPU usage по контейнерам
rate(container_cpu_usage_seconds_total{container_label_logging="promtail"}[5m]) * 100

# Использование памяти (MB)
container_memory_usage_bytes{container_label_logging="promtail"} / 1024 / 1024

# Network RX throughput (MB/s)
rate(container_network_receive_bytes_total[5m]) / 1024 / 1024

# Network TX throughput (MB/s)
rate(container_network_transmit_bytes_total[5m]) / 1024 / 1024
```

### Создание дашбордов

1. В Grafana: `Dashboards` → `New` → `New Dashboard`
2. Добавьте панели с интересующими метриками
3. Сохраните дашборд

**Рекомендуемые дашборды:**
- Docker Container & Host Metrics (ID: 179)
- cAdvisor exporter (ID: 14282)
- Loki & Promtail (ID: 13639)

Импорт: `Dashboards` → `Import` → введите ID

## Хранение данных

### Retention (время хранения)

- **Prometheus:** 7 дней
- **Loki:** 7 дней
- **Grafana:** неограниченно (дашборды, настройки)

### Volumes (персистентность)

```bash
# Список volumes
docker volume ls | grep -E "(prometheus|loki|grafana)"

# Посмотреть использование места
docker volume inspect main_prometheus-data
docker volume inspect main_loki-data
docker volume inspect main_grafana-data
```

### Полная очистка данных

```bash
cd ~/rob_box_project/docker/main
./scripts/disable_monitoring.sh
docker volume rm main_prometheus-data main_loki-data main_grafana-data
```

## Потребление ресурсов

### Main Pi (полный стек)

| Сервис      | RAM (idle) | RAM (active) | CPU (avg) |
|-------------|------------|--------------|-----------|
| cAdvisor    | ~50 MB     | ~80 MB       | 1-2%      |
| Prometheus  | ~100 MB    | ~200 MB      | 2-3%      |
| Loki        | ~50 MB     | ~100 MB      | 1-2%      |
| Promtail    | ~20 MB     | ~40 MB       | 0.5-1%    |
| Grafana     | ~100 MB    | ~150 MB      | 1-2%      |
| **ИТОГО**   | **~320 MB**| **~570 MB**  | **5-10%** |

### Vision Pi (легкий стек)

| Сервис      | RAM (idle) | RAM (active) | CPU (avg) |
|-------------|------------|--------------|-----------|
| cAdvisor    | ~50 MB     | ~80 MB       | 1-2%      |
| Promtail    | ~20 MB     | ~40 MB       | 0.5-1%    |
| **ИТОГО**   | **~70 MB** | **~120 MB**  | **1.5-3%**|

### Сетевой трафик

- **Vision Pi → Main Pi (Promtail):** ~50-200 KB/s
- **Main Pi → Prometheus (scraping):** ~10-50 KB/s

## Диагностика

### Проверка статуса

```bash
# Main Pi
docker ps | grep -E "(cadvisor|prometheus|loki|promtail|grafana)"

# Vision Pi
docker ps | grep -E "(cadvisor|promtail)"
```

### Проверка логов

```bash
# Main Pi
docker logs grafana
docker logs prometheus
docker logs loki
docker logs promtail

# Vision Pi
docker logs cadvisor-vision
docker logs promtail-vision
```

### Проверка подключения Vision Pi → Main Pi

```bash
# На Vision Pi
docker exec promtail-vision wget -qO- http://10.1.1.10:3100/ready

# Должен вернуть: ready
```

### Проверка сбора метрик

```bash
# cAdvisor на Main Pi
curl http://localhost:8080/metrics | grep container_cpu

# cAdvisor на Vision Pi (с Main Pi)
curl http://10.1.1.11:8080/metrics | grep container_cpu

# Prometheus targets
curl http://localhost:9090/api/v1/targets | jq '.data.activeTargets[] | {job, health}'
```

### Troubleshooting

**Проблема:** Grafana не показывает метрики с Vision Pi

**Решение:**
```bash
# Проверьте что Prometheus scraping работает
curl http://localhost:9090/api/v1/targets

# Проверьте доступность cAdvisor на Vision Pi
curl http://10.1.1.11:8080/metrics

# Проверьте файрвол (если настроен)
sudo iptables -L -n | grep 8080
```

**Проблема:** Promtail не отправляет логи с Vision Pi

**Решение:**
```bash
# На Vision Pi проверьте что Loki доступен
wget -qO- http://10.1.1.10:3100/ready

# Проверьте конфигурацию Promtail
docker exec promtail-vision cat /etc/promtail/config.yml

# Посмотрите логи Promtail
docker logs promtail-vision
```

**Проблема:** Высокое потребление CPU/RAM

**Решение:**
```bash
# Увеличьте интервал сбора метрик в prometheus.yml
scrape_interval: 30s  # было 15s

# Увеличьте housekeeping interval cAdvisor
--housekeeping_interval=60s  # было 30s

# Перезапустите
./scripts/disable_monitoring.sh
./scripts/enable_monitoring.sh
```

## Интеграция с существующими скриптами

Мониторинг совместим с существующими скриптами диагностики:

```bash
# Системный мониторинг (расширенный с метриками из Grafana)
cd ~/rob_box_project/docker
./monitor_system.sh

# Диагностика Vision Pi
cd ~/rob_box_project/docker/vision
./diagnose.sh

# Real-time мониторинг
cd ~/rob_box_project/docker/vision
./realtime_monitor.sh
```

## Лучшие практики

### Включайте мониторинг когда:

- ✅ Отлаживаете проблемы производительности
- ✅ Проводите нагрузочные тесты
- ✅ Настраиваете новые функции
- ✅ Удаленно работаете с роботом

### Выключайте мониторинг когда:

- ✅ Автономная работа робота (экономия ресурсов)
- ✅ Максимальная производительность критична
- ✅ Батарея разряжена

### Оптимизация

```yaml
# docker/main/config/monitoring/prometheus.yml
global:
  scrape_interval: 30s  # Увеличить если нужна экономия CPU
  evaluation_interval: 30s

# docker/main/config/monitoring/loki-config.yaml
limits_config:
  retention_period: 72h  # Уменьшить до 3 дней если мало места
```

## Дополнительные ресурсы

- [Prometheus Documentation](https://prometheus.io/docs/)
- [Grafana Documentation](https://grafana.com/docs/)
- [Loki Documentation](https://grafana.com/docs/loki/)
- [cAdvisor GitHub](https://github.com/google/cadvisor)

## Changelog

**2025-10-24:** Первая версия системы мониторинга
- Добавлены cAdvisor, Prometheus, Loki, Promtail, Grafana
- Создана документация и скрипты управления
- Интеграция с docker-compose через профили
