# Мониторинг Rob Box - Инструкция по развёртыванию

## 📦 Что было добавлено

### Файлы конфигурации

**Main Pi:**
```
docker/main/config/monitoring/
├── prometheus.yml              # Сбор метрик
├── loki-config.yaml            # Агрегация логов
├── promtail-config.yaml        # Пересылка логов
├── grafana-datasources.yaml    # Источники данных Grafana
└── README.md                   # Описание конфигов
```

**Vision Pi:**
```
docker/vision/config/monitoring/
└── promtail-config.yaml        # Пересылка логов на Main Pi
```

### Скрипты управления

**Main Pi:**
```bash
docker/main/scripts/enable_monitoring.sh    # Включить мониторинг
docker/main/scripts/disable_monitoring.sh   # Выключить мониторинг
```

**Vision Pi:**
```bash
docker/vision/scripts/enable_monitoring.sh   # Включить мониторинг
docker/vision/scripts/disable_monitoring.sh  # Выключить мониторинг
```

### Docker Compose изменения

**Добавлены сервисы:**
- `cadvisor` - метрики контейнеров (обе Pi)
- `prometheus` - сбор метрик (Main Pi)
- `loki` - агрегация логов (Main Pi)
- `promtail` - пересылка логов (обе Pi)
- `grafana` - веб-интерфейс (Main Pi)

**Профили Docker Compose:**
Все сервисы мониторинга используют профиль `monitoring`, что позволяет легко включать/выключать:
```yaml
profiles: ["monitoring"]
```

**Метки для логирования:**
Все основные сервисы получили метку для сбора логов:
```yaml
labels:
  logging: "promtail"
```

## 🚀 Первое развёртывание

### Шаг 1: Обновить код на обеих Pi

**Main Pi:**
```bash
cd ~/rob_box_project
git pull origin <branch-name>  # например: main, develop, или feature branch
```

**Vision Pi:**
```bash
cd ~/rob_box_project
git pull origin <branch-name>  # например: main, develop, или feature branch
```

### Шаг 2: Включить мониторинг

**Main Pi:**
```bash
cd ~/rob_box_project/docker/main
./scripts/enable_monitoring.sh
```

Вывод:
```
================================================
  Включение мониторинга на Main Pi
================================================

✓ Конфигурационные файлы найдены

Запуск сервисов мониторинга...
[+] Running 5/5
 ✔ Container cadvisor     Started
 ✔ Container prometheus   Started
 ✔ Container loki         Started
 ✔ Container promtail     Started
 ✔ Container grafana      Started

================================================
  Мониторинг успешно запущен!
================================================

Доступ к сервисам:
  • Grafana:    http://localhost:3000 (admin/robbox)
  • Prometheus: http://localhost:9090
  • cAdvisor:   http://localhost:8080
  • Loki:       http://localhost:3100
```

**Vision Pi:**
```bash
cd ~/rob_box_project/docker/vision
./scripts/enable_monitoring.sh
```

### Шаг 3: Проверить статус

**Main Pi:**
```bash
docker ps | grep -E "(cadvisor|prometheus|loki|promtail|grafana)"
```

Должно показать 5 запущенных контейнеров.

**Vision Pi:**
```bash
docker ps | grep -E "(cadvisor|promtail)"
```

Должно показать 2 запущенных контейнера.

### Шаг 4: Открыть Grafana

Откройте в браузере:
```
http://10.1.1.10:3000
```

Логин: `admin`  
Пароль: `robbox`

## 📊 Проверка работоспособности

### 1. Проверить метрики в Prometheus

```bash
curl http://10.1.1.10:9090/api/v1/targets | jq '.data.activeTargets[] | {job, health}'
```

Ожидаемый вывод:
```json
{
  "job": "cadvisor-main",
  "health": "up"
}
{
  "job": "cadvisor-vision",
  "health": "up"
}
```

### 2. Проверить логи в Loki

В Grafana:
1. Перейти в `Explore` (иконка компаса слева)
2. Выбрать источник данных `Loki`
3. Ввести запрос: `{container="oak-d"}`
4. Нажать `Run query`

Должны появиться логи с контейнера oak-d.

### 3. Проверить связь Vision Pi → Main Pi

На Vision Pi:
```bash
# Замените 10.1.1.10 на IP адрес вашего Main Pi (указан в docker/main/config/zenoh_router_config.json5)
docker exec promtail-vision wget -qO- http://10.1.1.10:3100/ready
```

Должно вернуть: `ready`

## 🎯 Типичные задачи мониторинга

### Просмотр логов конкретного контейнера

В Grafana Explore (Loki):
```logql
{container="rtabmap"}
```

### Поиск ошибок

```logql
{container="rtabmap"} |= "error"
```

### Логи с Vision Pi

```logql
{host="vision-pi"}
```

### CPU usage контейнера

В Grafana Explore (Prometheus):
```promql
rate(container_cpu_usage_seconds_total{container_label_logging="promtail"}[5m]) * 100
```

### Memory usage (MB)

```promql
container_memory_usage_bytes{container_label_logging="promtail"} / 1024 / 1024
```

### Network throughput (MB/s)

```promql
rate(container_network_receive_bytes_total[5m]) / 1024 / 1024
```

## 🛑 Выключение мониторинга

### При автономной работе робота

Для экономии ресурсов можно отключить мониторинг:

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

**ВАЖНО:** Данные мониторинга (метрики, логи) сохраняются в volumes и не удаляются при выключении.

### Полное удаление данных

Если нужно освободить место на диске:

```bash
cd ~/rob_box_project/docker/main
./scripts/disable_monitoring.sh
docker volume rm main_prometheus-data main_loki-data main_grafana-data
```

## 📈 Потребление ресурсов

### Main Pi (полный стек)

| Сервис     | RAM (idle) | RAM (active) | CPU (avg) | Сеть         |
|------------|------------|--------------|-----------|--------------|
| cAdvisor   | ~50 MB     | ~80 MB       | 1-2%      | -            |
| Prometheus | ~100 MB    | ~200 MB      | 2-3%      | 10-50 KB/s   |
| Loki       | ~50 MB     | ~100 MB      | 1-2%      | -            |
| Promtail   | ~20 MB     | ~40 MB       | 0.5-1%    | 50-200 KB/s  |
| Grafana    | ~100 MB    | ~150 MB      | 1-2%      | -            |
| **ИТОГО**  | **~320 MB**| **~570 MB**  | **5-10%** | **60-250 KB/s** |

### Vision Pi (легкий стек)

| Сервис     | RAM (idle) | RAM (active) | CPU (avg) | Сеть         |
|------------|------------|--------------|-----------|--------------|
| cAdvisor   | ~50 MB     | ~80 MB       | 1-2%      | -            |
| Promtail   | ~20 MB     | ~40 MB       | 0.5-1%    | 50-200 KB/s  |
| **ИТОГО**  | **~70 MB** | **~120 MB**  | **1.5-3%**| **50-200 KB/s** |

## ⚠️ Важные замечания

1. **Firewall:** Если на Pi настроен firewall, откройте порты:
   - 3000 (Grafana)
   - 8080 (cAdvisor)
   - 9090 (Prometheus)
   - 3100 (Loki)

2. **Retention:** Логи и метрики хранятся 7 дней. Для изменения редактируйте:
   - `docker/main/config/monitoring/loki-config.yaml` (retention_period)
   - `docker/main/config/monitoring/prometheus.yml` (storage.tsdb.retention.time)

3. **Безопасность:** Пароль Grafana по умолчанию `robbox`. Смените после первого входа:
   - Grafana → Configuration → Users → admin → Change Password

4. **Производительность:** При высокой нагрузке увеличьте интервалы:
   - `scrape_interval: 30s` в prometheus.yml (было 15s)
   - `--housekeeping_interval=60s` для cAdvisor (было 30s)

## 📚 Дополнительная документация

- **Полное руководство:** [docs/guides/MONITORING_SYSTEM.md](../guides/MONITORING_SYSTEM.md)
- **Быстрая справка:** [docs/MONITORING_QUICK_REF.md](../MONITORING_QUICK_REF.md)
- **Конфигурация:** [docker/main/config/monitoring/README.md](../../docker/main/config/monitoring/README.md)

## 🆘 Поддержка

При проблемах:
1. Проверьте логи: `docker logs <container-name>`
2. Проверьте статус: `docker ps -a`
3. См. раздел Troubleshooting в [MONITORING_SYSTEM.md](../../docs/guides/MONITORING_SYSTEM.md)
4. Создайте issue в GitHub с описанием проблемы

---

**Дата:** 24 октября 2025  
**Версия:** 1.0  
**Автор:** Rob Box Project Team
