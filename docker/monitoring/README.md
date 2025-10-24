# Мониторинг Rob Box - Отдельная машина мониторинга

Этот каталог содержит Docker Compose конфигурацию для запуска стека мониторинга Rob Box на отдельной машине в локальной сети WiFi.

## Зачем отдельная машина?

Вынос мониторинга на отдельную машину имеет следующие преимущества:

1. **Экономия ресурсов робота** - освобождает ~250-450 MB RAM на Main Pi
2. **Независимая работа** - мониторинг работает даже если робот выключен
3. **Лучшая производительность** - не конкурирует за ресурсы с ROS нодами
4. **Удобный доступ** - постоянный IP адрес для Grafana

## Архитектура

```
┌──────────────────────┐         ┌──────────────────────┐
│   Vision Pi          │         │      Main Pi         │
│   (10.1.1.11)        │         │   (10.1.1.10)        │
│                      │         │                      │
│  ┌────────────────┐  │         │  ┌────────────────┐  │
│  │   cAdvisor     │──┼────┐    │  │   cAdvisor     │──┼────┐
│  │   :8080        │  │    │    │  │   :8080        │  │    │
│  └────────────────┘  │    │    │  └────────────────┘  │    │
│                      │    │    │                      │    │
│  ┌────────────────┐  │    │    │  ┌────────────────┐  │    │
│  │   Promtail     │──┼────┼────┼──│   Promtail     │──┼────┤
│  │   (логи)       │  │    │    │  │   (логи)       │  │    │
│  └────────────────┘  │    │    │  └────────────────┘  │    │
└──────────────────────┘    │    └──────────────────────┘    │
                            │                                 │
                            │ Метрики (Prometheus)            │
                            │ Логи (Loki)                     │
                            │                                 │
                            ▼                                 ▼
                    ┌──────────────────────────────────────────┐
                    │  Monitoring Machine (WiFi)               │
                    │  (например: 10.1.1.50)                   │
                    │                                          │
                    │  ┌────────────┐  ┌──────────────┐       │
                    │  │ Prometheus │  │     Loki     │       │
                    │  │   :9090    │  │    :3100     │       │
                    │  └─────┬──────┘  └──────┬───────┘       │
                    │        │                │               │
                    │        └────────┬───────┘               │
                    │                 ▼                       │
                    │          ┌──────────────┐              │
                    │          │   Grafana    │◄──── :3000   │
                    │          └──────────────┘              │
                    │                                         │
                    └─────────────────────────────────────────┘
```

## Требования

- Linux-машина с Docker и Docker Compose
- Подключение к той же WiFi сети что и робот
- Минимум 1GB свободной RAM
- Минимум 10GB свободного места на диске

## Быстрый старт

### 1. Настройка конфигурации

Скопируйте пример конфигурации:

```bash
cd ~/rob_box_project/docker/monitoring
cp .env.example .env
```

Отредактируйте `.env` файл:

```bash
# IP addresses of the robot's Raspberry Pi computers
MAIN_PI_IP=10.1.1.10
VISION_PI_IP=10.1.1.11

# Grafana admin password (change in production!)
GRAFANA_PASSWORD=your_secure_password

# Robot ID for filtering metrics and logs (optional)
ROBOT_ID=rob_box_01
```

### 2. Запуск стека мониторинга

```bash
./scripts/start_monitoring.sh
```

Вывод:
```
================================================
  Запуск стека мониторинга Rob Box
================================================

Конфигурация:
  Main Pi:    10.1.1.10
  Vision Pi:  10.1.1.11
  Robot ID:   rob_box_01

Запуск контейнеров мониторинга...
[+] Running 3/3
 ✔ Container loki         Started
 ✔ Container prometheus   Started
 ✔ Container grafana      Started

================================================
  Мониторинг успешно запущен!
================================================

Доступ к сервисам:
  • Grafana:    http://localhost:3000 (admin/your_secure_password)
  • Prometheus: http://localhost:9090
  • Loki:       http://localhost:3100
```

### 3. Настройка Raspberry Pi

На **обоих Raspberry Pi** установите переменную окружения LOKI_HOST:

```bash
# Main Pi и Vision Pi
cd ~/rob_box_project/docker/main  # или docker/vision
echo "LOKI_HOST=10.1.1.50" >> .env  # замените на IP вашей машины мониторинга
```

Запустите агенты мониторинга:

```bash
# Main Pi
cd ~/rob_box_project/docker/main
./scripts/enable_monitoring.sh

# Vision Pi
cd ~/rob_box_project/docker/vision
./scripts/enable_monitoring.sh
```

### 4. Доступ к Grafana

Откройте в браузере:
```
http://<IP-адрес-машины-мониторинга>:3000
```

Логин: `admin`  
Пароль: тот что указали в `.env`

**Dashboard "Rob Box - Мониторинг робота"** автоматически установлен и настроен как домашняя страница. Он включает:

- **Обзор робота** - общая загрузка CPU и памяти на обеих Pi
- **CPU по контейнерам** - детальная загрузка каждого контейнера
- **Память по контейнерам** - использование RAM по контейнерам
- **Сеть** - входящий и исходящий трафик
- **Логи контейнеров** - просмотр логов с фильтрацией
- **Специализированные панели** для RTAB-Map, OAK-D, Voice Assistant
- **Панель ошибок** - все ошибки из всех контейнеров

## Использование

### Просмотр логов

1. В Grafana перейдите в `Explore` (иконка компаса)
2. Выберите источник данных `Loki`
3. Примеры запросов:

```logql
# Логи конкретного контейнера
{container="rtabmap"}

# Логи с Vision Pi
{host="vision-pi"}

# Ошибки в контейнере
{container="oak-d"} |= "error"
```

### Просмотр метрик

1. В Grafana перейдите в `Explore`
2. Выберите источник данных `Prometheus`
3. Примеры запросов:

```promql
# CPU usage по контейнерам
rate(container_cpu_usage_seconds_total[5m]) * 100

# Memory usage (MB)
container_memory_usage_bytes / 1024 / 1024

# Network throughput
rate(container_network_receive_bytes_total[5m]) / 1024 / 1024
```

## Управление

### Остановка мониторинга

```bash
./scripts/stop_monitoring.sh
```

### Проверка статуса

```bash
docker-compose ps
```

### Просмотр логов контейнеров

```bash
docker-compose logs -f grafana
docker-compose logs -f prometheus
docker-compose logs -f loki
```

## Хранение данных

Данные хранятся в Docker volumes:

- `prometheus-data` - метрики (7 дней)
- `loki-data` - логи (7 дней)
- `grafana-data` - дашборды и настройки (неограниченно)

### Очистка данных

Для освобождения места:

```bash
# Остановка и удаление volumes
docker-compose down -v

# Или удаление конкретного volume
docker volume rm monitoring_prometheus-data
```

## Потребление ресурсов

| Сервис     | RAM (idle) | RAM (active) | CPU (avg) | Диск      |
|------------|------------|--------------|-----------|-----------|
| Prometheus | ~100 MB    | ~200 MB      | 2-3%      | ~500MB/7d |
| Loki       | ~50 MB     | ~100 MB      | 1-2%      | ~200MB/7d |
| Grafana    | ~100 MB    | ~150 MB      | 1-2%      | ~50MB     |
| **ИТОГО**  | **~250 MB**| **~450 MB**  | **4-7%**  | **~750MB**|

## Диагностика

### Проблема: Grafana не видит данные с Raspberry Pi

**Решение:**

1. Проверьте что агенты запущены на обеих Pi:
```bash
# На Main Pi
docker ps | grep -E "(cadvisor|promtail)"

# На Vision Pi
docker ps | grep -E "(cadvisor|promtail)"
```

2. Проверьте доступность Loki с Pi:
```bash
# На Main Pi или Vision Pi
curl http://<monitoring-machine-ip>:3100/ready
```

3. Проверьте что LOKI_HOST правильно настроен в `.env` на обеих Pi

### Проблема: Prometheus не собирает метрики с Pi

**Решение:**

1. Проверьте доступность cAdvisor:
```bash
curl http://10.1.1.10:8080/metrics | head
curl http://10.1.1.11:8080/metrics | head
```

2. Проверьте targets в Prometheus:
```
http://<monitoring-machine-ip>:9090/targets
```

Все targets должны быть в состоянии UP.

## Безопасность

⚠️ **Важно:**

1. **Измените пароль Grafana** в `.env` перед запуском
2. Рекомендуется использовать **firewall** для ограничения доступа к портам:
   - 3000 (Grafana) - доступен только из локальной сети
   - 9090 (Prometheus) - закрыт извне
   - 3100 (Loki) - закрыт извне, открыт для Pi

3. Пример настройки firewall (UFW):
```bash
sudo ufw allow from 10.1.1.0/24 to any port 3000
sudo ufw allow from 10.1.1.10 to any port 3100
sudo ufw allow from 10.1.1.11 to any port 3100
```

## См. также

- [Полное руководство по мониторингу](../../docs/guides/MONITORING_SYSTEM.md)
- [Быстрая справка](../../docs/MONITORING_QUICK_REF.md)
- [Гайд по развёртыванию](../../docs/deployment/MONITORING_DEPLOYMENT.md)

## Поддержка

При возникновении проблем:
1. Проверьте логи контейнеров: `docker-compose logs`
2. Проверьте сетевую связность между машинами
3. Создайте issue в GitHub с описанием проблемы
