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

### Аппаратные требования

- Linux-машина (Ubuntu 20.04+ или Debian 11+ рекомендуется)
- Подключение к той же WiFi сети что и робот
- Минимум 1GB свободной RAM
- Минимум 10GB свободного места на диске
- Сетевая доступность к Raspberry Pi по IP адресам

### Программные требования

**Docker и Docker Compose:**

Для Ubuntu/Debian:

```bash
# Установка Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Установка Docker Compose (для Ubuntu 20.04/22.04)
sudo apt update
sudo apt install docker-compose -y

# Добавление текущего пользователя в группу docker
sudo usermod -aG docker $USER

# Применение изменений групп (или перелогиньтесь)
newgrp docker

# Проверка установки
docker --version
docker-compose --version
```

**Минимальные версии:**
- Docker: 20.10+
- Docker Compose: 1.29+ (или Docker Compose v2)

**Проверка доступности Docker:**

```bash
# Проверка что Docker daemon запущен
docker info

# Если ошибка "permission denied", выполните:
sudo systemctl start docker
sudo systemctl enable docker
```

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

## Устранение неполадок при развёртывании

### Ошибка: "docker-compose: command not found"

**Причина:** Docker Compose не установлен.

**Решение:**
```bash
sudo apt update
sudo apt install docker-compose -y
```

Для современных систем с Docker 20.10+ можно использовать встроенный Docker Compose v2:
```bash
docker compose version
# Если работает, можно создать alias
echo 'alias docker-compose="docker compose"' >> ~/.bashrc
source ~/.bashrc
```

### Ошибка: "urllib3.exceptions.URLSchemeUnknown: Not supported URL scheme http+docker"

**Причина:** Пользователь не имеет доступа к Docker socket (`/var/run/docker.sock`).

**Решение:**
```bash
# Добавить пользователя в группу docker
sudo usermod -aG docker $USER

# Применить изменения
newgrp docker

# Или перелогиньтесь в систему
```

Проверка:
```bash
groups | grep docker  # Должна быть группа docker
docker ps             # Должно работать без sudo
```

### Ошибка: "error mounting ... read-only file system" (Grafana)

**Причина:** Некоторые системы блокируют bind-mount отдельных файлов.

**Решение:** Уже исправлено в текущей версии - используется монтирование директории вместо отдельных файлов.

Если проблема сохраняется:
```bash
# Проверьте права на config/grafana
ls -la config/grafana
chmod -R 755 config/grafana
```

### Ошибка: "mkdir /tmp/loki/rules: permission denied" (Loki)

**Причина:** Loki запускается от UID 10001, и не может создать директорию с правами root.

**Решение:** Уже исправлено в текущей версии - используется init-контейнер для создания директорий с правильными правами.

Конфигурация в `docker-compose.yaml`:
```yaml
services:
  loki-init:
    image: busybox:latest
    volumes:
      - loki-data:/tmp/loki
    command: sh -c "mkdir -p /tmp/loki/rules /tmp/loki/chunks && chmod -R 777 /tmp/loki"
  
  loki:
    depends_on:
      loki-init:
        condition: service_completed_successfully
    # ... остальная конфигурация
```

Init-контейнер автоматически создаёт необходимые директории с правильными правами перед запуском Loki.

### Ошибка: "Cannot connect to the Docker daemon"

**Причина:** Docker daemon не запущен.

**Решение:**
```bash
# Запустить Docker daemon
sudo systemctl start docker

# Включить автозапуск
sudo systemctl enable docker

# Проверить статус
sudo systemctl status docker
```

### Ошибка: Контейнеры запускаются, но сразу останавливаются

**Причина:** Ошибка в конфигурации или порты уже заняты.

**Решение:**
```bash
# Проверить логи контейнеров
docker-compose logs loki
docker-compose logs prometheus
docker-compose logs grafana

# Проверить что порты свободны
sudo netstat -tulpn | grep -E ':(3000|3100|9090)'

# Если порты заняты, остановите конфликтующие службы
# или измените порты в docker-compose.yaml
```

### Проблема: Низкая производительность или высокая нагрузка

**Причина:** Недостаточно ресурсов или слишком частый scraping.

**Решение:**

1. Увеличьте интервал scraping в `config/prometheus.yml`:
```yaml
scrape_configs:
  - job_name: 'cadvisor-main'
    scrape_interval: 30s  # Вместо 15s
```

2. Уменьшите retention period:
```bash
# В docker-compose.yaml для Prometheus
command:
  - '--storage.tsdb.retention.time=3d'  # Вместо 7d
```

3. Ограничьте память для контейнеров:
```yaml
services:
  prometheus:
    deploy:
      resources:
        limits:
          memory: 512M
```

### Проблема: Нет подключения к Raspberry Pi

**Причина:** Сетевая недоступность или firewall.

**Решение:**

1. Проверьте сетевую доступность:
```bash
ping 10.1.1.10
ping 10.1.1.11
```

2. Проверьте что порты открыты на Pi:
```bash
# На машине мониторинга
telnet 10.1.1.10 8080
telnet 10.1.1.11 8080
```

3. Проверьте firewall на Pi:
```bash
# На Raspberry Pi
sudo ufw status
# Если активен, добавьте правила:
sudo ufw allow from 10.1.1.0/24 to any port 8080
sudo ufw allow from 10.1.1.0/24 to any port 9080
```

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
