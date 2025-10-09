# Руководство для AI агентов - Rob Box Project

## 📋 Оглавление
- [Обзор системы](#обзор-системы)
- [Доступ к Raspberry Pi](#доступ-к-raspberry-pi)
- [Инструментарий мониторинга и диагностики](#инструментарий-мониторинга-и-диагностики)
- [Анализ последних изменений](#анализ-последних-изменений)
- [Текущая архитектура Zenoh](#текущая-архитектура-zenoh)
- [Скрипты обновления и управления](#скрипты-обновления-и-управления)

---

## Обзор системы

**Rob Box Project** - автономный робот на базе двух Raspberry Pi 4 с OAK-D Lite камерой и RTAB-Map SLAM.

### Компоненты системы

**Vision Pi (10.1.1.11 eth0 / 10.1.1.21 wlan0)**
- OAK-D Lite камера (Intel Movidius MyriadX)
- AprilTag детекция
- Zenoh router для маршрутизации данных

**Main Pi (10.1.1.10 eth0 / 10.1.1.20 wlan0)**
- RTAB-Map SLAM система
- Zenoh router (центральный узел)
- Подключение к внешнему серверу zenoh.robbox.online:7447

### Сетевая топология

```
Vision Pi (eth0: 10.1.1.11)  ←→  Main Pi (eth0: 10.1.1.10)
     ↓                                    ↓
WiFi (wlan0: 10.1.1.21)         WiFi (wlan0: 10.1.1.20)
   (управление)                        (управление)
                                         ↓
                               zenoh.robbox.online:7447
```

**⚠️ ВАЖНО**: 
- **Gigabit Ethernet (eth0)**: ТОЛЬКО для передачи данных между Pi
- **WiFi (wlan0)**: ТОЛЬКО для SSH доступа и управления
- В конфигурациях Zenoh router используются ТОЛЬКО Ethernet IP (10.1.1.10, 10.1.1.11)

---

## Доступ к Raspberry Pi

### Credentials

| Параметр | Vision Pi | Main Pi |
|----------|-----------|---------|
| **Ethernet IP** | 10.1.1.11 | 10.1.1.10 |
| **WiFi IP** | 10.1.1.21 | 10.1.1.20 |
| **Username** | ros2 | ros2 |
| **Password** | open | open |

### SSH подключение

```bash
# Из Windows (PowerShell) с использованием sshpass через WSL
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.21  # Vision Pi
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.20  # Main Pi

# Прямое SSH подключение (если требуется интерактивный доступ)
ssh ros2@10.1.1.21  # Vision Pi
ssh ros2@10.1.1.20  # Main Pi
```

### Выполнение команд на удаленных Pi

```bash
# Vision Pi - проверка контейнеров
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.21 'docker ps'

# Main Pi - просмотр логов
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.20 'docker logs rtabmap'

# Обновление кода и перезапуск
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.21 \
  'cd ~/rob_box_project/docker/vision && git pull && docker-compose down && docker-compose up -d'
```

---

## Инструментарий мониторинга и диагностики

### 📊 Скрипты мониторинга

#### 1. `docker/monitor_system.sh` - Общий системный мониторинг
**Местоположение**: `docker/monitor_system.sh`  
**Назначение**: Универсальный мониторинг CPU, памяти, сети, температуры для обоих Pi  
**Использование**:
```bash
# На Vision Pi
ssh ros2@10.1.1.21
cd ~/rob_box_project/docker
./monitor_system.sh
```

**Что показывает**:
- Тип Pi (определяется автоматически по контейнерам)
- CPU нагрузка (общая система + контейнер)
- Память (общая + контейнер)
- Температура процессора
- Throttling состояние (пониженное напряжение, частота и т.д.)
- Сеть (RX/TX на eth0 и wlan0)
- Топ процессов
- Статус Docker контейнеров

**Рекомендации**:
- ✅ **ОСТАВИТЬ** - универсальный инструмент для быстрой диагностики
- Запускать при подозрении на проблемы с производительностью

---

#### 2. `docker/vision/realtime_monitor.sh` - Real-time мониторинг камеры
**Местоположение**: `docker/vision/realtime_monitor.sh`  
**Назначение**: Детальный мониторинг работы OAK-D камеры в реальном времени  
**Использование**:
```bash
# На Vision Pi
cd ~/rob_box_project/docker/vision
./realtime_monitor.sh [duration_in_seconds]  # по умолчанию 30 секунд
```

**Что показывает** (каждые 2 секунды):
- CPU (usr/sys/idle)
- Сеть eth0 (RX/TX KB/s)
- RGB publisher count
- Depth publisher count
- Температура
- Throttling состояние

**Рекомендации**:
- ✅ **ОСТАВИТЬ** - ключевой инструмент для отладки камеры
- Использовать для проверки стабильности публикации топиков
- Запускать после изменения конфигурации камеры

---

#### 3. `docker/vision/monitor_camera_startup.sh` - Мониторинг запуска камеры
**Местоположение**: `docker/vision/monitor_camera_startup.sh`  
**Назначение**: Комплексный мониторинг запуска камеры с сохранением логов  
**Использование**:
```bash
# На Vision Pi
cd ~/rob_box_project/docker/vision
./monitor_camera_startup.sh
```

**Что делает**:
- Перезапускает камеру
- Фоновый мониторинг CPU (60 секунд, каждые 2 секунды)
- Фоновый мониторинг питания/throttling
- Фоновый мониторинг сети eth0
- Логирование ROS 2 топиков
- Сохранение всех логов в `/tmp/camera_monitor_YYYYMMDD_HHMMSS/`

**Рекомендации**:
- ⚠️ **УСТАРЕЛ** - функционал дублируется `realtime_monitor.sh`
- 🗑️ **УДАЛИТЬ** или **ОБЪЕДИНИТЬ** с `realtime_monitor.sh`
- Сохранение логов полезно, но можно реализовать опцией в realtime_monitor

---

### 🔍 Скрипты диагностики

#### 4. `docker/diagnose_data_flow.sh` - Диагностика связи между Pi
**Местоположение**: `docker/diagnose_data_flow.sh`  
**Назначение**: Проверка всего пути передачи данных: Vision Pi → Main Pi  
**Использование**:
```bash
# На локальной машине (Windows PowerShell)
cd d:\PROJECTS\rob_box_project\docker
wsl bash ./diagnose_data_flow.sh
```

**Что проверяет**:
1. Полный перезапуск камеры через docker-compose
2. Publisher count на Vision Pi
3. Сетевой трафик eth0 на Vision Pi
4. Subscriber count на Main Pi
5. Итоговая диагностика

**Рекомендации**:
- ✅ **ОСТАВИТЬ** - критичный инструмент для диагностики межплатной связи
- Запускать при проблеме "Did not receive data since 5 seconds"
- Автоматизирует весь процесс диагностики с использованием sshpass

---

#### 5. `docker/vision/diagnose.sh` - Диагностика камеры на Vision Pi
**Местоположение**: `docker/vision/diagnose.sh`  
**Назначение**: Быстрая проверка статуса камеры и топиков  
**Использование**:
```bash
# На Vision Pi
cd ~/rob_box_project/docker/vision
./diagnose.sh
```

**Что проверяет**:
1. Статус контейнера oak-d
2. Логи камеры (ERROR/WARN)
3. Список ROS 2 топиков от камеры
4. Информация о топиках (compressed)
5. Публикация топиков (5 секунд)
6. Нагрузка системы (CPU/память)

**Рекомендации**:
- ✅ **ОСТАВИТЬ** - быстрая диагностика на Vision Pi
- Использовать для первичной проверки камеры

---

#### 6. `docker/vision/force_publish.sh` - Принудительная публикация топиков
**Местоположение**: `docker/vision/force_publish.sh`  
**Назначение**: Отключение lazy publisher для OAK-D  
**Использование**:
```bash
# На Vision Pi
cd ~/rob_box_project/docker/vision
./force_publish.sh
```

**Что делает**:
- Отключает `color.i_enable_lazy_publisher` 
- Отключает `depth.i_enable_lazy_publisher`
- Проверяет publisher count

**Рекомендации**:
- ⚠️ **УСТАРЕЛ** - `update_and_restart.sh` уже включает эту логику
- 🗑️ **УДАЛИТЬ** - функционал дублируется, можно оставить как справочник

---

### 🔧 Скрипты настройки

#### 7. `docker/tune.sh` - Настройка сетевых буферов (CycloneDDS)
**Местоположение**: `docker/tune.sh`  
**Назначение**: Увеличение сетевых буферов для CycloneDDS  
**Использование**:
```bash
# На любом Pi
cd ~/rob_box_project/docker
sudo bash tune.sh
```

**Рекомендации**:
- ⚠️ **УСТАРЕЛ** - система перешла на Zenoh, CycloneDDS больше не используется
- 🗑️ **УДАЛИТЬ** - оставить только как исторический артефакт в документации

---

### 🚀 Скрипты обновления и управления

#### 8. `docker/vision/update_and_restart.sh` - Обновление и перезапуск Vision Pi
**Местоположение**: `docker/vision/update_and_restart.sh`  
**Назначение**: Автоматизированное обновление кода и перезапуск контейнеров  
**Использование**:
```bash
# На Vision Pi
cd ~/rob_box_project/docker/vision
./update_and_restart.sh
```

**Что делает**:
1. Останавливает контейнеры (`docker-compose down --remove-orphans`)
2. Обновляет код из GitHub (`git pull origin main`)
3. Запускает контейнеры (`docker-compose up -d`)
4. Отключает lazy publisher (force publish)
5. Показывает логи камеры

**Рекомендации**:
- ✅ **ОСТАВИТЬ** - ключевой инструмент для deployment
- Использовать после каждого коммита в репозиторий

---

#### 9. `docker/main/update_and_restart.sh` - Обновление и перезапуск Main Pi
**Местоположение**: `docker/main/update_and_restart.sh`  
**Назначение**: Автоматизированное обновление кода и перезапуск RTAB-Map  
**Использование**:
```bash
# На Main Pi
cd ~/rob_box_project/docker/main
./update_and_restart.sh
```

**Что делает**:
1. Останавливает контейнеры (`docker-compose down --remove-orphans`)
2. Обновляет код из GitHub (`git pull origin main`)
3. Запускает контейнеры (`docker-compose up -d`)
4. Показывает логи RTAB-Map

**Рекомендации**:
- ✅ **ОСТАВИТЬ** - ключевой инструмент для deployment
- Запускать после обновления Vision Pi

---

### 📝 Вспомогательные скрипты (в config/)

#### 10. `docker/vision/config/start_oak_d.sh` - Старт камеры OAK-D
**Местоположение**: `docker/vision/config/start_oak_d.sh`  
**Назначение**: Запуск ноды OAK-D внутри контейнера  
**Рекомендации**:
- ✅ **ОСТАВИТЬ** - используется Dockerfile для запуска камеры

#### 11. `docker/vision/config/start_apriltag.sh` - Старт AprilTag детекции
**Местоположение**: `docker/vision/config/start_apriltag.sh`  
**Назначение**: Запуск AprilTag детектора внутри контейнера  
**Рекомендации**:
- ✅ **ОСТАВИТЬ** - используется Dockerfile для запуска AprilTag

---

## Анализ последних изменений

### Коммиты (последние 7)

#### 1. `abd2629 - fixes`
**Дата**: Последний коммит  
**Изменения**:
- Удален `docker/main/discovery/Dockerfile` (FastDDS discovery сервер больше не нужен)
- Обновлены `zenoh_router_config.json5` для Vision и Main Pi
- Обновлены `docker-compose.yaml` для Vision и Main Pi

**Анализ**: Финальная очистка после перехода на Zenoh. Удалены артефакты CycloneDDS/FastDDS.

---

#### 2. `4bd59ec - default zenoh configuration`
**Дата**: Предыдущий коммит  
**Изменения**:
- Обновлены конфигурации Zenoh router и session для обоих Pi
- Удален `zenoh_local_session_config.json5` на Vision Pi
- Обновлен `Dockerfile` для AprilTag

**Анализ**: Переход на стандартные конфигурации Zenoh (по умолчанию из документации). Упрощение архитектуры.

---

#### 3-7. `520457e - c1e3d7a` - Revert коммиты
**Дата**: Откаты предыдущих попыток  
**Изменения**: Откат всех попыток использовать `rmw_zenohd` (ROS 2 Zenoh router)

**Анализ**: 
Пользователь экспериментировал с разными подходами:
1. Пытался использовать `rmw_zenohd` (ROS 2 специфичный Zenoh router)
2. Пытался исправить запуск через `source setup.bash`
3. Пытался создать кастомные Dockerfiles для роутеров с `rmw_zenoh_cpp`
4. Исправлял GPG ключи ROS 2 репозитория
5. Добавлял `LD_LIBRARY_PATH` для библиотек Zenoh

**В итоге**: Все откатил и перешёл на **стандартный Zenoh router** (`eclipse/zenoh:latest`), который работает напрямую без ROS 2 обёртки.

---

### Ключевые выводы из изменений

1. **Zenoh router**: Используется стандартный `eclipse/zenoh:latest`, НЕ `rmw_zenohd`
2. **FastDDS discovery**: Полностью удалён, больше не используется
3. **Конфигурации Zenoh**: Стандартные из документации, без кастомных модификаций
4. **Архитектура упрощена**: Меньше зависимостей, меньше кастомных Dockerfiles

---

## Текущая архитектура Zenoh

### Docker Compose конфигурация

#### Vision Pi (`docker/vision/docker-compose.yaml`)

```yaml
services:
  zenoh-router:
    image: eclipse/zenoh:latest
    container_name: zenoh-router-vision
    network_mode: host
    environment:
      - RUST_LOG=zenoh=info
    volumes:
      - ./config/zenoh_router_config.json5:/zenoh_config.json5
      - /dev/shm:/dev/shm
    command: -c /zenoh_config.json5
    restart: unless-stopped
    healthcheck:
      test: ["CMD-SHELL", "wget -qO- http://localhost:8000/@/local/router || exit 1"]
      interval: 5s
      timeout: 10s
      retries: 20
      start_period: 30s

  oak-d:
    # ...
    environment:
      - RMW_IMPLEMENTATION=rmw_zenoh_cpp
      - ZENOH_ROUTER_CHECK_ATTEMPTS=10
      - RUST_LOG=zenoh=info
      - ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      - ZENOH_CONFIG=/config/zenoh_session_config.json5
    depends_on:
      - zenoh-router
```

#### Main Pi (`docker/main/docker-compose.yaml`)

```yaml
services:
  zenoh-router:
    image: eclipse/zenoh:latest
    container_name: zenoh-router
    network_mode: host
    environment:
      - RUST_LOG=zenoh=info
    volumes:
      - ./config/zenoh_router_config.json5:/zenoh_config.json5
    command: -c /zenoh_config.json5
    restart: unless-stopped
    healthcheck:
      test: ["CMD-SHELL", "wget -qO- http://localhost:8000/@/local/router || exit 1"]
      interval: 5s
      timeout: 10s
      retries: 20
      start_period: 30s

  rtabmap:
    # ...
    environment:
      - RMW_IMPLEMENTATION=rmw_zenoh_cpp
      - ZENOH_ROUTER_CHECK_ATTEMPTS=10
      - RUST_LOG=zenoh=info
      - ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      - ZENOH_CONFIG=/config/zenoh_session_config.json5
      - LD_LIBRARY_PATH=/opt/ros/humble/opt/zenoh_cpp_vendor/lib:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib
    depends_on:
      - zenoh-router
```

### Конфигурации Zenoh

#### Router конфигурация (для обоих Pi)
**Файлы**: 
- `docker/vision/config/zenoh_router_config.json5`
- `docker/main/config/zenoh_router_config.json5`

**Ключевые параметры**:
```json5
{
  mode: "router",
  connect: {
    endpoints: []  // Зависит от Pi
  },
  // ... стандартная конфигурация из Zenoh документации
}
```

**Vision Pi router**: Подключается к Main Pi router (пусто в стандартной конфигурации, используется автообнаружение)  
**Main Pi router**: Подключается к внешнему серверу `zenoh.robbox.online:7447`

#### Session конфигурация (для ROS 2 нод)
**Файлы**:
- `docker/vision/config/zenoh_session_config.json5`
- `docker/main/config/zenoh_session_config.json5`

**Ключевые параметры**:
```json5
{
  mode: "peer",
  connect: {
    endpoints: ["tcp/localhost:7447"]  // Подключение к локальному router
  },
  // ... стандартная конфигурация
}
```

---

## Скрипты обновления и управления

### Workflow обновления системы

#### 1. Локальные изменения (на Windows)
```bash
cd d:\PROJECTS\rob_box_project
# Вносим изменения в код/конфигурации
git add .
git commit -m "Описание изменений"
git push
```

#### 2. Обновление Vision Pi
```bash
# Удалённое выполнение с Windows
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.21 \
  'cd ~/rob_box_project/docker/vision && ./update_and_restart.sh'

# Или через SSH
ssh ros2@10.1.1.21
cd ~/rob_box_project/docker/vision
./update_and_restart.sh
```

#### 3. Обновление Main Pi
```bash
# Удалённое выполнение с Windows
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.20 \
  'cd ~/rob_box_project/docker/main && ./update_and_restart.sh'

# Или через SSH
ssh ros2@10.1.1.20
cd ~/rob_box_project/docker/main
./update_and_restart.sh
```

#### 4. Проверка статуса
```bash
# Vision Pi
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.21 'docker ps'

# Main Pi
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.20 'docker ps'
```

---

## Рекомендации по скриптам

### ✅ ОСТАВИТЬ (критичные)
1. **docker/monitor_system.sh** - универсальный мониторинг
2. **docker/vision/realtime_monitor.sh** - детальный мониторинг камеры
3. **docker/diagnose_data_flow.sh** - диагностика межплатной связи
4. **docker/vision/diagnose.sh** - быстрая диагностика камеры
5. **docker/vision/update_and_restart.sh** - deployment Vision Pi
6. **docker/main/update_and_restart.sh** - deployment Main Pi
7. **docker/vision/config/start_oak_d.sh** - запуск камеры (используется Dockerfile)
8. **docker/vision/config/start_apriltag.sh** - запуск AprilTag (используется Dockerfile)

### ⚠️ УСТАРЕЛИ / ТРЕБУЮТ ПЕРЕСМОТРА
1. **docker/vision/monitor_camera_startup.sh** - дублирует realtime_monitor.sh
   - Рекомендация: удалить или объединить с realtime_monitor.sh
2. **docker/vision/force_publish.sh** - логика в update_and_restart.sh
   - Рекомендация: удалить, оставить как справочник
3. **docker/tune.sh** - для CycloneDDS (больше не используется)
   - Рекомендация: удалить, оставить в TROUBLESHOOTING.md как историю

### 📚 Документация
Текущая документация актуальна и хорошо структурирована:
- **ARCHITECTURE.md** - архитектура системы
- **TROUBLESHOOTING.md** - решение проблем (обновлен пользователем)
- **OPTIMIZATION_README.md** - оптимизация камеры
- **QUICK_START_RU.md** - быстрый старт
- **README_RU.md** - основная инструкция
- **SYSTEM_TUNING.md** - настройка системы
- **CHANGES.md** - история изменений
- **CHECKLIST.md** - чеклист настройки
- **SUMMARY.md** - краткая сводка

---

## Следующие шаги для агентов

### При работе с системой

1. **Всегда проверяйте последние коммиты** перед внесением изменений
   ```bash
   git log -5 --oneline --name-status
   ```

2. **Используйте правильные IP адреса**:
   - SSH доступ: WiFi IP (10.1.1.21, 10.1.1.20)
   - Конфигурации Zenoh: Ethernet IP (10.1.1.11, 10.1.1.10)

3. **Диагностика проблем**:
   - Сначала `diagnose.sh` на Vision Pi
   - Затем `diagnose_data_flow.sh` для межплатной связи
   - `monitor_system.sh` для общей производительности

4. **Deployment**:
   - Всегда `git push` перед обновлением Pi
   - Обновляйте Vision Pi → Main Pi в таком порядке
   - Проверяйте логи после каждого обновления

5. **Zenoh router**:
   - Используйте стандартный `eclipse/zenoh:latest`
   - НЕ используйте `rmw_zenohd` (откачено пользователем)
   - Конфигурации в `zenoh_router_config.json5` и `zenoh_session_config.json5`

---

## История миграции на Zenoh

### Было (CycloneDDS)
- Multicast discovery между Pi
- Проблемы с обнаружением нод (0 KB/s сетевой трафик)
- Необходимость FastDDS discovery сервера

### Стало (Zenoh)
- Router-to-router архитектура
- Vision Pi router ↔ Main Pi router ↔ zenoh.robbox.online
- Стандартный `eclipse/zenoh:latest` router
- Peer mode для ROS 2 нод (подключение к localhost:7447)

### Попытки использования rmw_zenohd (откачены)
Пользователь пытался использовать специфичный для ROS 2 `rmw_zenohd` router:
- Создавал кастомные Dockerfiles
- Исправлял GPG ключи ROS 2
- Добавлял LD_LIBRARY_PATH для Zenoh библиотек

**В итоге**: Откатил все изменения и перешёл на стандартный Zenoh router, который работает стабильно.

---

**Документ создан**: 2025-10-09  
**Версия системы**: Zenoh architecture (после миграции с CycloneDDS)  
**Последний анализированный коммит**: abd2629
