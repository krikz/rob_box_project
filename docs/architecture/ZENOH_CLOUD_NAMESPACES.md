# Zenoh: Облачное подключение и конфигурация Namespace

## 📋 Содержание
- [Обзор](#обзор)
- [Функция Zenoh Namespace](#функция-zenoh-namespace)
- [Текущая реализация](#текущая-реализация)
- [Облачная топология](#облачная-топология)
- [Детали конфигурации](#детали-конфигурации)
- [Как это работает](#как-это-работает)
- [Тестирование и валидация](#тестирование-и-валидация)
- [Устранение неполадок](#устранение-неполадок)

---

## Обзор

Этот документ объясняет, как робот Rob Box интегрируется с облачным Zenoh роутером на `zenoh.robbox.online:7447`, используя функцию **namespace** Zenoh для изоляции роботов.

### Ключевые концепции

1. **Zenoh Namespace**: Префикс, автоматически добавляемый ко всем ключевым выражениям (топикам)
2. **Robot ID**: Уникальный идентификатор каждого робота (например, `RBXU100001`)
3. **Cloud Router**: Центральный Zenoh роутер на `zenoh.robbox.online:7447`
4. **Изоляция топиков**: Топики каждого робота имеют префикс `robots/{ROBOT_ID}/`

---

## Функция Zenoh Namespace

### Что такое Zenoh Namespace?

Из [документации Zenoh](https://github.com/eclipse-zenoh/zenoh/blob/main/DEFAULT_CONFIG.json5):

> **Префикс namespace.**
> Если указан, все исходящие ключевые выражения будут автоматически префиксированы указанной строкой,
> а все входящие ключевые выражения будут очищены от указанного префикса.
> 
> Префикс namespace должен удовлетворять всем ограничениям ключевых выражений и не может содержать символы подстановки ('*').
> 
> Namespace применяется к сессии.
> Например, если сессия имеет namespace "1", то `session.put("my/keyexpr", my_message)` поместит сообщение в `1/my/keyexpr`. То же самое применяется ко всем операциям в этой сессии.

### Как это работает

```
Без namespace:
  Публикация:  /cmd_vel       →  Zenoh:  /cmd_vel
  Подписка:    /odom          →  Zenoh:  /odom

С namespace "robots/RBXU100001":
  Публикация:  /cmd_vel       →  Zenoh:  robots/RBXU100001/cmd_vel
  Подписка:    /odom          →  Zenoh:  robots/RBXU100001/odom
  
  НО с точки зрения робота, топики по-прежнему /cmd_vel и /odom!
  Namespace прозрачен для ROS 2 нод.
```

**Преимущества:**
- ✅ Полная изоляция топиков между роботами
- ✅ Прозрачность для ROS 2 кода (изменения не требуются)
- ✅ Простой мониторинг облака: подписка на `robots/**`
- ✅ Простой мониторинг отдельного робота: подписка на `robots/RBXU100001/**`
- ✅ Нет конфликтов с ROS_DOMAIN_ID

---

## Текущая реализация

### Конфигурация Robot ID

**Расположение:** `docker/vision/.env` и `docker/main/.env`

```bash
# Robot ID для изоляции namespace
ROBOT_ID=RBXU100001
```

**Требования к формату:**
- Только буквенно-цифровые символы
- Без спецсимволов, кроме подчёркивания
- Уникальный для каждого робота
- Рекомендуемый формат: `RBXU` + 6 цифр (например, `RBXU100001`, `RBXU100002`)

### Скрипт-обёртка для Namespace

**Расположение:** `docker/common/scripts/ros_with_namespace.sh` (общий для всех Pi, ADR-0159 В4; монтируется как `/ros_scripts`)

Этот скрипт:
1. Читает переменную окружения `ROBOT_ID`
2. Копирует базовую конфигурацию сессии в `/tmp/zenoh_session_config.json5`
3. Раскомментирует и устанавливает `namespace: "robots/{ROBOT_ID}"`
4. Устанавливает `ZENOH_SESSION_CONFIG_URI` на сгенерированную конфигурацию
5. Запускает ROS ноду

**Ключевой код:**
```bash
# Раскомментируем и заменяем namespace
sed -i "s|// namespace: \"my/namespace\"|namespace: \"robots/$ROBOT_ID\"|g" "$GENERATED_CONFIG"

# Обновляем ZENOH_SESSION_CONFIG_URI на сгенерированный файл
export ZENOH_SESSION_CONFIG_URI="$GENERATED_CONFIG"
```

### Интеграция с Docker Compose

**Расположение:** `docker/vision/docker-compose.yaml`

Все ROS сервисы используют обёртку:

```yaml
oak-d:
  environment:
    - ROBOT_ID=${ROBOT_ID}  # Для генерации namespace
  command: ["/ros_scripts/ros_with_namespace.sh", "/scripts/start_oak_d.sh"]

lslidar:
  environment:
    - ROBOT_ID=${ROBOT_ID}
  command: ["/ros_scripts/ros_with_namespace.sh", "/scripts/start_lslidar.sh"]

# ... аналогично для apriltag, led-matrix-driver, voice-assistant, perception
```

---

## Облачная топология

### Сетевая архитектура

```
                    ┌──────────────────────────┐
                    │  zenoh.robbox.online     │ (Облако)
                    │       :7447              │
                    │  Хранилище: robots/**    │
                    └────────────┬─────────────┘
                                 │
                                 │ TCP/TLS
                                 │
                   ┌─────────────▼──────────────┐
                   │  Zenoh Router Main Pi      │
                   │    (10.1.1.10:7447)        │
                   │    режим: router           │
                   └──────┬──────────────┬──────┘
                          │              │
                   TCP/IP │              │ TCP/IP
                          │              │
          ┌───────────────▼──┐      ┌───▼──────────────────┐
          │ Zenoh Router     │      │  ROS ноды Main Pi    │
          │ Vision Pi        │      │  (через rmw_zenoh)   │
          │ (10.1.1.11:7447) │      │  - rtabmap           │
          │ режим: router    │      │  - nav2              │
          └──────┬───────────┘      │  - twist_mux         │
                 │                  └──────────────────────┘
          ┌──────▼────────────┐
          │  ROS ноды         │
          │  Vision Pi        │
          │  (через rmw_zenoh)│
          │  - oak-d          │
          │  - lslidar        │
          │  - apriltag       │
          │  - voice          │
          └───────────────────┘
```

### Пример потока топика

Для робота `RBXU100001`, публикующего `/cmd_vel`:

```
1. ROS нода публикует:       /cmd_vel
2. rmw_zenoh добавляет префикс: robots/RBXU100001/cmd_vel
3. Zenoh сессия отправляет:  robots/RBXU100001/cmd_vel
4. Локальный роутер пересылает: robots/RBXU100001/cmd_vel
5. Роутер Main Pi отправляет:  robots/RBXU100001/cmd_vel → zenoh.robbox.online
6. Облачное хранилище сохраняет: robots/RBXU100001/cmd_vel
```

---

## Детали конфигурации

### Конфигурация роутера Vision Pi

**Расположение:** шаблон `docker/common/config/zenoh_router_config.json5`; адреса — `ZENOH_LISTEN_IP`/`ZENOH_CONNECT_ENDPOINTS` в `docker/vision/docker-compose.yaml`

```json5
{
  mode: "router",
  connect: {
    endpoints: [
      "tcp/10.1.1.10:7447"  // Подключение к роутеру Main Pi
    ],
  },
  listen: {
    endpoints: ["tcp/[::]:7447"],
  },
  scouting: {
    multicast: { enabled: false }
  }
}
```

**Назначение:** Маршрутизация топиков Vision Pi к Main Pi

### Конфигурация роутера Main Pi

**Расположение:** шаблон `docker/common/config/zenoh_router_config.json5`; адреса — `ZENOH_LISTEN_IP`/`ZENOH_CONNECT_ENDPOINTS` в `docker/main/docker-compose.yaml`

```json5
{
  mode: "router",
  connect: {
    endpoints: [
      "tcp/zenoh.robbox.online:7447"  // Подключение к облаку
    ],
  },
  listen: {
    endpoints: ["tcp/[::]:7447"],
  },
  plugins: {
    rest: {
      http_port: 8000,
    },
    storage_manager: {
      storages: {
        robot_data: {
          key_expr: "robots/**",  // Хранение всех данных роботов
          volume: {
            id: "memory",
          },
        },
      },
    },
  },
}
```

**Назначение:** Маршрутизация всех топиков робота в облако и локальное хранение

### Конфигурация сессии (обе Pi)

**Расположение:** `docker/common/config/zenoh_session_config.json5` (адрес роутера `${ZENOH_LISTEN_IP}` подставляет `ros_with_namespace.sh`)

Базовая конфигурация (namespace закомментирован по умолчанию):

```json5
{
  mode: "peer",
  connect: {
    endpoints: ["tcp/localhost:7447"],  // Подключение к локальному роутеру
  },
  listen: {
    endpoints: ["tcp/localhost:0"],  // Случайный порт
  },
  
  // Namespace закомментирован по умолчанию
  // Раскомментируется обёрткой ros_with_namespace.sh
  // namespace: "my/namespace",
}
```

**Сгенерированная конфигурация** во время выполнения в `/tmp/zenoh_session_config.json5`:

```json5
{
  mode: "peer",
  connect: {
    endpoints: ["tcp/localhost:7447"],
  },
  listen: {
    endpoints: ["tcp/localhost:0"],
  },
  
  // Раскомментировано и установлено ros_with_namespace.sh
  namespace: "robots/RBXU100001",
}
```

---

## Как это работает

### Пошаговый процесс

#### 1. Запуск контейнера

Когда запускается сервис Vision Pi:

```bash
docker-compose up oak-d
```

Docker выполняет:

```yaml
command: ["/ros_scripts/ros_with_namespace.sh", "/scripts/start_oak_d.sh"]
environment:
  - ROBOT_ID=RBXU100001
  - ZENOH_CONFIG=/config/zenoh_session_config.json5
```

#### 2. Конфигурация Namespace

`ros_with_namespace.sh` выполняет:

```bash
#!/bin/bash
ROBOT_ID=RBXU100001  # Из окружения

# Генерируем конфигурацию с namespace
GENERATED_CONFIG="/tmp/zenoh_session_config.json5"
cp /config/zenoh_session_config.json5 "$GENERATED_CONFIG"
sed -i "s|// namespace: \"my/namespace\"|namespace: \"robots/$ROBOT_ID\"|g" "$GENERATED_CONFIG"

# Указываем на сгенерированную конфигурацию
export ZENOH_SESSION_CONFIG_URI="$GENERATED_CONFIG"

# Запускаем ноду
exec /scripts/start_oak_d.sh
```

#### 3. Запуск ROS ноды

Нода OAK-D запускается с:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI=/tmp/zenoh_session_config.json5

ros2 launch depthai_ros_driver camera.launch.py
```

#### 4. Публикация топика

Когда OAK-D публикует `/camera/rgb/image_raw`:

```
ROS нода → rmw_zenoh → Zenoh сессия (с namespace) → Локальный роутер → Главный роутер → Облако
```

Zenoh видит: `robots/RBXU100001/camera/rgb/image_raw`

#### 5. Подписка в облаке

Облако может подписаться на:

```bash
# Все роботы, все топики
robots/**

# Конкретный робот, все топики
robots/RBXU100001/**

# Конкретный робот, конкретный топик
robots/RBXU100001/camera/rgb/image_raw

# Все роботы, только топики камеры
robots/*/camera/**
```

---

## Тестирование и валидация

### Проверка конфигурации Namespace

**На Vision Pi:**

```bash
# Проверить переменную окружения
docker exec oak-d env | grep ROBOT_ID
# Ожидается: ROBOT_ID=RBXU100001

# Проверить сгенерированную конфигурацию
docker exec oak-d cat /tmp/zenoh_session_config.json5 | grep namespace
# Ожидается: namespace: "robots/RBXU100001",

# Проверить применение конфигурации
docker logs oak-d 2>&1 | grep "Robot ID\|Namespace"
# Ожидается:
# 🤖 Robot ID: RBXU100001
# 📡 Namespace: robots/RBXU100001
```

**На Main Pi:**

```bash
# Проверить REST API Zenoh роутера для топиков
curl http://localhost:8000/@/local/subscriber | jq

# Должны быть видны топики с префиксом robots/RBXU100001/
# Пример:
# {
#   "key": "robots/RBXU100001/camera/rgb/image_raw",
#   ...
# }
```

### Тестирование облачного подключения

**С машины разработки:**

```bash
# Установить инструменты Zenoh
cargo install zenoh --features=unstable

# Подписаться на все топики роботов
z_sub -e "tcp/zenoh.robbox.online:7447" -k "robots/**"

# Подписаться на конкретного робота
z_sub -e "tcp/zenoh.robbox.online:7447" -k "robots/RBXU100001/**"

# Запросить доступные топики
z_get -e "tcp/zenoh.robbox.online:7447" -s "robots/**"
```

### Валидация изоляции топиков

**Тест с несколькими роботами:**

```bash
# Робот 1 (RBXU100001)
# Публикует: /cmd_vel → robots/RBXU100001/cmd_vel

# Робот 2 (RBXU100002)  
# Публикует: /cmd_vel → robots/RBXU100002/cmd_vel

# Проверка отсутствия перекрёстных помех:
z_sub -e "tcp/zenoh.robbox.online:7447" -k "robots/RBXU100001/cmd_vel"
# Должны быть видны только сообщения от Робота 1
```

---

## Устранение неполадок

### Проблема: Топики не появляются в облаке

**Симптомы:**
- Облако не видит топики робота
- `z_sub` не возвращает данных

**Проверка:**

1. **Убедиться, что ROBOT_ID установлен:**
   ```bash
   docker exec oak-d env | grep ROBOT_ID
   ```

2. **Проверить сгенерированную конфигурацию:**
   ```bash
   docker exec oak-d cat /tmp/zenoh_session_config.json5 | grep namespace
   ```

3. **Проверить облачное подключение:**
   ```bash
   # На Main Pi
   curl http://localhost:8000/@/router/status
   ```

4. **Проверить логи Zenoh роутера:**
   ```bash
   docker logs zenoh-router
   docker logs zenoh-router-vision
   ```

**Решение:**
- Если namespace не установлен: проверить файл `.env` на наличие `ROBOT_ID=...`
- Если namespace неправильный: перезапустить контейнеры: `docker-compose restart`
- Если нет облачного подключения: проверить фаервол, сетевое соединение

### Проблема: Неправильный префикс Namespace

**Симптомы:**
- Топики появляются как `robots/default/...` вместо `robots/RBXU100001/...`

**Причина:** ROBOT_ID не передаётся в контейнер

**Решение:**

```bash
# Проверить docker-compose.yaml содержит:
environment:
  - ROBOT_ID=${ROBOT_ID}

# Проверить файл .env:
cat docker/vision/.env | grep ROBOT_ID
# Должно быть: ROBOT_ID=RBXU100001

# Перезапустить с явной переменной окружения:
ROBOT_ID=RBXU100001 docker-compose up -d
```

### Проблема: Namespace не применяется к некоторым нодам

**Симптомы:**
- Некоторые топики имеют namespace, некоторые нет

**Причина:** Сервис не использует обёртку ros_with_namespace.sh

**Решение:**

Проверить `docker-compose.yaml`:

```yaml
# ✅ ПРАВИЛЬНО
services:
  my-node:
    command: ["/ros_scripts/ros_with_namespace.sh", "/scripts/start_node.sh"]
    environment:
      - ROBOT_ID=${ROBOT_ID}

# ❌ НЕПРАВИЛЬНО (отсутствует обёртка)
services:
  my-node:
    command: ["/scripts/start_node.sh"]
```

### Проблема: Невозможно подписаться на облачные топики

**Симптомы:**
- `z_sub` не может подключиться
- Таймаут подключения

**Проверка:**

1. **Доступность облачного роутера:**
   ```bash
   telnet zenoh.robbox.online 7447
   # Должно подключиться
   ```

2. **Правила фаервола:**
   ```bash
   # На Main Pi
   sudo ufw status
   # Порт 7447 должен быть разрешён
   ```

3. **DNS разрешение:**
   ```bash
   nslookup zenoh.robbox.online
   ping zenoh.robbox.online
   ```

**Решение:**
- Добавить правило фаервола: `sudo ufw allow 7447/tcp`
- Проверить VPN/сетевое подключение
- Убедиться, что облачный роутер запущен

---

## ROS 2 Domain ID vs Zenoh Namespace

### Ключевые отличия

| Функция | ROS_DOMAIN_ID | Zenoh Namespace |
|---------|---------------|-----------------|
| **Уровень** | Сетевая изоляция | Префикс топика |
| **Назначение** | Предотвращение перекрёстных помех DDS | Иерархическая организация топиков |
| **Видимость** | Ноды в разных доменах не видят друг друга | Все топики видимы, просто с префиксом |
| **Облако** | Не видно в именах топиков | Видно в ключевых выражениях |
| **Случай использования** | Несколько роботов в одной сети | Облачная организация |

### Rob Box использует оба

```
ROS_DOMAIN_ID=0  (по умолчанию)
  └── Все ноды общаются через DDS/Zenoh
      └── Zenoh namespace: robots/RBXU100001/
          └── Все топики с префиксом: robots/RBXU100001/cmd_vel и т.д.
```

**Преимущества:**
- ROS_DOMAIN_ID: Изоляция локальной сети
- Zenoh namespace: Облачная организация и мониторинг

---

## Соображения безопасности

### Текущая настройка (без TLS)

```
Main Pi ──TCP (незашифрованный)──> zenoh.robbox.online:7447
```

**Риски:**
- Трафик не зашифрован
- Нет аутентификации
- Любой может публиковать/подписываться

**Приемлемо для:**
- Разработки
- Тестирования
- Закрытых сетей

### Будущее: Настройка TLS/QUIC

**Рекомендуется для production:**

```json5
{
  connect: {
    endpoints: [
      "tls/zenoh.robbox.online:7448"  // TLS на порту 7448
    ]
  },
  transport: {
    link: {
      tls: {
        root_ca_certificate: "/certs/ca.pem",
        connect_certificate: "/certs/robot.pem",
        connect_private_key: "/certs/robot.key",
        verify_name_on_connect: true,
      }
    }
  }
}
```

**Преимущества:**
- Зашифрованный трафик
- Взаимная аутентификация
- Контроль доступа на основе сертификатов

---

## Справочная информация

### Официальная документация

1. **Конфигурация Zenoh:**
   - https://github.com/eclipse-zenoh/zenoh/blob/main/DEFAULT_CONFIG.json5

2. **Дизайн rmw_zenoh:**
   - https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md

3. **ROS 2 Namespaces:**
   - https://design.ros2.org/articles/topic_and_service_names.html

4. **Ключевые выражения Zenoh:**
   - https://github.com/eclipse-zenoh/roadmap/blob/main/rfcs/ALL/Key%20Expressions.md

### Файлы проекта

- `docker/common/scripts/ros_with_namespace.sh` - Скрипт-обёртка для namespace
- `docker/common/config/zenoh_session_config.json5` - Шаблон конфигурации сессии (все Pi)
- `docker/common/config/zenoh_router_config.json5` - Шаблон конфигурации роутера (все Pi; облачное подключение Main Pi — `ZENOH_CONNECT_ENDPOINTS` в `docker/main/docker-compose.yaml`)
- `docker/vision/.env` - Конфигурация ROBOT_ID
- `docs/architecture/SOFTWARE.md` - Обзор архитектуры Zenoh
- `docs/development/AGENT_GUIDE.md` - Руководство по разработке для AI агентов

---

## История изменений

### 2025-10-23 - Первоначальная документация
- Задокументирована функция Zenoh namespace
- Объяснена текущая реализация с ros_with_namespace.sh
- Добавлены диаграммы облачной топологии
- Создано руководство по устранению неполадок
- Добавлены процедуры тестирования

---

**Автор:** AI Agent (GitHub Copilot)  
**Последнее обновление:** 2025-10-24  
**Статус:** Активный - реализация Namespace работает в production
