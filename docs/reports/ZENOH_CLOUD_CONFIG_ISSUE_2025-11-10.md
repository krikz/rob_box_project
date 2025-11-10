# Исправление конфигурации Zenoh облачного роутера

**Дата:** 2025-11-10  
**Проблема:** Команды через веб API не доходят до робота  
**Статус:** 🔍 Диагностика

---

## 🔴 Проблема

Когда отправляются команды через веб API на топик:
```
PUT https://zenoh.robbox.online/robots/RBXU100001/0/cmd_vel_voice
```

Робот не получает команды. В логах twist_mux видно, что он подписан на топик `cmd_vel_voice`, но сообщения не приходят.

**Логи twist_mux:**
```
[INFO] [1762762270.569858128] [twist_mux]: Topic handler 'topics.voice' subscribed to topic 'cmd_vel_voice': timeout = 1.000000s , priority = 25.
```

---

## 🔍 Анализ проблемы

### Текущая конфигурация облачного Zenoh роутера

```json5
{
  "mode": "peer",  // ❌ ПРОБЛЕМА 1: должен быть "router"!
  "listen": {
    "endpoints": [
      "tcp/0.0.0.0:7447"
    ]
  },
  "timestamping": {
    "enabled": true
  },
  "plugins": {
    "rest": {
      "http_port": 8000
    },
    "storage_manager": {
      "storages": {
        "robot_data": {
          "key_expr": "robots/**",
          "volume": {
            "id": "memory"
          }
        }
      }
    }
  }
}
```

### Обнаруженные проблемы

#### ❌ Проблема 1: Режим "peer" вместо "router"

**Почему это проблема:**

Zenoh имеет три режима работы:
- **client** - подключается к роутеру, не маршрутизирует
- **peer** - peer-to-peer соединения, ограниченная маршрутизация
- **router** - полная маршрутизация между всеми подключенными клиентами

В режиме `peer`:
- Узел НЕ гарантирует маршрутизацию между всеми подключенными клиентами
- Подходит для peer-to-peer сетей, где все узлы знают друг о друге
- **НЕ подходит** для централизованного облачного роутера

**Что происходит:**
1. Робот подключается к облаку как peer
2. Веб API публикует через REST плагин
3. Облачный Zenoh в режиме peer **НЕ маршрутизирует** сообщение от REST к роботу
4. Команда теряется

#### ❌ Проблема 2: Storage не пересылает сообщения в реальном времени

**Storage manager:**
```json5
"storage_manager": {
  "storages": {
    "robot_data": {
      "key_expr": "robots/**",
      "volume": {
        "id": "memory"
      }
    }
  }
}
```

**Что делает storage:**
- ✅ Сохраняет последнее значение ключа в память
- ✅ Позволяет запрашивать через GET запросы
- ❌ **НЕ пересылает** PUT запросы подписчикам в реальном времени

**Что нужно:**
- Облачный роутер должен работать как **полноценный маршрутизатор**
- PUT запросы через REST API должны маршрутизироваться ко всем подписчикам
- Storage - это дополнительная функция для истории, не для маршрутизации

---

## ✅ Полное решение

### Шаг 1: Исправить конфигурацию облачного роутера

См. [docs/cloud/README.md](../cloud/README.md) для детальных инструкций.

**Кратко:**
```bash
# На zenoh.robbox.online
sudo systemctl stop zenoh-router
sudo nano /etc/zenoh/config.json5
# Изменить: "mode": "peer" → "mode": "router"
# Добавить: routing.router.peers_failover_brokering = true
sudo systemctl start zenoh-router
```

### Шаг 2: Установить мост Zenoh REST → ROS

**Проблема:** REST API не знает о ROS типах сообщений. Нужен мост.

**Решение:** Использовать Python скрипт `zenoh_rest_bridge.py`

#### На роботе:

1. **Установить zenoh-python:**
   ```bash
   pip3 install eclipse-zenoh
   ```

2. **Скопировать скрипт моста:**
   ```bash
   cp ~/rob_box_project/scripts/zenoh_rest_bridge.py ~/
   ```

3. **Запустить мост:**
   ```bash
   # Через ROS2
   source /opt/ros/humble/setup.bash
   python3 ~/zenoh_rest_bridge.py
   
   # Или через systemd сервис (рекомендуется)
   sudo nano /etc/systemd/system/zenoh-rest-bridge.service
   ```

4. **Systemd сервис (опционально):**
   ```ini
   [Unit]
   Description=Zenoh REST to ROS Bridge
   After=network.target
   
   [Service]
   Type=simple
   User=ros2
   Environment="ROS_DOMAIN_ID=0"
   Environment="RMW_IMPLEMENTATION=rmw_zenoh_cpp"
   ExecStart=/usr/bin/python3 /home/ros2/zenoh_rest_bridge.py
   Restart=always
   
   [Install]
   WantedBy=multi-user.target
   ```

5. **Включить сервис:**
   ```bash
   sudo systemctl enable zenoh-rest-bridge
   sudo systemctl start zenoh-rest-bridge
   sudo systemctl status zenoh-rest-bridge
   ```

#### Использование:

```bash
# Отправить команду через REST API
curl -X PUT http://zenoh.robbox.online/robots/RBXU100001/cmd_vel_web_bridge \
  -H "Content-Type: application/json" \
  -d '{"linear":{"x":0.1,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}'

# Мост получит JSON, преобразует в Twist и опубликует в ROS топик cmd_vel_voice
# twist_mux получит команду и передаст в /diff_drive_controller/cmd_vel_unstamped
```

### Альтернативное решение: zenoh-plugin-dds

Для production рекомендуется использовать официальный [zenoh-plugin-dds](https://github.com/eclipse-zenoh/zenoh-plugin-dds):

```json5
// На облачном роутере
{
  "mode": "router",
  "plugins": {
    "dds": {
      "domain": 0,
      "allow": {
        "topics": ["cmd_vel_voice"]
      }
    }
  }
}
```

Это автоматически:
- Конвертирует между Zenoh и DDS/ROS2
- Обрабатывает типы сообщений
- Поддерживает CDR сериализацию

```json5
{
  /// ✅ КРИТИЧНО: режим "router" для полной маршрутизации
  "mode": "router",
  
  /// Endpoints для подключения (облачный роутер обычно не подключается к другим)
  "connect": {
    "endpoints": []
  },
  
  /// Слушаем на всех интерфейсах
  "listen": {
    "endpoints": [
      "tcp/0.0.0.0:7447"
    ]
  },
  
  /// Отключаем multicast scouting (не нужно в облаке)
  "scouting": {
    "multicast": {
      "enabled": false
    }
  },
  
  /// Включаем timestamping для ROS топиков
  "timestamping": {
    "enabled": true
  },
  
  /// Настройки маршрутизации для роутера
  "routing": {
    "router": {
      /// Включаем failover brokering - роутер пересылает между peers
      "peers_failover_brokering": true
    }
  },
  
  /// Плагины
  "plugins": {
    /// REST API для веб доступа
    "rest": {
      "http_port": 8000
    },
    /// Storage для сохранения истории (опционально)
    "storage_manager": {
      "storages": {
        "robot_data": {
          "key_expr": "robots/**",
          "volume": {
            "id": "memory"
          }
        }
      }
    }
  }
}
```

### Ключевые изменения

1. **`"mode": "router"`** - полная маршрутизация между всеми клиентами
2. **`"peers_failover_brokering": true`** - роутер пересылает сообщения между peers
3. **`"scouting.multicast.enabled": false`** - отключаем multicast в облаке

---

## 📊 Как работает маршрутизация

### До исправления (режим peer)

```
Веб API (REST)                     Робот (peer)
      │                                  │
      │ PUT /robots/.../cmd_vel_voice    │
      ↓                                  │
Облачный Zenoh (peer)                   │
      │                                  │
      │ ❌ НЕ маршрутизирует             │
      │    к другому peer                │
      └──────────────────────────────────┘
                 Сообщение потеряно
```

### После исправления (режим router)

```
Веб API (REST)                     Робот (peer)
      │                                  │
      │ PUT /robots/.../cmd_vel_voice    │ Подписан на cmd_vel_voice
      ↓                                  ↑
Облачный Zenoh (router)                 │
      │                                  │
      │ ✅ Маршрутизирует                │
      │    к подписчикам                 │
      └──────────────────────────────────┘
              Сообщение доставлено ✓
```

---

## 🔧 Применение исправления

### На облачном сервере (zenoh.robbox.online)

1. **Остановить Zenoh роутер:**
   ```bash
   # Если запущен через systemd
   sudo systemctl stop zenoh-router
   
   # Если запущен через Docker
   docker stop zenoh-router
   ```

2. **Обновить конфигурацию:**
   ```bash
   # Создать резервную копию
   sudo cp /etc/zenoh/config.json5 /etc/zenoh/config.json5.backup
   
   # Применить новую конфигурацию (см. выше)
   sudo nano /etc/zenoh/config.json5
   ```

3. **Запустить Zenoh роутер:**
   ```bash
   # Через systemd
   sudo systemctl start zenoh-router
   sudo systemctl status zenoh-router
   
   # Через Docker
   docker start zenoh-router
   docker logs -f zenoh-router
   ```

4. **Проверить логи:**
   ```bash
   # Должно быть: "mode": "router"
   # Должно быть подключение от робота
   docker logs zenoh-router | grep -i "mode\|connection"
   ```

### Проверка на роботе

После обновления облачного роутера, перезапустить Zenoh роутер на роботе:

```bash
# Main Pi
ssh ros2@10.1.1.20
cd ~/rob_box_project/docker/main
docker-compose restart zenoh-router

# Проверить подключение
docker logs zenoh-router | grep -i "connect\|cloud"
```

---

## 🧪 Тестирование

### 1. Проверка маршрутизации через REST API

```bash
# Отправить команду через REST
curl -X PUT https://zenoh.robbox.online/robots/RBXU100001/0/cmd_vel_voice \
  -H "Content-Type: application/octet-stream" \
  --data-binary @twist_message.bin

# На роботе проверить логи twist_mux
docker logs -f twist-mux
# Должно появиться сообщение о получении команды
```

### 2. Проверка подписчиков через Zenoh REST API

```bash
# Запросить список подписчиков
curl http://zenoh.robbox.online:8000/@/local/subscriber | jq

# Должен быть виден робот, подписанный на robots/RBXU100001/**
```

### 3. Проверка с zenoh CLI tools

```bash
# Установить zenoh (на машине разработки)
cargo install zenoh --features=unstable

# Подписаться на топики робота
z_sub -e "tcp/zenoh.robbox.online:7447" -k "robots/RBXU100001/**"

# В другом терминале опубликовать
z_put -e "tcp/zenoh.robbox.online:7447" \
      -k "robots/RBXU100001/0/cmd_vel_voice" \
      -v "test"

# Первый терминал должен получить сообщение
```

---

## 📝 Дополнительная информация

### Почему режим важен

Из документации Zenoh:

> **Router mode:**
> - Полная маршрутизация между всеми подключенными клиентами
> - Поддерживает failover brokering
> - Оптимизирован для централизованных топологий
> - **Рекомендуется для облачных/центральных узлов**
>
> **Peer mode:**
> - Peer-to-peer коммуникация
> - Ограниченная маршрутизация
> - Подходит для распределенных сетей
> - **НЕ рекомендуется для централизованных роутеров**

### ROS топики и Zenoh ключи

**КРИТИЧНО:** ROS топики через rmw_zenoh используют **сложный формат ключа**!

ROS топик `cmd_vel_voice` преобразуется в Zenoh ключ:
```
robots/RBXU100001/0/cmd_vel_voice/geometry_msgs::msg::dds_::Twist_/RIHS01_...
```

Где:
- `robots/RBXU100001` - Zenoh namespace (из ROBOT_ID)
- `0` - ROS_DOMAIN_ID
- `cmd_vel_voice` - имя топика
- `geometry_msgs::msg::dds_::Twist_` - тип сообщения
- `RIHS01_...` - хаш типа

**Проблема REST API:**

Из исходников Zenoh REST плагина ([lib.rs](https://github.com/eclipse-zenoh/zenoh/blob/main/plugins/zenoh-plugin-rest/src/lib.rs)):

```rust
async fn write(mut req: Request<(Arc<Session>, String)>) -> tide::Result<Response> {
    let key_expr = path_to_key_expr(req.url().path(), &req.state().1)?;
    let res = session.put(&key_expr, bytes).encoding(encoding).await;
    // ...
}

fn path_to_key_expr<'a>(path: &'a str, zid: &str) -> ZResult<KeyExpr<'a>> {
    let path = path.strip_prefix('/').unwrap_or(path);
    KeyExpr::try_from(path)  // Просто использует путь URL как ключ!
}
```

REST API **НЕ знает** о ROS типах сообщений и хашах. Он просто:
1. Берёт путь URL: `/robots/RBXU100001/0/cmd_vel_voice`
2. Публикует в Zenoh с этим ключом
3. **Не добавляет** тип сообщения и хаш!

**Результат:** ROS нода подписана на `robots/.../cmd_vel_voice/geometry_msgs::msg::dds_::Twist_/RIHS01_...`, но REST публикует в `robots/.../cmd_vel_voice` - ключи не совпадают!

### ✅ Правильное решение

**Вариант 1: Использовать ROS bridge (рекомендуется)**

Создать отдельный ROS2 узел-мост, который:
1. Подписывается на Zenoh топик через REST (простой ключ)
2. Публикует в ROS топик (с правильным типом)

```python
# Пример моста
class ZenohToRosBridge(Node):
    def __init__(self):
        super().__init__('zenoh_ros_bridge')
        # ROS publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_voice', 10)
        
        # Zenoh subscriber (через REST или напрямую)
        # Подписывается на упрощенный ключ
        self.create_subscription(...)
```

**Вариант 2: Публиковать на полный ключ (сложно)**

Нужно узнать точный хаш типа и публиковать на полный ключ:
```bash
# 1. Узнать полный ключ через ros2 topic info
ssh ros2@10.1.1.20
ros2 topic list | grep cmd_vel_voice
# Смотреть логи rmw_zenoh для полного ключа

# 2. Публиковать на полный ключ
curl -X PUT https://zenoh.robbox.online/robots/RBXU100001/0/cmd_vel_voice/geometry_msgs::msg::dds_::Twist_/RIHS01_abc123... \
  --data-binary @twist.cdr
```

**Проблема:** Формат CDR сообщения тоже должен быть правильным (бинарный формат DDS).

**Вариант 3: Использовать Zenoh-DDS bridge (самое правильное)**

Eclipse Zenoh предоставляет [zenoh-plugin-dds](https://github.com/eclipse-zenoh/zenoh-plugin-dds), который:
- Автоматически конвертирует между Zenoh и DDS
- Правильно обрабатывает типы сообщений
- Поддерживает ROS2 из коробки

### ⚠️ Важно

Даже если исправить режим облачного роутера на `router`, REST API **всё равно не будет работать** для ROS топиков без дополнительного моста или плагина DDS!

---

## ✅ Чеклист

### Исправление облачного роутера
- [ ] Обновить конфигурацию облачного Zenoh роутера
- [ ] Изменить `"mode": "peer"` на `"mode": "router"`
- [ ] Добавить `"peers_failover_brokering": true`
- [ ] Перезапустить облачный роутер
- [ ] Проверить логи на наличие ошибок

### Установка моста (выберите один вариант)

**Вариант A: Python мост (быстро, для тестирования)**
- [ ] Установить `eclipse-zenoh` на роботе: `pip3 install eclipse-zenoh`
- [ ] Скопировать `scripts/zenoh_rest_bridge.py` на робота
- [ ] Запустить мост: `python3 zenoh_rest_bridge.py`
- [ ] (Опционально) Создать systemd сервис для автозапуска

**Вариант B: zenoh-plugin-dds (production)**
- [ ] Установить zenoh-plugin-dds на облачном роутере
- [ ] Настроить плагин в конфигурации
- [ ] Перезапустить роутер

### Тестирование
- [ ] Перезапустить роутер на роботе
- [ ] Запустить скрипт диагностики: `scripts/diagnose_zenoh_cloud.sh`
- [ ] Отправить тестовую команду через REST API
- [ ] Проверить получение команды в логах twist_mux
- [ ] Проверить движение робота
- [ ] Обновить документацию

---

## 🔗 Связанные документы

- [ZENOH_CLOUD_NAMESPACES.md](../architecture/ZENOH_CLOUD_NAMESPACES.md)
- [Zenoh Configuration Reference](https://github.com/eclipse-zenoh/zenoh/blob/main/DEFAULT_CONFIG.json5)
- [rmw_zenoh Design](https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md)

---

## 📌 Резюме

**Основная причина проблемы:** Облачный Zenoh роутер работает в режиме `peer`, который не маршрутизирует сообщения между клиентами.

**Решение:** Изменить режим на `router` и включить `peers_failover_brokering`.

**Ожидаемый результат:** После исправления команды через REST API будут корректно маршрутизироваться к роботу.
