# RViz Zenoh Critical Configuration Fix

**Date**: 2025-11-19  
**Status**: ✅ **CRITICAL FIX IDENTIFIED**  
**Issue**: RViz не может подключиться к локальному Zenoh роутеру

## 🔴 КРИТИЧНЫЕ НАХОДКИ

### 1. Использовать `ZENOH_SESSION_CONFIG_URI` вместо `ZENOH_CONFIG`

**Проблема**: RViz запускался с переменной `ZENOH_CONFIG`, но rmw_zenoh_cpp её **НЕ ЧИТАЛ**.

**Решение**: На роботе (Main Pi и Vision Pi) используется `ZENOH_SESSION_CONFIG_URI`:

```bash
# ❌ НЕ РАБОТАЕТ
export ZENOH_CONFIG=/tmp/zenoh_rviz_config_RBXU100001.json5

# ✅ РАБОТАЕТ (как на роботе)
export ZENOH_SESSION_CONFIG_URI=/tmp/zenoh_rviz_config_RBXU100001.json5
```

**Подтверждение с робота**:
```bash
# Vision Pi - oak-d контейнер
$ docker exec oak-d env | grep ZENOH
ZENOH_SESSION_CONFIG_URI=/tmp/zenoh_session_config.json5
ZENOH_ROUTER_CHECK_ATTEMPTS=10

# Main Pi - rtabmap контейнер  
$ docker exec rtabmap env | grep ZENOH
ZENOH_SESSION_CONFIG_URI=/tmp/zenoh_session_config.json5
ZENOH_ROUTER_CHECK_ATTEMPTS=10
```

### 2. Использовать mode: "peer" вместо mode: "client"

**Проблема**: Начальная конфигурация использовала `mode: "client"`, но это **НЕ РАБОТАЕТ** для подключения к локальному роутеру.

**Решение**: Vision Pi и все ROS ноды используют `mode: "peer"`:

```json5
// ❌ НЕ РАБОТАЕТ
{
  "mode": "client",
  "connect": {
    "endpoints": ["tcp/10.1.1.249:7447"]
  }
}

// ✅ РАБОТАЕТ (как на Vision Pi)
{
  "mode": "peer",
  "connect": {
    "endpoints": ["tcp/10.1.1.249:7447"]
  },
  "listen": {
    "endpoints": ["tcp/localhost:0"]
  }
}
```

**Подтверждение с Vision Pi** (`/tmp/zenoh_session_config.json5`):
```json5
{
  mode: "peer",  // ✅ НЕ client!
  connect: {
    endpoints: ["tcp/10.1.1.11:7447"],  // Подключается к ЛОКАЛЬНОМУ роутеру
  },
  listen: {
    endpoints: ["tcp/localhost:0"],
  },
  namespace: "robots/RBXU100001",
  // ... остальная конфигурация
}
```

### 3. Gossip scouting configuration

**Важно**: На Vision Pi используется специфичная конфигурация gossip:

```json5
"gossip": {
  "enabled": true,
  "target": {
    "peer": ["router"]  // ✅ Отправлять gossip только роутеру
  },
  "autoconnect": {
    "peer": ["router", "peer"]
  }
}
```

Это минимизирует трафик между peer-нодами при старте.

### 4. Listen на localhost, НЕ на всех интерфейсах

**Vision Pi конфигурация**:
```json5
"listen": {
  "endpoints": ["tcp/localhost:0"],  // ✅ Только localhost
}
```

**Не использовать**:
```json5
"listen": {
  "endpoints": ["tcp/[::]:0"],  // ❌ Слушает на всех интерфейсах
}
```

**Причина**: ROS ноды коммуницируют только с локальным роутером через localhost. Весь внешний трафик идёт через роутер.

## 📝 Правильная конфигурация для RViz

### Template: `local_test/zenoh_client_config.json5`

```json5
{
  "mode": "peer",
  "connect": {
    "endpoints": ["tcp/10.1.1.249:7447"]
  },
  "listen": {
    "endpoints": ["tcp/localhost:0"]
  },
  "scouting": {
    "multicast": {
      "enabled": false
    },
    "gossip": {
      "enabled": true,
      "target": {
        "peer": ["router"]
      },
      "autoconnect": {
        "peer": ["router", "peer"]
      }
    }
  }
}
```

**Примечание**: `namespace` добавляется автоматически скриптом `start_rviz.sh`.

### Script: `scripts/start_rviz.sh`

```bash
# Set environment variables
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI="$ZENOH_CONFIG"  # ✅ НЕ ZENOH_CONFIG!
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ZENOH_ROUTER_CHECK_ATTEMPTS=30
export RUST_LOG=zenoh=warn
export LD_LIBRARY_PATH=/opt/ros/kilted/opt/zenoh_cpp_vendor/lib:/opt/ros/kilted/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}

# CRITICAL FIX: Prioritize ROS Ogre vendor libraries over system libraries
export LD_LIBRARY_PATH="/opt/ros/kilted/opt/rviz_ogre_vendor/lib:${LD_LIBRARY_PATH}"
```

## 🐛 Проблема с Ogre библиотеками (побочная)

После обновления пакетов RViz возникла несовместимость библиотек Ogre:

### Проблема
```bash
rviz2: error while loading shared libraries: libOgreMain.so.1.12.1: cannot open shared object file
```

### Решение
```bash
# Удалить системную версию Ogre 1.12.10
sudo apt remove --purge libogre-1.12-dev libogre1.12.10 -y

# RViz будет использовать свою версию из /opt/ros/kilted/opt/rviz_ogre_vendor/lib/
```

**Важно**: Скрипт `start_rviz.sh` уже приоритизирует ROS Ogre vendor библиотеки через `LD_LIBRARY_PATH`.

## 🔄 Сравнение конфигураций

| Параметр | ❌ Неправильно (было) | ✅ Правильно (стало) | Источник |
|----------|----------------------|---------------------|----------|
| **Переменная окружения** | `ZENOH_CONFIG` | `ZENOH_SESSION_CONFIG_URI` | Vision Pi / Main Pi |
| **Mode** | `client` | `peer` | Vision Pi конфиг |
| **Listen endpoint** | `tcp/[::]:0` | `tcp/localhost:0` | Vision Pi конфиг |
| **Gossip target** | Не указано | `{"peer": ["router"]}` | Vision Pi конфиг |
| **Connect endpoint** | `tcp/10.1.1.249:7447` | `tcp/10.1.1.249:7447` | ✅ Было правильно |
| **Namespace** | `robots/RBXU100001` | `robots/RBXU100001` | ✅ Было правильно |

## 📚 Файлы изменены

1. ✅ `local_test/zenoh_client_config.json5` - изменён mode на "peer", listen на localhost, добавлен gossip target
2. ✅ `scripts/start_rviz.sh` - изменена переменная на `ZENOH_SESSION_CONFIG_URI`

## 🧪 Тестирование

### До исправления
```bash
$ bash scripts/start_rviz.sh
[WARN] [rmw_zenoh_cpp]: Unable to connect to a Zenoh router.
[WARN] [rmw_zenoh_cpp]: Unable to connect to a Zenoh router.
# ... бесконечные попытки, RViz не видит топики
```

### После исправления
```bash
$ bash scripts/start_rviz.sh
# RViz запускается и успешно подключается к Zenoh роутеру
# Топики от робота видны в списке
```

## 🎯 Выводы

1. **rmw_zenoh_cpp читает ТОЛЬКО `ZENOH_SESSION_CONFIG_URI`**, не `ZENOH_CONFIG`
2. **Mode "peer" обязателен** для работы с локальным роутером
3. **Listen на localhost** минимизирует трафик между нодами
4. **Gossip target на router** избегает избыточного трафика при старте
5. **Конфигурация должна совпадать** с той, что используется на роботе (Vision Pi / Main Pi)

## 📖 Связанная документация

- `docs/fixes/RVIZ_ZENOH_FIX_FINAL.md` - предыдущая версия фикса (частично неверная)
- `docs/architecture/ZENOH_CLOUD_NAMESPACES.md` - документация по namespace
- `docker/vision/config/zenoh_session_config.json5` - reference конфигурация Vision Pi
- `docker/main/config/zenoh_session_config.json5` - reference конфигурация Main Pi

## ⚠️ ВАЖНО

При обновлении конфигурации Zenoh на роботе (Main Pi / Vision Pi) **ОБЯЗАТЕЛЬНО** синхронизировать изменения с `local_test/zenoh_client_config.json5`!

---
**Автор**: AI Agent  
**Дата**: 2025-11-19  
**Версия rmw_zenoh_cpp**: 0.1.6-1jammy.20250910.193535
