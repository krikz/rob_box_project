# RViz Zenoh Namespace Investigation Report

**Дата:** 2025-11-17  
**Проблема:** RViz не видит топики от роботов через Zenoh  
**Статус:** В процессе диагностики  

---

## 📋 Описание проблемы

RViz, запущенный на локальной машине (Host PC), не может видеть топики ROS 2 от роботов, подключенных через Zenoh router. При запуске `ros2 topic list` видны только локальные топики:

```
/parameter_events
/rosout
```

Топики от Main Pi и Vision Pi (например, `/tf`, `/camera/rgb/image_raw`, `/scan`) **не видны**.

---

## 🏗️ Архитектура системы

### Текущая Zenoh топология

```
┌─────────────────────────────────────────────────────────────────┐
│ Main Pi (10.1.1.20)                                             │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │ Zenoh Router (mode: "router")                               │ │
│ │ - listen: tcp/0.0.0.0:7447                                  │ │
│ │ - connect: tcp/zenoh.robbox.online:7447 (cloud)             │ │
│ └─────────────────────────────────────────────────────────────┘ │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │ ROS 2 Nodes (mode: "peer")                                  │ │
│ │ - rtabmap, nav2, vesc, lslidar, perception                  │ │
│ │ - namespace: "robots/RBXU100001" (добавляется динамически)  │ │
│ └─────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                            ↕ Gigabit Ethernet
┌─────────────────────────────────────────────────────────────────┐
│ Vision Pi (10.1.1.21)                                           │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │ Zenoh Router (mode: "router")                               │ │
│ │ - listen: tcp/0.0.0.0:7447                                  │ │
│ │ - connect: tcp/10.1.1.20:7447 (Main Pi)                     │ │
│ └─────────────────────────────────────────────────────────────┘ │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │ ROS 2 Nodes (mode: "peer")                                  │ │
│ │ - oak-d, apriltag, voice-assistant, led-matrix              │ │
│ │ - namespace: "robots/RBXU100001" (добавляется динамически)  │ │
│ └─────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                            ↕ WiFi
┌─────────────────────────────────────────────────────────────────┐
│ Host PC (10.1.1.5)                                              │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │ Local Zenoh Router (Docker, mode: "router")                 │ │
│ │ - listen: tcp/0.0.0.0:7447                                  │ │
│ │ - connect: tcp/10.1.1.20:7447 (Main Pi)                     │ │
│ │ - connect: tcp/zenoh.robbox.online:7447 (cloud)             │ │
│ └─────────────────────────────────────────────────────────────┘ │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │ RViz2 (rmw_zenoh_cpp)                                       │ │
│ │ - mode: "peer" (предполагается)                            │ │
│ │ - namespace: ??? (ПРОБЛЕМА ЗДЕСЬ!)                         │ │
│ └─────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🔍 Ключевое открытие: Zenoh Namespace

### Как работает namespace в Zenoh

Из исходников Zenoh (`zenoh/DEFAULT_CONFIG.json5`):

```json5
  // /// Namespace prefix.
  // /// If specified, all outgoing key expressions will be
  // /// automatically prefixed with specified string,
  // /// and all incoming key expressions
  // /// will be stripped of specified prefix.
  // /// The namespace prefix should satisfy
  // /// all key expression constraints
  // /// and additionally it can not contain
  // /// wild characters ('*').
  // /// Namespace is applied to the session.
  // /// E. g. if session has a namespace of "1" then 
  // /// session.put("my/keyexpr", my_message),
  // /// will put a message into 1/my/keyexpr. 
  // /// Same applies to all other operations within this session.
  // namespace: "my/namespace",
```

**Принцип работы:**

1. **Namespace = автоматический префикс для всех key expressions**
2. **Outgoing**: Zenoh добавляет префикс ко всем публикуемым топикам
3. **Incoming**: Zenoh стирает префикс со всех получаемых топикам

**Пример:**

```python
# Session с namespace: "robots/RBXU100001"
session.put("tf", data)  # Реально публикуется: "robots/RBXU100001/tf"

# Другая session БЕЗ namespace подписывается:
subscriber.subscribe("robots/RBXU100001/tf")  # НЕ НАЙДЁТ - нужен точный match!

# Другая session С namespace: "robots/RBXU100001"
subscriber.subscribe("tf")  # НАЙДЁТ - Zenoh автоматически добавит префикс
```

### Как namespace добавляется в робота

**Скрипт:** `docker/main/scripts/ros_with_namespace.sh`

```bash
#!/bin/bash
set -e

# Get ROBOT_ID from environment or default
ROBOT_ID=${ROBOT_ID:-RBXU100001}

# Generate session config from template
TEMPLATE_CONFIG="/config/shared/zenoh_session_config.json5"
OUTPUT_CONFIG="/tmp/zenoh_session_config_${ROBOT_ID}.json5"

# Copy template
cp "$TEMPLATE_CONFIG" "$OUTPUT_CONFIG"

# Add namespace to the config (insert after mode line)
if grep -q '"mode": "peer"' "$OUTPUT_CONFIG"; then
  sed -i 's|"mode": "peer",|"mode": "peer",\n  "namespace": "robots/'$ROBOT_ID'",|' "$OUTPUT_CONFIG"
else
  sed -i 's|"mode": "client",|"mode": "client",\n  "namespace": "robots/'$ROBOT_ID'",|' "$OUTPUT_CONFIG"
fi

# Export config path for RMW
export ZENOH_SESSION_CONFIG_URI="$OUTPUT_CONFIG"

# Execute the command with ROS namespace
exec "$@"
```

**Результат:** Все ROS 2 ноды на Main Pi и Vision Pi работают в namespace `robots/RBXU100001`.

---

## 🚨 Диагностированная проблема

### Проблема 1: RViz запускается БЕЗ namespace

**Скрипт:** `scripts/start_rviz.sh`

```bash
# Add namespace to the config (insert after mode line)
# Support both "client" and "peer" modes
if grep -q '"mode": "peer"' "$ZENOH_CONFIG"; then
  sed -i 's|"mode": "peer",|"mode": "peer",\n  "namespace": "robots/'$ROBOT_ID'",|' "$ZENOH_CONFIG"
else
  sed -i 's|"mode": "client",|"mode": "client",\n  "namespace": "robots/'$ROBOT_ID'",|' "$ZENOH_CONFIG"
fi
```

**ОДНАКО:** Проверка показала что скрипт **НЕ СМОГ** добавить namespace!

**Причина:** Регулярное выражение ищет `"mode": "peer"` или `"mode": "client"` **С КАВЫЧКАМИ**, но в реальном конфиге используется **БЕЗ КАВЫЧЕК**:

```json5
mode: "peer",  // ← Без кавычек вокруг "mode"
```

### Проблема 2: Namespace mismatch

**Текущее состояние:**

| Компонент | Namespace | Key Expression | Реальный ключ в Zenoh |
|-----------|-----------|----------------|------------------------|
| Main Pi nodes | `robots/RBXU100001` | `tf` | `robots/RBXU100001/tf` |
| Vision Pi nodes | `robots/RBXU100001` | `camera/rgb/image_raw` | `robots/RBXU100001/camera/rgb/image_raw` |
| RViz | **НЕТ (default)** | `tf` | `tf` ← НЕ СОВПАДАЕТ! |
| RViz | **НЕТ (default)** | `camera/rgb/image_raw` | `camera/rgb/image_raw` ← НЕ СОВПАДАЕТ! |

**Результат:** RViz подписывается на `tf`, но реально топик называется `robots/RBXU100001/tf` → **NAMESPACE MISMATCH** → топики не видны.

---

## 📊 Логи и диагностика

### Лог 1: Local Zenoh Router подключён к Main Pi

**Команда:**
```bash
docker logs zenoh-router-local 2>&1 | tail -50 | grep -E "Face\{[0-9]" | sort | uniq
```

**Результат:**
```
2025-11-14T16:58:30.563027Z DEBUG  rx-1 ThreadId(07) zenoh::net::routing::dispatcher::token: Face{5, 842a12d3567f3182d1d39c240171d32d} Undeclare token 0 (robots/RBXU100001/@ros2_lv/0/.../0/0/NN/%/%/_ros2cli_173)
2025-11-14T16:58:30.563225Z DEBUG  rx-1 ThreadId(07) zenoh::net::routing::dispatcher::token: Face{5, 842a12d3567f3182d1d39c240171d32d} Undeclare unknown token 478
2025-11-14T16:58:54.790267Z DEBUG  rx-1 ThreadId(07) zenoh::net::routing::dispatcher::pubsub: Face{5, 842a12d3567f3182d1d39c240171d32d} Declare subscriber 0 (@/2465358d915f9ac264a51fc298a9bfc1/peer/config/**)
```

**Вывод:**
- ✅ **Face{5}** существует - это Main Pi (UUID: `842a12d3567f3182d1d39c240171d32d`)
- ✅ Топики видны с префиксом **`robots/RBXU100001/...`**
- ✅ Сетевое подключение работает

### Лог 2: ros2 topic list с rmw_zenoh_cpp

**Команда:**
```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI=/home/ros2/rob_box_project/local_test/zenoh_local_session_minimal.json5
ros2 topic list
```

**Результат:**
```
/parameter_events
/rosout
```

**Вывод:**
- ❌ Только локальные топики
- ❌ Топики от Main Pi (`robots/RBXU100001/tf`, `robots/RBXU100001/scan`) **НЕ ВИДНЫ**
- ❌ Namespace mismatch

### Лог 3: Проверка zenoh_rviz_config_RBXU100001.json5

**Команда:**
```bash
cat /tmp/zenoh_rviz_config_RBXU100001.json5 | grep -A2 -B2 namespace
```

**Ожидалось:**
```json5
mode: "peer",
namespace: "robots/RBXU100001",
```

**Реально:**
```json5
mode: "peer",
// namespace: "my/namespace",  ← ЗАКОММЕНТИРОВАНО!
```

**Вывод:**
- ❌ Скрипт `start_rviz.sh` **НЕ СМОГ** раскомментировать и заменить namespace
- ❌ Регулярное выражение `sed` не сработало из-за неправильного pattern matching

---

## 🔧 Корневая причина

### 1. Неправильный sed pattern в start_rviz.sh

**Текущий код:**
```bash
if grep -q '"mode": "peer"' "$ZENOH_CONFIG"; then
  sed -i 's|"mode": "peer",|"mode": "peer",\n  "namespace": "robots/'$ROBOT_ID'",|' "$ZENOH_CONFIG"
```

**Проблема:** Ищет `"mode": "peer"` (с кавычками вокруг `mode`), но в реальном конфиге:
```json5
mode: "peer",  // ← БЕЗ кавычек вокруг mode
```

**Решение:** Исправить pattern:
```bash
if grep -q 'mode: "peer"' "$ZENOH_CONFIG"; then
  sed -i 's|mode: "peer",|mode: "peer",\n  namespace: "robots/'$ROBOT_ID'",|' "$ZENOH_CONFIG"
```

### 2. Namespace должен быть консистентным

Все компоненты системы должны использовать **ОДИН И ТОТ ЖЕ namespace**:

| Компонент | Должен быть namespace |
|-----------|-----------------------|
| Main Pi ROS nodes | `robots/RBXU100001` ✅ |
| Vision Pi ROS nodes | `robots/RBXU100001` ✅ |
| **RViz** | `robots/RBXU100001` ❌ (сейчас: нет) |
| **ros2 topic list** | `robots/RBXU100001` ❌ (сейчас: нет) |

---

## ✅ Предлагаемые решения

### Решение 1: Исправить start_rviz.sh (БЫСТРОЕ)

**Файл:** `scripts/start_rviz.sh`

**Изменения:**

```bash
# БЫЛО (НЕ РАБОТАЕТ):
if grep -q '"mode": "peer"' "$ZENOH_CONFIG"; then
  sed -i 's|"mode": "peer",|"mode": "peer",\n  "namespace": "robots/'$ROBOT_ID'",|' "$ZENOH_CONFIG"
else
  sed -i 's|"mode": "client",|"mode": "client",\n  "namespace": "robots/'$ROBOT_ID'",|' "$ZENOH_CONFIG"
fi

# СТАЛО (РАБОТАЕТ):
if grep -q 'mode: "peer"' "$ZENOH_CONFIG"; then
  sed -i 's|mode: "peer",|mode: "peer",\n  namespace: "robots/'$ROBOT_ID'",|' "$ZENOH_CONFIG"
else
  sed -i 's|mode: "client",|mode: "client",\n  namespace: "robots/'$ROBOT_ID'",|' "$ZENOH_CONFIG"
fi
```

**Альтернатива (более надёжная - раскомментировать существующую строку):**

```bash
# Раскомментировать и заменить namespace строку
sed -i 's|// namespace: "my/namespace",|namespace: "robots/'$ROBOT_ID'",|' "$ZENOH_CONFIG"
```

### Решение 2: Создать отдельный скрипт ros_with_namespace.sh для Host PC

**Файл:** `scripts/ros_with_namespace.sh` (новый)

```bash
#!/bin/bash
set -e

# Get ROBOT_ID from environment or default
ROBOT_ID=${ROBOT_ID:-RBXU100001}

# Generate session config from template
TEMPLATE_CONFIG="$HOME/rob_box_project/docker/main/config/zenoh_session_config.json5"
OUTPUT_CONFIG="/tmp/zenoh_session_config_${ROBOT_ID}.json5"

# Copy template
cp "$TEMPLATE_CONFIG" "$OUTPUT_CONFIG"

# Uncomment and replace namespace
sed -i 's|// namespace: "my/namespace",|namespace: "robots/'$ROBOT_ID'",|' "$OUTPUT_CONFIG"

# Export config path for RMW
export ZENOH_SESSION_CONFIG_URI="$OUTPUT_CONFIG"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Execute the command
exec "$@"
```

**Использование:**
```bash
./scripts/ros_with_namespace.sh ros2 topic list
./scripts/ros_with_namespace.sh rviz2
```

### Решение 3: Добавить namespace в environment переменные (РЕКОМЕНДУЕТСЯ)

**Создать:** `scripts/setup_ros2_zenoh_env.sh`

```bash
#!/bin/bash
# Source this file before running ROS 2 commands

export ROBOT_ID=${ROBOT_ID:-RBXU100001}
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Generate Zenoh session config with namespace
TEMPLATE_CONFIG="$HOME/rob_box_project/docker/main/config/zenoh_session_config.json5"
OUTPUT_CONFIG="/tmp/zenoh_session_config_${ROBOT_ID}.json5"

cp "$TEMPLATE_CONFIG" "$OUTPUT_CONFIG"
sed -i 's|// namespace: "my/namespace",|namespace: "robots/'$ROBOT_ID'",|' "$OUTPUT_CONFIG"

export ZENOH_SESSION_CONFIG_URI="$OUTPUT_CONFIG"

echo "✅ ROS 2 environment configured:"
echo "   ROBOT_ID=$ROBOT_ID"
echo "   RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "   ZENOH_SESSION_CONFIG_URI=$ZENOH_SESSION_CONFIG_URI"
echo ""
echo "You can now run:"
echo "  ros2 topic list"
echo "  rviz2"
```

**Использование:**
```bash
source scripts/setup_ros2_zenoh_env.sh
ros2 topic list  # Теперь видит топики с namespace
rviz2            # Теперь видит топики от роботов
```

---

## 🧪 План тестирования

### Тест 1: Проверка namespace в конфиге

```bash
# 1. Запустить исправленный start_rviz.sh
./scripts/start_rviz.sh

# 2. Проверить что namespace добавлен
cat /tmp/zenoh_rviz_config_RBXU100001.json5 | grep namespace

# Ожидается:
# namespace: "robots/RBXU100001",
```

### Тест 2: Проверка видимости топиков

```bash
# 1. Source окружение с namespace
source scripts/setup_ros2_zenoh_env.sh

# 2. Список топиков
ros2 topic list

# Ожидается:
# /camera/color/image_raw
# /camera/depth/image_rect_raw
# /scan
# /tf
# /tf_static
# ... (топики БЕЗ префикса robots/RBXU100001, Zenoh автоматически стирает)
```

### Тест 3: RViz видит данные

```bash
# 1. Запустить RViz с namespace
source scripts/setup_ros2_zenoh_env.sh
rviz2

# 2. В RViz добавить:
#    - TF display
#    - Image display для /camera/color/image_raw
#    - LaserScan display для /scan

# Ожидается:
# ✅ TF дерево отображается
# ✅ Изображение с камеры видно
# ✅ LiDAR сканы видны
```

---

## 📝 Чеклист для Pull Request

- [ ] Исправить `scripts/start_rviz.sh` - убрать кавычки в sed pattern
- [ ] Создать `scripts/setup_ros2_zenoh_env.sh` для настройки окружения
- [ ] Обновить `docs/guides/RVIZ_SETUP.md` с инструкциями по namespace
- [ ] Добавить проверку namespace в `scripts/diagnose.sh`
- [ ] Протестировать на Host PC:
  - [ ] `ros2 topic list` видит топики от роботов
  - [ ] RViz отображает TF, камеру, LiDAR
  - [ ] Нет ошибок в логах
- [ ] Обновить `README.md` с примером использования
- [ ] Добавить этот отчет в `docs/fixes/RVIZ_ZENOH_NAMESPACE_FIX.md`

---

## 📚 Полезные ссылки

- **Zenoh DEFAULT_CONFIG.json5:** https://github.com/eclipse-zenoh/zenoh/blob/main/DEFAULT_CONFIG.json5
- **Zenoh Documentation:** https://zenoh.io/docs/
- **RMW Zenoh Documentation:** https://github.com/ros2/rmw_zenoh
- **Issue:** [GitHub Issue link here]

---

## 🔗 Связанные документы

- `docs/architecture/SYSTEM_OVERVIEW.md` - Архитектура Zenoh
- `docs/guides/TROUBLESHOOTING.md` - Общая диагностика
- `ZENOH_FIX_SUMMARY_2025-11-10.md` - Предыдущие исправления Zenoh
- `docker/main/scripts/ros_with_namespace.sh` - Скрипт добавления namespace (робот)
- `scripts/start_rviz.sh` - Скрипт запуска RViz (Host PC)

---

**Автор:** AI Agent (Claude Sonnet 4)  
**Дата создания:** 2025-11-17  
**Последнее обновление:** 2025-11-17  
**Версия:** 1.0
