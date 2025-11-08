# Fix: RViz не видит топики робота - решение проблемы namespace

## Проблема
RViz подключается к Zenoh router, но не видит топики робота. Показывает только локальные топики `/parameter_events` и `/rosout`.

## Причина
**Namespace изоляция!**

- Робот публикует топики с namespace: `robots/RBXU100001/*`
- RViz подключался БЕЗ namespace и не видел топики робота

## Архитектура namespace в Zenoh

```
┌─────────────────────────────────────────────────────────┐
│ Zenoh Router (no namespace)                             │
│ - Просто маршрутизирует весь трафик                     │
│ - НЕ применяет namespace                                │
└────────┬────────────────────────────────────────────────┘
         │
         ├──────────────────────────────────┐
         │                                  │
┌────────▼─────────┐              ┌─────────▼──────────┐
│ ROS Nodes        │              │ RViz Client        │
│ (session config) │              │ (client config)    │
│                  │              │                    │
│ namespace:       │              │ namespace:         │
│ "robots/        │              │ "robots/           │
│  RBXU100001"     │              │  RBXU100001"       │
│                  │              │                    │
│ Publish:         │              │ Subscribe:         │
│ /cmd_vel         │              │ /cmd_vel           │
│    ↓             │              │    ↓               │
│ robots/          │◄─────────────┤ robots/            │
│ RBXU100001/      │   sees!      │ RBXU100001/        │
│ cmd_vel          │              │ cmd_vel            │
└──────────────────┘              └────────────────────┘
```

## Решение

Обновлён скрипт `scripts/start_rviz.sh`:

1. **Автоматическое определение ROBOT_ID** (по умолчанию `RBXU100001`)
2. **Генерация Zenoh client config с namespace**
3. **Инъекция namespace в config**: `"namespace": "robots/RBXU100001"`

### Что делает скрипт

```bash
# Берёт ROBOT_ID из окружения или дефолт
ROBOT_ID="${ROBOT_ID:-RBXU100001}"

# Копирует шаблон config
cp local_test/zenoh_client_config.json5 /tmp/zenoh_rviz_config_${ROBOT_ID}.json5

# Добавляет namespace в config
sed -i 's|"mode": "client",|"mode": "client",\n  "namespace": "robots/'$ROBOT_ID'",|' /tmp/zenoh_rviz_config_${ROBOT_ID}.json5
```

### Результат

**До:**
```json
{
  "mode": "client",
  "connect": {
    "endpoints": ["tcp/10.1.1.10:7447"]
  }
}
```

**После:**
```json
{
  "mode": "client",
  "namespace": "robots/RBXU100001",
  "connect": {
    "endpoints": ["tcp/10.1.1.10:7447"]
  }
}
```

## Тестирование

```bash
# Запуск RViz с правильным namespace
cd /home/ros2/rob_box_project
./scripts/start_rviz.sh

# Проверка топиков
ros2 topic list
# Теперь должны быть видны:
# /camera/rgb/image_raw
# /scan
# /cmd_vel
# /tf
# и т.д.
```

## Дополнительно

Если нужен другой ROBOT_ID:

```bash
ROBOT_ID=MY_CUSTOM_ID ./scripts/start_rviz.sh
```

## Файлы изменены

- `scripts/start_rviz.sh` - добавлена генерация config с namespace

## Связанные скрипты на роботе

- `docker/main/scripts/generate_zenoh_session_config.sh` - генерация session config
- `docker/main/scripts/ros_with_namespace.sh` - универсальная обёртка для ROS нод
- `docker/main/scripts/zenoh-router/start_zenoh_router.sh` - запуск router БЕЗ namespace

## Ключевые моменты

✅ **Zenoh Router** - БЕЗ namespace (просто маршрутизирует)  
✅ **Session config** (ROS nodes) - С namespace `robots/RBXU100001`  
✅ **Client config** (RViz) - С тем же namespace для видимости топиков  
✅ **Namespace формат** - `robots/{ROBOT_ID}`

---

**Дата:** 2025-11-08  
**Коммит:** Добавлен namespace в RViz client config для видимости топиков робота
