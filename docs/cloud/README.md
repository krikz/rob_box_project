# Конфигурация облачного Zenoh роутера

Этот документ описывает правильную конфигурацию Zenoh роутера для облачного развертывания на `zenoh.robbox.online`.

---

## 🔴 Проблема

Команды через веб API не доходят до робота из-за **неправильного режима работы** облачного Zenoh роутера.

**Симптомы:**
- PUT запросы через REST API возвращают 200 OK
- Робот подписан на топик (видно в логах twist_mux)
- Сообщения НЕ доходят до робота

**Причина:** Облачный роутер работает в режиме `peer` вместо `router`.

---

## ✅ Решение

### Быстрое исправление

**На сервере zenoh.robbox.online:**

```bash
# 1. Остановить Zenoh
sudo systemctl stop zenoh-router
# или
docker stop zenoh-router

# 2. Создать резервную копию
sudo cp /etc/zenoh/config.json5 /etc/zenoh/config.json5.backup

# 3. Скачать правильную конфигурацию
wget https://raw.githubusercontent.com/krikz/rob_box_project/main/docs/cloud/zenoh_router_config.json5 \
     -O /etc/zenoh/config.json5

# 4. Проверить конфигурацию
cat /etc/zenoh/config.json5 | grep '"mode"'
# Должно быть: mode: "router",

# 5. Запустить Zenoh
sudo systemctl start zenoh-router
# или
docker start zenoh-router

# 6. Проверить логи
sudo journalctl -u zenoh-router -f
# или
docker logs -f zenoh-router
```

### Ключевые изменения

| Параметр | Было | Стало | Почему |
|----------|------|-------|--------|
| `mode` | `"peer"` | `"router"` | ✅ Полная маршрутизация между клиентами |
| `peers_failover_brokering` | отсутствует | `true` | ✅ Роутер пересылает между peers |
| `scouting.multicast.enabled` | `true` | `false` | ✅ Не нужно в облаке |

---

## 📊 Как это работает

### До исправления (режим peer)

```
Веб API                         Робот
   │                               │
   │ PUT /robots/.../cmd_vel       │
   ↓                               │
Zenoh (peer)                      │
   │                               │
   ❌ НЕ маршрутизирует            │
      к другому peer               │
```

**Результат:** Сообщение теряется

### После исправления (режим router)

```
Веб API                         Робот
   │                               │
   │ PUT /robots/.../cmd_vel       │ Подписан
   ↓                               ↑
Zenoh (router)                    │
   │                               │
   ✅ Маршрутизирует               │
      к подписчикам                │
```

**Результат:** Сообщение доставлено ✓

---

## 🧪 Тестирование

### 1. Проверка режима работы

```bash
# Через REST API
curl http://zenoh.robbox.online:8000/@/router/config | jq '.mode'
# Должно быть: "router"
```

### 2. Тест отправки команды

```bash
# Отправить тестовую команду (Twist message)
# Создать файл twist.json с командой
cat > twist.json << 'EOF'
{
  "linear": {"x": 0.1, "y": 0.0, "z": 0.0},
  "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
}
EOF

# Отправить через REST API
curl -X PUT https://zenoh.robbox.online/robots/RBXU100001/0/cmd_vel_voice \
  -H "Content-Type: application/json" \
  -d @twist.json

# На роботе проверить логи
ssh ros2@10.1.1.20
docker logs -f twist-mux
# Должно появиться сообщение о получении команды
```

### 3. Проверка подписчиков

```bash
# Запросить список активных подписчиков
curl http://zenoh.robbox.online:8000/@/router/subscribers | jq

# Должны быть видны роботы подписанные на robots/**
```

---

## 📝 Детали конфигурации

### Режим "router" vs "peer"

**Router mode:**
- ✅ Полная маршрутизация между всеми клиентами
- ✅ Поддерживает failover brokering
- ✅ Оптимизирован для централизованных топологий
- ✅ **Рекомендуется для облачных узлов**

**Peer mode:**
- ⚠️ Peer-to-peer коммуникация
- ⚠️ Ограниченная маршрутизация
- ⚠️ Подходит для распределенных сетей
- ❌ **НЕ подходит для централизованных роутеров**

### Failover Brokering

```json5
routing: {
  router: {
    peers_failover_brokering: true
  }
}
```

**Что делает:**
- Роутер обнаруживает, что два peer'а не соединены напрямую
- Автоматически пересылает сообщения между ними
- **Критично для работы REST API → Robot коммуникации**

### Storage Manager

```json5
storage_manager: {
  storages: {
    robot_data: {
      key_expr: "robots/**",
      volume: { id: "memory" }
    }
  }
}
```

**Что делает:**
- ✅ Сохраняет последнее значение каждого ключа
- ✅ Позволяет GET запросы для получения истории
- ❌ **НЕ пересылает** PUT запросы в реальном времени

**Важно:** Storage - это дополнительная функция. Основная маршрутизация выполняется режимом `router`.

---

## 🔍 Диагностика

### Проблема: Команды всё ещё не доходят

**Проверить:**

1. **Режим роутера:**
   ```bash
   curl http://zenoh.robbox.online:8000/@/router/config | jq '.mode'
   ```
   Должно быть `"router"`

2. **Подключение робота:**
   ```bash
   curl http://zenoh.robbox.online:8000/@/router/sessions | jq
   ```
   Должна быть сессия от IP робота

3. **Подписки робота:**
   ```bash
   curl http://zenoh.robbox.online:8000/@/router/subscribers | jq
   ```
   Должны быть подписки на `robots/RBXU100001/**`

4. **Логи роутера на роботе:**
   ```bash
   ssh ros2@10.1.1.20
   docker logs zenoh-router | grep -i "connect\|cloud"
   ```
   Должно быть успешное подключение к облаку

### Проблема: Неправильный формат сообщения

ROS топики через Zenoh используют бинарный формат CDR (Common Data Representation).

**Правильный способ отправки:**

```bash
# Использовать ROS инструменты для сериализации
ros2 topic pub --once /cmd_vel_voice geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}"
```

**Альтернатива для REST API:**
- Использовать Zenoh bridge для конвертации JSON → CDR
- Или создать отдельный сервис-мост на роботе

---

## 🔗 Связанные документы

- [ZENOH_CLOUD_CONFIG_ISSUE_2025-11-10.md](../reports/ZENOH_CLOUD_CONFIG_ISSUE_2025-11-10.md) - Детальный анализ проблемы
- [ZENOH_CLOUD_NAMESPACES.md](../architecture/ZENOH_CLOUD_NAMESPACES.md) - Документация по namespace
- [Zenoh Documentation](https://zenoh.io/docs/) - Официальная документация

---

## ✅ Чеклист применения

- [ ] Остановить Zenoh роутер на облаке
- [ ] Создать резервную копию текущей конфигурации
- [ ] Применить новую конфигурацию
- [ ] Проверить что `mode: "router"`
- [ ] Проверить что `peers_failover_brokering: true`
- [ ] Запустить Zenoh роутер
- [ ] Проверить логи на наличие ошибок
- [ ] Перезапустить Zenoh роутер на роботе
- [ ] Протестировать отправку команд через REST API
- [ ] Проверить получение команд в логах twist_mux
- [ ] Обновить документацию развертывания

---

**Последнее обновление:** 2025-11-10  
**Автор:** AI Agent (GitHub Copilot)
