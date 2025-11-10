# 🔧 Краткое руководство: Возврат к рабочей конфигурации

## 📋 Ситуация

После добавления Zenoh namespace REST API перестал работать с ROS топиками.

## ✅ Хорошие новости!

**Данные УЖЕ идут через облако!** Видны топики: `robots/RBXU100001/0/...`

- ✅ Namespace работает правильно  
- ✅ Domain ID (`/0`) - это нормально (часть формата rmw_zenoh)
- ✅ Данные от робота доходят до облака

**Проблема:** REST API не может публиковать на сложные ROS ключи с типами.

---

## ⭐ РЕШЕНИЕ: Вернуть DDS Bridge

**Это работало раньше!** DDS плагин автоматически транслирует между REST/Zenoh и ROS.

### Шаг 1: Настроить облачный роутер

**На zenoh.robbox.online:**

```bash
# 1. Остановить Zenoh
sudo systemctl stop zenoh-router

# 2. Установить DDS плагин (если ещё не установлен)
cargo install zenoh-plugin-dds

# 3. Скачать конфигурацию с DDS плагином
wget https://raw.githubusercontent.com/krikz/rob_box_project/main/docs/cloud/zenoh_router_config.json5 \
  -O /tmp/zenoh_config.json5

# 4. Раскомментировать секцию DDS
sed -i 's|^    // dds:|    dds:|' /tmp/zenoh_config.json5
sed -i 's|^    //   |      |' /tmp/zenoh_config.json5

# Или вручную:
nano /tmp/zenoh_config.json5
# Найти секцию "// dds:" и убрать комментарии

# 5. Применить конфигурацию
sudo cp /tmp/zenoh_config.json5 /etc/zenoh/config.json5

# 6. Запустить Zenoh
sudo systemctl start zenoh-router

# 7. Проверить логи
sudo journalctl -u zenoh-router -n 50
# Должно быть: 
# - "mode": "router"
# - DDS plugin loaded
```

### Шаг 2: Проверить конфигурацию

```bash
# Режим должен быть router
curl -s http://localhost:8000/@/router/config | grep -o '"mode":"[^"]*"'
# Вывод: "mode":"router"

# DDS плагин должен быть активен
curl -s http://localhost:8000/@/router/status | grep -i dds
```

### Шаг 3: Тестировать REST API

```bash
# ⭐ ВАЖНО: Namespace ОБЯЗАТЕЛЕН для выбора робота!
# Формат: http://zenoh.robbox.online/robots/{ROBOT_ID}/{topic}

# Для робота RBXU100001:
curl -X PUT http://zenoh.robbox.online/robots/RBXU100001/cmd_vel_voice \
  -H "Content-Type: application/json" \
  -d '{"linear":{"x":0.1,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}'

# Для веб команд:
curl -X PUT http://zenoh.robbox.online/robots/RBXU100001/cmd_vel_web \
  -H "Content-Type: application/json" \
  -d '{"linear":{"x":0.1},"angular":{"z":0.0}}'

# Для другого робота (например, RBXU100002):
curl -X PUT http://zenoh.robbox.online/robots/RBXU100002/cmd_vel_voice \
  -H "Content-Type: application/json" \
  -d '{"linear":{"x":0.1},"angular":{"z":0.0}}'
```

**DDS плагин автоматически:**
- ✅ Принимает команду с namespace (выбор робота)
- ✅ Конвертирует JSON → ROS Twist
- ✅ Добавляет правильные типы сообщений и domain
- ✅ Публикует в DDS/ROS на конкретном роботе
- ✅ Только выбранный робот получает команду!

---

## 📝 Конфигурация DDS плагина

В `/etc/zenoh/config.json5` должно быть:

```json5
plugins: {
  rest: {
    http_port: 8000,
  },
  
  // ⭐ DDS ПЛАГИН - раскомментируйте это:
  dds: {
    domain: 0,           // ROS_DOMAIN_ID
    mode: "peer",        // Режим DDS
    allow: {
      topics: [
        "cmd_vel_voice",  // Голосовые команды
        "cmd_vel_web",    // Веб команды
        "cmd_vel_joy",    // Джойстик
      ]
    }
  },
  
  storage_manager: {
    // ...
  }
}
```

### ⚠️ Важно про Namespace

**Namespace сохраняется!** DDS плагин работает **ВНУТРИ** namespace каждого робота.

REST API путь: `http://zenoh.robbox.online/robots/{ROBOT_ID}/{topic}`

- `robots/{ROBOT_ID}` - выбор конкретного робота (namespace)
- `{topic}` - ROS топик (например, `cmd_vel_voice`)

DDS плагин автоматически:
1. Получает команду с путём `robots/RBXU100001/cmd_vel_voice`
2. Добавляет domain (`0`), тип сообщения и хаш
3. Публикует в ROS на **конкретном роботе**

**Каждый робот изолирован через свой namespace!**

---

## 🔍 Диагностика

### Проблема: DDS плагин не загружается

```bash
# Проверить установлен ли плагин
ls /usr/local/lib/libzenoh_plugin_dds.so
# или
cargo install --list | grep zenoh-plugin-dds

# Переустановить
cargo install --force zenoh-plugin-dds
```

### Проблема: Команды всё ещё не доходят

```bash
# На роботе проверить логи twist_mux
docker logs -f twist-mux

# Должно появиться получение команды на cmd_vel_voice
```

### Проблема: Нужен отладочный вывод

```bash
# Запустить Zenoh с debug логами
RUST_LOG=zenoh=debug,zenoh_plugin_dds=debug zenohd -c /etc/zenoh/config.json5
```

---

## 🎯 Чем это лучше Python моста

| Параметр | DDS Bridge | Python мост |
|----------|------------|-------------|
| **Работало раньше** | ✅ Да | ❌ Нет |
| **Производительность** | ✅ Высокая | ⚠️ Средняя |
| **Простота REST API** | ✅ Простые ключи | ⚠️ Нужен мост |
| **Автоматическая конвертация** | ✅ Да | ⚠️ Ручная |
| **Типы сообщений** | ✅ Автоматически | ⚠️ Только Twist |
| **Поддержка** | ✅ Официальный плагин | ⚠️ Кастомный скрипт |

---

## 📖 Дополнительно

- **Полная документация:** `docs/cloud/README.md`
- **Анализ проблемы:** `docs/reports/ZENOH_CLOUD_CONFIG_ISSUE_2025-11-10.md`
- **Про Domain ID:** `docs/reports/ZENOH_NAMESPACE_AND_DOMAIN_EXPLANATION.md`
- **Zenoh-plugin-dds:** https://github.com/eclipse-zenoh/zenoh-plugin-dds

---

**Готово!** После применения REST API будет работать как раньше! 🎉
