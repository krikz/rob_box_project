# 🔧 Решение проблемы REST API с rmw_zenoh

## 📋 Ситуация

После добавления Zenoh namespace REST API перестал работать с ROS топиками.

## ✅ Хорошие новости!

**Данные УЖЕ идут через облако!** Видны топики: `robots/RBXU100001/0/...`

- ✅ Namespace работает правильно  
- ✅ Domain ID (`/0`) - это нормально (часть формата rmw_zenoh)
- ✅ Данные от робота доходят до облака
- ✅ **Все ноды работают напрямую через rmw_zenoh** (переход с DDS для снижения нагрузки на сеть)

**Проблема:** REST API не может публиковать на сложные ROS ключи с типами.

---

## 🎯 Почему rmw_zenoh?

Проект **специально перешёл с DDS на rmw_zenoh** для:
- ✅ Снижения нагрузки на сеть
- ✅ Более эффективной маршрутизации
- ✅ Лучшей масштабируемости
- ✅ Поддержки облачных роутеров

**Возврат к DDS bridge НЕ рекомендуется** - это вернёт проблемы с нагрузкой на сеть.

---

## ⭐ РЕШЕНИЕ: Python мост REST → ROS

Легковесный мост для конвертации REST API команд в ROS топики.

### Шаг 1: Настроить облачный роутер

**На zenoh.robbox.online:**

```bash
# 1. Остановить Zenoh
sudo systemctl stop zenoh-router

# 2. Скачать конфигурацию
wget https://raw.githubusercontent.com/krikz/rob_box_project/main/docs/cloud/zenoh_router_config.json5 \
  -O /tmp/zenoh_config.json5

# 3. Применить конфигурацию
sudo cp /tmp/zenoh_config.json5 /etc/zenoh/config.json5

# 4. Запустить Zenoh
sudo systemctl start zenoh-router

# 5. Проверить логи
sudo journalctl -u zenoh-router -n 50
# Должно быть: 
# - "mode": "router"
# - peers_failover_brokering: true
```

### Шаг 2: Установить Python мост на роботе

**На роботе (SSH: ros2@10.1.1.20):**

```bash
# 1. Установить zenoh-python
pip3 install eclipse-zenoh

# 2. Скопировать скрипт моста
cd ~/rob_box_project
git pull
cp scripts/zenoh_rest_bridge.py ~/

# 3. Запустить мост
source /opt/ros/humble/setup.bash
python3 ~/zenoh_rest_bridge.py

# Должно появиться:
# ✅ Мост запущен и готов к работе!
# Подписка на Zenoh топик: cmd_vel_web_bridge
# Публикация в ROS топик: cmd_vel_voice
```

### Шаг 3: Тестировать REST API

```bash
# ⭐ ВАЖНО: 
# 1. Namespace ОБЯЗАТЕЛЕН для выбора робота
# 2. Используется специальный топик для моста: cmd_vel_web_bridge
# 3. Формат данных: CDR бинарный (НЕ JSON!)

# Из веб-приложения (TypeScript/JavaScript):
# См. docs/examples/zenoh_rest_client.ts
const cdrData = serializeTwist(0.1, 0.0);  // linear, angular
fetch('http://zenoh.robbox.online/robots/RBXU100001/cmd_vel_web_bridge', {
  method: 'PUT',
  headers: { 'Content-Type': 'application/octet-stream' },
  body: cdrData
});

# Из командной строки (с готовым CDR файлом):
curl -X PUT http://zenoh.robbox.online/robots/RBXU100001/cmd_vel_web_bridge \
  -H "Content-Type: application/octet-stream" \
  --data-binary @twist.cdr
```

**Python мост автоматически:**
- ✅ Получает CDR данные на `cmd_vel_web_bridge`
- ✅ Декодирует CDR → ROS Twist
- ✅ Публикует в ROS топик `cmd_vel_voice` с правильным типом
- ✅ twist_mux получает команду (приоритет 25)
- ✅ Робот движется!

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
