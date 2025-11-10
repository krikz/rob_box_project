# Объяснение: Zenoh Namespace + ROS Domain ID

**Дата:** 2025-11-10  
**Вопрос:** Откуда взялся `/0` (domain) в топиках после добавления namespace?

---

## 🔍 Краткий ответ

**`/0` был всегда!** Это `ROS_DOMAIN_ID`, который **обязательная часть** формата ключей rmw_zenoh.

Когда вы добавили Zenoh namespace, он просто стал **видимым** в полном пути топика.

---

## 📚 Формат ключей в rmw_zenoh

Из [официальной документации rmw_zenoh](https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md):

> Формат ключевых выражений для топиков:
> 
> `<domain_id>/<fully_qualified_name>/<type_name>/<type_hash>`

### Компоненты:

1. **`<domain_id>`** - Значение `ROS_DOMAIN_ID` (по умолчанию `0`)
2. **`<fully_qualified_name>`** - Полное имя топика (например, `/cmd_vel_voice`)
3. **`<type_name>`** - Тип сообщения (например, `geometry_msgs::msg::dds_::Twist_`)
4. **`<type_hash>`** - Хаш типа (например, `RIHS01_abc123...`)

### Пример:

Для топика `cmd_vel_voice` с `ROS_DOMAIN_ID=0`:

```
0/cmd_vel_voice/geometry_msgs::msg::dds_::Twist_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18
```

**Domain ID (`0`) был ВСЕГДА в начале!**

---

## 🌐 Что изменилось с добавлением Zenoh Namespace

### До добавления namespace:

```
Zenoh ключ:
  0/cmd_vel_voice/geometry_msgs::msg::dds_::Twist_/RIHS01_...
  ↑
  ROS_DOMAIN_ID (всегда был здесь!)
```

### После добавления namespace `robots/RBXU100001`:

```
Zenoh ключ:
  robots/RBXU100001/0/cmd_vel_voice/geometry_msgs::msg::dds_::Twist_/RIHS01_...
  ↑                 ↑
  Namespace         ROS_DOMAIN_ID (никуда не делся, просто стал виден дальше)
```

**Namespace добавляется КАК ПРЕФИКС** ко всему ключу, включая domain_id.

---

## 🎯 Почему это правильно

### 1. ROS_DOMAIN_ID - обязательная часть rmw_zenoh

Из документации:

> The `<domain_id>` prevents any communication between Sessions using different `ROS_DOMAIN_ID`, 
> even when they are connecting to the same Zenoh infrastructure.

**Назначение:** Изоляция между разными ROS доменами в одной Zenoh сети.

### 2. Zenoh Namespace - дополнительная изоляция

**Назначение:** Изоляция между разными роботами в облаке.

### 3. Они работают вместе

```
Уровень 1: Zenoh Namespace
  └─ robots/RBXU100001/    ← Изоляция роботов
     └─ Уровень 2: ROS Domain
        └─ 0/              ← Изоляция ROS доменов
           └─ cmd_vel_voice ← Топик
```

**Результат:** Полная изоляция по двум уровням:
- Разные роботы не видят друг друга (namespace)
- Разные домены на одном роботе не видят друг друга (domain_id)

---

## ❓ Почему раньше работало

### До namespace (обычный DDS):

```
ROS Node → DDS → Zenoh Bridge (zenoh-plugin-dds) → Zenoh Cloud
```

**Zenoh Bridge** автоматически:
- Конвертировал DDS ↔ Zenoh
- Обрабатывал типы сообщений
- **Не добавлял дополнительный namespace** (был только domain_id)

### С namespace (rmw_zenoh напрямую):

```
ROS Node → rmw_zenoh (с namespace) → Zenoh Cloud
```

**rmw_zenoh с namespace:**
- Добавляет namespace: `robots/RBXU100001/`
- Добавляет domain: `0/`
- **Итоговый ключ:** `robots/RBXU100001/0/cmd_vel_voice/...`

---

## 🔧 Что делать

### Вариант 1: Вернуть DDS Bridge (как раньше) ✅ РЕКОМЕНДУЕТСЯ

**На облачном роутере включить zenoh-plugin-dds:**

```json5
{
  "mode": "router",
  "plugins": {
    "dds": {
      "domain": 0,
      "allow": {
        "topics": ["cmd_vel_voice", "cmd_vel_web", "cmd_vel_joy"]
      }
    }
  }
}
```

**Преимущества:**
- Работает как раньше
- Автоматическая конвертация DDS ↔ Zenoh
- REST API может публиковать на простые ключи
- Не нужен Python мост

**На роботе:**
- Убрать namespace из конфигурации (или оставить для изоляции)
- Использовать обычный ROS_DOMAIN_ID=0

### Вариант 2: Оставить namespace + использовать Python мост

**Текущее решение:**
- Namespace остаётся: `robots/RBXU100001`
- Python мост конвертирует REST → ROS
- Публикуете на: `/robots/RBXU100001/cmd_vel_web_bridge`
- Мост получает и публикует в ROS топик

---

## 📊 Сравнение вариантов

| Параметр | DDS Bridge | Namespace + Python мост |
|----------|------------|-------------------------|
| Сложность настройки | Средняя | Низкая |
| Производительность | ✅ Высокая | ⚠️ Средняя |
| Изоляция роботов | ⚠️ Нужна ручная | ✅ Автоматическая |
| REST API | ✅ Простые ключи | ⚠️ Нужен мост |
| Работало раньше | ✅ Да | ❌ Нет |

---

## 💡 Рекомендация

**Для вашего случая: вернитесь к DDS Bridge**

Поскольку:
1. ✅ Это работало раньше
2. ✅ Вам не нужна изоляция namespace (один робот)
3. ✅ REST API будет работать напрямую
4. ✅ Меньше сложности в обслуживании

### Шаги:

1. **На облачном роутере:**
   ```bash
   # Установить zenoh-plugin-dds
   cargo install zenoh-plugin-dds
   
   # Обновить конфигурацию (см. docs/cloud/zenoh_router_config.json5)
   # Раскомментировать секцию dds плагина
   ```

2. **На роботе:**
   ```bash
   # Убрать namespace из zenoh конфигурации
   # Или закомментировать строку namespace в /tmp/zenoh_session_config.json5
   ```

3. **REST API будет работать на простых ключах:**
   ```bash
   curl -X PUT http://zenoh.robbox.online/cmd_vel_voice \
     -H "Content-Type: application/octet-stream" \
     --data-binary @twist.cdr
   ```

---

## 🔗 Ссылки

- [rmw_zenoh Design](https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md)
- [zenoh-plugin-dds](https://github.com/eclipse-zenoh/zenoh-plugin-dds)
- [Zenoh Key Expressions](https://github.com/eclipse-zenoh/roadmap/blob/main/rfcs/ALL/Key%20Expressions.md)

---

**Вывод:** Domain ID (`/0`) был всегда! Он просто стал виден после добавления namespace. Это правильное поведение rmw_zenoh.
