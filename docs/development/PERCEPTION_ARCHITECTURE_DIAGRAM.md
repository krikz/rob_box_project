# Архитектура Perception System - До и После

## 🔴 ТЕКУЩАЯ АРХИТЕКТУРА (До улучшений)

```
┌─────────────────────────────────────────────────────────────┐
│                  EXTERNAL SENSORS                            │
│  /vision  /pose  /odom  /rosout  /stt  /dialogue_response   │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           ↓
┌─────────────────────────────────────────────────────────────┐
│              CONTEXT AGGREGATOR NODE                         │
│  ┌───────────────────────────────────────────────────────┐  │
│  │  ❌ NO node monitoring                                 │  │
│  │  ❌ NO internet detection                              │  │
│  │  ❌ NO time awareness                                  │  │
│  │  ❌ NO equipment health monitoring                     │  │
│  │  ✅ Memory & summarization (DeepSeek)                 │  │
│  │  ✅ System logs monitoring (/rosout)                  │  │
│  └───────────────────────────────────────────────────────┘  │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           │ /perception/context_update
                           │ (PerceptionEvent)
                           ↓
┌─────────────────────────────────────────────────────────────┐
│              REFLECTION NODE                                 │
│  ┌───────────────────────────────────────────────────────┐  │
│  │  ✅ Event-driven thinking                             │  │
│  │  ✅ DeepSeek AI for thoughts                          │  │
│  │  ❌ NO proactive checks                               │  │
│  │  ❌ NO long-term memory                               │  │
│  │  ⚠️  Limited context awareness                        │  │
│  └───────────────────────────────────────────────────────┘  │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           ↓
                  /voice/tts/request
                  /reflection/internal_thought
```

**Проблемы:**
- 🔴 Робот НЕ знает, какие ноды упали
- 🔴 Робот НЕ знает о потере интернета
- 🔴 Робот НЕ осознаёт текущее время
- 🔴 Робот НЕ мониторит оборудование (камера, LIDAR, моторы)

---

## 🟢 НОВАЯ АРХИТЕКТУРА (После улучшений)

```
┌─────────────────────────────────────────────────────────────┐
│                  EXTERNAL SENSORS                            │
│  /vision  /pose  /odom  /rosout  /stt  /dialogue_response   │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           ↓
┌─────────────────────────────────────────────────────────────┐
│           UNIFIED CONTEXT PROVIDER                           │
│  ┌───────────────────────────────────────────────────────┐  │
│  │  ✅ NodeAvailabilityMonitor                           │  │
│  │     • ros2 node list every 5s                         │  │
│  │     • active/failed/missing detection                 │  │
│  │                                                        │  │
│  │  ✅ InternetConnectivityMonitor                       │  │
│  │     • ping 8.8.8.8, 1.1.1.1 every 30s                │  │
│  │     • online/offline status                           │  │
│  │                                                        │  │
│  │  ✅ TimeAwarenessProvider                             │  │
│  │     • current time with timezone                      │  │
│  │     • period detection (morning/day/evening/night)    │  │
│  │     • human-readable formatting                       │  │
│  │                                                        │  │
│  │  🔜 EquipmentHealthMonitor (Stage 2)                  │  │
│  │     • camera FPS monitoring                           │  │
│  │     • LIDAR scan rate monitoring                      │  │
│  │     • motors RPM monitoring                           │  │
│  │     • SLAM status monitoring                          │  │
│  │                                                        │  │
│  │  ✅ Memory & summarization (DeepSeek)                 │  │
│  │  ✅ System logs monitoring (/rosout)                  │  │
│  └───────────────────────────────────────────────────────┘  │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           │ /perception/context_update
                           │ (Enhanced PerceptionEvent)
                           │   • current_time_human
                           │   • time_period
                           │   • internet_available
                           │   • active_nodes / failed_nodes
                           │   • equipment_summary_json
                           ↓
┌─────────────────────────────────────────────────────────────┐
│              REFLECTION NODE (Enhanced)                      │
│  ┌───────────────────────────────────────────────────────┐  │
│  │  ✅ Event-driven thinking                             │  │
│  │  ✅ DeepSeek AI with enhanced context                 │  │
│  │  ✅ Time-aware responses                              │  │
│  │     • "Доброе утро!" (morning)                        │  │
│  │     • "Сейчас вечер, скоро ночь"                      │  │
│  │                                                        │  │
│  │  ✅ System-aware responses                            │  │
│  │     • "Нет интернета, используем локальный режим"     │  │
│  │     • "Нода /camera упала, проверяю"                  │  │
│  │                                                        │  │
│  │  🔜 ProactiveReflectionTimer (Stage 2)                │  │
│  │     • Battery check every 60s                         │  │
│  │     • Equipment check every 300s                      │  │
│  │     • Idle greeting every 600s                        │  │
│  │                                                        │  │
│  │  🔜 Long-term memory (Stage 4)                        │  │
│  │     • SQLite database                                 │  │
│  │     • User preferences                                │  │
│  └───────────────────────────────────────────────────────┘  │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           ↓
                  /voice/tts/request (enhanced)
                  /reflection/internal_thought
```

**Улучшения:**
- ✅ Робот ЗНАЕТ статус всех критичных нод
- ✅ Робот ЗНАЕТ о потере/восстановлении интернета
- ✅ Робот ОСОЗНАЁТ текущее время и период суток
- 🔜 Робот МОНИТОРИТ состояние оборудования (Stage 2)
- 🔜 Робот ПРОАКТИВНО предупреждает о проблемах (Stage 2)

---

## 📊 СРАВНЕНИЕ КОНТЕКСТА

### До улучшений (Current PerceptionEvent)

```python
{
  "stamp": {...},
  "vision_context": "...",
  "pose": {...},
  "velocity": {...},
  "is_moving": false,
  "battery_voltage": 12.3,
  "temperature": 45.2,
  "apriltag_ids": [],
  "system_health_status": "healthy",
  "health_issues": [],
  "memory_summary": "...",
  "speech_summaries": "[...]",
  "robot_response_summaries": "[...]",
  "robot_thought_summaries": "[...]",
  "vision_summaries": "[...]",
  "system_summaries": "[...]"
}
```

**Пробелы:**
- ❌ Нет информации о времени
- ❌ Нет информации о нодах
- ❌ Нет информации об интернете
- ❌ Нет информации об оборудовании

### После улучшений (Enhanced PerceptionEvent)

```python
{
  "stamp": {...},
  "vision_context": "...",
  "pose": {...},
  "velocity": {...},
  "is_moving": false,
  "battery_voltage": 12.3,
  "temperature": 45.2,
  "apriltag_ids": [],
  
  // ✅ НОВОЕ: Time awareness
  "current_time_human": "2025-10-21 12:30:00",
  "time_period": "day",
  "time_context_json": "{
    'hour': 12,
    'minute': 30,
    'weekday': 'Monday',
    'weekday_ru': 'Понедельник',
    'period_ru': 'день',
    'timezone': 'Europe/Moscow'
  }",
  
  // ✅ НОВОЕ: Internet connectivity
  "internet_available": true,
  
  // ✅ НОВОЕ: ROS2 nodes status
  "active_nodes": [
    "/audio_node",
    "/stt_node",
    "/dialogue_node",
    "/reflection_node",
    "/oak_d_node",
    "/lslidar_node"
  ],
  "failed_nodes": [],
  "missing_nodes": ["/nav2"],
  
  // ✅ НОВОЕ: Equipment health
  "equipment_summary_json": "{
    'oak_d_camera': {
      'status': 'active',
      'fps': 30.2,
      'issues': []
    },
    'lslidar': {
      'status': 'active',
      'scan_rate': 10.0,
      'issues': []
    },
    'vesc_motors': {
      'status': 'active',
      'left_rpm': 150,
      'right_rpm': 148,
      'issues': []
    }
  }",
  
  "system_health_status": "healthy",
  "health_issues": [],
  "memory_summary": "...",
  "speech_summaries": "[...]",
  "robot_response_summaries": "[...]",
  "robot_thought_summaries": "[...]",
  "vision_summaries": "[...]",
  "system_summaries": "[...]"
}
```

**Преимущества:**
- ✅ Полная информация о времени и периоде суток
- ✅ Статус всех критичных нод (active/failed/missing)
- ✅ Статус интернет-подключения
- ✅ Детальная информация о состоянии оборудования

---

## 🔄 ПРИМЕР ИСПОЛЬЗОВАНИЯ (До vs После)

### Сценарий: Пользователь спрашивает "Как дела?"

#### До улучшений

```
👤 User: "Роббокс, как дела?"

🤖 Reflection Node думает:
  Context: {
    battery: 12.3V,
    is_moving: false,
    vision: "empty room"
  }

🗣️ Robot: "У меня всё хорошо, стою на месте"
```

**Проблема:** Робот НЕ знает:
- 🔴 Что /camera нода упала (нет картинки)
- 🔴 Что интернет недоступен (API может не работать)
- 🔴 Что сейчас вечер (нет time-aware ответа)

#### После улучшений

```
👤 User: "Роббокс, как дела?"

🤖 Reflection Node думает:
  Enhanced Context: {
    current_time: "2025-10-21 19:30:00",
    time_period: "evening",
    battery: 12.3V,
    is_moving: false,
    internet_available: true,
    active_nodes: 6,
    failed_nodes: ["/oak_d_node"],
    vision: "N/A (camera offline)",
    equipment: {
      oak_d_camera: "timeout"
    }
  }

🗣️ Robot: "Добрый вечер! В целом всё в порядке, но есть 
           небольшая проблема - камера не отвечает. 
           Остальное оборудование работает нормально."
```

**Преимущество:** Робот ЗНАЕТ контекст:
- ✅ Время суток → "Добрый вечер!"
- ✅ Проблема с камерой → сообщает пользователю
- ✅ Интернет доступен → может использовать AI
- ✅ Полная осведомлённость о системе

---

## 🎯 ЭТАПЫ ВНЕДРЕНИЯ

### Stage 1: Critical Monitoring ✅ ГОТОВО (1-2 недели)

```
┌─────────────────────────────────┐
│  NodeAvailabilityMonitor        │  ✅ Created
├─────────────────────────────────┤
│  InternetConnectivityMonitor    │  ✅ Created
├─────────────────────────────────┤
│  TimeAwarenessProvider          │  ✅ Created
├─────────────────────────────────┤
│  PerceptionEvent.msg (updated)  │  ✅ Created
└─────────────────────────────────┘

TODO:
- [ ] Integrate into context_aggregator_node
- [ ] Update reflection_node prompts
- [ ] Update dialogue_node for fallback
- [ ] Test on robot
```

### Stage 2: Equipment Monitoring (1-2 недели)

```
┌─────────────────────────────────┐
│  EquipmentHealthMonitor         │  🔜 TODO
│  • Camera FPS monitoring        │
│  • LIDAR scan rate              │
│  • Motors RPM                   │
│  • SLAM status                  │
├─────────────────────────────────┤
│  ProactiveReflectionTimer       │  🔜 TODO
│  • Battery checks               │
│  • Equipment checks             │
│  • Idle greetings               │
└─────────────────────────────────┘
```

### Stage 3: Architecture (2-3 недели)

```
┌─────────────────────────────────┐
│  UnifiedContextProvider         │  🔜 TODO
│  • Single source of truth       │
│  • Service API for context      │
├─────────────────────────────────┤
│  Refactor context_aggregator    │  🔜 TODO
│  • Use UnifiedContextProvider   │
│  • Merge health_monitor         │
└─────────────────────────────────┘
```

### Stage 4: Optional Features (опционально)

```
┌─────────────────────────────────┐
│  LongTermMemory (SQLite)        │  ⚪ Optional
├─────────────────────────────────┤
│  NLU for intent detection       │  ⚪ Optional
├─────────────────────────────────┤
│  ML for problem prediction      │  ⚪ Optional
└─────────────────────────────────┘
```

---

## 📈 ОЖИДАЕМЫЕ МЕТРИКИ

### Надёжность (+40%)

**До:**
- Ручное обнаружение проблем
- Реакция на сбои: ~5-30 минут

**После:**
- Автоматическое обнаружение: ~5 секунд (ноды)
- Автоматические уведомления пользователю
- Fallback режимы при проблемах

### Observability (+60%)

**До:**
- Видимость: только через логи
- Мониторинг: только ошибки/предупреждения

**После:**
- Полный real-time мониторинг
- Dashboard-ready данные
- Structured context API

### User Experience (+30%)

**До:**
- Роботические ответы без контекста
- Нет осознания времени
- Нет предупреждений о проблемах

**После:**
- Time-aware приветствия
- Context-aware ответы
- Проактивные предупреждения

---

**Дата:** 21 октября 2025  
**Статус:** Stage 1 компоненты готовы к интеграции
