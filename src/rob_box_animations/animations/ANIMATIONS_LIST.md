# LED Matrix Animations - Complete List

## 📋 All Animations (25+)

### 🚨 Emergency Services (3)
1. **police_lights** - Полицейские мигалки (синий-красный)
2. **road_service** - Дорожная служба (желтый маяк)
3. **ambulance** - Скорая помощь (красный + крест)

### 🤖 Robot States (8)
4. **idle** - Простой (мягкое дыхание)
5. **charging** - Зарядка (индикатор заполнения)
6. **low_battery** - Низкий заряд (красное мигание)
7. **sleeping** - Спящий режим (закрытые глаза)
8. **waking_up** - Пробуждение (глаза открываются)
9. **active** - Активный (яркие глаза)
10. **overheating** - Перегрев (красно-желтое мигание)
11. **error** - Ошибка (красный X + мигание)

### 🎯 Mission States (5)
12. **mission_received** - Задание получено (зеленое мигание)
13. **mission_active** - Выполняется (постоянный белый)
14. **mission_complete** - Выполнено (зеленая галочка)
15. **mission_failed** - Провалено (красный X)
16. **waiting_for_mission** - Ожидание задания (пульсация синим)

### 💬 Emotions (7)
17. **happy** - Счастливый (улыбка + яркие глаза)
18. **sad** - Грустный (грустное лицо)
19. **angry** - Злой (красные глаза + хмурое лицо)
20. **surprised** - Удивленный (широкие глаза + O)
21. **confused** - Смущенный (косые глаза + ?)
22. **thinking** - Думающий (глаза в сторону + ...)
23. **celebrating** - Празднование (конфетти + !)

### 🔔 Notifications (5)
24. **info** - Информация (синий i)
25. **warning** - Предупреждение (желтый треугольник)
26. **alert** - Тревога (красное мигание)
27. **message_received** - Сообщение получено (конверт)
28. **call_incoming** - Входящий вызов (телефон + звонок)

### 🚗 Navigation (6)
29. **turn_left** - Поворот налево (стрелка влево)
30. **turn_right** - Поворот направо (стрелка вправо)
31. **reversing** - Задний ход (стрелки назад + бипер)
32. **braking** - Торможение (красные стоп-сигналы)
33. **accelerating** - Ускорение (зеленые стрелки вперед)
34. **hazard_lights** - Аварийная сигнализация (все мигают)

### 👋 Interaction (5)
35. **greeting** - Приветствие (махающая рука + "HI")
36. **goodbye** - Прощание ("BYE" + волна)
37. **yes** - Да (зеленая галочка + кивок)
38. **no** - Нет (красный X + качание)
39. **acknowledge** - Подтверждение (синий ! + мигание)

### 🔧 Diagnostic (4)
40. **self_test** - Самодиагностика (последовательное включение)
41. **calibration** - Калибровка (вращающиеся паттерны)
42. **sensor_check** - Проверка датчиков (сканирование)
43. **firmware_update** - Обновление ПО (прогресс-бар)

### 🎨 Decorative (5)
44. **rainbow** - Радуга (цветная волна)
45. **matrix_rain** - Матричный дождь (зеленые символы)
46. **fire** - Огонь (красно-желтое пламя)
47. **water** - Вода (синие волны)
48. **stars** - Звезды (мерцающие точки)

### ⚙️ System (2)
49. **startup** - Запуск (заполнение сверху вниз)
50. **shutdown** - Выключение (опустошение снизу вверх)

---

## 🎬 Priority Matrix

| Priority | Categories |
|----------|-----------|
| CRITICAL | error, overheating, low_battery |
| HIGH | emergency services, alert, hazard_lights |
| NORMAL | states, missions, navigation, notifications |
| LOW | emotions, interaction, decorative |

## 📊 Panel Usage

### Front Headlights (Eyes) - 8×8 each
- Emotions (eye expressions)
- Navigation (turn signals)
- Status indicators
- Alert patterns

### Rear Headlights (Taillights) - 8×8 each
- Brake lights (red)
- Reverse lights (white)
- Turn signals
- Hazard lights

### Main Display (Mouth) - 5×25
- Text messages
- Icons and symbols
- Progress bars
- Facial expressions
- Status information

## 🔄 Animation Transitions

Smooth transitions between animations:
- **fade** - Fade in/out (duration: 200-500ms)
- **slide** - Slide left/right (duration: 300ms)
- **wipe** - Wipe top/bottom (duration: 250ms)
- **instant** - No transition (0ms)

## 🎯 State Machine Integration

```yaml
robot_states:
  IDLE:
    default_animation: idle
    low_priority_allow: [decorative, interaction]
  
  CHARGING:
    default_animation: charging
    override: [low_battery, error]
  
  MISSION_ACTIVE:
    default_animation: mission_active
    navigation_allow: [turn_left, turn_right, braking]
    override: [error, alert, hazard_lights]
  
  ERROR:
    default_animation: error
    override: [critical only]
```

## 📝 Implementation Checklist

### Phase 1: Core Animations (Priority: HIGH)
- [ ] idle
- [ ] charging
- [ ] low_battery
- [ ] error
- [ ] startup
- [ ] shutdown

### Phase 2: Mission & Navigation (Priority: NORMAL)
- [ ] mission_received
- [ ] mission_active
- [ ] mission_complete
- [ ] turn_left
- [ ] turn_right
- [ ] braking

### Phase 3: Emergency Services (Priority: HIGH)
- [ ] police_lights
- [ ] road_service
- [ ] ambulance

### Phase 4: Emotions (Priority: LOW)
- [ ] happy
- [ ] sad
- [ ] surprised
- [ ] thinking

### Phase 5: Extended Features (Priority: LOW)
- [ ] All remaining animations
- [ ] Custom user animations
- [ ] Animation editor tool
