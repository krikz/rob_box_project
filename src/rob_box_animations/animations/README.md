# LED Matrix Animations - rob_box

Набор анимаций для LED матриц робота с поддержкой нескольких панелей одновременно.

## 📁 Структура

```
animations/
├── README.md
├── manifests/          # Манифесты анимаций (YAML)
│   ├── police.yaml
│   ├── road_service.yaml
│   ├── ambulance.yaml
│   ├── idle.yaml
│   ├── charging.yaml
│   ├── mission_received.yaml
│   ├── mission_active.yaml
│   ├── mission_complete.yaml
│   ├── startup.yaml
│   └── shutdown.yaml
└── frames/             # Кадры анимаций (PNG)
    ├── police/
    ├── road_service/
    ├── ambulance/
    ├── idle/
    ├── charging/
    ├── mission_received/
    ├── mission_active/
    ├── mission_complete/
    ├── startup/
    └── shutdown/
```

## 🎬 Формат манифеста

```yaml
name: "police_lights"
description: "Мигалки полицейской машины"
version: "1.0"
author: "rob_box"
duration_ms: 1000        # Общая длительность цикла
loop: true               # Зацикливать анимацию
fps: 10                  # Частота кадров

# Панели с анимациями
panels:
  # Передние фары (глаза)
  - logical_group: "headlight_front_left"
    frames:
      - image: "frames/police/front_left_001.png"
        duration_ms: 100
      - image: "frames/police/front_left_002.png"
        duration_ms: 100
      # ...
  
  - logical_group: "headlight_front_right"
    frames:
      - image: "frames/police/front_right_001.png"
        duration_ms: 100
      - image: "frames/police/front_right_002.png"
        duration_ms: 100
  
  # Задние фары
  - logical_group: "headlight_rear_left"
    frames:
      - image: "frames/police/rear_left_001.png"
        duration_ms: 100
  
  - logical_group: "headlight_rear_right"
    frames:
      - image: "frames/police/rear_right_001.png"
        duration_ms: 100
  
  # Рот (главный дисплей)
  - logical_group: "main_display"
    frames:
      - image: "frames/police/mouth_001.png"
        duration_ms: 100
      - image: "frames/police/mouth_002.png"
        duration_ms: 100

# Метаданные
metadata:
  priority: "high"       # Приоритет анимации
  category: "emergency"  # Категория
  tags: ["police", "emergency", "lights"]
```

## 🎨 Формат изображений

### Размеры панелей
- **Передние/задние фары:** 8×8 пикселей (RGB)
- **Главный дисплей (рот):** 5×25 пикселей (RGB)

### Формат файлов
- **Тип:** PNG
- **Цветовое пространство:** RGB (без альфа-канала)
- **Глубина цвета:** 8 бит на канал
- **Имена файлов:** `{panel}_{frame_number:03d}.png`

### Пример
```
frames/police/
├── front_left_001.png   # 8×8, красный
├── front_left_002.png   # 8×8, синий
├── front_right_001.png  # 8×8, синий
├── front_right_002.png  # 8×8, красный
├── mouth_001.png        # 5×25, текст "POLICE"
└── mouth_002.png        # 5×25, пустой
```

## 🚨 Список анимаций

### Экстренные службы

#### 1. police_lights (Полиция)
- **Передние:** Синий-красный чередуются Left↔Right
- **Задние:** Синий-красный чередуются Left↔Right
- **Рот:** "POLICE" мигает

#### 2. road_service (Дорожная служба)
- **Передние:** Желтый вращающийся маяк
- **Задние:** Желтый вращающийся маяк
- **Рот:** "ROAD SERVICE" бегущая строка

#### 3. ambulance (Скорая помощь)
- **Передние:** Красный мигает синхронно
- **Задние:** Красный мигает синхронно
- **Рот:** Красный крест + "AMBULANCE"

### Состояния робота

#### 4. idle (Простой)
- **Передние:** Мягкое дыхание белым светом
- **Задние:** Тусклое свечение красным
- **Рот:** Закрытый рот → медленное открытие/закрытие

#### 5. charging (Зарядка)
- **Передние:** Зеленый индикатор уровня заряда
- **Задние:** Выключены
- **Рот:** Анимация заполнения батареи слева направо

#### 6. mission_received (Задание получено)
- **Передние:** Быстрое мигание зеленым
- **Задние:** Быстрое мигание зеленым
- **Рот:** Удивленный рот + "!" символ

#### 7. mission_active (Выполняется задание)
- **Передние:** Постоянный белый свет
- **Задние:** Постоянный красный свет
- **Рот:** Решительное выражение (прямая линия)

#### 8. mission_complete (Задание выполнено)
- **Передние:** Зеленая галочка ✓
- **Задние:** Зеленая галочка ✓
- **Рот:** Улыбка + "DONE"

### Системные

#### 9. startup (Запуск)
- **Передние:** Заполнение сверху вниз
- **Задние:** Заполнение сверху вниз
- **Рот:** Логотип rob_box появляется

#### 10. shutdown (Выключение)
- **Передние:** Опустошение снизу вверх
- **Задние:** Опустошение снизу вверх
- **Рот:** "BYE" исчезает

## 🎮 Использование

### Воспроизведение анимации

```python
from rob_box_animations import AnimationPlayer

player = AnimationPlayer()
player.load_manifest('manifests/police.yaml')
player.play()
```

### CLI
```bash
ros2 run rob_box_animations player --manifest manifests/police.yaml
```

## 🔧 Создание своей анимации

1. **Создайте кадры** в графическом редакторе (GIMP/Photoshop)
   - Размеры: 8×8 для фар, 5×25 для рта
   - Формат: PNG RGB
   - Сохраните в `frames/my_animation/`

2. **Создайте манифест** `manifests/my_animation.yaml`
   ```yaml
   name: "my_animation"
   description: "Моя крутая анимация"
   loop: true
   fps: 10
   panels:
     - logical_group: "main_display"
       frames:
         - image: "frames/my_animation/mouth_001.png"
           duration_ms: 100
   ```

3. **Протестируйте**
   ```bash
   ros2 run rob_box_animations player --manifest manifests/my_animation.yaml
   ```

## 📊 Приоритеты анимаций

При конфликте нескольких анимаций используется приоритет:

1. **critical** - Критические (аварийные остановки)
2. **high** - Высокий (экстренные службы)
3. **normal** - Обычный (состояния робота)
4. **low** - Низкий (idle режим)

## 🎯 Roadmap

- [ ] Создать все 10 базовых анимаций
- [ ] Разработать Animation Player Node
- [ ] Интеграция с robot state machine
- [ ] Добавить transitions между анимациями
- [ ] Web-интерфейс для предпросмотра
- [ ] Генератор анимаций из JSON
- [ ] Библиотека готовых эффектов (пульсация, вращение, волны)

## 📚 Ссылки

- [LED Matrix Integration](../docs/reference/LED_MATRIX_INTEGRATION.md)
- [ros2leds Repository](https://github.com/krikz/ros2leds)
