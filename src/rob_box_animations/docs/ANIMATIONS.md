# 🎨 Система LED Анимаций

Документация системы анимаций для WS2812B LED матриц Rob Box.

---

## 📊 Обзор

Rob Box использует **381 адресуемый WS2812B LED** для визуального отображения состояний:

- **4× матрицы 8×8** (256 LED) — колёса робота
- **1× матрица 25×5** (125 LED) — центральный дисплей

### Архитектура

```
ROS2 Compositor → Animation Manager → pi5neo Driver → SPI → WS2812B LEDs
```

**Пакет:** [ros2leds](https://github.com/nosknut/ros2leds) (git submodule)  
**Драйвер:** pi5neo library (Raspberry Pi 5)  
**Интерфейс:** SPI (GPIO10 — MOSI)  
**Питание:** 5V/25A PSU (до 100% яркости)

---

## 🎬 Галерея Анимаций

### 🚨 Emergency (Экстренные службы)

| Анимация | Описание | Превью |
|----------|----------|--------|
| **Police** | Синие/красные мигающие огни + статичный текст "ПОЛИЦИЯ" | [](../../../assets/animations/police.gif) |
| **Ambulance** | Красно-белая комбинация + медицинский крест | [](../../../assets/animations/ambulance.gif) |
| **Fire Truck** | Красно-оранжевые огни + текст "FIRE" | [](../../../assets/animations/fire_truck.gif) |
| **Road Service** | Жёлтая пульсация на всех матрицах | [](../../../assets/animations/road_service.gif) |

### 🧭 Navigation (Навигация)

| Анимация | Описание | Превью |
|----------|----------|--------|
| **Turn Left** | Оранжевое мигание FL/RL + красный стоп-сигнал RR | [](../../../assets/animations/turn_left.gif) |
| **Turn Right** | Оранжевое мигание FR/RR + красный стоп-сигнал RL | [](../../../assets/animations/turn_right.gif) |
| **Braking** | Расширяющиеся белые фары + ярко-красная пульсация | [](../../../assets/animations/braking.gif) |
| **Accelerating** | Нарастающие белые фары + оранжевое свечение | [](../../../assets/animations/accelerating.gif) |

### 😊 Emotions (Эмоции)

| Анимация | Описание | Превью |
|----------|----------|--------|
| **Idle** | Спокойное дыхание (пульсация 0.5→1.0) | [](../../../assets/animations/idle.gif) |
| **Happy** | Радужные глаза + улыбка | [](../../../assets/animations/happy.gif) |
| **Sad** | Синие грустные глаза + грустный рот | [](../../../assets/animations/sad.gif) |
| **Angry** | Красная пульсация + сердитое лицо | [](../../../assets/animations/angry.gif) |
| **Surprised** | Широко открытые глаза + O-образный рот | [](../../../assets/animations/surprised.gif) |
| **Thinking** | Боковой взгляд + анимированные точки "..." | [](../../../assets/animations/thinking.gif) |
| **Talking** | Моргание + анимированный рот с волнами | [](../../../assets/animations/talking.gif) |
| **Victory** | Радужный цикл + галочка ✓ | [](../../../assets/animations/victory.gif) |

### ⚙️ System (Системные)

| Анимация | Описание | Превью |
|----------|----------|--------|
| **Sleep** | Закрытые глаза + тусклая синяя пульсация + ZZZ | [](../../../assets/animations/sleep.gif) |
| **Wakeup** | Открывающиеся глаза + нарастающая яркость + зевок | [](../../../assets/animations/wakeup.gif) |
| **Charging** | Нейтральные глаза + зелёная пульсация + анимация батареи | [](../../../assets/animations/charging.gif) |
| **Low Battery** | Тусклое красное мигание + критическая батарея | [](../../../assets/animations/low_battery.gif) |
| **Error** | X-глаза + красное мигание + текст "ERROR" | [](../../../assets/animations/error.gif) |

---

## 📐 Технические детали

### Структура проекта

```
src/rob_box_animations/
├── animations/
│   ├── frames/              # PNG кадры (8×8 и 25×5)
│   │   ├── police/
│   │   ├── navigation/
│   │   ├── emotions/
│   │   └── system/
│   └── manifests/           # YAML манифесты v2.5
│       ├── police_lights.yaml
│       ├── turn_left.yaml
│       ├── happy.yaml
│       └── ...
└── scripts/
    ├── generate_animation_frames.py  # Генератор кадров
    └── visualize_animations.py       # Превью анимаций
```

### Формат манифеста (v2.5)

```yaml
name: "animation_name"
version: "2.5"
duration_ms: 1000
loop: true
fps: 10

panels:
  - logical_group: "wheel_front_left"
    offset_ms: 0
    frames:
      - image: "frames/category/frame_01.png"
        duration_ms: 100

metadata:
  priority: "normal"
  category: "navigation"
  tags: ["navigation", "turn"]
```

### Кириллический шрифт

Для текста используется custom **3×5 pixel font**:

```python
cyrillic_map = {
    'П': [[1,1,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1]],
    'О': [[1,1,1],[1,0,1],[1,0,1],[1,0,1],[1,1,1]],
    'Л': [[0,0,1],[0,1,0],[0,1,0],[1,0,0],[1,0,0]],
    'И': [[1,0,1],[1,0,1],[1,1,1],[1,0,1],[1,0,1]],
    'Ц': [[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,1,1]],
    'Я': [[1,1,1],[1,0,1],[1,1,1],[0,1,0],[1,0,1]],
}
```

---

## 🛠️ Разработка

### Генерация кадров

```bash
# Все анимации
python3 scripts/generate_animation_frames.py --animation all

# Конкретная анимация
python3 scripts/generate_animation_frames.py --animation happy
```

### Превью

```bash
python3 scripts/visualize_animations.py
```

Откроется интерактивное окно с превью всех 5 матриц в реальном времени.

### Создание новой анимации

1. Добавить метод в `generate_animation_frames.py`
2. Создать YAML манифест
3. Сгенерировать кадры и протестировать

---

## 📊 Статистика

- **Всего анимаций:** 21
- **Всего кадров:** 600+
- **Размер библиотеки:** ~50KB (PNG) + ~54MB (GIF превью)
- **FPS:** 10 (стандарт)
- **Средняя длительность:** 1-3 секунды
- **Формат кадров:** RGB888 PNG (прозрачный фон)

---

## 🔗 Связанная документация

- [ros2leds package](https://github.com/nosknut/ros2leds) — LED управление
- [rob_box_animations README](../README.md) — детали пакета
- [SOFTWARE.md](../../../architecture/SOFTWARE.md) — архитектура программного обеспечения

---

**Версия:** 2.5  
**Последнее обновление:** 12 октября 2025
