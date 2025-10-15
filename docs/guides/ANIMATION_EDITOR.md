# Animation Editor - Руководство по использованию

## 🎨 Обзор

Animation Editor - полнофункциональный редактор для создания LED анимаций для робота РОББОКС. Позволяет редактировать пиксели, управлять кадрами через timeline, и экспортировать анимации в формате YAML + PNG.

## 🚀 Быстрый старт

### Запуск редактора

```bash
cd ~/rob_box_project
./tools/animation_editor/launch.sh
```

Или напрямую:

```bash
python3 tools/animation_editor/main.py --animations-dir src/rob_box_animations/animations
```

## 📝 Основной workflow

### 1. Создание новой анимации

1. `File → New Animation` (или `Ctrl+N`)
2. Заполните свойства в правой панели:
   - **Name**: имя анимации (без пробелов)
   - **Description**: описание
   - **FPS**: частота кадров (обычно 10)
   - **Loop**: зациклить анимацию

### 2. Выбор панели для редактирования

В Timeline внизу выберите панель из выпадающего списка:

- **wheel_front_left** (8×8) - Левое переднее колесо (зеленая рамка)
- **wheel_front_right** (8×8) - Правое переднее колесо (зеленая рамка)
- **wheel_rear_left** (8×8) - Левое заднее колесо (красная рамка)
- **wheel_rear_right** (8×8) - Правое заднее колесо (красная рамка)
- **main_display** (25×5) - Главный дисплей "рот" (желтая рамка)

### 3. Добавление кадров

- Кнопка **➕ Add Frame** - добавить пустой кадр
- Кнопка **📋 Duplicate** - дублировать текущий кадр
- Кнопка **🗑️ Delete** - удалить кадр

### 4. Рисование

1. Выберите цвет в левой панели **COLOR PALETTE**:
   - Кликните на один из preset цветов (16 штук)
   - Или нажмите **🎨 Custom Color** для выбора своего цвета
   
2. Кликните на LED в центральной области canvas
   - Одиночный клик - закрасить пиксель
   - Перетаскивание мыши - рисовать

3. Для стирания используйте **🧹 Eraser** или выберите черный цвет

### 5. Настройка времени

- Выделите кадр в Timeline
- Измените **Duration** (ms) внизу Timeline
- По умолчанию: 100ms (= 10 FPS)

### 6. Сохранение

- `File → Save` (или `Ctrl+S`) - сохранить
- `File → Save As` (или `Ctrl+Shift+S`) - сохранить как

Файлы сохраняются в:
```
src/rob_box_animations/animations/
├── manifests/
│   └── my_animation.yaml    # Манифест
└── frames/
    └── my_animation/
        ├── wheel_front_left_frame_000.png
        ├── wheel_front_left_frame_001.png
        └── ...
```

## 🎯 Панели робота

### Расположение LED матриц

```
         ↑ FRONT ↑
    ┌─────────────────┐
    │   MOUTH (25×5)  │ ← Главный дисплей (желтый)
    │   ┌─────────┐   │
    │   └─────────┘   │
FL  │                 │  FR ← Передние колеса (зеленые)
8×8 │                 │ 8×8
    │                 │
    │     КОРПУС      │
    │                 │
RL  │                 │  RR ← Задние колеса (красные)
8×8 │                 │ 8×8
    └─────────────────┘
         ↓ REAR ↓
```

### Logical groups

| Группа | Размер | Описание | Применение |
|--------|--------|----------|------------|
| `wheel_front_left` | 8×8 | Левое переднее | Индикация поворота, эмоции |
| `wheel_front_right` | 8×8 | Правое переднее | Индикация поворота, эмоции |
| `wheel_rear_left` | 8×8 | Левое заднее | Задние огни, тормоза |
| `wheel_rear_right` | 8×8 | Правое заднее | Задние огни, тормоза |
| `main_display` | 25×5 | Главный дисплей | Эмоции (рот), текст |
| `wheels_front` | 16×8 | Оба передних | Синхронная анимация |
| `wheels_rear` | 16×8 | Оба задних | Синхронная анимация |
| `wheels_all` | 16×16 | Все 4 колеса | Полная синхронная анимация |

## ⌨️ Горячие клавиши

| Клавиши | Действие |
|---------|----------|
| `Ctrl+N` | Новая анимация |
| `Ctrl+O` | Открыть |
| `Ctrl+S` | Сохранить |
| `Ctrl+Shift+S` | Сохранить как |
| `Ctrl+Q` | Выход |

## 💡 Советы и приёмы

### 1. Работа с палитрой

- **Preset цвета** оптимизированы для LED:
  - Базовые: Black, White, Red, Green, Blue, Yellow, Cyan, Magenta
  - Дополнительные: Orange, Purple, Pink, Lime
  - Тёмные: Dark Red, Dark Green, Dark Blue, Gray

- **Custom Color**: можете выбрать любой RGB цвет
- **Eraser**: быстро очистить пиксели (черный)

### 2. Создание анимаций

- **Начните с ключевых кадров**: создайте начальный и конечный кадр
- **Используйте Duplicate**: для промежуточных кадров дублируйте и редактируйте
- **Правильная длительность**:
  - Быстрая анимация: 50-100ms на кадр
  - Средняя: 100-200ms
  - Медленная: 200-500ms

### 3. Разные панели - разные цели

- **Передние колеса (green)**: яркие, информативные (поворотники, эмоции)
- **Задние колеса (red)**: стоп-сигналы, движение назад
- **Main display**: эмоции, текст, иконки

### 4. Оптимизация

- Меньше кадров = меньше памяти
- Используйте loop для повторяющихся анимаций
- Группируйте похожие анимации (например, все эмоции в одной папке)

## 🔧 Редактирование существующих анимаций

```bash
# Открыть существующую анимацию
File → Open
# Выберите файл в src/rob_box_animations/animations/manifests/

# Примеры анимаций:
# - emergency_stop.yaml
# - happy_robot.yaml
# - nav_turning_left.yaml
# - sys_low_battery.yaml
```

## 📦 Экспорт и использование

После сохранения анимация доступна в ROS2:

```bash
# Проверить установленные анимации
ros2 run rob_box_animations list_animations

# Воспроизвести анимацию
ros2 service call /animation_player/play_animation rob_box_animations/srv/PlayAnimation "{name: 'my_animation', loop: true}"
```

## 🐛 Решение проблем

### Редактор не запускается

```bash
# Проверить зависимости
pip install -r tools/animation_editor/requirements.txt

# Проверить Python версию (нужен 3.8+)
python3 --version
```

### Не находит анимации

```bash
# Проверить путь к animations dir
ls src/rob_box_animations/animations/manifests/

# Указать путь явно
python3 tools/animation_editor/main.py --animations-dir /полный/путь/к/animations
```

### Ошибки при сохранении

- Проверьте права на запись в `src/rob_box_animations/animations/`
- Убедитесь что Name не содержит пробелов и спецсимволов
- Проверьте что есть хотя бы один кадр в анимации

## 📚 Дополнительная документация

- **Полная документация Editor**: [tools/animation_editor/README.md](../../tools/animation_editor/README.md)
- **Формат анимаций**: [src/rob_box_animations/animations/README.md](../../src/rob_box_animations/animations/README.md)
- **LED система**: [docs/packages/LED_ANIMATIONS.md](../packages/LED_ANIMATIONS.md)

## 🎓 Примеры workflow

### Создание эмоции "Happy"

1. New Animation: name="happy_face"
2. Выбрать panel: `main_display` (25×5)
3. Add Frame
4. Нарисовать улыбку:
   - Левый глаз: 2 желтых пикселя
   - Правый глаз: 2 желтых пикселя
   - Рот: дуга из зеленых пикселей
5. Duplicate Frame
6. Слегка изменить рот (анимация)
7. Duration: 200ms на кадр
8. Loop: Yes
9. Save

### Создание поворотника

1. New Animation: name="turn_left"
2. Выбрать panel: `wheels_front` (16×8)
3. Add 5 frames
4. Кадр 1: стрелка влево (желтый)
5. Кадры 2-5: постепенное затухание
6. Duration: 100ms
7. Loop: Yes
8. Save

---

**Создано для РОББОКС** | [GitHub](https://github.com/krikz/rob_box_project) | Версия 1.0.0
