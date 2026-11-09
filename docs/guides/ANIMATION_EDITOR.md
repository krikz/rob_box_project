<!-- stale-paths: ignore-file (описывает удалённое или старую раскладку; кандидат в архив или переписывание, ADR-0163) -->
# Animation Editor v2 - Руководство по использованию

## 🎨 Обзор

Animation Editor v2 - полнофункциональный редактор для создания LED анимаций для робота РОББОКС с поддержкой **ключевых кадров (keyframes)**. Позволяет редактировать пиксели всех LED матриц одновременно, управлять анимацией через timeline, и экспортировать в формате YAML + PNG.

**Новое в v2:**
- ✨ Keyframe-based архитектура (вместо frame-by-frame)
- 🎯 Одновременное редактирование всех панелей
- 📍 Чекбоксы активации панелей
- 🔄 Автоконвертация из старого формата
- 💾 Загрузка существующих анимаций

## 🚀 Быстрый старт

### ⚡ Установка для Windows (одна команда!)

Откройте **PowerShell** и выполните:

```powershell
irm https://raw.githubusercontent.com/krikz/rob_box_project/feature/voice-assistant/tools/animation_editor/install_and_run.ps1 | iex
```

Скрипт автоматически:
- ✅ Проверит Python 3.8+
- ✅ Скачает редактор
- ✅ Установит зависимости (pillow, numpy, pyyaml)
- ✅ Создаст папку для анимаций
- ✅ Запустит редактор

**Требования:** Python 3.8+ с pip ([скачать](https://www.python.org/downloads/))

**Альтернатива:** См. [tools/animation_editor/INSTALL_WINDOWS.md](../../tools/animation_editor/INSTALL_WINDOWS.md)

### 🐧 Запуск на Linux/ROS2

```bash
cd ~/rob_box_project
python3 tools/animation_editor/main.py --animations-dir src/rob_box_animations/animations
```

**Установка зависимостей:**
```bash
pip install pillow numpy pyyaml
```

## 📝 Основной workflow (Keyframe-Based)

### 1. Создание новой анимации

1. `File → New Animation` (или `Ctrl+N`)
2. Заполните свойства в правой панели **⚙️ PROPERTIES**:
   - **Name**: имя анимации (без пробелов, например `happy_face`)
   - **Description**: описание (`Happy robot face animation`)
   - **FPS**: частота кадров (обычно 10)
   - **Loop**: зациклить анимацию (обычно ✓)

### 2. Добавление ключевых кадров

В нижней панели **TIMELINE**:

1. Нажмите кнопку **+ Add Keyframe** - добавится ключевой кадр в текущей позиции времени
2. Переместите слайдер времени **Time** для выбора момента времени (например, 500ms)
3. Добавьте новые keyframes в нужные моменты
4. Кнопка **🗑️ Delete Keyframe** - удалить текущий keyframe (если их больше одного)

**Keyframes на timeline:**
- 🟢 Зеленая точка - текущий выбранный keyframe
- 🟠 Оранжевая точка - остальные keyframes
- 🔴 Красная линия - текущая позиция воспроизведения

### 3. Активация панелей для редактирования

В правой панели **🎨 Active Panels** (под Properties):

Включите чекбоксы нужных панелей:
- **🔴 FL Wheel** - Левое переднее колесо (8×8)
- **🔴 FR Wheel** - Правое переднее колесо (8×8)
- **🔵 RL Wheel** - Левое заднее колесо (8×8)
- **🔵 RR Wheel** - Правое заднее колесо (8×8)
- **🟢 Main Display** - Главный дисплей "рот" (25×5)

💡 **Важно:** Только активные панели можно редактировать и они будут отображаться в анимации!

💡 **Подсказка:** Если чекбоксы неактивны (серые), сначала добавьте keyframe!

### 4. Рисование

1. Выберите цвет в левой панели **COLOR PALETTE**:
   - 8 основных цветов: White, Red, Green, Blue, Yellow, Cyan, Magenta, Black
   
2. Кликните на LED в центральной области canvas:
   - Одиночный клик - закрасить один пиксель
   - Перетаскивание мыши - рисовать линии
   - ❗ Рисование работает **только на активных панелях** (с включенным чекбоксом)

3. Для стирания выберите черный цвет (Black)

**Полезные приемы:**
- `Edit → Clear Frame` - очистить текущий keyframe
- `Edit → Fill Frame` - заполнить текущим цветом

### 5. Создание анимации из нескольких кадров

**Пример:** Мигающий смайлик

1. **Keyframe 0ms:** Нарисуйте счастливое лицо на main_display
2. **Keyframe 500ms:** Сдвиньте слайдер на 500ms, добавьте keyframe
3. **Keyframe 500ms:** Измените рот (например, меньше)
4. **Keyframe 1000ms:** Сдвиньте на 1000ms, добавьте keyframe
5. **Keyframe 1000ms:** Верните счастливое лицо
6. Установите Duration = 1000ms, Loop = ✓
7. Нажмите **▶️ Play** для предпросмотра

### 6. Настройка времени и длительности

- **Time slider** - текущее время воспроизведения (0 - duration_ms)
- **Duration** - общая длительность анимации в миллисекундах
  - Изменяется через spinbox внизу timeline
  - Диапазон: 100-60000ms
- Keyframes автоматически сортируются по времени

### 7. Предпросмотр

- Кнопка **▶️ Play** в toolbar - воспроизвести анимацию
- Во время воспроизведения показываются все активные панели одновременно
- Автоматическое зацикливание если включен Loop
- Нажмите **▶️** еще раз для остановки

### 8. Сохранение и загрузка

**Сохранение:**
- `File → Save` (или `Ctrl+S`) - сохранить текущую анимацию
- `File → Save As` (или `Ctrl+Shift+S`) - сохранить под новым именем
- Формат: keyframe-based YAML + PNG кадры

**Загрузка:**
- `File → Open` (или `Ctrl+O`) - открыть существующую анимацию
- Поддерживается загрузка **старого frame-based формата**
  - Автоматическая конвертация в keyframes
  - Создается keyframe для каждого уникального момента времени

**Структура файлов:**
```
animations/
├── manifests/
│   └── my_animation.yaml    # Манифест с keyframes
└── frames/
    └── my_animation/
        ├── kf000_wheel_front_left.png    # Keyframe 0, левое переднее
        ├── kf000_main_display.png        # Keyframe 0, главный дисплей
        ├── kf001_wheel_front_left.png    # Keyframe 1
        └── ...
```

**Формат манифеста (новый):**
```yaml
name: happy_face
description: Happy robot face animation
duration_ms: 1000
fps: 10
loop: true
format: keyframe  # Маркер нового формата
keyframes:
  - time_ms: 0
    panels:
      - panel: main_display
        image: frames/happy_face/kf000_main_display.png
        active: true
  - time_ms: 500
    panels:
      - panel: main_display
        image: frames/happy_face/kf001_main_display.png
        active: true
```

## 🎯 Панели робота

### Расположение LED матриц

```
         ↑ FRONT ↑
    ┌─────────────────┐
    │   MOUTH (25×5)  │ ← Главный дисплей (🟢 Main Display)
    │   ┌─────────┐   │    Выразительное лицо, текст
    │   └─────────┘   │
FL  │                 │  FR ← Передние колеса (🔴 FL/FR Wheel)
8×8 │                 │ 8×8   Индикаторы, анимации
    │                 │
    │     КОРПУС      │
    │                 │
RL  │                 │  RR ← Задние колеса (🔵 RL/RR Wheel)
8×8 │                 │ 8×8   Поворотники, стоп-сигналы
    └─────────────────┘
```

### Размеры панелей

| Панель | Размер | Пиксели | Применение |
|--------|--------|---------|------------|
| wheel_front_left | 8×8 | 64 | Левое переднее - анимации, индикаторы |
| wheel_front_right | 8×8 | 64 | Правое переднее - анимации, индикаторы |
| wheel_rear_left | 8×8 | 64 | Левое заднее - поворотники, стоп |
| wheel_rear_right | 8×8 | 64 | Правое заднее - поворотники, стоп |
| main_display | 25×5 | 125 | Главный дисплей - лицо, текст, эмоции |

## ⌨️ Горячие клавиши

| Клавиша | Действие |
|---------|----------|
| `Ctrl+N` | Новая анимация |
| `Ctrl+O` | Открыть анимацию |
| `Ctrl+S` | Сохранить |
| `Ctrl+Shift+S` | Сохранить как |
| `Ctrl+Q` | Выход |
| Click on canvas | Нарисовать пиксель |
| Drag on canvas | Рисовать линию |

## 💡 Советы и приёмы

### Создание симметричных анимаций для колес

1. Нарисуйте анимацию для **FL Wheel** (левое переднее)
2. Сохраните файл
3. Откройте в редакторе картинок, отзеркальте по горизонтали
4. Используйте для **FR Wheel** (правое переднее)

### Повторное использование паттернов

1. Создайте базовую анимацию (например, вращение)
2. Активируйте все 4 колеса
3. Нарисуйте одинаковый паттерн
4. Сохраните как шаблон

### Плавные переходы

- Используйте промежуточные keyframes
- Пример: 0ms (закрыто) → 250ms (полуоткрыто) → 500ms (открыто)
- Большее количество keyframes = более плавная анимация

### Синхронизация панелей

- Добавьте keyframe в конкретный момент (например, 500ms)
- Активируйте несколько панелей (например, все 4 колеса)
- Нарисуйте на всех - они будут синхронизированы по времени

## 🐛 Решение проблем

### Чекбоксы панелей неактивны (серые)

**Проблема:** Не могу включить панели  
**Решение:** Сначала добавьте keyframe кнопкой "Add Keyframe"

### Не могу рисовать на панели

**Проблема:** Клики по canvas не работают  
**Решение:** 
1. Убедитесь что keyframe добавлен
2. Включите чекбокс нужной панели в правой части (🎨 Active Panels)
3. Только активные панели доступны для редактирования

### Анимация не воспроизводится

**Проблема:** Play не работает  
**Решение:**
1. Убедитесь что есть хотя бы 2 keyframes
2. Проверьте что Duration > 0
3. Убедитесь что хотя бы одна панель активна

### Старая анимация не открывается

**Проблема:** Ошибка при открытии .yaml файла  
**Решение:** 
- Редактор автоматически конвертирует старый формат
- Если ошибка persist, проверьте консоль на Python traceback
- Убедитесь что все PNG файлы на месте

### Python ошибки при запуске

**Windows:**
```powershell
# Переустановить зависимости
pip uninstall pillow numpy pyyaml -y
pip install pillow numpy pyyaml
```

**Linux:**
```bash
pip3 install --user --upgrade pillow numpy pyyaml
```

## 🎓 Примеры workflow

### Пример 1: Happy Face (простая эмоция)

1. New Animation: name=`happy_face`, fps=10, loop=true
2. Add Keyframe (0ms)
3. Enable: 🟢 Main Display
4. Select: White color
5. Draw eyes: два пикселя в верхней части
6. Draw smile: изогнутая линия внизу
7. Save

### Пример 2: Turn Signal (поворотник)

1. New Animation: name=`turn_left`, fps=10, loop=true
2. Add Keyframe (0ms)
   - Enable: 🔵 RL Wheel (заднее левое)
   - Select: Yellow
   - Draw: стрелка влево
3. Add Keyframe (500ms)
   - Same, but brighter or full fill
4. Duration = 1000ms
5. Save

### Пример 3: Synchronized Wheel Spin

1. New Animation: name=`wheel_spin`, fps=20, loop=true
2. Add Keyframe (0ms)
   - Enable: все 4 колеса
   - Draw: вертикальная линия
3. Add Keyframe (50ms)
   - Draw: диагональ /
4. Add Keyframe (100ms)
   - Draw: горизонталь —
5. Add Keyframe (150ms)
   - Draw: диагональ \
6. Duration = 200ms
7. Save

### Пример 4: Talking Animation

1. New Animation: name=`talking`, fps=15, loop=true
2. Keyframe 0ms: рот закрыт (small line)
3. Keyframe 100ms: рот полуоткрыт (bigger line)
4. Keyframe 200ms: рот открыт (wide mouth)
5. Keyframe 300ms: полуоткрыт
6. Keyframe 400ms: закрыт
7. Duration = 500ms

## 📦 Экспорт и использование в ROS2

После создания анимации в редакторе:

1. Сохраните в `src/rob_box_animations/animations/manifests/`
2. Пересоберите ROS2 пакет:
   ```bash
   cd ~/rob_box_project
   colcon build --packages-select rob_box_animations
   ```
3. Используйте в ROS2:
   ```python
   from rob_box_animations import AnimationPlayer
   
   player = AnimationPlayer()
   player.load_animation('happy_face')
   player.play()
   ```

## 🔗 Дополнительные ресурсы

- **Основной README:** [tools/animation_editor/README.md](../../tools/animation_editor/README.md)
- **Windows установка:** [tools/animation_editor/INSTALL_WINDOWS.md](../../tools/animation_editor/INSTALL_WINDOWS.md)
- **Примеры анимаций:** `src/rob_box_animations/animations/manifests/`

## 📧 Поддержка

Если возникли проблемы:
1. Проверьте раздел "🐛 Решение проблем" выше
2. Откройте issue на GitHub
3. Приложите traceback из консоли (если есть)
