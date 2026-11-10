<!-- stale-paths: ignore-file (описывает удалённое или старую раскладку; кандидат в архив или переписывание, ADR-0163) -->
# rob_box Animation Editor

Полнофункциональный редактор анимаций для LED матриц робота rob_box с поддержкой ключевых кадров.

## 🚀 Быстрая установка (Windows)

**Для пользователей Windows - установка одной командой!**

Откройте PowerShell и выполните:

```powershell
irm https://raw.githubusercontent.com/krikz/rob_box_project/feature/voice-assistant/tools/animation_editor/install_and_run.ps1 | iex
```

Скрипт автоматически:
- ✅ Проверит Python 3.8+
- ✅ Скачает редактор
- ✅ Установит зависимости
- ✅ Запустит редактор

См. [INSTALL_WINDOWS.md](./INSTALL_WINDOWS.md) для альтернативных способов установки.

## 📦 Установка (Linux/macOS)

### Зависимости

```bash
pip install pillow numpy pyyaml
```

### Быстрый запуск

```bash
# Из корня проекта rob_box_project
python3 tools/animation_editor/main.py --animations-dir src/rob_box_animations/animations
```

## Возможности

✨ **Основные функции:**
- 🎨 Редактирование пикселей на всех LED матрицах (4 колеса + главный дисплей)
- 🎬 Timeline с ключевыми кадрами (keyframes)
- 🖌️ Палитра цветов с 8 основными цветами
- ⏱️ Настройка времени для каждого ключевого кадра
- 💾 Сохранение и загрузка анимаций (поддержка старого формата)
- ▶️ Предпросмотр анимаций
- 🔄 Автоматическая конвертация из старого формата

## Использование

### Основной workflow (keyframe-based)

1. **Создание новой анимации**: File → New Animation
2. **Добавление ключевого кадра**: Кнопка "Add Keyframe" в Timeline
3. **Активация панелей**: Включите чекбоксы нужных панелей справа (🔴 Wheels, 🟢 Main Display)
4. **Редактирование пикселей**:
   - Выберите цвет в палитре слева
   - Кликните на LED в активной панели
   - Перетаскивайте мышь для рисования
5. **Добавление следующих кадров**: Переместите слайдер времени, добавьте новый keyframe
6. **Предпросмотр**: Кнопка "Play" в toolbar
7. **Сохранение**: File → Save или Ctrl+S

### Горячие клавиши

- `Ctrl+N` - Новая анимация
- `Ctrl+O` - Открыть
- `Ctrl+S` - Сохранить
- `Ctrl+Shift+S` - Сохранить как
- `Ctrl+Q` - Выход

### Панели робота

- **wheel_front_left** (8×8) - Левое переднее колесо
- **wheel_front_right** (8×8) - Правое переднее колесо
- **wheel_rear_left** (8×8) - Левое заднее колесо
- **wheel_rear_right** (8×8) - Правое заднее колесо
- **main_display** (25×5) - Главный дисплей (рот)
- **wheels_front** (16×8) - Оба передних колеса
- **wheels_rear** (16×8) - Оба задних колеса
- **wheels_all** (16×16) - Все 4 колеса

## Структура проекта

```
animation_editor/
├── __init__.py          # Package init
├── app.py               # Главное приложение
├── models.py            # Data models (Animation, Frame, Panel)
├── canvas.py            # Robot Canvas для отрисовки
├── timeline.py          # Timeline компонент
├── palette.py           # Color Palette компонент
main.py                  # Entry point
setup.py                 # Setup script
README.md                # Эта документация
```

## Архитектура

### Models
- **Frame** - один кадр анимации (numpy array RGB)
- **Panel** - панель с коллекцией кадров
- **Animation** - анимация с несколькими панелями

### GUI Components
- **RobotCanvas** - визуализация робота с LED матрицами
- **Timeline** - управление кадрами
- **ColorPalette** - выбор цвета
- **AnimationEditorApp** - главное окно приложения

## Формат файлов

Анимации сохраняются в формате YAML manifest + PNG frames:

```
animations/
├── manifests/
│   └── my_animation.yaml
└── frames/
    └── my_animation/
        ├── wheel_front_left_frame_000.png
        ├── wheel_front_left_frame_001.png
        └── ...
```

### Пример manifest:

```yaml
name: my_animation
description: Example animation
duration_ms: 1000
fps: 10
loop: true
panels:
  - logical_group: wheel_front_left
    offset_ms: 0
    frames:
      - image: frames/my_animation/wheel_front_left_frame_000.png
        duration_ms: 100
      - image: frames/my_animation/wheel_front_left_frame_001.png
        duration_ms: 100
```

## TODO / Roadmap

- [ ] Воспроизведение анимаций в реальном времени
- [ ] Eyedropper tool для выбора цвета из кадра
- [ ] Onion skinning (показывать предыдущий/следующий кадр полупрозрачным)
- [ ] Импорт изображений/GIF
- [ ] Экспорт в GIF/видео
- [ ] Библиотека готовых паттернов
- [ ] Undo/Redo
- [ ] Copy/Paste frames
- [ ] Поиск и замена цветов
- [ ] Пакетное редактирование

## Зависимости

- Python 3.8+
- tkinter (обычно включен в Python)
- numpy
- Pillow
- PyYAML

## License

MIT License - см. LICENSE файл в корне проекта

## Contributing

Pull requests приветствуются! Для крупных изменений сначала создайте issue для обсуждения.
