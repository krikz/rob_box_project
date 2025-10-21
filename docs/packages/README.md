# 📦 Документация ROS 2 пакетов

Документация пакетов проекта Rob Box.

## 🎤 Voice Assistant - rob_box_voice

**Полная документация:** [`src/rob_box_voice/docs/`](../../src/rob_box_voice/docs/)

AI голосовой ассистент с DeepSeek, Vosk STT, Silero TTS.

### Основные документы:
- [**README.md**](../../src/rob_box_voice/README.md) - Обзор пакета
- [**ARCHITECTURE_OVERVIEW.md**](../../src/rob_box_voice/docs/ARCHITECTURE_OVERVIEW.md) - Архитектура системы
- [**PHASE3_STT_IMPLEMENTATION.md**](../../src/rob_box_voice/docs/PHASE3_STT_IMPLEMENTATION.md) - STT нода
- [**PHASE4_SOUND_IMPLEMENTATION.md**](../../src/rob_box_voice/docs/PHASE4_SOUND_IMPLEMENTATION.md) - Sound нода
- [**PHASE5_COMMAND_IMPLEMENTATION.md**](../../src/rob_box_voice/docs/PHASE5_COMMAND_IMPLEMENTATION.md) - Command нода + Nav2

### Руководства пользователя:
- [**SOUND_EFFECTS_QUICKSTART.md**](SOUND_EFFECTS_QUICKSTART.md) - Быстрый старт со звуками
- [**SOUND_EFFECTS_INTEGRATION.md**](SOUND_EFFECTS_INTEGRATION.md) - Интеграция звуковых эффектов
- [**MAPPING_COMMANDS.md**](MAPPING_COMMANDS.md) - Голосовые команды картографии
- [**MAPPING_COMMANDS_SUMMARY.md**](MAPPING_COMMANDS_SUMMARY.md) - Краткое резюме команд
- [**MAPPING_COMMANDS_TESTING.md**](MAPPING_COMMANDS_TESTING.md) - Тестирование команд

### Разработка:
- [**CUSTOM_TTS_TRAINING.md**](../../src/rob_box_voice/docs/CUSTOM_TTS_TRAINING.md) - Тренировка кастомного голоса
- [**SILERO_QUICK_START.md**](../../src/rob_box_voice/docs/SILERO_QUICK_START.md) - Быстрый старт Silero TTS
- [**STT_TTS_QUICK_REFERENCE.md**](../../src/rob_box_voice/docs/STT_TTS_QUICK_REFERENCE.md) - Справочник STT/TTS

## 🎨 Animations - rob_box_animations

**Полная документация:** [`src/rob_box_animations/docs/`](../../src/rob_box_animations/docs/)

Система LED анимаций для WS2812B матриц.

### Основные документы:
- [**README.md**](../README.md) - Обзор пакета
- [**ANIMATIONS.md**](../../src/rob_box_animations/docs/ANIMATIONS.md) - Полная документация системы
- [**AUDIO_REACTIVE.md**](../../src/rob_box_animations/AUDIO_REACTIVE.md) - Аудио-реактивные анимации

## 🚀 Системные пакеты

### rob_box_bringup
Запуск системы, launch файлы.

**Документация:** [`src/rob_box_bringup/`](../../src/rob_box_bringup/)

### rob_box_description
URDF модель робота, Gazebo симуляция.

**Документация:** [`src/rob_box_description/`](../../src/rob_box_description/)

## 🔌 Интеграционные пакеты

### vesc_nexus
Интеграция VESC моторных контроллеров.

**Документация:** [github.com/krikz/vesc_nexus](https://github.com/krikz/vesc_nexus)

### robot_sensor_hub_msg
ROS 2 сообщения для ESP32 сенсорного хаба.

**Документация:** [`src/robot_sensor_hub_msg/README.md`](../../src/robot_sensor_hub_msg/README.md)

### ros2leds
Драйвер WS2812B LED лент.

**Документация:** [github.com/krikz/ros2leds](https://github.com/krikz/ros2leds)

### led_matrix_driver
Драйвер LED матриц для отображения анимаций.

**Документация:** [`src/led_matrix_driver/`](../../src/led_matrix_driver/)

## 📝 Правила документации пакетов

Каждый ROS 2 пакет должен содержать:

1. **README.md** в корне пакета:
   - Назначение и функциональность
   - Зависимости
   - Описание нод (topics, services, parameters)
   - Примеры использования

2. **docs/** директория (опционально):
   - Детальная документация
   - Диаграммы архитектуры
   - Руководства по разработке
   - Фазы реализации (для больших фич)

3. **package.xml** с корректными метаданными

4. **.rosdoc2.yaml** (опционально) для автогенерации документации

## 🔗 Связанные документы

- [Архитектура системы](../architecture/SYSTEM_OVERVIEW.md)
- [Software компоненты](../architecture/SOFTWARE.md)
- [Руководства по настройке](../guides/)

---

**Навигация:** [← Назад в docs/](../README.md)
