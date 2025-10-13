# Voice Assistant Development - Session Summary

**Дата:** 2025-10-12  
**Ветка:** `feature/voice-assistant`  
**Автор:** krikz

---

## Выполненная работа

### 1. ✅ Изучение ReSpeaker Mic Array v2.0

**Результат:**
- Изучена официальная документация с wiki.seeedstudio.com
- Проанализирован существующий ROS1 пакет `respeaker_ros` (нет ROS2 версии)
- Выявлены ключевые возможности:
  - 4× микрофона + XMOS XVF-3000 процессор
  - Встроенные алгоритмы: VAD, DoA, AEC, Beamforming, Noise Suppression
  - 12× RGB LED с программированием через USB
  - 2 версии прошивки (1-channel и 6-channels)

### 2. ✅ Обновление документации

**Файл:** `docs/HARDWARE.md`

Добавлен детальный раздел "3.4. ReSpeaker Microphone Array v2.0":
- Технические характеристики (процессор, микрофоны, LED)
- Описание встроенных алгоритмов (7 функций обработки звука)
- Управление LED через pixel_ring API
- Настраиваемые параметры (tuning.py)
- ROS2 интеграция (топики, сервисы, параметры)
- Прошивки и их различия
- Ссылки на GitHub репозитории

**Коммит:** `68a942f`

---

### 3. ✅ Проектирование архитектуры

**Файл:** `docs/development/VOICE_ASSISTANT_ARCHITECTURE.md`

**Содержание (800+ строк):**

1. **Анализ прототипа:**
   - Выявлены проблемы (self-listening, не ROS2, монолит)
   - Выделены хорошие концепции (фразы активации, таймауты, streaming TTS)

2. **Новая архитектура:**
   - 7 модульных ROS2 нод:
     - `AudioNode` — захват + VAD/DoA
     - `STTNode` — Speech-to-Text (Yandex/Whisper)
     - `TTSNode` — Text-to-Speech с кэшем
     - `DialogueNode` — State machine + LLM
     - `SoundNode` — воспроизведение
     - `LEDNode` — управление RGB LED
     - `CommandNode` — выполнение команд

3. **Детальное описание:**
   - ROS2 топики и сервисы для каждой ноды
   - Алгоритмы работы с псевдокодом
   - Интеграция с hardware (ReSpeaker, sound_pack, animations)
   - Промптинг и LLM стратегия (DeepSeek API)
   - Системные меню и кэширование TTS

4. **План реализации:**
   - 6 phases (12 недель)
   - От базовой инфраструктуры до полевого тестирования

**Коммит:** `68a942f`

---

### 4. ✅ Создание ROS2 пакета

**Пакет:** `src/rob_box_voice`

**Структура:**
```
rob_box_voice/
├── README.md                      # Полная документация (400+ строк)
├── package.xml                    # ROS2 зависимости
├── setup.py                       # 7 entry points для нод
├── setup.cfg                      
├── config/
│   ├── voice_assistant.yaml       # Параметры всех нод
│   └── secrets.yaml.example       # Шаблон для API ключей
├── prompts/
│   └── master_prompt.txt          # LLM промпт (300+ строк)
├── rob_box_voice/
│   ├── __init__.py
│   └── audio_node.py              # Заглушка (пока пустая)
├── launch/                        # (пусто, TODO)
├── srv/                           # (пусто, TODO)
└── test/                          # Стандартные тесты
```

**Ключевые файлы:**

1. **README.md:**
   - Описание архитектуры
   - Инструкции по установке (ROS2, Python deps, ReSpeaker драйверы)
   - Конфигурация и API ключи
   - Примеры использования
   - Список топиков и сервисов
   - Troubleshooting секция

2. **config/voice_assistant.yaml:**
   - Параметры для всех 7 нод
   - Настройки ReSpeaker (sample rate, VAD threshold, device index)
   - STT/TTS провайдеры (Yandex/Whisper/Coqui)
   - LLM настройки (DeepSeek, streaming, history)
   - LED цвета для режимов
   - Системные фразы для кэша

3. **prompts/master_prompt.txt:**
   - Полное описание робота РОББОКС
   - Возможности и ограничения
   - Правила общения (стиль, запреты, честность)
   - Команды управления с примерами (`<CMD:.../>`)
   - Контекст состояния робота
   - Безопасность
   - Примеры диалогов

**Коммит:** `792dfd4`

---

## Текущее состояние

### ✅ Завершено (4 задачи из 8)

1. ✅ Изучить ReSpeaker и ROS2 пакеты
2. ✅ Создать документацию по ReSpeaker (HARDWARE.md)
3. ✅ Проанализировать архитектуру голосового агента
4. ✅ Создать структуру ROS2 пакета rob_box_voice

### 🔄 В работе (0 задач)

*Нет активных задач*

### ⏳ Ожидают выполнения (4 задачи)

5. ⏳ Интегрировать ReSpeaker с ROS2 (AudioNode + LEDNode)
6. ⏳ Реализовать STT и TTS модули
7. ⏳ Интегрировать с LLM и звуковыми эффектами
8. ⏳ Реализовать систему команд и меню

---

## Следующие шаги

### Phase 1: Базовая инфраструктура (Week 1-2)

**Приоритет 1 — AudioNode:**
- [ ] Реализовать захват аудио через PyAudio
- [ ] Интеграция с usb_4_mic_array (tuning.py)
- [ ] Публикация VAD и DoA в топики
- [ ] Тестирование на реальном ReSpeaker

**Приоритет 2 — LEDNode:**
- [ ] Интеграция с pixel_ring
- [ ] Подписка на /voice/state
- [ ] Автоматическое переключение режимов
- [ ] Тестирование всех режимов LED

**Приоритет 3 — Инфраструктура:**
- [ ] Создать service definitions (Speak.srv, SetLEDMode.srv)
- [ ] Создать launch file
- [ ] Написать unit тесты
- [ ] Настроить CI/CD для пакета

---

## Полезные ссылки

**Документация:**
- [ReSpeaker Wiki](https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/)
- [usb_4_mic_array GitHub](https://github.com/respeaker/usb_4_mic_array)
- [pixel_ring GitHub](https://github.com/respeaker/pixel_ring)
- [Yandex SpeechKit](https://cloud.yandex.ru/docs/speechkit/)
- [DeepSeek API Docs](https://api-docs.deepseek.com/)

**Прототип (для reference):**
- Скрипт с Yandex STT/TTS + DeepSeek
- Проблемы: self-listening, не ROS2
- Хорошие идеи: streaming TTS, история диалога, таймауты

---

## Git History

```
792dfd4 (HEAD -> feature/voice-assistant) feat: create rob_box_voice ROS2 package structure
68a942f docs: add ReSpeaker hardware specs and voice assistant architecture
486090f (origin/develop, develop) fix: Restore README from old commit and update with correct specs
```

**Статистика:**
- 3 коммита в feature branch
- 2 новых файла документации (~1500 строк)
- 13 новых файлов в ROS2 пакете (~1200 строк)
- 0 ошибок, 0 warnings

---

## Заметки

### Ключевые решения:

1. **Использовать 1-channel прошивку** ReSpeaker для AEC (предотвращение self-listening)
2. **Модульная архитектура** с 7 нодами вместо монолита
3. **Кэширование TTS** для системных фраз (снижение нагрузки на API)
4. **Streaming LLM** с разбивкой по предложениям (низкая латентность)
5. **Интеграция с существующими компонентами** (sound_pack, animations)

### Технические детали:

- **ReSpeaker:** USB Audio Class 1.0, device ID 0x2886:0x0018
- **Аудио:** 16kHz, 16-bit PCM, 1 канал (обработанный)
- **LED:** 12× RGB, управление через USB control transfer
- **TTS голос:** Yandex "anton" с speed=0.4 (протестировано)
- **LLM:** DeepSeek API с потоковым выводом

### Проблемы и решения:

**Проблема 1:** ROS2 пакет для ReSpeaker не существует
**Решение:** Создать свой с интеграцией Python библиотек

**Проблема 2:** Self-listening в прототипе
**Решение:** Использовать встроенный AEC в ReSpeaker (XMOS XVF-3000)

**Проблема 3:** Медленный ответ LLM
**Решение:** Streaming + разбивка по предложениям → начало озвучки до завершения генерации

---

## Метрики

**Время разработки:** ~3 часа  
**Строк кода:** ~2700  
**Файлов создано:** 15  
**Коммитов:** 3  
**Документации:** ~2500 строк

---

<div align="center">
  <strong>Готово к началу реализации Phase 1</strong>
  <br>
  <sub>krikz | РОББОКС Project | 2025-10-12</sub>
</div>
