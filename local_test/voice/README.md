# Voice Assistant Test Scripts

Тестовые скрипты для разработки голосового ассистента Rob Box.

## Категории тестов

### 🗣️ Dialogue / LLM тесты
- **test_deepseek_methods.py** - тест разных методов подключения к DeepSeek API
- **test_dialogue_simple.py** - простой тест dialogue node
- **test_qwen_dialogue.py** - тест Qwen API с streaming
- **test_qwen_streaming.py** - детальный тест streaming ответов Qwen
- **test_qwen_plain_text.py** - тест plain text режима без JSON
- **test_qwen_native.py** - тест нативного Qwen API
- **test_qwen_full_prompt.py** - тест с полным промптом
- **test_qwen_enable_search.py** - тест web search в Qwen
- **test_streaming.py** - общий тест streaming

### 🎤 STT (Speech-to-Text) тесты
- **test_vosk_simple.py** - простой тест Vosk распознавания
- **test_stt_launch.py** - launch файл для STT node

### 🔊 Sound/Audio тесты
- **test_sound_node.py** - тест audio node (v1)
- **test_sound_node_v2.py** - тест audio node (v2)
- **test_sound_comprehensive.py** - комплексный тест аудио системы
- **test_short_prompt.py** - тест с коротким промптом

### 🚀 Integration тесты
- **test_voice_assistant.launch.py** - полный launch файл для голосового ассистента
  - STT → Dialogue (DeepSeek) → TTS → Sound

## Запуск

### Один тест
```bash
cd /home/ros2/rob_box_project
source install/setup.bash
python3 local_test/voice/test_qwen_dialogue.py
```

### Launch файл
```bash
ros2 launch local_test/voice/test_voice_assistant.launch.py
```

## Требования
- DeepSeek API key: `source src/rob_box_voice/.env.secrets`
- Qwen API key (для Qwen тестов)
- Vosk модель (для STT)
- ReSpeaker микрофон (для аудио тестов)
