# rob_box_voice/config

Per-node ROS2 parameter files (issue #1004 fix, ADR-0004).

## Структура (по нодам)

| Файл                | Нода           | Назначение                              |
|---------------------|----------------|------------------------------------------|
| audio_node.yaml     | audio_node     | ReSpeaker VAD, sample rate, эхо-гейты    |
| stt_node.yaml       | stt_node       | Vosk/Yandex STT, fallback/retry (979)    |
| tts_node.yaml       | tts_node       | Yandex/Silero/MiniMax TTS provider       |
| dialogue_node.yaml  | dialogue_node  | LLM (MiniMax primary, DeepSeek fallback) |
| sound_node.yaml     | sound_node     | Sound effects pack, volume, animation   |
| led_node.yaml       | led_node       | LED matrix brightness/colors            |
| command_node.yaml   | command_node   | Nav2 команды, waypoints                 |

## voice_assistant.yaml (легаси)

Монолитный voice_assistant.yaml со вложенной структурой остаётся здесь как
"single source of truth" для:

1. **Тестов** `test_dialogue_shell.py::test_both_voice_configs_route_dialogue_to_deepseek`
   — проверяет, что source и docker копии экспонируют одинаковые параметры.
2. **Внешних скриптов** (`scripts/record_yandex_voice_v1_old.py` и др.), которые
   читают общий конфиг.
3. **Operators**, которые привыкли редактировать один файл.

launch файлы (`voice_assistant.launch.py`, `basic_test.launch.py`,
`voice_assistant_headless.launch.py`) теперь грузят **per-node** YAML в каждую
ноду — это правильный ROS2 паттерн, тот же, что в `led_matrix_driver` /
`led_matrix_compositor`.

## Правило

* **Меняйте per-node файлы** для реальных параметров работающей ноды.
* **Меняйте voice_assistant.yaml**, только если добавляете новые параметры,
  которые должны попасть во все ноды — и не забудьте продублировать их в
  per-node файлы соответствующих нод.
