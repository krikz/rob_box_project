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
| command_node.yaml   | command_node   | Nav2 команды (waypoints — через MCP)    |
| speaker_id_node.yaml| speaker_id_node | Голосовая биометрия (issue #1077)       |

## voice_assistant.yaml (сводка)

Монолитный voice_assistant.yaml больше НЕ является рабочим конфигом
(с 2026-08-05, issue #1004 fix). Оставлен как человекочитаемая **сводка**
в стандартном ROS 2 формате (верхний ключ = имя ноды, `ros__parameters:`
внутри). Живые конфиги — per-node файлы выше.

Старый формат (`/**: ros__parameters: <node>: <param>`) создавал
dotted-имена (`dialogue_node.llm_provider`), которые ноды НЕ читали —
весь YAML молча игнорировался (rclpy отбрасывает необъявленные ключи без
ошибки). Не возвращайтесь к нему.

## Docker-деплой (Vision Pi)

Операторские конфиги: `docker/vision/config/voice_assistant/<node>.yaml`
(монтируются в `/config/voice_assistant` внутри контейнера и читаются
`voice_assistant_headless.launch.py` через `config_dir:=/config/voice_assistant`).
Правки применяются при рестарте контейнера.

## Правило (1:1)

* Каждый ключ в YAML ноды должен быть объявлен через `declare_parameter`
  в коде этой ноды (иначе rclpy его молча игнорирует — класс бага issue #1004).
* Регрессионный тест: `test/test_yaml_param_consistency.py`.
* Меняйте per-node файлы для реальных параметров работающей ноды.
