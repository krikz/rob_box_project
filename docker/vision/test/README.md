# Интеграционные тесты Voice Assistant

Контейнерная тест-среда для `dialogue_node` — тестирует логику агентского цикла без реального железа.

## Архитектура

```
┌─────────────────────────────────────────────────────────┐
│                  docker-compose.test.yml                │
│                                                         │
│  ┌──────────────┐  ROS2/Zenoh  ┌──────────────────────┐ │
│  │  voice-test  │◄────────────►│  scenario-runner     │ │
│  │              │              │                      │ │
│  │  dialogue_node (only)       │  1. inject STT msg   │ │
│  │  ARM image via QEMU  ←opt   │  2. wait response    │ │
│  │                    ┐        │  3. assert           │ │
│  │  /voice/stt/result │        └──────────────────────┘ │
│  │  /audio/vad        │                                 │
│  │  /voice/dialogue/response                            │
│  │  /voice/animation/request                            │
│  └──────────────┘                                       │
│         │ HTTP OpenAI /v1/chat/completions              │
│  ┌──────────────┐                                       │
│  │   mock-llm   │  FastAPI, скриптованные ответы        │
│  │  :8765       │  via YAML rules или scripted queue    │
│  └──────────────┘                                       │
│  ┌──────────────┐                                       │
│  │ zenoh-router │  Такой же как в prod                  │
│  └──────────────┘                                       │
└─────────────────────────────────────────────────────────┘
```

## Быстрый старт

```bash
# amd64 (быстро, image должен быть собран для amd64)
cd docker/vision/test
make test

# ARM image через QEMU (тестирует реальный prod образ)
make test-arm
```

## Структура

```
test/
├── docker-compose.test.yml   # Главный compose файл
├── Makefile                  # Удобные команды
├── mock_llm/
│   ├── Dockerfile
│   └── server.py             # FastAPI OpenAI-compatible мок
├── scenario_runner/
│   ├── Dockerfile
│   ├── runner.py             # Исполнитель сценариев (rclpy)
│   └── scenarios/
│       ├── 01_basic_dialogue.yaml
│       ├── 02_tool_call.yaml
│       └── 03_robustness.yaml
├── config/
│   ├── start_voice_test.sh        # Запускает только dialogue_node
│   ├── voice_assistant_test.yaml  # ROS2 params: mock LLM, без wake word
│   ├── zenoh_router_test.json5    # Роутер для теста
│   └── zenoh_test_session.json5   # Сессия для всех тест-контейнеров
└── results/                  # JSON результаты (gitignored)
```

## Формат сценария

```yaml
# scenarios/my_test.yaml
llm_rules:
  - trigger: "привет"       # если в запросе есть это слово
    response: "Привет!"     # → LLM вернёт этот текст

default_response: "Понял."

scenarios:
  - name: greeting_test
    steps:
      - inject_stt: "привет"        # публикуем в /voice/stt/result
        timeout_s: 15               # ждём ответ
        assert_response_contains:   # проверяем содержимое
          - "Привет"

      - set_llm_responses:          # задаём очередь точных ответов
          - "Точный ответ первый"
          - "Точный ответ второй"
      - inject_stt: "любой вопрос"
        assert_response_contains: "Точный ответ"

      - inject_vad: true            # VAD прерывание
      - wait_ms: 200
      - inject_vad: false
```

### Поддерживаемые поля шага

| Поле | Описание |
|------|----------|
| `inject_stt: "text"` | Публикует в `/voice/stt/result` |
| `inject_vad: true/false` | Публикует в `/audio/vad` |
| `wait_ms: 500` | Пауза в миллисекундах |
| `set_llm_responses: [...]` | Загружает очередь ответов в mock-llm |
| `timeout_s: 15` | Таймаут ожидания ответа |
| `assert_response_contains: str/list` | Проверяет текст ответа |
| `assert_animation: "name"` | Проверяет `/voice/animation/request` |
| `assert_no_response: true` | Убеждается что ответа НЕТ |
| `fail_fast: false` | Продолжать сценарий после падения шага |

## QEMU (тест на реальном ARM образе)

```bash
# Установить QEMU один раз
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

# Запустить с ARM образом
IMAGE_TAG=test PLATFORM=linux/arm64 make test-arm
```

В GitHub Actions:
```yaml
- uses: docker/setup-qemu-action@v3
  with:
    platforms: arm64
- run: |
    cd docker/vision/test
    IMAGE_TAG=${{ env.IMAGE_TAG }} PLATFORM=linux/arm64 make test-arm
```

## Добавить свой сценарий

1. Создать `scenario_runner/scenarios/04_my_feature.yaml`
2. Определить `llm_rules` для mock-ответов
3. Написать шаги с `inject_stt` + `assert_response_contains`
4. `make test`

Сценарии выполняются в алфавитном порядке имён файлов.
