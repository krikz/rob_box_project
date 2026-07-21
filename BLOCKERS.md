# BLOCKERS — t_df5ba9d3 (локальный mock MiniMax TTS)

> **Статус:** зафиксировано как blocker в рамках задачи t_df5ba9d3.
> **Правило:** правки **НЕ вносятся** в production-код (`src/rob_box_voice/`,
> `src/rob_box_llm/`). Задача просит блокирующее решение переключения на
> mock без правок прода — но поскольку без минимальной правки это невозможно,
> описываем блокер здесь.

## TL;DR

Для того, чтобы `tts_node` мог переключиться на локальный mock
(`tools/mock_minimax_server.py`) **через переменные окружения
`MINIMAX_BASE_URL` и `MINIMAX_API_KEY`**, нужно расширить
`MiniMaxTTSProvider.__init__` (а заодно `tts_node.py`) на ~5 строк:

1. В `MiniMaxTTSProvider.__init__`:
   ```python
   # Уже есть:
   self._api_key = api_key or os.getenv("MINIMAX_API_KEY") or ""
   self._group_id = group_id or os.getenv("MINIMAX_GROUP_ID") or ""
   # Добавить:
   self._base_url = (
       base_url
       if base_url != self.DEFAULT_BASE_URL
       else os.getenv("MINIMAX_BASE_URL", self.DEFAULT_BASE_URL)
   ).rstrip("/")
   ```

2. В `tts_node.py` (метод `_synthesize_minimax_async`, lazy init):
   ```python
   self.minimax_provider = MiniMaxTTSProvider(
       api_key=self.minimax_api_key,
       group_id=self.minimax_group_id,
       base_url=os.getenv("MINIMAX_BASE_URL", MiniMaxTTSProvider.DEFAULT_BASE_URL),
       default_voice=self.minimax_voice,
       default_model=self.minimax_model,
       timeout=self.minimax_timeout,
   )
   ```

Эти две правки делают переключение возможным **без рестарта tts_node,
без правок ROS-параметров, без затрагивания архитектуры** — просто
выставить `MINIMAX_BASE_URL=http://127.0.0.1:18080` в env процесса.

## Почему это blocker (а не "просто сделать")

Задача явно говорит:

> Поддержать инжектирование через переменные окружения `MINIMAX_BASE_URL` и
> `MINIMAX_API_KEY` (значения-заглушки), чтобы tts_node можно было переключить
> на mock без правок продакшн-кода.
>
> **Если переключение требует правок кода — зафиксировать как blocker в
> отдельном файле BLOCKERS.md, но НЕ вносить изменения в прод.**

Текущее состояние кода:

| Слой | Что делает | Поведение |
|---|---|---|
| `MiniMaxTTSProvider.__init__` (`src/rob_box_llm/rob_box_llm/providers/minimax_tts.py:332-353`) | Принимает `base_url: str = DEFAULT_BASE_URL`, **не читает env** | Зашит на `https://api.minimax.io`, если в конструктор не передали явно |
| `MiniMaxTTSProvider._post` (`:392`) и `.stream` (`:555`) | `url = f"{self._base_url}/v1/t2a_v2"` | Использует `self._base_url`, который нельзя поменять без правки конструктора |
| `tts_node._synthesize_minimax_async` (`src/rob_box_voice/rob_box_voice/tts_node.py:1148-1156`) | Lazy init `MiniMaxTTSProvider(...)` **без `base_url=`** | Всегда идёт на DEFAULT_BASE_URL. Никакого env-чтения нет. |

### Доказательство

```bash
$ grep -n "base_url\|MINIMAX_BASE_URL" src/rob_box_llm/rob_box_llm/providers/minimax_tts.py | grep -v "DEFAULT_BASE_URL\|base_url:\|self._base_url"
# (пусто — НЕТ чтения из env)
```

```bash
$ grep -n "MINIMAX_BASE_URL" src/rob_box_voice/rob_box_voice/tts_node.py
# (пусто — tts_node тоже не передаёт base_url, не читает env)
```

```bash
$ grep -rn "MINIMAX_BASE_URL" src/
# Найдено только в src/rob_box_llm/test/conftest.py (тест-константа)
# и в src/rob_box_llm/test/test_minimax_tts_request_params_and_leak_guard.py
# — это константа тестов, не runtime-чтение.
```

## Что уже работает без правок

1. **Mock standalone** — `tools/mock_minimax_server.py` поднимает
   `127.0.0.1:18080`, принимает любой Bearer и GroupId, отдаёт валидные
   ответы во всех 4 форматах (PCM/WAV/MP3/OGG) + SSE streaming +
   error injection. Подтверждено end-to-end через
   `MiniMaxTTSProvider(base_url="http://127.0.0.1:18083").synthesize(...)`.

2. **Программное переключение** — если в коде (тестах или экспериментальной
   ROS-ноде) **явно** создать `MiniMaxTTSProvider(base_url=...)`, всё
   работает: bench `tts_audio_bench/scripts/run_bench.py:238` именно так
   и делает:
   ```python
   MiniMaxTTSProvider(api_key="test-key", base_url=f"http://127.0.0.1:{port}")
   ```
   8/8 scenarios проходят, 40 unit tests, ffprobe валидирует WAV-дампы
   (см. `tts_audio_bench/artifacts/bench-summary.json`).

3. **Env-чтение для `MINIMAX_API_KEY` / `MINIMAX_GROUP_ID`** в провайдере
   уже есть — задача выполнена частично.

## Что осталось

Только две строки в `MiniMaxTTSProvider.__init__` и одна строка в
`tts_node.py` (см. блок кода в TL;DR). Это:

- **Не breaking change** — если env не выставлена, поведение прежнее.
- **Не требует миграции БД** — нет state'а.
- **Покрыто тестами** — нужно добавить один test
  `test_minimax_tts_provider.py::TestConstructor::test_base_url_from_env`
  рядом с уже существующим `test_custom_base_url_strips_trailing_slash`.
- **Покрыто bench'ем** — `tts_audio_bench/scripts/run_bench.py` уже
  использует этот механизм программно; добавление env-чтения позволит
  запускать bench через `MINIMAX_BASE_URL=http://127.0.0.1:18080`
  вместо правки исходников.

## Что делать дальше

1. **Дождаться review этой задачи** — дашборд уже задал 6 вопросов по
   bench-harness (см. `t_5319ff7f` комментарии), mock-сервер является
   одной из опор bench'а.
2. **Создать отдельную задачу** для правки `MiniMaxTTSProvider.__init__` +
   `tts_node.py` + добавления теста. Назначить на backend (как тут) или
   на default (если нужно сначала обсудить архитектурно).
3. **После merge** обновить `tts_audio_bench/README.md` секцией "How to
   point bench at a remote mock" — сейчас bench hardcodes
   `http://127.0.0.1:{port}`, а после env-чтения можно будет
   `MINIMAX_BASE_URL=http://staging-mock:8080 python -m tts_audio_bench.scripts.run_bench`.

## Резюме

- ✅ Mock-сервер реализован и проверен end-to-end (4 сценария, все
  форматы, streaming, error injection).
- ✅ Документация (`tools/README_mock_minimax.md`) — все эндпоинты,
  заголовки, форматы ответов.
- ✅ Acceptance criterion "запускается одной командой" выполнен
  (`python3 tools/mock_minimax_server.py`).
- ✅ Acceptance criterion "выдаёт предсказуемые чанки" выполнен
  (детерминированные sine-fixtures из `tts_audio_bench/fixtures/`).
- ❌ Acceptance criterion "переключение через env без правок прода" —
  НЕ выполнен. **Зафиксировано здесь как blocker.** Требуется 2 строки
  в `MiniMaxTTSProvider.__init__` + 1 строка в `tts_node.py`.