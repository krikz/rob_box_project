# ADR-0143 — MiniMax игнорирует `tool_choice="required"` и именованную форму — Bug B/C-ретраи остаются

**Дата:** 2026-09-28
**Статус:** Принято (отказ; пробник прогнан, raw-вывод приложен)
**Автор:** шиди (Claude Code) по заданию шисюна
**Issue:** [#3135](https://github.com/krikz/rob_box_project/issues/3135) (Ш3 из `docs/design/2026-09-28-music-dj-systemic-analysis.md`, PR #3130, §2 П4, §6)
**Связанные:** #1881 (общий бюджет ретраев), `MusicGuard` (user=3, DJ=2), issue #2967 (`force_tool_choice="required"` на первом запросе DJ-auto-перехода — уже в коде, см. риск ниже), `03.2-CONTEXT.md` на ветке `feature/phase-3.2-music-testing` (для MiMo в 06.2026 поддерживался только `auto`)

## 1. Вопрос

`agent_core.py` лечит «LLM обязана была вызвать тул, но не вызвала» ретраями с промптом `[CRITICAL]`. Вокруг этого — общий бюджет ретраев (#1881), бюджеты `MusicGuard` (user=3, DJ=2), 10 флагов `*_retry_used`, `_retry_dispatched_in_turn`. Побочный эффект, пойманный на живом роботе 28.09: ретрай DJ-перехода вернул сам промпт как ответ (`spoken='[DJ_AUTO — ПЕРЕХОД #2] Ты Снупдог…'`).

Гипотеза: `tool_choice="required"` (или именованная форма `{"type":"function","function":{"name":...}}`) гарантирует вызов тула на уровне API и делает часть ретраев ненужной. Нужно было проверить это ЭМПИРИЧЕСКИ на реальном эндпоинте, а не по документации.

## 2. Метод

Провайдер и эндпоинт — те же, что использует `rob_box_voice` в проде (`docker/vision/config/voice_assistant/dialogue_node.yaml`, секция `minimax`):

- `base_url = https://api.minimax.io/v1`
- `model = MiniMax-M3`
- `thinking = {"type": "disabled"}` (совпадает с `DEFAULT_THINKING_POLICY` в `src/rob_box_llm/rob_box_llm/providers/minimax.py`, чтобы thinking не путал поведенческий тест — урок из уроков ветки phase-3.2, коммит `b76e30350`)

Пробник — `scripts/research/minimax_tool_choice_probe.py`. Прогнан НА РОБОТЕ (Vision Pi, 10.1.1.21) внутри контейнера `voice-assistant`, ключ `MINIMAX_API_KEY` читался из env контейнера, никогда не покидал контейнер и не печатался.

Промпт — «привет, как дела?» (безобидный, без принуждения модель отвечает текстом). Один тул — `get_time` (без аргументов). По 3 запроса на вариант `tool_choice`: `auto`, `required`, именованный `{"type":"function","function":{"name":"get_time"}}`. Смотрели: HTTP-статус, наличие `tool_calls` в ответе, `finish_reason`, ошибки API (HTTP-уровня и в теле `base_resp`).

## 3. Raw-результат (9 запросов, 28.09.2026, Vision Pi → voice-assistant → api.minimax.io)

```
docker exec voice-assistant python3 /tmp/minimax_tool_choice_probe.py
```

```json
{
  "base_url": "https://api.minimax.io/v1",
  "model": "MiniMax-M3",
  "prompt": "привет, как дела?",
  "requests_per_variant": 3,
  "variants": {
    "auto": {
      "tool_choice_sent": "auto",
      "runs": [
        {"http_status": 200, "elapsed_s": 2.063, "finish_reason": "stop", "has_tool_calls": false},
        {"http_status": 200, "elapsed_s": 1.438, "finish_reason": "stop", "has_tool_calls": false},
        {"http_status": 200, "elapsed_s": 1.182, "finish_reason": "stop", "has_tool_calls": false}
      ]
    },
    "required": {
      "tool_choice_sent": "required",
      "runs": [
        {"http_status": 200, "elapsed_s": 1.207, "finish_reason": "stop", "has_tool_calls": false},
        {"http_status": 200, "elapsed_s": 2.532, "finish_reason": "stop", "has_tool_calls": false},
        {"http_status": 200, "elapsed_s": 1.639, "finish_reason": "stop", "has_tool_calls": false}
      ]
    },
    "named_get_time": {
      "tool_choice_sent": {"type": "function", "function": {"name": "get_time"}},
      "runs": [
        {"http_status": 200, "elapsed_s": 1.55,  "finish_reason": "stop", "has_tool_calls": false},
        {"http_status": 200, "elapsed_s": 1.743, "finish_reason": "stop", "has_tool_calls": false},
        {"http_status": 200, "elapsed_s": 1.727, "finish_reason": "stop", "has_tool_calls": false}
      ]
    }
  }
}
```

(Полные `content_snippet` — реальный текстовый ответ модели на каждый запрос — приложены в issue #3135 и в PR; здесь опущены ради краткости, содержание не влияет на вердикт: во всех 9 запросах `has_tool_calls: false`, `finish_reason: "stop"`, никакой `base_resp`-ошибки и никакой HTTP-ошибки ни разу не было.)

## 4. Вердикт: **НЕТ**

MiniMax-M3 (через `https://api.minimax.io/v1`, OpenAI-совместимый Chat Completions) **не поддерживает принудительный `tool_choice`**:

- `tool_choice="required"` — 3/3 запроса вернули HTTP 200 без единого вызова тула, `finish_reason: "stop"`, чистый текст. Это ХУЖЕ, чем отказ с ошибкой: API молча принимает параметр и игнорирует его — тот же паттерн «тихой деградации», что и в проблеме, которую мы пытались закрыть (Bug B/C).
- Именованная форма `{"type":"function","function":{"name":"get_time"}}` — тот же результат: 3/3 без `tool_calls`.
- Ни разу не было HTTP-ошибки или `base_resp`-ошибки — то есть по коду ответа отличить «отклонил параметр» от «проигнорировал параметр» нельзя, отличать пришлось по факту отсутствия `tool_calls`.

Подтверждается независимо:
- Официальная документация MiniMax (`https://platform.minimax.io/docs/api-reference/text-openai-api`) не описывает `tool_choice` вообще — таблицы параметров с `required`/`any`/именованной формой нет; единственный пример в `tool_calling_guide.md` (MiniMax-AI/MiniMax-M2.5 на GitHub) использует `tool_choice="auto"`.
- Сторонний обзор (WebSearch, промптfoo/litellm-класс интеграций) прямо утверждает: «tool_choice supports auto and none» — то есть `required` и именованная форма не входят в заявленный набор.
- Совпадает с уроком для MiMo (`03.2-CONTEXT.md`, ветка `feature/phase-3.2-music-testing`, 06.2026): для соседнего OpenAI-совместимого провайдера тоже был задокументирован только `auto`.

## 5. Что НЕ меняется в этом PR

Ретраи Bug B/Bug C, бюджеты `MusicGuard` (user=3, DJ=2), общий бюджет `_retry_dispatched_in_turn` (#1881) и 10 флагов `*_retry_used` — **остаются без изменений**. Принудительный `tool_choice` не может их заменить, потому что провайдер его не выполняет.

### Риск: issue #2967 уже отправляет `tool_choice="required"` в проде

`src/rob_box_harness/rob_box_harness/core/agent_core.py:1019-1025` (`_run_with_tools`, `force_tool_choice="required" if is_dj_auto else None`) и `agent_core.py:1232-1247` (`_settings_with_forced_tool_choice`) уже шлют `tool_choice="required"` на первом запросе DJ-auto-перехода — это реализация #2967, принятая ДО этого пробника, в предположении, что MiniMax его уважает. Пробник показывает: этот параметр на проде сейчас **не делает ничего** — DJ-auto-переход полагается на Bug-B-ретрай точно так же, как если бы `force_tool_choice` не передавался вовсе. Это не регрессия (тихая деградация к прежнему поведению, не к худшему), но комментарий в коде («closes that gap at the source instead of relying entirely on the retry») сейчас **фактически неверен** — гарантии нет, полагаемся полностью на ретрай. Заведена отдельная карточка на правку комментария/докстрок в #2967-коде, чтобы не смешивать её с этим ADR (эта карточка не меняет код, только фиксирует факт).

Где именно это всплывёт в коде, если MiniMax когда-нибудь добавит поддержку `required` (проверять заново перед любым включением):
- `src/rob_box_harness/rob_box_harness/core/agent_core.py:1025` — единственная точка, где `force_tool_choice` формируется для реального вызова.
- `src/rob_box_harness/rob_box_harness/core/agent_core.py:1244-1247` (`_settings_with_forced_tool_choice`) — единственная точка, где `tool_choice` подмешивается в `LLMSettings`.
- `src/rob_box_llm/rob_box_llm/providers/deepseek.py:367-368` (`_OpenAICompatibleProvider._build_kwargs`) — единственная точка, где `s.tool_choice` реально уходит в `kwargs["tool_choice"]` на wire-формат; общая для `MiniMaxProvider` (наследует `_OpenAICompatibleProvider`, `src/rob_box_llm/rob_box_llm/providers/minimax.py:239`).

## 6. Альтернативы (отклонены/отложены)

- **Слепо доверять #2967 и убрать Bug B/C-ретраи для DJ-auto.** Отклонено: пробник показывает `tool_choice="required"` не работает — без ретраев DJ-auto-переход будет молча отвечать текстом без вызова `compose_music`, то есть регрессия к живому багу 28.09, который и завёл issue #3135.
- **Повторить пробник с другим промптом/большим числом попыток.** Отложено: 9/9 результат стабилен, `base_resp` ни разу не сигнализировал об ошибке параметра — ложноотрицательный результат (что MiniMax ВООБЩЕ не видел параметр) маловероятен, потому что запрос дошёл (HTTP 200, реальный текстовый ответ на промпт). Если появится сигнал, что MiniMax обновил API (release notes, новый номер модели), пробник переисполняется тем же скриптом — раскладка по прежнему валидна.
- **Написать собственный ретрай-цикл поверх `required`, ловя `has_tool_calls=False` как ошибку.** Не рассматривалось как замена Bug B/C — это ровно то же самое, что уже делают Bug B/C-ретраи, только с другим триггером; выигрыша в сложности нет.

## 7. Последствия

- Шаг Ш3 из `docs/design/2026-09-28-music-dj-systemic-analysis.md` закрыт результатом «нет» — карту работ по музыке/DJ нужно скорректировать: замена ретраев принудительным `tool_choice` НЕ входит в план, пока MiniMax не объявит поддержку.
- Комментарий/докстроки issue #2967 в `agent_core.py` вводят в заблуждение (обещают гарантию, которой нет) — отдельная лёгкая карточка на правку текста (не поведения).
- Если робот сменит провайдера (например на модель с реальной поддержкой `tool_choice`), пробник (`scripts/research/minimax_tool_choice_probe.py`) переиспользуется без изменений — достаточно поменять `BASE_URL`/`MODEL`.
