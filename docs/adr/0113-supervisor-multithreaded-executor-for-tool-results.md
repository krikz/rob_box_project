# ADR-0075: avatar_supervisor использует MultiThreadedExecutor — фикс голода /mcp/result callback'а

| Поле | Значение |
|---|---|
| Статус | Accepted |
| Дата | 2026-09-08 |
| Issue | [#2131](https://github.com/krikz/rob_box_project/issues/2131) (operator-agent P0: «ТАРС не может выполнить ни один инструмент — однопоточный spin голодает on_result, таймаут 10 с») |
| Контекст | `LLMToolCallAdapter` уже использует `ReentrantCallbackGroup` для `/mcp/result` (`llm_adapter.py:60`), но callback диспетчеризуется только если executor — `MultiThreadedExecutor`. `avatar_supervisor` (`supervisor_node.py:2971`) крутится на `rclpy.spin(node)` → `SingleThreadedExecutor` по умолчанию. `dialogue_node` тем же адаптером пользуется успешно (`dialogue_node.py:6216` — `MultiThreadedExecutor`). |
| Связанные | ADR-0051 §2.7 (avatar_supervisor = ТАРС), ADR-0066 §6.7 (suspension личности через `/dialogue/control`), [issue #1988](https://github.com/krikz/rob_box_project/issues/1988) (operator-agent 04a) |

## TL;DR

`avatar_supervisor` переводится с `rclpy.spin(node)` на `rclpy.executors.MultiThreadedExecutor()` (как `dialogue_node`). Это позволяет `ReentrantCallbackGroup` для `/mcp/result` действительно диспетчеризовать callback `on_result()` в фоновом потоке, пока основной поток блокирован в `result_event.wait(timeout)` (`llm_adapter.py:226`). Под `SingleThreadedExecutor` callback физически не мог быть доставлен — отсюда стабильный «таймаут 10 с, ответ приходит через 1.3–1.7 с после».

## Корневая причина

`LLMToolCallAdapter.execute_tool_call_sync()` (`src/rob_box_mcp_tools/rob_box_mcp_tools/llm_adapter.py:189–250`):

1. Публикует запрос в `/mcp/execute`.
2. Регистрирует `result_event[request_id]` ПЕРЕД публикацией (`llm_adapter.py:208–209`) — защита от race.
3. **Блокирует поток** через `result_event.wait(timeout=timeout)` (`llm_adapter.py:226`).
4. Если executor — однопоточный, callback `on_result()` (subscriber на `/mcp/result`, тот же ROS-узел) физически не может выполниться, пока основной поток ждёт. Это deadlock «поток ждёт event, который мог бы выставиться только в callback'е того же потока».
5. Истекает таймаут 5 с (default `self.timeout` в `llm_adapter.py:95`) → `{"success": False, "error": "Timeout ожидания результата инструмента"}` (`llm_adapter.py:233`).
6. После отпускания `wait()` executor всё-таки крутит pending callback — поэтому в логах `on_result` приходит через 1.3–1.7 с ПОСЛЕ таймаута (см. таблицу в issue #2131).

`ReentrantCallbackGroup` в `llm_adapter.py:60` правильный по замыслу, но не работает без многопоточного executor'а — `ReentrantCallbackGroup` лишь разрешает одному и тому же callback'у вызываться параллельно самому себе и другим callback'ам из той же группы; **доставка callback'а всё равно зависит от executor'а**.

## Решение

`supervisor_node.py:2965–2977` (`main`):

```python
def main(args: Optional[list] = None) -> None:
    """Console-script entry point: ``ros2 run rob_box_supervisor supervisor_node``."""
    if not rclpy.ok():
        rclpy.init(args=args)
    node = AvatarSupervisor()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

Идентичный паттерн уже применён в `dialogue_node.py:6213–6226` (рабочий образец).

## Почему не альтернативы

| Альтернатива | Почему отклонена |
|---|---|
| А. Поднять `default_timeout` в `executors/ros_mcp.py:48` (с 10 до 30 с) | Не решает проблему — callback всё равно голодает, удлиняем только страдание. Прямо зафиксировано в issue #2131 как out-of-scope. |
| Б. Перейти на асинхронный путь (`AsyncToolExecutor` в адаптере уже есть, `llm_adapter.py:98`) | Более глубокий рефактор — нужно менять контракт `execute_tool_call_sync` (его вызывает `ROSMCPToolProvider`, а тот — `AgentCore`). Это отдельная большая карточка (issue #2131 «out of scope» явно отделяет). |
| В. Крутить `rclpy.spin_once()` внутри `wait()` (busy-poll) | Хрупкий хак, требует тайм-аутов polling и добавляет boilerplate. MultiThreadedExecutor — каноничный ROS-2 паттерн и уже применён в `dialogue_node`. |
| Г. Заменить `ReentrantCallbackGroup` на `MutuallyExclusiveCallbackGroup` | Ничего не меняет: под однопоточным executor'ом callback не доставляется в любой группе, пока основной поток заблокирован. |
| Д. `MultiThreadedExecutor` только для supervisor'а, остальные callback'и — default | То же самое, что выбранное решение (default callback group = `MutuallyExclusiveCallbackGroup`, его потоки под MTE не «распараллеливаются» сверх callback'ов той же группы). Гарантия сериализации default-callback'ов сохраняется. |

## Инварианты (что НЕ должно сломаться)

1. **Callback'и default-группы остаются сериализованными** — `MultiThreadedExecutor` сериализует callback'и одной `MutuallyExclusiveCallbackGroup` так же, как single-threaded executor. Параллельность возможна только между callback'ами из разных `ReentrantCallbackGroup`-ов. В supervisor только `LLMToolCallAdapter` использует `ReentrantCallbackGroup` (`llm_adapter.py:60`), и она изолирована от остальных callback'ов ноды.
2. **State-машины супервизора не используют `threading.Lock` / `threading.Event` напрямую** — проверено grep'ом (`supervisor_node.py` — 0 упоминаний `threading.`).
3. **Лениво создаваемый `LLMToolCallAdapter`** (`_ensure_agent_core`, `supervisor_node.py:1863`) — ничего не меняет: `MultiThreadedExecutor` уже работает к моменту первой инициализации адаптера (а значит `ReentrantCallbackGroup` начнёт диспетчеризовать callback'и сразу).
4. **`destroy_node()`** — после `executor.shutdown()` (как в `dialogue_node.main`), чтобы executor освободил узел до `destroy_node`.
5. **Контракт `LLMToolCallAdapter`** — не меняется. Адаптер уже корректен; фикс только на стороне executor'а supervisor'а.

## Регрессионный тест

`src/rob_box_mcp_tools/test/test_llm_adapter.py` (новый файл) — **двухслойная** регрессия, потому что контракт «доставка callback под MTE vs STE» нельзя проверить на mock-rclpy (mock-rclpy не запускает ни `spin()`, ни диспетчеризацию):

### Слой 1 (без rclpy, ВСЕГДА работает — главный гейт)

- **`test_supervisor_main_uses_multithreaded_executor`** — статический AST-тест: парсит `supervisor_node.main`, проверяет, что тело использует `rclpy.executors.MultiThreadedExecutor` и НЕ содержит прямого `rclpy.spin(node)` в `main`. Блокирует регрессию к STE.
- **`test_llm_adapter_uses_reentrant_callback_group_for_mcp_result`** — парсит `LLMToolCallAdapter.__init__`, проверяет, что подписка на `/mcp/result` создаётся с `callback_group=<ReentrantCallbackGroup>`. Это контракт адаптера, нарушение которого (как и в случае supervisor'а) приводит к голоду.
- **`test_llm_adapter_execute_tool_call_sync_returns_on_event_set`** — семантический тест на mock-ноде: вызывает `execute_tool_call_sync(...)` и в фоновом потоке через `adapter.on_result(msg)` (имитация callback'а) доставляет результат. Проверяет, что `result_event.wait()` корректно просыпается и `execute_tool_call_sync` возвращает payload, а не таймаут. Это unit-тест логики `Event.set()/wait()`, не зависит от типа executor'а — но **именно его мы хотим видеть зелёным**, потому что если кто-то «починит» `execute_tool_call_sync` через `spin_once` внутри `wait()`, наш `on_result` уже не будет работать так же.

### Слой 2 (требует реальный `rclpy` — skip без него)

- **`test_execute_tool_call_sync_does_not_starve_on_multithreaded_executor`** — поднимает настоящий rclpy node + `MultiThreadedExecutor` в потоке, публикует результат на `/mcp/result` через `Timer`/`Publisher`, проверяет что `execute_tool_call_sync` возвращает результат за < 1 с. **Пропускается**, если `rclpy` недоступен (`pytest.importorskip("rclpy")`). Запускается в CI (там rclpy установлен в ROS-образе).

Тесты Слоя 1 — это **Definition of Done** issue #2131 пункт 4 («тест, падающий на текущем коде и зелёный после правки»): на старом коде `supervisor_node.main` (`rclpy.spin(node)`) AST-тест падает, после правки — зелёный.

## Definition of Done (issue #2131)

- [ ] `get_robot_status` через `/avatar/command` возвращает реальный статус, не «таймаут»; raw `/avatar/command_result` приложен — **на стороне e2e-process / merge-gate**
- [ ] `latency_ms` в ответе < 5000 (сейчас 16000–19000); raw приложен — **на стороне e2e-process / merge-gate**
- [ ] В `docker logs avatar-supervisor` нет строки `⏱️ Timeout ожидания результата` за прогон; raw приложен — **на стороне e2e-process / merge-gate**
- [ ] Тест, падающий на текущем коде и зелёный после правки — **этот PR** (AST-тест `test_supervisor_main_uses_multithreaded_executor`)

Пункты 1–3 — live-валидация e2e. Эта карточка фиксирует код + регресс-тест (пункт 4). Архитектурное решение — почему именно MTE, а не timeout / async path / busy-poll — зафиксировано в ADR (см. «Почему не альтернативы»).

## Затронутые файлы

- `src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py:2965–2977` — `main()` (фикс).
- `docs/adr/0072-supervisor-multithreaded-executor-for-tool-results.md` — этот документ.
- `src/rob_box_mcp_tools/test/test_llm_adapter.py` — новый файл (регресс-тест).

## Follow-up (отдельные карточки, не блокируют эту)

- ADR-0066 §6.7 — удаление `_voice_input_mode_before_swap` и т. п. (отдельная developer-карточка, не архитектор).
- Sender/срез (отдельная карточка, помечена в issue #2131).
- Полный переход на `AsyncToolExecutor` — отдельная карточка, если MultiThreadedExecutor окажется недостаточен (issue #2131 «out of scope»).
- Если на каком-то ещё node проявится тот же класс багов (`rclpy.spin(node)` + `ReentrantCallbackGroup` subscriber + блокирующий `Event.wait`) — добавить audit-тест, сканирующий `src/` на эту комбинацию. Не блокирует этот PR.
