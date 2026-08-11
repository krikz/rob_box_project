# `rob_box_voice.scheduler` — Phase 1 MVP TaskScheduler

Минимальный жизнеспособный планировщик (issue #968 §11.1) с тремя
FIFO-каналами: **voice / music / anim**.

## Зачем

Перед фазой 1 race `stop_music()` против TTS-чанка приводил к тому,
что музыка обрывалась на середине рэпа (регресс v36). MVP убирает
race, сериализуя канал **voice** так, что два `speak_text` (и
любой голосовой side-effect, включая `stop_music` после TTS) не
могут пересечься на железе, при этом `music` и `anim` каналы
работают параллельно и не блокируют voice.

## Что НЕ входит в MVP

| Фича | Где живёт | Фаза |
|------|-----------|------|
| AcceptanceGate / AWAITING_CONFIRMATION | `rob_box_harness.core.acceptance` | 1.5 ✅ уже сделано в этой ветке |
| SchedulerEventBus (battery / obstacle) | — | 2 |
| Двухуровневое quick-decide (LLM < 800мс) | — | 2 |
| Reflex-канал / «стой» / «направо» | — | 1.5+ |
| SegmentEstimator / ETA / speculative pre-gen | — | 3 |

Эти фичи **нарочно** не подключены к MVP — каждая будет опираться
на стабильный core (то, что описано ниже) и добавляться отдельным
PR. Acceptance-гейт из фазы 1.5 уже работает поверх gate-уровня;
планировщик — это слой ниже: каналы и порядок исполнения.

## Публичная поверхность

```python
from rob_box_voice.scheduler import (
    TaskScheduler,        # фасад
    SchedulerTask,        # то, что кладём в submit()
    TaskResult,           # что возвращает executor
    ChannelKind,          # VOICE / MUSIC / ANIM
    TaskStatus,           # QUEUED/SCHEDULED/RUNNING/COMPLETED/FAILED/CANCELLED
    ChannelStatus,        # снимок одного канала
    TaskSubmitError,      # неизвестный канал / shutdown / нет executor
    TaskNotFoundError,    # wait/cancel по несуществующему id
)
```

### Жизненный цикл

```python
import asyncio
from rob_box_voice.scheduler import (
    TaskScheduler, SchedulerTask, TaskResult, ChannelKind,
)

async def _speak(task: SchedulerTask) -> TaskResult:
    # publish SSML to /voice/dialogue/response ...
    return TaskResult(payload={"speech_id": "abc"})

async def main() -> None:
    sched = TaskScheduler()
    sched.start()                                  # idempotent

    task = sched.submit(SchedulerTask(
        task_id="t1",
        tool="speak_text",
        channel=ChannelKind.VOICE,
        executor=_speak,
        args={"text": "Привет!"},
    ))
    await sched.wait_all()                         # все каналы опустели
    assert task.status.value == "COMPLETED"

    sched.shutdown()
```

### Ключевые гарантии

- **FIFO на канал.** На одном канале задачи идут строго по очереди,
  `asyncio.Lock` не даёт двум executor'ам работать одновременно.
- **Параллельность между каналами.** `execute_music_code` не
  задерживает `speak_text` — два канала независимы.
- **FIFO для voice = фикс регресса v36.** `speak_text` → TTS-finished
  → `speak_text` → TTS-finished → `stop_music` строго в этом порядке;
  `stop_music` не может «обогнать» голос.
- **Ошибка executor'а не роняет pump.** Исключение становится
  `FAILED` + `task.error`, следующая задача канала исполняется.
- **Cancel для QUEUED.** Задача, которая ещё не попала в lock,
  снимается через `sched.cancel(task_id)`. RUNNING-задачу MVP
  не прерывает — это будет Phase 2 (`SchedulerEventBus`).
- **`[CHANNELS]` snapshot.** `sched.channel_status(kind)` /
  `sched.all_statuses()` возвращают `ChannelStatus` с полями
  `queue_depth`, `current_task_id`, `current_tool`, `eta_s`. Phase 3
  подключит ETA через `sched.set_eta_provider(callable)`.

### Что Phase 2/3 могут менять, не ломая MVP

| Расширение | Как добавить |
|-----------|--------------|
| ETA | `sched.set_eta_provider(lambda task: 1.5)` |
| Новый канал | `TaskScheduler(channels=(VOICE, MUSIC, ANIM, NAV))` + `ChannelKind.NAV` |
| Priority | Phase 2 — отдельный PR, поверх MVP API |
| Preemption | Phase 2 — через `SchedulerEventBus`, не через MVP cancel |

## Сценарии, которые MVP закрывает (acceptance §11.1)

| Критерий | Тест |
|----------|------|
| `stop_music()` не доходит до железа раньше конца TTS-чанка | `test_task_scheduler_integration.py::test_two_speak_text_then_stop_music_preserve_order` |
| FIFO внутри voice-канала | `test_task_scheduler.py::test_per_channel_serial_execution` |
| Music не блокирует voice | `test_task_scheduler.py::test_different_channels_do_not_block_each_other` |
| LLM видит блок `[CHANNELS]` | `test_task_scheduler.py::TestChannelStatus` |
| Ошибка executor'а не валит канал | `test_task_scheduler.py::test_failing_task_marks_failed_and_keeps_pump_alive` |
| Cancel до старта | `test_task_scheduler.py::test_cancel_queued_task` |
| Два speak_text подряд → каждый < 50мс (proxy) | `test_task_scheduler_integration.py::test_concurrent_music_does_not_delay_voice` |

## Тесты

```bash
# Только unit (быстро)
PYTHONPATH=src/rob_box_voice:src/rob_box_harness \
  python -m pytest src/rob_box_voice/test/test_task_scheduler.py -v \
    -o asyncio_mode=auto -o testpaths=src/rob_box_voice/test

# Unit + integration
PYTHONPATH=src/rob_box_voice:src/rob_box_harness \
  python -m pytest src/rob_box_voice/test/test_task_scheduler*.py -v \
    -o asyncio_mode=auto -o testpaths=src/rob_box_voice/test
```

Сейчас 25 unit + 4 integration = **29 passed**.

## Что делает LLM-интеграция (W7, отдельный PR)

Диалоговый shell (`dialogue_node.py`) подключается к планировщику
через тонкий адаптер: каждый `function_tool` (speak_text,
execute_music_code, stop_music, play_animation, …) оборачивается в
`SchedulerTask` и кладётся в нужный канал. Acceptance-гейт
(фаза 1.5) остаётся поверх — он решает, класть ли сегмент в
`AWAITING_CONFIRMATION` или сразу в `PENDING`. MVP-планировщик
принимает уже классифицированные сегменты.

Документ-источник: `docs/design/SCHEDULER_DESIGN.md` §10 / §11.1.