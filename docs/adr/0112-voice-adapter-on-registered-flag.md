# ADR-0112: `VoiceEncounterAdapter.on_registered` — закрыть расхождение `mcp_server` ↔ адаптер по `event="registered"`

| Поле | Значение |
|---|---|
| Статус | **Proposed** (решение к ревью товарища Шифу; код менять ещё рано — issue #2650, выкатка после ревью) |
| Дата | 2026-09-16 |
| Автор | architect (Hermes Agent), kanban `t_233109eb` |
| Тип | architecture decision — снять расхождение между реализацией и документацией |
| Родители | ADR-0096 (EncounterSeam, **Proposed**), ADR-0105 (EncounterSeam in-process first, **Proposed**), ADR-0013 (incremental delivery), ADR-0018 (честный FAIL) |
| Связанные | issue #2650 (компонентное ревью `src/rob_box_mcp_tools` за 2026-09-15), issue #2442 (EncounterSeam — следующие шаги), issue #1770 (первая реплика должна найти профиль сразу после регистрации), `src/rob_box_mcp_tools/.../mcp_server.py:533-599`, `src/rob_box_harness/.../encounter/voice_adapter.py:64-95`, `src/rob_box_harness/.../encounter/base.py:146` |

> **TL;DR.** После миграции `mcp_server` + `tools/memory` на `EncounterSeam` (issue #2442, коммит `8c869e1e`) обработчик `MCPServer._on_speaker_result` и голосовой адаптер `VoiceEncounterAdapter.on_speaker_result` **расходятся в обработке `event="registered"`**: `mcp_server` кормит шов (`seam.observe(...)`) и обновляет кэш, `VoiceEncounterAdapter` — no-op. Это сознательная развилка, оставленная в коде (`mcp_server.py:546-552` ссылается на issue #1770 и говорит «сохраняем 1:1»), но она **не закреплена в адаптере**, и любая правка парсинга payload в адаптере молча не доедет до `mcp_server`. Фикс — флаг `on_registered: Literal["observe", "noop"]` на конструкторе `VoiceEncounterAdapter`, дефолт `"noop"` (поведение `dialogue_node` сегодня), `mcp_server` создаёт адаптер с `on_registered="observe"`. Парсинг payload уезжает в `acquaintance_from_speaker_result` (один путь для обоих потребителей), `mcp_server` перестаёт парсить JSON своими руками.

---

## 1. Контекст

### 1.1 Что есть сегодня

Один и тот же топик `/voice/speaker/result` (JSON от `speaker_id_node`) сегодня обрабатывают три независимых обработчика, каждый из которых парсит JSON своими руками:

| Потребитель | Файл | Семантика `event="registered"` | Парсинг |
|---|---|---|---|
| `MCPServer._on_speaker_result` | `src/rob_box_mcp_tools/.../mcp_server.py:533-599` | **observe** — кормит шов, обновляет `current_speaker_id` (issue #1770) | `json.loads` + ручные `data.get("is_known")`/`speaker_id`/`name`/`confidence` (`:564-586`) |
| `dialogue_node._on_speaker_result` | `src/rob_box_voice/.../dialogue_node.py:1998-2018` | **noop** — лог-инфо, ранний `return` (issue #1077) | `json.loads` + `data.get("event")` |
| `VoiceEncounterAdapter.on_speaker_result` | `src/rob_box_harness/.../encounter/voice_adapter.py:73-95` | **noop** — возврат `seam.current()` без изменений (issue #2442, не используется сегодня никем, кроме тестов) | `acquaintance_from_speaker_result(payload)` — единственный, кто дёргает `seam.observe` |

Никто из потребителей не использует адаптер. Сам адаптер введён в PR #2593 (issue #2442) и подключён только в тестах; в `MCPServer` и `dialogue_node` остаются «старые» синхронные обработчики.

### 1.2 Что не так

Две проблемы.

**Расхождение между кодом и документацией.** Докстринг `VoiceEncounterAdapter.on_speaker_result` (`:81-84`):

> Registration-ack (`{"event": "registered", ...}`) — не сигнал присутствия (то же самое исключение уже делают оба текущих потребителя); возвращает текущую Встречу без изменений.

«Оба текущих потребителя» = `mcp_server` + `dialogue_node`. Утверждение **не соответствует коду** `mcp_server._on_speaker_result:574-576` (этот потребитель **наблюдает** `registered`). То есть документация адаптера неверна — младший воркер, который будет читать код в 2027 году, не сможет доверять этому абзацу.

**Расхождение между местом парсинга и местом действия.** `mcp_server` не зовёт `acquaintance_from_speaker_result` и парсит JSON своими руками. Это значит:

- Любая правка формата payload в адаптере (`_clean_str`, обработка `null`/`undefined`, добавление поля `epithet`, см. `voice_adapter.py:28-35`) **молча не доедет** до `mcp_server` и до `tools/memory._current_encounter_speaker_id`.
- Любая правка `seam.observe(...)` (новый аргумент, изменение `now=`, новая ветка логики для face-канала) **молча не доедет** до `mcp_server`.
- Конкретный класс регрессии при добавлении face-канала (следующий шаг issue #2442 / #2531): face-адаптер будет подписан на `/vision/hailo/events` через ту же `seam.observe`, а `mcp_server` продолжит ходить в обход — итоговый `Encounter.current()` в `mcp_server` будет «голос-only» даже когда человек стоит перед камерой.

### 1.3 Чего не делать

- **Не делать `event="registered"` observe в `VoiceEncounterAdapter` по умолчанию.** У `dialogue_node` (`:2006-2016`) есть отдельная логика для `registered`: лог-инфо с упоминанием `name` и id, ранний выход **до** взятия `_speaker_lock` и записи в `_current_speaker`. Если бы адаптер дефолтно `observe`'ил — `dialogue_node` начал бы считать registration-ack обычным присутствием и обновлять `_current_speaker` (это поведение сегодня отсутствует). Это смена поведения, которая ломает инвариант «registered — не сигнал присутствия» (ADR-0096 §2.3).
- **Не переносить парсинг JSON в `mcp_server`** — это и есть текущее состояние, и оно проблема.
- **Не вводить второй адаптер** (`VoiceEncounterAdapterObserve`, `VoiceEncounterAdapterNoop`) — два класса на одну семантическую развилку хуже, чем флаг с двумя значениями (`Literal`).
- **Не менять семантику issue #1770** (первая реплика после `registered` обязана попасть в профиль) — это документированный контракт, под него заточен e2e-сценарий issue #1770.

---

## 2. Решение

### 2.1 Контракт адаптера

`src/rob_box_harness/rob_box_harness/encounter/voice_adapter.py`:

```python
from typing import Literal

class VoiceEncounterAdapter:
    """Голосовой адаптер шва «Встреча» (issue #2442).

    :param seam: :class:`EncounterSeam`, в который кормится сигнал.
    :param on_registered: семантика для ``{"event": "registered", ...}``:

      * ``"observe"`` (по умолчанию для потребителей, которые рассчитывают,
        что следующая реплика сразу попадёт в профиль — issue #1770) —
        адаптер вызывает ``seam.observe(VOICE, Acquaintance, 0.0)``.
        Требует ``speaker_id``; если его нет — no-op (см. ADR-0096 §2.3:
        «профиль под per-session tag создавать нельзя», issue #2440).
      * ``"noop"`` (по умолчанию для потребителей, которые трактуют
        registration-ack как служебный лог — ``dialogue_node``, issue
        #1077) — адаптер возвращает ``seam.current(now=now)`` без записи.
    """

    def __init__(
        self,
        seam: EncounterSeam,
        *,
        on_registered: Literal["observe", "noop"] = "noop",
    ) -> None:
        self._seam = seam
        self._on_registered = on_registered

    async def on_speaker_result(
        self,
        payload: Mapping[str, Any],
        *,
        now: Optional[float] = None,
    ) -> Optional[Encounter]:
        if payload.get("event") == "registered":
            if self._on_registered == "observe":
                speaker_id = _clean_str(payload.get("speaker_id"))
                if speaker_id is None:
                    return self._seam.current(now=now)
                who = Acquaintance(
                    id=speaker_id,
                    name=_clean_str(payload.get("name")),
                )
                return await self._seam.observe(
                    EncounterChannel.VOICE, who, 0.0, now=now
                )
            return self._seam.current(now=now)

        # ... rest без изменений
        who = acquaintance_from_speaker_result(payload)
        confidence = who.confidence if who is not None and who.confidence is not None else 0.0
        return await self._seam.observe(
            EncounterChannel.VOICE, who, confidence, now=now
        )
```

`acquaintance_from_speaker_result` уже не использует `is_known=True` фильтр для `speaker_id` (он есть внутри), и парсинг JSON уезжает из `mcp_server`. Утилита `_clean_str` уже определена в этом же модуле (`:31-35`) — используем её.

### 2.2 Подключение `MCPServer`

`src/rob_box_mcp_tools/.../mcp_server.py:389` (где `self._encounter_seam` заводится):

```python
self._encounter_seam = EncounterSeam(MemoryIdentitySeam(InMemoryStore()))
self._voice_adapter = VoiceEncounterAdapter(
    self._encounter_seam, on_registered="observe"
)
```

`MCPServer._on_speaker_result` (`:533-599`) становится тонкой обёрткой:

```python
def _on_speaker_result(self, msg: "String") -> None:
    try:
        data = json.loads(msg.data or "{}")
    except (TypeError, ValueError):
        return
    if not isinstance(data, dict):
        return
    asyncio.run(self._voice_adapter.on_speaker_result(data))
```

Логика «transition only once» и красивые логи про «[issue 1770] current_speaker_id: ∅ → uuid...» остаются на стороне `mcp_server` (это presentation, а не семантика Encounter). Тесты в `test_mcp_server_speaker_result.py` остаются валидными на уровне observable-контракта (`_current_encounter_speaker_id()` до/после).

### 2.3 Почему `Literal`, а не `bool`

`on_registered: Literal["observe", "noop"]` (а не `on_registered: bool = False`):

- `Literal` документирует два допустимых режима в IDE/типе — у `bool` без констант имена состояний приходится гуглить по коду.
- Будущие режимы (например, `"warn_only"`) — добавление нового литерала не ломает существующих вызовов, а добавление нового значения `bool` по соседству с двумя другими значениями того же `bool` — ломает (потеря семантики).
- Цена: одна строка импорта `from typing import Literal`. Уже есть в `mcp_server.py` (`Optional`, `Mapping` — тот же модуль).

### 2.4 Контракт тестов

Существующие тесты остаются валидны:

- `test_encounter.py::TestVoiceEncounterAdapter` — дефолт `on_registered="noop"` сохраняет сегодняшнее поведение адаптера (`test_registered_event_is_not_a_presence_signal`, `:195-202`). Добавляем второй кейс: `VoiceEncounterAdapter(seam, on_registered="observe")` для `registered` с `speaker_id` записывает Встречу с этим `who`.
- `test_mcp_server_speaker_result.py::TestOnSpeakerResult` — `test_registered_event_updates_cache` (`:302-312`) остаётся зелёным, потому что `mcp_server` создаёт адаптер с `on_registered="observe"`. Поведение `_current_encounter_speaker_id()` после `registered`-сообщения — `uuid-new`, как сегодня.
- `test_voice_memory_adapter.py` — не зависит от `VoiceEncounterAdapter`, остаётся без изменений.

Новые тесты (1 шт): `test_voice_adapter_with_observe_flag.py` — модуль-тест на `VoiceEncounterAdapter(seam, on_registered="observe")`:

1. `{"event": "registered", "speaker_id": "u1", "name": "А"}` → `seam.current().who.id == "u1"`, `channels == {VOICE}`, `confidence == 0.0`.
2. `{"event": "registered", "speaker_id": ""}` (пустая строка) → no-op (не пишем «призрака»), `seam.current()` возвращает прежнее значение.
3. `{"event": "registered"}` без `speaker_id` → no-op.
4. `{"event": "registered", "speaker_id": "u1", "name": "<script>"}` (`_clean_str` через `_JUNK_NAMES`) → `name` отбрасывается, `id` сохраняется.

Существующий тест `test_registered_event_is_not_a_presence_signal` остаётся без изменений и продолжает покрывать дефолт `"noop"`.

---

## 3. Trade-off

| Аспект | A: `Literal["observe","noop"]` (предложен) | B: Всегда observe в адаптере | C: Два адаптера (`Observe`/`Noop`) | D: Ничего не делать |
|---|---|---|---|---|
| Кол-во мест парсинга JSON | **1** (адаптер) | 1 (адаптер) | 1 (адаптер) | 2 (адаптер + `mcp_server`) |
| Семантика `dialogue_node` | сохраняется (дефолт `"noop"`) | **ломается** — `dialogue_node` начнёт писать в `_current_speaker` на `registered`, чего сегодня не делает | сохраняется | сохраняется (расхождение) |
| Семантика `mcp_server` | сохраняется (`on_registered="observe"`) | сохраняется (была бы починена заодно) | сохраняется | сохраняется (расхождение) |
| Регрессия при добавлении face-канала | **нет** — адаптер один | нет | **возможна** — два класса надо синхронизировать | **да** — расхождение растёт |
| Размер diff | ~30 строк (адаптер + `mcp_server` + 1 тест) | ~10 строк | ~80 строк (два класса) | 0 |
| Стоимость входа | прочитать `Literal` и 2 строки в `__init__` | 0 | два класса, выбор правильного | 0 |
| Поведение issue #1770 | сохраняется | сохраняется | сохраняется | сохраняется |

**Победитель — A.** B ломает `dialogue_node` (issue #1077 явно документирует registration-ack как «не обновлять `_current_speaker`»). C — overengineering для одной развилки. D — накапливает технический долг (issue #2650, severity MEDIUM).

---

## 4. План выкатки

1. **Воркер backend.** Коммит в ветке воркера (этот ADR — обоснование, не код):
   - `voice_adapter.py`: добавить `on_registered`, использовать `_clean_str` в `observe`-ветке.
   - `mcp_server.py`: `__init__` — `self._voice_adapter = VoiceEncounterAdapter(self._encounter_seam, on_registered="observe")`. `_on_speaker_result` — `asyncio.run(self._voice_adapter.on_speaker_result(data))` (с сохранением `data` как `dict` через `json.loads`).
   - Тесты: дополнить `test_encounter.py` двумя кейсами (observe/no-op для `registered` без `speaker_id`), `test_mcp_server_speaker_result.py` остаётся as-is.
2. **CI.** Зелёный прогон (Unit Tests ROS2 + Python). Следить за тем, что `test_voice_memory_adapter` не зависит от адаптера — должен остаться зелёным.
3. **PR в `develop`.** Reviewer — pr-reviewer по обычному процессу.
4. **Следующая карточка (Phase 2 face-канал)** получает зафиксированный `VoiceEncounterAdapter` как единственную точку входа — добавление face делается через новый `FaceEncounterAdapter(seam)`, без правки `mcp_server`.

---

## 5. Что не внутри

- **Перенос `dialogue_node._on_speaker_result` на адаптер.** Это отдельная карточка; здесь мы только фиксируем, что дефолт `"noop"` сохраняет сегодняшнее поведение `dialogue_node`. Когда дойдёт очередь — `dialogue_node` создаст `VoiceEncounterAdapter(seam, on_registered="noop")` и уберёт свой обработчик.
- **Подключение face-канала (`/vision/hailo/events` → `FaceEncounterAdapter`).** Отдельная карточка по issue #2531 / #2583.
- **Распределённая схема EncounterSeam** (отдельная `encounter_node`, топик `/encounter/state`, список Встреч) — ADR-0096 §2.5, отложена ADR-0105 до появления лица.
- **Изменение `speaker_id_node` (формат payload, `epithet`-фильтр).** Отдельный трек.