# ADR-0101 — «Повод» (Occasion): единый шов «можно ли заговорить»

| Поле | Значение |
|---|---|
| Статус | Proposed (issue #2536, kanban t_a62b9143) |
| Дата | 2026-09-15 |
| Автор | architect |
| Контекст | Серия #2531/«Взгляд», #2532/«Проекция восприятия», #2442/«Встреча», #2440/«Знакомый» даёт роботу данные о присутствии человека, но `dialogue_node` всё ещё не имеет шва «можно ли заговорить без вейк-слова» |
| Затрагивает | `src/rob_box_voice/rob_box_voice/dialogue_node.py` (швы `_on_stt`, `_dispatch_turn`, `_run_turn`, `_on_startup_greeting*`, `_on_inactivity_check`), `core/dj_mode.py`, `stt_node.py`, новое `src/rob_box_voice/rob_box_voice/core/occasion.py`, существующий (мёртвый) `src/rob_box_perception/rob_box_perception/core/event_detector.py` |
| Родители | ADR-0021 (decomposition discipline для `dialogue_node`), ADR-0065 (wake-words SSoT в коде), ADR-0029 (wake-gate cold-start — root cause VAD), ADR-0070 (systemic no_wake_word fail-streak) |
| Связанные | issue #2536 (этот), #2442 (Встреча — источник повода `meeting`), #2531 (Взгляд — реальный кадр, маркер для стаб-фильтра), #2532 (Проекция восприятия), #2398 (real inference), #992 (Bug A-F — породили `is_synthetic=True`), #1881 (`_synthetic_retries_left`), `scripts/lint/seam_baseline.json` (задокументированный разрыв `/voice/stt/request`), осиротевший `src/rob_box_voice/rob_box_voice/startup_greeting_node.py` |
| Closes (после реализации) | #2536 |

> **TL;DR.** Ввести модуль **Повод** (`core/occasion.py`) с интерфейсом
> `may_speak(Occasion) → Verdict` — единая точка решения «можно ли сейчас
> инициировать новый ход». Сейчас этим занимается **размазанная** логика:
> вейк-гейт в `_on_stt`, разовый флаг `_startup_greeting_fired`, 4
> независимых кулдауна (DJ-tick, unclear_cooldown_s, inactivity-таймер,
> разовый флаг), и `is_synthetic=True` в семи местах как ярлык «не
> новый ход», а не «без слова пользователя». Ни один из этих механизмов
> не знает о существовании других. Повод консолидирует: общий дебаунс
> между источниками + per-источник кулдаун + стаб-фильтр для зрения +
> явные источники повода (wake / dj_tick / startup / meeting). Вейк-слово
> остаётся первым источником повода, байт-в-байт эквивалентная по
> наблюдаемому поведению.

---

## 1. Контекст и бизнес-проблема

### 1.1. Сценарий-заказчик серии

Денис входит в мастерскую → робот замечает событие → поднимает биометрию →
понимает, кто это → знает, что давно не виделись → заговаривает первым:
«о, привет, Денчик, как дела» → и по контексту времени суток или прошлых
бесед задаёт вопрос.

Четыре открытых (один смержен) issue серии дают **данные** для сценария:

| # | Статус | Даёт |
|---|---|---|
| #2440 «Знакомый» | merged (`8cb84bc9`) | устойчивый id человека |
| #2531 «Взгляд» | open, priority:high | единый шов источника кадра |
| #2532 «Проекция восприятия» | open, priority:high | путь `PerceptionEvent` → Личность |
| #2442 «Встреча» | open | единое значение присутствия |

Но даже если все четыре шва завтра заработают идеально — робот
**физически не сможет заговорить первым**, потому что у `dialogue_node`
нет входа в ход, кроме вейк-слова.

### 1.2. Что сегодня служит «правом заговорить»

Проверено 2026-09-15 (raw, файл:строка):

1. **Единственный вход в ход — STT-текст после вейк-слова** (`dialogue_node.py:2108` → `:2189` → `:2217`). Гейт `if tg_chat_id is None and not has_wake_word(...)` — единственный, кто решает «запускать ли LLM-цикл от лица извне». При провале — либо накопление в `_speech_accumulator`, либо инкремент `_llm_skipped_counter["no_wake_word"]`, в обеих ветках `return`. `_run_turn` не вызывается.

2. **TG-маркер `[TG:<chat_id>]`** (`dialogue_node.py:2119-2129`) — единственный обход вейк-гейта. Это всё ещё человек, который написал первым.

3. **`is_synthetic=True` в семи местах `_dispatch_turn`** — ярлык «не считать новым запросом пользователя внутри уже открытого хода» (комментарий в `dialogue_node.py:3423-3431` прямо фиксирует это). Ни один из семи вызовов не заводит ход *извне* по инициативе устройства: babble/code/action-claim/regurgitate/turn-guards/music/tool-skip — всё это ретраи внутри уже идущего хода. Седьмой `_drain_pending_user_messages` (`dialogue_node.py:6241`, issue #968) — склейка накопленных фраз ОДНОГО уже открытого диалога.

4. **Единственная незапрошенная реплика** — `_startup_greeting_fired` (`dialogue_node.py:5842-5894`) + `_startup_greeting_text`/`_startup_greeting_sec` (default `12.0` и пустая строка; в проде `docker/vision/config/voice_assistant/dialogue_node.yaml:133-134` — `12.0` и фиксированный `"Я на связи, все системы в норме!"`). Без LLM, без восприятия, без адресата. **Ровно один раз за uptime.**

5. **Примитив кулдауна написан, покрыт тестами и мёртв**: `EventDetector.should_react_to_event` (`src/rob_box_perception/rob_box_perception/core/event_detector.py:144-173`) — единственный импортёр во всём репозитории — `src/rob_box_perception/test/unit/core/test_event_detector.py:6` (подтверждено `git grep` по `origin/develop`).

6. **Кулдауны размазаны** и не знают друг о друге:
   - DJ-тик: `dialogue_node.py:923` + `core/dj_mode.py:230-270` (`DJ_TICK_INTERVAL_S`).
   - Стартовая реплика: булев одноразовый флаг `_startup_greeting_fired` (не кулдаун).
   - `stt_node._maybe_speak_unclear` (`stt_node.py:876-888`): свой `unclear_cooldown_s`.
   - 5-секундный inactivity-таймер (`dialogue_node.py:922` + `:6395-6404`) — переводит DSM в IDLE, **никогда не говорит**.

7. **Осиротевший `startup_greeting_node.py`** в `rob_box_voice` — модуль есть, нигде не запускается (нет в `setup.py:51-66` console_scripts, оба launch-файла имеют комментарий «Отдельная startup_greeting_node убрана»). Единственный импортёр — собственный тест. Подтверждено `git grep`.

8. **`/voice/stt/request` без подписчика**: `rob_box_mcp_tools/.../tools/dialogue.py:706` (`ListenForResponseTool.__init__` создаёт паблишер), подписчика нет ни одного; задокументировано в `scripts/lint/seam_baseline.json:17` (`publishers_without_local_subscriber`). Следствие: даже когда робот скажет «привет, Денчик» первым, `listen_for_response` не откроет продолжение диалога без нового вейк-слова — сообщение об «активации микрофона» уходит в пустоту.

### 1.3. Что такое «Повод» (домен)

**Повод** — это «что даёт право заговорить первым». Это **последний
недостающий шов** сценария встречи: не источник данных, а решение
«начинать ли ход».

Сегодня в коде нет доменного термина «повод» — есть разбросанные
триггеры хода (wake, DJ-tick, startup). Зрительный повод ещё не
существует, потому что #2531/#2532/#2398 не закрыты; но решение «можно
ли использовать повод X, когда он появится» должно быть подготовлено
**заранее**, чтобы зрительный повод не застал нас с поличным.

### 1.4. Терминология

| Термин | Значение |
|---|---|
| **Повод (Occasion)** | Семантическое описание «что даёт право заговорить»: wake / dj_tick / startup / meeting / inactivity_acknowledgement / ... |
| **Verdict** | Решение `may_speak`: `allow` (можно, инициатор = повод) / `defer` (кулдаун) / `refuse` (стаб-фильтр / disabled) |
| **Источник повода (occasion source)** | Конкретная подписка ROS2, timer или callback, который **сообщает** о поводе (но не решает, говорить ли). `dialogue_node` подписан на источники поводов; **решение** принимает Повод. |
| **Ход (turn)** | Один проход LLM-цикла `_run_turn`. Не путать с **поводом**: повод может дать `defer` (не ход), `allow` (один ход), `refuse` (не ход и не попытка). |
| **Синтетический ход (`is_synthetic=True`)** | Ярлык для ретраев **внутри уже открытого хода** (babble, code, action-claim, …). Не «ход без слова пользователя», а «повторный внутренний вызов того же хода» (см. комментарий в `dialogue_node.py:3423-3431`). |

---

## 2. Инвариант (как должно быть)

```text
core/occasion.OccasionGate.may_speak(Occasion) -> Verdict
                                          ↓
                                  общий дебаунс
                                  per-source кулдаун
                                  стаб-фильтр (vision only)
                                          ↓
                                  allow / defer / refuse
                                          ↓
                     dialogue_node._dispatch_turn(is_synthetic=True)
                                          ↓
                                  _run_turn (без изменений)
```

1. **Вейк-слово остаётся первым источником повода** и наблюдаемое поведение `_on_stt` → `has_wake_word` → `_dispatch_turn` остаётся байт-в-байт эквивалентным (регрессия здесь недопустима; см. §6 Acceptance).

2. **Источники повода — регистрируются явно**: wake_word, dj_tick, startup, meeting, inactivity_acknowledgement. Каждый источник имеет per-source кулдаун и общий дебаунс между источниками.

3. **Стаб-фильтр обязателен для зрения**: повод `meeting` с `payload.event_type == 'person'` И `payload.source_camera in {'unknown', 'stub'}` (или иной надёжный маркер реального кадра — определяется в #2531) → `refuse`. Этот фильтр — **защита от детерминированного таймера `StubHEFLoader`** (каждые 2 секунды, `vision_hailo_loader.py:116`), который иначе генерировал бы непрерывный повод.

4. **Повод консолидирует существующие разрозненные кулдауны**:
   - DJ-tick (`DJ_TICK_INTERVAL_S`) → per-source кулдаун повода `dj_tick`.
   - `_startup_greeting_fired` → одноразовая отметка повода `startup`.
   - `stt_node._maybe_speak_unclear` (`unclear_cooldown_s`) → per-source кулдаун повода `unclear_acknowledgement`.
   - 5-секундный inactivity-таймер (`_on_inactivity_check`) → потенциальный источник повода `inactivity_acknowledgement` (если решено включить).

5. **Повод сам по себе не публикует реплику**. Он выдаёт `Verdict`; решение «что сказать» — отдельный шов (`_run_turn` с пометкой инициатора, либо специализированный callback для `startup` / `meeting`).

6. **Шов «решение» отделён от шва «данные»**: `dialogue_node` подписан на топики-источники данных (например `/perception/context_update`, `/vision/hailo/events`) **только** для целей **данных** (профиль, имя, время суток, контекст встречи); решение «заговорить» остаётся за `OccasionGate`. Это устраняет риск «размазывания» решения по callback-ам подписки.

---

## 3. Решение

### 3.1. Новый модуль `core/occasion.py`

Расположение: `src/rob_box_voice/rob_box_voice/core/occasion.py` (рядом с
`core/dialogue_text.py`, `core/dj_mode.py`, `core/dialogue_guards.py` —
архитектурно на своём месте).

```python
# core/occasion.py — sketch, не финальная реализация

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional
import time


class VerdictKind(str, Enum):
    ALLOW = "allow"
    DEFER = "defer"
    REFUSE = "refuse"


@dataclass(frozen=True)
class Occasion:
    """Семантическое описание «что даёт право заговорить»."""
    kind: str                # 'wake_word' | 'dj_tick' | 'startup' |
                             # 'meeting' | 'unclear_acknowledgement' |
                             # 'inactivity_acknowledgement'
    payload: dict = field(default_factory=dict)  # произвольный контекст
    is_user_initiated: bool = False  # True для wake_word / TG-маркера,
                                     # False для автономных поводов


@dataclass
class Verdict:
    kind: VerdictKind
    reason: str              # человекочитаемое объяснение
    retry_after_s: Optional[float] = None  # для DEFER — когда повторить


class OccasionGate:
    """Единая точка решения «можно ли заговорить без слова пользователя».

    Консолидирует:
      - общий дебаунс между источниками (любые два повода ближе N секунд → DEFER)
      - per-source кулдаун (каждый источник — своя «не чаще чем» норма)
      - стаб-фильтр для vision (event_type='person' + source_camera='unknown' → REFUSE)
      - одноразовые маркеры (startup — только один раз за uptime)
    """

    def __init__(self, *, global_debounce_s: float = 2.0,
                 source_cooldowns: dict[str, float] | None = None,
                 stub_event_type: str = "person",
                 stub_source_cameras: frozenset[str] = frozenset({"unknown", "stub"}),
                 one_shot_kinds: frozenset[str] = frozenset({"startup"})):
        self._global_debounce_s = global_debounce_s
        self._source_cooldowns = source_cooldowns or {}
        self._stub_event_type = stub_event_type
        self._stub_source_cameras = stub_source_cameras
        self._one_shot_kinds = one_shot_kinds
        self._last_fire_at: dict[str, float] = {}
        self._last_any_at: float = 0.0
        self._consumed_one_shot: set[str] = set()
        # Эту строку оставляем в виде «комментария-привязки» для воркера,
        # чтобы использовал существующий EventDetector вместо своего dict:
        # from rob_box_perception.core.event_detector import EventDetector
        # self._detector = EventDetector(cooldown_interval=global_debounce_s)

    def may_speak(self, occasion: Occasion, now: float | None = None) -> Verdict:
        """Решить, можно ли инициировать ход для данного повода.

        Чистая функция от (state, occasion) — тестируется без ROS2.
        """
        # 1. Стаб-фильтр (vision only)
        if occasion.kind == "meeting":
            payload = occasion.payload or {}
            event_type = payload.get("event_type")
            source_camera = payload.get("source_camera")
            if (event_type == self._stub_event_type
                    and source_camera in self._stub_source_cameras):
                return Verdict(VerdictKind.REFUSE,
                               f"stub event: event_type={event_type!r} "
                               f"source_camera={source_camera!r}")

        # 2. Одноразовые маркеры (startup)
        if occasion.kind in self._one_shot_kinds:
            if occasion.kind in self._consumed_one_shot:
                return Verdict(VerdictKind.DEFER, "one-shot already consumed")
            # allow ниже — отметим после успешного allow

        # 3. User-initiated поводы (wake_word) — без кулдауна и без
        #    стаб-фильтра. Это сохраняет байт-в-байт поведение _on_stt.
        if occasion.is_user_initiated:
            return Verdict(VerdictKind.ALLOW, "user-initiated")

        # 4. Per-source кулдаун
        now = now if now is not None else time.monotonic()
        cooldown = self._source_cooldowns.get(occasion.kind, 0.0)
        last = self._last_fire_at.get(occasion.kind)
        if last is not None and (now - last) < cooldown:
            return Verdict(VerdictKind.DEFER,
                           f"source cooldown ({cooldown}s)",
                           retry_after_s=cooldown - (now - last))

        # 5. Глобальный дебаунс (любой-два повода ближе N)
        if (now - self._last_any_at) < self._global_debounce_s:
            return Verdict(VerdictKind.DEFER,
                           f"global debounce ({self._global_debounce_s}s)",
                           retry_after_s=self._global_debounce_s - (now - self._last_any_at))

        # 6. allow + bookkeeping
        return Verdict(VerdictKind.ALLOW, "ok")

    def mark_consumed(self, occasion: Occasion, now: float | None = None) -> None:
        """Вызывается после успешной отправки хода — обновляет cooldowns."""
        now = now if now is not None else time.monotonic()
        self._last_fire_at[occasion.kind] = now
        self._last_any_at = now
        if occasion.kind in self._one_shot_kinds:
            self._consumed_one_shot.add(occasion.kind)

    # ---- диагностика ----
    def stats(self) -> dict:
        return {
            "last_fire_at": dict(self._last_fire_at),
            "last_any_at": self._last_any_at,
            "consumed_one_shot": sorted(self._consumed_one_shot),
        }
```

**Почему так, а не иначе** (развилка решений):

| Альтернатива | Почему отвергнута |
|---|---|
| Хранить cooldowns внутри `dialogue_node` (как сейчас, разрозненно) | Не решает задачу: даже новый источник (встреча) не имеет куда «приткнуться», кроме как в ещё одну переменную в `__init__`. |
| Сделать `OccasionGate` подписчиком ROS2-топиков | Слишком много ответственности. Повод должен быть **решением**, а не «ещё одной нодой с подписками». Подписки остаются в `dialogue_node`; callback публикует повод через `gate.may_speak(Occasion(...))`. |
| Использовать существующий `EventDetector.should_react_to_event` как **внутренний** механизм | Принято: §3.2. `EventDetector` уже покрыт юнит-тестами и имеет нужную семантику кулдауна; повод **оборачивает** его, добавляя стаб-фильтр, общий дебаунс и одноразовые маркеры. |
| Превратить `EventDetector` в самостоятельный «publish»-сервис (через DI) | Избыточно для текущей задачи. Достаточно импортировать `EventDetector` в `OccasionGate` как per-source кулдаун-механизм; EventDetector не публикует ничего сам. |
| Делать повод асинхронным (queue + worker) | Для текущего масштаба (RPS ≈ 0.1, не десятки) — лишний слой. Синхронный `may_speak` в callback-хендлере достаточно. |

### 3.2. Использовать `EventDetector` как per-source кулдаун

`src/rob_box_perception/rob_box_perception/core/event_detector.py:144-173`
уже имеет нужную семантику (`should_react_to_event`, `mark_event_reacted`,
`cooldown_interval`). Сейчас **единственный импортёр** — собственный
юнит-тест; это **мёртвый код**.

**После введения Повода**:
- `OccasionGate` владеет одним `EventDetector`-экземпляром (или
  использует свой dict, если архитектор сочтёт DI лишним).
- `should_react_to_event(<source_kind>)` — per-source кулдаун.
- `mark_event_reacted(<source_kind>)` — фиксация «заговорили».

Это **оживляет** мёртвый примитив и одновременно **не дублирует** его:
Повод — обёртка над EventDetector + стаб-фильтр + глобальный дебаунс +
одноразовые маркеры.

> **Уточнение по семантике**: `EventDetector.should_react_to_event` уже
> имеет edge-detection (`check_state_change`), но для кулдауна
> используется **не** edge, а **интервал**. Повод использует только
> кулдаун-часть; edge-detection не нужен.

### 3.3. Изменения в `dialogue_node.py`

#### 3.3.1. `_on_stt` (`:2189-2217`) — байт-в-байт эквивалентно

```python
# БЫЛО: dialogue_node.py:2189
if tg_chat_id is None and not has_wake_word(text_lower, self._wake_words):
    accumulator = getattr(self, "_speech_accumulator", None)
    if getattr(self, "_accumulate_no_wake_enabled", False) and accumulator is not None:
        accumulator.add(...)
        self.get_logger().info(...)
    else:
        self._llm_skipped_counter["no_wake_word"] += 1
        self.get_logger().info(...)
        self._maybe_log_skip_summary()
    return

# СТАЛО (после введения Повода):
# 1. wake_word — это user-initiated повод; gating остаётся прежним.
# 2. НИКАКИХ изменений в самой проверке has_wake_word.
# 3. НИКАКИХ изменений в поведении backlog/counter.

# Опционально (НЕ в первом PR) — добавить hook после успешного wake:
verdict = self._occasion.may_speak(Occasion(kind="wake_word",
                                             is_user_initiated=True))
# verdict всегда ALLOW для user_initiated — hook чисто наблюдательный.
```

**Регрессия здесь недопустима.** Существующие юнит-тесты `_on_stt` /
`has_wake_word` проходят без изменений ожидаемых значений. Подробнее —
§6 Acceptance.

#### 3.3.2. `_dispatch_turn` (`:2742-2800`) — новый параметр `occasion`

```python
# СТАЛО:
def _dispatch_turn(self, text: str, *,
                   tg_chat_id: str | None = None,
                   is_synthetic: bool = False,
                   occasion: Occasion | None = None) -> None:
    ...
```

**Семантика `is_synthetic`** остаётся прежней: «повторный внутренний
вызов того же хода» (babble / code / action-claim / …). Это **не**
меняется.

**Семантика `occasion`**: если `occasion is not None`, после
успешного `_run_turn` сделать `self._occasion.mark_consumed(occasion)`.
Для повода `wake_word` с `is_user_initiated=True` `mark_consumed`
**не нужен** (wake_word не имеет per-source кулдауна — это
user-initiated).

#### 3.3.3. `_on_startup_greeting*` (`:5842-5894`) — миграция

```python
# БЫЛО:
def _on_startup_greeting_finish(self):
    if self._startup_greeting_fired:
        return
    self._startup_greeting_fired = True
    ...

# СТАЛО:
def _on_startup_greeting_finish(self):
    verdict = self._occasion.may_speak(
        Occasion(kind="startup", is_user_initiated=False,
                 payload={"text": pick_greeting(self._startup_greeting_text)}))
    if verdict.kind != VerdictKind.ALLOW:
        self.get_logger().info(f"startup greeting deferred: {verdict.reason}")
        return
    self._occasion.mark_consumed(Occasion(kind="startup"))
    # ... остальная логика sfx → phrase → _publish_response
```

Проверка удалением: без `OccasionGate` поведение совпадает (один раз за
uptime, при DSM=IDLE). С `OccasionGate` — то же + стандартный
механизм кулдаунов доступен другим источникам.

#### 3.3.4. `_on_inactivity_check` (`:6395-6404`) — потенциальный источник повода

Сейчас метод **только** переводит DSM в IDLE. Если в будущем решено
сделать робота, который через 30 секунд тишины говорит «ты тут?» —
это **новый** источник повода `inactivity_acknowledgement`. В рамках
этого ADR — **не реализуется**, только резервируется место в
`OccasionGate.source_cooldowns`.

#### 3.3.5. DJ-тик (`core/dj_mode.py:230-270`) — миграция

`tick()` решает «можно ли играть музыку и нужен ли DJ-комментарий».
Сейчас `next_transition_at = now + POSTPONE_INTERVAL_S` (если диалог
активен) и жёсткий стоп по `DJ_AUTO_MAX_TRANSITIONS`.

**Предложение**: если DJ-тик хочет **заговорить** (а не только
передать музыку), он публикует `Occasion(kind="dj_tick")` и ждёт
`ALLOW`. Если `DEFER` — пропускает этот тик; `REFUSE` не ожидается
для DJ.

**В первом PR** (только Повод без DJ-миграции) — DJ продолжает
работать как раньше; миграция — в отдельной worker-карточке.

### 3.4. Стаб-фильтр (vision only)

Это **критическая** защита от ложных срабатываний в период, пока
#2531/#2532/#2398 не закрыты.

```python
# core/occasion.py — стаб-фильтр
if occasion.kind == "meeting":
    payload = occasion.payload or {}
    event_type = payload.get("event_type")
    source_camera = payload.get("source_camera")
    if (event_type == self._stub_event_type
            and source_camera in self._stub_source_cameras):
        return Verdict(VerdictKind.REFUSE,
                       f"stub event: event_type={event_type!r} "
                       f"source_camera={source_camera!r}")
```

**Текущая сигнатура стаба** (подтверждено в issue #2532):
`event_type='person'`, `source_camera='unknown'`. До закрытия #2531
(`event_type=='person'` + явный не-стаб маркер) **любой** повод
`meeting` с этими полями — `REFUSE`.

**Условие снятия стаб-фильтра** (после #2531): стаб-фильтр остаётся,
но список «плохих» `source_camera` сужается до того, что **физически
не может быть реальным кадром** (например, маркер `source_camera='stub'`
или специальное поле `payload.is_real: bool`). До этого момента —
консервативный фильтр.

### 3.5. Что НЕ делаем в этом PR

1. **Не подключаем `meeting` как живой источник повода.** Повод
   принимает `meeting` в API, но **вызывающая сторона** пока не
   существует. Это даёт каркас для #2442 без риска болтливости.

2. **Не подключаем `inactivity_acknowledgement`.** Резервируем имя в
   `source_cooldowns`, но не публикуем повод.

3. **Не мигрируем DJ-тик на Повод.** Отдельная worker-карточка.

4. **Не удаляем `_startup_greeting_fired`.** В этом PR — Повод
   начинает работать **параллельно** с флагом; миграция полная
   (флаг удалён, Повод — единственный источник истины) — отдельный
   шаг после стабилизации.

5. **Не чиним `listen_for_response` паблишер в пустоту.**
   Задокументировано в §5; чинится отдельной issue.

6. **Не удаляем осиротевший `startup_greeting_node.py`.** В §5
   зафиксировано как находка; удаление — отдельная worker-карточка.

---

## 4. Альтернативы, которые рассматривали

### A. CQRS-разделение повода от данных

**Идея**: отдельный ROS2-сервис `occasions/evaluate`, который принимает
событие и возвращает `Verdict`.

**Почему отвергнута**: лишний слой для текущего масштаба (RPS ≈ 0.1).
Повод вызывается синхронно из callback-а подписки `dialogue_node`;
RTT к сервису не нужен. Если в будущем нагрузка вырастет — переход на
сервис тривиален (заменить `self._occasion.may_speak(...)` на
`self._occasion_client.call_async(...)`).

### B. Event-sourcing для поводов

**Идея**: хранить все поводы в журнале, разрешать на основе replay.

**Почему отвергнута**: для шва «можно ли заговорить» event-sourcing
не нужен. Это **не аудит** и **не воспроизведение**; это решение на
месте. Если в будущем понадобится аудит «почему робот сказал X в
момент Y» — добавим логирование, не event store.

### C. Reinforcement learning для политики поводов

**Идея**: ML-модель решает «когда говорить» на основе истории.

**Почему отвергнута (в этом ADR)**: вне scope. ADR закрепляет
**детерминированный** шов с параметризованными cooldowns. ML-политика
может быть добавлена **поверх** (выход Повода — фича для ML), но не
вместо.

### D. Полностью консолидировать ВСЕ кулдауны в один EventDetector

**Идея**: всё состояние кулдаунов — в `EventDetector`.

**Почему отчасти принята, но не полностью**: `EventDetector` используется
**как per-source кулдаун**, но не как единственный шов (стаб-фильтр,
общий дебаунс, одноразовые маркеры — не его ответственность).
Полная консолидация = смешение ответственности.

### E. Не вводить Повод вообще — пусть каждый источник имеет свой флаг

**Идея**: «работает же сейчас».

**Почему отвергнута**: болтливость. Без общего дебаунна робот может
говорить от DJ-тика и встречи одновременно. Без per-source кулдауна —
на каждый кадр. Без стаб-фильтра — на каждый тик стаба (каждые 2
секунды, см. #2532). Сценарий «Денис заходит → робот приветствует»
без дисциплины превращается в «Денис заходит → робот говорит каждую
секунду до конца разговора». ADR-0013 (incremental delivery) тоже
против «ещё один флаг в __init__» — ровно та же bug-class, что в
#1389 (PR #1395 закрепил R2 State SSoT).

---

## 5. Находки вне scope этого PR (зафиксировать отдельно)

Эти находки обнаружены в ходе анализа, но **не чинятся** этим PR:

| Находка | Файл:строка | Что с ней делать |
|---|---|---|
| Осиротевший `startup_greeting_node.py` (есть в дереве, нигде не запускается; единственный импортёр — собственный тест) | `src/rob_box_voice/rob_box_voice/startup_greeting_node.py`, `src/rob_box_voice/setup.py:51-66`, оба launch-файла | **Отдельная worker-карточка** `chore(voice): удалить осиротевший startup_greeting_node.py`. Решение: удалить или зарегистрировать. **Не блокирует** ADR-0101. |
| `/voice/stt/request` без подписчика | `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/dialogue.py:706`, `scripts/lint/seam_baseline.json:17` | **Отдельная worker-карточка** `fix(mcp): listen_for_response → реальный подписчик`. Следствие: даже после введения Повода продолжение диалога после реплики «привет, Денчик» всё равно требует нового вейк-слова. Сценарий-заказчик предусматривает это исправление отдельным шагом. |
| `_drain_pending_user_messages` (`:6241`) — `is_synthetic` не передан → `False` | `dialogue_node.py:6241` | Стилистически стоит явно передать `is_synthetic=True` и добавить поясняющий комментарий «склейка накопленных фраз ОДНОГО диалога». Микро-PR, не блокирует. |
| ADR-0070/0029 не противоречат ослаблению вейк-гейта | (см. issue body §Риски 2) | Зафиксировать в PR-описании ADR-0101: ни одно из этих ADR не говорит о запрете автономных поводов; оба — про cold-start race VAD и STT теряет wake на первом слове. Ослабление вейк-гейта для **нового источника** повода (а не для текстового входа) не взаимодействует с этой проблемой. |

---

## 6. Acceptance (измеримо, без «вроде работает»)

После реализации **обязательны** все пункты ниже. Каждый — атомарный,
проверяемый, с конкретным артефактом.

### 6.1. Регрессия на вейк-слово

- **Что**: набор unit-тестов `_on_stt` / `has_wake_word` /
  `_dispatch_turn` (обычный ход, TG-маркер, no_wake_word →
  backlog/counter) даёт **идентичный** результат до и после введения
  Повода.
- **Артефакт**: `git diff origin/develop..HEAD -- src/rob_box_voice/test/unit/node/test_barge_in_policy.py src/rob_box_voice/test/unit/core/test_wake_words_config.py` →
  **expected: 0 changed lines в expected_values** (могут быть новые
  тесты, не правки существующих).
- **Команда проверки**:
  ```bash
  cd /workspace/src/rob_box_voice
  python -m pytest test/unit/node/test_barge_in_policy.py \
                   test/unit/core/test_wake_words_config.py \
                   -v 2>&1 | tee /tmp/wake_regression.txt
  ```
- **Критерий PASS**: 0 failed, 0 errored; все expected значения
  идентичны.

### 6.2. Per-source кулдаун (синтетический интеграционный)

- **Что**: при повторной подаче одного и того же повода «meeting» с
  cooldown=10s чаще, чем раз в 10 секунд, `may_speak` возвращает
  `DEFER` на все вызовы, кроме первого.
- **Артефакт**: `src/rob_box_voice/test/unit/core/test_occasion.py` —
  новый unit-тест, без ROS2.
- **Команда проверки**:
  ```bash
  cd /workspace/src/rob_box_voice
  python -m pytest test/unit/core/test_occasion.py -v 2>&1 | tee /tmp/occasion_test.txt
  ```
- **Критерий PASS**: тест зелёный; первый `may_speak` → ALLOW,
  повторный через 1 секунду → DEFER с `retry_after_s ≈ 9.0`.

### 6.3. Глобальный дебаунс

- **Что**: при подаче `Occasion(kind='dj_tick')` через 0.5 секунды
  после `Occasion(kind='meeting')` (с разными source_cooldowns, оба
  выше глобального дебаунса) — второй → DEFER с
  `reason='global debounce'`.
- **Артефакт**: тот же `test_occasion.py`, новый test-case.
- **Критерий PASS**: тест зелёный.

### 6.4. Стаб-фильтр

- **Что**: `Occasion(kind='meeting', payload={'event_type': 'person',
  'source_camera': 'unknown'})` → `REFUSE` (без ALLOW).
- **Артефакт**: `test_occasion.py::test_stub_filter_refuses`.
- **Критерий PASS**: тест зелёный.

### 6.5. Одноразовый маркер (startup)

- **Что**: первый вызов `may_speak(Occasion(kind='startup'))` →
  ALLOW. Второй (без `mark_consumed` на первый) → DEFER с
  `reason='one-shot already consumed'`. После `mark_consumed` —
  DEFER (`one-shot already consumed`).
- **Артефакт**: `test_occasion.py::test_startup_one_shot`.
- **Критерий PASS**: тест зелёный.

### 6.6. user_initiated bypass

- **Что**: `Occasion(kind='wake_word', is_user_initiated=True)` —
  ВСЕГДА `ALLOW`, независимо от cooldowns.
- **Артефакт**: `test_occasion.py::test_user_initiated_bypasses_cooldown`.
- **Критерий PASS**: тест зелёный.

### 6.7. `_on_stt` байт-в-байт (полный)

- **Что**: после введения Повода поведение `_on_stt` для
  существующих unit-тестов — без изменений.
- **Артефакт**: `git diff origin/develop..HEAD -- 'src/rob_box_voice/test/unit/node/*'` →
  **только новые файлы / новые тесты**; `expected_values` неизменны.
- **Команда**:
  ```bash
  cd /workspace/src/rob_box_voice
  python -m pytest test/unit/node/ -v 2>&1 | tee /tmp/all_node_tests.txt
  ```
- **Критерий PASS**: 0 failed, 0 errored; **deliberate identical
  output** до и после.

### 6.8. Интеграция через `_dispatch_turn`

- **Что**: синтетический e2e (НЕ живой робот): повод `meeting` через
  тестовый адаптер → `_dispatch_turn(..., is_synthetic=True,
  occasion=Occasion(kind='meeting', payload={'event_type': 'person',
  'source_camera': 'main_camera'}))` → `_run_turn` доходит до LLM с
  ходом, **инициатором которого не было STT/TG-сообщение в этом
  цикле**.
- **Артефакт**: `test/unit/node/test_occasion_dispatch.py::test_meeting_occasion_runs_turn`.
- **Команда**:
  ```bash
  cd /workspace/src/rob_box_voice
  python -m pytest test/unit/node/test_occasion_dispatch.py -v 2>&1 | tee /tmp/dispatch_test.txt
  ```
- **Критерий PASS**: тест зелёный; в captured logs —
  `started turn occasion=meeting` без wake_word.

### 6.9. `EventDetector` оживлён

- **Что**: `EventDetector` имеет **нового** продакшен-импортёра
  (`OccasionGate`).
- **Артефакт**: `git grep "from rob_box_perception.core.event_detector import EventDetector" -- 'src/'` →
  **2 совпадения**: тест (`test/unit/core/test_event_detector.py`) +
  прод (`core/occasion.py`).
- **Критерий PASS**: ровно 2 совпадения; ни одного другого прода.

### 6.10. e2e-маркер (для следующего раунда, не блокирует PR)

Этот PR — **каркас**. Живой e2e-тест с реальным роботом —
**отдельная e2e-карточка** после того, как:
- Повод стабилизируется в unit-тестах (6.1-6.9 PASS);
- Шов `meeting` получит хотя бы фиктивный вызыватель
  (тестовый адаптер).

**Маркер для e2e-процесса** (заполняется воркером, не архитектором):

```yaml
## e2e
# Когда станет возможным (после #2442 + #2531 closed):
voice_text: "Робот, представь — Денис зашёл в мастерскую"
voice_file: .github/e2e/voice_commands/rabot_denis_prishyol.ogg
volume: 150
record_seconds: 60
llm: minimax-m3
tts: minimax-male-qn-qingse
stt: yandex
# Ожидание: робот поднимает встречу через Повод → НЕ говорит сам
# (стаб-фильтр REFUSE), логирует "[occasion] refused: stub event"
```

---

## 7. Что НЕ меняется (жёсткая граница)

Это **не** входит в этот PR. Воркеру-реализатору **запрещено** трогать
эти файлы без отдельного issue:

1. `core/dj_mode.py` — миграция DJ-тика на Повод = отдельная карточка.
2. `stt_node._maybe_speak_unclear` — миграция = отдельная карточка.
3. `_drain_pending_user_messages` (`:6241`) — стилистическое
   `is_synthetic=True` = микро-PR, не блокирует.
4. Удаление `_startup_greeting_fired` — после стабилизации
   (отдельный шаг).
5. Реальный подписчик на `/voice/stt/request` — отдельная issue.
6. Удаление `startup_greeting_node.py` — отдельная worker-карточка.
7. Подключение живого `meeting` источника (после #2442+#2531) —
   отдельная карточка **с обязательной** защитой стаб-фильтра.

---

## 8. План реализации (для воркера backend)

После accept этого ADR — воркер реализует **один** PR в следующем порядке:

1. **PR-A (этот ADR, после accept)**: реализация `core/occasion.py` +
   юнит-тесты 6.1-6.7 + `EventDetector`-reactivation 6.9.
2. **PR-B** (после PR-A): миграция `_on_startup_greeting*` на Повод;
   `_startup_greeting_fired` остаётся как fallback (параллельно).
3. **PR-C** (после PR-B): удаление `_startup_greeting_fired`;
   Повод — единственный источник истины.
4. **PR-D** (отдельный поток): миграция DJ-тика.
5. **PR-E** (отдельный поток): миграция `_maybe_speak_unclear`.
6. **PR-F** (после #2442 + #2531): живой `meeting` источник через
   подписку `/perception/context_update`.

Каждый PR — отдельная kanban-карточка. PR-A — **минимум** для
закрытия #2536. PR-B-F — follow-up.

---

## 9. Метрики (baseline → target)

| Метрика | Baseline | Target (после PR-A) | Способ проверки |
|---|---|---|---|
| Файлов в `dialogue_node.py` | (текущее) | **без изменений** в этом PR | `wc -l src/rob_box_voice/rob_box_voice/dialogue_node.py` |
| Продакшен-импортёров `EventDetector` | 0 | 1 (`OccasionGate`) | `git grep -c "from rob_box_perception.core.event_detector"` в `src/rob_box_voice/` |
| Юнит-тестов на Повод | 0 | ≥ 6 (6.2-6.6 + dispatch) | `pytest test/unit/core/test_occasion.py --collect-only -q` |
| Связей «подписка → решение» в `dialogue_node.py` | размазаны | в одном месте (callback строит `Occasion` и зовёт `may_speak`) | code review PR-A |
| Одноразовых маркеров (булев флаг `_startup_greeting_fired`) | 1 | 1 (флаг ещё есть, миграция PR-B/C) | grep |

---

## 10. Связанные ADR и принципы

- **ADR-0021 R2 State SSoT** — каждый счётчик/словарь/enum имеет
  один источник правды. Повод консолидирует 4 разрозненных кулдауна в
  один шов.
- **ADR-0013 Incremental delivery** — этот PR — каркас; полная
  миграция (PR-B-F) — отдельными маленькими PR.
- **ADR-0018 Capability-honest** — стаб-фильтр обязателен, пока
  источник не отличим от заглушки. Это capability-honest применённый
  к новой возможности.
- **ADR-0065** — wake-words SSoT в коде. Повод **не** трогает
  wake-слова; байт-в-байт эквивалентная семантика `_on_stt`.
- **ADR-0029, ADR-0070** (Proposed) — оба говорят о cold-start race
  VAD и STT-теряет-wake. Не взаимодействуют с Поводом (новый повод
  минует STT/VAD целиком).

---

## 11. Решение

**Принять** ADR-0101 со следующими условиями:

1. Реализация идёт через серию маленьких PR (PR-A…F) — §8.
2. PR-A — минимальный каркас: `core/occasion.py` + 6 unit-тестов
   (6.2-6.6) + 1 dispatch-тест (6.8) + регрессионный 6.1/6.7 +
   оживление `EventDetector` (6.9).
3. Живой `meeting` источник **не подключается** до закрытия #2442 +
   #2531.
4. Вейк-слово остаётся **первым источником** повода, байт-в-байт
   эквивалентное поведение (тесты 6.1/6.7 PASS).
5. Стаб-фильтр обязателен с первого дня.

**Статус**: Proposed (ждёт accept от товарища Шифу перед началом
реализации).
