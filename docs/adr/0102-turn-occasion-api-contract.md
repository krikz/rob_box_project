# ADR-0102 — «Повод» API: контракт между Perception-серией и dialogue_node

|| Поле | Значение |
||---|---|
|| Статус | Proposed (issue #2536, kanban t_38c55959) |
|| Дата | 2026-09-15 |
|| Автор | architect worker (t_38c55959) |
|| Родительский ADR | **ADR-0101** «Повод — единый шов „можно ли заговорить"» (PR #2575, смержен 2026-09-15 13:37, merge_sha `e7c9abc6`, docs/adr/0101-occasion-unified-turn-entry.md) |
|| Решает | task body брифа: «спроектировать контракт повода: API топика и decision-ноды» |
|| Дочерние карточки | t_7f1a4919 (tester, e2e-сценарий «Денис заходит»), t_9d3b4421 (techwriter, **DEPRECATED** — номер ADR-0066 занят, см. §0), t_a98a25e1 (architect, разведка текущих топиков perception_*) |
|| Связанные issue | #2536 (родитель), #2531 (Взгляд, merged), #2532 (Проекция восприятия), #2442 (Встреча), #2440 (Знакомый, merged), #2398 (real inference), #992 (Bug A-F), #1881 (`_synthetic_retries_left`) |
|| Топики, которые этот ADR именует | `/voice/turn_occasion` (отвергнут, см. §3.2), `/voice/occasion/request` (отвергнут), `/voice/occasion/verdict` (отвергнут), `Occasion` Python-объект — **принят** |

> **TL;DR.** **Никакого нового ROS2-топика повода не вводится.** Повод — это
> **Python-объект `Occasion`**, конструкция которого живёт в callback-ах
> подписок `dialogue_node` на существующие топики-данные
> (`/perception/context_update`, `/vision/hailo/events`,
> `/voice/dialogue/response`, `/voice/dj_mode`, `/voice/stt/result`),
> а **решение** «заговорить или нет» принимает in-process
> `OccasionGate.may_speak(Occasion) → Verdict` (ADR-0101 §3.1).
> Этот ADR фиксирует **контракт API** между Perception-серией
> (#2531/#2532/#2442) и `dialogue_node`: какие данные из этих топиков
> превращаются в `Occasion`, кто строит объект, и как `Occasion` проходит
> через `OccasionGate`. Сценарий «Денис заходит → робот говорит первым»
> реализуется без добавления новых ROS2-узлов и без расширения
> LLM-цикла — ADR-0101 §3.1 строка 269 фиксирует эту развилку явно.

---

## 0. Контекст задачи t_38c55959 и почему «API топика» — формулировка-ловушка

Бриф карточки просит «спроектировать контракт повода: API топика и decision-ноды»
с явной опцией «выходной топик `/voice/turn_occasion` (имя — предложить)».
Дочерняя карточка t_9d3b4421 (techwriter) формулирует это буквально как
ADR-0066 «Повод — вход в ход dialogue_node». **Обе формулировки устарели**
на момент создания карточек: **ADR-0101 уже был принят за ~30 минут до**
декомпозиции (`t_38c55959` создан в `2026-09-15 12:48`,
PR #2575 смержен в `2026-09-15 13:37`), и ADR-0101 §3.1 + §4-A **явно
отвергают** ROS2-топик как API повода.

Кроме того, **номер ADR-0066 занят** с 2025 (commit `4f201b487`, PR #2121:
`docs(adr): исправить вторую коллизию ADR-0054 — dialogue/control pause/resume
это ADR-0066`). Номер «0066» в брифе techwriter-карточки — **коллизия имён**,
а не запрос на новый ADR с тем же номером.

Этот ADR (0102) — **уточняющий** к ADR-0101: фиксирует **API-чертёж**
(типы `Occasion` / `Verdict` / `SourceKind`, конструкция, контракт
между perception-callback-ом и `OccasionGate`), **диаграмму потоков данных**
(Mermaid), **правила обновления существующих подписок** в `dialogue_node`
под шов данных-vs-решение, и **trade-off разбор трёх вариантов**
(A — отдельная ROS2-нода, B — расширение dialogue_node, C — подписка
из LLM-цикла) в свете ADR-0101.

---

## 1. Бизнес-проблема (без изменений из issue #2536)

Денис заходит в мастерскую → робот замечает событие → поднимает биометрию →
понимает, кто это → знает, что давно не виделись → заговаривает первым.
#2440 (Знакомый) уже смержен, #2531 (Взгляд) уже смержен
(PR #2578, merge_sha `8143e610`), #2532/#2442 ещё open — но даже когда
все три закроются, робот **физически не сможет заговорить первым**,
потому что у `dialogue_node` нет шва «можно ли инициировать ход извне».

Этот шов = **Повод** (ADR-0101). Настоящий ADR фиксирует **как именно**
повод попадает в `dialogue_node`.

## 2. Что есть сегодня (raw, проверено 2026-09-15)

### 2.1. Существующие подписки `dialogue_node` (полный реестр, `dialogue_node.py:638-782`)

|| # | Топик | Тип | Что используется для |
||---|---|---|---|
|| 1 | `/voice/stt/result` | `String` | wake-gate → `_dispatch_turn` |
|| 2 | `/dialogue/control` | `String` | ADR-0066 pause/resume FSM |
|| 3 | `/voice/command/feedback` | `String` | команда-результаты |
|| 4 | `/voice/stt/speaker` | `String` | speaker-tagging |
|| 5 | `/voice/speaker/result` | `String` | speaker-id результат |
|| 6 | `/voice/speaker/epithet_request` | `String` | эпитеты |
|| 7 | `/audio/vad` | `Bool` | VAD-сегменты |
|| 8 | `/voice/tts/finished` | `String` | TTS завершил |
|| 9 | `/voice/tts/current_voice` | `String` | текущий голос |
|| 10 | `/voice/tts/provider_state` | `String` | состояние TTS-провайдера |
|| 11 | `/voice/generated_music/state` | `String` | генератор музыки |
|| 12 | `/voice/music/state` | `String` | плеер музыки |
|| 13 | `/voice/tts/batch_complete` | `String` | TTS-батч завершён |
|| 14 | `/voice/tts/batch_registered` | `String` | TTS-батч зарегистрирован |
|| 15 | `/voice/sound/state` | `String` | sound-плеер |
|| 16 | `/odom` | `Odometry` | только `_pose_snapshot` |
|| 17 | `/voice/dj_mode` | `String` | DJ-тик (→ `_dispatch_dj_turn` → `_dispatch_turn` без `is_synthetic`) |
|| 18 | `/mcp/tools` | `String` | MCP tool-skip retry (issue #1777) |

**Ни одного топика восприятия** (`/perception/context_update`,
`/vision/hailo/events`) в подписках `dialogue_node` нет. Это **raw-факт**:
`grep -n "create_subscription" src/rob_box_voice/rob_box_voice/dialogue_node.py`
подтверждает 18 подписок и **ровно 0** подписок на perception.

### 2.2. PerceptionEvent-серия публикует (проверено `context_aggregator_node.py`)

`/perception/context_update` (тип `PerceptionEvent` из
`src/rob_box_perception_msgs/msg/PerceptionEvent.msg`) публикуется
`context_aggregator_node.publish_event()` с частотой `publish_rate=2.0 Hz`.
Поля (61 строка, выборочно): `vision_events_json` (JSON-массив
`VisionEvent`), `current_time_human`, `time_period`, `internet_available`,
`battery_voltage`, `system_health_status`, `memory_summary`,
`speech_summaries`. **Сейчас не читается никем в `dialogue_node`** —
это и есть «последний недостающий шов».

`/vision/hailo/events` (тип `VisionEvent` из
`src/rob_box_perception_msgs/msg/VisionEvent.msg`) публикуется
`vision_hailo_node` с полями `source_camera`, `event_type`, `class_name`,
`bbox_*`, `confidence`, `embedding_id`, `display_name`, `attributes_json`.
**Сейчас потребляется только `context_aggregator_node`** (`_hailo_events_sub`
на :225) и его собственным юнит-тестом. В `dialogue_node` подписки нет.

### 2.3. Существующие «размазанные» кулдауны (все в `dialogue_node.py` и смежных нодах)

|| Кулдаун | Файл:строка | Семантика |
||---|---|---|
|| `_startup_greeting_fired` (булев) | `dialogue_node.py:5856-5858` | один раз за весь uptime |
|| `_last_unclear_at` (`unclear_cooldown_s`) | `stt_node.py:876-888` | per-source, нет глобального дебаунса |
|| `next_transition_at` (DJ) | `dj_mode.py:230-270` | per-source + диалог/sound активны → postpone |
|| `form_ends_at` (DJ) | `dj_mode.py:263-265` | форма как нижняя граница |
| `_on_inactivity_check` (5с таймер) | `dialogue_node.py:6395-6404` | только DSM→IDLE, **никогда не говорит** |

**Ни один из них не знает о существовании других.** Это и есть
то, что ADR-0101 §1.2 п.6 называет «размазанные кулдауны».

### 2.4. Что ADR-0101 уже решил и что осталось открытым

ADR-0101 §3.1 **уже ввёл** API в виде Python-модуля:

```python
class OccasionGate:
    def may_speak(self, occasion: Occasion, now: float | None = None) -> Verdict: ...
    def mark_consumed(self, occasion: Occasion, now: float | None = None) -> None: ...
```

§3.3 уже **зафиксировал швы** в `dialogue_node.py`:

- `_on_stt` (`:2189-2217`) — байт-в-байт эквивалентно, без изменений gate
- `_dispatch_turn` (`:2742-2800`) — новый kw-параметр `occasion: Occasion | None = None`
- `_on_startup_greeting*` (`:5842-5894`) — миграция на `Occasion(kind="startup", ...)`
- `_on_inactivity_check` (`:6395-6404`) — резерв имени, без реализации
- DJ-тик (`dj_mode.py:230-270`) — резерв имени, без реализации

**Что осталось открытым из брифа t_38c55959** (и что этот ADR закрывает):

1. **Точные типы и контракт `Occasion`** для perception-источников:
   кто строит, какие поля обязательны, как сериализуется (если
   когда-нибудь понадобится логирование).
2. **Конструкция `Occasion` в callback-ах** новых perception-подписок
   в `dialogue_node`: что триггерит повод `meeting` от
   `VisionEvent.event_type=='person'`, что триггерит повод от
   `PerceptionEvent.time_period` change, и т.п.
3. **Связь с `_dispatch_turn`**: явный kw `occasion=...` уже есть
   в ADR-0101 §3.3.2, но **не документировано**, как `Occasion` живёт
   вместе с `is_synthetic=True` (для ретраев) и `is_synthetic=False`
   (для нового хода от повода).
4. **Trade-off разбор** вариантов A/B/C с явным verdict «выбрано B
   (in-process), A отвергнут потому-то, C отвергнут потому-то».
5. **Mermaid-диаграмма** потоков данных: кто куда публикует, где
   конструируется `Occasion`, где живёт `OccasionGate`.

## 3. Решение: API-чертёж (in-process, без новых ROS2-узлов)

### 3.1. Три варианта из брифа — verdict

| Вариант | Суть | Вердикт | Обоснование |
|---|---|---|---|
| **A. Отдельная ROS2-нода `occasion_node`** с топиком `/voice/turn_occasion` (или `/voice/occasion/request` + `/voice/occasion/verdict`) | Новая нода подписывается на perception-топики, решает `may_speak`, публикует `TurnOccasion`. `dialogue_node` подписывается на `/voice/turn_occasion` | **Отвергнут** | ADR-0101 §4-A уже отверг CQRS-вариант: RPS ≈ 0.1, RTT к сервису не нужен, повод синхронный в callback-е. **Дополнительно**: новая нода = новый healthcheck, новый launch-file, новая зависимость в `docker-compose`, новый параметр `ROS_DOMAIN_ID`. Повод — **решение**, не **сервис** (ADR-0101 §3.1 строка 269: «`OccasionGate` подписчиком ROS2-топиков — слишком много ответственности»). |
| **B. Расширение `dialogue_node` через in-process модуль `core/occasion.py`** (ADR-0101 §3.1) | `dialogue_node` подписывается на perception-топики **только для данных** (ADR-0101 §3.1 п.6), callback-и конструируют `Occasion(...)` и зовут `self._occasion.may_speak(...)`. `Verdict` синхронный. | **Принят** | ADR-0101 уже это зафиксировал. Шов «решение» отделён от шва «данные». Подписки добавляются к существующему списку `dialogue_node`, новых узлов нет. |
| **C. Подписка прямо в LLM-цикле** (`_run_turn` дополнительно подписан на perception) | LLM-цикл сам подписывается на perception, конструирует `Occasion`, вызывает `may_speak`. | **Отвергнут** | `_run_turn` уже занят: 10 вызовов `_dispatch_turn` (ADR-0101 §1.2 п.5), 7 из них — ретраи с `is_synthetic=True`. Подписка в `_run_turn` нарушает §3.1 п.6: шов «решение» смешивается с телом хода, callback приходит во время LLM-инструментального вызова, race на `self._synthetic_retries_left`. Тестируемость падает: `_run_turn` становится одновременно потребителем, решателем и исполнителем. |

**Trade-off резюме**:

|| Что выигрываем | Что теряем |
|---|---|---|
| A (отдельная нода) | Изоляция `OccasionGate` (можно тестировать как сервис), потенциальный RPS-headroom | +1 healthcheck, +1 launch-file, +1 docker-compose сервис, +latency (RTT), ADR-0101 §4-A отверг |
| **B (in-process, выбран)** | Минимальный diff, переиспользует существующие подписки `dialogue_node`, шов «решение» отделён от «данных» (ADR-0101 §3.1 п.6) | `OccasionGate` тестируется без ROS2 (юнит-тест, ADR-0101 §6.2) — это **плюс**, не минус |
| C (LLM-цикл) | Минимальный код в `dialogue_node` (вообще не трогаем `__init__`) | Race на `self._synthetic_retries_left`, теряем шов «решение»-отдельно-от-«данные», LLM-цикл становится god-object |

### 3.2. Почему `/voice/turn_occasion` (имя из брифа) **не используется**

Бриф предлагает имя `/voice/turn_occasion`. **Причины не использовать**:

1. **Нет подписчиков и паблишеров на этот топик в текущем коде**
   (`grep -rn "/voice/turn_occasion" src/` → 0 совпадений в `develop`,
   `git grep` по `origin/develop` → 0). Вводить топик «с нуля» = новый
   seam-разрыв (publisher без локального subscriber наоборот: на этот
   топик должен быть **один** паблишер — сама `dialogue_node` —
   и **ноль** локальных подписчиков; подписчик — внешний e2e-харнесс,
   ADR-0101 §6.10). Это **анти-паттерн seam-baseline**, зафиксированный
   в `scripts/lint/seam_baseline.json:17` для аналогичного топика
   `/voice/stt/request` (паблишер без подписчика).

2. **Топик создаёт второй шов «решение»-отдельно-от-«данные»**:
   если повод публикуется в `/voice/turn_occasion`, то
   `dialogue_node` подписан на этот топик **и на perception-топики**
   одновременно. Это нарушает ADR-0101 §3.1 п.6: «dialogue_node подписан
   на топики-источники данных **только** для целей данных». Два канала =
   два места, где может «размазаться» решение.

3. **RTT не нужен**: `may_speak` — синхронная pure-функция
   (`ADR-0101 §3.1:202-262`). RTT к сервису/топику — лишний латенси
   на пути «callback → решение → `_dispatch_turn`» (типичный бюджет
   повода: `global_debounce_s=2.0`, `per-source cooldown` — порядка
   десятков секунд).

4. **Для observability** достаточно существующего топика
   `/voice/dialogue/state` (он публикуется `dialogue_node._publish_state`,
   `dialogue_node.py:2120+`) или `/rosout` — повод-метрика
   `self._occasion.stats()` (ADR-0101 §3.1:255-261) логируется через
   `get_logger().info(...)` и попадает в `/rosout` без нового топика.

**Где имя `/voice/turn_occasion` оправдано** (вне scope этого ADR):
e2e-харнесс может **читать** этот топик, если отдельная worker-карточка
когда-нибудь **решит** вынести `OccasionGate` в отдельную ноду —
но это явный future-migration, ADR-0101 §4-A: «переход на сервис
тривиален, заменить `self._occasion.may_speak(...)` на
`self._occasion_client.call_async(...)`».

### 3.3. Контракт API (Python, in-process)

Это **расширение** ADR-0101 §3.1 до полного типа с perception-полями.
Точное расположение: `src/rob_box_voice/rob_box_voice/core/occasion.py`
(подтверждено ADR-0101 §3.1 строка 137).

```python
# core/occasion.py — API-чертёж для PR-A (ADR-0101 §8)
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Any


class VerdictKind(str, Enum):
    ALLOW = "allow"        # ход разрешён, инициатор = источник повода
    DEFER = "defer"        # кулдаун / глобальный дебаунс / one-shot
    REFUSE = "refuse"      # стаб-фильтр / disabled


class SourceKind(str, Enum):
    """Зарегистрированные источники повода.
    Чтобы расширить — добавить элемент + кулдаун в source_cooldowns.
    """
    WAKE_WORD          = "wake_word"               # ADR-0101 §3.3.1
    DJ_TICK            = "dj_tick"                  # ADR-0101 §3.3.5
    STARTUP            = "startup"                  # ADR-0101 §3.3.3
    # MEETING / INACTIVITY_ACKNOWLEDGEMENT — зарезервированы,
    # подключаются в PR-F (ADR-0101 §8) после #2442+#2531.


@dataclass(frozen=True)
class Occasion:
    """Семантическое описание «что даёт право заговорить»."""
    source: SourceKind          # кто сообщил о поводе
    payload: dict = field(default_factory=dict)
                                # контекст источника:
                                #   WAKE_WORD   → {"text": "...", "wake_phrase": "..."}
                                #   DJ_TICK     → {"intent": "...", "track": "..."}
                                #   STARTUP     → {"text": "...", "sec": 12.0}
                                #   MEETING     → {"event_type": "person",
                                #                  "source_camera": "...",
                                #                  "display_name": "...",
                                #                  "embedding_id": "..."}
                                #   INACTIVITY  → {"idle_sec": 35.0}
    is_user_initiated: bool = False
                                # True ТОЛЬКО для WAKE_WORD / TG-маркера
                                # (user_initiated = всегда ALLOW, ADR-0101 §3.1:226-227)


@dataclass
class Verdict:
    kind: VerdictKind
    reason: str                 # человекочитаемое объяснение (для /rosout)
    retry_after_s: Optional[float] = None  # для DEFER — когда повторить


class OccasionGate:
    """Контракт ADR-0101 §3.1 + §3.2. Реализация — в PR-A."""

    def may_speak(self, occasion: Occasion,
                  now: float | None = None) -> Verdict: ...
    def mark_consumed(self, occasion: Occasion,
                      now: float | None = None) -> None: ...
    def stats(self) -> dict: ...   # для /rosout-логирования
```

**Инвариант** (ADR-0101 §2 п.3, дословно): **«Стаб-фильтр обязателен для
зрения»** — повод `MEETING` с `payload.event_type == 'person'` И
`payload.source_camera in {'unknown', 'stub'}` → REFUSE.
`SourceKind.MEETING` принимается API сразу (ADR-0101 §3.5 п.1),
но **вызывающая сторона** пока не существует — это даёт каркас для #2442
без риска болтливости.

### 3.4. Контракт между perception-callback-ом и `OccasionGate`

```python
# dialogue_node.py — НОВЫЙ callback (PR-A НЕ трогает этот код; PR-F — после #2442+#2531)

def _on_perception_event(self, msg: PerceptionEvent) -> None:
    """Шов ДАННЫЕ: только обновляем кэш контекста, повод НЕ строим здесь."""
    # 1. Обновить кэш контекста (для LLM-промпта, не для решения).
    self._perception_cache.update(msg)
    # 2. Никакого may_speak. Это _on_perception_event, не _on_meeting.

def _on_meeting_signal(self, msg: VisionEvent) -> None:
    """Шов РЕШЕНИЕ: конструируем Occasion(MEETING, ...) и зовём may_speak."""
    occasion = Occasion(
        source=SourceKind.MEETING,
        payload={
            "event_type":    msg.event_type,
            "source_camera": msg.source_camera,
            "display_name":  msg.display_name,
            "embedding_id":  msg.embedding_id,
            "confidence":    msg.confidence,
        },
        is_user_initiated=False,
    )
    verdict = self._occasion.may_speak(occasion)
    if verdict.kind is VerdictKind.ALLOW:
        self._dispatch_turn(
            text="",  # LLM сгенерирует приветствие из контекста
            tg_chat_id=None,
            is_synthetic=True,
            occasion=occasion,        # ← НОВОЕ (ADR-0101 §3.3.2)
        )
        self._occasion.mark_consumed(occasion)
    else:
        self.get_logger().info(
            f"🎭 [occasion] meeting {verdict.kind.value}: {verdict.reason}"
        )
```

**Ключевое разделение** (ADR-0101 §3.1 п.6):

| Callback | Шов | Зовёт `may_speak`? | Зовёт `_dispatch_turn`? |
|---|---|---|---|
| `_on_stt` (existing) | gate + решение (wake_word) | ДА (наблюдательно, ADR-0101 §3.3.1) | ДА (через has_wake_word) |
| `_on_perception_event` | **данные** | НЕТ | НЕТ |
| `_on_meeting_signal` | **решение** | ДА | ДА (только при ALLOW) |
| `_on_dj_mode` (existing) | gate + решение (dj_tick) | ДА (PR-D) | ДА (через `_dispatch_dj_turn`) |
| `_on_startup_greeting` (existing) | gate + решение (startup) | ДА (PR-B) | ДА (через `_publish_response`) |
| `_on_inactivity_check` | (резерв имени) | — | — (PR-E) |

**`_on_perception_event`** существует **только** для обновления
`_perception_cache` (имя/время суток/батарея/контекст встречи),
которое LLM-промпт читает при формировании ответа. **Решения
о начале хода здесь нет** — это устраняет риск «размазывания» решения
по callback-ам подписки (ADR-0101 §3.1 п.6).

### 3.5. Связь с `is_synthetic` (ADR-0101 §3.3.2)

`is_synthetic` уже имеет чёткую семантику в кодовой базе
(ADR-0101 §1.2 п.5: «повторный внутренний вызов того же хода»).
Повод `MEETING` (источник = устройство) **комбинируется** с
`is_synthetic=True` по следующей логике:

|| Ситуация | `is_synthetic` | `occasion` |
|---|---|---|---|
| wake_word от пользователя | `False` | `Occasion(WAKE_WORD, is_user_initiated=True)` |
| DJ-тик хочет сказать | `False` (ADR-0101 §3.3.5 «is_synthetic не нужен для DJ») | `Occasion(DJ_TICK, ...)` |
| Встреча от зрения | `True` (повторный запуск **внутренней** логики `_run_turn`, не «новый ход от пользователя») | `Occasion(MEETING, ...)` |
| Стартовая реплика | `False` (не LLM, прямая `_publish_response`) | `Occasion(STARTUP, ...)` |
| Babble/code/action-claim ретрай | `True` | `None` (это не повод, это ретрай) |
| `_drain_pending_user_messages` | `False` | `None` (это склейка, не повод) |

`is_synthetic=True` **не значит «нет пользователя»** — это ярлык «ход не
открыт пользовательским текстом» (`ADR-0101 §1.2 п.5`, диалк `dialogue_node.py:3423-3431`).
В комбинации с `occasion=Occasion(MEETING)` это означает
«_run_turn запущен по поводу встречи, без нового пользовательского
текста». **Семантика не конфликтует**, потому что:
- `is_synthetic` = «как учитывать в budget retrыев» (ADR-0101 §1.2 п.5: «ТОЛЬКО на user-initiated»).
- `occasion` = «почему ход запущен» (ADR-0101 §3.3.2).

Это два разных измерения, не одно.

### 3.6. Что делать с существующими кулдаунами (ADR-0101 §3.3.5, §3.5)

| Существующий механизм | Файл:строка | Миграция | Карточка |
|---|---|---|---|
| `_startup_greeting_fired` | `dialogue_node.py:5856-5858` | Параллельно (флаг остаётся) → PR-B → PR-C (флаг удалён) | отдельная worker-карточка |
| DJ `next_transition_at` + `POSTPONE_INTERVAL_S` | `dj_mode.py:230-270` | Параллельно (DJ продолжает работать) → PR-D | отдельная worker-карточка |
| `_last_unclear_at` (`unclear_cooldown_s`) | `stt_node.py:876-888` | → PR-E | отдельная worker-карточка |
| `_on_inactivity_check` (5с таймер) | `dialogue_node.py:6395-6404` | Резерв имени в `source_cooldowns` | отдельная worker-карточка |
| `_synthetic_retries_left` budget | `dialogue_node.py:3394-3437` | **Не мигрирует** — это per-turn budget, а не per-source cooldown | n/a |

**Никакая миграция не делается в PR-A** (ADR-0101 §8: «PR-A — каркас»).
PR-A = `core/occasion.py` + 6 unit-тестов (ADR-0101 §6.2-6.6)
+ EventDetector-reactivation (ADR-0101 §6.9) + регрессионный на
wake_word (ADR-0101 §6.1/6.7).

## 4. Диаграмма потоков данных (Mermaid)

```mermaid
flowchart LR
    subgraph "Perception Pipeline (ADR-0089 Phase 1+)"
        OE["vision_hailo_node<br/>(#2531 merged)"] -->|/vision/hailo/events<br/>VisionEvent| CA
        CA["context_aggregator_node<br/>(publish_rate=2.0 Hz)"] -->|/perception/context_update<br/>PerceptionEvent| DN
    end

    subgraph "Existing Voice Sources (без изменений)"
        ST["stt_node<br/>(wake_word gate)"] -->|/voice/stt/result| DN
        DJ["core/dj_mode.py<br/>(DJ_TICK_INTERVAL_S=5s)"] -->|/voice/dj_mode| DN
        SU["dialogue_node._on_startup_greeting<br/>(_startup_greeting_sec=12)"] -->|self-call| DN
        TG["telegram_node<br/>(TG-маркер [TG:chat_id])"] -->|/voice/stt/result| DN
    end

    subgraph "dialogue_node (B — расширение, ADR-0101 §3.3)"
        DN["dialogue_node.py<br/>(6934 строк)"]
        subgraph "Новые подписки — шов ДАННЫЕ"
            OP["_on_perception_event<br/>(НЕ строит Occasion)"]
        end
        subgraph "Новые подписки — шов РЕШЕНИЕ"
            OM["_on_meeting_signal<br/>(строит Occasion MEETING)"]
        end
        subgraph "Existing callbacks"
            OS["_on_stt<br/>(wake_word gate)"]
            OJ["_on_dj_mode<br/>(dj_tick)"]
        end
    end

    subgraph "core/occasion.py (новый модуль, PR-A)"
        OG["OccasionGate<br/>may_speak(Occasion) → Verdict"]
        ED["EventDetector<br/>(оживлён ADR-0101 §3.2)"]
    end

    subgraph "downstream"
        DT["_dispatch_turn<br/>(kw: occasion=Occasion|MEETING,<br/>is_synthetic=True)"]
        RT["_run_turn<br/>(тот же код, без изменений)"]
    end

    CA -.PerceptionEvent.vision_events_json.-> OP
    OE -. VisionEvent .-> OM
    ST -. wake_phrase .-> OS
    DJ -. dj_tick .-> OJ
    SU -. startup_done .-> OJ

    OP -->|_perception_cache.update| DN
    OS -->|has_wake_word| DT
    OJ -->|dj_tick| DT
    OM -->|may_speak Occasion MEETING| OG
    OG --> ED
    OG -->|Verdict ALLOW| DT
    OG -->|Verdict DEFER/REFUSE| ROS["/rosout<br/>(лог-уровень)"]

    DT -->|run_coroutine_threadsafe| RT

    style OG fill:#e6f3ff,stroke:#0066cc
    style ED fill:#e6f3ff,stroke:#0066cc
    style OM fill:#fff4e6,stroke:#cc6600
    style OP fill:#e6ffe6,stroke:#009900
```

**Что показывает диаграмма**:

- **Шов данных** (`_on_perception_event`) — отдельный callback, **не строит** `Occasion`, только обновляет кэш для LLM-промпта.
- **Шов решения** (`_on_meeting_signal`) — отдельный callback, **строит** `Occasion(MEETING, ...)`, зовёт `may_speak`. Это будущий PR-F (после #2442+#2531).
- **`OccasionGate`** — in-process модуль, синхронный, без RTT.
- **`EventDetector`** — per-source кулдаун (ADR-0101 §3.2), единственный продакшен-импортёр после PR-A.
- **Существующие источники** (STT wake, DJ, startup) — мигрируют постепенно (PR-B/C/D/E), без правок существующих швов `_dispatch_turn` / `_run_turn` в PR-A.

## 5. Чертёж API (полный, для реализатора PR-A)

```yaml
# Файл: src/rob_box_voice/rob_box_voice/core/occasion.py
# Где: рядом с core/dj_mode.py, core/dialogue_guards.py, core/dialogue_text.py
# Размер: ~200 строк (ADR-0101 §3.1 эскиз = 121 строка + ~80 строк тестов)

types:
  VerdictKind: enum(str) { ALLOW, DEFER, REFUSE }
  SourceKind:  enum(str) {
    WAKE_WORD,           # user-initiated; ADR-0101 §3.3.1
    DJ_TICK,             # PR-D; ADR-0101 §3.3.5
    STARTUP,             # PR-B; ADR-0101 §3.3.3
    # MEETING,           # PR-F (после #2442+#2531); резерв имени
    # INACTIVITY_ACK,    # PR-E (после решения «30с тишины → ты тут?»); резерв
  }
  Occasion: dataclass(frozen) {
    source: SourceKind
    payload: dict[str, Any]
    is_user_initiated: bool = False
  }
  Verdict: dataclass {
    kind: VerdictKind
    reason: str
    retry_after_s: float | None = None
  }

class OccasionGate:
  ctor: (
    global_debounce_s: float = 2.0,
    source_cooldowns: dict[SourceKind, float] | None = None,
    stub_event_type: str = "person",
    stub_source_cameras: frozenset[str] = frozenset({"unknown", "stub"}),
    one_shot_kinds: frozenset[SourceKind] = frozenset({SourceKind.STARTUP}),
  )

  may_speak(occasion: Occasion, now: float | None = None) -> Verdict
    # Pure-функция от (state, occasion). Тестируется без ROS2 (ADR-0101 §6.2).

  mark_consumed(occasion: Occasion, now: float | None = None) -> None
    # Вызывается после успешного _dispatch_turn. Обновляет _last_fire_at.

  stats() -> dict
    # {"last_fire_at": {...}, "last_any_at": float, "consumed_one_shot": [...]}
    # Для /rosout-логирования (ADR-0101 §3.1:255-261).

# Существующий EventDetector (ADR-0101 §3.2) — продакшен-импортёр ОДИН раз:
from rob_box_perception.core.event_detector import EventDetector
# Используется как per-source cooldown внутри may_speak (НЕ edge-detection).
```

**Что НЕ входит в API** (явно):

- `may_speak` **не имеет** параметра `topic_name` — нет ROS2-контекста.
- `Verdict` **не имеет** поля `published_to_topic` — нечего публиковать.
- `Occasion` **не сериализуется** в ROS-сообщение (нет `.msg` файла для него).
  Если когда-нибудь понадобится observability — добавим
  `rob_box_voice_msgs/msg/Occasion.msg` отдельной карточкой, не блокирует PR-A.

## 6. Альтернативы (явный trade-off)

### A. Отдельная ROS2-нода `occasion_node`

| Плюс | Минус |
|---|---|
| Изоляция `OccasionGate` (тестируется как сервис) | +1 healthcheck, +1 launch, +1 docker-compose сервис |
| Потенциальный RPS-headroom | RTT на пути callback → решение → dispatch (для RPS=0.1 лишний латенси) |
| Возможность multi-subscriber (e2e-харнесс, мониторинг) | Подписчик e2e лучше читать из `/rosout` или нового `Occasion.msg`, не из ещё одного `/voice/turn_occasion` |
| ADR-0101 §4-A отвергнут: «CQRS-сервис — лишний слой для текущего масштаба» | (см. ADR-0101 §4-A) |

### B. Расширение `dialogue_node` через in-process `core/occasion.py` — **ВЫБРАНО**

| Плюс | Минус |
|---|---|
| Минимальный diff: +1 файл `core/occasion.py`, +N строк в `dialogue_node.__init__` | `OccasionGate` живёт в `rob_box_voice`, не reusable из других нод (но других нод и нет) |
| Переиспользует существующие подписки `dialogue_node` | `dialogue_node.py` уже 6934 строк (ADR-0021 R1 — это concern, но ADR-0101 §3.1 уже выбрал именно это расположение) |
| Шов «решение» отделён от шва «данные» (ADR-0101 §3.1 п.6) | Никаких минусов на observability — `stats()` логируется |
| Юнит-тестируется без ROS2 (ADR-0101 §6.2) | |

### C. Подписка прямо в LLM-цикле (`_run_turn`)

| Плюс | Минус |
|---|---|
| Минимальный код в `dialogue_node.__init__` | Race на `self._synthetic_retries_left` (callback во время LLM-инструментального вызова) |
| Не трогаем существующие 10 вызовов `_dispatch_turn` | Шов «решение» смешивается с телом хода — нарушает ADR-0101 §3.1 п.6 |
| | `_run_turn` становится god-object (потребитель + решатель + исполнитель) |
| | Тестируемость падает (нужен mock LLM + подписка одновременно) |

## 7. Что НЕ меняется (жёсткая граница)

Это **не** входит в этот ADR. Воркеру-реализатору **запрещено** трогать
эти файлы без отдельной issue:

1. **Никаких новых ROS2-топиков** — `/voice/turn_occasion`,
   `/voice/occasion/request`, `/voice/occasion/verdict` **не создаются**
   в этом PR и его потомках (B, C, D, E, F).
2. **Никакого `Occasion.msg`** — Python-объект `Occasion` не
   сериализуется в ROS-сообщение в этом PR.
3. **`_run_turn` не трогаем** — это тело хода, не шов решения.
4. **`_dispatch_turn` — минимальный kw** (ADR-0101 §3.3.2):
   `occasion: Occasion | None = None`. Семантика `is_synthetic` не меняется.
5. **Существующие 18 подписок `dialogue_node`** остаются (ADR-0101 §1.2 п.5).
   PR-A добавляет **ровно одну новую подписку** (`_on_perception_event`
   на `/perception/context_update`) для шва данных и **ровно одну**
   для шва решения (`_on_meeting_signal` на `/vision/hailo/events`),
   но **последняя остаётся NO-OP** до PR-F.
6. **`dialogue_node.py:5842-5894` (`_on_startup_greeting*`)** не мигрирует
   в PR-A (ADR-0101 §3.5 п.4: «`_startup_greeting_fired` остаётся как
   fallback параллельно, миграция = PR-B»).
7. **Живой `meeting` источник** не подключается до #2442 + #2531 +
   (явное наличие `is_real: bool` или другого маркера в `VisionEvent.payload`).
8. **`mcp_server.py:706` (`/voice/stt/request` без подписчика)** —
   отдельная worker-карточка (ADR-0101 §5 + `seam_baseline.json:17`).
9. **`startup_greeting_node.py` (осиротевший)** — отдельная worker-карточка
   (ADR-0101 §5 + `seam_baseline.json:17`).

## 8. План реализации (для воркера backend)

**PR-A — каркас** (минимальный, единственный обязательный для #2536):

1. Новый файл `src/rob_box_voice/rob_box_voice/core/occasion.py` по
   чертёжу §3.3 + §5.
2. Юнит-тесты `src/rob_box_voice/test/unit/core/test_occasion.py`:
   `test_stub_filter_refuses`, `test_global_debounce`,
   `test_user_initiated_bypasses_cooldown`,
   `test_startup_one_shot`, `test_per_source_cooldown`,
   `test_wake_word_bypass` (ADR-0101 §6.2-6.6).
3. Регрессионный набор `_on_stt` / `has_wake_word` остаётся зелёным
   (ADR-0101 §6.1/6.7).
4. `EventDetector` — единственный новый продакшен-импортёр
   (`core/occasion.py`), ADR-0101 §6.9.
5. **`dialogue_node.py`** — **только** `__init__` (параметры
   `_occasion_global_debounce_s`, `_occasion_cooldowns`) +
   `test_occasion_dispatch.py` (ADR-0101 §6.8).
6. **Никаких новых подписок** в `dialogue_node.__init__` в PR-A.

**PR-B…F** — отдельные worker-карточки (ADR-0101 §8), этот ADR их не
блокирует.

## 9. Acceptance (измеримо, без «вроде работает»)

После реализации **обязательны** все пункты ниже. Каждый — атомарный,
проверяемый, с конкретным артефактом.

### 9.1. Сериализация типа `Occasion` НЕ сериализуется

- **Что**: в коде нет ни одного `.msg` файла с именем `TurnOccasion`
  или `Occasion`; нет ни одного вызова `serialize_message(...)` /
  `deserialize_message(...)` для `Occasion`.
- **Артефакт**: `git grep -rn "TurnOccasion\|Occasion" src/rob_box_*_msgs/`
  → 0 совпадений.
- **Команда**:
  ```bash
  cd /workspace
  git grep -rn "TurnOccasion\|class Occasion" src/rob_box_*_msgs/ || echo "OK: no msg"
  ```
- **Критерий PASS**: 0 совпадений в `*_msgs/` пакетах.

### 9.2. Никаких новых топиков `/voice/turn_occasion`, `/voice/occasion/*`

- **Что**: PR-A не вводит новых ROS2-топиков с именами `/voice/turn_occasion`,
  `/voice/occasion/request`, `/voice/occasion/verdict`.
- **Артефакт**: `git grep -rn "/voice/turn_occasion\|/voice/occasion" src/`
  → 0 совпадений в PR-A diff.
- **Команда**:
  ```bash
  cd /workspace
  git diff origin/develop..HEAD -- src/ | grep -E "/voice/(turn_occasion|occasion)" \
    && echo "FAIL" || echo "PASS"
  ```
- **Критерий PASS**: «PASS» (diff пуст по этим именам).

### 9.3. In-process API работает

- **Что**: `OccasionGate.may_speak(Occasion)` синхронно возвращает
  `Verdict` без обращения к ROS2.
- **Артефакт**: `pytest src/rob_box_voice/test/unit/core/test_occasion.py -v`.
- **Команда**:
  ```bash
  cd /workspace/src/rob_box_voice
  python -m pytest test/unit/core/test_occasion.py -v 2>&1 | tee /tmp/occasion_test.txt
  ```
- **Критерий PASS**: ≥6 test-cases PASS (stub_filter, global_debounce,
  per_source_cooldown, startup_one_shot, user_initiated_bypass,
  wake_word_observation), 0 failed.

### 9.4. Регрессия на вейк-слово (из ADR-0101 §6.1/6.7, дословно)

- **Что**: поведение `_on_stt` / `has_wake_word` / `_dispatch_turn`
  (обычный ход, TG-маркер, no_wake_word → backlog/counter) идентично
  до и после PR-A.
- **Артефакт**: `git diff origin/develop..HEAD -- 'src/rob_box_voice/test/unit/node/*'`
  → только новые файлы, `expected_values` неизменны.
- **Команда**:
  ```bash
  cd /workspace/src/rob_box_voice
  python -m pytest test/unit/node/ -v 2>&1 | tee /tmp/wake_regression.txt
  ```
- **Критерий PASS**: 0 failed, 0 errored; **deliberate identical output**
  до и после.

### 9.5. `EventDetector` оживлён (из ADR-0101 §6.9, дословно)

- **Что**: `EventDetector` имеет нового продакшен-импортёра.
- **Артефакт**: `git grep "from rob_box_perception.core.event_detector import EventDetector" -- 'src/'`
  → **ровно 2 совпадения**: тест + `core/occasion.py`.
- **Команда**:
  ```bash
  cd /workspace
  git grep -c "from rob_box_perception.core.event_detector import EventDetector" -- 'src/' | sort
  ```
- **Критерий PASS**: ровно 2 совпадения; ни одного другого прода.

### 9.6. e2e-маркер (после PR-A + PR-F, не блокирует PR-A)

Этот ADR — **каркас**. Живой e2e-тест с реальным роботом — отдельная
e2e-карточка (t_7f1a4919, tester) после того, как:

- Повод стабилизируется в unit-тестах (9.3-9.5 PASS);
- Шов `meeting` получит хотя бы фиктивный вызыватель (тестовый адаптер,
  PR-F);
- #2532/#2442 закроются, маркер «это не стаб» появится в `VisionEvent.payload`.

## 10. Метрики (baseline → target)

| Метрика | Baseline | Target (после PR-A) | Способ проверки |
|---|---|---|---|
| ROS2-топиков с именем `/voice/turn_occasion*` или `/voice/occasion*` | 0 | **0** (не вводятся) | `git grep` |
| Файлов в `dialogue_node.py` | 6934 строк | **+≤30 строк** в `__init__` (только параметры `OccasionGate`) | `wc -l` |
| Продакшен-импортёров `EventDetector` | 0 | 1 (`OccasionGate`) | `git grep` |
| Юнит-тестов на Повод | 0 | ≥6 (ADR-0101 §6.2-6.6 + dispatch) | `pytest --collect-only -q` |
| Швов «подписка → решение» в `dialogue_node.py` | 4 размазанных | 1 (`_on_meeting_signal` строит `Occasion`, `may_speak` решает; остальные callback-и — данные) | code review |
| Новых ROS2-нод | 0 | 0 (in-process) | `git diff --name-only` |

## 11. Связанные ADR и принципы

- **ADR-0101** (родитель) — фиксирует API `core/occasion.py` как
  in-process модуль. Этот ADR (0102) **расширяет** его до полного
  type-чертежа + явный verdict по вариантам A/B/C.
- **ADR-0065** — wake-words SSoT в коде. Повод **не** трогает
  wake-слова; байт-в-байт эквивалентная семантика `_on_stt`
  (ADR-0101 §3.3.1).
- **ADR-0066** — `dialogue/control pause/resume` (другая тема, **номер
  занят**). Этот ADR — **0102**, не «0066».
- **ADR-0021 (R2 State SSoT)** — каждый счётчик/словарь/enum имеет
  один источник правды. `OccasionGate` консолидирует 4 разрозненных
  кулдауна в один шов.
- **ADR-0013 (Incremental delivery)** — PR-A минимальный, миграции
  (PR-B…F) — отдельными маленькими PR.
- **ADR-0018 (Capability-honest)** — стаб-фильтр обязателен, пока
  источник не отличим от заглушки.
- **ADR-0029, ADR-0070 (Proposed)** — cold-start race VAD и STT-теряет-wake.
  Не взаимодействуют с Поводом (новый повод минует STT/VAD целиком).
- **ADR-0089 (AI HAT+ deployment)** — Phase 1.5 real inference (#2398) +
  PerceptionEvent.vision_events_json. Этот ADR использует эти данные
  для шва «данные», но **не** для решения.

## 12. Решение

**Принять** ADR-0102 со следующими условиями:

1. **Никакого нового ROS2-топика повода.** `/voice/turn_occasion`
   (имя из брифа) **не используется** — обоснование §3.2.
2. **API — in-process Python-объект** `Occasion` / `Verdict` /
   `SourceKind` по чертёжу §3.3 + §5.
3. **Вариант B (расширение dialogue_node через in-process
   `core/occasion.py`) — выбран.** A (отдельная ROS2-нода) и C
   (подписка в LLM-цикле) — отвергнуты с обоснованием §3.1 + §6.
4. **Шов «решение» отделён от шва «данные»** (ADR-0101 §3.1 п.6):
   `_on_perception_event` обновляет кэш и НЕ строит `Occasion`;
   `_on_meeting_signal` строит `Occasion` и зовёт `may_speak`.
5. **`is_synthetic=True` + `occasion=Occasion(MEETING)`** —
   валидная комбинация (семантика не конфликтует, §3.5).
6. **PR-A остаётся каркасом** (ADR-0101 §8) — никакой миграции
   существующих кулдаунов, никаких новых ROS2-нод, никакого
   `TurnOccasion.msg`.
7. **Номер ADR-0066 — занят** (commit `4f201b487`, PR #2121,
   тема `dialogue/control pause/resume`). Дочерняя карточка
   t_9d3b4421 (techwriter) **DEPRECATED** — её тело описывает
   этот же ADR-0102, а не новый ADR-0066. Архитектор снимает
   задачу с techwriter и закрывает дочку через `kanban_request_changes`
   с указанием на этот ADR.

**Статус**: Proposed (ждёт accept от товарища Шифу перед началом
реализации PR-A).

---

## Приложение А. История изменений

| Дата | Событие | Автор |
|---|---|---|
| 2026-09-15 | ADR-0101 смержен через PR #2575 (merge_sha `e7c9abc6`) | Denis |
| 2026-09-15 | Дочерние карточки t_38c55959 / t_7f1a4919 / t_9d3b4421 / t_a98a25e1 созданы auto-decomposer после ADR-0101 | agent-flow-triage |
| 2026-09-15 | ADR-0102 создан в ответ на t_38c55959 (бриф просил «API топика» — уточнено как in-process Python-объект) | architect worker (t_38c55959) |