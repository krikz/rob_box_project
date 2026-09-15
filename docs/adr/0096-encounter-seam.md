# ADR-0096: «Встреча» — единый шов присутствия, объединяющий голосовой и зрительный каналы

| Поле | Значение |
|---|---|
| Статус | **Partially superseded** — §2.5, §9.3, §9.4 заменены ADR-0105 (`0105-encounter-in-process-first.md`). Остальное (структура значения, §6 edge-кейсы, §7 lifecycle, §8 тест-план, stub-фильтр) — в силе. |
| Заменено | `docs/adr/0105-encounter-in-process-first.md` — в части «отдельная нода + EncounterState.msg + список встреч + `кто: str`». Реализация #2442 (PR #2593, `19bfeb00`) пошла in-process, с одной текущей Встречей и типизированным `who: Acquaintance`; причина расхождения — коллизия номеров ADR (issue #2582), документ при реализации не был найден. §10 Q-3 (config-fix топика камеры) закрыт PR #2578. |
| Дата | 2026-09-14 |
| Автор | architect (Hermes Agent), kanban `t_13fc8f29` |
| Контекст | Сценарий-заказчик: «Денис входит в мастерскую → робот замечает событие → поднимает биометрию → понимает, кто это → знает, что давно не виделись → заговаривает первым». Issue #2442 закрывает четвёртый шаг («робот сейчас находится рядом именно с этим человеком») — отдельный, сегодня не существующий как значение, утверждение о присутствии. Голосовой и зрительный каналы сегодня живут независимо (см. §1.1) — диалог знает про `_current_speaker` из `/voice/speaker/result`, perception-context знает про `vision_events_json`, но **никто** не сливает их в одно значение «кто присутствует прямо сейчас, с какой уверенностью, каким каналом подтверждено». Шов «Встреча» (по аналогии с ADR-0093 для transient ring, ADR-0094/0095 как шаблоны) даёт единый read-only интерфейс `Encounter.current()` → value-объект `{кто, уверенность, каналы, с_какого_момента, виделись_до_этого}`. Шов использует `Знакомый.id` из #2440 как поле «кто», но **не** переоткрывает identity-логику — это разные оси (identity = «кто» vs encounter = «встретились ли прямо сейчас»), разные уровни шва, разные карточки. |
| Затрагивает | (a) новый модуль `src/rob_box_harness/rob_box_harness/encounter/` (интерфейс + dataclass `Encounter`); (b) голосовой адаптер `/voice/speaker/result` → encounter state; (c) зрительный адаптер `/vision/hailo/events` → encounter state; (d) удаление дублирующего state (`dialogue_node._current_speaker`, `mcp_server.current_speaker_id`, fallback в `memory.py:88-91,176-178,290-292`); (e) конфигурационный fix `input_topic` → реальный топик камеры (`/camera/camera/color/image_raw`); (f) очистка/огранение буфера `_hailo_events` в `context_aggregator_node.py:299-305`; (g) новая политика `vision_events_json`/`vision_event_count` (явное deprecated-or-removed); (h) acceptance-тесты. |
| Родители | ADR-0018 (честный FAIL), ADR-0013 (incremental delivery), ADR-0080 (eight seams), ADR-0089 (AI HAT+ deployment — Phase 1.5 consumer-side), ADR-0091 (MiniMax STT), ADR-0092 (pregenerate contract), ADR-0093 (unknown-speaker ring), ADR-0094 (image-versions workflow), ADR-0095 (PR pollution detection) |
| Связанные | issue #2442 (этот issue), issue #2440 («Знакомый» — шов identity, upstream-контракт для поля «кто»), issue #2359 (VisionEvent контракт полей — **не трогаем** этот issue), issue #2398 (RealHEFLoader, Phase 1.5 producer-side уже merged), issue #2348 (пороги speaker-id), `src/rob_box_voice/rob_box_voice/dialogue_node.py:473,1957-1978,3072-3152`, `src/rob_box_voice/rob_box_voice/speaker_id_node.py:19-21`, `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py:341,485-528,1313-1331`, `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/memory.py:88-91,176-178,290-292`, `src/rob_box_perception/rob_box_perception/context_aggregator_node.py:299-305,507-535`, `src/rob_box_perception/rob_box_perception/vision_hailo_node.py:112-126,257`, `src/rob_box_perception/rob_box_perception/vision_hailo_loader.py:119-130,501-512`, `docker/vision/config/hailo_models.yaml:37-38`, `docker/vision/config/oak-d/oak_d_config.yaml:28-62`, `docs/adr/0089-ai-hat-plus-deployment.md` §9 (Phase 1.5 consumer-side) |

> **TL;DR.** Между двумя независимыми потоками восприятия (голос → `/voice/speaker/result`, зрение → `/vision/hailo/events`) и тремя независимыми потребителями (`dialogue_node._current_speaker`, `mcp_server.current_speaker_id`, три fallback в `memory.py`) — модуль `Encounter` в `rob_box_harness/encounter/` с интерфейсом `current() -> Encounter | None`. `Encounter` — value-объект с полями `кто: Знакомый.id | None`, `уверенность: float`, `каналы: set[Literal["голос","лицо"]]`, `с_какого_момента: float`, `виделись_до_этого: float | None`. Два адаптера (voice/face) **пишут** в encounter state; три потребителя **читают** `current()`. `_current_speaker`, `current_speaker_id`, три fallback'а и XML `<user_profile>` исчезают. **Шов сам не публикует в `mcp_server.on_perception_update` `vision_events_json` целиком** — `mcp_server` вместо этого читает `Encounter.current()` и сам решает, что положить в `perception_context_tool.update_context`. Голосовой канал — первичный (он работает сегодня); зрительный канал подключается **после** конфигурационного fix (топик + транспорт), иначе `vision_hailo_node` физически не получает кадров. Edge-кейсы (нет никого / только голос / только лицо / оба расходятся / stub / cold start / рассинхрон каналов) — таблица §6. Тест-план — §8. **Допущение к реализации**: голосовой адаптер может стартовать сразу (он работает сегодня), зрительный — после отдельного config-fix карточки (см. §10 Open questions).

---

## 0. Что внутри и что — нет

**Внутри.** Дизайн структуры `Encounter` (§2.1) и шовного интерфейса `EncounterRegistry.current()` / `update_signal()` (§2.2). Два адаптера: voice (§2.3) и face (§2.4). Три миграции потребителей: `dialogue_node._current_speaker` → `Encounter.current()`, `mcp_server.current_speaker_id` → `Encounter.current()`, три fallback в `memory.py` → `Encounter.current().кто`. Конфигурационный fix `input_topic` (`/oak/rgb/image_raw/compressed` → `/camera/camera/color/image_raw`) как **отдельная config-only карточка** (см. §10 Open Q-3). Политика `vision_events_json`/`vision_event_count` — explicit deprecation с планом удаления после периода наблюдения (§5.4). Очистка `_hailo_events` буфера в `publish_event` (§5.2). Stub-filter (face-адаптер обязан игнорировать `source_camera == 'stub'`). 8 edge-кейсов (§6). Lifecycle (§7). Тест-план (8 acceptance-критериев, §8). 5 отвергнутых альтернатив (§9). 4 открытых вопроса для товарища Шифу (§10).

**Не внутри.** Реализация модуля `Encounter` (отдельная карточка после ревью этого ADR). Реализация `face_adapter` для stub-фильтра (отдельная карточка внутри Vision Pi владельца, потому что требует переделки `vision_hailo_loader.py`). Phase 2 (face recognition с эмбеддингами и БД `/data/faces.db`, см. ADR-0089) — это downstream, **после** этого ADR и после #2440. Изменение `speaker_id_node.yaml` thresholds (issue #2348, уже отдельный трек). Изменение `dialogue_node._build_dynamic_system_context` формата XML — только если `Encounter` диктует другую схему (предварительно — нет, см. §2.1.3). Полный перенос `vision_events_json` в новый low-level debug-канал с явной пометкой (deprecation после периода наблюдения, см. §5.4). ADR-0093 ring-буфер (sibling — встречи с **неизвестными**, отдельный шов).

---

## 1. Контекст и бизнес-проблема

### 1.1 Текущая схема (что есть, проверено 2026-09-14)

Три независимых «знания о том, кто рядом» живут в трёх процессах, написанных разными авторами, в разное время, для разных целей:

| Место хранения | Что хранит | Кто пишет | Кто читает |
|---|---|---|---|
| `dialogue_node._current_speaker` (`dialogue_node.py:473,1962,2162-2163,2807-2808,3072-3073,6226-6227`) | Словарь от `/voice/speaker/result` целиком (`is_known`, `speaker_id`, `name`, `confidence`, ...), под `threading.Lock`. | `dialogue_node._on_speaker_result` (`:1957-1978`) | `_build_dynamic_system_context` рендерит `<user_profile>` (`:3114-3138`) и одна диагностика (`:2162-2163,2807-2808`) |
| `mcp_server.current_speaker_id` (`mcp_server.py:341,485-528`) | Только `str(speaker_id)` (полный UUID). | `mcp_server._on_speaker_result` (`:485`) от того же `/voice/speaker/result`, но в **другом процессе** | Три `memory.py` tools (`:88-91,176-178,290-292`) с одинаковым паттерном `kwargs.get("speaker_id") or getattr(self.node, "current_speaker_id", None)` |
| `context_aggregator_node._hailo_events` (`context_aggregator_node.py:299-305,507-535`) | Список dict'ов от `/vision/hailo/events` (`VisionEvent` → dict), сериализуется в `PerceptionEvent.vision_events_json` + `vision_event_count` | `on_hailo_vision_event` (`:299-305`) | `publish_event` (`:507-535`) кладёт в `PerceptionEvent`, но `mcp_server.on_perception_update` (`mcp_server.py:1313-1331`) **не читает** эти поля — `grep -n 'vision_events_json\|vision_event_count' mcp_server.py` даёт ноль совпадений |

**Физически:** голосовой и зрительный каналы не встречаются **нигде** в коде (verified: `grep -rn 'speaker.*vision\|vision.*speaker' src/ --include='*.py' | grep -v test` — совпадений вне `ADR-0089` плана нет). `dialogue_node` ничего не знает про vision; `mcp_server` ничего не знает про voice (кроме `current_speaker_id`); `context_aggregator` не публикует в `/voice/*` и не подписан на `/voice/speaker/result`.

### 1.2 Что ломается

| Сценарий | Сегодня | С `Encounter` |
|---|---|---|
| Денис входит, молчит 5 сек | Робот видит человека в кадре (если бы кадр дошёл — но он сейчас не доходит, см. §5.1), но voice ничего не публикует → LLM не знает | `Encounter.current().каналы == {лицо}`, `кто = None`, `уверенность = 0.8`. LLM получает контекст «рядом кто-то один, канал=лицо, голоса ещё нет». |
| Денис говорит «Привет» | Голос распознан, LLM получает `<user_profile name="Денис" speaker_id="abc12345" confidence="0.92"/>` (`:3136`) | `Encounter.current().кто = Знакомый.id="abc12345-..."`, `каналы == {голос}`. После распознавания лица (если доступно): `каналы == {голос, лицо}`, `уверенность = max(0.92, 0.85) = 0.92`. LLM получает то же, но без усечения `[:8]` и с явным каналом-флагом. |
| Денис говорит, в кадре — другой человек | Voice → Денис. Vision → кто-то другой. **Нет шва, который сказал бы «кто-то ещё пришёл, но молчит»** → LLM ответит только про Дениса, второй человек потерян | `Encounter.current()` возвращает **одну** Встречу (того, кто заговорил — Дениса). «Кто-то ещё в кадре» — отдельный signal, **out of scope** этого ADR (см. §10 Open Q-2). |
| Stub-режим (`hailo_enabled=false`) | Vision шлёт fake «person» каждые 2 сек (`:257 publish_when_no_input=True`). Если бы mcp_server читал `vision_events_json` — LLM бы видел синтетических людей. | Face-адаптер фильтрует `source_camera == 'stub'` (см. §2.4) — `Encounter.current()` в stub не публикует канал-лицо. Privacy-stoop держится даже при подключении consumer-side. |
| Vision буфер «замерзает» при остановке потока | `:299-305` — очистка только внутри колбэка. Если топик замолк, `_hailo_events` хранит последний набор **навсегда**; `publish_event` продолжает переотправлять те же события без метки «протухло» | Очистка перенесена в `publish_event` (см. §5.2), плюс `maxlen` guard на `_hailo_events` |
| Vision пакеты приходят burst'ом (5 FPS реально сейчас, до 100+ при `hailo_enabled=true`) | `_hailo_events = List[Dict]` без `maxlen`, при 5 FPS × memory_window=60s × N detections/frame = потенциально сотни-тысячи dict'ов в JSON на каждом тике | `maxlen = publish_rate_hz × memory_window × expected_max_detections_per_frame` с явным cap, см. §5.2 |

### 1.3 Что НЕ нужно делать

- **Не нужно объединять `/voice/speaker/result` и `/vision/hailo/events` в один топик.** Они несут разную семантику (event-driven utterance vs continuous frame stream) и разный lifecycle. Шов Encounter — **на стороне потребителя**, не транспорта.
- **Не нужно отказываться от `perception_context_tool.update_context`.** Это рабочий канал для `mcp_server.on_perception_update`. Просто `mcp_server` теперь читает `Encounter.current()` и кладёт его поля в `update_context`, а не выковыривает `vision_events_json` (который никогда не читался).
- **Не нужно менять контракт `VisionEvent.msg`.** Issue #2359 — отдельный трек про форму сообщения. Этот issue про то, что **после** публикации сообщения (см. issue #2442 «Разграничение с #2359»).
- **Не нужно трогать ADR-0093 (unknown-speaker ring).** Ring — слой для транзиентной идентификации **неизвестных** (Голос-1, Голос-2). Encounter — слой для **присутствия** с любым `кто` (None / ring-transient-label / `Знакомый.id`). Ring живёт в voice-адаптере как opaque source of `transient_label`; Encounter читает его через тот же сигнал, что и `/voice/speaker/result`. Швы не дублируют — Encounter **потребляет** ring'овый `transient_label` как `кто` при отсутствии `Знакомый.id`.
- **Не нужно включать config-fix `input_topic` в PR этого ADR.** Это **отдельная config-only карточка** (§5.1, §10 Open Q-3) — конфигурационная правка с высокой вероятностью регрессии для OAK-D driver, требует отдельного e2e-теста. Без неё face-адаптер не получает кадров, **но** voice-адаптер уже работает, и `Encounter.current()` корректно возвращает Встречу только с голосовым каналом. Этот ADR **специфицирует интерфейс** Encounter и адаптеров, а config-fix — отдельный шаг.
- **Не нужно делать Phase 2 (face embedding-recognition).** Face-адаптер в этом ADR пишет `event_type='face'` и `embedding_id=''` для **детекции** лица (visual presence, без identification). Embedding-recognition (ArcFace + `/data/faces.db`) — ADR-0089 Phase 2, отдельный трек, потребитель того же шва через `Знакомый.id`.

---

## 2. Решение

### 2.1 Структура `Encounter` (value-объект)

Расположение: `src/rob_box_harness/rob_box_harness/encounter/encounter.py` (рядом с `core/`, `memory/` — там же, где живут value-объекты harness'а).

```python
# src/rob_box_harness/rob_box_harness/encounter/encounter.py

from dataclasses import dataclass, field
from typing import Literal, Optional, Set

Channel = Literal["голос", "лицо"]


@dataclass(frozen=True, slots=True)
class Encounter:
    """Одна встреча — один человек рядом с роботом прямо сейчас.

    Несколько людей одновременно = несколько Encounter'ов в registry
    (см. §2.2 list API). Здесь — один Encounter, immutable.
    """

    кто: Optional[str]                         # Знакомый.id из #2440 (полный UUID),
                                              # или ring.transient_label ("Голос-N")
                                              # из ADR-0093 при отсутствии identity,
                                              # или None (присутствие без опознания).
    уверенность: float                         # max по всем каналам, в [0, 1].
    каналы: Set[Channel]                       # {"голос"}, {"лицо"}, или {"голос", "лицо"}.
                                              # Пустое множество — Встреча не активна.
    с_какого_момента: float                    # time.monotonic() начала непрерывного
                                              # присутствия. Для склейки (merge) —
                                              # максимум по двум каналам.
    видели_до_этого: Optional[float]           # секунды с момента предыдущей Встречи
                                              # с тем же `кто`, или None (никогда).

    def __post_init__(self) -> None:
        if not (0.0 <= self.уверенность <= 1.0):
            raise ValueError(f"уверенность вне [0,1]: {self.уверенность}")
        # frozen=True запрещает мутации; frozenset в каналы для hashability.
```

**Почему frozen + slots**: Encounter прокидывается через asyncio, под `threading.Lock`, через LLM-prompt (как context string) — нужны hashability для кеша и защита от случайной мутации в consumer'ах.

**Почему `кто: Optional[str]`, а не объект `Знакомый`**: Encounter — пакет в `rob_box_harness`, не имеет права зависеть от `rob_box_voice` или будущего `rob_box_identity` напрямую (иначе нарушаем ADR-0080 — порт шов, не shared domain type). Идентичность передаётся **строкой** (UUID или ring-transient-label). Связывание с объектом `Знакомый` — задача потребителя через `Знакомый.resolve(id)` (по ADR-0093/0094 паттерну: id-typed edge, объект-разрешение на стороне owner'а).

#### 2.1.1 Confidence fusion (как считается `уверенность`)

**Решение: `max(c_голос, c_лицо)`** — максимум, не среднее, не geometric mean. Обоснование:

| Канал | Что значит «0.9» | Что значит «0.5» |
|---|---|---|
| Голос | d-vector cosine к known `speaker_id` выше `identify_threshold=0.72` (ADR-0089 + #2348) | ниже threshold — unknown |
| Лицо | bbox detection confidence выше NMS IoU + class threshold | bbox есть, но низкая уверенность |

`max` отражает интуицию: «если хоть один канал уверен, встреча реальна». `mean` занижает; `geometric mean` ещё сильнее. Потребитель (LLM prompt) видит одно число и одно множество каналов — этого достаточно, чтобы решить «обращаться по имени» (оба канала + уверенность ≥ 0.85) vs «поприветствовать незнакомца» (лицо есть, голоса нет, `кто=None`).

#### 2.1.2 Слияние двух каналов с разным `кто` (collision)

Если voice говорит `кто=A` а face говорит `кто=B` в один и тот же момент — это **два разных Encounter'а**, не один. Registry хранит список (см. §2.2). LLM получает список всех активных Встреч, не одну. Это явное поведение, не edge-case — иначе теряется информация «в мастерской сейчас двое».

#### 2.1.3 Сериализация для LLM prompt

Текущий формат `dialogue_node._build_dynamic_system_context` (`:3114-3138`):
```xml
<user_profile>
  <name>Денис</name>
  <speaker_id>abc12345</speaker_id>          # усечён до 8 символов
  <epithet>мастер</epithet>
  <confidence>0.92</confidence>
</user_profile>
```

После Encounter (предложение):
```
[ACTIVE ENCOUNTERS]
- кто=abc12345-... (или "Голос-1" ring-transient), каналы={голос, лицо}, уверенность=0.92, с_какого_момента=14:32:05, видели_до_этого=3 дня назад
- кто=None, каналы={лицо}, уверенность=0.71, с_какого_момента=14:32:09
```

Усечение `[:8]` убирается (issue #2440 «Дефект B» — там же корень, не здесь, но миграция на `Encounter` логически чинит обе проблемы сразу). XML-формат заменяется на markdown-список — проще для LLM (он читает markdown роднее), проще для тестов (assert по подстроке, не XPath). Имя/эпитет резолвятся в LLM-prompt через `Знакомый.resolve(кто)` если нужно (out of scope для этого ADR — Encounter только говорит «вот id», разрешение имени — ответственность prompt-сборки, см. ADR-0093 §1.2 «эпитет-словарь»).

### 2.2 Шовный интерфейс: `EncounterRegistry`

```python
# src/rob_box_harness/rob_box_harness/encounter/registry.py

import threading
import time
from typing import List, Optional

from .encounter import Encounter


class EncounterRegistry:
    """Глобальный singleton-реестр активных Встреч.

    Voice и face адаптеры пишут через update_signal().
    Потребители (dialogue_node, mcp_server, MCP tools) читают через current().
    """

    def __init__(self, *, merge_window_sec: float = 1.0) -> None:
        self._lock = threading.Lock()
        self._encounters: List[Encounter] = []
        # merge_window_sec — окно, в пределах которого voice+face с одним `кто`
        # сливаются в один Encounter (а не два). Подробнее §6 edge-case 5.
        self._merge_window_sec = merge_window_sec

    def current(self) -> List[Encounter]:
        """Snapshot активных Встреч. Lock на чтение — atomic copy."""
        with self._lock:
            return list(self._encounters)

    def update_signal(
        self,
        *,
        кто: Optional[str],
        уверенность: float,
        канал: Literal["голос", "лицо"],
        now: Optional[float] = None,
    ) -> None:
        """Адаптер вызывает при новом сигнале от своего канала.

        Логика:
          1. Найти существующий Encounter с тем же `кто` (если None — отдельный пул).
          2. Если найден и с_какого_момента в пределах merge_window → merge:
             - добавить канал в множество
             - уверенность = max(текущая, новая)
             - с_какого_момента = min(с_какого_момента_обоих)
          3. Если не найден → создать новый Encounter.
          4. Если канал = "лицо" и signal.source == "stub" → игнорировать (privacy).
        """
        ...

    def evict_expired(self, *, ttl_sec: float, now: Optional[float] = None) -> int:
        """Вызывается периодически (например из publish_event). Удаляет Встречи,
        которые не подтверждались ни одним каналом дольше ttl_sec.

        TTL по умолчанию — параметр registry (см. §7 Lifecycle).
        Возвращает количество удалённых.
        """
        ...
```

**Singleton pattern**: `EncounterRegistry()` инстанцируется **один раз** в `rob_box_harness.setup()` (или lazy-import при первом обращении). Несколько процессов (`dialogue_node`, `mcp_server`) подписываются через ROS 2 service/topic — **НЕТ**, это создаст distributed state с гонками. Решение: один процесс-владелец (`encounter_node` новый, см. §2.5) держит singleton in-memory и публикует `/encounter/state` (новый msg); остальные читают через subscription. Подробнее §3.

### 2.3 Голосовой адаптер

Расположение: `src/rob_box_harness/rob_box_harness/encounter/voice_adapter.py`.

```python
def on_speaker_result(
    registry: EncounterRegistry,
    msg: dict,                       # payload /voice/speaker/result
    *,
    identity_resolver: Optional[Callable[[str], Optional[str]]] = None,
) -> None:
    """Голосовой сигнал → EncounterRegistry.update_signal.

    Args:
        registry: куда писать.
        msg: dict с полями is_known, speaker_id, name, confidence, transient_label,
            ring_size (ADR-0093 расширения).
        identity_resolver: функция (raw_speaker_id) -> Знакомый.id | None.
            По умолчанию None — используется raw_speaker_id или transient_label.
            После мержа #2440 передаётся `Знакомый.resolve`.
    """
    if not msg.get("is_known") and not msg.get("transient_label"):
        # Никто не говорит — Encounter может истечь по TTL, evict_expired
        # разберётся. Здесь не трогаем registry.
        return

    raw_id = msg.get("speaker_id") or msg.get("transient_label") or ""
    кто = identity_resolver(raw_id) if identity_resolver else raw_id or None
    уверенность = float(msg.get("confidence", 0.0))

    registry.update_signal(
        кто=кто,
        уверенность=уверенность,
        канал="голос",
    )
```

**Кто вызывает**: новый `encounter_node` (см. §2.5) подписан на `/voice/speaker/result` параллельно с `dialogue_node._on_speaker_result` и `mcp_server._on_speaker_result` в **переходный период**. После миграции всех потребителей (§4) старые подписки удаляются.

**identity_resolver** в этом ADR не реализуется (это контракт из #2440). До #2440 — `identity_resolver=None`, `кто` = `raw_speaker_id` или `transient_label`. После #2440 — `identity_resolver = lambda raw: Знакомый.resolve(raw)`.

### 2.4 Зрительный (face) адаптер

Расположение: `src/rob_box_harness/rob_box_harness/encounter/face_adapter.py`.

```python
def on_vision_event(
    registry: EncounterRegistry,
    msg: dict,                       # payload VisionEvent (или его dict-нормализация)
    *,
    identity_resolver: Optional[Callable[[str], Optional[str]]] = None,
) -> None:
    """Зрительный сигнал → EncounterRegistry.update_signal.

    Только event_type == "face" влияет на Encounter.
    "person" / "object" / "scene" — игнорируются (см. §5.3).
    source_camera == "stub" — игнорируются (privacy-stoop, см. ADR-0089 §2.2).
    """
    if msg.get("event_type") != "face":
        return
    if msg.get("source_camera") == "stub":
        return  # privacy: stub-events НЕ подтверждают присутствие

    raw_embedding_id = msg.get("embedding_id") or ""
    кто = identity_resolver(raw_embedding_id) if identity_resolver else (raw_embedding_id or None)
    уверенность = float(msg.get("confidence", 0.0))

    if уверенность <= 0.0:
        return  # детекция без уверенности — шум

    registry.update_signal(
        кто=кто,
        уверенность=уверенность,
        канал="лицо",
    )
```

**Stub-filter** — обязательный, не опциональный. ADR-0089 §2.2 (Phase 1.5 «Privacy-stoop») явно требует этого для mcp_server; мы требуем то же на уровне адаптера — defence-in-depth. Если stub когда-нибудь начнёт публиковать `event_type='face'` (для тестов), адаптер его игнорирует. Это инвариант, не настройка.

**event_type='face' в реальном коде сегодня не публикуется ни разу** (verified: `grep -rn "event_type.*=.*['\"]face['\"]" src/ --include='*.py'` — ноль совпадений вне VisionEvent.msg декларации). Поэтому до отдельной карточки **«Vision Pi: face detection pipeline»** (внутри Vision Pi владельца) face-адаптер Encounter **молча игнорирует всё** — это корректное поведение: `Encounter.current()` возвращает только voice-канал.

### 2.5 Encounter node (точка интеграции с ROS 2)

Расположение: `src/rob_box_perception/rob_box_perception/encounter_node.py` (рядом с `context_aggregator_node.py`, потому что оба работают с `VisionEvent` и публикуют в perception-пространство).

```python
class EncounterNode(Node):
    """Подписан на /voice/speaker/result + /vision/hailo/events.
    Держит EncounterRegistry in-memory.
    Публикует /encounter/state (новый msg EncounterState).
    """

    def __init__(self) -> None:
        super().__init__("encounter")
        self._registry = EncounterRegistry(merge_window_sec=1.0)
        self._id_resolver = None  # wire после #2440 (Знакомый.resolve)

        self.create_subscription(String, "/voice/speaker/result",
            lambda msg: on_speaker_result(self._registry, json.loads(msg.data),
                                          identity_resolver=self._id_resolver), 10)

        self.create_subscription(VisionEvent, "/vision/hailo/events",
            lambda msg: on_vision_event(self._registry, vision_event_to_dict(msg),
                                        identity_resolver=self._id_resolver), 10)

        # Публикация /encounter/state — раз в publish_rate_hz (default 2.0).
        self._state_pub = self.create_publisher(EncounterState, "/encounter/state", 10)
        self._timer = self.create_timer(0.5, self._publish_state)

    def _publish_state(self) -> None:
        # TTL eviction: встречи без подтверждения > encounter_ttl_sec удаляются.
        evicted = self._registry.evict_expired(ttl_sec=3.0)
        if evicted:
            self.get_logger().info(f"🕐 [encounter] evicted {evicted} expired")
        active = self._registry.current()
        state = build_encounter_state_msg(active)   # см. §2.5.1
        self._state_pub.publish(state)
```

#### 2.5.1 Новый msg: `EncounterState.msg`

Расположение: `src/rob_box_perception_msgs/msg/EncounterState.msg`.

```
# Активные Встречи прямо сейчас. Публикуется encounter_node с частотой publish_rate_hz.
builtin_interfaces/Time stamp
Encounter[] encounters          # см. ниже

# Encounter
string кто                      # Знакомый.id (UUID) или transient_label (Голос-N) или пусто
float32 уверенность              # [0, 1]
string[] каналы                  # ["голос", "лицо"] — массив для совместимости с .msg
builtin_interfaces/Time с_какого_момента
float32 видели_до_этого_sec       # -1.0 если None
```

**Почему не JSON-blob**: `Encounter` — структурированный тип, читается из нескольких процессов (dialogue_node, mcp_server, TARS-cockpit, future UI). JSON-blob делает потребителей парсерами вместо типизированных подписчиков — та же ошибка, что `vision_events_json` уже сделал (issue #2442 «Проверка удалением»: `grep -rln 'vision_events_json'` — только продюсер, ноль консьюмеров). Правило: если поле структурируемое в `.msg` — оно должно быть в `.msg`.

**`EncounterState` vs расширение `PerceptionEvent`**: отдельный msg — потому что Encounter живёт дольше, чем один perception tick. `PerceptionEvent` — снапшот «сейчас». `EncounterState` — состояние присутствия (изменяется реже, читается чаще). Разные lifecycle → разные msg.

---

## 3. Concurrency и процессная модель

### 3.1 Почему singleton-in-process, а не distributed

| Подход | Плюсы | Минусы |
|---|---|---|
| Singleton `EncounterRegistry` в `encounter_node` (предложено) | Lock один, state один, debug просто | Один процесс — если упал, state потерян (cold start) |
| `RoboSOT` через `/encounter/state` (distributed) | Несколько читателей без зависимости от producer'а | Сериализация каждого тика, eventual consistency, гонки на merge |
| Каждый процесс держит свою копию | Robust к падению | Шов теряется — каждый процесс снова собирает по-своему (исходная проблема) |

**Решение: singleton в `encounter_node`**, публикация `/encounter/state` для **только-чтения** подписчиков. `dialogue_node` и `mcp_server` **не** держат свою копию — они подписываются на `/encounter/state` и кешируют последний msg (read-only cache). При падении `encounter_node` подписчики получают последний кешированный `EncounterState` и видят «нет свежих данных» через `stamp` — это **явный** сигнал «присутствие неизвестно», лучше, чем разные процессы с разными state'ами.

### 3.2 threading.Lock vs asyncio.Lock

Текущий `dialogue_node._current_speaker` использует `threading.Lock` (`dialogue_node.py:474`). ROS 2 executor — multi-threaded (при `MultiThreadedExecutor`) или single-threaded (default). `EncounterRegistry` использует **`threading.Lock`** для совместимости с обоими режимами — short critical section (list copy + dict update) не страдает от GIL. rclpy async API не трогаем.

---

## 4. Миграция потребителей

### 4.1 `dialogue_node._current_speaker` → `/encounter/state`

| Файл:строка | Что сейчас | Что после |
|---|---|---|
| `dialogue_node.py:473` | `self._current_speaker: dict = {"is_known": False}` | Удалить. Заменить на подписку `/encounter/state` (новый subscription в `__init__`). |
| `dialogue_node.py:474` | `self._speaker_lock = threading.Lock()` | Удалить (lock больше не нужен, шов immutable). |
| `dialogue_node.py:1957-1978` (`_on_speaker_result`) | Пишет в `self._current_speaker` | Удалить (логика перенесена в `voice_adapter.on_speaker_result` в `encounter_node`). |
| `dialogue_node.py:2162-2163, 2807-2808` | Читают `self._current_speaker` (диагностика) | Заменить на `last_encounter_state.encounters` (read-only). |
| `dialogue_node.py:3072-3073, 3114-3138` (`<user_profile>`) | XML-блок в LLM prompt | Заменить на `[ACTIVE ENCOUNTERS]` markdown-список (см. §2.1.3). |
| `dialogue_node.py:6226-6227` | Сброс `_current_speaker` при disconnect | Удалить (encounter сам истекает по TTL). |

**Подписка** в `dialogue_node.__init__`:
```python
self._last_encounter_state: Optional[EncounterState] = None
self.create_subscription(EncounterState, "/encounter/state",
    lambda msg: setattr(self, "_last_encounter_state", msg), 10)
```

В `_build_dynamic_system_context`: `if self._last_encounter_state: render_encounters_markdown(self._last_encounter_state)`.

### 4.2 `mcp_server.current_speaker_id` → `/encounter/state`

| Файл:строка | Что сейчас | Что после |
|---|---|---|
| `mcp_server.py:341` | `self.current_speaker_id: Optional[str] = None` | Удалить. |
| `mcp_server.py:485-528` (`_on_speaker_result`) | Пишет в `self.current_speaker_id` | Удалить (логика в `encounter_node.voice_adapter`). |
| `mcp_server.py:1313-1331` (`on_perception_update`) | Читает только `battery_percentage`, `internet_available`, `mapping_mode` (vision_events_json **игнорируется**) | Расширить: добавить чтение `last_encounter_state` (из подписки `/encounter/state`) → положить в `perception_context_tool.update_context` (`active_encounters_count`, `active_encounters_json` — сериализованный список для LLM). |
| `mcp_server.py:337-346` (комментарий про fallback на `current_speaker_id`) | Ссылается на удаляемое поле | Обновить комментарий: «fallback на `/encounter/state` через подписку». |

**Подписка** в `mcp_server` (аналогично §4.1).

### 4.3 `memory.py` 3× fallback → один шов

| Файл:строка | Что сейчас | Что после |
|---|---|---|
| `memory.py:88-91` (`MemorySaveTool._run` начало) | `speaker_id = kwargs.get("speaker_id") or getattr(self.node, "current_speaker_id", None)` | `speaker_id = kwargs.get("speaker_id") or _resolve_encounter_kто(self.node)` где `_resolve_encounter_kто` — helper в `mcp_server`, читает `last_encounter_state.encounters[0].кто` (первый в списке = самый уверенный при сортировке по `уверенность`). |
| `memory.py:176-178` (`MemorySearchTool` начало) | Аналогичный паттерн | Аналогично |
| `memory.py:290-292` (`MemoryContextTool` начало) | Аналогичный паттерн | Аналогично |

**Helper** располагается в `mcp_server.py` (новый private метод `_current_encounter_kто() -> Optional[str]`), импортируется из `memory.py`. Один путь вместо трёх — закрывает issue #2442 «голосовой фоллбэк на "текущего спикера" продублирован трижды».

### 4.4 Что НЕ мигрируем

| Что | Почему |
|---|---|
| `dialogue_node._handle_speaker_turn` (`:3268-3336`) | Это flow-управление диалогом (turn-taking), не state. Шов Encounter тут не нужен. |
| `speaker_id_node._on_merge_request` (`:431-477`) | Это merge биометрических эмбеддингов, живёт в `speaker_id_node`. Миграция — в рамках #2440, не этого ADR. |
| `dialogue_node:3302 (await touch_speaker)` | Это `last_seen` запись в `voice_memory.db`. После #2440 — `touch_speaker(Знакомый.id)`, не `tag`. Не этот ADR. |

---

## 5. Параллельные фиксы (из issue #2442, в scope этого ADR)

Эти правки **тривиальны** (config, очистка буфера, удаление мёртвого кода) и не требуют отдельных ADR. Они перечислены в issue #2442 «Что нужно» как обязательные шаги — реализуются в той же карточке, что и Encounter.

### 5.1 Конфигурационный fix: `input_topic` для `vision_hailo_node`

| Файл | Строка | Что сейчас | Что после |
|---|---|---|---|
| `docker/vision/config/hailo_models.yaml` | 38 | `input_topic: /oak/rgb/image_raw/compressed` | `input_topic: /camera/camera/color/image_raw` |
| `src/rob_box_perception/rob_box_perception/vision_hailo_node.py` | 112 | `self.declare_parameter('input_topic', '/oak/rgb/image_raw/compressed')` | `self.declare_parameter('input_topic', '/camera/camera/color/image_raw')` |

**Альтернатива**: включить `i_publish_compressed: true` в `docker/vision/config/oak-d/oak_d_config.yaml:44,61` (оба — color и depth) и оставить compressed-топик. Решение товарища Шифу: **прямой raw-топик** предпочтительнее — он быстрее (без JPEG-encode/decode), проще в отладке (виден в `ros2 topic hz`). Если OAK-D driver не публикует raw на Pi 5 — fallback на compressed, отдельная карточка.

**ВАЖНО**: эта правка **отдельная карточка** (см. §10 Open Q-3), потому что требует:
- проверки `ros2 topic list | grep camera` на роботе
- e2e-теста реального потока (5 FPS, не stub)
- возможного регресса в OAK-D driver (Pi 5 ARM64)

Без неё face-адаптер Encounter получает ноль кадров — **но** Encounter корректно работает только по голосовому каналу (graceful degradation, см. §6 edge-case 4).

### 5.2 Очистка буфера `_hailo_events` в `publish_event`

`src/rob_box_perception/rob_box_perception/context_aggregator_node.py`:

| Строка | Что сейчас | Что после |
|---|---|---|
| 299-305 | `on_hailo_vision_event` — добавляет + чистит по `cutoff` | Без изменений (источник правды — момент прихода события) |
| 507-535 (`publish_event`) | Читает `self._hailo_events` как есть | **Добавить в начало `publish_event`**: повторная чистка по `cutoff = now - self.memory_window` (защита от «заморозки» буфера при остановке потока). |
| `_hailo_events: List[Dict]` | Без `maxlen` | Перевести на `collections.deque(maxlen=publish_rate_hz * memory_window * EXPECTED_MAX_DETECTIONS_PER_FRAME)`. Default: `2.0 * 60 * 10 = 1200` записей. При превышении — `deque` автоэвикция FIFO, JSON-сериализация не падает. |

**`EXPECTED_MAX_DETECTIONS_PER_FRAME = 10`** — разумный upper bound для YOLOv8n (1-3 person + 2-5 object обычно). Если реальный поток превысит — увеличить, замерить в проде.

### 5.3 Заполнение `event_type='face'` и фильтрация stub

| Файл | Что | Когда |
|---|---|---|
| `src/rob_box_perception/rob_box_perception/vision_hailo_loader.py:119-130,508-513` | Сейчас оба продюсера жёстко пишут `'event_type': 'person'` и пустые `'embedding_id': ''`, `'display_name': ''` | **Не меняем в этом ADR.** Phase 2 (face recognition) начнёт заполнять. До этого face-адаптер Encounter (§2.4) фильтрует всё, что не `event_type='face'`. |

Stub-filter (`source_camera == 'stub'`) реализован **в `face_adapter.on_vision_event`** (§2.4), не в продюсере. Это инвариант адаптера — если когда-нибудь stub начнёт публиковать `event_type='face'` (для тестов), Encounter его игнорирует.

### 5.4 `vision_events_json` / `vision_event_count` — explicit deprecation

| Действие | Где | Когда |
|---|---|---|
| Пометить поля как **deprecated** | `PerceptionEvent.msg` (шапка комментария) | Этот PR |
| Добавить лог `⚠️ [encounter] vision_events_json deprecated, migrate to /encounter/state` | `context_aggregator_node.publish_event`, на каждый тик пока поля не пустые | Этот PR |
| Удалить поля через 1 прод-месяц наблюдения | Отдельная карточка, **не этот ADR** | После наблюдения |

`mcp_server.on_perception_update` (`mcp_server.py:1313-1331`) уже **не** читает эти поля (verified). Поэтому удаление безопасно по потребителям — но до удаления держим для обратной совместимости с любыми внешними подписчиками (TARS-cockpit, может быть).

---

## 6. Edge-кейсы (таблица)

| # | Ситуация | Ожидаемое поведение | Тест |
|---|---|---|---|
| 1 | Никого рядом (тишина + пустая сцена > TTL) | `EncounterRegistry.current() == []`. Через 1 сек `evict_expired` удаляет все Встречи. LLM получает `[ACTIVE ENCOUNTERS] (none)`. | Unit: создать Encounter, sleep > TTL, вызвать evict_expired, assert empty. |
| 2 | Только голос, без зрения (нормальный кейс) | `Encounter(кто=raw_speaker_id, уверенность=0.92, каналы={"голос"}, ...)`. | Integration: voice signal → current() → assert каналы={голос}. |
| 3 | Только лицо, без голоса (человек молча стоит) | `Encounter(кто=embedding_id или None, уверенность=0.85, каналы={"лицо"}, ...)`. LLM видит «кто-то один молча, по лицу — может быть Знакомый X». | Integration: vision signal → current() → assert каналы={лицо}. |
| 4 | Vision stub активен, voice молчит | `Encounter.current() == []` — face-адаптер фильтрует `source_camera='stub'`. LLM НЕ видит синтетических людей. | Unit: stub signal → current() → assert empty. |
| 5 | Voice и face сигналы в один Encounter, разные `кто` (collision) | **Два Encounter'а**: один с `кто=voice_id`, другой с `кто=face_id`. LLM видит список, решает сам. | Integration: emit voice=A + face=B → current() → assert len==2, ids разные. |
| 6 | Voice и face с одним `кто` в пределах `merge_window_sec=1.0` | **Один Encounter**: каналы={голос, лицо}, уверенность=max, с_какого_момента=min. | Integration: emit voice=A @ t=0 + face=A @ t=0.5 → current() → assert len==1, channels={голос,лицо}. |
| 7 | Cold start (нет ни одного сигнала) | `current() == []` сразу. Никаких default state. | Unit: fresh registry → current() → assert empty. |
| 8 | `encounter_node` упал, потом поднялся | Подписчики видят `stamp` старый → LLM получает сигнал «присутствие неизвестно». После рестарта — новое Encounter с новым `с_какого_момента`. Не пытаемся восстанавливать state — encounter ephemeral by design. | Integration: kill encounter_node → check stamp — assert old; restart → new Encounter → stamp fresh. |

---

## 7. Lifecycle

| Событие | Действие | Где |
|---|---|---|
| `encounter_node.__init__` | Создаёт `EncounterRegistry(merge_window_sec=1.0)`, подписки на `/voice/speaker/result` + `/vision/hailo/events`, publisher `/encounter/state`, timer на `_publish_state` (period = `1 / publish_rate_hz`, default 2.0 Hz) | `encounter_node.py:__init__` |
| Сигнал от voice | `voice_adapter.on_speaker_result` → `registry.update_signal(кто=..., уверенность=..., канал="голос")` | `voice_adapter.py` |
| Сигнал от vision | `face_adapter.on_vision_event` → после фильтров (event_type='face', source_camera != 'stub') → `registry.update_signal(...)` | `face_adapter.py` |
| Timer `_publish_state` (каждые 0.5 сек при `publish_rate_hz=2.0`) | `evict_expired(ttl_sec=3.0)` → `current()` → publish `/encounter/state` | `encounter_node.py:_publish_state` |
| `dialogue_node` / `mcp_server` получают `/encounter/state` | Кешируют последний msg в `self._last_encounter_state` (read-only) | `dialogue_node.py:subscription`, `mcp_server.py:subscription` |
| Во время LLM turn | `dialogue_node._build_dynamic_system_context` читает `_last_encounter_state`, рендерит `[ACTIVE ENCOUNTERS]` | `dialogue_node.py:_build_dynamic_system_context` |
| `encounter_node` shutdown | rclpy автоматически закрывает subscriptions/publishers. State не сериализуется — ephemeral by design. | ROS 2 default |
| `encounter_node` crash → restart | Новый процесс с пустым registry. Encounter'ы начинаются с нуля. Это **by design** (см. edge-case 8). | — |

**TTL параметры**:
- `merge_window_sec = 1.0` — окно, в котором voice+face с одним `кто` сливаются в один Encounter. Больше — больше шанс на ошибочный merge (двух разных людей в 2 секундах). Меньше — больше шанс на дублирование (лицо и голос сказали «я A» в разные моменты).
- `encounter_ttl_sec = 3.0` — время без подтверждения, после которого Encounter удаляется. 3 секунды = типичный utterance gap (пауза между фразами). Меньше — premature eviction (робот скажет «Денис ушёл» во время паузы). Больше — stale state (человек ушёл 30 сек назад, а LLM всё ещё думает, что он рядом).

Оба параметра — `declare_parameter` в `encounter_node`, можно тюнить через ROS 2 launch без правки кода.

---

## 8. Тест-план (Acceptance для реализации)

### 8.1 Unit-тесты (`src/rob_box_harness/test/test_encounter.py`)

| # | Тест | Инвариант |
|---|---|---|
| 1 | `test_encounter_value_object_is_frozen` | `Encounter(...).__setattr__` бросает `AttributeError`. |
| 2 | `test_encounter_confidence_in_range` | `Encounter(уверенность=1.5)` бросает `ValueError`. |
| 3 | `test_registry_update_signal_creates_new` | Пустой registry, `update_signal(кто=A, ...)` → `current() == [Encounter(кто=A, ...)]`. |
| 4 | `test_registry_merges_same_кто_in_window` | `update_signal(кто=A, канал="голос", t=0)` + `update_signal(кто=A, канал="лицо", t=0.5)` → один Encounter с `каналы={"голос","лицо"}`. |
| 5 | `test_registry_does_not_merge_outside_window` | С `merge_window_sec=1.0`: voice @ t=0 + face @ t=2.0 → два Encounter'а. |
| 6 | `test_registry_collision_different_кто` | voice `кто=A` + face `кто=B` → два Encounter'а. |
| 7 | `test_registry_evict_expired` | Encounter @ t=0, sleep до t=5, `evict_expired(ttl_sec=3.0, now=5)` → пусто. |
| 8 | `test_face_adapter_filters_non_face` | `event_type='person'` → ignore. `event_type='face'` → update_signal. |
| 9 | `test_face_adapter_filters_stub` | `source_camera='stub'` → ignore (defence-in-depth, даже если event_type='face'). |
| 10 | `test_voice_adapter_uses_resolver` | `identity_resolver = lambda raw: "KNOWN-" + raw` → `кто="KNOWN-abc"` в update_signal. |

### 8.2 Integration-тест (`src/rob_box_perception/test/integration/test_encounter_node.py`)

| # | Тест | Что проверяет |
|---|---|---|
| 11 | `test_encounter_node_publishes_state` | rclpy spin 2 сек, подписка на `/encounter/state`, emit `/voice/speaker/result` → assert msg приходит с `encounters[0].кто == expected`. |
| 12 | `test_encounter_node_handles_dropped_vision_topic` | После конфиг-фикса vision может быть down — assert `current()` всё ещё возвращает voice-only Encounter (graceful degradation, §6 edge-case 4). |
| 13 | `test_encounter_state_staleness_signal` | Kill `encounter_node`, check `msg.stamp` — assert старее, чем «сейчас» — это сигнал подписчикам «нет свежих данных». |

### 8.3 Acceptance из issue #2442 (п.7 — буквальные критерии)

| # | Критерий | Как проверить |
|---|---|---|
| A | Unit-тест на модуль Встречи — voice (`кто=X`, `уверенность=0.9`) + face (`кто=X`, `уверенность=0.8`) в окне <1с → один Encounter, `кто=X`, `каналы={голос,лицо}`, `уверенность=max(0.9,0.8)=0.9` | `pytest src/rob_box_harness/test/test_encounter.py::test_registry_merges_same_кто_in_window` |
| B | `grep -rn 'vision_events_json\|_current_speaker\|current_speaker_id' src/rob_box_voice src/rob_box_mcp_tools --include='*.py'` не даёт совпадений вне самого `encounter/` и его адаптеров | После миграции (§4) — буквально grep. |
| C | На живом роботе `ros2 topic hz /vision/hailo/events` после правки топика (п.5.1) показывает частоту, соответствующую реальному потоку (5 FPS для OAK-D, не 2 сек/period для stub) | Это **отдельная карточка** (config-fix), не acceptance этого ADR. Зафиксировано в §10 Open Q-3. |

### 8.4 Regression guard

| Что | Тест |
|---|---|
| `<user_profile>` XML больше не появляется в LLM prompt | `test_dialogue_node_system_context_no_user_profile_xml` (grep на отсутствие `<user_profile>` в `_build_dynamic_system_context` output) |
| `mcp_server.on_perception_update` не читает `vision_events_json` напрямую | `test_mcp_server_on_perception_update_uses_encounter_state` (mock PerceptionEvent, assert `perception_context_tool.update_context` получает `active_encounters_count > 0`, **не** `vision_events_json` ключ) |

### 8.5 Что НЕ тестируем в этом ADR

- Реальный OAK-D поток (это acceptance config-fix карточки).
- Face recognition через embedding (Phase 2, ADR-0089).
- `Знакомый.resolve` (issue #2440).

---

## 9. Альтернативы (явно отвергнутые)

### 9.1 Encounter как часть `PerceptionEvent.msg`

**Плюсы**: один msg, нет нового топика.
**Минусы**: `PerceptionEvent` публикуется с частотой `publish_rate_hz` (2 Hz) и **содержит** всё — battery, internet, mapping_mode. Encounter — high-level state, живёт дольше perception tick. Смешивание заставит dialogue_node/mcp_server фильтровать всё подряд. **Вердикт**: разные lifecycle → разные msg (см. §2.5.1).

### 9.2 Встреча через DBus/IPC вместо in-memory registry

**Плюсы**: переживает crash `encounter_node`.
**Минусы**: +50-200ms latency на каждый read, distributed state, шов Encounter становится «сетевой вызов» — это дороже, чем ephemeral state. **Вердикт**: ephemeral state — by design (issue #2442 «присутствие = "прямо сейчас"»). Если нужна persistence — это другая задача (например, `/encounter/journal` для ADR-0089 Phase 2).

### 9.3 Один Encounter на всех (кто сейчас рядом — один человек)

**Плюсы**: проще LLM prompt.
**Минусы**: теряем «в мастерской сейчас двое» (issue #2442 §1.2 collision). **Вердикт**: список Encounter'ов в `EncounterState.encounters[]` — корректная семантика.

### 9.4 `кто` как объект `Знакомый` (а не строка)

**Плюсы**: type-safe.
**Минусы**: encounter живёт в `rob_box_harness`, `Знакомый` — в будущем `rob_box_identity` (#2440). Чтобы `Encounter` зависел от `Знакомый`, нужен shared dependency — нарушение ADR-0080 (порт-шов, не shared domain type). **Вердикт**: `кто: str`, identity-resolver на стороне owner'а.

### 9.5 Использовать `PerceptionEvent.vision_events_json` (как планировал ADR-0089)

**Плюсы**: «бесплатно», уже в msg.
**Минусы**: `mcp_server` это поле **никогда не читал** (verified), `dialogue_node` не подписан на `PerceptionEvent`, `memory.py` тоже. Нулевой consumer base = нулевая ценность. ADR-0089 §9 предлагал «смёрджить JSON blob в `perception_context_tool.update_context`» — этот ADR заменяет план на структурированный msg `/encounter/state`, что лучше по всем осям (типизация, testability, debuggability). **Вердикт**: deprecate `vision_events_json` (см. §5.4).

### 9.6 Объединить с #2440 (сделать identity+encounter одной карточкой)

**Плюсы**: одна PR, один review.
**Минусы**: #2440 — чистая identity-логика (merge UUID ↔ Yandex tag ↔ voice_facts.speaker_id). #2442 — fusion + presence (voice + face merge window, stub-filter, TTL eviction). Разные оси, разные acceptance, разные тесты. Смешивание = один PR на 1000+ строк, review устаёт. **Вердикт**: sibling ADR'ы (ADR-0096 этот + ADR для #2440 когда Шифу его инициирует). Связь через `identity_resolver` (§2.3) — explicit injection, не implicit dependency.

---

## 10. Открытые вопросы (для товарища Шифу / implementer'а)

1. **`merge_window_sec = 1.0`**: достаточно для типичного «лицо увидело, через 0.5 сек голос заговорил — тот же человек». Меньше (0.5) — лучше для «два человека подряд заговорили» (collision защита). Больше (2.0) — лучше для «лицо увидело, человек повернулся, через 1.5 сек голос» (slow turn). Товарищ Шифу — какой default?
2. **«Несколько людей одновременно»**: §2.1.2 решает через список Encounter'ов. **Out of scope**: «лицо в кадре, голос от другого человека в фоне» — это `event_type='face'` vs voice. Если voice `кто=A` и face `кто=None` (новый embedding) — это два разных Encounter'а, не один. Товарищ Шифу — это корректное поведение?
3. **Config-fix карточка** (§5.1): `input_topic` → `/camera/camera/color/image_raw`. **Отдельная карточка** (не часть этого ADR) — потому что требует e2e-теста реального OAK-D потока. Подтвердить что выделяем отдельно?
4. **`event_type='face'` продюсер**: §2.4 + §5.3 — до отдельной карточки «Vision Pi: face detection pipeline» (Phase 2 из ADR-0089) — face-адаптер Encounter **молча игнорирует всё**. Это graceful degradation. Товарищ Шифу — подтвердить что voice-only режим приемлем до Phase 2?

---

## 11. Связанные карточки и PR

| Карточка / PR | Статус | Связь |
|---|---|---|
| `t_13fc8f29` (этот ADR) | running → completed после ревью | — |
| `t_5986a91c` (sibling — ADR для #2440 «Знакомый») | running в параллельной worktree | `identity_resolver` — explicit injection point, encounter **не** зависит от merge #2440 |
| PR #2432 (ADR-0089 Phase 1.5, RealHEFLoader) | merged | Producer-side готов, consumer-side = этот ADR |
| Issue #2359 (VisionEvent контракт полей) | open | Этот ADR **не трогает** контракт msg |
| Issue #2348 (калибровка порогов speaker-id) | open | Отдельный трек, не блокер |
| ADR-0089 (AI HAT+ deployment) | Accepted | §9 Phase 1.5 consumer-side = суть этого ADR |
| ADR-0093 (unknown-speaker ring) | Proposed (PR #2424) | Ring → Encounter через `transient_label` поле |

**После ревью этого ADR товарищем Шифу**:
- Реализационная карточка для `encounter_node` + адаптеров + миграция `dialogue_node`/`mcp_server`/`memory.py` (owner: backend).
- Config-fix карточка для `input_topic` (owner: devops / Vision Pi).
- Тесты по §8 (owner: tester).

---

## 12. Что проверить перед merge этого ADR

- [ ] Все ссылки на файл:строку актуальны (origin/develop SHA на момент PR).
- [ ] §10 Open questions либо разрешены Шифу, либо явно оставлены для implementer'а с пометкой «требует ответа перед merge PR».
- [ ] Раздел 8 (тест-план) покрывает все 3 acceptance из issue #2442 (п.7) — A, B, C.
- [ ] `voice_adapter` корректно работает с `identity_resolver=None` (до #2440).
- [ ] `face_adapter` фильтрует stub **до** `update_signal` (privacy-invarianta).
- [ ] `EncounterState.msg` объявлен в `CMakeLists.txt` пакета `rob_box_perception_msgs`.
- [ ] `encounter_node` добавлен в launch-файл (с `declare_parameter` для `merge_window_sec`, `encounter_ttl_sec`, `publish_rate_hz`).
- [ ] Никаких изменений в VisionEvent.msg (issue #2359 отдельно).
- [ ] Никаких изменений в `voice_memory.db` schema (issue #2440 / ADR-0055 отдельно).
- [ ] `grep -rn 'vision_events_json\|_current_speaker\|current_speaker_id' src/` **до** миграции = много совпадений (для baseline); **после** миграции = только в `encounter/` и его тестах (для regression guard).

---

> **Конец ADR-0096.** Реализация — после ревью товарищем Шифу. До этого момента архитектурный контракт зафиксирован здесь; любые правки в `_current_speaker` / `current_speaker_id` / `<user_profile>` / `vision_events_json` сверх §4-5 — регрессия.
