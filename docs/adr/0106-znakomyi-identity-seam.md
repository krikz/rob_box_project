# ADR-0106: «Знакомый» — единый шов идентичности человека взамен трёх несвязанных ключей

> Ранее фигурировал как ADR-0095 в коммите `acdb4cad` (PR #2465). Перенумерация 2026-09-15: см. issue #2582 — сосед `0095-pr-pollution-detection.md` влит раньше (`42cd89be`, PR #2447) и остался на 0095.

| Поле | Значение |
|---|---|
| Статус | **Proposed** (дизайн; реализация — отдельная карточка, после ревью Шифу) |
| Дата | 2026-09-14 |
| Автор | architect (Hermes Agent), kanban `t_5986a91c` |
| Контекст | Сегодня в проекте сосуществуют три **независимых, никак не синхронизированных** ключа, описывающих «кто этот человек»: биометрический UUID реземблайзера (`speakers.db`, `uuid.uuid4()`), эфемерный `speaker_tag` диаризации Yandex (`"0"`/`"1"`, не переживает даже разбиение одной фразы на две) и `speaker_id` в `voice_turns`/`voice_facts` (`voice_memory.db`), используемый MCP-инструментами памяти. На каждом из них живут свои атрибуты (имя, эпитет, история эпитетов, теги, sentiment — на ключе 1; `{first_seen, last_seen, dialog_count}` — на ключе 2; факты — на ключе 3). Из-за этого сценарий «давно не виделись» не вычислим **архитектурно**, а не из-за нехватки кода. |
| Затрагивает | (a) новый модуль `src/rob_box_harness/rob_box_harness/identity/` — интерфейс «Знакомый» (resolve/note_seen/since_last_seen/merge); (b) первый адаптер — голосовой (`src/rob_box_voice/rob_box_voice/utils/speaker_embeddings.py::SpeakerDatabase` через тонкую обёртку); (c) перенос `last_seen`/`first_seen`/`dialog_count` с ключа `speaker_scope(yandex_tag)` на `speaker_scope(znakomyi.id)` — `src/rob_box_harness/rob_box_harness/memory/base.py:450-560`, вызывающая сторона `src/rob_box_voice/rob_box_voice/dialogue_node.py:3302`; (d) объединённый merge в `src/rob_box_voice/rob_box_voice/speaker_id_node.py:431-477` через шов; (e) усечение `speaker_id` в system-context `src/rob_box_voice/rob_box_voice/dialogue_node.py:3136` и согласование контракта `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/memory.py:62-72,149-155,263-269`; (f) калибровка порогов (дефект A) — `src/rob_box_voice/config/speaker_id_node.yaml:15`, `docker/vision/config/voice_assistant/speaker_id_node.yaml:10`; (g) расширение `src/rob_box_voice/test/test_yaml_param_consistency.py` проверкой **значений**; (h) интеграционный acceptance-тест (issue #2440, п.8 Acceptance). |
| Родители | ADR-0018 (честный FAIL), ADR-0013 (incremental delivery), ADR-0080 (eight-seams), ADR-0093 (in-memory ring для неизвестных — сосед по identity-слою, расширяет UX, но не закрывает долгосрочную идентификацию) |
| Связанные | issue #2440 (этот ADR — его ответ), issue #2348 (калибровка порогов + merge — закрывает практическую сторону дефектов A/C; этот ADR закрывает **архитектурный** корень), issue #1077 (speaker profiles), issue #1770 (memory_context speaker_id), ADR-0037 (memory layers — определяет «где живёт scope=speaker:...»; этот ADR не меняет ADR-0037, он меняет **что** подставляется в уже принятый шаблон), ADR-0055 (consolidation файла БД — другой вопрос: «где хранить», не «кто есть спикер»), ADR-0089 Phase 2 (face — второй адаптер к предлагаемому шву; ADR-0089:300 явно откладывает arbitration голос+лицо без указания, через что она будет реализована — этот ADR задаёт контракт заранее) |

> **TL;DR.** Между потребителями (dialogue_node, mcp_server, UI-логи) и сырыми сигналами идентичности (resemblyzer d-vector, Yandex speaker_tag) — модуль-шов `rob_box_harness.identity` с value-объектом `Знакомый` (стабильный `id`, имя, эпитет, факты) и тремя операциями: `resolve(signal) -> Знакомый`, `note_seen(znakomyi) -> None`, `since_last_seen(znakomyi) -> float | None`, плюс `merge(a, b)` как операция шва. Голос становится **первым адаптером**: `resolve()` внутри вызывает `SpeakerDatabase.identify()/register_or_merge()` и возвращает `Знакомый.id = biometric_uuid`, **не** Yandex tag. `tag` остаётся только внутри `SpeakerTracker` как сигнал подтверждения реплики, наружу из шва не выходит. ADR-0089 Phase 2 (лицо) проектируется через **тот же** `resolve(face_signal)`, иначе придётся второй раз заводить профиль-with-last_seen. Дефекты A/B/C из #2440 закрываются в рамках реализации этого ADR (а не отдельной карточкой), потому что они — прямые следствия отсутствия шва.

---

## 0. Что внутри и что — нет

**Внутри этого ADR.**

- Интерфейс `Знакомый` (dataclass) и три (четыре) операции: `resolve(signal) -> Знакомый`, `note_seen(znakomyi)`, `since_last_seen(znakomyi)`, `merge(a, b)`. Контракт переданных сигналов, контракт возвращаемого объекта.
- Схема первого адаптера (голос): что шов берёт из `SpeakerDatabase`, что отдаёт; как Yandex `speaker_tag` остаётся внутри `_handle_speaker_turn`/`SpeakerTracker` (как и сейчас — это локальный сигнал подтверждения реплики, см. `speaker_profiles.py:1-22`), но **не выходит за пределы `dialogue_node`**.
- Миграция `last_seen`/`first_seen`/`dialog_count`: с `speaker_scope(yandex_tag)` на `speaker_scope(znakomyi.id)`. Прямое и обратное преобразование исторических фактов (data migration plan).
- Объединённый merge в `_on_merge_request` через шов: `merge_speaker_facts()` + `merge_speakers()` теперь работают в одном пространстве id, вызываются парой.
- Усечение `speaker_id` в `<system_context>` — отмена, передаём полный UUID; согласование описаний параметров в MCP-тулах `MemorySaveTool`/`MemorySearchTool`/`MemoryContextTool`.
- Калибровка порогов (дефект A) — `identify_threshold 0.75 → 0.72`, `register_match_threshold` объявляется в YAML. Это часть архитектурного фикса: прод сейчас работает на дефекте, который ADR должен немедленно закрыть, иначе любые acceptance-тесты будут сняты на «правильных» числах, а прод останется на «неправильных». Делается в том же PR, что и шов (atomic change).
- Расширение `test_yaml_param_consistency.py` проверкой **значений** порогов (защита от регрессии дефекта A).
- Acceptance-тест issue #2440 п.8: интеграционный тест, который краснеет на текущем коде и зеленеет после реализации шва.
- DoD: 8 пунктов из issue #2440 + критерии «архитектурной зрелости» шва.

**Не внутри этого ADR.**

- Реализация модуля `identity/` (отдельная карточка, после ревью этого ADR — оценка сложности ~3-5 дней; см. §11).
- Изменение прод-конфигов, не упомянутых в (f) выше (например, переключение `/data/voice_memory.db` → `/data/harness_voice.db` — это ADR-0055).
- Дизайн persistent-storage для неизвестных спикеров (отдельная задача; ADR-0093 ring буфер — соседний слой, не подмена).
- Дизайн arbitration «голос + лицо» (когда оба адаптера существуют — это отдельная карточка, см. ADR-0089:300 и §10.3).
- Изменение `dialogue_node` UI/UX за пределами замены источника `speaker_id` и снятия усечения.
- Перенос `dialogue_node._handle_speaker_turn` (структура логики подтверждения реплики остаётся — меняется только то, что наружу выходит `Знакомый`, а не `tag`).

---

## 1. Контекст и бизнес-проблема

### 1.1 Сценарий-заказчик (business driver)

Денис входит в мастерскую → робот поднимает биометрию → понимает, кто это → знает, что давно не виделись → заговаривает первым. Сегодня последний пункт невозможен **в принципе** — не из-за отсутствия данных, а потому что данные пишутся против трёх независимых ключей одного человека. `last_seen` пишется исключительно против Yandex `speaker_tag` (`memory/base.py:496-521`, вызов из `dialogue_node.py:3302`), который в следующей сессии почти наверняка достанется другому человеку или тому же человеку под другим tag. Биометрический `speaker_id` (тот самый, что реально идентифицирует голос между сессиями) живёт отдельно и `last_seen` против него **никто не пишет**.

### 1.2 Три ключа (raw state, проверено 2026-09-14)

| # | Имя ключа | Где живёт | Что на нём хранится | Время жизни | Стабильность |
|---|---|---|---|---|---|
| 1 | `biometric_uuid` | `speakers.db`, `speakers.speaker_id` PK (`uuid.uuid4()`, `speaker_embeddings.py:433`); `name`, `epithet`, `epithet_history`, `tags`, `sentiment_score`; эмбеддинги в `embeddings` | биометрический профиль голоса | persistent | **стабилен между сессиями** (d-vector ≈ match) |
| 2 | `yandex_tag` | in-memory per-utterance (`speaker_profiles.py:17-22`, `dialogue_node.py:3302`); `speaker_scope(f"{tag}")` в `memory/base.py:454` | `{first_seen, last_seen, dialog_count}` (через `Fact` в scope `speaker:<tag>`) | per-session | **нестабилен**: Yandex может разбить один голос на `"0"` и `"1"` в одной фразе; в новой сессии — другой tag |
| 3 | `voice_memory_speaker_id` | `voice_turns.speaker_id`, `voice_facts.speaker_id` (`/data/voice_memory.db`, миграция `migrations/009_voice_memory_speaker_id.sql`) | факты разговора (transcript → facts) | persistent | по идее = biometric_uuid из (1); на практике — отдельный столбец, синхронизации с (1)/(2) **нет** |

Ни один из трёх ключей не связан с другими программно. `merge_speakers()` (см. `speaker_id_node.py:445-459`) переносит только эмбеддинги **внутри ключа (1)**; ключ (2) переживает merge как был; ключ (3) синхронизации не имеет вообще.

### 1.3 Принцип «один адаптер — гипотетический шов, два — настоящий»

ADR-0089 Phase 2 уже сейчас (см. `docs/adr/0089-ai-hat-plus-deployment.md:63-66,263`) проектирует для лица отдельную БД `/data/faces.db` с собственными `last_seen_at`/`seen_count`, **независимую** от `speakers.db`, и отдельной строкой (там же, строка 300) откладывает «Multi-modal speaker_id_node arbitration (голос + лицо)» на «отдельную карточку» — то есть второй адаптер к идентичности человека уже запланирован и уже проектируется в отрыве от первого. Если не завести общий шов сейчас, Phase 2 повторит велосипед профиля-с-last_seen второй раз, и объединять два расходящихся хранилища идентичности придётся постфактум.

**Этот ADR — контракт, который Phase 2 обязана соблюсти.** Лицо становится вторым адаптером к тому же шву, а не второй параллельной реализацией.

---

## 2. Дефекты, которые закрывает шов

(Нумерация и формулировки — issue #2440.)

### 2.1 Дефект A — калибровка порога не доехала до робота

| Файл | Что | Цитата |
|---|---|---|
| `src/rob_box_voice/rob_box_voice/utils/speaker_embeddings.py:79` | Откалиброванное значение в коде | `IDENTIFY_THRESHOLD: float = 0.72` |
| `src/rob_box_voice/rob_box_voice/utils/speaker_embeddings.py:130` | Откалиброванное значение в коде | `REGISTER_MATCH_THRESHOLD: float = 0.75` |
| `src/rob_box_voice/config/speaker_id_node.yaml:15` | YAML в src (не менялся с `50ddbce8b`, 12 августа) | `identify_threshold: 0.75` |
| `docker/vision/config/voice_assistant/speaker_id_node.yaml:10` | YAML в docker (не менялся с `8d780c78e`, 12 августа) | `identify_threshold: 0.75` |
| оба YAML | `register_match_threshold` | **отсутствует ключ** |
| `src/rob_box_voice/rob_box_voice/speaker_id_node.py:71,78,99-100` | параметр из YAML монкипатчит модульную константу | `_se_mod.IDENTIFY_THRESHOLD = threshold` |
| `speaker_embeddings.py:50-59` | таблица калибровки | 0.75 → 36.4% распознанных; 0.72 → 63.6% |

ROS 2 params-файл через `parameters=[speaker_id_node_yaml]` (см. `launch/voice_assistant.launch.py:205+`, `docker/vision/config/voice_assistant/voice_assistant_headless.launch.py:183-188`) перебивает `declare_parameter`-дефолт — значит на роботе сейчас работает `0.75`, а не `0.72`. Калибровка коммита `d4c058af6` не доехала.

`src/rob_box_voice/test/test_yaml_param_consistency.py` (regression guard из #1004) проверяет только **имена** ключей YAML↔`declare_parameter` — тест зелёный при полностью устаревшем числе.

**Что делает шов.** Это не задача шва, но закрывается **в том же PR**, потому что acceptance-тест шва (§8) без откалиброванных порогов будет снят на «правильных» числах, а прод останется на «неправильных» — прод провалится в первую же минуту боевой эксплуатации. Правки атомарны: оба YAML → 0.72/0.75, `test_yaml_param_consistency.py` теперь проверяет **значения**.

### 2.2 Дефект B — усечённый `speaker_id` в инструкции для LLM

| Файл | Что | Цитата |
|---|---|---|
| `src/rob_box_voice/rob_box_voice/dialogue_node.py:3090` | извлечение id из профиля | `sp_id = sp.get("speaker_id") or ""` |
| `src/rob_box_voice/rob_box_voice/dialogue_node.py:3136` | усечение в system-context | `f"<speaker_id>{sp_id[:8]}</speaker_id>"` |
| `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/memory.py:62-72,149-155,263-269` | описание параметра в трёх MCP-тулах | «Опционально: voice-biometric id текущего спикера (из `<system_context>/<speaker_id>`)» |
| `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py:_on_speaker_result` | fallback в коде | сохраняется **полный** UUID (комментарий: «сохраняем UUID и используем его как fallback») |
| `src/rob_box_voice/rob_box_voice/core/voice_memory.py:644-656`, `_speaker_clause` | сравнение в БД | `(vt.speaker_id = ? OR vt.speaker_id IS NULL)` — точное, без `LIKE`/префикса |

Если LLM выполняет буквальную инструкцию тула и передаёт `speaker_id` из `<system_context>` (8 символов из 36-символьного UUID), точное сравнение не совпадёт ни с одной строкой — факт молча уйдёт в «global» ветку, поиск не найдёт персональных записей.

**Что делает шов.** Шов отдаёт в `<system_context>` **полный** `Знакомый.id` (UUID), усечение снимается. Описание параметра в MCP-тулах переписывается: «полный UUID из `<system_context>/<speaker_id>`, передавай целиком». Сравнение в БД продолжает быть точным, что **правильно** — после фикса оно начинает работать.

### 2.3 Дефект C — `merge_speaker_facts` не вызывается из прода

| Файл | Что |
|---|---|
| `src/rob_box_harness/rob_box_harness/memory/base.py:523-560` | определение `merge_speaker_facts(store, src_tag, dst_tag)` |
| `src/rob_box_harness/rob_box_harness/memory/__init__.py:30,62` | экспорт |
| `src/rob_box_voice/rob_box_voice/utils/speaker_embeddings.py:504` | докстринг-упоминание: «вызывающий код... должен ОТДЕЛЬНО вызвать» |
| `src/rob_box_harness/test/test_speaker_memory.py` | unit-тест |
| `src/rob_box_voice/rob_box_voice/speaker_id_node.py:431-477` | `_on_merge_request`, **вызывает только** `self._db.merge_speakers(src_id, dst_id)` (строка 453) |

Прод-точка слияния — `_on_merge_request` — вызывает только `merge_speakers()`. `merge_speaker_facts()` нигде в проде не вызывается. И даже если бы вызов добавили механически — слои не совпадают по оси идентичности: `merge_speakers()` оперирует биометрическим UUID (ключ 1), `merge_speaker_facts()` оперирует `speaker_scope(tag)`, где `tag` — Yandex-тег (ключ 2). Это **разные** пространства идентичности — склейка (1) не имеет прямого отображения на (2) без маппинга, которого сейчас не существует.

**Что делает шов.** После §3 `merge_speaker_facts` и `merge_speakers` работают в одном пространстве id (биометрический UUID), и их можно вызвать парой без ручного маппинга tag↔uuid. `_on_merge_request` после реализации шова вызывает `identity.merge(znakomyi_a, znakomyi_b) -> znakomyi_result`, а шов сам разруливает оба слоя.

---

## 3. Решение: интерфейс шва «Знакомый»

### 3.1 Value-объект `Знакомый`

```python
# src/rob_box_harness/rob_box_harness/identity/types.py
from dataclasses import dataclass, field
from typing import Optional
import time


@dataclass(frozen=True)
class Znakomyi:
    """Единый value-объект идентичности человека.

    Один id стабилен между сессиями и между адаптерами (голос, лицо, RFID).
    Все атрибуты профиля (имя, эпитет, история эпитетов, теги, sentiment,
    last_seen, dialog_count) — либо в самом dataclass, либо доступны через
    MemoryStore в scope=speaker:<id>. Сам dataclass не знает, откуда он
    пришёл (resemblyzer, Yandex, face-recognition) — это и есть шов.

    Attributes:
        id: стабильный идентификатор (для голосового адаптера —
            biometric_uuid из speakers.speakers.speaker_id).
        name: отображаемое имя (может быть None, если человек ещё не
            представился; не путать с эпитетом).
        epithet: персистентный эпитет («Кудрявый», «Скоростной» —
            см. research/voice-epithet-design.md).
        first_seen: unix-time первого подтверждённого контакта.
        dialog_count: число подтверждённых реплик за всё время.
            last_seen НЕ хранится в dataclass — читается через
            since_last_seen() с записью в MemoryStore. Это даёт
            согласованность с ADR-0037 и не дублирует state.
    """

    id: str
    name: Optional[str] = None
    epithet: Optional[str] = None
    first_seen: Optional[float] = None
    dialog_count: Optional[int] = None
    # Признак «создан только что в этой сессии» — для UX-подсказки
    # dialogue_node (см. speaker_profiles.format_speaker_context:155-159).
    is_new: bool = field(default=False, compare=False)

    def __post_init__(self) -> None:
        if not self.id:
            raise ValueError("Znakomyi.id must be non-empty")
        if not _UUID_RE.fullmatch(self.id):
            # Голосовой адаптер возвращает biometric_uuid — формат UUIDv4.
            # Если будущий адаптер (RFID/NFC) захочет другой формат —
            # ослабим эту проверку, но не раньше, чем он появится.
            raise ValueError(f"Znakomyi.id must be UUID, got {self.id!r}")
```

**Дизайн-решения.**

- `id: str`, не `UUID` — облегчает сериализацию в JSON (для MCP-тулов и system-context), `str(UUID)` дешёв.
- `frozen=True` — value-объект иммутабелен; мутация = создание нового через `dataclasses.replace`.
- `name`/`epithet`/`first_seen`/`dialog_count` живут **в dataclass**, а не в `MemoryStore`. Почему: эти четыре поля нужны **каждому** потребителю (`dialogue_node._build_dynamic_system_context`, `mcp_server._on_speaker_result`, UI-логи) на каждом resolve; заставлять их ходить в `MemoryStore.search_facts(scope="speaker:<id>", query="profile")` на каждый utterance — лишний I/O. `last_seen` — наоборот, **не** в dataclass: он обновляется на каждый подтверждённый ход (через `note_seen`) и читается только когда нужно вычислить «давно не виделись» — это операция с побочным эффектом записи, в dataclass ей не место.
- `is_new` помечен `compare=False`, потому что это **сессионный** признак (был ли создан в текущей сессии), не часть идентичности — он не должен влиять на равенство двух `Знакомый`.

### 3.2 Сигналы и операции шва

```python
# src/rob_box_harness/rob_box_harness/identity/__init__.py
from .types import Znakomyi
from .seam import IdentitySeam


__all__ = ["Znakomyi", "IdentitySeam"]


# Контракт сигналов (structural typing).
# VoiceSignal — то, что приходит из speaker_id_node после resolve.
# FaceSignal — что придёт из vision_face_node после Phase 2 ADR-0089.
Signal = "VoiceSignal | FaceSignal"   # pseudo-type; реально — два dataclass'а
```

```python
# src/rob_box_harness/rob_box_harness/identity/seam.py
from typing import Protocol, Optional, runtime_checkable
from .types import Znakomyi


class IdentitySeam(Protocol):
    """Контракт шва идентичности. Реализация по умолчанию —
    VoiceIdentityAdapter (см. §5.1). Второй адаптер (Face) появится
    в ADR-0089 Phase 2.

    Все операции — асинхронные, потому что в проде (dialogue_node)
    они вызываются из колбэков ROS 2, а I/O к speakers.db / MemoryStore
    — асинхронный (SQLite через to_thread / MemoryStore.search_facts).
    """

    async def resolve(self, signal: "Signal") -> Znakomyi:
        """Поставить сигнал в соответствие Знакомому.

        Семантика:
        - если сигнал распознан (similarity >= identify_threshold для голоса,
          >= face_match_threshold для лица) — вернуть существующего
          Знакомого с is_new=False;
        - если не распознан — зарегистрировать нового, вернуть Знакомого
          с is_new=True и persist=True (сразу в БД, см. §5.2);
        - если сигнал от шумного/неполного источника (Yandex tag=None,
          face confidence ниже min_face_confidence) — вернуть None.
          Это НЕ ошибка, это значит «нечего резолвить».

        Side effects: возможно INSERT в speakers.db (для голоса),
        INSERT/UPDATE в MemoryStore scope=speaker:<id> (для профиля).
        """
        ...

    async def note_seen(self, znakomyi: Znakomyi, *, now: float | None = None) -> None:
        """Зафиксировать контакт с человеком (обновить last_seen и
        dialog_count в scope=speaker:<id>). Вызывается на каждый
        подтверждённый ход спикера (после SpeakerTracker.note_phrase()
        вернул True).

        Атомарна относительно profile-fact: пишет один Fact целиком
        (dict {first_seen, last_seen, dialog_count}), чтобы не было
        состояния «last_seen обновлён, dialog_count нет».
        """
        ...

    async def since_last_seen(self, znakomyi: Znakomyi, *, now: float | None = None) -> float | None:
        """Сколько секунд прошло с последнего контакта. None — никогда
        не виделись (first_seen == last_seen == now; dialog_count == 1).

        Используется для UX-реплик «Давно не виделись, N дней».
        Это ЧИСТОЕ ЧТЕНИЕ (без побочных эффектов), в отличие от
        note_seen — специально, чтобы UI-логи могли опрашивать
        без риска race с записью.
        """
        ...

    async def merge(self, src: Znakomyi, dst: Znakomyi) -> Znakomyi:
        """Объединить двух Знакомых (src → dst). Возвращает dst после
        слияния. Используется при ручном merge в _on_merge_request
        или при auto-merge через register_or_merge().

        Атомарна в смысле «оба слоя (биометрия + факты) либо оба
        объединены, либо оба нет». После неё src.id перестаёт
        существовать в speakers.db и MemoryStore.
        """
        ...
```

**Дизайн-решения.**

- Четыре операции, не три — `merge()` обязателен, потому что иначе дефект C (§2.3) решается добавлением ещё одного ручного вызова в `_on_merge_request`, и при появлении третьего адаптера (Phase 2.5) придётся писать третий. Шов делает merge один раз, адаптеры подписываются.
- `resolve()` возвращает `Znakomyi | None` (через Protocol это `Optional[Znakomyi]`), не бросает — «некому резолвить» это нормальный кейс, не исключение. Исключения только для реальных ошибок (БД недоступна, corrupted embedding).
- `since_last_seen()` без сайд-эффектов — чтобы UI-логи и dialogue_node могли опрашивать сколько угодно раз без race.
- Все методы `async` — потому что в проде они вызываются из ROS 2 callbacks, и даже в тестах mock-объекты проще делать `async`. Синхронная обёртка (если нужна для скриптов) добавляется отдельной convenience-функцией, не ломает контракт.

### 3.3 Чего шов НЕ делает (явные non-goals)

- **Не хранит эпитеты и их историю** — это слой `epithets.py` (`src/rob_box_voice/rob_box_voice/epithets.py`, см. ADR-0093 §3.1). Шов хранит **текущий** эпитет в `Znakomyi.epithet`; смену эпитета делает эпитетный модуль, шов лишь принимает обновлённый `Znakomyi`.
- **Не знает про Yandex tag** — tag входит в `VoiceSignal` как поле «speaker_tag», но шов его **игнорирует**. Tag — внутренний сигнал `_handle_speaker_turn` для `SpeakerTracker` (подтверждение реплики, см. `speaker_profiles.py:8-22`). После `SpeakerTracker.note_phrase()` вернул True, voice-адаптер вызывает `resolve(voice_signal)` и **забывает tag**.
- **Не делает arbitration между адаптерами** — когда появятся голос + лицо одновременно, **отдельная** карточка (см. §10.3). Шов гарантирует только, что оба адаптера возвращают `Znakomyi` с тем же `id` для того же человека.
- **Не решает privacy/GDPR** — `transient_label` из ADR-0093, opt-in/opt-out, удаление профиля по запросу — отдельная задача. Шов только фиксирует, что `id` стабилен.

---

## 4. Архитектура (схема зависимостей)

```
┌────────────────────────────────────────────────────────────────────┐
│                       Потребители (consumers)                      │
│                                                                    │
│   dialogue_node ─┐                                                 │
│   mcp_server ────┼─→ IdentitySeam.resolve(signal) → Znakomyi       │
│   UI/логи ───────┘   IdentitySeam.note_seen(znakomyi)             │
│                      IdentitySeam.since_last_seen(znakomyi)        │
│                      IdentitySeam.merge(a, b) → c                  │
└────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌────────────────────────────────────────────────────────────────────┐
│                       IdentitySeam (Protocol)                      │
│                                                                    │
│   src/rob_box_harness/rob_box_harness/identity/seam.py:Protocol    │
└────────────────────────────────────────────────────────────────────┘
                                  │
            ┌─────────────────────┴─────────────────────┐
            ▼                                           ▼
┌──────────────────────────┐               ┌──────────────────────────┐
│  VoiceIdentityAdapter    │               │  FaceIdentityAdapter     │
│  (первый адаптер)        │               │  (ADR-0089 Phase 2)      │
│                          │               │                          │
│  оборачивает             │               │  оборачивает             │
│  SpeakerDatabase         │               │  FaceDatabase            │
│  (resemblyzer d-vector)  │               │  (InsightFace / ArcFace) │
└──────────────────────────┘               └──────────────────────────┘
            │                                           │
            ▼                                           ▼
┌──────────────────────────┐               ┌──────────────────────────┐
│  /data/speakers.db       │               │  /data/faces.db          │
│  (биометрия голоса)      │               │  (биометрия лица)        │
│  SpeakerDatabase         │               │  FaceDatabase            │
└──────────────────────────┘               └──────────────────────────┘
            │
            └────────────────────┐
                                 ▼
┌────────────────────────────────────────────────────────────────────┐
│                 MemoryStore (общий для всех адаптеров)             │
│                                                                    │
│   scope=speaker:<Znakomyi.id>                                      │
│     Fact(key="profile", value={first_seen, last_seen, dialog_count})│
│     Fact(key="имя", value=...)                                       │
│     Fact(key="предпочтения", value=...)                             │
│                                                                    │
│   ADR-0037 §2.5 уже фиксирует scope=speaker:<...> как имя;          │
│   этот ADR меняет ЧТО подставляется в <...>.                       │
└────────────────────────────────────────────────────────────────────┘
```

**Ключевые следствия.**

1. Потребители (`dialogue_node`, `mcp_server`, UI) **никогда** не импортируют `SpeakerDatabase`, `FaceDatabase`, `speaker_profiles`, `memory.base.speaker_scope` напрямую — только `identity.Znakomyi` + `identity.IdentitySeam`. Это и есть «шов».
2. `SpeakerTracker` (`speaker_profiles.py`) остаётся **внутри** `dialogue_node._handle_speaker_turn` (он про подтверждение реплики по tag, это ортогональная задача). Наружу он не выходит — на выходе из `_handle_speaker_turn` уже `Znakomyi`, не `tag`.
3. `MemoryStore` — общий слой для всех адаптеров. ADR-0037 §2.5 уже зафиксировал `scope=speaker:<...>` как имя; этот ADR не переоткрывает ADR-0037, а лишь фиксирует, что в `<...>` подставляется **биометрический UUID** (после `resolve`), а **не** Yandex tag.
4. ADR-0055 (consolidation БД) — независим: где физически лежит `voice_memory.db` (отдельный файл или общий с harness) — не меняет контракт шва. После ADR-0055 три key-пространства переносятся в один файл, но с тем же `id` пространством.

---

## 5. Адаптеры

### 5.1 VoiceIdentityAdapter — первый и единственный сейчас

```python
# src/rob_box_harness/rob_box_harness/identity/voice_adapter.py
class VoiceIdentityAdapter:
    """Обёртка над SpeakerDatabase для шва.

    Не меняет SpeakerDatabase (она уже зеленеет по тестам #2348).
    Просто маршалит идентификацию в шов.
    """

    def __init__(self, db: SpeakerDatabase, memory: MemoryStore) -> None:
        self._db = db
        self._memory = memory

    async def resolve(self, signal: VoiceSignal) -> Znakomyi | None:
        if signal.embedding is None or signal.yandex_tag is None:
            # Нечего резолвить (тишина / Vosk fallback).
            return None
        speaker_id = self._db.identify(signal.embedding)
        if speaker_id is None:
            # Не похож ни на кого известного — заводим нового.
            speaker_id = self._db.register(embedding=signal.embedding, name=None)
            is_new = True
        else:
            is_new = False
        # Тянем имя/эпитет/first_seen/dialog_count из профиля.
        profile = await _ensure_speaker_profile(self._memory, speaker_id)
        return Znakomyi(
            id=speaker_id,
            name=profile.get("name"),
            epithet=profile.get("epithet"),
            first_seen=profile.get("first_seen"),
            dialog_count=profile.get("dialog_count"),
            is_new=is_new,
        )

    async def note_seen(self, znakomyi: Znakomyi, *, now: float | None = None) -> None:
        await touch_speaker(self._memory, znakomyi.id, now=now)
        # NB: touch_speaker в новой версии принимает znakomyi.id
        # вместо yandex_tag (см. §6.1).

    async def since_last_seen(self, znakomyi: Znakomyi, *, now: float | None = None) -> float | None:
        profile = await get_speaker_profile(self._memory, znakomyi.id)
        if profile is None or "last_seen" not in profile:
            return None
        return (now or time.time()) - profile["last_seen"]

    async def merge(self, src: Znakomyi, dst: Znakomyi) -> Znakomyi:
        # Атомарно: оба слоя.
        moved = self._db.merge_speakers(src.id, dst.id)        # биометрия
        moved_facts = await merge_speaker_facts(self._memory, src.id, dst.id)  # факты
        logger.info(f"merge {src.id[:8]} → {dst.id[:8]}: embeddings={moved}, facts={moved_facts}")
        # Возвращаем dst (он и есть «результат»).
        return dataclasses.replace(dst)
```

**Дизайн-решения.**

- `VoiceIdentityAdapter` живёт в `rob_box_harness.identity`, а не в `rob_box_voice.utils` — потому что **сам шов** принадлежит harness (общий слой идентичности, не специфичный для голоса). `SpeakerDatabase` остаётся в `rob_box_voice` (это специфичная для голоса БД), адаптер её **импортирует** и оборачивает.
- `resolve()` принимает `VoiceSignal` (dataclass с `embedding`, `yandex_tag`, `confidence` и т.д.), не сырой embedding — потому что `SpeakerTracker` уже вычисляет confidence, и повторно его считать бессмысленно. Контракт `VoiceSignal` фиксируется в `identity/voice_signal.py`.
- `register()` без `name` — потому что имя ещё не спросили (extract_speaker_name может сработать позже, на следующей реплике). Это **не** ошибка: имя `None` — допустимое состояние `Znakomyi.name`.

### 5.2 Что попадает в Znakomyi прямо, а что через MemoryStore

| Поле Znakomyi | Источник | Обновляется |
|---|---|---|
| `id` | биометрический UUID (от SpeakerDatabase.identify/register) | при register/merge |
| `name` | `MemoryStore.search_facts(scope, query="name")` или `extract_speaker_name(text)` | когда пользователь представился |
| `epithet` | `epithets.choose_epithet(...)` (отдельный модуль) | эпитетный цикл |
| `first_seen` | `MemoryStore.search_facts(scope, query="profile")["first_seen"]` | один раз при register |
| `dialog_count` | то же | на каждый `note_seen` |
| `last_seen` | **не в dataclass**, читается через `since_last_seen()` | на каждый `note_seen` |

Логика: поля, которые нужны **каждому потребителю на каждом resolve** (`name`, `epithet`, `first_seen`, `dialog_count`), — в dataclass. Поля, которые нужны редко и обновляются часто (`last_seen`), — в MemoryStore.

### 5.3 Будущий FaceIdentityAdapter (ADR-0089 Phase 2)

В Phase 2 появляется второй адаптер, симметричный по контракту:

```python
class FaceIdentityAdapter:
    def __init__(self, db: FaceDatabase, memory: MemoryStore, seam: IdentitySeam) -> None:
        self._db = db
        self._memory = memory
        self._seam = seam   # для arbitration (см. §10.3)

    async def resolve(self, signal: FaceSignal) -> Znakomyi | None:
        # ... аналогично Voice, но над /data/faces.db ...
```

**Гарантия шва:** оба адаптера возвращают `Znakomyi` с **одним и тем же `id`**, потому что оба пишут в `MemoryStore` под `scope=speaker:<uuid>`, и `uuid` генерируется **один раз** при первом контакте (любым адаптером), а второй адаптер при первом «узнавании» лица находит существующий профиль по другому каналу (например, через cross-modal association table — деталь Phase 2).

ADR-0089 Phase 2 ОБЯЗАН реализовать `FaceIdentityAdapter`, следующий этому контракту; в противном случае карточка ADR-0106 остаётся `Proposed` и Phase 2 блокируется на ревью.

---

## 6. Миграция (пошаговая)

### 6.1 Шаг 1: `memory/base.py` — `speaker_scope` принимает любой ключ, не только Yandex tag

Текущее `speaker_scope(tag)` (строка 454-456) формально работает с любым `str`, но **семантически** привязано к Yandex tag (докстринг «Yandex `speaker_tag`», все вызывающие передают `tag`). Миграция — переименовать параметр в `speaker_id`, обновить докстринг, **сохранить обратную совместимость** (старое имя `tag` → алиас):

```python
def speaker_scope(speaker_id: str) -> str:
    """Scope-ключ памяти для спикера.

    Начиная с ADR-0106, ``speaker_id`` — это стабильный биометрический
    UUID (или любой стабильный ключ, который адаптер шва IdentitySeam
    назначил этому человеку). До ADR-0106 сюда передавался Yandex
    ``speaker_tag`` ("0"/"1"), что приводило к расщеплению профиля
    одного человека между сессиями (issue #2440).
    """
    return f"{SPEAKER_SCOPE_PREFIX}{speaker_id}"


# Backward-compat alias — старый код на 1-2 версии ещё может
# вызывать speaker_scope(tag=...), не падаем.
speaker_scope_tag = speaker_scope  # deprecated: use speaker_scope(speaker_id)
```

`touch_speaker`, `ensure_speaker_profile`, `merge_speaker_facts` переименовывают параметр `tag` → `speaker_id` без изменения логики.

**Совместимость данных.** Существующие факты в `MemoryStore` под `scope=speaker:0`, `scope=speaker:1` (Yandex tags) **остаются как были**. Они не мигрируют автоматически — после деплоя они просто перестают читаться (никто не пишет в них), а через retention-цикл ADR-0037 будут удалены. Это **намеренно**: исторические данные, привязанные к нестабильному ключу, не имеют долгосрочной ценности (содержимое — `{first_seen, last_seen, dialog_count}` за пару сессий). Если окажется, что какие-то факты под `speaker:0` всё-таки важны (например, имя, которое человек назвал в прошлой сессии), это всплывёт как отдельная задача с explicit data-migration SQL.

### 6.2 Шаг 2: `dialogue_node._handle_speaker_turn` — на выходе Znakomyi, не tag

Сейчас (`dialogue_node.py:3268-3336`, в частности 3302):

```python
profile = await touch_speaker(self._memory, tag)   # tag — Yandex
```

После:

```python
znakomyi = await self._identity.resolve(voice_signal)   # signal уже содержит tag
if znakomyi is None:
    return
await self._identity.note_seen(znakomyi)
```

`SpeakerTracker.note_phrase(tag, duration_s)` остаётся внутри `_handle_speaker_turn` — он отвечает за «streak подряд идущих фраз с одним tag», это ортогональная identity-проблеме задача (anti-flap по tag). Но **наружу** из `_handle_speaker_turn` выходит `Znakomyi`, а не `tag`.

### 6.3 Шаг 3: `_build_dynamic_system_context` — убрать усечение, отдать полный UUID

`dialogue_node.py:3136`:

```python
# БЫЛО:
lines.append(f"   <speaker_id>{sp_id[:8]}</speaker_id>")
# СТАЛО:
lines.append(f"   <speaker_id>{sp_id}</speaker_id>")   # полный UUID
```

`sp_id` теперь приходит не из `sp.get("speaker_id")` (старая форма), а из `znakomyi.id` после `resolve()`. Длина строки — 36 символов, для LLM это шум (несколько токенов), но **безопасный** (UUID не несёт PII).

### 6.4 Шаг 4: MCP-тулы — переписать описание параметра

`src/rob_box_mcp_tools/rob_box_mcp_tools/tools/memory.py:62-72,149-155,263-269`:

```python
# БЫЛО:
description=(
    "Опционально: voice-biometric id текущего спикера (из "
    "<system_context>/<speaker_id>). Если передан — факт "
    "сохраняется ТОЛЬКО этому пользователю; иначе факт "
    "становится глобальным. ВСЕГДА передавай speaker_id для "
    "персональных фактов (имя, предпочтения)."
)
# СТАЛО:
description=(
    "Опционально: voice-biometric id текущего спикера — полный UUID "
    "из <system_context>/<speaker_id>. Если передан — факт "
    "сохраняется ТОЛЬКО этому пользователю; иначе факт "
    "становится глобальным. ВСЕГДА передавай полный speaker_id "
    "(36 символов, с дефисами) для персональных фактов "
    "(имя, предпочтения)."
)
```

### 6.5 Шаг 5: `_on_merge_request` — вызывать `identity.merge()`

`speaker_id_node.py:445-459`:

```python
# БЫЛО:
moved = self._db.merge_speakers(src_id, dst_id)   # только эмбеддинги
# СТАЛО:
src_z = await self._identity.resolve(VoiceSignal(speaker_id=src_id, ...))
dst_z = await self._identity.resolve(VoiceSignal(speaker_id=dst_id, ...))
result_z = await self._identity.merge(src_z, dst_z)
# identity.merge() сам вызывает и merge_speakers(), и merge_speaker_facts().
```

### 6.6 Шаг 6: калибровка порогов (дефект A, атомарно)

| Файл | До | После |
|---|---|---|
| `src/rob_box_voice/config/speaker_id_node.yaml:15` | `identify_threshold: 0.75` | `identify_threshold: 0.72` |
| `docker/vision/config/voice_assistant/speaker_id_node.yaml:10` | `identify_threshold: 0.75` | `identify_threshold: 0.72` |
| оба YAML | `register_match_threshold: <отсутствует>` | `register_match_threshold: 0.75` |

Это **в том же PR**, что и шов — без этого acceptance-тест шва (§8) будет снят на «правильных» числах, а прод останется на «неправильных».

### 6.7 Шаг 7: `test_yaml_param_consistency.py` — проверка значений

Расширить тест: после проверки имён ключей проверять, что `identify_threshold`/`register_match_threshold` в YAML совпадают с `speaker_embeddings.IDENTIFY_THRESHOLD`/`REGISTER_MATCH_THRESHOLD`. Если расходятся — тест красный, CI падает.

---

## 7. Acceptance / DoD (8 пунктов из issue #2440)

| # | Пункт issue #2440 | Где закрывается | DoD |
|---|---|---|---|
| 1 | Модуль «Знакомый» с интерфейсом `resolve/note_seen/since_last_seen/merge` | §3 этого ADR | Файлы `src/rob_box_harness/rob_box_harness/identity/{__init__.py,types.py,seam.py}` существуют, `IdentitySeam` Protocol определён, `Znakomyi` dataclass — frozen. |
| 2 | Голосовая биометрия — первый адаптер | §5.1 | `VoiceIdentityAdapter` реализован, `resolve()` возвращает `Znakomyi.id = biometric_uuid`. Прод-кода в `dialogue_node`, который импортирует `speaker_embeddings.SpeakerDatabase` напрямую, больше нет. |
| 3 | `last_seen`/`first_seen`/`dialog_count` на ключе `speaker_scope(znakomyi.id)` | §6.1, §6.2 | `memory/base.py:touch_speaker` принимает `speaker_id` (не `tag`); `dialogue_node:3302` вызывает `note_seen(znakomyi)`, а не `touch_speaker(tag)`. |
| 4 | `merge` как операция шва; `_on_merge_request` вызывает объединённый merge | §6.5 | `_on_merge_request` вызывает `identity.merge(a, b)`; внутри — `merge_speakers()` + `merge_speaker_facts()`, атомарно. |
| 5 | Убрать усечение `speaker_id` в `<system_context>`; согласовать MCP-тулы | §6.3, §6.4 | `dialogue_node.py:3136` отдаёт полный UUID; описания параметров в `memory.py:62-72,149-155,263-269` обновлены; fallback в `mcp_server._on_speaker_result` продолжает работать (он уже использует полный UUID). |
| 6 | Расширить `test_yaml_param_consistency.py` проверкой **значений** | §6.7 | Тест проверяет и имена, и значения `identify_threshold`/`register_match_threshold` против кода. CI красный при расхождении. |
| 7 | Задокументировать контракт шва как обязательную точку интеграции для ADR-0089 Phase 2 | §5.3 | Этот ADR (документ) + комментарий в `docs/adr/0089-ai-hat-plus-deployment.md:300` (правка в этом PR или в Phase 2 PR): «arbitration гол+лицо — через IdentitySeam, см. ADR-0106 §5.3». |
| 8 | Acceptance-тест (см. §8 ниже) | §8 этого ADR | Тест краснеет на текущем коде, зеленеет после реализации шва. |

**Дополнительный DoD (архитектурная зрелость шва, выходит за рамки issue #2440).**

- Шов не импортирует `rclpy`/`ros2` — тестируется в pure-Python.
- `IdentitySeam` — `Protocol`, не абстрактный класс; mock-реализации для тестов пишутся без наследования.
- В `dialogue_node` / `mcp_server` / UI grep на `speaker_embeddings`, `speaker_scope`, `yandex_tag` возвращает **только** `voice_signal.py` (где эти типы инкапсулированы) и комментарии-докстринги.
- ADR-0037 §2.5 не переоткрывается; в этом PR добавляется одна строка в его changelog: «§2.5 scope=speaker:<...>: с ADR-0106 в <...> подставляется биометрический UUID (было — Yandex tag)».

---

## 8. Тест-план

### 8.1 Acceptance-тест issue #2440 п.8 (главный интеграционный)

Расположение: `src/rob_box_harness/test/test_identity_seam_acceptance.py` (в `rob_box_harness`, а не `rob_box_voice`, потому что шов — harness-уровень).

```python
import pytest
from rob_box_harness.identity import Znakomyi, IdentitySeam
from rob_box_harness.identity.voice_adapter import VoiceIdentityAdapter
from rob_box_voice.utils.speaker_embeddings import SpeakerDatabase
from rob_box_harness.memory.base import InMemoryStore


@pytest.mark.asyncio
async def test_resolve_same_person_different_yandex_tag(tmp_path):
    """Acceptance #2440: один человек под разными Yandex tag в разных
    сессиях резолвится в одного Знакомого, since_last_seen даёт
    реальный интервал.

    Шаги:
    (a) Зарегистрировать голосовой профиль, resolve() под tag='0'.
    (b) note_seen() под tag='0'.
    (c) Пересоздать SpeakerTracker / dialogue_node (эмуляция рестарта).
    (d) Подать ТОТ ЖЕ эмбеддинг под ДРУГИМ Yandex tag='1'.
    (e) resolve() обязан вернуть тот же Znakomyi.id, что в (a).
    (f) since_last_seen() обязан вернуть положительный интервал
        (не None, не 0, не пересозданный first_seen).
    """
    db = SpeakerDatabase(tmp_path / "speakers.db")
    memory = InMemoryStore()
    seam = VoiceIdentityAdapter(db, memory)

    # (a) Первая сессия, tag='0'.
    emb = _make_synthetic_embedding(seed=42)
    sig_0 = VoiceSignal(embedding=emb, yandex_tag="0", confidence=0.95)
    z_0 = await seam.resolve(sig_0)
    assert z_0 is not None and z_0.is_new is True
    assert isinstance(z_0.id, str) and len(z_0.id) == 36

    # (b) note_seen.
    await seam.note_seen(z_0, now=1000.0)

    # (c) Эмуляция рестарта — новый seam, но та же БД (persistent).
    seam2 = VoiceIdentityAdapter(
        SpeakerDatabase(tmp_path / "speakers.db"), memory
    )

    # (d) Тот же эмбеддинг, другой tag — '1' (Yandex так делает).
    sig_1 = VoiceSignal(embedding=emb, yandex_tag="1", confidence=0.95)
    z_1 = await seam2.resolve(sig_1)

    # (e) Тот же Знакомый.
    assert z_1.id == z_0.id, (
        f"Expected resolve(tag='1') == resolve(tag='0')={z_0.id[:8]}, "
        f"got {z_1.id[:8]}. IdentitySeam не работает."
    )

    # (f) since_last_seen возвращает реальный интервал.
    interval = await seam2.since_last_seen(z_1, now=2000.0)
    assert interval is not None
    assert interval > 0, f"Expected positive interval, got {interval}"
    # dialog_count должен быть 1 (note_seen был 1 раз), а не сброшен.
    z_1_after = await seam2.resolve(sig_1)
    assert z_1_after.dialog_count == 1
```

**Что краснеет на текущем коде.**

- На текущем коде `resolve()` либо отсутствует (голос напрямую возвращает tag), либо возвращает разные id (если бы tag использовался как id). Тест упадёт на шаге (e) — `z_1.id != z_0.id`.
- Шаг (f) упадёт по-другому: даже если бы id совпали, `last_seen` лежит под `scope=speaker:0` (от первой сессии), а во второй сессии читается `scope=speaker:1` — там None.

### 8.2 Unit-тесты шва

`src/rob_box_harness/test/test_identity_seam.py`:

- `test_resolve_creates_new_znakomyi` — неизвестный embedding → `is_new=True`, persist в БД.
- `test_resolve_returns_existing` — повторный embedding → `is_new=False`, тот же id.
- `test_resolve_returns_none_on_empty_signal` — `embedding=None` или `yandex_tag=None` → `None`.
- `test_note_seen_updates_dialog_count_and_last_seen` — два вызова → `dialog_count=2`, `last_seen` обновлён.
- `test_since_last_seen_returns_none_for_unknown` — неизвестный Znakomyi (не было note_seen) → `None`.
- `test_since_last_seen_returns_interval` — note_seen(now=1000), since_last_seen(now=2500) → 1500.0.
- `test_merge_combines_both_layers` — два Znakomyi, merge → embeddings и facts перенесены в dst, src очищен.
- `test_merge_atomicity` — если `merge_speakers()` упал, `merge_speaker_facts()` НЕ выполняется (rollback через try/except в `identity.merge()`).

### 8.3 Regression-тест на усечение

`src/rob_box_voice/test/test_speaker_id_truncation.py` (новый):

- `test_system_context_full_uuid` — после `dialogue_node._build_dynamic_system_context` `<speaker_id>...</speaker_id>` содержит **полный** UUID (36 символов), не 8.
- `test_mcp_tool_description_full_uuid` — описание параметра `speaker_id` в `memory.py` содержит фразу «полный UUID» / «36 символов».

### 8.4 Regression-тест на пороги

Расширение `src/rob_box_voice/test/test_yaml_param_consistency.py`:

```python
def test_threshold_values_match_code():
    """ADR-0106: проверка ЗНАЧЕНИЙ порогов, не только имён."""
    code_identify = speaker_embeddings.IDENTIFY_THRESHOLD
    code_register = speaker_embeddings.REGISTER_MATCH_THRESHOLD
    for yaml_path in [
        "src/rob_box_voice/config/speaker_id_node.yaml",
        "docker/vision/config/voice_assistant/speaker_id_node.yaml",
    ]:
        cfg = yaml.safe_load(read_file(yaml_path))
        params = cfg["speaker_id_node"]["ros__parameters"]
        assert params.get("identify_threshold") == pytest.approx(code_identify), (
            f"{yaml_path}: identify_threshold {params.get('identify_threshold')} "
            f"!= code {code_identify}"
        )
        assert params.get("register_match_threshold") == pytest.approx(code_register), (
            f"{yaml_path}: register_match_threshold {params.get('register_match_threshold')} "
            f"!= code {code_register}"
        )
```

### 8.5 Тесты, которые НЕ пишутся в этом PR

- Arbitration голос + лицо (появится с FaceIdentityAdapter в Phase 2).
- Multi-modal association table (как Face находит существующий Voice Znakomyi по cross-modal подсказке).

---

## 9. Связанные ADR и issues

| Ссылка | Связь |
|---|---|
| ADR-0018 (честный FAIL) | Родитель. Шов строится так, чтобы тесты могли **честно падать** на старом коде и **честно зеленеть** на новом. |
| ADR-0013 (incremental delivery) | Родитель. Миграция — пошаговая (§6), каждый шаг отдельно коммитится и тестируется. |
| ADR-0080 (eight-seams) | Родитель. Шов «Знакомый» — кандидат в девятый шов, если Шифу согласится; иначе остаётся внутренней границей harness. |
| ADR-0093 (ring-буфер неизвестных) | Сосед. Ring даёт transient_label для UI; шов даёт persistent id для backend. Они не конфликтуют — ring может жить как pre-resolve слой. |
| ADR-0037 (memory layers) | Не переоткрывается; меняется **что** подставляется в `speaker:<...>`. |
| ADR-0055 (БД consolidation) | Не пересекается: «где хранить» vs «кто есть спикер». |
| ADR-0089 Phase 2 (face) | Этот ADR — контракт, который Phase 2 обязана соблюсти (§5.3). |
| issue #2440 | Этот ADR — прямой ответ. 8 acceptance пунктов = §7. |
| issue #2348 (калибровка + merge) | Закрывает практическую сторону дефектов A/C; этот ADR закрывает **архитектурный** корень. Правки по #2348 (пороги, merge_speaker_facts) включаются в тот же PR. |
| issue #1077 (speaker profiles) | Родительский контекст — профиль спикера; этот ADR переводит его на стабильный ключ. |
| issue #1770 (memory_context speaker_id) | Решается побочно: после §6.3 `<system_context>/<speaker_id>` содержит полный UUID, поиск работает. |
| `migrations/009_voice_memory_speaker_id.sql` | Колонка `voice_facts.speaker_id` уже ждёт биометрический UUID; этот ADR наконец даёт ей правильный источник. |

---

## 10. Открытые вопросы и риски

### 10.1 Вопрос: что делать с историческими `scope=speaker:0`, `scope=speaker:1`?

**Решение (принято в этом ADR):** ничего автоматически. Содержимое — `{first_seen, last_seen, dialog_count}` за пару сессий, долгосрочной ценности нет. Через retention-цикл ADR-0037 (TTL на short-term слое) факты уйдут сами. Если всплывёт, что там были ценные `Fact(key="имя", value="Денис")` — это всплывёт как отдельная задача с explicit migration SQL (см. §6.1).

**Альтернатива (отклонена):** миграция «best-effort» при деплое — перебрать все scope=`speaker:<digit>` и переписать в `speaker:<new_uuid>`. **Почему нет:** нет маппинга `tag → uuid` для исторических сессий (tag был per-session, не сохранялся); без маппинга миграция превратится в спам-merge и/или потерю данных.

### 10.2 Вопрос: ADR-0055 + этот ADR — порядок?

**Решение:** этот ADR первым. ADR-0055 про физическое расположение файлов БД, и ему всё равно, какой id в столбце — стабильный или нет. Наоборот, после ADR-0106 ADR-0055 становится проще: переносить нужно одно key-пространство (биометрический UUID), а не три.

### 10.3 Вопрос: arbitration голос + лицо — как именно?

**Не в этом ADR.** Это отдельная карточка, которая появится, когда оба адаптера существуют. Ожидаемая схема:

```
                ┌─────────────────────┐
                │  Multi-modal         │
                │  arbiter (Phase 2.5) │
                │                     │
                │  signal_v + signal_f │
                │       │             │
                │       ▼             │
                │  cross-modal match? │
                │   ┌───┴───┐         │
                │  yes     no        │
                │   │       │         │
                │   ▼       ▼         │
                │ merge  conflict     │
                │ into   → human      │
                │ existing review     │
                └─────────────────────┘
```

Шов гарантирует только, что оба адаптера возвращают `Znakomyi` — arbiter оркестрирует их.

### 10.4 Риск: `SpeakerDatabase` уже зеленеет по тестам #2348 — можно ли её НЕ трогать?

**Да.** `VoiceIdentityAdapter` оборачивает, не модифицирует. Если в реализации окажется, что нужны новые методы в `SpeakerDatabase` (например, `get_speaker_with_profile()`), они добавляются **минимально**, с тестами, и ADR-0106 остаётся «обёрткой», а не «переписыванием».

### 10.5 Риск: `Znakomyi.id` — UUID, а будущий RFID-адаптер захочет integer

**Решение (на сейчас):** жёсткая проверка `UUID_RE` в `__post_init__`. Когда появится реальный адаптер с другим форматом — ослабим до `non-empty str` с переходным периодом.

### 10.6 Риск: `merge_speaker_facts` сейчас в `rob_box_harness`, а `VoiceIdentityAdapter` импортирует и его, и `SpeakerDatabase` — не circular?

**Нет.** Граф зависимостей: `rob_box_harness.identity.voice_adapter` → `rob_box_voice.utils.speaker_embeddings` (нисходящая), `rob_box_harness.identity.voice_adapter` → `rob_box_harness.memory.base` (тот же пакет). `rob_box_harness.memory.base` НЕ импортирует `rob_box_voice` — проверено (`grep -r "from rob_box_voice" src/rob_box_harness/` пусто). Circular imports нет.

### 10.7 Риск: scope=speaker:<uuid> начинает жить очень долго

**Принято.** ADR-0037 уже определяет retention для этого слоя (long-term с TTL по факту использования). Шов не вводит новых retention-политик.

---

## 11. План реализации (фазы → карточки)

Этот ADR — **дизайн**. Реализация — отдельные карточки, последовательность определяется порядком шагов §6. Оценка снизу — по принципу «Pragmatic Clean Architecture», не YAGNI, но и не over-engineering.

| Фаза | Карточка | Что делает | Оценка |
|---|---|---|---|
| **1. Skeleton** | `t_xxx: feat(identity): skeleton модуля IdentitySeam + Znakomyi + Protocol` | Файлы `identity/{__init__.py,types.py,seam.py,voice_signal.py}`, без реальной логики; `VoiceIdentityAdapter` — заглушка, бросает `NotImplementedError`. Unit-тесты на dataclass. | 1 день |
| **2. Migration data layer** | `t_xxx: refactor(memory): speaker_scope(tag) → speaker_scope(speaker_id) с back-compat алиасом` | `memory/base.py:454-560`, переименование параметра + алиас. Тесты существующих `test_speaker_memory.py` продолжают зеленеть (back-compat). | 0.5 дня |
| **3. Calibration atomic** | `t_xxx: fix(voice): пороги 0.75 → 0.72 в YAML + register_match_threshold + value-check тест` | §6.6 + §6.7 + расширение `test_yaml_param_consistency.py`. Это можно сделать **отдельным PR**, не дожидаясь шва — он самодостаточен. | 0.5 дня |
| **4. Wire voice** | `t_xxx: feat(identity): VoiceIdentityAdapter + dialogue_node._handle_speaker_turn → znakomyi` | §5.1 + §6.2. `VoiceIdentityAdapter` реальный, `dialogue_node.py:3302` → `note_seen(znakomyi)`. Acceptance-тест §8.1. | 2-3 дня |
| **5. Wire merge** | `t_xxx: feat(identity): identity.merge() + _on_merge_request через шов` | §6.5. Дефект C закрывается. | 0.5 дня |
| **6. Wire consumer** | `t_xxx: fix(voice): убрать усечение speaker_id в system-context + MCP-тулы` | §6.3 + §6.4. Дефект B закрывается. | 0.5 дня |
| **7. ADR-0089 Phase 2 prep** | `t_xxx: docs(adr): ADR-0089:300 ссылка на IdentitySeam как контракт arbitration` | §5.3, правка одной строки в ADR-0089 + новая карточка на FaceIdentityAdapter. | 0.5 дня |

**Суммарно:** ~5-6 дней разработки + 1 день ревью Шифу + 1 день на разрешение конфликтов в CI. **8 рабочих дней** до полного закрытия issue #2440.

**Рекомендация по выкатке.** Фазы 2, 3 можно выкатить **независимо** от 1 — это не требует шва. Фазы 4, 5, 6 зависят от 1. Фаза 7 — после 4 (когда шов реально используется в проде хотя бы одним адаптером).

**Рекомендация по CI.** На каждой фазе — свой PR в develop, каждый со своим набором CI-чеков (Python lint, shell, YAML, Docker compose). Acceptance-тест §8.1 добавляется в фазе 4 и **красный** до фазы 4 — это нормально (issue #2440 п.8 «тест краснеет на текущем коде»).

---

## 12. Что НЕ делается (anti-scope)

- Не переименовываем `SpeakerTracker` и не меняем его сигнатуру (`note_phrase(tag, duration_s)` остаётся). Это ортогональная задача.
- Не переносим `dialogue_node` в новый модуль — рефакторинг структуры выходит за рамки identity-шва.
- Не добавляем face-распознавание. ADR-0089 Phase 2 — отдельный проект.
- Не трогаем ADR-0037 и ADR-0055 (кроме changelog-строки в первом).
- Не вводим retention-политики для `scope=speaker:<uuid>` — действующие правила ADR-0037 работают как есть.
- Не делаем `transient_label → biometric_uuid` reverse-mapping (например, чтобы UI мог показать «Голос-1» для нового профиля). Это UX, не идентичность.
- Не меняем `mcp_server._on_speaker_result` — он уже сохраняет полный UUID в fallback, после ADR-0106 это станет основным путём.

---

## 13. Решение (резюме для Шифу)

**Принять.** Реализация — по фазам §11. Первый PR (фаза 3 — калибровка порогов) можно делать **сегодня** независимо от остального; остальные — после ревью этого ADR.

**Ключевые риски для Шифу:**

1. Исторические данные `scope=speaker:<yandex_tag>` остаются сиротами, но их содержимое — short-term профиль за 1-2 сессии, долгосрочной ценности нет (§10.1).
2. Шов требует дисциплины: всё, что **выходит** из `dialogue_node` / `_handle_speaker_turn` наружу, должно быть `Znakomyi`, не `tag`. Это культурный сдвиг, не только код-ревью.
3. Acceptance-тест §8.1 будет **красным** до фазы 4 — это by design (issue #2440 п.8), но CI увидит его как failed. Шифу нужно либо временно `xfail`-пометить, либо держать в отдельном PR, который мержится одновременно с фазой 4.

**Что этот ADR даёт проекту.** Один **стабильный** вопрос «кто этот человек» с одним **стабильным** ответом, поверх которого можно строить UX «давно не виделись», голос + лицо arbitration, и любые будущие биометрии (RFID, gait) — без повторения бага «три независимых профиля одного человека».

---

## 14. Changelog

- **2026-09-14** — Initial draft (Proposed), kanban `t_5986a91c`, привязан к issue #2440.
