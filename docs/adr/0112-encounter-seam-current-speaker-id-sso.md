# ADR-0112 — `EncounterSeam.current_speaker_id()`: единственная точка чтения «кто сейчас» в шве «Встреча»

**Дата:** 2026-09-16
**Статус:** Accepted (planned — реализация в Phase 1 на следующем цикле компонентного ревью `rob_box_harness`)
**Автор:** architect worker (kanban t_cae53268, issue #2649)
**Тип:** architecture correction (расширение API существующего seam'а)
**Родители:** ADR-0080 (порт-шов, не shared domain type), ADR-0105 (in-process «Встреча»), ADR-0013 (инкрементальная поставка), ADR-0018 (capability-honest)
**Заменяет:** ad-hoc комментарии «цикл импортов → дублируем» в `mcp_server.py:601-614` и `tools/memory.py:22-45`
**Связанные:** issue #2649 (component review 2026-09-15), issue #2442 («Встреча»), ADR-0105 §3 «Контракт шва»

---

## 1. Контекст и проблема

### 1.1. Дубль, найденный при компонентном ревью `src/rob_box_mcp_tools` 2026-09-15

Одна и та же четырёхстрочная логика чтения `node._encounter_seam.current().who.id` воспроизведена **в двух местах основного кода** и **в двух местах тестов**:

| Файл | Строка | Что |
|---|---|---|
| `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py` | 601–614 | метод `MCPServer._current_encounter_speaker_id()` |
| `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/memory.py` | 22–45 | модульная функция `_current_encounter_speaker_id(node)` |
| `src/rob_box_mcp_tools/test/test_mcp_server_speaker_result.py` | 229–233 | дубль в `_StubNode._current_encounter_speaker_id` (тест) |
| `src/rob_box_mcp_tools/test/test_tools/test_memory_speaker_id.py` | 288 | фабрика `_FakeNode._encounter_seam = EncounterSeam(...)` (косвенно та же логика через `.current()`) |

Оба «оригинальных» места обосновывают дублирование в docstring'ах одной и той же фразой: *«не импортируем напрямую — цикл импортов»*. Это обоснование **ошибочно** для целевого исправления, см. §1.3.

### 1.2. Почему это структурная, а не косметическая проблема

Текущая семантика «кто сейчас говорит» живёт в **четырёх** файлах одновременно. Любая правка API «Встречи» (см. §3.2) пробежит по всем четырём, и ни один review-лист не поймает пропуск — комментарии в обоих местах явно фиксируют сознательное дублирование, и при code review это выглядит «by design».

Конкретные сценарии дрейфа (по сегодняшнему плану развития шва):

1. **Добавление нового поля в `Acquaintance`** (например, `display_name` — в ADR-0105 §7 open question). Если «speaker_id» начнёт читаться через `display_name` — оба места поменяются, а тестовые стабы — нет, и регрессия поедет.
2. **Изменение `EncounterSeam.current()` API** (например, на `current() -> Optional[EncounterView]` без `who` — при выделении `EncounterView` value-объекта). Два продублированных `.current().who.id` не отловят ошибку типов одинаково.
3. **Миграция на дисковый `IdentitySeam`** (`MemoryIdentitySeam` → `SqliteIdentitySeam` в рамках #2440). Это не сломает дубль, но любой новый слой между `who` и id (`since_last_seen`, таймауты) будет продублирован.
4. **Распространение шва** на `dialogue_node` (следующий потребитель после `mcp_server`, ADR-0105 §3). Сейчас дубль — два места, после распространения будет три-четыре.

### 1.3. Почему «цикл импортов» — не препятствие

Текущие импорты в обоих файлах:

```python
# mcp_server.py:31
from rob_box_harness.encounter import EncounterSeam, EncounterChannel
# (ниже, на 35+)
from .tools import (MemorySaveTool, MemorySearchTool, MemoryContextTool, ...)
```

```python
# tools/memory.py:19
from ..base import MCPTool, MCPToolParameter, MCPToolResult
# НЕ импортирует ни mcp_server, ни EncounterSeam (только через getattr на node)
```

`tools/memory.py` уже может импортировать `EncounterSeam` напрямую — `rob_box_harness.encounter` это общий пакет, от которого зависит `mcp_server`, а не наоборот. Добавление метода на `EncounterSeam` **не создаёт цикл**: импорт `from rob_box_harness.encounter import EncounterSeam` в `tools/memory.py` идёт «вниз» по слоям, не вверх.

Единственное, что реально делает `tools/memory.py` через `getattr(node, "_encounter_seam", None)` — это читает seam **по duck-typed контракту с узлом**, чтобы не зависеть от типа `MCPServer`. Это **корректное** решение проблемы «tools/memory не знает тип узла», и оно должно остаться. Исправление ниже не убирает `getattr` — оно убирает `.current().who.id` после него.

---

## 2. Решение

### 2.1. Добавить `EncounterSeam.current_speaker_id() -> Optional[str]` в шов

В `src/rob_box_harness/rob_box_harness/encounter/base.py`, рядом с методом `current()`:

```python
def current_speaker_id(self, *, now: Optional[float] = None) -> Optional[str]:
    """Acquaintance.id текущей Встречи, или ``None``.

    Convenience-обёртка над :meth:`current` для самого частого
    потребительского запроса — «кто сейчас говорит». Не делает I/O,
    делегирует в :meth:`current` (а значит наследует таймаут-логику).

    Используется :class:`rob_box_mcp_tools.MCPServer` и
    :mod:`rob_box_mcp_tools.tools.memory` вместо независимого
    дублирования ``self._encounter_seam.current().who.id`` — см.
    ADR-0112 §1.1 (issue #2649).

    :returns: ``Acquaintance.id`` текущей Встречи, или ``None`` если
        Встречи нет, или она истекла по таймауту, или ``who is None``
        (анонимное присутствие — issue #2442 §1.2).
    """
    current = self.current(now=now)
    if current is None or current.who is None:
        return None
    return current.who.id
```

**Минимальный дифф.** Один метод, ~12 строк (включая docstring), никаких изменений сигнатур существующих API.

### 2.2. Заменить оба дубля на вызов seam-метода

**`mcp_server.py:601-614`** (метод `MCPServer._current_encounter_speaker_id`):

```python
def _current_encounter_speaker_id(self) -> Optional[str]:
    """Issue #2442 — ``speaker_id`` текущей Встречи, или ``None``.

    Делегирует :meth:`EncounterSeam.current_speaker_id`. См. ADR-0112 —
    шов один, читателей больше одного, метод шва — единственная точка
    чтения ``Acquaintance.id``.
    """
    return self._encounter_seam.current_speaker_id()
```

**`tools/memory.py:22-45`** (модульная функция):

```python
def _current_encounter_speaker_id(node: object) -> Optional[str]:
    """Issue #2442 — единственный путь к «кто сейчас», вместо трёх копий.

    Тонкая обёртка над ``node._encounter_seam.current_speaker_id()``
    (ADR-0112): модуль ``tools/memory`` не импортирует :class:`MCPServer`
    напрямую (тот импортирует этот модуль — цикл), поэтому читает seam
    через ``getattr`` по контракту. Сам seam — общий пакет
    ``rob_box_harness.encounter``, импорт не циклит. Отсутствие шва на
    узле (старый fake-node в тестах, узел без ``speaker_id_enabled``)
    — молчаливый ``None``, как и раньше при отсутствии атрибута.
    """
    seam = getattr(node, "_encounter_seam", None)
    if seam is None:
        return None
    return seam.current_speaker_id()
```

### 2.3. Обновить тесты — убрать третий дубль

**`test_mcp_server_speaker_result.py:229-233`** (`_StubNode._current_encounter_speaker_id`):

```python
def _current_encounter_speaker_id(self) -> Optional[str]:
    return self._encounter_seam.current_speaker_id()
```

Тест **`test_memory_speaker_id.py:288`** уже использует `EncounterSeam` напрямую — менять не надо, кроме комментария (см. §3 touchpoint #4).

### 2.4. Что НЕ делаем (важно)

- **НЕ убираем `getattr(node, "_encounter_seam", None)` в `tools/memory.py`** — это не цикл импортов, а корректный duck-typed контракт с узлом (см. §1.3).
- **НЕ выносим `tools/memory.py` в подмодуль, импортирующий `mcp_server`** — это и есть тот самый цикл, от которого защищается текущая структура через `getattr`.
- **НЕ делаем seam-метод `@cached_property`** — `current()` имеет side-effect (сбрасывает `_encounter` на таймауте в `_current_locked`), и `current_speaker_id` должен наследовать это поведение, а не кешировать.
- **НЕ добавляем в шов ещё и `current_confidence()`, `current_channels()`, и т.п.** — YAGNI, дубль пока в одном месте; добавим по мере появления третьего потребителя.
- **НЕ переименовываем `Encounter.who.id` в `Encounter.who_id`** — это отдельный рефакторинг (см. ADR-0105 §7 open question), вне скоупа этой карточки.

---

## 3. Touchpoints

| # | Файл | Что меняется |
|---|---|---|
| 1 | `src/rob_box_harness/rob_box_harness/encounter/base.py` | **NEW**: метод `EncounterSeam.current_speaker_id()` — делегирует `current()` |
| 2 | `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py:601-614` | заменить тело `MCPServer._current_encounter_speaker_id` на вызов seam-метода |
| 3 | `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/memory.py:22-45` | заменить тело модульной `_current_encounter_speaker_id` на вызов seam-метода через getattr |
| 4 | `src/rob_box_mcp_tools/test/test_mcp_server_speaker_result.py:229-233` | заменить дубль в `_StubNode` на вызов seam-метода; комментарий обновить ссылкой на ADR-0112 |
| 5 | `src/rob_box_mcp_tools/test/test_tools/test_memory_speaker_id.py` | комментарий в `_FakeNode` дополнить ссылкой на `EncounterSeam.current_speaker_id` (кода менять не надо — фабрика уже работает через шов) |
| 6 | `docs/adr/0105-encounter-in-process-first.md` §3 «Контракт шва» | дополнить список операций: `current_speaker_id()` (read-only convenience над `current()`) |

**Out of scope** (явно): распространение seam-метода на `dialogue_node` (следующий потребитель, отдельная карточка — ADR-0105 §3 «оба читают один и тот же шов»).

---

## 4. Альтернативы, которые отвергли

### Альтернатива A: literal — оставить дубль, добавить только перекрёстные ссылки в комментариях

- **Плюсы**: zero diff в логике; «комментарии же объясняют».
- **Минусы**: ровно та проблема, которую диагностирует issue #2649 — *«комментарии явно фиксируют сознательное дублирование, но не отменяют того, что это один логический контракт в двух местах»*. Любая будущая правка шва всё равно пробежит по обоим, и ревью пропустит.
- **Вердикт**: ❌ отвергнуто — лечит симптом, не причину.

### Альтернатива B: добавить seam-метод, но оставить `tools/memory.py` зависимым от `mcp_server` через type-hint

- **Плюсы**: «правильная» типизация вместо `getattr`.
- **Минусы**: **создаёт цикл импортов**, от которого текущий код защищается через `getattr`. `mcp_server` импортирует `tools/memory` через `from .tools import (...)`, а обратный импорт `mcp_server.MCPServer` в `tools/memory.py` — это `ImportError` при первой попытке. Текущее решение через `getattr` — корректное.
- **Вердикт**: ❌ отвергнуто — вводит цикл, который ADR-0112 явно избегает.

### Альтернатива C: вынести «read Acquaintance.id» в free function в `rob_box_harness.encounter`

```python
# rob_box_harness/encounter/__init__.py
def current_speaker_id_of(seam: Optional[EncounterSeam]) -> Optional[str]:
    if seam is None:
        return None
    cur = seam.current()
    if cur is None or cur.who is None:
        return None
    return cur.who.id
```

- **Плюсы**: «метод без метода» — нет API-расширения класса, только утилита.
- **Минусы**: seam в API не упоминается → читатели не найдут эту утилиту через `EncounterSeam.<tab>`. Тесты (`test_mcp_server_speaker_result.py:229`) тоже не догадаются импортировать её, и останется дубль. Текущая карточка решает **четыре** места дубля, а не два.
- **Вердикт**: ❌ отвергнуто — функция не помогает в тестах, где seam уже есть как объект.

### Альтернатива D (выбрана): метод `EncounterSeam.current_speaker_id()`

- **Плюсы**:
  - Минимальный дифф (один метод, ~12 строк).
  - Seam расширяется органично — это тот же seam, который читают `dialogue_node` и `mcp_server`, и его API естественно включает «кто сейчас» как convenience.
  - Тесты тоже переходят на вызов seam-метода — все **четыре** места становятся одним контрактом.
  - Документация шва (`base.py:1-36`) уже фиксирует «шов отвечает на вопрос „встретились ли мы **сейчас**"» — добавить `current_speaker_id` естественно.
- **Минусы**: расширяет API seam'а на одну операцию. Но: это именно то, для чего seam и существует (ADR-0080 «порт-шов, не shared domain type»), плюс добавление read-only convenience — минимальный риск.
- **Вердикт**: ✅ принято.

---

## 5. Acceptance criteria

### Phase 1 (эта карточка)

- [ ] `EncounterSeam.current_speaker_id()` добавлен в `src/rob_box_harness/rob_box_harness/encounter/base.py`, делегирует `current()`, возвращает `Optional[str]`.
- [ ] `MCPServer._current_encounter_speaker_id` (mcp_server.py:601-614) заменён на одну строку `return self._encounter_seam.current_speaker_id()`.
- [ ] Модульная функция `_current_encounter_speaker_id` (tools/memory.py:22-45) заменена на одну строку `return seam.current_speaker_id()` после getattr-проверки.
- [ ] `_StubNode._current_encounter_speaker_id` (test_mcp_server_speaker_result.py:229-233) заменён на вызов seam-метода.
- [ ] Юнит-тесты `test_mcp_server_speaker_result.py` (8 кейсов на speaker_result fallback) — без изменений в ассертах, проходят.
- [ ] Юнит-тесты `test_tools/test_memory_speaker_id.py` (3 класса, 9+ кейсов) — без изменений в ассертах, проходят.
- [ ] CI зелёный по `src/rob_box_mcp_tools` (pytest).
- [ ] В `tools/memory.py:22-45` обновлён комментарий: убран аргумент «цикл импортов», оставлен только duck-typed контракт с узлом (см. §2.2).
- [ ] ADR-0105 §3 «Контракт шва» дополнен: список операций теперь включает `current_speaker_id()` как read-only convenience.

### Phase 2 (отдельная карточка, не в этом ADR)

- Распространение `current_speaker_id()` на `dialogue_node` (следующий потребитель `EncounterSeam`).
- Потенциально: выделение `EncounterView` value-объекта без `who.id` (ADR-0105 §7 open question #1).

---

## 6. Capability-honest режим (ADR-0018)

`EncounerSeam.current_speaker_id()` **не меняет capability-контракт шва**:

1. Нет новых внешних зависимостей (только `current()`, который уже есть).
2. Нет новых failure modes: если `current()` вернул `None` или `who is None` — `current_speaker_id()` тоже вернёт `None`, как и обе исходные копии.
3. Нет silent fallback: тесты на `who is None` и `current is None` остаются зелёными без изменений в ассертах.
4. Никаких новых ENV-флагов, launch-параметров или runtime-конфигурации.

Семантика **идентична** обеим копиям, которые заменяются — соответствие проверяется тем, что существующие тесты проходят без изменений в ассертах (acceptance criteria §5).

---

## 7. Open questions

1. **Добавлять ли `current_speaker_id()` как `@property`?** Сейчас `current()` принимает `now: Optional[float] = None` для тестируемости — это сигнатура, не свойство. `current_speaker_id` наследует сигнатуру, поэтому остаётся методом. Если когда-нибудь появится «real-time» вариант без параметра времени — пересмотреть.
   *Решение (предложение)*: оставить методом, как `current()`.

2. **Стоит ли добавить симметричный `current_speaker() -> Optional[Acquaintance]`?** Параллельный getter, чтобы потребители, которым нужен весь `Acquaintance` (например, для `display_name` в будущем), не лазили в `.current().who` снова.
   *Решение (предложение)*: НЕ в этой карточке — YAGNI. Когда появится конкретный потребитель `Acquaintance` (не только id), добавим отдельной фазой и пересмотрим ADR-0112.

3. **Когда обновлять ADR-0105 §3 «Контракт шва»?** Там сейчас перечислены только `current()` (read) и `observe()` (write). `current_speaker_id()` — третья операция шва. Это не breaking change, но список операций должен быть полным — touchpoint #6 в §3.

---

## 8. Change log

| Дата | Автор | Изменение |
|---|---|---|
| 2026-09-16 | architect (t_cae53268, issue #2649) | Initial ADR-0112. Принят план: `EncounterSeam.current_speaker_id()` как единственная точка чтения «кто сейчас». Дубль в `mcp_server.py` и `tools/memory.py` (плюс тестовый стаб) заменяется вызовом seam-метода. Phase 2 (распространение на `dialogue_node`) — отдельная карточка. |

*ADR-0112 расширяет API существующего seam'а (ADR-0105) без изменения архитектуры.*
