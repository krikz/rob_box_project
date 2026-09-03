# ADR-0037: Слои памяти робота (working / short / long / episodic / procedural) и persistence policy

| Поле | Значение |
|---|---|
| Статус | **Proposed** (после merge PR в develop → Accepted) |
| Дата | 2026-08-31 |
| Автор | architect (Hermes Agent); карточка `t_6f5ddb67`, issue #1774 |
| Контекст | Баг: после рестарта DJ-сессии `Дровосек` всё ещё «активен», хотя юзер давно переключился на «сбацай джаз для Ивана». Root cause — turns/dialogue-history пишутся в SQLite (per-scope) и воспроизводятся при старте ноды; **DJ state** (`DJModeController.state`) живёт только в RAM и при рестарте сбрасывается, а turns из старой DJ-сессии остаются в `turns.scope=last_dialog` и контекст LLM по-прежнему видит DJ-промпты. |
| Затрагивает | `src/rob_box_harness/rob_box_harness/memory.py` (MemoryStore ABC — расширение контракта), `src/rob_box_harness/rob_box_harness/memory/sqlite_voice.py` (DDL + политика TTL), `src/rob_box_voice/rob_box_voice/core/dj_mode.py` (DJSession-scoped persistence), `src/rob_box_voice/rob_box_voice/dialogue_node.py` (scope resolver), ADR-0001 §2.4.3 (MemoryStore), issue #1770 (per-speaker memory isolation). |
| Родители | ADR-0001 §2.4.3 (MemoryStore port), ADR-0018 (honesty culture — никаких голословных «восстановилось»), ADR-0013 (incremental delivery — этот шаг описывает контракт, реализация — следующая карточка для `backend`). |
| Связанные | issue #1774 (эта задача), issue #1770 (memory_context не фильтрует по speaker_id), issue #1766 (backlog priority), issue #1684 (исследование архитектуры памяти), issue #1101 (farewell hook при DJ OFF). |

---

## TL;DR

Память робота — это **не один слой**, а пять ортогональных слоёв с разной persistence policy:

| # | Слой | Где живёт | Переживает рестарт? | Когда чистится |
|---|------|-----------|---------------------|----------------|
| 1 | **Working** | RAM процесса (per-turn переменные в `DialogCore.process_input`) | ❌ нет | сразу после `await llm.complete(...)` возвращает `DialogResult` |
| 2 | **Short-term** | SQLite `turns` table, `scope=last_dialog` / `chat:tg:<id>` | ✅ да | TTL 24ч по `created_at`; явная `clear_turns(scope)` при DJ OFF / roleplay end |
| 3 | **Long-term facts** | SQLite `facts` table, `scope=speaker:<tag>` / `user:<id>` | ✅ да | TTL per-fact через `metadata.ttl_seconds`; либо нет TTL (семантические факты) |
| 4 | **Episodic** | SQLite `turns` + новый `episodes` table, `scope=episode:<uuid>` | ✅ да | TTL 7д после `episode.closed_at`; сохраняется как «дневник сессии» |
| 5 | **Procedural** | SQLite `facts` (key=`tts_voice`, `language`, `dj_persona`) с TTL + singleton `event_profile` | ✅ частично | per-fact TTL; `event_profile` перезаписывается, история не хранится |

Ключевое решение: **`DJModeController.state` остаётся в RAM (working слой)**, НЕ персистируется между рестартами. Но **turns из DJ-сетов персистируются в `scope=episode:<uuid>`**, и при рестарте робот НЕ продолжает DJ-цикл (нет persona в event_profile → DJ неактивен). Это закрывает баг #1774.

Второе ключевое решение: **per-scope memory isolation уже корректна** (scope — это `turns.scope TEXT NOT NULL` в DDL, ADR-0001 §2.4.3 + code в `sqlite_voice.py:54`). Проблема была только в **lacking TTL и явных границ сессий**. ADR-0037 формализует «когда и что чистится» — это и есть persistence policy.

---

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем (баг #1774, юзерский сценарий)

1. Юзер: «робот, включи DJ Дровосек» → LLM вызывает `set_dj_mode(enabled=true, persona="Дровосек")` → `DJModeController.state.enabled=True, persona="Дровосек"`.
2. Робот диджеит ~10 минут. Все turn'ы пишутся в SQLite: `turns.scope="last_dialog"`, content начинается с `[DJ_AUTO ...] Ты Дровосек...`.
3. Юзер «задеплоил свежий код» (или ОС упала, или systemd перезапустил ноду после краша).
4. После рестарта: `DJModeController` создаётся заново, `state.enabled=False, persona=""`. **НО** `turns.scope="last_dialog"` всё ещё содержит DJ-промпты.
5. Юзер: «робот, сбацай джаз для Ивана» → LLM вызывает `load_recent(scope="last_dialog", limit=20)` → контекст забит «[DJ_AUTO переход #7 — Дровосек, тема: лесная хижина]».
6. LLM продолжает DJ-цикл «Дровосек», игнорируя новый intent. **Баг.**

`DJModeController` (см. `src/rob_box_voice/rob_box_voice/core/dj_mode.py`) хранит state **только в RAM** (`self.state = DJState(...)`, line 83). Это правильно для runtime-механики (тики, 5-секундный loop, persona). Но **отсутствие маркера «сессия закрыта» в БД** означает, что turns выглядят как обычный диалог — и `load_recent` поднимает их в контекст LLM.

### 1.2 Что уже есть в коде (as-is)

#### 1.2.1 `MemoryStore` ABC (`memory.py:118-258`)

```python
class MemoryStore(abc.ABC):
    # Conversation + facts (scoped)
    async def load_recent(self, scope: str, *, limit: int = 20) -> list[Turn]
    async def append_turn(self, scope: str, turn: Turn) -> None
    async def clear_turns(self, scope: str) -> int
    async def save_fact(self, scope: str, fact: Fact) -> None
    async def search_facts(self, scope: str, query: str, *, top_k: int = 5) -> list[Fact]
    async def list_facts(self, scope: str, *, limit: int = 50) -> list[Fact]
    # Waypoints (GLOBAL navigation memory)
    async def save_waypoint(self, name: str, x: float, y: float, theta: float = 0.0) -> None
    async def list_waypoints(self) -> list[Waypoint]
    async def delete_waypoint(self, name: str) -> bool
    async def clear_waypoints(self) -> int
    # FAQ (PER-EVENT knowledge base)
    async def load_faq(self, event_id: str, items: Iterable[Mapping[str, Any]]) -> int
    async def search_faq(self, event_id: str, query: str, *, limit: int = 5) -> list[FAQItem]
    # Event profile (SINGLETON active event context)
    async def set_event_profile(self, profile: Mapping[str, Any]) -> None
    async def get_event_profile(self) -> dict[str, Any] | None
```

#### 1.2.2 SQLiteVoiceMemory (`memory/sqlite_voice.py`)

```python
# DDL (line 51-104)
_TURNS_DDL:  CREATE TABLE turns  (id, scope TEXT NOT NULL, role, content, name, tool_call_id, metadata_json, created_at REAL)
_FACTS_DDL:  CREATE TABLE facts  (id, key TEXT NOT NULL, value, scope TEXT NOT NULL, metadata_json, created_at)
_WAYPOINTS_DDL: CREATE TABLE waypoints (name PK, x, y, theta, created_at, updated_at)
_FAQ_ITEMS_DDL: CREATE TABLE faq_items (id, event_id, question, answer, category, source, created_at)
_EVENT_PROFILE_DDL: CREATE TABLE event_profile (id PK CHECK(id=1), profile_json, updated_at)
```

**Сильные стороны as-is:**
- scope — это PK-часть индекса `idx_turns_scope (scope, created_at)` → изоляция per-scope работает на уровне SQL.
- `clear_turns(scope)` уже есть (`memory.py:137-142`, `sqlite_voice.py:330-336`) — но **никем не вызывается**.
- `metadata_json` уже есть на `turns` и `facts` — есть куда положить `ttl_seconds`, `episode_id`, `closed_at`.

**Слабые стороны as-is:**
- Нет TTL — `created_at` записан, но **ни один запрос не фильтрует по нему** (`load_recent` берёт только последние N, не старше N дней).
- `episode` не существует как концепция — DJ-сет и обычный диалог лежат в одном `scope=last_dialog`.
- `event_profile` singleton — только один на весь робот. Несколько «активных» сущностей (DJ + roleplay + обычный диалог) конфликтуют.
- `metadata_json` schema не описана — каждая реализация может писать что угодно.

### 1.3 Что просит юзер (issue #1774 acceptance)

Цитирую:

> Найти документ с описанием **как должна работать память на роботе**: долгосрочная (long-term), краткосрочная (short-term), рабочая (working) — и как они должны сосуществовать.

> **Юзер недоволен**: turns пишутся в базу и после рестарта они «фигачат все заново». Пример: робот был диджеем → юзер задеплоил свежий код → запустил робота → говорит «сбацай джаз для Ивана» → робот продолжает старую тему «диджей Дровосек».

> | Слой | Должно содержать | Что случилось сейчас |
> |---|---|---|
> | Working | «сбацай джаз для Ивана» | ✅ ok |
> | Short-term (dialog) | последний turn про Дровосек | Всё ещё в scope |
> | Long-term (facts) | «Денчик любит зелёный чай» (факт, не событие) | Нет (только в voice_facts) |
> | DJ state | «диджей Дровосек, тема: лесная хижина» | scope=last_dialog, воспроизвелось |
> | Event profile (singleton) | активный event | «Дровосек всё ещё активен» → **вот это! баг** |

ADR-0037 — это ответ на «как должна работать память». Реализация — отдельная карточка для `backend`.

### 1.4 Что НЕ нужно делать (anti-goals)

1. **НЕ** вводить event-sourcing / CQRS. Память не event log, а projection.
2. **НЕ** заменять SQLite на Redis/Postgres. SQLiteVoiceMemory уже работает, DDL WAL.
3. **НЕ** трогать формат turns/facts (это сломает обратную совместимость с `voice_memory.py` старой БД `voice_memory.db`).
4. **НЕ** персистить `DJModeController.state`. Это working слой, он должен умирать с процессом.
5. **НЕ** делать единый монолитный «memory manager». Распределённая ответственность через `MemoryStore` ABC уже правильно.

---

## 2. Принятое решение

### 2.1 Пять слоёв памяти (таблица контракта)

```
┌──────────────────────────────────────────────────────────────────────────┐
│  WORKING (in-RAM)                                                         │
│    • Текущий turn: text, speaker_tag, is_dj_auto, preclassified_event     │
│    • Dynamic system context (live XML snapshot, ADR-0001 §2.5)           │
│    • DJModeController.state (persona, theme, next_transition_at)         │
│    • DSM state (IDLE/LISTENING/DIALOGUE/SILENCED)                        │
│  ── Persistence: ❌ нет. Умирает с процессом. ──                          │
│  ── Cleanup: при return из process_input / tick() / event handler.       │
├──────────────────────────────────────────────────────────────────────────┤
│  SHORT-TERM (SQLite turns, scope=last_dialog)                            │
│    • Последние N=20 turns текущего диалога                                │
│    • scope="last_dialog" — общий для voice-ноды                          │
│    • scope=f"chat:tg:{chat_id}" — для telegram                            │
│  ── Persistence: ✅ SQLite `turns` table. ──                              │
│  ── TTL: 24ч по created_at. По истечении cron удаляет.                   │
│  ── Explicit cleanup: clear_turns("last_dialog") при DJ OFF / roleplay   │
│    end / voice_session_reset.                                             │
├──────────────────────────────────────────────────────────────────────────┤
│  LONG-TERM FACTS (SQLite facts, scope=speaker:<tag>)                     │
│    • Семантические факты: «любит зелёный чай», «зовут Денчик»            │
│    • scope=speaker:<tag> для voice, scope=user:<id> для будущего auth    │
│    • Профиль спикера: Fact(key="profile", value=dict)                    │
│  ── Persistence: ✅ SQLite `facts` table. ──                              │
│  ── TTL: per-fact через metadata.ttl_seconds; без TTL = «навсегда».     │
│  ── Explicit cleanup: clear_facts(scope, key=...).                       │
├──────────────────────────────────────────────────────────────────────────┤
│  EPISODIC (SQLite turns + новый episodes table)                          │
│    • Закрытые DJ-сеты, roleplay-сессии, event-сессии                     │
│    • scope=episode:<uuid> для turns в эпизоде                            │
│    • episodes: id PK, kind (dj|roleplay|event), persona, theme,          │
│      started_at, closed_at, metadata_json                                │
│  ── Persistence: ✅ SQLite `episodes` table. ──                              │
│  ── TTL: 7 дней после closed_at. Потом удаляются (или архивируются).    │
│  ── Cleanup: cron раз в час DELETE WHERE closed_at < now-7d.            │
│  ── При рестарте: НЕ восстанавливаются в working/short-term.            │
├──────────────────────────────────────────────────────────────────────────┤
│  PROCEDURAL (SQLite facts с TTL + event_profile singleton)              │
│    • Настройки: tts_voice, language, dj_persona, music_volume            │
│    • «Какой голос у робота», «какой таймзоне»                            │
│    • event_profile (singleton, id=1 CHECK) — текущий активный «контекст» │
│  ── Persistence: ✅ SQLite `facts` (key=tts_voice, etc.) + event_profile.│
│  ── TTL: per-fact. tts_voice — без TTL (юзер выбрал). music_volume — 7д.│
│  ── Cleanup: clear_facts(scope, key="music_volume") при смене.           │
│  ── event_profile перезаписывается атомарно; история не хранится.       │
└──────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Persistence policy per слой (таблица принятия решений)

| Слой | Restart-safe? | TTL default | TTL override | Cleanup trigger | Где хранится |
|------|---------------|-------------|--------------|-----------------|--------------|
| Working | ❌ | n/a | n/a | При exit из handler | RAM |
| Short-term | ✅ | 24ч | `metadata.ttl_seconds` | cron (раз в час) + `clear_turns(scope)` | `turns` table |
| Long-term facts | ✅ | ∞ (семантические) | `metadata.ttl_seconds` | `clear_facts(scope, key)` или user forget | `facts` table |
| Episodic | ✅ (read-only после закрытия) | 7д после closed_at | `metadata.ttl_days` | cron | `turns` (scope=episode:&lt;uuid&gt;) + `episodes` table |
| Procedural | ✅ | per-key | `metadata.ttl_seconds` | user change | `facts` table + `event_profile` table |
| FAQ | ✅ | ∞ (загружается при старте event'а) | n/a | при `load_faq` (replace) | `faq_items` table |
| Waypoints | ✅ | ∞ | n/a | `delete_waypoint` / `clear_waypoints` | `waypoints` table |

### 2.3 Глобальные vs scoped (per ADR-0001 §2.4.3)

MemoryStore API уже разделяет:

- **Scoped (per-actor / per-dialog)**: `load_recent(scope)`, `append_turn(scope)`, `save_fact(scope)`, `search_facts(scope)`, `list_facts(scope)`. Эти данные **изолированы** по scope-string.
- **Global (singleton)**: `set_event_profile` / `get_event_profile` (один на робота), `save_waypoint` / `list_waypoints` (навигация общая).
- **Per-event**: `load_faq(event_id)` / `search_faq(event_id)` — отдельный namespace.

Это правильно. ADR-0037 не меняет эту модель, но **формализует имена scope'ов** (см. §2.5).

### 2.4 DJ-mode и roleplay как scoped-сессия с маркером границы

**DJ-mode:**

```
Когда включён:
  self._memory.set_event_profile({
      "active_event": "dj_session",
      "episode_id": "<uuid>",       # новое поле
      "persona": "Дровосек",
      "theme": "лесная хижина",
      "started_at": iso8601,
      "next_transition_at": <float>,
  })

  При каждом tick (auto-transition) в DJ-mode:
    append_turn(scope=f"episode:{uuid}", turn=Turn(...))

При DJ OFF / auto-stop:
  self._memory.set_event_profile({})     # очистить active_event
  self._memory.update_episode(uuid, closed_at=now())   # пометить эпизод закрытым
  clear_turns(scope="last_dialog")       # НЕ подмешивать DJ-промпты в обычный диалог
```

**Ключевое:** при рестарте ноды `set_event_profile` остаётся в SQLite (последнее значение), НО `DJModeController.state` создаётся пустым. **Робот НЕ продолжает DJ-цикл**, потому что persona/theme — в working layer (DJModeController.state), а не в event_profile. Если юзер хочет продолжить — нужно явно `set_dj_mode(enabled=true, persona=...)` снова.

**Roleplay:** симметрично DJ — отдельный `episode_id` в `episodes` table, `kind="roleplay"`, `persona`, `scenario`. При рестарте — roleplay persona в working слое (отдельный контроллер), turns — в episodic.

### 2.5 Стандартизированные scope-имена

```python
# Из memory.py + speaker_profiles.py:
SPEAKER_SCOPE_PREFIX = "speaker:"        # voice: «Денчик» по speaker_tag
USER_SCOPE_PREFIX    = "user:"            # future auth
DIALOG_SCOPE         = "last_dialog"      # voice: текущий диалог
TELEGRAM_CHAT_SCOPE  = "chat:tg:"         # telegram: один чат

# Новые (ADR-0037):
EPISODE_SCOPE_PREFIX = "episode:"         # episodic: закрытые DJ-сеты / roleplay
PROCEDURAL_SCOPE     = "procedural"       # ключ факта = настройка (tts_voice и т.д.)
DJ_EPISODE_KIND      = "dj"
ROLEPLAY_EPISODE_KIND = "roleplay"
EVENT_EPISODE_KIND   = "event"            # для future events
```

### 2.6 Расширение `MemoryStore` ABC (минимальное)

Добавляем **только** методы, без которых ADR не имеет смысла:

```python
class MemoryStore(abc.ABC):
    # ... существующие методы ...

    # ── Episodes (ADR-0037 §2.4) ─────────────────────────────────────
    @abc.abstractmethod
    async def create_episode(
        self,
        kind: str,                   # "dj" | "roleplay" | "event"
        *,
        metadata: Mapping[str, Any],
    ) -> str:
        """Create a new episode, return its ``episode_id`` (UUID4 string)."""

    @abc.abstractmethod
    async def close_episode(self, episode_id: str) -> None:
        """Mark episode closed (set closed_at = now)."""

    @abc.abstractmethod
    async def get_active_episode(self, kind: str) -> dict | None:
        """Return the active episode of given kind, or None if none active."""

    # ── TTL (ADR-0037 §2.2) ─────────────────────────────────────────
    @abc.abstractmethod
    async def prune_expired(self, *, now: float | None = None) -> int:
        """Delete turns/facts whose ``metadata.ttl_seconds`` has elapsed.
        Returns the number of rows removed. Called by cron hourly."""

    # ── Optional helpers (default implementations in ABC) ──────────────
    async def clear_facts(self, scope: str, *, key: str | None = None) -> int:
        """Delete facts in scope; if key given, only that key."""
        raise NotImplementedError
```

`InMemoryStore` и `SQLiteVoiceMemory` обязаны реализовать все абстрактные методы. `prune_expired` — cron entry-point, не в hot path.

### 2.7 Per-scope memory isolation (закрывает #1770)

Уже корректна в `SQLiteVoiceMemory`:
- `turns.scope TEXT NOT NULL` — PK индекса `idx_turns_scope`.
- `load_recent(scope)` → `WHERE scope = ?` (`sqlite_voice.py:253`).
- `save_fact(scope, fact)` → `INSERT INTO facts (...) VALUES (..., scope, ...)` (`sqlite_voice.py:367`).

**Правило:** **любой** query в `MemoryStore` принимает `scope` и фильтрует по нему. Нет query «all facts» / «all turns» на проде.

Исключение — `event_profile` (singleton) и `waypoints` (global). Они явно marked as global в ABC docstring (`memory.py:32-34`).

Issue #1770 (memory_context не фильтрует по speaker_id) — это **нарушение этого правила** в `voice_memory.py` (старая БД, не MemoryStore). ADR-0037 не покрывает `voice_memory.py` напрямую (отдельный компонент), но добавляет §2.9 в acceptance для backend-воркера.

### 2.8 Пример: что происходит при рестарте

| Шаг | Что в SQLite | Что в RAM после restart | Что делает DialogCore |
|-----|--------------|-------------------------|------------------------|
| 1. Turn «привет» от speaker `tag=42` | `turns(scope="last_dialog", role="user", content="привет")` | (нет — RAM чист) | `load_recent("last_dialog", limit=20)` поднимает «привет» |
| 2. DJ tick «[DJ_AUTO переход #7] Дровосек» | `turns(scope="episode:dj-uuid-1", ...)` | DJModeController.state пуст | `get_active_episode("dj")` → None (closed) → DJ не активен |
| 3. Юзер: «сбацай джаз для Ивана» | `turns(scope="last_dialog", ...)` новый turn | DJModeController.state пуст | LLM вызывает `set_dj_mode(enabled=true, persona="Джаз-Иван")` → создаётся `episode_id=dj-uuid-2`, persona в working |
| 4. LLM видит контекст | `load_recent("last_dialog", limit=20)` = `[привет, ..., сбацай джаз]` (DJ-промпты из episode НЕ попадают, потому что их scope=`episode:dj-uuid-1`, не `last_dialog`) | DJ persona в working | LLM понимает «новый DJ-цикл, не продолжать старый» |

**Чистый результат:** turns из старого DJ-сета остаются в `turns` (можно прочитать через `get_episode_history(episode_id)`), но НЕ подмешиваются в LLM-context для нового диалога. Робот не «фигачит всё заново».

### 2.9 Acceptance criteria (для backend-воркера по реализации)

#### 2.9.1 Контрактные (до merge)

- [ ] `MemoryStore` ABC расширен методами `create_episode` / `close_episode` / `get_active_episode` / `prune_expired` / `clear_facts`.
- [ ] `InMemoryStore` (`memory.py:261+`) реализует все 5 новых методов.
- [ ] `SQLiteVoiceMemory` (`memory/sqlite_voice.py`) реализует все 5 новых методов + DDL для новой таблицы `episodes`.
- [ ] DDL миграция: `CREATE TABLE IF NOT EXISTS episodes (id TEXT PK, kind TEXT, persona TEXT, started_at REAL, closed_at REAL, metadata_json TEXT)`.
- [ ] `prune_expired` обновляет оба слоя: `turns` (по `metadata.ttl_seconds`) и `facts` (по `metadata.ttl_seconds`).

#### 2.9.2 Тесты (pytest, до merge)

- [ ] `test_memory.py`: новые тесты для `create_episode` / `close_episode` / `prune_expired` (TTL на разных слоях).
- [ ] `test_sqlite_voice_memory.py`: интеграционные тесты для `episodes` table на реальном SQLite.
- [ ] `test_per_scope_isolation.py`: НЕ существующий файл; добавить тест «scope=speaker:42 facts не видны из scope=user:7».
- [ ] `test_prune_expired_cron.py`: добавлен, проверяет «через 25ч turns с metadata.ttl_seconds=86400 удаляются».
- [ ] `bash scripts/agent_flow/tests/test_adr_collision.sh` → exit 0 (ADR-0037 уникален, проверка по ADR-0030).

#### 2.9.3 Поведенческие (e2e, через 1 раунд после merge)

- [ ] Голос: «включи DJ Дровосек» → работает как раньше (DJ играет, persona в event_profile).
- [ ] `systemctl restart rob_box_voice` → робот НЕ продолжает DJ-цикл (event_profile пуст после close_episode).
- [ ] Голос: «как зовут Денчика?» → LLM видит факт из `facts(scope="speaker:42", key="name", value="Денчик")`.
- [ ] Голос: «сбацай джаз» после DJ-рестарта → LLM не видит «Дровосек» в контексте (turns из episode НЕ подмешаны).
- [ ] TTL: turn с `metadata.ttl_seconds=1` удаляется при следующем `prune_expired`.

#### 2.9.4 Production gates

- [ ] `pytest -v src/rob_box_harness/test/test_memory.py src/rob_box_harness/test/test_sqlite_voice_memory.py` → exit 0.
- [ ] `pytest -v src/rob_box_harness/test/test_per_scope_isolation.py src/rob_box_harness/test/test_prune_expired_cron.py` → exit 0 (новые).
- [ ] `mypy --strict src/rob_box_harness/rob_box_harness/memory.py src/rob_box_harness/rob_box_harness/memory/sqlite_voice.py` → exit 0.
- [ ] `bash scripts/agent_flow/tests/test_adr_collision.sh` → exit 0 (ADR-0037 уникален).

---

## 3. Альтернативы, которые мы отвергли

| Альтернатива | Почему отвергли |
|---|---|
| **Event sourcing** (turns = append-only log, state = projection) | Overkill для памяти робота. Не даёт ничего, чего не даёт SQLite с `metadata_json` + cron-prune. |
| **Redis/Postgres вместо SQLite** | SQLiteVoiceMemory уже работает, WAL, ~5K turns в production. Redis — ещё одна зависимость, ещё один backup. |
| **Персистить `DJModeController.state` в БД** | DJ persona — это working layer, не конфиг. Если юзер хочет продолжить — он скажет. «Восстанавливать DJ автоматически после рестарта» = silent magic, нарушает ADR-0018 (честный FAIL). |
| **Глобальный `episode_id` в каждом turn** | Сложнее SQL, нужен JOIN. Хватает `scope=episode:<uuid>` (уже работает с per-scope index). |
| **Один «event_profile» для всех режимов** (DJ+roleplay+обычный) | Singleton уже есть, но при DJ И roleplay одновременно конфликт. Решение: `event_profile` — только «текущий user-facing режим», `episodes` table — все эпизоды. |
| **TTL на уровне LLM (забывать в контексте)** | Хрупко (нет гарантий, что LLM «забудет»). TTL на уровне БД = физическое удаление, гарантия. |
| **Удалять episode turns сразу при close** | Юзер может захотеть посмотреть «что DJ играл вчера». Episodic = дневник. Удаляются через 7д cron'ом. |
| **Merge event_profile и episodes** | Разные lifecycle: event_profile = текущий «режим», перезаписывается; episodes = история, append-only. Объединение = путаница. |

---

## 4. Trade-offs

| Что получаем | Чем платим |
|---|---|
| Чёткая модель «что переживает рестарт, что нет» — баг #1774 закрывается root-cause | +5 методов в MemoryStore ABC, +1 таблица в SQLite |
| Per-scope isolation формализована правилами (issue #1770) | Каждый новый query в MemoryStore должен помнить про scope — review checklist |
| Episodic memory = дневник DJ-сетов / roleplay | TTL 7д — если юзер хочет хранить дольше, нужен явный «archive episode» (за scope ADR-0037) |
| TTL — физическая очистка, не «забывание в контексте» | Cron на каждый час — ещё одна cron-job (но уже есть десятки других) |
| `DJModeController.state` остаётся в RAM (быстро, no I/O) | После рестарта робот не «помнит», что был DJ — это by design, юзер должен явно перезапустить |
| `event_profile` singleton → одна точка для «активный режим» | Несколько одновременных режимов (DJ + roleplay) не лезут в singleton — решено через `episodes` table |
| Обратная совместимость: `load_recent` / `save_fact` не меняются | Старая `voice_memory.py` БД `voice_memory.db` остаётся отдельно; миграция не требуется |

---

## 5. Не делаем

1. **Не** персистим DJ/roleplay state между рестартами — это working слой. Если юзер хочет «авто-DJ после рестарта», это отдельный ADR про «boot-time persona».
2. **Не** вводим Redis. SQLite + индексы + cron-prune достаточно.
3. **Не** трогаем `voice_memory.py` (отдельный компонент). Связь с ним — через то, что `dialogue_node` для LLM-context использует `voice_memory.py` (старая БД, без speaker_id), а `MemoryStore` (новая) — для **per-scope facts и turns**. В перспективе — migration `voice_memory.db` → `SQLiteVoiceMemory`, но это ADR-0038+.
4. **Не** делаем «автоматическое удаление turn'ов по содержимому» (например, «забудь всё про Дровосек»). Удаление — только через явную команду (DJ OFF / roleplay end / user-initiated forget).
5. **Не** храним raw audio. Память = текст + факты + метаданные. Audio — отдельная subsystem (snapshot_store.py).

---

## 6. Ссылки

- **Карточка-источник**: `t_6f5ddb67` (эта работа).
- **Issue-источник**: #1774 (этот ADR), #1770 (per-speaker isolation), #1684 (исследование архитектуры памяти), #1766 (backlog priority), #1101 (DJ farewell hook).
- **Код (текущее состояние)**:
  - `src/rob_box_harness/rob_box_harness/memory.py:1-570` (ABC + InMemoryStore + speaker helpers)
  - `src/rob_box_harness/rob_box_harness/memory/sqlite_voice.py:1-644` (SQLiteVoiceMemory, DDL line 51-104)
  - `src/rob_box_harness/rob_box_harness/core/dialog_core.py:280-399` (DialogCore init — `memory` port)
  - `src/rob_box_harness/rob_box_harness/harnesses/telegram.py:624-643` (scope = `f"tg:{chat_id}"`)
  - `src/rob_box_voice/rob_box_voice/core/dj_mode.py:32-83` (DJState + DJModeController — working layer)
  - `src/rob_box_voice/rob_box_voice/dialogue_node.py:2441-2510` (`_handle_speaker_turn` — scope=`speaker:<tag>`)
  - `src/rob_box_voice/rob_box_voice/speaker_profiles.py:13-136` (`speaker_scope(tag)` helper)
- **ADR-связи**:
  - ADR-0001 §2.4.3 (MemoryStore port — этот ADR его расширяет)
  - ADR-0018 (честный FAIL лучше красивого PASS — никаких silent restore)
  - ADR-0013 (incremental delivery — этот шаг описывает контракт, реализация — следующая карточка для backend)
  - ADR-0030 (ADR numbering — этот = 0037, проверка через test_adr_collision)
- **CONTRIBUTING.md §2f** (ADR-review rules) — этот ADR следует формату MADR (Markdown Any Decision Record).

---

## 7. Следующие шаги

1. **architect (этот PR)** — коммитит ADR-0037, push в `z-{agent}/t_6f5ddb67-...`, открывает PR в `develop`.
2. **merge-gate** проверяет: ADR-номер 0037 уникален (`ADR-collision-guard` ADR-0030 / test_merge_gate_adr_collision.sh) — должно проходить.
3. **reviewer** (от Шифу) — approve / request changes.
4. **Шифу** мержит PR после green CI + review.
5. **backend (следующая карточка)** — реализует `create_episode` / `close_episode` / `prune_expired` + DDL миграцию + тесты per §2.9.
6. **Мониторинг**: первые 24ч после merge backend-PR следить за `prune_expired` cron — если удаляет > 1000 turns/час, возможно, TTL выставлен слишком короткий.