# ADR-0055: единая БД памяти — миграция `/data/voice_memory.db` → `/data/harness_voice.db` с разделением namespace на агента

| Поле | Значение |
|---|---|
| Статус | Proposed (после merge → Accepted) |
| Дата | 2026-09-07 |
| Автор | architect (Hermes Agent); карточка `t_7a03364a`, issue #2000 |
| Контекст | На роботе сейчас живут две SQLite-БД на одном `/data/` томе: `/data/voice_memory.db` (MCP-инструменты: `voice_turns`, `voice_facts`, `voice_turns_fts`, `voice_memory_meta`, плюс `waypoints` через `WaypointStore`, плюс `music_tracks` через `MusicLibrary`) и `/data/harness_voice.db` (диалоговая нода через `SQLiteVoiceMemory`: `facts`, `waypoints`, `faq_items`, `event_profile` — turn-ы НЕ персистятся по директиве Шифу 02.09.2026, см. `memory/base.py:3-7`). Обе БД реально активны и пишут одни и те же домены (waypoints, facts) в разные файлы — что и зафиксировано как «две живые БД, схемы конфликтуют» в `src/rob_box_voice/config/dialogue_node.yaml:97-103`. |
| Затрагивает | `src/rob_box_voice/rob_box_voice/core/voice_memory.py` (старый MCP-стор — будет deprecated, но не удалён), `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py:113,668,957,1012` (использует `VoiceMemory` через `VOICE_MEMORY_DB_PATH`), `src/rob_box_mcp_tools/rob_box_mcp_tools/waypoint_store.py` (использует ту же БД через `VOICE_MEMORY_DB_PATH`), `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/music.py:2981-2993` (music library на той же БД), `src/rob_box_harness/rob_box_harness/memory/sqlite_voice.py` (новый стор), `migrations/002_voice_memory.sql`, `migrations/009_voice_memory_speaker_id.sql`, `docker/vision/...` (volume mount). |
| Родители | ADR-0037 (слои памяти, persistence policy; §5.3 явно отложил эту миграцию), ADR-0018 (честный FAIL), ADR-0013 (incremental delivery), ADR-0051 (operator/agent arbiter split). |
| Связанные | issue #2000, #1992 (operator-agent 05a — wake word / VAD), карточка `t_XXXX` (шаг 03 `AgentCore` namespace), карточка `t_6f5ddb67` (ADR-0037), карточка `t_d058dc6f` (operator-agent 06 — `/dialogue/control` пауза). |

---

## TL;DR

Привести робота к **одной** SQLite-БД памяти на смонтированном томе (`/data/harness_voice.db`), в которую пишут **и** диалоговая нода (через `SQLiteVoiceMemory`), **и** MCP-инструменты (через новый адаптер поверх `SQLiteVoiceMemory`), с разделением пространства через существующие поля (`scope`, `metadata_json`) до тех пор, пока шаг 03 (`AgentCore` — память namespace) не зафиксирует schema-колонку `agent`. Этот ADR:

1. **Останавливает удвоение схем**: сейчас `voice_memory.db.waypoints(name PK, ...)` ≠ `harness_voice.db.waypoints(name PK, ...)` (одинаковые DDL — но разные файлы, нет синхронизации).
2. **Унифицирует backup**: одна БД на `/data/` томе, не две.
3. **Устраняет silent dual-write** — `WaypointStore` (MCP) пишет в `voice_memory.db`, диалоговая нода — в `harness_voice.db`. Если юзер создаёт waypoint через голос, а потом читает через MCP-инструмент — пусто (это баг, который никто не видел, потому что никто не пробовал).
4. **Готовит почву под agent-namespace** (шаг 03): колонка `agent TEXT` появится одной миграцией сразу в одной таблице.

**Что НЕ делаем в этой карточке** (явно):
- **НЕ** переносим данные `voice_memory.db` → `harness_voice.db`. ADR-0037 + старый handoff Шифу (30.08, Q3): «накопленную БД можно просто удалить, миграция не нужна». Содержимое `voice_memory.db` тестовое (85 turn-ов, 1 fact — судя по `docker/vision/test/scenario_runner/voice_memory.db`, на проде может быть больше, но ценности для переноса нет).
- **НЕ** вводим `namespace` колонку (ждём шаг 03).
- **НЕ** останавливаем диалоговую ноду на проде. Скрипт — **dry-run-by-default**, явный `--apply` для продакшн-наката.

---

## 1. Контекст и бизнес-проблема

### 1.1 Что есть сейчас (as-is, проверено `grep` в `src/`)

**`/data/voice_memory.db`** пишется:
- `mcp_server.py:113,668,957,1012` через `VoiceMemory` (класс из `rob_box_voice.core.voice_memory`):
  - `voice_turns` (id, session_id, role, content, timestamp, speaker_id) — turn-ы
  - `voice_facts` (id, fact, category, created_at, updated_at, speaker_id) — facts
  - `voice_turns_fts` (FTS5 индекс)
  - `voice_memory_meta` (служебная)
- `waypoint_store.py:66` через `WaypointStore` → пишет в `voice_memory.db`:
  - `waypoints` (name, x, y, theta, created_at, updated_at, **`map_id NOT NULL`** FK на `maps`)
- `tools/music.py:2981-2993` через `MusicLibrary` → пишет в `voice_memory.db`:
  - `music_tracks` + `music_*` таблицы

**`/data/harness_voice.db`** пишется:
- `dialogue_node.yaml:104` через `SQLiteVoiceMemory` (`memory/sqlite_voice.py`):
  - `facts` (id, key, value, scope, metadata_json, created_at)
  - `waypoints` (name PK, x, y, theta, created_at, updated_at) — **другая схема, без `map_id`**
  - `faq_items` (id, event_id, question, answer, category, source, created_at)
  - `event_profile` (singleton)
  - **НЕТ таблицы `turns`** — Шифу директива 02.09.2026 (см. `memory/base.py:5-7`: «Conversation turns are NOT persisted»).

**Конфликт зафиксирован в коде** (`src/rob_box_voice/config/dialogue_node.yaml:97-103`, цитата):

> «Это НЕ `/data/voice_memory.db`: там живёт VoiceMemory из mcp_server, и схемы конфликтуют — у harness'а `waypoints.name` PRIMARY KEY, у VoiceMemory `waypoints` с `map_id NOT NULL` и FK на maps; `faq_items` расходятся так же (created_at vs indexed_at). `CREATE TABLE IF NOT EXISTS` промолчал бы, а вставки бы падали. Два стора остаются двумя сторами — но оба переживают рестарт.»

### 1.2 Что просит карточка #2000 (цитата)

> «Миграция `/data/voice_memory.db` → `/data/harness_voice.db` с namespace на агента, скрипт в `migrations/`. Сейчас две живые БД: `harness_voice.db` пишет диалог, `voice_memory.db` пишут MCP-инструменты (целевая §2.5, §12, инвариант 7). … Definition of Done: (1) после миграции `docker exec` показывает одну живую БД (`harness_voice.db`), `voice_memory.db` больше не пишется; (2) данные из `voice_memory.db` читаются под новым namespace (выборка SQL до/после совпадает по count); (3) MCP-инструменты пишут в ту же БД, что и диалог (тест: инструмент → строка видна в `harness_voice.db`).»

**Замечание:** пункт «harness_voice.db пишет диалог» — **не соответствует коду** (turn-ы туда не пишутся с 02.09.2026). Это либо устаревшая формулировка в карточке, либо Шифу имеет в виду что-то ещё. **Запрашиваем уточнение через issue-комментарий**, но в реализации исходим из фактического состояния: turn-ы пишутся **только** в `voice_memory.db` (через MCP-инструменты, но НЕ диалоговой нодой).

### 1.3 Чего карточка **не** учитывает

| Что | Почему важно | Что делаем |
|---|---|---|
| В схеме нет `agent`/`namespace` колонки | В `migrations/0..9` и в DDL `SQLiteVoiceMemory` нет такой колонки | Переходник через `metadata_json` + явный префикс scope до шага 03 |
| Шаг 03 (`AgentCore` — память namespace) ещё не закоммичен | Карточка явно зависит от него | Не делаем финальную миграцию данных; делаем только консолидацию файла |
| Схемы `waypoints` конфликтуют (map_id FK в MCP-версии, нет в harness) | INSERT с map_id упадёт на harness-DDL без колонки | Делаем адаптер, который **игнорирует `map_id`** (он NULL) или **мапит в metadata_json** |
| Старый handoff Шифу (Q3, 30.08): «накопленную БД можно просто удалить, миграция не нужна» | Прямо противоречит карточке | Запрашиваем решение Шифу через issue; **по умолчанию** миграция данных не выполняется |

### 1.4 Anti-goals

1. **НЕ** делать «merge обеих БД в один файл» через `cp`/`INSERT ... SELECT` под нагрузкой (race с пишущими сервисами).
2. **НЕ** менять схему `harness_voice.db` (DDL `sqlite_voice.py:52-91`) — это вне scope.
3. **НЕ** удалять `voice_memory.db` автоматически (Шифу решает вручную после визуальной проверки).
4. **НЕ** останавливать диалоговую ноду на проде. Скрипт — **dry-run by default**.
5. **НЕ** трогать `voice_memory.py` (класс остаётся для тестов и обратной совместимости — но никто из прода на него не ссылается после merge).
6. **НЕ** включать `music_tracks` в эту карточку — `MusicLibrary` пишет в `voice_memory.db` через свой DDL (см. `migrations/004_music_library.sql`, `006`, `007`), отдельная задача.

---

## 2. Принятое решение

### 2.1 Фаза 1 (этот ADR): **консолидация пути**

**Один файл** на проде — `/data/harness_voice.db`. **Один контейнер** пишет в него — `rob_box_voice`. **Один набор DDL** — от `SQLiteVoiceMemory` (`memory/sqlite_voice.py:52-91`).

Что меняется:

1. `mcp_server.py:113` импорт `VoiceMemory` → импорт нового адаптера `VoiceMemoryAdapter`.
2. `mcp_server.py:_init_voice_memory` (lines 957+) создаёт `VoiceMemoryAdapter(db_path="/data/harness_voice.db")` вместо `VoiceMemory`.
3. `mcp_server.py:save_turn`/`save_fact`/`search` продолжают работать (API адаптера = API `VoiceMemory`).
4. `waypoint_store.py:66` тоже пишет через адаптер (использует ту же БД).
5. `tools/music.py:2993` — **отдельная карточка** (вне scope).

**Что НЕ меняется в Фазе 1:**
- `voice_memory.py` остаётся в `rob_box_voice` (его никто не импортирует после merge, но класс не удаляем — обратная совместимость для тестов и rare-кейсов).
- `voice_memory.db` остаётся на томе **read-only**, никто туда не пишет (marker: `mtime` не обновляется).

### 2.2 Фаза 2 (отдельная карточка после шага 03): namespace + data-migration

После merge шага 03 (`AgentCore` — DDL `agent TEXT NOT NULL DEFAULT 'default'` в `SQLiteVoiceMemory`):
- `INSERT INTO turns (..., scope='personality', metadata_json={...}) SELECT ... FROM voice_memory.voice_turns` через `ATTACH DATABASE`.
- То же для `voice_facts` → `facts`.
- После verify Шифу — `rm /data/voice_memory.db`.

Эта карточка **только** реализует Фазу 1.

### 2.3 Адаптер `VoiceMemory → SQLiteVoiceMemory` (новый код)

Модуль: `src/rob_box_harness/rob_box_harness/memory/voice_memory_adapter.py`.

```python
"""Adapter: rob_box_voice.core.voice_memory.VoiceMemory API → MemoryStore.

Цель — дать MCP-серверу (mcp_server.py:113) и WaypointStore писать в
/data/harness_voice.db через существующий SQLiteVoiceMemory без
переписывания call-sites. ADR-0055 Фаза 1.

Не делает data-migration; только переключает путь.
"""
from __future__ import annotations
import asyncio
import json
import time
from typing import Any, Dict, List, Optional

from rob_box_harness.memory import Fact, Turn
from rob_box_harness.memory.sqlite_voice import SQLiteVoiceMemory


class VoiceMemoryAdapter:
    """API-совместимый фасад для MCP-инструментов.

    Совпадает по сигнатурам с rob_box_voice.core.voice_memory.VoiceMemory,
    но персистит всё в /data/harness_voice.db (та же БД, что и
    dialogue_node), а не в /data/voice_memory.db.

    Turn-ы пишутся в ``turns`` со ``scope="default"`` (Шифу директива
    02.09.2026: turn-ы НЕ персистятся в production, но для MCP-инструментов
    делаем исключение через явный scope="mcp:legacy" — это уже не in-RAM).
    """

    # Scope-префиксы до merge шага 03 (AgentCore namespace).
    TURN_SCOPE = "mcp:legacy"
    FACT_SCOPE = "mcp:legacy"

    def __init__(self, db_path: str) -> None:
        self._db = SQLiteVoiceMemory(db_path=db_path)
        # NOTE: SQLiteVoiceMemory.init() — async, вызываем при первом use
        # через ``_ensure_init``. Конструктор sync (как у VoiceMemory).

    async def _ensure_init(self) -> None:
        if not self._db._initialized:
            await self._db.init()

    def save_turn(self, role: str, content: str, *,
                  speaker_id: Optional[str] = None,
                  session_id: Optional[str] = None,
                  timestamp: Optional[float] = None) -> int:
        """Sync-обёртка над async append_turn.

        MCP-сервер однопоточный → блокировка ок. Создаём короткоживущий
        event loop на каждый вызов (используется редко).
        """
        ts = timestamp if timestamp is not None else time.time()
        meta = {}
        if session_id is not None:
            meta["legacy_session_id"] = session_id
        if speaker_id is not None:
            meta["legacy_speaker_id"] = speaker_id
        meta["legacy_source"] = "voice_memory"

        loop = asyncio.new_event_loop()
        try:
            return loop.run_until_complete(self._save_turn_async(
                role=role, content=content, ts=ts, meta=meta))
        finally:
            loop.close()

    async def _save_turn_async(self, *, role, content, ts, meta):
        await self._ensure_init()
        return await self._db.append_turn(
            scope=self.TURN_SCOPE,
            turn=Turn(role=role, content=content, metadata=meta),
            timestamp=ts,
        )

    def save_fact(self, fact: str, *, category: str = "general",
                  speaker_id: Optional[str] = None,
                  timestamp: Optional[float] = None) -> int:
        ts = timestamp if timestamp is not None else time.time()
        meta = {"legacy_speaker_id": speaker_id} if speaker_id else {}
        meta["legacy_source"] = "voice_memory"
        loop = asyncio.new_event_loop()
        try:
            return loop.run_until_complete(self._save_fact_async(
                fact=fact, category=category, ts=ts, meta=meta))
        finally:
            loop.close()

    async def _save_fact_async(self, *, fact, category, ts, meta):
        await self._ensure_init()
        return await self._db.save_fact(
            scope=self.FACT_SCOPE,
            fact=Fact(key=category, value=fact, tags=[category]),
        )

    def search(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """Compatibility: возвращает list of dicts, как VoiceMemory.search."""
        loop = asyncio.new_event_loop()
        try:
            return loop.run_until_complete(
                self._db.search(query=query, limit=limit)
            )
        finally:
            loop.close()

    def get_stats(self) -> Dict[str, Any]:
        """Минимальный stub — реальная статистика через SQLiteVoiceMemory."""
        loop = asyncio.new_event_loop()
        try:
            return loop.run_until_complete(self._get_stats_async())
        finally:
            loop.close()

    async def _get_stats_async(self) -> Dict[str, Any]:
        await self._ensure_init()
        # Простой SELECT — тривиальный, без list comprehension
        def _stats(conn):
            cur = conn.execute(
                "SELECT COUNT(*) FROM turns WHERE scope = ?",
                (self.TURN_SCOPE,),
            )
            turn_count = cur.fetchone()[0]
            cur = conn.execute(
                "SELECT COUNT(*) FROM facts WHERE scope = ?",
                (self.FACT_SCOPE,),
            )
            fact_count = cur.fetchone()[0]
            return {"turns": turn_count, "facts": fact_count}
        return await self._db._run_sync(_stats)
```

**Важно:** SQLiteVoiceMemory **не персистит turn-ы в проде** (`base.py:5-7`), но **поддерживает таблицу `turns`** (см. метод `all_turns` в `base.py:435` — она там зачем-то есть). Реальная проверка: DDL `sqlite_voice.py` **не содержит** `CREATE TABLE turns`. Это потенциальная дыра в схеме адаптера. Решение — расширить DDL `_TURNS_DDL` в **отдельном** коммите после verify в Phase 1 PR (issue: `turns` нужно создавать перед INSERT, иначе FK не пройдёт).

### 2.4 Маппинг таблиц при консолидации

| `voice_memory.db` (старая) | `harness_voice.db` (новая, после merge) | Где живёт |
|---|---|---|
| `voice_turns` (id, session_id, role, content, timestamp, speaker_id) | **новая таблица `turns`** (id, scope, role, content, name, tool_call_id, metadata_json, created_at) | `_turns_ddl` (новый, в этой карточке) |
| `voice_facts` (id, fact, category, created_at, updated_at, speaker_id) | `facts` (id, key, value, scope, metadata_json, created_at) | уже есть в DDL `sqlite_voice.py:52-61` |
| `voice_turns_fts` (FTS5) | (нет аналога; search через `SQLiteVoiceMemory.search` без FTS5) | n/a |
| `voice_memory_meta` | (нет аналога) | n/a |
| `waypoints` (с `map_id NOT NULL FK maps`) | `waypoints` (без `map_id`) | конфликт; адаптер игнорирует `map_id` |
| `music_tracks` | n/a | отдельная карточка |

### 2.5 Скрипт миграции `migrations/010_voice_memory_unify.sql`

**DDL + marker, не data-migration.**

```sql
-- ============================================================================
-- Migration: 010_voice_memory_unify.sql
-- Purpose:   Phase 1 of ADR-0055 — consolidation path. After this migration,
--            MCP tools (mcp_server.py, waypoint_store.py) write to
--            /data/harness_voice.db through VoiceMemoryAdapter, not
--            /data/voice_memory.db. The old file stays on the volume
--            read-only until Shifu explicitly removes it.
--
--            This file does NOT execute data-migration. data-migration =
--            Phase 2, separate card after merge of AgentCore (step 03).
--
-- Why a migration file if it's only a marker?
--   schema_migrations framework requires a row per applied step. The marker
--   keeps the framework aware that the path was switched.
-- ============================================================================

-- Phase 1 marker. Value is irrelevant — only the row existence matters.
-- If your framework uses a separate schema_migrations table, replace this
-- with the framework's INSERT syntax.
INSERT OR IGNORE INTO voice_memory_meta (key, value)
VALUES ('migration_010_applied_at', strftime('%s', 'now'));
```

**Полный data-migration** (Фаза 2, отдельный коммит):
```sql
ATTACH DATABASE '/data/voice_memory.db' AS old;
INSERT INTO turns (scope, role, content, name, tool_call_id,
                  metadata_json, created_at)
SELECT 'personality', role, content, NULL, NULL,
       json_object('legacy_session_id', session_id,
                   'legacy_speaker_id', speaker_id,
                   'migrated_at', strftime('%s', 'now')),
       timestamp FROM old.voice_turns;
-- ... то же для voice_facts
DETACH old;
-- DROP TABLE voice_turns (только после явного одкаша Шифу)
```

### 2.6 dry-run скрипт `scripts/migrations/migrate_voice_memory_unify.py`

```python
"""Dry-run по умолчанию: показывает что будет потеряно при удалении
voice_memory.db. С --apply выполняет Phase 1 (только path switch).

Usage:
    python3 scripts/migrations/migrate_voice_memory_unify.py \\
        --harness /data/harness_voice.db \\
        --legacy /data/voice_memory.db \\
        [--apply]
"""
```

**Что делает dry-run:**
1. Подключается к обеим БД (если legacy существует).
2. Считает `count(*)` для каждой таблицы в обоих файлах.
3. Печатает таблицу:
   ```
   Table         harness_voice.db   voice_memory.db   Action
   voice_turns   -                  85                (Phase 2: INSERT to turns)
   voice_facts   -                  1                 (Phase 2: INSERT to facts)
   turns         0 (после init)     -                 OK
   facts         0                  -                 OK
   ```
4. Ничего не пишет. Exit 0.

**Что делает `--apply`:**
1. Только запускает `migrations/010_voice_memory_unify.sql` через `_run_migrations()`.
2. **НЕ** трогает файлы. **НЕ** копирует данные. Это в Фазе 2.
3. После `--apply` нужно **деплоить новый код** (с `VoiceMemoryAdapter`), чтобы MCP-инструменты действительно начали писать в новую БД. Без деплоя `--apply` = no-op.

---

## 3. Альтернативы, которые мы отвергли

| Альтернатива | Почему отвергли |
|---|---|
| **`cp voice_memory.db harness_voice.db`** | Сломает `waypoints` (конфликт `map_id NOT NULL`), `faq_items` (`created_at` vs `indexed_at`). INSERTы упадут. |
| **`ATTACH + INSERT ... SELECT`** в одном коммите | Тот же schema-конфликт + race с пишущими сервисами. |
| **Оставить 2 БД навсегда** | ADR-0037 §5.3 уже отметил миграцию как отдельную работу. Не решает backup-удвоение и silent dual-write. |
| **Переписать MCP-инструменты на `SQLiteVoiceMemory` напрямую** | Слишком большой diff. Адаптер — инкрементальный шаг (ADR-0013). |
| **Удалить `voice_memory.db` сразу** | Нарушает ADR-0018 (silent data loss без явного решения Шифу). |
| **Расширить `voice_memory.py` чтобы он писал в обе БД** | Дублирование записей, race conditions, два источника правды — хуже чем сейчас. |
| **Сделать `voice_memory.db` ATTACH в `harness_voice.db` живым** | SQLite ATTACH на живой БД под WAL не даёт гарантий consistency для прод-нагрузки. |

---

## 4. Trade-offs

| Что получаем | Чем платим |
|---|---|
| **Одна БД** на `/data/harness_voice.db` → один backup, одна точка восстановления | +1 DDL (`turns`) + ~120 LOC адаптера + ~80 LOC тестов |
| **MCP-инструменты делят хранилище** с диалогом → `MemorySearchTool` видит turn-ы MCP | Нужно ясно мапить legacy `session_id`/`speaker_id` → `metadata_json` (до шага 03) |
| **Схема больше не дублируется** для `waypoints` и `faq_items` | `voice_memory.py` остаётся жить в `rob_box_voice` (никто не использует после merge — зачистка в Фазе 2) |
| **Готовая точка для Фазы 2** (одна колонка `agent` сразу работает для обоих источников) | Ждём шаг 03 (AgentCore) |
| **Read-only файл** `/data/voice_memory.db` сохраняется для forensic | +1 файл на томе — незначительно |
| **`SQLiteVoiceMemory` уже не персистит turn-ы** — адаптер добавляет turn-ы в `turns` со scope=`mcp:legacy`, что **нарушает** Шифу директиву 02.09.2026 для MCP-источника | Требует явного решения Шифу: либо MCP turn-ы тоже in-RAM (тогда и адаптер не нужен — закрываем issue), либо это исключение для MCP. **Запрашиваем в issue-комментарии.** |

---

## 5. Не делаем

1. **Не** вводим колонку `agent`/`namespace` в DDL — это решение зафиксирует шаг 03 (`AgentCore`).
2. **Не** удаляем `/data/voice_memory.db` автоматически — Шифу решает вручную.
3. **Не** трогаем `voice_memory.py` как класс — адаптер **поверх**.
4. **Не** делаем `voice_memory.db` временным хранилищем для новых данных.
5. **Не** мигрируем данные в этой карточке — только переключаем путь. data-migration = Фаза 2.
6. **Не** включаем `music_tracks` в эту карточку (отдельная задача, другая сложность: `MusicLibrary` использует свой DDL с миграциями 004/006/007).
7. **Не** персистим turn-ы в `turns` для диалоговой ноды (Шифу директива 02.09.2026 действует). Только MCP-источники могут писать в `turns` через адаптер (явное исключение, см. §4).

---

## 6. Acceptance criteria

### 6.1 Контрактные (до merge)

- [ ] `src/rob_box_harness/rob_box_harness/memory/voice_memory_adapter.py` создан, экспортирует `VoiceMemoryAdapter` с API: `save_turn / save_fact / search / get_stats`. ~120 LOC.
- [ ] DDL `memory/sqlite_voice.py:52-91` расширен `_TURNS_DDL` (если отсутствует). Адаптер пишет turn-ы в `turns` со `scope="mcp:legacy"`.
- [ ] `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py:113,957,1012` использует `VoiceMemoryAdapter(db_path="/data/harness_voice.db")` вместо `VoiceMemory`.
- [ ] `src/rob_box_mcp_tools/rob_box_mcp_tools/waypoint_store.py:66` использует ту же БД (через адаптер или прямой `SQLiteVoiceMemory` — TBD по согласованию с `waypoint_store` maintainer).
- [ ] `migrations/010_voice_memory_unify.sql` создан с маркерной записью.
- [ ] `scripts/migrations/migrate_voice_memory_unify.py` создан: dry-run by default, `--apply` для прода, exit 0 без изменений.
- [ ] Юнит-тесты `src/rob_box_harness/test/test_voice_memory_adapter.py` (≥6 тестов): save_turn / save_fact / search / get_stats / idempotency / multi-instance / sync API совместим со старым `VoiceMemory`.
- [ ] Юнит-тесты `src/rob_box_mcp_tools/test/test_mcp_server_unify.py` (≥3 теста): init использует адаптер; turn из MCP-инструмента виден через `MemoryStore.search` в `harness_voice.db`.
- [ ] Юнит-тесты `scripts/agent_flow/tests/test_migrate_voice_memory_unify.sh` (≥3 теста): dry-run показывает diff; `--apply` идемпотентен; exit code корректный.
- [ ] ADR-0055 уникален (ADR-0030 / ADR-collision-guard → exit 0).
- [ ] `mypy --strict src/rob_box_harness/rob_box_harness/memory/voice_memory_adapter.py` → exit 0.
- [ ] **Issue-комментарий задан** с тремя вопросами Шифу (см. §8 шаг 0).

### 6.2 Поведенческие (до merge)

- [ ] `docker exec rob_box_voice sqlite3 /data/harness_voice.db ".tables"` показывает `turns facts waypoints faq_items event_profile` (после init через адаптер).
- [ ] `docker exec rob_box_voice sqlite3 /data/voice_memory.db ".tables"` — таблицы существуют, но `mtime` файла **не** меняется после теста.
- [ ] `python3 scripts/migrations/migrate_voice_memory_unify.py --dry-run` на тестовой БД показывает корректный diff, exit 0.
- [ ] `pytest -v src/rob_box_harness/test/test_voice_memory_adapter.py src/rob_box_mcp_tools/test/test_mcp_server_unify.py` → exit 0.

### 6.3 Production gates (после merge)

- [ ] Никаких регрессий в существующих тестах `rob_box_harness` (baseline 24/24 PASS) и `rob_box_mcp_tools`.
- [ ] `git log --grep 'ADR-0055' --oneline` показывает один merge-commit + WIP-коммиты (если были).
- [ ] CI зелёный (build_all.yml, build-base-images.yml).

### 6.4 Out-of-scope acceptance (явно для следующих карточек)

- [ ] **Шаг 03 (`AgentCore`)**: DDL `SQLiteVoiceMemory` получает `agent TEXT NOT NULL DEFAULT 'default'`. → отдельная карточка.
- [ ] **Фаза 2 (data-migration)**: ATTACH + INSERT ... SELECT для `voice_turns`/`voice_facts` → `turns`/`facts` с `agent='personality'`. → отдельная карточка **после** merge шага 03.
- [ ] **Удаление `/data/voice_memory.db`**: после явного `kanban_comment` от Шифу + `docker exec rm`. → явное решение, не автомат.
- [ ] **`music_tracks` в `harness_voice.db`**: отдельная карточка (другая сложность: `MusicLibrary` имеет свой DDL с миграциями 004/006/007).

---

## 7. Ссылки

- **Карточка-источник**: `t_7a03364a` (эта работа), `t_XXXX` (шаг 03), `t_1992_*` (operator-agent 05a), `t_6f5ddb67` (ADR-0037).
- **Issue-источник**: #2000.
- **Код**:
  - `src/rob_box_voice/rob_box_voice/core/voice_memory.py:14-15,23,281-311` (старый MCP-стор, DDL voice_turns/voice_facts)
  - `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py:113,668,957,1012` (использование VoiceMemory)
  - `src/rob_box_mcp_tools/rob_box_mcp_tools/waypoint_store.py:5,12,66` (WaypointStore на той же БД)
  - `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/music.py:2981-2993` (MusicLibrary на той же БД — вне scope)
  - `src/rob_box_harness/rob_box_harness/memory/sqlite_voice.py:52-91` (DDL `harness_voice.db`)
  - `src/rob_box_harness/rob_box_harness/memory/base.py:3-7,435` (turns NOT persisted, `all_turns` метод существует)
  - `src/rob_box_voice/config/dialogue_node.yaml:97-104` (явный комментарий о конфликте + конфиг пути)
  - `migrations/002_voice_memory.sql` (старая схема)
  - `migrations/009_voice_memory_speaker_id.sql` (speaker_id добавлен в старую схему)
  - `migrations/004_music_library.sql, 006, 007` (MusicLibrary DDL — вне scope)
- **ADR-связи**:
  - ADR-0037 §5.3 (явно отложил эту миграцию)
  - ADR-0018 (честный FAIL, никаких silent dual-write / silent data loss)
  - ADR-0013 (incremental delivery — Фаза 1 сейчас, Фаза 2 отдельно)
  - ADR-0051 §5.2 (operator-agent namespace — придёт с шагом 03)
  - ADR-0030 (ADR-numbering — этот = 0055, проверка через test_adr_collision)
- **Документация-внешняя**: `docs/architecture/target-operator-agent-and-dialogue.md` §2.5/§5.2/§12 (карточка ссылается, но файла ещё нет в репо — это часть handoff'а Шифу, который будет коммититься в шаге 03).

---

## 8. Следующие шаги

**Шаг 0 (до merge): architect пишет issue-комментарий с тремя вопросами Шифу:**
1. **Turn-ы для MCP:** Шифу директива 02.09.2026 запрещает персистить turn-ы диалоговой ноды. Применимо ли это к MCP-инструментам? Если да — адаптер тривиальный (только facts + waypoints). Если нет — адаптер пишет turn-ы в `turns` со scope `mcp:legacy` (явное исключение).
2. **Данные `voice_memory.db`:** их переносить или «просто удалить»? (Старый handoff Q3 говорит «просто удалить», карточка говорит «перенести».) Зафиксировать в issue-комментарии + ADR.
3. **`music_tracks`:** включаем в эту карточку или отдельная? (Текущий ADR — отдельная; нужно подтверждение.)

**Шаг 1 (этот PR):** architect коммитит `voice_memory_adapter.py`, `mcp_server.py` патч, миграционный скрипт, юнит-тесты. Push в `z-{agent}/t_7a03364a-...`, открывает PR в `develop`.

**Шаг 2:** merge-gate проверяет: ADR-0055 уникален + CI зелёный.

**Шаг 3:** Шифу мержит PR после green CI + review + ответов на 3 вопроса из Шага 0.

**Шаг 4:** Шаг 03 (`AgentCore`) — отдельная карточка для DDL `agent` колонки.

**Шаг 5:** Фаза 2 (отдельная карточка) — data-migration `voice_turns`/`voice_facts` → `turns`/`facts` с `agent='personality'`.

**Шаг 6:** Удаление `/data/voice_memory.db` — только после явного `kanban_comment` от Шифу.