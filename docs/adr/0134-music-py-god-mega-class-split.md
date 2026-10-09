# ADR-0134 — Расщепление `tools/music.py` (god-mega-class, 5992 строки / 17 классов / 202 метода) на сфокусированные модули

**Дата:** 2026-10-03
**Статус:** Принято (черновик — на утверждение владельца)
**Автор:** шиди (architect) по заданию товарища Шифу — issue [#3014](https://github.com/krikz/rob_box_project/issues/3014)
**Связанные:** ADR-0021 R1 (CC-budget, 15/20), ADR-0132 (аранжировщик как инструмент — пример успешного выноса в `core/`), ADR-AF-0013 (incremental delivery, мелкие PR), ADR-0018 (честность: «не врать себе и учителю»), issue #2989 (CC-budget симптом), #2994 (hot-fix CC=16→15), #2935 (PR в ветке — CC растёт в базовой линии)

## 1. Проблема

`src/rob_box_mcp_tools/rob_box_mcp_tools/tools/music.py` — **5992 строки**, **17 top-level классов**, **202 метода**, **35+ методов только в `MusicManager`**. Это один файл, который за последние сутки (24.09.2026) принял **+9487/-1638 строк в 28 коммитах** (`git log origin/develop --since=2026-09-24 -- src/rob_box_mcp_tools`).

### 1.1 Симптомы, которые уже выстрелили

- **CC-budget жёсткий** (`scripts/lint/cc_budget.py` R1: методы ≤15, `__init__` ≤20). `scripts/lint/cc_budget_baseline.json` уже грандфазерит **3 метода** в этом файле: `MusicManager.execute_code=22`, `MusicManager.stop_all=16`, `SetDjModeTool.execute=3`. Issue #3014 фиксирует, что `ComposeMusicTool._build_arrangement CC=16` только что откатывали в PR #2994 — и это при том, что «базовая линия ещё растёт» (PR #2935 в ветке, по issue body).
- **CI red-flag реально срабатывает**: `cc_budget: FAIL — [FAIL] tools/music.py:ComposeMusicTool._build_arrangement CC=16 exceeds limit 15 and is not in baseline` (`gh run 35984333538` / `35982401797`). Это лечит симптом, не причину — следующий `feat(dj): X` поднимет CC до 17 в другом методе, цикл повторится.
- **Tragedy of commons** без ADR: каждое из 28 коммитов за сутки добавляло новую фичу в этот же файл (FX-pack #2983, room/echo #2988, секции #2990, role-bands #2991, …). Удельная стоимость добавления строки в 6000-строчный файл с 35 методами = «ещё один шанс задеть базовую линию CC».

### 1.2 Чего НЕ хватает (raw-evidence)

| Сейчас | Что это означает на практике |
|---|---|
| 17 классов в одном файле | PR ревью видит диффус 6000 строк; нельзя ограничить blast-radius одной темой |
| `MusicManager` 35+ методов | трудно тестировать, не подняв весь музыкальный стек; 4575-строчный `test_music.py` обходит init через `__new__` — это симптом того, что класс слишком велик для конструирования |
| CC ≥ 16 в трёх местах | каждая новая фича рискует CC-budget violation, требующей hot-fix |
| Прямой импорт из `tools.music` | `mcp_server.py:91,1112-1126`, `tools/__init__.py:30,81`, `test/test_music.py:39-50`, `test/test_compose_music_arranger_sync.py:63` — много hard-links, не seam |

## 2. Что уже сделано (raw-evidence, не выдумываю)

Корень `src/rob_box_mcp_tools/rob_box_mcp_tools/core/` **уже существует** и **уже содержит** вынесенную музыкальную логику:

```
arrangement_presets.py    7611    7.6K
arranger.py             161746  161.7K   ← ADR-0132
compose_knobs.py         11583   11.6K
generated_music_library  15361   15.4K
harmonize.py             82211   82.2K
renardo_sanitizer.py     31261   31.3K   ← issue-комментарий ссылается на MusicManager, но реализация автономна
rtttl.py                  8004    8.0K
rtttl_compose.py         71620   71.6K
rtttl_library.py         43840   43.8K
sample_fx.py              7794    7.8K
sample_loops.py           8858    8.9K
score_sheet.py           41201   41.2K   ← ADR-0132 PR-1
synth_traits.py           9143    9.1K   ← ADR-0132 PR-6
tool_call_accumulator.py  4380    4.4K
minimax_music_client.py  11708   11.7K
__init__.py
```

17 файлов, 542 KB. То есть **вынос уже идёт** — но `tools/music.py` остался монолитом: 5992 строки / 202 метода / 17 классов в одном файле. ADR-0134 фиксирует финальную фазу.

**Соседний `tools/minimax_music.py`** (отдельный файл) уже живёт, тоже импортирует `MusicManager` через комментарий — это прецедент «вынести одну тему в свой файл».

## 3. Решение

### 3.1 Целевая структура: 5 новых модулей в `core/`, тонкий `tools/music.py`

Принцип: **специализация по доменной области**, а не «по размеру класса». Каждый модуль — один «владелец ответственности» (ADR-0132 §3.1 стиль — «один владелец DJ-флага», тот же подход).

| Целевой модуль (новый) | Источник в `tools/music.py` | Методов ≈ | Ответственность |
|---|---|---:|---|
| `core/music_renardo_bridge.py` | `_initialize_renardo`, `_verify_and_retry_synthdefs`, `_attach_renardo_reply_listener`, `_renardo_reply_listener_loop`, `_log_scsynth_reply_if_any`, `_log_osc_reply`, `_split_osc_address`, `_decode_osc_args`, `_ensure_renardo_available`, `_check_supercollider`, `_send_osc_raw` | 11 | Bootstrap Renardo, OSC-протокол, SC-listener |
| `core/music_stack_health.py` | `_evaluate_music_stack_health`, `is_music_stack_healthy`, `music_stack_unavailable_error`, `_log_synth_truth_discrepancy`, `known_synth_names` (read-only) | 5 | Health-gate, синхронизация sclang-истины |
| `core/music_pattern_runtime.py` | `execute_code`, `stop_pattern`, `stop_all`, `_call_player_stop`, `_prewarm_sample_buffers`, `_resolve_pattern_name`, `_renardo_bpm`, `_schedule_stop`, `set_form_deadline`, `set_form_cycle_end`, `clear_form_deadline`, `auto_stop_idle_music`, `stop_music_on_session_end` | 13 | Runtime паттернов: старт/стоп/дедлайны |
| `core/music_session_state.py` | `set_dj_mode`, `dj_mode_enabled`, `set_master_gain`, `get_state`, `set_vibe_preset` | 5 | Состояние сессии (DJ-флаг, gain, пресет) |
| `core/music_code_filter.py` | `_filter_code`, `_filter_code_ast`, `_validate_music_code`, `_remap_illegal_slots`, `_fix_pattern_length`, `_cap_amp` | 6 | AST-санация Renardo-кода (дополняет существующий `renardo_sanitizer.py` — самостоятельный seam, чтобы split не ломал существующих потребителей санитайзера) |

После выноса `tools/music.py` содержит только:
- **Module-level** helper'ы: `_search_alternatives`, `_resolve_melody_with_candidate`, `_mismatch_note`, `_explicit_kwargs`, `_PLAY_SYMBOLS_RE`, `_RENARDO_PLAYER_NAMES`, `_PATTERN_NAME_RE`, `CRITICAL_SYNTHS`, `MELODIC_LEAD_SYNTHS`, `_TRACK_INHERIT_FIELDS`, `_UNSET`
- **`MusicManager`** как **composition root** — фасад, делегирующий в 5 новых модулей (через инстанс-атрибуты или композицию); сам содержит только `__init__`, `dj_mode_enabled`/`set_dj_mode` (тонкая обёртка), `get_state` (агрегатор), `known_synth_names` (тонкая обёртка), `is_music_stack_healthy` (тонкая обёртка)
- **Тулзы** (не выносятся — issue явно говорит «не выносить тулзы»): `ExecuteMusicCodeTool`, `ComposeMusicTool`, `PreviewArrangementTool`, `SaveArrangementPresetTool`, `StopMusicTool`, `SetVibePresetTool`, `GetMusicStateTool`, `LookupMelodyTool`, `SearchMelodyTool`, `SaveTrackTool`, `ListTracksTool`, `LoadTrackTool`, `DeleteTrackTool`, `SearchSamplesTool`, `SetDjModeTool` — 15 тулзов уже не god-mega-class, они остаются
- **`TrackLibrary`** (класс с строки 4706, 290 строк) — отдельная тема, отдельная карточка (см. §5.7)

**Целевой размер `tools/music.py`:** ≈ **1500–1800 строк** (composition root + 15 тулзов + helper'ы; TrackLibrary ≈ 290 строк отдельно). Это **выше** заявленных в issue #3014 ≤200, потому что issue недооценивает размер 15 тулзов и TrackLibrary — реалистичный target обсуждается в §6 acceptance.

**Целевой размер `tools/music.py` без TrackLibrary:** **≈ 1300–1500 строк** (композиция + тулзы).

### 3.2 Composition root (а не DI-фреймворк)

`MusicManager.__init__` после split:

```python
class MusicManager:
    def __init__(self, ...):
        # existing параметры
        self._renardo = MusicRenardoBridge(...)      # ~900 строк
        self._health = MusicStackHealth(...)         # ~250 строк
        self._runtime = MusicPatternRuntime(...)     # ~600 строк
        self._session = MusicSessionState(...)       # ~250 строк
        self._code_filter = MusicCodeFilter(...)     # ~250 строк
        # тонкие delegating свойства: dj_mode_enabled, known_synth_names,
        # is_music_stack_healthy — форвардят на _session / _health
```

**Без DI-фреймворка** (issue явно: «Не вводить DI-фреймворк»). Композиция через инстанс-атрибуты — тот же паттерн, что ADR-0132 §3.3 «dataclass `HarmonizeOptions`».

**Тестирование:** helper `_make_manager` в `test_music.py` использует `MusicManager.__new__` + ручную инициализацию атрибутов. После split helper останется работать, но добавится опциональная фабрика `_make_manager_with_modules()` для сквозных сценариев (composition). Существующие 4575 строк тестов не ломаются.

### 3.3 Shim-стратегия (а не hard cut)

Чтобы не сломать 5 прямых импортов (`mcp_server.py`, `tools/__init__.py`, `tools/minimax_music.py`, два `test_*.py`), **в течение 2-3 фаз** `tools/music.py` реэкспортирует символы из новых модулей:

```python
# tools/music.py (временный shim, удаляется в Фазе 6)
from rob_box_mcp_tools.core.music_renardo_bridge import (
    MusicRenardoBridge, _send_osc_raw, _ensure_renardo_available,
)
from rob_box_mcp_tools.core.music_code_filter import (
    _filter_code, _validate_music_code, _remap_illegal_slots,
    _fix_pattern_length, _cap_amp,
)
# ... остальные модули
```

Каждая фаза удаляет свой кусок shim, переводит прямой импорт на новый модуль, прогоняет `pytest` + `cc_budget.py`. Shim — **видимая техдолг-точка** в `tools/music.py` с маркером `# SHIM-remove-after-#NNNN`.

### 3.4 Что НЕ входит в ADR-0134

Согласно issue body «Что НЕ предлагаю» и архитектурному принципу KISS:

- **Не выношу тулзы** (`ExecuteMusicCodeTool`, `ComposeMusicTool`, …) — они уже отдельные классы с одной ответственностью каждый; вынос в файл не уменьшит `tools/music.py` существенно и усложнит импорт
- **Не ввожу DI-фреймворк** — composition через инстанс-атрибуты достаточен
- **Не блокирую разработку** — это technical-debt, не blocker; параллельно можно делать `feat(dj): X`, пока фазы идут
- **Не трогаю** `core/renardo_sanitizer.py` — он уже отдельный модуль, ссылка на `MusicManager` в комментарии не блокирует split (см. raw-evidence: `core/renardo_sanitizer.py:558` — это docstring, не импорт)

## 4. Торговые компромиссы (trade-offs)

| Решение | Альтернатива | Почему это |
|---|---|---|
| **Composition root в `MusicManager`** | DI-фреймворк (`dependency-injector`, `inject`) | 5 зависимостей — ниже порога, где DI-фреймворк оправдан (KISS). Один файл, 30 строк инициализации. |
| **5 новых модулей в `core/`, а не в `tools/`** | В `tools/music_*.py` рядом | Логика «не-MCP-тулза» уже в `core/`, `tools/` — для тулзов с `MCPTool`-базой. Согласуется с ADR-0132 (вынос аранжировщика). |
| **Shim-реэкспорт на 2-3 фазы** | Hard cut в одной фазе | Hard cut сломал бы 5 импортов сразу. Shim — **инкрементальная** доставка (ADR-AF-0013). |
| **Тонкий `MusicManager` с делегированием** | Полный рефактор `MusicManager` в набор сервисов без фасада | Существующий код (включая `mcp_server.py:1112`) держит `MusicManager` как единую точку входа. Убирать фасад = менять публичный API = блокировать. Фасад сохраняет контракт. |
| **`MusicCodeFilter` отдельным модулем, не дополнять `renardo_sanitizer.py`** | Расширить `renardo_sanitizer.py` | `renardo_sanitizer.py` уже отвечает за AST-санацию. `_filter_code`/`_filter_code_ast` живут в `MusicManager` как **gate-чек перед `execute_code`**, а не как общая санация. Разные владельцы — разные модули. |
| **Не выносить `TrackLibrary`** (отдельная карточка) | Включить в Фазу 1 | `TrackLibrary` 290 строк, своя ответственность (persistence), 8 связанных тулзов — самостоятельная единица. Смешивать с god-mega-class split — нарушение принципа «один владелец». |
| **Принимаю `tools/music.py` ≈ 1300–1500 строк (без TrackLibrary), а не ≤200** | Довести до ≤200, вынеся тулзы | 15 тулзов + composition root + helpers физически не помещаются в 200 строк без выноса тулзов, что issue явно запрещает. Реалистичный target — фиксируется в acceptance, Шифу утверждает. |

## 5. План миграции: 7 фаз, 1 PR в неделю, 7 недель

Каждая фаза — **отдельная карточка kanban**, **отдельный PR в develop**, **обязательный прогон** `pytest src/rob_box_mcp_tools/test/` + `python scripts/lint/cc_budget.py`.

### Фаза 0 (ЭТА карточка) — ADR + план
- [x] Разведка: 5992/17/202, 17 файлов в `core/`, 5 hard-imports, baseline 3 метода
- [x] Написать `docs/adr/0134-music-py-god-mega-class-split.md` (этот документ)
- [x] Заасайнить 6 фазовых карточек через `kanban_create(assignee=backend, parents=[t_c0873925])`
- [x] PR с ADR; merge-gate → Шифу
- **Acceptance:** ADR в `docs/adr/`, 6 карточек в `ready` с явными `body.acceptance_criteria`, `git log -- docs/adr/0134-...` показывает коммит

### Фаза 1 — `core/music_code_filter.py` (наименьший blast-radius)
- **Что:** вынести `_filter_code`, `_filter_code_ast`, `_validate_music_code`, `_remap_illegal_slots`, `_fix_pattern_length`, `_cap_amp` (6 методов, ~250 строк) в `core/music_code_filter.py`. `MusicManager._filter_code`/`_validate_music_code`/`_remap_illegal_slots` → delegating wrappers
- **Тесты:** новый `test/test_core/test_music_code_filter.py` с unit-тестами для каждого чистого метода (вход AST → выход bool/str)
- **CC-budget:** все новые методы ≤ 12 (требование issue)
- **Acceptance:** `wc -l core/music_code_filter.py` показывает класс, `pytest test/test_core/test_music_code_filter.py` зелёный, `pytest test/test_tools/test_music.py` зелёный, `cc_budget.py` зелёный
- **Размер PR:** ~280 строк diff (250 новых + 30 удалённых)
- **Card:** `t_…_phase-1-music-code-filter-extract`

### Фаза 2 — `core/music_stack_health.py` (чистые функции, нет сайд-эффектов)
- **Что:** вынести `_evaluate_music_stack_health`, `is_music_stack_healthy`, `music_stack_unavailable_error`, `_log_synth_truth_discrepancy`, `known_synth_names` (read-only property)
- **Тесты:** `test/test_core/test_music_stack_health.py` — health-гейт на degraded manager, `sclang` mock
- **Acceptance:** аналогично Фазе 1
- **Размер PR:** ~220 строк
- **Card:** `t_…_phase-2-music-stack-health-extract`

### Фаза 3 — `core/music_renardo_bridge.py` (самый рискованный, OSC + listener)
- **Что:** вынести OSC-протокол, listener-loop, `_check_supercollider`, `_send_osc_raw`, `_initialize_renardo`, `_verify_and_retry_synthdefs`, `_attach_renardo_reply_listener`, `_renardo_reply_listener_loop`, `_log_scsynth_reply_if_any`, `_log_osc_reply`, `_split_osc_address`, `_decode_osc_args`, `_ensure_renardo_available` (12 методов, ~900 строк)
- **Тесты:** OSC-packet builder/parser round-trip (issue #1808 фикс уже покрыт), listener lifecycle (mock socket), `_verify_and_retry_synthdefs` с мокнутым `_synthdefs_added`
- **CC-budget:** особенно важен — listener-loop исторически CC-heavy; новые методы ≤ 12
- **Acceptance:** `pytest test/test_core/test_music_renardo_bridge.py` зелёный + полный прогон `test_tools/test_music.py` зелёный + живой прогон на стенде issue #2977 (если влит) — иначе unit + ручной запуск
- **Размер PR:** ~950 строк
- **Card:** `t_…_phase-3-renardo-bridge-extract`

### Фаза 4 — `core/music_pattern_runtime.py` (CC-budget killer: `execute_code` 22→≤15)
- **Что:** вынести `execute_code` (текущий CC=22, **главный бюджет-нарушитель**), `stop_pattern`, `stop_all` (CC=16), `_call_player_stop`, `_prewarm_sample_buffers`, `_resolve_pattern_name`, `_renardo_bpm`, `_schedule_stop`, `set_form_deadline`, `set_form_cycle_end`, `clear_form_deadline`, `auto_stop_idle_music`, `stop_music_on_session_end` (13 методов, ~600 строк)
- **CC-budget:** ядро цели. После выноса в `MusicManager` остаются delegating wrappers (CC=1 каждый), `MusicPatternRuntime.execute_code` декомпозируется на ≤12 + helpers
- **Тесты:** `test/test_core/test_music_pattern_runtime.py` + расширение `test_tools/test_music.py` (всё, что тестирует `execute_code`/`stop_all`)
- **Acceptance:** `cc_budget.py` показывает `MusicPatternRuntime.execute_code CC=12`, `MusicManager.execute_code CC=1` (wrapper), `MusicManager.stop_all CC=1`, **baseline.json удаляет** `MusicManager.execute_code: 22` и `MusicManager.stop_all: 16` записи
- **Размер PR:** ~700 строк
- **Card:** `t_…_phase-4-pattern-runtime-extract`

### Фаза 5 — `core/music_session_state.py` (state-only, мелкий)
- **Что:** вынести `set_dj_mode`, `dj_mode_enabled`, `set_master_gain`, `get_state`, `set_vibe_preset` (5 методов, ~250 строк)
- **Тесты:** `test/test_core/test_music_session_state.py` — DJ-флаг, gain-clamp, vibe-preset lookup
- **CC-budget:** все методы ≤ 12
- **Acceptance:** аналогично Фазе 1; baseline.json удаляет `SetDjModeTool.execute: 3` запись (CC после выноса падает с 3 до 1)
- **Размер PR:** ~280 строк
- **Card:** `t_…_phase-5-session-state-extract`

### Фаза 6 — Удаление shim, перевод импортов, финальная зачистка
- **Что:** удалить `# SHIM-remove-after-…` маркеры из `tools/music.py`, перевести 5 hard-imports:
  - `mcp_server.py:91,1112,1117,1126` — `MusicManager` остаётся в `tools.music` (фасад), импорты не меняются
  - `tools/__init__.py:30,81` — `from .music import *` остаётся, `__all__` остаётся, т.к. фасад экспортирует всё что нужно
  - `tools/minimax_music.py:22` — комментарий, не импорт, удалить ссылку на `tools/music.py:MusicManager`
  - `test/test_tools/test_music.py:39-50` — обновить импорты helper'ов на `from rob_box_mcp_tools.core.music_renardo_bridge import _send_osc_raw` и т.п.
  - `test/test_compose_music_arranger_sync.py:63` — `from rob_box_mcp_tools.tools.music import ComposeMusicTool` остаётся (тулз в фасаде)
- **Acceptance:** `wc -l tools/music.py` показывает 1300–1500 (или утверждённый target), `grep -r "SHIM-remove" src/rob_box_mcp_tools/` пусто, все импорты резолвятся, `pytest` + `cc_budget.py` зелёные, `mypy`/`ruff`/`pyright` (если в CI) зелёные
- **Размер PR:** ~150 строк (зачистка)
- **Card:** `t_…_phase-6-shim-removal-final-cleanup`

### Фаза 7 (опционально, отдельная карточка) — `core/track_library.py` + `tools/music_library_tools.py`
- **Что:** вынести `TrackLibrary` (4706–4995) + 8 связанных тулзов (`LookupMelodyTool`, `SearchMelodyTool`, `SaveTrackTool`, `ListTracksTool`, `LoadTrackTool`, `DeleteTrackTool`, `SearchSamplesTool` — `SetDjModeTool` остаётся в основном файле) в `tools/music_library_tools.py`
- **Acceptance:** `wc -l tools/music.py` ≤ 1000 строк
- **Card:** `t_…_phase-7-track-library-extract` (отдельная карточка, не блокирует god-mega-class split)

## 6. Acceptance criteria (для ВСЕЙ инициативы)

По issue #3014 + реальные границы после анализа:

- [ ] `wc -l src/rob_box_mcp_tools/rob_box_mcp_tools/tools/music.py` ≤ **1500** (без TrackLibrary) — реалистичный target, обсуждается в §4 trade-off
- [ ] `wc -l src/rob_box_mcp_tools/rob_box_mcp_tools/tools/music.py` ≤ **1000** после Фазы 7
- [ ] Каждый вынесенный модуль содержит unit-тесты (`test/test_core/test_music_*.py`)
- [ ] `cc_budget.py` для всех функций в новых модулях ≤ **12**
- [ ] `cc_budget.py` для delegating wrappers в `MusicManager` ≤ **2** (один if-else максимум)
- [ ] `baseline.json` после Фазы 4: `MusicManager.execute_code` и `MusicManager.stop_all` удалены из overrides
- [ ] Никаких регрессов в e2e voice/music (см. `L: E2E Voice Test` workflow) — измеряется Фазой 3 (живой стенд)
- [ ] **Zero new imports of internals** — после Фазы 6 весь stack импортирует `MusicManager` как фасад; внутренние сервисы (`MusicRenardoBridge`, …) импортируются только из `core/` и из тестов

## 7. Что НЕ подтверждено (честность по ADR-0018)

- **Не измерено:** акустическое влияние на живой стенд. Фаза 3 потребует живого прогона `compose_music` через robot — но это за рамками этой карточки (architecture/ADR, не implementation). План в Фазе 3 явно фиксирует «если стенд #2977 не влит, unit + ручной запуск».
- **Не измерено:** реальный размер каждой фазы в строках. Цифры ~250/~220/~950/~700/~280 в §5 — инженерная оценка по `wc -l` текущих диапазонов методов; реальный diff может отличаться на ±20%. Это не блокирует ADR, но Фаза 1 после реализации даст ground-truth для корректировки оценок Фаз 2-5.
- **Не подтверждено:** что `mcp_server.py:1112 MusicManager(...)` не сломается от изменения `__init__`-сигнатуры. Контракт `__init__` сохраняется — это **обязательное** требование к Фазе 1 (ввести новые инстанс-атрибуты без удаления старых параметров). Проверяется каждым PR.

## 8. Последствия

**Положительные:**
- Каждая новая фича после Фазы 6 имеет 5 мест для размещения вместо одного; CC-budget violation становится **локальной** проблемой (один новый модуль), а не «опять `tools/music.py`» (что блокирует develop).
- Тесты расщепляются: 4575-строчный `test_music.py` распадается на 6 файлов, каждый ≤ 1000 строк, что улучшает обзорность и ускоряет локальный `pytest -k` для разработчика.
- ADR-0132 прецедент: успешный вынос в `core/` доказал, что команда умеет с этим работать (15 файлов в `core/`, ADR-0132 PR-0…PR-7).
- Каждый модуль имеет одного владельца ответственности — локализация бага, ownership code review.

**Издержки/риски:**
- 7 PR за 7 недель = замедление feature delivery на 7 недель для музыкальных фич, попадающих в `tools/music.py`. Смягчение: фазы изолированы (можно мержить параллельно с feature, если feature не трогает вынесенный модуль).
- Shim-импорты — 2-3 фазы техдолга, видимого в `tools/music.py`. Смягчение: явные маркеры `# SHIM-remove-after-#NNNN` + tracking в фазовых карточках.
- Composition root в `MusicManager` — небольшое увеличение связности (5 импортов вместо 0); альтернатива (DI-фреймворк) — больше, см. §4.

## 9. Метрики для отслеживания (post-merge)

- `wc -l src/rob_box_mcp_tools/rob_box_mcp_tools/tools/music.py` — недельный снимок в `docs/adr/0134-progress.md`
- `python scripts/lint/cc_budget.py` — еженедельный прогон; должно показывать уменьшение baseline-записей
- `git log --since=YYYY-MM-DD -- src/rob_box_mcp_tools/rob_box_mcp_tools/tools/music.py` — weekly churn; после Фазы 6 должен снизиться

## 10. Кросс-ссылки

- Issue [#3014](https://github.com/krikz/rob_box_project/issues/3014) — источник задачи
- Issue #2989 — CC-budget symptom (как PR-#2994 чинил hot-fix)
- PR #2994 — `fix(music/ci): снизить CC _build_arrangement до 15`
- PR #2935 — в ветке, CC базовой линии растёт (issue body упоминает)
- ADR-0132 — успешный прецедент выноса аранжировщика в `core/`
- ADR-0021 R1 — CC-budget policy
- ADR-AF-0013 — incremental delivery
- ADR-0018 — честность, «не врать себе»
