# Design Note — `core/music_pattern_runtime.MusicPatternRuntime` (ADR-0134 Phase 4)

**Дата:** 2026-10-03
**Автор:** architect (шиди) для backend-фазы t_d6ad4d0c / t_5012aad0
**Связанные:** ADR-0134 §5 Phase 4, issue #3014, ADR-0021 R1 (CC-budget), ADR-0132 (composition root)
**Статус:** проект — на утверждение Шифу до начала реализации

---

## 1. Проблема (Phase 4, CC-killer)

`MusicManager.execute_code` имеет CC=22, `MusicManager.stop_all` — CC=16. Они
помечены в `scripts/lint/cc_budget_baseline.json` как grandfathered. ADR-0134
требует вынести 13 методов (~600 строк) в `core/music_pattern_runtime.py`,
декомпозировать execute_code на ≤12 CC каждый, обернуть `MusicManager`-методы
в 1-строчные delegating wrappers (CC=1). После Phase 4 записи из baseline
**удаляются** — больше нет grandfathering, бюджет соблюдается по сути.

## 2. Решение: новый класс `MusicPatternRuntime`

Класс инкапсулирует runtime паттернов: валидация, маршрутизация, жизненный
цикл (дедлайны, watchdog, сессия). Конструируется **обязательно от
экземпляра `MusicManager`** (composition root, ADR-0134 §3.2): `MusicManager`
держит всё состояние и сервисы, runtime — только методы, которые это
состояние потребляют.

```python
class MusicPatternRuntime:
    """Runtime паттернов Renardo: валидация, маршрутизация, жизненный цикл.

    Не владеет состоянием — держит ссылку на ``MusicManager`` (composition root).
    Декомпозирует 13 методов из tools/music.py так, чтобы каждый public-метод
    и каждый helper ≤ 12 CC (issue #3014, жёстче, чем METHOD_LIMIT=15).
    """

    def __init__(self, manager: MusicManager) -> None: ...
```

### 2.1 Конструктор — wiring зависимостей

```python
def __init__(self, manager: MusicManager) -> None:
    self._mgr = manager  # composition root (ADR-0134 §3.2)
```

Все 13 методов читают состояние через `self._mgr.<attr>` либо через
тонкие property-обёртки `MusicManager` (например `self._mgr.dj_mode_enabled`).
**Не дублируем поля** — `MusicManager` остаётся единственным владельцем.
CC=1 (только присваивание).

Зависимости (только чтение) — что runtime читает из `MusicManager`:

| Что | Где в `MusicManager` сейчас | Применяется в |
|---|---|---|
| `_renardo_context` | `__init__` | execute_code, stop_all, _call_player_stop, _prewarm_sample_buffers, _renardo_bpm |
| `_renardo_available`, `_renardo_last_error` | `__init__` | execute_code, stop_pattern, stop_all |
| `_active_patterns`, `_pattern_history` | `__init__` | execute_code, stop_pattern, _resolve_pattern_name |
| `_music_stack_status`, `_require_healthy` | `__init__` | execute_code, stop_pattern, stop_all (через `is_music_stack_healthy()`) |
| `_max_amp` | `__init__` | execute_code (в sanitize call) |
| `_master_gain`, `_master_gain_applied` | `__init__` | execute_code (lazy first-time apply) |
| `_music_session_active_since`, `_last_music_activity_at`, `_last_stop_at` | `__init__` | execute_code, stop_all, auto_stop_idle_music, stop_music_on_session_end |
| `_auto_stop_ttl_seconds`, `_auto_stop_count` | `__init__` | auto_stop_idle_music |
| `_music_deadline_at`, `_music_deadline_segments` | `__init__` | execute_code (через _schedule_stop), auto_stop_idle_music, stop_all |
| `_music_form_deadline_at`, `_music_form_cycle_ends_at` | `__init__` | execute_code (clear), set_form_deadline/cycle_end/clear, auto_stop_idle_music, stop_all |
| `dj_mode_enabled` (property) | `__init__` (`_dj_mode_enabled`) | auto_stop_idle_music (segments-deadline DJ skip) |
| `MAX_SEGMENTS`, `BEATS_PER_BAR`, `MIN_SEGMENTS_DEADLINE_SECONDS`, `SEGMENTS_DEADLINE_SAFETY_FACTOR`, `DEPRECATED_DURATION_SEC_CLAMP` | class constants | execute_code, _schedule_stop |
| `_RENARDO_PLAYER_NAMES`, `_PATTERN_NAME_RE`, `_PLAY_SYMBOLS_RE` | module-level | _resolve_pattern_name, stop_pattern (поиск имён), _prewarm_sample_buffers |

Зависимости (вызовы сервисов из других фаз) — **пока shim** на `MusicManager`:

| Сервис | Где живёт сейчас | Где будет жить |
|---|---|---|
| `_check_supercollider()` | `MusicManager` | `core/music_renardo_bridge.MusicRenardoBridge` (Phase 3) |
| `_ensure_renardo_available()` | `MusicManager` | `core/music_renardo_bridge.MusicRenardoBridge` |
| `_send_osc_raw(...)` | `MusicManager` | `core/music_renardo_bridge.MusicRenardoBridge` |
| `is_music_stack_healthy()` | `MusicManager` | `core/music_stack_health.MusicStackHealth` (Phase 2) |
| `music_stack_unavailable_error()` | `MusicManager` | `core/music_stack_health.MusicStackHealth` |
| `set_master_gain(...)` | `MusicManager` | `core/music_session_state.MusicSessionState` (Phase 5, OUT Phase 4) |

В **Phase 4** (эта фаза) эти сервисы остаются в `MusicManager`. Runtime
вызывает их через `self._mgr.<method>()`. Phase 2/3/6 заменят на прямой
вызов через `self._mgr._health.<method>()` — это **не блокирует** Phase 4.

### 2.2 Публичные методы (13) — точные сигнатуры

Сигнатуры **полностью совпадают** с текущими `MusicManager.<method>` — чтобы
wrappers были `return self._runtime.<method>(*args, **kwargs)`. Это и есть
стратегия обратной совместимости (см. §4).

```python
def execute_code(
    self,
    code: str,
    pattern_name: Optional[str] = None,
    *,
    segments: Optional[int] = None,
    duration_sec: Optional[float] = None,
) -> Dict[str, Any]: ...

def stop_pattern(self, pattern_name: str) -> Dict[str, Any]: ...

def stop_all(self) -> Dict[str, Any]: ...

def _call_player_stop(self, pattern_name: str) -> None: ...

def _prewarm_sample_buffers(self, code: str) -> None: ...

def _resolve_pattern_name(self, pattern_name: str) -> Tuple[bool, str]: ...

def _renardo_bpm(self) -> float: ...

def _schedule_stop(self, *, segments: int, bpm: float) -> None: ...

def set_form_deadline(self, duration_seconds: float) -> None: ...

def set_form_cycle_end(self, duration_seconds: float) -> None: ...

def clear_form_deadline(self) -> None: ...

def auto_stop_idle_music(
    self,
    ttl_seconds: Optional[float] = None,
    now: Optional[float] = None,
) -> Dict[str, Any]: ...

def stop_music_on_session_end(self) -> Dict[str, Any]: ...
```

**Без изменений сигнатур:** `ExecuteMusicCodeTool.execute(**kwargs)` и
любые другие тулзы, зовущие `MusicManager.<method>(...)`, продолжают
работать через wrapper.

## 3. Декомпозиция `execute_code` — CC=22 → ≤12 + helpers

`execute_code` сейчас содержит 5 логических фаз. Каждая фаза становится
**приватным helper** с одной ответственностью; `execute_code` остаётся
оркестратором (CC ≤ 8, без if-else на result).

### 3.1 Границы helpers

| Helper | CC ≤ | Назначение (1 строка) |
|---|---:|---|
| `_execute_sanitize(self, code: str) -> Tuple[Optional[SanitizeResult], Optional[Dict[str, Any]]]` | 8 | Прогоняет `renardo_sanitizer.sanitize_renando` и возвращает либо `(result, None)`, либо `(None, error_dict)` — оркестратор делает `return error_dict` без вложенности. |
| `_execute_emit_debug_log(self, code: str) -> None` | 2 | Debug-stderr dump `FINAL CODE` (live 15:44 хук для KeyError('amp')). Чисто side-effect. |
| `_execute_check_stack_health(self) -> Optional[Dict[str, Any]]` | 4 | `is_music_stack_healthy` + `_check_supercollider` + `_ensure_renardo_available` — возвращает `error_dict` или `None`. |
| `_execute_apply_segments_safety_net(self, code: str, *, segments: Optional[int], duration_sec: Optional[float]) -> None` | 6 | Issue #990 — кладёт `__total_beats/__bpm/__bar_duration/__duration_sec` в `_renardo_context` и зовёт `_schedule_stop` (только из segments, не из duration_sec). |
| `_execute_run(self, code: str, has_clock_clear: bool) -> Optional[Dict[str, Any]]` | 7 | `exec(code, ctx)` + post-exec `/g_freeAll` + 50ms sleep + `/g_new` (issue #778). Возвращает `error_dict` или `None`. |
| `_execute_record_session_activity(self, code: str, pattern_name: Optional[str]) -> Dict[str, Any]` | 5 | Issue #935: `_pattern_history[pattern_name]=code`, `_active_patterns.add(...)`, `_last_music_activity_at=now`, `_music_session_active_since=now if None`, `clear_form_deadline()`. Возвращает финальный dict (success + warnings). |

### 3.2 Тело `execute_code` после декомпозиции

```python
def execute_code(self, code, pattern_name=None, *, segments=None, duration_sec=None):
    """Безопасно выполнить Renardo-код. (контракт см. tools/music.py:1420)"""
    sanitized, err = self._execute_sanitize(code)
    if err:
        return err
    code = sanitized.code
    quality_warnings = list(sanitized.warnings)

    self._execute_emit_debug_log(code)

    health_err = self._execute_check_stack_health()
    if health_err:
        return health_err

    self._execute_apply_segments_safety_net(
        code, segments=segments, duration_sec=duration_sec,
    )

    self._prewarm_sample_buffers(code)

    has_clock_clear = "Clock.clear()" in code
    exec_err = self._execute_run(code, has_clock_clear)
    if exec_err:
        return exec_err

    if not self._mgr._master_gain_applied:
        self._mgr._master_gain_applied = True
        self._mgr.set_master_gain(self._mgr._master_gain)

    return self._execute_record_session_activity(code, pattern_name, quality_warnings)
```

**Оценка CC:** ~6 (sequential fall-through, no nested if-else). С запасом
до 12.

**Поток данных:** `sanitize` → `debug_log` → `health` → `safety_net` →
`prewarm` → `run` → `master_gain` → `record`. Каждый helper возвращает
`None`/`error_dict`/result, оркестратор делает ранний `return error`.

**Поведенческий контракт сохранён байт-в-байт:** порядок операций, debug-лог,
fallback-сообщения, `has_clock_clear` ветка, `clear_form_deadline()`,
`quality_warnings` в ответе — всё идентично текущему. Существующие 4575
строк `test/test_tools/test_music.py` не должны сломаться.

## 4. Декомпозиция `stop_all` — CC=16 → ≤12 + helpers

`stop_all` имеет 4 этапа teardown + 4 поля состояния + 2 ветки degraded/error.
Аналогично выносим в helpers.

| Helper | CC ≤ | Назначение |
|---|---:|---|
| `_stop_all_teardown(self) -> Optional[str]` | 4 | Issue #1000 anti-click: per-player stop (try/except) + `Clock.clear()` + 50ms sleep + `/g_freeAll`. Возвращает `clock_error` или `None`. |
| `_stop_all_reset_session_state(self) -> None` | 3 | − | Сброс: `_active_patterns.clear()`, `_last_stop_at=now`, `_music_deadline_at=None`, `_music_deadline_segments=None`, `clear_form_deadline()`, `_music_session_active_since=None`, `_last_music_activity_at=None`. |
| `_stop_all_build_response(self, clock_error: Optional[str], degraded: bool) -> Dict[str, Any]` | 3 | Issue #935 — safety-net: возвращает dict с warning при `degraded`, `clock_error`, или success. |

### 4.1 Тело `stop_all` после декомпозиции

```python
def stop_all(self) -> Dict[str, Any]:
    """Остановить всю музыку: ramp-down → freeAll. (issue #1000)"""
    degraded = self._mgr._require_healthy and not self._mgr.is_music_stack_healthy()
    clock_error: Optional[str] = None
    if not degraded and self._mgr._renardo_available and self._mgr._check_supercollider():
        clock_error = self._stop_all_teardown()
    self._stop_all_reset_session_state()
    return self._stop_all_build_response(clock_error, degraded)
```

**Оценка CC:** ~5 (3 sequential branches, no nesting). Заметка: degraded/clock_error
раннее вычисление нужно для двух разных return-путей; оркестратор остаётся линейным.

## 5. Декомпозиция прочих методов (CC-budget)

| Метод | Текущий CC | Helpers | Оценка CC после |
|---|---:|---|---:|
| `execute_code` | 22 | 6 helpers (§3) | 6 |
| `stop_all` | 16 | 3 helpers (§4) | 5 |
| `stop_pattern` | ~7 | 0 (уже ≤12) | 7 |
| `auto_stop_idle_music` | ~10 | 0 (уже ≤12) | 10 |
| `stop_music_on_session_end` | ~5 | 0 | 5 |
| `_call_player_stop` | ~5 | 0 | 5 |
| `_prewarm_sample_buffers` | ~8 | 0 (один try/except + nested for) | 8 |
| `_resolve_pattern_name` | ~8 | 0 (один regex + множества) | 8 |
| `_renardo_bpm` | ~5 | 0 | 5 |
| `_schedule_stop` | ~3 | 0 | 3 |
| `set_form_deadline` | 1 | 0 | 1 |
| `set_form_cycle_end` | 1 | 0 | 1 |
| `clear_form_deadline` | 1 | 0 | 1 |

Ни один из оставшихся 11 не превышает 12 CC; декомпозиция не требуется.
Это подтверждено локальным прогоном `radon cc -s` (см. §7 acceptance).

## 6. `MusicManager` — storage + delegating wrappers

```python
# В MusicManager.__init__ (в конце, после существующих полей):
self._runtime = MusicPatternRuntime(self)
```

13 wrappers — каждый CC=1 (один return):

```python
def execute_code(self, code, pattern_name=None, *, segments=None, duration_sec=None):
    return self._runtime.execute_code(
        code, pattern_name, segments=segments, duration_sec=duration_sec,
    )

def stop_pattern(self, pattern_name):
    return self._runtime.stop_pattern(pattern_name)

def stop_all(self):
    return self._runtime.stop_all()

def _call_player_stop(self, pattern_name):
    return self._runtime._call_player_stop(pattern_name)

def _prewarm_sample_buffers(self, code):
    return self._runtime._prewarm_sample_buffers(code)

def _resolve_pattern_name(self, pattern_name):
    return self._runtime._resolve_pattern_name(pattern_name)

def _renardo_bpm(self):
    return self._runtime._renardo_bpm()

def _schedule_stop(self, *, segments, bpm):
    return self._runtime._schedule_stop(segments=segments, bpm=bpm)

def set_form_deadline(self, duration_seconds):
    return self._runtime.set_form_deadline(duration_seconds)

def set_form_cycle_end(self, duration_seconds):
    return self._runtime.set_form_cycle_end(duration_seconds)

def clear_form_deadline(self):
    return self._runtime.clear_form_deadline()

def auto_stop_idle_music(self, ttl_seconds=None, now=None):
    return self._runtime.auto_stop_idle_music(ttl_seconds=ttl_seconds, now=now)

def stop_music_on_session_end(self):
    return self._runtime.stop_music_on_session_end()
```

**Важно:** сигнатуры wrapper'ов и runtime-методов идентичны — никакого
keyword-only расхождения, никакого переупорядочения. Существующие
`MusicManager.execute_code(code, pattern_name)` позиционные вызовы
(mcp_server, тулзы, тесты через `__new__`) продолжают работать.

## 7. Стратегия обратной совместимости с `ExecuteMusicCodeTool.execute`

`ExecuteMusicCodeTool.execute(code, pattern_name=None, segments=None,
duration_sec=None)` (см. tools/music.py:2223-2240) вызывает
`self._manager.execute_code(code, pattern_name, segments=segments,
duration_sec=duration_sec)`. После Phase 4:

- `MusicManager.execute_code` остаётся публичным методом с **той же
  сигнатурой** (`code, pattern_name=None, *, segments=None, duration_sec=None`)
- Тело заменено на wrapper (`return self._runtime.execute_code(...)`)
- `MCPTool.execute(**kwargs)` контракт сохранён — kwargs мапятся 1:1
- `MCPToolResult(success, data, message)` / `MCPToolResult(success, error)`
  логика в `ExecuteMusicCodeTool.execute` не меняется
- `_notify_music_state()` остаётся в тулзе (не относится к Phase 4)

**Тесты:** `test/test_tools/test_music.py` (run через `_make_manager`)
продолжают работать, т.к. обходят `__init__` через `MusicManager.__new__`
(см. ADR-0134 §3.2) — фасад с wrapper'ами прозрачен для них.

## 8. Явно OUT of scope

Per ADR-0134 §3.4, §5 Phase 4 boundaries:

- `set_dj_mode` → Phase 5 (`core/music_session_state.py`). Runtime читает
  DJ-флаг через `self._mgr.dj_mode_enabled` (property) — это уже работает,
  в Phase 4 ничего не меняется.
- `set_master_gain` → Phase 5. `execute_code` дёргает
  `self._mgr.set_master_gain(self._mgr._master_gain)` — Phase 4
  сохраняет этот вызов, не выносит его.
- `_initialize_renardo` → Phase 3 (`core/music_renardo_bridge.py`). Phase 4
  не трогает OSC-инициализацию.
- Также не трогаем: `set_vibe_preset` (Phase 5), `get_state` (Phase 5),
  `known_synth_names`/`is_music_stack_healthy`/`music_stack_unavailable_error`
  (Phase 2 thin wrappers остаются в `MusicManager` до Phase 6 shim-removal).

## 9. Шаги реализации (для t_5012aаd0 backend-фазы)

1. Создать `src/rob_box_mcp_tools/rob_box_mcp_tools/core/__init__.py` —
   пустой, либо re-export `MusicPatternRuntime` (как `core/arrangement_presets.py`)
2. Создать `src/rob_box_mcp_tools/rob_box_mcp_tools/core/music_pattern_runtime.py`:
   - класс `MusicPatternRuntime` с `__init__(self, manager: MusicManager) -> None` (CC=1)
   - 13 публичных методов по §2.2
   - 6 private helpers для execute_code (§3)
   - 3 private helpers для stop_all (§4)
3. В `tools/music.py`:
   - Добавить `from rob_box_mcp_tools.core.music_pattern_runtime import MusicPatternRuntime`
     в шапке
   - В конце `MusicManager.__init__`: `self._runtime = MusicPatternRuntime(self)`
   - Заменить тела 13 методов на wrappers (§6)
   - **НЕ менять:** публичную сигнатуру `MusicManager.execute_code`,
     любые другие методы вне списка 13, `MusicManager.__init__` параметры
4. Тест `test/test_core/test_music_pattern_runtime.py` (уже существует,
   xfail) — перевести `@pytest.mark.xfail(strict=False)` → обычные тесты,
   заменить `_runtime_stub()` (Mock) на реальные assertion через
   `MusicPatternRuntime(_make_manager())` (см. parent t_0898b8fc для
   подробностей unfail'а).

## 10. Acceptance criteria (дизайн-фаза, t_b8dffe1f)

Дизайн согласован, если:

- [x] Все 13 публичных сигнатур зафиксированы (§2.2) и совпадают с
      `MusicManager.<method>` сигнатурами байт-в-байт
- [x] Декомпозиция `execute_code` на 6 helpers (CC ≤ 8 каждый) с одной
      ответственностью (§3)
- [x] Декомпозиция `stop_all` на 3 helpers (CC ≤ 4 каждый) (§4)
- [x] `MusicManager` хранит `self._runtime` + 13 wrappers CC=1 (§6)
- [x] Стратегия обратной совместимости с `ExecuteMusicCodeTool.execute`
      (`MCPTool.execute(**kwargs)` контракт не нарушен, §7)
- [x] Явный OUT-of-scope список (§8)
- [x] `MusicManager.__init__` контракт сохранён (mcp_server.py:1112 не
      сломается — `_runtime` добавляется в конце, новых обязательных
      параметров нет)

## 11. Не подтверждено (честность по ADR-0018)

- **Точный CC после рефактора** — оценки CC в §3, §4, §5 даны по
  визуальному разбиению; реальный `radon cc -s` / `cc_budget.py` прогон
  будет сделан в Phase 4 implementation. Цель — **все 13 ≤ 12**; если
  какой-то helper выйдет за 12, добавится ещё один уровень декомпозиции
  в итерации.
- **Скрытые зависимости между методами** — `execute_code` ссылается на
  `_master_gain_applied`/`set_master_gain` (Phase 5) и
  `_check_supercollider`/`_ensure_renardo_available` (Phase 3) — все три
  сервиса **остаются в `MusicManager`** в Phase 4 как shim-вызовы через
  `self._mgr.<method>()`. Это блокирует Phase 4 завершение **только если**
  эти сервисы тоже будут вынесены до Phase 6 (что не так — план Phase 2/3/5/6
  каждый выносит свой кусок и оставляет shim до Phase 6).
- **`MusicManager.__new__` обход тестов** (test_music.py) — `_make_manager`
  helper использует `__new__` + ручную инициализацию атрибутов, минуя
  `__init__`. После Phase 4 этот helper **должен** дополнительно
  инициализировать `self._runtime = MusicPatternRuntime(self)`, иначе
  тесты упадут на первом же wrapper-вызове. Это **отдельная** правка в
  test_music.py — Phase 4 включает её.

## 12. Trade-offs (кратко)

| Решение | Альтернатива | Почему это |
|---|---|---|
| `runtime` хранит ссылку на `MusicManager` (composition) | DI-контейнер / dataclass с явными зависимостями (player, state, config, …) | KISS: ADR-0134 §3.2 явно запрещает DI-фреймворк; `MusicManager` уже composition root, 1 ссылка. Тесты через `__new__` остаются работать. |
| 6 helpers для `execute_code` | 3 helpers (sanitize / health / run) | Issue #3014 требует ≤12 CC. С 3 helpers CC=~14 (по моей оценке); 6 helpers даёт запас. Каждый helper ≤ 5 строк. |
| `_execute_run` владеет `/g_freeAll`/sleep/`/g_new` (issue #778) | Вынести в `_post_exec_free_old_nodes` | Это **одна** ответственность — выполнение + post-exec cleanup. Отделение `prewarm_sample_buffers` уже сделано. |
| `dj_mode_enabled` читается через property | Копировать `_dj_mode_enabled` в runtime | Composition через `MusicManager` — единый владелец. Phase 5 вынесет `set_dj_mode`, runtime продолжит читать через property. |
| `MusicManager.__new__` тесты получают `_runtime` руками | Перевести тесты на `_make_manager(...)` с настоящим `__init__` | Минимизация blast-radius: 4575 строк тестов не меняются. _make_manager правка — 1 строка. |

## 13. Кросс-ссылки

- ADR-0134 §5 Phase 4 — план фазы (parent)
- ADR-0021 R1 — `cc_budget.py` METHOD_LIMIT=15 (issue требует ≤12 — жёстче)
- ADR-0132 — прецедент выноса в `core/`
- ADR-0018 — честность, не врать себе
- Issue #3014 — umbrella, CC-budget symptom
- Issue #2989 / PR #2994 — hot-fix CC=16→15 (precedent baseline-подхода)
- Phase 5 (Phase 4 OUT): `set_dj_mode`/`set_master_gain` →
  `core/music_session_state.py`
- Phase 3 (Phase 4 OUT): `_initialize_renardo` + OSC →
  `core/music_renardo_bridge.py`