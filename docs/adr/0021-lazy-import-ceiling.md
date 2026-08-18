# ADR-0021: Lazy-import ceiling — обязательные зависимости на top-of-file

Status: accepted (refactor, ARCH-review #1405)
Date: 2026-08-18
Deciders: backend (kanban t_f53fe5ef)
Refs: issue #1413, issue #1405

## Context

В `rob_box_voice/dialogue_node.py` 18 lazy-импортов внутри тел методов.
Среди них:

- **stdlib (must-be-top)**: `yaml`, `traceback`, `uuid`, `concurrent.futures`
  — все уже либо top-level, либо дублируются.
- **Дубликаты**: `os as _os` (top-level `os` уже на line 24);
  `from rob_box_harness.core.dialogue_state_machine import (DialogueEvent
  as _DE, DialogueStateKind as _DSK)` (модуль уже на top с нормальными
  именами, line 39).
- **Optional**: skill-классы (top-level try/except), `rob_box_mcp_tools`
  (нет `<exec_depend>`).
- **Same-package**: `rob_box_voice.scheduler.tool_executor`,
  `rob_box_voice.tts_voice_registry`.

**Проблема**: lazy-импорты маскируют ошибки до момента первого вызова.
Если пакет `yaml` исчезнет из multi-stage build — `voice-assistant`
стартует, но **первый TTS-batch падает** с `ImportError` вместо
честного `ModuleNotFoundError` на старте. Это нарушает принцип
fail-fast и усложняет дебаг в проде (live 12.08 — аналогичный баг с
`RcutilsLogger`-импортом, ретро #1356).

**Аудит показал 18 lazy-импортов на 4000+ строк — слишком много**.
Без формального правила «lazy = только optional» количество продолжит
расти.

## Решение

**Lazy-import ceiling: максимум 2 на файл, только для optional
зависимостей.**

### Категории импортов и правила

| Категория | Примеры | Куда |
|---|---|---|
| **stdlib** | `yaml`, `traceback`, `uuid`, `concurrent.futures`, `os` | **top-of-file (обязательно)** |
| **declared `<exec_depend>`** | `rob_box_harness.health`, `rob_box_harness.config`, `rob_box_harness.memory` | **top-of-file (обязательно)** |
| **Same-package** | `rob_box_voice.tts_voice_registry`, `rob_box_voice.scheduler.*`, `rob_box_voice.core.*` | **top-of-file** |
| **Optional feature** (try/except + `is_*_enabled` guard) | `rob_box_voice.skills.*`, `prometheus_client` через `is_metrics_enabled` | **lazy (обосновано)** |
| **Optional runtime dep** (нет в package.xml) | `rob_box_mcp_tools.llm_adapter` (нет `<exec_depend>`), `ament_index_python.packages` (probe-based, ROS runtime only) | **lazy (обосновано)** |

### Что ЗАПРЕЩЕНО

- **Дубликаты top-level импортов** внутри методов (`import uuid as _uuid`
  когда `uuid` уже на top).
- **Lazy stdlib** (`import yaml as _yaml` внутри функции) — должен быть
  на top, иначе невалидный build не падает на старте.
- **Lazy declared deps** (`from rob_box_harness.X import Y` внутри
  метода) — top, иначе отсутствующий dep не падает до первого вызова.

### Что ОБОСНОВАНО остаётся lazy

- `from ament_index_python.packages import get_package_share_directory`
  — используется в try/except с fallback на default prompt;
  probe-pattern (для `tool_provider='ros_mcp'`) тоже обёрнут в
  try/except. На top это уронит диалог-ноду при отсутствии
  `ament_index_python` (что нереально в ROS2-окружении, но возможно в
  изолированных тестах без conftest).
- `from rob_box_mcp_tools.llm_adapter import LLMToolCallAdapter` —
  явно optional: комментарий в коде говорит «no <exec_depend>»; грузится
  только при `tool_provider='ros_mcp'`. На top — лишняя зависимость для
  тех, кто использует `fake`/`none`.
- `from rob_box_voice.scheduler.tool_executor import
  SchedulerToolExecutor` — W7b fail-open: если scheduler не стартует,
  адаптер возвращается unwrapped, voice не ломается. Lazy — чтобы
  unit-тесты, не поднимающие scheduler, не зависели от опциональной
  инфраструктуры.
- Skill-модули (`rob_box_voice.skills.*`) — top-level try/except,
  optional features, импортятся как `MusicSkill = None` если модуль
  сломан.

### Метрики

- **Baseline** (до рефактора): 18 lazy-импортов в `dialogue_node.py`.
- **После рефактора (#1413)**: 10 lazy-импортов (все justified).
- **Ceiling** (формула): `lazy_count ≤ 2 × N_optional_features`. Для
  dialogue_node: 6 skills + 2 ament_index + 1 mcp + 1 scheduler = 10.
  Потолок на 2026-08: **≤ 12** (запас 2 на новые optional).

## Enforcement

**Тест-регрессор**: `test/unit/node/test_dialogue_node_imports.py`
(добавлен в #1413). Проверяет:

1. `test_top_of_file_has_required_imports` — yaml/traceback/uuid/
   concurrent.futures/tts_voice_registry/rob_box_harness.health
   **обязаны** быть top-of-file.
2. `test_no_lazy_stdlib_or_required_imports` — stdlib и declared
   зависимости **не должны** появляться как lazy.
3. `test_lazy_imports_within_whitelist` — все lazy должны быть в
   whitelist (justified) или добавить в whitelist с обоснованием.
4. `test_lazy_import_ceiling_count` — total lazy ≤ 12.
5. `test_no_duplicate_imports_of_top_level_modules` — нет дублей
   top-level внутри функций.
6. `test_yaml_safe_load_uses_top_level_binding` — нет ссылок на
   `_yaml`/`_tb`/`_uuid`/`_os` (удалённые lazy-алиасы).

CI (`G-Run Tests.yml` → colcon test) запускает этот тест после
`colcon build`. При регрессии PR получает red check от pytest.

## Альтернативы

### A. Оставить всё как есть
- ❌ Не решает проблему fail-fast. 18 lazy-импортов — тех-долг, который
  продолжит накапливаться.

### B. Жёсткий лимит: «0 lazy-imports, кроме как в try/except ImportError»
- ❌ Ломает `rob_box_mcp_tools` (нужен только при `tool_provider='ros_mcp'`)
  и `scheduler.tool_executor` (W7b fail-open). Пакет
  `rob_box_voice` перестанет быть самодостаточным без необязательных
  зависимостей.

### C. **Принято**: ceiling 12 с whitelist для optional + автоматический
регресс-тест.
- ✅ Top-of-file: всё обязательное. Fail-fast при сломанном build.
- ✅ Lazy остаётся только для обоснованных optional.
- ✅ Регресс-тест ловит новые нарушения в PR-чеке.
- ✅ Не ломает существующие optional-флоу (mcp, scheduler, skills).

## Consequences

### Положительные
- Import-time валидация build: если declared dep сломан, нода падает
  на старте, а не на первом TTS-batch.
- Удалены 3 дубля + 1 алиас (`os as _os`) — экономия 33 строк,
  читаемость +.
- Регресс-тест автоматически отлавливает новые нарушения.
- ADR делает правило явным — новые разработчики сразу знают «не
  добавляй lazy без обоснования».

### Отрицательные / риски
- `import yaml` на top — если пакет реально исчезнет из requirements.txt,
  `voice-assistant` перестанет стартовать. Митигация: requirements.txt
  декларирует `pyyaml>=6.0.3`, при сборке Docker-image проверяется
  pip-install.
- `from rob_box_harness.health import ...` на top — если submodule health
  сломается, нода упадёт. Митигация: `rob_box_harness` объявлен как
  `<exec_depend>` и проверяется в `colcon build`.
- Тест читает AST, не запускает import. Если кто-то напишет
  `__import__("yaml")` динамически, тест не поймает. Митигация: такое
  считается нарушением code-style, ловится на code review.

## Rollout

- **Phase 1** (этот ADR, kanban t_f53fe5ef): рефактор dialogue_node.py
  + ADR + регресс-тест.
- **Phase 2** (deferred): аудит остальных модулей
  (`audio_node.py`, `tts_node.py`, `stt_node.py`, `command_node.py`).
  Шаблон: тот же регресс-тест, адаптированный под каждый модуль.
- **Phase 3** (deferred): pre-commit hook, который запускает
  `test_dialogue_node_imports.py` (через `pytest --co`) и блокирует
  коммит при новых нарушениях.

## Acceptance

(См. issue #1413 / kanban t_f53fe5ef)

- [x] Перенесены на top: yaml, traceback, uuid (уже), concurrent.futures
      (уже), tts_voice_registry, rob_box_harness.health.
- [x] Удалён дубль `os as _os`.
- [x] Удалён дубль `from rob_box_harness.core.dialogue_state_machine
      import ... as _DE, _DSK` — заменён на прямое использование
      top-level имён.
- [x] Lazy kept (justified): skills.*, ament_index_python.*,
      rob_box_mcp_tools.*, rob_box_voice.scheduler.*.
- [x] Регресс-тест `test_dialogue_node_imports.py` (8 кейсов).
- [x] ADR-0021 (этот документ).

## See also

- ARCH-review #1405 (proactive counter: `_llm_skipped_counter` через SSoT).
- Issue #1405 (SSoT кортеж).
- Issue #1356 retro (RcutilsLogger-monkey-patch, принцип fail-fast).
- Issue #1404 (рефактор voice → ADR серия).
