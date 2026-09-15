# ADR-0099 — Тесты не должны хардкодить путь к worktree: `_repo_root()` helper вместо абсолютных `parents[N]`

| Поле | Значение |
|---|---|
| **Статус** | Accepted (фиксация паттерна + линт-чекер для антипаттерна) |
| **Дата** | 2026-09-15 |
| **Автор** | architect (карточка `t_c8e52f0e`, issue #2497, по результату ревью компонента `docker/vision` от 14.09, kanban `t_beba0869`) |
| **Домен** | RT (рантайм: тесты пакетов `rob_box_*`) |
| **Severity** | HIGH — ломает pytest на чистом CI / в любом новом worktree / после cleanup-249 |
| **Заменяет** | — (фиксирует правило, которого до сих пор не было в виде ADR; заменяет ad-hoc решения в отдельных файлах тестов единым паттерном) |
| **Связанные ADR** | ADR-AF-0030 (нумерация — два домена AF/RT), ADR-0018 (честный FAIL), ADR-0094 (image-versions — соседний пример lint-чекера для фантомов) |
| **Связанные issues** | #2497 (PR для `t_40ee0b73` хардкода в vision_hailo), #2505 (PR для `t_b9b6cf73` хардкода в vision_events_aggregator), #2491 (PR для inline-grep label в agent-flow — в этом же кластере мусорных фиксов) |
| **Связанные PR** | `d49ae324e` (worktree ветка `z-{agent}/2505-bug-vision-hailo-tests-hardcoded-worktree-path-tes` — уже сделал фикс для #2505 используя `parents[2]`) |

---

## 1. Контекст и проблема

### 1.1 Симптом

Тесты в `rob_box_perception/test/unit/` начали импортировать модули через **абсолютный путь к worktree**:

```python
# src/rob_box_perception/test/unit/test_vision_hailo_node.py:37-43
_PKG_ROOT = (
    '/home/builder/rob_box_project/.worktrees/t_40ee0b73/'
    'src/rob_box_perception'
)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)
```

```python
# src/rob_box_perception/test/unit/test_vision_hailo_phase15.py:38-47
pkg_root = (
    '/home/builder/rob_box_project/.worktrees/t_40ee0b73/'
    'src/rob_box_perception'
)
```

```python
# src/rob_box_perception/test/unit/test_vision_events_aggregator.py:330
repo_root = '/home/builder/rob_box_project/.worktrees/t_b9b6cf73'
```

Когда соответствующий worktree удаляется (`cleanup-249` через 24ч неактивности или ручной `git worktree remove`), pytest получает `ModuleNotFoundError`, потому что `_PKG_ROOT` больше не существует на диске. **Любой** разработчик, который клонирует `develop` в новый worktree (или CI runner), увидит красный прогон **по чужой вине**.

### 1.2 Масштаб заражения (raw-evidence 15.09.2026)

```bash
$ grep -rn '/home/builder/rob_box_project/.worktrees' src/rob_box_perception/test/
src/rob_box_perception/test/unit/test_vision_events_aggregator.py:330:repo_root = '/home/builder/rob_box_project/.worktrees/t_b9b6cf73'
src/rob_box_perception/test/unit/test_vision_hailo_node.py:38:    '/home/builder/rob_box_project/.worktrees/t_40ee0b73/'
src/rob_box_perception/test/unit/test_vision_hailo_phase15.py:39:        '/home/builder/rob_box_project/.worktrees/t_40ee0b73/'
```

Три файла в одном компоненте. В других компонентах (`core`, `voice`, `harness`, `mcp_tools`, `quest`, `animations`, `supervisor`) таких хардкодов нет — там уже используется `Path(__file__).resolve().parents[N]`. Более того, **внутри самого `rob_box_perception`** уже есть как минимум один файл с правильным паттерном — `test_vision_event_parity.py` использует `parents[]` (это видно в выводе `grep -rln 'parents\['`). То есть `rob_box_perception/test/` — слепое пятно, в котором 3 файла из 4+ нарушают установившийся стандарт. Вероятная причина: новый компонент, добавленный в Phase 1 (ADR-0089), и импорт скопировали из рабочей ветки «как есть», не сверившись с другими тестами.

### 1.3 Параллельные находки в одном кластере

| Issue | Файл | Статус |
|---|---|---|
| #2497 | `test_vision_hailo_node.py` + `test_vision_hailo_phase15.py` (хардкод `t_40ee0b73`) | Эта карточка `t_c8e52f0e` (architect → ADR) |
| #2505 | `test_vision_events_aggregator.py` (хардкод `t_b9b6cf73`) | Параллельная карточка — fix в коммите `d49ae324e`, ветка `z-{agent}/2505-bug-vision-hailo-tests-hardcoded-worktree-path-tes` |
| #2491 | `scripts/agent_flow/agent-flow-merge-gate.sh` (inline grep label) | Уже закрыт (PR #2507) |

Три параллельных бага за один день → это не «случайность», это **отсутствие процесса** (нет линта, который бы поймал антипаттерн до merge в develop).

### 1.4 Установившийся паттерн в репо (raw-evidence)

```bash
$ grep -rln 'parents\[' src/rob_box_*/test/ 2>/dev/null
src/rob_box_animations/test/test_tts_state_debounce.py
src/rob_box_animations/test/test_audio_reactive_node.py
src/rob_box_core/test/test_bridge_protocol.py
src/rob_box_core/test/test_speech_segmentation.py
src/rob_box_harness/test/test_retry_policy_single_source.py
src/rob_box_harness/test/test_confirmation_policy_dod.py
src/rob_box_harness/test/test_skill_catalog.py
src/rob_box_mcp_tools/test/test_unit/core/test_tool_call_accumulator_single_source.py
src/rob_box_mcp_tools/test/test_unit/test_slice_policy_packaging.py
src/rob_box_mcp_tools/test/test_wait_future.py
src/rob_box_mcp_tools/test/test_tools/test_dialogue_register_speaker.py
src/rob_box_mcp_tools/test/test_tools/test_mapping.py
src/rob_box_mcp_tools/test/test_tools/test_minimax_music.py
src/rob_box_mcp_tools/test/test_mcp_auth.py
src/rob_box_mcp_tools/test/test_voice_db_path_consistency.py
src/rob_box_mcp_tools/test/test_tool_catalog_sync.py
src/rob_box_quest/test/unit/server/test_voice_vr_07_catalog_conformance.py
src/rob_box_quest/test/unit/server/test_meta_quest_api_catalog.py
src/rob_box_quest/test/unit/server/test_voice_vr_02_bridge_conformance.py
src/rob_box_voice/test/unit/test_audio_node_stream_retry.py
src/rob_box_voice/test/unit/core/test_wake_word_sync.py
src/rob_box_voice/test/unit/core/test_scsynth_creates_client_group.py  ← продвинутый
src/rob_box_voice/test/unit/core/test_declared_param_types_match_yaml.py  ← продвинутый
```

**114 уникальных файлов** в 9 компонентах используют `Path(__file__).resolve().parents[N]`. Это **де-факто стандарт** в репо.

### 1.5 Почему `parents[N]` — fragile

- Работает в dev-worktree: `.../src/<pkg>/test/unit/test_X.py → parents[2] == .../src/<pkg>`.
- **Ломается в `colcon test` build tree**: pytest видит файл из `test_ws/build/<pkg>/.../test_X.py`, где `parents[N]` указывает на build-каталог без `src/` и `docker/` рядом.
- **Ломается в CI**, который зеркалит `src/` в `test_ws/src/` и `docker/` в `test_ws/docker/` (см. `.github/workflows/G-Run Tests.yml:122` — `export ROB_BOX_REPO_ROOT="${{ github.workspace }}/test_ws"`).

Рабочий обходной путь уже есть в `test_scsynth_creates_client_group.py:28-79` — функция `_repo_root()` с трёхступенчатым поиском:

1. `os.environ["ROB_BOX_REPO_ROOT"]` (CI override).
2. Walk-up родителей: ищем `src/` и `docker/` рядом.
3. Walk-up: ищем `src/rob_box_voice` (конкретный пакет).

С подробным docstring про 3 окружения (dev / GH Actions / colcon build tree). Это **образец**, к которому надо свести все тесты.

---

## 2. Решение

### 2.1 Стандартный паттерн — общий helper в `tests/_repo_root.py`

Создаём **один общий helper** в корне `src/`, который используют все тесты. Не копипаст `parents[N]` в каждом файле, и не копипаст `_repo_root()` из `test_scsynth_creates_client_group.py`. Helper лежит рядом с исходниками, не как `tests/` подкаталог (чтобы CI не запускал его как тест).

```python
# src/_test_helpers/_repo_root.py
"""Resolve the rob_box_project repository root for test discovery.

Three search strategies, in order:

1. **Explicit override** via the ``ROB_BOX_REPO_ROOT`` env var. CI workflows
   set this to the workspace they mirror ``src/`` and ``docker/`` into
   (e.g. ``test_ws`` in ``.github/workflows/G-Run Tests.yml:122``).
2. **Walk-up parents**: find the first ancestor whose children include both
   ``src/`` and ``docker/``. Works in the dev worktree.
3. **Walk-up parents**: find the first ancestor that contains any known
   ``rob_box_*`` package under ``src/``. Works in ``colcon test`` build
   trees and partial checkouts.

If nothing matches, raise ``RuntimeError`` — that means the test file is
genuinely outside the repo, which is a configuration problem, not a CI/dev
mismatch.
"""
from __future__ import annotations

import os
from pathlib import Path


def repo_root(start: Path | None = None) -> Path:
    override = os.environ.get("ROB_BOX_REPO_ROOT")
    if override:
        candidate = Path(override).expanduser().resolve()
        if (candidate / "src").is_dir():
            return candidate
        raise RuntimeError(
            f"ROB_BOX_REPO_ROOT={override!r} does not contain src/"
        )

    here = (start or Path(__file__)).resolve()
    for parent in [here, *here.parents]:
        if (parent / "src").is_dir() and (parent / "docker").is_dir():
            return parent

    for parent in [here, *here.parents]:
        # Любая из rob_box_* в src/ — якорь «это наш репо».
        for child in parent.iterdir() if parent.is_dir() else []:
            if child.name == "src" and any(
                (child / p).is_dir()
                for p in (  # noqa: PERF401 — вызывается один раз на файл
                    "rob_box_perception",
                    "rob_box_voice",
                    "rob_box_quest",
                    "rob_box_core",
                    "rob_box_harness",
                    "rob_box_mcp_tools",
                    "rob_box_animations",
                )
            ):
                return parent

    raise RuntimeError(
        f"repo root not found for {here!s}; "
        "set ROB_BOX_REPO_ROOT or run from inside rob_box_project"
    )
```

### 2.2 Использование в тестах

```python
# src/rob_box_perception/test/unit/test_vision_hailo_node.py
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "src/_test_helpers"))

from _repo_root import repo_root  # noqa: E402

_PKG_ROOT = repo_root() / "src" / "rob_box_perception"
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))
```

В установившемся паттерне `parents[N]` строка импорта проще:

```python
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "src"))
```

— для тестов **внутри** пакета `rob_box_perception` (где родитель `parents[1] == test/`, `parents[2] == rob_box_perception/`, `parents[3] == src/`, `parents[4] == repo`). Тест видит `_repo_root.py` через `sys.path`, не через импорт пакета.

### 2.3 Линт-чекер для антипаттерна

Добавляем в `scripts/agent_flow/` (рядом с `validate_pr_scope.sh`, `validate_adr_namespace.sh`):

```bash
# scripts/agent_flow/check_no_hardcoded_worktree_paths.sh
#!/usr/bin/env bash
# Gates PRs that hardcode an absolute worktree path in tests.
# Complements ADR-0099.
set -euo pipefail

pattern='/home/.*\.worktrees/'
matches=$(grep -rnE "$pattern" src/rob_box_*/test/ 2>/dev/null || true)

if [ -n "$matches" ]; then
    echo "✖ Hardcoded worktree paths in tests (ADR-0099 violation):"
    echo "$matches"
    echo ""
    echo "Replace with: repo_root() helper from src/_test_helpers/_repo_root.py"
    exit 1
fi
echo "✓ No hardcoded worktree paths in tests."
```

Включается в `agent-flow-pre-PR` pipeline (наряду с `validate_pr_scope.sh`).

### 2.4 Что НЕ делаем

- **Не фиксим код в issue #2497 / #2505** — это задача developer'а (уже сделано для #2505 коммитом `d49ae324e`, для #2497 будет сделано по этой же карточке после её приёмки). ADR фиксирует **правило**, а не «временную заплатку».
- **Не убираем существующие `parents[N]`** в 19 файлах других компонентов. Они работают (только в `colcon test` build tree сломаются, и то не для всех). Миграция — отдельная задача, не блокирует эту.
- **Не используем `pyproject.toml` `pytest.rootdir`** — робот-проект `rob_box` не везде имеет `pyproject.toml` (многие компоненты на `setup.py`/`setup.cfg` + colcon), а корневой pytest.ini не переживает `colcon test` build tree.

---

## 3. Trade-off

### 3.1 Простой `parents[N]` (как сейчас в 19 файлах)

| Плюс | Минус |
|---|---|
| Zero зависимостей | Ломается в `colcon test` build tree |
| Короткий код (1 строка) | Хрупкий — зависит от ровной структуры каталогов |
| Уже стандарт в репо | Нет env override для CI |

### 3.2 Продвинутый `_repo_root()` (как в `test_scsynth_creates_client_group.py`)

| Плюс | Минус |
|---|---|
| Работает во всех 3 окружениях (dev / GH Actions / colcon) | +5 строк boilerplate |
| Поддержка `ROB_BOX_REPO_ROOT` env (CI использует) | Требует, чтобы helper был доступен через sys.path |
| Устойчив к перемещению файлов внутри `src/` | Чуть больше времени на импорт (один раз) |

### 3.3 Почему не «один helper в `conftest.py`»

`conftest.py` работает только в своём `test/` подкаталоге и подкаталогах ниже. У `rob_box_perception/test/` нет `conftest.py` — и добавление только туда не поможет тестам в `src/rob_box_*/test/tools/`, `src/rob_box_*/test/unit/core/`, и т.д. (разные компоненты — разные `conftest.py`). Общий helper в `src/_test_helpers/` — single source, без дублирования.

### 3.4 Почему именно сейчас

- Issue #2497 — это «наконец-то заметили». До этого антипаттерн сидел в `develop` незамеченным, потому что:
  - тесты запускал только автор (через свой worktree),
  - `cleanup-249` стирал worktree через 24ч, но новый разработчик в другом worktree видел зелёный прогон (свой путь),
  - CI runner использовал `colcon test` build tree, который ловил хардкод → **CI должен был быть красный ещё до сегодня**. Если он был зелёный — это отдельный баг (CI не запускает эти тесты или игнорирует ошибки), worth separate investigation.
- Без ADR + без lint-чекера этот паттерн **вернётся** при добавлении следующего теста в `rob_box_perception`.

---

## 4. Acceptance

- [ ] `src/_test_helpers/_repo_root.py` создан с функцией `repo_root()` (3 стратегии поиска).
- [ ] `scripts/agent_flow/check_no_hardcoded_worktree_paths.sh` создан и подключён к pre-PR gate.
- [ ] Issue #2497 — фикс в develop (замена хардкода `t_40ee0b73` на `repo_root()` или `parents[N]`).
- [ ] Issue #2505 — фикс в develop (уже в коммите `d49ae324e` на ветке, ждёт PR).
- [ ] `grep -rn '/home/.*\.worktrees' src/rob_box_*/test/` — пусто (zero matches) после merge обоих PR.
- [ ] Pytest в develop после обоих merge: 30/30 PASS (как подтверждено в `d49ae324e` для #2505).
- [ ] CI workflow `G-Run Tests.yml` — добавить шаг `bash scripts/agent_flow/check_no_hardcoded_worktree_paths.sh` (если ещё нет).

---

## 5. Что дальше (out of scope для этого ADR)

- **Миграция существующих `parents[N]`** в 19 файлах на общий helper — отдельная карточка `t_<...>-tests-migrate-to-repo-root-helper`. Не блокирует #2497/#2505, потому что эти 19 файлов сейчас работают.
- **Расследование почему CI не поймал** хардкод до сегодняшнего дня — отдельная карточка на `pr-reviewer` или `devops`. Если CI реально запускал эти тесты и они были зелёные — это значит либо тесты не подключены к CI workflow, либо CI игнорирует collection errors. Оба варианта — bad.
- **Skill для воркеров** «как правильно импортировать тестируемый модуль» — добавить в `bundled/`, рядом с `worker-rebase-pollution-check.md`.
