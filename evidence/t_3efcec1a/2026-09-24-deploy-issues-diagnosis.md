# Deploy issues on develop/staging 2026-09-24 — диагностика (issue #2997)

> Карточка: `t_3efcec1a`
> Workflow run: <https://github.com/krikz/rob_box_project/actions/runs/35990876432> (conclusion=success, deploy грин)
> Автор разбора: devops (агент Шифу)
> Дата: 2026-09-24

## TL;DR

Из трёх пунктов, помеченных `deploy-signature` в issue #2997, **один — реальный дефект**, два — ожидаемая работа системы:

| # | Симптом | Сорс | Реальный баг? |
|---|---------|------|--------------|
| 1 | `mcp_server` crash-loop (`FileNotFoundError: data/sample_fx.json`) | `voice-assistant` | **ДА** — пропущен `package_data` в `setup.py` (PR #2983) |
| 2 | Vision-hailo `WARN: hailortcli не установлен, но HAILO_ENABLED=true` | `vision-hailo` | нет — graceful degradation, Hailo ускорителя физически нет на этом VisionPi |
| 3 | `rtabmap` WARN: `OptimizerG2O.cpp:1324 Computing marginals: vertex 401 has negative hessian index (-1)` | `rtabmap` | нет — штатный WARN rtabmap при отсутствии visual features / начале работы SLAM |

Деплой-пайплайн (workflow run 35990876432) завершился `conclusion=success` — т.е. pull → up → healthcheck прошли. Все контейнеры Vision Pi и Main Pi в `docker ps` показывают `Up N minutes (healthy)`. Это не «деплой поломан», это «после деплоя в контейнерах сыпятся runtime-ошибки» — фикс живёт в коде/setup.py, а не в deploy-инфраструктуре.

## Реальный дефект #1: mcp_server crash-loop (CRITICAL)

### Сырое наблюдение (vision pi 10.1.1.21, deploy 24.09.2026 11:06 UTC)

```
$ docker exec voice-assistant bash -c "find /ws/install/rob_box_mcp_tools -name sample_fx* 2>&1"
/ws/install/rob_box_mcp_tools/lib/python3.10/site-packages/rob_box_mcp_tools/core/__pycache__/sample_fx.cpython-310.pyc
/ws/install/rob_box_mcp_tools/lib/python3.10/site-packages/rob_box_mcp_tools/core/sample_fx.py
# data/sample_fx.json ОТСУТСТВУЕТ

$ docker logs voice-assistant --since 10m 2>&1 | grep -c "mcp_server.*process has died"
33   # 33 краша за 10 минут = crash-loop каждые ~20 секунд

$ docker logs voice-assistant --tail 200 2>&1 | grep -E "mcp_server.*FileNotFoundError|Traceback"
[mcp_server-10] Traceback (most recent call last):
  ...
  File ".../rob_box_mcp_tools/tools/music.py", line 2625, in <module>
    enum=sorted(sample_fx.fx_catalog()),
  File ".../rob_box_mcp_tools/core/sample_fx.py", line 71, in fx_catalog
    raw = json.loads(_CATALOG_FILE.read_text(encoding="utf-8"))
  FileNotFoundError: [Errno 2] No such file or directory:
    '/ws/install/rob_box_mcp_tools/lib/python3.10/site-packages/rob_box_mcp_tools/data/sample_fx.json'
[ERROR] [mcp_server-10]: process has died [pid 1189, exit code 1, ...]
```

### Корневая причина

`core/sample_fx.py:71` читает JSON-каталог из `_CATALOG_FILE = Path(__file__).resolve().parent.parent / "data" / "sample_fx.json"`. В репо (`origin/develop` SHA `259b2f89e`) этот файл существует:

```bash
$ git show origin/develop:src/rob_box_mcp_tools/rob_box_mcp_tools/data/sample_fx.json
{ "_comment": [...], "pack1_dir": "1_pitchglitch_samples",
  "fx": { "gunshot_1": {...}, "gunshot_2": {...}, ... } }
```

Но в `setup.py:package_data['rob_box_mcp_tools.data']` (тот же коммит) `sample_fx.json` НЕ указан — глоб `*.yaml` ловит только `.yaml`, и явно перечислены только `sample_loops.json` + `arrangement_presets.json`:

```python
package_data={
    'rob_box_mcp_tools.data': [
        '*.yaml', 'rtttl_melodies.jsonl.gz', 'sample_loops.json',
        'arrangement_presets.json',
    ],
},
```

setuptools копирует в install-дерево только то, что перечислено в `package_data` (или попадает под glob). `.json` под `*.yaml` не подходит, и `sample_fx.json` явно не указан → файл не доезжает до `/ws/install/.../data/`.

### Автор дефекта

PR #2983 (`0f911419d feat(music #2968): FX-слой в compose_music…`) — добавил `core/sample_fx.py` + `data/sample_fx.json`, но **забыл обновить `setup.py:package_data`**. Фикс — добавить `'sample_fx.json'` в список.

### Артефакты деплоя, на которых воспроизводится

- Build-run `35978835380` (24.09.2026 09:02:40Z → 09:11:56Z, headSha `0f911419d`): `conclusion=success`, собрал образ с `core/sample_fx.py`, но без `data/sample_fx.json` в install.
- Deploy-run `35990876432` (24.09.2026 11:04:17Z → 11:09:48Z, headSha `259b2f89e`): `conclusion=success`, выкатил этот образ на VisionPi.
- Контейнер `voice-assistant` на 10.1.1.21 создан `2026-09-24T11:06:20Z` (= сразу после pull), `mcp_server` стартует PID 127/929/1189… и падает в `FileNotFoundError`.

## Фикс (коммит на ветке `z-devops/2997-fix-sample-fx-package-data`)

1. `src/rob_box_mcp_tools/setup.py` — добавлен `'sample_fx.json'` в `package_data['rob_box_mcp_tools.data']`.
2. `src/rob_box_mcp_tools/test/test_package_data_includes_data_files.py` — **новый** регрессионный тест, который для каждого `.json/.yaml/.jsonl*` в `data/` проверяет покрытие в `setup.py:package_data` (явное имя или glob).

### Прогон тестов

```
$ PYTHONPATH=src python3 -m pytest src/rob_box_mcp_tools/test/test_package_data_includes_data_files.py -v
collected 5 items
src/rob_box_mcp_tools/test/test_package_data_includes_data_files.py::test_data_file_covered_by_package_data[arrangement_presets.json] PASSED
src/rob_box_mcp_tools/test/test_package_data_includes_data_files.py::test_data_file_covered_by_package_data[rtttl_melodies.jsonl.gz] PASSED
src/rob_box_mcp_tools/test/test_package_data_includes_data_files.py::test_data_file_covered_by_package_data[sample_fx.json] PASSED   # ← главная цель теста
src/rob_box_mcp_tools/test/test_package_data_includes_data_files.py::test_data_file_covered_by_package_data[sample_loops.json] PASSED
src/rob_box_mcp_tools/test/test_package_data_includes_data_files.py::test_data_file_covered_by_package_data[slice_policy.yaml] PASSED

5 passed in 0.06s
```

Sanity check — тест ловит регрессию (на сломанном `setup.py` падает только `sample_fx.json`):

```
$ git stash   # откатил setup.py на «битый»
$ PYTHONPATH=src python3 -m pytest ...test_package_data_includes_data_files.py
FAILED ...::test_data_file_covered_by_package_data[sample_fx.json]
  AssertionError: data/sample_fx.json не покрыт setup.py:package_data. ...
1 failed, 4 passed
```

Существующие тесты `test_sample_fx.py` (20 тестов) + `test_arranger_fx.py` (24 теста) после фикса тоже зелёные — `49 passed in 0.14s`. Регрессия не ломает контракт sample_fx.

## Дефекты #2 и #3 — ложные срабатывания (для issue-комментария)

### Vision Pi: `hailortcli не установлен, но HAILO_ENABLED=true`

`vision-hailo` стартует в режиме graceful degradation: если `hailortcli` нет (нет Hailo-ускорителя на VisionPi), нода работает без него и просто не публикует inference. Это by-design — VisionPi несёт камеры (OAK-D, ceiling), но не содержит Hailo-8 модуля. Лог:

```
$ docker exec vision-hailo bash -c "which hailortcli"
# пусто — hailortcli не установлен
$ docker ps | grep vision-hailo
vision-hailo   Up 4 minutes (healthy)
```

Никаких действий не требуется — `vision-hailo` `healthy` (healthcheck не падает). Если Шифу хочет чистый лог без WARN, фикс — это убрать `HAILO_ENABLED=true` из env vision-hailo для этой машины, либо установить hailortcli (отдельный workstream, не deploy-issue).

### Main Pi: `rtabmap` WARN `negative hessian index` + `Missing visual features`

rtabmap WARNING о `OptimizerG2O::optimize() Computing marginals: vertex 401 has negative hessian index (-1)` — это **известный штатный WARN** библиотеки rtabmap (см. [rtabmap/libpointmatcher issue #19](https://github.com/introlab/rtabmap/issues/19) и `Memory.cpp:3776::computeTransform() Missing visual features`). Предупреждение появляется в начале работы SLAM, когда граф ещё не сформирован и нет визуальных фич для computeTransform. rtabmap остаётся работоспособным, никакие данные не теряются — оптимизатор просто пропускает этот шарик и продолжает.

Дополнительно: `Sensor UART /dev/ttyAMA0 not available; reads will no-op until hardware is attached.` — `perception_bridge` явно стартует в stub-режиме (`stub=True period=0.1s`), это тоже by-design.

Никаких действий не требуется. rtabmap и perception `Up 3 minutes` без healthcheck-фейлов.

## Что делать

- **Шифу/merge-gate**: мерджить PR (devops создаст, когда подтвердит CI) с фиксом `setup.py` + регрессионным тестом — фикс тривиальный (1 строка), тест защищает от повторения класса багов.
- **e2e-process**: после merge + нового деплоя `mcp_server` перестанет падать (на стейджинге, потом на проде по расписанию). e2e на этом фиксе не нужен — дефект чисто packaging/setuptools, runtime-эффект 100% воспроизводится через `docker logs voice-assistant | grep FileNotFoundError`.
- **Никаких ручных `docker restart`/`sed -i` на роботе**: AGENTS.md прямо запрещает править то, что в ревизионных файлах. Следующий деплой сольёт.

## Где живёт артефакт

- `src/rob_box_mcp_tools/setup.py` — фикс.
- `src/rob_box_mcp_tools/test/test_package_data_includes_data_files.py` — регрессионный тест.
- Комментарий в issue #2997 (планируется отправить через `gh issue comment`).
- PR (планируется): base=develop, head=`z-devops/2997-fix-sample-fx-package-data`.