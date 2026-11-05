# ADR-0166: Ревизия структуры репо — корень, каталоги, мёртвые скрипты и пакеты

| Поле | Значение |
|---|---|
| Статус | Accepted (товарищ Шифу, чат приложения 05.11.2026; ответы на §11 — `docs/shifu-decisions.md`, запись `d-20261105-repo-structure`; вопрос №10 — запись `d-20261105-cpr-office`). В этом PR ничего не удалено и не перенесено — чистка идёт отдельными PR по §10 |
| Дата | 2026-11-05 |
| Автор | Claude Code (шиди) по задаче товарища Шифу 05.11 (чат): «проверить структуру всего проекта — в корне и вообще везде есть какие-то файлы и папки, осколки каких-то решений… провести ревизию структуры всего проекта» |
| Домен | RT-нумерация (репо целиком) |
| Затрагивает | корневые файлы, `.agents/`, `.devcontainer/`, `.github/`, `.vscode/`, `assets/`, `docker/`, `evidence/`, `examples/`, `host/`, `local_test/`, `migrations/`, `packaging/`, `scripts/`, `sound_pack/`, `src/`, `tests/`, `tools/`, `tts_audio_bench/`, `.gitignore` |
| Не затрагивает | содержимое `docs/` — это ADR-0163 (карта, архив, свалки); процесс и hermes — ADR-0162 (уже выполнены: `openspec/`, `specs/`, `.planning/`, `.hermes/`, `.kanban/`, `scripts/agent_flow/`, корневой `analysis/` в develop отсутствуют) |
| Родители | ADR-AF-0013 (маленькие PR), ADR-0018 (честность), ADR-0162, ADR-0163 |

> **Коротко.** Репо в целом живое: из 20 корневых каталогов в git 14 живы целиком (их читают CI, compose, Dockerfile, тесты),
> 3 — частично (`tools/`, `local_test/`, `examples/`), 3 — мертвы или под вопросом (`.devcontainer/`, `tts_audio_bench/`, `.vscode/`).
> Явный мусор небольшой: **4 корневых файла** (`clog`, `coverage.json`, два пустых маркера hermes),
> **1 workflow** с пометкой DEPRECATED, **6 одноразовых скриптов**, **4 устаревшие копии** стартовых скриптов
> в `docker/scripts/`, «осиротевший» `tests/integration/` и тест на несуществующий модуль.
> Крупнее — **«тесты-декорации»**: около 25 тестов лежат в репо, но ни один workflow их не запускает.
> И **тяжёлые бинарники**: 61 МБ текстур офиса Clearpath (лицензия не указана) и 3 МБ дублей STL.
> Спорное (dev-утилиты `local_test/`, стенд `tts_audio_bench/`, `rob_box_bringup`, сообщения сенсорного хаба) — в вопросах Шифу.

Метки достоверности: **измерено** — есть вывод команды (§1, тело PR); **по коду** — прочитано в файлах; **догадка** — без проверки.

---

## 1. Как мерил

База: `origin/develop` @ `acfa00560` (05.11.2026). Все команды — в рабочей копии, без робота и без сети, кроме `gh run list`.

- Состав и размер: `git ls-files --format='%(objectsize) %(path)'` → 3998 файлов, 144 МБ в HEAD. История: `git count-objects -vH` → `size-pack: 471.80 MiB`.
- Последний коммит: `git log --format='C %ad' --date=short --name-only`, первое появление пути.
  **Осторожно:** дата `2026-09-28` у многих файлов — это merge-коммит релиза `3941f8486 chore: release develop to main (#3098)` (2024 файла), а не правка. Такая дата значит «не позже», реальная правка старше. Так же даты 03.11–05.11 у старых файлов бывают от массовых PR (`#4281`, `#4411`, `#4415`), которые задели строку-комментарий.
- «Кто читает»: поиск имени файла (для `.py` — ещё имени модуля) по всем отслеживаемым текстовым файлам, отдельно код (`src/ scripts/ .github/ docker/ tools/ tests/ local_test/ host/ .agents/`, корень) и `docs/`. Самоссылка не считается. Поиск по имени ловит и ложные совпадения (`main.py`, `app.py`), такие строки разобраны руками.
- «Запускается ли тест»: `grep` по `.github/workflows/*.yml` на путь файла **и** на каталог (`pytest scripts/testing/`, `pytest tests/unit`).
- Игнор, но в git: `git ls-files -ci --exclude-standard`.
- Дубли: одинаковый blob в `git ls-files -s`.
- Прогоны workflow: `gh run list --workflow <file> --limit 3`.

---

## 2. Корневые файлы

| Файл | Назначение | Кто читает (измерено) | Последний | Вердикт |
|---|---|---|---|---|
| `AGENTS.md`, `CONTEXT.md`, `README.md` | точка входа, словарь, витрина | всё | 05.11 | **оставить** |
| `CONTRIBUTING.md`, `.cursorrules` | указатели на `AGENTS.md` (ADR-0163) | GitHub, Cursor | 05.11 | **оставить** |
| `SECURITY.md` | политика уязвимостей (стандарт GitHub) | GitHub | 09.04 | **оставить** |
| `Makefile` | цели `lint-cc`, `test-tts`, `build*`, `ci`, `audit`; все пути из Makefile существуют (проверено) | люди | 19.10 | **оставить** |
| `pytest.ini`, `conftest.py` | корневой pytest-конфиг: `testpaths = src/rob_box_harness/test`, coverage-гейт 85 % по harness, авто-skip `network` | `pytest` из корня | 28.09 / 24.07 | **оставить**; комментарий в `pytest.ini` про «удалённый каркас Harness» сверить (пакет `rob_box_harness` жив: 46 файлов его импортируют) |
| `.gitignore`, `.gitattributes`, `.gitmodules` | git; сабмодули `src/vesc_nexus`, `src/ros2leds` | git, CI | 04.11 | **оставить**, `.gitignore` поправить (§8) |
| `.dockerignore` | контекст сборки из корня | buildx | 09.03 | **оставить** |
| `.env.example` | шаблон `.env` | люди | 20.10 | **оставить** |
| `.hadolint.yaml`, `.yamllint.yml` | конфиги линтеров | `G: Lint Code` (шаги hadolint, yamllint) | 2025-10 | **оставить** |
| `.pre-commit-config.yaml` | хуки detect-secrets, black, isort, flake8, yamllint, shellcheck | никто в CI; хук `detect-secrets` ссылается на `--baseline .secrets.baseline`, а такого файла в репо нет — хук упадёт у первого, кто поставит pre-commit | 2025-10-16 | **вопрос Шифу №3**: починить (baseline) или удалить |
| `.actrc` | конфиг `act` (локальный запуск GitHub Actions) | никто; пример в нём ссылается на `build-vision-services.yml`, которого нет | 2025-10-21 | **удалить** |
| `.agent_flow_worker_started` | пустой маркер воркера hermes | никто (0 ссылок) | ≤28.09 | **удалить** (ADR-0162) |
| `.wip-t_8172bf15` | пустой маркер kanban-карточки hermes `t_8172bf15` | никто | ≤28.09 | **удалить** (ADR-0162) |
| `clog` | 591 КБ: лог старта ROS-контейнера с Zenoh (`RBXU100001`) | никто; **в `.gitignore` есть `/clog`, но файл закоммичен** | 30.01 | **удалить** |
| `coverage.json` | 36 КБ: отчёт coverage.py 7.13.4 от 21.02, пути Windows `src\\rob_box_voice\\…` | никто (тест `test_architecture_class_metrics.py` пишет свой `coverage.json` во `tmp_path`) | 21.02 | **удалить**, добавить в `.gitignore` |

`MAINTENANCE` (гейт hermes) в develop уже нет.

---

## 3. Корневые каталоги

| Каталог | Файлов / размер | Последний | Назначение | Кто читает (измерено) | Вердикт |
|---|---|---|---|---|---|
| `src/` | 1947 / 95 МБ | 05.11 | ROS- и Python-пакеты | Dockerfile, colcon, CI | **оставить**, разбор — §4 |
| `docker/` | 271 / 11 МБ | 05.11 | образы, compose трёх Pi, sim/HIL, build-хост, мониторинг | деплой, сборка | **оставить**, мелочь — §6.4 |
| `docs/` | 617 / 22 МБ | 05.11 | документация | — | ADR-0163 |
| `scripts/` | 319 / 3.5 МБ | 05.11 | скрипты: CI, lint, e2e, деплой, музыка, диалог | workflow, люди | **оставить**, мёртвое — §5 |
| `.github/` | 136 / 2.2 МБ | 05.11 | workflow, e2e-сценарии, шаблоны | GitHub | **оставить**, мелочь — §6.5 |
| `tests/` | 153 / 1.8 МБ | 05.11 | общие тесты вне пакетов | `G: Lint Code` (`pytest tests/unit`), `vision-cold-start-test.yml` | **оставить**, мёртвое — §6.1 |
| `.agents/` | 165 / 1 МБ | 05.11 | скилы агентов (SOT) | `.claude/skills` (junction), агенты | **оставить**, мелочь — §6.6 |
| `evidence/` | 116 / 3.2 МБ | 26.10 | сырые замеры | `gen_night_marathon.py` → `tts-voice-distinctness-*`; docstring `speaker_embeddings.py` → `duration-vs-score-2026-09-22` | **оставить** (ADR-0163); старые разовые — вопрос №7 |
| `architecture/` | 3 / 70 КБ | 04.11 | машинные файлы CI | `G-`/`L-Architecture Audit.yml`, `ownership_check.py` | **оставить** (ADR-0163) |
| `migrations/` | 11 / 50 КБ | 13.10 | SQL-миграции БД голоса, точек, музыки | `docker/sound/docker-compose.yaml:256` монтирует в `/migrations`; `voice_memory.py`, `waypoint_store.py`, `faq_store.py` | **оставить** (робот читает) |
| `sound_pack/` | 57 / 0.8 МБ | 23.10 | звуки-эффекты (mp3) | `docker/sound/docker-compose.yaml:224` монтирует в `/ws/sound_pack`; `sound_catalog.json` читают 11 файлов кода | **оставить** (робот читает) |
| `packaging/` | 1 | 03.11 | `dialog-core/pyproject.toml` — сборка ядра диалога без ROS | `G-Dialog-Core-Package.yml`, `scripts/dialogue/run.ps1` | **оставить** |
| `host/` | 7 | 28.09 | systemd-юниты и CAN-настройка хостов Pi | `scripts/setup/setup_vision_pi*.sh`, `start_ros2_control.sh` | **оставить** |
| `assets/` | 1 (`logo.svg`) | 2025-10 | логотип | `README.md:2` | **оставить** (перенос в `docs/assets/` ничего не даёт, ломает README) |
| `examples/` | 1 (`tts_minimax_example.py`) | 24.07 | пример MiniMax TTS | только 3 дока (`docs/api/MINIMAX_TTS.md`, `docs/guides/MINIMAX_TTS_GETTING_STARTED.md`, `docs/guides/README.md`) | **переместить** в `src/rob_box_llm/examples/` (документация пакета — рядом с пакетом, ADR-0163 §3.1), ссылки в трёх доках поправить |
| `tools/` | 63 / 0.6 МБ | 05.11 | генераторы и стенды | частично CI (§6.2) | **разобрать по файлам** (§6.2) |
| `local_test/` | 61 / 0.3 МБ | 20.10 | dev-утилиты на ноутбуке (Zenoh, RViz, Quest smoke) | частично (§6.3) | **разобрать по файлам** (§6.3), вопрос №4 |
| `tts_audio_bench/` | 45 / 1.1 МБ | 03.11 (попутно) | стенд аудиоконвейера TTS (июль, эпоха test-bench MiniMax) | никто вне `tools/` (а те — тоже никто, §6.2); CI не запускает | **вопрос Шифу №5** |
| `.devcontainer/` | 1 (`Dockerfile`) | 2025-10-11 | DevContainer ROS Humble | **нет `devcontainer.json`** — VS Code этот контейнер не откроет; ссылки только в `docs/development/DEVCONTAINERS_ANALYSIS.md` | **удалить** |
| `.vscode/` | 1 (`settings.json`) | 04.08 | URDF-визуализатор, пути ROS | VS Code; **в `.gitignore` есть `.vscode/`, но файл закоммичен** | **вопрос Шифу №6** |
| `.claude/` | — | — | в git нет (локальные `skills`-junction и `worktrees`, оба в `.gitignore`) | — | ничего не делать |

**Вне git, на машине Шифу** (не репо, а рабочая копия `D:\PROJECTS\rob_box_project`; по `ls`): в корне лежат `.hadolint.yaml;C` (огрызок Windows-пути), `.tmp_3924`, `err.log`, `p1.log`–`p3.log`, `pytest3859.log`, `__pycache__`, 20 каталогов `artifacts_live_check_*`, `live_check_*`, а также уже удалённые в develop `openspec/`, `specs/`, `.planning/`, `.hermes/`, `.kanban/`, `analysis/`, `MAINTENANCE` и старые корневые `*.md` (рабочая копия отстала от develop). В git это не попадает (большая часть в `.gitignore`), но путает при поиске. Чистить руками Шифу или по его «да» (вопрос №8); не glob-ом (`rm -rf C*` однажды стёр `C:/`).

---

## 4. `src/` — пакеты

Счёт: «импортов» — файлы вне пакета с `from|import <pkg>`; «Docker» — Dockerfile/compose/скрипты в `docker/`, где пакет назван; «зависимость» — `package.xml`/`setup.py`/`CMakeLists` других пакетов (измерено).

| Пакет | Файлов | Импортов | Docker | Зависимость | Вердикт |
|---|---|---|---|---|---|
| `rob_box_core` | 61 | 201 | 16 | 14 | **жив** |
| `rob_box_dialog`, `rob_box_dialog_msgs` | 189 + 23 | 166 | 9 | 5 | **жив** |
| `rob_box_music` | 134 | 132 | 4 | 3 | **жив** |
| `rob_box_llm` | 47 | 96 | 2 | 3 | **жив** |
| `rob_box_voice` | 454 | 64 | 20 | 1 | **жив**; внутри 24 файла `docs/` и 6 корневых `*.md` (`HOW_TO_RUN`, `INSTALL`, `PITCH_SHIFT_EXPLANATION`, `QUICKSTART_TTS_TRAINING`, `SECRETS_GUIDE`, `CHANGELOG`) — сверить с кодом отдельно (вне этого ADR) |
| `rob_box_harness` | 99 | 46 | 1 | 2 | **жив** |
| `rob_box_mcp_tools` | 270 | 18 | 11 | 1 | **жив** |
| `rob_box_perception`, `_msgs` | 106 + 6 | 2 | 13 | — | **жив** |
| `rob_box_supervisor`, `_msgs` | 53 + 12 | 3 | 6 | 2 | **жив** |
| `rob_box_quest` | 271 | 2 | 7 | 2 | **жив** |
| `rob_box_telegram`, `rob_box_teleop`, `rob_box_animations`, `rob_box_sim` | 54 / 21 / 29 / 35 | 1 / 1 / 4 / 10 | 5 / 5 / 5 / 12 | — | **живы** (образы и compose) |
| `rob_box_description` | 64 / **67 МБ** | — | 10 | 2 | **жив**, но тяжёлый: §7 |
| `ros2leds`, `vesc_nexus` | сабмодули | — | 12 / 15 | — | **живы** |
| `rob_box_bringup` | 11 | 0 | **0** | 0 | **вопрос Шифу №2**: ни в одном образе; launch-файлы для RViz на ноутбуке, зовёт `scripts/utils/start_rviz.sh` и гайды `docs/guides/VISUALIZATION.md`, `QUICK_START.md`; `rob_box.rviz` берёт из `local_test/`. Последний коммит 24.07. CI его собирает (rsync `src/` без исключения в `G-Run Tests.yml:129`) |
| `robot_sensor_hub_msg` | 6 | 1 (`scripts/testing/test_sensor_hub.py`, ручной, CI его исключает) | **0** | 0 | **вопрос Шифу №2**: сообщения ESP32-хаба; прошивки и ноды-потребителя в репо нет (ADR-0159:29,104 — «`perception_bridge` их не использует»); `G-Run Tests.yml:129,605` исключает пакет из сборки. Нужен, если хаб будут доделывать по ADR-0159 |

Заброшенных пакетов «никто не импортирует и не собирает» нет. Дублей пакетов нет. Два пакета без образа — выше.

**Раскладка тестов.** Тесты пакета живут в `src/<pkg>/test/` (13 пакетов, 915 файлов). Корневой `tests/` — для того, что не принадлежит пакету: CI-workflow, docker-контракты, e2e-скрипты, sim, cold-start (138 файлов в `tests/unit`). Раскладка осмысленная, менять не предлагаю. Исключения — §6.1.

---

## 5. `scripts/` — мёртвое и «декорации»

Из 310 скриптов (без `README.md`/`__init__.py`) у **82 нет ни одной ссылки из кода и CI**, у 64 из них нет ссылок и в `docs/` (измерено, таблица в теле PR). Но «нет ссылки» ≠ «мёртвый»: `pytest scripts/testing/` (`G-Lint Code.yml:504`) собирает каталог целиком, сцены `scene_set/scenes/*.yaml` читает `conductor.py` по маске, фикстуры `music/critic/fixtures/` читает `verify.py`. После ручного разбора:

### 5.1 Мёртвые (одноразовые или наследие) — удалить

| Скрипт | Почему мёртв (по коду) | Последний |
|---|---|---|
| `scripts/run_av21_tests.sh` | патчит venv `/home/builder/.hermes/hermes-agent/…` — рантайм hermes (ADR-0162) | ≤28.09 |
| `scripts/build/patch_workflows_apt_cache.sh` | разовый патч workflow «Дата: 2025-10-25», APT_PROXY уже во всех образах | 17.01 |
| `scripts/build/patch_main_workflows_base_image.sh` | правит `.github/workflows/build-main-services.yml` — такого файла нет | 20.10 (попутно) |
| `scripts/maintenance/rename_meshes.sh` | `cd FROM_FUSION_360/meshes` — каталога нет, меши давно переименованы | 17.01 |
| `scripts/maintenance/rename_fusion360_meshes.sh` | то же, разовое переименование экспорта Fusion 360 | 24.07 |
| `scripts/setup/setup_vision_pi_legacy.sh` | обёртка «для обратной совместимости» над `setup_node.sh vision`; 0 ссылок | 24.07 |

### 5.2 Тесты-декорации: лежат, но ни один workflow их не запускает

Это тот же дефект, что описан в `G-Run Tests.yml:676` и `G-Lint Code.yml:484` («тест написан, но не в CI», ADR-0057). Тесты свежие и дешёвые, поэтому вердикт — **подключить к CI**, а не удалять (вопрос Шифу №1):

- `scripts/ci/tests/`: `test_e2e_voice_check_patterns_step_before.sh`, `test_e2e_voice_patterns_with_spaces.sh`, `test_e2e_voice_robot_ros_quoting.sh`, `test_validate_test_ws_dirs.sh`, `test_workflow_no_scp_dotrex.sh` — перенесены из `scripts/agent_flow/` в #4411 (05.11), шаг в CI не добавлен; а также `test_push_image_versions.sh` (28.08), `test_check_image_versions_usage.sh`.
- `scripts/testing/*.sh` вне явного списка `GUARDS` (`G-Run Tests.yml:710–718`): `test_e2e_act_warmup.sh` (#3886), `test_e2e_when_robot_asked.sh` (#2809), `test_yaml_loader_env_override.sh`, `test_submodule_cache_invalidation.sh`.
- `scripts/e2e/tests/test_marathon_markers.sh` (03.11, #3934).
- `scripts/music/`: `test_loudness_nrt_v2.py`, `test_pdmx_batch.py`, `test_score_corpus_stats.py`, `test_score_import.py`, `critic/test_judge.py`, `critic/test_verify.py`, `live_dj/test_v2log.py`.
- `scripts/dialog/rap_sync/test_rap_sync.py`.
- `tools/music_bench/tests/` (3), `tools/audio_capture_harness/test_audio_capture_harness.py`, `tts_audio_bench/scripts/test_*.py` (4) — см. §6.2 и вопрос №5.
- `tests/dialog_adapters/` **запускается** (`G-Lint Code.yml:201–202`) — не декорация.

**Прежде чем подключать** — прогнать каждый локально: что-то могло покраснеть, пока его никто не звал (так было с `test_nav2_current_depth_obstacles.py`, `G-Lint Code.yml:486`). Красные — чинить или удалять по одному, с raw-выводом.

### 5.3 Живые по назначению, но без ссылок — оставить

Ручные утилиты, которые человек зовёт из терминала: `scripts/md_to_pdf.py`, `scripts/av27_voice_picker_smoke.py` (WS-смоук Quest), `scripts/testing/check_aec.py`, `check_respeaker_devices.py`, `scripts/maintenance/cleanup_dead_voice_dbs.sh`, `scripts/setup/setup_sound_pi.sh`, `scripts/station/inventory.sh`, `scripts/robot_dev/tgsend.sh`, `scripts/diagnostics/watchdog-tts.sh`, `tune.sh`, `voice_e2e_dialog.sh`, `scripts/utils/fix_renardo_synthdef_conflicts.py` (Renardo жив: `docker/sound/supercollider`), `scripts/music/research/theme_gate_regression.py`, `scripts/migrations/migrate_voice_memory_unify.py` (разовая миграция 010 — **догадка**: на роботе уже применена; удалить после проверки БД, вопрос №7).

### 5.4 Структурные огрызки `scripts/`

- **Три проверялки ссылок**: `scripts/diagnostics/check_links.py`, `check_broken_links.sh`, `fix_broken_links.py` (0 ссылок из кода). ADR-0163 §5 требует link-check в каждом PR переноса — оставить одну (`check_links.py`), две удалить.
- **`scripts/dialog/` и `scripts/dialogue/`** — два каталога про одно (диалог v2 без робота). `dialogue/` — 4 файла консоли поверх `dialog/dry_stand.py`. Слить в `scripts/dialog/console/` (правка `G-Dialog-Core-Package.yml`, `packaging/`, README).
- **`scripts/tests/`** (2 yaml-фикстуры cold-start) рядом с `scripts/testing/` — перенести в `tests/vision/cold_start/fixtures/` (правка `vision-cold-start-test.yml:98` и `paths:`).
- **Корень `scripts/`**: `av27_voice_picker_smoke.py`, `md_to_pdf.py`, `vision_pi_healthcheck.sh` — разложить по подкаталогам (`testing/`, `docs/`, `monitoring/`).
- `scripts/README.md` описывает `patch_*_workflows.sh` как живые — поправить вместе с §5.1.

---

## 6. Остальные каталоги по файлам

### 6.1 `tests/`

| Путь | Что | Вердикт |
|---|---|---|
| `tests/integration/` (`__init__.py` + `_artifacts/` из 3 файлов) | сам тест `test_e2e_harness_minimax.py` удалён (≤28.09); остались артефакты прогона hermes: `pytest_run.txt` с путём `/home/builder/hermes-share/…/.worktrees/t_dcdaa9b0` | **удалить** |
| `tests/test_github_summary_comment.py` | импортирует `pr907.github_summary_comment` — **модуля нет** (был в корневом `analysis/`, удалён по ADR-0163); не в `tests/unit`, CI не собирает | **удалить** |
| `tests/e2e/live/` (`run_live_e2e.sh`, `snapshot.sh`) | дизайн `docs/design/E2E_TESTING_DESIGN*.md` (август); `run_live_e2e.sh` зовёт `aggregate_verdicts.py`, которого нет; e2e сейчас — `L-E2E Voice Test` и марафон | **вопрос №7** (рекомендую удалить) |
| `tests/unit/`, `tests/vision/`, `tests/dialog_adapters/` | гоняет CI | **оставить** |

### 6.2 `tools/`

| Путь | Кто читает (измерено) | Вердикт |
|---|---|---|
| `architecture_*.py` (9) | `G-`/`L-Architecture Audit.yml`, `tests/unit/scripts/` | **оставить** |
| `gen_tool_catalog.py`, `gen_bridge_protocol_ts.py` | тесты, `G-Bridge-Protocol-Drift.yml` | **оставить** |
| `led_face_lab/` | превью лица, `face_node.yaml:6`, `ANIMATIONS.md` | **оставить** |
| `animation_editor/` (15, 2025-10) | `G-Lint Code.yml` линтит, `dependabot.yml` обновляет; редактор анимаций LED на десктопе | **оставить** (вопрос, нужен ли он ещё, — не задаю: живой dependabot-шум мал) |
| `gui/` (3, 20.10) | `README.md`, `scripts/utils/start_robot_gui.sh` | **оставить** |
| `music_bench/` (8) | 0 ссылок из кода; тесты не в CI | **вопрос №5** |
| `diagnostics/` (5, январь) | 0 ссылок из кода (одометрия) | **переместить** в `scripts/diagnostics/` (там уже диагностика) |
| `mock_minimax_server.py`, `README_mock_minimax.md`, `ffmpeg_decode/` (7), `audio_capture_harness/` (6) | только друг друга и `tts_audio_bench/`; CI нет; июль (эпоха test-bench MiniMax) | **вопрос №5** — вместе с `tts_audio_bench/` |

### 6.3 `local_test/` (dev-утилиты на ноутбуке)

| Группа | Кто читает | Вердикт |
|---|---|---|
| `RENARDO_REFERENCE.md` | **робот**: `docker/sound/docker-compose.yaml:254` монтирует в контейнер | **оставить на месте** (перенос — только вместе с compose и деплоем) |
| `test_quest_caddyfile.py` | `G-Quest-Caddyfile-Contract.yml:81` | **оставить** (перенос — вместе с workflow) |
| `quest_smoke.sh`, `quest_smoke_lib.py`, `test_quest_smoke_lib.py`, `_dev_fake_quest.py` | `Caddyfile:47`, evidence | **оставить** |
| `zenoh_*.json5` (11), `rob_box*.rviz` (2), `start_zenoh.sh` | скилы `zenoh-dev-setup`, `motor-testing`, `scripts/utils/*`, launch `rob_box_bringup` | **оставить**; `zenoh_local_session_with_namespace.json5.backup`, `zenoh_template_session.json5`, `zenoh_local_session_minimal.json5`, `zenoh_local_config.json5` (0 ссылок из кода) — **удалить** |
| `voice/` (17: пробы Qwen/DeepSeek/Vosk/sound_node, январь–июль) | 0 ссылок вне каталога | **вопрос №4** (рекомендую удалить) |
| `hardware/` (7: пробы ReSpeaker/USB VAD/LED, январь–апрель) | 1 — комментарий в `respeaker_interface.py` | **вопрос №4** |
| `calibrate_odometry.py`, `depth_viewer_rt.py`, `send_waypoint_forward.py`, `move_test.py`, `visualize_apriltag*.{py,sh}` (3), `monitor_apriltag.sh`, `view_camera.sh`, `launch_rviz*.sh`, `auto_test.sh`, `quick_test.sh`, `fake_lidar_publisher.py`, `test_rtabmap_2d_lidar.launch.py`, `test_start_zenoh_lifecycle.sh` | ручные, 0–1 ссылок; `move_test.py` — скил `motor-testing` | **вопрос №4** |

Имя `local_test/` вводит в заблуждение (это не тесты, а dev-окружение ноутбука). Целевое место — `tools/dev/` (вопрос №4); `RENARDO_REFERENCE.md` при этом лучше увезти в `docker/sound/config/` (одна правка compose).

### 6.4 `docker/`

- **`docker/scripts/start_{lslidar,oak_d,apriltag,robot_state_publisher}.sh`** — устаревшие копии. Compose монтирует `docker/main/scripts/<svc>/` и `docker/vision/scripts/<svc>/`, а не `docker/scripts/` (по коду: `docker/main/docker-compose.yaml:93,303`, `docker/vision/docker-compose.yaml:78`). Содержимое отличается (md5 разные, измерено). **Удалить** 4 файла.
- `docker/scripts/{check_power_status,check_usb_devices,monitor_power_live,power_help,setup_can0}.sh` — ручные утилиты питания/CAN (24.07). **Переместить** в `scripts/diagnostics/power/`, `power_help.sh` поправить пути (в нём `ssh … /home/ros2/rob_box_project/docker/scripts/…`).
- Три разных `update_and_restart.sh` (`docker/main/`, `docker/vision/`, `docker/build/scripts/`) — не дубли (md5 разные), у каждого свой хост. Оставить.
- `docker/build/` — инфраструктура build-хоста (apt-cacher-ng, registry, раннеры; `test_cleanup_registry_floating_tags.py` читает скрипт). 5 `*.md` внутри (`APT_CACHER_NG_*`, `DEPLOYMENT_SUMMARY`, `QUICKSTART`) — кандидаты в `docs/` по ADR-0163, не здесь.
- `docker/monitoring/` — Grafana/Loki/Prometheus на сервере; агенты (`--profile monitoring`) поднимает `L-Deploy and Verify.yml:570`. Жив. 5 `*.md` — как выше.
- `docker/base/vendor/*.deb` (9.7 МБ, rmw_zenoh 0.1.8) — **жив**: `Dockerfile.depthai:49` `COPY vendor/.`; пин версии, которой нет в ROS-репо. Оставить.
- `docker/{main,sound,vision}/.env` закоммичены при `.env` в `.gitignore`. Внутри — `ROS_DISTRO`, `IMAGE_TAG`, `REGISTRY`, `ROBOT_ID`, пороги лица; **секретов нет** (измерено, значения не печатал). Деплой их читает (compose). Оставить, в `.gitignore` добавить явное исключение (§8).

### 6.5 `.github/`

| Путь | Факт | Вердикт |
|---|---|---|
| `workflows/build-all-local.yml` | `name: "[DEPRECATED] build-all-local"`, только `workflow_dispatch` с галкой «я понимаю»; последние прогоны `18817359175`, `18817337587` — 26.10.**2025**, оба `failure` | **удалить** |
| `workflows/vision-cold-start-test.yml` | единственный workflow без префикса `G-/L-/B-` | **переименовать** в `G-Vision-Cold-Start-Tests.yml` (вместе с самоссылкой в `paths:`) |
| `workflows/L-Diagnose Runner Network.yml` | ручной; последний прогон `36551967874` 29.09 | **оставить** (диагностика раннера) |
| `PULL_REQUEST_TEMPLATE_PERCEPTION_REVIEW.md` | 2025-10-21; ссылки только из `docs/development/*` 2025 года | **удалить** |
| `copilot-instructions.md` | указатель на `AGENTS.md` | ADR-0163 |
| `.github/scripts/` (13), `.github/workflows/scripts/` (8), `scripts/ci/` (26) | три места для CI-скриптов | **вопрос №9** (сводить ли в одно место) |

### 6.6 `.agents/`

- `.agents/issues/` (3 черновика issue Quest phase-1.5…1.7) — не скилы; ссылки из `docs/plans/2026-09-05-operator-agent-architecture-handoff.md` и ADR-0051 (история). **Удалить** после сверки, что issue заведены (догадка: заведены — фазы Quest давно позади).
- `.agents/github-copilot/` (2 скила) — проверить, читает ли их кто-то кроме Copilot; оставить.
- `.agents/skills/` — 56 скилов, часть сторонние наборы (`hz-*` Meta Quest — 9, «superpowers»-набор — около 12). Ревизия скилов — отдельная тема, не структура; не трогаю.

---

## 7. Крупные бинарники и дубли в git

HEAD: 144 МБ, история `size-pack` 472 МБ (измерено). Файлов ≥1 МБ — 17, вместе 82 МБ.

| Что | Размер | Кто читает | Вердикт |
|---|---|---|---|
| `src/rob_box_description/models/cpr_office/` (текстуры 3K, `office.dae`) | **61.3 МБ**, 20 файлов | только `worlds/office_cpr.world`; ни один launch и ни один compose этот мир не грузит (поиск `office_cpr` — только docs/research) | **вопрос №10**. `model.config` называет «office from clearpath robotics», лицензии нет (`docs/research/2026-11-04-gazebo-sim-navigation.md:42`). План сима (там же, строка 174, шаг 3) берёт `office_cpr` только «при совместимой лицензии» |
| `src/rob_box_description/URDF_EXPORT/` | 3.0 МБ, 21 файл; 6 STL — побайтовые копии `meshes/` (измерено, одинаковый blob) | никто: `COLCON_IGNORE`, ROS1-экспорт; research 04.11 называет «мусор, не использовать» | **удалить** |
| `src/rob_box_description/ENCODER.stl` (корень пакета) | 48 КБ, 30.01 | никто: имя `ENCODER.stl` нигде не встречается, в `urdf/` слова «encoder» нет (измерено) | **удалить** в том же PR |
| `docs/assets/rob_box_rviz_demo.gif` | 8.1 МБ | `docs/guides/VISUALIZATION.md` | **оставить** (ADR-0163); можно пережать в MP4/WebP — по желанию |
| `docker/base/vendor/*.deb` | 9.7 МБ | `Dockerfile.depthai` | **оставить** (§6.4) |
| `tts_audio_bench/fixtures/` | 1 МБ, 30 файлов; `reference/*.wav` = `wav/*.wav` побайтно (6 пар) | §6.2 | вместе с вопросом №5 |
| `evidence/role-voices-live-2026-10-26/` | 2 МБ, 70 файлов; `playlog_2026-10-23.jsonl` = `evidence/role-voices-live-2026-10-23/playlog.jsonl` | — | вопрос №7 |

Удаление из HEAD **не уменьшит** клон: блобы остаются в истории. Ужать историю можно только переписыванием (`git filter-repo`) с force-push в `develop` и `main` — это ломает все ветки и worktree воркеров. Не предлагаю (вопрос №10, вариант «Б» — только с явного «да» Шифу).

---

## 8. `.gitignore`

Закоммичено при правиле игнора (`git ls-files -ci --exclude-standard`, измерено):

| Файл | Правило | Что делать |
|---|---|---|
| `clog` | `/clog` | `git rm` (§2) |
| `docker/{main,sound,vision}/.env` | `.env` | оставить; добавить `!docker/*/.env` с комментарием «конфиг деплоя, без секретов; секреты — в `.env.secrets`» |
| `.vscode/settings.json` | `.vscode/` | по вопросу №6: `!.vscode/settings.json` или `git rm --cached` |
| `docker/vision/vision-hailo/wheels/.gitignore`, `.gitkeep` | `wheels/` | оставить; добавить `!docker/vision/vision-hailo/wheels/.gitkeep` |

Добавить в игнор (правила нет, артефакты уже появлялись):
- `coverage.json` — сейчас закоммичен (§2).
- `/.tmp_*/`, `/*;C` — огрызки Windows (в рабочей копии Шифу: `.tmp_3924`, `.hadolint.yaml;C`).

Уже покрыто, добавлять не нужно (по чтению правил): корневые логи `err.log`, `p1.log`, `pytest3859.log` — правилом `/*.log`; `artifacts_live_check_*`, `live_check_*` — `/artifacts_live*`, `/live_check*`.
Каталогов вида `C:`/`C;` от pytest на Windows в git нет (измерено: в `git ls-files` таких путей нет).

---

## 9. Целевая структура корня

```
AGENTS.md  CONTEXT.md  README.md  CONTRIBUTING.md  SECURITY.md   — вход, словарь, витрина
Makefile  pytest.ini  conftest.py                                — сборка и тесты
.gitignore .gitattributes .gitmodules .dockerignore .env.example — git и docker
.hadolint.yaml .yamllint.yml (.pre-commit-config.yaml — по вопросу №3)
.cursorrules                                                     — указатель на AGENTS.md
.github/      — CI, e2e-сценарии, шаблоны
.agents/      — скилы агентов
src/          — пакеты (тесты пакета — src/<pkg>/test/)
docker/       — образы и compose (робот, build-хост, sim, мониторинг)
scripts/      — скрипты по назначению: ci, lint, dev, deploy, e2e, testing, diagnostics, setup, maintenance, music, dialog, …
tests/        — тесты вне пакетов (CI, docker-контракты, e2e-скрипты, sim, cold-start)
tools/        — генераторы, аудит архитектуры, dev-приложения (tools/dev/ — бывший local_test/, вопрос №4)
docs/         — документация (ADR-0163)
evidence/     — сырые замеры
architecture/ — машинные файлы CI
migrations/   — SQL (монтируется на роботе)
sound_pack/   — звуки (монтируется на роботе)
host/         — systemd и настройка хостов Pi
packaging/    — сборка dialog-core
assets/       — логотип README
```

Уходят из корня: `clog`, `coverage.json`, `.actrc`, `.agent_flow_worker_started`, `.wip-t_8172bf15`, `.devcontainer/`, `examples/`; по вопросам — `local_test/`, `tts_audio_bench/`, `.vscode/`.

---

## 10. План маленькими PR (ADR-AF-0013)

Правила для каждого PR:
- до правки — `git grep -n '<путь>'` по всему репо, raw-вывод в тело PR; ссылки правятся в том же PR;
- удалять `git rm` по точным путям, без glob; переносить `git mv`;
- пути, которые читают робот и деплой (`migrations/`, `sound_pack/`, `local_test/RENARDO_REFERENCE.md`, `docker/*/scripts/`, `docker/*/.env`, compose, `scripts/hil/`, `.github/e2e/`, `evidence/tts-voice-distinctness-*`), **не двигать** без правки compose/скрипта в том же PR и без проверки деплоем;
- после — гарды `scripts/lint` и зелёный CI (run_id в теле PR).

| Шаг | PR | Объём | Зависит от |
|---|---|---|---|
| 1 | **Этот PR**: ADR-0166 (Proposed) | 1 файл | — |
| 2 | **Корневой мусор**: `git rm` `clog`, `coverage.json`, `.actrc`, `.agent_flow_worker_started`, `.wip-t_8172bf15`, `.devcontainer/Dockerfile`; `.gitignore` — `coverage.json`, исключения `!docker/*/.env`, `!…/wheels/.gitkeep` | 6 файлов + `.gitignore` | 1. Риска нет: никто не читает |
| 3 | **Мёртвые тесты и скрипты**: `tests/integration/`, `tests/test_github_summary_comment.py`, §5.1 (6 скриптов), `docker/scripts/start_*.sh` (4), `build-all-local.yml`, `PULL_REQUEST_TEMPLATE_PERCEPTION_REVIEW.md`, две из трёх проверялок ссылок; `scripts/README.md` | ~20 файлов | 1 |
| 4 | **Тесты-декорации → CI** (§5.2): сначала локальный прогон каждого с raw; зелёные — шаг в `G-Lint Code`/`G-Run Tests`; красные — отдельные issue | 1–2 PR | ответ №1 |
| 5 | `src/rob_box_description/URDF_EXPORT/` и `ENCODER.stl` → `git rm` | 22 файла, −3 МБ в HEAD | 1 |
| 6 | Мелкие переносы: `examples/` → `src/rob_box_llm/examples/`; `tools/diagnostics/` → `scripts/diagnostics/`; `docker/scripts/` (питание, CAN) → `scripts/diagnostics/power/`; `scripts/tests/` → `tests/vision/cold_start/fixtures/`; файлы из корня `scripts/` по подкаталогам; `vision-cold-start-test.yml` → `G-Vision-Cold-Start-Tests.yml` | по PR на пункт | 1 |
| 7 | `scripts/dialogue/` → `scripts/dialog/console/` | 4 файла + workflow + packaging | 1 |
| 8 | `local_test/` → `tools/dev/` + удалить пробы `voice/`, `hardware/` и т. п.; `RENARDO_REFERENCE.md` → `docker/sound/config/` с правкой compose и деплоем | 2 PR | ответ №4 |
| 9 | `tts_audio_bench/` + `tools/{ffmpeg_decode,audio_capture_harness,mock_minimax*,music_bench}` | 1 PR | ответ №5 |
| 10 | `rob_box_bringup`, `robot_sensor_hub_msg`, `cpr_office`, `.pre-commit-config.yaml`, `.vscode/`, старые `evidence/`, `tests/e2e/live/` | по ответам | ответы №2, 3, 6, 7, 10 |

Первым — шаг 2: только мусор, ноль потребителей, проверка — CI зелёный.

Критерий готовности: в корне только то, что в §9; `git ls-files -ci --exclude-standard` пуст или каждое исключение явно описано в `.gitignore`; каждый `test_*` в репо либо запускается CI, либо явно помечен «ручной» в его docstring.

---

## 11. Что решать Шифу (простым языком)

1. **Тесты, которые никто не запускает (~25 штук).** Они лежат в репо, но CI их не зовёт, поэтому ничего не ловят.
   - **А (рекомендую):** прогнать каждый, зелёные подключить к CI, красные — починить или удалить по одному.
   - Б: удалить все разом.
   - В: оставить как есть.
2. **Два пакета без образа: `rob_box_bringup` (запуск RViz на ноутбуке) и `robot_sensor_hub_msg` (сообщения для платы с датчиками ESP32).** На робот не попадают.
   - **А (рекомендую):** `rob_box_bringup` оставить, если RViz с ноутбука ещё нужен; `robot_sensor_hub_msg` оставить до решения по плате (ADR-0159).
   - Б: удалить оба (вернуть из истории, если понадобятся).
3. **`.pre-commit-config.yaml`** — настройки проверок перед коммитом; сейчас сломаны (нет файла `.secrets.baseline`), и CI их не использует.
   - **А (рекомендую):** удалить; проверки и так делает CI.
   - Б: починить (создать baseline) и пользоваться локально.
4. **`local_test/` — инструменты для ноутбука (Zenoh, RViz, пробы микрофона и LLM).** Половина — январские пробы, которыми никто не пользуется.
   - **А (рекомендую):** живое перенести в `tools/dev/`, пробы (`voice/`, `hardware/`, старые скрипты) удалить.
   - Б: только удалить пробы, папку не переименовывать.
   - В: не трогать.
5. **Стенд звука TTS (`tts_audio_bench/` и 4 инструмента в `tools/`)** — июльский стенд проверки MiniMax. CI его не запускает, но каждый рефакторинг голоса приходится его править (03.11 — PR #4281).
   - **А (рекомендую):** удалить.
   - Б: подключить его тесты к CI и оставить.
6. **`.vscode/settings.json`** — настройки редактора VS Code; лежит в репо, хотя по правилам должен игнорироваться.
   - **А (рекомендую):** оставить в репо как общие настройки и разрешить явно в `.gitignore`.
   - Б: убрать из репо (у каждого свои настройки).
7. **Старые разовые материалы**: `evidence/` до 01.10 (кроме тех двух, что читает код), `tests/e2e/live/` (августовский живой e2e), разовая миграция `migrate_voice_memory_unify.py`.
   - **А (рекомендую):** удалить то, что не читает код; миграцию — после проверки, что на роботе она применена.
   - Б: оставить всё как историю.
8. **Мусор в вашей рабочей папке `D:\PROJECTS\rob_box_project`** (не в git): логи, 20 папок `artifacts_live_check_*`, файл `.hadolint.yaml;C`, удалённые в develop папки `openspec/`, `.hermes/` и т. п. Рабочая копия отстала от develop.
   - **А (рекомендую):** вы обновляете её сами (`git pull`) и удаляете мусор по списку из этого ADR.
   - Б: поручить агенту — список по именам, удаление по одному файлу, после вашего «да».
9. **CI-скрипты в трёх местах** (`.github/scripts/`, `.github/workflows/scripts/`, `scripts/ci/`).
   - **А (рекомендую):** пока не трогать — переносы ломают пути в workflow, выгода мала.
   - Б: свести всё в `scripts/ci/` отдельным эпиком.
10. **61 МБ текстур офиса Clearpath** (`cpr_office`) — мир для симулятора; сейчас его ничего не запускает, лицензия не указана.
    - **А (рекомендую):** удалить из репо сейчас; если сим-эпик его возьмёт — скачивать скриптом из источника с известной лицензией.
    - Б: оставить до сим-эпика.
    - В (только явным «да»): дополнительно вычистить из истории git, чтобы клон стал легче. Ломает все ветки и рабочие копии агентов — не рекомендую.
    - **Решение Шифу 05.11 (чат приложения, через координатора):** «61 МБ текстур cpr_office в репозитории не нужны» — вариант А: удалить из HEAD (`models/cpr_office/` и мир `worlds/office_cpr.world`, который грузит только её; PR #4445). Историю git не переписывать (вариант В не выбран).
11. **Принять ADR** (Proposed → Accepted) — тогда идём по шагам §10, начиная с шага 2.

---

## 12. Последствия

- Плюс: в корне около 32 пунктов вместо 42 (22 файла и 20 каталогов в git сейчас), и у каждого понятная роль; поиск по репо не цепляет мёртвые скрипты и копии; тесты в репо = тесты в CI.
- Минус: десяток маленьких PR; риск задеть путь, который читает робот. Защита — правила §10 (grep до, деплой после для путей робота).
- Не меняется: код пакетов, рантайм робота (кроме шага 8 — одна правка compose с деплоем).
- **Чего не проверял** (честно): цели `Makefile` поштучно (только существование путей); живость каждого из 56 скилов `.agents/skills`; содержимое `src/rob_box_voice/docs/` против кода; прогоны тестов-декораций (§5.2) — ни один не запускал, их состояние неизвестно; рабочую копию на роботе.
