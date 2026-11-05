# ADR-0163: Порядок в документации; spec-driven попытка (OpenSpec, GSD) — deprecated

| Поле | Значение |
|---|---|
| Статус | Proposed (только анализ и пометки; переносов и удалений в этом PR нет) |
| Дата | 2026-11-05 |
| Автор | Claude Code (шиди) по задаче товарища Шифу 05.11: «навести порядок в документации; была попытка перейти на spec-driven, что-то пошло не так, надо вычистить» |
| Домен | RT-нумерация (документация репо целиком). Номер 0162 оставлен параллельному ADR «текущий процесс + hermes/agent-flow deprecated» |
| Затрагивает | `openspec/`, `specs/`, `.planning/`, `docs/research/openspec-pilot/`, `scripts/import_adr_to_openspec.py`, `docs/` (карта), корневые `*.md` |
| Не затрагивает | `.hermes/`, `scripts/agent_flow/` (вместе с `agent-flow-openspec-sync.sh`), их workflow, `.kanban/`, раздел «Процесс-команды» в `AGENTS.md` — это параллельный ADR-0162 |
| Родители | ADR-AF-0013 (маленькие PR), ADR-0018 (честность), ADR-0038 и ADR-0039 (OpenSpec — их этот ADR предлагает заменить) |

> **Коротко.** В репо было две попытки работать «от спецификаций»: GSD (`.planning/`, 15.05–28.07) и OpenSpec
> (`openspec/`, `specs/`, `docs/research/openspec-pilot/`, 31.08–03.09). Обе заглохли. Код их не читает,
> CI их не проверяет. Живое внутри одно: `specs/avatar-floor-protocol` описывает реальный контракт супервизора.
> Его переносим в `docs/architecture/`, остальное — в архив, потом удаляем. Документация в `docs/` разрослась
> до 32 каталогов; предлагается 8 основных и `docs/archive/`, одна карта (`docs/README.md`) и правило,
> куда агент кладёт новый документ.

Метки достоверности ниже: **измерено** — есть вывод команды (в теле PR), **по коду** — прочитано в файлах, **догадка** — вывод без проверки.

---

## 1. Spec-driven попытка: что было и что осталось

### 1.1 GSD (`.planning/`) — заглохло 28.07

- **Что это** (по коду). GSD — шаблон «проект → milestone → фазы → PLAN/RESEARCH/SUMMARY/VALIDATION». `.planning/STATE.md`: «Milestone 1 — Качество кодовой базы», фаза 6 из 6, `status: in-progress`, `last_updated: 2026-07-28`. 86 файлов.
- **Когда** (измерено, `git log -- .planning`). Первый коммит 15.05 (`e4e8ca24f docs: map existing codebase`, `09db0310a chore: initialize GSD project — Milestone 1`). Последний содержательный — 28.07 (`b97c9b4c4 docs(06): add REVIEW-v2.md`). Дальше только попутные правки: 31.07, 15.08 (TECH_DEBT.md), 20.10 (замена адреса в двух файлах в PR #3667).
- **Кто пользуется сейчас.**
  - Код и CI: никто. Две ссылки — в docstring тестов `src/rob_box_perception/test/test_perception_bridge.py:9` и `src/rob_box_telegram/test/test_telegram_bridge.py:15` (по коду; это комментарий, не путь, который открывается).
  - Метка `source:gsd` жива как привычка, а не как процесс (измерено): 206 issue, первое 15.05, последнее 24.10; открыто 3 (#4009, #4014, #4015). Её велит ставить `.github/copilot-instructions.md:17,166`. Её фильтрует триаж hermes (ADR-0031) — это часть ADR-0162.
- **Вывод.** Deprecated. Каталог в архив. Метку `source:gsd` новым issue не ставить (решать Шифу, §6).

### 1.2 OpenSpec (`openspec/`, `specs/`, пилот) — заглохло 03.09

- **Что это** (по коду). ADR-0038 (31.08, Accepted): новые изменения писать в `openspec/changes/<name>/`, а 45 старых ADR импортировать скриптом. ADR-0039: hermes-триаж сам создаёт change-папку, merge-gate её архивирует.
- **Когда** (измерено). 31.08 — #1798 (исследование) и #1814 (ADR-0038). 01.09 — #1818 (импорт) и #1829 (хуки синхронизации). 03.09 — последние два change: `avatar-floor-protocol` (#1927, AV-12) и `quest-voice-pipeline-panel` (7830f84da). Больше в `openspec/` и `specs/` не коммитили.
- **Почему заглохло** (по коду: `docs/research/why-agents-dont-use-openspec.md`, 01.09). Хук создания change-папки ни разу не сработал: cron триажа работал в устаревшем клоне без скрипта синхронизации, а хук был warn-only. Из 6 PR после #1829 ни в одном нет change-папки. Подтверждение (измерено): `openspec/changes/archive/` в git нет. Значит, архивирующий хук merge-gate тоже ни разу не дошёл до конца.
- **Где лежит.**
  - `openspec/changes/` — 6 файлов, 2 change, оба сделаны руками.
  - `specs/avatar-floor-protocol/spec.md` — 1 файл.
  - `docs/research/openspec-pilot/` — **116 файлов**: 49 импортированных ADR-папок, `config.yaml` и 6 скилов `.hermes`. Это 76 % из 152 файлов `docs/research` (измерено). На деле исследований в `docs/research` около 36.
  - `scripts/import_adr_to_openspec.py`.
  - `.agents/skills/openspec-workflow/`.
  - `docs/process/agent-flow-openspec-integration.md`.
  - Хук в `scripts/agent_flow/agent-flow-merge-gate.sh:2517–2560` и `agent-flow-openspec-sync.sh` — **ADR-0162**.
- **Кто пользуется сейчас** (измерено, `git grep`).
  - `.github/workflows`, `scripts/lint`, `tests/`: ноль ссылок на `openspec`/`import_adr`.
  - `src/`: одна ссылка — в docstring `src/rob_box_core/test/test_avatar_command_unity.py:4`.
  - `AGENTS.md`, `CONTRIBUTING.md`, `.claude/`: нет.
  - Скрипты: только merge-gate hermes, warn-only.
- **Что живое.** `specs/avatar-floor-protocol/spec.md` описывает **работающий** контракт. Сервисы `acquire_floor`/`release_floor`/`set_avatar_mode` на типизированных `src/rob_box_supervisor_msgs/srv/{AcquireFloor,ReleaseFloor,SetAvatarMode}.srv` существуют (по коду). Ни код, ни тесты этот spec не читают (измерено: `avatar-floor-protocol` упоминают только 3 плана в `docs/plans/` от 02–05.09). `quest-voice-pipeline-panel` — описание фичи Quest. Жива ли панель сейчас — не проверял (**догадка**: жива, Quest переехал на Sound Pi в #3667).
- **Вывод.** OpenSpec deprecated. Предлагаю ADR-0038 и ADR-0039 перевести в `Superseded by ADR-0163` после принятия этого ADR (отдельным PR, §5 шаг 4). Контракт floor-протокола — в `docs/architecture/avatar-floor-protocol.md`. Остальное — в архив, потом удалить.

### 1.3 Соседи: корневые каталоги, которые выглядят как мусор

| Каталог | Файлов | Последний коммит | Кто читает (измерено) | Вердикт |
|---|---|---|---|---|
| `architecture/` (корень) | 3 | 04.11 | CI: `G-Architecture Audit.yml` (`ownership.yml`), `L-Architecture Audit.yml` (`runtime-baseline.json`), `scripts/lint/ownership_check.py`; `src/rob_box_llm` → `minimax-provider.md` | **живой**, не трогать. Имя путают с `docs/architecture/`; переименование — только вместе с CI, и не в этом эпике |
| `evidence/` | 116 | 26.10 | `scripts/e2e/gen_night_marathon.py` читает `evidence/tts-voice-distinctness-*/`; ссылки из `G-Quest-Caddyfile-Contract.yml`, `e2e_voice_lib.sh` | **живой** (замеры и сырые выводы) |
| `local_test/` | 61 | 20.10 | `G-Quest-Caddyfile-Contract.yml` гоняет `local_test/test_quest_caddyfile.py`; `docker/sound/docker-compose.yaml:254` монтирует `RENARDO_REFERENCE.md` | **частично живой**. Остальное разобрать отдельно (не в этом ADR) |
| `.agents/` | 171 | 19.10 | `.gitignore:1-2`: «единственный SOT скиллов — `.agents/skills/`»; `.github/copilot-instructions.md` | **живой**. Убрать только `skills/openspec-workflow` |
| `analysis/` (корень) | 12 | 14.09 | никто (10 из 12 — ревью PR #907 от июля) | **архив** |
| `.kanban/` | 1 | 25.08 | `scripts/agent_flow` | ADR-0162 |

---

## 2. Карта `docs/` (на develop `7dc4a962b`, 05.11)

Столбцы: «код» — сколько файлов в `src/ scripts/ .github/ tests/ docker/ .claude/` ссылаются на `docs/<dir>/`; «вход» — то же для `AGENTS.md`/`CONTRIBUTING.md`/`README.md`/`docs/README.md`; «доки» — сколько других доков ссылаются на каталог. «Свежих» — сколько файлов меняли с 06.10. Измерено скриптом из тела PR. Дата последнего коммита иногда обманывает: массовые PR вроде #3667 (20.10) задели много файлов попутно.

| Каталог | Файлов | Последний | Свежих | Код | Вход | Доки | Назначение | Вердикт |
|---|---|---|---|---|---|---|---|---|
| adr | 199 | 04.11 | 21 | 73 | 3 | 128 | решения (RT `NNNN`, AF `AF-NNNN`) | **оставить** |
| architecture | 31 | 04.11 | 9 | 60 | 2 | 94 | как устроено сейчас; `meta-quest-api.md` читает тест | **оставить** (источник фактов) |
| research | 152 | 04.11 | 14 | 50 | 0 | 18 | исследования; 116 из 152 — пилот OpenSpec; `jev/scenarios/acceptance.yaml` читают тест и скрипт | **оставить**, пилот → архив |
| plans | 65 | 04.11 | 5 | 65 | 0 | 25 | планы эпиков; `2026-09-15-service-manifest.md` — опора CI-гарда | **оставить** активные, закрытые → архив |
| guides | 37 | 04.11 | 8 | 19 | 1 | 19 | руководства | **оставить** |
| e2e | 6 | 04.11 | 1 | 9 | 0 | 4 | марафон, сценарии | **оставить** (кандидат в runbooks) |
| process | 8 | 03.11 | 2 | 4 | 0 | 11 | как работаем | ADR-0162 |
| dialog | 13 | 28.10 | 13 | 11 | 0 | 4 | диалог v2, замеры | **оставить** |
| assets | 33 | 21.10 | 37 | 2 | 0 | 2 | картинки | **оставить** |
| design | 21 | 20.10 | 2 | 32 | 2 | 32 | дизайн-доки; `ARCHITECTURE_AUDIT.md` — путь в CI | **оставить** |
| development | 64 | 20.10 | 6 | 36 | 1 | 24 | разработка; 85 из 97 файлов в истории старше 09.2026; есть `archive/` (15) и `agents/` (13 промтов «11 AI-агентов», январь) | **разобрать**: живое → guides, остальное → архив |
| reports | 65 | 20.10 | 1 | 10 | 0 | 16 | отчёты; 58 из 71 старше 09.2026; уже есть `archive/` | **свалка → архив** |
| diagrams | 13 | 20.10 | 2 | 0 | 1 | 6 | Mermaid | **оставить** (слить с architecture — позже) |
| schemas | 2 | 20.10 | 1 | 1 | 0 | 2 | схемы | **оставить** |
| deployment | 7 | 19.10 | 3 | 7 | 1 | 6 | деплой | **→ runbooks** (позже) |
| analysis | 14 | 12.10 | 1 | 3 | 0 | 11 | разборы | **→ research или архив** |
| music | 12 | 12.10 | 8 | 5 | 0 | 7 | музыка (ADR-0157/0158) | **оставить** |
| runbooks | 4 | 22.09 | 0 | 2 | 0 | 3 | как действовать | **оставить**, сюда стягивать |
| grant | 1 | 21.09 | 0 | 0 | 0 | 0 | заявка | **архив** |
| retros | 6 | 16.09 | 0 | 0 | 0 | 4 | ретро | **архив** |
| diagnostics | 4 | 16.09 | 0 | 0 | 0 | 0 | разовая диагностика | **→ research/архив** |
| investigations | 3 | 14.09 | 0 | 0 | 0 | 4 | расследования | **→ research** |
| recon | 3 | 14.09 | 0 | 0 | 0 | 0 | разведка | **→ research/архив** |
| verification | 2 | 14.09 | 0 | 0 | 0 | 0 | проверки | **архив** |
| agents | 3 | 07.09 | 0 | 0 | 0 | 0 | `domain.md`, `issue-tracker.md`, `triage-labels.md`: их читают скилы `.agents/skills/code-review` и `setup-matt-pocock-skills` | **оставить** (живой, хоть и без ссылок из доков) |
| voice | 1 | 01.09 | 0 | 0 | 0 | 1 | — | **→ architecture/архив** |
| packages | 10 | 30.08 | 0 | 0 | 0 | 5 | обзор пакетов | **оставить** (сверить с `src/`) |
| fixes | 43 | 19.08 | 0 | 1 | 1 | 6 | фикс-заметки; **все 46** в истории старше 09.2026 | **свалка → архив целиком** |
| testing | 2 | 13.08 | 0 | 0 | 0 | 0 | — | **архив** |
| api | 2 | 04.08 | 0 | 0 | 1 | 1 | MiniMax API | **→ architecture** |
| integration | 1 | 22.07 | 0 | 0 | 0 | 0 | — | **архив** |
| prompts | 4 | 27.02 | 0 | 0 | 0 | 1 | старые промты | **архив** |

Одиночные файлы в корне `docs/`:
- `docs/analysis_1344_teleop.md`, `docs/p0-voice-bugs-decisions.md`, `docs/process-fix-roadmap.md`, `docs/refactoring-plan.md`, `docs/music_bench.md`, `docs/MERMAID_DIAGRAMS.md` (30.10.2025) — в соответствующие каталоги или в архив.
- `docs/DOCUMENTATION_STRUCTURE.md` дублирует `docs/README.md` и описывает старую структуру — заменяется картой из этого PR.

### 2.1 Дубли, противоречия, устаревшие факты

| Где | Что не так | Достоверность | Как чинить |
|---|---|---|---|
| `docs/architecture/SYSTEM_OVERVIEW.md:95,152` | «2×10S LiPo», «37V (10S LiPo) × 2». В `HARDWARE.md:713–745` (исправлен 28.10) — 2× smartONE 10S2P **Li-ion**, параллельно, 36/42 В | по коду (два файла противоречат друг другу); факт про батарею — с этикетки, фото Шифу 28.10 | шаг 2 |
| `README.md`, раздел «Cold-start при недоступных образах (Vision Pi)» | `supercollider`, `voice-assistant`, `telegram-bot`, `avatar-arbiter` описаны как сервисы Vision Pi. По `docker/sound/docker-compose.yaml` они на Sound Pi (ADR-0159, с 19–20.10) | по коду | шаг 2 |
| `AGENTS.md`, «Где это уже записано» | ссылка `docs/adr/0013-incremental-delivery-over-big-bang.md` битая: файл переименован в `AF-0013-…`, а номер `0013` теперь у `0013-respeaker-dsp-tuning-mix-ch0.md` | измерено (link-check) | шаг 2, согласовать с ADR-0162 (тот же файл) |
| `README.md` | битые пути `docker/monitor_system.sh`, `scripts/setup/setup_vision_pi.sh:setup_autostart` (второй — ссылка на функцию, не битая) | измерено | шаг 2 |
| `docs/guides/POWER_MANAGEMENT.md:93,110,180` | 3–5S/4S LiPo — это про HAT питания RPi, не про батарею робота | по коду; **не ошибка**, но читается как противоречие | пометить контекст в шаге 2 |
| Точки входа «как работать» | `AGENTS.md`, `CONTRIBUTING.md` (983 строки), `.github/copilot-instructions.md` (сейчас «главное» в нём — `.agents/skills/context-engineering` и метка `source:gsd`), `.cursorrules`, `docs/development/AGENT_GUIDE.md` (1172 строки), `CONTEXT.md`. Шесть мест, разные правила | по коду | одна точка входа — `AGENTS.md` (§3), остальные ссылаются на неё; согласовать с ADR-0162 |
| Корень репо | `SPEC_CURRENT.md` (версия 2.0 от 27.07, ветка `feature/harness-p0-foundation`), `PRD.md` (19.02), `progress.md` («обновляется агентами после каждой задачи»; последний раз 16.09), `BLOCKERS.md` (21.07, test-bench MiniMax), `TEST_BENCH.md` (21.07), `VESC_TIMEOUT_FIX_CHANGELOG.md` (08.11.2025), `ROADMAP.md` (24.07), `ROADMAP_SIMPLE.md` (13.01), `CHANGELOG.md` (16.09) | измерено (даты); суть — по коду | в `docs/archive/root/`; `CHANGELOG`/`ROADMAP` — решать Шифу (§6) |
| `docs/README.md` до этого PR | последняя правка 21.07; нет ADR, research, plans, runbooks, Sound Pi | измерено | заменён в этом PR |

**Чего ещё не проверял** (честно): все 199 ADR на просроченные статусы «Proposed», и факты в `docs/guides/*` против кода. Это отдельные карточки, если Шифу захочет.

---

## 3. Целевая структура

```
AGENTS.md                 — ЕДИНСТВЕННАЯ точка входа «как работать» (правила, честность, куда класть доки)
CONTEXT.md                — доменный словарь
README.md                 — что за робот, быстрый старт, ссылка на docs/README.md
docs/README.md            — карта документации (этот PR)
docs/adr/                 — решения: NNNN (рантайм), AF-NNNN (процесс). Шаблон — как у ADR-0161
docs/architecture/        — как устроено СЕЙЧАС: железо, сеть, контракты, топики. Один факт — одно место
docs/research/            — исследования и замеры с датой в имени: YYYY-MM-DD-<тема>.md (+ подпапка, если данных много)
docs/plans/               — планы активных эпиков; закрыт эпик → docs/archive/
docs/runbooks/            — как действовать: деплой, восстановление, e2e, марафон
docs/guides/              — руководства для людей (настройка, запуск, MiniMax и т. п.)
docs/process/             — как работаем (владелец — ADR-0162)
docs/{dialog,music,design,diagrams,schemas,assets,packages,agents}/ — тематические, остаются
docs/archive/<YYYY-MM>-<тема>/ — всё закрытое и устаревшее, только для истории
evidence/                 — сырые выводы и замеры (не доки)
architecture/ (корень)    — машинные файлы CI (ownership.yml, runtime-baseline.json)
```

**Отчёты и фикс-заметки.** Новые `docs/reports/*` и `docs/fixes/*` не заводим.
- Что сделано и raw-вывод — в теле PR и в issue (так уже требует AGENTS.md).
- Если из фикса выросло долгое знание («почему так устроено») — правим `docs/architecture/` или runbook в том же PR.
- Если это решение — пишем ADR.

### 3.1 Правило на будущее: куда агент кладёт новый документ

Проверяй по порядку, бери первое совпадение:

1. Принято решение с вариантами и последствиями → `docs/adr/NNNN-<slug>.md` (номер — следующий свободный на develop и в открытых PR).
2. Описание того, как система устроена сейчас → правка в `docs/architecture/` (новый файл — только если темы там нет).
3. Пошаговая инструкция «как сделать/починить» → `docs/runbooks/`.
4. Замер, исследование, сравнение → `docs/research/YYYY-MM-DD-<тема>.md`; сырые выводы → `evidence/<тема>-YYYY-MM-DD/`.
5. План эпика → `docs/plans/YYYY-MM-DD-<тема>.md`.
6. Отчёт о работе, разбор PR, фикс-заметка → **не файл**: тело PR / комментарий в issue.
7. Ничего не подошло → спроси шисюна или Шифу; новый каталог в `docs/` без ADR не заводим.

Каталогами `openspec/`, `specs/`, `.planning/` не пользуемся.

---

## 4. Таблица решений

| Что | Решение | Шаг (§5) |
|---|---|---|
| `openspec/changes/avatar-floor-protocol/`, `specs/avatar-floor-protocol/` | **перенести** контракт в `docs/architecture/avatar-floor-protocol.md`, потом удалить | 3 |
| `openspec/changes/quest-voice-pipeline-panel/` | **перенести** в `docs/design/` (если панель жива) или в архив | 3 |
| `openspec/`, `specs/` целиком | **deprecated** (пометка в этом PR) → удалить после шага 3 и после того, как ADR-0162 снимет хуки merge-gate | 3 |
| `docs/research/openspec-pilot/` (116) | **удалить позже** (в git-истории останется; ценность — только как пример неудачи, она описана в `why-agents-dont-use-openspec.md`) | 4 |
| `scripts/import_adr_to_openspec.py`, `.agents/skills/openspec-workflow/` | **удалить позже** | 4 |
| ADR-0038, ADR-0039 | **Superseded by ADR-0163** | 4 |
| `docs/process/agent-flow-openspec-integration.md` | ADR-0162 | — |
| `.planning/` (86) | **deprecated** (пометка) → `docs/archive/2026-05-gsd/` или удалить (решать Шифу). Две ссылки в docstring тестов поправить в том же PR | 5 |
| `analysis/` (корень, 12) | **архив** | 6 |
| `docs/fixes/` (43) | **архив** целиком | 6 |
| `docs/reports/` (65) | **архив**: всё старше 01.10 (живые ссылки из кода — 10 файлов — переписать или оставить файл) | 6 |
| `docs/analysis/`, `investigations/`, `recon/`, `diagnostics/` | **→ research** (живое) / **архив** | 7 |
| `grant/`, `retros/`, `verification/`, `testing/`, `integration/`, `prompts/`, `voice/` | **архив** | 7 |
| `docs/development/` | **разобрать**: стандарты Docker и сборки (на них ссылаются CI и README) → `guides/`, остальное → архив | 8 |
| `docs/deployment/`, `docs/e2e/` | **→ runbooks** | 8 |
| `docs/api/` | **→ architecture** | 8 |
| Корневые `SPEC_CURRENT.md`, `PRD.md`, `progress.md`, `BLOCKERS.md`, `TEST_BENCH.md`, `VESC_TIMEOUT_FIX_CHANGELOG.md`, `ROADMAP_SIMPLE.md` | **архив** `docs/archive/root/` | 9 |
| `docs/DOCUMENTATION_STRUCTURE.md` | **удалить позже** (заменён `docs/README.md`) | 9 |
| `architecture/`, `evidence/`, `.agents/`, `docs/agents/` | **оставить** (живые) | — |

---

## 5. План маленькими PR (ADR-AF-0013)

Общие правила для каждого PR с переносом:
- **До** переноса: `git grep -n '<старый путь>'`, raw-вывод — в тело PR. Ссылки правятся в том же PR.
- **После**: link-check по `AGENTS.md`, `CONTRIBUTING.md`, `README.md`, `docs/README.md` (скрипт из тела этого PR) — 0 новых битых ссылок.
- Переносить только `git mv`, удалять только `git rm` по точным путям, без glob.
- Файлы, которые читает код, не двигать без правки кода в том же PR:
  - `docs/architecture/meta-quest-api.md`;
  - `docs/research/jev/scenarios/acceptance.yaml`;
  - `docs/plans/2026-09-15-service-manifest.md`;
  - `docs/design/ARCHITECTURE_AUDIT.md` и `docs/development/DOCKER_STANDARDS.md` (пути в `on.paths` workflow).
- Один PR — один каталог или одна тема, ≤ ~150 файлов переносов.

| Шаг | PR | Зависит от |
|---|---|---|
| 1 | **Этот PR**: ADR-0163 (Proposed), пометки DEPRECATED в `openspec/`, `specs/`, `.planning/`, новая карта `docs/README.md` | — |
| 2 | Устаревшие факты: `SYSTEM_OVERVIEW.md` (батарея), `README.md` (cold-start → Sound Pi, битые пути), ссылка на AF-0013 в `AGENTS.md` | 1; `AGENTS.md` — после ADR-0162 |
| 3 | Floor-протокол → `docs/architecture/avatar-floor-protocol.md`; quest-панель → `docs/design/`; `git rm` `openspec/`, `specs/` | 1 + ADR-0162 снял хук merge-gate |
| 4 | `git rm` `docs/research/openspec-pilot/`, `scripts/import_adr_to_openspec.py`, `.agents/skills/openspec-workflow/`; ADR-0038/0039 → Superseded | 3 |
| 5 | `.planning/` → архив или `git rm` (по решению Шифу); поправить 2 docstring | 1 |
| 6 | `docs/fixes/`, корневой `analysis/`, старые `docs/reports/` → `docs/archive/` | 1 |
| 7 | Мелкие каталоги (`analysis`, `investigations`, `recon`, `diagnostics`, `grant`, `retros`, `verification`, `testing`, `integration`, `prompts`, `voice`) → research/архив | 6 |
| 8 | `development/`, `deployment/`, `e2e/`, `api/` → guides/runbooks/architecture | 7 |
| 9 | Корневые устаревшие `*.md` → `docs/archive/root/`; `docs/DOCUMENTATION_STRUCTURE.md` → `git rm` | 6; ответ Шифу по CHANGELOG/ROADMAP |

Критерий готовности эпика: в `docs/` не больше 16 каталогов первого уровня (сейчас 32; целевой список — §3). Каждый живой каталог есть в `docs/README.md`. `openspec/`, `specs/`, `.planning/` в develop нет. Link-check по четырём входным файлам — 0 битых ссылок.

---

## 6. Что решать Шифу

1. **`.planning/`: архив или удалить?** История останется в git в любом случае. Предлагаю удалить (вариант Б): содержимое — снимок кода на май–июль, он устарел.
2. **Метка `source:gsd`.** А — перестать ставить новым issue, старые оставить. Б — снять со всех 206 (массовая правка, шумно). Предлагаю А.
3. **`CHANGELOG.md` и `ROADMAP.md`** (не обновлялись с 16.09 и 24.07). А — в архив. Б — оставить и обновить. Предлагаю А: их роль сейчас играют эпики в issue.
4. **Принять ADR** (Proposed → Accepted), тогда можно идти по шагам 2–9.

---

## 7. Последствия

- Плюс: одна карта и одно правило. Агент перестаёт плодить `reports/`/`fixes/`. 116 файлов пилота не засоряют поиск по `docs/research`.
- Минус: много PR с `git mv`, есть риск битых ссылок. Защита — link-check в каждом PR (§5). Отдельный CI-гард на ссылки — по желанию, отдельной карточкой.
- Не меняется: код, CI, рантайм робота.
