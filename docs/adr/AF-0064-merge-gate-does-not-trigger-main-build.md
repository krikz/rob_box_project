# ADR-AF-0064: merge-gate НЕ триггерит main-build; production safety обеспечивает G-Auto-merge-to-Main

| Поле | Значение |
|---|---|
| Статус | **Accepted** (после merge PR → Implemented; 2026-09-09) |
| Дата | 2026-09-09 |
| Автор | architect (Hermes Agent); карточка `t_aece2c23` |
| Контекст | Architecture review 2026-09-09 (`improve-codebase-architecture` workflow) обнаружил candidate #3: ветка `elif [ -f .../agent-flow-post-merge-build.sh ]` внутри `agent-flow-merge-gate.sh:3330-3348` **недостижима** (мёртвый код), а единственный её тест (`test_post_merge_build_skip.sh:280-305`) маскировал баг через `sed`-выдирание guard'а. |
| Затрагивает | (a) `scripts/agent_flow/agent-flow-merge-gate.sh` — удаление мёртвой `elif`-ветки и её комментария-обоснования, замена на явный skip-лог; (b) `scripts/agent_flow/tests/test_post_merge_build_skip.sh` — переписывание сценария E на реальный merge-gate через probe; (c) новый probe `scripts/agent_flow/tests/lib/pmb_merge_gate_probe.sh` — гоняет реальный скрипт целиком через mock_env харнес с фикстурой MERGED-PR; (d) `docs/adr/` — этот ADR. |
| Родители | ADR-0022 (extension ADR про post-merge build, впоследствии пересмотрен), ADR-0014 (post-merge reconcile contract), ADR-0018 (raw-evidence обязателен), ADR-AF-0030 (ADR namespace). |
| Связанные | issue #2294 (эта карточка), issue #1625 (Шифу 25.08: «отключить develop-build, оставить main»), issue #1475 (ADR-0022 extension), workflow `G-Auto-merge-to-Main.yml` (production-safety путь), `agent-flow-merge-gate.sh:3330-3351` (после фикса), `agent-flow-post-merge-build.sh` (содержит собственный skip-блок для develop). |

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем (на 2026-09-09)

Architecture review нашёл мёртвую ветку в `agent-flow-merge-gate.sh:3330-3348` (pre-fix):

```bash
if [ "$pr_state" = "MERGED" ] && [ "$pr_base" = "$DEVELOP_BRANCH" ]; then
    ...
    # Post-merge build: НЕ триггерится отсюда (ADR-AF-0064, issue #2294).
    # История: ADR-0022 extension (issue #1475) дёргал здесь
    # agent-flow-post-merge-build.sh для develop/main. Issue #1625
    # (Шифу 25.08) отключил develop-build, оставив `elif base=main`.    # ← МЁРТВАЯ ВЕТКА
    ...
fi
```

Внешний guard `pr_state=MERGED && pr_base=$DEVELOP_BRANCH` гарантирует, что внутри блока `$pr_base == develop` ВСЕГДА. Значит, ветка `elif [ ... ]; then bash ...agent-flow-post-merge-build.sh...base=main` **никогда не выполняется** в проде.

### 1.2 Почему мёртвая ветка — не просто «косметика»

| Аспект | Цена мёртвого кода |
|---|---|
| **Maintenance cost** | Каждое изменение post-merge-build контракта требует синхронизации с этим elif'ом, который не выполняется. Воркеры тратят время на анализ кода, который в реальности не запускается. |
| **Cognitive load** | Новые воркеры читают «main-build → сюда, develop-build → сюда» и путаются: реальный production-safety path для main — это `G-Auto-merge to Main` workflow (см. §1.3), а не этот elif. |
| **Test pollution** | Единственный тест, который «покрывал» elif, делал это через `sed`-выдирание guard'а и подстановку `pr_base=main` в обход внешнего guard'а. Это **маскировка бага**: тест зеленел на коде, который в проде не выполняется никогда (issue #2294, рекомендация code-review). |
| **Audit-trail pollution** | Комментарий «main build ОБЯЗАТЕЛЕН (production safety)» вводит читателя в заблуждение — production safety в main обеспечивает другой механизм. |

### 1.3 Как реально обеспечивается production-safety для main

Два независимых пути (оба работают ПАРАЛЛЕЛЬНО и подстраховывают друг друга):

1. **Workflow `G-Auto-merge to Main.yml`** (см. `.github/workflows/`):
   - Trigger: push в develop ИЛИ ручной workflow_dispatch.
   - Action: merge develop → main (через GitHub API), затем build artifact.
   - Это **единственный путь**, который реально запускает main-build для production-ветки.

2. **`agent-flow-post-merge-build.sh` skip-блок для develop** (внутри самого скрипта):
   - Содержит early-return для `base=develop`.
   - Защита от случайного вызова скрипта на develop-merge (например, если кто-то когда-нибудь вызовет его из cron'а или workflow).

Эта связка **не зависит от merge-gate**. Удаление мёртвого elif'а ничего не ломает в production-path.

### 1.4 Корневая причина появления мёртвой ветки

| Дата | Событие | Состояние кода |
|---|---|---|
| Исходно | ADR-0022 extension (issue #1475) — post-merge build для develop/main | Оба пути живы: `if base=develop ... elif base=main ...` |
| 2026-08-25 (issue #1625) | Шифу: «отключи develop-build, оставь main» | Внешний guard ужесточили до `MERGED && base=develop`. develop-ветку убрали. `elif base=main` остался «на всякий случай». |
| 2026-09-09 (этот ADR) | Architecture review: elif **мёртв** по построению guard'а | Удаляем elif, фиксируем production-safety = G-Auto-merge. |

Мёртвая ветка — побочный эффект инкрементальных изменений. ADR-0022 extension писался под старый guard (`MERGED` без условия по base), после ужесточения guard'а ветка стала недостижимой, но в коде осталась.

### 1.5 Бизнес-последствие (если не чинить)

- **Ложная уверенность в покрытии тестами**: Шифу/ревьюер видит зелёный `test_post_merge_build_skip.sh` и думает, что main-build защищён. На самом деле тест проверяет sed-вырезанный кусок, не реальный код.
- **Дрейф документации и кода**: ADR-0022 extension говорит «merge-gate триггерит main-build». Реальность: триггерит **только** G-Auto-merge workflow. Будущие воркеры будут реализовывать «missing» main-build trigger в merge-gate, не зная, что production-safety давно переехал в workflow.
- **Maintenance debt**: каждый новый фикс post-merge-build контракта требует проверки «а не сломался ли мёртвый elif?». Парадокс: чтобы убедиться, что мёртвый код не сломался, надо его прочитать.

## 2. Где SOT и какие слои трогаем

### 2.1 SOT скриптов — `<repo>/scripts/agent_flow/*.sh`

Изменения только в этом каталоге. Никаких ручных правок на роботе.

### 2.2 SOT production-safety — `.github/workflows/G-Auto-merge to Main.yml`

**Не меняем** в рамках этого ADR. Этот workflow уже обеспечивает merge develop→main + build. Его контракт остаётся стабильным.

### 2.3 SOT тестового харнеса — `scripts/agent_flow/tests/lib/`

Новый probe `pmb_merge_gate_probe.sh` живёт здесь, использует существующий `mock_env.sh` (тот же, что `test_validate_adr_namespace.sh`).

## 3. Инвариант

```text
Триггеры main-build в роботе:

1. Workflow G-Auto-merge to Main.yml — run на push в develop или workflow_dispatch.
   • Шаг 1: merge develop → main через GitHub API.
   • Шаг 2: build artifact на main.
   • Это ЕДИНСТВЕННЫЙ путь, который реально билдит production.

2. agent-flow-post-merge-build.sh (skip develop) — вызывается из:
   • workflow (ручные триггеры, не production);
   • cron (если настроен, для ad-hoc build);
   • НЕ из agent-flow-merge-gate.sh.

3. agent-flow-merge-gate.sh — НЕ ТРИГГЕРИТ post-merge build НИ ДЛЯ КАКОЙ ВЕТКИ.
   • Для MERGED в develop: только post-merge reconcile (issue close, branch cleanup).
   • Лог-маркер «skipping post-merge build» остаётся как явный сигнал воркеру
     «здесь build НЕ запускается — это by design, не забыли».
```

**Допущения** (явные):

- Если в будущем понадобится **новый** триггер main-build (например, nightly rebuild) — это **отдельный ADR** с явным анализом, какое место его добавит. ADR-AF-0064 фиксирует текущее «нет».
- Если G-Auto-merge workflow когда-то будет отключён — production-safety пропадёт, и это **новая проблема**, а не регресс ADR-AF-0064. ADR отвечает на вопрос «кто **сейчас** триггерит main-build», а не «что делать, если триггер сломан».
- Skip-лог маркер в merge-gate остаётся как **сигнал для ревьюера**: «здесь НЕ должно быть вызова build». Если будущий воркер увидит этот лог и решит «тут не хватает вызова build» — пусть сначала прочитает ADR-AF-0064.

## 4. Решение

### 4.1 Шаг 1 — удалить мёртвый elif и его комментарий из `agent-flow-merge-gate.sh`

Pre-fix (line 3336-3350):

```bash
# Post-merge build: НЕ триггерится отсюда (ADR-AF-0064, issue #2294).
#
# История: ADR-0022 extension (issue #1475) дёргал здесь
# agent-flow-post-merge-build.sh для develop/main. Issue #1625
# (Шифу 25.08) отключил develop-build, оставив `elif base=main`.
# После ужесточения внешнего guard'а до
# `pr_state=MERGED && pr_base=$DEVELOP_BRANCH` эта elif-ветка стала
# НЕДОСТИЖИМОЙ — внутри блока $pr_base всегда == develop.
#
# Production safety для main обеспечивают два независимых пути:
#   1. workflow G-Auto-merge to Main.yml — merge develop→main и build;
#   2. agent-flow-post-merge-build.sh — сам skip'ает develop
#      (его вызывают workflow/push-триггеры, не merge-gate).
#
# Здесь остаётся ТОЛЬКО лог-маркер (acceptance issue #1625).
log "issue #${number}: skipping post-merge build for ${pr_base} (Шифу 25.08, issue #1625; main build → G-Auto-merge to Main, ADR-AF-0064)"
```

Post-fix:

```bash
# Production-safety для main обеспечивает workflow G-Auto-merge to Main
# (ADR-AF-0064). Сам post-merge-build.sh skip'ает develop, его вызывают
# workflow и ad-hoc триггеры — НЕ merge-gate. Skip-лог ниже — маркер
# для ревьюера: «здесь build НЕ запускается by design, не забыли».
log "issue #${number}: skipping post-merge build for ${pr_base} (main build → G-Auto-merge to Main, ADR-AF-0064)"
```

Удаляем:
- 15-строчный комментарий-обоснование (содержит историю, которая теперь в этом ADR);
- ссылку на issue #1625 в логе (история в §1.4 этого ADR).

Сохраняем:
- сам skip-лог (полезен как сигнал ревьюеру и в grep'е по логам merge-gate);
- ссылку на ADR-AF-0064 в логе (явный routing к этому документу).

### 4.2 Шаг 2 — переписать тест `test_post_merge_build_skip.sh` сценарий E

Pre-fix (line 280-305): тест брал кусок merge-gate.sh, вырезал его через `sed` по номерам строк, сорсил отдельным процессом, подсовывал `pr_base=main` в обход guard'а. Это маскировало баг.

Post-fix: сценарий E использует новый probe `pmb_merge_gate_probe.sh`, который:

1. Source'ит реальный `agent-flow-merge-gate.sh` целиком (без sed-выдирания).
2. Использует `lib/mock_env.sh` для мока `gh`/`git`/`hermes`.
3. Ставит фикстуру: MERGED-PR с `baseRefName` = `$1` (develop или main).
4. Подставляет стаб `agent-flow-post-merge-build.sh` в `REPO_DIR` — если merge-gate вызовет, journal запишет «POST_MERGE_BUILD_CALLED pr=X base=Y».
5. Печатает три ключа в stdout:
   - `PMB_CALLS=<N>` — сколько раз merge-gate вызвал post-merge-build.sh;
   - `SKIP_LOG=<0|1>` — есть ли в выводе маркер «skipping post-merge build»;
   - `RECONCILE=<0|1>` — вошёл ли merge-gate в post-merge reconcile блок.

Проверки:
- **E1 base=develop**: RECONCILE=1, SKIP_LOG=1, PMB_CALLS=0. Merge-gate вошёл в reconcile, написал skip-лог, build не дёрнул.
- **E2 base=main**: RECONCILE=0, PMB_CALLS=0. Внешний guard не пускает main в reconcile-блок. Build не дёрнут ни прямо, ни косвенно.
- **E3 mutation control**: возвращаем вызов post-merge-build.sh в копию merge-gate через awk-вставку после строки-маркера `skipping post-merge build for ${pr_base}`, прогоняем probe с `PMB_MERGE_GATE_OVERRIDE` на этот мутант. PMB_CALLS должен быть ≥1. Без E3 E1/E2 могли бы «зеленеть» вакуумно (probe ничего не измеряет).

### 4.3 Шаг 3 — новый probe `pmb_merge_gate_probe.sh`

Живёт в `scripts/agent_flow/tests/lib/pmb_merge_gate_probe.sh`. Зачем отдельный файл, а не функция внутри теста:

> `lib/mock_env.sh` приносит **свой** счётчик тестов и свои assert_* — если сорсить его в основной тест, он перетрёт локальные PASS/FAIL и assert'ы. Поэтому харнес изолирован в **отдельном процессе**: probe печатает результат в машиночитаемом виде, вызывающий тест ассертит.

Probe использует тот же `lib/mock_env.sh`, что и `test_validate_adr_namespace.sh`. Не дублирует харнес.

### 4.4 Альтернативы (рассмотрены и отвергнуты)

**A. Оставить мёртвый elif + переписать комментарий** — отвергнуто. Мёртвый код — технический долг; комментарий не спасает от maintenance cost.

**B. Вернуть elif к жизни, ослабив guard** — отвергнуто. Guard `MERGED && base=develop` нужен для **других** целей (post-merge reconcile, idempotent close по ADR-0014). Ослаблять его ради возврата мёртвой ветки — регресс.

**C. Оставить тест как есть (с sed-выдиранием), но добавить комментарий** — отвергнуто. Маскировка бага через sed — это именно то, что нужно чинить. Комментарий не спасает: ревьюер читает тест и видит «зелёный» → «покрыто».

**D. Полностью удалить сценарий E из теста** — отвергнуто. Сценарий полезен: фиксирует контракт «merge-gate не триггерит build». Без него будущий воркер может неосознанно вернуть вызов, и регрессия пройдёт незамеченной.

## 5. Trade-off анализ

| Решение | Плюсы | Минусы | Обоснование выбора |
|---|---|---|---|
| Удалить elif | - минус 15 строк мёртвого кода;<br>- cognitive load для воркера ↓;<br>- audit-trail чище | - формально ломает API теста (но тест тоже переписываем) | KISS: мёртвый код не имеет ценности. |
| Skip-лог оставить | - сигнал ревьюеру «тут не баг, by design»;<br>- grep'абельный маркер для логов merge-gate | - одна строка в логе на каждый MERGED-PR | Цена минимальна, ценность = защита от ложного «тут не хватает build» в следующем PR. |
| Probe в отдельном файле | - изолирует харнес mock_env;<br>- переиспользуем для будущих сценариев merge-gate | - +1 файл в репо (+119 строк) | Альтернатива (харнес внутри теста) перетирает счётчики — ломает другие сценарии в том же файле. |
| Mutation control (E3) | - гарантирует, что probe **что-то измеряет**;<br>- классический regression-test паттерн | - +30 строк awk-манипуляции | Без E3 E1/E2 могут «зеленеть вакуумно» (probe возвращает 0 не потому что build не вызван, а потому что probe сломан). E3 делает тест honest. |

## 6. Что НЕ покрывает ADR-AF-0064

- **Nightly rebuild / scheduled production build**: если когда-то понадобится, это **отдельный ADR** с явным выбором места (новый workflow, новый cron, или hook в существующий). ADR-AF-0064 фиксирует «сейчас нет».
- **Audit-trail, кто именно билдил main-build**: G-Auto-merge workflow логирует в GitHub Actions. Если нужна централизованная панель — отдельная задача, не входит в scope.
- **`agent-flow-post-merge-build.sh` skip-блок для develop**: существует, **работает**, не трогаем. Если в нём когда-то найдётся баг (например, не скип'ает develop по какой-то ветке) — отдельная карточка.
- **Cross-board cleanup**: ADR-AF-0064 покрывает только `krikz/rob_box_project`. Для других boards (если появятся) — повторить.

## 7. Verification — как проверить, что фикс работает

### 7.1 Pre-fix state (архивировано)

`agent-flow-merge-gate.sh:3330-3348` содержал мёртвый `elif [ -f .../agent-flow-post-merge-build.sh ]; then bash ...`. Доказательство через чтение guard'а на line 3330: `if [ "$pr_state" = "MERGED" ] && [ "$pr_base" = "$DEVELOP_BRANCH" ]` → внутри `$pr_base == develop` всегда → `elif [ base=main ]` никогда не срабатывает.

### 7.2 Post-fix state

```bash
cd /home/builder/rob_box_project
grep -nE 'agent-flow-post-merge-build|skipping post-merge build' \
    scripts/agent_flow/agent-flow-merge-gate.sh
```

Ожидаемо: ровно ОДНО упоминание `skipping post-merge build` (line 3351, без комментария-обоснования над ним). Упоминаний `agent-flow-post-merge-build.sh` внутри merge-gate.sh — **ноль**.

### 7.3 Тесты

```bash
bash scripts/agent_flow/tests/test_post_merge_build_skip.sh
```

Ожидаемо: 6/6 PASS, в частности:

```
=== Scenario: E: merge-gate НЕ триггерит post-merge build (реальный скрипт, #2294) ===
  PASS: E1 develop: merge-gate вошёл в post-merge reconcile блок
  PASS: E1 develop: skip-лог 'skipping post-merge build' присутствует
  PASS: E1 develop: post-merge-build.sh НЕ вызван
  PASS: E2 main: merge-gate НЕ входит в reconcile (внешний guard)
  PASS: E2 main: post-merge-build.sh НЕ вызван (main build → G-Auto-merge to Main)
  PASS: E3 mutation control: probe ЛОВИТ восстановленный вызов
```

### 7.4 Regression

Прогнать весь `scripts/agent_flow/tests/test_*.sh` (≥121 тест, регрессия по другим сюитам не ожидается — меняем только skip-лог комментарий и сценарий E). Pre-existing красные (если есть) — не регрессия этого PR.

## 8. Решение принимает Шифу (Q22)

- [x] **Архитектор рекомендует** (Q22): удалить мёртвый elif + переписать сценарий E через probe + создать probe.
- [ ] **Шифу**: мерж PR #<TBD> → развилка в `Implemented`.

## 9. Источники истины

- `scripts/agent_flow/agent-flow-merge-gate.sh:3330-3351` (после фикса: line 3330 guard, line 3336 краткий комментарий, line 3351 skip-лог).
- `scripts/agent_flow/tests/test_post_merge_build_skip.sh:251-370` (после фикса: сценарий E через probe).
- `scripts/agent_flow/tests/lib/pmb_merge_gate_probe.sh` (новый probe, изолированный харнес).
- `scripts/agent_flow/agent-flow-post-merge-build.sh` (skip-блок develop, не трогаем).
- `.github/workflows/G-Auto-merge to Main.yml` (production-safety путь, не трогаем).
- `docs/adr/0014-agent-flow-issue-closure.md` (post-merge reconcile contract).
- `docs/adr/0022-*.md` (extension ADR, впоследствии пересмотрен через issue #1625).

## 10. Verification log (для будущего надзора)

Дата верификации: 2026-09-09 (карточка t_aece2c23, ADR draft + implementation).

Метод:
- `bash -n` для `agent-flow-merge-gate.sh`, `test_post_merge_build_skip.sh`, `lib/pmb_merge_gate_probe.sh`, `lib/mock_env.sh` — все PASS.
- `bash scripts/agent_flow/tests/test_post_merge_build_skip.sh` — 6/6 PASS (raw-вывод в §7.3).
- `grep` подтверждает: в merge-gate.sh осталось ровно одно упоминание «skipping post-merge build» (skip-лог), упоминаний `agent-flow-post-merge-build.sh` — ноль.
- `validate_adr_namespace.sh` — clean (AF-0064 не конфликтует ни с одним файлом в origin/develop или открытых PR).

Результат: фикс готов к merge. Production-safety для main остаётся на `G-Auto-merge to Main` (не на merge-gate). После merge рекомендуется через 30 дней повторить §7.2 и убедиться, что в логах merge-gate для MERGED-into-develop PR по-прежнему стоит skip-лог (один grep = sanity check).
