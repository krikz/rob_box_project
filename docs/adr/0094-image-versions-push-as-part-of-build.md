# ADR-0094 — `.image-versions.*` SHA-tag push: живём в develop из build job (после утери ADR-0031)

**Дата:** 2026-09-14
**Статус:** Accepted (фиксация текущего положения + lint-чекер для phantom env-переменных)
**Автор:** architect (карточка t_dfc688db, issue #2426, по результату ADR-сверки t_a6f2f7e5)
**Тип:** architecture decision, recovering lost ADR
**Связанные ADR:** ADR-AF-0030 (нумерация — два домена AF/RT), ADR-0018 (честность), ADR-0079 (nightly-review persistence), ADR-0089 (AI HAT+ deployment — соседний docker/vision overlay)
**Связанные issues:** #1630 (первоначальный запрос — вынести push в отдельный workflow), #2425 (phantom `MICRO_ROS_AGENT_TAG`), #2377 (phantom SUPERVISOR_TAG/QUEST_TAG), #1388 (race-safe retry), #1560 (concurrency gate)
**Заменяет:** `docs/adr/0031-sha-tag-push-out-of-build-workflow.md` (утерян при renumber 25.08, PR #1662 → `0031-gsd-orphan-triage.md`)

> **Почему ADR-0094, а не «восстановление ADR-0031».** ADR-AF-0030 §1.3 запрещает
> переиспользовать номер, под которым уже существует другой файл. 25.08 в develop
> был влит `0031-gsd-orphan-triage.md` (PR #1662, `0009381b5`). Восстанавливать
> `0031-sha-tag-push-out-of-build-workflow.md` под тем же именем = новая коллизия.
> Берём ADR-0094 — следующий свободный в RT-домене.

---

## 1. Контекст и проблема

### 1.1 История утерянного ADR-0031

25.08.2026 в develop существовал `docs/adr/0031-sha-tag-push-out-of-build-workflow.md`
(3 ревизии, последний коммит `5a49225a6`). В тот же день при renumber
ADR-номеров (PR #1662) файл был заменён на `0031-gsd-orphan-triage.md`,
а `0031-sha-tag-push-out-of-build-workflow.md` исчез из develop.

```
$ git rev-list --all -- docs/adr/0031-sha-tag-push-out-of-build-workflow.md
5a49225a67d8af2df2d0fbb60d7de9037a6f8334
8388c8f6bd08ec727332a10a8b5c205ecc0d12f1
e8f34d1d6596e4e1c2344176a3ff65885a4e56c9

$ git rev-list --all -- docs/adr/0031-gsd-orphan-triage.md | head -1
f43fd16fdb37c4d43609e0bde4dbee59e03ad312
```

Это известный класс сборе процесса нумерации (см. ADR-AF-0030 §1.1), но
**не отменяет того факта, что сам ADR был нужен**: фиксированное в нём решение
про SHA-tag push сейчас не соблюдается.

### 1.2 Что было в ADR-0031 (rev3, `5a49225a6`)

ADR-0031 предлагал:

1. Вынести SHA-tag push в отдельный workflow `L-Update Image Versions.yml`.
2. Триггеры: `workflow_call` (build → update), `workflow_dispatch` (manual retry), `schedule: 0 * * * *` (hourly cron).
3. Race-condition dedup через `gh api` + проверка сообщения последних 20 коммитов в develop.
4. Build-workflow (Main/Vision) делает только registry push + пишет `.image-versions.*` на диск; `git push` в develop уходит из build в update-workflow.
5. Acceptance: build НЕ делает `git push` в develop, update-workflow делает.

### 1.3 Что произошло в реальности

Ни один из пунктов 1.1–1.5 не реализован.

```
$ ls .github/workflows/ | grep -i "update"
# (пусто — L-Update Image Versions.yml не существует)

$ ls docs/adr/0031*
docs/adr/0031-gsd-orphan-triage.md        # это НЕ про image-versions

$ git branch -r | grep image-versions
# (пусто — ветки ci/image-versions-main/-vision не существуют)

$ git log --oneline -10 --grep="SHA tags"
ca70f2ea ci: main SHA tags → dev-75c8f41 [skip ci]
e8df5c41 ci: vision SHA tags → dev-75c8f41 [skip ci]
75c8f41d ci: vision SHA tags → dev-87bb7ab [skip ci]
...
```

SHA-tag push делается **напрямую из build job** `update-image-versions` в
`L-Build Main Pi Services.yml:326` и `L-Build Vision Pi Services.yml:521`,
через `scripts/ci/push-image-versions.sh`. Скрипт `scripts/ci/commit-image-versions.sh`
(который должен был работать с per-service ветками `ci/image-versions-main/-vision`)
есть в репозитории, но **никем не вызывается**.

`docs/process/HOTFIX.md` §«Preventative checklist» (post-#1142) ссылается на
«SHA-теги идут в `ci/image-versions`» — это утверждение устарело и само стало
архитектурным долгом: документация говорит одно, код делает другое.

### 1.4 Симптомы потери ADR

1. **Phantom env-переменная** в `.image-versions.*`:
   ```
   $ for tag in TWIST_MUX_TAG MICRO_ROS_AGENT_TAG ...; do
       hits=$(grep -rl "$tag" docker/ scripts/ \
              | grep -v '.image-versions' | wc -l)
       echo "$tag: $hits usages"
     done
   TWIST_MUX_TAG:           1
   MICRO_ROS_AGENT_TAG:     0     # ← phantom
   ROBOT_STATE_PUBLISHER:   1
   RTABMAP_TAG:             1
   ROS2_CONTROL_TAG:        1
   LSLIDAR_TAG:             1
   PERCEPTION_TAG:          1
   NAV2_TAG:                1
   ```
   `MICRO_ROS_AGENT_TAG` (issue #2425) и `SUPERVISOR_TAG`/`QUEST_TAG` (issue #2377) — никто
   не проверяет, что они **используются** хотя бы в одном compose/script/dockerfile.
   Build пишет → CI коммитит → deploy игнорирует. ADR, который бы это
   запретил как acceptance criterion, отсутствует.

2. **Race-condition dedup полагается на скрипты, не на ADR.** В §4.2 ADR-0031 был
   `gh api` fingerprint-check + `IMAGE_VERSIONS_DEDUP_REMOTE_QUERY` feature flag.
   Сейчас есть только `pull --rebase` в `push-image-versions.sh` (строки 47-71)
   + `GIT_TERMINAL_TIMEOUT=60`. При 3 параллельных раундах (develop + 2 e2e)
   race всё ещё существует (см. issue #t_dd49849b — 2.7h+ hang).

3. **Build НЕ изолирован от git push.** Если `git push` в develop зависает на 2.7h
   (наблюдалось 25.08), `update-image-versions` job падает по `timeout-minutes: 10`,
   build помечается failed → ретрай → ещё один SHA-tag коммит. Acceptance ADR-0031
   «build НЕ делает git push в develop» не выполнен.

---

## 2. Что НЕ предлагается

- **Не предлагаю «восстановить ADR-0031 с тем же номером»** — создаст новую
  коллизию (AF-0030 SOT по нумерации). Номер 0031 занят другим документом
  (`0031-gsd-orphan-triage.md`).
- **Не предлагаю «вернуть per-service ветки `ci/image-visions-main/-vision`»** —
  было отклонено в самом ADR-0031 (вариант B, §3 таблица) из-за deploy-проблем
  (#1244 ретро: deploy не подтягивает `.image-versions` оттуда). Ветки в текущем
  deploy contract не читаются.
- **Не предлагаю «вынести push в отдельный workflow прямо сейчас»** — это было
  основное предложение ADR-0031, но с 25.08 (90 дней назад) ни одна попытка
  реализации не дошла до develop. Почему — см. §3.1 (отложено, см. ADR-0030 §1.3
  «отложено»: требует отдельной карточки + e2e-смок-теста на deploy).
- **Не предлагаю «откатить HOTFIX.md до актуального состояния»** — это
  mechanical fix, не architecture decision. Делается отдельной devops-карточкой.

---

## 3. Принятое решение (что фиксируем)

### 3.1 Фиксируем текущее положение (as-is)

SHA-tag push `.image-versions.{dev,test,latest}` делается в develop напрямую
из job `update-image-versions` build-workflow (Main/Vision), через
`scripts/ci/push-image-versions.sh`. Никакого отдельного
`L-Update Image Versions.yml` workflow нет.

Это **не идеал** (см. §1.4), но это **то, что работает сейчас** и для чего
есть bash-скрипты с retry+rebase+timeout. ADR фиксирует это положение как
SOT, чтобы:

- Любой будущий рефакторинг (например, реализация «варианта A» ADR-0031)
  опирался на документированный baseline, а не на утерянный ADR.
- HOTFIX.md и другие process-docs синхронизировались с этим baseline.

### 3.2 Добавляем lint-чекер phantom env-переменных (b)

Принятый ранее в ADR-0031 §6 acceptance criterion «env-переменные в
`.image-versions.*` должны быть использованы хотя бы в одном compose/script
файле компонента» — **сохраняется как валидное требование**, но enforcement
делается не через architecture review, а через механический CI-step.

**Реализация:** новый скрипт `scripts/ci/check_image_versions_usage.sh`
(запускается в CI как часть PR-проверки `L-Build All & Push to GHCR.yml` или
отдельным `lint-images.yml`):

```bash
#!/usr/bin/env bash
# Проверяет, что каждый *_TAG из docker/{main,vision}/.image-versions.*
# используется хотя бы в одном файле вне .image-versions.
#
# Использование:
#   check_image_versions_usage.sh            # проверяет все .image-versions.*
#   check_image_versions_usage.sh docker/main # проверяет только docker/main
#
# Выход:
#   0 — все *_TAG используются
#   1 — есть phantom *_TAG (выводит список)
set -euo pipefail
COMPONENT="${1:-}"

for f in docker/${COMPONENT}/.image-versions.*; do
  [ -f "$f" ] || continue
  component=$(dirname "$f" | sed 's|docker/||')
  while IFS='=' read -r key val; do
    [ -z "$key" ] && continue
    [[ "$key" =~ ^# ]] && continue
    # Ищем использование $key как compose env / dockerfile ARG / shell var
    hits=$(grep -rl --include="*.yml" --include="*.yaml" \
                       --include="*.env" --include="*.sh" \
                       --include="Dockerfile*" --include="*.dockerfile" \
           "$key" docker/"$component" scripts/ 2>/dev/null \
      | grep -v ".image-versions" || true)
    if [ -z "$hits" ]; then
      echo "❌ phantom: $component/$key (value=$val) — не используется" >&2
      RC=1
    fi
  done < "$f"
done
exit "${RC:-0}"
```

CI-step:

```yaml
- name: Check image-versions usage (no phantoms)
  run: bash scripts/ci/check_image_versions_usage.sh
```

**Scope:** только `docker/main` и `docker/vision` (как и сам build-pipeline).
Vision-сервисы (`OAK_D_TAG`, `LED_MATRIX_TAG`, `VOICE_*_TAG` и т.д.) проверяются
по тому же правилу.

### 3.3 Процедура удаления сервиса (c)

При удалении `docker/<component>/<service>/` (любой каталог с Dockerfile,
compose-фрагментом или ROS-пакетом):

1. Удалить сервис и его зависимости из CI-workflow (`L-Build Main/Vision Pi Services.yml`).
2. **Удалить соответствующий `*_TAG`** из
   `docker/<component>/.image-versions.{dev,test,latest}` (если есть).
3. Удалить env-переменную из `docker-compose*.yml`/`compose.env` (если есть).

Шаг 2 — **полуавтоматический**: после шага 1 запускается
`scripts/ci/check_image_versions_usage.sh` (см. §3.2) и подсвечивает
orphan-теги. Удаление — ручное, отдельным коммитом в develop с сообщением
`docker(<component>): drop orphan <TAG>`.

Шаг 3 — manual, так как compose-файлы могут использовать env через
`environment:` блок с другим именем (например, `IMAGE_TAG` вместо `*_TAG`).

### 3.4 Что НЕ делается (d)

- SHA-tag push **НЕ выделяется в отдельный workflow** (вариант A ADR-0031
  отклонён, см. §2). Текущая архитектура развивается иначе: build-job
  `update-image-versions` остаётся в `L-Build Main/Vision Pi Services.yml`.
- Per-service ветки `ci/image-versions-main/-vision` **НЕ возвращаются**
  (вариант B ADR-0031 отклонён, см. §2).
- **Никаких изменений в `scripts/ci/push-image-versions.sh`** — retry+rebase+timeout
  (lines 47-71) — рабочий baseline. Если потребуется dedup через `gh api`
  (вариант §4.2 ADR-0031) — это отдельная задача, не блокирует принятие
  ADR-0094.

---

## 4. Acceptance criteria

- [ ] **ADR-0094 в develop.** Этот файл закоммичен и смержен.
- [ ] **Phantom-чекер существует.** `scripts/ci/check_image_versions_usage.sh`
      запускается в CI и завершается с кодом 0 на текущем состоянии develop
      (после ручного удаления `MICRO_ROS_AGENT_TAG` и других phantom-тегов —
      отдельная карточка, см. §6).
- [ ] **HOTFIX.md синхронизирован.** Секция «Preventative checklist» обновлена:
      SHA-теги идут в develop, не в `ci/image-versions`. Делается в рамках
      этой же PR (одна карточка = одна сессия, см. process-rules).
- [ ] **Удаление сервиса проходит через §3.3.** В `CONTRIBUTING.md` (если
      есть секция про docker/<component>/) или в HOTFIX.md §«Preventative
      checklist» добавлен пункт «перед merge удаления сервиса — прогнать
      `check_image_versions_usage.sh` и почистить orphan-теги».

---

## 5. Риски и митигации

| Риск | Митигация |
|---|---|
| Phantom-чекер даёт false-positive (тег используется через `${VAR}_TAG` паттерн) | Whitelist в начале скрипта (`KNOWN_PHANTOMS` env var) + ручной override при ревью |
| Удаление сервиса забывают синхронизировать с `.image-versions.*` | §3.3 делает это **полу-автоматическим**: orphan-теги подсвечиваются CI, но удаляются руками (см. почему не полный автотмат в §3.3) |
| HOTFIX.md поедет снова, если кто-то внесёт изменения в `.image-versions.*` flow без ADR | ADR-AF-0030 §2.1 + validate_adr_namespace.sh — pre-merge guard на коллизии номеров, но **не** на content drift. Это принятый trade-off (см. ADR-AF-0030 §3.2 «Phase 2 не решает content drift») |
| `MICRO_ROS_AGENT_TAG` и `SUPERVISOR_TAG`/`QUEST_TAG` остаются в `.image-versions.*` до того, как devops-карточка их удалит | ADR-0094 **фиксирует**, что они phantom, но **не требует** их немедленного удаления. Удаление — отдельная карточка (см. §6) |

---

## 6. Связанные задачи (не делаются в этом PR)

Этот PR делает **только**: добавляет ADR-0094, скрипт-чекер, синхронизирует
HOTFIX.md. Следующие работы — отдельными карточками:

1. **devops:** удалить phantom-теги из `.image-versions.*` (`MICRO_ROS_AGENT_TAG`,
   `SUPERVISOR_TAG`, `QUEST_TAG`) + связанные env-переменные из compose.
   Issue #2425, #2377.
2. **devops:** добавить `gh api`-based dedup в `push-image-versions.sh`
   (вариант §4.2 ADR-0031, отложен с 25.08).
3. **architect (отложено):** реализовать вариант A ADR-0031 — вынести push в
   отдельный `L-Update Image Versions.yml` workflow. Требует e2e smoke-test
   на deploy (сейчас ветки не существует → smoke невозможен → задача заморожена).

---

## 7. Тесты

- **Unit:** `scripts/ci/tests/test_check_image_versions_usage.sh`
  - Создаёт tempdir с `docker/main/.image-versions.dev` (один used, один phantom) + fake `docker/main/compose.yml`
  - Ожидает exit 1 и упоминание phantom-тега в stderr.
  - Удаляет fake compose → exit 1 для обоих.
  - Удаляет phantom-тег → exit 0.
- **Shellcheck:** в CI (`L-Build All & Push to GHCR.yml` или отдельный
  `lint-images.yml`) — `shellcheck -S warning scripts/ci/check_image_versions_usage.sh`.
- **Integration (out-of-scope):** ручной запуск через
  `bash scripts/ci/check_image_versions_usage.sh` после merge → должен
  сообщить о текущих phantom-тегах (см. §1.4 #1) и exit 1, пока их не удалят.

---

## 8. Rollout

1. Merge этой PR → develop.
2. CI-build на develop: lint-step провалится на phantom-тегах → отдельная
   devops-карточка (§6 #1) чистит их → build зелёный.
3. HOTFIX.md обновлён до актуального состояния — будущие CI-workflow изменения
   ссылаются на правильный baseline.

---

## 9. История и supersede

- **2026-08-25:** ADR-0031 (proposed → rev3) — вынести SHA-tag push в отдельный
  workflow. Утерян при renumber (PR #1662, `0009381b5`).
- **2026-09-14:** ADR-сверка t_a6f2f7e5 → обнаружен архитектурный долг.
- **2026-09-14:** настоящий ADR-0094 (этот файл) — фиксирует текущее
  положение + добавляет phantom-чекер + синхронизирует HOTFIX.md.

Если в будущем будет реализован вариант A ADR-0031 (отдельный workflow) —
ADR-0094 supersede'ится новым ADR (следующий свободный номер в RT-домене),
а этот файл помечается `Superseded by ADR-NNNN`.