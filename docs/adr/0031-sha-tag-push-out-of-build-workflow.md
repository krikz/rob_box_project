# ADR-0031: SHA-tag push вне build workflow — отдельный `L-Update Image Versions.yml`

| Поле            | Значение                                                                                                                                                                                              |
|-----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Статус          | **Proposed** (PR открыт из ветки `z-{agent}/1630-...`)                                                                                                                                                |
| Дата            | 2026-08-25                                                                                                                                                                                            |
| Автор           | devops (Hermes Agent)                                                                                                                                                                                  |
| Kanban          | `t_0dc7e76c`                                                                                                                                                                                          |
| Контекст        | Issue [#1630](https://github.com/krikz/rob_box_project/issues/1630) — SHA-tag push в develop напрямую из `L-Build Main/Vision Pi Services.yml` удлиняет build и спамит develop SHA-tag коммитами          |
| Затрагивает     | `.github/workflows/L-Update Image Versions.yml` (NEW), `.github/workflows/L-Build Main Pi Services.yml`, `.github/workflows/L-Build Vision Pi Services.yml`, `scripts/ci/push-image-versions.sh`     |
| Родители        | ADR-0018 (честность), ADR-0014 (issue-closure), #1628 (timeout+retry), #1560 (concurrency gate), #1482 (GITHUB_SHA + verify), #1388 (race-safe helper)                                                  |
| Предшественники | #1139 v2 (per-service ветки — ревёртнут в #1244 из-за deploy), #1316 (push в develop через retry+rebase)                                                                                              |
| Связанные       | Issue [#1630](https://github.com/krikz/rob_box_project/issues/1630), [#1075](https://github.com/krikz/rob_box_project/issues/1075) (SHA-tag спам), [#1388](https://github.com/krikz/rob_box_project/issues/1388) (race), [#1506](https://github.com/krikz/rob_box_project/issues/1506) (e2e зависают), [#1560](https://github.com/krikz/rob_box_project/issues/1560) (concurrency), ADR-0030 (e2e stale-branch guard) |
| Реализация      | PR будет открыт после коммита реализации                                                                                                                                                              |

---

## 1. Контекст и бизнес-проблема

`L-Build Main Pi Services.yml` и `L-Build Vision Pi Services.yml` после успешного build делают `git pull --rebase && git commit && git push` в develop (см. `update-image-versions` job, строки 382-525). Это создаёт три проблемы:

1. **Build долгий.** Git network ops могут висеть (наблюдалось 2.7h+, issue #t_dd49849b). Даже с `timeout-minutes: 10` из #1628 это +10 мин к каждому build.
2. **develop засоряется** SHA-tag коммитами (по 2-3 на каждый build, ~13 PR за 25.08 — `git log --grep='ci:.*SHA tags'`).
3. **Race-condition:** main и vision оба коммитят в одну ветку. PR #1388 добавил retry+rebase, но каждый retry создаёт новый коммит, если SHA тот же.

Issue #1630 формализует: **вынести SHA-tag push в отдельный workflow**, чтобы build был чисто registry-push.

## 2. Что НЕ работает в текущем решении (PR #1628 уже смержен)

`#1628` починил только видимый fail (`timeout-minutes: 10` + `GIT_TERMINAL_TIMEOUT=60`). Но:

- **Build всё ещё зависит от успешного push** — `update-image-versions` в `needs:` от build-* jobs. Push падает → весь build помечается failed → ретрай → ещё один SHA-tag коммит (если бы push прошёл с первого раза).
- **Concurrency gate (#1566) работает только на уровне workflow**, не на уровне отдельных jobs. Три разных workflow (develop + 2 e2e) могут одновременно войти в `update-image-versions` и race'иться на git ref (наблюдалось в #t_dd49849b).
- **Нет retry-trigger** — если push упал, никто его не повторит. develop остаётся со stale `.image-versions` пока кто-то вручную не запустит build.

## 3. Рассмотренные варианты

| Вариант | Плюсы | Минусы | Решение |
|---|---|---|---|
| **A. Вынести push в отдельный workflow `L-Update Image Versions.yml`**, build вызывает его через `workflow_call` | Build становится чисто registry-only (быстрее). Падение push не валит build. Cron-trigger для retry. | +1 workflow, +1 yaml to maintain | ✅ **Принят** |
| B. Вернуться к per-service `ci/image-versions-*` веткам (как #1139 v2) | Не засоряет develop | Deploy не подтягивает .image-versions оттуда (#1244 ретро) | ❌ Отклонён |
| C. Кэшировать SHA-tag push в GitHub cache, не в git | — | Deploy не знает про cache | ❌ Отклонён |
| D. Делать push через pre-receive hook на runner | Контроль на стороне робота | Требует self-hosted hook infra, не работает с GHA `actions/checkout` | ❌ Отклонён |
| E. Вообще убрать `.image-versions` (deploy тянет по latest-stable) | Меньше артефактов | Нарушает существующий deploy contract | ❌ Отклонён (рассмотрено в ADR-0030, отложено) |

## 4. Решение

### 4.1 Новый workflow `.github/workflows/L-Update Image Versions.yml`

**Triggers:**
- `workflow_call` (build workflow зовёт после build)
- `workflow_dispatch` (ручной retry)
- `schedule: 0 * * * *` (hourly cron для retry пропущенных push'ей)

**Inputs:**
- `service: 'main' | 'vision'`
- `docker_tag: 'dev' | 'test' | 'latest' | 'local'` (наследуется из build)
- `source_sha: string` (GitHub SHA билда, для traceability)

**Job `update-versions`:**
1. `actions/checkout@v7` c `fetch-depth: 0`
2. Pull `origin/develop` (current_branch)
3. **Read** `.image-versions.${docker_tag}` с диска — build его уже записал.
4. **Sed SHA tags** (тот же набор, что в build).
5. **Dedup** (см. 4.2).
6. `git commit -m "ci: ${service} SHA tags → ${tag}-${sha} [skip ci]"`
7. `bash scripts/ci/push-image-versions.sh "$CURRENT_BRANCH" $service`

**Не делает:** docker build, docker push, registry verify (это всё остаётся в build workflow).

### 4.2 Race-condition dedup в `push-image-versions.sh`

**Перед push** — проверить через `gh api`:

```bash
COMMIT_MSG="ci: ${service} SHA tags → ${tag}-${sha}"
gh api "repos/${GH_REPO}/commits?sha=${BRANCH}&per_page=20" \
  | jq -r '.[].commit.message' \
  | grep -F -q "${tag}-${sha}" \
  && { echo "ℹ️  dedup: SHA tag ${tag}-${sha} уже в ${BRANCH} — skip push"; exit 0; }
```

- **Cheap** (1 API call, 20 коммитов).
- **Idempotent** — два параллельных билда с одинаковым SHA не плодят дублей.
- **Feature-flagged** через `IMAGE_VERSIONS_DEDUP_REMOTE_QUERY=1` (default ON, можно выключить для отладки).

### 4.3 Build workflow (`L-Build Main/Vision Pi Services.yml`)

- ❌ **Удалить** job `update-image-versions` (строки 382-525).
- ✅ **Оставить** `tag_and_push()` + `verify_in_registry()` внутри build jobs — это registry-операция, не git.
- ✅ **Добавить** в каждый build job финальный шаг: `WRITE .image-versions.${tag}` на диск + вызов `L-Update Image Versions.yml` через `uses:` с `if: success() && inputs.push_to_registry != false`.

**NB:** вызов отдельного workflow из build job через `uses:` создаст `workflow_run` trigger, который и так покрыт schedule. Это даёт **retry-ability** без повторного build.

### 4.4 Что остаётся неизменным

- `L-Build All & Push to GHCR.yml` — там уже per-service модель через `commit-image-versions.sh` (для main branch). Не трогаем.
- `scripts/ci/push-image-versions.sh` — оставляем retry+rebase+timeout логику из #1628.
- `scripts/ci/commit-image-versions.sh` — не затрагивается.

## 5. Инвариант завершения (как должно стать)

```text
build workflow RUN:
  build-* → registry push (verify_in_registry)
  финал  → write .image-versions.${tag} (только fs)
  финал  → workflow_call L-Update Image Versions.yml (best-effort)
  
L-Update Image Versions.yml RUN:
  pull origin/develop
  read .image-versions.${tag} (уже записан build'ом)
  sed SHA tags
  dedup через gh api → skip если уже есть
  git commit + push (retry+rebase+timeout)
  НЕ блокирует build
  
если push упал:
  build всё равно success (его tag ушёл в registry)
  hourly cron retry'ит push
```

## 6. Acceptance criteria (из issue #1630, пересмотрено)

- [ ] **Push SHA-тегов вынесен в отдельный workflow** `L-Update Image Versions.yml` — создаём
- [ ] **Build workflow НЕ делает `git push` в develop** — `update-image-versions` job удалён
- [ ] **Build workflow только пушит в registry** (localhost:5000 + ghcr.io) — без изменений
- [ ] **Update-image-versions запускается отдельно** — workflow_call + workflow_dispatch + cron `0 * * * *`
- [ ] **Если SHA-tag push падает — build не повторяется** — `update-image-versions` больше не в `needs:`
- [ ] **Race-condition dedup** — `gh api` чек в `push-image-versions.sh`
- [ ] **e2e не зависят от SHA-tag** — уже выполнено (ADR-0030, latest-stable tag)

## 7. Риски

| Риск | Митигация |
|---|---|
| Новый workflow падает → SHA-tags не доходят до develop | Cron retry каждый час. Параллельно build уже положил тег в registry — deploy может его подтянуть через IMAGE_TAG override |
| Dedup через `gh api` тормозит push на 1-2 сек | Приемлемо (push и так 5-30 сек). Кэш через ETags если будет проблема |
| Build workflow вызывает `L-Update` через `workflow_call` — добавляет уровень индирекции | Это намеренно — разделение concerns |
| Кто-то забудет обновить оба build workflow | Чеклист в PR description + grep-тест в CI |

## 8. Тесты

- **Unit:** `scripts/ci/tests/test_push_image_versions_dedup.sh` — fake `gh api` возвращает existing commit → скрипт exit 0 без push
- **Unit:** `scripts/ci/tests/test_update_image_versions_workflow.sh` — bashlint нового workflow файла
- **Integration (out-of-scope):** ручной запуск через `gh workflow run "L-Update Image Versions.yml" -f service=main -f docker_tag=dev` после merge → проверить, что появился коммит `ci: main SHA tags → dev-${sha} [skip ci]` в develop

## 9. Rollout

1. Merge этого PR → develop
2. Следующий build на develop: build success, push не делает git push, workflow_call L-Update (может упасть — не критично)
3. Cron каждый час повторяет пропущенные push
4. Через 1-2 дня: проверить `git log develop --grep='ci:.*SHA tags' | wc -l` — должно быть **меньше**, чем до rollout (ранее ~13 за день, target ≤ 3)
