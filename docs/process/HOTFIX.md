# Hotfix & Tag-Handling Procedures

Этот документ фиксирует операционные процедуры для CI/CD релизного трека rob_box.
Создан как сопровождение к фиксу #1041 (DOCKER_TAG mapping для git tags).

## DOCKER_TAG mapping (issue #1041)

`L-Build Vision Pi Services.yml` и `L-Build Main Pi Services.yml` определяют
`DOCKER_TAG` по `GITHUB_REF_TYPE` + `GITHUB_REF_NAME`:

| Source ref                        | DOCKER_TAG |
|-----------------------------------|------------|
| `refs/tags/vX.Y.Z-N-<sha>-<dist>-rcN` | `latest` |
| `refs/heads/main`                 | `latest`   |
| `refs/heads/develop`              | `dev`      |
| `refs/heads/{feature,feat,copilot,z-{e2e},z-{agent}}/*` | `test` |
| любая другая branch               | `local`    |

**Почему tag-detection явный, а не через `BRANCH_NAME`:** для git tag
`GITHUB_REF = refs/tags/v1.0.0-...`, после `strip refs/heads/` остаётся
`refs/tags/v1.0.0-...`, что не матчит ни одну ветку и падает в else →
`DOCKER_TAG=local`. Файлы `.image-versions.local` в репо не существуют
(есть только `.dev`/`.latest`/`.test`), поэтому build падал с warning
`docker/vision/.image-versions.local not found`. Полный список
`.image-versions.*` репозитория — это `.dev` и `.latest` (`.test` создаётся
runtime-скиптами).

**Согласованность с L-Deploy and Verify.yml:** release-деплой уже использует
`IMAGE_TAG=latest` для production. Поэтому `v*` → `latest` — это одна точка
истины, а не новая "release-ветка".

**SHA-теги коммитятся в текущую ветку (decision #1316, 15.08):** SHA-теги
`.image-versions.*` коммитятся ПРЯМО в ту ветку, из которой собран билд
(`develop`/`main`/test/round). 🔴 НЕ выносить в `ci/image-versions-*` ветки —
этот механизм сломал деплой дважды (#1139 13.08, #1244 13.08) и НЕ работает
при `registry_source=local` (деплой не грузит `.image-versions` из ci-ветки,
compose тянет голые `dev-*` теги → роботы получают старые образы). Коммиты
`ci: SHA tags → dev-*` в develop — это НОРМАЛЬНО, не «засорение» (issue #1316).

## Backfill для уже-прошедших релиз-тегов

Релиз `v1.0.0-1-2ce5de92-humble-rc1` был собран ДО этого фикса — без
`.image-versions.local`. Сборка не падала (warning), но и SHA-теги для
vision-сервисов не были закоммичены (build упал на стадии
`commit image-versions`). ⚠️ Ниже описан СТАРЫЙ порядок (ветка
`ci/image-versions`) — он оставлен только для истории; с 15.08 (decision
#1316) SHA-теги коммитятся ПРЯМО в текущую ветку, см. раздел выше.

### Как восстановить SHA-теги после-фактум

1. По артефактам workflow run `L-Build Vision Pi Services` для тега
   `v1.0.0-1-2ce5de92-humble-rc1` (run_id из #1041, например 31152997328)
   достать SHA каждого сервиса из собранных docker images:

   ```bash
   # self-hosted runner, где проходил build
   docker images --format '{{.Repository}}:{{.Tag}} {{.ID}}' \
     | grep 'rob_box/vision-.*-humble-latest' \
     | awk '{print $1, $3}'
   ```

2. Рассчитать короткие SHA (7 символов) и записать в
   `docker/vision/.image-versions.latest` в ветке `ci/image-versions`:

   ```bash
   git fetch origin ci/image-versions
   git checkout ci/image-versions
   # обновить docker/vision/.image-versions.latest
   git commit -m "ci: vision SHA tags → v1.0.0-1-<sha> [skip ci]"
   git push origin ci/image-versions
   ```

3. Прогнать `L-Build Vision Pi Services` с `--ref v1.0.0-1-2ce5de92-humble-rc1`
   повторно — теперь workflow корректно определит `DOCKER_TAG=latest`, найдёт
   `.image-versions.latest` и сам обновит SHA-теги.

### После фикса

Следующий релиз (`v1.0.0-2-<build>-humble-rc1`) этим мануалом пользоваться
не должен — фикс `GITHUB_REF_TYPE=tag` уже в `L-Build Vision Pi Services.yml`
и `L-Build Main Pi Services.yml` (merged в develop).

## Preventative checklist для CI-workflow изменений

Перед merge любого изменения в `.github/workflows/L-Build * Services.yml`:

- [ ] `DOCKER_TAG` mapping всё ещё соответствует таблице выше
- [ ] Если добавлен новый branch-prefix в `DOCKER_TAG=test` — добавить его
      в ОБА файла (Vision Pi и Main Pi), иначе асимметрия
- [ ] Если меняется `.image-versions.*` flow — учитывать, что SHA-теги
      коммитятся ПРЯМО в текущую ветку (develop/main), НЕ в `ci/image-versions-*`
      (decision #1316, см. issue #1316)
- [ ] Прогон workflow в `workflow_dispatch` с реальным release-тегом
      (`v*`) для smoke-test, прежде чем закрывать задачу

## Связанные ADR

- ADR-0014 — merge-gate требует e2e-доклад в issue перед `kanban complete`
- ADR-0015 — naming convention веток (z-{agent}/*, z-{e2e}/*, copilot/*)
- issue #1316 (decision) — SHA-теги НЕ выносить в ci/image-versions-* ветки,
  коммитить в develop/текущую ветку (запрет навсегда; c873f479/#1142 и
  85db3f5c/#1244 сломали деплой дважды)
