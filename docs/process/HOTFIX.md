# Hotfix & Release Process

Этот документ описывает **что делать**, когда нужно экстренно починить или
дополнить релиз, и **как** релизные теги должны проходить через CI/CD.

Создан в рамках issue #1041.

---

## 1. Релизные теги → DOCKER_TAG

В workflow `L-Build Vision Pi Services.yml` и `L-Build Main Pi Services.yml`
есть блок `prepare`, который мапит `GITHUB_REF` в `DOCKER_TAG` (`latest`,
`dev`, `test`, `local`). До фикса #1041 теги `refs/tags/v*` падали в ветку
`else` → `DOCKER_TAG=local`, а файла `.image-versions.local` в репо нет
(есть только `.dev`, `.latest`, `.test`).

**Фикс:** при `GITHUB_REF_TYPE=tag` и имени `v*` явно ставим
`DOCKER_TAG=latest` (релиз ⇒ production). Это согласовано с
`L-Deploy and Verify.yml`, где `environment=production` уже мапится
на `IMAGE_TAG=latest`.

### Маппинг (после фикса)

| GITHUB_REF / источник         | DOCKER_TAG | Файл версий                  |
|-------------------------------|------------|------------------------------|
| `refs/tags/v*`                | `latest`   | `.image-versions.latest`      |
| `refs/heads/main`             | `latest`   | `.image-versions.latest`      |
| `refs/heads/develop`          | `dev`      | `.image-versions.dev`        |
| `refs/heads/feature/*`, `feat/*`, `copilot/*`, `z-{e2e}/*`, `z-{agent}/*` | `test` | `.image-versions.test` |
| любой другой branch или PR    | `local`    | ❌ файла нет (dev-only)       |

### Кто использует эти файлы

- `update-image-versions` job в `L-Build * Pi Services.yml` правит
  `docker/vision/.image-versions.<DOCKER_TAG>` (или `docker/main/...`)
  после успешной сборки, потом коммитит в ветку `ci/image-versions`
  ([см. #1139](https://github.com/krikz/rob_box_project/issues/1139)).
- `L-Deploy and Verify.yml` на Pi делает
  `git checkout origin/ci/image-versions -- docker/*/.image-versions.*`,
  потом читает `.image-versions.${IMAGE_TAG}` и подменяет теги в
  `docker compose pull/up`.

---

## 2. Что делать, если релизный билд ушёл без `.image-versions`

Симптом: после билда на теге в workflow-логе видно
`⚠️ docker/vision/.image-versions.<TAG> not found` (раньше — `local`).
В этом случае `.image-versions.<TAG>` файл остаётся со старыми SHA,
`L-Deploy and Verify` подтянет то, что есть, но **новые SHA-теги образов
не будут замаплены в файл**.

### Быстрый путь (без перезапуска всего билда)

1. Понять, что за билд прошёл — найти Run в Actions, где
   `update-image-versions` job выполнился (даже с warning).
2. Из логов этого job достать `NEW_TAG` (формат
   `<DOCKER_TAG>-<SHORT_SHA>`).
3. На билд-машине вручную пропатчить `.image-versions.<DOCKER_TAG>`:

   ```bash
   cd ~/rob_box_project
   SHORT_SHA=<sha>               # из логов CI
   DOCKER_TAG=latest             # = production для tag-build

   # Vision Pi
   sed -i \
     -e "s|^OAK_D_TAG=.*|OAK_D_TAG=${DOCKER_TAG}-${SHORT_SHA}|" \
     -e "s|^RTABMAP_SYNC_TAG=.*|RTABMAP_SYNC_TAG=${DOCKER_TAG}-${SHORT_SHA}|" \
     -e "s|^LED_MATRIX_TAG=.*|LED_MATRIX_TAG=${DOCKER_TAG}-${SHORT_SHA}|" \
     -e "s|^CEILING_CAMERA_TAG=.*|CEILING_CAMERA_TAG=${DOCKER_TAG}-${SHORT_SHA}|" \
     -e "s|^VOICE_RESOURCES_TAG=.*|VOICE_RESOURCES_TAG=${DOCKER_TAG}-${SHORT_SHA}|" \
     -e "s|^VOICE_ASSISTANT_TAG=.*|VOICE_ASSISTANT_TAG=${DOCKER_TAG}-${SHORT_SHA}|" \
     -e "s|^TELEGRAM_BOT_TAG=.*|TELEGRAM_BOT_TAG=${DOCKER_TAG}-${SHORT_SHA}|" \
     docker/vision/.image-versions.${DOCKER_TAG}

   # Main Pi
   sed -i \
     -e "s|^TWIST_MUX_TAG=.*|TWIST_MUX_TAG=${DOCKER_TAG}-${SHORT_SHA}|" \
     -e "s|^MICRO_ROS_AGENT_TAG=.*|MICRO_ROS_AGENT_TAG=${DOCKER_TAG}-${SHORT_SHA}|" \
     -e "s|^ROBOT_STATE_PUBLISHER_TAG=.*|ROBOT_STATE_PUBLISHER_TAG=${DOCKER_TAG}-${SHORT_SHA}|" \
     -e "s|^RTABMAP_TAG=.*|RTABMAP_TAG=${DOCKER_TAG}-${SHORT_SHA}|" \
     -e "s|^ROS2_CONTROL_TAG=.*|ROS2_CONTROL_TAG=${DOCKER_TAG}-${SHORT_SHA}|" \
     -e "s|^LSLIDAR_TAG=.*|LSLIDAR_TAG=${DOCKER_TAG}-${SHORT_SHA}|" \
     -e "s|^PERCEPTION_TAG=.*|PERCEPTION_TAG=${DOCKER_TAG}-${SHORT_SHA}|" \
     -e "s|^NAV2_TAG=.*|NAV2_TAG=${DOCKER_TAG}-${SHORT_SHA}|" \
     docker/main/.image-versions.${DOCKER_TAG}

   # Закоммитить в ci/image-versions (НЕ в develop/main/vision)
   git checkout -B ci/image-versions
   git add docker/vision/.image-versions.${DOCKER_TAG} \
           docker/main/.image-versions.${DOCKER_TAG}
   git commit -m "ci: backfill ${DOCKER_TAG} SHA tags → ${SHORT_SHA} [skip ci]"
   git push -u origin ci/image-versions
   ```

4. После пуша перезапустить `L-Deploy and Verify` (он подтянет
   свежие `.image-versions` в `git checkout origin/ci/image-versions -- ...`).

### Правильный путь (перезалить релиз)

1. Сделать фикс в CI (см. PR по #1041).
2. Удалить сломанный тег и залить новый:
   ```bash
   git tag -d v1.0.0-1-2ce5de92-humble-rc1
   git push origin :refs/tags/v1.0.0-1-2ce5de92-humble-rc1
   git tag v1.0.0-2-<новый-build-sha>-humble-rc1
   git push origin v1.0.0-2-<новый-build-sha>-humble-rc1
   ```
3. Запустить `L: Build All Services` с этим тегом через `workflow_dispatch`,
   выбрав `ref=v1.0.0-2-...-humble-rc1`.
4. Готово — `.image-versions.latest` будет обновлён.

---

## 3. Превентивные меры

- **Не использовать теги формата `refs/tags/v*` без проверки CI** —
  после фикса #1041 билд на теге корректно пишет
  `.image-versions.latest`, но лучше сначала прогнать на develop
  (`L-Build All Services` через `workflow_dispatch` с `ref=develop`),
  и тег оформлять после зелёного smoke-теста.
- **Не удалять `.image-versions.latest` вручную** — это откатит
  CI-image-preview на Pi. Если нужно "заблокировать" какую-то регрессию,
  лучше откатить тег в registry (`docker rmi <sha>`), или
  выкатить hotfix-релиз с обновлённым файлом.
- **Любой fix в CI-workflows — подумать, не сломает ли он теги**:
  блок `prepare` живёт в трёх местах
  (`L-Build Main Pi Services.yml`, `L-Build Vision Pi Services.yml`,
  `L-Build Single Service.yml`). Изменения в одной — нет, но проверять
  все три.

---

## 4. Changelog

- **2026-08-12** — создан документ, [issue #1041](https://github.com/krikz/rob_box_project/issues/1041)
  (DOCKER_TAG=local для `refs/tags/v*` → фикс на `latest`).
