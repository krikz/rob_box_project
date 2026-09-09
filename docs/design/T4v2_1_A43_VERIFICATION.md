# T4v2.1 — A43 voice-action-server aiohttp FIX VERIFIED

**Date**: 2026-08-06 11:23 MSK (post-deploy)
**Branch**: feature/harness-p0-foundation
**Commits**: 2e6ab214 + 337548a3
**Live robot**: 10.1.1.21 (Vision Pi)
**Build host**: 10.1.1.249
**Image after fix**: `10.1.1.249:5000/krikz/rob_box:voice-assistant-humble-test@sha256:b0a6ce31513c1033be7e7e316250a185e87e0b2f28579fdbba3d06d870a84e4d`

---

## BEFORE (06.08 09:00–10:50 MSK, see E2E_TESTING_DESIGN_v2 §5.1)

voice-action-server в `Restarting (1)` цикле. 399 рестартов с 06.08 01:03.

```
ModuleNotFoundError: No module named 'aiohttp'
File "/ws/install/rob_box_voice/lib/python3.10/site-packages/rob_box_voice/action_server/http_server.py", line 19, in main
    from aiohttp import web
```

A43 OPEN_DEGRADED по snapshot.sh v2: `unhealthy_container_count > 0`.

## FIX

Корень: `docker/vision/voice_assistant/requirements.txt` декларирует `<aiohttp>`, но
этот файл НЕ потребляется Dockerfile (он использует только `voice_base/requirements.txt`
через `pip install -r` в `docker/vision/voice_base/Dockerfile`). Без `voice_base` rebuild
старый pip-кэш тоже не переустанавливал aiohttp.

Двойная фикса:

1. **Dockerfile pin (commit 337548a3)** — добавлен явный `RUN pip3 install` в
   `voice_assistant` app-layer: `aiohttp>=3.9.1,<4.0`. Размещение выбрано
   преднамеренно: `voice_base` не трогаем, иначе инвалидируется ~10 GB
   pytorch/renardo cache у всех downstream consumers.

2. **requirements.txt + http_server.py (commit 2e6ab214)** — резервные меры:
   `aiohttp` явно указан в `voice_assistant/requirements.txt` (для случая,
   если кто-то в будущем добавит `pip install -r requirements.txt` шаг),
   а `http_server.py` обёрнут в `_import_aiohttp()` хелпер с понятным
   `RuntimeError` хинтом (вместо криптического `ModuleNotFoundError`).

## DEPLOY (на живом роботе 10.1.1.21)

```
cd /home/ros2/rob_box_project            # build host 10.1.1.249
git fetch origin                         # picked up 337548a3
git checkout 337548a3                    # detached HEAD на commit с fix

SOURCE_HASH=$(find src/rob_box_voice/rob_box_voice \
              src/rob_box_animations/rob_box_animations \
              -type f -name "*.py" | sort | xargs sha256sum | sha256sum | awk '{print $1}')

docker buildx build \
  --platform linux/arm64 \
  --file docker/vision/voice_assistant/Dockerfile \
  --tag "localhost:5000/krikz/rob_box:voice-assistant-humble-test" \
  --tag "localhost:5000/krikz/rob_box:voice-assistant-humble-test-337548a" \
  --load \
  --add-host=host.docker.internal:host-gateway \
  --build-arg="BASE_IMAGE=ghcr.io/krikz/rob_box:voice-base-humble-test" \
  --build-arg="SOURCE_HASH=${SOURCE_HASH}" \
  .
# → built sha256:b9070785d7fb6cc923221f609a2f19b35a05b610fcf510172792ea3c7114c2c9

docker push localhost:5000/krikz/rob_box:voice-assistant-humble-test
# → digest sha256:b0a6ce31513c1033be7e7e316250a185e87e0b2f28579fdbba3d06d870a84e4d
```

**ВНИМАНИЕ**: docker-compose попытка `up -d` на 10.1.1.21 хангает — compose
резолвит `ghcr.io/krikz/rob_box:voice-assistant-humble-test` (без registry
override) и долбится в интернет. Workaround: прямой `docker run` с локальным
registry image + корректным healthcheck (см. `/tmp/run_voice_action.sh` на
роботе). Альтернатива для будущего: поправить compose на `${SERVICE_IMAGE_PREFIX}`
override через `.env.feature` (там уже есть переменная, но не подцепляется).

## AFTER (06.08 11:23 MSK)

```
$ docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Image}}" | grep voice
voice-action-server   Up 3 minutes (healthy)   10.1.1.249:5000/krikz/rob_box:voice-assistant-humble-test
voice-assistant       Up 26 minutes (healthy)  ff0aac5c7a0e

$ curl http://127.0.0.1:8765/healthz
{"ok": true, "active_goals": 0, "shutting_down": false}

$ docker logs voice-action-server --tail 200 | grep -iE 'module|traceback|error'
(no matches)

$ bash tests/e2e/live/snapshot.sh
[snapshot] state_snapshot.json + snapshot.json written
state_snapshot.json:
{
  "snapshot_at": "2026-08-06T08:22:15.195396Z",
  "schema": "v2",
  "robot": "10.1.1.21",
  "voice_action_server": {
    "module_not_found_crashes": 0,
    "restart_count_total": 0,
    "degraded": false
  },
  "unhealthy_container_count": 0
}
```

## ACCEPTANCE — A43

- [x] `docker ps` на 10.1.1.21: voice-action-server STATUS = `Up 3 minutes (healthy)`
- [x] `docker logs voice-action-server --tail 200` НЕ содержит ModuleNotFoundError
- [x] snapshot.sh v2: `unhealthy_container_count == 0`, `module_not_found_crashes: 0`
- [x] E2E_TESTING_DESIGN_v2 §D A43: OPEN_DEGRADED → VERIFIED (commit 337548a3)

## NOTES

1. **Stale doc reference**: в моём коммите 337548a3 §B.4 ссылается на
   «Коммит: 13a2a060», которого нет в истории. По факту фикс = `2e6ab214
   + 337548a3`. Косметика — не блокирует A43 VERIFIED, но стоит поправить
   одним словом в следующем PR.

2. **docker-compose gotcha**: при деплое compose пытается стянуть
   `ghcr.io/krikz/rob_box:voice-assistant-humble-test` (полный upstream
   registry), а не из `localhost:5000`. На роботе это приводит к зависанию.
   Workaround сейчас — прямой `docker run`. Долгосрочно — перевыпустить
   `docker/vision/.env.test` так, чтобы `SERVICE_IMAGE_PREFIX` смотрел на
   локальный registry, или вынести registry override в отдельный `.env`.

3. **voice-assistant** все ещё работает на SHA `ff0aac5c7a0e` (старом) — это
   отдельный контейнер, тоже использует voice-assistant image, но не запускает
   http_server. После редеплоя всех связанных с voice_assistant сервисов через
   compose, оба перейдут на новый SHA. Для A43 это не блокер.
