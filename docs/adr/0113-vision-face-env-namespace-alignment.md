# ADR-0113 — vision-face: согласование имён ENV с vision-hailo (убрать `FACE_*` namespace)

**Дата:** 2026-09-15
**Статус:** Accepted (issue #2655, F-1 из component review 2026-09-15, t_4b487ef8)
**Автор:** architect worker (kanban t_53258025)
**Тип:** architecture decision + bug-fix (silently-broken integration, MEDIUM severity)

## 1. Контекст

`docker/vision/docker-compose.yaml` для сервиса `vision-face` (ADR-0089 §2.1 Phase 2,
issue #2599 PR-A — детекция лица RetinaFace отдельным процессом) экспортирует в
контейнер переменные окружения с префиксом `FACE_*`:

```yaml
# docker/vision/docker-compose.yaml:732-733
- HAILO_ENABLED=${FACE_HAILO_ENABLED:-false}
- HEF_PATH=${FACE_HEF_PATH:-}
```

Внутри контейнера их читает `docker/vision/scripts/vision-hailo/start_vision_face.sh`
(по образцу `start_vision_hailo.sh`):

```bash
# docker/vision/scripts/vision-hailo/start_vision_face.sh:15-16
HAILO_ENABLED="${HAILO_ENABLED:-false}"
HEF_PATH="${HEF_PATH:-}"
```

То есть **compose маппит `FACE_*` → общие `HAILO_ENABLED/HEF_PATH`**, но при дефолте
`FACE_HAILO_ENABLED` в `.env` НЕ задан → compose подставляет `:-false` → в контейнере
`HAILO_ENABLED=false`. Скрипт читает `HAILO_ENABLED=false` → запускает launch с
`hailo_enabled:=false` → нода стартует в stub-режиме.

**Симптом:** Оператор, поставивший `FACE_HAILO_ENABLED=true` в `.env` по аналогии с
другими параметрами лицевого контура, не получает real inference. Vision-face молча
работает в stub-режиме (capability-deception, нарушает ADR-0018 «capability-honest»).
Маркер `hailo_enabled=true` в логе ноды устанавливается только при `HAILO_ENABLED=true`,
а в текущем `.env.example` (`docker/vision/.env.example:1-76`) про `FACE_HAILO_ENABLED`
вообще ничего нет — оператор **не может** найти правильное имя по документации.

У vision-hailo (line 670-671) контракт согласован: compose читает `HAILO_ENABLED` /
`HEF_PATH` (без префикса), и `start_vision_hailo.sh` читает те же имена. У vision-face —
**рассогласование namespace**, выявленное component review 2026-09-15 (t_4b487ef8).

### 1.1. Почему это MEDIUM, а не HIGH

- Стейбл-режим работы по умолчанию (`HAILO_ENABLED=false`) — это stub, как и было до
  введения vision-face (Phase 1). Дефект **не ломает существующий CI/dev-режим**.
- Регрессия наступает при попытке активировать real inference — оператор видит
  ошибку только через healthcheck (`/scripts/healthcheck_face_frame.sh`), когда
  `/vision/hailo/events` молчит. Это **silently-broken integration**, маскируемая
  stub-режимом (противоречит ADR-0018 §capability-honest).

### 1.2. Что проверил (file:line)

- `docker/vision/docker-compose.yaml:732-733` — `HAILO_ENABLED=${FACE_HAILO_ENABLED:-false}`,
  `HEF_PATH=${FACE_HEF_PATH:-}`.
- `docker/vision/scripts/vision-hailo/start_vision_face.sh:15` — `HAILO_ENABLED="${HAILO_ENABLED:-false}"`
  (без `FACE_` префикса).
- `docker/vision/scripts/vision-hailo/start_vision_face.sh:16` — `HEF_PATH="${HEF_PATH:-}"`.
- `docker/vision/scripts/vision-hailo/start_vision_face.sh:101` — launch arg
  `hailo_enabled:=${HAILO_ENABLED}`.
- `docker/vision/scripts/vision-hailo/start_vision_face.sh:111` — launch arg
  `hef_path:=${HEF_PATH}` (только если непустой).
- `docker/vision/docker-compose.yaml:670-671` — vision-hailo использует общие имена
  (для сравнения: там всё корректно).
- `docker/vision/scripts/vision-hailo/start_vision_hailo.sh:23-24` — vision-hailo
  читает `HAILO_ENABLED`/`HEF_PATH` без префикса (для сравнения).
- `docker/vision/.env.example:1-76` — `FACE_*` нигде не упоминается; следовательно,
  префикс **никогда не задокументирован** для оператора.
- `tests/unit/scripts/test_start_vision_hailo_yaml_loader.py` — тестирует
  YAML-loader через `HAILO_ENABLED`/`HEF_PATH`, не зависит от имён ENV → **фикс
  не ломает контракт тестов**.

## 2. Решение

**Убрать `FACE_` префикс в compose, использовать общие `HAILO_ENABLED` / `HEF_PATH` —
как у vision-hailo.** Минимальное, согласовано с существующим контрактом.

```diff
# docker/vision/docker-compose.yaml (vision-face, ~732-733)
-      - HAILO_ENABLED=${FACE_HAILO_ENABLED:-false}
-      - HEF_PATH=${FACE_HEF_PATH:-}
+      - HAILO_ENABLED=${HAILO_ENABLED:-false}
+      - HEF_PATH=${HEF_PATH:-}
```

`start_vision_face.sh` **не меняется** — он уже читает правильные имена.

### 2.1. Альтернатива (отвергнута)

**Вариант B — явный лицевой namespace + fallback на общие имена:**
```bash
HAILO_ENABLED="${FACE_HAILO_ENABLED:-${HAILO_ENABLED:-false}}"
HEF_PATH="${FACE_HEF_PATH:-${HEF_PATH:-}}"
```

Плюс: сохранение отдельного namespace (теоретическая возможность оператору задать
только `FACE_*`, не трогая `HAILO_ENABLED`).
Минус:
- Дополнительная сложность в bash (двойной fallback).
- Namespace нигде не задокументирован и не используется — нельзя проверить контракт.
- vision-hailo уже использует общие имена — расходимся с соседним сервисом.

**Выбор — вариант A** (минимум правок, согласовано с vision-hailo, единый контракт).

### 2.2. Принцип (новое правило для всех vision-* сервисов)

> Все vision-* сервисы используют **общие** имена `HAILO_ENABLED` / `HEF_PATH` /
> `STUB_PERIOD_SEC` / `CONFIDENCE_THRESHOLD` / `NMS_IOU_THRESHOLD` / `GAZE_SOURCE` /
> `FIRST_FRAME_TIMEOUT_SEC` / `OUTPUT_TOPIC`. Namespace-префикс (например,
> `FACE_*`, `OBJECT_*`) **запрещён** — каждое имя должно работать одинаково во
> всех vision-* entrypoint-скриптах.

Если в будущем понадобится per-service override (например, face-confidence ≠
object-confidence), вводить его в `hailo_models.yaml` (SSoT) с явным
`face.confidence_threshold` / `object.confidence_threshold` — **не** через
ENV-префиксы.

## 3. Trade-off

| Что выигрываем | Что теряем |
|---|---|
| Единый контракт с vision-hailo (single namespace, единый operator mental model) | Теоретическую возможность задать `FACE_HAILO_ENABLED` независимо от `HAILO_ENABLED` (которая нигде не задокументирована и никем не использовалась) |
| Минимальный diff (2 строки compose, ноль — bash) | — |
| Bash-скрипт уже корректен — фикс только в compose | — |
| Тесты `test_vision_face_loader.py` / `test_start_vision_hailo_yaml_loader.py` продолжают работать без правок | — |
| Соответствует ADR-0018 «capability-honest» — нет silently-broken integration | — |

## 4. Что НЕ делаем сейчас

- **Не** вводим `FACE_*` namespace через bash-fallback (вариант B отвергнут).
- **Не** правим `.env.example` — он и так не упоминает `FACE_*` (фикс только
  убирает мёртвую ссылку).
- **Не** добавляем capability-honest smoke-test в CI на этот фикс — это будет
  отдельная задача (см. §5).

## 5. Follow-up (отдельные карточки)

- **F-1.1 (эта карточка t_53258025):** фикс compose-имён (2 строки).
- **F-1.2 (отдельная карточка, architect):** capability-honest smoke-test —
  `docker compose -f docker/vision/docker-compose.yaml config | grep HAILO_ENABLED`
  + git blame pre-receive hook на будущие расхождения namespace.
- **F-1.3 (отдельная карточка, devops):** ADR-0113 → line в
  `docs/development/AI_DEVELOPMENT_REVIEW.md` и `KNOWN_OPERATIONAL_WARNINGS.md` —
  «все vision-* сервисы используют общие имена ENV».

## 6. Acceptance (для PR)

- [ ] `docker compose -f docker/vision/docker-compose.yaml config | grep -E 'HAILO_ENABLED|HEF_PATH'`
      для сервиса `vision-face` показывает `HAILO_ENABLED=${HAILO_ENABLED:-false}`,
      `HEF_PATH=${HEF_PATH:-}` (без `FACE_`).
- [ ] `grep -n 'FACE_HAILO_ENABLED\|FACE_HEF_PATH' docker/vision/docker-compose.yaml` → пусто.
- [ ] `grep -n 'HAILO_ENABLED\|HEF_PATH' docker/vision/scripts/vision-hailo/start_vision_face.sh` →
      две строки `HAILO_ENABLED=` и `HEF_PATH=` (без `FACE_`).
- [ ] Existing pytest: `pytest src/rob_box_perception/test/unit/test_vision_face_loader.py
      tests/unit/scripts/test_start_vision_hailo_yaml_loader.py -v` — без регрессий.

## 7. Touchpoints

- ADR-0089 §2.1 Phase 2 (vision-face отдельный процесс).
- ADR-0018 §capability-honest (нет silently-broken integration).
- ADR-0110 (vision-hailo launch decoupling — тот же контракт имён).
- Issue #2655 (component review 2026-09-15, t_4b487ef8).
- Issue #2599 (PR-A face detection — этот фикс блокирует активацию real inference).