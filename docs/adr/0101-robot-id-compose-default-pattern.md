# ADR-0101 — ROBOT_ID inline-default в docker-compose (vision + main)

**Дата:** 2026-09-15
**Статус:** Accepted (architect verdict, fix handed off to backend)
**Автор:** architect worker (карточка t_be87cd1d, issue #2569)
**ADR-number:** 0101 (collision-fix, 2026-09-15: см. issue #2582, ADR-AF-0068 — два соседних PR #2575/«Повод» и #2578/«Взгляд» получили 0101 независимо; этот ADR — первый по merge-time `9485018f`, остальные переехали в 0103 и 0104)
**Тип:** Configuration / developer-experience decision

## Контекст

Vision-сервисы (и main-сервисы) объявляют `ROBOT_ID=${ROBOT_ID}` без
inline-дефолта. Если `.env` пуст / отсутствует / не содержит `ROBOT_ID` —
CI-сборка, свежий clone или локальный dev без `.env` ловят
`"empty value for variable"` и `docker compose up` падает до старта.

Issue #2569 зафиксировал blocker на `docker/vision/docker-compose.yaml:665`
(vision-hailo, первый vision-hailo-сервис в рамках ADR-0089 Phase 1). При
аудите того же файла найдено ещё 8 аналогичных вхождений. Параллельно
найден **тот же баг** в `docker/main/docker-compose.yaml` (9 вхождений).

## Решение

**Применить compose-style inline default `:-<value>` к каждому вхождению
`ROBOT_ID=${ROBOT_ID}`.**

В `docker/vision/docker-compose.yaml`:
```yaml
- ROBOT_ID=${ROBOT_ID:-rob_box_dev}
```

То же — в `docker/main/docker-compose.yaml`.

### Почему именно `:-rob_box_dev`

- Согласовано с issue #2569 (предлагает именно это значение).
- Читаемо, явно "dev", одно слово — соответствует стилю остальных
  дефолтов в `docker/vision/docker-compose.yaml` (`${HAILO_ENABLED:-false}`,
  `${HEF_PATH:-}`, `${ROS_DISTRO:-humble}`, `${VISION_HAILO_TAG:-${IMAGE_TAG}}`).
- НЕ использовать `RBXU100001`: это production-ID конкретной машины
  (`docker/vision/.env`, `scripts/utils/start_*.sh`) — для dev-default
  неправильно: оператор скопирует dev-default на прод, namespace-ы
  пересекутся. `rob_box_dev` такое случайное копирование делает
  заметным (другое имя, явно "dev").
- НЕ использовать `robbox_01` из `.env.example`: `.env.example` —
  пример для ручной правки, его значение не должно быть дефолтом
  (иначе CI сольёт namespace с тем, кто забыл скопировать example).

### Почему inline `:-default`, а не другие варианты

| Альтернатива | Trade-off |
|---|---|
| Inline `:-<value>` в compose (выбрано) | Уже принято в этом же файле для `HAILO_ENABLED`, `HEF_PATH`, `ROS_DISTRO`, `SERVICE_IMAGE_PREFIX`. Минимальный diff, ноль новых абстракций. |
| Top-level `environment: { ROBOT_ID: ${ROBOT_ID:-rob_box_dev} }` в compose | Работает, но вводит второй канал рядом с per-service `environment:`. Лишняя сложность ради того же результата. |
| Shell wrapper / Makefile с `export ROBOT_ID=${ROBOT_ID:-rob_box_dev}` перед `up` | Утечка runtime-policy в shell; compose всё равно должен быть идемпотентен без обёрток. Не лечит, а маскирует. |
| Сделать ROBOT_ID обязательным в `.env.example` и провалидировать в CI | Правильно для production-policy, но не решает «свежий clone сломан из коробки». Дополняющая мера, не замена. |

### Почему один PR на vision + main, а не два

- Тот же шаблон, та же переменная, та же защита.
- Разделение = удвоение CI-прогонов, review-циклов и координации
  между воркерами (architect → backend → backend).
- Один и тот же дефолт (`rob_box_dev`) согласован в обоих файлах.
- Риск регрессии = ноль: фикс полностью additive, ни одного потребителя
  не меняет (поведение при `ROBOT_ID` в `.env` идентично).

## Scope фикса

- `docker/vision/docker-compose.yaml` — 9 вхождений (lines 34, 70, 103,
  211, 367, 438, 486, 538, 665).
- `docker/main/docker-compose.yaml` — 9 вхождений (lines 15, 41, 72, 100,
  189, 227, 256, 310, 378). Тот же баг.
- **Не трогаем**:
  - `docker/vision/.env.example` — уже корректно документирует
    `ROBOT_ID=robbox_01` (строки 30-37).
  - `docker/vision/.env` — production-конфиг, изменение только
    под реальный deploy.
  - `docker/vision/README.md` — проверить ссылку на `.env.example`,
    но правка только при фактическом отсутствии (raw-проверка
    обязательна).

## Что вне scope этой карточки

- **CI guard** `docker compose config -q` перед `up` — это **devops**.
  Архитектурно поддерживаю; реализация — отдельная карточка с
  assignee=devops. Не блокирует текущий фикс.
- **Privacy/namespace policy** для `rob_box_dev` в production —
  решается policy-level (`.env` обязателен на проде), не inline-дефолтом.
- **Audit других compose-файлов** (`docker/build/docker-compose.yaml`,
  `docker/monitoring/...`) на тот же паттерн — сделать grep по
  `\$\{[A-Z_]+\}` без `:-` после, **отдельной карточкой** (analyst
  или backend). В этой карточке — только `vision` + `main` (issue scope).

## Acceptance criteria (для backend-воркера)

1. Все 18 строк (9 vision + 9 main) заменены на `:-rob_box_dev`.
2. `docker compose -f docker/vision/docker-compose.yaml config -q` →
   exit 0 с пустым `.env` (тест: `mv docker/vision/.env /tmp/.env.bak &&
   docker compose -f docker/vision/docker-compose.yaml config -q`).
3. `docker compose -f docker/vision/docker-compose.yaml config -q` →
   exit 0 с реальным `.env` (regression: явный `ROBOT_ID` не
   перезаписан дефолтом).
4. `git grep -nE '\$\{ROBOT_ID\}(?!:)' docker/` → пусто (negative
   lookahead требует `rg` или `grep -P`; или эквивалент через
   `git grep -E '\$\{ROBOT_ID\}[^:]'` — должно быть пусто).
5. Raw-вывод pytest / `docker compose config` в issue comment.

## Trade-offs

- **Pro:** один фикс защищает 18 сервисов от "empty value for variable"
  в любом deployment без `.env` (CI, новый clone, локальная разработка).
- **Pro:** согласованность с уже-применённым паттерном в этом же
  compose-файле (`HAILO_ENABLED`, `HEF_PATH`, и т.д.).
- **Con:** оператор может случайно задеплоить с `rob_box_dev` на
  production. **Mitigation:** дефолт явно содержит `dev`; production
  должен иметь `.env` с реальным `ROBOT_ID` (это уже policy, не наш
  scope). CI guard `docker compose config -q` + обязательный
  `.env.production` — отдельная задача.

## Связанные

- issue #2569 (original report)
- ADR-0089 Phase 1, touchpoint vision-hailo
- ADR-0099 (vision-hailo binding install strategy)
- PR (после merge): будет ссылка
