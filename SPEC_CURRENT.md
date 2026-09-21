# SPEC_CURRENT — Текущее состояние и ближайшие шаги

> **Версия**: 2.0
> **Дата**: 2026-07-27
> **Ветка**: `feature/harness-p0-foundation`
> **PR**: https://github.com/krikz/rob_box_project/pull/907

---

## 1. Где мы сейчас

### 1.1 Завершённые этапы

| Этап | Результат | Статус |
|---|---|---|
| **ADR-0001** — Архитектура харнесов | 956 строк: `Harness[StateT]` ABC, 5 портов, state-store, lifecycle hooks для DialogNode / PersistentNode / TelegramNode | ✅ Accepted |
| **Harness Framework P0** | `rob_box_harness` (P0): `Harness[StateT]`, lifecycle, ports (`LLMProvider` / `ToolProvider` / `MemoryStore` / `SideEffectBus` / `Transport` + `Clock`), `HarnessConfig` (YAML+ENV), `HarnessRegistry` + `run_harness()` entry-point, dummy `echo`/`upper` харнесы, 88 тестов / 90% coverage / mypy strict-clean | ✅ **Done** |
| **MiniMax Provider в Harness** | `rob_box_harness.providers.minimax.MiniMaxProvider` (ADR-0001 M1–M10): env-auth (`MINIMAX_API_KEY`, YAML-литералы запрещены), `chat(messages, **kwargs)` shortcut, retry с экспоненциальным backoff (только `RateLimitError` / `TimeoutError`), 56 тестов / 95% coverage / mypy strict-clean | ✅ **Done** |
| **MiniMax LLM-провайдер в `rob_box_llm`** | `MiniMaxProvider` (PR #907): OpenAI-compatible адаптер, `MiniMax-M3` (text+vision+tools), `base_resp` envelope, key redaction, capabilities API | ✅ Done |
| **MiniMax TTS** | `MiniMaxTTSProvider` (ADR-0007), пайплайн до ROS-топика `/voice/audio/speech` | ✅ Done |
| **PR #907** — Код ревью | Все частные ревью (backend, frontend, security, architect) проведены | ✅ Done |

| **Phase 6: Harness P0 Finalization** | Documentation consolidated (no duplicates, adr/ canonical), Docker integration, harness adapters (Dialog/Persistent/Telegram), port implementations (ROS2Transport/SQLiteVoiceMemory), test coverage gaps closed, PR #907 audit complete | ✅ Done |

### 1.2 Что уже настроено

- **Ветка** `feature/harness-p0-foundation` — P0 завершён, готов к мержу в main (PR #907)

---

## 2. Диаграмма компонентов (P0 — landed)

```mermaid
graph TB
    subgraph "Adapters (P1 — Phase 6 implemented)"
        DA[DialogHarness<br/>voice input]
        PA[PersistentHarness<br/>audio/stt/tts/sound/led/cmd]
        TA[TelegramHarness<br/>tg bot]
    end

    subgraph "Harness Layer (P0 — ✅ landed)"
        H[Harness[StateT]<br/>+ LifecycleHooks]
        LF[Lifecycle<br/>init / run / teardown]
        REG[HarnessRegistry]
        RUN[run_harness name, input, config]
    end

    subgraph "Ports (P0 — ✅ landed)"
        LLMP[LLMProvider port]
        TP[ToolProvider port]
        MS[MemoryStore port]
        SEB[SideEffectBus port]
        TR[Transport port]
        CK[Clock port]
    end

    subgraph "Built-in Harnesses (P0 — ✅ landed)"
        ECHO[EchoHarness]
        UPPER[UpperHarness]
        DUMMY_PROV[DummyLLMProvider]
    end

    subgraph "rob_box_harness.providers (P0 — ✅ landed)"
        MM_PROV[MiniMaxProvider<br/>+ RetryPolicy<br/>+ chat]
    end

    subgraph "rob_box_llm (PR #907 — ✅ landed)"
        DS_PROV[DeepSeekProvider]
        MIMO_PROV[MiMoProvider]
        MM_UP[MiniMaxProvider<br/>upstream]
        TTS_PROV[MiniMaxTTSProvider]
    end

    subgraph "External"
        MM_API[(MiniMax API<br/>api.minimax.io)]
        DS_API[(DeepSeek API)]
        MIMO_API[(MiMo API)]
        ROS2[(ROS2 topics)]
    end

    DA --> H
    PA --> H
    TA --> H

    ECHO -.extends.-> H
    UPPER -.extends.-> H
    H --> LF
    LF --> REG
    REG --> RUN

    H --> LLMP
    H --> TP
    H --> MS
    H --> SEB
    H --> TR
    H --> CK

    LLMP -.binds.-> MM_PROV
    LLMP -.binds.-> DS_PROV
    LLMP -.binds.-> MIMO_PROV
    LLMP -.binds.-> DUMMY_PROV

    MM_PROV -.delegates HTTP.-> MM_UP
    MM_UP --> MM_API
    DS_PROV --> DS_API
    MIMO_PROV --> MIMO_API

    SEB -.publishes.-> ROS2
    TR -.subscribes.-> ROS2
    TTS_PROV -.publishes audio.-> ROS2
```

**Что в P0 (✅ landed):** `rob_box_harness` (фреймворк с `Harness[StateT]`, lifecycle, 5 портов, `HarnessRegistry` + `HarnessFactory` + `run_harness()`, dummy-харнесы `EchoHarness`/`UpperHarness`, `DummyLLMProvider`) + `rob_box_harness.providers.minimax.MiniMaxProvider` (env-auth, `chat()`-shortcut, retry).
**Что в P1 (✅ Phase 6 implemented):** три реальных харнеса (`DialogHarness` / `PersistentHarness` / `TelegramHarness`, ADR-0001 §2.7), `ROS2Transport`, `SQLiteVoiceMemory`.

---

## 3. Что нужно сделать (ROADMAP — ближайшие шаги)

### Этап A: PR #907 review → main (как было)

| # | Задача | Исполнитель | Критерий готовности |
|---|---|---|---|
| A1 | Опубликовать сводный комментарий в PR #907 | `pr-reviewer` | Комментарий виден в PR #907 | ✅ Done |
| A2 | Выставить review state (APPROVE / REQUEST_CHANGES) | `pr-reviewer` | Review state установлен | ✅ Done |
| A3 | `git checkout main && git merge feature/harness-p0-foundation` | Человек | Fast-forward или конфликты разрешены | ⏳ Pending |
| A4 | `git push origin main` | Человек | main обновлён | ⏳ Pending |

### Этап B: P1 — Реальные харнесы (USE ADR-0001 §2.7)

| # | Задача | Исполнитель | Критерий | ADR |
|---|---|---|---|---|
| B1 | `DialogHarness` поверх `DialogueNode` | `backend` | Тесты >= 80%, `run_harness("dialog", input)` end-to-end | ✅ Done | ADR-0001 §2.7.1 — снято по ADR-0051 §3.2 (каркас Harness удалён, issue #1985, PR #2075); трассировка Kanban `t_f83e9cf9` |
| B2 | `PersistentHarness` (audio/stt/tts/sound/led/cmd) | `backend` | Один харнес драйвит все 6 persistent-нод | ✅ Done | ADR-0001 §2.7.2 — снято по ADR-0051 §3.2 (каркас Harness удалён, issue #1985, PR #2075); трассировка Kanban `t_f83e9cf9` |
| B3 | `TelegramHarness` поверх `TelegramNode` | `backend` | Тесты >= 50%, мост к voice через skill | ✅ Done | ADR-0001 §2.7.3 — снято по ADR-0051 §3.2 (каркас Harness удалён, issue #1985, PR #2075); трассировка Kanban `t_f83e9cf9` |
| B4 | `ROS2Transport` (реальный) | `backend` | `Transport` interface реализован для ROS2-топиков | ✅ Done | ADR-0001 §2.4.5 — снято по ADR-0051 §3.2 (каркас Harness удалён, issue #1985, PR #2075); трассировка Kanban `t_f83e9cf9` |
| B5 | `SQLiteVoiceMemory` / `RedisStore` | `backend` | `MemoryStore` interface для persistent history | ✅ Done | ADR-0001 §2.4.3 |

### Этап C: Покрытие тестами (ADR-0001 §5)

| # | Задача | Текущее | Цель |
|---|---|---|---|
| C1 | `DialogueNode` test coverage | 9% | 80%+ | ✅ Done |
| C2 | `TelegramNode` test coverage | 0% | 50%+ | ✅ Done |
| C3 | MCP-инструменты (для `ToolProvider`) | — | 70%+ | ✅ Done |

### Этап D: Документация (как было, после P0)

| # | Задача | Исполнитель | Файл |
|---|---|---|---|
| D1 | **Harness quickstart** — как создать свой харнес | `techwriter` | `docs/guides/harness-quickstart.md` | ✅ Done |
| D2 | **Обновить SPEC_CURRENT** — пометить P0 как Done | `techwriter` | `SPEC_CURRENT.md` | ✅ Done |
| D3 | **Обновить ROADMAP** — стадии 2 / 5 | `techwriter` | `ROADMAP.md` | ✅ Done |

---

## 4. Ограничения (что нельзя менять)

- **Не трогать `main`** до завершения review PR #907 и ручного подтверждения
- **Не менять ADR-0001** — он уже прошёл полный цикл ревью (MADR, Accepted)
- **Не дублировать задачи** — все новые задачи создавать через GitHub Issues с соответствующими labels
- **API-ключи — только через env** (`MINIMAX_API_KEY`); YAML-литералы запрещены (ADR-0001 §2.5.3, M7)

---

## 5. Как работать с этим документом

Этот документ — **источник истины** для всех разработчиков на ближайшие задачи. Work is tracked via GitHub Issues. Use `gh issue create` with appropriate labels:

```bash
gh issue create --title "[ID] description" \
  --label "type:functional,priority:high,source:gsd" \
  --repo krikz/rob_box_project
```

Разработчик обязан:
1. Прочитать `SPEC_CURRENT.md` и `docs/adr/0001-harness-architecture.md`
2. Следовать ROADMAP (этап B — P1 харнесы)
3. Соблюдать ограничения (раздел 4)
4. При завершении закрыть issue с summary

---

## 6. Связанные документы

- [`docs/adr/0001-harness-architecture.md`](docs/adr/0001-harness-architecture.md) — ADR-0001 (MADR, Accepted)
- [`docs/adr/0002-minimax-provider.md`](docs/adr/0002-minimax-provider.md) — ADR-0002 (MiniMax LLM)
- [`docs/adr/0007-minimax-tts-integration-final.md`](docs/adr/0007-minimax-tts-integration-final.md) — ADR-0007 (MiniMax TTS)
- [`src/rob_box_harness/README.md`](src/rob_box_harness/README.md) — Harness Framework API
- [`src/rob_box_harness/rob_box_harness/providers/README.md`](src/rob_box_harness/rob_box_harness/providers/README.md) — MiniMax LLM provider API
- [`docs/guides/harness-quickstart.md`](docs/guides/harness-quickstart.md) — how-to: создать свой харнес
- [`docs/guides/MINIMAX.md`](docs/guides/MINIMAX.md) — MiniMax LLM user-guide
- [`docs/guides/MINIMAX_TTS_GETTING_STARTED.md`](docs/guides/MINIMAX_TTS_GETTING_STARTED.md) — MiniMax TTS getting-started
- [`docs/architecture/minimax-provider.md`](docs/architecture/minimax-provider.md) — архитектурный обзор MiniMax

---

## 7. Cold-start / deploy: Vision Pi при недоступных образах (issue #2610, ADR-0111)

**Контекст.** До фикса `robbox-vision.service` после ребута Vision Pi **не поднимал ни одного контейнера**: `docker compose up -d` шёл в `10.1.1.249:5000` (katana, build-host, см. `.env`), и если katana offline — pull fail `dial tcp 10.1.1.249:5000: connect: no route to host` → systemd exit 1. Все 11 образов были закэшированы на Pi — фиксу сеть не нужна. Подробности и raw-evidence — [`docs/architecture/diagnostics/2026-09-15-robbox-vision-pull-failure.md`](docs/architecture/diagnostics/2026-09-15-robbox-vision-pull-failure.md).

**Статус (обновлено 2026-09-21):** фиксы смёржены в `develop`. Пункт
ADR-0111 §2.1 про bake Renardo-сэмплов в `voice-base` **не реализован так,
как описан ниже в истории PR** — вместо bake-in-образ выбран другой
механизм: [ADR-0125](docs/adr/0125-resource-pack-host-delivery-seam.md)
(Ресурсный пак) и [ADR-0126](docs/adr/0126-renardo-samples-host-delivery.md)
(уточняет ADR-0111 §2.1). Образ `voice-resources` и init-контейнер
`voice-resources-init` удалены целиком, `profiles: [init]` в compose больше
нет. Сэмплы и STT/TTS-модели (Vosk, Silero) кладёт на хост Vision Pi
(`/opt/rob_box/samples`, `/opt/rob_box/models`) шаг деплоя, контейнеры видят
их bind-mount'ом. Проверено на роботе 21.09.2026 (raw: `docker inspect`
bind-mount, `ls` моделей/сэмплов внутри контейнера, прогрев Vosk/Silero).

- [PR #2619](https://github.com/krikz/rob_box_project/pull/2619) — ADR-0111 (merged).
- [PR #2617](https://github.com/krikz/rob_box_project/pull/2617) — диагностика `2026-09-15-robbox-vision-pull-failure.md`.
- [PR #2634](https://github.com/krikz/rob_box_project/pull/2634) — `docker/vision/docker-compose.yaml`: `pull_policy: missing` ×17 (исходная версия компоуза с `voice-resources-init` под `profiles:[init]` позже заменена по ADR-0126).
- [PR #2635](https://github.com/krikz/rob_box_project/pull/2635) — `scripts/setup/setup_vision_pi.sh`: systemd-юнит `robbox-vision.service` с `Restart=on-failure` + `StartLimitBurst=5` + `StartLimitIntervalSec=600` + `--pull never` best-effort.

**Принятое поведение** (текущее):

- Vision Pi **поднимает стек частично**, а не валится целиком, если один из образов недоступен (registry offline, отсутствует тег, переключение DNS).
- `Restart=on-failure` + `RestartSec=60` + `StartLimitBurst=5` + `StartLimitIntervalSec=600` в `robbox-vision.service` — systemd сам поднимет юнит после транзитных сбоев.
- `pull_policy: missing` для всех image-based сервисов + `--pull never` через override (или `--ignore-pull-failures` в deploy-шаге) — compose не пытается рефетчить то, что уже локально.
- Никакого init-профиля для сэмплов/моделей больше нет: Ресурсный пак кладёт их на хост ДО `docker compose up` (ADR-0125/ADR-0126), compose про registry для этих ресурсов не знает.

### Что делать, если образ недоступен

1. **Определить, какие сервисы не стартовали:**

    ```bash
    docker compose -f ~/rob_box_project/docker/vision/docker-compose.yaml ps --format json \
      | jq -r '.[] | select(.State != "running") | "\(.Name)\t\(.State)\t\(.ExitCode // "-")\t\(.Error // "-")"'
    ```

2. **Посмотреть логи конкретного сервиса:**

    ```bash
    docker compose logs --tail=200 <service-name>
    ```

3. **Проверить статус systemd-юнита:**

    ```bash
    sudo systemctl status robbox-vision --no-pager -l
    ```

4. **Пополнить Ресурсный пак** (если сэмплы/модели на хосте не на месте — сеть нужна ЕМУ, не `docker compose`):

    ```bash
    sudo bash docker/vision/scripts/resource_pack/apply_resource_pack.sh --dry-run  # план, без сети
    sudo bash docker/vision/scripts/resource_pack/apply_resource_pack.sh           # применить
    ```

5. **Полный cold-start с гарантией использования локального кэша:**

    ```bash
    docker compose --pull never up -d
    ```

### Сценарии «один образ/ресурс недоступен»

| Сценарий | Что происходит | Что делать |
|----------|---------------|-----------|
| `/opt/rob_box/samples` или `/opt/rob_box/models` пусты/отсутствуют | `supercollider`/`voice-assistant` поднимаются в `synth-only mode` (без музыки), STT/TTS без моделей не стартуют | `sudo bash docker/vision/scripts/resource_pack/apply_resource_pack.sh` когда сеть доступна |
| Один из образов не скачался | Остальные сервисы работают; systemd рестартует юнит, повторный `pull` сделает best-effort | Дождаться registry или поднять руками: `docker compose --pull never up -d <service>` |
| Нужно обновить сэмплы Renardo | Ресурсный пак идемпотентен (sha256/marker) — повторный прогон no-op, если манифест не менялся | `sudo bash docker/vision/scripts/resource_pack/apply_resource_pack.sh --only renardo-samples` |

### Профили compose (Vision Pi)

- **`default`** (без `--profile`): все основные сервисы. Используется в `robbox-vision.service` для повседневного старта. Init-профиля для сэмплов/моделей больше нет — их кладёт на хост Ресурсный пак ДО `docker compose up` (ADR-0125/ADR-0126).
- **`monitoring`** (без изменений): `cadvisor-vision`, `promtail-vision`.
- **`ai`** (без изменений): `ollama`.

### Главный инвариант

Стек **всегда** поднимает базовые сервисы без обращения к registry за сэмплами/моделями — они уже на хосте до `docker compose up` (ADR-0125, ADR-0126, уточняют [ADR-0111 §2.1](docs/adr/0111-voice-resources-image-sourcing.md#21-voice-resources-больше-не-отдельный-образ)). Образ `voice-resources` и init-контейнер удалены целиком. Defense-in-depth: `--ignore-pull-failures` (`.github/workflows/L-Deploy and Verify.yml:381`) + `pull_policy: missing` (compose) + `Restart=on-failure` (systemd) — независимые слои защиты от каскадного краша.

### Какие сервисы считаются критичными, а какие — опциональными

| Категория | Сервисы | Что произойдёт, если образ недоступен |
|-----------|---------|---------------------------------------|
| **Критичные** (без них стек бесполезен) | `ros2_bridge`, `zenoh-router`, `hailo`, `vision_node` (лицевая/person), `avatar-arbiter` | Робот «глух и слеп» — голос и зрение не работают. Но **стек всё равно поднимется**, systemd рестартует и логи покажут причину. |
| **Опциональные** (можно без них) | `supercollider` (без сэмплов на хосте → `synth-only mode`), `monitoring`-профиль, `ai`-профиль | `voice-assistant` стартует в `synth-only mode` (без музыки), `cadvisor`/`promtail`/`ollama` просто не поднимаются, остальное работает. |

### Обоснование выбора (ADR-0111 §4, механика доставки позже уточнена ADR-0125/ADR-0126)

Рассматривались альтернативы: (a) поднять registry на Vision Pi — отклонено (attack surface + disk usage + operational overhead, [ADR-0111 §4.A](docs/adr/0111-voice-resources-image-sourcing.md#4a-поднять-registry-на-vision-pi-вариант-c-из-body-карточки)); (b) GHCR + fallback на katana — отклонено (production не должен зависеть от dev-окружения, [ADR-0111 §4.B](docs/adr/0111-voice-resources-image-sourcing.md#4b-ghcr-по-умолчанию--опциональный-fallback-на-katana)); (c) оставить `voice-resources-init` + `pull_policy: missing` + cached bundle — отклонено (не убирает архитектурный SPOF, не решает first-boot, [ADR-0111 §4.C](docs/adr/0111-voice-resources-image-sourcing.md#4c-оставить-voice-resources-init--pull_policy-missing--cached-bundle-на-pi)); (d) bake в supercollider — отклонено (исторически только runtime scsynth, не должен знать про Renardo pipeline, [ADR-0111 §4.D](docs/adr/0111-voice-resources-image-sourcing.md#4d-bake-сэмплы-в-supercollider-образ)).

**Принятое решение (из тела карточки t_ca7fa165, шаг 3 — «вынесение/не вынесение данных из образа»):** [ADR-0111 §2.1](docs/adr/0111-voice-resources-image-sourcing.md#21-voice-resources-больше-не-отдельный-образ) зафиксировал **ликвидацию `voice-resources` как image-based init-сервиса**. Вариант "bake в `voice-base`" из §2.4, описанный там как принятый, на практике заменён другим механизмом — Ресурсным паком на хосте ([ADR-0125](docs/adr/0125-resource-pack-host-delivery-seam.md), [ADR-0126](docs/adr/0126-renardo-samples-host-delivery.md)): сэмплы и модели не запекаются ни в один образ, их кладёт на хост шаг деплоя, контейнеры получают bind-mount'ом.

### Референсы

- Issue: [#2610](https://github.com/krikz/rob_box_project/issues/2610) (Vision Pi не поднимается без katana)
- ADR: [`docs/adr/0111-voice-resources-image-sourcing.md`](docs/adr/0111-voice-resources-image-sourcing.md)
- Диагностика: [`docs/architecture/diagnostics/2026-09-15-robbox-vision-pull-failure.md`](docs/architecture/diagnostics/2026-09-15-robbox-vision-pull-failure.md)
- Реализация: PR #2617 (диагностика), PR #2634 (compose), PR #2635 (systemd unit), PR #2619 (ADR-0111)
- Финальная сборка всего в один PR: PR (планируется, ветка `wt/t_ef836b9c`)
- Связанные: issue #2095 / retro `t_d01fe536+t_9d35468d` (race-condition в init), ADR-0094 (`.image-versions.*` SHA-tag push), ADR-0018 (честность)
