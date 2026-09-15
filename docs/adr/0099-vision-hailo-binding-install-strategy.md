# ADR-0099 — vision-hailo binding install strategy (Phase 1 vs Phase 1.5+)

**Дата:** 2026-09-15
**Статус:** Accepted (Phase-1 scope, ADR-0089 §3 touchpoint #9 уточнение)
**Автор:** architect (kanban t_25cd6f13, issue #2499)
**Тип:** deployment / image-build decision (vision-hailo Dockerfile cleanup)
**Связанные ADR:** ADR-0089 (AI HAT+ Hailo-8 deployment), ADR-0018 (agent honesty culture — capability-honest)
**Разрешает issue:** #2499 (Dockerfile pip hailort vs hailo_platform)
**ADR-number:** 0099 (collision-fix, 2026-09-15: см. issue #2582 — сосед `0099-test-repo-root-discovery-pattern.md` получил тот же номер и переехал в ADR-0107; этот ADR первый по merge-time, коммит `d646c3c4` (PR #2517))

---

## 1. Проблема

Issue #2499 (ревью docker/vision за 2026-09-14) сигнализирует:

1. `docker/vision/vision-hailo/Dockerfile:50-55` содержит `pip install hailort==4.18.0` (или `hailort` без версии) как best-effort fallback.
2. `src/rob_box_perception/rob_box_perception/vision_hailo_loader.py:192,209,226` использует `# type: ignore[import-not-found]` для lazy-import `hailo_platform`.

Issue трактует это как MEDIUM-bug ("pip fallback ставит не то", "mypy не защищает") и предлагает:
> 1. Убрать `pip install hailort` (PyPI).
> 2. Использовать **только apt** путь: добавить Hailo apt-repo в Dockerfile.
> 3. Добавить `RUN python3 -c "import hailo_platform"` как **fatal** step build'а (без `|| true`).
> 4. Перенести smoke-test `hailortcli --version` ПОСЛЕ импорта `hailo_platform`.
> 5. Убрать `# type: ignore[import-not-found]` после того как binding ставится корректно.

**Эта issue содержит технически неверный анализ контракта Phase 1.** Реальный контракт (ADR-0089 + существующий код) подразумевает, что `hailo_platform` на Phase 1 **опциональный**. Цитаты:

- **ADR-0089 §9 (принято)**: acceptance Phase 1 — *"`hailortcli scan` на Vision Pi возвращает `hailo8` device"*. Про `import hailo_platform` — ни слова. Issue #2499 ссылается на "ADR §9: vision_hailo_node успешно импортирует hailo_platform и инициализирует VDevice" — **этого текста в §9 нет**, неверная цитата.
- **ADR-0089 §3 touchpoint #9** (line 151): *"`docker/vision/vision-hailo/Dockerfile` — NEW: hailort + tappas + **python binding** (ARM64 base)"*. Touchpoint формулирует артефакты, но **не задаёт** когда (Phase 1 vs 1.5) и через какой package source.
- **`docker/vision/scripts/vision-hailo/start_vision_hailo.sh:79-97`** (уже в репо):
  capability-honest degraded mode — если `HAILO_ENABLED=true`, а `import hailo_platform` падает, нода **продолжает работать в stub-режиме с логом**, не fatal.
- **`docker/vision/scripts/vision-hailo/hailo_smoke.py:63-69`** (уже в репо):
  Python binding (hailo_platform) → **warning, не fatal**. Phase 1 acceptance — это `hailortcli scan`, не import.
- **`src/rob_box_perception/test/unit/test_vision_hailo_phase15.py:292-296`** (уже в репо):
  `test_real_loader_init_failure_is_available_false` явно требует, что **без `hailo_platform` `is_available()` возвращает `False`** — это и есть контракт capability-honest.
- **`src/rob_box_perception/rob_box_perception/vision_hailo_node.py:62-77`** — best-effort pattern для `numpy` и `cv2` (через `# type: ignore[import-not-found]`). `hailo_platform` логически принадлежит той же категории.

### 1.1. Что означает "pip install hailort (PyPI) поставил не то"

Факт-проверка от 2026-09-15:

- `https://pypi.org/pypi/hailort/json` → **HTTP 404**. То есть пакета `hailort` на PyPI **сейчас нет** (community wrapper удалён / недоступен, либо никогда не публиковался под этим именем).
- Следовательно, **`pip install hailort` в Dockerfile всегда упадёт** с 404 / PackageNotFoundError. Это поглощается `2>/dev/null || true` конструкцией и превращается в WARNING в логе.
- Это **dead code**: попытка чего-то достичь, чего не существует.

### 1.2. Где брать official `hailo_platform` binding

Из [официального guide https://hailo.ai/developer-zone/] и обсуждений на community.hailo.ai (см. [community.hailo.ai/t/hailort-to-4-18-on-rpi5/2720]):

| Distribution | HailoRT (CLI + daemon) | `hailo_platform` Python binding |
|---|---|---|
| x86_64 dev | Hailo Dataflow Compiler SDK (.whl) | Hailo Dataflow Compiler SDK (.whl) |
| ARM64 (RPi5) | `hailort_X.Y.Z_arm64.deb` + `hailort-pcie-driver_X.Y.Z_all.deb` (Hailo Developer Zone) | `hailort-X.Y.Z-cpXYZ-cpXYZ-linux_aarch64.whl` (Hailo Developer Zone) |
| CI эмуляция | нет (нет железа) | нет |

Официальный `hailo_platform` **никогда не публиковался на PyPI**. Все community-врапперы — сторонние.

### 1.3. Что обязано и не должно быть в CI-сборке

CI (self-hosted runner `rob-box`):
- Если runner на RPi5 + AI HAT+ → apt + .whl из `/opt/rob_box/vendor` или из смонтированного cache — **Phase 1.5 problem, не Phase 1**.
- Если runner без железа (типичный случай на текущий момент) → **только stub**. `hailo_platform` НЕ нужен на CI-сборке, чтобы базовые unit-тесты прошли.

---

## 2. Решение (что делаем)

### 2.1. Phase 1 (current default) — `hailo_platform` опциональный

**Принцип**: capability-honest — если binding недоступен, нода продолжает в stub-режиме с понятным WARN в логе, не падает build, не падает runtime.

#### Изменения в `docker/vision/vision-hailo/Dockerfile`

1. **Убрать мёртвый `pip install hailort==4.18.0`**. PyPI возвращает 404, нет смысла делать сетевой round-trip и `|| true`-маскировать.
2. **Оставить `apt-get install -y --no-install-recommends libhailort-dev || true`** — C-заголовки, нужны для Phase 1.5/2 на Vision Pi (если потом кто-то будет собирать C-extension wheel локально). На CI: ignore через `|| true`.
3. **Добавить build-arg `HAILO_INSTALL_C_DEV={true,false}`** (default `true` — соответствует текущему поведению). Это даёт прозрачный opt-out для CI без Internet к Hailo apt-repo.
4. **Перенести `hailortcli --version` smoke-test в `start_vision_hailo.sh`** где он уже есть (lines 100-110). В Dockerfile — только base-install + явный комментарий-обоснование.

Пример финальной ступени Dockerfile (Phase 1):

```dockerfile
# --- HailoRT Python binding — Phase 1 опциональный (capability-honest, ADR-0099) ---
# Phase 1 default (CI / dev без железа): hailo_platform не нужен, нода работает в stub.
# install pip-fallback (hailort==4.18.0 от PyPI) был удалён: пакет отозван / 404.
# Real install — Phase 1.5: см. start_vision_hailo.sh (degraded mode) + ADR-0099 §2.2.
ARG HAILO_INSTALL_C_DEV=true
RUN if [ "${HAILO_INSTALL_C_DEV}" = "true" ]; then \
        apt-get update && \
        apt-get install -y --no-install-recommends libhailort-dev || \
            echo "WARN: libhailort-dev install skipped (apt-repo недоступен)"; \
        rm -rf /var/lib/apt/lists/*; \
    fi
```

#### Изменения в `src/rob_box_perception/rob_box_perception/vision_hailo_loader.py`

1. Lazy-import остаётся (это контракт — import только в `_ensure_initialized()`).
2. `# type: ignore[import-not-found]` остаётся, но с поясняющим комментарием (mypy `warn_unused_ignores=True` всё равно молчит потому что подавляем именно тот error, который наблюдаем).
3. Объяснить в module-docstring (line 19-21 уже есть упоминание "Phase 1.5 ability") что lazy-import — это design decision для capability-honest, а не workaround.

### 2.2. Phase 1.5 (отдельная карточка, scope = consumer-side + real inference)

Phase 1.5 НЕ в скоупе этого ADR. Когда дойдёт — формализуется в отдельной карточке с явным build-target:

- Build-arg `HAILO_INSTALL_BINDING={none,apt,whl}` (default `none` для CI).
- `apt` mode: добавить Hailo apt-repo в Dockerfile + `apt install hailort python3-hailo` (только на Vision Pi runner).
- `whl` mode: скопировать `hailort-X.Y.Z-cpXYZ-linux_aarch64.whl` в build-context через CI-secret + `pip install /wheels/*.whl`.
- Тогда же делается `RUN python3 -c "import hailo_platform"` как **fatal** step (под `HAILO_INSTALL_BINDING != none`), и ADR-0089 §9 acceptance расширяется проверкой `is_available() → True` для Phase 1.5.

---

## 3. Trade-offs

| Решение | Плюс | Минус |
|---|---|---|
| Убрать `pip install hailort` (Phase 1) | Чище Dockerfile, нет 404-warning'а в CI логе; экономит ~10-30 сек build time | Документировано явно что Phase 1 не ставит pip-binding (новые воркеры могут не понять почему) |
| Оставить `# type: ignore[import-not-found]` на lazy-import | mypy не падает; lazy-import — design для capability-honest | mypy не видит интерфейс `hailo_platform` (мы и не должны — это чужой пакет без stub) |
| Не добавлять `RUN python3 -c "import hailo_platform"` на Phase 1 build | CI не падает на runner без Hailo apt-repo; Phase 1 acceptance §9 остаётся `hailortcli scan` | Issue #2499 останется "open" если её читать буквально — нужны комментарии Шифу + этот ADR |
| Вынести `hailortcli --version` smoke в `start_vision_hailo.sh` | Dockerfile не делает лишних команд; smoke запускается только когда `HAILO_ENABLED=true` (т.е. когда есть смысл проверять) | Smoke не запускается на CI в Phase 1 (hailo apt-repo нет) — но это OK для Phase 1 acceptance |

---

## 4. Что НЕ делаем в этом ADR

- ❌ Не правим `vision_hailo_loader.py` business-logic (RealHEFLoader, NMS, post-process). Это Phase 1.5 (issue #2398) scope, ревьюер не открывает PR (kanban t_beba0869 — out-of-scope в task body).
- ❌ Не трогаем `start_vision_hailo.sh` capability-honest check (он уже корректен).
- ❌ Не трогаем ADR-0089 §9 acceptance (он отражает Phase 1 корректно).
- ❌ Не добавляем `Hailo apt-repo` в Phase 1 build — это Phase 1.5 deployment card на Vision Pi, не визион-Dockerfile.
- ❌ Не выкатываем `HAILO_INSTALL_BINDING={none,apt,whl}` в Phase 1 — избыточно для текущего default (none).

---

## 5. Acceptance criteria (Phase 1 cleanup)

- [ ] `docker/vision/vision-hailo/Dockerfile` не содержит `pip install hailort` (PyPI fallback удалён).
- [ ] `Dockerfile` имеет build-arg `HAILO_INSTALL_C_DEV={true,false}` (default `true`); `libhailort-dev` install — opt-in через него.
- [ ] `hailortcli --version` smoke-test **убран** из Dockerfile (уже есть в `start_vision_hailo.sh:100-110`).
- [ ] `vision_hailo_loader.py` имеет module-docstring с явной ссылкой на этот ADR (Phase 1 capability-honest, Phase 1.5 real binding — separate card).
- [ ] `pytest test_vision_hailo_node.py test_vision_hailo_phase15.py` — 21/21 зелёные (Phase 1 контракт сохраняется: `is_available() → False` без binding).
- [ ] CI-build vision-hailo на self-hosted `rob-box` runner — без WARNING в логе про PyPI 404.

---

## 6. Открытые вопросы (для шисюн / Шифу)

1. **Где fixed reference на официальный `hailo_platform` wheel**: на момент 2026-09 wheel рекомендуется через Hailo Developer Zone (требует регистрацию). Стоит ли настроить `opt/rob_box/vendor` cache в CI (как сделано для dpkg'ов `local_packages`)? Или рекомендовать apt как preferred path?
2. **Когда делать Phase 1.5 apt-repo integration**: отдельная карточка после merge этого ADR, или объединить с Phase 1.5 consumer-side (`mcp_server.py` stub-filter, issue #2406 parent)?
3. **Issue #2499 — закрывать или конвертировать в checklist**: раз предлагаемое "fatal import hailo_platform" противоречит Phase 1 контракту, либо закрываем issue с пояснением (этот ADR), либо оставляем open до Phase 1.5 карточки.

---

## 7. Change log

| Дата | Автор | Изменение |
|---|---|---|
| 2026-09-15 | architect (t_25cd6f13, issue #2499) | Initial ADR. Issue #2499 — false-positive (claim не соответствует §9 acceptance). Принято решение: Phase 1 cleanup (убираем dead `pip install hailort`, добавлем build-arg для C-dev headers), Phase 1.5 — отдельной карточкой. |

*ADR-0099 — формальное уточнение ADR-0089 §3 touchpoint #9. Все ссылки и контракты проверены реальным grep + чтением файлов, не выдуманы.*
