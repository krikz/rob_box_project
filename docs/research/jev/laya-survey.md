# Survey: Laya (`convaiinnovations/laya`) как self-hosted провайдер (issue #3084)

> **Ограничение исследования.** `huggingface.co` из среды недоступен (egress 403), поэтому model card напрямую не прочитана. Все факты ниже взяты из исходников, которые на него ссылаются и пинуют его:
> - `ollaya-dev/ollaya` `f42c89e` — Rust-демон, отдаёт Laya через TypeSafe-совместимый API;
> - `r33drichards/laya-vision` `d4075b0` — research-форк Laya;
> - `bladedevoff/stuntd` `102a631`.
>
> Ссылки — в формате `файл:строка` этих репозиториев. **Ни одного замера Laya на Pi или на katana не сделано**: веса скачать нельзя (HF заблокирован), Pi из среды недоступен.

## 1. Что это за модель

| Параметр | Значение | Источник |
|---|---|---|
| Архитектура | encoder (ModernBERT / mmBERT) + decision head: `type_emb` (choice / score / noul), 2 слоя TransformerEncoder, scorer на позиции `[MASK]` каждого варианта + `act_head` («act / escalate») | ollaya `convert/ollaya_convert/arch.py:164-189`; laya-vision `laya/common.py:143-179` |
| `laya:en` | ModernBERT-large, **421M** параметров, контекст 512 токенов | ollaya `site/content/library/laya.md:8`, `catalog.py:115-116` |
| `laya:multilingual` | mmBERT-base, **322M**, 1024 токена, «100+» языков | `laya.md:9`, `catalog.py:117-118` |
| `laya:typed-decisions` | ModernBERT-large, 421M, 1024 токена, только английский, дообучена | `laya.md:10`, `catalog.py:119-121` |
| Веса на диске | en 842.6 MB, multilingual 643.8 MB (≈2 байта на параметр → fp16/bf16; это мой вывод, в источниках не сказано) | ollaya `registry/v2/library/laya/manifests/en:38`, `multilingual:38` |
| Примитивы | choice, score, noul — все три; noul = P(true) по двум вариантам `[false, true]` | laya-vision `common.py:11,21-32`; ollaya `docs/api.md:400-402` |
| Лимиты | 1–256 вопросов, 2–255 вариантов, 2–10 уровней score; вариантов реально ≈125 (en) / ≈250 (multilingual) при `head_max_len=192` | ollaya `docs/api.md:309,318-320,350-351` |
| Русский | явного заявления **нет**. Кириллицу роутер `laya` отправляет в `laya:multilingual`; точность на русском нигде не приведена | ollaya `docs/api.md:1308`, `crates/ollaya-lang/src/script.rs:21` |
| Лицензия | веса **Apache-2.0**; код Ollaya, stuntd и laya-vision — Apache-2.0 | `laya.md:1,117`; `ollaya/LICENSE`; `stuntd/pyproject.toml:11` |

Лицензия для нашего применения подходит: коммерческое использование разрешено.

## 2. Железо

| Вопрос | Ответ | Источник |
|---|---|---|
| CPU-only? | **да**: ONNX Runtime, fp32 на CPU | ollaya `README.md:73-75` |
| ARM64 (Pi 5)? | есть релизный `ollaya-linux-arm64.tar.zst` («CPU only») и multi-arch Docker `ghcr.io/ollaya-dev/ollaya` (amd64 + arm64, база debian:trixie-slim) | `docs/distribution.md:29`; `release.yml:35-50,264-268`; `Dockerfile:79-103` |
| glibc | нативный бинарь требует **glibc ≥ 2.38** | `scripts/install.sh:72-77`; `docs/distribution.md:201,748` |
| Квантизация | только fp16 (GPU) и fp32 (CPU). INT8 / INT4 / GGUF-графов Laya **нет** | `validate.rs:570-573`; `laya.md:12`; `README.md:57-58` |
| GPU | CUDA-сборки только для x86-64 (CUDA 13: драйвер R580+, sm_75–sm_120; CUDA 12: R525+) | `docs/distribution.md:117-118,202` |
| VRAM | `laya:en` fp16 — 814 MiB, замер на RTX 4090 | `docs/distribution.md:390` |
| RAM на CPU | **не найдено** | — |
| Латентность, GPU | RTX 4090 fp16, 5 вопросов: 9.6 мс (en), 8.1 мс (multilingual). Tesla T4: en 39.5 мс на 1 вопрос, 158.6 мс на 10 | `laya.md:93,95-100` |
| Латентность, CPU | только качественно: «a few hundred ms on a CPU», модель CPU не указана. **Цифр для ARM нет** | `skills/ollaya-decisions/SKILL.md:13` |

### Сверка с нашим deployment

Наше железо (`docs/architecture/SYSTEM_OVERVIEW.md:117-118`):

- **Main Pi** — Raspberry Pi 5, 4× Cortex-A76 @ 2.4 GHz, 16 GB. Занят SLAM, Nav2 и LSLIDAR.
- **Vision Pi** — Raspberry Pi 5, 4× Cortex-A76, 8 GB. Занят vision и голосом.
- **katana** — build-host и registry, доступен не всегда: в `docs/` зафиксированы эпизоды «katana выключен».

| Хост | Вердикт по документам | Почему |
|---|---|---|
| Main Pi (16 GB) | **Технически возможно**, не рекомендуется без замера | Memory: fp32 421M ≈ 1.7 GB только на веса (оценка 421M × 4 байта). CPU: 4 ядра уже загружены Nav2 и SLAM, а инференс энкодера на CPU отнимет ядра у навигации |
| Vision Pi (8 GB) | **Возможно с риском** | Memory: `laya:multilingual` (322M) в fp32 ≈ 1.3 GB — оценка. Нагрузка: делит CPU с vision и голосом |
| katana | **Не опора** | Нет гарантии доступности. Если там есть CUDA GPU (в репо не нашёл спецификации katana), это лучший вариант по латентности, но только как необязательный провайдер |

По контейнерам:

- Голосовой контейнер собран на ROS Humble (`docker/vision/voice_assistant/Dockerfile:12`, база `voice-base-humble`). Humble живёт на Ubuntu 22.04, где glibc 2.35 < 2.38 — это мой вывод по версии Humble, а не проверка образа. Значит, нативный бинарь Ollaya внутри нашего контейнера не запустится.
- Реальный путь — отдельный контейнер `ghcr.io/ollaya-dev/ollaya` (arm64, trixie) на `127.0.0.1:11435`. Наш `LayaProvider` ходит в него по HTTP.

## 3. Качество и калибровка (по данным авторов, не нашим)

- Базовые чекпоинты на typed-decisions zero-shot близки к случайному угадыванию: 0.362. У `laya:typed-decisions` — 0.766, у Jev 1.13 — 0.727 (`laya.md:103,111`).
- Больше ~20 вариантов — слабое место: Banking77 0.425 против 0.870 у Jev (`laya.md:112`).
- **Сырые чекпоинты переуверены.** Для `laya:multilingual` температуры не подогнаны; авторы прямо говорят подгонять их на своих данных (`laya.md:113`). ECE 0.081 после temperature fitting — только для английского (`laya.md:99`).
- fp16 и fp32 совпадают на 99.1–99.6% решений (`api.md:414-416`).

Вывод: **для русских фраз робота калибровки нет**. Это прямо подтверждает правило из комментария Шифу: Laya не допускается к safety-critical решениям. Оно зашито в `routing.py` (`FORBIDDEN_FOR_SAFETY`), и тест проверяет, что такой конфиг не загружается.

## 4. API — почему `LayaProvider` = тот же HTTP-клиент

Ollaya отдаёт:

- `POST /v1/systemone` и `GET /v1/models` в формате TypeSafe;
- `GET /` для liveness — всегда 200;
- порт по умолчанию `127.0.0.1:11435`.

Источники: `crates/ollaya-server/src/http.rs:94-108`, `docs/api.md:447-455,1548`.

Ответ `/v1/*` содержит ровно `model`, `answers`, `usage` (`api.md:1176-1193`). Есть одна ловушка: без явного `model=laya` клиент шлёт `jev-latest` и получает `404 MODEL_NOT_FOUND` (`api.md:1167-1170`). Поэтому у нас `LAYA_MODEL` по умолчанию `laya`, а 404 маппится в `ProviderConfigError`.

`STATE_TRUNCATED` на `/v1/*` приходит как 422, а не как тихая обрезка (`api.md:354-358`). Мы тоже считаем это ошибкой и уходим в fallback.

## 5. Сравнительная таблица (заполнена фактами; где фактов нет, так и написано)

| Property | Jev (TypeSafe) | Laya (через Ollaya) | Deterministic |
|---|---|---|---|
| Deployment | remote API `api.typesafe.ai` | self-hosted контейнер, arm64 и amd64 | in-process код |
| Latency | чужие замеры: p50 272–371 мс, p95 533–830 мс, холодный старт ≈ 2.6 с (jev-guard, jev-engineering). **Наш путь не замерен** | GPU 8–40 мс (вендор); CPU «сотни мс», без цифр. **На Pi 5 не замерено** | < 1 мс (in-process; отдельно не мерил) |
| Cost | $0.042 / 1M input tokens (vendor); jev-engineering насчитал $0.0000189 за вызов | $0 + CPU робота | $0 |
| Network | обязателен | нет (loopback) | нет |
| Privacy | state уходит вендору | on-prem | on-prem |
| Калибровка | чужие данные противоречивы: 92.2% / ECE 0.041 (Nautilus), но «highest-confidence bucket least accurate» (jev-guard) | переуверена без подгонки; для русского данных нет | N/A |
| Failure mode | 401/429/5xx, таймаут, дрейф между версиями | OOM, медленный CPU-инференс, нет модели (404), 422 truncation | детерминирован |
| Роль у нас | quality-critical; safety — только эскалация | latency-critical, **не safety** | всегда последний шаг и safety-путь |

## 6. Что нужно замерить (не сделано — нет доступа к Pi и HF)

```bash
# на Vision Pi (arm64), отдельный контейнер
docker run -d --name ollaya -p 127.0.0.1:11435:11435 ghcr.io/ollaya-dev/ollaya:latest
docker exec ollaya ollaya pull laya:multilingual     # веса тянутся с HF — нужен доступ
docker stats ollaya --no-stream                      # RAM после загрузки
LAYA_BASE_URL=http://127.0.0.1:11435 LAYA_MODEL=laya:multilingual \
  PYTHONPATH=src/rob_box_harness:src/rob_box_llm:src/rob_box_core \
  python scripts/research/jev_eval.py --provider laya > laya_vision_pi.json
# → summary.latency_ms.laya.p50/p95, summary.provider_outcomes; повторить при работающем голосе/vision
```

Критерий go для Laya на Pi (предложение): p95 ≤ 150 мс под рабочей нагрузкой **и** отсутствие деградации realtime-путей (Nav2 / voice) в тот же прогон. Иначе Laya остаётся только кандидатом на katana или отпадает.
