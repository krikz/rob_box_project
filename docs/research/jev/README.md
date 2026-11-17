# Jev / Laya как decision layer — исследование и spike (issue #3084)

Состав исследования:

- [`awesome-jev-survey.md`](awesome-jev-survey.md) — pre-work: survey экосистемы, паттерны, antipatterns, анализ 5 точек интеграции;
- [`laya-survey.md`](laya-survey.md) — Laya: железо, лицензия, self-host на Pi, сравнительная таблица;
- [`scenarios/acceptance.yaml`](scenarios/acceptance.yaml) — размеченные офлайн-сценарии;
- код: `src/rob_box_harness/rob_box_harness/decision/`;
- тесты: `src/rob_box_harness/test/test_decision_*.py`;
- прогон оценки: `scripts/research/jev_eval.py`.

**Статус: research spike.** Модуль **не подключён** ни к одному продовому пути робота. Его нет в `agent_core`, `AcceptanceGate`, `dialogue_node` и `tts_node`. Без `JEV_API_KEY` / `LAYA_BASE_URL` провайдеры просто не создаются.

## Архитектура

```
caller (свой класс решения + свой детерминированный fallback)
   │
   ▼
DecisionOrchestrator.decide(class, state, questions, fallback=...)
   │  routing[class] = [jev | laya ...], один deadline_ms на цепочку, min_confidence
   │  для каждого провайдера: circuit breaker → wait_for(остаток дедлайна) → validate → порог
   ▼
первый валидный уверенный ответ   ИЛИ   fallback() вызывающего кода (всегда)
```

| Модуль | Роль |
|---|---|
| `types.py` | `ChoiceQuestion` / `ScoreQuestion` / `NoulQuestion`, ответы, `DecisionClass`, иерархия ошибок, `validate_answers` |
| `providers.py` | протокол `DecisionProvider`, `DeterministicProvider`, `FakeDecisionProvider` |
| `systemone_http.py` | `SystemOneHttpProvider`: Jev (`api.typesafe.ai`) и Laya (Ollaya `127.0.0.1:11435`) — один клиент; `jev_from_env`, `laya_from_env` |
| `routing.py` | YAML / mapping → цепочки по классам; `laya` запрещена в `safety_critical`; `deterministic` только последним; потолок дедлайна 2 с |
| `health.py` | `CircuitBreaker` (closed / open / half-open), `DecisionMetrics` (счётчики `<provider>_{success,timeout,error,invalid,low_confidence,circuit_open,unconfigured}`, `fallback_used`, p50/p95) |
| `orchestrator.py` | fail-to-local-fallback, **0 ретраев**, переключение `set_routing` / `register` / `unregister` без рестарта |
| `acceptance_escalation.py` | acceptance PoC: модель может только поднять решение до `require`; `EMERGENCY_TOOLS` сигнал не читают |
| `scheduler_shadow.py` | scheduler PoC в shadow-режиме: исполняется всегда вердикт `quick_decide` |
| `evaluation.py` | 3 режима (deterministic / model_only / guarded) × метрики |

## API contract (System One)

Контракт взят из исходников `typesafe-sdk-python` v0.7.2 (`f078f1e`), потому что `api.typesafe.ai/docs` из среды недоступен. jev-guard проверял его против живого API (`docs/05-m1-findings.md`).

- Эндпоинты: `POST /v1/systemone`, `GET /v1/models`. Авторизация: `Authorization: Bearer <key>`.
- Запрос: `{"model", "state": str|object|array, "questions": {id: {"type": "noul"|"choice"|"score", "instructions"?, "criteria"}}}`.
  - Для `choice` поле `criteria` обязательно: `{label: description|null}`.
  - Для `score` поле `criteria` обязательно: список, индекс в нём = балл.
- Ответ: `{"model", "answers", "usage": {"input_tokens", "output_tokens"}}`, request id — в заголовке `x-typesafe-request-id`.
  - `noul`: `{"noul": p}`, **confidence нет**.
  - `choice`: `choice`, `confidence`, `probabilities`.
  - `score`: `score` (матожидание), `confidence`, `legend`, `probabilities{"0".."n"}`.
- Ошибки: 400/404/422 → `ProviderConfigError`, 401/403 → `ProviderAuthError`, 429 → `ProviderRateLimited`, 5xx → `ProviderUnavailable`, транспорт → `ProviderUnavailable` / `ProviderTimeout`, битая схема → `ProviderInvalidResponse`.
- Model id: SDK по умолчанию шлёт `jev-latest`; живой API отвечал `jev-1.13.0` (jev-guard, awesome-jev «Current model»). У нас по умолчанию пин `jev-1.13.0`. **Через `GET /v1/models` на нашем ключе не подтверждено** — ключа в среде нет.

Конфиг (env, секреты не коммитятся):

| Переменная | Назначение | По умолчанию |
|---|---|---|
| `JEV_API_KEY` | ключ TypeSafe; нет ключа → провайдер не создаётся | — |
| `JEV_MODEL` | модель Jev | `jev-1.13.0` |
| `JEV_BASE_URL` | эндпоинт Jev | `https://api.typesafe.ai` |
| `LAYA_BASE_URL` | адрес Ollaya; нет адреса → Laya выключена | — |
| `LAYA_MODEL` | модель Laya | `laya` |

## Статус acceptance criteria issue — честно

| Критерий | Статус | Evidence |
|---|---|---|
| Survey awesome-jev → конспект | ✅ | `awesome-jev-survey.md` |
| Survey Laya → конспект, железо, лицензия | ✅ по документам | `laya-survey.md`; model card не прочитана (HF 403) |
| Замер Laya на Pi 4/5 | ❌ **не сделано** | нет доступа к Pi и HF; команды — в `laya-survey.md §6` |
| API contract задокументирован | ✅ по исходникам SDK | см. выше |
| Model id подтверждён `GET /v1/models` | ❌ **не сделано** | нет `JEV_API_KEY`; `list_models()` реализован и покрыт тестом на mock |
| `DecisionProvider` + Jev / Laya / Deterministic + fake, unit tests без сети | ✅ | 80 тестов, raw-вывод в PR |
| Ключ только из env, не в логах | ✅ | `test_orchestrator_with_http_errors_never_logs_key`, `test_http_status_mapping` |
| Timeout / error / invalid / unreachable → deterministic fallback | ✅ офлайн | `test_failure_modes_fall_back_to_deterministic[*]`, `test_timeout_stops_waiting_at_deadline` |
| Дедлайн соблюдается | ✅ офлайн | 50 мс дедлайн: ответ < 0.5 с при провайдере, висящем 10 с |
| Circuit breaker, runtime-переключение провайдера | ✅ | `test_circuit_*`, `test_runtime_provider_switch_without_restart` |
| Метрики `jev_success / timeout / error / invalid / fallback_used` + `provider_used` | ✅ | `DecisionMetrics`, `Decision.provider` |
| Scheduler PoC на офлайн-сценариях, без права исполнять action | ⚠️ частично | shadow-PoC есть, исполняется только вердикт правил. Включение противоречит `SCHEDULER_DESIGN.md §4.7` → **нужно решение Шифу** |
| Acceptance PoC: Jev не может ослабить `stop_navigation` / hard rules | ✅ | `test_stop_navigation_never_depends_on_signal`, `test_every_catalog_tool_is_never_weakened`, `test_evaluation_model_always_no_cannot_weaken_policy` |
| `stop_navigation` / emergency: 0 runtime-зависимости от Jev | ✅ | модуль не подключён к прод-пути; `escalate_acceptance` для `EMERGENCY_TOOLS` возвращает вход, не читая сигнал |
| Outage Jev не влияет на readiness | ✅ по построению | модуль не участвует в health / readiness и в boot |
| p50 / p95 и стоимость на реальном deployment path | ❌ **не сделано** | нужен ключ и прогон с робота: `jev_eval.py --provider jev` |
| Go / no-go по измерениям | ⚠️ предварительное | ниже; финальное — только после замеров |

## Предварительное go / no-go

**Production integration: NO-GO сейчас.** Своих измерений нет, а чужие данные говорят против:

1. p95 Jev по независимым замерам — 533–830 мс, холодный старт ≈ 2.6 с. На safety-классе с дедлайном 400 мс заметная доля вызовов уйдёт в fallback. Выигрыш надо доказать нашим замером, а не вендорским «70–500 ms».
2. Калибровка на чужих данных противоречива: jev-guard получил 50% против 68% у правил, а самый уверенный бакет оказался самым неточным. На русских командах робота данных нет вообще.
3. Структурно, по офлайн-прогону с фейком: даже «идеальная» модель в режиме `model_only` даёт false_safe на `notify`-инструментах, потому что Noul не выражает трёхуровневую политику. Любая схема «модель решает» хуже схемы «модель только эскалирует».
4. Scheduler P0 противоречит принятому §4.7 («одна LLM, без уровня 2»).

**Условный GO на следующий шаг (не production):** shadow-замер acceptance-эскалации с реальным Jev на `scenarios/acceptance.yaml` плюс реальные логи tool call'ов. Критерии перехода:

- p95 ≤ дедлайна класса;
- ноль false_safe в `guarded`;
- прирост human_review по сравнению с политикой не больше N (N задаёт Шифу);
- модель ловит хотя бы один класс ошибок политики. Кандидат уже виден: `delete_track` сейчас `pass_through`, хотя операция необратима.

## Офлайн-прогон (фейки, НЕ Jev)

```bash
PYTHONPATH=src/rob_box_harness:src/rob_box_llm:src/rob_box_core python scripts/research/jev_eval.py --provider fake-down
PYTHONPATH=src/rob_box_harness:src/rob_box_llm:src/rob_box_core python scripts/research/jev_eval.py --provider fake-agree
```

16 сценариев, результат прогона 28.09.2026:

| Режим | fake-down: accuracy | fake-down: false_safe | fake-agree: accuracy | fake-agree: false_safe | fake-agree: review |
|---|---|---|---|---|---|
| deterministic | 0.9375 | `delete_track` | 0.9375 | `delete_track` | 8 |
| model_only | — (0 ответов) | — | 0.8125 | `stop_music`, `finish_mapping`, `set_volume` | 9 |
| guarded | 0.9375 | `delete_track` | **1.0** | — | 9 |

- В fake-down все 16 решений ушли в fallback. После 3 ошибок circuit открылся: `jev_error=3`, `jev_circuit_open=13`.
- fake-agree — это фейк, который отвечает по нашей разметке. Он проверяет **механику** оценки, а не качество Jev.

## Что осталось (нужны доступы / решения)

1. `JEV_API_KEY` в secret store, чтобы прогнать:
   - `GET /v1/models` → подтвердить model id;
   - `jev_eval.py --provider jev` с Vision Pi → p50 / p95, токены, исходы.
2. Доступ к HF и Pi → `laya-survey.md §6`: RAM, латентность, влияние на Nav2 и голос.
3. Решение Шифу по `SCHEDULER_DESIGN.md §4.7`: оставить scheduler только в shadow или пересмотреть «без уровня 2».
4. Утвердить разметку `scenarios/acceptance.yaml`. Отдельно — строгость `delete_track`: это находка, а не часть этого PR.
5. Дедлайны и пороги в `routing.py` — стартовые догадки. Пересмотреть по нашим p95 и калибровке.
