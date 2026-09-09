# Тестирование пакета rob_box_harness

Краткая инструкция: что гоняет CI для harness-пакета, как запускать
локально, и где живут маркеры / coverage-гейт.

## TL;DR

```bash
# Только unit (без сети, без секретов) — то, что бегает в default-CI:
pip install -e src/rob_box_harness
pip install -e src/rob_box_llm  # если ещё не стоит
pip install pytest pytest-cov pytest-asyncio pytest-mock
pytest -m 'not network'

# С coverage-отчётом (>=85% — gate):
pytest -m 'not network' --cov-report=html

# Network/integration (по умолчанию auto-skip без секретов):
MINIMAX_API_KEY=sk-... pytest -m network -v
```

## CI: какие джобы и что они делают

См. `.github/workflows/G-Harness-Tests.yml`.

| Job             | Триггер                                              | Что делает                                                                  | Обязательная для merge? |
|-----------------|------------------------------------------------------|-----------------------------------------------------------------------------|-------------------------|
| `unit-tests`    | push в `develop` / `main` / `feature/**`, любой PR    | `pytest -m 'not network'`, coverage gate 85%                                | **Да**                  |
| `integration-tests` | `workflow_dispatch` (ручной) или `schedule`     | `pytest -m 'network or integration'`, replay по умолчанию, live по запросу  | Нет                     |
| `test-summary`  | после `unit-tests`                                   | Пишет итог в GitHub Step Summary                                            | —                       |

`integration-tests` намеренно вынесена из default-CI: replay-тесты
иногда ломаются на обновлении фикстур, а live — платный и медленный.
Запускается вручную через **Actions → G: Harness Tests → Run workflow**.

## Маркеры

Определены в `pytest.ini` (корень). `--strict-markers` падает на
неизвестных именах.

| Маркер        | Назначение                                                  |
|---------------|-------------------------------------------------------------|
| `unit`        | Быстрые тесты без I/O, сети, hardware                       |
| `harness`     | Тесты ядра harness framework (lifecycle, registry, runner)  |
| `integration` | Широкий интеграционный: сеть, hardware, ROS2                |
| `network`     | Тесты, требующие реального сетевого вызова (MiniMax и др.) |
| `slow`        | >1 секунды (используйте `-m 'not slow'` для smoke-прогона)   |

`integration` и `network` пересекаются, но `network` — подмножество
`integration`. В CI мы гоняем `pytest -m 'network or integration'`,
локально с одним маркером обычно достаточно.

## Auto-skip сетeвых тестов

`conftest.py` в корне смотрит на `NETWORK_REQUIRED_ENV_VARS`. Сейчас
там только `MINIMAX_API_KEY`. Если переменная не выставлена и тест
помечен `network`/`integration`:

* `pytest -m 'not network'` — тесты и так не выбираются, ничего не
  делаем.
* `pytest -m network` без секрета — `skip` с понятным сообщением
  («need one of: MINIMAX_API_KEY»). Лучше, чем тихий проход, который
  маскирует регрессии.
* `pytest` без `-m` — молча `skip`, чтобы локальный прогон
  ``pytest`` без секретов не падал.

## Coverage gate

* `addopts = --cov=rob_box_harness --cov-report=term-missing
  --cov-fail-under=85`
* `[coverage:run].source = rob_box_harness` — считаем только
  производственный код harness-пакета; тесты, `__pycache__`, `venv`
  и т.п. — в `omit`.
* Источник `rob_box_harness` означает, что coverage
  автоматически покрывает и `rob_box_harness/providers/minimax.py`.
  Не нужно дублировать `--cov=...` для каждого провайдера.

## Branch protection (required check)

`unit-tests` — обязательная джоба для merge в `develop` / `main`.

Настройка (один раз, через `gh` API или Settings → Branches):

```bash
# Требует admin:repo и право настраивать branch protection.
gh api \
  --method PUT \
  -H "Accept: application/vnd.github+json" \
  /repos/krikz/rob_box_project/branches/develop/protection \
  --input - <<'JSON'
{
  "required_status_checks": {
    "strict": true,
    "contexts": ["Harness unit tests (no network)"]
  },
  "enforce_admins": false,
  "required_pull_request_reviews": null
}
JSON
```

Имя чекa в `contexts` берётся из `name:` соответствующего step-а в
workflow (см. `unit-tests` job → name: `Harness unit tests (no
network)`).

## Локальная отладка упавшего CI

```bash
# Воспроизвести ровно то, что бегает в CI:
pip install -e src/rob_box_harness
pip install -e src/rob_box_llm
pytest -m 'not network' -v \
  --cov-report=xml:coverage.xml \
  --cov-report=html:htmlcov
xdg-open htmlcov/index.html
```

## Когда добавляете новый тест

1. **Без сети/IO** — никаких маркеров не нужно, попадёт в
   `pytest -m 'not network'` автоматически.
2. **С реальным API** — пометьте `@pytest.mark.network`. Без
   `MINIMAX_API_KEY` в env тест будет `skip` (а не `xfail`).
3. **Hardware/ROS2** — `@pytest.mark.integration` + отдельная
   job-а, если нужно запускать на self-hosted runner.
4. **Долгий** (>1с) — `@pytest.mark.slow`, чтобы можно было гонять
   `pytest -m 'not slow'` для smoke-проверки перед коммитом.
