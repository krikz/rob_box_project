# ADR-0088 — Cron tick-summary policy: stdout markers обязательны

**Дата:** 2026-09-09
**Статус:** Accepted
**Автор:** товарищ architect (по карточке t_e3fc9bfe, ретро t_8fba04b9 / issue #1977)
**Связанные ADR:** AF-0063, ADR-0079 (persistence — отдельно, про отчёты nightly-review)
**Тип:** process policy (hermes-agent + scripts/agent_flow/*)

## Контекст

`merge-gate` cron (architect profile, every 5m) последние 50+ тиков имел статус
**`silent (empty output)`** при `exit code 0`. То же наблюдалось для
`agent-flow-e2e-process` (частично silent) и `agent-flow-blocked-watchdog`.

**Root cause** (найдено в этом фиксе):

`hermes_cli.subcommands.cron` обрабатывает cron-job'ы в режиме `--no-agent`:
> *«the script IS the job and its stdout is delivered verbatim. Empty stdout = silent»*

То есть `silent` = **empty stdout**, не stderr. Скрипты agent-flow исторически
писали **только в stderr** (`log() { printf ... >&2; }`). Это даёт два
практических бага:

1. **Невозможно диагностировать без ssh**: cron output файл содержит только
   `Status: silent (empty output)`. Без захода на хост нельзя понять,
   запускался ли скрипт, дошёл ли до main loop, были ли issues для
   обработки. Особенно критично ночью — карточки висят в `running`, но
   неясно, что merge-gate вообще что-то делает.

2. **Ложно-зелёный статус**: cron pipeline считает тик успешным (exit=0),
   даже если по факту скрипт упал на середине через `set -euo pipefail`
   без возможности откатиться (без exit-кода ошибки).

**Наблюдения** (ретро t_8fba04b9 §2.4, §7 lesson 1):

- `merge-gate` cron (job 1082e70dc68f): 50+ тиков silent → невозможно понять,
  почему process метки не ставятся автоматически. Issue #1977 завис в
  `needs-e2e` 5.5 часов.
- `agent-flow-e2e-process` (job 73dcdece0619): partial silent — tick 22:37–22:42
  имел output, потом silent при rate-limit + no-issues path.
- `agent-flow-blocked-watchdog`: писал structured summary в stderr (line 245),
  но не имел tick-start → невозможно было сказать, дошёл ли main loop.

## Решение

Каждый cron-скрипт в `scripts/agent_flow/*` ОБЯЗАН иметь **tick-summary logging**
с тремя инвариантами:

1. **`out()` helper** — пишет в **stdout** (это то, что cron delivery читает)
   + опционально в per-day log-файл `~/.hermes/profiles/<profile>/logs/<script>/YYYY-MM-DD.log`.
   Старый `log()` (stderr) сохраняется для ручной отладки и incident
   detection, но cron-visible output **обязан** идти через `out()`.

2. **Tick-start marker** вызывается **после** успешного прохождения всех gate'ов
   (MAINTENANCE / flock / gh auth), но **до** main work-loop. Если gate
   сработал раньше (skip-tick) — marker не нужен, там gate уже пишет
   в stderr.

3. **Tick-end marker** вызывается явно перед exit, **и** через `trap EXIT`
   ловит аварийные exit'ы через `set -e` / kill / SIGTERM. Гарантирует
   наличие marker даже при crash.

4. **Per-day log-файл** — best-effort, mkdir может не быть доступен; не
   влияет на cron-visible stdout.

### Контракт (что должен делать каждый agent-flow cron-скрипт)

```bash
# 1. Helper рядом с log() — пишет в stdout (cron) + опционально в log-файл.
out() { ... printf '%s\n' "$_line"; ... }

# 2. Structured markers — префикс "# TICK_SUMMARY:" для cron-pipeline парсинга.
tick_start_marker() { out "# TICK_SUMMARY: start pid=$$ script=... ..."; }
tick_end_marker()   { out "# TICK_SUMMARY: end <counters>"; }

# 3. Trap EXIT — ловит аварийные exit'ы.
trap 'tick_end_marker 2>/dev/null || true' EXIT

# 4. Tick-start — после успешного auth/maintenance/flock.
tick_start_marker
```

### Применённые изменения (этот PR)

- `scripts/agent_flow/agent-flow-merge-gate.sh` — добавлен `out()` +
  `tick_start_marker()` + `tick_end_marker()` + trap EXIT + вызов
  `tick_start_marker` после G2 (auth) + `tick_end_marker` перед exit.
  ~46 строк добавлено.

- `scripts/agent_flow/agent-flow-e2e-process.sh` — то же, что для merge-gate.
  ~31 строка.

- `scripts/agent_flow/agent-flow-blocked-watchdog.sh` — то же (скрипт
  уже имел summary в stderr, добавлен marker в stdout). ~30 строк.

- `tests/agent_flow/test_merge_gate_logging.sh` — регрессионный тест:
  mock gh + git (PATH shim), запуск merge-gate с пустым input →
  проверка наличия `# TICK_SUMMARY: start` и `# TICK_SUMMARY: end`
  в stdout + per-day log-файле. **PASS** в этой реализации.

## Альтернативы (рассмотренные, но отвергнутые)

1. **Поменять hermes_cli.subcommands.cron, чтобы читал stderr вместо stdout.**
   Отвергнуто: cron delivery API устоялся, downstream-ы (notification
   channels) уже работают со stdout. Изменение API сломает совместимость
   с существующими cron-скриптами, которые ПРАВИЛЬНО пишут в stdout
   (например, alert-скрипты «memory low»).

2. **Добавить отдельный marker-файл вместо stdout.**
   Отвергнуто: cron delivery уже работает через stdout, добавлять второй
   канал доставки = инфраструктурная сложность без выгоды.

3. **Только log-файл, без stdout marker.**
   Отвергнуто: log-файл доступен только через ssh, а silent-bag именно
   в том, что нельзя зайти на хост ночью без потери времени. Stdout —
   это cron delivery channel.

## Последствия

**Плюсы:**
- `silent (empty output)` больше не маскирует silent-bag'и. Если скрипт
  упал через `set -e` на середине — `tick_end_marker` всё равно
  сработает через trap EXIT, и cron покажет **частичный output**, что
  сразу видно в pipeline.
- Per-day log-файлы в `~/.hermes/profiles/architect/logs/<script>/`
  дают post-mortem диагностику без ssh (`tail -f` по локальной ФС).
- Маркеры `# TICK_SUMMARY:` структурированы — cron-pipeline может парсить
  без grep по произвольному тексту (будущее улучшение).

**Минусы:**
- Все три скрипта получают ~30 строк boilerplate. Дублирование
  устранено только в `lib_agent_flow_common.sh` для части helper'ов —
  `out()` намеренно дублируется (LOG_PREFIX у каждого свой).
- При rate-limit / skip-tick marker не появляется в stdout (gate сам
  пишет в stderr). Это by-design: gate-лог уже несёт диагностику,
  marker был бы шумом.

**Совместимость:**
- Существующие `log()` (stderr) — **не меняются**. Обратная совместимость
  100%. Скрипты-читатели stderr продолжают работать.
- Cron-pipeline видит теперь non-empty stdout на каждом тике, что
  убирает «silent»-class incident'ы.

## Применение policy к новым cron-скриптам

Любой новый `scripts/agent_flow/*-cron*.sh` или `scripts/agent_flow/*-watchdog*.sh`
ОБЯЗАН иметь helper `out()` + `tick_start_marker()` + `tick_end_marker()`
+ `trap 'tick_end_marker 2>/dev/null || true' EXIT`. PR без этого будет
rejected на review (см. ретро t_8fba04b9 §7 lesson 1).

Рекомендуется вынести helper'ы в `lib_agent_flow_common.sh` для будущих
скриптов (сейчас намеренно оставлено inline — `LOG_PREFIX` per-script).

## Ссылки

- Issue #1977 — `silent (empty output)` для merge-gate (50+ тиков)
- Ретро t_8fba04b9 §2.4 — symptom (cron output без диагностики)
- Ретро t_8fba04b9 §7 lesson 1 — silent = невозможно диагностировать
- `~/.cache/uv/archive-v0/.../hermes_cli/subcommands/cron.py:65` —
  «Empty stdout = silent» (root cause)
- `tests/agent_flow/test_merge_gate_logging.sh` — регрессионный тест
- `hermes cron incidents` — список incident'ов для будущего мониторинга
  (см. `hermes cron incidents` после deploy).