# ADR-0016: health_monitor cross-container scope leak — stop-gap dedup exclusion, defer container-scoped refactor

| Поле         | Значение                                                                |
|--------------|-------------------------------------------------------------------------|
| Статус       | **Accepted**                                                            |
| Дата         | 2026-08-12                                                              |
| Автор        | architect (Hermes Agent)                                                |
| Формат       | MADR (Markdown Any Decision Record)                                     |
| Контекст     | Issue #775 (Deployment Critical: staging / Main / perception / critical_log — false-positive) |
| Связанные    | ADR-0010 (perception bridge), PR #1145 (watchdog duplicate token holders) |

---

## 1. Контекст и проблема

### 1.1 Что наблюдалось

Деплой-чек `L-Deploy and Verify` создал issue **#775** с заголовком
`🚨 Deployment Critical: staging / Main / perception / critical_log`. В evidence:

```text
[health_monitor-1]   [ERROR] telegram_node (8s ago): Telegram bot crashed
(attempt 12): Timed out. Restarting in
```

На первый взгляд — perception-связанный крэш. На самом деле:

| Что                                    | Где живёт                                                  |
|----------------------------------------|------------------------------------------------------------|
| `telegram_node` (Vision)               | `docker/vision/docker-compose.yaml` → сервис `telegram-bot` |
| `health_monitor` (Main)                | `docker/main/docker-compose.yaml` → сервис `perception`    |
| Общий `ROS_DOMAIN_ID=0` + `network_mode: host` | весь робот — один DDS-домен                              |
| `health_monitor` подписан на `/rosout`  | получает Log-сообщения от **всех** нод в домене             |
| `print_report()` печатает ERROR в stdout| ошибки `telegram_node` попадают в **perception** docker logs |

Деплой-чек сканирует stdout **perception**-контейнера → видит ERROR →
открывает issue против **perception**. Реальная проблема — в vision-контейнере.

### 1.2 Корневая причина (уже исправлена)

Retro `t_5af222ea` (PR #1145): 4 gateway-а держали один `TELEGRAM_BOT_TOKEN`,
по умолчанию подключался только один, остальные циклились `bot token already
in use` каждые 300s. watchdog детектит таких и рестартит.

→ Сам по себе `telegram_node` скоро перестанет крашиться. Это **не**
инцидент для perception.

### 1.3 Архитектурный smell

`health_monitor` агрегирует ERROR-логи со **всего** ROS-домена и печатает их
в свой stdout. Это даёт **scope leak**: проблема vision-контейнера
транслируется как «проблема perception». Деплой-чек **де-факто** работает
только если оператор вручную разбирается, чей именно ERROR.

Smell формально:

> *Bounded context* (DDD) «main/perception» сливает ERROR-события из
> *bounded context* «vision/telegram» в общий лог. Деплой-алёрт
> интерпретирует это как нарушение контракта perception-контейнера.

## 2. Рассмотренные варианты

### 2.1 Минимальный stop-gap: добавить telegram_node в dedup-исключения ✅ ВЫБРАН

**Что:** одна строка в `CRITICAL_EXCLUDE_BY_SCOPE["main"]` фильтрует
`[health_monitor-1] [ERROR] telegram_node ...` при сканировании main-scope.

**Плюсы:**
- 1 файл, 1 правка + 2 unit-теста → нулевой риск регрессии;
- Совместимо с существующими тестами dedup;
- Не ломает детект реальных perception-проблем.

**Минусы:**
- Не лечит сам smell — это пластырь, а не лечение;
- Для каждого следующего «внешнего» источника ERROR придётся добавлять
  новое правило вручную.

### 2.2 Скоупинг health_monitor по контейнеру

**Что:** `health_monitor` параметризуется списком «своих» нод (по имени или
регулярке) и печатает только их ошибки. Чужие ERROR/WARN — в отдельный
«foreign errors» канал, либо просто игнорируются.

**Плюсы:**
- Лечит smell системно, а не симптом;
- Деплой-чек получает **семантически корректную** картину «как дела
  именно у этого контейнера»;
- Снижает шум в perception-логах в целом (не только для telegram).

**Минусы:**
- Требует явной карты «какая нода к какому контейнеру относится» — это
  продуктовое решение, я как архитектор его **принимать не должен** без
  PM/оператора;
- Может скрыть легитимные кросс-контейнерные ошибки (если voice-assistant
  в vision не публикует ожидаемый топик, perception это не узнает → хуже
  диагностируемость);
- Рефактор требует интеграционных тестов на нескольких контейнерах — не
  для текущего hot-fix бюджета.

### 2.3 Полностью убрать `print_report` ERROR из docker logs

**Что:** `health_monitor` печатает отчёт в отдельный файл / топик, а не в
stdout, который деплой-чек сканирует.

**Минусы:** теряется удобство grep-а по `docker logs perception` для оператора;
перенос логики детектор-чек должен знать про новый файл/топик; **та же**
проблема scope leak, просто перенесена.

### 2.4 Отключить `health_monitor` вовсе

**Минусы:** теряем реальную пользу (мониторинг активен для оператора и
звуковых алёртов). Не вариант.

## 3. Решение

Принять **вариант 2.1** как stop-gap **прямо сейчас**. Зафиксировать
**вариант 2.2** как **deferred** работу — её нужно делать отдельно, с
продуктовым вводом и интеграционными тестами.

Реализация (PR к #775):

```python
# .github/scripts/deployment_issue_dedup.py
CRITICAL_EXCLUDE_BY_SCOPE = {
    "main": [
        # ... existing patterns ...
        # Scope leak (issue #775): telegram_node lives in the vision
        # container; its ERROR lines leak into the perception container's
        # docker logs via the shared /rosout bus. PR #1145 fixes the
        # upstream root cause; this exclusion stops the false-positive.
        r"\[health_monitor.*\]\s+\[error\]\s+telegram_node",
        r"telegram bot crashed",
    ],
    "vision": [],
}
```

+ 2 регрессионных теста:

- `test_extract_relevant_log_line_ignores_telegram_node_in_main_scope`
  — main-scope НЕ поднимает алёрт.
- `test_extract_relevant_log_line_still_catches_telegram_node_in_vision_scope`
  — vision-scope продолжает ловить настоящие проблемы telegram-бота.

## 4. Последствия

### Положительные

- #775 больше не будет переоткрываться до тех пор, пока telegram_node
  крашится — issue остаётся за vision-контейнером, как и должно быть.
- Восходящий фикс #1145 делает свою работу; stop-gap не конфликтует.

### Отрицательные / риски

- Если в perception появится **реальный** ERROR, маскирующийся под
  «telegram_node ...» в тексте — он тоже будет исключён. Это маловероятно
  (имя ноды достаточно специфично), но см. §5.

- Долгосрочно — нужен **вариант 2.2**, иначе при росте числа контейнеров
  список исключений будет расти как снежный ком.

## 5. Когда пересматривать

- Когда в production появится **второй** случай scope leak (т.е. ещё
  одна vision-нода роняет deployment-алёрт в main-контейнере).
- Когда PM/оператор явно попросит «мониторь каждый контейнер
  независимо» — это инициирует вариант 2.2.
- Если dedup-список main-scope вырастет > 25 строк — это явный сигнал,
  что пора делать скоупинг.

## 6. Связанные

- Issue **#775** — исходный false-positive alert.
- PR **#1145** — upstream watchdog fix для duplicate token holders
  (commit `3b6d377c`).
- ADR-0010 — perception architecture (зачем health_monitor вообще
  существует в perception-контейнере).
