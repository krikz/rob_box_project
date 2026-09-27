# ADR-0021-r1 — CC-budget: ratchets, phantom-detection, потолок CC>30

**Дата:** 2026-09-15
**Статус:** proposed
**Автор:** devops (по issue #2626)
**Связанные:** ADR-0021, ADR-0018 (честный FAIL), ADR-0013 (incremental delivery), issue #2626, #2186, #1984, #2077

---

## Контекст

К моменту написания #2626 (15.09) сторож `scripts/lint/cc_budget_refs.py`
(заведён #2186 ровно для того, чтобы блокировать молчаливое пополнение
baseline) **не работал**:

- 51 exemption в `cc_budget_baseline.json`, **0** записей в `_refactor_cards`.
- Все 51 жили в `_legacy_acknowledged` — и для них ref-card не требовалась.
- Гард печатал `OK — все exemptions имеют ref-cards` (технически верно:
  ref-cards не требовались — значит, их отсутствие = «всё ок»).
- Сентябрь: восемь bump'ов (`#2547/#2548/#2549/#2557/#2559/#2560/#2562/#2565`)
  без единой новой ref-карточки. `DialogueNode._handle_result` дошёл до CC=85
  (5.7× бюджета), `_run_turn` до CC=59, `_on_stt` до CC=54.

Дополнительно вскрылись четыре дефекта конструкции сторожа:

1. **Legacy-долг иммутабельный, но гард не запрещает bump `cc`** у
   legacy-записи: можно правкой `exemptions` поднять `cc` у метода,
   который всё ещё сидит в `_legacy_acknowledged`, и сторож пропустит —
   потому что не сравнивает.
2. **`cc_budget.py` печатает `[info] recovered` и пропускает**. Любой
   рефакторинг, реально снизивший CC ниже baseline (SetDjModeTool CC 18→3,
   dialogue_callback 29→22, spec_from_flat 19→16, execute_code 23→22) —
   гард не требует обновить baseline. Следующий PR может вернуть CC
   обратно, и сторож промолчит (3 ≤ 18).
3. **`_build_single_provider` (CC=23) лежит в baseline, но метода в коде
   нет** (логику вынесли в `rob_box_harness/providers/catalog.py`). Запись
   будет лежать вечно.
4. **`--verify-remote` итерируется только по `_refactor_cards`** — у нас
   0 записей, поэтому сетевая проверка проходит вхолостую. Плюс даже при
   наличии карточек проверяется только метка `type:tech-debt`, но **не
   state=open**: закрытая карточка проходит проверку. Именно так
   `#1984`/`#2077` (родительские «decomp backlog» gate'ы) были
   закрыты — а 48 legacy-exempt продолжали на них ссылаться.

Сторож **оформлен как шов без потребителя** (`G-Lint Code.yml` не зовёт
`--verify-remote`, нет nightly-job, нет post-merge проверки), поэтому
все четыре дефекта не были видны в работе.

## Решение

Дополняем ADR-0021 R1 пятью инвариантами и одной политикой:

### R-1a. Frozen legacy

Запись в `_legacy_acknowledged` обязывает автора baseline **никогда** не
правлять `exemptions[path][method] = новое_значение > legacy.cc`. Это
иммутабельность baseline-значения для legacy-методов. Правка возможна
только в одну сторону: перенести запись из `_legacy_acknowledged` в
обычный exempt с заполненным `_refactor_cards` (т.е. когда рефакторинг
завёл карточку, автор выносит метод из-под амнистии). Шаги:

- `scripts/lint/cc_budget_refs.py check_local()` сравнивает
  `exemptions[path][method]` с `legacy_acknowledged[].cc`. Если
  `exemptions > legacy` → `[FAIL] «рост legacy-записи: cc=N выше
  legacy=N; уберите запись из _legacy_acknowledged и добавьте
  _refactor_cards → #NNNN»`. Это та проверка, которой #2186 не
  хватало; она сама по себе закрывает восемь сентябрьских bump'ов.
- Запрещено **снижать** cc у legacy-записи тоже — это путь к
  «фото лежит, реальность изменилась», т.е. legacy-cc должен
  совпадать с реальным cc до завершения рефакторинга.

### R-1b. Ratchet (односторонний хряповик)

`scripts/lint/cc_budget.py` сейчас печатает `[info] … recovered; refresh
baseline with --update-baseline` и завершается с `OK`. Заменяем на
`[FAIL]` с требованием обновить baseline в **том же PR** (тоб автор
рефакторинга явно снимает «сверх-cc»). Плюс отдельный CI-шаг, который
проходит по baseline и падает, если **любое** значение `exemptions`
меньше измеренного CC — независимо от того, кто его снизил.

### R-1c. Phantom-detection

После измерения CC по текущему коду `cc_budget.py` берёт
множество ключей baseline (`path:method`) и вычитает из него множество
найденных функций. Если baseline содержит ключ, которого нет в
коде — `[FAIL] «фантомная запись: удалите из baseline»` (по факту это
сигнал «рефакторинг сделан, забыли прибраться» — **приятный** FAIL).
Фантомы собираются и в `cc_budget_refs.py` для кросс-валидации.

### R-1d. `--verify-remote`: честная проверка

- `check_remote()` итерируется по **обоим** источникам: `_refactor_cards`
  (для новых exempt) **и** по `#NNNN`-ссылкам, вытащенным из `since:`
  полей `_legacy_acknowledged` (для legacy-долг). Regex `r"#(\d+)"` —
  любая `#NNNN` в `since:` трактуется как претензия на открытую карточку.
- Каждая вытащенная issue проверяется на **state=open** через поле
  `"state": "open"` в ответе `gh api /repos/{owner}/{repo}/issues/{n}`.
  Если карточка закрыта — `[FAIL]` с предложением либо
  переоткрыть, либо удалить из baseline.
- Метка `type:tech-debt` остаётся обязательной — но только для
  `_refactor_cards` (legacy-карточки могут иметь любую метку, потому что
  родительские gate'ы были о других вещах).

### R-1e. Потолок CC>30

Новый exemption с `CC > 2 × METHOD_LIMIT` (т.е. `CC > 30`) не принимается
**без явной ADR-записи**. Это catch для паттерна «функция-монстр»:
`_on_json_cmd` дошёл до CC=107 (#2186), `_synthesize_and_play` до
CC=124 (#2078) — оба раза «просто лимит + амнистия» не сработали.
`scripts/lint/cc_budget.py` в `cmd_update_baseline` отказывается
записывать новый exempt с cc>30, если рядом в baseline не появится
запись `"_adr_reference": "docs/adr/NNNN-..."` (формат ссылки на ADR).

Это **политика**, а не скриптовый запрет: скрипт можно обмануть, поставив
любую ссылку. Но (а) скрипт делает это явным, а не молчаливым
bump'ом; (б) rev-gate / review-бот может grep'ать эту ссылку при
обзоре.

### R-1f. Честное сообщение гарда

`scripts/lint/cc_budget_refs.py main()` теперь печатает не «OK — все
exemptions имеют ref-cards», а разбивку:

```
cc_budget_refs: 51 exemptions: 4 с ref-card, 47 legacy (амнистия #2186), 0 фантомных
```

Если хотя бы один legacy — это **тоже не OK**, а долг под контролем.
Дополнительно каждая цифра пишется в логе одной строкой, чтобы
`grep`-поиск в CI-выводе и ночной run легко её парсили.

### R-1g. Nightly-job

`--verify-remote` включается в новый `.github/workflows/G-CC-Ref-Remote-Verify.yml`
по расписанию `cron: '7 4 * * *'` (07 минут после 04:00 UTC — out of phase
с merge-gate и codeql, чтобы не было stampede). Job:
checkout → setup-python → install gh → auth через `${{ secrets.GITHUB_TOKEN }}`
→ `python scripts/lint/cc_budget_refs.py --verify-remote` с `GH_REPO` в env.
**FAIL → уведомление** через `GITHUB_STEP_SUMMARY` (читается тем же
механизмом, что и `G-Run Tests.yml`).

На каждом PR по-прежнему работает только локальная проверка
(`check_local`), потому что сетевая зависимость ломает CI при
flaky-network и жжёт rate-limit (`5000 req/h` на runner-токене).

---

## Альтернативы, которые мы НЕ выбрали

### ❌ Удалить `_legacy_acknowledged` сразу

- **Почему нет**: 47 legacy-exempt закрыты 4-5 реф-карточками, не
  одной (см. `_handle_result` → #2556, `_on_stt` → #2628, и т.д. по
  тексту issue #2626). Один PR не закроет их все; блокировать всю
  разработку на месяц ради немедленного refactor — нарушение ADR-0013.
- **Когда бы подошло**: при иной структуре — если бы legacy-exempt
  можно было сразу снять одним PR'ом. Сейчас нельзя.

### ❌ Делать verify-remote на каждом PR

- **Почему нет**: rate-limit (`5000 req/h` на GitHub API для токена
  runner'а) + flaky-network → CI становится красным из-за сети, а не
  из-за кода. Уже обожглись на `validate_adr_namespace.sh`.
- **Когда бы подошло**: при наличии self-hosted runner'а с кэшированным
  токеном и при изоляции сетевых ошибок от логических.

### ❌ Запретить любые exempt вовсе

- **Почему нет**: реальный рефакторинг dialogue_node.py — это 3-5 PR
  (ADR-0021 + ADR-0013), и без grandfather он заблокирует всю
  разработку voice-пакета на неделю.
- **Когда бы подошло**: в проекте без legacy-долга, который начинаем
  с нуля. У нас не тот случай.

---

## Последствия

### Положительные

- `_legacy_acknowledged` становится **не amplify**, а **shrink-only**:
  cc не растёт, refactor снимает.
- `_handle_result` / `_run_turn` / `_on_stt` / `_run_with_tools` имеют
  явные ref-карточки (#2556/#2627/#2628/#2629) и убраны из legacy —
  видно в `_refactor_cards`.
- `--verify-remote` ловит «карточка закрыта → ref-ссылка висит» на
  следующий день, а не через месяц.
- Ночной job даёт дрифт в ту же дату, что и сам PR, а не через месяц.

### Отрицательные / риски

- **Стоимость ночного job**: 1-2 мин. на запуск; итого ~1 час
  runner-времени в месяц. Незначительно.
- **Гард становится строже**: придётся чистить
  `_legacy_acknowledged` параллельно с refactor'ами. Это — основная
  работа следующих 2-3 месяцев; ADR-0013 на нашей стороне.
- **`--update-baseline` против `cc>30`**: иногда CC>30 — это
  объективно (парсер SSML, шлюз JSON-CMD). Политика R-1e не
  запрещает, а требует ADR — но ADR надо писать. Это шанс, не риск.

### Нейтральные

- ADR-0021-r1 **дополняет** ADR-0021, не отменяет. R2-R5 остаются.
- ADR-0021-r1 не затрагивает пакеты за пределами `rob_box_voice`
  и `rob_box_harness` — это всё ещё их внутренняя дисциплина
  декомпозиции.

---

## План внедрения

1. **Этап 1 (этот PR)**: реализовать R-1a, R-1b, R-1c, R-1f в
   `scripts/lint/`, обновить `scripts/lint/cc_budget_baseline.json`
   (удалить фантом `_build_single_provider`, актуализировать cc у
   recovered-методов, перенести 4 критичные записи из legacy в
   `_refactor_cards`), создать ночной workflow.
3. **Этап 2 (следующие PR'ы)**: бэк-филл `_refactor_cards` для
   остальных 47 legacy-exempt, по мере готовности соответствующих
   декомпозиционных карточек.
4. **Этап 3 (1-2 цикла)**: ADR-0021-r1 становится **active** после
   approve товарища Шифу.

---

## Acceptance для ADR (он сам)

- [ ] Скрипты `scripts/lint/cc_budget.py` и `cc_budget_refs.py`
      реализуют R-1a, R-1b, R-1c, R-1d, R-1e, R-1f.
- [ ] `scripts/lint/test_cc_budget_refs.py` расширен на все пять
      дефектов (по конвенции #2118): legacy-bump, recovered,
      phantom, verify-remote с closed-issue, CC>30 без ADR.
- [ ] Nightly workflow `.github/workflows/G-CC-Ref-Remote-Verify.yml`
      создан и прошёл хотя бы один dry-run (workflow_dispatch).
- [ ] `cc_budget_baseline.json` обновлён: фантом удалён, recovered
      актуализированы, 4 ref-card в `_refactor_cards`.
- [ ] Товарищ Шифу одобрил ADR (не «silent self-instruction write»,
      ADR-0018).

---

## Связанные

- ADR-0021 R1 — базовый CC-budget.
- ADR-0018 — честный FAIL.
- ADR-0013 — incremental delivery (per-bag workflow).
- Issue #2626 — bug(process), описавший все пять дефектов.
- Issue #2186 — изначальная ref-card гард (закрыта, остаётся parent
  gate для 47 legacy-exempt'ов в поле `since:`).
- Issue #1984 / #2077 — родительские gate'ы для decomp backlog
  (закрыты, см. R-1d — теперь это ловится).