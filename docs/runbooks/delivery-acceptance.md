# Приёмка доставки релизов (chuangong)

Как принимать путь доставки обновлений: офлайн на фикстурах и (когда разрешит Шифу) на роботе.
Текст следует ASD-STE100 на русском. Живой пилот здесь **не** описан как пройденный.

Связано: ADR-0168 (фаза B), `host/chuangong/`, `docs/runbooks/live-verification.md` §8,
rbt_infra ADR-0003. Карточки: #4375, #4694.

Метки: **[код]** — есть в `host/chuangong` сейчас; **[фаза C]** — план, execute ещё нет;
**[решение Шифу]** — открытый вопрос, код не выбирать.

---

## 0. Границы

| Уже есть **[код]** | Ещё нет **[фаза C]** |
|---|---|
| Подпись minisign, fetch, stage, report | `compose down/up`, symlink `current`, host-миграции |
| Adoption доказанного legacy-стека | Реальное переключение стека |
| Dry-run `preflight` / `activate` / `rollback` (план, `execute=false`) | Автоматический откат после health fail |
| Блокировки: HIL-lock, dirty/hotpatch, bad signature, no space | Ритуал biguan/chuguan на роботе |
| `F: Update Robot` action=`status` | action=`update`/`rollback`/`cancel` с тикетом |

Офлайн-приёмка (этот runbook + pytest) **не** заменяет живую проверку фазы C.
Не пишите «пилот пройден» без raw с трёх Pi и разрешения Шифу.

---

## 1. Главный путь: baseline → исправный → падающий → авто-откат

Цель: один непрерывный сценарий приёмки. На C1 (сейчас) шаги 3–4 — dry-run и health-блок;
реальное переключение — только после реализации фазы C и разрешения Шифу.

### 1.1 Baseline **[код]**

1. Три узла в `observe` (или локальные фикстуры World).
2. В state: `current` = доказанный baseline (через `adopt --apply` или ручная история в тесте),
   `previous` = null или старший релиз.
3. HIL-lock нет, dirty/hotpatch нет, место на диске есть.
4. Канал `dev` указывает на baseline или на кандидата — зафиксируйте имя релиза.

**PASS (офлайн):** `python -m chuangong run` / tick → `state=available` или `staged`,
`reason` пуст, `hil_locked` не true, `dirty` false.

**PASS (живо, когда поставят агента):** `F: Update Robot` action=`status` — три узла без
`fetch_error`/`stage_error`/`blocked`.

### 1.2 Исправный релиз **[код] dry-run; [фаза C] execute**

1. CI собрал и подписал релиз (`L: Build All Services` → `release.py sign` → `verify` ключом
   `host/chuangong/keys/release.pub`).
2. Агент: fetch → minisign.verify → schema → (mode=stage) bundle+images.
3. Preflight: staged, не HIL, не dirty, current доказан, compat.min_previous ок.
4. Activate dry-run: план Vision → Sound → Main, все шаги `execute=false`,
   health=`not_verified` (в dry-run так и должно быть; «healthy» без проверки — FAIL).

**PASS (офлайн):** см. §6 сценарий `healthy_release`. State `activation_planned`,
`would.current` = имя кандидата, история current/previous **не** менялась.

**PASS (живо, фаза C):** после execute — три Pi на новом `current`, контейнеры healthy,
метрика `rbx_release_info{…,state=…}`. Без raw — не PASS.

### 1.3 Падающий релиз **[код] блок health; [фаза C] execute+health**

1. Кандидат staged и preflight прошёл.
2. Health после переключения (или в тесте Health-фейк) даёт `unhealthy` с raw detail
   (например `nav2: exited (127)`).

**PASS (офлайн сейчас):** activate с `Health(status=unhealthy, …)` → `state=blocked`,
`reason=health_check_failed`, detail = сырая строка. История не переписана.
Состояние **не** `rollback_planned` (авто-откат из health ещё не вызван — см. §1.4).

**PASS (живо, фаза C):** переключение не остаётся на сломанном релизе; см. §1.4.

### 1.4 Автоматический откат **[фаза C]** — критерии отдельно от §2.1

Задумка ADR-0003 / ADR-0168: после fail health агент **сам** активирует `previous`
(порядок ролей тот же, **без** ручного `chuangong rollback` / workflow).

| Критерий | Авто-откат (§1.4) | Ручной откат (§2.1) |
|---|---|---|
| Кто инициирует | health fail → код агента | оператор CLI / workflow |
| Офлайн сейчас | `blocked` / `health_check_failed`; **нет** `rollback_planned` из одного activate | явный `rollback()` → `rollback_planned`, план `execute=false` |
| История current/previous | не меняется (отказ) | не меняется (только план) |
| Стек compose/symlink | не тронут | не тронут |
| Живо, фаза C | robot на previous **без** ручного входа | robot на previous **после** ручного rollback |

**Сейчас [код]:** health fail только блокирует activate. `rollback()` **не** вызывается
из health и **не** execute-ит compose. Авто-связь health→rollback — код фазы C, не промпт.

**PASS (офлайн до фазы C):** S2 — activate + unhealthy → `blocked`/`health_check_failed`,
не `rollback_planned`, история прежняя. Это доказательство **отказа**, не авто-отката
и не восстановления стека.

**PASS (живо, фаза C):** сломанный релиз → без ручного `F: Update Robot` robot снова на
previous; raw: state.json, docker ps, journal chuangong. Пока execute нет — в чеклисте
`NOT_IMPLEMENTED`, не PASS.

Восстановление стека доказывает тест исполнителя, не эта фикстура.

---

## 2. Отдельные сценарии

### 2.1 Ручной откат **[код] dry-run; [фаза C] execute** — не путать с §1.4

Оператор (workflow / CLI) явно вызывает rollback. Это **не** health-автомат §1.4:
ожидаемый офлайн-результат другой (`rollback_planned`, а не `health_check_failed`).

```text
chuangong rollback          # C1: план, execute=false
# фаза C: тот же вход, но compose/symlink
```

| Условие | Ожидание (ручной) |
|---|---|
| `current` null | `blocked` / `current_unproven` |
| `previous` null | `blocked` / `no_previous` |
| previous не staged | `blocked` / `release_not_staged` |
| previous подписан и staged | `rollback_planned`, план Vision→Sound→Main, `execute=false` |

**PASS (офлайн):** S3 `test_manual_rollback_plan` — план и `would.current=previous`;
история current/previous **не** swap; compose не вызывался.
Восстановление стека доказывает тест исполнителя, не эта фикстура.

### 2.2 Плохая подпись **[код]**

Проверка только через `chuangong.minisign.verify` (тот же модуль, что CI `sign` и агент fetch).
Новый криптопротокол не вводить.

| Фикстура | Ожидание |
|---|---|
| Тело release.yaml изменено, `.minisig` старый | `SignatureError` / tick `fetch_error` reason `bad_signature` |
| Подпись другим keynum | `fetch_error`, «другим ключом» |
| Мусор вместо `.minisig` | `SignatureError` |
| Валидная пара ключ+подпись | `verify` возвращает trusted comment; tick `available`/`staged` |

На диск stage при bad_signature **ничего** не пишется (ни bundle, ни pull).

**PASS (офлайн):** §6 `healthy_release` vs `bad_signature` — контраст обязателен.

### 2.3 HIL-lock **[код]**

Файл-замок Main Pi: тот же путь, что `scripts/deploy/check_hil_lock.sh`
(по умолчанию `/tmp/hil.lock`; в агенте `Config.hil_lock`).

- На **main**: `hil_lock.exists()` → `hil_locked=true` → activate/preflight/rollback
  `blocked` / `hil_locked`.
- На vision/sound: `hil_locked=null` (замок только main).

**PASS:** lock есть → activation не планируется; lock снят → preflight может пройти
(если остальное ок).

### 2.4 Hotpatch / dirty **[код]**

Источники dirty (probe_flags):

1. Флаг `/var/lib/chuangong/dirty` (hotpatch пока **не** ставит — фаза C; агент уже читает).
2. Контейнер с меткой `/ws/HOTPATCH` (`scripts/robot_dev/hotpatch.sh push` →
   `Docker.hotpatched_containers()`).
3. Ошибка probe hotpatch → fail-closed: `dirty=true`, reason при activate
   `dirty_probe_failed`.

**PASS:** любой dirty → `blocked` / `dirty` (или `dirty_probe_failed`); после снятия
hotpatch и флага — dirty false.

### 2.5 Отказ одной роли **[код] отчёт; [фаза C] оркестрация**

Три независимых state key: `robots/<id>/ops/chuangong/state/{main,vision,sound}`.

| Симптом одной роли | Что видно |
|---|---|
| bad_signature / registry | узел в `fetch_error`, остальные могут быть `available` |
| no_space / pull fail | узел `stage_error`, другие не обязаны падать |
| не staged при activate | на этом cfg.node `blocked` / `release_not_staged` |
| status workflow | `chuangong.status` зелёный **только** если все три без ошибок |

**PASS (офлайн):** одна роль в World с bad sig → её tick `fetch_error`; вторая роль
с тем же валидным артефактом (другой pubkey path / тот же) — `available`. Сводный
status не зелёный, пока есть ошибка.

**PASS (живо, фаза C):** оркестратор не объявляет флот обновлённым, если один узел
не `current`; raw по трём state.json.

---

## 3. biguan / chuguan — варианты индикации (не в коде робота)

Имена зарезервированы в `CONTEXT.md`: «состояние робота во время ритуала обновления и
выход из него». **Кода biguan/chuguan нет.** Фразу и сцену Шифу ещё не выбрал
(ADR-0168 U8). Ниже — **проверяемые** варианты на уже существующих швах. Не включать
в агент/личность, пока нет решения. Не объявлять один вариант «единственным».

### Вариант A — речь через владельца SpeechEvent **[код шва есть]**

- Запрос речи: `/voice/speech/request` (writer: `speech_client`).
- Факт: `/voice/speech/event` (`rob_box_dialog_msgs/SpeechEvent`), kinds:
  queued / synthesized / started / progress / finished / cancelled / rejected / voice_changed.
- **Проверка:** в логе voice-assistant есть `SpeechEvent started` … `finished` с тем же
  `speech_id`; текст реплики — из raw, не из «ожидаемой» константы runbook.
- **Нет:** отдельного kind или топика `biguan`/`chuguan`. События «ритуал начался» как
  типа SpeechEvent **нет** — нужно решение Шифу (кто говорит: Личность / ТАРС / молча).

### Вариант B — лицо и кольцо через Expression **[код шва есть]**

- Запрос: `/voice/expression/request` (`ExpressionRequest`, writer: `expression_client`).
- Факт: `/voice/expression/event` (`ExpressionEvent`), executors face | ring | sound
  (ownership.yml; ADR-0150 §5.5.2).
- **Проверка:** на период «затвора» — `ExpressionEvent` started от face и/или ring;
  на «выход» — смена сцены или finished + возврат idle.
- **Нет:** сцены/анимации с именем `biguan` или `chuguan` в словаре. Имя сцены —
  решение Шифу (из существующих `KNOWN_ANIMATIONS` или новая сцена отдельной карточкой).
  Топика `/voice/biguan` **нет** — не выдумывать.

### Вариант C — только операторский контур (без анимации на роботе) **[код частичный]**

- Индикация: `state.json` + zenoh `robots/<id>/ops/chuangong/state/<node>` +
  `metrics.prom` (`rbx_release_info`, `rbx_release_dirty`, `rbx_release_hil_locked`).
- Пульт: `F: Update Robot` status / Job Summary.
- **Проверка:** смена `state`/`current`/`reason` по трём узлам; человек у пульта видит
  таблицу. Робот может молчать и не менять лицо.
- **События biguan на роботе нет** — и это допустимый вариант, если Шифу так решит.

### Сводка для решения Шифу

| # | Канал | Что уже есть | Чего нет |
|---|---|---|---|
| A | SpeechEvent | `/voice/speech/event` | kind/ритуал, утверждённая фраза |
| B | ExpressionEvent | `/voice/expression/event` face/ring | сцена `biguan`/`chuguan` |
| C | chuangong state | state.json, zenoh, metrics | UX «ритуала» для человека рядом с роботом |

До решения: в живом чеклисте фазы C пункт «анимация biguan» = `NOT_IMPLEMENTED` /
«события нет, нужно решение Шифу», не FAIL из‑за отсутствия фразы.

---

## 4. Офлайн-прогон (обязателен до любого «готово» по сценариям подписи)

Из корня репо или `host/chuangong`:

```bash
cd host/chuangong
python -m pytest tests/test_delivery_acceptance_scenarios.py -v
```

Тот же набор подхватывает `G: Lint Code` (`host/chuangong/tests`).

Скрипт-чеклист (exit 0 = все офлайн-сценарии зелёные):

```bash
python scripts/testing/delivery_acceptance_checklist.py
```

Чеклист **не** ходит на робота и **не** пишет, что пилот пройден.

---

## 5. Живой чеклист (только с разрешением Шифу, фаза C)

Не выполнять из этого PR. Когда execute и ритуал появятся:

1. Замок стенда, HIL off если нужен update, backup previous staged.
2. Baseline status — три узла.
3. Исправный релиз — update — raw `docker ps` и state.json.
4. Сломанный релиз (inject unhealthy) — **авто-откат** §1.4: previous без ручного входа;
   отдельно — **ручной** rollback §2.1: после явного `chuangong rollback` / workflow.
   Офлайн-фикстуры S2/S3 это не заменяют (отказ и план, не стек).
5. Повторить §2.2–2.5 точечно на железе.
6. biguan/chuguan — по выбранному Шифу варианту A/B/C; иначе `NOT_IMPLEMENTED`.
   В робота из этого runbook biguan **не** включать.

Доказательства: как в `live-verification.md` (команда, полный вывод, PASS/FAIL, GMT+1).

---

## 6. Матрица офлайн-сценариев → тест

| ID | Сценарий | Ожидаемый machine outcome | Тест |
|---|---|---|---|
| S0 | baseline observe | `available`, подпись ok | `test_baseline_observe_signed_release` |
| S1 | исправный релиз (stage+activate dry-run) | `staged` → `activation_planned` | `test_healthy_release_activation_planned` |
| S2 | health unhealthy → **отказ** (авто-откат ещё не вызван) | `blocked`/`health_check_failed`; **не** `rollback_planned` | `test_failing_release_health_blocks` |
| S3 | **ручной** rollback plan (оператор) | `rollback_planned`, `execute=false`; история не swap | `test_manual_rollback_plan` |
| S4 | bad signature | `fetch_error`/`bad_signature`, диск пуст | `test_bad_signature_rejected` |
| S5 | HIL-lock | `blocked`/`hil_locked` | `test_hil_lock_blocks_activation` |
| S6 | hotpatch dirty | `blocked`/`dirty` | `test_hotpatch_dirty_blocks_activation` |
| S7 | отказ одной роли | одна `fetch_error`, другая ok | `test_single_role_fetch_failure` |

S2 и S3 — **разные** критерии: S2 = отказ activate (задел авто §1.4, без вызова rollback);
S3 = явный ручной `rollback()` (§2.1). Ни S2, ни S3 не доказывают восстановление стека:
восстановление стека доказывает тест исполнителя, не эта фикстура.

Контраст S1 vs S4 обязан использовать **один** API: `chuangong.minisign.verify`
(прямо или через `fetch_release` / `tick`).
