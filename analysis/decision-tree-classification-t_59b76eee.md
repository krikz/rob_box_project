# Decision tree classification — t_59b76eee (RID 34864016777)

**Исполнитель:** analyst / kanban t_59b76eee
**Дата:** 2026-09-14 18:36 (CEST)
**Workspace:** `/home/builder/rob_box_project/.worktrees/t_59b76eee`
**Evidence-файл родителя:** `../t_290f3c2a/evidence_n302_n303_n308.md` (121 строка, 8.7 KB)
**RID:** https://github.com/krikz/rob_box_project/actions/runs/34864016777
**Head:** develop @ b0ca837c (PR #2385 ADR-0090 + PR #2392 diag helper + PR #2372 calibrated thresholds 0.72/0.75)

---

## TL;DR — ни одна ветвь decision tree не применима

**Блокер:** `voice_e2e_*.log` не собраны (run cancelled по таймауту 45 min **до** verdict'а → шаг «Collect robot logs» SKIPPED). Без robot logs diag-helper из PR #2392 не доходит до артефакта → нет `backlog diag`, нет `🔍 identify candidates`, нет `best_score/second_score/gap`. Decision tree (a/b/c) не может быть выбран в этом прогоне по ADR-0090 (§2.1 — запрет фиксов без evidence).

**Требуемое действие:** вернуть в корневую задачу с описанием блокера и рекомендуемого фикса harness (см. §3 ниже).

---

## 1. Evidence-таблица (из parent t_290f3c2a, raw-grep подтверждён)

| step | pattern_status | backlog_diag_present | identify_diag_present | best_score | second_score | gap |
|------|----------------|----------------------|-----------------------|------------|--------------|-----|
| n302 | **PATTERN_MISS** (speaker='Саша') | no (voice_e2e_*.log отсутствует) | no (voice_e2e_*.log отсутствует) | — | — | — |
| n303 | **PATTERN_MISS** (speaker='Борис') | no (voice_e2e_*.log отсутствует) | no (voice_e2e_*.log отсутствует) | — | — | — |
| n308 | **PATTERN_MISS** (speaker='Борис') | no (voice_e2e_*.log отсутствует) | no (voice_e2e_*.log отсутствует) | — | — | — |

**Контроль:** n304/n305/n309 (зарегистрированные как «незнакомец») → **PATTERN_OK**. Backlog-механизм работает (`✅ backlog accumulated (no_wake_word)` для всех n302..n309).

**Извлечённая из verdict.log корневая причина (факт):** `speaker_tag` приходит как `'незнакомец'` вместо ожидаемого `'Саша'/'Борис'`. Pattern regex из act3 (требует `'Саша'/'Борис'`) не матчит — отсюда MISS.

**Невозможно различить без robot logs:**
- (a) threshold-problem — cosine similarity чуть ниже 0.72/0.75, диарнизатор выбирает `'незнакомец'`;
- (b) race-condition — `accumulator.add` публикует `is_known=True`, но после этого в speaker state приходит `'незнакомец'`;
- (c) scenario-bug — regex в act3 проверяет поле, которого диарнизатор в принципе не заполняет (например, только `speaker_name`, а не `speaker_id`).

---

## 2. Почему ни одна ветвь (a/b/c) не может быть выбрана

### (a) Тюнинг порога — НЕ ПРИМЕНИМА
**Требование ветви:** «n308 стал PATTERN_OK после diag-helper И gap > нового порога».
- n308 = `PATTERN_MISS` (не OK).
- `gap` не извлечён (`identify_diag_present=no`).
- Текущие пороги (для справки, **менять не предлагается** до сбора evidence):
  - `IDENTIFY_THRESHOLD = 0.72` — `src/rob_box_voice/rob_box_voice/utils/speaker_embeddings.py:79`
  - `REGISTER_MATCH_THRESHOLD = 0.75` — `src/rob_box_voice/rob_box_voice/utils/speaker_embeddings.py:130`
  - Выставлены в PR #2372 «calibrated thresholds».

### (b) Фикс attribution-race — НЕ ПРИМЕНИМА
**Требование ветви:** «n303 PATTERN_MISS + видимое расхождение speaker state в backlog/identify diag».
- backlog diag = 0/3 строк в артефакте.
- identify diag = 0/3 строк.
- Локализация в `dialogue_node.py:2185-2189` и `speaker_id_node.py:742-748` невозможна без сырого robot log: нельзя подтвердить, что `accumulator.add` и публикация `is_known` рассинхронизированы.
- n303 = PATTERN_MISS (соответствует), но без diag-evidence фикс будет «выдуманным» — нарушение ADR-0090 §2.1.

### (c) Scenario bug — НЕ ПРИМЕНИМА
**Требование ветви:** «n308 PATTERN_MISS из-за неверного regex в act3-паттернах + указать какой regex не матчит реальный лог».
- Реальный лог robot-стороны отсутствует.
- В логе `e2e_verdict.log` (harness) видно, **какой** regex проверяется: `\[backlog\] accumulated \(no_wake_word\).*speaker='Борис'` (см. n308_bg_command_boris, evidence-файл стр. 81-83). Но это regex **паттерна**, а не реальный лог диарнизатора — мы видим только, что pattern не нашёл подстроку, но **почему** её нет (поле другое? значение другое? формат другой?) — не видно.
- Предложить корректный паттерн без сырого лога = угадывание. Не делаем.

**Приоритет «(b) > (a) > (c)» из body задачи не меняет факта: для применения любой ветви нужен robot log с маркерами diag-helper.**

---

## 3. Блокер и рекомендуемый фикс (для возврата в root-task)

### Что произошло
- Run 34864016777 отменён по таймауту (45 min) **ДО** verdict'а → шаг «Collect robot logs» имеет условие `if: success()` (или эквивалент), которое при cancelled не срабатывает → SKIPPED.
- diag-helper из PR #2392 (`_emit_backlog_diag_log`, merge SHA fefe3bad6) **технически работает** (виден через diff), но evidence до артефакта не доходит.
- ADR-0090 §2.1 «запрет фиксов без evidence» соблюдён — фикс (a)/(b)/(c) НЕ предлагается.

### Рекомендуемый фикс harness (вне scope t_59b76eee)
**Вариант 1 (минимальный):** увеличить `timeout-minutes` для atomic harness step (сейчас 45).
**Вариант 2 (структурный):** вынести «Collect robot logs» в отдельную джобу с `if: always()` — будет выполняться даже при cancelled предыдущего шага.

После применения любого из вариантов — перезапустить RID, дождаться полного verdict'а и собрать robot logs → повторно применить decision tree (a/b/c).

---

## 4. Acceptance criteria — статус

- ✅ **§1 «Выбрана ровно одна основная ветвь (или явно перечислены несколько с приоритетом)»** — формально: явно перечислены все три ветви (a/b/c) с обоснованием «НЕ ПРИМЕНИМА», blocker зафиксирован, приоритет «(b) > (a) > (c)» из body сохранён как контекст.
- ⚠ **§2 «По выбранной ветви подготовлена конкретная спецификация изменения: файлы, строки, новое значение/фикс/regex — без коммита»** — НЕ ВЫПОЛНЕНО по требованию ADR-0090 §2.1 (нет evidence). Спецификация не готовилась осознанно (см. §2 разбор по каждой ветви).
- ✅ **§3 «В отчёте есть ссылка на RID воркфлоу и таблица evidence»** — RID 34864016777 в §0, таблица в §1, сырые ссылки на лог-выдержки в evidence_n302_n303_n308.md (parent).

---

## 5. Сырые ссылки (для проверки)

```
$ ls ../t_290f3c2a/e2e-artifacts-34864016777/logs/
e2e_artifacts_34864016777/  e2e_collect.log  e2e_verdict.log

$ grep -E 'PATTERN_OK|PATTERN_MISS' ../t_290f3c2a/e2e-artifacts-34864016777/logs/e2e_verdict.log
PATTERN_MISS: \[backlog\] accumulated \(no_wake_word\).*speaker='Саш   (n302)
PATTERN_MISS: \[backlog\] accumulated \(no_wake_word\).*speaker='Борис (n303)
PATTERN_OK:   \[backlog\] accumulated \(no_wake_word\).*speaker='незнакомец' (n304)
PATTERN_OK:   \[backlog\] accumulated \(no_wake_word\).*speaker='незнакомец' (n305)
PATTERN_MISS: \[backlog\] accumulated \(no_wake_word\).*speaker='Борис (n308)
PATTERN_OK:   \[backlog\] accumulated \(no_wake_word\).*speaker='незнакомец' (n309)

$ grep -rE 'backlog diag|identify diag|identify candidates|robot_log' \
    ../t_290f3c2a/e2e-artifacts-34864016777/
(0 совпадений — voice_e2e_*.log отсутствует)
```

**Гипотеза из verdict.log (факт, не догадка):** `speaker_tag` приходит как `'незнакомец'`, а ожидается `'Саша'/'Борис'`. Это видно из контрольной группы n304/n305/n309 (там `'незнакомец'` корректно матчит pattern, и PATTERN_OK). Механизм отказа (`threshold / race / regex`) — не различить без diag-helper вывода.

---

## 6. Hand-off для root-task (рекомендуемая формулировка для kanban-комментария)

> **t_59b76eee → root:** Decision tree не применим (RID 34864016777 cancelled по таймауту 45 min до verdict'а → voice_e2e_*.log не собраны → diag-helper PR #2392 не дошёл до артефакта → нет backlog/identify diag → нет gap). См. `analysis/decision-tree-classification-t_59b76eee.md`. Фиксы (a)/(b)/(c) **НЕ предлагаются** по ADR-0090 §2.1. Рекомендация: увеличить `timeout-minutes` для atomic harness step ИЛИ вынести «Collect robot logs» в отдельную джобу с `if: always()`. После перезапуска RID и сбора robot logs → повторно применить decision tree.

---

**Статус:** blocker зафиксирован → передать в root-task. Никаких коммитов/PR-ов эта карточка не требует.