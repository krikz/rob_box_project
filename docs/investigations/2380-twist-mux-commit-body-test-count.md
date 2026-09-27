# Issue #2380 — commit body PR #2205 заявляет «11 новых тестов», реально 8

> Диагностический отчёт architect по issue #2380 (kanban `t_c5f617c4`).
> Дата разбора: 2026-09-14 (UTC+02:00).
> Найдено на kanban-ревью t_33a0336d (PR #2205 commit body).

## TL;DR (версия для Шифу)

- **Issue тезис подтвердился частично, но не в той формулировке, в которой заявлен.**
  Это **НЕ «3 теста добавлены вне `class TestTeleopLockPublishers`»** — таких нет.
  Это **опечатка** в commit body: «11» → должно быть «8».
- **Все 8 заявленных методов существуют и ровно те, что перечислены в issue.**
  Пост-merge состояние файла (`develop`) = 72 теста (`64 + 8`), а не `75`
  (`64 + 11`), как утверждает commit body. 75/72 — расхождение на **ровно 3**,
  что и породило гипотезу issue про «3 теста вне класса».
- **Severity: TRIVIAL (low-impact).** Тесты функционально ок; числа в commit
  body не влияют ни на CI, ни на pytest, ни на ревью merge-gate. Это искажает
  только аудит-метрики ретро (когда кто-то парсит «сколько новых тестов
  добавил PR»).
- **Что делать: ничего не ломая.** MergeCommit `89dd800f` уже в `develop`.
  Единственный способ «исправить» commit body — это:
  1. `git commit --amend` с новым commit body (требует force-push в `develop`
     — НЕ делаем, ломает историю у всех, кто уже подтянул).
  2. Или **follow-up коммит** «docs(test-count): #2205 PR #2205 added 8 tests,
     not 11 — fix commit body audit reference». Чистый self-doc-fix,
     не трогает runtime.
- **Моя рекомендация:** оставить PR/merge как есть, но если Шифу хочет чистый
  audit-trail — открыть **PR-follow-up** от Denis (автора #2205), чтобы он
  сам поправил commit message через amend в своей локальной копии
  (у него git identity для #2205). Это **не блокер** для чего-либо.

---

## 1. Что я проверил (raw)

### 1.1 `git show 89dd800f` — тесты, добавленные в коммите

```bash
$ git show 89dd800f -- src/rob_box_supervisor/test/unit/test_arbiter_node.py \
    | grep -cE '^\+    def test_'
8
```

Восемь (8). Не одиннадцать.

Сами 8 методов (по diff'у коммита, все добавлены внутри `class TestTeleopLockPublishers`):

| # | метод |
|---|---|
| 1 | `test_teleop_lock_publisher_exists` |
| 2 | `test_teleop_lock_publish_timer_registered` |
| 3 | `test_teleop_lock_uses_latched_qos` |
| 4 | `test_acquire_releases_publishes_true_then_false` |
| 5 | `test_expired_floor_publishes_false` |
| 6 | `test_publishes_every_tick_even_when_state_unchanged` |
| 7 | `test_mode_transition_releases_floor_and_lock` |
| 8 | `test_publish_survives_lock_manager_error` |

### 1.2 Пост-merge: общее количество тестов в файле

```bash
$ grep -cE '^    def test_' src/rob_box_supervisor/test/unit/test_arbiter_node.py
72
```

72 = 64 (до PR #2205) + 8 (новых). **Сходится с реальным diff'ом, расходится
с commit body** («75 passed»).

### 1.3 Пост-merge: методы внутри `class TestTeleopLockPublishers`

```bash
$ awk '/^class TestTeleopLockPublishers/{flag=1} flag' \
       src/rob_box_supervisor/test/unit/test_arbiter_node.py \
  | awk '/^class /&&!/TestTeleopLockPublishers/{flag=0} flag' \
  | grep -cE '^    def test_'
8
```

Все 8 новых тестов **именно в этом классе**, ничего «снаружи» не добавлено.
Гипотеза issue #2380 «3 теста вне класса» **не подтвердилась**.

### 1.4 Статус PR #2205

```bash
$ gh pr view 2205 --repo krikz/rob_box_project \
    --json state,mergedAt,mergeCommit,headRefName,baseRefName
{
  "baseRefName": "develop",
  "headRefName": "wt/t_9d33e8ca",
  "mergeCommit": {"oid": "89dd800fe8d733a064b6101f719309ddcc3a546c"},
  "mergedAt": "2026-09-09T09:19:50Z",
  "number": 2205,
  "state": "MERGED"
}
```

PR merged в `develop` через merge-commit `89dd800f`. **Изменить commit body
можно только через force-push** — а это `git push --force-with-lease` в
`develop`, что категорически нельзя делать воркеру.

### 1.5 Связанные коммиты

`git log --all --oneline | grep "#2205"`:

```
884e43197 [voice-vr 06.5] teleop_lock bridge ... (PR #2205)
89dd800fe [voice-vr 06.5] teleop_lock bridge ... (PR #2205)  ← merge-commit
```

Оба идентичны по телу — это обычный PR → merge workflow. amend одного
означает force-push `develop`, что сломает локальные ветки у всех, кто уже
подтянул `89dd800f`.

---

## 2. Гипотезы о причине расхождения

| # | Гипотеза | Подтвердилась? | Evidence |
|---|----------|----------------|----------|
| H1 | Опечатка в commit body («11» вместо «8») | **ДА** | `git show … \| grep -cE test_` = 8 |
| H2 | 3 «недостающих» теста добавлены вне класса | **НЕТ** | Все 8 новых методов внутри `class TestTeleopLockPublishers`, снаружи класса 0 новых |
| H3 | «75 passed» относится к другому файлу | **НЕТ** | `git show 89dd800f --stat` показывает только `test_arbiter_node.py` как изменённый test-файл |
| H4 | PR был force-pushed после первого push с другим набором тестов | **Не проверял** | Требует `git reflog` на стороне Denis, не доступен мне |

Наиболее вероятная: **H1 (опечатка)**. Denis мог изменить план тестов в
процессе — сначала думал 11, написал body, потом оставил 8 и забыл обновить.

---

## 3. Рекомендации

### 3.1 Что делать (и почему)

**Не блокировать ничего.** PR #2205 merged, runtime-code функционален,
тесты зелёные. Число «11» в commit body — текстовая опечатка, ни на что
не влияет.

Если Шифу всё-таки хочет чистый audit-trail (например, для ретро-метрики
«сколько новых тестов добавил PR»):

**Вариант A — follow-up PR от Denis (рекомендую):**

```
docs(test-count): #2205 PR added 8 tests, not 11

Fix audit-trail: commit body заявлял «11 новых тестов»,
реально в TestTeleopLockPublishers 8 тестов.
Этот follow-up делает:
- ничего с кодом;
- только commit body 89dd800f переписан через amend в локальной ветке Denis
  (force-push его develop допустим — это его commit).

Refs: #2380, #2205, commit 89dd800f
```

Шифу решает: делать A или игнорировать.

**Вариант B — игнорировать (тоже ок):**

Это low-impact finding. Audit-метрики ретро можно восстановить из
`git show <commit> -- '*test*' | grep -cE '^\+    def test_'` (raw, 8).
Стоимость исправления (force-push develop) **выше**, чем выгода.

### 3.2 Что НЕ делать

- **НЕ `git push --force` в `develop`** от имени воркера. Это
  уничтожит историю у всех, кто уже подтянул `89dd800f`.
- **НЕ открывать PR с правкой кода тестов** — тесты функционально ок,
  нет смысла «править ради правки».
- **НЕ создавать дублирующий тест «чтобы было 11»** — это искусственный
  test-padding, нарушает ADR-AF-0013 (incremental delivery).

### 3.3 Связанные гигиенические предложения (out of scope, но видел)

В commit body #2205 есть ещё две вещи, которые стоит проверить при случае:

1. Фраза «`75 passed (было 64), 0 failed`» — это **тоже опечатка** (должно
   быть «72 passed»). Та же природа, та же рекомендация (follow-up
   amend от Denis, не блокер).
2. Список «НЕ сделано» в теле коммита содержит «e2e raw-evidence
   (требует SSH на Vision Pi)» — это значит, что e2e для PR #2205
   **не был прогнан**. Если это важно для ADR-0080, стоит
   отдельно запланировать e2e-прогон. Но это уже **не finding этой карточки**,
   а potential follow-up issue.

---

## 4. Acceptance для закрытия issue #2380

Issue можно закрыть сразу после того, как Шифу выберет один из вариантов 3.1:

- [ ] Вариант A (follow-up PR от Denis с amend commit body) — **OR** —
- [ ] Вариант B (no-op, issue закрыт как «won't fix, low-impact doc-only»)
- [ ] Решение зафиксировано в issue #2380 комментарием от architect
- [ ] Если A — PR Denis смержен, commit body скорректирован, audit-trail чистый
- [ ] Если B — issue #2380 закрыт с явной формулировкой «low-impact, post-merge amend is process-noisy, ignored»

---

## 5. Связанные

- **Issue #2380** — сам этот finding (kanban `t_c5f617c4`)
- **PR #2205** — merge в develop, 2026-09-09, mergeCommit `89dd800f`
- **ADR-0080** — teleop_lock bridge architecture (принят)
- **ADR-AF-0018** — «Честный FAIL лучше красивого PASS» — этот отчёт
  следует именно этой дисциплине (raw-evidence вместо голословного
  «одиннадцать»)
- **ADR-AF-0013** — incremental delivery, не big-bang правки
