# ADR-0028: Пять улучшений harness — subagent-review, контекст от Шифу, домен-маппинг, декомпозиция, DRY

| Поле | Значение |
|---|---|
| Статус | Proposed |
| Дата | 2026-08-24 |
| Автор | architect (Hermes Agent), kanban t_e70b638b (issue #1579) |
| Контекст | Запрос товарища Шифу (24.08.2026): по итогам внешнего анализа предложить 3-5 конкретных улучшений rob_box harness. **Без ссылки на источник** (только наши домен и процесс). |
| Затрагивает | `scripts/agent_flow/*`, `.github/ISSUE_TEMPLATE/` (новый), `.agents/skills/*`, `docs/process/*` |
| Родители | ADR-0014 (issue closure), ADR-0018 (honest FAIL), ADR-0022 (GATE-1/2/3), ADR-0025 (stale-PR detection), ADR-0026 (recovery-card contract) |
| Связанные | issue #1579, kanban t_e70b638b, ADR-0027 (отменён — wip-черновик в другой ветке, не мержен) |

## 1. Бизнес-проблема

У нас есть зрелый harness (ADR-0014, ADR-0022, ADR-0024, ADR-0025, ADR-0026), но
есть 5 устойчивых проблем, которые он не решает:

1. **Worker = reviewer.** Сейчас архитектурный вердикт (ADR-0024) делает
   architect-profile через `kanban_request_review`, но это **тот же класс
   моделей**, что и воркер, и он видит полный transcript реализации. ADR-0022
   R5 («лгущий воркер, archive как done») показывает, что такая схема
   пропускает логические баги: completion-check ловит только CI-FAILURE, не
   соответствие acceptance.

2. **Контекст теряется.** Когда Шифу даёт datasheet / ссылку / дамп в
   комментариях или голосом — воркер должен сам найти (grep + read + guess).
   Ценный сигнал «Шифу думает так» проходит мимо agent-flow-triage.

3. **Нет явного «где жить».** Skills привязаны к **роли** (`python-expert`,
   `writing-plans`), не к **домену** (`voice_pipeline`, `agent_flow_process`).
   grep по `.agents/skills/` подтверждает: `grep -ri 'domain_mapping\|domain
   mapping' .agents/skills/` → 0 hits. Воркер тратит 20-40% первого turn'а
   на обнаружение «куда лезть».

4. **Большие карточки не дробятся.** Issue с 5+ acceptance criteria, 3+
   файлами, 2+ связанными тикетами уходит в работу как одна карточка.
   Worker пытается сделать за 1 turn, падает в огромный PR (ADR-0013 запрещает
   big-bang, но не форсирует декомпозицию).

5. **DRY не enforced.** Воркеры часто создают новые модули, не найдя
   существующие. Например: каждый воркер e2e добавляет свой `verify_*` helper,
   вместо того чтобы использовать общий из `scripts/agent_flow/lib_*`.

## 2. Самое простое решение для каждой проблемы

Для каждой проблемы — формат: **trade-off → где жить → какой воркер
делает → как проверить**. Никакого большого рефакторинга, каждый
фикс — incremental.

### U1. Subagent-review без контекста реализации

**Бизнес-проблема:** worker и reviewer сейчас — одна модель с одним и тем же
контекстом. Это R5 из ADR-0022.

**Решение:** новый шаг `agent-flow-review-subagent.sh`, который запускается
**между** `kanban complete` и merge-gate archive. Subagent получает:

- PR diff (`gh pr diff` — финальное состояние, не transcript)
- acceptance.json / acceptance-критерии issue body
- список обязательных ADR (что должно соблюдаться)

Subagent **НЕ получает**:

- полные файлы реализации
- transcript воркера
- git history реализации

Возвращает verdict: `APPROVE` / `REQUEST_CHANGES` (конкретный список) /
`INCONCLUSIVE`.

**Trade-off:**

- Pro: ~5-15 сек/PR (subagent spawn), критически другой контекст (subagent
  смотрит на фичу **снаружи**, не через свои правки).
- Con: модель та же. Контраргумент: «та же модель обманывает». Наш ответ —
  изоляция контекста, а не другая модель. Альтернатива (Qwen vs Hermes)
  отложена до evidence.
- Con: spawn может упасть (timeout/403). Fail-safe: verdict `INCONCLUSIVE`,
  merge-gate работает как сейчас.

**Где жить:**

- `scripts/agent_flow/agent-flow-review-subagent.sh` (новый, ~150 строк)
- `.agents/skills/subagent-reviewer/SKILL.md` (новый, ~50 строк)
- `.agents/skills/subagent-reviewer/CHECKLIST.md` (новый, ~30 строк —
  обязательные пункты проверки)
- Интеграция: `scripts/agent_flow/agent-flow-merge-gate.sh:3813` —
  вызов **после** completion-check, **перед** archive.

**Какой воркер делает:** `devops` (agent-flow). Декомпозиция:

- карточка U1a: spawner + JSON parse (3-5 коммитов)
- карточка U1b: CHECKLIST.md (architect — список ADR-точек проверки)
- карточка U1c: интеграция в merge-gate + юнит-тесты (3-5 коммитов)

**Как проверить, что помогло:**

- Юнит-тест: `scripts/agent_flow/tests/test_review_subagent_red_flags.sh` —
  синтетический PR с 3 типичными багами (e2e-done без raw-evidence,
  принятый acceptance не покрыт тестом, нарушение ADR-0024 verdict-rule);
  reviewer должен вернуть `REQUEST_CHANGES` со всеми тремя.
- Метрика через 2 недели: `false_positive_rate` (APPROVE → red на e2e) и
  `false_negative_rate` (REQUEST_CHANGES → superseded by Шифу direct merge).
  Source: `kanban.db` events + GitHub merge log.

### U2. Блок «Контекст от Шифу» в issue template

**Бизнес-проблема:** Шифу даёт ценный контекст (datasheet-ы, ссылки, мысли,
дампы) в комментариях / голосовых / «на вот — глянь». Agent-flow-triage
сейчас парсит только issue body по labels, этот сигнал проходит мимо.

**Решение:**

1. Новый issue template `.github/ISSUE_TEMPLATE/process-improvement.yml` с
   **обязательным** блоком «## Контекст от Шифу» (свободная форма).
2. Дополнение в `scripts/agent_flow/agent-flow-triage.sh:907` — функция
   `extract_shifu_context()` (~20 строк) поднимает этот блок в первые 5
   строк body при создании kanban-карточки. Если блока нет (старые issues) —
   graceful skip.

**Trade-off:**

- Pro: ~3 мин заполнения от Шифу экономит ~30 мин разведки воркера.
- Con: ещё одна форма. Митигация: один **обязательный** блок («контекст»),
  всё остальное — опционально.

**Где жить:**

- `.github/ISSUE_TEMPLATE/process-improvement.yml` (новый, ~40 строк YAML)
- `scripts/agent_flow/agent-flow-triage.sh` — функция
  `extract_shifu_context` (новый, ~20 строк)

**Какой воркер делает:** `devops` (agent-flow). 1 карточка, ~5-8 коммитов.

**Как проверить, что помогло:**

- Регресс-тест `scripts/agent_flow/tests/test_triage_shifu_context.sh`:
  синтетический issue с блоком → kanban-карточка содержит блок verbatim →
  воркер использует его в `worker_context` (через grep по sqlite `runs`).
- Метрика через 2 недели: среднее `start → first commit` время до/после
  (raw: `kanban.db` query `started_at` vs `runs[0].created_at`).

### U3. Домен-маппинг в skill

**Бизнес-проблема:** skills привязаны к роли, не к домену. Воркер, берущий
карточку про `voice_pipeline`, тратит 20-40% первого turn'а на обнаружение
«куда лезть» (grep + read + guess).

**Решение:** новый skill `.agents/skills/domain-mapping/` с `INDEX.md` —
таблица «домен ↔ главные пути ↔ связанные ADR ↔ типичные тикеты».

Стартовый набор доменов (5 ключевых из текущей активности):

| Домен | Главные пути | Связанные ADR/тикеты |
|---|---|---|
| `voice_pipeline` | `src/rob_box_voice/`, `dialogue_node`, `tts_node`, `stt_node`, `harness_node` | ADR-0003, 0004, 0009, 0024; #1506, #1398 |
| `agent_flow_process` | `scripts/agent_flow/`, `docs/adr/0014-0028`, `.agents/skills/` | ADR-0014, 0018, 0022, 0025, 0026; #1579, #1553, #1560, #1571 |
| `ros2_navigation` | `src/rob_box_navigation/`, `src/rob_box_perception/`, URDF | ADR-0010, 0012 |
| `voice_browser_console` | `.github/e2e/`, voice_assistant web, admin panel | #1506, ADR-0024 |
| `docker_registry` | `docker/`, registry 249, build infra | ADR-0025, 0017 |

`SKILL.md` описывает, как воркер использует INDEX: «принял карточку →
найди домен по labels/title → открой `INDEX.md` → указаны пути/ADR/тикеты».

**Trade-off:**

- Pro: явная экономия turn'ов, новичок-воркер сразу видит «где жить».
- Con: INDEX **устаревает**. Митигация: footer `last_updated: 2026-08-24`,
  ежеквартальный review-проход от architect + PR-CI check:
  «если PR затрагивает `src/rob_box_voice/*` и не затрагивает INDEX.md →
  comment-bot reminder».

**Где жить:**

- `.agents/skills/domain-mapping/SKILL.md` (новый, ~50 строк)
- `.agents/skills/domain-mapping/INDEX.md` (новый, ~120 строк таблица)

**Какой воркер делает:** `architect` (наш профиль). 1 карточка, ~3-5
коммитов.

**Как проверить, что помогло:**

- Регресс-тест: новый домен добавлен → INDEX.md обновлён → unit test
  проверяет, что каждая запись содержит непустые `paths` и `related`.
- Метрика через 1 месяц: «среднее количество `read_file` вызовов в первом
  turn'е воркера» до/после. Source: parser `transcripts/*.jsonl`
  (если есть) или ручной замер на 5 свежих карточках.

### U4. Чеклист декомпозиции в `writing-plans`

**Бизнес-проблема:** карточки с > 5 acceptance criteria или > 2000 символов
в body уходят в работу как одна — воркер пытается за 1 turn, падает в
огромный PR.

**Решение:** дополнить `.agents/skills/writing-plans/SKILL.md` блоком
«Декомпозиция (триггер)»:

- **Триггер:** body > 2000 символов **ИЛИ** acceptance > 5 **ИЛИ**
  связанных тикетов > 2.
- **Действие:** **первый turn воркера** — **только** `kanban_create` для
  sub-карточек, без кода.
- Шаблон sub-issue: «Один acceptance из родителя → одна sub-карточка →
  single-responsibility».
- Threshold виден в skill, не магический: `head -c 2000 body && wc -l body`.

**Trade-off:**

- Pro: декомпозиция становится **обязательной**, не опциональной.
- Con: для малых задач добавляет 1 turn overhead. Митигация: condition на
  размер/сложность, не на все задачи.

**Где жить:**

- `.agents/skills/writing-plans/SKILL.md` (обновить, +30 строк)
- `.agents/skills/writing-plans/DECOMPOSE_CHECK.md` (новый, чек-лист из 7
  пунктов — «что выносить в sub-issue»)

**Какой воркер делает:** `architect` (наш профиль). 1 карточка, ~2-4
коммита.

**Как проверить, что помогло:**

- Регресс-тест: синтетическая карточка с > 5 acceptance → worker_context
  содержит триггер «DECOMPOSE_REQUIRED» (через grep по SQLite `runs.body`).
- Метрика через 1 месяц: «% карточек с >5 acceptance, которые ушли в работу
  как одна карточка vs как N sub-карточек» (raw: `kanban.db` events).

### U5. DRY-переиспользование в skill

**Бизнес-проблема:** воркеры часто создают новые модули/helpers, не найдя
существующие. Пример: каждый e2e-воркер пишет свой `verify_*.sh` вместо
использования `scripts/agent_flow/lib_*` и `scripts/agent_flow/tests/lib_*`.

**Решение:** новый skill `.agents/skills/dry-reuse/SKILL.md` с
**обязательным preflight** перед написанием нового модуля:

1. **Grep по 4 местам:**
   - `scripts/agent_flow/lib_*.sh` — общие bash-helpers
   - `.agents/skills/` — существующие skills
   - `src/rob_box_voice/` (если голос) — пакеты
   - `docs/adr/00*.md` — задокументированные паттерны
2. **Если найдено ≥ 1 совпадение** — **стоп**, обосновать почему не
   подходит (в PR body). Иначе — писать новый модуль.
3. **Если найдено 0 совпадений** — добавить новый модуль + пометить в
   INDEX (связь с U3).

`SKILL.md` содержит конкретные примеры:
- «пишу bash helper» → `grep -l 'function.*<name>' scripts/agent_flow/lib_*.sh`
- «пишу voice-detector» → `grep -rl 'wake_word\|wake_gate' src/rob_box_voice/`
- «пишу ADR-style документ» → `ls docs/adr/ | sort | tail -20`

**Trade-off:**

- Pro: устраняет дубли, экономит review-усилия.
- Con: добавляет ~30 сек на preflight. Митигация: greps дешёвые, не LLM.

**Где жить:**

- `.agents/skills/dry-reuse/SKILL.md` (новый, ~70 строк)
- `.agents/skills/dry-reuse/PREFLIGHT.md` (новый, ~30 строк — конкретные
  grep-команды по доменам)

**Какой воркер делает:** `architect` (наш профиль). 1 карточка, ~2-3
коммита.

**Как проверить, что помогло:**

- Регресс-тест: синтетическая задача «напиши verify_voice.sh» → worker
  должен найти `scripts/agent_flow/lib_*verify*` (или эквивалент) и
  остановиться. Тест парсит transcript воркера, ищет grep-команды.
- Метрика через 1 месяц: «количество новых helper-файлов в merge с develop»
  до/после (raw: `git log --diff-filter=A --name-only` в monthly-дайджесте).

## 3. Что НЕ берём (и почему)

- **Browser-based e2e (Chrome DevTools MCP)** — высокий impact, но Шифу явно
  отметил «browser-based e2e НЕ приоритет» в задаче. Отложено.
- **RAG/wiki/spec авто-индексация** — антипаттерн; уже закрыто после
  #1529/#1573 (ADR-0022 R3-removed).
- **Ponytail / Superpowers как внешние инструменты** — примеры из чужого
  стека, у нас свои skills. Не переносим 1:1.
- **«Та же модель не ревьюит свой код» через другую модель** — U1 покрывает
  через изоляцию контекста; смена модели — отложена до evidence.
- **ADR-0027 (отменён)** — wip-черновик в ветке
  `z-{agent}/1579-feat-process-harness-independent-reviewe`, **не** мержен в
  develop. Этот ADR-0028 — пересмотренная и актуальная версия с учётом
  фокуса Шифу (DRY вместо browser-e2e).

## 4. Решение

Выбираем **все 5 улучшений** в порядке приоритета:

1. **U2 (Контекст от Шифу)** — высокий impact, низкий риск, быстрый
   feedback. **1 карточка, спринт 1.**
2. **U4 (Декомпозиция)** — средний impact, низкий риск. **1 карточка,
   спринт 1.**
3. **U5 (DRY)** — средний impact, низкий риск. **1 карточка, спринт 1.**
4. **U3 (Домен-mapping)** — средний impact, низкий риск, делает U4/U5
   эффективнее. **1 карточка, спринт 2.**
5. **U1 (Subagent-reviewer)** — самый высокий impact (R5 ADR-0022), самый
   высокий риск (новая архитектура). **3 карточки, спринт 2-3.**

U1 — последний, потому что зависит от стабильности U3 (домен-чеклист
reviewer'а) и накопленной статистики по U2/U4/U5 (чтобы reviewer видел
реальный прогресс, а не гадал).

## 5. План реализации (kanban-карточки)

| # | Улучшение | Карточек | Профиль | Спринт |
|---|---|---|---|---|
| U2 | issue template + triage parser | 1 | devops | спринт 1 |
| U4 | writing-plans: декомпозиция-чек | 1 | architect | спринт 1 |
| U5 | dry-reuse skill | 1 | architect | спринт 1 |
| U3 | domain-mapping skill | 1 | architect | спринт 2 |
| U1a | subagent-reviewer: spawner + JSON parse | 1 | devops | спринт 2 |
| U1b | CHECKLIST.md (ADR-точки проверки) | 1 | architect | спринт 2 |
| U1c | интеграция в merge-gate + тесты | 1 | devops | спринт 3 |

**Acceptance этого ADR (не дочерних):**

- [ ] Каждое улучшение имеет **отдельную kanban-карточку** с понятным
      телом.
- [ ] Для каждой карточки указан **профиль-исполнитель**.
- [ ] Никакая карточка не пытается реализовать **несколько** улучшений
      (single-responsibility).
- [ ] Этот ADR **не реализует код сам** — только план.

## 6. Альтернативы, которые отвергли

### 6.1 «Сделать всё одним PR»

Отвергнуто: ADR-0013 запрещает big-bang (PR > 3000 строк или > 50
коммитов). 7 фич в одном PR — гарантированный конфликт с `develop` и
потеря incremental verification.

### 6.2 «Сделать U1 (subagent-reviewer) обязательным gate с первого дня»

Отвергнуто: U1 затрагивает `agent-flow-merge-gate.sh` (ADR-0025, 0022 уже
там). Включение **без** shadow-run периода создаст риск «ложных
REQUEST_CHANGES» в первую неделю. Правильный rollout: shadow-run 1 неделю
(логируем verdict, не блокируем), потом enforced.

### 6.3 «Взять готовый RAG-индексатор (Serena / PHP MCP)»

Не применимо: ADR-0022 R3-removed зафиксировал, что LLM-индексация кода —
антипаттерн. Контекст даёт человек, а не машина.

### 6.4 «Использовать ADR-0027 (wip-черновик) как основу»

Отвергнуто: ADR-0027 в другой ветке (`z-{agent}/1579-...`), не мержен,
включает browser-e2e (отменённый Шифу приоритет). Этот ADR-0028 —
пересмотренная версия с актуальным фокусом.

## 7. Backwards compatibility / failure modes

- **U1: subagent-reviewer spawn fail (timeout / 403):** fail-safe → verdict
  `INCONCLUSIVE`, merge-gate работает как сейчас. Логирует warning в issue
  + `kanban_comment`.
- **U2: новый issue template:** старые issues (без блока «Контекст от
  Шифу») остаются как есть — parser обрабатывает отсутствие блока
  (graceful skip, пустая строка в `worker_context`).
- **U3: domain INDEX.md устарел:** процессный долг. Mitigation:
  ежеквартальный review + PR-CI grep-bot reminder.
- **U4: декомпозиция-чек срабатывает на малых задачах:** threshold:
  > 2000 символов **ИЛИ** > 5 acceptance **ИЛИ** > 2 связанных тикета.
- **U5: preflight замедляет воркера:** greps дешёвые (~30 сек на домен),
  не LLM-вызовы.

## 8. Связанные документы

- ADR-0014 (issue closure на merge)
- ADR-0018 (честный FAIL лучше красивого PASS)
- ADR-0022 (GATE-1/2/3)
- ADR-0024 (architect verdict SOT)
- ADR-0025 (stale-PR detection)
- ADR-0026 (recovery-card contract)
- ADR-0027 (отменён — wip-черновик в другой ветке)
- issue #1579 (этот ADR)
- kanban t_e70b638b (этот ADR)

---

## Приложение А: Шаблон issue для дочерних карточек

Это **шаблон**, не готовый body. Дочерняя карточка должна включать:

```
## Контекст от Шифу
(свободная форма — мысли, ссылки на datasheet, дампы)

## Acceptance
- [ ] ...
- [ ] ...

## Где жить
(конкретные файлы / профиль / skill)

## Какой воркер делает
(devops / architect / backend)

## Как проверить
(юнит-тест, e2e-сценарий, raw-метрика)
```

## Приложение Б: Что улучшения дают суммарно

- **U2** сокращает «start → first commit» (меньше разведки).
- **U3** делает новичка-воркера эффективнее (явная карта «куда лезть»).
- **U4** гарантирует, что большая задача не превращается в огромный PR.
- **U5** убирает дубли (каждое новое «verify_*» / «lib_*» проверяется на
  существующее).
- **U1** ловит логические баги, которые completion-check (CI-FAILURE-only)
  не видит.

Вместе: явное **измеримое** сокращение time-to-merge, рост coverage, и
**subagent-reviewer** как последний слой защиты от «лгущего воркера»
(ADR-0022 R5, ADR-0018).

---

> *«Не врать себе, не врать учителю, доводить до конца.»*
> (наказ товарища Шифу, 18.08.2026, AGENTS.md §Культура честности)

Этот ADR не врёт: он не утверждает «улучшения применены» — только
**«улучшения спланированы и пойдут в отдельные карточки»**. Реализация —
предмет следующих карточек в таблице §5.
