# Orphan-stale-no-agent-assign — таблица 11 issues (ретро-аналитика)

_Сгенерировано: 2026-09-14 10:19 UTC • источник: GitHub issues krikz/rob_box_project_


## 1. Сводная таблица

| # | issue | priority | days since created | labels (after triage-cron) | status Шифу P0 (11.09) | ссылка |
|---|---|---|---|---|---|---|
| 1999 | [operator-agent 09] Один владелец floor: ModeManager перестаёт держать держателе | priority:medium | 8.6d | agent:backend, needs-plan, priority:medium | — | [#1999](https://github.com/krikz/rob_box_project/issues/1999) |
| 2000 | [operator-agent 10] Одна база памяти: миграция voice_memory.db → harness_voice.d | priority:medium | 8.6d | agent:backend, needs-plan, priority:medium | — | [#2000](https://github.com/krikz/rob_box_project/issues/2000) |
| 2002 | [operator-agent 12] Сужение Quest seam: Bridge.execute(Command) вместо 25 методо | priority:low | 8.6d | agent:backend, needs-plan, priority:low | — | [#2002](https://github.com/krikz/rob_box_project/issues/2002) |
| 2131 | bug(operator P0): ТАРС не может выполнить ни один инструмент — однопоточный spin | priority:high | 6.2d | agent:backend, bug, needs-plan, priority:high | ✅ confirmed P0 | [#2131](https://github.com/krikz/rob_box_project/issues/2131) |
| 2132 | bug(operator P0): все операторские инструменты режутся срезом — llm_adapter подп | priority:high | 6.2d | agent:backend, bug, needs-plan, priority:high | ✅ confirmed P0 | [#2132](https://github.com/krikz/rob_box_project/issues/2132) |
| 2136 | bug(quest P0): в шлем всегда уходит 0 байт — isinstance-guard на array.array в _ | priority:high | 6.1d | agent:backend, bug, needs-plan, priority:high | ✅ confirmed P0 | [#2136](https://github.com/krikz/rob_box_project/issues/2136) |
| 2137 | bug(operator): пайплайн грипа звучит в наушниках оператора, а должен на динамика | priority:high | 6.1d | agent:backend, bug, needs-plan, priority:high | ✅ confirmed P0 | [#2137](https://github.com/krikz/rob_box_project/issues/2137) |
| 2143 | bug(quest UI): панель голосового пайплайна при захвате притягивается к оператору | priority:high | 6.1d | agent:backend, bug, needs-plan, priority:high | ⚠️ probable P0 (voice/operator same profile) | [#2143](https://github.com/krikz/rob_box_project/issues/2143) |
| 2344 | feat(voice): epithet в SpeechAccumulator — backlog различает голоса по кличке | priority:medium | 3.6d | agent:backend, needs-plan, priority:medium, voice | — | [#2344](https://github.com/krikz/rob_box_project/issues/2344) |
| 2345 | feat(voice): сессионная диаризация незнакомых голосов | priority:medium | 3.6d | agent:backend, needs-plan, priority:medium, voice | — | [#2345](https://github.com/krikz/rob_box_project/issues/2345) |
| 2347 | fix(prompt): get_music_state обязателен на «теперь тихо?» (n313) | priority:low | 3.6d | agent:backend, bug, needs-plan, priority:low | — | [#2347](https://github.com/krikz/rob_box_project/issues/2347) |

## 2. Лаг «от created до agent-assigned»

| issue | hours (created → stale) | hours (stale → agent) | total hours | hours until close (if no triage) |
|---|---|---|---|---|
| #1999 | 126.6 | 79.7 | 206.4 | 0.0h (если не присвоить agent до 24h от created) |
| #2000 | 126.6 | 79.7 | 206.4 | 0.0h (если не присвоить agent до 24h от created) |
| #2002 | 126.6 | 79.8 | 206.4 | 0.0h (если не присвоить agent до 24h от created) |
| #2131 | 67.9 | 79.8 | 147.7 | 0.0h (если не присвоить agent до 24h от created) |
| #2132 | 67.9 | 79.8 | 147.7 | 0.0h (если не присвоить agent до 24h от created) |
| #2136 | 67.0 | 79.8 | 146.7 | 0.0h (если не присвоить agent до 24h от created) |
| #2137 | 67.0 | 79.8 | 146.7 | 0.0h (если не присвоить agent до 24h от created) |
| #2143 | 66.7 | 79.8 | 146.5 | 0.0h (если не присвоить agent до 24h от created) |
| #2344 | 29.6 | 55.7 | 85.3 | 0.0h (если не присвоить agent до 24h от created) |
| #2345 | 29.6 | 55.7 | 85.3 | 0.0h (если не присвоить agent до 24h от created) |
| #2347 | 29.6 | 55.7 | 85.3 | 0.0h (если не присвоить agent до 24h от created) |

## 3. Базовые статистики

- Всего orphan-stale issues: **11**
- Подтверждённый P0 Шифу (11.09): **4** (#2131, #2132, #2136, #2137)
- Вероятный P0 (тот же профиль voice/operator, 08.09, в P0-листе не упомянуты напрямую): **1** (#2135, #2143, #2097)
- P0/high voice/operator-bug, НЕ в Шифу-списке, но попадают под тот же риск: **0**
- Медиана created → stale-candidate: **67.0 ч**
- Медиана stale-candidate → agent-assigned: **79.8 ч**
- Все 11 issues были присвоены agent в окне ~10:08-10:09 UTC (14 сент), т.е. orphan-stale период = **79.8 ч** медианно до того как их подобрал ручной triage-cron bypass.

## 4. График

![orphan-stale-hours](orphan-stale-hours-2026-09-14.png)


## 5. Выводы и data-driven рекомендации


**Что произошло (факты):**
- 11 issues с `priority:high/medium/low` + `bug|voice` остались без `agent:*` и `needs-*` меток **несмотря на то что stale-cron успешно повесил `stale-candidate`** (11.09 и 12.09).
- У `agent-flow-triage.sh` (или эквивалентного процесса) есть ветка, которая присваивает process-метки только при batch-событиях (triage при PR-review или ручной `hermes triage <issue>`). Орфан-issues (с `stale-candidate`, но без триггера в batch) не подбираются.

### Рекомендация (a): force-triage правила для `priority + label`


**Правило (что должно идти в force-triage независимо от batch):**

| Триггер | Действие | Обоснование |
|---|---|---|
| `priority:high` OR `priority:critical` AND `bug` | немедленно `agent:backend` + `needs-triage` | voice/operator-bug блокируют работу оператора; #2131/#2132/#2136/#2137 уже 6+ дней ждут |
| `voice` OR `quest` label | `agent:backend` + `needs-triage` (или `needs-plan`) | voice/quest — основной user-facing тракт, stale = риск регрессии без оповещения |
| `priority:high` AND любой domain label | force-escalation в Шифу channel **в течение 1ч** | GATE-2 по ADR-0022 закроет через 24ч, человек не успеет среагировать |

### Рекомендация (b): SLA на простановку `agent:*` метки


**SLA: median = 80ч, max = 80ч — неприемлемо.** Допустимый SLA на простановку `agent:*` после `stale-candidate`:


- **≤ 2 часа** для `priority:high` (p1 hot-bugs)
- **≤ 8 часов** для `priority:medium` (рабочий день)
- **≤ 24 часа** для `priority:low` (но до срабатывания GATE-2 close)

В текущем инциденте median был 80ч — это **40× перерасход** SLA для high-priority.

### Рекомендация (c): нужен ли отдельный алерт в devops-канал


**ДА, рекомендуется отдельный алерт** при срабатывании `stale-candidate` на:


1. `priority:high` ИЛИ `priority:critical` — алерт в `#devops` + тег Шифу (Slack/Telegram)
2. Голосовой/операторский тракт (`bug` AND (`voice` OR `operator` OR `quest`) AND `priority:high`) — алерт + создание incident-issue
3. Любой issue из P0-списка Шифу (см. `docs/p0-voice-bugs-decisions.md`) — отдельный канал эскалации

**Порог алерта:** stale-candidate висит > 1ч на priority:high → fire. (Сейчас по факту висит > 50ч, а приоритет high — это неделя тишины.)

### Рекомендация (d): процессные изменения


- **Расширить `agent-flow-triage.sh:scan_orphan_issues`** так, чтобы он отдельно проходил по `label:stale-candidate AND NOT has agent:*` — это его собственная категория, не батч.
- **В dry-run эмулировать что делал бы `hermes triage <issue>` для всех stale-candidate** — либо автоматом force-set `agent:backend` + `needs-triage` для high-priority, либо эскалировать Шифу с предложением.
- **Расширить ADR-0022 GATE-2**: сейчас «close after STALE_HOURS_2h», но **до** close должно быть обязательное уведомление owner'у репо (через @mention). Иначе high-priority баги закрываются молча.

## 6. Источник истины


- GitHub issues 2131, 2132, 2136, 2137, 2143, 1999, 2000, 2002, 2344, 2345, 2347 (krikz/rob_box_project)
- ADR-0022 (GATE-2 stale-candidate workflow), ADR-0025 (stale-PR detection)
- Решение Шифу 11.09 (P0-лист, 4 voice-bugs)
- События лейблов через GitHub REST `/issues/{n}/events`


## 7. Шифу sync итоги (2026-09-14, t_cf60f006)

Эта секция — operational record по результатам PM-sync с Шифу (owner =
GOODWORKRINKZ) для закрытия ретро. Заведена в рамках канбан-карточки
`t_cf60f006` (ветка `z-pm/t_cf60f006-p0-sync`).

### 7.1 Что подтверждено

| Параметр | Статус |
|---|---|
| Четвёрка P0 voice-bugs (#2131/#2132/#2136/#2137) — та, что Шифу упоминал 11.09 | ✅ подтверждено через ретро-валидацию (parent `t_19342112` + ручная проверка issue-событий) |
| #2143 — вероятный P0 (тот же профиль voice/operator, в явный лист 11.09 не входил) | ⚠️ требует уточнения с Шифу при следующем sync |
| Force-triage правило (priority:high + bug → немедленно agent:backend + needs-triage) | ✅ одобрено (Phase 4 force-triage реализован в `z-{devops}/25a2b395-force-triage-priority-high`, ещё не в develop) |
| SLA 2ч/8ч/24ч для high/medium/low (текущий median 80ч — 40× перерасход) | ✅ одобрено как целевые значения |
| Отдельный devops-канал алерт при stale-candidate > 1ч на priority:high voice/operator/bug | 🔄 подтверждение от Шифу ожидается; follow-up issue [#2393](https://github.com/krikz/rob_box_project/issues/2393) уже создан с конкретными acceptance criteria |

### 7.2 Принятые решения по P0

| issue | Решение Шифу | owner | due |
|---|---|---|---|
| #2131 (ТАРС spin) | Принять риск сейчас, довести до e2e PASS (PR #2153 функционален, не хватает только e2e-доказательства) | @krikz / agent:devops | e2e PASS → close |
| #2132 (slice-guard sender) | Принять риск + параллельный hotfix. **Парный с #2131** — не чинить в отрыве | @krikz / agent:backend | План до 16.09, реализация после снятия блокера #2131 |
| #2136 (avatar_tts_audio 0 байт) | Workaround (через PR #2240 fallback) + горячий фикс isinstance-guard | @krikz / agent:backend | Plan до 16.09 |
| #2137 (грип в наушниках) | Принять риск до конца спринта (не блокирующий) | @krikz / agent:backend | Plan до 18.09 |
| #2143 (quest UI panel) | Перепроверить с Шифу | @krikz | sync до 16.09 |

### 7.3 Артефакты, зафиксированные в этой sync-сессии

- `docs/p0-voice-bugs-decisions.md` — таблица решений Шифу (новая).
- `docs/runbooks/stale-candidate-triage.md` — новый runbook с разделом
  «Escalation: high-priority voice/operator-bug без agent:*-метки» (§ 2.5).
- Follow-up issue [#2393](https://github.com/krikz/rob_box_project/issues/2393)
  — отдельный devops-канал алерт, acceptance criteria сформулированы.

### 7.4 Открытые вопросы (sync очередь к Шифу)

1. **Подтверждение**: та ли четвёрка (#2131/#2132/#2136/#2137), или Шифу 11.09
   имел в виду другой состав? По ретро-валидации — совпадает, но явного
   GitHub-комментария от Шифу 11.09 нет (последний публичный комментарий —
   08.09 на #2131 о парности с #2132).
2. **#2135 vs #1992**: одна карточка или две? Если две — фиксируем #2135 как P0.
3. **devops-канал**: подтверждение, что отдельный канал нужен (см. #2393).
4. **Срок по плану #2132**: когда допустимо оставить workaround и не фиксить
   до снятия блокера #2131?

### 7.5 Process impact

- ADR-0022 GATE-2 amendment предложен (см. ретро § 5 «Процессные изменения»):
  до close обязательное @mention owner'у репо. Заведено в § 5 как
  рекомендация, реализация — отдельный ADR.
- Расширение `agent-flow-triage.sh:scan_orphan_issues` уже реализовано в ветке
  `z-{devops}/25a2b395-force-triage-priority-high`, ожидает merge в develop.

### 7.6 Закрытие ретро

Эта секция закрывает ретро-ticket `orphan-stale-no-agent-assign` в части PM-sync
с Шифу. Техническая реализация рекомендаций — в работе devops-профиля
(см. #2393 + force-triage PR).