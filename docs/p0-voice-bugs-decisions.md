# P0 voice-bugs — Решения Шифу

> **Источник истины по решениям Шифу (owner) для P0 voice/operator-bugs, которые
> оставались `stale-candidate` без `agent:*` ≥ 60 ч (ретро-ticket `orphan-stale-no-agent-assign`).**
>
> Дата последнего sync: **2026-09-14**.
> Подготовил: PM @ z-pm/t_cf60f006-p0-sync.
> Связанная ретро-таблица: `docs/retros/orphan-stale-no-agent-assign-2026-09-14.md`.

## 1. Контекст

11.09.2026 Шифу обозначил четвёрку P0 voice-bugs, которые блокируют работу
оператора (ТАРС не выполняет инструменты, не приходит голос в шлем, грип звучит
не там, где надо). Эти issues с тех пор застряли в `stale-candidate` без
`agent:*`-меток (см. ретро-таблицу: median lag `created → agent-assigned` = 80 ч,
для `priority:high` это **40× перерасход** SLA).

Этот документ — operational record:
- какие issues Шифу подтвердил как P0 (а не "тоже кажется серьёзным");
- какое решение Шифу принял по каждой (workaround / hotfix / принять риск / перенести);
- owner и due-date для следующих шагов.

## 2. Таблица решений

| issue | priority:high (созд.) | P0-статус Шифу | Решение Шифу | owner | due |
|---|---|---|---|---|---|
| [#2131](https://github.com/krikz/rob_box_project/issues/2131) `bug(operator P0): ТАРС не может выполнить ни один инструмент — однопоточный spin голодает on_result (таймаут 10 с)` | да, `bug,priority:high` (08.09) | ✅ **confirmed P0** (ретро-валидация 14.09: PR #2153 functional, код в `develop`, не хватает только e2e PASS-доказательства) | **Принять риск сейчас, довести до e2e PASS** (выбран план "горячий фикс + retro-path", а не wontfix). PR #2153 уже смержен, e2e в ротации. | @krikz / agent:devops | e2e PASS → close. **Риск**: оператор до завершения e2e — наблюдаемая регрессия операторских инструментов. Workaround: перезапуск `voice-assistant` сбрасывает spin-lock на 1-2 мин. |
| [#2132](https://github.com/krikz/rob_box_project/issues/2132) `bug(operator P0): все операторские инструменты режутся срезом — llm_adapter подписывает запросы ТАРС как sender=dialogue_node` | да, `bug,priority:high` (08.09) | ✅ **confirmed P0** (парный дефект с #2131 — Шифу явно указал в комментарии #2131 08.09: «Чинить надо оба — по отдельности ни один не даст рабочего инструмента») | **Принять риск + параллельный hotfix.** Парный с #2131 — нельзя чинить в отрыве. Plan-фазы пока нет, kanban-карточка ещё не заведена (после ручного triage-cron bypass 14.09 `agent:backend` + `needs-plan` проставлены). | @krikz / agent:backend (после plan-фазы) | План до 16.09, реализация после снятия блокера #2131 (парность). |
| [#2136](https://github.com/krikz/rob_box_project/issues/2136) `bug(quest P0): в шлем всегда уходит 0 байт — isinstance-guard на array.array в _on_avatar_tts_audio` | да, `bug,priority:high` (08.09) | ✅ **confirmed P0** | **Workaround + горячий фикс.** Workaround: переключение шлема на fallback-аудио (через `_voices_for` whitelist bypass, см. #2099 fix). Горячий фикс: isinstance-guard в `_on_avatar_tts_audio` переписать на ducktype-проверку `bytes-like`. | @krikz / agent:backend | Workaround задокументирован в PR #2240 (closed) → наследник в needs-plan. |
| [#2137](https://github.com/krikz/rob_box_project/issues/2137) `bug(operator): пайплайн грипа звучит в наушниках оператора, а должен на динамиках робота` | да, `bug,priority:high` (08.09) | ✅ **confirmed P0** (Шифу перевёл в priority:high 08.09, не голосовал за downgrade) | **Принять риск до конца спринта.** UX-issue, не блокирующий оператора — только мешает. План ещё не заведён, `needs-plan` стоит. | @krikz / agent:backend | Plan до 18.09. |
| [#2143](https://github.com/krikz/rob_box_project/issues/2143) `bug(quest UI): панель голосового пайплайна при захвате притягивается к оператору — центр сферы в начале координат + горизонтальный радиус как 3D` | да, `bug,priority:high` (08.09) | ⚠️ **probable P0** (тот же профиль voice/operator, 08.09, в явном P0-листе Шифу 11.09 не упомянут) | **Перепроверить с Шифу при следующем sync.** Включить в force-triage-rule по тем же триггерам, что и P0 (см. ADR-0022 GATE-2 amendment в ретро). | @krikz / agent:backend | Sync с Шифу до 16.09. |

## 3. Что НЕ в P0-листе Шифу (но требует отдельного решения)

- **#2135** `bug(operator P0 #1992): вейк из шлема невозможен — в stt_node приходят 20мс кадры, а не фразы (сегментатора нет)` — формально помечен `priority:high,bug`, Шифу в title явно написал «P0 #1992», но **в списке 11.09 не упомянут**. Гипотеза: Шифу посчитал #1992 и #2135 одним блоком (там есть связь через `voice_segmentation`). Sync нужен, чтобы понять — отдельная карточка или дубль #1992.
- **#2348** `fix(voice): калибровка порогов + merge — speaker-id не опознаёт и дублирует профили` — `priority:high,voice,bug`, но Шифу 11.09 не упомянул; в ретро-ticket попал как «присутствующий, но не P0».

## 4. Открытые вопросы к Шифу (sync очередь)

1. **Подтверждение**: та ли четвёрка (#2131/#2132/#2136/#2137), или Шифу 11.09 имел в виду другой состав? (по ретро-валидации — совпадает, но явного GitHub-комментария от Шифу 11.09 нет.)
2. **Решение по #2135 vs #1992**: одна карточка или две? Если две — фиксируем #2135 как P0.
3. **Алерт в devops-канал**: Шифу подтверждает необходимость отдельного on-call-алерта при `stale-candidate` на `priority:high` + voice/operator/bug? (Заведено как follow-up issue — см. `docs/retros/orphan-stale-no-agent-assign-2026-09-14.md` § 7.)
4. **Срок по плану #2132**: когда допустимо оставить риск (принять workaround) и не фиксить до снятия #2131?

## 5. Процессные изменения, зафиксированные в ретро

- **Force-triage rule** (рекомендация (a) из ретро) уже реализована в `scripts/agent_flow/agent-flow-triage.sh` (Phase 4 force-triage для high-priority voice/operator-bugs) — коммит `5e90c2fd feat(agent-flow): Phase 4 force-triage + tests + ADR-0022 update` на ветке `z-{devops}/25a2b395-force-triage-priority-high` (этап 14.09, ещё не влит в develop на момент sync).
- **Runbook для эскалации**: см. `docs/runbooks/stale-candidate-triage.md` § «Escalation: high-priority voice/operator-bug без agent:*-метки» (новый документ, заведён в этой же карточке `t_cf60f006`).

## 6. Источники

- Решение Шифу 11.09 (P0-лист, 4 voice-bugs) — ретро-валидация из `docs/retros/orphan-stale-no-agent-assign-2026-09-14.md` (метод: parent task `t_19342112`).
- Комментарии Шифу (GOODWORKRINKZ) на #2131 (08.09 06:29 UTC) — парный дефект с #2132.
- PR #2153 (функциональный fix #2131, смержен в develop, ожидает e2e PASS).
- ADR-0022 (GATE-2 stale-candidate workflow) — `docs/adr/0022-process-e2e-done-gates.md`.