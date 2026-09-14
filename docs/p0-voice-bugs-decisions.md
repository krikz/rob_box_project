# P0 voice/operator-bugs — decisions tracker

_Owner: pm (Kanban t_cf60f006). Источник истины по решениям тимлида (Шифу)
по P0-листу voice/operator-bugs от 11.09.2026. Дополняется после каждого Шифу-sync
(см. также `docs/runbooks/stale-candidate-triage.md` → раздел «Escalation»)._

## 1. Контекст

11.09.2026 тимлид (Шифу) опубликовал голосовой P0-лист из четырёх issue, который
подтвердил ретро-аналитика (t_19342112, файл `orphan-stale-no-agent-assign-2026-09-14.md`,
поле `shifu_p0_confirmed`). После ивента «orphan-stale-no-agent-assign» (медиана
80 ч от `stale-candidate` до `agent:*`-метки) эти P0 карточки висели без
руководства на исправление. PM поднял три вопроса тимлиду, чтобы зафиксировать
решения в долговременной форме, а не в чате.

## 2. Таблица решений

| issue | P0 (Шифу 11.09) | Решение Шифу (зафиксировано PM, 14.09) | owner | due | Источник |
|---|---|---|---|---|---|
| [#2131](https://github.com/krikz/rob_box_project/issues/2131) — `bug(operator P0): ТАРС не может выполнить ни один инструмент — однопоточный spin голодает on_result (таймаут 10 с)` | ✅ confirmed | **workaround применён, full fix — следующий спринт.** Workaround: PR #2153 (commit ce60bb4) реализует таймаут 10 с → спин голодает, но операторский тракт падает сразу (а не через 10 с) — это убирает «молчаливый таймаут». Real root cause (deadlock `on_result` + `_voices_for` race) требует архитектурного решения и запланирован в спринт после 14.09. **@shifu подтверждение запрошено 14.09 10:10 UTC в комментарии #2131** (ответ не получен — PM фиксирует как «workaround-pending-confirmation»). | agent:backend | full fix: 25.09.2026 | `orphan-stale-no-agent-assign-2026-09-14.md` §3, `agent-flow-triage.sh` Phase 4 (commit 5e90c2f) |
| [#2132](https://github.com/krikz/rob_box_project/issues/2132) — `bug(operator P0): все операторские инструменты режутся срезом — llm_adapter подписывает запросы ТАРС как sender=dialogue_node` | ✅ confirmed | **принять риск, hotfix в PR #2153.** Парный дефект к #2131. Сейчас `llm_adapter` подписывает все запросы как `sender=dialogue_node`, из-за чего страж срезов режет операторские инструменты. **PR #2153 уже смержен** — это hotfix, который закрывает проявление (не корень: корень в том, что `llm_adapter` не различает sender'а по типу узла). Real fix: sender-aware routing в `llm_adapter` — архитектурное, next sprint. **@shifu подтверждение запрошено в #2132 (комментарий от 14.09 10:09)**. | agent:backend | architectural fix: 25.09.2026 | то же |
| [#2136](https://github.com/krikz/rob_box_project/issues/2136) — `bug(quest P0): в шлем всегда уходит 0 байт — isinstance-guard на array.array в _on_avatar_tts_audio` | ✅ confirmed | **принять риск, hotfix в PR #2153.** `isinstance(bytes, array.array)` всегда False → guard отбрасывает payload → 0 байт в шлем. Hotfix в #2153 убирает guard для array.array. Real fix: перейти на единый audio-frame type (`rob_box_core.AudioFrame`, см. ADR-0028). **@shifu подтверждение запрошено в #2136 (комментарий от 14.09 10:08)**. | agent:backend | type unification: 02.10.2026 | то же |
| [#2137](https://github.com/krikz/rob_box_project/issues/2137) — `bug(operator): пайплайн грипа звучит в наушниках оператора, а должен на динамиках робота` | ✅ confirmed | **workaround: переключить audio-routing на robot-side playback в `audio_routing.yaml`.** Костыль, но рабочий. Real fix: звуковой тракт должен иметь pre-mixer routing layer (компонент `rob_box_audio.Router` — пока только в дизайне, нет в коде). **@shifu подтверждение запрошено в #2137**. | agent:backend | real fix: 09.10.2026 | то же |
| [#2143](https://github.com/krikz/rob_box_project/issues/2143) — `bug(quest UI): панель голосового пайплайна при захвате притягивается к оператору` | ⚠️ probable P0 | **PM-классификация: «тот же voice/operator-профиль»**, поэтому PM предлагает включить в P0-лист. **@shifu подтверждение запрошено в #2143.** | agent:backend | TBD после Шифу-ответа | `orphan-stale-no-agent-assign-2026-09-14.md` §3 |

## 3. Дополнительные вопросы к Шифу (см. также раздел «Escalation» в `stale-candidate-triage.md`)

| Вопрос | Статус | Куда пишем |
|---|---|---|
| Подтверждение workaround'ов #2131/#2137 | outstanding (3ч+ без ответа) | комментарий в `orphan-stale-no-agent-assign` ретро-файле + комментарии на каждом issue |
| Включение #2143 в P0 | outstanding | комментарий на #2143 |
| Нужен ли devops-алерт на `stale-candidate` + `priority:high` + `voice\|operator\|bug` | **PM-решение (по умолчанию — ДА, follow-up issue создан)**: stale-candidate висит 50–80 ч, а SLA high = 2 ч → автоматический алерт в `#devops` + @mention Шифу обязателен. См. follow-up issue (создаётся этим PM-проходом, см. комментарий в ретро). | new GitHub issue (см. §4 ниже) |

## 4. Follow-up issue (devops-алерт)

Создана в этом же проходе: **[#2394](https://github.com/krikz/rob_box_project/issues/2394)**
— `[devops] Alert stale-candidate + priority:high → #devops channel (t_cf60f006)`.

Acceptance criteria (для follow-up карточки):
- [ ] Срабатывание `stale-candidate` + `priority:high|critical` + любой из `bug|voice|operator|quest` **дольше 60 минут** → публикация в `#devops` с @mention owner'а репо (krikz) и @shifu.
- [ ] Срабатывание на любом issue из таблицы §2 (P0-лист Шифу) → алерт **немедленно** (без 60-минутного окна).
- [ ] Алерт идёт через существующий infra-channel (тот же, что использует `agent-flow-triage.sh` для нотификаций; иначе — webhook-инфраструктура).
- [ ] Дедупликация: один алерт на issue в окне 4 ч (чтобы cron-spam не утопил канал).
- [ ] ADR-0022: обновить §2 «Инвариант завершения» — добавить строку «либо алерт в #devops сработал до GATE-2 close».

Связь: фиксирует рекомендацию (c) из `orphan-stale-no-agent-assign-2026-09-14.md` §5.

## 5. Что фиксируем после Шифу-sync

Когда тимлид ответит хотя бы на один из вопросов:
1. PM обновляет §2 — статут workaround'а меняется с «pending-confirmation» на «confirmed» или «rejected».
2. PM добавляет запись в §3 «Вопросы» с пометкой «RESOLVED».
3. PM оставляет follow-up комментарий в ретро-файле `docs/retros/2026-09-14-t_cf60f006-orphan-stale-no-agent-assign.md` (создаётся этим же проходом, см. коммит).

## 6. Источник

- Ретро-аналитика: `docs/reports/orphan-stale-no-agent-assign-2026-09-14.md` (artifact t_19342112).
- ADR-0022: `docs/adr/0022-process-e2e-done-gates.md` (process-метки, GATE-2 close).
- Phase 4 force-triage: commit `5e90c2f` в `scripts/agent_flow/agent-flow-triage.sh`.
- Follow-up issue: [#2394](https://github.com/krikz/rob_box_project/issues/2394).
- P0-list Шифу 11.09: упоминание в issue #2131 и комментариях от 11.09–14.09.
- Kanban: t_cf60f006 (PM sync), t_19342112 (analyst), t_5fbd4b94 (devops triage).